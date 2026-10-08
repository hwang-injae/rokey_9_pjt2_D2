# -*- coding: utf-8 -*-
"""작업 관리자 TaskManager — 자동 조립 상태표 · 실패 대응표 · 시작 점검 (ROS 없이 동작, SDD 5장 · 7.1).

task 는 로봇 PC 에서 돈다. 웹 화면 · 음성은 웹 PC 에 있고 다리(bridge)가 ROS 이름 그대로 대신 부르므로(E-26~E-28),
이 클래스는 MQTT 를 전혀 모른다. task 노드(task_node.py)가 콜백에서 on_* · command 를 부르고,
작업 스레드 하나가 run_once() 를 되풀이한다.
바깥 일(이동 · 관측 · 집기 · 방송)은 생성할 때 받은 io 객체로 한다. io 가 갖춰야 할 것(ROS 노드나 시험용 가짜):
  get_design(design_id, should_abort) -> (ok, reason, design)   설계 조회(/d2/hmi/get_design, design/1 dict). 제한 시간은 io 가 건다
  services_ready() -> [이름]                        아직 안 떠 있는 서버 이름들(없으면 빈 목록, 기다리지 않는다)
  move_to(target, should_abort, speed_ratio=1.0) -> (ok, reason)  복구 뒤 첫 이동만 비율 지정
  check_progress(block_ids, should_abort) -> (ok, reason, rows)   rows = {block_id: {state, dx_m, dy_m, dz_m, top_z_m}}
  pick_place(goal, should_abort, on_feedback, on_result) -> (ok, reason)    goal = TaskPlanner.next_block 의 FOUND dict. 피드백 step · 결과 수신 시각을 콜백으로 알린다
  request_stop(reason) -> (ok, message)             정지 노드에 먼저 세우라고 요청(응답은 접수 여부)
  publish_state(dict) · publish_progress(dict)
move_to · check_progress · pick_place 는 io 가 robot.yaml 의 제한 시간(service_s 3 · move_to_s 45 · pick_place_s 90)을 건다.
시간 초과 때 request_stop(reason) 으로 정지 노드에 먼저 요청하고, stopped 신호를 확인하기 전에는 복구하지 않는다. 기다리는 동안
should_abort() 가 참이 되면(정지 신호 · Ctrl+C) 곧바로 빠져나와 (False, 'STOPPED') 로 돌려주고, 진행 중인 pick_place 목표는 취소한다.

범위: 자동 조립 상태 11개 + 기록(CSV · run_id, W065 — RunLogger), 설계 조회 · 결과 저장 · 시간 제한(W119 일부).
웹 끊김은 WAIT_HMI 로 기다린다(서기 궤적 아님). 스캔은 촬영 자세 이동 → 점군 수집 → 추론 → 검토 순서다.
run_id 는 조립 출발(READY → CHECK) 또는 스캔 시작(SCAN_MOVE) 때 새로 만든다. 조립 정지 · 복구 · 공급 보충은 같은 ID 를 쓴다.
끝난 조립의 build/1 요약은 last_build(최근 결과 표시용)와 pending_builds(run_id 별 미전송 요약)에 둔다.
/d2/hmi/save_build 로 보내는 일은 task 노드가 하고(네트워크 대기는 이 클래스의 잠금 밖), 이 클래스는 무엇을 보낼지(builds_to_send) ·
답이 오면 어떻게 할지(build_result)만 정한다. 저장이 확인된 run_id 만 목록에서 지운다 — 실패 · 시간 초과 · 잘못된 답은 요약을 남긴다.
WAIT_SUPPLY(공급 채우기 · [계속])는 05 작업분류 W119 ②의 일부지만 SDD 5장 상태표에 있어 W044 에서 먼저 구현했다 — **W119 완료가 아니다**
(W119 는 스캔 흐름이 더 남아 있다).
복구(RECOVER) 뒤 첫 성공한 move_to 는 robot.yaml recover.speed_ratio(0.5)로 움직인다. 조립 중이 아닐 때(IDLE · READY · DONE)
정지됐다 풀린 뒤 돌아갈 곳은 SDD 에
없어서 가장 가까운 규칙(비조립 RECOVER → IDLE)만 따른다(IRD 12장 W121 다시 확인 대상).
"""
import copy
import json
import logging
import threading
import time

from d2_task.block_picker import BlockPicker
from d2_task.run_logger import RunLogger
from d2_task.scatter_pick import ScatterFlow, StepTracker
from d2_task.task_planner import TaskPlanner

RECIPE_SCHEMA = 'cad_recipe/1.0'

LOG = logging.getLogger('d2_task')

# 조립이 진행 중인 상태 — 정지 · 실패 뒤 RECOVER 에서 CHECK 로 이어 간다
RUN_STATES = ('CHECK', 'SELECT', 'PICK_PLACE', 'WAIT_SUPPLY', 'WAIT_HMI', 'VERIFY', 'RECOVER', 'ERROR')
# 로봇이 서야 하는 결과 — 정지 노드가 세웠다(STOPPED · CANCELED) / 그리퍼 피드백이 없어 세웠다(NO_FEEDBACK, IRD 7장 → STOPPED)
STOP_REASONS = ('STOPPED', 'CANCELED', 'NO_FEEDBACK')


def wait_until(event, should_abort, poll_s=0.05, timeout_s=None):
    """event 가 켜질 때까지 기다리되, should_abort() 가 참이 되면 바로 빠져나온다. timeout_s 를 주면 그 시간(monotonic)이 지나도 빠져나온다.

    반환: True = event 가 켜졌다(답이 왔다), False = 중단 신호가 먼저 왔다(또는 timeout_s 가 지났다).
    poll_s 는 중단 신호를 확인하는 간격일 뿐 기다리는 시간 제한이 아니다. 완전 블로킹 대기(future.result())로
    정지 처리가 막히지 않게 하려고 task_node 의 모든 기다림이 이것을 거친다.
    """
    deadline = None if timeout_s is None else time.monotonic() + timeout_s
    while True:
        if should_abort() or (deadline is not None and time.monotonic() >= deadline):
            return False
        wait_s = poll_s if deadline is None else min(poll_s, max(0.0, deadline - time.monotonic()))
        if event.wait(wait_s):
            # 이미 켜진 이벤트도 취소 · 시간 초과보다 우선하지 않는다. 경계에서 늦은 성공을 적용하지 않기 위해서다
            return not should_abort() and (deadline is None or time.monotonic() < deadline)


class TaskManager:
    """자동 조립 상태표. 설계를 고르고(select_design) 출발(start)하면 진행 확인 → 다음 블록 → 집기·놓기 → 배치 확인을 되풀이한다.

    들어오는 것(콜백 스레드): command(HmiCommand) · on_intent(음성) · on_safety(정지 노드) · on_gripper · shutdown.
    나가는 것: io 로 /d2/task/state · /d2/task/progress 방송, move_to · check_progress · pick_place 호출.
    바깥 영향: 로봇은 pick_place · move_to 를 통해서만 움직인다(이 클래스는 팔을 직접 안 건드린다).
    실패: TIMEOUT 은 먼저 정지 요청 후 정지 상태를 기다린다(IRD 7장). ERROR · GRIPPER_NO_RESPONSE 는 ERROR 로 간다.
    IRD 에 없는 reason 도 같은 공통 분기로 일반 실패 → ERROR 로 간다.
    정지 노드가 멈추면(safety/state stopped) 어느 상태에서든 STOPPED 로 가고, 잠금이 풀린 신호가 새로 오면 RECOVER 로 간다.
    """

    def __init__(self, cfg, io, logger=None, clock=time.monotonic, monitor_hmi=False, open_width_m=None):
        """cfg = robot.yaml dict, io = 위 설명의 바깥 일 담당, logger = RunLogger(없으면 파일 없이 run_id 만 만든다), clock = 단조 시계(시험용).
        monitor_hmi 는 웹 생존 신호 감시 여부. 이 클래스의 기본은 False(시험 · 웹 없이 동작)이고, task 노드는 design_source 와 따로 노드 파라미터 monitor_hmi(기본 true)로 정해 넘긴다(E-62).
        공급 방식은 cfg['supply_mode'](없으면 slots): slots = 공급 칸 6개(기본), scatter = 흩뿌린 공급(관측 → 후보 고르기, E-55). scatter 는 open_width_m(m)을 명시로 받아야
        집기 요청을 만든다 — 열림 폭이 확정 전이라 노드는 아직 None 을 넘기고, 그러면 scatter 선택은 ERROR 로 끝난다(실제 흩뿌림 실행 보류).
        만들 때 디스크에 보관돼 있던 미전송 요약을 되살려 다시 보낼 수 있게 한다(로봇 작업을 받기 전). 처음 상태는 IDLE."""
        self.cfg, self.io = cfg, io
        self.logger = logger if logger is not None else RunLogger()
        self.run_id = None                        # 지금(또는 방금 끝난) 조립의 run_id. 새 설계를 고르거나 IDLE 로 가면 비운다
        self.last_build = None                    # 가장 최근에 끝난 조립의 build/1 요약(표시용). 새 출발로는 지우지 않는다
        self.pending_builds = {}                  # run_id → 아직 DB 저장이 확인 안 된 build/1 요약(복사본)
        self._build_inflight = set()              # 지금 save_build 요청을 보내고 답을 기다리는 run_id
        self._build_last_try = {}                 # run_id → 마지막으로 보낸 시각(monotonic, 다시 보내기 간격 계산)
        self._build_ready = set()                 # 디스크 보관이 끝났거나(성공 · 실패) 보관하지 않는 run_id — 이것만 보낸다
        self._build_queued = {}                   # run_id → 대기 목록에 들어온 시각(monotonic). 디스크가 늦으면 이 시각으로 기다림을 끊는다
        self._clock = clock
        self.command_s = cfg['timeout']['command_s']   # 화면 명령(선택 · 출발)을 받은 뒤 확정해야 하는 시간(s) — 넘으면 실행하지 않고 TIMEOUT
        self._epoch = 0                           # 상태가 바뀔 때마다 +1 — 설계 조회 중에 상태가 바뀌었는지 알아보는 표
        self._steps = StepTracker()               # 집기 요청마다 마지막 피드백 step · 결과 수신 시각(수집만 — 재시도 판단에는 아직 안 쓴다)
        self._pick_seq = 0                        # pick_place 요청마다 새 번호
        self.last_pick = None                     # 가장 최근 pick_place 가 끝났을 때의 {request_id, ok, reason, step, result_at}
        self._stop_count = 0                      # stopped=true 신호를 받은 횟수 — 정지 뒤 [다시 시작]까지 끝나도 그 전에 보낸 요청의 늦은 답을 알아본다
        self._lookup_token = 0                    # 가장 최근 설계 조회 요청 번호 — 이전 요청의 늦은 답을 버리는 데 쓴다
        self._lock = threading.RLock()            # 콜백 스레드와 작업 스레드가 같이 만지는 값들
        self.state = 'IDLE'
        self.planner = None
        self.design_id = None
        self.block_id = None                      # 지금 다루는 블록
        self.goal = None                          # 보낼(보낸) pick_place 목표 = next_block 의 FOUND
        self.placed = None                        # 방금 놓은 블록(VERIFY 에서 확인)
        self.safety = None                        # 마지막 safety_state/1
        self.safety_count = 0                     # safety/state 를 받은 횟수 — 다시 시작 감지에 쓴다(시계가 다른 PC 의 stamp 는 안 믿는다)
        self.gripper = None                       # 마지막 gripper_state/1
        self._entered_count = 0                   # STOPPED · ERROR 에 들어갈 때의 safety_count
        self._assembling = False                  # 정지됐을 때 조립 중이었나(RECOVER 가 CHECK 로 이어 갈지 정한다)
        self._pending_start = False               # WAIT_SUPPLY 에서 받은 [계속]
        self._supply_moved = False                # WAIT_SUPPLY 에서 관측 자세에 이미 도착했나(도착해야 알림 · [계속]을 받는다)
        self._reobserved = False                  # UNKNOWN_BLOCK 재관측을 이미 한 번 했나
        self._plan_retry = 0                      # PLAN_FAILED 다시 계획 횟수
        self._grasp_stage = 0                     # GRASP_FAILED: 1 = 같은 칸 재시도 중, 2 = 다음 칸 시도 중
        self._shutdown = False
        self._timeout_pending = False             # 정지 요청의 성공 응답만으로는 서 있다고 볼 수 없다. stopped 신호가 필요하다
        self._recover_slow = False                # 복구 뒤 첫 이동이 성공할 때까지 저속을 유지한다
        self._monitor_hmi = monitor_hmi
        self.supply_mode = cfg.get('supply_mode', 'slots')
        if self.supply_mode not in ('slots', 'scatter'):
            raise ValueError(f"robot.yaml supply_mode 는 slots · scatter 중 하나다: {self.supply_mode!r}")
        self._scatter = (ScatterFlow(cfg, BlockPicker(cfg['find']['min_gap_mm']), self.supply_mode, open_width_m)
                         if self.supply_mode == 'scatter' else None)
        self._hmi_seen_at = None                  # PC 사이 stamp 대신 이 PC 의 단조 시계로 수신 간격을 잰다
        self._hmi_pause = False                   # 잠깐 끊겼다 다시 붙어도 사람의 start 전에는 풀지 않는다
        self._hmi_recheck = False                 # WAIT_HMI 의 start 는 SELECT 에서 진행표부터 다시 확인한다
        self._scan_run = False                    # 스캔 CSV 는 남기되 조립 결과 build/1 로 보내지 않는다
        self._scan_poses, self._scan_index = (), 0
        for run_id, summary in self.logger.load_pending():
            self.pending_builds[run_id] = summary
            self._build_ready.add(run_id)         # 이미 디스크에 있다
        if self.pending_builds:
            LOG.warning('디스크에 보관된 미전송 요약 %d개를 되살렸다: %s', len(self.pending_builds), ', '.join(self.pending_builds))
        self._handlers = {'CHECK': self._check, 'SELECT': self._select, 'PICK_PLACE': self._pick_place,
                          'VERIFY': self._verify, 'WAIT_SUPPLY': self._wait_supply, 'STOPPED': self._stopped,
                          'RECOVER': self._recover, 'ERROR': self._error,
                          'SCAN_MOVE': self._scan_move, 'SCAN_CAPTURE': self._scan_capture, 'SCAN_INFER': self._scan_infer}

    # ---------- 콜백 스레드에서 들어오는 것 ----------
    def on_safety(self, st):
        """정지 노드의 safety_state/1(dict)을 적는다. 상태 전이는 작업 스레드가 run_once 에서 한다(콜백은 변수만)."""
        with self._lock:
            self.safety = st
            self.safety_count += 1
            if isinstance(st, dict) and st.get('stopped'):
                self._stop_count += 1
                if self._scatter is not None:
                    self._scatter.invalidate()        # 관측 · 조회 중이던 흩뿌림 답은 정지 뒤에 적용하지 않는다

    def on_gripper(self, st):
        """그리퍼 노드의 gripper_state/1(dict)을 적는다. grasped 가 참이면 블록을 쥐고 있다."""
        with self._lock:
            self.gripper = st

    def on_hmi_alive(self, body):
        """웹 생존 JSON 의 alive 가 정확히 true 일 때 수신 시각을 갱신한다. 깨진 값은 무시하고 외부 stamp 는 사용하지 않는다.

        재접속은 대기를 풀지 않는다. 현재 블록 처리 중 끊겼다가 돌아온 경우도 사람의 계속 버튼을 기다린다.
        """
        with self._lock:
            self.poll_hmi()                       # 갱신 전 끊김을 기억해야 빠른 재접속으로 대기 규칙이 사라지지 않는다
            if isinstance(body, dict) and body.get('alive') is True:
                self._hmi_seen_at = self._clock()

    def _hmi_fresh(self):
        """(잠금 안) 웹 생존 신호가 lost_after_s 초 안에 왔는가. 감시 안 하는 로컬 개발은 항상 참."""
        return not self._monitor_hmi or (self._hmi_seen_at is not None and
                                         self._clock() - self._hmi_seen_at < self.cfg['mqtt']['lost_after_s'])

    def poll_hmi(self):
        """타이머가 조립 중 웹 끊김을 기억한다(메모리만, 이동 · 상태 방송 없음). 작업 스레드가 블록을 끝낸 뒤 WAIT_HMI 로 간다."""
        with self._lock:
            if self.run_id and self.state in RUN_STATES and not self._hmi_fresh():
                self._hmi_pause = True

    def shutdown(self):
        """Ctrl+C 등 끝낼 때 부른다. 기다리는 중인 호출이 곧바로 빠져나오고 pick_place 목표가 취소된다."""
        self._shutdown = True

    def finalize(self, flush_s=2.0):
        """끝낼 때(Ctrl+C · 종료) 맨 마지막에 부른다 — 진행 중인 목표 취소를 요청한 **뒤에**. 조립 중이었으면 기록을 닫고
        (ERROR 중이면 ERROR, 그 밖은 STOPPED) 큐에 남은 줄이 파일에 쓰일 때까지 flush_s 초 안에서 기다린다.

        요약 확정은 메모리에서 바로 되고, 기다리는 것은 파일 쓰기뿐이라 디스크가 멈춰도 flush_s 를 넘기지 않는다.
        시간 안에 못 끝나면 경고를 남긴다(flush 의 True 는 '다 처리했다'이지 '저장 성공'이 아니라서 file_ok 도 함께 본다).
        """
        with self._lock:
            if self.logger.active:
                self._finish_run('ERROR' if self.state == 'ERROR' else 'STOPPED')
        if not self.logger.flush(flush_s):
            LOG.warning('기록 파일에 아직 다 안 써졌다(%.1f초 안에 못 끝냄) — 마지막 줄이 CSV 에 없을 수 있다. 요약(last_build)은 메모리에 있다', flush_s)

    # ---------- 결과 저장(save_build) — 보내는 일은 task 노드, 정하는 일은 여기 ----------
    def builds_to_send(self, now, retry_s):
        """지금 /d2/hmi/save_build 로 보낼 요약들 [(run_id, 요약 복사본)]. 보내는 것으로 적어 두므로 호출한 쪽이 꼭 보내야 한다.

        답을 기다리는 중인 것 · 마지막으로 보낸 지 retry_s(monotonic 초)가 안 된 것 · 끝내는 중이면 내지 않는다.
        now = time.monotonic(). 잠금 안에서 메모리만 만진다(네트워크 대기 없음).
        """
        with self._lock:
            if self._shutdown:
                return []
            out = []
            for run_id, summary in self.pending_builds.items():
                last = self._build_last_try.get(run_id)
                if run_id in self._build_inflight or (last is not None and now - last < retry_s):
                    continue
                if run_id not in self._build_ready and self._clock() - self._build_queued.get(run_id, 0.0) < retry_s:
                    continue                      # 디스크 보관이 끝나길 잠깐 기다린다. 디스크가 늦으면 retry_s 뒤에는 기다리지 않고 보낸다
                self._build_inflight.add(run_id)
                self._build_last_try[run_id] = now
                out.append((run_id, copy.deepcopy(summary)))
            return out

    def build_result(self, run_id, success, response_json):
        """save_build 의 답을 그 요청의 run_id 에만 적용한다. 반환: 저장이 확인돼 대기 목록에서 지웠으면 True.

        서비스 success 가 참이고 응답 JSON 이 객체이며 ok 가 참(true)일 때만 지운다. 그 밖(실패 · 깨진 JSON · ok 거짓)은 요약을 남긴다.
        이미 지워졌거나 모르는 run_id 의 늦은 답은 아무것도 안 바꾼다 — 다른 조립의 요약은 건드리지 않는다.
        """
        with self._lock:
            self._build_inflight.discard(run_id)
            if run_id not in self.pending_builds:
                return False
            if success is not True or not self._response_ok(response_json):
                LOG.warning('결과 저장 실패(run %s) — 요약을 남기고 나중에 다시 보낸다: success=%r %.80r', run_id, success, response_json)
                return False
            del self.pending_builds[run_id]
            self._build_last_try.pop(run_id, None)
            self._build_ready.discard(run_id)
            self._build_queued.pop(run_id, None)
            self.logger.delete_pending(run_id)      # 저장이 확인된 그 run 의 보관 파일만 지운다(기록 스레드가)
            return True

    def _pending_persisted(self, run_id, ok):
        """(기록 스레드가 부른다) 미전송 요약의 디스크 보관이 끝났다. 성공 · 실패 모두 보낼 수 있게 한다(실패하면 메모리 재전송만)."""
        with self._lock:
            if run_id in self.pending_builds:       # 보관이 늦어 메모리로 보내 이미 저장 성공 처리한 run_id 는 다시 넣지 않는다
                self._build_ready.add(run_id)

    def build_failed(self, run_id, why):
        """save_build 요청이 못 나갔거나 시간 안에 답이 없었다. 요약은 남기고 다시 보낼 수 있게 한다."""
        with self._lock:
            self._build_inflight.discard(run_id)
        LOG.warning('결과 저장을 못 했다(run %s): %s — 요약을 남긴다', run_id, why)

    @staticmethod
    def _response_ok(response_json):
        """응답 JSON 글자가 객체이고 ok 가 true(참인 값이 아니라 정확히 true)인가."""
        try:
            body = json.loads(response_json)
        except (TypeError, ValueError):
            return False
        return isinstance(body, dict) and body.get('ok') is True

    def halted(self):
        """정지 신호 · 종료 · 시간 초과 뒤 정지 확인 대기면 참. io 가 새 동작을 막고 기다림을 중단할 때 쓴다."""
        st = self.safety
        return self._shutdown or self._timeout_pending or bool(st and st.get('stopped'))

    def command(self, cmd, design_id=''):
        """화면 버튼 명령(HmiCommand, 다리가 웹 화면 대신 부른다). 반환: (success, reason).

        select_design = 설계를 **조회**(/d2/hmi/get_design)하고 시작 점검 → READY. start = READY 에서 [설계 선택] 때 받아 둔 설계로 출발(점검 한 번 더) → CHECK(다시 조회하지 않는다),
        WAIT_SUPPLY 에서는 [계속](관측 자세 도착 뒤), WAIT_HMI 는 웹 재연결 뒤 진행 확인부터 SELECT 로 이어 간다(조회 없음).
        scan 은 IDLE·SCAN_REVIEW 에서 새 촬영을 시작한다. cancel 은 READY·ERROR·SCAN_REVIEW 를 IDLE 로 돌린다(음성 cancel 과 같다, S-16).
        조회는 이 호출 스레드가 잠금 **밖에서** 기다린다(최대 timeout.service_s). 조회 실패 · 시간 초과 · 잘못된 답이면 새 조립을 시작하지 않고
        거절한다(로컬 파일로 몰래 대신하지 않는다 — 개발용 로컬은 노드가 design_source=local 로 명시해야 한다).
        reason 은 IRD 7장에 이미 있는 코드만 쓴다: 운전 중 'BUSY', 정지 중 'STOPPED', 조회 시간 초과 'TIMEOUT'. 맞는 코드가 없는 거절(설계 없음 ·
        점검 실패 · 아직 없는 명령)은 reason 을 빈 값으로 두고 이유를 state/1 의 message 글자로만 알린다.
        """
        if cmd == 'select_design':
            return self._lookup_then(design_id, 'select')
        if cmd == 'start':
            return self._lookup_then(design_id, 'start')
        with self._lock:
            if cmd == 'scan':
                return self._begin_scan()
            if cmd == 'cancel':
                if not self._cancel_to_idle():
                    return self._reject(self._busy_reason(), '준비·오류·스캔 검토 상태에서만 취소할 수 있다')
                return True, ''
            return self._reject('', f'모르는 명령: {cmd}')

    def on_intent(self, intent, design_id=None):
        """음성 의도(intent/1, 다리가 ROS 토픽으로 낸다). start·select_design은 버튼과 같고 cancel은 READY·ERROR·SCAN_REVIEW에서 받는다.

        그 밖(request_design 등)은 무시. 출발할 수 없는 때의 음성 start 는 무시하고 알림 voice_start_ignored 만 낸다(IRD 8.5).
        설계 조회가 있는 동작은 잠금 밖에서 command 로 한다.
        """
        if intent == 'select_design' and design_id:
            self.command('select_design', design_id)
        elif intent == 'start':
            with self._lock:
                can = self.state in ('READY', 'WAIT_SUPPLY', 'WAIT_HMI') or (self.state == 'IDLE' and design_id)
                if not can:
                    self._publish('voice_start_ignored', '지금은 음성 출발을 받을 수 없어 무시했다')
                    return
            ok, _ = self.command('start', design_id or '')
            with self._lock:
                if not ok and self.state == 'WAIT_SUPPLY':      # 아직 관측 자세로 가는 중이라 못 받았다
                    self._publish('voice_start_ignored', '관측 자세로 가는 중이라 음성 출발을 무시했다')
        elif intent == 'cancel':
            with self._lock:
                self._cancel_to_idle()

    def _cancel_to_idle(self):
        """(잠금 안) READY·ERROR·SCAN_REVIEW 에서 취소해 IDLE 로 간다. 반환: 취소했나. 화면 cancel 과 음성 cancel 이 같이 쓴다(S-16).

        ERROR 에서는 조립 기록을 ERROR 로 닫는다. 운전 중에는 받지 않는다. 로봇은 움직이지 않는다.
        """
        if self.state not in ('READY', 'ERROR', 'SCAN_REVIEW'):
            return False
        if self.state == 'ERROR':
            self._finish_run('ERROR')
        self.planner, self.design_id, self.block_id = None, None, None
        self._clear_run()
        self._set('IDLE', None, '취소했다')
        return True

    # ---------- 작업 스레드 ----------
    def run_once(self):
        """지금 상태의 일을 한 단계만 하고 돌아온다(작업 스레드가 되풀이해서 부른다).

        정지 신호를 매번 먼저 본다. WAIT_HMI 는 서기 정지가 아니므로 현재 액션은 중단하지 않는다.
        단계 안에서 예상 못 한 예외가 나면 로그를 남기고 ERROR 로 간다(팔은 pick_place 가 서 있게 한다).
        """
        self._apply_safety()
        self.poll_hmi()
        with self._lock:
            if self._hmi_pause and self.state in ('CHECK', 'SELECT', 'PICK_PLACE', 'WAIT_SUPPLY'):
                self._enter_wait_hmi()
        handler = self._handlers.get(self.state)
        if handler is None:
            return
        try:
            handler()
        except Exception as e:        # noqa: BLE001 — 어떤 오류든 멈추고 사람을 부른다
            LOG.exception('상태 %s 에서 예상 못 한 오류', self.state)
            self._to_error(f'예상 못 한 오류({e!r})')

    # ---------- 상태별 일 ----------
    def _check(self):
        """CHECK: 관측 자세로 → 모든 블록을 진행 확인 → 어긋남이 있으면 ERROR, 없으면 SELECT."""
        ok, why = self._move_observe()
        if not ok:
            return self._failed(why, '관측 자세 이동')
        if not self._observe([b['block_id'] for b in self.planner.blocks]):
            return None
        problems = self.planner.judge()
        if problems:
            return self._offset_over(problems)
        self._set('SELECT', None, '')
        return None

    def _move_observe(self):
        """관측 자세 이동 결과(ok, reason). 복구 뒤 첫 이동은 저속을 유지하고 성공한 뒤에만 해제한다."""
        if self._recover_slow:
            ok, why = self.io.move_to('observe', self.halted, speed_ratio=self.cfg['recover']['speed_ratio'])
        else:
            ok, why = self.io.move_to('observe', self.halted)
        if ok:
            self._recover_slow = False
        return ok, why

    def _select(self):
        """SELECT: 작업 판단에 다음 블록을 묻는다. 있으면 PICK_PLACE, 끝이면 DONE, 못 본 블록은 재관측 1번 뒤 ERROR."""
        if self._hmi_recheck:
            ok, why = self._move_observe()
            if not ok:
                return self._failed(why, '웹 재연결 뒤 관측 자세 이동')
            if not self._observe([b['block_id'] for b in self.planner.blocks]):
                return
            problems = self.planner.judge(expect_present=[self.placed] if self.placed else ())
            if problems:
                return self._offset_over(problems)
            if self.placed:
                row = self.planner.progress[self.placed]
                self.logger.record_measure(self.placed, row['dx_m'], row['dy_m'], row['dz_m'])
            self._hmi_recheck = False
            self.poll_hmi()
            if self._hmi_pause:
                return self._enter_wait_hmi()
        r = self._next_goal()
        if r is None:                          # 흩뿌림은 전환까지 끝냈거나(집기 · 대기 · 오류) 이동 · 조회 실패를 처리했거나 정지 뒤 늦은 답을 버렸다
            return None
        status = r['status']
        if status == 'FOUND':
            self._start_goal(r)
            self._set('PICK_PLACE', None, f'{r["block_id"]} 를 집으러 갑니다')
        elif status == 'DONE':
            with self._lock:               # DONE 방송과 요약 확정 사이에 새 설계 선택 · 출발이 끼어들지 못하게 한 잠금 안에서(파일 I/O 없음)
                self.block_id = None
                self._set('DONE', 'done', '조립이 끝났어요')
                self._finish_run('DONE')
        elif status == 'UNKNOWN_BLOCK' and not self._reobserved:
            self._reobserved = True
            self._set('CHECK', None, f'{r["block_id"]} 가 있는지 못 봐서 한 번 더 봅니다')
        elif status == 'WAIT_SUPPLY':
            self._enter_wait_supply(r['block_id'], r.get('alert'))
        else:                                  # NO_SUPPORT · 재관측 뒤에도 UNKNOWN_BLOCK
            self._to_error(f'{r["block_id"]}: {status}')

    def _next_goal(self, pick_message='{block} 를 집으러 갑니다'):
        """다음 집기 목표를 공급 방식에 맞게 정한다(목표 블록은 작업 판단이, 어디서 집을지는 방식이 정한다).

        slots: planner.next_block() 를 그대로 돌려준다(부르는 쪽이 목표를 적용한다).
        scatter: planner.next_target() 로 블록을 정하고, 집을 블록이 있으면 _scatter_goal 이 관측 → 후보 고르기 → **목표 적용과 PICK_PLACE 전환까지**
        한 잠금 안에서 끝내고 None 을 돌려준다. 블록이 없는 경우(DONE · NO_SUPPORT · UNKNOWN_BLOCK)는 그 status dict 를 돌려준다.
        None = 이미 처리했다(전환 · 이동/조회 실패 · 설정 미완 · 정지 뒤 늦은 답 버림) — 부르는 쪽은 아무것도 더 하지 않는다.
        """
        if self._scatter is None:
            return self.planner.next_block()
        target = self.planner.next_target()
        if target['status'] != 'FOUND':
            return target
        self._scatter_goal(target, pick_message)
        return None

    def _scatter_goal(self, target, pick_message):
        """흩뿌린 공급에서 target 블록을 집을 후보를 고르고 결과를 적용한다: 설정 점검 → observe_supply 이동 → find_blocks → ScatterFlow 고르기.

        설정(열림 폭)이 안 정해졌으면 **로봇을 움직이기 전에** ERROR 로 끝낸다. 이동 · 조회는 잠금 밖에서 기다린다.
        적용(commit)과 그 결과(목표 적용 · PICK_PLACE 또는 WAIT_SUPPLY 전환)는 **한 잠금 안에서** 한다 — 정지 신호(on_safety)는 같은 잠금을 얻어야
        반영되므로 commit 확인 뒤 전환 전에 정지가 끼지 못한다. commit 은 요청 번호 · 상태 변화 · 정지 횟수 · 정지 중 여부를 그 안에서 다시 확인한다
        (정지가 끼었거나, 정지 뒤 [다시 시작]까지 끝났어도 그 전 요청의 늦은 답은 적용하지 않는다). 로봇은 move_to 외에 움직이지 않는다.
        """
        if not self._scatter.configured:
            self._to_error('흩뿌린 공급 설정(열림 폭)이 아직 정해지지 않았다')
            return
        stops_before = self._stop_count
        ticket = self._scatter.begin((self._epoch, stops_before))
        ok, why = self.io.move_to('observe_supply', self.halted)
        if not ok:
            self._failed(why, '공급 관측 자세 이동')
            return
        ok, why, body = self.io.find_blocks(self.run_id, self.halted)
        if not ok or self.halted():
            self._failed('STOPPED' if self.halted() else why, 'find_blocks 조회')
            return
        result = self._scatter.prepare(target, body)
        with self._lock:
            applied = self._scatter.commit(ticket, lambda: ((self._epoch, self._stop_count), self.halted()), result)
            if applied:
                self._apply_scatter_result(target, result, pick_message)
                return
        if self._stop_count != stops_before:       # 조회 중에 정지가 있었다(이미 [다시 시작]까지 끝났어도) → 다른 정지 때처럼 STOPPED 절차로
            self._failed('STOPPED', 'find_blocks 조회 중 정지')

    def _apply_scatter_result(self, target, result, pick_message):
        """(잠금 안) commit 이 적용된 흩뿌림 결과를 상태에 반영한다: PICK → 목표 + PICK_PLACE, 비었음 · 맞는 블록 없음 → WAIT_SUPPLY, 그 밖은 ERROR."""
        status, block_id = result['status'], target['block_id']
        if status == 'PICK':
            self._start_goal(dict(result['request'], status='FOUND'))
            self._set('PICK_PLACE', None, pick_message.format(block=block_id))
        elif status == 'EMPTY':
            self._enter_wait_supply(block_id, ('supply_empty', '공급 영역에 블록이 없어요. 블록을 흩뿌린 뒤 [계속]을 누르세요'))
        elif status == 'NO_MATCH':
            counts = result['counts']
            only_tilted = counts.get('tilted', 0) > 0 and all(n == 0 for k, n in counts.items() if k != 'tilted')
            # 기울어진 블록만 남았으면 IRD 알림 tilted_block. 그 밖의 '맞는 블록 없음'은 message_id 없이 글자만 — 새 이름(no_match_block)을 둘지는 PL 선택
            self._enter_wait_supply(block_id, ('tilted_block' if only_tilted else None, result['message'] + ' — 정리한 뒤 [계속]을 누르세요'))
        elif status == 'NOT_CONFIGURED':
            self._to_error('흩뿌린 공급 설정(열림 폭)이 아직 정해지지 않았다')
        else:                                      # LOOKUP_FAILED — 횟수 상한 정책이 정해지기 전이라 다시 시도하지 않고 사람을 부른다
            self._to_error(f'find_blocks 응답을 쓸 수 없다({result.get("reason", status)})')

    def _pick_place(self):
        """PICK_PLACE: 목표 하나를 pick_place 로 보내고 결과를 SDD 7.1 표대로 처리한다.

        요청마다 새 번호를 만들어 피드백 step · 결과 수신 시각(time.time())을 모으고, 끝나면 self.last_pick 에 남긴다
        ({request_id, ok, reason, step, result_at} — result_at 이 None 이면 결과 없이 끝난 것). 수집만 하고 분기에는 아직 쓰지 않는다.
        """
        self._pick_seq += 1
        rid = self._pick_seq
        self._steps.begin(rid)                 # 이 요청의 피드백 · 결과만 받는다. 이전 요청의 늦은 것은 번호가 달라 버려진다
        ok, why = self.io.pick_place(self.goal, self.halted,
                                     on_feedback=lambda step: self._steps.on_feedback(rid, step),
                                     on_result=lambda at: self._steps.on_result(rid, at))
        # 결과 처리에 넘길 수집값. result_at 이 None 이면 결과를 못 받고 끝난 것(거절 · 시간 초과 · 우리 쪽 취소) — 이유(why)로 어느 경우인지 가른다
        self.last_pick = {'request_id': rid, 'ok': ok, 'reason': why,
                          'step': self._steps.last_step(rid), 'result_at': self._steps.result_at(rid)}
        self._steps.end(rid)                   # 이 번호의 늦은 피드백이 다음 요청에 섞이지 않게 닫는다
        if ok:
            self.placed = self.goal['block_id']
            self.logger.record_placed(self.placed, self.goal['supply_slot'])
            self.poll_hmi()
            if self._hmi_pause:
                return self._enter_wait_hmi()
            self._set('VERIFY', None, f'{self.placed} 를 놓았다. 확인합니다')
            return None
        self.logger.log('fail', self.goal['block_id'], f'pick_place {why}')
        if self._scatter is not None and why in ('PLAN_FAILED', 'GRASP_FAILED', 'SLOT_EMPTY'):
            # 흩뿌림은 실패 뒤 다시 계획 · 다시 관측(failure_action)을 아직 연결하지 않았다 → 서서 사람을 부른다(실제 흩뿌림 실행 보류)
            return self._to_error(f'{self.goal["block_id"]}: {why}(흩뿌림 집기 실패 뒤 재시도는 아직 연결 전)')
        if why == 'PLAN_FAILED':
            if self._plan_retry < 1:           # 같은 목표로 다시 계획 1번
                self._plan_retry += 1
            else:
                self._to_error(f'{self.goal["block_id"]}: PLAN_FAILED(다시 계획 1번도 실패)')
        elif why == 'GRASP_FAILED':
            self._grasp_failed()
        elif why == 'SLOT_EMPTY':
            self._slot_empty()
        else:
            # TIMEOUT 은 정지 노드에 요청하고 상태를 확인한다. 그 밖의 정식 코드 · 모르는 코드는 공통 분기를 탄다
            self._failed(why, f'{self.goal["block_id"]} 집기·놓기', log=False)

    def _verify(self):
        """VERIFY: 관측 자세에서 방금 놓은 블록과 그 받침만 다시 보고, 없거나 어긋났으면 ERROR."""
        ok, why = self.io.move_to('observe', self.halted)
        if not ok:
            return self._failed(why, '관측 자세 이동')
        block = next(b for b in self.planner.blocks if b['block_id'] == self.placed)
        if not self._observe([self.placed] + block['supports']):
            return None
        row = self.planner.progress[self.placed]      # 판정 전에 측정값부터 남긴다(어긋나서 ERROR 가 되어도 값은 기록)
        self.logger.record_measure(self.placed, row['dx_m'], row['dy_m'], row['dz_m'])
        problems = self.planner.judge(expect_present=[self.placed])
        if problems:
            return self._offset_over(problems)
        self._set('SELECT', None, '')
        return None

    def _wait_supply(self):
        """WAIT_SUPPLY: 먼저 관측 자세로 1번 간다. 도착한 뒤에야 '공급 채워 주세요'(supply_empty)를 방송하고 [계속]을 받는다.

        [계속](start)이 오면 공급 칸을 다 있음으로 되돌려 같은 블록으로 PICK_PLACE 를 다시 시작한다(SELECT 로 돌아가지 않는다).
        도착 전에 온 start 는 command 가 거절하므로 여기서는 진행하지 않는다.
        """
        if not self._supply_moved:
            ok, why = self.io.move_to('observe', self.halted)
            if not ok:
                return self._failed(why, '공급 대기 자세 이동')
            with self._lock:
                self._supply_moved = True
                self._publish('supply_empty', '공급 칸이 비었어요. 채운 뒤 [계속]을 누르세요')
            return None
        with self._lock:
            if not self._pending_start:
                return None
            self._pending_start = False
        if self._scatter is not None:
            self._scatter.after_wait_start()        # 흩뿌림은 다시 관측하고 조회한다. 공급 칸 표시는 건드리지 않는다
        else:
            self.planner.supply_refilled()
        r = self._next_goal('공급을 채웠다. {block} 를 이어서 집습니다')
        if r is None:                          # 흩뿌림은 전환까지 끝냈다(다시 기다림 · 집기 · 오류)
            return None
        if r['status'] != 'FOUND':
            return self._to_error(f'공급을 채웠는데 다음 블록을 못 골랐다({r["status"]})')
        self._start_goal(r)
        self._set('PICK_PLACE', None, f'공급을 채웠다. {r["block_id"]} 를 이어서 집습니다')
        return None

    def _stopped(self):
        """STOPPED: 들어온 뒤 잠금이 풀린 safety/state 가 새로 오면 RECOVER 로 간다(SDD 5장)."""
        with self._lock:
            if self._unlocked_since(self._entered_count):
                self._set('RECOVER', None, '다시 시작 — 쥔 블록을 확인합니다')

    def _recover(self):
        """RECOVER: 조립 중이었으면 쥔 블록이 없는지 확인(gripper/state)하고 진행 확인부터 다시 한다. 쥐고 있거나 모르면 ERROR.

        조립 중이 아니었으면(IDLE · READY · DONE 에서 정지) IDLE 로 간다 — SDD 5장의 비조립 RECOVER → IDLE 규칙(IRD 12장 W121 다시 확인 대상).
        조립 중 복구는 첫 관측 이동에 recover.speed_ratio 를 쓰며, 실패했으면 다음 시도도 저속을 유지한다.
        """
        if not self._assembling:
            self.planner, self.design_id, self.block_id = None, None, None
            self._clear_run()
            self._set('IDLE', None, '정지가 풀렸어요. 설계를 다시 고르세요')
            return None
        grip = self.gripper
        if grip is None or grip.get('grasped'):
            return self._to_error('블록을 쥐고 있거나 그리퍼 상태를 모른다. 사람이 확인한 뒤 [다시 시작]')
        self._recover_slow = True
        self._set('CHECK', None, '진행 확인부터 다시 합니다')
        return None

    def _error(self):
        """ERROR: 사람이 확인할 때까지 기다린다. 정지 확인 대기 중이면 stopped 신호 전에는 복구를 막는다.

        정지 노드의 resume 은 잠겨 있지 않아도 safety/state 를 다시 내보내므로(최신 main safety_stop_node), 그것이 신호다.
        """
        with self._lock:
            if self._timeout_pending:
                return                         # 정지 상태를 한 번도 못 받았다면 새 unlocked 신호도 복구 허가가 아니다
            if self._unlocked_since(self._entered_count):
                self._assembling = self.planner is not None and not self._scan_run
                self._set('RECOVER', None, '다시 시작 — 쥔 블록을 확인합니다')

    # ---------- 스캔: 네트워크 대기는 잠금 밖 ----------
    def _begin_scan(self):
        """잠금 안에서 IDLE·SCAN_REVIEW의 scan 명령을 점검하고 새 run으로 시작한다. 로봇 이동은 작업 스레드에서 한다.

        설정은 robot.yaml scan.poses, 단위 없음. 점검 실패는 현재 상태·기록을 보존하며 (False, reason)을 반환한다.
        """
        if self.state not in ('IDLE', 'SCAN_REVIEW') or self._shutdown:
            return self._reject(self._busy_reason(), '지금은 스캔을 시작할 수 없다')
        poses = self.cfg.get('scan', {}).get('poses')
        if not isinstance(poses, list) or not poses or any(
                not isinstance(p, str) or f'{p}_pose' not in self.cfg for p in poses) or len(set(poses)) != len(poses):
            return self._reject('ERROR', 'scan.poses가 없거나 촬영 자세 설정과 맞지 않는다')
        ok, code, text = self._precheck(scan=True)
        if not ok:
            return self._reject(code, f'스캔 시작 점검 실패: {text}')
        self._clear_run()
        self.planner, self.design_id, self.block_id, self.placed = None, None, None, None
        self._scan_run = True
        self._scan_poses, self._scan_index = tuple(poses), 0
        self.run_id = self.logger.start_run('', 0)     # 구조가 아직 설계가 아니므로 CSV 만 열고 build/1 은 만들지 않는다
        self.logger.log('scan', None, 'start')
        self._set('SCAN_MOVE', 'scan_running', '스캔 촬영 자세로 갑니다')
        return True, ''

    def _scan_move(self):
        """현재 scan.poses 자세로 이동한다(m·rad는 motion 담당). 실패는 공통 정지·오류 처리, 성공 뒤에만 촬영 상태로 간다."""
        pose = self._scan_poses[self._scan_index]
        ok, why = self.io.move_to(pose, self.halted)
        if not ok:
            return self._failed(why, '스캔 자세 이동')
        with self._lock:
            if self.halted():
                return self._failed('STOPPED', '스캔 자세 이동')
            self._set('SCAN_CAPTURE', 'scan_running', f'{pose}에서 촬영합니다')

    def _scan_capture(self):
        """현재 자세·run_id로 점군을 수집한다. points는 비음수 정수. 잘못된 답은 SCAN_FAILED, 정지 뒤 답은 버린다."""
        pose = self._scan_poses[self._scan_index]
        ok, why, body = self.io.scan_capture(pose, self.run_id, self.halted)
        if not ok or self.halted():
            return self._failed('STOPPED' if self.halted() else why, '스캔 촬영')
        points = body.get('points') if isinstance(body, dict) else None
        if not isinstance(body, dict) or body.get('ok') is not True or isinstance(points, bool) or not isinstance(points, int) or points < 0:
            return self._failed('SCAN_FAILED', '스캔 촬영 응답')
        with self._lock:
            if self.halted():
                return self._failed('STOPPED', '스캔 촬영 결과 적용')
            self.logger.log('scan_capture', None, f'{pose}: {points}')
            self._scan_index += 1
            self._set('SCAN_MOVE' if self._scan_index < len(self._scan_poses) else 'SCAN_INFER', 'scan_running', '스캔 촬영을 마쳤습니다')

    def _scan_infer(self):
        """모은 점군을 추론해 scan_result/1로 방송한 뒤 검토를 기다린다. 단위 mm, 로봇 이동 없음.

        구조 합격 검사는 HMI의 check_design 요청 때 한다. 여기서는 JSON 형식·필수 값만 확인하며 잘못된 답은 SCAN_FAILED.
        """
        ok, why, body = self.io.scan_infer(self.run_id, self.halted)
        if not ok or self.halted():
            return self._failed('STOPPED' if self.halted() else why, '스캔 추론')
        try:
            blocks = body['blocks']
            if body.get('ok') is not True or not isinstance(blocks, dict) or blocks.get('schema') != 'blocks/1':
                raise ValueError('blocks/1 객체가 아니다')
            if any(not isinstance(blocks.get(k), str) or not blocks[k] for k in ('design_id', 'family')):
                raise ValueError('design_id · family 가 없다')
            rows = blocks.get('blocks')
            if not isinstance(rows, list) or not rows:
                raise ValueError('추론 블록이 없다')
            orders = set()
            for b in rows:
                order = b['order']
                if isinstance(order, bool) or not isinstance(order, int) or order <= 0 or order in orders:
                    raise ValueError('블록 order 가 잘못됐다')
                orders.add(order)
                if b.get('ori') not in ('x', 'y', 'xe', 'ye', 'zx', 'zy') or not isinstance(b.get('inferred', False), bool):
                    raise ValueError('블록 ori · inferred 가 잘못됐다')
                if any(isinstance(b[k], bool) or not isinstance(b[k], (int, float)) for k in ('x', 'y', 'z')):
                    raise ValueError('블록 좌표가 숫자가 아니다')
            count = body['inferred_count']
            if isinstance(count, bool) or not isinstance(count, int) or not 0 <= count <= len(rows) or not isinstance(body['image_path'], str):
                raise ValueError('inferred_count · image_path 가 잘못됐다')
            cloud = body.get('cloud_path', '')                # 화면 점군 창용 PLY 경로(로봇 PC). 선택 칸(10/8 PL, IRD 4.2 · 6장): 없거나 빈 글자 = 점군 없음 → 스캔은 그대로, 결과에서 뺀다
            if not isinstance(cloud, str):                    # 글자가 아닌 값(null · 숫자 · 목록 …)만 잘못된 응답이다 → SCAN_FAILED
                raise ValueError('cloud_path 가 글자가 아니다')
            result = dict(schema='scan_result/1', run_id=self.run_id, blocks=copy.deepcopy(blocks),
                          inferred_count=count, image_path=body['image_path'], poses_used=list(self._scan_poses))
            if cloud:
                result['cloud_path'] = cloud                  # 비어 있지 않은 글자는 그대로 넘긴다(작업 관리자는 파일을 열지 않는다)
            json.dumps(result, allow_nan=False)       # NaN · Infinity 를 방송·저장 가능한 결과로 넘기지 않는다
        except (KeyError, TypeError, ValueError) as e:
            return self._to_error(f'SCAN_FAILED: 추론 응답이 잘못됐다({e})')
        with self._lock:
            if self.halted():
                return self._failed('STOPPED', '스캔 추론 결과 적용')
            self.io.publish_scan_result(result)
            self._finish_run('DONE')
            self._set('SCAN_REVIEW', 'scan_review', '스캔 결과를 확인하고 저장·다시 스캔·취소를 선택하세요')

    # ---------- 실패 · 공급 처리 ----------
    def _grasp_failed(self):
        """GRASP_FAILED: 같은 칸 1번 더 → 다음 칸 1개 → 거기서도 실패하면 ERROR. 모든 칸을 돌지 않는다.

        다음 칸이 없으면(next_block 이 WAIT_SUPPLY 로 알려도) 공급이 빈 것이 아니라 재시도 칸이 없는 것이므로
        '공급 채워 주세요'를 띄우지 않고 ERROR 로 간다(SLOT_EMPTY 의 WAIT_SUPPLY 와 섞지 않는다).
        """
        self._grasp_stage += 1
        block = self.goal['block_id']
        if self._grasp_stage == 1:             # 같은 칸으로 다시(상태 그대로)
            return None
        if self._grasp_stage == 2:
            r = self.planner.next_block(avoid_slots=(self.goal['supply_slot'],))
            if r['status'] == 'FOUND' and r['block_id'] == block:
                self.goal = r
                return None
            return self._to_error(f'{block}: GRASP_FAILED(같은 칸 재시도 뒤 쓸 다른 칸이 없다)')
        return self._to_error(f'{block}: GRASP_FAILED(다음 칸에서도 실패)')

    def _slot_empty(self):
        """SLOT_EMPTY: 그 칸을 빔으로 적고 같은 블록의 다음 칸으로. 맞는 칸이 다 비었으면 진짜 WAIT_SUPPLY 로 간다."""
        self.planner.mark_slot_empty(int(self.goal['supply_slot']))
        r = self.planner.next_block()
        if r['status'] == 'FOUND' and r['block_id'] == self.goal['block_id']:
            self.goal, self._grasp_stage = r, 0
        elif r['status'] == 'WAIT_SUPPLY':
            self._enter_wait_supply(r['block_id'])
        else:
            self._to_error(f'공급 칸을 바꾸는 중 이상한 결과({r["status"]})')

    def _enter_wait_supply(self, block_id, alert=None):
        """WAIT_SUPPLY 상태로 들어간다. 칸 방식은 알림(supply_empty)을 아직 안 낸다 — 관측 자세에 도착한 뒤 _wait_supply 가 낸다.

        흩뿌림은 이미 observe_supply 에 와 있으므로 alert=(message_id, 글자)를 주면 도착 확인 없이 그 알림으로 바로 들어간다.
        """
        with self._lock:
            self.block_id = block_id
            self._pending_start = False        # 이전에 눌린 [계속]은 버린다
            if alert is not None:
                self._supply_moved = True
                self._set('WAIT_SUPPLY', alert[0], alert[1])
            else:
                self._supply_moved = False
                self._set('WAIT_SUPPLY', None, '공급이 필요해요. 관측 자세로 갑니다')

    def _enter_wait_hmi(self):
        """현재 동작이 끝난 뒤 WAIT_HMI 를 알린다. 정지 요청 · 공급 초기화 · 조립 요약 확정은 하지 않는다."""
        with self._lock:
            if self.state != 'WAIT_HMI':
                self._pending_start = False
                self._set('WAIT_HMI', 'hmi_lost', '웹 연결이 끊겼어요. 다시 연결한 뒤 [계속]을 누르세요')

    def _failed(self, reason, where, log=True):
        """이동 · 관측 · 집기 실패를 처리한다. TIMEOUT 은 잠금 밖에서 정지 요청 후 상태 확인, 정지류는 STOPPED, BUSY 는 재시도.

        모르는 reason(IRD 7장에 없는 문자열)도 이름을 따로 다루지 않고 일반 실패로 ERROR 에 보낸다. 글자는 알림에 그대로 적는다.
        """
        if log:
            self.logger.log('fail', self.block_id, f'{where}: {reason or "이유 없음"}')
        if reason == 'TIMEOUT':
            with self._lock:
                self._timeout_pending = True
            try:
                ok, message = self.io.request_stop('TIMEOUT')    # 파일 기록 대기 · 상태 복구보다 먼저, 잠금 밖에서 보낸다
            except Exception as e:              # 정지 요청 자체가 실패해도 다음 이동을 막고 사람에게 알린다
                ok, message = False, repr(e)
            with self._lock:
                if self.safety and self.safety.get('stopped'):
                    self._enter_stopped()
                else:
                    self._to_error(f'{where}: TIMEOUT — 정지 상태 확인 대기(요청 {"접수" if ok else "실패"}: {message})')
            return
        if reason in STOP_REASONS:             # 정지 노드의 신호를 기다린다
            self._enter_stopped()
        elif reason == 'BUSY':                 # 기다렸다 다시: 같은 상태에 머무르면 다음 단계가 다시 부른다
            return
        else:
            self._to_error(f'{where} 실패: {reason or "이유 없음"}')

    def _offset_over(self, problems):
        """판정에서 OFFSET_OVER 가 나왔다. 첫 문제를 알림에 적고 ERROR 로 간다."""
        first = problems[0]
        self._to_error(f'{first["block_id"]}: OFFSET_OVER — {first["detail"]}', 'offset_over')

    def _observe(self, block_ids):
        """블록들을 check_progress 로 보고 진행표에 넣고 progress 를 방송한다. 실패하면 알아서 상태를 바꾸고 False."""
        ok, why, rows = self.io.check_progress(block_ids, self.halted)
        if not ok:
            self._failed(why, '진행 확인')
            return False
        try:
            self.planner.update_progress(rows)
        except ValueError as e:
            self._to_error(f'진행 확인 답이 이상하다: {e}')
            return False
        self.io.publish_progress(self.planner.progress_message(self.run_id, time.time()))
        return True

    # ---------- 설계 고르기 · 출발 · 시작 점검 ----------
    def _start_lookup_target(self, design_id):
        """(잠금 안) 출발 명령이 설계 조회부터 해야 하면 그 design_id, 아니면 None(바로 _start 가 처리: [계속] · 거절).

        READY 에서는 [설계 선택] 때 받아 둔 설계를 그대로 쓴다(E-55 ①, 10/7 PL — 다시 조회하지 않는다). IDLE 에서 설계를 포함한 음성 start 만
        그 설계를 조회한다.
        """
        if self.state == 'IDLE' and design_id:
            return design_id
        return None

    def _lookup_then(self, design_id, mode):
        """설계를 조회한 뒤 mode 에 따라 적용한다(select = 고르기 → READY, start = 고르기(IDLE 이면) + 출발).

        ① 잠금 안: 지금 가능한 상태인지 보고 요청 번호 · 상태 표(epoch)를 적는다 ② **잠금 밖**: io.get_design 으로 기다린다
        ③ 잠금 안: 더 새 요청이 있거나 · 상태가 바뀌었거나 · 정지 · 종료 중이면 답을 버리고, 아니면 검증해서 적용한다.
        """
        received = self._clock()                       # 명령을 받은 순간 — timeout.command_s 안에 확정 못 하면 실행하지 않는다
        with self._lock:
            if mode == 'start':
                design_id = self._start_lookup_target(design_id)
                if design_id is None:
                    return self._start()
            if mode == 'select' and self.state not in ('IDLE', 'READY', 'DONE', 'SCAN_REVIEW'):
                return self._reject(self._busy_reason(), '지금은 설계를 바꿀 수 없다')
            if not design_id:
                return self._reject('', f'설계 파일을 못 읽었다: {design_id!r}')
            self._lookup_token += 1
            token, epoch, was_halted = self._lookup_token, self._epoch, self.halted()    # 이미 정지 중이면 시작 점검이 이유를 알린다
        try:
            ok, why, design = self.io.get_design(design_id, self.halted)
        except Exception as e:   # noqa: BLE001 — 조회가 어떻게 실패해도 새 조립을 시작하지 않는다
            ok, why, design = False, 'ERROR', None
            LOG.warning('설계 조회 중 예외: %r', e)
        with self._lock:
            if self._shutdown or (self.halted() and not was_halted):      # 조회하는 동안 정지 · 종료가 왔다
                return self._reject('STOPPED', '정지 · 종료 중이라 설계 조회 결과를 버렸다')
            if token != self._lookup_token or epoch != self._epoch:
                return self._reject('BUSY', '조회하는 동안 다른 요청이나 상태 변화가 있어 이 조회 결과를 버렸다')
            if self._clock() - received > self.command_s:      # 늦은 출발 · 선택은 허용하지 않는다(웹은 이미 시간 초과로 보였다)
                return self._reject('TIMEOUT', f'명령을 받은 뒤 {self.command_s} 초 안에 확정하지 못해 실행하지 않았다')
            if not ok:
                return self._reject(why, f'설계를 못 가져왔다: {design_id!r} ({why or "이유 없음"})')
            planner, problem = self._planner_from(design, design_id)
            if problem:
                return self._reject('', problem)
            if mode == 'select':
                return self._apply_select(planner, design_id)
            if self.state == 'IDLE':
                ok, why = self._apply_select(planner, design_id)
                if not ok:
                    return ok, why
            return self._start()

    def _planner_from(self, design, design_id):
        """조회 답(design/1)을 검증해 TaskPlanner 를 만든다. 반환: (planner, 문제 글자). 문제가 없으면 문제 글자는 빈 값.

        검증: 객체 · schema design/1 · 요청한 design_id 와 같음 · recipe 가 객체이고 schema 가 cad_recipe/1.0 · structure 가 객체(두 파일은
        block 이름으로 연결한다).
        레시피 내용은 TaskPlanner 가 robot.yaml 과 맞는지 본다. 잘못된 입력은 조립하지 않고 문제 글자를 반환한다.
        """
        if not isinstance(design, dict) or design.get('schema') != 'design/1':
            return None, '설계 조회 답이 design/1 이 아니다'
        if design.get('design_id') != design_id:
            return None, f'조회 답의 design_id({design.get("design_id")!r})가 요청({design_id!r})과 다르다'
        recipe = design.get('recipe')
        if not isinstance(recipe, dict) or recipe.get('schema') != RECIPE_SCHEMA:
            return None, f'조회 답의 recipe 가 {RECIPE_SCHEMA} 객체가 아니다'
        if not isinstance(design.get('structure'), dict):
            return None, '조회 답에 structure 가 없다'
        try:
            planner = TaskPlanner(self.cfg, recipe, design['structure'])
        except (ValueError, KeyError, TypeError) as e:
            return None, f'레시피를 못 읽는다: {e}'
        if not planner.blocks:
            return None, '레시피에 블록이 없다'
        return planner, ''

    def _apply_select(self, planner, design_id):
        """(잠금 안) 조회한 설계로 시작 점검을 통과하면 READY. IDLE · READY · DONE 에서만."""
        if self.state not in ('IDLE', 'READY', 'DONE', 'SCAN_REVIEW'):
            return self._reject(self._busy_reason(), '지금은 설계를 바꿀 수 없다')
        ok, code, text = self._precheck()
        if not ok:
            return self._reject(code, f'시작 점검 실패: {text}')
        self.planner, self.design_id, self.block_id, self.placed = planner, design_id, None, None
        self._clear_run()
        self._set('READY', 'ready_to_start', f'{design_id} 준비됐어요. 출발을 누르세요')
        return True, ''

    def _start(self):
        """(lock 안) READY 출발(→ CHECK), WAIT_SUPPLY · WAIT_HMI 에서 계속. 설계 조회는 _lookup_then 이 먼저 한다.

        WAIT_SUPPLY 에서는 관측 자세에 도착해 supply_empty 를 낸 뒤에만 받는다(도착 전 start 는 BUSY 로 거절, 진행시키지 않는다).
        """
        if self.state == 'WAIT_HMI':
            ok, code, text = self._precheck()
            if not ok:
                return self._reject(code, f'계속 점검 실패: {text}')
            self._hmi_pause, self._hmi_recheck = False, True
            self.logger.log('command', self.block_id, 'continue_hmi')
            self._set('SELECT', None, '웹이 연결됐어요. 진행 확인부터 이어갑니다')
            return True, ''
        if self.state == 'WAIT_SUPPLY':
            if not self._supply_moved:
                return self._reject('BUSY', '아직 관측 자세로 가는 중이에요. 도착한 뒤 [계속]을 누르세요')
            self._pending_start = True
            self.logger.log('command', self.block_id, 'continue')    # 같은 run_id 로 이어 간다
            return True, ''
        if self.state != 'READY':
            if self.state == 'IDLE':
                return self._reject('', '설계를 먼저 고르세요')
            return self._reject(self._busy_reason(), '지금은 출발할 수 없다')
        ok, code, text = self._precheck()
        if not ok:
            return self._reject(code, f'시작 점검 실패: {text}')
        self._reobserved = False
        self.placed = self.block_id = None
        self.run_id = self.logger.start_run(self.design_id, len(self.planner.blocks))   # 실제 출발 때만 새 run_id
        self._set('CHECK', None, '진행 확인을 시작합니다')
        return True, ''

    def _precheck(self, scan=False):
        """시작 점검(M-06) 중 숫자 없는 항목. 반환: (통과, 거절 reason, 이유 글자).

        ① 조립 또는 scan=True의 촬영·추론 서버가 떠 있다 ② 정지 노드 신호를 받았고 잠기지 않았다 ③ 그리퍼가 비었다.
        신호가 몇 초 안에 왔는지(신선도)는 숫자가 필요해서 보지 않는다. 사람이 영역 밖인지는 누르는 사람이 확인한다.
        """
        missing = self.io.services_ready(scan=True) if scan else self.io.services_ready()
        if missing:
            return False, '', f'서버가 안 떠 있다: {", ".join(missing)}'
        if self._timeout_pending:
            return False, 'STOPPED', '시간 초과 뒤 정지 상태 확인을 기다린다'
        if not self._hmi_fresh():
            return False, '', '웹 생존 신호를 못 받았어요. 연결을 확인하세요'      # IRD 7장 BUSY 는 '기다렸다 다시'라 웹이 자동 재시도한다 → 맞는 코드가 없어 비움(E-62)
        st = self.safety
        if st is None:
            return False, '', '정지 노드 신호를 아직 못 받았다'
        if st.get('stopped') or st.get('locked'):
            return False, 'STOPPED', '정지 상태다. [다시 시작]을 먼저 누르세요'
        if self.gripper is None:
            return False, '', '그리퍼 신호를 아직 못 받았다'
        if self.gripper.get('grasped'):
            return False, '', '그리퍼가 블록을 쥐고 있다'
        return True, '', ''

    # ---------- 상태 바꾸기 · 방송 ----------
    def _start_goal(self, found):
        """다음 블록 목표를 잡고 재시도 횟수를 비운다."""
        self.goal, self.block_id = found, found['block_id']
        self._plan_retry = self._grasp_stage = 0
        self._reobserved = False

    def _apply_safety(self):
        """정지 노드가 stopped 를 보냈으면 어느 상태에서든 STOPPED 로 간다."""
        with self._lock:
            if self.safety and self.safety.get('stopped') and self.state != 'STOPPED':
                self._enter_stopped()

    def _enter_stopped(self):
        """STOPPED 로 간다. 조립 중이었는지 적어 두고(RECOVER 가 쓴다), 눌려 있던 [계속]은 버린다."""
        with self._lock:
            self._timeout_pending = False
            if self.state == 'STOPPED':
                return
            self._assembling = self.state in RUN_STATES and not self._scan_run
            self._entered_count = self.safety_count
            self._pending_start = False
            self.logger.count_stop()
            self._set('STOPPED', 'stopped', '멈췄어요. 원인을 없앤 뒤 [다시 시작]을 누르세요')

    def _to_error(self, message, message_id=None):
        """ERROR 로 간다. 로봇은 서 있고 사람을 부른다. message_id 는 IRD 값만 쓰고 맞는 것이 없으면 null."""
        with self._lock:
            self._entered_count = self.safety_count
            self.logger.log('error', self.block_id, message)
            self._set('ERROR', message_id, message)

    def _unlocked_since(self, count):
        """(lock 안) count 번째 이후에 잠금이 풀린 safety/state 가 새로 왔나. [다시 시작]을 알아보는 규칙."""
        st = self.safety
        return bool(st) and self.safety_count > count and not st.get('stopped') and not st.get('locked')

    def _finish_run(self, result):
        """열린 CSV를 result로 닫는다. 조립만 build/1 요약을 보존하고 스캔은 DB 조립 목록에 넣지 않는다. run_id는 화면에 남긴다."""
        summary = self.logger.finish(result)
        if self._scan_run:
            return                                  # 스캔은 CSV 와 scan_result 로 남기고 조립 build/1 로 저장하지 않는다
        if summary is not None:
            with self._lock:
                self.last_build = summary
                run_id = summary['run_id']
                self.pending_builds[run_id] = copy.deepcopy(summary)    # 저장이 확인될 때까지 run_id 별로 남긴다
                self._build_queued[run_id] = self._clock()
                # 디스크 보관이 끝난 것만 보낸다(재시작 복구를 위해). 보관을 안 쓰는 설정이면 바로 보낼 수 있다(재시작 복구 안 됨)
                if not self.logger.save_pending(run_id, copy.deepcopy(summary), lambda ok, r=run_id: self._pending_persisted(r, ok)):
                    self._build_ready.add(run_id)

    def _clear_run(self):
        """run_id 를 비운다(새 설계 · IDLE). 닫히지 않은 기록이 있으면 STOPPED 로 닫는다."""
        if self.logger.active:
            self._finish_run('STOPPED')
        self.logger.clear()
        self.run_id = None
        if self._scatter is not None:
            self._scatter.invalidate()
        self._recover_slow = False
        self._hmi_pause = self._hmi_recheck = False
        self._scan_run = False

    def _busy_reason(self):
        """거절할 때 쓰는 기존 코드: 정지 중이면 STOPPED, 그 밖에는 BUSY."""
        return 'STOPPED' if self.state == 'STOPPED' else 'BUSY'

    def _reject(self, reason, message):
        """(lock 안) 명령을 거절하고 이유를 알림으로 낸다. 상태는 그대로."""
        LOG.info('거절(%s): %s', reason or '-', message)
        self._publish(None, message)
        return False, reason

    def _set(self, state, message_id=None, message=''):
        """상태를 바꾸고 /d2/task/state 로 알린다."""
        with self._lock:
            LOG.info('%s → %s %s', self.state, state, message)
            self.state = state
            self._epoch += 1
            self.logger.log('state', self.block_id, state)
            self._publish(message_id, message)

    def _publish(self, message_id, message):
        """state/1 한 건을 보낸다. run_id 는 조립 중(끝난 직후 포함)에만 값, message_id 는 IRD 2장 값 또는 null."""
        self.io.publish_state({'schema': 'state/1', 'stamp': time.time(), 'mode': 'auto', 'state': self.state,
                               'run_id': self.run_id, 'design_id': self.design_id, 'block_id': self.block_id,
                               'message_id': message_id, 'message': message})

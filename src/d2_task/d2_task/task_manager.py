# -*- coding: utf-8 -*-
"""작업 관리자 TaskManager — 자동 조립 상태표 · 실패 대응표 · 시작 점검 (ROS 없이 동작, SDD 5장 · 7.1).

task 는 로봇 PC 에서 돈다. 웹 화면 · 음성은 웹 PC 에 있고 다리(bridge)가 ROS 이름 그대로 대신 부르므로(E-26~E-28),
이 클래스는 MQTT 를 전혀 모른다. task 노드(task_node.py)가 콜백에서 on_* · command 를 부르고,
작업 스레드 하나가 run_once() 를 되풀이한다.
바깥 일(이동 · 관측 · 집기 · 방송)은 생성할 때 받은 io 객체로 한다. io 가 갖춰야 할 것(ROS 노드나 시험용 가짜):
  load_recipe(design_id) -> dict | None            레시피(assembly.recipe/1.0 또는 cad_recipe/1.0)를 읽는다. 없으면 None
  services_ready() -> [이름]                        아직 안 떠 있는 서버 이름들(없으면 빈 목록, 기다리지 않는다)
  move_to(target, should_abort) -> (ok, reason)
  check_progress(block_ids, should_abort) -> (ok, reason, rows)   rows = {block_id: {state, dx_m, dy_m, dz_m, top_z_m}}
  pick_place(goal, should_abort) -> (ok, reason)    goal = TaskPlanner.next_block 의 FOUND dict
  publish_state(dict) · publish_progress(dict)
move_to · check_progress · pick_place 는 지금 답을 기다리는 시간 제한을 두지 않는다. 제한 값은 W121에서 정해졌고(IRD 7장 TIMEOUT —
robot.yaml timeout.service_s 5 · move_to_s 30 · pick_place_s 90) 거는 것은 W119 에서 한다. 대신 기다리는 동안
should_abort() 가 참이 되면(정지 신호 · Ctrl+C) 곧바로 빠져나와 (False, 'STOPPED') 로 돌려주고, 진행 중인 pick_place 목표는 취소한다.

범위: 자동 조립 상태 11개 + 기록(CSV · run_id, W065 — RunLogger). 스캔 상태(SCAN_*) · get_design · save_build · 웹 끊김(hmi_lost)은 W119.
run_id 는 조립을 실제로 출발시킬 때(READY → CHECK) 새로 만든다. 정지 · 복구 · 공급 보충 뒤 계속할 때는 같은 ID 를 쓴다.
끝난 조립의 build/1 요약은 last_build(최근 결과 표시용)와 pending_builds(run_id 별 미전송 요약)에 둔다.
/d2/hmi/save_build 로 보내는 일은 task 노드가 하고(네트워크 대기는 이 클래스의 잠금 밖), 이 클래스는 무엇을 보낼지(builds_to_send) ·
답이 오면 어떻게 할지(build_result)만 정한다. 저장이 확인된 run_id 만 목록에서 지운다 — 실패 · 시간 초과 · 잘못된 답은 요약을 남긴다.
WAIT_SUPPLY(공급 채우기 · [계속])는 05 작업분류 W119 ②의 일부지만 SDD 5장 상태표에 있어 W044 에서 먼저 구현했다 — **W119 완료가 아니다**
(W119 는 get_design · 스캔 흐름 · DB 점검 · builds · 웹 끊김이 더 남아 있다).
아직 구현하지 않은 것(W119): ① 복구(RECOVER) 때 첫 move_to 를 저속으로 — 인터페이스는 W121에서 정해졌다(MoveTo.speed_ratio,
robot.yaml recover.speed_ratio 0.5) ② 시간 제한(위). 조립 중이 아닐 때(IDLE · READY · DONE) 정지됐다 풀린 뒤 돌아갈 곳은 SDD 에
없어서 가장 가까운 규칙(비조립 RECOVER → IDLE)만 따른다(IRD 12장 W121 다시 확인 대상).
"""
import copy
import json
import logging
import threading
import time

from d2_task.run_logger import RunLogger
from d2_task.task_planner import TaskPlanner

LOG = logging.getLogger('d2_task')

# 조립이 진행 중인 상태 — 정지 · 실패 뒤 RECOVER 에서 CHECK 로 이어 간다
RUN_STATES = ('CHECK', 'SELECT', 'PICK_PLACE', 'WAIT_SUPPLY', 'VERIFY', 'RECOVER', 'ERROR')
# 로봇이 서야 하는 결과 — 정지 노드가 세웠다(STOPPED · CANCELED) / 그리퍼 피드백이 없어 세웠다(NO_FEEDBACK, IRD 7장 → STOPPED)
STOP_REASONS = ('STOPPED', 'CANCELED', 'NO_FEEDBACK')


def wait_until(event, should_abort, poll_s=0.05):
    """event 가 켜질 때까지 기다리되, should_abort() 가 참이 되면 바로 빠져나온다.

    반환: True = event 가 켜졌다(답이 왔다), False = 중단 신호가 먼저 왔다.
    poll_s 는 중단 신호를 확인하는 간격일 뿐 기다리는 시간 제한이 아니다. 완전 블로킹 대기(future.result())로
    정지 처리가 막히지 않게 하려고 task_node 의 모든 기다림이 이것을 거친다.
    """
    while not event.wait(poll_s):
        if should_abort():
            return False
    return True


class TaskManager:
    """자동 조립 상태표. 설계를 고르고(select_design) 출발(start)하면 진행 확인 → 다음 블록 → 집기·놓기 → 배치 확인을 되풀이한다.

    들어오는 것(콜백 스레드): command(HmiCommand) · on_intent(음성) · on_safety(정지 노드) · on_gripper · shutdown.
    나가는 것: io 로 /d2/task/state · /d2/task/progress 방송, move_to · check_progress · pick_place 호출.
    바깥 영향: 로봇은 pick_place · move_to 를 통해서만 움직인다(이 클래스는 팔을 직접 안 건드린다).
    실패: 알려진 실패 코드(IRD 7장)는 SDD 7.1 표대로 처리한다(TIMEOUT · ERROR · GRIPPER_NO_RESPONSE 는 정식 코드이고 → ERROR).
    IRD 에 없는 reason 도 같은 공통 분기로 일반 실패 → ERROR 로 간다.
    정지 노드가 멈추면(safety/state stopped) 어느 상태에서든 STOPPED 로 가고, 잠금이 풀린 신호가 새로 오면 RECOVER 로 간다.
    """

    def __init__(self, cfg, io, logger=None, clock=time.monotonic):
        """cfg = robot.yaml dict, io = 위 설명의 바깥 일 담당, logger = RunLogger(없으면 파일 없이 run_id 만 만든다), clock = 단조 시계(시험용).
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
        for run_id, summary in self.logger.load_pending():
            self.pending_builds[run_id] = summary
            self._build_ready.add(run_id)         # 이미 디스크에 있다
        if self.pending_builds:
            LOG.warning('디스크에 보관된 미전송 요약 %d개를 되살렸다: %s', len(self.pending_builds), ', '.join(self.pending_builds))
        self._handlers = {'CHECK': self._check, 'SELECT': self._select, 'PICK_PLACE': self._pick_place,
                          'VERIFY': self._verify, 'WAIT_SUPPLY': self._wait_supply, 'STOPPED': self._stopped,
                          'RECOVER': self._recover, 'ERROR': self._error}

    # ---------- 콜백 스레드에서 들어오는 것 ----------
    def on_safety(self, st):
        """정지 노드의 safety_state/1(dict)을 적는다. 상태 전이는 작업 스레드가 run_once 에서 한다(콜백은 변수만)."""
        with self._lock:
            self.safety = st
            self.safety_count += 1

    def on_gripper(self, st):
        """그리퍼 노드의 gripper_state/1(dict)을 적는다. grasped 가 참이면 블록을 쥐고 있다."""
        with self._lock:
            self.gripper = st

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
        """멈춰야 하면 참 — 정지 노드가 stopped 를 보냈거나 끝내는 중. io 의 기다림이 이것을 should_abort 로 쓴다."""
        st = self.safety
        return self._shutdown or bool(st and st.get('stopped'))

    def command(self, cmd, design_id=''):
        """화면 버튼 명령(HmiCommand, 다리가 웹 화면 대신 부른다). 반환: (success, reason). 바로 답한다(다리의 응답 기다림이 짧다).

        select_design = 설계를 읽고 시작 점검 → READY. start = READY 에서 출발(점검 한 번 더) → CHECK,
        WAIT_SUPPLY 에서는 [계속](관측 자세에 도착한 뒤에만). 그 밖 · 운전 중 · ERROR 중은 거절한다.
        reason 은 IRD 7장에 이미 있는 코드만 쓴다: 운전 중 'BUSY', 정지 중 'STOPPED'. 맞는 코드가 없는 거절(설계 없음 ·
        점검 실패 · 아직 없는 명령)은 reason 을 빈 값으로 두고 이유를 state/1 의 message 글자로만 알린다.
        """
        with self._lock:
            if cmd == 'select_design':
                return self._select_design(design_id)
            if cmd == 'start':
                return self._start(design_id)
            return self._reject('', '이 명령은 아직 없다(스캔 흐름은 W119)' if cmd == 'scan' else f'모르는 명령: {cmd}')

    def on_intent(self, intent, design_id=None):
        """음성 의도(intent/1, 다리가 ROS 토픽으로 낸다). start · select_design 은 화면 버튼과 똑같이, cancel 은 READY · ERROR 에서만.

        그 밖(request_design 등)은 무시. 출발할 수 없는 때의 음성 start 는 무시하고 알림 voice_start_ignored 만 낸다(IRD 8.5).
        """
        with self._lock:
            if intent == 'select_design' and design_id:
                self._select_design(design_id)
            elif intent == 'start':
                if self.state in ('READY', 'WAIT_SUPPLY') or (self.state == 'IDLE' and design_id):
                    ok, _ = self._start(design_id or '')
                    if not ok and self.state == 'WAIT_SUPPLY':      # 아직 관측 자세로 가는 중이라 못 받았다
                        self._publish('voice_start_ignored', '관측 자세로 가는 중이라 음성 출발을 무시했다')
                else:
                    self._publish('voice_start_ignored', '지금은 음성 출발을 받을 수 없어 무시했다')
            elif intent == 'cancel' and self.state in ('READY', 'ERROR'):
                if self.state == 'ERROR':
                    self._finish_run('ERROR')
                self.planner, self.design_id, self.block_id = None, None, None
                self._clear_run()
                self._set('IDLE', None, '취소했다')

    # ---------- 작업 스레드 ----------
    def run_once(self):
        """지금 상태의 일을 한 단계만 하고 돌아온다(작업 스레드가 되풀이해서 부른다).

        정지 신호를 매번 먼저 본다. 기다리는 상태(IDLE · READY · DONE)는 아무것도 안 한다.
        단계 안에서 예상 못 한 예외가 나면 로그를 남기고 ERROR 로 간다(팔은 pick_place 가 서 있게 한다).
        """
        self._apply_safety()
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
        ok, why = self.io.move_to('observe', self.halted)
        if not ok:
            return self._failed(why, '관측 자세 이동')
        if not self._observe([b['block_id'] for b in self.planner.blocks]):
            return None
        problems = self.planner.judge()
        if problems:
            return self._offset_over(problems)
        self._set('SELECT', None, '')
        return None

    def _select(self):
        """SELECT: 작업 판단에 다음 블록을 묻는다. 있으면 PICK_PLACE, 끝이면 DONE, 못 본 블록은 재관측 1번 뒤 ERROR."""
        r = self.planner.next_block()
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
            self._enter_wait_supply(r['block_id'])
        else:                                  # NO_SUPPORT · 재관측 뒤에도 UNKNOWN_BLOCK
            self._to_error(f'{r["block_id"]}: {status}')

    def _pick_place(self):
        """PICK_PLACE: 목표 하나를 pick_place 로 보내고 결과를 SDD 7.1 표대로 처리한다."""
        ok, why = self.io.pick_place(self.goal, self.halted)
        if ok:
            self.placed = self.goal['block_id']
            self.logger.record_placed(self.placed, self.goal['supply_slot'])
            self._set('VERIFY', None, f'{self.placed} 를 놓았다. 확인합니다')
            return None
        self.logger.log('fail', self.goal['block_id'], f'pick_place {why}')
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
            # 정지류 · BUSY 는 _failed 가 따로 처리한다. TIMEOUT · ERROR · GRIPPER_NO_RESPONSE 는 IRD 7장 정식 코드이고
            # SDD 7.1 이 ERROR 로 정했다. IRD 에 없는 reason 도 같은 공통 분기(일반 실패 → ERROR)를 탄다
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
        self.planner.supply_refilled()
        r = self.planner.next_block()
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
        복구 때의 '저속'(MoveTo.speed_ratio · recover.speed_ratio)은 아직 안 건다(W119).
        """
        if not self._assembling:
            self.planner, self.design_id, self.block_id = None, None, None
            self._clear_run()
            self._set('IDLE', None, '정지가 풀렸어요. 설계를 다시 고르세요')
            return None
        grip = self.gripper
        if grip is None or grip.get('grasped'):
            return self._to_error('블록을 쥐고 있거나 그리퍼 상태를 모른다. 사람이 확인한 뒤 [다시 시작]')
        self._set('CHECK', None, '진행 확인부터 다시 합니다')
        return None

    def _error(self):
        """ERROR: 로봇은 서 있다. 들어온 뒤 잠금이 풀린 safety/state 가 새로 오면([다시 시작]) RECOVER 로 간다.

        정지 노드의 resume 은 잠겨 있지 않아도 safety/state 를 다시 내보내므로(최신 main safety_stop_node), 그것이 신호다.
        """
        with self._lock:
            if self._unlocked_since(self._entered_count):
                self._assembling = True
                self._set('RECOVER', None, '다시 시작 — 쥔 블록을 확인합니다')

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

    def _enter_wait_supply(self, block_id):
        """WAIT_SUPPLY 상태로 들어간다. 알림(supply_empty)은 아직 안 낸다 — 관측 자세에 도착한 뒤 _wait_supply 가 낸다."""
        with self._lock:
            self.block_id = block_id
            self._supply_moved = False
            self._pending_start = False        # 이전에 눌린 [계속]은 버린다
            self._set('WAIT_SUPPLY', None, '공급이 필요해요. 관측 자세로 갑니다')

    def _failed(self, reason, where, log=True):
        """집기·놓기 말고 이동 · 관측 호출이 실패했을 때. 정지류는 STOPPED, BUSY 는 같은 상태에서 다시, 그 밖은 모두 ERROR.

        모르는 reason(IRD 7장에 없는 문자열)도 이름을 따로 다루지 않고 일반 실패로 ERROR 에 보낸다. 글자는 알림에 그대로 적는다.
        """
        if log:
            self.logger.log('fail', self.block_id, f'{where}: {reason or "이유 없음"}')
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
    def _select_design(self, design_id):
        """(lock 안) 레시피를 읽고 시작 점검을 통과하면 READY. IDLE · READY · DONE 에서만."""
        if self.state not in ('IDLE', 'READY', 'DONE'):
            return self._reject(self._busy_reason(), '지금은 설계를 바꿀 수 없다')
        recipe = self.io.load_recipe(design_id) if design_id else None
        if recipe is None:
            return self._reject('', f'설계 파일을 못 읽었다: {design_id!r}')
        try:
            planner = TaskPlanner(self.cfg, recipe)
        except (ValueError, KeyError, TypeError) as e:
            return self._reject('', f'레시피를 못 읽는다: {e}')
        ok, code, text = self._precheck()
        if not ok:
            return self._reject(code, f'시작 점검 실패: {text}')
        self.planner, self.design_id, self.block_id, self.placed = planner, design_id, None, None
        self._clear_run()
        self._set('READY', 'ready_to_start', f'{design_id} 준비됐어요. 출발을 누르세요')
        return True, ''

    def _start(self, design_id):
        """(lock 안) READY 에서 출발(→ CHECK), WAIT_SUPPLY 에서 [계속]. IDLE 에서 설계 포함 음성 start 는 고른 뒤 출발.

        WAIT_SUPPLY 에서는 관측 자세에 도착해 supply_empty 를 낸 뒤에만 받는다(도착 전 start 는 BUSY 로 거절, 진행시키지 않는다).
        """
        if self.state == 'WAIT_SUPPLY':
            if not self._supply_moved:
                return self._reject('BUSY', '아직 관측 자세로 가는 중이에요. 도착한 뒤 [계속]을 누르세요')
            self._pending_start = True
            self.logger.log('command', self.block_id, 'continue')    # 같은 run_id 로 이어 간다
            return True, ''
        if self.state == 'IDLE' and design_id:
            ok, why = self._select_design(design_id)
            if not ok:
                return ok, why
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

    def _precheck(self):
        """시작 점검(M-06) 중 숫자 없는 항목. 반환: (통과, 거절 reason, 이유 글자).

        ① 서버 3개가 떠 있다 ② 정지 노드 신호를 받았고 잠기지 않았다 ③ 그리퍼 신호를 받았고 비었다.
        신호가 몇 초 안에 왔는지(신선도)는 숫자가 필요해서 보지 않는다. 사람이 영역 밖인지는 누르는 사람이 확인한다.
        """
        missing = self.io.services_ready()
        if missing:
            return False, '', f'서버가 안 떠 있다: {", ".join(missing)}'
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
            if self.state == 'STOPPED':
                return
            self._assembling = self.state in RUN_STATES
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
        """열린 조립 기록을 result(DONE · STOPPED · ERROR)로 닫고 build/1 요약을 last_build 에 둔다. run_id 는 화면에 남긴다."""
        summary = self.logger.finish(result)
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
            self.logger.log('state', self.block_id, state)
            self._publish(message_id, message)

    def _publish(self, message_id, message):
        """state/1 한 건을 보낸다. run_id 는 조립 중(끝난 직후 포함)에만 값, message_id 는 IRD 2장 값 또는 null."""
        self.io.publish_state({'schema': 'state/1', 'stamp': time.time(), 'mode': 'auto', 'state': self.state,
                               'run_id': self.run_id, 'design_id': self.design_id, 'block_id': self.block_id,
                               'message_id': message_id, 'message': message})

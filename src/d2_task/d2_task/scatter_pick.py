# -*- coding: utf-8 -*-
"""흩뿌림 집기 준비 ScatterFlow — 목표 블록을 유지한 채 find_blocks 결과로 후보를 고르고 집기 요청을 준비한다 (ROS 없이 동작, W130).

TaskManager 의 SELECT 안에서(공개 상태를 늘리지 않고) 쓰려고 만든 계산 클래스다. 아직 TaskManager · 로봇 호출에 연결하지 않았다.
호출 흐름(연결할 때): ① SELECT 가 TaskPlanner 에서 목표 블록(block_id · 잡기 · 놓을 자세)을 정한다 → ② begin() 으로 요청 표를 받는다 →
③ 잠금 밖에서 observe_supply 로 가서 find_blocks 를 부르고 prepare() 로 후보를 계산한다(계산만 — 상태를 바꾸지 않는다) →
④ **TaskManager 잠금 안에서** commit() 이 요청 번호 · 상태 · 정지 · 종료를 다시 확인하고 같은 구간에서 후보를 적용한다(확인과 적용 사이에 정지가 끼지 못한다) →
⑤ 실패하면 on_failure() 가 실행 단계 · 잡힘 상태로 다음 동작(REPLAN_SAME · WAIT · REOBSERVE · RECOVER · ERROR)을 알려 준다.
설정은 명시적으로 주입한다: supply_mode == 'scatter' 이고 open_width_m 이 정해져야 실행 가능한 요청을 만든다.
open_width_m 은 task 노드가 0 으로 넘긴다 — PickPlace 에서 0 은 '잡기마다 robot.yaml grasp_open_pick_m 을 쓴다'이므로 목표 블록의 잡기가 바뀌어도 폭이 맞는다.
"""
import copy
import threading

from d2_motion.motion_math import column, half_height, quat_from_axes, rot_z, rpy_matrix

from d2_task.block_picker import BlockPicker, grasp_target

SUPPLY_SCATTER = 'scatter'
# 세우기(E-37 · E-77): 목표 자세(위 축) → 눕힌 블록을 집을 잡기 (10/11 16:50 PL — 세움 FLAT_SHORT · 옆세움 FLAT_LONG)
REORIENT_GRASP = {'LENGTH': 'FLAT_SHORT', 'WIDTH': 'FLAT_LONG'}

# 실패 뒤 동작. 이유만으로 정하지 않는다 — 실행 단계(phase)와 잡힘 상태(grasped)로 '접촉 전'이 확인될 때만 후보를 다시 쓴다.
# PLAN_FAILED 는 접근 계획뿐 아니라 집은 뒤 들어 올리기 · 운반 · 놓기에서도 나온다. 쥐었거나 상태가 불명확하면 기존 정지 · 복구 절차(RECOVER)로 넘긴다.
PRE_CONTACT = 'PRE_CONTACT'                # 손가락이 블록에 닿기 전 단계(접근 계획 · 이동)에서 실패했다고 확인됨
REPLAN_SAME, WAIT, REOBSERVE, RECOVER, ERROR = 'REPLAN_SAME', 'WAIT', 'REOBSERVE', 'RECOVER', 'ERROR'
ALWAYS_RECOVER = ('TIMEOUT', 'STOPPED', 'CANCELED')                   # 중간에 멈췄다 — 닿았는지 · 쥐었는지 모른다
ALWAYS_ERROR = ('ERROR', 'GRIPPER_NO_RESPONSE', 'NO_FEEDBACK')        # 사람 호출 (그리퍼 응답 · 피드백 없음은 작업 관리자 ERROR, E-74)


def failure_action(reason, phase=None, grasped=None, area_unchanged=None):
    """실패 이유 + 실행 단계 + 잡힘 상태 → 다음 동작.

    phase: PRE_CONTACT 이면 접촉 전 실패가 확인됨, 그 밖(None 포함)은 불명확. grasped: 그리퍼가 블록을 쥐고 있나(False 가 확인돼야 안전).
    area_unchanged: 다른 동작이 공급 영역을 안 바꿨다는 확인(BUSY 만 본다).
    반환: REPLAN_SAME(같은 관측 후보로 다시 계획) · WAIT(기다렸다 같은 요청, 후보 유지) · REOBSERVE(후보 버리고 다시 관측 — 새 집기 전에
    손에 블록이 없음이 확인된 경우만) · RECOVER(기존 정지 · 복구 절차로 — 단순 재관측 뒤 새 집기를 시작하지 않는다) · ERROR(사람 호출).
    """
    if reason == 'LOOKUP_FAILED':
        return REOBSERVE                         # 조회만 실패했다 — 로봇은 움직이지 않았다(횟수 상한은 정책 미정)
    if reason in ALWAYS_ERROR:
        return ERROR
    if reason in ALWAYS_RECOVER:
        return RECOVER
    if grasped is not False:                     # 쥐었거나(True) 모르면(None) 복구 절차
        return RECOVER if reason in ('PLAN_FAILED', 'BUSY', 'GRASP_FAILED', 'SLOT_EMPTY') else ERROR
    if reason == 'PLAN_FAILED':
        return REPLAN_SAME if phase == PRE_CONTACT else RECOVER
    if reason == 'BUSY':
        return WAIT if area_unchanged is True else REOBSERVE
    if reason in ('GRASP_FAILED', 'SLOT_EMPTY'):
        return REOBSERVE                         # 닿았을 수 있어 블록이 밀렸다 — 이전 후보는 버리되 손에는 없음이 확인됨
    return ERROR                                 # 모르는 이유는 조용히 재사용하지 않는다


STEPS = ('approach', 'grasp', 'lift', 'move', 'place', 'retreat')     # PickPlace 피드백 step
CONTACT_UNKNOWN, POST_CONTACT = 'CONTACT_UNKNOWN', 'POST_CONTACT'


def contact_phase(step, reason):
    """PickPlace 피드백의 마지막 step 과 실패 이유 → 접촉 전후(박진용 확인, 10/7).

    approach 안에는 '블록 위에서 그리퍼 열기 → 블록 옆으로 수직 하강'이 들어 있어서 step 만으로 접촉 전이라고 확정하지 않는다:
    approach + PLAN_FAILED = 접촉 전 실패로 확인된 조합(PRE_CONTACT — 전체 동작에서 안 움직였다는 뜻이 아니다) ·
    approach + 그 밖의 실패 = 접촉 여부 모름(CONTACT_UNKNOWN) ·
    grasp 이후(grasp · lift · move · place · retreat) = 접촉 후(POST_CONTACT). 모르는 step · 없으면 CONTACT_UNKNOWN.
    """
    if step == 'approach':
        return PRE_CONTACT if reason == 'PLAN_FAILED' else CONTACT_UNKNOWN
    if step in STEPS:
        return POST_CONTACT
    return CONTACT_UNKNOWN


def grasped_now(state, result_time, now, max_age_s):
    """gripper/state 로 '지금 쥐고 있나'. 믿을 수 없으면 None(모름 — 복구 절차로 간다).

    state = 마지막 gripper_state/1 dict(stamp = 로봇 PC 시계 초 · grasped). stamp · result_time · now 는 **같은 시계(time.time())** 로 잰 값이어야
    한다 — 시간 제한용 단조 시계(time.monotonic)와 섞지 않는다. 믿는 조건(박진용 확인): 값이 PickPlace 결과를 받은 뒤 시각
    (stamp ≥ result_time)이고 now − stamp ≤ max_age_s(1초마다 + 바뀔 때 보내므로 3초). max_age_s 는 부르는 쪽이 명시로 준다(여기서 정하지 않는다).
    """
    try:
        stamp, grasped = state['stamp'], state['grasped']
    except (KeyError, TypeError):
        return None
    ok = all(isinstance(v, (int, float)) and not isinstance(v, bool) and v == v and abs(v) != float('inf')
             for v in (stamp, result_time, now, max_age_s))
    if not ok or not isinstance(grasped, bool) or max_age_s <= 0:
        return None
    if stamp < result_time or now - stamp > max_age_s or stamp > now:
        return None
    return grasped


class StepTracker:
    """집기 요청마다 PickPlace 피드백의 마지막 step 을 따로 기억한다(ROS 없음).

    begin(request_id) 로 새 요청을 시작하면 이전 요청의 step 은 지워지고, 이전 요청의 늦은 피드백(request_id 가 다름)은 버린다.
    request_id 는 부르는 쪽이 요청마다 새로 만드는 번호(예: 증가하는 정수)다. 모르는 step 도 기록하지 않는다.
    같은 요청 안에서 단계는 STEPS 순서로만 앞으로 간다: 이미 지난 단계나 같은 단계의 늦은 피드백(예: grasp 뒤에 늦게 온 approach)은 버린다 —
    그러지 않으면 접촉 뒤인데 '접촉 전(PRE_CONTACT)'으로 되돌아가 같은 후보로 다시 계획하게 된다.
    결과(PickPlace result)를 실제로 받은 시각도 요청마다 한 번만 적는다(on_result). 결과 없이 끝난 요청(거절 · 시간 초과 · 우리 쪽 취소)은 None 이다.
    """

    def __init__(self):
        """요청 없음으로 시작한다."""
        self._lock = threading.Lock()
        self._current = None
        self._step = None
        self._result_at = None

    def begin(self, request_id):
        """새 요청을 시작한다. 마지막 step 과 결과 수신 시각을 비운다."""
        with self._lock:
            self._current, self._step, self._result_at = request_id, None, None

    def on_feedback(self, request_id, step):
        """피드백을 받았다. 지금 요청의 것이고 알려진 step 이며 마지막 step 보다 **앞으로 가는** 것일 때만 기록한다. 반환: 기록했으면 True."""
        with self._lock:
            if request_id != self._current or request_id is None or step not in STEPS:
                return False
            if self._step is not None and STEPS.index(step) <= STEPS.index(self._step):
                return False                       # 역행 · 중복은 버린다
            self._step = step
            return True

    def on_result(self, request_id, received_at):
        """결과를 받았다. 지금 요청의 첫 결과일 때만 수신 시각(time.time() 초)을 적는다. 반환: 적었으면 True."""
        with self._lock:
            if request_id != self._current or request_id is None or self._result_at is not None:
                return False
            self._result_at = received_at
            return True

    def last_step(self, request_id):
        """그 요청의 마지막 step. 지금 요청이 아니거나 피드백이 없었으면 None."""
        with self._lock:
            return self._step if request_id == self._current and request_id is not None else None

    def result_at(self, request_id):
        """그 요청의 결과 수신 시각(time.time() 초). 지금 요청이 아니거나 결과를 못 받았으면 None."""
        with self._lock:
            return self._result_at if request_id == self._current and request_id is not None else None

    def end(self, request_id):
        """요청이 끝났다(결과 처리 뒤). 더 이상 그 요청의 피드백 · 결과를 받지 않는다."""
        with self._lock:
            if request_id == self._current:
                self._current = self._step = self._result_at = None


UP_INDEX = {'LENGTH': 0, 'WIDTH': 1, 'THICKNESS': 2}     # block_actual_m [길이, 폭, 두께] 에서 위 축 치수 자리


def _finite(v):
    import math
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def block_pose(cfg, up, yaw_deg, x_m, y_m, top_z_m):
    """관측한 블록의 중심 자세 (xyz m, 쿼터니언 xyzw). 공급 칸 자세 계산(slot_block_pose)과 같은 규약이다.

    up = 위를 향한 블록 축, yaw_deg = 긴 변이 base x 와 이루는 각(세운 블록은 윗면 25 mm 변 기준, find_blocks 와 같음).
    중심 높이 = 윗면 − 위로 향한 치수의 절반(block_actual_m). 윗면 높이에 카메라 보정 오차가 있으면 그대로 따라온다.
    """
    import math
    yaw = math.radians(yaw_deg)
    if up == 'THICKNESS':
        rot = rpy_matrix(0.0, 0.0, yaw)
    elif up == 'WIDTH':
        rot = rot_z(rpy_matrix(math.pi / 2, 0.0, 0.0), yaw)
    elif up == 'LENGTH':
        rot = rot_z(rpy_matrix(0.0, -math.pi / 2, 0.0), yaw - math.pi / 2)
    else:
        raise ValueError(f'up={up!r}')
    center = (x_m, y_m, top_z_m - half_height(rot, cfg['block_actual_m']))
    return center, quat_from_axes(*(column(rot, k) for k in range(3)))


class ScatterFlow:
    """흩뿌림 집기 한 건의 준비와 재시도 판단. 입력 dict 는 바꾸지 않고 복사해서 쓴다.

    입력: cfg(robot.yaml dict), picker(BlockPicker), supply_mode · open_width_m(명시 주입 — 둘 다 정해져야 실행 가능한 요청을 만든다).
    바깥 영향: 없음(계산만). 로봇 · 서비스를 부르지 않는다.
    """

    def __init__(self, cfg, picker, supply_mode=None, open_width_m=None):
        """설정을 받는다. open_width_m 은 None(미정) 또는 0 이상 유한한 수(m)."""
        if not isinstance(picker, BlockPicker):
            raise ValueError('picker 가 BlockPicker 가 아니다')
        if open_width_m is not None and not (isinstance(open_width_m, (int, float)) and not isinstance(open_width_m, bool)
                                              and 0 <= open_width_m < float('inf')):
            raise ValueError(f'open_width_m 이 이상하다: {open_width_m!r}')
        self.cfg, self.picker = cfg, picker
        self.supply_mode, self.open_width_m = supply_mode, open_width_m
        self._token = 0
        self._lock = threading.Lock()
        self.candidate = None                 # 지금 유효한 집기 요청(commit 으로만 정해지고, 재계획 때만 다시 쓴다)

    @property
    def configured(self):
        """흩뿌림 모드가 명시됐고 열림 폭이 정해졌나. 아니면 실행 가능한 요청을 만들지 않는다."""
        return self.supply_mode == SUPPLY_SCATTER and self.open_width_m is not None

    # ---------- 늦은 답 막기 ----------
    def begin(self, epoch):
        """관측 · 조회를 시작한다. 이전 요청은 무효가 되고, 같은 epoch(상태 변화 표)와 함께 쓸 표를 돌려준다. 후보도 비운다."""
        with self._lock:
            self._token += 1
            self.candidate = None
            return (self._token, epoch)

    def is_current(self, ticket, epoch, halted):
        """(읽기만) 답을 적용해도 되나: 더 새 요청이 없고 · 상태가 안 바뀌었고 · 정지 · 종료 중이 아닐 때만 참. 적용은 commit 으로 한다."""
        with self._lock:
            return (not halted) and ticket == (self._token, epoch)

    def commit(self, ticket, state_fn, result):
        """prepare() 결과를 적용한다. 요청 번호 · 상태 · 정지 · 종료 재확인과 후보 적용을 한 구간에서 한다. 반환: 적용했으면 True.

        state_fn() → (epoch, halted) 를 이 구간 안에서 한 번 읽는다. 부르는 쪽(TaskManager)은 **자기 잠금을 잡은 채** 부른다 —
        그래야 확인 직후 정지 신호가 끼어들지 못한다(정지는 같은 잠금을 얻어야 반영된다). 계산(prepare)은 잠금 밖에서 한다.
        PICK 이면 후보를 정하고, 그 밖의 결과면 후보를 비운다. 늦은 답이면 아무것도 안 바꾼다.
        """
        with self._lock:
            epoch, halted = state_fn()
            if halted or ticket != (self._token, epoch):
                return False
            if result.get('status') in ('PICK', 'REORIENT'):
                self.candidate = {'request': copy.deepcopy(result['request']), 'candidate': copy.deepcopy(result['candidate']),
                                  'gap_mm': result['gap_mm']}
            else:
                self.candidate = None
            return True

    def invalidate(self):
        """후보를 버린다(정지 · 종료 · 상태 변화 때 TaskManager 가 부른다)."""
        with self._lock:
            self._token += 1
            self.candidate = None

    # ---------- 후보 고르기와 요청 준비 ----------
    def prepare(self, target, response, allow_reorient=False):
        """목표 블록 + find_blocks 응답 → 집기 요청 준비. 반환 dict 의 status:

        (후보는 이 함수가 정하지 않는다 — 계산만. 적용은 commit.)
        PICK          request = PickPlace 목표 칸(block_id · supply_slot '' · grasp · pick_pose · place_pose · open_width_m · obstacles)과 candidate(복사본)
                      obstacles = 같은 응답에서 고른 블록 하나(인덱스)만 뺀 나머지 전부의 중심 자세 [(xyz m, 쿼터니언 xyzw)](under · top 포함, 자르기 없음).
                      고른 블록만 있었으면 빈 목록 — 정상이다(다른 블록 없음, 공급 칸 가정 상자로 안 돌아감).
                      기울어진(tilted) 블록도 장애물에 넣는다 — 지금(핵심 단계)은 집기 요청을 막지 않고 IRD 4.2 그대로 '윗면 = 가장 높은 점인 반듯한 상자'로
                      싣는다(E-68, 10/8 PL). 챌린지 ④(W135)에서 민범진의 기울기 칸으로 정확한 기울기 상자로 바꾼다. 깔린 블록(under)도 장애물이다.
                      기울어진 블록을 **고르지 않는** 것은 BlockPicker 가 한다(E-43): 그런 블록만 남으면 아래 NO_MATCH.
        BLOCKED       요청을 만들지 않는다(reason): INVALID_OBSTACLE = 고르지 않은 블록 중 형식이 틀린 것이 있다(조용히 빼지 않는다 — 상자를 모른다).
                      사람 안내는 guide 'CHECK_BLOCKS'.
        NOT_CONFIGURED 모드 · 열림 폭이 아직 정해지지 않음 — 요청을 만들지 않는다
        EMPTY         공급에 블록이 없다 → 채우기 안내(guide 'REFILL')
        NO_MATCH      블록은 있는데 맞는 후보가 없다 → 제외 이유(counts · message)를 안내(guide 'CHECK_BLOCKS')
        LOOKUP_FAILED 조회 실패 · 잘못된 응답 — 공급 부족이 아니다(reason)
        REORIENT      (allow_reorient 이고 목표가 세움 · 옆세움인데 맞는 후보가 없을 때만) 눕힌 블록을 빈 자리(spare_spots)에 세우는 요청.
                      request 는 PICK 과 같은 칸 + 내부 표시 'reorient': True(액션으로는 안 감 — task_node 는 이름 있는 칸만 읽는다).
                      세울 수 없으면(눕힌 후보 없음 · 기울어진 블록이 있음 · 빈 자리 없음) REORIENT 를 만들지 않고 NO_MATCH 그대로.
        target = {block_id, grasp, place_pose}. 목표 block_id · 놓을 자세 · 잡기는 그대로 유지하고 복사해서 쓴다.
        """
        if not self.configured:
            return {'status': 'NOT_CONFIGURED'}
        if not isinstance(response, dict) or response.get('ok') is not True or not isinstance(response.get('blocks'), list):
            return {'status': 'LOOKUP_FAILED', 'reason': 'BAD_RESPONSE'}
        up, axis = grasp_target(target['grasp'], self.cfg)
        picked = self.picker.pick(response['blocks'], up, axis)
        if picked['status'] == 'NONE':
            if picked['reason'] == 'EMPTY':
                return {'status': 'EMPTY', 'guide': 'REFILL'}
            if picked['counts']['invalid'] == len(response['blocks']):
                return {'status': 'LOOKUP_FAILED', 'reason': 'ALL_INVALID'}      # 쓸 수 있는 블록이 하나도 없는 응답 — 조회 오류
            if allow_reorient and up in REORIENT_GRASP:
                reorient = self._reorient(target, response['blocks'], up)
                if reorient is not None:
                    return reorient
            return {'status': 'NO_MATCH', 'guide': 'CHECK_BLOCKS', 'counts': dict(picked['counts']),
                    'message': self._why(picked['counts'])}
        c = picked['block']
        obstacles = self._obstacles(response['blocks'], picked['index'])
        if isinstance(obstacles, dict):
            return obstacles
        center, quat = block_pose(self.cfg, c['up'], c['yaw_deg'], c['x_m'], c['y_m'], c['top_z_m'])
        request = {'block_id': target['block_id'], 'supply_slot': '', 'grasp': target['grasp'],
                   'pick_pose': (center, quat), 'place_pose': copy.deepcopy(target['place_pose']),
                   'open_width_m': self.open_width_m, 'obstacles': obstacles}
        return {'status': 'PICK', 'request': copy.deepcopy(request), 'candidate': copy.deepcopy(c), 'gap_mm': picked['gap_mm']}

    def _reorient(self, target, blocks, up):
        """눕힌 블록 하나를 빈 자리에 세우는 요청(REORIENT) 또는 None(세울 수 없음 → 부르는 쪽이 NO_MATCH).

        잡기 = REORIENT_GRASP[up](PL 확정), 눕힌 후보는 그 잡기로 BlockPicker 의 같은 규칙으로 고른다. 놓을 자세 = 빈 자리 중심 · yaw_deg 에
        블록을 up 축이 위로 서게(block_pose — 공급 칸과 같은 yaw 규약, 세움은 윗면 25 mm 변 방향). block_id 는 목표 그대로(장면 · 진행표에서 같은 이름).
        빈 자리 확인은 놓기 전 필수 — 이번 관측에서 외곽을 아는 블록(고른 블록 제외)의 수평 외곽과 세운 블록 외곽이 겹치면 점유.
        기울어진 블록이 하나라도 있으면 외곽을 확정할 수 없어 세우지 않는다(관측 불확실성 — 근사 상자로 대신하지 않음).
        손가락 접근 공간은 W131 접근 경로가 정해지기 전이라 보지 않는다. 바깥 영향 없음(계산만).
        yaw 는 확정 규약(LENGTH 위 = WIDTH 방향, WIDTH 위 = LENGTH 방향, base x 0° · 반시계 + · 180° 같음)으로 spare_spots yaw_deg 를 그대로 쓴다.
        임시 시험 전제(운영 활성화 전 확인 필요): spare_spots 좌표는 임시(W146 실기 확인 대기) · 빈 자리는 외곽 겹침만 확인(W131 손가락 · 회전 · 접근 · 후퇴 공간 없음).
        운영 TaskManager 는 allow_reorient 기본 False 라 이 요청을 만들지 않는다 — W131 두 grasp 지원 · W146 정식 좌표 · W131 과 맞춘 공간 판정 뒤 활성화.
        """
        try:                                  # 세울 때만 검증한다 — 세우기가 꺼져 있으면 잘못된 spare_spots 가 일반 집기를 막지 않는다
            spots = self._spare_spots(self.cfg.get('spare_spots', []))
        except ValueError as e:
            return {'status': 'BLOCKED', 'reason': 'INVALID_SPARE_SPOTS', 'guide': 'CHECK_BLOCKS', 'message': str(e)}
        grasp = REORIENT_GRASP[up]
        lie_up, axis = grasp_target(grasp, self.cfg)
        picked = self.picker.pick(blocks, lie_up, axis)
        if picked['status'] != 'FOUND' or any(b.get('tilted') is True for b in blocks):
            return None
        known = [b for i, b in enumerate(blocks) if i != picked['index'] and self.picker.is_valid(b)]
        spot = next((s for s in spots
                     if not any(self._rects_overlap(self._spot_rect(s, up), self._block_rect(b)) for b in known)), None)
        if spot is None:
            return None                       # 두 자리 모두 점유는 PL 제외 범위(미지원) — 점유된 자리로 보내지 않고, 부르는 쪽의 NO_MATCH 로 끝난다(한계)
        obstacles = self._obstacles(blocks, picked['index'])
        if isinstance(obstacles, dict):
            return obstacles
        c = picked['block']
        pick_pose = block_pose(self.cfg, c['up'], c['yaw_deg'], c['x_m'], c['y_m'], c['top_z_m'])
        place_pose = block_pose(self.cfg, up, spot['yaw_deg'], spot['x_m'], spot['y_m'],
                                self.cfg['table_z_m'] + self.cfg['block_actual_m'][UP_INDEX[up]])
        request = {'block_id': target['block_id'], 'supply_slot': '', 'grasp': grasp, 'pick_pose': pick_pose, 'place_pose': place_pose,
                   'open_width_m': self.open_width_m, 'obstacles': obstacles, 'reorient': True}
        return {'status': 'REORIENT', 'request': copy.deepcopy(request), 'candidate': copy.deepcopy(c), 'gap_mm': picked['gap_mm'],
                'spot': spots.index(spot)}

    @staticmethod
    def _spare_spots(spots):
        """robot.yaml spare_spots → 검증한 목록(값은 그대로). 형식이 틀리면 ValueError(조용히 빼지 않는다) — 세울 때만 부른다."""
        if not isinstance(spots, list):
            raise ValueError('robot.yaml spare_spots 가 목록이 아니다')
        for k, s in enumerate(spots, 1):
            if not isinstance(s, dict) or not all(_finite(s.get(key)) for key in ('x_m', 'y_m', 'yaw_deg')):
                raise ValueError(f'robot.yaml spare_spots {k}번이 {{x_m, y_m, yaw_deg}} 숫자가 아니다')
        return [dict(s) for s in spots]

    def _footprint(self, up):
        """위 축별 수평 외곽 (yaw 방향 길이, 직각 방향 길이) m — find_blocks yaw 규약(눕힘 · 옆세움 = LENGTH 변, 세움 = WIDTH 변)."""
        length, width, thick = self.cfg['block_actual_m']
        return {'THICKNESS': (length, width), 'WIDTH': (length, thick), 'LENGTH': (width, thick)}[up]

    def _block_rect(self, b):
        return (b['x_m'], b['y_m'], b['yaw_deg']) + self._footprint(b['up'])

    def _spot_rect(self, s, up):
        return (s['x_m'], s['y_m'], s['yaw_deg']) + self._footprint(up)

    @staticmethod
    def _rects_overlap(a, b):
        """돌린 두 직사각형(x, y, yaw_deg, 길이, 폭)이 넓이를 가지고 겹치나(분리축). 변이 닿기만 하면 겹침 아님 — 여유값은 두지 않는다."""
        import math

        def axes_corners(r):
            x, y, yaw, length, width = r
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            ux, vx = (c, s), (-s, c)
            pts = [(x + i * length / 2 * ux[0] + j * width / 2 * vx[0], y + i * length / 2 * ux[1] + j * width / 2 * vx[1])
                   for i in (-1, 1) for j in (-1, 1)]
            return (ux, vx), pts

        (a_axes, a_pts), (b_axes, b_pts) = axes_corners(a), axes_corners(b)
        for ax in a_axes + b_axes:
            pa = [p[0] * ax[0] + p[1] * ax[1] for p in a_pts]
            pb = [p[0] * ax[0] + p[1] * ax[1] for p in b_pts]
            if min(max(pa), max(pb)) - max(min(pa), min(pb)) <= 1e-9:
                return False
        return True

    def _obstacles(self, blocks, picked_index):
        """같은 응답에서 고른 인덱스 하나만 빼고 나머지를 장애물 자세로 바꾼다. 반환: 자세 목록, 또는 요청을 막는 BLOCKED dict.

        나머지 중 형식이 틀린 블록이 있으면 일부만 보내지 않고 요청을 막는다(조용히 빼면 그 자리를 비었다고 보고 지나간다).
        tilted · under(덮임) · top(겹침 중 위)도 모두 상자를 만든다(E-68). tilted 는 지금 '가장 높은 점인 반듯한 상자'(top_z_m 기준), 챌린지 ④ 때 기울기 상자로 바꾼다.
        under 의 top_z_m 이 그 블록 자신의 윗면인지는 비전(민범진) 확인 전이다.
        """
        others = [(i, b) for i, b in enumerate(blocks) if i != picked_index]
        for i, b in others:
            if not self.picker.is_valid(b):
                return {'status': 'BLOCKED', 'reason': 'INVALID_OBSTACLE', 'index': i, 'guide': 'CHECK_BLOCKS',
                        'message': f'장애물 블록 {i}번의 값이 잘못돼 집기를 시작하지 않는다'}
        return [block_pose(self.cfg, b['up'], b['yaw_deg'], b['x_m'], b['y_m'], b['top_z_m']) for _, b in others]

    @staticmethod
    def _why(counts):
        """제외 이유 개수 → 사람이 읽을 안내 글자(0 인 것은 뺀다)."""
        names = {'under': '다른 블록에 덮임', 'tilted': '기울어짐', 'other_up': '필요한 자세가 아님',
                 'no_clear': '손가락 틈 부족', 'narrow': '틈이 기준보다 좁음', 'invalid': '잘못된 값'}
        parts = [f'{names[k]} {n}개' for k, n in counts.items() if n and k in names]
        return '맞는 블록이 없다: ' + (', '.join(parts) if parts else '이유 없음')

    # ---------- 실패 뒤 ----------
    def on_failure(self, reason, phase=None, grasped=None, area_unchanged=None):
        """집기 실패(또는 조회 실패) 뒤 다음 동작(failure_action). REPLAN_SAME · WAIT 이면 지금 후보를 유지하고, 그 밖이면 후보를 버린다."""
        action = failure_action(reason, phase, grasped, area_unchanged)
        if action not in (REPLAN_SAME, WAIT):
            with self._lock:
                self.candidate = None
        return action

    def reusable_request(self):
        """같은 관측으로 다시 계획할 때 쓸 요청(복사본). 후보가 버려졌으면 None."""
        with self._lock:
            return None if self.candidate is None else copy.deepcopy(self.candidate['request'])

    def after_wait_start(self):
        """흩뿌림 대기(WAIT_SUPPLY)에서 [계속](start)을 받았을 때: 다시 관측하고 조회한다. 공급 칸은 초기화하지 않는다."""
        self.invalidate()
        return {'reobserve': True, 'reset_supply_slots': False}

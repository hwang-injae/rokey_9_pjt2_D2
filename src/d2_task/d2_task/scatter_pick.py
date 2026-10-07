# -*- coding: utf-8 -*-
"""흩뿌림 집기 준비 ScatterFlow — 목표 블록을 유지한 채 find_blocks 결과로 후보를 고르고 집기 요청을 준비한다 (ROS 없이 동작, W130).

TaskManager 의 SELECT 안에서(공개 상태를 늘리지 않고) 쓰려고 만든 계산 클래스다. 아직 TaskManager · 로봇 호출에 연결하지 않았다.
호출 흐름(연결할 때): ① SELECT 가 TaskPlanner 에서 목표 블록(block_id · 잡기 · 놓을 자세)을 정한다 → ② begin() 으로 요청 표를 받는다 →
③ 잠금 밖에서 observe_supply 로 가서 find_blocks 를 부른다 → ④ is_current() 로 정지 · 종료 · 새 요청 · 상태 변화가 없었는지 확인한 뒤
⑤ prepare() 로 후보를 고르고 집기 요청(PickPlace 목표 칸)을 만든다 → ⑥ 실패하면 on_failure() 가 후보를 버릴지(재관측) 유지할지(같은 관측으로 재계획) 알려 준다.
설정은 명시적으로 주입한다: supply_mode == 'scatter' 이고 open_width_m 이 정해져야 실행 가능한 요청을 만든다(이름 · 기본값 · 열림 폭은 확정 전).
"""
import copy

from d2_motion.motion_math import column, half_height, quat_from_axes, rot_z, rpy_matrix

from d2_task.block_picker import BlockPicker, grasp_target

SUPPLY_SCATTER = 'scatter'

# pick_place 실패 이유 → 다음 동작. 블록이 움직였을 가능성이 있으면 이전 후보를 다시 쓰지 않는다(REOBSERVE).
RETRY_ACTIONS = {
    'PLAN_FAILED': 'REPLAN_SAME',          # 계획만 실패 — 팔이 안 움직였고 블록 그대로 → 같은 관측 · 후보로 다시 계획
    'BUSY': 'WAIT',                        # 다른 목표 실행 중 — 기다렸다 같은 요청
    'GRASP_FAILED': 'REOBSERVE',           # 손가락이 닿았다 — 블록이 밀렸을 수 있다
    'SLOT_EMPTY': 'REOBSERVE',             # 그 자리에 블록이 없었다 — 관측이 틀렸거나 움직였다
    'TIMEOUT': 'REOBSERVE',                # 중간에 멈췄다 — 닿았는지 모른다
    'STOPPED': 'REOBSERVE',
    'CANCELED': 'REOBSERVE',
    'NO_FEEDBACK': 'REOBSERVE',
    'LOOKUP_FAILED': 'REOBSERVE',          # find_blocks 실패 · 시간 초과 · 잘못된 답 — 빈 공급이 아니다(횟수 상한은 정책 미정)
    'ERROR': 'ERROR',                      # 사람 호출
    'GRIPPER_NO_RESPONSE': 'ERROR',
}


def failure_action(reason):
    """pick_place(또는 조회) 실패 이유 → REPLAN_SAME · WAIT · REOBSERVE · ERROR. 모르는 이유는 ERROR(조용히 재사용하지 않는다)."""
    return RETRY_ACTIONS.get(reason, 'ERROR')


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
        self.candidate = None                 # 지금 유효한 집기 요청(재계획 때만 다시 쓴다)

    @property
    def configured(self):
        """흩뿌림 모드가 명시됐고 열림 폭이 정해졌나. 아니면 실행 가능한 요청을 만들지 않는다."""
        return self.supply_mode == SUPPLY_SCATTER and self.open_width_m is not None

    # ---------- 늦은 답 막기 ----------
    def begin(self, epoch):
        """관측 · 조회를 시작한다. 이전 요청은 무효가 되고, 같은 epoch(상태 변화 표)와 함께 쓸 표를 돌려준다. 후보도 비운다."""
        self._token += 1
        self.candidate = None
        return (self._token, epoch)

    def is_current(self, ticket, epoch, halted):
        """답을 적용해도 되나: 더 새 요청이 없고 · 상태가 안 바뀌었고 · 정지 · 종료 중이 아닐 때만 참."""
        return (not halted) and ticket == (self._token, epoch)

    # ---------- 후보 고르기와 요청 준비 ----------
    def prepare(self, target, response):
        """목표 블록 + find_blocks 응답 → 집기 요청 준비. 반환 dict 의 status:

        PICK          request = PickPlace 목표 칸(block_id · supply_slot '' · grasp · pick_pose · place_pose · open_width_m)과 candidate(복사본)
        NOT_CONFIGURED 모드 · 열림 폭이 아직 정해지지 않음 — 요청을 만들지 않는다
        EMPTY         공급에 블록이 없다 → 채우기 안내(guide 'REFILL')
        NO_MATCH      블록은 있는데 맞는 후보가 없다 → 제외 이유(counts · message)를 안내(guide 'CHECK_BLOCKS')
        LOOKUP_FAILED 조회 실패 · 잘못된 응답 — 공급 부족이 아니다(reason)
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
            return {'status': 'NO_MATCH', 'guide': 'CHECK_BLOCKS', 'counts': dict(picked['counts']),
                    'message': self._why(picked['counts'])}
        c = picked['block']
        center, quat = block_pose(self.cfg, c['up'], c['yaw_deg'], c['x_m'], c['y_m'], c['top_z_m'])
        request = {'block_id': target['block_id'], 'supply_slot': '', 'grasp': target['grasp'],
                   'pick_pose': (center, quat), 'place_pose': copy.deepcopy(target['place_pose']),
                   'open_width_m': self.open_width_m}
        self.candidate = {'request': request, 'candidate': copy.deepcopy(c), 'gap_mm': picked['gap_mm']}
        return {'status': 'PICK', 'request': copy.deepcopy(request), 'candidate': copy.deepcopy(c), 'gap_mm': picked['gap_mm']}

    @staticmethod
    def _why(counts):
        """제외 이유 개수 → 사람이 읽을 안내 글자(0 인 것은 뺀다)."""
        names = {'under': '다른 블록에 덮임', 'tilted': '기울어짐', 'other_up': '필요한 자세가 아님', 'no_gap_info': '틈 정보 없음',
                 'no_clear': '손가락 틈 부족', 'narrow': '틈이 기준보다 좁음', 'invalid': '잘못된 값'}
        parts = [f'{names[k]} {n}개' for k, n in counts.items() if n and k in names]
        return '맞는 블록이 없다: ' + (', '.join(parts) if parts else '이유 없음')

    # ---------- 실패 뒤 ----------
    def on_failure(self, reason):
        """집기 실패(또는 조회 실패) 뒤 다음 동작. REPLAN_SAME · WAIT 이면 지금 후보를 유지하고, 그 밖(REOBSERVE · ERROR)이면 후보를 버린다.

        블록이 움직였을 가능성이 있는 실패에는 이전 후보를 다시 쓰지 않는다.
        """
        action = failure_action(reason)
        if action not in ('REPLAN_SAME', 'WAIT'):
            self.candidate = None
        return action

    def reusable_request(self):
        """같은 관측으로 다시 계획할 때 쓸 요청(복사본). 후보가 버려졌으면 None."""
        return None if self.candidate is None else copy.deepcopy(self.candidate['request'])

    def after_wait_start(self):
        """흩뿌림 대기(WAIT_SUPPLY)에서 [계속](start)을 받았을 때: 다시 관측하고 조회한다. 공급 칸은 초기화하지 않는다."""
        self.candidate = None
        return {'reobserve': True, 'reset_supply_slots': False}

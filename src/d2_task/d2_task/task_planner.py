# -*- coding: utf-8 -*-
"""작업 판단 TaskPlanner — 진행표를 지키고 다음에 놓을 블록 하나를 고른다 (ROS 없이 동작, SDD 6.4).

task 노드(TaskManager)가 부른다. ROS 없이 시험할 수 있게 노드와 나눴다(팀 규칙 ②).
좌표 계산은 새로 만들지 않고 집기·놓기가 쓰는 d2_motion.motion_math 를 그대로 쓴다(SDD 3.3: 변환은 한 곳).
"""
import math

from d2_motion.motion_math import column, quat_from_axes, recipe_blocks, slot_block_pose, up_axis
from d2_task.recipe_document import RecipeDocument

STATES = ('present', 'absent', 'occluded', 'unknown')   # IRD 2장 '블록 관측 states'


def _nan_to_none(v):
    """JSON 에는 NaN 을 쓸 수 없어서(안 잰 값) None 으로 바꾼다."""
    return None if isinstance(v, float) and math.isnan(v) else v


class TaskPlanner:
    """recipe(구조) · placements(조립 방법)와 관측 결과로 진행표를 만들고 다음 블록을 고른다.

    입력: robot.yaml 을 읽은 dict(cfg), 레시피 dict. 단위는 노드끼리 m·rad.
    바깥 영향: 없음(계산만). 로봇·메시지를 건드리지 않는다.
    실패: 레시피 · 설정이 서로 안 맞으면 만들 때 ValueError. 다음 블록을 못 고르면 예외 대신 status 로 알린다.
    """

    def __init__(self, cfg, recipe, placements):
        """레시피 블록을 sequence 순서로 펼치고, 블록마다 받침 block_id 를 붙인다. 진행표는 모두 unknown 으로 시작한다.

        recipe 는 구조 dict(recipe/2.0, mm), placements 는 조립 방법 dict(placements/2.0). 바깥 영향 없음.
        블록 크기·잡기·받침이 잘못되면 ValueError 를 낸다. 좌표 계산은 recipe_blocks 한 곳에서 한다.
        """
        self.cfg = cfg
        document = RecipeDocument(recipe, placements)
        recipe = document.geometry
        self.design_id = recipe['model']['model_id']
        blocks = recipe_blocks(cfg, recipe)
        steps = sorted(recipe['steps'], key=lambda s: s['sequence'])
        if any(st['grasp'] != b['grasp'] for st, b in zip(steps, blocks)):
            raise ValueError('recipe 의 grasp 가 블록 방향 · grasp_axis 와 다르다')
        block_of = {st['instance_id']: b['block_id'] for st, b in zip(steps, blocks)}
        self.blocks = [dict(b, supports=[block_of[i] for i in st['support_instance_ids']])
                       for st, b in zip(steps, blocks)]
        # 시작 때는 아무것도 본 적이 없다. 첫 진행 확인(관측) 전에 놓으면 이미 있는 블록 위에 겹쳐 놓을 수 있다.
        self.progress = {b['block_id']: {'state': 'unknown', 'dx_m': float('nan'), 'dy_m': float('nan'),
                                         'dz_m': float('nan'), 'top_z_m': float('nan')} for b in self.blocks}
        self.empty_slots = set()

    def update_progress(self, observed):
        """관측 결과를 진행표에 넣는다. observed = {block_id: {'state', 'dx_m', 'dy_m', 'dz_m', 'top_z_m'}}.

        state 는 present · absent · occluded · unknown. 안 잰 값은 칸을 빼면 NaN 으로 둔다(측정값만 기록, 판정 없음 — E-20).
        레시피에 없는 block_id 나 모르는 state 가 하나라도 있으면 아무것도 바꾸지 않고 ValueError.
        """
        for block_id, obs in observed.items():
            if block_id not in self.progress:
                raise ValueError(f'레시피에 없는 블록: {block_id}')
            if obs.get('state') not in STATES:
                raise ValueError(f'{block_id}: state 는 {STATES} 중 하나 ({obs.get("state")!r})')
        for block_id, obs in observed.items():
            row = self.progress[block_id]
            row['state'] = obs['state']
            for key in ('dx_m', 'dy_m', 'dz_m', 'top_z_m'):
                row[key] = obs.get(key, float('nan'))

    def mark_slot_empty(self, slot):
        """집기·놓기가 SLOT_EMPTY 를 돌려준 공급 칸(1부터)을 '빔'으로 적는다. 다음 next_block 부터 그 칸을 피한다."""
        if not 1 <= slot <= len(self.cfg['supply_slots']):
            raise ValueError(f'공급 칸 {slot} 이 없다 (1 ~ {len(self.cfg["supply_slots"])})')
        self.empty_slots.add(slot)

    def supply_refilled(self):
        """사람이 공급을 채우고 [계속](start)을 눌렀을 때 부른다. 모든 칸을 다시 '있음'으로 본다."""
        self.empty_slots.clear()

    def next_target(self):
        """다음에 놓을 블록(어디서 집을지는 정하지 않는다)을 돌려준다. 공급 방식(칸 · 흩뿌림)과 상관없는 목표 선택이다.

        status: FOUND(block_id · grasp · place_pose · index) · DONE · NO_SUPPORT(block_id) · UNKNOWN_BLOCK(block_id). 규칙은 next_block 과 같다.
        계산만 한다(진행표를 바꾸지 않는다).
        """
        for index, b in enumerate(self.blocks):
            state = self.progress[b['block_id']]['state']
            if state == 'present':
                continue
            if state != 'absent':
                return {'status': 'UNKNOWN_BLOCK', 'block_id': b['block_id']}
            if any(self.progress[s]['state'] != 'present' for s in b['supports']):
                return {'status': 'NO_SUPPORT', 'block_id': b['block_id']}
            return {'status': 'FOUND', 'block_id': b['block_id'], 'grasp': b['grasp'],
                    'place_pose': (b['center'], b['quat']), 'index': index}
        return {'status': 'DONE'}

    def next_block(self, avoid_slots=()):
        """레시피 sequence 에서 아직 안 놓인 첫 블록을 골라 집기·놓기 목표를 돌려준다.

        avoid_slots = 이번 호출에서만 피할 공급 칸 번호들(1부터). 헛잡은 칸 대신 다음 칸을 쓰게 할 때 넘긴다
        (빔으로 적지는 않는다 — 다음 호출에서는 다시 후보가 된다).
        돌려주는 dict 의 status:
          FOUND        block_id · supply_slot('1'~) · grasp · pick_pose · place_pose. pose = (xyz m, 쿼터니언 xyzw), base_link,
                       블록 중심 + 블록 자신의 축(PickPlace.action 과 같다)
          DONE         모든 블록이 놓였다
          NO_SUPPORT   그 블록의 받침이 놓여 있지 않다 (block_id)
          UNKNOWN_BLOCK 그 블록이 있는지 없는지 못 봤다(occluded · unknown) → 재관측 (block_id)
          WAIT_SUPPLY  그 블록의 잡기에 맞는 공급 칸이 빔 · avoid_slots 때문에 하나도 안 남았다 (block_id).
                       avoid_slots 를 넘긴 호출의 WAIT_SUPPLY 는 '공급이 비었다'가 아니라 '다른 칸이 없다'이다 —
                       부르는 쪽(TaskManager)이 구분해야 한다
        공급 칸은 robot.yaml supply_slots 의 grasp 가 블록 잡기와 같은 칸(칸마다 그리퍼 방향이 고정 — 10/6 다시 교시)
        중 빔으로 적히지 않고 피할 칸도 아닌 칸을, 같은 잡기 블록 순번대로 돌려 쓴다. 고르는 규칙은 d2_motion run_recipe 와 같다.
        place_pose 높이는 recipe_blocks 가 받침의 실제 윗면 위에 실측 블록(block_actual_m)으로 쌓아 올린 값이다.
        robot.yaml 에 그 잡기의 공급 칸이 아예 없으면 ValueError(설정 오류).
        """
        t = self.next_target()
        if t['status'] != 'FOUND':
            return t
        slot = self._pick_slot(t['index'], avoid_slots)
        if slot is None:
            return {'status': 'WAIT_SUPPLY', 'block_id': t['block_id']}
        center, rot = slot_block_pose(self.cfg, slot)
        return {'status': 'FOUND', 'block_id': t['block_id'], 'supply_slot': str(slot), 'grasp': t['grasp'],
                'pick_pose': (center, quat_from_axes(*(column(rot, k) for k in range(3)))),
                'place_pose': t['place_pose']}

    def judge(self, expect_present=()):
        """진행표에서 확실히 어긋난 블록을 찾는다(IRD 7장 OFFSET_OVER). 문제가 없으면 빈 목록.

        입력: expect_present = 지금 있어야 하는 block_id 들(방금 놓은 블록). 반환: [{'block_id', 'reason', 'detail'}].
        잡는 것: 있어야 하는데 없음 · 높이(dz_m)가 두께 절반보다 어긋남 · 받침이 없는데 위층만 있음(무너짐).
        occluded · unknown 은 문제로 안 본다(못 본 것이지 어긋난 것이 아니다 → next_block 이 UNKNOWN_BLOCK 으로 재관측).
        높이 기준 block_actual_m[2] / 2 는 SDD 6.3 '기준 안'(안)이다. 높이 기준은 W058(손목 보정) 뒤에 다시 본다.
        ±2 mm 판정 · 위층 보정은 하지 않는다(E-20, 오차는 측정값만). '설계 밖 블록'은 check_progress 가 설계 블록만
        묻기 때문에 여기서 표현할 수 없다(W121 확인 사항).
        없는 block_id 를 넘기면 ValueError.
        """
        for block_id in expect_present:
            if block_id not in self.progress:
                raise ValueError(f'레시피에 없는 블록: {block_id}')
        limit = self.cfg['block_actual_m'][2] / 2
        problems = []
        for block_id in expect_present:
            if self.progress[block_id]['state'] == 'absent':
                problems.append({'block_id': block_id, 'reason': 'OFFSET_OVER', 'detail': '놓은 블록이 없다'})
        for b in self.blocks:
            row = self.progress[b['block_id']]
            if row['state'] != 'present':
                continue
            if not math.isnan(row['dz_m']) and abs(row['dz_m']) > limit:
                problems.append({'block_id': b['block_id'], 'reason': 'OFFSET_OVER',
                                 'detail': f'높이가 {row["dz_m"] * 1000:.1f} mm 어긋났다(기준 ±{limit * 1000:.1f} mm)'})
            lost = [s for s in b['supports'] if self.progress[s]['state'] == 'absent']
            if lost:
                problems.append({'block_id': b['block_id'], 'reason': 'OFFSET_OVER',
                                 'detail': f'받침 {", ".join(lost)} 이 없는데 위층만 있다'})
        return problems

    def progress_message(self, run_id, obs_stamp):
        """진행표를 /d2/task/progress 의 JSON progress/1 (dict) 로 만든다. 안 잰 값(NaN)은 null.

        입력: run_id(없으면 None), obs_stamp = 관측한 시각(s). center_m · quat 은 설계 자리(base_link, m).
        by 는 robot 만 쓴다(스캔 실물은 W119).
        """
        blocks = []
        for b in self.blocks:
            row = self.progress[b['block_id']]
            blocks.append({'block_id': b['block_id'], 'state': row['state'], 'by': 'robot',
                           'center_m': list(b['center']), 'quat': list(b['quat']),
                           'top_z_m': _nan_to_none(row['top_z_m']), 'dz_m': _nan_to_none(row['dz_m']),
                           'dx_m': _nan_to_none(row['dx_m']), 'dy_m': _nan_to_none(row['dy_m'])})
        return {'schema': 'progress/1', 'design_id': self.design_id, 'run_id': run_id,
                'obs_stamp': obs_stamp, 'blocks': blocks}

    def _pick_slot(self, index, avoid_slots=()):
        """self.blocks[index] 를 집을 공급 칸 번호(1부터). 맞는 칸이 모두 비었거나 피할 칸뿐이면 None.

        칸에 grasp 가 없으면 같은 자세(block_up)의 칸이면 된다. 같은 잡기로 앞서 놓일 블록 수만큼 칸을 돌려 고르게 쓴다.
        avoid_slots 는 정수로 바꿔 빈 칸(empty_slots)과 함께 후보에서 뺀다.
        """
        b = self.blocks[index]
        up = up_axis(b['rot'])
        same = [k + 1 for k, st in enumerate(self.cfg['supply_slots'])
                if st.get('grasp', b['grasp']) == b['grasp'] and st.get('block_up', 'THICKNESS') == up]
        if not same:
            raise ValueError(f'{b["block_id"]}: {b["grasp"]} 공급 칸이 robot.yaml 에 없다')
        avoid = {int(s) for s in avoid_slots}
        free = [s for s in same if s not in self.empty_slots and s not in avoid]
        if not free:
            return None
        turn = sum(1 for o in self.blocks[:index] if o['grasp'] == b['grasp'])
        return free[turn % len(free)]

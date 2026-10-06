# -*- coding: utf-8 -*-
"""작업 판단 TaskPlanner — 진행표를 지키고 다음에 놓을 블록 하나를 고른다 (ROS 없이 동작, SDD 6.4).

task 노드(TaskManager)가 부른다. ROS 없이 시험할 수 있게 노드와 나눴다(팀 규칙 ②).
좌표 계산은 새로 만들지 않고 집기·놓기가 쓰는 d2_motion.motion_math 를 그대로 쓴다(SDD 3.3: 변환은 한 곳).
"""
from d2_motion.motion_math import column, quat_from_axes, recipe_blocks, slot_block_pose, up_axis

STATES = ('present', 'absent', 'occluded', 'unknown')   # IRD 2장 '블록 관측 states'


class TaskPlanner:
    """레시피(assembly.recipe/1.0)와 관측 결과로 진행표를 만들고, sequence 순서로 다음 블록을 고른다.

    입력: robot.yaml 을 읽은 dict(cfg), 레시피 dict. 단위는 노드끼리 m·rad.
    바깥 영향: 없음(계산만). 로봇·메시지를 건드리지 않는다.
    실패: 레시피 · 설정이 서로 안 맞으면 만들 때 ValueError. 다음 블록을 못 고르면 예외 대신 status 로 알린다.
    """

    def __init__(self, cfg, recipe):
        """레시피 블록을 sequence 순서로 펼치고, 블록마다 받침 block_id 를 붙인다. 진행표는 모두 unknown 으로 시작한다.

        블록 크기가 robot.yaml 의 block_size_m 와 다르면 recipe_blocks 가 ValueError 를 낸다.
        """
        self.cfg = cfg
        self.design_id = recipe['model']['model_id']
        blocks = recipe_blocks(cfg, recipe)
        steps = sorted(recipe['steps'], key=lambda s: s['sequence'])
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

    def next_block(self):
        """레시피 sequence 에서 아직 안 놓인 첫 블록을 골라 집기·놓기 목표를 돌려준다.

        돌려주는 dict 의 status:
          FOUND        block_id · supply_slot('1'~) · grasp · pick_pose · place_pose. pose = (xyz m, 쿼터니언 xyzw), base_link,
                       블록 중심 + 블록 자신의 축(PickPlace.action 과 같다)
          DONE         모든 블록이 놓였다
          NO_SUPPORT   그 블록의 받침이 놓여 있지 않다 (block_id)
          UNKNOWN_BLOCK 그 블록이 있는지 없는지 못 봤다(occluded · unknown) → 재관측 (block_id)
          WAIT_SUPPLY  그 블록의 자세(눕힘 · 세움)에 맞는 공급 칸이 모두 비었다 (block_id) → 사람이 채운 뒤 supply_refilled()
        공급 칸은 그 자세로 놓인 칸 중 빔으로 적히지 않은 칸을, 같은 자세 블록 순번대로 돌려 쓴다
        (위에서 집으므로 자세를 못 바꾼다. 같은 칸만 쓰면 한 칸이 먼저 빈다).
        robot.yaml 에 그 자세의 공급 칸이 아예 없으면 ValueError(설정 오류).
        """
        for index, b in enumerate(self.blocks):
            state = self.progress[b['block_id']]['state']
            if state == 'present':
                continue
            if state != 'absent':
                return {'status': 'UNKNOWN_BLOCK', 'block_id': b['block_id']}
            if any(self.progress[s]['state'] != 'present' for s in b['supports']):
                return {'status': 'NO_SUPPORT', 'block_id': b['block_id']}
            slot = self._pick_slot(index)
            if slot is None:
                return {'status': 'WAIT_SUPPLY', 'block_id': b['block_id']}
            center, rot = slot_block_pose(self.cfg, slot)
            return {'status': 'FOUND', 'block_id': b['block_id'], 'supply_slot': str(slot), 'grasp': b['grasp'],
                    'pick_pose': (center, quat_from_axes(*(column(rot, k) for k in range(3)))),
                    'place_pose': (b['center'], b['quat'])}
        return {'status': 'DONE'}

    def _pick_slot(self, index):
        """self.blocks[index] 를 집을 공급 칸 번호(1부터). 맞는 칸이 모두 비었으면 None.

        같은 자세로 앞서 놓일 블록 수만큼 칸을 돌려, 칸마다 고르게 쓴다.
        """
        up = up_axis(self.blocks[index]['rot'])
        same = [k + 1 for k, st in enumerate(self.cfg['supply_slots']) if st.get('block_up', 'THICKNESS') == up]
        if not same:
            raise ValueError(f'{self.blocks[index]["block_id"]}: {up} 가 위로 놓인 공급 칸이 robot.yaml 에 없다')
        free = [s for s in same if s not in self.empty_slots]
        if not free:
            return None
        turn = sum(1 for b in self.blocks[:index] if up_axis(b['rot']) == up)
        return free[turn % len(free)]

# -*- coding: utf-8 -*-
"""받침 경유 흩뿌림 집기(robot.yaml scatter.via_spare, 10/13 PL 10시 지시) 시험 — ROS 없이 TaskManager · ScatterFlow 를 가짜 IO 로 돌린다.

흐름: 공급 관측(find_blocks height_map) → 고르기(BlockPicker.rank) → 받침에 놓음(조립 아님 — 옆으로 선 · 선 블록은 그 자세로) → 받침 위 촬영 자세
→ find_blocks area 'spare' → 선 블록이면 잡기 TIP 으로 눕히고 다시 잼 → 반대 폭으로 맞추기(CENTER_*) → 같은 잰 값에서 레시피 잡기로 다시 집어
조립(1 mm 띄운 자리) → VERIFY. 실패 규칙: 팔이 움직인 실패만 셈 · 움직이기 전 길 거절은 다음 후보 · 연달아 3번이면 사람.
받침에 놓은 블록이 안 보이면 사람(1번째 다시 잼 · 연달아 2번째 공급에서) — one_pick 과 같음.
"""
import copy

from d2_task.scatter_pick import tilt_block_pose
from d2_task.task_manager import PLAN_STREAK, TaskManager
from test_block_picker import CFG, blk
from test_scatter_connect import ScatterIO, calls, fast_wait, start
from test_task_manager import IDS, SAFE_OK, drive

SPARE_CFG = dict(copy.deepcopy(CFG), supply_mode='scatter', scatter=dict(CFG['scatter'], via_spare=True, max_attempts=3))
SPOT = SPARE_CFG['spare_spots'][0]
EMPTY_HM = {'x0_m': 0.0, 'y0_m': 0.0, 'res_m': 0.002, 'nx': 1, 'ny': 1, 'z_m': [None]}    # 깊이는 받았고 막는 점은 없음
L, W, T = SPARE_CFG['block_actual_m']


def supply():
    """공급에 눕힌 블록 둘(틈 넉넉) — 높이 지도는 없음(깊이 미리 거르기 안 함)."""
    top = SPARE_CFG['table_z_m'] + T
    return {'ok': True, 'blocks': [blk(x=0.30, y=0.20, top=top, length=60, width=60), blk(x=0.45, y=0.22, top=top, length=60, width=60)]}


def on_spare(up='THICKNESS'):
    """받침 위 블록 하나(find_blocks area 'spare' 꼴)."""
    size = {'THICKNESS': T, 'WIDTH': W, 'LENGTH': L}[up]
    b = blk(x=SPOT['x_m'] + 0.001, y=SPOT['y_m'] - 0.002, top=SPOT['z_m'] + size, up=up, length=99, width=99, thickness=99)
    b.update(yaw_deg=-89.0, base_z_m=SPOT['z_m'], confidence=1.0)
    return {'ok': True, 'blocks': [b]}


class SpareIO(ScatterIO):
    """find_blocks 선택 칸(height_map · area)을 받아 기록하고, area 'spare' 면 spare_script(없으면 받침 위 눕힌 블록)를 돌려준다."""

    def __init__(self):
        super().__init__()
        self.opts, self.spare_script = [], []

    def find_blocks(self, run_id, should_abort, **opts):
        self.opts.append(dict(opts))
        self.events.append(('call', 'find_blocks', run_id))
        if opts.get('area') == 'spare':
            return (True, '', self.spare_script.pop(0) if self.spare_script else on_spare())
        step = self.find_script.pop(0) if self.find_script else (True, '', supply())
        return step(self) if callable(step) else step


def make():
    io = SpareIO()
    m = TaskManager(SPARE_CFG, io, open_width_m=0.0)
    io.manager = m
    fast_wait(m)
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def test_블록마다_받침에_눕히고_받침에서_다시_집어_끝까지_조립한다():
    m, io = make()
    start(m)
    drive(m, 'DONE')
    assert m.last_build['placed'] == len(IDS)
    legs = [g for g in io.goals]
    assert len(legs) == 3 * len(IDS)                                             # 블록마다 받침에 옮기기 + 받침 맞추기 + 받침에서 다시 집기
    to_spare, center, from_spare = legs[0::3], legs[1::3], legs[2::3]
    assert all(g.get('via_spare') for g in to_spare) and not any(g.get('via_spare') for g in from_spare)
    other = {'FLAT_SHORT': 'FLAT_LONG', 'FLAT_LONG': 'FLAT_SHORT'}
    for c, g in zip(center, from_spare):                                          # 레시피 잡기의 반대 폭으로 맞춘 뒤 다시 재지 않고 같은 잰 값으로 집는다
        assert c.get('spare_center') and c['grasp'] == 'CENTER_' + other[g['grasp']] and c['pick_pose'] == g['pick_pose']
    for g in to_spare:                                                           # 받침 윗면(80 mm) 위에 눕힘
        assert abs(g['place_pose'][0][2] - (SPOT['z_m'] + T / 2)) < 1e-9
        assert abs(g['place_pose'][0][0] - SPOT['x_m']) < 1e-9 and abs(g['place_pose'][0][1] - SPOT['y_m']) < 1e-9
    for g in from_spare:                                                         # 받침에서 잰 중심 · 받침 높이로 집고, 장애물 없음
        assert abs(g['pick_pose'][0][0] - (SPOT['x_m'] + 0.001)) < 1e-9 and g['obstacles'] == []
        assert abs(g['pick_pose'][0][2] - (SPOT['z_m'] + T / 2)) < 1e-9
    targets = [c[1] for c in calls(io, 'move_to')]
    assert targets.count('observe_spare') == len(IDS)
    assert {'height_map': True} in io.opts and {'area': 'spare'} in io.opts


def test_받침에서_다시_집기는_레시피_잡기와_1mm_띄운_자리로_간다():
    m, io = make()
    start(m)
    drive(m, 'DONE')
    spread = m._spread
    from_spare = io.goals[2::3]
    for g in from_spare:
        part = next(p for p in m.planner.parts if p['part_id'] == g['part_id'])
        assert g['grasp'] == part['grasp']
        assert list(g['place_pose'][0]) == list(spread[g['part_id']])


def test_움직이기_전_길_거절은_세지_않고_연달아_3번이면_사람을_부른다():
    m, io = make()
    io.pick_script = [(False, 'PLAN_FAILED')] * PLAN_STREAK                      # 피드백 없음 = 팔이 안 움직였다
    start(m)
    drive(m, 'WAIT_SUPPLY')                                                      # 서지 않고 사람 확인(one_pick hold)
    assert len(io.goals) == PLAN_STREAK and all(g.get('via_spare') for g in io.goals)
    assert m._scatter_fail.get(io.goals[0]['part_id'], 0) == 0                   # 움직이기 전 거절은 시도 수에 안 셈
    assert len(m._spare_failed) == PLAN_STREAK                                   # 같은 후보를 다시 고르지 않게 기록
    assert m.command('start') == (True, '')                                      # [계속] → 처음 후보부터 다시 찍어 이어 감
    drive(m, 'DONE')
    assert m.last_build['placed'] == len(IDS)


def test_받침에_놓은_블록이_안_보이면_사람이_보고_계속이면_다시_잰다():
    m, io = make()
    io.spare_script = [{'ok': True, 'blocks': []}]
    start(m)
    drive(m, 'WAIT_SUPPLY')
    assert m._spare_on is not None and len(io.goals) == 1                        # 비었다고 보고 넘어가지 않는다(받침 기록 그대로)
    assert m.command('start') == (True, '')
    drive(m, 'DONE')
    assert m.last_build['placed'] == len(IDS)
    assert len(io.goals) == 3 * len(IDS)                                         # 다시 재서 그대로 이어 감(받침에 또 옮기지 않음)


def test_받침이_연달아_두_번_비어_보이면_계속_뒤_공급에서_다시_고른다():
    m, io = make()
    io.spare_script = [{'ok': True, 'blocks': []}, {'ok': True, 'blocks': []}]
    start(m)
    drive(m, 'WAIT_SUPPLY')
    assert m.command('start') == (True, '')
    m.run_once()                                                                 # [계속] → 받침을 다시 잼 → 또 비어 보임
    assert m.state == 'WAIT_SUPPLY' and m._spare_on is None
    assert m.command('start') == (True, '')
    drive(m, 'DONE')
    assert len(io.goals) == 3 * len(IDS) + 1                                     # 첫 블록은 받침에 한 번 더 옮겼다


def test_받침에_옆으로_선_블록이면_TIP_으로_눕히고_다시_재서_집는다():
    m, io = make()
    io.spare_script = [on_spare('WIDTH')]
    start(m)
    drive(m, 'DONE')
    tip = io.goals[1]
    assert tip['grasp'] == 'TIP' and tip.get('spare_tip')
    assert abs(tip['pick_pose'][0][2] - (SPOT['z_m'] + W / 2)) < 1e-9          # 받침 위 옆으로 선 블록 중심
    assert abs(tip['place_pose'][0][2] - (SPOT['z_m'] + T / 2)) < 1e-9          # 받침 가운데에 누움
    assert abs(tip['place_pose'][0][0] - SPOT['x_m']) < 1e-9 and abs(tip['place_pose'][0][1] - SPOT['y_m']) < 1e-9
    assert io.goals[2].get('spare_center') and len(io.goals) == 3 * len(IDS) + 1
    assert [c[1] for c in calls(io, 'move_to')].count('observe_spare') == len(IDS) + 1     # 눕힌 뒤 받침을 다시 잼


def test_기울어진_블록은_윗면_법선으로_두께_절반만_물고_받침에_그만큼_올려_놓는다():
    m, io = make()
    flow = m._scatter
    top = SPARE_CFG['table_z_m'] + T + 0.012
    tilted = blk(x=0.40, y=0.22, top=top + 0.01, tilted=True, overlap='top', length=60, width=60,
                 center_top_z_m=top, normal=[-0.2588, 0.0, 0.9659])
    target = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'place_pose': ((0.4, -0.07, 0.01), (0, 0, 0, 1))}
    assert flow.prepare_spare(target, {'ok': True, 'blocks': [tilted]})['status'] == 'NO_MATCH'      # 깊이 없으면 기울여 집기 안 함
    r = flow.prepare_spare(target, {'ok': True, 'blocks': [tilted], 'height_map': EMPTY_HM})
    assert r['status'] == 'SPARE' and r['request']['grasp'] == 'TILT'
    center, _, n = tilt_block_pose(dict(tilted), T)
    shift = SPARE_CFG['grasp_depth_m'] - T / 2
    assert all(abs(p - (c + k * shift)) < 1e-9 for p, c, k in zip(r['request']['pick_pose'][0], center, n))
    assert abs(r['request']['place_pose'][0][2] - (SPOT['z_m'] + T / 2 + shift)) < 1e-9


def test_옆으로_선_블록은_그_자세로_받침에_세워_놓는다():
    m, io = make()
    edge = blk(x=0.40, y=0.22, top=SPARE_CFG['table_z_m'] + W, up='WIDTH', length=60, thickness=60)
    target = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'place_pose': ((0.4, -0.07, 0.01), (0, 0, 0, 1))}
    r = m._scatter.prepare_spare(target, {'ok': True, 'blocks': [edge]})
    assert r['status'] == 'SPARE' and r['request']['grasp'].startswith('EDGE_')
    assert abs(r['request']['place_pose'][0][2] - (SPOT['z_m'] + W / 2)) < 1e-9


def test_세운_블록은_넘어지는_쪽이_받침_긴_변이_되게_놓는다():
    m, io = make()
    cfg = dict(SPARE_CFG, spare_base_m={'x_m': [0.2275, 0.5075], 'y_m': [-0.325, -0.245]})
    flow = type(m._scatter)(cfg, m._scatter.picker, supply_mode='scatter', open_width_m=0.0)
    assert flow.support_long_yaw() == 0.0
    stand = blk(x=0.40, y=0.22, top=SPARE_CFG['table_z_m'] + L, up='LENGTH', width=60, thickness=60)
    target = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'place_pose': ((0.4, -0.07, 0.01), (0, 0, 0, 1))}
    r = flow.prepare_spare(target, {'ok': True, 'blocks': [stand]})
    assert r['status'] == 'SPARE' and r['request']['grasp'].startswith('STAND_')
    from d2_task.scatter_pick import block_pose
    want = block_pose(cfg, 'LENGTH', -90.0, SPOT['x_m'], SPOT['y_m'], SPOT['z_m'] + L)
    assert all(abs(a - b) < 1e-9 for a, b in zip(r['request']['place_pose'][1], want[1]))


def test_세움_레시피는_세우기_교시가_없으면_사람을_부른다():
    cfg = copy.deepcopy(SPARE_CFG)
    cfg.pop('standup_taught')
    m = TaskManager(cfg, SpareIO(), open_width_m=0.0)
    target = {'part_id': IDS[0], 'grasp': 'STAND_SHORT', 'place_pose': ((0.4, -0.07, 0.04), (0, 0, 0, 1))}
    assert m._scatter.prepare_from_spare(target, on_spare())['status'] == 'STAND_NOT_READY'
    assert m._scatter.prepare_spare(target, supply())['status'] == 'STAND_NOT_READY'


def test_세움_레시피는_받침_가운데로_맞춘_뒤_교시_동작으로_세운다():
    """W131(10/13): 받침 위 눕힌 블록 → CENTER_FLAT_LONG 으로 받침 자리 가운데에 맞춤 → then = STANDUP_HIGH(받침 자리 눕힘 → 세움).
    옆세움(EDGE)은 CENTER_FLAT_SHORT → STANDUP_SIDE. 공급에서 받침으로 옮기는 1단계도 막지 않는다."""
    from d2_motion.motion_math import matrix_from_quat, up_axis
    m, io = make()
    for grasp, center, stand, up in (('STAND_SHORT', 'CENTER_FLAT_LONG', 'STANDUP_HIGH', 'LENGTH'),
                                     ('EDGE_SHORT', 'CENTER_FLAT_SHORT', 'STANDUP_SIDE', 'WIDTH')):
        target = {'part_id': IDS[0], 'grasp': grasp, 'place_pose': ((0.4, -0.07, 0.04), (0, 0, 0, 1))}
        assert m._scatter.prepare_spare(target, supply())['status'] == 'SPARE'
        r = m._scatter.prepare_from_spare(target, on_spare())
        assert r['status'] == 'SPARE_CENTER'
        c, t = r['request'], r['request']['then']
        assert c['grasp'] == center and c.get('spare_center') and not c.get('spare_stand')
        assert t['grasp'] == stand and t.get('spare_stand') and not t.get('spare_center')
        for pose in (c['pick_pose'], t['pick_pose'], t['place_pose']):                # 잰 중심이 아니라 받침 자리(교시 좌표)
            assert abs(pose[0][0] - SPOT['x_m']) < 1e-9 and abs(pose[0][1] - SPOT['y_m']) < 1e-9
        assert up_axis(matrix_from_quat(t['pick_pose'][1])) == 'THICKNESS' and up_axis(matrix_from_quat(t['place_pose'][1])) == up
        assert abs(t['place_pose'][0][2] - (SPOT['z_m'] + {'LENGTH': L, 'WIDTH': W}[up] / 2)) < 1e-9


def test_받침에_레시피_자세로_선_블록은_눕히지_않고_바로_집는다():
    m, io = make()
    target = {'part_id': IDS[0], 'grasp': 'STAND_SHORT', 'place_pose': ((0.4, -0.07, 0.04), (0, 0, 0, 1))}
    r = m._scatter.prepare_from_spare(target, on_spare('LENGTH'))
    assert r['status'] == 'PICK' and r['request']['grasp'] == 'STAND_SHORT'
    assert abs(r['request']['pick_pose'][0][2] - (SPOT['z_m'] + L / 2)) < 1e-9
    assert m._scatter.prepare_from_spare(target, on_spare('WIDTH'))['status'] == 'SPARE_TIP'     # 다른 자세로 선 블록은 그대로 눕힌다


def test_세우기가_끝나면_조립으로_세지_않고_받침을_다시_잰다():
    m, io = make()
    m.goal = {'part_id': IDS[0], 'grasp': 'STANDUP_HIGH', 'spare_stand': True, 'from_spare': True}
    m.last_pick = {'step': 'retreat'}
    m._spare_on = IDS[0]
    m.placed = None
    assert m._via_spare_result(True, '') is not False                        # False = 평소 성공 처리(놓음 · VERIFY)로 넘김 — 세우기는 아님
    assert m.state == 'SELECT' and m._spare_on == IDS[0] and m.placed is None   # 받침 기록 그대로 → 다음 SELECT 가 받침을 잰다


def test_맞추기_길이_안_나오면_맞추기_없이_다시_집는다():
    m, io = make()

    def script(io_):
        first = len(io_.goals) == 2                                               # 첫 블록의 맞추기만 길 거절(피드백 없음 = 안 움직임)
        return (False, 'PLAN_FAILED') if first else (True, '')
    io.pick_script = [script] * (3 * len(IDS))
    start(m)
    drive(m, 'DONE')
    assert [g['grasp'] for g in io.goals[:3]] == ['FLAT_SHORT', 'CENTER_FLAT_LONG', 'FLAT_SHORT']
    assert m.last_build['placed'] == len(IDS)
    assert m._plan_streak == 0


def test_움직이기_전_거절이면_다시_찍지_않고_같은_관측의_다음_후보로():
    m, io = make()
    io.pick_script = [(False, 'PLAN_FAILED')]                                     # 첫 후보: 움직이기 전 거절
    start(m)
    drive(m, 'DONE')
    first, second = io.goals[0], io.goals[1]
    assert first.get('via_spare') and second.get('via_spare') and first['attempt'] != second['attempt']
    finds = [o for o in io.opts if o.get('height_map')]
    assert len(finds) == len(IDS)                                                # 공급은 블록마다 한 번만 찍었다


def test_헛잡음이면_그_블록을_다시_찍어_같은_잡기로_한_번_더():
    m, io = make()
    io.pick_script = [(False, 'GRASP_FAILED')]
    io.pick_info = [{'step': 'grasp', 'grasped': False}]
    start(m)
    drive(m, 'DONE')
    first, again = io.goals[0], io.goals[1]
    assert again.get('regrasp') and again['grasp'] == first['grasp'] and again['attempt'] == first['attempt']
    assert m.last_build['placed'] == len(IDS)


def test_후보가_없으면_한_번_더_찍고_그래도_없으면_사람():
    m, io = make()
    edge_only = {'ok': True, 'blocks': [blk(x=0.40, y=0.22, top=SPARE_CFG['table_z_m'] + T, overlap='under', length=60, width=60)]}
    io.find_script = [(True, '', edge_only), (True, '', edge_only)]
    start(m)
    drive(m, 'WAIT_SUPPLY')
    assert len([o for o in io.opts if o.get('height_map')]) == 2


def test_팔이_움직인_실패는_공급과_받침을_합쳐_세고_다시_시작하면_비운다():
    m, io = make()
    io.pick_script = [(True, ''), (True, ''), (False, 'GRASP_FAILED'), (True, ''), (True, ''), (False, 'GRASP_FAILED')]
    io.pick_info = [None, None, {'step': 'grasp', 'grasped': False}, None, None, {'step': 'grasp', 'grasped': False}]
    start(m)
    drive(m, ('VERIFY', 'ERROR', 'DONE', 'WAIT_SUPPLY'))
    m._scatter_fail[IDS[0]] = 3
    m._reset_spare_tries()
    assert m._scatter_fail == {} and m._spare_failed == [] and m._plan_streak == 0


def test_시도_한도를_채우면_서지_않고_사람_확인_뒤_시도_수를_비워_이어_간다():
    m, io = make()
    n = SPARE_CFG['scatter']['max_attempts']
    io.pick_script = [(False, 'GRASP_FAILED')] * (2 * n)                         # 헛잡음 → 다시 찍어 한 번 더도 헛잡음 …
    io.pick_info = [{'step': 'grasp', 'grasped': False}] * (2 * n)
    start(m)
    drive(m, 'WAIT_SUPPLY')
    assert m._hold_then == 'reset' and m._scatter_fail[IDS[0]] >= n
    assert m.command('start') == (True, '')
    m.run_once()
    assert m._scatter_fail.get(IDS[0], 0) == 0


def test_출발하면_장면_정리_뒤_홈부터_간다():
    m, io = make()
    start(m)
    drive(m, 'SELECT')
    moves = [c[1] for c in calls(io, 'move_to')]
    assert moves[0] == 'home' and io.progress[0]['parts'] == []


def test_놓은_블록이_5mm_넘게_어긋나면_사람이_고치고_다시_확인한다():
    m, io = make()
    start(m)
    drive(m, 'VERIFY')
    part = m.placed
    real = io.check_progress

    def off(part_ids, should_abort):
        ok, why, rows = real(part_ids, should_abort)
        rows = {k: dict(v, dz_m=0.006) if k == part else v for k, v in rows.items()}
        return ok, why, rows
    io.check_progress = off
    m.run_once()
    assert m.state == 'WAIT_SUPPLY' and m._hold_then == 'verify'
    io.check_progress = real
    assert m.command('start') == (True, '')
    m.run_once()
    assert m.state == 'VERIFY'
    drive(m, 'DONE')


class W144IO(SpareIO):
    """W144 촬영 훅(io.w144_start · w144_after_place · w144_finish)을 기록한다. fail_at = 몇 번째 촬영에서 실패할지(1부터, None = 안 함)."""

    def __init__(self, fail_at=None):
        super().__init__()
        self.w144, self.fail_at = [], fail_at

    def w144_start(self, design_id, run_id):
        self.w144.append(('start', design_id, run_id))
        return (False, 'W144 기록 준비 실패: 필수 토픽 없음') if self.fail_at == 'start' else (True, '')

    def w144_event(self, event, **fields):
        self.w144.append(('event', event, fields))

    def w144_begin(self, part_id, sequence, center_m, quat_xyzw, evidence, retry_of=None):
        aid = f'a{sum(1 for e in self.w144 if e[0] == "begin") + 1}'
        self.w144.append(('begin', part_id, sequence, aid, retry_of))
        return aid, ''

    def w144_fail(self, action_id, part_id, sequence, reason, evidence):
        self.w144.append(('fail', action_id, part_id, reason))

    def w144_after_place(self, part_id, center_m, quat_xyzw, action_id=None, sequence=None, evidence=None):
        self.w144.append(('shot', part_id, tuple(center_m), tuple(quat_xyzw), action_id, evidence))
        n = sum(1 for e in self.w144 if e[0] == 'shot')
        return (False, 'ERROR') if self.fail_at == n else (True, '')

    def w144_finish(self, completed):
        self.w144.append(('finish', completed))


def make_w144(fail_at=None):
    io = W144IO(fail_at)
    m = TaskManager(SPARE_CFG, io, open_width_m=0.0)
    io.manager = m
    fast_wait(m)
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def test_W144_출발에_폴더_VERIFY_통과마다_설계_중심으로_촬영_끝에_ZIP():
    """one_pick start_rec · final_placed · finally 와 같은 때 — 촬영은 최종 자리 확인이 끝난 블록만, 설계 중심 · 방향(1 mm 띄우기 전)으로."""
    m, io = make_w144()
    start(m)
    drive(m, 'DONE')
    kinds = [e[0] for e in io.w144 if e[0] != 'event']
    assert kinds[0] == 'start' and kinds[-1] == 'finish' and io.w144[-1][1] is True
    shots = [e for e in io.w144 if e[0] == 'shot']
    begins = [e for e in io.w144 if e[0] == 'begin']
    assert [s[1] for s in shots] == list(IDS) and [b[1] for b in begins] == list(IDS)     # 블록마다 시도 하나(받침 경유 세 단계가 한 시도)
    for s, b in zip(shots, begins):
        part = next(p for p in m.planner.parts if p['part_id'] == s[1])
        assert s[2] == tuple(part['center']) and s[3] == tuple(part['quat'])
        assert s[4] == b[3] and [leg['leg'] for leg in s[5]['motion']] == ['to_spare', 'spare_center', 'from_spare']
    events = [e[1] for e in io.w144 if e[0] == 'event']
    assert events.count('pick_requested') == events.count('place_result') == 3 * len(IDS)
    assert events.count('candidate_selected') >= len(IDS)


def test_W144_촬영이_실패하면_다음_블록을_집지_않고_ERROR():
    m, io = make_w144(fail_at=1)
    start(m)
    drive(m, 'ERROR')
    goals_before = len(io.goals)
    assert [e[0] for e in io.w144].count('shot') == 1
    assert len(io.goals) == goals_before == 3                                      # 첫 블록(받침 옮기기 · 맞추기 · 다시 집기)만


def test_W144_기록_준비가_안_되면_움직이기_전에_ERROR():
    """INTEGRATION.md: 필수 토픽 · 녹화가 준비 안 되면 조립을 시작하지 않는다 — 출발 뒤 이동 · 집기 없이 ERROR."""
    m, io = make_w144(fail_at='start')
    assert m.command('select_design', 'bench') == (True, '')
    assert m.command('start') == (True, '') and m.state == 'ERROR'          # 출발은 받되 움직이기 전에 오류(사람이 카메라 · TF 를 보고 다시)
    assert io.goals == [] and calls(io, 'move_to') == [] and calls(io, 'find_blocks') == []


def test_W144_집기가_실패하면_그_시도를_닫고_다음_시도는_retry_of_로_잇는다():
    m, io = make_w144()
    io.pick_script = [(False, 'GRASP_FAILED')]                     # 첫 받침 옮기기에서 헛잡음(팔이 움직임)
    io.pick_info = [{'step': 'grasp', 'grasped': False}]
    start(m)
    drive(m, 'DONE')
    begins = [e for e in io.w144 if e[0] == 'begin']
    fails = [e for e in io.w144 if e[0] == 'fail']
    assert fails and fails[0][1] == begins[0][3] and fails[0][3] == 'GRASP_FAILED'
    assert begins[1][1] == begins[0][1] and begins[1][4] == begins[0][3]          # 같은 부품의 다음 시도 = retry_of 첫 시도

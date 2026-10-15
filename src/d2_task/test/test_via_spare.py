# -*- coding: utf-8 -*-
"""받침 경유 흩뿌림 집기(robot.yaml scatter.via_spare, 10/13 PL 10시 지시) 시험 — ROS 없이 TaskManager · ScatterFlow 를 가짜 IO 로 돌린다.

흐름: 공급 관측(find_blocks height_map) → 고르기(BlockPicker.rank) → 받침에 놓음(조립 아님 — 옆으로 선 · 선 블록은 그 자세로) → 받침 위 촬영 자세
→ find_blocks area 'spare' → 선 블록이면 잡기 TIP 으로 눕히고 다시 잼 → 반대 폭으로 집어 받침 가운데에 옮겨 놓기(10/14) → 받침 가운데에서 레시피 잡기로 다시 집어
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


def on_spare(up='THICKNESS', centered=False):
    """받침 위 블록 하나(find_blocks area 'spare' 꼴) — 받침 자리에서 x +1 · y −2 mm(2.2 mm) 비킴, centered 면 받침 자리 그대로."""
    size = {'THICKNESS': T, 'WIDTH': W, 'LENGTH': L}[up]
    dx, dy = (0.0, 0.0) if centered else (0.001, -0.002)
    b = blk(x=SPOT['x_m'] + dx, y=SPOT['y_m'] + dy, top=SPOT['z_m'] + size, up=up, length=99, width=99, thickness=99)
    b.update(yaw_deg=-89.0, base_z_m=SPOT['z_m'], confidence=1.0)
    return {'ok': True, 'blocks': [b]}


class SpareIO(ScatterIO):
    """find_blocks 선택 칸(height_map · area)을 받아 기록하고, area 'spare' 면 받침에 블록이 있을 때 spare_script(없으면 받침 위 눕힌 블록),
    없으면 빈 목록을 돌려준다. 받침에 블록이 있나(spare_has)는 집기가 성공할 때 따라간다 — 받침에 옮기기면 생기고, 받침에서 조립으로 집으면 없어진다
    받침 맞추기(spare_center)가 성공하면 그 뒤 받침 블록은 받침 자리 그대로로 잰다(10/14 — 맞춘 뒤 다시 재기)."""

    def __init__(self):
        super().__init__()
        self.opts, self.spare_script, self.spare_has, self.spare_centered = [], [], False, False

    def find_blocks(self, run_id, should_abort, **opts):
        self.opts.append(dict(opts))
        self.events.append(('call', 'find_blocks', run_id))
        if opts.get('area') == 'spare':
            if not self.spare_has:               # 받침에 아무것도 안 놓았다(출발 때 확인) — 준비한 답(spare_script)은 받침에 놓은 뒤에 쓴다
                return (True, '', {'ok': True, 'blocks': []})
            return (True, '', self.spare_script.pop(0) if self.spare_script else on_spare(centered=self.spare_centered))
        step = self.find_script.pop(0) if self.find_script else (True, '', supply())
        return step(self) if callable(step) else step

    def pick_place(self, goal, should_abort, on_feedback=None, on_result=None):
        res = ScatterIO.pick_place(self, goal, should_abort, on_feedback, on_result)
        if res[0] and goal.get('via_spare'):
            self.spare_has, self.spare_centered = True, False
        elif res[0] and goal.get('spare_center'):
            self.spare_centered = True
        elif res[0] and goal.get('from_spare') and not (goal.get('spare_stand') or goal.get('spare_tip') or goal.get('spare_center')):
            self.spare_has = False
        return res


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
    for c, g in zip(center, from_spare):                                          # 잰 자리에서 반대 폭으로 집어 받침 가운데에 놓고, 다시 재서 그 자리에서 집는다
        assert c.get('spare_center') and c['grasp'] == other[g['grasp']]
        assert abs(c['pick_pose'][0][0] - (SPOT['x_m'] + 0.001)) < 1e-9 and abs(c['pick_pose'][0][1] - (SPOT['y_m'] - 0.002)) < 1e-9
        assert abs(c['place_pose'][0][0] - SPOT['x_m']) < 1e-9 and abs(c['place_pose'][0][1] - SPOT['y_m']) < 1e-9
    for g in to_spare:                                                           # 받침 윗면(80 mm) 위에 눕힘
        assert abs(g['place_pose'][0][2] - (SPOT['z_m'] + T / 2)) < 1e-9
        assert abs(g['place_pose'][0][0] - SPOT['x_m']) < 1e-9 and abs(g['place_pose'][0][1] - SPOT['y_m']) < 1e-9
    for g in from_spare:                                                         # 받침 가운데 · 받침 높이로 집고, 장애물 없음
        assert abs(g['pick_pose'][0][0] - SPOT['x_m']) < 1e-9 and g['obstacles'] == []
        assert abs(g['pick_pose'][0][2] - (SPOT['z_m'] + T / 2)) < 1e-9
    targets = [c[1] for c in calls(io, 'move_to')]
    assert targets.count('observe_spare') == 2 * len(IDS)                         # 블록마다 받침에 놓은 뒤 + 맞춘 뒤 다시
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
    assert [c[1] for c in calls(io, 'move_to')].count('observe_spare') == 2 * len(IDS) + 1     # 블록마다 2번(놓은 뒤 · 맞춘 뒤) + 눕힌 뒤 한 번 더


def test_기울어진_블록은_윗면_법선으로_두께_절반만_물고_받침에_그만큼_올려_놓는다():
    m, io = make()
    flow = m._scatter
    top = SPARE_CFG['table_z_m'] + T + 0.012
    tilted = blk(x=0.40, y=0.22, top=top + 0.01, tilted=True, overlap='top', length=60, width=60,
                 center_top_z_m=top, normal=[-0.2588, 0.0, 0.9659])
    target = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'place_pose': ((0.4, -0.07, 0.01), (0, 0, 0, 1))}
    assert flow.prepare_spare(target, {'ok': True, 'blocks': [tilted]})['status'] == 'NO_MATCH'      # 깊이 없으면 기울여 집기 안 함
    assert flow.prepare_spare(target, {'ok': True, 'blocks': [tilted], 'height_map': EMPTY_HM})['status'] == 'NO_MATCH'   # 점 0개도 깊이 없음(10/14 점검)
    floor = dict(EMPTY_HM, z_m=[SPARE_CFG['table_z_m']])                      # 블록에서 먼 작업대 점 하나 — 깊이는 받았고 막는 점은 없음
    r = flow.prepare_spare(target, {'ok': True, 'blocks': [tilted], 'height_map': floor})
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
    """W131(10/13): 받침 위 눕힌 블록 → 잰 자리에서 FLAT_LONG 으로 집어 받침 자리 가운데에 놓음(10/14) → 다시 재서 2 mm 안이면 STANDUP_HIGH
    (받침 자리 눕힘 → 세움). 옆세움(EDGE)은 FLAT_SHORT → STANDUP_SIDE. 공급에서 받침으로 옮기는 1단계도 막지 않는다. 맞추기를 건너뛸 길(skip)은 없다."""
    from d2_motion.motion_math import matrix_from_quat, up_axis
    m, io = make()
    for grasp, center, stand, up in (('STAND_SHORT', 'FLAT_LONG', 'STANDUP_HIGH', 'LENGTH'),
                                     ('EDGE_SHORT', 'FLAT_SHORT', 'STANDUP_SIDE', 'WIDTH')):
        target = {'part_id': IDS[0], 'grasp': grasp, 'place_pose': ((0.4, -0.07, 0.04), (0, 0, 0, 1))}
        assert m._scatter.prepare_spare(target, supply())['status'] == 'SPARE'
        r = m._scatter.prepare_from_spare(target, on_spare())                    # 처음(맞추기 0번) — 맞추기
        assert r['status'] == 'SPARE_CENTER' and 'offset_mm' not in r
        c = r['request']
        assert c['grasp'] == center and c.get('spare_center') and not c.get('spare_stand') and c.get('skip') is None
        assert abs(c['pick_pose'][0][0] - (SPOT['x_m'] + 0.001)) < 1e-9 and abs(c['pick_pose'][0][1] - (SPOT['y_m'] - 0.002)) < 1e-9   # 잰 자리에서 집음
        assert abs(c['place_pose'][0][0] - SPOT['x_m']) < 1e-9 and abs(c['place_pose'][0][1] - SPOT['y_m']) < 1e-9
        r = m._scatter.prepare_from_spare(target, on_spare(centered=True), centers=1)   # 맞춘 뒤 다시 잼 — 받침 자리 그대로 → 세움
        assert r['status'] == 'SPARE_STAND' and r['offset_mm'] == 0.0
        t = r['request']
        assert t['grasp'] == stand and t.get('spare_stand') and not t.get('spare_center')
        for pose in (t['pick_pose'], t['place_pose']):                              # 받침 자리(교시 좌표)에서 세운다
            assert abs(pose[0][0] - SPOT['x_m']) < 1e-9 and abs(pose[0][1] - SPOT['y_m']) < 1e-9
        assert up_axis(matrix_from_quat(t['pick_pose'][1])) == 'THICKNESS' and up_axis(matrix_from_quat(t['place_pose'][1])) == up
        assert abs(t['place_pose'][0][2] - (SPOT['z_m'] + {'LENGTH': L, 'WIDTH': W}[up] / 2)) < 1e-9


def test_맞춘_뒤_다시_재서_2mm_넘으면_한_번_더_맞추고_두_번이면_사람_없이_이어_간다():
    """10/14 박진용: 맞춘 뒤 받침 자리에서 2 mm(CENTER_TOL_M) 넘으면 다시 맞춘다 — 블록당 CENTER_MAX(2)번까지, 넘겨도 사람을 부르지 않는다."""
    from d2_task.scatter_pick import CENTER_MAX
    m, io = make()
    off = on_spare()                                                              # 2.2 mm 비킴
    for grasp, after in (('FLAT_SHORT', 'PICK'), ('STAND_SHORT', 'SPARE_STAND')):
        target = {'part_id': IDS[0], 'grasp': grasp, 'place_pose': ((0.4, -0.07, 0.04), (0, 0, 0, 1))}
        r = m._scatter.prepare_from_spare(target, off, centers=1)
        assert r['status'] == 'SPARE_CENTER' and r['offset_mm'] == 2.2              # 1번 맞춘 뒤 2.2 mm → 한 번 더
        r = m._scatter.prepare_from_spare(target, off, centers=CENTER_MAX)
        assert r['status'] == after and r['offset_mm'] == 2.2                       # 2번 다 했으면 그대로 이어 감(사람 없음)
    target = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'place_pose': ((0.4, -0.07, 0.04), (0, 0, 0, 1))}
    r = m._scatter.prepare_from_spare(target, off, centers=CENTER_MAX)
    assert abs(r['request']['pick_pose'][0][0] - (SPOT['x_m'] + 0.001)) < 1e-9      # 일반은 잰 자리에서 집는다


def test_받침_맞추기는_다시_재고_어긋남을_기록한다():
    """10/14: 맞춘 뒤 다시 잰 어긋남을 조립 기록에 남긴다 — 실기에서 왜 못 맞추는지 숫자로 보려고."""
    m, io = make()
    got, log = [], m.logger.log
    m.logger.log = lambda kind, part_id=None, value=None, module='task': (got.append((kind, value)), log(kind, part_id, value, module))
    start(m)
    drive(m, 'DONE')
    again = [v for k, v in got if k == 'spare' and '다시 잼' in str(v)]
    assert len(again) == len(IDS) and all(v.endswith('0.0 mm') for v in again)   # 가짜 카메라는 맞춘 뒤 받침 자리 그대로
    assert m._spare_centers == 0


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
    assert [g['grasp'] for g in io.goals[:3]] == ['FLAT_SHORT', 'FLAT_LONG', 'FLAT_SHORT']
    assert abs(io.goals[2]['pick_pose'][0][0] - (SPOT['x_m'] + 0.001)) < 1e-9     # 옮기지 못했으니 잰 자리 그대로 집는다(skip)
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
    m.run_once()                                                                  # 닫기는 작업 스레드가 잠금 밖에서(#152 리뷰 3)
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
    assert m.command('start') == (True, '') and m.state == 'CHECK'          # 출발은 받고, 기록은 CHECK 첫 단계에서 잠금 밖으로 연다(#152 리뷰 3)
    m.run_once()
    assert m.state == 'ERROR'                                                 # 움직이기 전에 오류(사람이 카메라 · TF 를 보고 다시)
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


# ---------- 10/14 #152 리뷰 고침 ----------
def test_받침에_놓는_중에_멈추면_받침_기록을_남겨_다음엔_받침부터_잰다():
    """#152 리뷰 1: 놓기(place) 단계 이후의 실패 · 정지면 실물이 받침에 있을 수 있다 — 기록을 남겨 같은 자리에 또 놓지 않는다."""
    m, io = make()
    m.goal = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'via_spare': True, 'attempt': (0.3, 0.2, 'FLAT_SHORT', 0, 0.0)}
    m._spare_on = None
    for step, kept in (('move', None), ('place', IDS[0]), ('retreat', IDS[0])):
        m._spare_on = None
        m.last_pick = {'request_id': 1, 'step': step, 'result_at': None}
        m._via_spare_result(False, 'STOPPED')
        assert m._spare_on == kept                                                # 옮기는 중(move)이면 블록은 손에 — 받침 기록 안 함


def test_출발_홈_이동은_앞_블록이_남은_장면으로_먼저_하고_그_뒤에_장면을_비운다():
    """#152 리뷰 2: 빈 진행표로 장면을 비운 뒤 움직이면 실물이 남았을 때 장애물이 빠진 장면으로 간다."""
    m, io = make()
    order = []
    io.publish_progress = lambda msg: order.append(('progress', len(msg['parts'])))
    move = io.move_to
    io.move_to = lambda target, abort, speed_ratio=1.0: (order.append(('move', target)), move(target, abort, speed_ratio))[1]
    start(m)
    m.run_once()
    assert order.index(('move', 'home')) < order.index(('progress', 0))


def test_출발_홈_이동이_막히면_장면을_비우지_않고_사람을_부른다():
    m, io = make()
    sent = []
    io.publish_progress = lambda msg: sent.append(msg)
    io.move_script = [(False, 'PLAN_FAILED')]
    start(m)
    drive(m, 'ERROR')
    assert not any(msg['parts'] == [] for msg in sent)


def test_헛잡음_뒤_공급_관측_이동이_실패하면_넘기지_않고_실패_처리한다():
    """#152 리뷰 4: observe_supply 이동이 TIMEOUT 이어도 다시 찍으러 갔다 — 이제 평소 실패 처리(TIMEOUT 은 정지 요청부터)."""
    m, io = make()
    m.run_id = 'R1'
    io.find_script = [(True, '', {'ok': True, 'blocks': []})]                    # 집기 위에서 찍었는데 그 블록이 안 보임 → 공급 관측으로
    io.move_script = [(False, 'TIMEOUT')]
    goal = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'via_spare': True, 'attempt': (0.3, 0.2, 'FLAT_SHORT', 0, 0.0),
            'seen': supply()['blocks'][0]}
    assert m._spare_regrasp(goal) is True
    assert ('stop', 'TIMEOUT') in io.calls and calls(io, 'find_blocks') == [('find_blocks', 'R1')]


def test_값이_틀린_블록이_있으면_고르기_전에_막고_예외가_안_난다():
    """#152 리뷰 6: up 이 없거나 모르는 이웃 블록이 있으면 순위 계산이 예외로 멈췄다 — BLOCKED(다시 찍기)로."""
    m, io = make()
    target = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'place_pose': ((0.4, -0.07, 0.04), (0, 0, 0, 1))}
    for bad in (None, 'SIDE'):
        resp = supply()
        resp['blocks'][1]['up'] = bad
        r = m._scatter.prepare_spare(target, resp)
        assert r['status'] == 'BLOCKED' and r['index'] == 1


class LockIO(W144IO):
    """W144 열기 · 닫기를 부를 때 작업 관리자 잠금을 다른 스레드가 잡을 수 있었나(= 잠금 밖인가)를 남긴다."""

    def _free(self):
        import threading
        got = []

        def probe():
            ok = self.manager._lock.acquire(timeout=0.5)
            got.append(ok)
            if ok:                                     # RLock 은 잡은 스레드가 푼다
                self.manager._lock.release()
        t = threading.Thread(target=probe)
        t.start()
        t.join()
        return bool(got and got[0])

    def w144_start(self, design_id, run_id):
        self.w144.append(('start_free', self._free()))
        return super().w144_start(design_id, run_id)

    def w144_finish(self, completed):
        self.w144.append(('finish_free', self._free()))
        super().w144_finish(completed)


def test_W144_녹화_열기_닫기는_작업_관리자_잠금_밖에서_한다():
    """#152 리뷰 3: 잠금 안에서 녹화 시작(토픽 확인 최대 15 s) · 닫기(rosbag · ZIP 30 s)를 기다리면 그동안 정지 신호 처리가 막혔다."""
    io = LockIO()
    m = TaskManager(SPARE_CFG, io, open_width_m=0.0)
    io.manager = m
    fast_wait(m)
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    start(m)
    drive(m, 'DONE')
    m.run_once()
    free = dict((e[0], e[1]) for e in io.w144 if e[0] in ('start_free', 'finish_free'))
    assert free == {'start_free': True, 'finish_free': True}


# ---------- 10/14 10시 전체 점검 ② 박진용 ----------
def test_사람_확인을_웹_끊김으로_떠나면_계속_뒤_할_일을_비운다():
    """점검 ② 2ⓐ: WAIT_SUPPLY(놓은 뒤 확인 대기 then='verify') → WAIT_HMI 로 떠나면 _hold_then 이 남아 다음 [계속]이 엉뚱하게 VERIFY 로 갔다."""
    m, io = make()
    m.state, m._hold_then = 'WAIT_SUPPLY', 'verify'
    m._set('WAIT_HMI', 'hmi_lost', '')
    assert m._hold_then is None
    m.state, m._hold_then = 'SELECT', None
    m._hold(IDS[0], '사람 확인', then='verify')                              # 들어갈 때는 남는다
    assert m.state == 'WAIT_SUPPLY' and m._hold_then == 'verify'


def test_웹_다시_붙은_뒤_재확인도_받침_경유면_엄격하게_본다():
    """점검 ② 2ⓐ: 웹 재연결 뒤 SELECT 재확인이 높이 기준(place_dz_m 5 mm) 없이 통과해 고치지 않은 블록 위로 진행했다."""
    m, io = make()
    start(m)
    m.run_once()                                                              # CHECK → SELECT
    io.world.add(IDS[0])
    io.dz = 0.010                                                             # 놓은 블록이 10 mm 떠 있음(5 mm 넘음)
    m.placed, m._hmi_recheck, m.state = IDS[0], True, 'SELECT'
    m.run_once()
    assert m.state == 'WAIT_SUPPLY' and m._hold_then == 'verify' and not m._hmi_recheck


def test_헛잡음_뒤_다시_찍기_조회가_실패하면_실패_처리한다():
    """점검 ② 2ⓑ: find_blocks 가 TIMEOUT 이어도 정지 요청 없이 SELECT 로 갔다."""
    m, io = make()
    m.run_id = 'R1'
    io.find_script = [(False, 'TIMEOUT', None)]
    goal = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'via_spare': True, 'attempt': (0.3, 0.2, 'FLAT_SHORT', 0, 0.0),
            'seen': supply()['blocks'][0]}
    assert m._spare_regrasp(goal) is True
    assert ('stop', 'TIMEOUT') in io.calls


def test_헛잡음_뒤_다시_잡기는_새로_찍은_관측을_다음_후보의_바탕으로_남긴다():
    """점검 ② 2ⓒ: 다시 잡기가 움직이기 전에 거절되면 밀리기 전 옛 관측(_spare_seen)에서 다음 후보를 골라 빈 자리로 갔다."""
    m, io = make()
    m.run_id = 'R1'
    old = supply()
    new = supply()
    new['blocks'][0]['x_m'] += 0.01                                          # 헛잡음으로 10 mm 밀림
    m._spare_seen = ({'part_id': IDS[0]}, old)
    io.find_script = [(True, '', new)]
    goal = {'part_id': IDS[0], 'grasp': 'FLAT_SHORT', 'via_spare': True, 'attempt': (0.30, 0.20, 'FLAT_SHORT', 0, 0.0),
            'seen': old['blocks'][0], 'place_pose': ((0.4, -0.07, 0.04), (0, 0, 0, 1))}
    assert m._spare_regrasp(goal) is True and m.state == 'PICK_PLACE'
    assert m._spare_seen[1] is new


def test_W144_기록_시작이_실패하면_덜_된_기록을_닫는다():
    """점검 ② 3: 시작 실패 뒤 [다시 시작]으로 이어 가면 반쯤 연 기록을 썼다 — 실패하면 닫기(ZIP)를 잡아 둔다."""
    m, io = make_w144(fail_at='start')
    assert m.command('select_design', 'bench') == (True, '') and m.command('start') == (True, '')
    m.run_once()
    assert m.state == 'ERROR'
    m.run_once()
    assert ('finish', False) in io.w144


def test_조립_중이_아닐_때_정지가_풀려도_장면은_그대로_둔다():
    """10/14 #174 리뷰 ①: 완성 · 대기에서 정지 → 다시 시작은 실물 블록이 남았을 수 있어 빈 진행표를 보내지 않는다
    (장면은 다음 [조립] 출발 때 _clean_start 가 비운다 — 조립 끝 DONE 에도 그대로, 10/14 PL)."""
    m, io = make()
    io.progress.clear()
    m.state, m._assembling = 'RECOVER', False
    m._recover()
    assert m.state == 'IDLE'
    assert io.progress == []


def test_초기화_뒤_정리_끝이면_빈_진행표로_장면을_비운다():
    """10/14 박진용: [초기화](STOPPED 의 cancel = 사람이 치웠다는 확인) → [정리 끝](다시 시작)이면 장면의 blk_* 를 비운다."""
    m, io = make()
    io.progress.clear()
    m.state, m._assembling = 'STOPPED', True
    assert m._cleanup_stopped() == (True, '')
    m.state = 'RECOVER'
    m._recover()
    assert m.state == 'IDLE'
    assert io.progress and io.progress[-1]['parts'] == [] and io.progress[-1]['schema'] == 'progress/2.0'


def test_조립_중_정지면_W144_녹화를_닫고_다시_시작이면_새로_연다():
    """10/15 박진용: 정지로 멈춘 판은 녹화가 안 닫혀 원본(카메라 30 Hz)이 계속 쌓였다(10/14 V008 24 GB · V009 57 GB).
    조립 중 정지 → 바로 닫음(완료 아님) · [다시 시작]으로 이어 가면 CHECK 첫 단계가 새로 연다 · 끝나면 다시 닫음."""
    from test_task_manager import SAFE_STOP
    m, io = make_w144()
    start(m)
    m.run_once()                                                              # CHECK — 녹화 엶
    assert [e[0] for e in io.w144 if e[0] in ('start', 'finish')] == ['start']
    m.on_safety(SAFE_STOP)
    m.run_once()                                                              # STOPPED — 닫기 예약
    m.run_once()                                                              # 잠금 밖에서 닫음
    assert m.state == 'STOPPED' and ('finish', False) in io.w144
    m.on_safety(SAFE_OK)
    drive(m, 'DONE')
    m.run_once()                                                              # 끝 — 잠금 밖에서 닫음
    kinds = [e[0] if e[0] == 'start' else e for e in io.w144 if e[0] in ('start', 'finish')]
    assert kinds == ['start', ('finish', False), 'start', ('finish', True)]


class StrictW144IO(W144IO):
    """촬영기처럼 녹화마다 시도 번호를 따로 안다 — 새로 열면 비우고, 모르는 번호(retry_of · 실패 · 촬영)는 거절한다."""

    def w144_start(self, design_id, run_id):
        self.known = set()
        return super().w144_start(design_id, run_id)

    def w144_begin(self, part_id, sequence, center_m, quat_xyzw, evidence, retry_of=None):
        if retry_of is not None and retry_of not in self.known:
            return None, 'ERROR'
        aid, why = super().w144_begin(part_id, sequence, center_m, quat_xyzw, evidence, retry_of)
        self.known.add(aid)
        return aid, why

    def w144_fail(self, action_id, part_id, sequence, reason, evidence):
        self.w144.append(('fail', action_id, part_id, reason) if action_id in self.known else ('fail_rejected', action_id))

    def w144_after_place(self, part_id, center_m, quat_xyzw, action_id=None, sequence=None, evidence=None):
        if action_id is not None and action_id not in self.known:
            return False, 'ERROR'
        return super().w144_after_place(part_id, center_m, quat_xyzw, action_id, sequence, evidence)


def test_집는_중_정지_뒤_다시_시작이면_새_녹화에_시도를_새로_연다():
    """10/15 002 실기: 정지로 닫고 [다시 시작]으로 새로 연 녹화에 옛 시도 번호를 그대로 써 'action_id 없음' → ERROR 를 되풀이했다.
    다시 열면 시도 기록을 비워 같은 블록도 새 시도로 begin 하고, 끝까지 간다."""
    from test_task_manager import SAFE_STOP
    io = StrictW144IO()
    m = TaskManager(SPARE_CFG, io, open_width_m=0.0)
    io.manager = m
    fast_wait(m)
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    # 첫 집기 도중 정지(펜던트 · 화면) — 집기 노드가 STOPPED 로 끝나면 그 시도가 실패로 닫히고 다음 시도의 retry_of 가 된다
    io.pick_script = [lambda io: (io.manager.on_safety(SAFE_STOP), (False, 'STOPPED'))[1]]
    start(m)
    drive(m, 'STOPPED')
    m.run_once()
    m.run_once()                                                              # 닫음
    assert ('finish', False) in io.w144
    m.on_safety(SAFE_OK)
    drive(m, ('DONE', 'ERROR'))
    assert m.state == 'DONE'
    starts = [i for i, e in enumerate(io.w144) if e[0] == 'start']
    again = {e[3] for e in io.w144[starts[1]:] if e[0] == 'begin'}
    shots = [e for e in io.w144[starts[1]:] if e[0] == 'shot']
    assert shots and all(s[4] in again for s in shots)                         # 다시 연 뒤 촬영은 모두 새 녹화의 시도 번호로
    assert not [e for e in io.w144 if e[0] == 'fail_rejected']


def test_조립_중이_아닐_때_정지는_W144_녹화를_건드리지_않는다():
    """대기 · 완성 뒤 정지는 열린 녹화가 없다 — 닫기 · 다시 열기를 예약하지 않는다."""
    from test_task_manager import SAFE_STOP
    m, io = make_w144()
    m.on_safety(SAFE_STOP)
    m.run_once()
    m.run_once()
    m.on_safety(SAFE_OK)
    m.run_once()
    m.run_once()
    assert not [e for e in io.w144 if e[0] in ('start', 'finish')] and not m._w144_reopen


class OpenGateIO(W144IO):
    """실제 task_node 처럼 열기에 성공한 녹화만 '열림'으로 보고(w144_finish 로 닫힘), w144_start 결과와 pick_place 를 일어난 순서대로 order 에 적는다.
    starts = 열기 결과 대기열(True 성공 · False 실패, 비면 성공). pick 줄의 셋째 값 = 그때 녹화가 열려 있었나."""

    def __init__(self, starts=()):
        super().__init__()
        self.starts, self.order, self.open = list(starts), [], False

    def w144_start(self, design_id, run_id):
        super().w144_start(design_id, run_id)
        self.open = self.starts.pop(0) if self.starts else True
        self.order.append(('start', self.open))
        return (True, '') if self.open else (False, 'W144 기록 준비 실패: 필수 토픽 없음')

    def w144_finish(self, completed):
        super().w144_finish(completed)
        self.open = False

    def pick_place(self, goal, should_abort, on_feedback=None, on_result=None):
        self.order.append(('pick', goal['part_id'], self.open))
        return super().pick_place(goal, should_abort, on_feedback, on_result)


def make_gate(starts=(), clock=None):
    """make_w144 와 같은 준비. clock 을 주면 웹 연결 신호를 감시한다(monitor_hmi)."""
    io = OpenGateIO(starts)
    extra = {'clock': clock, 'monitor_hmi': True} if clock else {}
    m = TaskManager(SPARE_CFG, io, open_width_m=0.0, **extra)
    io.manager = m
    fast_wait(m)
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def picked_only_while_open(io):
    """집기가 있었고, 모두 녹화가 열린 뒤였나(완료 조건: 녹화 켜짐이면 열기 성공 전 pick_place 0번)."""
    picks_ = [e for e in io.order if e[0] == 'pick']
    return bool(picks_) and all(e[2] for e in picks_)


def test_W144_출발_직후_진행_확인_전에_웹이_끊겨도_계속하면_녹화를_연_뒤에_집는다():
    """10/15 재현: CHECK 첫 단계(녹화 열기) 전에 웹이 끊겨 WAIT_HMI → [계속]이 SELECT 로 가 녹화를 한 번도 안 열고 끝까지 조립했다."""
    now = [0.0]
    m, io = make_gate(clock=lambda: now[0])
    alive = [True]
    m.sleep_hook = lambda _n: alive[0] and m.on_hmi_alive({'alive': True})   # 붙어 있는 동안은 그리퍼 대기(가짜 시간)마다 연결 신호
    m.on_hmi_alive({'alive': True})
    start(m)
    alive[0] = False
    now[0] += SPARE_CFG['mqtt']['lost_after_s']
    m.run_once()
    assert m.state == 'WAIT_HMI' and io.order == []                          # 녹화 열기도 집기도 아직 없음
    alive[0] = True
    m.on_hmi_alive({'alive': True})
    assert m.command('start') == (True, '') and m.state == 'CHECK'          # 계속 → CHECK 부터(녹화를 먼저 연다)
    drive(m, ('DONE', 'ERROR'))
    assert m.state == 'DONE'
    assert io.order[0] == ('start', True) and picked_only_while_open(io)


def test_W144_녹화_열기가_실패한_뒤_다시_시작하면_다시_열고_또_실패하면_집지_않는다():
    """10/15 재현: 열기 실패 → ERROR → [다시 시작]이 다시 열지 않고 녹화 없이 끝까지 조립했다. 이제 다시 열고, 또 실패하면 ERROR(집기 0번)."""
    m, io = make_gate(starts=[False, False, True])
    start(m)
    m.run_once()
    assert m.state == 'ERROR' and io.order == [('start', False)]
    for expect in ('ERROR', 'DONE'):
        m.run_once()                                                          # 덜 된 기록 닫기(잠금 밖)
        m.on_safety(SAFE_OK)                                                  # 화면 [다시 시작] → 정지 노드가 safety/state 를 다시 냄
        m.run_once()
        assert m.state == 'RECOVER'
        drive(m, ('ERROR', 'DONE'))
        assert m.state == expect
        if expect == 'ERROR':                                                 # 두 번째 열기도 실패 — 움직이기 · 집기 전에 멈춤
            assert io.order == [('start', False), ('start', False)]
            assert io.goals == [] and calls(io, 'move_to') == [] and calls(io, 'find_blocks') == []
    assert [e for e in io.order if e[0] == 'start'] == [('start', False), ('start', False), ('start', True)]
    assert picked_only_while_open(io)


def test_W144_조립_중_정지_뒤_다시_시작해도_새_녹화를_연_뒤에만_집는다():
    """기존 정지 → 다시 시작(#185 · #188)도 같은 조건: 닫은 뒤의 집기는 새로 연 녹화가 열린 다음에만."""
    from test_task_manager import SAFE_STOP
    m, io = make_gate()
    io.pick_script = [lambda io: (io.manager.on_safety(SAFE_STOP), (False, 'STOPPED'))[1]]
    start(m)
    drive(m, 'STOPPED')
    m.run_once()
    m.run_once()                                                              # 닫음
    assert not io.open
    m.on_safety(SAFE_OK)
    drive(m, ('DONE', 'ERROR'))
    assert m.state == 'DONE'
    assert [e for e in io.order if e[0] == 'start'] == [('start', True), ('start', True)]
    assert picked_only_while_open(io)

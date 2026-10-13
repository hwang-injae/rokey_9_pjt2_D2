# -*- coding: utf-8 -*-
"""세우기 연결 시험 — 맞는 자세가 없을 때 눕힌 블록을 빈 자리(spare_spots)에 세우는 요청 (W130 · E-37 · E-77, 로봇 없음).

10/11 16:50 PL 확정만 시험한다: 세움 FLAT_SHORT · 옆세움 FLAT_LONG, spare_spots 는 robot.yaml, 목표당 최대 1회,
세운 뒤 재관측 · 후보 없으면 no_match_block, 세우기 성공은 조립 기록(placed · record_placed · VERIFY)에 남기지 않음.
실제 세우기 실행(W131)·빈 자리 둘 다 찬 경우의 전용 처리·손가락 접근 공간은 범위 밖이다.
"""
import copy
import math

import pytest

from d2_motion.motion_math import column, matrix_from_quat

from d2_task.block_picker import BlockPicker
from d2_task.scatter_pick import ScatterFlow, block_pose
from test_block_picker import CFG, MIN_GAP, blk
from d2_task.task_manager import TaskManager
from test_scatter_connect import SCATTER_CFG, ScatterIO, calls, start
from test_task_manager import SAFE_OK, SAFE_STOP, drive, picks, seen

STAND = {'part_id': '003_DESK_STAND_V000_LEG_001_01', 'grasp': 'STAND_SHORT', 'place_pose': ((0.43, -0.07, 0.04), (0, 0, 0, 1))}
EDGE = dict(STAND, grasp='EDGE_LONG')
FAR = dict(x=0.50, y=0.28)            # 빈 자리(설정 1번 · 2번)와 떨어진 눕힌 블록 자리


def lying(**kw):
    """눕힌 블록 — 두 방향 틈이 넉넉해 FLAT_SHORT · FLAT_LONG 어느 것으로도 집힌다."""
    return blk(**dict(FAR, length=40, width=40), **kw)


def flow(cfg=CFG):
    return ScatterFlow(cfg, BlockPicker(MIN_GAP), 'scatter', 0.0)


def response(*blocks):
    return {'ok': True, 'blocks': list(blocks)}


def test_세움_목표는_FLAT_SHORT로_집어_설정의_빈_자리에_세우는_요청이다():
    c = lying()
    r = flow().prepare(STAND, response(c), allow_reorient=True)
    assert r['status'] == 'REORIENT' and r['spot'] == 0
    req, spot = r['request'], CFG['spare_spots'][0]
    assert (req['part_id'], req['supply_slot'], req['grasp'], req['reorient']) == (STAND['part_id'], '', 'FLAT_SHORT', True)
    assert req['pick_pose'] == block_pose(CFG, 'THICKNESS', c['yaw_deg'], c['x_m'], c['y_m'], c['top_z_m'])
    # 바닥 = 빈 자리 z_m(받침 윗면, 없으면 작업면) — 10/13 새 받침 윗면 80 mm
    assert req['place_pose'] == block_pose(CFG, 'LENGTH', spot['yaw_deg'], spot['x_m'], spot['y_m'],
                                           spot.get('z_m', CFG['table_z_m']) + CFG['block_actual_m'][0])


def test_옆세움_목표는_FLAT_LONG으로_집어_폭이_위로_세운다():
    r = flow().prepare(EDGE, response(lying()), allow_reorient=True)
    spot = CFG['spare_spots'][0]
    assert r['status'] == 'REORIENT' and r['request']['grasp'] == 'FLAT_LONG'
    assert r['request']['place_pose'] == block_pose(CFG, 'WIDTH', spot['yaw_deg'], spot['x_m'], spot['y_m'],
                                                    spot.get('z_m', CFG['table_z_m']) + CFG['block_actual_m'][1])


@pytest.mark.parametrize('up, side', [('LENGTH', 1), ('WIDTH', 0)])
@pytest.mark.parametrize('yaw', [0.0, 30.0, -89.8, -90.0])
def test_세운_블록의_yaw는_확정_규약대로다(up, side, yaw):
    """10/11 확정: LENGTH 위 = WIDTH 방향, WIDTH 위 = LENGTH 방향이 base x 와 yaw(위에서 반시계 +, 180° 같음). 위 축은 base z."""
    _, q = block_pose(CFG, up, yaw, 0.3, 0.2, 0.1)
    rot = matrix_from_quat(q)
    v, z = column(rot, side), column(rot, 0 if up == 'LENGTH' else 1)        # 블록 축 0 = LENGTH, 1 = WIDTH
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    assert abs(abs(v[0] * c + v[1] * s) - 1) < 1e-9 and abs(v[2]) < 1e-9     # 180° 같음 → 부호는 보지 않는다
    assert abs(abs(z[2]) - 1) < 1e-9


def test_빈_자리는_robot_yaml_설정에서_읽는다():
    cfg = dict(copy.deepcopy(CFG), spare_spots=[{'x_m': 0.40, 'y_m': 0.15, 'yaw_deg': 30.0}])
    r = flow(cfg).prepare(STAND, response(lying()), allow_reorient=True)
    x, y, _ = r['request']['place_pose'][0]
    assert (x, y) == (0.40, 0.15)


def test_첫_빈_자리가_차_있으면_다음_자리에_세운다():
    # 빈 자리 두 곳을 이 시험에서 직접 준다 — robot.yaml spare_spots 는 10/13 새 받침 한 곳이 됐다(받침 수와 상관없이 '다음 자리' 규칙만 본다)
    cfg = dict(copy.deepcopy(CFG), spare_spots=[{'x_m': 0.30, 'y_m': 0.15, 'yaw_deg': 0.0}, {'x_m': 0.40, 'y_m': 0.15, 'yaw_deg': 30.0}])
    s1 = cfg['spare_spots'][0]
    occupied = blk(x=s1['x_m'], y=s1['y_m'], length=5, width=5)       # 틈이 좁아 집을 후보는 아니지만 자리를 차지한다
    r = flow(cfg).prepare(STAND, response(lying(), occupied), allow_reorient=True)
    assert r['status'] == 'REORIENT' and r['spot'] == 1


def test_허용하지_않거나_기울어진_블록이_있으면_세우지_않고_NO_MATCH():
    assert flow().prepare(STAND, response(lying()), allow_reorient=False)['status'] == 'NO_MATCH'
    assert flow().prepare(STAND, response(lying(), blk(x=0.20, y=0.30, tilted=True)), allow_reorient=True)['status'] == 'NO_MATCH'


def test_세우기_요청도_commit_뒤_같은_요청으로_다시_계획할_수_있다():
    f = flow()
    ticket = f.begin(1)
    r = f.prepare(STAND, response(lying()), allow_reorient=True)
    assert f.commit(ticket, lambda: (1, False), r) and f.reusable_request()['reorient'] is True


# ---------------- TaskManager 연결 (임시 시험 전제: 빈 자리 = 외곽 겹침만 · spare_spots 임시 좌표 — 운영은 allow_reorient 기본 False) ----------------
def make(allow_reorient=True):
    io = ScatterIO()
    m = TaskManager(SCATTER_CFG, io, open_width_m=0.0865, allow_reorient=allow_reorient)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def stand_run(find_script, allow_reorient=True):
    """첫 목표를 세움 잡기로 바꾼 흩뿌림 조립. find_blocks 답은 find_script 순서대로."""
    m, io = make(allow_reorient)
    io.find_script = list(find_script)
    start(m)
    m.planner.parts[0]['grasp'] = 'STAND_SHORT'
    return m, io


STANDING = (True, '', response(blk(x=0.50, y=0.28, up='LENGTH', width=40, thickness=40)))
LYING_ONLY = (True, '', response(lying()))


def test_세우기_성공은_조립_기록_없이_재관측으로_이어진다():
    m, io = stand_run([LYING_ONLY, STANDING])
    drive(m, 'PICK_PLACE')
    assert m.goal['reorient'] is True and m.goal['grasp'] == 'FLAT_SHORT'
    m.run_once()                                                         # 세우기 성공
    assert m.state == 'SELECT' and m.placed is None and 'VERIFY' not in seen(io)
    drive(m, 'VERIFY')                                                   # 다시 보고 방금 세운 블록을 일반 집기로
    assert len(calls(io, 'find_blocks')) == 2 and m.placed == m.planner.parts[0]['part_id']
    assert m.goal.get('reorient') is None and m.goal['grasp'] == 'STAND_SHORT'
    assert [p[0] for p in picks(io)] == [m.planner.parts[0]['part_id']] * 2      # 세우기도 같은 목표 part_id


def test_세운_뒤에도_후보가_없으면_두_번째_세우기_없이_no_match_block():
    m, io = stand_run([LYING_ONLY, LYING_ONLY])
    drive(m, 'WAIT_SUPPLY')
    assert len(picks(io)) == 1 and io.states[-1]['message_id'] == 'no_match_block'


def test_세우기_집기가_실패하면_기존_흩뿌림_실패_처리로_ERROR():
    m, io = stand_run([LYING_ONLY])
    io.pick_script = [(False, 'GRASP_FAILED')]
    drive(m, 'ERROR')
    assert len(picks(io)) == 1 and m.placed is None


def test_조회_중_정지가_끼면_세우기_요청을_적용하지_않는다():
    m, io = make()

    def stop_then_answer(io_):
        m.on_safety(SAFE_STOP)
        return LYING_ONLY

    io.find_script = [stop_then_answer]
    start(m)
    m.planner.parts[0]['grasp'] = 'STAND_SHORT'
    drive(m, 'STOPPED')
    assert not picks(io) and m.goal is None and not m._reoriented


def test_운영_기본값은_세우기를_만들지_않는다():
    """W131 두 grasp 지원 · W146 정식 좌표 · 손가락 공간 판정 전 — TaskManager 기본(task 노드가 쓰는 값)은 세우기 요청 없이 기존 no_match_block."""
    m, io = stand_run([LYING_ONLY], allow_reorient=False)
    drive(m, 'WAIT_SUPPLY')
    assert not picks(io) and io.states[-1]['message_id'] == 'no_match_block' and not m._reoriented


def test_task_노드는_세우기_허용을_넘기지_않는다():
    import ast
    from pathlib import Path
    tree = ast.parse((Path(__file__).parents[1] / 'd2_task/task_node.py').read_text(encoding='utf-8'))
    call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'TaskManager')
    assert 'allow_reorient' not in {k.arg for k in call.keywords}


BAD_SPOTS_CFG = dict(copy.deepcopy(SCATTER_CFG), spare_spots=[{'x_m': 'abc', 'y_m': 0.2, 'yaw_deg': 0.0}])


def test_잘못된_spare_spots는_세우기가_꺼져_있으면_일반_집기를_막지_않는다():
    io = ScatterIO()
    m = TaskManager(BAD_SPOTS_CFG, io, open_width_m=0.0865)               # 운영 기본(allow_reorient False) — 만들 때 실패하지 않는다
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    start(m)
    drive(m, 'DONE')
    assert m.last_build['placed'] == len(m.planner.parts)


def test_잘못된_spare_spots는_세우려는_순간_요청_없이_ERROR():
    io = ScatterIO()
    m = TaskManager(BAD_SPOTS_CFG, io, open_width_m=0.0865, allow_reorient=True)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    io.find_script = [LYING_ONLY]
    start(m)
    m.planner.parts[0]['grasp'] = 'STAND_SHORT'
    drive(m, 'ERROR')
    assert not picks(io) and 'spare_spots' in io.states[-1]['message']

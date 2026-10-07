# -*- coding: utf-8 -*-
"""흩뿌림 집기 준비 ScatterFlow 시험 (W130) — 가짜 find_blocks 응답으로 '목표 유지 → 후보 선택 → 집기 요청 준비'까지. 로봇 호출 없음."""
import copy
import math

import pytest

from d2_motion.motion_math import column, quat_from_axes, slot_block_pose
from d2_task.block_picker import BlockPicker
from d2_task.scatter_pick import ScatterFlow, block_pose, failure_action
from test_block_picker import CFG, MIN_GAP, blk

TARGET = {'block_id': '001_CHAIR_BENCH_B004', 'grasp': 'FLAT_LONG',
          'place_pose': ((0.43, -0.07, -0.0035), (0.0, 0.0, 0.0, 1.0))}


def flow(**kw):
    kw.setdefault('supply_mode', 'scatter')
    kw.setdefault('open_width_m', 0.0865)         # 시험에서 명시 주입한 값(확정값 아님)
    return ScatterFlow(CFG, BlockPicker(MIN_GAP), **kw)


def resp(*blocks, ok=True):
    return {'ok': ok, 'blocks': list(blocks)}


def test_FOUND는_목표_block_id_놓을_자세_잡기를_유지한다():
    f = flow()
    target = copy.deepcopy(TARGET)
    r = f.prepare(target, resp(blk(length=30)))
    req = r['request']
    assert r['status'] == 'PICK'
    assert (req['block_id'], req['grasp'], req['supply_slot']) == (TARGET['block_id'], 'FLAT_LONG', '')
    assert req['place_pose'] == TARGET['place_pose'] and req['open_width_m'] == 0.0865
    assert target == TARGET                                        # 목표 입력은 바뀌지 않는다


def test_선택한_후보와_요청은_복사본():
    f = flow()
    r = f.prepare(TARGET, resp(blk(length=30)))
    r['candidate']['gap_mm']['LENGTH'] = -1
    r['request']['place_pose'] = None
    again = f.reusable_request()
    assert again['place_pose'] == TARGET['place_pose'] and f.candidate['candidate']['gap_mm']['LENGTH'] == 30
    again['block_id'] = '다른'
    assert f.reusable_request()['block_id'] == TARGET['block_id']


def test_후보가_여럿이면_BlockPicker_규칙대로_틈이_넓은_것():
    r = flow().prepare(TARGET, resp(blk(x=0.3, length=25), blk(x=0.5, length=40)))
    assert r['status'] == 'PICK' and r['request']['pick_pose'][0][0] == 0.5 and r['gap_mm'] == 40


def test_EMPTY는_채우기_안내이고_요청을_만들지_않는다():
    f = flow()
    r = f.prepare(TARGET, resp())
    assert r == {'status': 'EMPTY', 'guide': 'REFILL'} and f.candidate is None


def test_NO_MATCH는_제외_이유를_안내하고_요청을_만들지_않는다():
    f = flow()
    r = f.prepare(TARGET, resp(blk(up='WIDTH', length=90), blk(overlap='under'), blk(tilted=True), blk(length=10)))
    assert r['status'] == 'NO_MATCH' and r['guide'] == 'CHECK_BLOCKS' and 'request' not in r and f.candidate is None
    assert r['counts']['other_up'] == 1 and r['counts']['under'] == 1 and r['counts']['tilted'] == 1 and r['counts']['no_clear'] == 1
    for text in ('다른 블록에 덮임 1개', '기울어짐 1개', '필요한 자세가 아님 1개', '손가락 틈 부족 1개'):
        assert text in r['message']


def test_EMPTY와_NO_MATCH는_서로_다른_안내():
    f = flow()
    assert f.prepare(TARGET, resp())['guide'] != f.prepare(TARGET, resp(blk(length=1)))['guide']


@pytest.mark.parametrize('bad', [None, [], 'x', {}, {'ok': False, 'blocks': []}, {'ok': 'true', 'blocks': []}, {'ok': True},
                                 {'ok': True, 'blocks': None}, {'ok': True, 'blocks': 'x'}, {'ok': True, 'blocks': {}}])
def test_조회_실패나_잘못된_응답은_공급_부족이_아니다(bad):
    r = flow().prepare(TARGET, bad)
    assert r == {'status': 'LOOKUP_FAILED', 'reason': 'BAD_RESPONSE'}


@pytest.mark.parametrize('kw', [dict(supply_mode=None), dict(supply_mode='slots'), dict(open_width_m=None), dict(supply_mode='SCATTER')])
def test_모드나_열림_폭이_정해지지_않으면_실행_가능한_요청을_만들지_않는다(kw):
    f = flow(**kw)
    r = f.prepare(TARGET, resp(blk(length=30)))
    assert not f.configured and r == {'status': 'NOT_CONFIGURED'} and f.candidate is None


@pytest.mark.parametrize('width', [-0.01, float('nan'), float('inf'), 'x', True])
def test_열림_폭이_이상하면_설정_오류(width):
    with pytest.raises(ValueError):
        flow(open_width_m=width)


def test_BlockPicker가_아니면_설정_오류():
    with pytest.raises(ValueError):
        ScatterFlow(CFG, object())


# ---------- 늦은 답 ----------
def test_정지_종료_새_요청_상태_변화_뒤_도착한_이전_응답은_적용하지_않는다():
    f = flow()
    t1 = f.begin(epoch=5)
    assert f.is_current(t1, epoch=5, halted=False)
    assert not f.is_current(t1, epoch=5, halted=True)              # 정지 · 종료 중
    assert not f.is_current(t1, epoch=6, halted=False)             # 상태가 바뀜
    t2 = f.begin(epoch=5)                                          # 더 새 요청
    assert not f.is_current(t1, epoch=5, halted=False) and f.is_current(t2, epoch=5, halted=False)


def test_새_요청을_시작하면_이전_후보를_버린다():
    f = flow()
    f.prepare(TARGET, resp(blk(length=30)))
    assert f.reusable_request() is not None
    f.begin(epoch=1)
    assert f.reusable_request() is None


# ---------- 흩뿌림 대기 후 start ----------
def test_흩뿌림_대기_뒤_start는_재관측하고_공급_칸을_초기화하지_않는다():
    f = flow()
    f.prepare(TARGET, resp(blk(length=30)))
    r = f.after_wait_start()
    assert r == {'reobserve': True, 'reset_supply_slots': False} and f.reusable_request() is None


# ---------- 실패 뒤 재관측 / 재계획 ----------
@pytest.mark.parametrize('reason, action', [
    ('PLAN_FAILED', 'REPLAN_SAME'), ('BUSY', 'WAIT'),
    ('GRASP_FAILED', 'REOBSERVE'), ('SLOT_EMPTY', 'REOBSERVE'), ('TIMEOUT', 'REOBSERVE'), ('STOPPED', 'REOBSERVE'),
    ('CANCELED', 'REOBSERVE'), ('NO_FEEDBACK', 'REOBSERVE'), ('LOOKUP_FAILED', 'REOBSERVE'),
    ('ERROR', 'ERROR'), ('GRIPPER_NO_RESPONSE', 'ERROR'), ('이상한_이유', 'ERROR'), ('', 'ERROR'), (None, 'ERROR')])
def test_실패_이유별_다음_동작(reason, action):
    assert failure_action(reason) == action


@pytest.mark.parametrize('reason', ['PLAN_FAILED', 'BUSY'])
def test_계획만_실패했으면_같은_관측의_후보를_다시_쓴다(reason):
    f = flow()
    first = f.prepare(TARGET, resp(blk(length=30)))['request']
    f.on_failure(reason)
    assert f.reusable_request() == first


@pytest.mark.parametrize('reason', ['GRASP_FAILED', 'SLOT_EMPTY', 'TIMEOUT', 'STOPPED', 'CANCELED', 'NO_FEEDBACK', 'LOOKUP_FAILED', 'ERROR', '모름'])
def test_블록이_움직였을_수_있으면_이전_후보를_다시_쓰지_않는다(reason):
    f = flow()
    f.prepare(TARGET, resp(blk(length=30)))
    f.on_failure(reason)
    assert f.reusable_request() is None


# ---------- 자세 계산: 공급 칸 자세 계산과 같은 규약 ----------
@pytest.mark.parametrize('slot', range(1, 7))
def test_관측_자세는_공급_칸_자세_계산과_같다(slot):
    center, rot = slot_block_pose(CFG, slot)
    st = CFG['supply_slots'][slot - 1]
    up = st.get('block_up', 'THICKNESS')
    top = center[2] + (CFG['block_actual_m'][{'THICKNESS': 2, 'WIDTH': 1, 'LENGTH': 0}[up]] / 2)
    got_center, got_quat = block_pose(CFG, up, st['yaw_deg'], center[0], center[1], top)
    want_quat = quat_from_axes(*(column(rot, k) for k in range(3)))
    assert got_center == pytest.approx(center, abs=1e-9)
    assert got_quat == pytest.approx(want_quat, abs=1e-9)


def test_높이는_윗면에서_위로_향한_치수의_절반을_뺀다():
    thick = CFG['block_actual_m'][2]
    c, _ = block_pose(CFG, 'THICKNESS', 0.0, 0.4, 0.2, -0.003)
    assert c[2] == pytest.approx(-0.003 - thick / 2)


def test_yaw를_돌리면_자세가_따라_돈다():
    _, q0 = block_pose(CFG, 'THICKNESS', 0.0, 0, 0, 0)
    _, q90 = block_pose(CFG, 'THICKNESS', 89.0, 0, 0, 0)
    assert q0 == pytest.approx((0, 0, 0, 1), abs=1e-9)
    assert q90[2] == pytest.approx(math.sin(math.radians(89) / 2), abs=1e-9)


def test_모르는_up은_오류():
    with pytest.raises(ValueError):
        block_pose(CFG, 'SIDE', 0, 0, 0, 0)


def test_두께_잡는_축의_목표는_틈_정보가_없으면_요청을_만들지_않는다():
    """EDGE_SHORT 목표 + THICKNESS 틈 칸이 없는 응답 → NO_MATCH(틈 정보 없음), 실행 요청 없음."""
    f = flow()
    r = f.prepare({'block_id': 'B', 'grasp': 'EDGE_SHORT', 'place_pose': TARGET['place_pose']}, resp(blk(up='WIDTH', length=80, width=80)))
    assert r['status'] == 'NO_MATCH' and r['counts']['no_gap_info'] == 1 and 'request' not in r

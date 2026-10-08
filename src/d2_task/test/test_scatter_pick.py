# -*- coding: utf-8 -*-
"""흩뿌림 집기 준비 ScatterFlow 시험 (W130) — 가짜 find_blocks 응답으로 '목표 유지 → 후보 선택 → 집기 요청 준비'까지. 로봇 호출 없음."""
import copy
import math
import threading
import time

import pytest

from d2_motion.motion_math import column, quat_from_axes, slot_block_pose
from d2_task.block_picker import BlockPicker
from d2_task.scatter_pick import ScatterFlow, StepTracker, block_pose, contact_phase, failure_action, grasped_now
from test_block_picker import CFG, MIN_GAP, blk

TARGET = {'block_id': '001_CHAIR_BENCH_B004', 'grasp': 'FLAT_LONG',
          'place_pose': ((0.43, -0.07, -0.0035), (0.0, 0.0, 0.0, 1.0))}


def flow(**kw):
    kw.setdefault('supply_mode', 'scatter')
    kw.setdefault('open_width_m', 0.0865)         # 시험에서 명시 주입한 값(확정값 아님)
    return ScatterFlow(CFG, BlockPicker(MIN_GAP), **kw)


def run(f, target, response, epoch=1):
    """begin → prepare(계산) → commit(적용)까지 한 번에. 반환: prepare 결과."""
    ticket = f.begin(epoch)
    r = f.prepare(target, response)
    assert f.commit(ticket, lambda: (epoch, False), r)
    return r


def resp(*blocks, ok=True):
    return {'ok': ok, 'blocks': list(blocks)}


def test_FOUND는_목표_block_id_놓을_자세_잡기를_유지한다():
    f = flow()
    target = copy.deepcopy(TARGET)
    r = run(f, target, resp(blk(length=30)))
    req = r['request']
    assert r['status'] == 'PICK'
    assert (req['block_id'], req['grasp'], req['supply_slot']) == (TARGET['block_id'], 'FLAT_LONG', '')
    assert req['place_pose'] == TARGET['place_pose'] and req['open_width_m'] == 0.0865
    assert target == TARGET                                        # 목표 입력은 바뀌지 않는다


def test_선택한_후보와_요청은_복사본():
    f = flow()
    r = run(f, TARGET, resp(blk(length=30)))
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
    r = run(f, TARGET, resp())
    assert r == {'status': 'EMPTY', 'guide': 'REFILL'} and f.candidate is None


def test_NO_MATCH는_제외_이유를_안내하고_요청을_만들지_않는다():
    f = flow()
    r = run(f, TARGET, resp(blk(up='WIDTH', length=90), blk(overlap='under'), blk(tilted=True), blk(length=10)))
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
    run(f, TARGET, resp(blk(length=30)))
    assert f.reusable_request() is not None
    f.begin(epoch=1)
    assert f.reusable_request() is None


# ---------- 흩뿌림 대기 후 start ----------
def test_흩뿌림_대기_뒤_start는_재관측하고_공급_칸을_초기화하지_않는다():
    f = flow()
    run(f, TARGET, resp(blk(length=30)))
    r = f.after_wait_start()
    assert r == {'reobserve': True, 'reset_supply_slots': False} and f.reusable_request() is None


# ---------- 실패 뒤: 이유만으로 정하지 않는다 ----------
PRE = dict(phase='PRE_CONTACT', grasped=False)


@pytest.mark.parametrize('args, action', [
    (('PLAN_FAILED',), 'RECOVER'),                                          # 단계를 모르면 복구 절차
    (('PLAN_FAILED', 'LIFT'), 'RECOVER'),                                   # 들어 올리기 · 운반 · 놓기 단계
    (('PLAN_FAILED', 'PRE_CONTACT', None), 'RECOVER'),                      # 잡힘 상태를 모르면
    (('PLAN_FAILED', 'PRE_CONTACT', True), 'RECOVER'),                      # 이미 쥐었다
    (('PLAN_FAILED', 'PRE_CONTACT', False), 'REPLAN_SAME'),                 # 접촉 전이 확인된 때만 재사용
    (('BUSY', None, False), 'REOBSERVE'),                                   # 공급 영역이 그대로인지 모르면 후보를 안 쓴다
    (('BUSY', None, False, False), 'REOBSERVE'),
    (('BUSY', None, False, True), 'WAIT'),                                  # 안 바뀌었음이 확인된 때만 유지
    (('BUSY', None, None, True), 'RECOVER'),
    (('GRASP_FAILED', None, False), 'REOBSERVE'),                           # 손에 없음이 확인되면 재관측
    (('GRASP_FAILED', None, True), 'RECOVER'),
    (('GRASP_FAILED',), 'RECOVER'),
    (('SLOT_EMPTY', None, False), 'REOBSERVE'),
    (('SLOT_EMPTY', None, None), 'RECOVER'),
    (('TIMEOUT', 'PRE_CONTACT', False), 'RECOVER'),                         # 중간에 멈춘 것은 늘 기존 정지 · 복구
    (('STOPPED', 'PRE_CONTACT', False), 'RECOVER'),
    (('CANCELED', 'PRE_CONTACT', False), 'RECOVER'),
    (('NO_FEEDBACK', 'PRE_CONTACT', False), 'RECOVER'),
    (('LOOKUP_FAILED',), 'REOBSERVE'),                                      # 조회만 실패 — 로봇은 안 움직임
    (('ERROR', 'PRE_CONTACT', False), 'ERROR'),
    (('GRIPPER_NO_RESPONSE', 'PRE_CONTACT', False), 'ERROR'),
    (('이상한_이유', 'PRE_CONTACT', False), 'ERROR'),
    (('', 'PRE_CONTACT', False), 'ERROR'),
    ((None, 'PRE_CONTACT', False), 'ERROR'),
    (('이상한_이유',), 'ERROR'),
])
def test_실패_이유_단계_잡힘_상태로_다음_동작을_정한다(args, action):
    assert failure_action(*args) == action


def test_접촉_전이_확인된_PLAN_FAILED만_후보를_다시_쓴다():
    f = flow()
    first = run(f, TARGET, resp(blk(length=30)))['request']
    assert f.on_failure('PLAN_FAILED', **PRE) == 'REPLAN_SAME' and f.reusable_request() == first
    assert f.on_failure('PLAN_FAILED', phase='LIFT', grasped=True) == 'RECOVER' and f.reusable_request() is None


def test_BUSY는_공급_영역이_그대로라는_확인이_있을_때만_후보를_유지():
    f = flow()
    first = run(f, TARGET, resp(blk(length=30)))['request']
    assert f.on_failure('BUSY', grasped=False, area_unchanged=True) == 'WAIT' and f.reusable_request() == first
    assert f.on_failure('BUSY', grasped=False) == 'REOBSERVE' and f.reusable_request() is None


@pytest.mark.parametrize('reason', ['GRASP_FAILED', 'SLOT_EMPTY', 'TIMEOUT', 'STOPPED', 'CANCELED', 'NO_FEEDBACK', 'LOOKUP_FAILED', 'ERROR', '모름'])
def test_블록이_움직였을_수_있으면_이전_후보를_다시_쓰지_않는다(reason):
    f = flow()
    run(f, TARGET, resp(blk(length=30)))
    f.on_failure(reason, **PRE)
    assert f.reusable_request() is None


# ---------- prepare 실패 · 전부 잘못된 응답 · 확인과 적용 ----------
@pytest.mark.parametrize('second', [
    lambda f: (f, resp()), lambda f: (f, resp(blk(length=1))), lambda f: (f, None), lambda f: (f, {'ok': True, 'blocks': [{}]}),
    lambda f: (ScatterFlow(CFG, BlockPicker(MIN_GAP)), resp(blk(length=30)))])
def test_prepare가_실패하면_이전_후보를_지운다(second):
    f = flow()
    run(f, TARGET, resp(blk(length=30)))
    assert f.reusable_request() is not None
    g, response = second(f)
    if g is not f:                                                 # NOT_CONFIGURED 는 설정이 다른 흐름이라 같은 객체 설정을 바꿔서 확인
        f.supply_mode = None
    run(f, TARGET, response)
    assert f.reusable_request() is None


@pytest.mark.parametrize('blocks', [[{}], [{}, {}], ['글자'], [None], [blk(yaw_deg=float('nan')), blk(x_m=1e999)]])
def test_쓸_수_있는_블록이_하나도_없는_응답은_조회_오류(blocks):
    r = flow().prepare(TARGET, resp(*blocks))
    assert r == {'status': 'LOOKUP_FAILED', 'reason': 'ALL_INVALID'}


def test_정상과_잘못된_후보가_섞이면_정상_후보에서_고른다():
    r = flow().prepare(TARGET, resp({}, blk(length=33), None))
    assert r['status'] == 'PICK' and r['gap_mm'] == 33


def test_잘못된_후보와_맞지_않는_정상_후보가_섞이면_NO_MATCH():
    r = flow().prepare(TARGET, resp({}, blk(length=5)))
    assert r['status'] == 'NO_MATCH'


def test_prepare는_계산만_하고_후보를_정하지_않는다():
    f = flow()
    f.prepare(TARGET, resp(blk(length=30)))
    assert f.reusable_request() is None


def test_확인과_적용_사이에_정지가_끼어들지_못한다():
    """TaskManager 잠금을 잡은 채 commit 하면, 그 사이 들어온 정지는 commit 이 끝난 뒤에야 반영되고 후보를 비운다."""
    f = flow()
    ticket = f.begin(epoch=1)
    result = f.prepare(TARGET, resp(blk(length=30)))
    manager_lock, state = threading.Lock(), {'halted': False}
    started = threading.Event()

    def slow_state():
        started.set()
        time.sleep(0.3)                                            # 확인을 읽은 뒤 적용하기 전에 정지가 도착하는 순간을 만든다
        return 1, state['halted']

    def committer():
        with manager_lock:
            assert f.commit(ticket, slow_state, result)

    def stopper():
        started.wait(5)
        with manager_lock:                                         # 정지 반영도 같은 잠금을 얻어야 한다
            state['halted'] = True
            f.invalidate()

    t1, t2 = threading.Thread(target=committer), threading.Thread(target=stopper)
    t1.start(); t2.start(); t1.join(5); t2.join(5)
    assert state['halted'] and f.reusable_request() is None        # 정지 뒤에 후보가 되살아나지 않는다


def test_commit은_정지_종료_새_요청_상태_변화이면_적용하지_않는다():
    f = flow()
    result = f.prepare(TARGET, resp(blk(length=30)))
    t = f.begin(epoch=2)
    assert not f.commit(t, lambda: (2, True), result)              # 정지
    assert not f.commit(t, lambda: (3, False), result)             # 상태 변화
    f.begin(epoch=2)
    assert not f.commit(t, lambda: (2, False), result)             # 더 새 요청
    assert f.reusable_request() is None
    t2 = f.begin(epoch=2)
    assert f.commit(t2, lambda: (2, False), result) and f.reusable_request() is not None


def test_commit이_PICK이_아니면_후보를_비운다():
    f = flow()
    run(f, TARGET, resp(blk(length=30)))
    t = f.begin(epoch=1)
    assert f.commit(t, lambda: (1, False), {'status': 'EMPTY', 'guide': 'REFILL'}) and f.reusable_request() is None


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


def test_옆세움_목표는_두께_틈이_빠진_응답이면_요청을_만들지_않는다():
    """EDGE_SHORT 목표 + 수평 축 THICKNESS 가 빠진 응답은 잘못된 값 → 요청 없음."""
    f = flow()
    bad = blk(up='WIDTH', length=80)
    del bad['gap_mm']['THICKNESS'], bad['clear']['THICKNESS']
    r = f.prepare({'block_id': 'B', 'grasp': 'EDGE_SHORT', 'place_pose': TARGET['place_pose']}, resp(bad))
    assert r['status'] != 'PICK' and 'request' not in r


def test_옆세움_목표는_두께_틈이_있으면_요청을_만든다():
    f = flow()
    r = f.prepare({'block_id': 'B', 'grasp': 'EDGE_SHORT', 'place_pose': TARGET['place_pose']}, resp(blk(up='WIDTH', length=80, thickness=30)))
    assert r['status'] == 'PICK' and 'request' in r


# ---------- 접촉 전후와 그리퍼 상태 (박진용 확인) ----------
@pytest.mark.parametrize('step, reason, phase', [
    ('approach', 'PLAN_FAILED', 'PRE_CONTACT'),
    ('approach', 'STOPPED', 'CONTACT_UNKNOWN'), ('approach', 'CANCELED', 'CONTACT_UNKNOWN'), ('approach', 'TIMEOUT', 'CONTACT_UNKNOWN'),
    ('approach', 'GRASP_FAILED', 'CONTACT_UNKNOWN'),
    ('grasp', 'PLAN_FAILED', 'POST_CONTACT'), ('lift', 'PLAN_FAILED', 'POST_CONTACT'), ('move', 'PLAN_FAILED', 'POST_CONTACT'),
    ('place', 'PLAN_FAILED', 'POST_CONTACT'), ('retreat', 'ERROR', 'POST_CONTACT'),
    (None, 'PLAN_FAILED', 'CONTACT_UNKNOWN'), ('이상한', 'PLAN_FAILED', 'CONTACT_UNKNOWN')])
def test_step_과_실패_이유로_접촉_전후를_정한다(step, reason, phase):
    assert contact_phase(step, reason) == phase


def test_approach의_PLAN_FAILED만_접촉_전으로_재계획():
    f = flow()
    first = run(f, TARGET, resp(blk(length=30)))['request']
    assert f.on_failure('PLAN_FAILED', phase=contact_phase('approach', 'PLAN_FAILED'), grasped=False) == 'REPLAN_SAME'
    assert f.reusable_request() == first
    assert f.on_failure('PLAN_FAILED', phase=contact_phase('lift', 'PLAN_FAILED'), grasped=False) == 'RECOVER'
    assert f.reusable_request() is None


def test_approach의_그_밖_실패는_접촉을_모르므로_후보를_버린다():
    f = flow()
    run(f, TARGET, resp(blk(length=30)))
    assert f.on_failure('GRASP_FAILED', phase=contact_phase('approach', 'GRASP_FAILED'), grasped=False) == 'REOBSERVE'
    assert f.reusable_request() is None


NOW = 1000.0


@pytest.mark.parametrize('state, result_time, expect', [
    ({'stamp': 999.0, 'grasped': False}, 998.0, False),          # 결과 뒤 시각 · 3초 이내
    ({'stamp': 999.0, 'grasped': True}, 998.0, True),
    ({'stamp': 997.0, 'grasped': False}, 996.0, False),          # 정확히 3초는 믿는다
    ({'stamp': 996.9, 'grasped': False}, 996.0, None),           # 3초 넘음
    ({'stamp': 997.5, 'grasped': False}, 998.0, None),           # 결과를 받기 전 값(과거 메시지)
    ({'stamp': 1001.0, 'grasped': False}, 998.0, None),          # 미래 시각(시계 이상)
    (None, 998.0, None), ({}, 998.0, None), ({'stamp': 999.0}, 998.0, None), ({'grasped': False}, 998.0, None),
    ({'stamp': 'x', 'grasped': False}, 998.0, None), ({'stamp': float('nan'), 'grasped': False}, 998.0, None),
    ({'stamp': 999.0, 'grasped': 'no'}, 998.0, None), ({'stamp': 999.0, 'grasped': None}, 998.0, None),
    ({'stamp': True, 'grasped': False}, 998.0, None)])
def test_그리퍼_상태는_결과_뒤_3초_이내_값만_믿는다(state, result_time, expect):
    assert grasped_now(state, result_time, NOW, 3.0) is expect


@pytest.mark.parametrize('age', [0, -1, None, float('nan'), 'x'])
def test_믿는_시간이_이상하면_모름(age):
    assert grasped_now({'stamp': 999.0, 'grasped': False}, 998.0, NOW, age) is None


def test_모르면_접촉_전이어도_복구_절차():
    assert failure_action('PLAN_FAILED', 'PRE_CONTACT', grasped_now({'stamp': 900.0, 'grasped': False}, 800.0, NOW, 3.0)) == 'RECOVER'


# ---------- 요청별 마지막 step (가짜 액션 · 그리퍼) ----------
def test_step은_요청마다_따로_기억하고_이전_요청의_늦은_피드백은_버린다():
    tr = StepTracker()
    tr.begin(1)
    assert tr.on_feedback(1, 'approach') and tr.on_feedback(1, 'grasp')
    assert tr.last_step(1) == 'grasp'
    tr.begin(2)                                                     # 새 요청 — 이전 step 은 지워진다
    assert tr.last_step(2) is None and tr.last_step(1) is None
    assert not tr.on_feedback(1, 'lift')                            # 요청 1 의 늦은 피드백
    assert tr.last_step(2) is None
    assert tr.on_feedback(2, 'approach') and tr.last_step(2) == 'approach'


@pytest.mark.parametrize('rid, step', [(None, 'approach'), (3, 'approach'), (2, 'SIDE'), (2, None), (2, '')])
def test_모르는_요청이나_step은_기록하지_않는다(rid, step):
    tr = StepTracker()
    tr.begin(2)
    assert not tr.on_feedback(rid, step) and tr.last_step(2) is None


def test_요청이_끝나면_더_받지_않는다():
    tr = StepTracker()
    tr.begin(1)
    tr.on_feedback(1, 'move')
    tr.end(1)
    assert not tr.on_feedback(1, 'place') and tr.last_step(1) is None
    tr.begin(2)
    tr.end(1)                                                       # 이미 끝난 요청의 end 는 지금 요청에 영향 없음
    assert tr.on_feedback(2, 'approach')


def test_가짜_액션과_그리퍼로_실패_처리_전체_흐름():
    """요청 1 은 grasp 뒤 실패(쥐고 있음) → 복구, 요청 2 는 approach 의 PLAN_FAILED + 결과 뒤 신선한 그리퍼(쥐지 않음) → 같은 후보 재계획."""
    f, tr = flow(), StepTracker()
    run(f, TARGET, resp(blk(length=30)))
    tr.begin(1)
    tr.on_feedback(1, 'approach'); tr.on_feedback(1, 'grasp'); tr.on_feedback(1, 'lift')
    result_time = 500.0                                             # 결과를 받은 시각(time.time 기준)
    held = grasped_now({'stamp': 500.5, 'grasped': True}, result_time, 501.0, 3.0)
    assert f.on_failure('PLAN_FAILED', contact_phase(tr.last_step(1), 'PLAN_FAILED'), held) == 'RECOVER'
    run(f, TARGET, resp(blk(length=30)))
    tr.begin(2)
    tr.on_feedback(1, 'lift')                                       # 요청 1 의 늦은 피드백이 요청 2 를 망치지 않는다
    tr.on_feedback(2, 'approach')
    free = grasped_now({'stamp': 600.5, 'grasped': False}, 600.0, 601.0, 3.0)
    assert f.on_failure('PLAN_FAILED', contact_phase(tr.last_step(2), 'PLAN_FAILED'), free) == 'REPLAN_SAME'
    assert f.reusable_request() is not None


def test_결과_뒤의_신선한_그리퍼_상태가_없으면_복구_절차():
    f, tr = flow(), StepTracker()
    run(f, TARGET, resp(blk(length=30)))
    tr.begin(1)
    tr.on_feedback(1, 'approach')
    stale = grasped_now({'stamp': 599.0, 'grasped': False}, 600.0, 601.0, 3.0)      # 결과 이전 값
    assert stale is None
    assert f.on_failure('PLAN_FAILED', contact_phase(tr.last_step(1), 'PLAN_FAILED'), stale) == 'RECOVER'
    assert f.reusable_request() is None


def test_시계를_섞으면_믿지_않는다():
    """단조 시계 값(작은 수)을 now 나 result_time 에 섞으면 stamp(로봇 PC 시계)와 어긋나 None 이 된다 — 같은 시계끼리만 쓴다."""
    wall = 1.79e9
    assert grasped_now({'stamp': wall, 'grasped': False}, wall - 1, 12345.0, 3.0) is None         # now 가 단조 시계 → stamp 가 미래
    assert grasped_now({'stamp': 12344.0, 'grasped': False}, wall, wall + 1, 3.0) is None         # stamp 가 단조 시계 → 결과보다 과거
    assert grasped_now({'stamp': wall, 'grasped': False}, wall - 1, wall + 1, 3.0) is False       # 같은 시계면 믿는다


def test_robot_yaml_공급_방식_스위치_기본은_slots():
    assert CFG['supply_mode'] == 'slots' and CFG['supply_mode'] in ('slots', 'scatter')       # E-55


# ---------- StepTracker 단계 역행 방지 (W130 미해결 검토: grasp 뒤 늦은 approach 가 PRE_CONTACT 로 되돌리던 것) ----------
def test_단계는_앞으로만_가고_늦은_앞단계는_버린다():
    t = StepTracker()
    t.begin(1)
    assert t.on_feedback(1, 'approach') and t.on_feedback(1, 'grasp') and t.on_feedback(1, 'lift')
    assert t.on_feedback(1, 'approach') is False and t.on_feedback(1, 'grasp') is False      # 역행 · 중복
    assert t.last_step(1) == 'lift'
    assert t.on_feedback(1, 'retreat') is True and t.last_step(1) == 'retreat'


def test_늦은_approach가_와도_접촉_후는_접촉_전으로_되돌아가지_않는다():
    """같은 요청이 grasp 까지 갔는데 approach 피드백이 늦게 오면, 마지막 step 으로 접촉 전후를 정하므로 PRE_CONTACT 로 퇴행하면 안 된다."""
    t = StepTracker()
    t.begin(7)
    t.on_feedback(7, 'grasp')
    t.on_feedback(7, 'approach')
    assert contact_phase(t.last_step(7), 'PLAN_FAILED') == 'POST_CONTACT'


def test_새_요청은_다시_처음_단계부터_받는다():
    t = StepTracker()
    t.begin(1)
    t.on_feedback(1, 'place')
    t.begin(2)
    assert t.last_step(2) is None and t.on_feedback(2, 'approach') is True
    assert t.on_feedback(1, 'retreat') is False                                  # 이전 요청의 늦은 피드백

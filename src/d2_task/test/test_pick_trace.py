# -*- coding: utf-8 -*-
"""집기 요청 번호 · 피드백 단계 · 결과 수신 시각 수집 시험 (W130, ROS 없이).

수집만 한다 — 자동 재시도 동작은 바뀌지 않는다. 요청마다 새 번호를 만들어 StepTracker 에 잇고, 현재 요청의 피드백만 받으며(단계 역행 · 중복 · 모르는
단계는 버림), 결과를 실제로 받았을 때만 수신 시각을 적고, 끝난 요청의 늦은 피드백 · 결과가 다음 요청에 섞이지 않는지 본다.
"""
import pytest

from test_task_manager import IDS, FakeIO, OK, SAFE_OK, drive, go, picks
from d2_task.task_manager import TaskManager
from test_task_manager import CFG

FULL = ['approach', 'grasp', 'lift', 'move', 'place', 'retreat']


class TraceIO(FakeIO):
    """pick_place 마다 plan 대로 피드백 · 결과 콜백을 부르고 결과를 돌려주는 가짜. 콜백은 늦은 호출 시험을 위해 보관한다."""

    def __init__(self):
        super().__init__()
        self.plan, self.callbacks = [], []

    def pick_place(self, goal, should_abort, on_feedback=None, on_result=None):
        self.events.append(('call', 'pick', goal['block_id'], goal['supply_slot']))
        plan = self.plan.pop(0) if self.plan else {}
        self.callbacks.append((on_feedback, on_result))
        for step in plan.get('steps', FULL):
            on_feedback(step)
        if plan.get('late'):                                  # 이전 요청의 콜백이 지금 요청 도중에 늦게 불린다
            old_fb, old_result = self.callbacks[plan['late']]
            old_fb('place'); old_result(9999.0)
        at = plan.get('result_at', 1000.0)
        if at is not None:
            on_result(at)
        ok, why = plan.get('ret', OK)
        if ok:
            self.world.add(goal['block_id'])
        return ok, why


def make():
    io = TraceIO()
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    assert m.command('select_design', 'bench') == OK and m.command('start') == OK
    return m, io


def test_요청마다_새_번호를_만들고_마지막_단계와_결과_시각을_남긴다():
    m, io = make()
    drive(m, 'VERIFY')
    assert m.last_pick == {'request_id': 1, 'ok': True, 'reason': '', 'step': 'retreat', 'result_at': 1000.0}
    drive(m, 'DONE')
    assert m.last_pick['request_id'] == len(IDS) and len(io.callbacks) == len(IDS)


def test_단계_역행_중복_모르는_단계는_무시한다():
    m, io = make()
    io.plan = [{'steps': ['approach', 'grasp', 'approach', 'grasp', 'dance', 'lift', 'approach']}]
    drive(m, 'VERIFY')
    assert m.last_pick['step'] == 'lift'


@pytest.mark.parametrize('ret, steps, result_at, final', [
    ((False, 'ERROR'), [], None, 'ERROR'),                      # 목표 거절 — 결과도 피드백도 없음
    ((False, 'TIMEOUT'), ['approach'], None, 'ERROR'),          # 시간 초과 — 우리가 취소, 결과 없음 (정지 요청 접수 확인 전이라 ERROR)
    ((False, 'CANCELED'), ['approach', 'grasp'], None, 'STOPPED'),   # 우리 쪽 취소 — 결과 없음
    ((False, 'CANCELED'), ['approach', 'grasp'], 1500.5, 'STOPPED'), # 서버가 보낸 CANCELED 결과 — 결과 있음
    ((False, 'GRASP_FAILED'), ['approach', 'grasp'], 1700.0, None), # 서버가 보낸 실패 결과
])
def test_결과_없이_끝난_것과_결과를_받은_것을_가른다(ret, steps, result_at, final):
    m, io = make()
    io.plan = [{'steps': steps, 'result_at': result_at, 'ret': ret}]
    m.run_once(); m.run_once(); m.run_once()                     # CHECK → SELECT → PICK_PLACE 까지 거쳐 첫 집기를 부른다
    while not io.callbacks:
        m.run_once()
    assert m.last_pick == {'request_id': 1, 'ok': False, 'reason': ret[1],
                           'step': steps[-1] if steps else None, 'result_at': result_at}


def test_끝난_요청의_늦은_피드백과_결과는_다음_요청에_섞이지_않는다():
    """1번 요청이 끝난 뒤 2번 요청 도중에 1번의 콜백이 늦게 불려도 2번 기록은 그대로다. 같은 번호가 끝난 뒤 불려도 이미 넘긴 값은 안 바뀐다."""
    m, io = make()
    io.plan = [{}, {'steps': ['approach'], 'result_at': 2000.0, 'late': 0}]
    drive(m, 'VERIFY')                                           # 1번 끝
    first = dict(m.last_pick)
    old_fb, old_result = io.callbacks[0]
    old_fb('retreat'); old_result(7777.0)                         # 끝난 뒤 늦게 도착 → 무시
    assert m.last_pick == first
    drive(m, 'SELECT'); drive(m, 'VERIFY')                        # 2번: 도중에 1번 콜백이 늦게 불린다(plan 의 late)
    assert m.last_pick == {'request_id': 2, 'ok': True, 'reason': '', 'step': 'approach', 'result_at': 2000.0}
    old_fb('lift'); old_result(8888.0)                            # 2번이 끝난 뒤에도 무시
    assert m.last_pick['step'] == 'approach' and m.last_pick['result_at'] == 2000.0


def test_수집만_하고_slots_동작은_그대로다():
    """같은 시나리오(GRASP_FAILED 다음 칸 재시도)가 수집 유무와 상관없이 기존과 같은 칸 순서로 간다."""
    m, io = go()
    drive(m, 'DONE')
    assert [p[1] for p in picks(io)] == ['2'] * 9 + ['1'] * 2

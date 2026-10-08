# -*- coding: utf-8 -*-
"""W119 WAIT_HMI: 현재 블록 완료 · 자동 재개 금지 · 진행 재확인 · 공급 상태 보존. ROS 없이 단조 시계를 조절한다."""
import threading

import pytest

from d2_task.task_manager import TaskManager
from test_task_manager import CFG, IDS, SAFE_OK, SAFE_STOP, FakeIO, assert_ird_only, drive, picks


class Clock:
    """외부 stamp 와 독립적인 가짜 단조 시계(초)."""

    def __init__(self):
        """수신 시각을 0초부터 잰다."""
        self.now = 0.0

    def __call__(self):
        """현재 시험 시각을 초로 반환한다."""
        return self.now


def make(monitor=True):
    """안전·그리퍼 신호만 받은 관리자. 웹 신호는 아직 없다."""
    clock, io = Clock(), FakeIO()
    m = TaskManager(CFG, io, clock=clock, monitor_hmi=monitor)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io, clock


def go():
    """웹이 연결된 상태에서 출발한다."""
    m, io, clock = make()
    m.on_hmi_alive({'alive': True, 'stamp': -100000})
    assert m.command('select_design', 'bench') == (True, '')
    assert m.command('start') == (True, '')
    return m, io, clock


def test_원격모드는_첫_연결_신호_전에_출발하지_않는다():
    m, io, _clock = make()
    assert m.command('select_design', 'bench') == (False, '')            # IRD BUSY 는 '기다렸다 다시'라 웹이 자동 재시도한다 → reason 비움 + message(E-62)
    assert m.state == 'IDLE' and m.run_id is None and not picks(io)


def test_수신시각으로_정확히_3초에_끊김을_판정한다():
    m, io, clock = go()
    clock.now = CFG['mqtt']['lost_after_s'] - 0.01
    m.run_once()
    assert m.state == 'SELECT'
    before = list(io.calls)
    clock.now = CFG['mqtt']['lost_after_s']
    m.run_once()
    assert m.state == 'WAIT_HMI' and io.calls == before
    assert io.states[-1]['message_id'] == 'hmi_lost'


@pytest.mark.parametrize('body', [None, [], 'true', {}, {'alive': 'true'}, {'alive': 1}, {'alive': False}])
def test_잘못된_연결_값으로_연결이_갱신되지_않는다(body):
    m, _io, clock = go()
    clock.now = CFG['mqtt']['lost_after_s']
    m.on_hmi_alive(body)
    m.run_once()
    assert m.state == 'WAIT_HMI'


def test_로컬_개발은_웹없이_조립한다():
    m, io, clock = make(monitor=False)
    clock.now = 1000
    assert m.command('select_design', 'bench') == (True, '')
    assert m.command('start') == (True, '')
    drive(m, 'DONE')
    assert len(picks(io)) == 11


def test_다음_목표를_보내기_전에_끊기면_새_블록을_집지_않는다():
    m, io, clock = go()
    drive(m, 'PICK_PLACE')
    clock.now = CFG['mqtt']['lost_after_s']
    m.run_once()
    assert m.state == 'WAIT_HMI' and not picks(io)
    assert m.logger.active and m.last_build is None and not m.pending_builds
    assert not m.halted()                                  # 웹 대기는 정지 궤적·액션 취소의 원인이 아니다
    assert not any(c[0] == 'stop' for c in io.calls)
    assert_ird_only(io)


def test_현재_블록은_끝내되_재접속만으로_다음_블록을_시작하지_않는다():
    m, io, clock = go()
    drive(m, 'PICK_PLACE')
    entered, release = threading.Event(), threading.Event()

    def blocked(_io):
        """집기·놓기 진행 중에 웹이 끊긴 뒤 돌아오는 순서를 만든다."""
        entered.set()
        assert release.wait(2)
        assert not m.halted()
        return True, ''

    io.pick_script = [blocked]
    t = threading.Thread(target=m.run_once)
    t.start()
    try:
        assert entered.wait(2)
        clock.now = CFG['mqtt']['lost_after_s']
        m.poll_hmi()
        m.on_hmi_alive({'alive': True, 'stamp': 100000})
    finally:
        release.set()
        t.join(2)
    assert not t.is_alive() and m.state == 'WAIT_HMI'
    assert io.world == {IDS[0]} and len(picks(io)) == 1
    assert m.logger.active and m.last_build is None
    for _ in range(4):
        m.run_once()
    assert m.state == 'WAIT_HMI' and len(picks(io)) == 1


@pytest.mark.parametrize('voice', [False, True])
def test_계속은_같은_run으로_진행을_확인한_뒤_다음_블록을_고른다(voice):
    m, io, clock = go()
    drive(m, 'PICK_PLACE')

    def disconnect(_io):
        """현재 블록을 놓는 동안 웹이 끊긴다."""
        clock.now = CFG['mqtt']['lost_after_s']
        return True, ''

    io.pick_script = [disconnect]
    m.run_once()
    assert m.state == 'WAIT_HMI'
    run_id = m.run_id
    m.planner.mark_slot_empty(1)
    assert m.command('start') == (False, '')             # 웹이 아직 안 돌아왔다(reason 비움 — 웹이 BUSY 로 자동 재시도하지 않게)
    m.on_hmi_alive({'alive': True})
    if voice:
        m.on_intent('start')
    else:
        assert m.command('start') == (True, '')
    assert m.state == 'SELECT' and m.run_id == run_id
    before = len(io.calls)
    m.run_once()
    assert [c[0] for c in io.calls[before:]] == ['move_to', 'check']
    assert m.planner.progress[IDS[0]]['state'] == 'present'
    assert m.state == 'PICK_PLACE' and m.goal['block_id'] == IDS[1]
    assert m.planner.empty_slots == {1} and m.run_id == run_id
    assert len(picks(io)) == 1                          # 다음 액션은 이 단계에서 아직 안 보낸다


def test_WAIT_SUPPLY와_웹_계속의_공급_초기화를_섞지_않는다():
    m, io, clock = go()
    io.pick_script = [(False, 'SLOT_EMPTY')]
    drive(m, 'WAIT_SUPPLY')
    m.run_once()
    assert m.planner.empty_slots == {2}
    clock.now = CFG['mqtt']['lost_after_s']
    m.run_once()
    assert m.state == 'WAIT_HMI'
    m.on_hmi_alive({'alive': True})
    assert m.command('start') == (True, '') and m.state == 'SELECT'
    m.run_once()
    assert m.state == 'WAIT_SUPPLY' and m.planner.empty_slots == {2}


def test_진행_재확인_중_다시_끊기면_새_목표를_보내지_않는다():
    m, io, clock = go()
    clock.now = CFG['mqtt']['lost_after_s']
    m.run_once()
    m.on_hmi_alive({'alive': True})
    m.command('start')
    observe = io.check_progress

    def lose_during_check(block_ids, should_abort):
        """재관측 답을 기다리던 중 연결 신호가 끊긴다."""
        clock.now += CFG['mqtt']['lost_after_s']
        return observe(block_ids, should_abort)

    io.check_progress = lose_during_check
    m.run_once()
    assert m.state == 'WAIT_HMI' and not picks(io)


def test_웹대기_중_정지와_종료는_여전히_우선한다():
    m, _io, clock = go()
    run_id = m.run_id
    clock.now = CFG['mqtt']['lost_after_s']
    m.run_once()
    m.on_safety(SAFE_STOP)
    m.run_once()
    assert m.state == 'STOPPED' and m.run_id == run_id
    m.shutdown()
    m.finalize(0)
    assert m.last_build['result'] == 'STOPPED'

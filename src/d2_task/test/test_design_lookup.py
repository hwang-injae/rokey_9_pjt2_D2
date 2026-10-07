# -*- coding: utf-8 -*-
"""설계 조회(get_design) 시험 (W119 ②) — 선택 · 출발 때 각각 조회하고, 잘못된 답 · 시간 초과 · 늦은 답 · 조회 중 정지를 처리한다.

조회는 잠금 밖에서 기다리므로 가짜 io 가 스레드를 붙잡아 두고 그 사이 다른 일을 해 본다(ROS 없음).
"""
import copy
import threading
import time

import pytest

from d2_task.run_logger import RunLogger
from d2_task.task_manager import TaskManager
from test_build_saving import OK_JSON, RETRY_S
from test_task_manager import CFG, RECIPE, SAFE_OK, SAFE_STOP, FakeIO, drive


def design(design_id='bench', recipe=RECIPE, **over):
    """정상 design/1 답."""
    return dict({'schema': 'design/1', 'design_id': design_id, 'recipe': recipe}, **over)


class LookupIO(FakeIO):
    """get_design 을 스크립트로 정한다: 호출마다 한 칸 꺼내 쓴다(함수면 호출해서 (ok, reason, design) 을 받는다)."""

    def __init__(self):
        super().__init__()
        self.lookups = []                         # 조회한 design_id 들
        self.lookup_script = []

    def get_design(self, design_id, should_abort):
        self.events.append(('call', 'load', design_id))
        self.lookups.append(design_id)
        step = self.lookup_script.pop(0) if self.lookup_script else (True, '', design(design_id))
        return step(self, design_id, should_abort) if callable(step) else step


def make():
    io = LookupIO()
    m = TaskManager(CFG, io, RunLogger(''))
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def test_선택_때_한_번_조회하고_출발은_받아_둔_설계를_쓴다():
    """E-55 ①: [출발]은 [설계 선택] 때 받아 둔 설계로 — 다시 조회하지 않는다."""
    m, io = make()
    assert m.command('select_design', 'bench') == (True, '') and m.state == 'READY'
    assert io.lookups == ['bench']
    planner = m.planner
    assert m.command('start') == (True, '') and m.state == 'CHECK'
    assert io.lookups == ['bench'] and m.planner is planner and m.run_id             # 출발에 조회 없음
    drive(m, 'DONE')


def test_마지막으로_선택한_설계로_출발한다():
    m, io = make()
    assert m.command('select_design', 'A') == (True, '') and m.command('select_design', 'B') == (True, '')
    assert m.command('start') == (True, '')
    assert io.lookups == ['A', 'B'] and m.design_id == 'B' and m.state == 'CHECK'


def test_출발은_조회가_망가져도_받아_둔_설계로_간다():
    """선택 뒤 조회가 실패하는 상태가 되어도 출발은 조회하지 않으므로 영향이 없다(출발 시간 초과가 조회 시간에 안 걸림)."""
    m, io = make()
    m.command('select_design', 'bench')
    io.lookup_script = [(False, 'TIMEOUT', None)] * 3
    assert m.command('start') == (True, '') and m.state == 'CHECK' and io.lookups == ['bench']


def test_음성으로_설계를_포함해_출발하면_조회_뒤_출발한다():
    m, io = make()
    m.on_intent('start', 'bench')
    assert m.state == 'CHECK' and io.lookups == ['bench']


BAD_DESIGNS = {
    '객체가 아님': ['글자', None, []],
    'schema 다름': [design(schema='design/2'), design(schema=None)],
    'design_id 다름': [design(design_id='다른설계')],
    'recipe 없음': [design(recipe=None), design(recipe='글자'), design(recipe=[])],
    'recipe schema 다름': [design(recipe=dict(RECIPE, schema='other/1')), design(recipe={})],
    '레시피 내용이 로봇 설정과 안 맞음': [design(recipe=dict(copy.deepcopy(RECIPE), steps=[]))],
}


@pytest.mark.parametrize('kind', BAD_DESIGNS)
def test_잘못된_응답이면_거절하고_상태는_그대로(kind):
    for bad in BAD_DESIGNS[kind]:
        m, io = make()
        io.lookup_script = [(True, '', bad)]
        ok, why = m.command('select_design', 'bench')
        assert not ok and m.state == 'IDLE' and m.planner is None and m.run_id is None
        assert io.states[-1]['message']


def test_음성_출발의_잘못된_응답이면_출발하지_않는다():
    m, io = make()
    io.lookup_script = [(True, '', design(design_id='다른설계'))]
    m.on_intent('start', 'bench')
    assert m.state == 'IDLE' and m.run_id is None and m.logger.active is False


@pytest.mark.parametrize('why', ['TIMEOUT', 'ERROR', ''])
def test_조회_실패나_시간_초과면_새_조립을_시작하지_않는다(why):
    m, io = make()
    io.lookup_script = [(False, why, None)]
    assert m.command('select_design', 'bench') == (False, why) and m.state == 'IDLE'
    io.lookup_script = [(False, why, None)]
    m.on_intent('start', 'bench')                                   # 설계를 포함한 음성 출발도 조회가 실패하면 시작하지 않는다
    assert m.state == 'IDLE' and m.run_id is None
    assert 'CHECK' not in [s['state'] for s in io.states]


def test_조회_함수가_예외를_내도_시작하지_않는다():
    m, io = make()
    io.lookup_script = [lambda *_: 1 / 0]
    assert m.command('select_design', 'bench') == (False, 'ERROR') and m.state == 'IDLE'


def test_로컬_파일로_몰래_대신하지_않는다():
    m, io = make()
    io.load_recipe = lambda design_id: RECIPE        # 예전 방식의 로컬 읽기가 있어도 쓰지 않는다
    io.lookup_script = [(False, 'TIMEOUT', None)]
    assert m.command('select_design', 'bench') == (False, 'TIMEOUT') and m.planner is None


class Blocked:
    """조회를 gate 가 열릴 때까지 붙잡고, 열리면 정해진 답을 낸다."""

    def __init__(self, answer):
        self.entered, self.gate, self.answer = threading.Event(), threading.Event(), answer

    def __call__(self, io, design_id, should_abort):
        self.entered.set()
        self.gate.wait(10)
        return self.answer


def in_thread(fn, *args):
    out = []
    t = threading.Thread(target=lambda: out.append(fn(*args)), daemon=True)
    t.start()
    return t, out


def test_조회하는_동안_잠금을_잡지_않는다():
    m, io = make()
    slow = Blocked((True, '', design()))
    io.lookup_script = [slow]
    t, out = in_thread(m.command, 'select_design', 'bench')
    assert slow.entered.wait(5)
    t0 = time.monotonic()
    m.on_safety(SAFE_OK)                                   # 안전 콜백 · 방송이 막히지 않는다
    m.on_gripper({'grasped': False})
    m.run_once()
    assert time.monotonic() - t0 < 0.5
    slow.gate.set()
    t.join(5)
    assert out == [(True, '')] and m.state == 'READY'


def test_이전_요청의_늦은_답은_버린다():
    m, io = make()
    old = Blocked((True, '', design()))
    io.lookup_script = [old]
    t, out = in_thread(m.command, 'select_design', 'bench')
    assert old.entered.wait(5)
    assert m.command('select_design', 'bench') == (True, '')        # 더 새 요청이 먼저 끝나 READY
    ready_planner = m.planner
    old.gate.set()
    t.join(5)
    assert out == [(False, 'BUSY')] and m.planner is ready_planner and m.state == 'READY'


def test_조회_중_상태가_바뀌면_답을_버린다():
    m, io = make()
    slow = Blocked((True, '', design()))
    io.lookup_script = [slow]
    t, out = in_thread(m.command, 'start', 'bench')                 # IDLE 에서 설계를 포함한 출발 — 조회하는 중
    assert slow.entered.wait(5)
    io.lookup_script = []
    assert m.command('select_design', 'bench') == (True, '')        # 그 사이 다른 선택이 끝나 READY
    slow.gate.set()
    t.join(5)
    assert out == [(False, 'BUSY')] and m.state == 'READY' and m.run_id is None


def test_조회_중_정지_신호가_오면_출발하지_않는다():
    m, io = make()
    slow = Blocked((True, '', design()))
    io.lookup_script = [slow]
    t, out = in_thread(m.command, 'start', 'bench')
    assert slow.entered.wait(5)
    m.on_safety(SAFE_STOP)
    m.run_once()                                                    # STOPPED 로
    slow.gate.set()
    t.join(5)
    assert out == [(False, 'STOPPED')] and m.state == 'STOPPED' and m.run_id is None


def test_조회_중_종료하면_결과를_버린다():
    m, io = make()
    slow = Blocked((True, '', design()))
    io.lookup_script = [slow]
    t, out = in_thread(m.command, 'select_design', 'bench')
    assert slow.entered.wait(5)
    m.shutdown()
    slow.gate.set()
    t.join(5)
    assert out == [(False, 'STOPPED')] and m.state == 'IDLE' and m.planner is None


def test_조회_함수는_정지_신호로_빠져나올_수_있다():
    """io 에 넘기는 should_abort 가 정지 · 종료를 알려 준다(노드의 기다림이 이것으로 빠져나온다)."""
    m, io = make()
    seen = []

    def step(_io, _id, should_abort):
        seen.append(should_abort())
        m.on_safety(SAFE_STOP)
        seen.append(should_abort())
        return False, 'STOPPED', None

    io.lookup_script = [step]
    assert m.command('select_design', 'bench') == (False, 'STOPPED') and seen == [False, True]


def test_보관이_늦어_저장_성공한_run_id는_뒤늦은_보관_알림이_다시_넣지_않는다(tmp_path):
    """디스크 보관이 늦어 메모리로 보내 성공 처리한 뒤 보관 알림이 도착해도 _build_ready 에 남지 않는다."""
    class Slow:
        def __init__(self, real, gate):
            self.real, self.gate = real, gate

        def write(self, text):
            self.gate.wait(10)
            return self.real.write(text)

        def flush(self):
            self.real.flush()

        def fileno(self):
            return self.real.fileno()

        def close(self):
            self.real.close()

    gate = threading.Event()
    now = [0.0]
    io = FakeIO()
    log = RunLogger(str(tmp_path), opener=lambda p, mode='r', **kw: Slow(open(p, mode, **kw), gate) if mode == 'w' else open(p, mode, **kw))
    m = TaskManager(CFG, io, log, lambda: now[0])
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    m.command('select_design', 'bench')
    m.command('start')
    drive(m, 'DONE')
    run_id = m.run_id
    now[0] = RETRY_S
    assert [r for r, _ in m.builds_to_send(RETRY_S, RETRY_S)] == [run_id]
    assert m.build_result(run_id, True, OK_JSON) is True
    gate.set()                                                      # 이제서야 보관이 끝난다
    assert log.flush(5.0)
    assert run_id not in m._build_ready and m.pending_builds == {}


# ---------- 명령 제한 시간(timeout.command_s) — 늦은 선택 · 출발은 실행하지 않는다 ----------
def timed(clock):
    io = LookupIO()
    m = TaskManager(CFG, io, RunLogger(''), lambda: clock[0])
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def test_설정에_명령_제한이_있다():
    assert CFG['timeout']['command_s'] == 4 and CFG['timeout']['command_s'] > CFG['timeout']['service_s']


def test_조회가_command_s를_넘기면_선택하지_않고_TIMEOUT():
    now = [100.0]
    m, io = timed(now)
    io.lookup_script = [lambda *_: (now.__setitem__(0, now[0] + CFG['timeout']['command_s'] + 0.1), (True, '', design()))[1]]
    assert m.command('select_design', 'bench') == (False, 'TIMEOUT')
    assert m.state == 'IDLE' and m.planner is None and io.states[-1]['message']


def test_command_s_안에_끝나면_선택한다():
    now = [100.0]
    m, io = timed(now)
    io.lookup_script = [lambda *_: (now.__setitem__(0, now[0] + CFG['timeout']['command_s']), (True, '', design()))[1]]   # 정확히 제한 시간
    assert m.command('select_design', 'bench') == (True, '') and m.state == 'READY'


def test_늦은_음성_출발도_실행하지_않는다():
    now = [100.0]
    m, io = timed(now)
    io.lookup_script = [lambda *_: (now.__setitem__(0, now[0] + 10), (True, '', design()))[1]]
    m.on_intent('start', 'bench')
    assert m.state == 'IDLE' and m.run_id is None and m.logger.active is False
    assert 'CHECK' not in [s['state'] for s in io.states]

# -*- coding: utf-8 -*-
"""W119 시간 초과 · 정지 확인 · 복구 저속 시험. ROS 없이 실제 task 노드의 통신 메서드를 가짜 future 로 실행한다."""
import ast
import json
import logging
import threading
import time
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import pytest

from d2_task.task_manager import wait_until
from test_task_manager import CFG, SAFE_OK, SAFE_STOP, drive, go


@pytest.fixture
def node():
    """ROS 설치 없이 TaskNode 의 실제 통신 메서드 본문을 실행한다. 클라이언트 · 메시지만 가짜로 제공한다."""
    path = Path(__file__).parents[1] / 'd2_task/task_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TaskNode')
    names = {'_call', 'move_to', 'check_progress', 'request_stop', 'pick_place', '_cancel_late', 'cancel_active',
             '_scan_query', 'scan_capture', 'scan_infer'}
    cls = ast.ClassDef(name='TaskNode', bases=[], keywords=[],
                       body=[n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in names],
                       decorator_list=[])
    code = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    msg = SimpleNamespace(Request=SimpleNamespace)
    scope = dict(threading=threading, time=time, json=json, wait_until=wait_until, MoveTo=msg, StopRequest=msg, JsonQuery=msg,
                 CheckProgress=msg, PickPlace=SimpleNamespace(Goal=SimpleNamespace),
                 to_pose=lambda pose: pose, rclpy=SimpleNamespace(ok=lambda: True), CANCEL_WAIT_S=0.01)
    exec(compile(code, str(path), 'exec'), scope)
    n = scope['TaskNode']()
    n.service_s = n.move_to_s = n.pick_place_s = 0.04
    n._active, n._active_lock = None, threading.Lock()
    n.manager = SimpleNamespace(design_id='design_with_BACK_BEAM', run_id='R20261008_100000_ab12')
    n.get_logger = lambda: logging.getLogger('test_task_node')
    return n


class Client:
    """호출을 기록하고 지정한 future 를 반환한다. 추적을 지워도 서버 동작은 취소되지 않는다."""

    def __init__(self, future=None, ready=True):
        """future 와 서비스 공개 여부를 지정한다. 바깥 영향 없이 요청과 제거를 기록한다."""
        self.future = future if future is not None else Future()
        self.requests, self.removed, self.ready = [], [], ready

    def call_async(self, request):
        """요청을 기록하고 기다림 없이 future 를 반환한다."""
        self.requests.append(request)
        return self.future

    def service_is_ready(self):
        """가짜 서비스 공개 여부."""
        return self.ready

    def remove_pending_request(self, future):
        """추적에서 지운 future 를 기록한다. 서버 동작은 취소하지 않는다."""
        self.removed.append(future)


class Handle:
    """수락된 목표와 결과 future, 취소 횟수."""

    def __init__(self):
        """결과는 아직 오지 않았고 취소도 안 된 목표를 만든다."""
        self.accepted, self.result, self.cancels = True, Future(), 0

    def get_result_async(self):
        """결과 future 를 반환한다(대기 없음)."""
        return self.result

    def cancel_goal_async(self):
        """취소 요청 횟수를 기록하고 즉시 접수 future 를 반환한다."""
        self.cancels += 1
        return completed(SimpleNamespace())


def completed(result):
    """결과가 이미 온 future."""
    f = Future()
    f.set_result(result)
    return f


def goal():
    """통신 시험용 목표. 포즈는 변환하지 않아도 되는 가짜 값이다."""
    return dict(block_id='b1', supply_slot='1', grasp='FLAT_SHORT', pick_pose=None, place_pose=None)


def test_이동_시간초과는_서버취소로_오해하지_않는다(node):
    node.move_cli = Client()
    t0 = time.monotonic()
    assert node.move_to('observe', lambda: False) == (False, 'TIMEOUT')
    assert time.monotonic() - t0 < 0.3
    assert node.move_cli.removed == [node.move_cli.future]
    assert not node.move_cli.future.done()                   # 요청 추적만 지웠고 로봇 정지는 별도 요청이다


def test_관측_서비스도_시간제한이_있다(node):
    node.check_cli = Client()
    assert node.check_progress(['b1'], lambda: False) == (False, 'TIMEOUT', {})
    assert node.check_cli.removed == [node.check_cli.future]
    assert node.check_cli.requests[0].design_id == 'design_with_BACK_BEAM'
    assert node.check_cli.requests[0].run_id == 'R20261008_100000_ab12'      # E-60 ②: 한 판의 ID를 손목 블록 인식에 넘긴다


def test_run_id가_아직_없으면_빈_값(node):
    node.manager.run_id = None
    node.check_cli = Client()
    node.check_progress(['b1'], lambda: False)
    assert node.check_cli.requests[0].run_id == ''


def test_정지_요청은_task_TIME_OUT을_보낸다(node):
    node.stop_cli = Client(completed(SimpleNamespace(success=True, message='접수')))
    assert node.request_stop('TIMEOUT') == (True, '접수')
    assert node.stop_cli.requests[0].source == 'task'
    assert node.stop_cli.requests[0].reason == 'TIMEOUT'


@pytest.mark.parametrize('ready', [True, False])
def test_정지_서비스가_없거나_답이_없어도_기다림이_끝난다(node, ready):
    node.stop_cli = Client(ready=ready)
    t0 = time.monotonic()
    assert node.request_stop('TIMEOUT')[0] is False
    assert time.monotonic() - t0 < 0.3


def test_목표_수락이_늦으면_수락_즉시_취소한다(node):
    sent = Future()
    node.pick_cli = SimpleNamespace(send_goal_async=lambda _g: sent)
    assert node.pick_place(goal(), lambda: False) == (False, 'TIMEOUT')
    h = Handle()
    sent.set_result(h)
    assert h.cancels == 1 and node._active is None


def test_결과_시간초과는_취소를_먼저_보낸다(node):
    h = Handle()
    node.pick_cli = SimpleNamespace(send_goal_async=lambda _g: completed(h))
    assert node.pick_place(goal(), lambda: False) == (False, 'TIMEOUT')
    assert h.cancels == 1 and node._active is None


def test_취소_통신이_실패해도_TIME_OUT을_잃지_않는다(node):
    """취소 예외가 TIMEOUT 대신 일반 예외로 바뀌어 정지 서비스 호출을 건너뛰면 안 된다."""
    h = Handle()

    def broken_cancel():
        """취소 요청 통신이 실패한 경우."""
        raise RuntimeError('cancel transport unavailable')

    h.cancel_goal_async = broken_cancel
    node.pick_cli = SimpleNamespace(send_goal_async=lambda _g: completed(h))
    assert node.pick_place(goal(), lambda: False) == (False, 'TIMEOUT')
    assert node._active is None


def test_중단_신호가_이미_있으면_새_이동과_목표를_보내지_않는다(node):
    """정지 확인 중에도 호출부가 한 번 더 실행됐을 때 로봇에 새 요청을 보내지 않는다."""
    node.move_cli = Client()
    assert node.move_to('observe', lambda: True) == (False, 'STOPPED')
    assert not node.move_cli.requests
    sent = []
    node.pick_cli = SimpleNamespace(send_goal_async=lambda g: sent.append(g))
    assert node.pick_place(goal(), lambda: True) == (False, 'STOPPED')
    assert not sent


def test_수락과_결과에_같은_90초_예산을_쓴다(node, monkeypatch):
    h, budgets, clock = Handle(), [], [100.0]
    h.result.set_result(SimpleNamespace(result=SimpleNamespace(success=True, reason='')))
    node.pick_place_s = 90
    node.pick_cli = SimpleNamespace(send_goal_async=lambda _g: completed(h))

    def wait(_event, _abort, timeout_s):
        """수락 대기에 60초가 걸린 것으로 만든다."""
        budgets.append(timeout_s)
        clock[0] += 60
        return True

    scope = node.pick_place.__func__.__globals__
    monkeypatch.setitem(scope, 'wait_until', wait)
    monkeypatch.setitem(scope, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    assert node.pick_place(goal(), lambda: False) == (True, '')
    assert budgets == [90, 30]


def test_복구_첫이동만_저속이고_새조립_ID를_만들지_않는다():
    m, io = go()
    run_id = m.run_id
    m.on_safety(SAFE_STOP)
    m.run_once()
    m.on_safety(SAFE_OK)
    drive(m, 'CHECK')
    m.run_once()
    assert io.move_speeds == [CFG['recover']['speed_ratio']]
    drive(m, 'DONE')
    assert io.move_speeds[1:] and all(s == 1.0 for s in io.move_speeds[1:])
    assert m.run_id == run_id


def test_복구_이동_BUSY이면_재시도도_저속이다():
    m, io = go()
    m.on_safety(SAFE_STOP)
    m.run_once()
    m.on_safety(SAFE_OK)
    drive(m, 'CHECK')
    io.move_script = [(False, 'BUSY'), (True, '')]
    m.run_once()
    m.run_once()
    assert io.move_speeds == [0.5, 0.5]


@pytest.mark.parametrize('script', ['move_script', 'check_script', 'pick_script'])
def test_시간초과는_정지접수만으로_복구하지_않는다(script):
    m, io = go(**{script: [(False, 'TIMEOUT')]})
    io.stop_script = [(True, '접수')]
    drive(m, 'ERROR')
    before = list(io.calls)
    assert ('stop', 'TIMEOUT') in before
    m.on_safety(SAFE_OK)                                     # 실제 stopped 확인 없는 resume · 생존 방송은 허가가 아니다
    for _ in range(3):
        m.run_once()
    assert m.state == 'ERROR' and io.calls == before and m.halted()
    m.on_safety(SAFE_STOP)
    m.run_once()
    assert m.state == 'STOPPED'
    m.on_safety(SAFE_OK)
    drive(m, 'CHECK')
    m.run_once()
    assert io.move_speeds[-1] == 0.5


def test_정지_요청이_실패해도_새출발을_막는다():
    m, io = go(move_script=[(False, 'TIMEOUT')])
    drive(m, 'ERROR')
    assert m.command('start')[0] is False
    m.on_safety(SAFE_OK)
    m.run_once()
    assert m.state == 'ERROR'


def test_정지_요청_중에도_콜백은_잠금에_막히지_않는다():
    m, io = go(move_script=[(False, 'TIMEOUT')])
    entered, release = threading.Event(), threading.Event()

    def stop(_io):
        """정지 응답을 잠깐 늦춘다."""
        entered.set()
        assert release.wait(2)
        return True, '접수'

    io.stop_script = [stop]
    t = threading.Thread(target=m.run_once)
    t.start()
    try:
        assert entered.wait(2)
        done = threading.Event()
        callback = threading.Thread(target=lambda: (m.on_safety(SAFE_STOP), done.set()))
        callback.start()
        assert done.wait(1)
        callback.join(1)
    finally:
        release.set()
        t.join(2)
    assert not t.is_alive() and m.state == 'STOPPED'


def test_이미온_답도_중단과_만료보다_우선하지_않는다():
    done = threading.Event()
    done.set()
    assert not wait_until(done, lambda: True, timeout_s=1)
    assert not wait_until(done, lambda: False, timeout_s=0)


def test_CtrlC는_정지요청_뒤에만_기록을_마무리한다():
    """실제 main 의 종료 순서를 검증한다. 로봇 · executor · 기록은 순서만 기록하는 가짜다."""
    path = Path(__file__).parents[1] / 'd2_task/task_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    events = []

    def interrupt():
        """조립 단계 중 Ctrl+C 를 받은 것으로 만든다."""
        raise KeyboardInterrupt

    manager = SimpleNamespace(run_once=interrupt, shutdown=lambda: events.append('shutdown'),
                              finalize=lambda _s: events.append('finalize'))
    logger = SimpleNamespace(info=lambda _s: None, error=lambda _s: None)
    task = SimpleNamespace(manager=manager, get_logger=lambda: logger,
                           cancel_active=lambda: events.append('cancel'),
                           request_stop=lambda _why: (events.append('stop') or True, ''),
                           destroy_node=lambda: events.append('destroy'))
    executor = SimpleNamespace(add_node=lambda _n: None, spin=lambda: None,
                               shutdown=lambda **_kw: events.append('executor shutdown'))
    scope = dict(logging=logging, init_ros=lambda: None, TaskNode=lambda: task,
                 MultiThreadedExecutor=lambda: executor, threading=threading,
                 rclpy=SimpleNamespace(ok=lambda: True, shutdown=lambda: events.append('ros shutdown')),
                 time=time, LOOP_S=0, LOG_FLUSH_S=0)
    exec(compile(ast.Module(body=[main], type_ignores=[]), str(path), 'exec'), scope)
    scope['main']()
    assert events == ['shutdown', 'cancel', 'stop', 'finalize', 'executor shutdown', 'destroy', 'ros shutdown']


# ---------- 집기 요청 피드백 · 결과 수신 시각 수집 (W130, 수집만 — 재시도 동작은 안 바꾼다) ----------
def fb(step):
    """PickPlace 피드백 메시지 모양(msg.feedback.step) 가짜."""
    return SimpleNamespace(feedback=SimpleNamespace(step=step))


def test_피드백과_결과_수신_시각을_콜백으로_알린다(node, monkeypatch):
    """결과는 실제로 받은 순간의 time.time() 이고, 제한 시간 계산(monotonic)과 섞이지 않는다."""
    h, seen = Handle(), {'steps': [], 'at': []}
    h.result.set_result(SimpleNamespace(result=SimpleNamespace(success=True, reason='')))

    def send(_g, feedback_callback=None):
        feedback_callback(fb('approach')); feedback_callback(fb('grasp'))
        return completed(h)

    node.pick_cli = SimpleNamespace(send_goal_async=send)
    scope = node.pick_place.__func__.__globals__
    monkeypatch.setitem(scope, 'time', SimpleNamespace(monotonic=time.monotonic, time=lambda: 1234.5))
    assert node.pick_place(goal(), lambda: False, on_feedback=seen['steps'].append, on_result=seen['at'].append) == (True, '')
    assert seen == {'steps': ['approach', 'grasp'], 'at': [1234.5]}


def test_콜백을_안_주면_피드백_인자를_넘기지_않는다(node):
    h = Handle()
    h.result.set_result(SimpleNamespace(result=SimpleNamespace(success=True, reason='')))
    node.pick_cli = SimpleNamespace(send_goal_async=lambda _g: completed(h))          # feedback_callback 인자를 받지 않는 가짜
    assert node.pick_place(goal(), lambda: False) == (True, '')


def test_서버가_보낸_실패_결과도_수신_시각이_있다(node):
    """결과를 받았다(수신 시각 있음)와 결과 없이 끝났다(없음)를 가른다: 서버의 CANCELED 결과는 앞쪽."""
    h, at = Handle(), []
    h.result.set_result(SimpleNamespace(result=SimpleNamespace(success=False, reason='CANCELED')))
    node.pick_cli = SimpleNamespace(send_goal_async=lambda _g: completed(h))
    assert node.pick_place(goal(), lambda: False, on_result=at.append) == (False, 'CANCELED') and len(at) == 1


def test_거절_시간초과_우리쪽_취소는_결과_수신_시각이_없다(node):
    at = []
    rejected = Handle(); rejected.accepted = False
    node.pick_cli = SimpleNamespace(send_goal_async=lambda _g: completed(rejected))
    assert node.pick_place(goal(), lambda: False, on_result=at.append) == (False, 'ERROR')           # 거절
    h = Handle()                                                                                      # 결과가 안 옴 → 시간 초과
    node.pick_cli = SimpleNamespace(send_goal_async=lambda _g: completed(h))
    assert node.pick_place(goal(), lambda: False, on_result=at.append) == (False, 'TIMEOUT')
    flag = {'abort': False}                                                                           # 기다리다 우리 쪽이 취소
    h2 = Handle()
    node.pick_cli = SimpleNamespace(send_goal_async=lambda _g: completed(h2))
    node.pick_place_s = 5

    def abort_soon():
        time.sleep(0.05)
        flag['abort'] = True

    threading.Thread(target=abort_soon).start()
    assert node.pick_place(goal(), lambda: flag['abort'], on_result=at.append) == (False, 'CANCELED')
    assert at == [] and h2.cancels == 1

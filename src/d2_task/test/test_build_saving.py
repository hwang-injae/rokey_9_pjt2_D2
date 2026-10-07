# -*- coding: utf-8 -*-
"""결과 저장(save_build) 시험 (W119 ①) — 끝난 조립의 build/1 요약을 run_id 별로 들고 있다가 저장이 확인된 것만 지운다.

ROS 없이 TaskManager 의 builds_to_send · build_result · build_failed 를 가짜 서비스 대신 직접 부른다.
(실제 노드가 서비스를 부르고 답을 이 메서드로 넘기는 부분은 task_node 의 일 — 따로 ROS 로 확인한다.)
"""
import json

import pytest

from d2_task.run_logger import RunLogger
from d2_task.task_manager import TaskManager
from test_task_manager import CFG, SAFE_OK, FakeIO, drive

OK_JSON = '{"ok": true}'
RETRY_S = 5.0                                  # robot.yaml timeout.service_s


def new_manager():
    """파일 기록 없이 run_id 만 만드는 TaskManager + 가짜 io, 신호는 받은 상태."""
    io = FakeIO()
    m = TaskManager(CFG, io, RunLogger(''))
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def assemble(m, io):
    """bench 를 골라 출발해 DONE 까지 돌린다. 반환: 그 조립의 run_id. 작업대는 매번 비운다(새 조립)."""
    io.world = set()
    assert m.command('select_design', 'bench') == (True, '')
    assert m.command('start') == (True, '')
    drive(m, 'DONE')
    return m.run_id


def test_끝난_조립의_요약은_run_id별_대기_목록에_복사본으로_들어간다():
    m, io = new_manager()
    a = assemble(m, io)
    assert list(m.pending_builds) == [a] and m.pending_builds[a] == m.last_build
    assert m.pending_builds[a] is not m.last_build                # 복사본 — 표시용 값을 고쳐도 대기 요약은 그대로
    m.last_build['placed'] = -1
    assert m.pending_builds[a]['placed'] == 11


def test_보낼_요약도_복사본이고_보냈다고_적힌다():
    m, io = new_manager()
    a = assemble(m, io)
    (run_id, summary), = m.builds_to_send(100.0, RETRY_S)
    assert run_id == a and summary == m.pending_builds[a] and summary is not m.pending_builds[a]
    summary['placed'] = -1
    assert m.pending_builds[a]['placed'] == 11
    assert m.builds_to_send(100.0, RETRY_S) == []                  # 답을 기다리는 중에는 또 보내지 않는다


def test_success와_ok가_모두_참이어야_지운다():
    m, io = new_manager()
    a = assemble(m, io)
    m.builds_to_send(0.0, RETRY_S)
    assert m.build_result(a, True, OK_JSON) is True
    assert m.pending_builds == {} and m.last_build['run_id'] == a  # 최근 결과 표시용 last_build 는 남는다


@pytest.mark.parametrize('success, body', [
    (False, OK_JSON),                          # 서비스 실패
    (True, '{"ok": false}'),                   # 처리했지만 저장 실패
    (True, '{"ok": "true"}'),                  # 글자 true 는 참이 아니다
    (True, '{"ok": 1}'),
    (True, '{}'),
    (True, '[]'),
    (True, '"ok"'),
    (True, 'null'),
    (True, ''),
    (True, '{깨진'),
    (True, None),
])
def test_실패하거나_잘못된_답이면_요약을_남긴다(success, body):
    m, io = new_manager()
    a = assemble(m, io)
    m.builds_to_send(0.0, RETRY_S)
    assert m.build_result(a, success, body) is False
    assert list(m.pending_builds) == [a]


def test_A_저장_실패_뒤_B가_조립되고_저장돼도_A_요약은_남는다():
    m, io = new_manager()
    a = assemble(m, io)
    m.builds_to_send(0.0, RETRY_S)
    m.build_result(a, False, '')                                    # A 저장 실패
    b = assemble(m, io)                                             # B 조립
    assert a != b and set(m.pending_builds) == {a, b}
    sent = {r for r, _ in m.builds_to_send(0.5, RETRY_S)}           # 방금 실패한 A 는 아직 간격 안, B 만 나간다
    assert sent == {b}
    assert m.build_result(b, True, OK_JSON) is True                 # B 저장 성공
    assert list(m.pending_builds) == [a] and m.pending_builds[a]['result'] == 'DONE'


def test_다시_보내기는_간격이_지난_뒤에만():
    m, io = new_manager()
    a = assemble(m, io)
    assert [r for r, _ in m.builds_to_send(10.0, RETRY_S)] == [a]
    m.build_failed(a, '시간 초과')
    assert m.builds_to_send(10.0 + RETRY_S - 0.1, RETRY_S) == []
    assert [r for r, _ in m.builds_to_send(10.0 + RETRY_S, RETRY_S)] == [a]
    assert m.build_result(a, True, OK_JSON) is True
    assert m.builds_to_send(1000.0, RETRY_S) == []                  # 저장된 것은 다시 안 보낸다


def test_늦은_응답은_그_요청의_run_id만_바꾼다():
    m, io = new_manager()
    a = assemble(m, io)
    m.builds_to_send(0.0, RETRY_S)
    m.build_failed(a, '5초 안에 답이 없다')                         # 시간 초과로 포기
    b = assemble(m, io)
    m.builds_to_send(1.0, RETRY_S)
    assert m.build_result(a, True, OK_JSON) is True                 # A 의 늦은 성공 답 → A 만 지운다
    assert list(m.pending_builds) == [b]
    assert m.build_result(a, False, '') is False                    # A 의 또 다른 늦은 답은 무시
    assert m.build_result('R_모르는_번호', True, OK_JSON) is False  # 모르는 run_id 도 무시
    assert list(m.pending_builds) == [b] and m.pending_builds[b]['run_id'] == b


def test_늦은_실패_응답이_이미_저장된_요약을_되살리지_않는다():
    m, io = new_manager()
    a = assemble(m, io)
    m.builds_to_send(0.0, RETRY_S)
    assert m.build_result(a, True, OK_JSON) is True
    assert m.build_result(a, False, '') is False
    assert m.pending_builds == {}


def test_정지와_ERROR로_끝난_조립의_요약도_대기_목록에_들어간다():
    m, io = new_manager()
    io.pick_script = [(False, 'PLAN_FAILED')] * 2
    assert m.command('select_design', 'bench') == (True, '')
    assert m.command('start') == (True, '')
    drive(m, 'ERROR')
    run_id = m.run_id
    m.on_intent('cancel')                                           # 사람이 그만둔다 → ERROR 로 닫힘
    assert m.pending_builds[run_id]['result'] == 'ERROR'
    m2, io2 = new_manager()
    assert m2.command('select_design', 'bench') == (True, '')
    assert m2.command('start') == (True, '')
    drive(m2, 'SELECT')
    m2.shutdown()
    m2.finalize(1.0)                                                # 조립 중에 끝낸다 → STOPPED
    assert [b['result'] for b in m2.pending_builds.values()] == ['STOPPED']


def test_복구할_수_있는_일시_정지는_요약을_대기_목록에_넣지_않는다():
    m, io = new_manager()
    io.pick_script = [(False, 'STOPPED')]
    assert m.command('select_design', 'bench') == (True, '')
    assert m.command('start') == (True, '')
    drive(m, 'STOPPED')
    assert m.pending_builds == {}                                   # 아직 끝난 조립이 아니다
    m.on_safety(SAFE_OK)
    drive(m, 'DONE')
    assert [b['result'] for b in m.pending_builds.values()] == ['DONE']


def test_종료_중에는_새로_보내지_않고_대기_요약은_남는다():
    m, io = new_manager()
    a = assemble(m, io)
    m.builds_to_send(0.0, RETRY_S)                                  # A 는 보내고 답을 기다리는 중
    m.build_failed(a, '아직 답 없음')
    m.shutdown()
    assert m.builds_to_send(100.0, RETRY_S) == []                   # 종료 중에는 네트워크 일을 새로 시작하지 않는다
    assert list(m.pending_builds) == [a]                            # 요약은 지우지 않는다
    assert m.build_result(a, True, OK_JSON) is True                 # 이미 나간 요청의 늦은 성공 답은 그대로 반영
    assert m.pending_builds == {}


def test_여러_요약이_쌓여도_run_id별로_따로_처리한다():
    m, io = new_manager()
    ids = [assemble(m, io) for _ in range(3)]
    assert len(set(ids)) == 3
    assert {r for r, _ in m.builds_to_send(0.0, RETRY_S)} == set(ids)
    assert m.build_result(ids[1], True, OK_JSON) is True
    assert m.build_result(ids[0], True, '{"ok": false}') is False
    m.build_failed(ids[2], 'x')
    assert set(m.pending_builds) == {ids[0], ids[2]}
    assert json.dumps(list(m.pending_builds.values()), allow_nan=False)


# ---------- BuildSender: 요청을 future 객체로 구분 ----------
from d2_task.build_sender import BuildSender  # noqa: E402


class FakeFuture:
    """서비스 호출 결과 흉내. done() 으로 직접 끝낸다."""

    def __init__(self):
        self.callbacks, self.value, self.error = [], None, None

    def add_done_callback(self, cb):
        self.callbacks.append(cb)

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def done(self, success=True, body=OK_JSON, error=None):
        self.value = type('Res', (), {'success': success, 'response_json': body})()
        self.error = error
        for cb in self.callbacks:
            cb(self)


class Rig:
    """BuildSender + 가짜 서비스. 보낸 요청(future)을 sent 에 쌓고, 시계는 now 를 직접 올린다."""

    def __init__(self, ready=True):
        self.m, self.io = new_manager()
        self.sent, self.cancelled, self.now, self.ready = [], [], 0.0, ready
        self.sender = BuildSender(self.m, self._call, self.cancelled.append, lambda: self.ready, RETRY_S, lambda: self.now)

    def _call(self, summary):
        f = FakeFuture()
        f.summary = summary
        self.sent.append(f)
        return f


def test_첫_요청_시간_초과_뒤_재전송하면_이전_요청의_늦은_응답은_무시된다():
    for late in (dict(success=True), dict(success=False, body=''), dict(error=RuntimeError('늦은 예외'))):
        r = Rig()
        a = assemble(r.m, r.io)
        r.sender.poll()                                             # 첫 요청
        first = r.sent[0]
        r.now = RETRY_S
        r.sender.poll()                                             # 시간 초과 → 포기 → 같은 run_id 로 두 번째 요청
        assert r.cancelled == [first] and len(r.sent) == 2 and r.sender.waiting() == [a]
        second = r.sent[1]
        first.done(**late)                                          # 첫 요청의 늦은 성공 · 실패 · 예외
        assert r.sender.waiting() == [a] and list(r.m.pending_builds) == [a]    # 현재(두 번째) 요청은 계속 추적, 요약 그대로
        r.now = RETRY_S + 1
        r.sender.poll()
        assert len(r.sent) == 2                                     # 두 번째가 끝나기 전에는 추가 전송 없음
        second.done()                                               # 두 번째가 성공
        assert r.m.pending_builds == {} and r.sender.waiting() == []


def test_포기한_요청의_늦은_성공은_요약을_지우지_않는다():
    r = Rig()
    a = assemble(r.m, r.io)
    r.sender.poll()
    r.now = RETRY_S
    r.sender.poll()
    r.sent[0].done(success=True)                                    # 포기한 요청이 늦게 성공해도 현재 요청이 확인하기 전까지는 유지
    assert list(r.m.pending_builds) == [a]


def test_서버가_안_떠_있으면_보내지_않고_요약을_남긴다():
    r = Rig(ready=False)
    a = assemble(r.m, r.io)
    r.sender.poll()
    assert r.sent == [] and list(r.m.pending_builds) == [a]
    r.ready = True
    r.sender.poll()
    assert len(r.sent) == 1


def test_요청을_못_만들면_요약을_남기고_다음에_다시():
    r = Rig()
    a = assemble(r.m, r.io)
    r.sender._call = lambda s: (_ for _ in ()).throw(ValueError('NaN'))
    r.sender.poll()
    assert list(r.m.pending_builds) == [a] and r.sender.waiting() == []
    r.sender._call = r._call
    r.now = RETRY_S
    r.sender.poll()
    assert len(r.sent) == 1


def test_정상_흐름은_한_번만_보낸다():
    r = Rig()
    a = assemble(r.m, r.io)
    r.sender.poll()
    r.sent[0].done()
    r.now = 100.0
    r.sender.poll()
    assert len(r.sent) == 1 and r.m.pending_builds == {} and r.sent[0].summary['run_id'] == a

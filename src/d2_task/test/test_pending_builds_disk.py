# -*- coding: utf-8 -*-
"""미전송 build/1 요약의 디스크 보관 시험 (W119) — log_dir/pending_builds/<run_id>.json.

저장 → 전송 → 확인 뒤 삭제 순서, 시작할 때 복원, 깨진 파일 · 임시 파일, 저장 · 삭제 실패, 디스크가 멈춘 동안의 정지 · 취소.
파일 쓰기는 기록 스레드가 하므로 파일을 보기 전에 flush 로 다 처리될 때까지 기다린다.
"""
import json
import os
import threading
import time

import pytest

from d2_task.run_logger import RunLogger
from d2_task.task_manager import TaskManager
from test_build_saving import OK_JSON, RETRY_S
from test_task_manager import CFG, SAFE_OK, FakeIO, drive


def make(tmp_path, opener=open, clock=time.monotonic):
    """디스크 보관을 켠(log_dir 있음) TaskManager."""
    io = FakeIO()
    log = RunLogger(str(tmp_path), opener=opener)
    m = TaskManager(CFG, io, log, clock)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io, log


def assemble(m, io):
    io.world = set()
    assert m.command('select_design', 'bench') == (True, '')
    assert m.command('start') == (True, '')
    drive(m, 'DONE')
    return m.run_id


def files(tmp_path):
    """pending_builds 폴더의 파일 이름들(없으면 빈 목록)."""
    folder = tmp_path / 'pending_builds'
    return sorted(os.listdir(folder)) if folder.is_dir() else []


class GatedFile:
    """진짜 파일을 감싸되 쓰기를 gate 가 열릴 때까지 붙잡는다(느리거나 멈춘 디스크)."""

    def __init__(self, real, gate):
        self.real, self.gate = real, gate

    def write(self, text):
        self.gate.wait(30)
        return self.real.write(text)

    def flush(self):
        self.real.flush()

    def fileno(self):
        return self.real.fileno()

    def close(self):
        self.real.close()


def gated_opener(gate, only_mode=None):
    """gate 가 열릴 때까지 쓰기가 멈추는 opener. only_mode 를 주면 그 mode 로 여는 파일만(예: 'w' = 미전송 요약)."""
    def opener(path, mode='r', **kw):
        f = open(path, mode, **kw)
        return GatedFile(f, gate) if only_mode in (None, mode) else f
    return opener


def test_저장_뒤_새_객체에서_복원되고_다시_보낼_수_있다(tmp_path):
    m, io, log = make(tmp_path)
    a = assemble(m, io)
    assert log.flush(5.0) and files(tmp_path) == [f'{a}.json']
    saved = json.loads((tmp_path / 'pending_builds' / f'{a}.json').read_text(encoding='utf-8'))
    assert saved == m.pending_builds[a] and saved['schema'] == 'build/1'
    m2, io2, log2 = make(tmp_path)                                  # 재시작: 새 객체
    assert m2.pending_builds == {a: saved}
    assert [r for r, _ in m2.builds_to_send(0.0, RETRY_S)] == [a]   # 로봇 작업을 받기 전에 이미 복원돼 있고 바로 보낼 수 있다
    assert m2.build_result(a, True, OK_JSON) is True
    assert log2.flush(5.0) and files(tmp_path) == []


def test_저장이_확인된_run만_파일을_지우고_다른_run은_남는다(tmp_path):
    m, io, log = make(tmp_path)
    a = assemble(m, io)
    b = assemble(m, io)
    assert log.flush(5.0) and files(tmp_path) == sorted([f'{a}.json', f'{b}.json'])
    m.builds_to_send(0.0, RETRY_S)
    m.build_result(a, False, '')                                    # A 저장 실패
    assert m.build_result(b, True, OK_JSON) is True                 # B 저장 성공
    assert log.flush(5.0) and files(tmp_path) == [f'{a}.json']      # A 미전송 파일은 B 성공 뒤에도 남는다
    m3, _, _ = make(tmp_path)
    assert list(m3.pending_builds) == [a]


def test_실패하거나_잘못된_답이면_파일을_안_지운다(tmp_path):
    m, io, log = make(tmp_path)
    a = assemble(m, io)
    m.builds_to_send(0.0, RETRY_S)
    for ok, body in ((False, OK_JSON), (True, '{"ok": false}'), (True, '깨진')):
        assert m.build_result(a, ok, body) is False
    assert log.flush(5.0) and files(tmp_path) == [f'{a}.json']


def test_빠른_응답에도_삭제_뒤_파일이_다시_만들어지지_않는다(tmp_path):
    for _ in range(5):
        m, io, log = make(tmp_path)
        a = assemble(m, io)
        for _ in range(200):                                        # 보관이 끝나자마자 보내고 바로 성공 응답
            sent = m.builds_to_send(0.0, RETRY_S)
            if sent:
                break
            time.sleep(0.001)
        assert [r for r, _ in sent] == [a]
        assert m.build_result(a, True, OK_JSON) is True
        assert log.flush(5.0) and files(tmp_path) == []


def test_디스크가_늦어_보관이_안_끝난_채_보내고_응답이_와도_파일은_남지_않는다(tmp_path):
    gate = threading.Event()
    now = [0.0]
    m, io, log = make(tmp_path, gated_opener(gate, 'w'), clock=lambda: now[0])
    try:
        a = assemble(m, io)
        assert m.builds_to_send(0.0, RETRY_S) == []                 # 보관이 안 끝났으니 잠깐 기다린다
        now[0] = RETRY_S
        assert [r for r, _ in m.builds_to_send(RETRY_S, RETRY_S)] == [a]    # 디스크가 늦으면 기다리지 않고 메모리로 보낸다
        assert m.build_result(a, True, OK_JSON) is True             # 보관이 끝나기 전에 성공 응답
    finally:
        gate.set()
    assert log.flush(5.0) and files(tmp_path) == []                 # 저장 뒤 삭제 순서라 파일이 되살아나지 않는다


def test_깨진_파일과_임시_파일은_복원하지_않고_지우지도_않는다(tmp_path):
    m, io, log = make(tmp_path)
    a = assemble(m, io)
    assert log.flush(5.0)
    good = json.loads((tmp_path / 'pending_builds' / f'{a}.json').read_text(encoding='utf-8'))
    folder = tmp_path / 'pending_builds'

    def put(name, value):
        text = value if isinstance(value, str) else json.dumps(value)
        (folder / name).write_text(text, encoding='utf-8')

    put('R_깨짐.json', '{깨진')
    put('R_배열.json', '[]')
    put('R_schema.json', dict(good, run_id='R_schema', schema='build/2'))
    put('R_이름다름.json', dict(good, run_id='다른이름'))
    put('R_칸없음.json', {k: v for k, v in dict(good, run_id='R_칸없음').items() if k != 'placed'})
    put('R_자료형.json', dict(good, run_id='R_자료형', placed='11'))
    put('R_결과.json', dict(good, run_id='R_결과', result='FAILED'))
    put('R_블록.json', dict(good, run_id='R_블록', blocks=[{'block_id': 'B1', 'dz_m': 'x', 'dx_m': None, 'dy_m': None}]))
    put('R_NaN.json', '{"schema":"build/1","run_id":"R_NaN","duration_s":NaN}')
    put('R_임시.json.tmp', dict(good, run_id='R_임시'))              # 쓰다 만 임시 파일
    before = files(tmp_path)
    got = RunLogger(str(tmp_path)).load_pending()
    assert [r for r, _ in got] == [a]
    assert files(tmp_path) == before                                # 아무것도 지우지 않았다


def test_폴더가_없거나_log_dir이_비면_복원할_것이_없다(tmp_path):
    assert RunLogger(str(tmp_path / '없음')).load_pending() == []
    assert RunLogger('').load_pending() == []


def test_파일_저장이_실패해도_메모리로_다시_보낼_수_있고_CSV는_그대로(tmp_path):
    def opener(path, mode='r', **kw):
        if mode == 'w':
            raise OSError('저장 실패')                                # 미전송 요약 파일만 실패
        return open(path, mode, **kw)

    m, io, log = make(tmp_path, opener)
    a = assemble(m, io)
    assert log.flush(5.0)
    assert files(tmp_path) == [] and log.file_ok                    # 요약 파일은 없지만 CSV 기록(file_ok)은 살아 있다
    assert [r for r, _ in m.builds_to_send(0.0, RETRY_S)] == [a]    # 저장 실패여도 메모리 재전송은 이어진다
    assert m.build_result(a, True, OK_JSON) is True
    assert log.flush(5.0)
    assert not [n for n in os.listdir(tmp_path / 'pending_builds') if n.endswith('.tmp')] if (tmp_path / 'pending_builds').is_dir() else True


def test_쓰다_실패하면_임시_파일을_남기지_않는다(tmp_path):
    class Dies:
        def __init__(self, real):
            self.real = real

        def write(self, _text):
            raise OSError('디스크 가득')

        def flush(self):
            pass

        def close(self):
            self.real.close()

    def opener(path, mode='r', **kw):
        f = open(path, mode, **kw)
        return Dies(f) if mode == 'w' else f

    m, io, log = make(tmp_path, opener)
    assemble(m, io)
    assert log.flush(5.0) and files(tmp_path) == []


def test_파일_삭제가_실패해도_메모리는_정리되고_다음_시작에_다시_보낼_수_있다(tmp_path, monkeypatch):
    m, io, log = make(tmp_path)
    a = assemble(m, io)
    assert log.flush(5.0)
    m.builds_to_send(0.0, RETRY_S)

    def broken_remove(path):
        raise PermissionError('삭제 못 함')

    monkeypatch.setattr(os, 'remove', broken_remove)
    assert m.build_result(a, True, OK_JSON) is True and m.pending_builds == {}
    assert log.flush(5.0) and files(tmp_path) == [f'{a}.json']      # 파일은 남았다 → 다음 시작에 다시 전송(HMI 가 같은 run_id 중복을 처리해야 함)
    monkeypatch.undo()
    assert list(make(tmp_path)[0].pending_builds) == [a]


def test_log_dir이_비면_디스크_보관_없이_바로_보낸다():
    io = FakeIO()
    m = TaskManager(CFG, io, RunLogger(''))
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    a = assemble(m, io)
    assert [r for r, _ in m.builds_to_send(0.0, RETRY_S)] == [a]
    assert RunLogger('').save_pending('R', {}, lambda ok: None) is False


def test_디스크가_멈춘_동안에도_정지와_종료는_기다리지_않는다(tmp_path):
    gate = threading.Event()
    m, io, log = make(tmp_path, gated_opener(gate))
    try:
        t0 = time.monotonic()
        assert m.command('select_design', 'bench') == (True, '')
        assert m.command('start') == (True, '')
        drive(m, 'SELECT')
        m.on_safety({'stopped': True, 'locked': True, 'reason': 'STOP_REQUEST'})
        m.run_once()
        assert m.state == 'STOPPED'
        m.on_safety(SAFE_OK)
        drive(m, 'DONE')                                            # 끝나서 요약을 디스크에 맡기는 순간에도 안 막힌다
        m.shutdown()
        m.finalize(0.3)
        assert time.monotonic() - t0 < 3.0 and len(m.pending_builds) == 1
    finally:
        gate.set()
    assert log.flush(5.0)


def test_보관한_요약의_칸은_build_1_그대로(tmp_path):
    m, io, log = make(tmp_path)
    a = assemble(m, io)
    assert log.flush(5.0)
    saved = json.loads((tmp_path / 'pending_builds' / f'{a}.json').read_text(encoding='utf-8'))
    assert set(saved) == {'schema', 'run_id', 'design_id', 'result', 'placed', 'total', 'duration_s', 'stop_count', 'blocks'}
    assert saved['blocks'][0].keys() == {'block_id', 'dz_m', 'dx_m', 'dy_m'}
    assert saved['blocks'][0]['dx_m'] is None

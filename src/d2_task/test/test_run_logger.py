# -*- coding: utf-8 -*-
"""RunLogger(W065) 시험 — ROS · 로봇 없이. 기록기 단위 시험 + TaskManager 가 기록 시점을 정하는지(가짜 io, 가짜 시계).

TaskManager 쪽은 test_task_manager 의 가짜 io · 도우미를 가져와 쓴다: 정상 완료 · 정지 후 계속 · 공급 보충 후 계속 ·
실패 · 취소 종료 · DONE 과 새 출발 사이 경쟁 · 디스크가 멈춘 동안의 정지 · 취소 · 파일 쓰기 실패.
파일은 전용 쓰기 스레드가 쓰므로 CSV 를 읽기 전에는 기록기의 flush() 로 다 쓰일 때까지 기다린다(rows · kinds 가 한다).
"""
import csv
import json
import logging
import re
import threading
import time
import uuid
from pathlib import Path

import pytest

from d2_task.run_logger import COLUMNS, RunLogger
from d2_task.task_manager import TaskManager
from test_task_manager import CFG, IDS, SAFE_OK, SAFE_STOP, FakeIO, drive

T0 = time.mktime((2026, 10, 10, 12, 0, 0, 0, 0, -1))      # 2026-10-10 12:00
RUN_ID = r'R20261010_[0-9a-f]{32}'
LIVE = []                                                  # 이 시험에서 만든 기록기들 — CSV 를 읽기 전에 모두 flush


@pytest.fixture(autouse=True)
def _reset_live():
    LIVE.clear()
    yield
    LIVE.clear()


class Clock:
    """시험용 시계. now 를 직접 올린다."""

    def __init__(self, now=T0):
        self.now = now

    def __call__(self):
        return self.now


def logger(path, clock=None, **kw):
    """시험용 RunLogger(flush 대상에 등록)."""
    log = RunLogger(str(path) if path else '', clock or Clock(), **kw)
    LIVE.append(log)
    return log


def rows(path):
    """모든 기록기를 flush 한 뒤 CSV 를 dict 목록으로 읽는다."""
    for log in LIVE:
        assert log.flush(5.0)
    with open(path, encoding='utf-8', newline='') as f:
        return list(csv.DictReader(f))


def kinds(path):
    """CSV 의 (kind, block_id, value) 들."""
    return [(r['kind'], r['block_id'], r['value']) for r in rows(path)]


# ---------- RunLogger 단위 ----------
def test_run_id_모양():
    log = logger(None)
    assert re.fullmatch(RUN_ID, log.start_run('d', 3))


def test_run_id는_새_객체_재시작_다른_폴더에서도_겹치지_않는다(tmp_path):
    ids = set()
    for _ in range(3):                                       # 재시작처럼 매번 새 객체, 파일 기록 꺼짐(폴더 없음)
        log = logger(None)
        for _ in range(3):
            ids.add(log.start_run('d', 1))
            log.finish('DONE')
    for sub in ('a', 'b'):                                   # 저장 폴더가 달라도
        log = logger(tmp_path / sub)
        ids.add(log.start_run('d', 1))
        log.finish('DONE')
    log = logger(tmp_path / 'a')                             # 같은 폴더를 다시 열어도
    ids.add(log.start_run('d', 1))
    assert len(ids) == 9 + 2 + 1


def test_폴더가_같고_이름이_같아도_덮어쓰지_않는다(tmp_path):
    same = uuid.UUID(int=7)
    first = logger(tmp_path, new_id=lambda: same)
    run_id = first.start_run('d', 1)
    first.log('placed', 'B1', '원본')
    first.finish('DONE')
    original = rows(tmp_path / f'{run_id}.csv')
    again = logger(tmp_path, new_id=lambda: same)            # 같은 ID 가 나오는 극단적인 경우
    assert again.start_run('d', 1) == run_id
    again.log('placed', 'B2', '덮어쓰기')
    again.finish('DONE')
    assert again.flush(5.0) and not again.file_ok
    assert rows(tmp_path / f'{run_id}.csv') == original


def test_CSV_칸과_머리글(tmp_path):
    log = logger(tmp_path)
    run_id = log.start_run('bench', 11)
    log.finish('DONE')
    got = rows(tmp_path / f'{run_id}.csv')
    assert tuple(got[0]) == COLUMNS
    assert got[0]['run_id'] == run_id and got[0]['design_id'] == 'bench' and got[0]['module'] == 'task'
    assert re.fullmatch(r'2026-10-10T12:00:00\.\d{3}', got[0]['time'])
    assert [r['kind'] for r in got] == ['start', 'end'] and got[1]['value'] == 'DONE'


def test_log_dir의_틸데는_홈으로_바뀐다(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    log = logger('~/d2_data/runs')
    log.start_run('d', 1)
    log.finish('DONE')
    assert log.flush(5.0) and len(list((tmp_path / 'd2_data' / 'runs').glob('R*.csv'))) == 1


def test_못_잰_값은_0이_아니라_빈_칸과_null(tmp_path):
    log = logger(tmp_path)
    run_id = log.start_run('d', 2)
    log.record_placed('B1', 2)
    log.record_measure('B1', float('nan'), None, 0.0008)
    build = log.finish('DONE')
    assert kinds(tmp_path / f'{run_id}.csv')[2:5] == [('dx_mm', 'B1', ''), ('dy_mm', 'B1', ''), ('dz_mm', 'B1', '0.800')]
    assert build['blocks'] == [{'block_id': 'B1', 'dz_m': 0.0008, 'dx_m': None, 'dy_m': None}]
    json.dumps(build, allow_nan=False)


@pytest.mark.parametrize('bad', [float('inf'), -float('inf'), 'x', True])
def test_유한한_숫자가_아니면_못_잰_값(bad):
    log = logger(None)
    log.start_run('d', 1)
    log.record_placed('B1')
    log.record_measure('B1', bad, bad, bad)
    assert log.finish('DONE')['blocks'][0] == {'block_id': 'B1', 'dz_m': None, 'dx_m': None, 'dy_m': None}


def test_build_1_요약_칸과_단위():
    clock = Clock()
    log = logger(None, clock)
    run_id = log.start_run('chair_v1.1', 18)
    log.record_placed('B1')
    log.record_placed('B2')
    log.record_measure('B1', 0.0001, -0.0002, 0.0003)
    log.count_stop()
    log.count_stop()
    clock.now += 742.5
    build = log.finish('STOPPED')
    assert build == {'schema': 'build/1', 'run_id': run_id, 'design_id': 'chair_v1.1', 'result': 'STOPPED',
                     'placed': 2, 'total': 18, 'duration_s': 742.5, 'stop_count': 2,
                     'blocks': [{'block_id': 'B1', 'dz_m': 0.0003, 'dx_m': 0.0001, 'dy_m': -0.0002},
                                {'block_id': 'B2', 'dz_m': None, 'dx_m': None, 'dy_m': None}]}


def test_같은_블록을_다시_놓아도_한_번만_센다():
    log = logger(None)
    log.start_run('d', 2)
    log.record_placed('B1')
    log.record_placed('B1')
    assert log.finish('DONE')['placed'] == 1


def test_두_번_닫거나_result가_틀리면():
    log = logger(None)
    assert log.finish('DONE') is None                      # 열린 run 이 없다
    log.start_run('d', 1)
    with pytest.raises(ValueError):
        log.finish('FAILED')
    assert log.finish('DONE') is not None and log.finish('DONE') is None


def test_새_run을_열면_열린_run은_STOPPED로_닫힌다(tmp_path):
    log = logger(tmp_path)
    first = log.start_run('d', 1)
    second = log.start_run('d', 1)
    assert first != second and kinds(tmp_path / f'{first}.csv')[-1] == ('end', '', 'STOPPED')


def test_폴더를_못_만들면_파일_없이_run_id와_요약은_계속(tmp_path):
    blocker = tmp_path / '파일'
    blocker.write_text('폴더가 아니다', encoding='utf-8')
    log = logger(blocker / '하위')
    assert re.fullmatch(RUN_ID, log.start_run('d', 1))
    log.record_placed('B1')
    assert log.finish('DONE')['placed'] == 1
    assert log.flush(5.0) and not log.file_ok


def test_파일_기록을_끄면_enabled가_거짓이고_flush는_바로():
    log = logger(None)
    assert not log.enabled and log.flush(0.0)


def test_쓰는_도중_실패해도_예외를_내지_않고_파일만_끈다(tmp_path):
    class Broken:
        def write(self, _text):
            raise OSError('디스크 가득')

        def flush(self):
            pass

        def close(self):
            pass

    log = logger(tmp_path, opener=lambda *a, **k: Broken())
    log.start_run('d', 2)
    log.record_placed('B1')                                 # 예외 없이 지나간다
    log.record_measure('B1', 0.0, 0.0, 0.0)
    assert log.flush(5.0) and not log.file_ok
    assert log.finish('ERROR')['placed'] == 1


def test_이전_run의_늦은_파일_열기_오류가_새_run의_기록을_멈추지_않는다(tmp_path):
    """① 이전 run 파일 열기를 대기 ② 이전 run 종료 뒤 새 run 시작 ③ 이전 파일 열기에서 오류 ④ 새 run 에 기록 ⑤ 새 CSV · 요약이 모두 정상."""
    gate = threading.Event()
    opened = []

    def opener(path, mode, **kw):
        opened.append(path)
        if len(opened) == 1:
            gate.wait(10)
            raise OSError('이전 run 파일 열기 실패')
        return open(path, mode, **kw)

    log = logger(tmp_path, opener=opener)
    old = log.start_run('d', 1)                       # ① 쓰기 스레드가 이 run 의 파일 열기에서 멈춘다
    old_build = log.finish('STOPPED')                 # ② 이전 run 종료
    new = log.start_run('d', 1)                       #    새 run 시작
    gate.set()                                        # ③ 이전 파일 열기가 이제서야 오류를 낸다
    assert log.flush(5.0)                             #    (오류 처리가 끝나고 새 run 의 start 까지 처리된 뒤)
    assert log.file_ok                                #    새 run 의 기록은 켜져 있다
    log.record_placed('B1', 2)                        # ④ 새 run 에 블록 기록 후 종료
    log.record_measure('B1', None, None, 0.0004)
    build = log.finish('DONE')
    got = rows(tmp_path / f'{new}.csv')               # ⑤ 새 CSV 와 요약이 모두 정상
    assert [(r['kind'], r['value']) for r in got] == [('start', '1'), ('placed', 'slot 2'), ('dx_mm', ''), ('dy_mm', ''),
                                                      ('dz_mm', '0.400'), ('end', 'DONE')]
    assert {r['run_id'] for r in got} == {new}        # 이전 run 의 줄이 섞이지 않는다
    assert build['placed'] == 1 and build['result'] == 'DONE' and old_build['run_id'] == old
    assert not (tmp_path / f'{old}.csv').exists()     # 이전 run 은 파일을 못 만들었다


def test_이전_run의_파일_열기가_늦게_성공해도_새_run_줄이_섞이지_않는다(tmp_path):
    gate = threading.Event()
    opened = []

    def opener(path, mode, **kw):
        opened.append(path)
        if len(opened) == 1:
            gate.wait(10)
        return open(path, mode, **kw)

    log = logger(tmp_path, opener=opener)
    old = log.start_run('d', 1)
    log.record_placed('A1')
    log.finish('DONE')
    new = log.start_run('d', 1)
    log.record_placed('B1')
    gate.set()
    log.finish('DONE')
    assert [(r['kind'], r['block_id']) for r in rows(tmp_path / f'{old}.csv')] == [('start', ''), ('placed', 'A1'), ('end', '')]
    assert [(r['kind'], r['block_id']) for r in rows(tmp_path / f'{new}.csv')] == [('start', ''), ('placed', 'B1'), ('end', '')]


def test_쓰기가_밀려_큐가_차도_부르는_쪽은_안_막힌다(tmp_path, monkeypatch):
    import d2_task.run_logger as mod
    monkeypatch.setattr(mod, 'QUEUE_LIMIT', 5)
    gate = threading.Event()

    class Stuck:
        def write(self, _text):
            gate.wait(10)

        def flush(self):
            pass

        def close(self):
            pass

    log = logger(tmp_path, opener=lambda *a, **k: Stuck())
    log.start_run('d', 100)
    t0 = time.monotonic()
    for k in range(200):
        log.record_placed(f'B{k}')
    assert time.monotonic() - t0 < 1.0 and not log.file_ok          # 큐가 차면 기록을 끄고 넘어간다
    assert log.finish('DONE')['placed'] == 200                      # 요약은 그대로 정확하다
    gate.set()


# ---------- TaskManager 가 기록 시점을 정한다 ----------
def build(tmp_path, clock=None, io=None, **scripts):
    """정지 · 그리퍼 신호를 받은 TaskManager(진짜 RunLogger 사용)를 bench 로 READY 까지."""
    io = io or FakeIO()
    log = logger(tmp_path, clock)
    m = TaskManager(CFG, io, log)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    for name, value in scripts.items():
        setattr(io, name, value)
    assert m.command('select_design', 'bench') == (True, '')
    return m, io


def start(m):
    assert m.command('start') == (True, '')


def csv_of(tmp_path, m):
    return tmp_path / f'{m.run_id}.csv'


def test_출발할_때_run_id가_생기고_READY에는_없다(tmp_path):
    m, io = build(tmp_path)
    assert m.run_id is None and io.states[-1]['run_id'] is None
    start(m)
    assert re.fullmatch(RUN_ID, m.run_id)
    assert io.states[-1]['run_id'] == m.run_id
    rows(csv_of(tmp_path, m))                                  # 파일이 만들어졌다


def test_점검에_실패하면_run_id를_만들지_않는다(tmp_path):
    m, io = build(tmp_path)
    io.missing = ['/d2/motion/pick_place']
    assert not m.command('start')[0] and m.run_id is None and list(tmp_path.iterdir()) == []


def test_정상_완료_기록과_요약(tmp_path):
    m, io = build(tmp_path)
    start(m)
    run_id = m.run_id
    drive(m, 'DONE')
    assert m.run_id == run_id and io.states[-1]['run_id'] == run_id          # 끝난 직후에도 화면에 남는다
    b = m.last_build
    assert (b['result'], b['placed'], b['total'], b['stop_count']) == ('DONE', 11, 11, 0) and b['run_id'] == run_id
    assert [x['block_id'] for x in b['blocks']] == IDS and all(x['dz_m'] == 0.0 for x in b['blocks'])
    assert all(x['dx_m'] is None and x['dy_m'] is None for x in b['blocks'])  # 가짜 io 는 dx · dy 를 못 잼
    got = rows(csv_of(tmp_path, m))
    assert got[0]['kind'] == 'start' and got[0]['value'] == '11' and got[-1]['kind'] == 'end' and got[-1]['value'] == 'DONE'
    assert sum(r['kind'] == 'placed' for r in got) == 11 and sum(r['kind'] == 'dz_mm' for r in got) == 11
    assert {r['value'] for r in got if r['kind'] in ('dx_mm', 'dy_mm')} == {''}   # 못 잰 값 = 빈 칸
    assert [r['value'] for r in got if r['kind'] == 'state'][:3] == ['CHECK', 'SELECT', 'PICK_PLACE']
    json.dumps(b, allow_nan=False)


def test_progress_방송에도_run_id(tmp_path):
    m, io = build(tmp_path)
    start(m)
    drive(m, 'SELECT')
    assert io.progress[-1]['run_id'] == m.run_id


def test_정지_뒤_다시_시작해도_같은_run_id(tmp_path):
    m, io = build(tmp_path, pick_script=[(False, 'STOPPED')])
    start(m)
    run_id = m.run_id
    drive(m, 'STOPPED')
    assert m.run_id == run_id and io.states[-1]['run_id'] == run_id
    m.on_safety(SAFE_OK)
    drive(m, 'DONE')
    assert {s['run_id'] for s in io.states if s['state'] in ('STOPPED', 'RECOVER', 'CHECK', 'DONE')} == {run_id}
    assert m.last_build['stop_count'] == 1 and m.last_build['result'] == 'DONE'
    got = kinds(csv_of(tmp_path, m))
    assert len(list(tmp_path.glob('*.csv'))) == 1
    assert ('fail', IDS[0], 'pick_place STOPPED') in got and ('state', IDS[0], 'STOPPED') in got
    assert [k for k, _, v in got if k == 'state' and v == 'RECOVER']


def test_정지_신호로_멈춘_뒤에도_같은_run_id(tmp_path):
    m, io = build(tmp_path)
    start(m)
    drive(m, 'SELECT')
    run_id = m.run_id
    m.on_safety(SAFE_STOP)
    m.run_once()
    m.on_safety(SAFE_OK)
    drive(m, 'DONE')
    assert m.run_id == run_id and m.last_build['stop_count'] == 1


def test_공급_보충_뒤_계속해도_같은_run_id(tmp_path):
    m, io = build(tmp_path, pick_script=[(False, 'SLOT_EMPTY')])
    start(m)
    run_id = m.run_id
    drive(m, 'WAIT_SUPPLY')
    m.run_once()                                                # 관측 자세 도착 → supply_empty
    assert m.command('start') == (True, '')
    drive(m, 'DONE')
    assert m.run_id == run_id and m.last_build['result'] == 'DONE'
    assert ('command', IDS[0], 'continue') in kinds(csv_of(tmp_path, m))


def test_새_조립은_새_run_id이고_이전_요약은_남는다(tmp_path):
    m, io = build(tmp_path)
    start(m)
    first = m.run_id
    drive(m, 'DONE')
    assert m.command('select_design', 'bench') == (True, '')
    assert m.run_id is None and io.states[-1]['run_id'] is None   # 새 설계를 고르면 run_id 는 비운다
    assert m.last_build['run_id'] == first                         # 보내기 전의 요약은 남는다(W119 가 보낸다)
    start(m)
    assert m.run_id != first and m.last_build['run_id'] == first   # 새 출발로도 지우지 않는다
    assert m.logger.flush(5.0) and len(list(tmp_path.glob('*.csv'))) == 2


def test_실패_뒤_취소하면_ERROR로_닫는다(tmp_path):
    m, io = build(tmp_path, pick_script=[(False, 'PLAN_FAILED')] * 2)
    start(m)
    run_id = m.run_id
    drive(m, 'ERROR')
    assert m.run_id == run_id
    m.on_intent('cancel')
    assert m.state == 'IDLE' and m.run_id is None and io.states[-1]['run_id'] is None
    b = m.last_build
    assert (b['result'], b['placed'], b['run_id']) == ('ERROR', 0, run_id)
    got = kinds(tmp_path / f'{run_id}.csv')
    assert got[-1] == ('end', '', 'ERROR') and any(k == 'error' for k, _, _ in got)


def test_일부만_놓고_끝내면_STOPPED로_닫는다(tmp_path):
    m, io = build(tmp_path)
    start(m)
    drive(m, 'SELECT')
    m.run_once()
    m.run_once()                                                # 1번 블록을 놓는다
    m.shutdown()                                                # Ctrl+C: 중단 플래그만 — 기록은 아직 안 닫는다
    assert m.halted() and m.last_build is None
    m.finalize(5.0)                                             # 취소 요청 뒤 마지막에 기록을 닫는다
    b = m.last_build
    assert b['result'] == 'STOPPED' and b['placed'] >= 1 and b['total'] == 11
    assert kinds(tmp_path / f'{b["run_id"]}.csv')[-1] == ('end', '', 'STOPPED')
    m.finalize(5.0)                                             # 두 번 불러도 괜찮다
    assert m.last_build is b


def test_ERROR에서_끝내면_ERROR로_닫는다(tmp_path):
    m, io = build(tmp_path, pick_script=[(False, 'PLAN_FAILED')] * 2)
    start(m)
    drive(m, 'ERROR')
    m.shutdown()
    m.finalize(5.0)
    assert m.last_build['result'] == 'ERROR'


def test_오차가_커서_ERROR가_돼도_측정값은_남는다(tmp_path):
    m, io = build(tmp_path)
    io.dz = 0.02                                                # 두께 절반(7.5 mm)보다 크게 어긋남
    start(m)
    drive(m, 'ERROR')
    got = kinds(csv_of(tmp_path, m))
    assert ('dz_mm', IDS[0], '20.000') in got and any(k == 'error' for k, _, _ in got)


def test_기록기가_없어도_TaskManager는_예전처럼_돈다():
    io = FakeIO()
    m = TaskManager(CFG, io)                                    # logger 인자 없음 → 파일 없이 run_id 만
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    m.command('select_design', 'bench')
    m.command('start')
    drive(m, 'DONE')
    assert m.run_id and m.last_build['placed'] == 11


# ---------- DONE 과 새 출발 사이 경쟁 ----------
class RacingIO(FakeIO):
    """DONE 을 방송하는 순간 다른 스레드(화면 명령)가 새 설계 선택 · 출발을 시도한다."""

    def __init__(self):
        super().__init__()
        self.intruder = None
        self.intruder_results = []

    def publish_state(self, msg):
        super().publish_state(msg)
        if msg['state'] == 'DONE' and self.intruder is None:
            def intrude():
                self.intruder_results.append(self.manager.command('select_design', 'bench'))
                self.intruder_results.append(self.manager.command('start'))
            self.intruder = threading.Thread(target=intrude, daemon=True)
            self.intruder.start()


def test_DONE_방송과_요약_확정_사이에_새_출발이_끼어들지_못한다(tmp_path):
    io = RacingIO()
    m, _ = build(tmp_path, io=io)
    start(m)
    first = m.run_id
    real_finish = m._finish_run

    def slow_finish(result):
        """요약을 확정하기 직전에 스레드가 바뀌는 최악의 순간을 만든다 — 잠금 안이면 끼어들 수 없다."""
        if result == 'DONE':
            time.sleep(0.3)
        return real_finish(result)

    m._finish_run = slow_finish
    drive(m, 'DONE')
    io.intruder.join(5.0)
    assert io.intruder_results == [(True, ''), (True, '')]
    assert m.state == 'CHECK' and m.run_id != first and m.logger.active       # 새 조립은 열린 채로
    assert m.last_build['run_id'] == first and (m.last_build['result'], m.last_build['placed']) == ('DONE', 11)
    assert kinds(tmp_path / f'{first}.csv')[-1] == ('end', '', 'DONE')
    assert kinds(csv_of(tmp_path, m))[-1][0] in ('state', 'start')              # 새 조립 파일에는 end 가 없다


# ---------- 기록이 정지 · 취소를 막지 않는다 ----------
class StalledDisk:
    """쓰기를 gate 가 열릴 때까지 붙잡아 두는 가짜 파일(느리거나 멈춘 디스크)."""

    def __init__(self, gate):
        self.gate = gate

    def write(self, _text):
        self.gate.wait(30)

    def flush(self):
        pass

    def close(self):
        pass


def stalled(tmp_path, gate, **scripts):
    """쓰기가 멈춘 디스크로 TaskManager 를 READY 까지."""
    io = FakeIO()
    log = logger(tmp_path, opener=lambda *a, **k: StalledDisk(gate))
    m = TaskManager(CFG, io, log)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    for name, value in scripts.items():
        setattr(io, name, value)
    assert m.command('select_design', 'bench') == (True, '')
    return m, io


def test_디스크가_멈춰도_정지_신호에_바로_선다(tmp_path):
    gate = threading.Event()
    m, io = stalled(tmp_path, gate)
    try:
        t0 = time.monotonic()
        start(m)
        drive(m, 'SELECT')
        m.on_safety(SAFE_STOP)                                  # 안전 콜백
        m.run_once()
        assert m.state == 'STOPPED' and io.states[-1]['message_id'] == 'stopped'
        m.on_safety(SAFE_OK)
        drive(m, 'DONE')                                        # 복구 · 완료까지 기록을 안 기다리고 간다
        assert time.monotonic() - t0 < 2.0
        assert m.last_build['result'] == 'DONE' and m.last_build['stop_count'] == 1
    finally:
        gate.set()


class Warnings:
    """d2_task 로거의 경고 글자를 모으는 핸들러(이 환경에서는 pytest caplog 가 안 돼서 직접 단다)."""

    def __init__(self):
        self.texts = []
        self.handler = logging.Handler(logging.WARNING)
        self.handler.emit = lambda record: self.texts.append(record.getMessage())

    def __enter__(self):
        logging.getLogger('d2_task').addHandler(self.handler)
        return self

    def __exit__(self, *exc):
        logging.getLogger('d2_task').removeHandler(self.handler)


def test_finalize가_시간_안에_못_끝나면_경고를_남긴다(tmp_path):
    gate = threading.Event()
    m, io = stalled(tmp_path, gate)
    try:
        start(m)
        with Warnings() as w:
            m.finalize(0.2)
        assert any('아직 다 안 써졌다' in t for t in w.texts)
    finally:
        gate.set()


def test_finalize가_시간_안에_끝나면_경고가_없다(tmp_path):
    m, io = build(tmp_path)
    start(m)
    with Warnings() as w:
        m.finalize(5.0)
    assert not [t for t in w.texts if '아직 다 안 써졌다' in t]


def test_디스크가_멈춰도_종료는_취소가_먼저고_기록은_시간_안에_포기한다(tmp_path):
    gate = threading.Event()
    m, io = stalled(tmp_path, gate)
    order = []
    try:
        start(m)
        drive(m, 'SELECT')
        m.run_once()
        t0 = time.monotonic()
        m.shutdown()
        order.append('flag')
        assert m.halted() and time.monotonic() - t0 < 0.5      # ① 중단 플래그는 디스크를 안 기다린다
        order.append('cancel')                                 # ② 진행 중인 목표 취소 요청(node.cancel_active 자리)
        m.finalize(0.3)                                        # ③ 기록 마무리 — 0.3 초 안에 돌아온다
        assert time.monotonic() - t0 < 2.0 and order == ['flag', 'cancel']
        assert m.last_build['result'] == 'STOPPED'             # 요약은 메모리에서 이미 확정
    finally:
        gate.set()


def test_디스크가_멈춘_사이에_진행_중인_취소와_안전_콜백이_처리된다(tmp_path):
    """pick_place 를 기다리는 중(should_abort)에 정지 신호가 오면 디스크가 멈춰 있어도 바로 빠져나온다."""
    from d2_task.task_manager import wait_until
    gate = threading.Event()

    def waits_for_stop(io):
        return (True, '') if wait_until(threading.Event(), io.manager.halted) else (False, 'STOPPED')

    m, io = stalled(tmp_path, gate, pick_script=[waits_for_stop])
    try:
        start(m)
        drive(m, 'PICK_PLACE')
        timer = threading.Timer(0.2, m.on_safety, [SAFE_STOP])
        t0 = time.monotonic()
        timer.start()
        m.run_once()
        timer.join()
        assert time.monotonic() - t0 < 2.0 and m.state == 'STOPPED'
    finally:
        gate.set()


def test_파일을_못_쓰는_폴더에서도_조립은_끝까지(tmp_path):
    blocker = tmp_path / '파일'
    blocker.write_text('폴더가 아니다', encoding='utf-8')
    m, io = build(blocker / '하위')
    start(m)
    drive(m, 'DONE')
    assert m.run_id and m.last_build['placed'] == 11
    assert m.logger.flush(5.0) and not m.logger.file_ok


def test_기록이_중간에_망가져도_정지와_복구는_그대로(tmp_path):
    class Dies:
        calls = 0

        def write(self, _text):
            Dies.calls += 1
            if Dies.calls > 6:
                raise OSError('디스크 오류')

        def flush(self):
            pass

        def close(self):
            pass

    io = FakeIO()
    log = logger(tmp_path, opener=lambda *a, **k: Dies())
    m = TaskManager(CFG, io, log)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    io.pick_script = [(False, 'STOPPED')]
    assert m.command('select_design', 'bench') == (True, '')
    start(m)
    drive(m, 'STOPPED')
    assert io.states[-1]['message_id'] == 'stopped'
    m.on_safety(SAFE_OK)
    drive(m, 'DONE')
    assert m.last_build['stop_count'] == 1 and m.last_build['placed'] == 11
    assert log.flush(5.0) and not log.file_ok


def test_취소_뒤에_기록이_실패해도_취소는_된다(tmp_path):
    gate = threading.Event()
    m, io = stalled(tmp_path, gate, pick_script=[(False, 'PLAN_FAILED')] * 2)
    try:
        start(m)
        drive(m, 'ERROR')
        t0 = time.monotonic()
        m.on_intent('cancel')
        assert m.state == 'IDLE' and m.last_build['result'] == 'ERROR' and time.monotonic() - t0 < 1.0
    finally:
        gate.set()

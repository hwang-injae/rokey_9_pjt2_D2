# -*- coding: utf-8 -*-
"""기록기 RunLogger — 조립 한 번(run_id)의 CSV 기록과 결과 요약(build/1)을 만든다 (ROS 없이 동작, W065, SDD 6.8 · IRD 6장).

TaskManager 가 기록할 때를 정하고 이 클래스는 받아 적기만 한다(기록기 노드는 없다 — IRD 3장).
CSV 칸: run_id · design_id · time · module · kind · block_id · value (SDD 기록 칸). 오차는 mm(파일은 mm, NFR-12).
저장 위치는 부르는 쪽(task 노드 파라미터 log_dir)이 정한다. 비어 있으면 파일은 쓰지 않고 run_id · 요약만 만든다.
저장소 안에는 쓰지 않는다 — 부르는 쪽이 저장소 밖 경로를 준다.

**미전송 build/1 요약은 log_dir/pending_builds/<run_id>.json 으로도 보관한다**(save_pending · delete_pending · load_pending, W119).
같은 폴더의 임시 파일에 쓰고 flush · fsync 한 뒤 os.replace 로 완성본을 만든다. CSV 의 file_ok 와는 따로 실패를 다룬다.

**파일 쓰기는 전용 스레드 하나가 한다.** 부르는 쪽(상태표 · 정지 · 취소 경로)은 메모리 값을 바꾸고 줄을 큐에 넣기만 하므로
디스크가 느리거나 멈춰도 정지 · 취소 처리가 기다리지 않는다. run_id 와 build/1 요약은 메모리에서 바로 확정된다.
"""
import csv
import json
import logging
import math
import os
import queue
import secrets
import threading
import time

LOG = logging.getLogger('d2_task')

COLUMNS = ('run_id', 'design_id', 'time', 'module', 'kind', 'block_id', 'value')
SCHEMA_BUILD = 'build/1'
RESULTS = ('DONE', 'STOPPED', 'ERROR')     # IRD 6장 build/1 result
# 쓰기 스레드가 막혔을 때 메모리가 끝없이 늘지 않게 하는 안전 한도. 조립 한 번이 수백 줄이라 정상일 때는 닿지 않는다
QUEUE_LIMIT = 10000


def _finite_or_none(v):
    """측정값 m → 그대로. 못 잰 값(None · NaN · 숫자 아님)은 None — 0 으로 만들지 않는다."""
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        return None
    return float(v)


class RunLogger:
    """조립 run 하나의 CSV 와 결과 요약. 한 번에 run 하나만 연다(새 조립은 start_run 으로 새 run_id).

    입력: log_dir(저장 폴더 — `~` 는 홈으로 바꾼다. 빈 값이면 파일 없음), clock(시계 — 시험용), opener(파일 열기 — 시험용),
    new_suffix(run_id 끝의 무작위 16진 4자를 만드는 함수 — 시험용).
    run_id: `R<YYYYMMDD>_<HHMMSS>_<무작위 16진 4자>`(IRD 2장, 10/7 PL) 예 R20261010_143512_a3f9. 시각순으로 정렬되고,
    두 PC 가 같은 초에 시작해도 무작위 4자로 구분한다. 한 프로그램 안에서는 이미 쓴 ID 를 다시 만들지 않고
    (같은 초에 연달아 시작해도 겹치지 않는다), 같은 이름의 CSV 가 폴더에 있으면 덮어쓰지 않는다.
    바깥 영향: log_dir 아래 `<run_id>.csv` 쓰기(쓰기 스레드가). 그 밖은 없다. 같은 이름의 파일은 덮어쓰지 않는다.
    실패: 폴더를 못 만들거나 쓰다가 실패하면 로그 한 번 남기고 그 run 의 파일 기록만 끈다 — 예외를 밖으로 내지 않고
    부르는 쪽은 기다리지도 않는다. run_id 와 요약은 파일이 꺼져도 계속 만든다. 여러 스레드에서 불러도 된다.
    """

    def __init__(self, log_dir='', clock=time.time, opener=open, new_suffix=lambda: secrets.token_hex(2)):
        """아직 run 이 없는 상태로 시작한다. 쓰기 스레드는 첫 start_run 때(log_dir 이 있을 때만) 뜬다."""
        self.log_dir = os.path.expanduser(log_dir) if log_dir else ''
        self.clock, self._opener, self._new_suffix = clock, opener, new_suffix
        self._used_ids = set()               # 이 프로그램에서 이미 쓴 run_id
        self.run_id = None
        self.design_id = None
        self.file_ok = False                 # 지금 run 의 CSV 를 쓰고 있나(쓰기 스레드가 실패하면 꺼진다)
        self._lock = threading.Lock()        # 메모리 값만 지킨다. 파일 I/O 는 이 잠금 안에서 하지 않는다
        self._queue = None
        self._thread = None
        self._active = False
        self._started = 0.0
        self._total = 0
        self._stop_count = 0
        self._placed = {}                    # block_id → {dx_m, dy_m, dz_m} (놓은 순서 유지, 값은 마지막 측정)

    @property
    def enabled(self):
        """파일 기록을 쓰는 설정인가(log_dir 이 있다). 켜져 있어도 쓰다가 실패하면 file_ok 가 꺼진다."""
        return bool(self.log_dir)

    @property
    def active(self):
        """run 이 열려 있나(start_run 뒤 finish 전)."""
        return self._active

    # ---------- run 열기 · 닫기 ----------
    def start_run(self, design_id, total):
        """새 run 을 연다. 반환: run_id. 파일 열기는 쓰기 스레드가 하므로 기다리지 않는다.

        이미 열린 run 이 있으면 먼저 STOPPED 로 닫는다(새 조립은 새 ID).
        """
        if self._active:
            self.finish('STOPPED')
        with self._lock:
            self.design_id, self._total = design_id, total
            self._stop_count, self._placed = 0, {}
            self._started = self.clock()
            self.run_id = self._make_run_id()
            self._active = True
            self.file_ok = self.enabled
            if self.file_ok:
                self._ensure_thread()
                self._put(('open', self.run_id, os.path.join(self.log_dir, f'{self.run_id}.csv')))
            run_id = self.run_id
        self.log('start', None, total)
        return run_id

    def finish(self, result):
        """run 을 닫고 build/1 요약(dict)을 돌려준다. result = DONE · STOPPED · ERROR. 이미 닫혔으면 None.

        요약: run_id · design_id · result · placed(놓은 블록 수) · total · duration_s · stop_count ·
        blocks[block_id · dz_m · dx_m · dy_m(m, 못 잰 값 null)]. 요약은 메모리에서 바로 확정한다(파일 I/O 없음).
        CSV 끝 줄(end)과 닫기는 쓰기 스레드가 이어서 한다.
        """
        if result not in RESULTS:
            raise ValueError(f'result 는 {RESULTS} 중 하나 ({result!r})')
        with self._lock:
            if not self._active:
                return None
            self._row('end', None, result)
            summary = {'schema': SCHEMA_BUILD, 'run_id': self.run_id, 'design_id': self.design_id, 'result': result,
                       'placed': len(self._placed), 'total': self._total,
                       'duration_s': round(self.clock() - self._started, 3), 'stop_count': self._stop_count,
                       'blocks': [{'block_id': b, 'dz_m': m['dz_m'], 'dx_m': m['dx_m'], 'dy_m': m['dy_m']}
                                  for b, m in self._placed.items()]}
            self._active = False
            if self._thread is not None:
                self._put(('close', self.run_id))
            return summary

    def flush(self, timeout_s=2.0):
        """지금까지 넣은 줄을 쓰기 스레드가 **다 처리할** 때까지 timeout_s 초 안에서 기다린다.

        반환: 다 처리했으면 True, 시간 안에 못 했으면(또는 큐가 가득 차 표시를 못 넣었으면) False.
        **True 가 파일 저장 성공은 아니다** — 쓰다가 실패한 줄도 '처리한 것'이다. 저장이 잘 됐는지는
        file_ok(지금 run 의 파일 기록이 켜져 있나)로 따로 본다. 끝낼 때 · 시험에서만 쓴다(상태표는 부르지 않는다).
        파일 기록을 안 쓰면 바로 True.
        """
        if self._thread is None:
            return True
        done = threading.Event()
        try:
            self._queue.put_nowait(('barrier', done))
        except queue.Full:
            return False
        return done.wait(timeout_s)

    # ---------- 적기 ----------
    def log(self, kind, block_id=None, value=None, module='task'):
        """CSV 한 줄(큐에 넣기만 한다). run 이 없거나 파일이 꺼져 있으면 아무것도 안 한다. 실패해도 예외를 내지 않는다."""
        with self._lock:
            self._row(kind, block_id, value, module)

    def record_placed(self, block_id, slot=None):
        """블록 하나를 놓았다(pick_place 성공). 결과 요약의 placed · blocks 에 들어간다. 오차는 아직 안 잼."""
        with self._lock:
            if not self._active:
                return
            self._placed.setdefault(block_id, {'dx_m': None, 'dy_m': None, 'dz_m': None})
            self._row('placed', block_id, '' if slot is None else f'slot {slot}')

    def record_measure(self, block_id, dx_m, dy_m, dz_m):
        """놓은 블록의 배치 확인 측정값(m)을 적는다. 못 잰 값(NaN · None)은 CSV 에 빈 칸, 요약에 null — 0 으로 만들지 않는다.

        CSV 는 mm 로 쓴다(dx_mm · dy_mm · dz_mm 세 줄). 요약(build/1)은 m 그대로.
        """
        with self._lock:
            if not self._active:
                return
            vals = {'dx_m': _finite_or_none(dx_m), 'dy_m': _finite_or_none(dy_m), 'dz_m': _finite_or_none(dz_m)}
            if block_id in self._placed:
                self._placed[block_id] = vals
            for key in ('dx_m', 'dy_m', 'dz_m'):
                v = vals[key]
                self._row(key[:2] + '_mm', block_id, '' if v is None else f'{v * 1000:.3f}')

    def count_stop(self):
        """정지가 한 번 있었다(build/1 stop_count)."""
        with self._lock:
            if self._active:
                self._stop_count += 1

    def clear(self):
        """run_id 를 비운다(새 설계를 고르거나 IDLE 로 돌아갈 때). 열린 run 이 있으면 STOPPED 로 닫는다."""
        if self._active:
            self.finish('STOPPED')
        with self._lock:
            self.run_id = None

    def _make_run_id(self):
        """(잠금 안) R<날짜>_<시각>_<무작위 4자>. 이 프로그램에서 이미 쓴 것과 같으면 무작위 4자를 다시 뽑는다."""
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(self._started))
        while True:
            run_id = f'R{stamp}_{self._new_suffix()}'
            if run_id not in self._used_ids:
                self._used_ids.add(run_id)
                return run_id

    # ---------- 미전송 요약 보관 (log_dir/pending_builds) ----------
    @property
    def pending_dir(self):
        """미전송 요약 폴더. log_dir 이 비어 있으면 빈 글자."""
        return os.path.join(self.log_dir, 'pending_builds') if self.log_dir else ''

    def save_pending(self, run_id, summary, on_done):
        """요약을 디스크에 보관하라고 쓰기 스레드에 맡긴다(기다리지 않는다). 반환: 맡겼으면 True.

        끝나면 쓰기 스레드가 on_done(ok)를 부른다(ok = 완성본이 디스크에 있다). 파일 기록이 꺼져 있거나 큐가 가득 차 못 맡기면
        False — on_done 은 안 불린다(부르는 쪽이 디스크 없이 진행). 실패는 CSV 의 file_ok 와 상관없다.
        """
        if not self.enabled:
            return False
        with self._lock:
            self._ensure_thread()
            try:
                self._queue.put_nowait(('pend_save', run_id, summary, on_done))
                return True
            except queue.Full:
                LOG.warning('기록 쓰기가 밀려 미전송 요약 %s 을 디스크에 못 맡긴다', run_id)
                return False

    def delete_pending(self, run_id):
        """저장이 확인된 run_id 의 보관 파일 삭제를 쓰기 스레드에 맡긴다. 다른 run 의 파일은 건드리지 않는다. 기다리지 않는다."""
        if not self.enabled:
            return
        with self._lock:
            self._ensure_thread()                    # 재시작 뒤 복원한 요약을 지울 때는 아직 스레드가 없다
            try:
                self._queue.put_nowait(('pend_del', run_id))
            except queue.Full:
                LOG.warning('기록 쓰기가 밀려 %s 의 보관 파일을 못 지웠다 — 다음 시작 때 다시 전송될 수 있다', run_id)

    def load_pending(self):
        """시작할 때 한 번: 보관된 미전송 요약을 읽는다. 반환: [(run_id, 요약 dict)] (파일 이름순).

        완성본(<run_id>.json)만 읽고 임시 파일(.tmp)은 무시한다. schema · run_id(파일 이름과 같음) · 필수 칸 · 자료형이 틀린 파일은
        지우지 않고 경고만 남기고 건너뛴다. 폴더가 없거나 log_dir 이 비어 있으면 빈 목록(파일 I/O 오류도 경고 뒤 빈 목록).
        """
        found = []
        folder = self.pending_dir
        if not folder or not os.path.isdir(folder):
            return found
        try:
            names = sorted(os.listdir(folder))
        except OSError as e:
            LOG.warning('미전송 요약 폴더를 못 읽는다(%s): %r', folder, e)
            return found
        for name in names:
            if not name.endswith('.json'):
                continue
            path = os.path.join(folder, name)
            try:
                with open(path, encoding='utf-8') as f:
                    summary = json.loads(f.read(), parse_constant=_reject_constant)
                why = _invalid_summary(summary, name[:-len('.json')])
            except (OSError, ValueError) as e:
                why = f'읽을 수 없다: {e!r}'
            if why:
                LOG.warning('미전송 요약 파일을 건너뛴다(지우지 않음) %s — %s', path, why)
            else:
                found.append((summary['run_id'], summary))
        return found

    def _pending_op(self, item):
        """(쓰기 스레드) 미전송 요약 파일 저장 · 삭제. 실패해도 CSV 쪽 상태는 건드리지 않는다."""
        ok = False
        try:
            folder = self.pending_dir
            final = os.path.join(folder, f'{item[1]}.json')
            if item[0] == 'pend_save':
                os.makedirs(folder, exist_ok=True)
                tmp = final + '.tmp'
                try:
                    f = self._opener(tmp, 'w', encoding='utf-8')
                    try:
                        f.write(json.dumps(item[2], ensure_ascii=False, allow_nan=False))
                        f.flush()
                        if hasattr(f, 'fileno'):
                            os.fsync(f.fileno())
                    finally:
                        f.close()
                    os.replace(tmp, final)
                    ok = True
                except BaseException:
                    try:
                        os.remove(tmp)
                    except OSError:
                        pass
                    raise
            else:
                try:
                    os.remove(final)
                except FileNotFoundError:
                    pass
                ok = True
        except Exception as e:   # noqa: BLE001
            LOG.warning('미전송 요약 파일 %s 실패(run %s) — %r%s', '저장' if item[0] == 'pend_save' else '삭제', item[1], e,
                        ' — 재시작 복구를 보장하지 않는다' if item[0] == 'pend_save' else ' — 다음 시작 때 다시 전송될 수 있다')
        if item[0] == 'pend_save':
            try:
                item[3](ok)
            except Exception:   # noqa: BLE001
                LOG.exception('미전송 요약 저장 완료 알림 실패')

    # ---------- 큐 · 쓰기 스레드 (잠금 안에서 부른다) ----------
    def _row(self, kind, block_id, value, module='task'):
        """(잠금 안) 줄 하나를 시각과 함께 큐에 넣는다."""
        if not self._active or not self.file_ok:
            return
        now = self.clock()
        stamp = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(now))
        self._put(('row', self.run_id, [self.run_id, self.design_id, f'{stamp}.{int(now % 1 * 1000):03d}', module, kind,
                           '' if block_id is None else block_id, '' if value is None else value]))

    def _put(self, item):
        """(잠금 안) 큐에 넣는다(기다리지 않는다). 가득 찼으면 줄을 못 넣은 것이므로 이 run 의 파일 기록을 끄고 False."""
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            if self.file_ok:
                LOG.warning('기록 쓰기가 밀려 있어 이 조립의 파일 기록을 끈다(큐 %d줄)', QUEUE_LIMIT)
            self.file_ok = False
            return False

    def _ensure_thread(self):
        """(잠금 안) 쓰기 스레드가 없으면 띄운다. 데몬이라 프로그램이 끝나는 것을 막지 않는다."""
        if self._thread is None:
            self._queue = queue.Queue(maxsize=QUEUE_LIMIT)
            self._thread = threading.Thread(target=self._write_loop, name='run_logger', daemon=True)
            self._thread.start()

    def _write_loop(self):
        """쓰기 스레드: 큐에서 꺼내 순서대로 파일에 쓴다. 파일 I/O 는 모두 여기서, 잠금 밖에서 한다.

        큐의 open · row · close 에는 run_id 가 붙어 있다. 지금 열린 파일(cur)의 run 이 아닌 줄은 버린다 — 파일을 못 연 run 의
        줄이 다른 run 의 파일에 섞이지 않는다. 실패하면 그 run 의 파일만 닫고, **지금 진행 중인 run 일 때만** file_ok 를 끈다
        (이전 run 의 늦은 오류가 새 run 의 기록을 멈추지 않게 확인 · 변경을 잠금으로 묶는다). 스레드는 계속 돈다.
        """
        f = writer = cur = None
        while True:
            item = self._queue.get()
            try:
                kind = item[0]
                if kind in ('pend_save', 'pend_del'):
                    self._pending_op(item)
                elif kind == 'barrier':
                    item[1].set()
                elif kind == 'open':
                    if f is not None:
                        f.close()
                    f = writer = cur = None
                    os.makedirs(os.path.dirname(item[2]), exist_ok=True)
                    f = self._opener(item[2], 'x', newline='', encoding='utf-8')     # x: 이미 있는 파일은 덮어쓰지 않는다
                    writer = csv.writer(f)
                    cur = item[1]
                    writer.writerow(COLUMNS)
                    f.flush()
                elif kind == 'row':
                    if cur == item[1]:
                        writer.writerow(item[2])
                        f.flush()
                elif kind == 'close':
                    if cur == item[1]:
                        f.close()
                        f = writer = cur = None
            except Exception as e:   # noqa: BLE001 — 기록 실패가 로봇 · 상태표로 번지지 않는다
                LOG.warning('기록 파일을 끈다(run %s) — %r', item[1] if item[0] != 'barrier' else '-', e)
                try:
                    if f is not None:
                        f.close()
                except Exception:   # noqa: BLE001
                    pass
                f = writer = cur = None
                if item[0] != 'barrier':
                    with self._lock:
                        if self.run_id == item[1]:
                            self.file_ok = False


def _reject_constant(name):
    """json.loads 가 NaN · Infinity 를 만나면 오류로(표준 JSON 이 아니다)."""
    raise ValueError(f'유한하지 않은 수({name})')


def _num_or_none(v):
    """측정값 칸: None 이거나 유한한 숫자(bool 아님)."""
    return v is None or (isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v))


def _invalid_summary(s, stem):
    """보관된 build/1 요약이 올바른가. 올바르면 빈 글자, 아니면 이유. stem = 파일 이름(확장자 뺀 것) = run_id."""
    if not isinstance(s, dict):
        return '객체가 아니다'
    if s.get('schema') != SCHEMA_BUILD:
        return f'schema 가 {SCHEMA_BUILD} 가 아니다'
    if s.get('run_id') != stem:
        return 'run_id 가 파일 이름과 다르다'
    for key in ('design_id', 'result'):
        if not isinstance(s.get(key), str) or not s[key]:
            return f'{key} 가 글자가 아니다'
    if s['result'] not in RESULTS:
        return f'result 가 {RESULTS} 가 아니다'
    for key in ('placed', 'total', 'stop_count'):
        if not isinstance(s.get(key), int) or isinstance(s[key], bool) or s[key] < 0:
            return f'{key} 가 0 이상 정수가 아니다'
    if not _num_or_none(s.get('duration_s')) or s.get('duration_s') is None:
        return 'duration_s 가 숫자가 아니다'
    if not isinstance(s.get('blocks'), list):
        return 'blocks 가 목록이 아니다'
    for b in s['blocks']:
        if not isinstance(b, dict) or not isinstance(b.get('block_id'), str) or \
                not all(k in b and _num_or_none(b[k]) for k in ('dz_m', 'dx_m', 'dy_m')):
            return 'blocks 항목이 올바르지 않다'
    return ''

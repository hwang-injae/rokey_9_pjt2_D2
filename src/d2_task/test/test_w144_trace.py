# -*- coding: utf-8 -*-
"""W144 원인 추적 기록 W144Trace 시험 — events.jsonl 칸 · rosbag 시작/종료 · manifest (ROS · 실제 녹화 없이, 가짜 프로세스)."""
import json
import signal

from d2_task import w144_trace
from d2_task.w144_trace import W144Trace, jsonable


class FakeProc:
    """ros2 bag record 대신 — SIGINT 를 받으면 metadata.yaml 을 쓰고 끝난다(정상 종료 흉내). hang=True 면 안 끝남."""

    def __init__(self, bag_dir, hang=False):
        self.pid, self.bag_dir, self.hang, self.returncode, self.signals = 4242, bag_dir, hang, None, []

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.hang and signal.SIGKILL not in self.signals:
            import subprocess
            raise subprocess.TimeoutExpired('ros2', timeout)
        self.returncode = 0 if signal.SIGKILL not in self.signals else -9
        return self.returncode


def make(tmp_path):
    folder = tmp_path / 'w144_X_1013_150000'
    folder.mkdir()
    return W144Trace(folder, 'R1', lambda: 123456789)


def test_이벤트_한_줄에_연결_키와_두_시각이_있다(tmp_path):
    t = make(tmp_path)
    t.event('pick_requested', part_id='P1', action_id='action_0001', sequence=3, retry_of=None, pick_pose=((0.1, 0.2, 0.3), (0, 0, 0, 1)),
            bad=float('nan'))
    row = json.loads((t.folder / 'events.jsonl').read_text().splitlines()[0])
    assert {k: row[k] for k in ('run_id', 'action_id', 'sequence', 'part_id', 'retry_of', 'event', 'ros_stamp_ns')} == \
        {'run_id': 'R1', 'action_id': 'action_0001', 'sequence': 3, 'part_id': 'P1', 'retry_of': None, 'event': 'pick_requested',
         'ros_stamp_ns': 123456789}
    assert isinstance(row['unix_stamp_ns'], int) and row['pick_pose'] == [[0.1, 0.2, 0.3], [0, 0, 0, 1]] and row['bad'] is None


def test_bag_은_폴더_옆에_tf_static_은_transient_local_로_녹화한다(tmp_path):
    t = make(tmp_path)
    seen = {}

    def popen(cmd, **kw):
        seen['cmd'], seen['kw'] = cmd, kw
        return FakeProc(t.bag_dir)

    ok, why = t.start_bag(['/a', '/tf_static'], {'/a': {'type': ['x/Y']}}, popen=popen, free_gb=100)
    assert ok and why == ''
    assert seen['cmd'][:5] == ['ros2', 'bag', 'record', '-o', str(t.bag_dir)] and t.bag_dir.parent == t.folder.parent
    assert '--qos-profile-overrides-path' in seen['cmd'] and 'transient_local' in (t.folder / 'bag_qos_overrides.yaml').read_text()
    assert seen['kw']['start_new_session'] and seen['kw']['env']['RCUTILS_LOGGING_BUFFERED_STREAM'] == '0'
    m = json.loads((t.folder / 'manifest.json').read_text())
    assert m['bag']['status'] == 'recording' and m['bag']['topics'] == ['/a', '/tf_static'] and m['disk_free_gb_at_start'] == 100


def test_디스크가_모자라면_녹화를_시작하지_않는다(tmp_path):
    t = make(tmp_path)
    ok, why = t.start_bag(['/a'], {}, popen=lambda *a, **k: (_ for _ in ()).throw(AssertionError('띄우면 안 됨')),
                          free_gb=w144_trace.MIN_FREE_GB - 1)
    assert not ok and '디스크' in why and t.manifest['bag']['status'] == 'not_started'


def test_정상_종료면_complete_metadata_가_없으면_incomplete(tmp_path, monkeypatch):
    t = make(tmp_path)
    t.start_bag(['/a'], {}, popen=lambda cmd, **kw: FakeProc(t.bag_dir), free_gb=100)

    def killpg(pid, sig):
        t.proc.signals.append(sig)
        if sig == signal.SIGINT:
            t.bag_dir.mkdir(exist_ok=True)
            (t.bag_dir / 'metadata.yaml').write_text('x')
            (t.bag_dir / 'b_0.mcap').write_text('x')
    monkeypatch.setattr(w144_trace.os, 'killpg', killpg)
    assert t.stop_bag() is True
    assert t.manifest['bag']['status'] == 'complete' and t.manifest['bag']['files'] == ['b_0.mcap', 'metadata.yaml']

    t2 = W144Trace(tmp_path / 'w144_Y', 'R2', lambda: 1)
    t2.folder.mkdir()
    t2.start_bag(['/a'], {}, popen=lambda cmd, **kw: FakeProc(t2.bag_dir), free_gb=100)
    monkeypatch.setattr(w144_trace.os, 'killpg', lambda pid, sig: t2.proc.signals.append(sig))
    assert t2.stop_bag('조립이 멈춤') is False
    assert t2.manifest['bag']['status'] == 'incomplete' and t2.manifest['bag']['reason'] == '조립이 멈춤'


def test_안_끝나는_녹화는_강제_종료하고_incomplete(tmp_path, monkeypatch):
    t = make(tmp_path)
    t.start_bag(['/a'], {}, popen=lambda cmd, **kw: FakeProc(t.bag_dir, hang=True), free_gb=100)
    monkeypatch.setattr(w144_trace.os, 'killpg', lambda pid, sig: t.proc.signals.append(sig))
    monkeypatch.setattr(w144_trace, 'BAG_STOP_S', 0.01)
    assert t.stop_bag() is False
    assert t.manifest['bag']['status'] == 'incomplete' and '강제 종료' in t.manifest['bag']['reason']


def test_구독_확인은_로그의_토픽_이름으로(tmp_path):
    t = make(tmp_path)
    t.bag_log = t.folder / 'bag_record.log'
    t.bag_log.write_text("[INFO] Subscribed to topic '/a'\n")
    assert t.wait_subscribed(['/a', '/b'], timeout_s=0.0, sleep=lambda _s: None) == ['/b']


def test_jsonable_은_튜플_숫자_NaN_을_기본형으로():
    assert jsonable({'a': (1, 2.5, float('inf'))}) == {'a': [1, 2.5, None]}


def test_줄이기는_정상_bag_일_때만_따로_띄운다(tmp_path):
    t = make(tmp_path)
    seen = []
    assert t.start_reduce(popen=lambda cmd, **kw: seen.append((cmd, kw))) is None and seen == []      # 녹화 안 함 — not_started
    t.manifest['bag'] = {'status': 'complete'}
    cmd = t.start_reduce(popen=lambda cmd, **kw: seen.append((cmd, kw)))
    assert cmd[1:3] == ['-m', 'd2_task.w144_reduce'] and '--delete-source' in cmd and str(t.bag_dir) in cmd
    assert seen[0][1]['start_new_session'] and json.loads((t.folder / 'manifest.json').read_text())['bag_reduce_started']['command'] == cmd

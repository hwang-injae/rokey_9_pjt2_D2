# -*- coding: utf-8 -*-
"""W144 원인 추적 기록 — events.jsonl · rosbag 녹화 프로세스 · manifest.json (ROS 없이 시험 가능, 10/13 #140 인계 구현).

한석형 #140 `tools/w144_capture/INTEGRATION.md`의 「배치별 원인 추적」 · 「rosbag 전체 기록」 요구를 정식 흐름(task 노드)에 붙인다.
- events.jsonl: 한 줄 = {run_id, action_id, sequence, part_id, retry_of, event, ros_stamp_ns, unix_stamp_ns, ...값}. 이름은 정식 코드의 part_id.
  이벤트: run_start · candidate_selected · pick_requested · place_result · observe_requested · observe_arrived · observation_saved ·
  attempt_failed · run_end (실제 단계 전환은 place_result 의 step 칸 — 피드백에 없는 단계 · 시각은 지어내지 않는다).
- rosbag: `ros2 bag record` 를 따로 띄워 첫 find_blocks 전에 시작, run_end 뒤 SIGINT 로 정상 종료하고 metadata.yaml 이 생겼는지 본다.
  정상 종료가 아니면 manifest 에 incomplete 와 이유를 남긴다(원자료는 지우지 않음).
- manifest.json: 녹화한 토픽 · 타입 · 발행 QoS · bag 명령 · 디스크 여유 · 시작/끝 시각 · 상태 · 실제 읽은 설정 파일과 해시.
로봇 · ROS 메시지에는 손대지 않는다(파일 쓰기와 녹화 프로세스만). 파일 쓰기 실패는 예외로 올린다 — 부르는 쪽이 오류 · 정지 경로로 보낸다.
"""
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

BAG_STOP_S = 30.0          # SIGINT 뒤 녹화 프로세스가 끝나고 metadata 를 쓸 때까지 기다리는 시간 — 넘으면 강제 종료 · incomplete
BAG_READY_S = 10.0         # 녹화 프로세스가 필수 토픽을 다 구독했다고 출력할 때까지 기다리는 시간
MIN_FREE_GB = 20.0         # 시작 때 디스크 여유가 이보다 적으면 녹화하지 않는다(카메라 원본 색 + 깊이 약 46 MB/s — 10분 약 27 GB, 계산값)


def sha256(path):
    """파일 sha256 16진 글자 또는 None(없음)."""
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def jsonable(v):
    """튜플 · numpy 수 등을 JSON 기본형으로(NaN · inf 는 None). 기록용 복사본을 만든다."""
    if isinstance(v, dict):
        return {str(k): jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [jsonable(x) for x in v]
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    if f != f or f in (float('inf'), float('-inf')):
        return None
    return int(f) if isinstance(v, int) else f


class W144Trace:
    """한 조립(run)의 events.jsonl · rosbag · manifest 를 맡는다.

    입력: folder = 촬영 폴더(PlacementRecorder 와 같은 곳 — events.jsonl · manifest.json 이 여기), run_id, ros_ns = ROS 시각(ns)을 주는 함수.
    bag 은 folder 옆 '<폴더 이름>_bag' 에 둔다 — 촬영기 ZIP(사진 전달용)에 수십 GB bag 이 같이 압축되지 않게. manifest 에 경로를 적는다.
    """

    def __init__(self, folder, run_id, ros_ns):
        self.folder, self.run_id, self.ros_ns = Path(folder), run_id, ros_ns
        self.events_path = self.folder / 'events.jsonl'
        self.bag_dir = self.folder.parent / (self.folder.name + '_bag')
        self.proc, self.bag_log = None, None
        self.manifest = {'schema': 'w144_manifest/1', 'run_id': run_id, 'part_id_name': 'part_id(PLM 정식 실행기)',
                         'events': self.events_path.name, 'bag_dir': str(self.bag_dir), 'bag': {'status': 'not_started'},
                         'units': {'length': 'm', 'quaternion': 'xyzw', 'frame': 'base_link', 'stamp': 'ns'}}

    # ---------------- events ----------------
    def event(self, event, part_id=None, action_id=None, sequence=None, retry_of=None, **data):
        """events.jsonl 한 줄을 더한다(바로 flush). 반환: 그 줄 dict. 파일 쓰기 실패는 OSError."""
        row = {'run_id': self.run_id, 'action_id': action_id, 'sequence': sequence, 'part_id': part_id, 'retry_of': retry_of,
               'event': event, 'ros_stamp_ns': int(self.ros_ns()), 'unix_stamp_ns': time.time_ns()}
        row.update(jsonable(data))
        with open(self.events_path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
            f.flush()
        return row

    # ---------------- manifest ----------------
    def save_manifest(self):
        """manifest.json 을 (임시 파일 → 바꾸기로) 쓴다."""
        tmp = self.folder / 'manifest.json.tmp'
        tmp.write_text(json.dumps(jsonable(self.manifest), ensure_ascii=False, indent=2, allow_nan=False))
        tmp.replace(self.folder / 'manifest.json')

    def add_files(self, paths):
        """실제로 읽은 설정 파일들의 경로 · sha256 을 manifest 'loaded_files' 에 적는다(동봉 파일이 실제 로드 파일인지 확인용)."""
        self.manifest['loaded_files'] = [{'path': str(Path(p).resolve()), 'sha256': sha256(p)} for p in paths if Path(p).is_file()]

    # ---------------- rosbag ----------------
    def bag_command(self, topics, qos_path):
        """ros2 bag record 명령(목록). topics = 녹화 토픽 이름, qos_path = QoS 덮어쓰기 YAML(/tf_static transient-local) 또는 None."""
        cmd = ['ros2', 'bag', 'record', '-o', str(self.bag_dir), '--topics', *topics]
        if qos_path:
            cmd += ['--qos-profile-overrides-path', str(qos_path)]
        return cmd

    def start_bag(self, topics, topic_info, popen=subprocess.Popen, free_gb=None):
        """녹화를 시작한다. topic_info = {이름: {'type', 'publishers_qos'}}(manifest 용). 반환: (시작했나, 이유).
        디스크 여유가 MIN_FREE_GB 보다 적으면 시작하지 않는다. 바깥 영향: 프로세스 하나 · bag 폴더 · 로그 파일."""
        free = shutil.disk_usage(self.folder).free / 1e9 if free_gb is None else free_gb
        self.manifest['disk_free_gb_at_start'] = round(free, 1)
        if free < MIN_FREE_GB:
            self.manifest['bag'] = {'status': 'not_started', 'reason': f'디스크 여유 {free:.0f} GB < {MIN_FREE_GB:.0f} GB'}
            self.save_manifest()
            return False, self.manifest['bag']['reason']
        qos_path = None
        if '/tf_static' in topics:
            qos_path = self.folder / 'bag_qos_overrides.yaml'
            qos_path.write_text('/tf_static:\n  durability: transient_local\n  reliability: reliable\n  history: keep_all\n')
        cmd = self.bag_command(topics, qos_path)
        self.bag_log = self.folder / 'bag_record.log'
        log = open(self.bag_log, 'w')
        env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='0')     # 'Subscribed to topic' 줄이 파일에 바로 보이게(파일이면 기본이 버퍼)
        self.proc = popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        self.manifest['bag'] = {'status': 'recording', 'command': cmd, 'topics': list(topics), 'topic_info': topic_info,
                                'started_unix_ns': time.time_ns(), 'started_ros_ns': int(self.ros_ns())}
        self.save_manifest()
        return True, ''

    def wait_subscribed(self, topics, timeout_s=BAG_READY_S, sleep=time.sleep):
        """녹화 프로세스 로그에 필수 토픽마다 'Subscribed to topic' 이 나올 때까지 기다린다. 반환: 아직 구독 안 된 토픽 목록(빈 = 다 됨)."""
        end = time.monotonic() + timeout_s
        left = list(topics)
        while True:
            text = self.bag_log.read_text(errors='replace') if self.bag_log and self.bag_log.exists() else ''
            left = [t for t in topics if f"'{t}'" not in text]
            if not left or time.monotonic() > end or (self.proc is not None and self.proc.poll() is not None):
                return left
            sleep(0.2)

    def start_reduce(self, popen=subprocess.Popen):
        """정상으로 닫힌 bag 을 줄이는 프로세스(d2_task.w144_reduce — 카메라 초당 2장 · 색 PNG, 확인되면 원본 지움)를 따로 띄운다.
        녹화 중에 솎지 않는 것은 색 · 깊이 짝 시각이 어긋나지 않게(10/13 한석형 허락 · 박진용: 56 GB → 1 GB). 노드가 꺼져도 끝까지 돈다(새 세션).
        반환: 띄운 명령(목록) 또는 None(bag 이 complete 가 아님). 결과는 <bag>_2hz/reduce_note.json · manifest 'bag_reduced'."""
        if self.manifest['bag'].get('status') != 'complete':
            return None
        cmd = [sys.executable, '-m', 'd2_task.w144_reduce', str(self.bag_dir), '--delete-source', '--manifest', str(self.folder / 'manifest.json')]
        log = open(self.folder / 'bag_reduce.log', 'w')
        popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.manifest['bag_reduce_started'] = {'command': cmd, 'unix_ns': time.time_ns()}
        self.save_manifest()
        return cmd

    def stop_bag(self, reason=''):
        """녹화를 정상 종료(SIGINT → 기다림)하고 상태를 manifest 에 남긴다. 반환: complete 인가.
        metadata.yaml 이 없거나 프로세스가 비정상으로 끝나면 incomplete(이유 포함). 이미 끝났거나 시작 안 했으면 그대로."""
        bag = self.manifest['bag']
        if self.proc is None:
            return bag.get('status') == 'complete'
        try:
            if self.proc.poll() is None:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout=BAG_STOP_S)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGKILL)
            self.proc.wait()
            reason = (reason + ' · ' if reason else '') + f'녹화 프로세스가 {BAG_STOP_S:.0f} s 안에 안 끝나 강제 종료'
        except ProcessLookupError:
            pass
        code = self.proc.returncode
        meta = self.bag_dir / 'metadata.yaml'
        complete = meta.exists() and not reason
        bag.update(status='complete' if complete else 'incomplete', returncode=code, stopped_unix_ns=time.time_ns(),
                   stopped_ros_ns=int(self.ros_ns()), metadata=meta.exists(), reason=reason or (None if meta.exists() else 'metadata.yaml 없음'),
                   files=sorted(p.name for p in self.bag_dir.iterdir()) if self.bag_dir.exists() else [])
        self.proc = None
        self.save_manifest()
        return complete

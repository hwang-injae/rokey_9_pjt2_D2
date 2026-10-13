# -*- coding: utf-8 -*-
"""W144 rosbag 줄이기 — 카메라 초당 2장 · 색 영상 무손실 PNG · 나머지 토픽 전부 그대로(10/13 한석형 허락, 56 GB → 1 GB).

녹화는 원본 그대로 하고(색 · 깊이 짝을 녹화 중에 따로 솎으면 시각이 어긋난다) 녹화가 정상으로 닫힌 뒤 이 모듈이 새 bag(<원본>_2hz)을 만든다.
- 색 /…/color/image_raw: header 시각 기준 PERIOD_S 마다 한 장 → CompressedImage(/…/color/image_raw/compressed,
  image_transport 형식 '<원 인코딩>; png compressed bgr8', header 그대로).
- 정렬 깊이 /…/aligned_depth_to_color/image_raw · 색 camera_info: 고른 색과 header 시각이 PAIR_S 안인 것만, 원시 그대로.
- 그 밖의 토픽은 전부 그대로. 저장은 mcap + zstd 청크 압축(무손실).
--delete-source 면 새 bag 의 metadata 가 있고 카메라 밖 토픽 수가 원본과 같을 때만 원본 폴더를 지운다(아니면 둘 다 남김).
task 노드가 W144 기록을 닫을 때 따로 띄운다: python3 -m d2_task.w144_reduce <원본 bag> --delete-source --manifest <manifest.json>
"""
import argparse
import collections
import json
import shutil
import sys
import time
from pathlib import Path

PERIOD_S = 0.5     # 초당 2장
PAIR_S = 0.02      # 색 영상과 깊이 · camera_info 를 같은 시각 짝으로 보는 차 (30 fps 한 칸 33 ms 보다 작게)


def stamp_of(data):
    """CDR 직렬 바이트에서 header.stamp(초) — 4바이트 머리 뒤 int32 sec · uint32 nanosec (메시지를 다 풀지 않음)."""
    return int.from_bytes(data[4:8], 'little', signed=True) + int.from_bytes(data[8:12], 'little') * 1e-9


class FramePicker:
    """색 · 깊이 · camera_info 흐름에서 남길 것을 고른다(ROS 없이 시험 가능).

    feed_color(stamp) → 남기면 True(색 시각을 기억). feed_pair(topic, stamp, item) → 지금 써야 할 item 목록(짝 색이 이미 왔으면 바로,
    아직이면 최근 목록에 두었다가 짝 색이 오면 pending_for(색 시각)이 돌려준다). 한 색 시각에 짝은 토픽마다 하나만."""

    def __init__(self, period_s=PERIOD_S, pair_s=PAIR_S, keep=20):
        self.period_s, self.pair_s = period_s, pair_s
        self.last = None
        self.kept = collections.deque(maxlen=keep)
        self.recent = collections.defaultdict(lambda: collections.deque(maxlen=keep))
        self.done = collections.defaultdict(set)

    def feed_color(self, stamp):
        if self.last is not None and stamp - self.last < self.period_s:
            return False
        self.last = stamp
        self.kept.append(stamp)
        return True

    def pending_for(self, cstamp):
        """색 시각 cstamp 의 짝으로 이미 와 있던 [(topic, item)]."""
        out = []
        for topic, q in self.recent.items():
            for st, item in list(q):
                if abs(st - cstamp) <= self.pair_s and cstamp not in self.done[topic]:
                    self.done[topic].add(cstamp)
                    q.remove((st, item))
                    out.append((topic, item))
                    break
        return out

    def feed_pair(self, topic, stamp, item):
        for cs in self.kept:
            if abs(cs - stamp) <= self.pair_s and cs not in self.done[topic]:
                self.done[topic].add(cs)
                return [item]
        self.recent[topic].append((stamp, item))
        return []


def reduce_bag(src, dst):
    """src bag → dst bag(위 규칙). 반환: 토픽별 쓴 수 dict. 바깥 영향: dst 폴더를 만든다. 실패는 예외."""
    import cv2
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import CompressedImage, Image

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(src), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t for t in reader.get_all_topics_and_types()}
    color = next(n for n in topics if n.endswith('/color/image_raw'))
    pairs = [n for n in topics if n.endswith('/aligned_depth_to_color/image_raw') or n.endswith('/color/camera_info')]
    color_c = color + '/compressed'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(dst), storage_id='mcap', storage_preset_profile='zstd_fast'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name, meta in topics.items():
        if name == color:
            writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=color_c, type='sensor_msgs/msg/CompressedImage',
                                                         serialization_format='cdr', offered_qos_profiles=meta.offered_qos_profiles))
        else:
            writer.create_topic(meta)
    pick = FramePicker()
    count = collections.Counter()
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == color:
            st = stamp_of(data)
            if not pick.feed_color(st):
                continue
            img = deserialize_message(data, Image)
            ch = 3 if img.encoding in ('rgb8', 'bgr8') else 1
            arr = np.frombuffer(bytes(img.data), np.uint8).reshape(img.height, img.step)[:, :img.width * ch].reshape(img.height, img.width, ch)
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if img.encoding == 'rgb8' else arr
            ok, png = cv2.imencode('.png', bgr)
            if not ok:
                raise RuntimeError('PNG 압축 실패')
            writer.write(color_c, serialize_message(CompressedImage(header=img.header, format=f'{img.encoding}; png compressed bgr8',
                                                                     data=png.tobytes())), t)
            count[color_c] += 1
            for tp, (d, tt) in pick.pending_for(st):
                writer.write(tp, d, tt)
                count[tp] += 1
        elif topic in pairs:
            for d, tt in pick.feed_pair(topic, stamp_of(data), (data, t)):
                writer.write(topic, d, tt)
                count[topic] += 1
        else:
            writer.write(topic, data, t)
            count[topic] += 1
    del writer                       # 닫아야 mcap 끝(요약) · metadata.yaml 이 써진다
    return dict(count)


def source_counts(src):
    """원본 metadata.yaml 의 토픽별 메시지 수 dict."""
    import yaml
    meta = yaml.safe_load((Path(src) / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    return {t['topic_metadata']['name']: int(t['message_count']) for t in meta['topics_with_message_count']}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('--delete-source', action='store_true')
    ap.add_argument('--manifest', default='')
    a = ap.parse_args(argv)
    src = Path(a.src).resolve()
    dst = src.with_name(src.name + '_2hz')
    note = {'source': str(src), 'output': str(dst), 'period_s': PERIOD_S, 'pair_s': PAIR_S, 'started_unix_s': time.time(),
            'color': '/…/color/image_raw → …/compressed (무손실 PNG, header 그대로)', 'depth_camera_info': '고른 색과 같은 시각 짝만, 원시 그대로',
            'others': '전부 그대로', 'storage': 'mcap zstd_fast(무손실)'}
    try:
        counts = reduce_bag(src, dst)
        orig = source_counts(src)
        cams = [n for n in orig if '/camera/' in n]
        same = all(counts.get(n) == c for n, c in orig.items() if n not in cams)
        ok = (dst / 'metadata.yaml').exists() and same
        note.update(counts=counts, source_counts=orig, verified=ok)
        if a.delete_source and ok:
            shutil.rmtree(src)
            note['source_deleted'] = True
        note['finished_unix_s'] = time.time()
    except Exception as e:                       # 줄이기가 실패해도 원본은 그대로 남는다
        note.update(error=repr(e), verified=False)
    if dst.exists():
        (dst / 'reduce_note.json').write_text(json.dumps(note, ensure_ascii=False, indent=2))
    if a.manifest and Path(a.manifest).exists():
        m = json.loads(Path(a.manifest).read_text())
        m['bag_reduced'] = {k: note.get(k) for k in ('output', 'verified', 'source_deleted', 'error', 'period_s')}
        Path(a.manifest).write_text(json.dumps(m, ensure_ascii=False, indent=2))
    print(json.dumps(note, ensure_ascii=False, indent=2))
    return 0 if note.get('verified') else 1


if __name__ == '__main__':
    sys.exit(main())

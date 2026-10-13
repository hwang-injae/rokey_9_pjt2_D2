"""W144 현실 기준 비교용 원자료 수집. 읽기 전용 ROS 구독·TF 조회와 로컬 저장만 수행한다."""
import argparse
import json
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def stamp(msg):
    """메시지 헤더 시각을 초로 변환한다. 외부 영향 없음."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def transform_record(tf):
    """TF의 원시 위치(m)·쿼터니언·시각·프레임을 저장 객체로 만든다. 외부 영향 없음."""
    t, q = tf.transform.translation, tf.transform.rotation
    return dict(parent=tf.header.frame_id, child=tf.child_frame_id,
                stamp_s=stamp(tf), translation_m=[t.x, t.y, t.z], quaternion_xyzw=[q.x, q.y, q.z, q.w])


class Capture(Node):
    """색·정렬 깊이·내참수와 촬영 시각의 TF를 저장한다. 로봇 명령·서비스·보정 변경 없음."""

    def __init__(self, args):
        """입력: CLI 설정. 토픽 구독·TF 수신 시작, 저장 실패는 예외로 중단한다."""
        super().__init__('w144_reference_capture')
        self.args = args
        self.bridge = CvBridge()
        self.colors = deque(maxlen=30)
        self.depths = deque(maxlen=30)
        self.info = None
        self.samples = []
        self.last_stamp = -1.0
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, args.prefix + '/color/image_raw', self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, args.prefix + '/aligned_depth_to_color/image_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.prefix + '/color/camera_info', self.on_info, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.save_pair)

    def on_color(self, msg):
        """색 영상 버퍼에 추가한다. 메시지·로봇 변경 없음."""
        self.colors.append(msg)

    def on_depth(self, msg):
        """정렬 깊이 영상 버퍼에 추가한다. 메시지·로봇 변경 없음."""
        self.depths.append(msg)

    def on_info(self, msg):
        """실제 색 카메라 내참수·왜곡을 보관한다. 외부 영향 없음."""
        self.info = msg

    def lookup(self, child, ts):
        """촬영 시각 base←child TF를 조회한다. 없으면 오류를 기록하며 최신 TF로 대체하지 않는다."""
        try:
            return dict(ok=True, value=transform_record(self.tf_buffer.lookup_transform(self.args.base, child, ts)))
        except Exception as exc:
            return dict(ok=False, error=str(exc))

    def save_pair(self):
        """새 색/깊이 쌍을 저장한다. 시각차·크기·인코딩 불일치는 거절, TF 누락은 명시해서 저장한다."""
        if self.info is None or not self.colors or not self.depths or len(self.samples) >= self.args.frames:
            return
        depth = self.depths[-1]
        ds = stamp(depth)
        if ds <= self.last_stamp:
            return
        color = min(self.colors, key=lambda m: abs(stamp(m) - ds))
        skew = abs(stamp(color) - ds)
        if skew > self.args.max_skew_ms / 1000:
            return
        d = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        c = self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
        if d.shape != c.shape[:2] or d.shape != (self.info.height, self.info.width):
            raise ValueError('색/정렬 깊이/내참수 크기가 다르다')
        if color.header.frame_id != self.info.header.frame_id:
            raise ValueError('색 영상과 내참수의 frame_id가 다르다')
        if depth.encoding == '16UC1':
            dm = d.astype(np.float32) / 1000
        elif depth.encoding == '32FC1':
            dm = d.astype(np.float32)
        else:
            raise ValueError('지원하지 않는 깊이 인코딩: ' + depth.encoding)
        if not np.any(np.isfinite(dm) & (dm > 0)):
            raise ValueError('유효한 깊이 점이 없다')
        ts = Time.from_msg(depth.header.stamp)
        record = dict(index=len(self.samples), depth_stamp_s=ds, color_stamp_s=stamp(color),
                      skew_ms=skew*1000, color_frame=color.header.frame_id, depth_frame=depth.header.frame_id,
                      depth_encoding=depth.encoding, camera_info=dict(frame=self.info.header.frame_id,
                      stamp_s=stamp(self.info), width=self.info.width, height=self.info.height,
                      k=list(self.info.k), d=list(self.info.d), r=list(self.info.r), p=list(self.info.p),
                      distortion_model=self.info.distortion_model),
                      base_camera=self.lookup(color.header.frame_id, ts), base_tcp=self.lookup(self.args.tcp, ts))
        name = f'frame_{len(self.samples):03d}'
        import cv2
        if not cv2.imwrite(str(self.args.out / (name + '.png')), c):
            raise RuntimeError('촬영 사진 저장 실패')
        np.savez_compressed(self.args.out / (name + '.npz'), color_bgr=c, depth_m=dm)
        (self.args.out / (name + '.json')).write_text(json.dumps(record, ensure_ascii=False, indent=2))
        self.samples.append(record)
        self.last_stamp = ds
        self.get_logger().info(f'원자료 {len(self.samples)}/{self.args.frames} 저장')


def main():
    """CLI 수집을 실행한다. 원자료만 저장하며 시간 초과·Ctrl+C는 불완전 기록을 남기고 비정상 종료한다."""
    p = argparse.ArgumentParser()
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--label', required=True)
    p.add_argument('--prefix', default='/camera/camera')
    p.add_argument('--base', default='base_link')
    p.add_argument('--tcp', default='rg2_tcp')
    p.add_argument('--frames', type=int, default=1)
    p.add_argument('--timeout-s', type=float, default=30)
    p.add_argument('--max-skew-ms', type=float, default=20)
    a = p.parse_args()
    if a.frames <= 0 or not (0 < a.timeout_s < float('inf')) or not (0 < a.max_skew_ms < float('inf')):
        p.error('횟수·시간·시각차는 유한한 양수여야 한다')
    a.out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Capture(a)
    end = time.monotonic() + a.timeout_s
    failure = None
    try:
        while rclpy.ok() and len(node.samples) < a.frames and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except BaseException as exc:
        failure = repr(exc)
    finally:
        complete = len(node.samples) == a.frames and failure is None
        summary = dict(label=a.label, frames=len(node.samples), requested=a.frames, complete=complete,
                       error=failure, purpose='원자료 수집: 정확도 통과 판정 아님',
                       camera_tf_available=sum(x['base_camera']['ok'] for x in node.samples),
                       tcp_tf_available=sum(x['base_tcp']['ok'] for x in node.samples),
                       reference_truth='별도 실제 측정 필요. 로봇 명령좌표를 정답으로 채택하지 않음')
        (a.out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    raise SystemExit(0 if complete else 1)


if __name__ == '__main__':
    main()

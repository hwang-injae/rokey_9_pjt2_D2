"""스캔 중 실시간 점군 미리보기 노드 (scan_live_preview) — 10/10 E-82 B · W168, IRD 4.1 `/d2/vision/scan_preview` · 6장 scan_preview/1.0 kind live.

손목 카메라가 '지금 보는' 점군을 1 ~ 2 Hz 로 저장해 알린다 → 다리가 화면 3D 로 넘기면 화면이 덮어 그린다(보기 전용 — 작업 관리자 · 추론은 안 씀).
wrist_block 은 고치지 않고 따로 노드(E-82 '따로 노드'). 계산(깊이 → 점 → 자르기 → 복셀 → PLY · 알림)은 live_cloud.LiveCloud(ROS 없음, 시험 있음).

입력(구독)
  <cam_prefix>/aligned_depth_to_color/image_raw(깊이 16UC1 mm 또는 32FC1 m) · <cam_prefix>/color/camera_info(내부값) — wrist_block 과 같은 토픽
  /d2/task/state(state/1 JSON, TRANSIENT_LOCAL) — state 가 'SCAN_' 으로 시작할 때만 낸다(only_during_scan, 기본). run_id = state 의 run_id(없으면 'live')
  TF base_link → rg2_tcp(robot.yaml frame_id · tcp_link) — **깊이 사진 시각**으로 묻고(최대 TF_WAIT_S), 못 구하면 그 장은 건너뜀
  카메라 자세 = tcp_pose.camera_pose_mm(TF × 손목 보정값 T_rg2tcp2camera · 기울기 밀림 고침) — wrist_block 과 같은 함수 · 같은 보정 파일
내보내기
  `<scan_dir>/<run_id>/live_<k>.ply`(base_link m, 복셀 5 mm · ≤ 500 KB, k = 1 ~ keep_files 를 돌려 씀 · 임시 파일 뒤 이름 바꾸기)
  `/d2/vision/scan_preview`(std_msgs/String, 기본 QoS = RELIABLE · VOLATILE · depth 10) {"schema":"scan_preview/1.0","stamp","run_id",
  "pose_id"(state 에 촬영 자세가 없어 빈 값),"index"(이 run 에서 낸 순번, 1부터),"kind":"live","cloud_path","points"}
  (선택) rviz_cloud 가 켜지면 `/d2/vision/scan_live_cloud`(sensor_msgs/PointCloud2, frame base_link — 자른 점, 복셀 전) — HMI 화면 전 RViz 확인용,
  IRD 에 없는 새 토픽이라 기본 꺼짐(PL 확인 전)

파라미터
  live_hz           낼 빠르기(Hz, 기본 1.5 — IRD 1 ~ 2 Hz). 0.2 ~ 5 로 자름
  only_during_scan  True(기본) = 작업 관리자 state 가 SCAN_* 일 때만 계산 · 발행(평소엔 아무것도 안 함). False = 늘(시험 · 확인용)
  scan_dir          스캔 폴더(비우면 ~/d2_data/scan — wrist_block 과 같음)
  calib_path        손목 보정값(비우면 d2_vision share/config T_rg2tcp2camera.npy — wrist_block 과 같음)
  cam_prefix        realsense 토픽 접두(기본 /camera/camera)
  tcp_pose_mm       시험용 rg2_tcp 자세 7개(x · y · z mm + 쿼터니언 x · y · z · w, 실수로). 모두 0 이면 TF
  keep_files        돌려 쓸 live 파일 수(기본 5)
  rviz_cloud        True 면 PointCloud2 도 냄(기본 False)

바깥 영향: 파일 쓰기(scan_dir 아래) · 토픽 발행뿐. 로봇 · 카메라 · 그리퍼 · 다른 노드에 명령을 보내지 않는다(로봇을 움직이지 않아 서기 처리 없음).
실패 때: 깊이 · 내부값 없음 · TF 못 구함 · 파일 못 씀 → 그 장은 건너뛰고 경고 로그(같은 이유는 WARN_EVERY_S 에 한 번). 노드는 죽지 않는다.
실행 (로봇 PC — 브링업(robot_state_publisher) · 카메라 · 작업 관리자와 같이):
  ros2 run d2_vision scan_live_preview                                   # 스캔 중에만 1.5 Hz
  ros2 run d2_vision scan_live_preview --ros-args -p rviz_cloud:=true    # RViz 로도(Fixed Frame base_link, 토픽 /d2/vision/scan_live_cloud)
  ros2 topic echo /d2/vision/scan_preview --field data                   # kind live 알림 확인
"""
import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String

from d2_vision.live_cloud import LIVE_KEEP, LiveCloud
from d2_vision.tcp_pose import TcpPose, camera_pose_mm, default_calib_path, load_calib, load_tilt_shift, transform_to_matrix_mm

TF_WAIT_S = 0.1           # 깊이 시각의 TF 가 아직 안 왔으면 이만큼만 기다림(robot_state_publisher 는 수십 Hz) — 1 ~ 2 Hz 틱을 막지 않게
WARN_EVERY_S = 10.0       # 같은 이유 경고는 이 간격에 한 번(1.5 Hz 로 계속 찍지 않게)
QOS_STATE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)   # 작업 관리자 state 와 같은 QoS(늦게 떠도 마지막 값을 받음)


class ScanLivePreview(Node):
    """스캔 중 '지금 보이는' 점군을 live_hz 로 PLY · scan_preview(kind live)로 내는 보기 전용 노드."""

    def __init__(self):
        """파라미터 · robot.yaml · 보정값을 읽고 구독(깊이 · camera_info · state) · TF · 발행 · 타이머를 만든다. 보정값을 못 읽으면 경고 후 계속(내지 않음)."""
        super().__init__('scan_live_preview')
        self.declare_parameter('live_hz', 1.5)
        self.declare_parameter('only_during_scan', True)
        self.declare_parameter('scan_dir', '')
        self.declare_parameter('calib_path', '')
        self.declare_parameter('cam_prefix', '/camera/camera')
        self.declare_parameter('tcp_pose_mm', [0.0] * 7)
        self.declare_parameter('keep_files', LIVE_KEEP)
        self.declare_parameter('rviz_cloud', False)

        p = Path(get_package_share_directory('d2_bringup')) / 'config' / 'robot.yaml'
        self.cfg = yaml.safe_load(p.read_text(encoding='utf-8'))
        self.live = LiveCloud(self.cfg, keep=int(self.get_parameter('keep_files').value))
        self.scan_dir = Path(self.get_parameter('scan_dir').value or Path.home() / 'd2_data' / 'scan')
        cp = self.get_parameter('calib_path').value or str(default_calib_path())
        self.tilt_shift = load_tilt_shift(cp)
        try:
            self.T_tcp2c = load_calib(cp)
        except (OSError, ValueError) as e:
            self.T_tcp2c = None
            self.get_logger().error('손목 보정값을 못 읽음(%s): %s — 미리보기를 내지 않는다' % (cp, e))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.depth, self.intr, self.state = None, None, None       # 마지막 깊이 (mm, stamp msg, 받은 순번) · 내부값 · state dict
        self.depth_seq, self.done_seq = 0, 0                        # 같은 깊이 장을 두 번 내지 않게
        self.warned = {}

        sub = ReentrantCallbackGroup()                              # 구독 · TF 는 타이머가 TF 를 기다리는 동안에도 돈다
        self.tcp = TcpPose(self, self.cfg.get('frame_id', 'base_link'), self.cfg.get('tcp_link', 'rg2_tcp'))
        prefix = self.get_parameter('cam_prefix').value
        self.create_subscription(Image, f'{prefix}/aligned_depth_to_color/image_raw', self.on_depth, 5, callback_group=sub)
        self.create_subscription(CameraInfo, f'{prefix}/color/camera_info', self.on_info, 5, callback_group=sub)
        self.create_subscription(String, '/d2/task/state', self.on_state, QOS_STATE, callback_group=sub)
        self.pub = self.create_publisher(String, '/d2/vision/scan_preview', 10)            # IRD E-82 기본 QoS(VOLATILE) — capture 와 같은 토픽
        self.pub_cloud = (self.create_publisher(PointCloud2, '/d2/vision/scan_live_cloud', 10)
                          if self.get_parameter('rviz_cloud').value else None)
        hz = min(max(float(self.get_parameter('live_hz').value), 0.2), 5.0)
        self.create_timer(1.0 / hz, self.tick, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info('scan_live_preview 시작: %.1f Hz · %s · 폴더 %s%s' % (
            hz, '스캔 중에만' if self.get_parameter('only_during_scan').value else '늘', self.scan_dir,
            ' · RViz /d2/vision/scan_live_cloud' if self.pub_cloud else ''))

    # ---------- 구독 ----------
    def on_depth(self, msg):
        """정렬된 깊이 한 장 → mm float32 로 마지막 것만 둔다(16UC1 = mm, 32FC1 = m)."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        if msg.encoding != '16UC1':
            img *= 1000.0
        with self.lock:
            self.depth_seq += 1
            self.depth = (img, msg.header.stamp, self.depth_seq)

    def on_info(self, msg):
        """camera_info → 내부값 (fx, fy, ppx, ppy) 픽셀."""
        self.intr = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_state(self, msg):
        """작업 관리자 state/1 JSON → dict(못 읽으면 그대로 둠)."""
        try:
            self.state = json.loads(msg.data)
        except ValueError:
            self.warn('state', '/d2/task/state 를 JSON 으로 못 읽음')

    # ---------- 한 틱 ----------
    def tick(self):
        """(타이머) 낼 때면 마지막 깊이 한 장 → 그 시각 TF × 보정 → LiveCloud.step → 발행. 실패는 경고 로그만(그 장 건너뜀)."""
        only = bool(self.get_parameter('only_during_scan').value)
        with self.lock:
            depth = self.depth
        fresh = depth is not None and depth[2] != self.done_seq
        T = None
        if fresh and self.T_tcp2c is not None and (not only or str((self.state or {}).get('state') or '').startswith('SCAN_')):
            T_b2t = self.read_tcp(depth[1])
            if T_b2t is not None:
                T = camera_pose_mm(T_b2t, self.T_tcp2c, self.tilt_shift)
                T[:3, 3] /= 1000.0
        t0 = time.monotonic()
        try:
            msg, pts, why = self.live.step(self.state, depth[0] if fresh else None, self.intr, T, self.scan_dir, only_during_scan=only)
        except (OSError, ValueError) as e:
            self.warn('write', '실시간 점군을 못 씀: %r' % (e,))
            return
        if msg is None:
            if why not in ('not_scanning',):
                self.warn(why, {'no_depth': '새 깊이 사진이 없다(카메라?)', 'no_intrinsics': 'camera_info 를 못 받음',
                                'no_pose': 'TF base_link → rg2_tcp 를 깊이 시각으로 못 구함(브링업?) — 그 장은 건너뜀'}.get(why, why)
                          + ('' if self.T_tcp2c is not None else ' · 보정값 없음'))
            return
        self.done_seq = depth[2]
        self.pub.publish(String(data=json.dumps(msg, ensure_ascii=False)))
        if self.pub_cloud is not None:
            self.pub_cloud.publish(self.cloud_msg(pts, depth[1]))
        self.get_logger().debug('scan_preview live %s #%d: 점 %d · %.0f ms' % (msg['run_id'], msg['index'], msg['points'],
                                                                             (time.monotonic() - t0) * 1000))

    def read_tcp(self, stamp):
        """깊이 시각 stamp 의 rg2_tcp 자세 4x4(mm). tcp_pose_mm 가 0 이 아니면 그것(시험용). 못 구하면 None(그 장 건너뜀)."""
        p = list(self.get_parameter('tcp_pose_mm').value)
        if any(abs(v) > 1e-9 for v in p):
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(p[3:7]).as_matrix()
            T[:3, 3] = p[:3]
            return T
        try:
            t = self.tcp.buffer.lookup_transform(self.tcp.frame_id, self.tcp.tcp_link, Time.from_msg(stamp), timeout=Duration(seconds=TF_WAIT_S))
        except Exception:                     # Lookup · Extrapolation · 시간 초과 — 그 장은 건너뜀
            return None
        return transform_to_matrix_mm(t)

    def cloud_msg(self, pts, stamp):
        """점 (N, 3) m → sensor_msgs/PointCloud2(frame base_link, x · y · z float32) — RViz 확인용."""
        from sensor_msgs_py import point_cloud2
        header = Header(stamp=stamp, frame_id=self.cfg.get('frame_id', 'base_link'))
        return point_cloud2.create_cloud_xyz32(header, np.asarray(pts, np.float32))

    def warn(self, key, text):
        """같은 이유(key) 경고는 WARN_EVERY_S 에 한 번만."""
        now = time.monotonic()
        if now - self.warned.get(key, -1e9) >= WARN_EVERY_S:
            self.warned[key] = now
            self.get_logger().warn(text)


def main(args=None):
    """노드를 멀티스레드 실행기로 돌린다(타이머가 TF 를 잠깐 기다리는 동안 /tf 구독이 돌게). 로봇에 명령이 없어 서기 처리는 없다."""
    rclpy.init(args=args)
    node = ScanLivePreview()
    exe = MultiThreadedExecutor(num_threads=3)
    exe.add_node(node)
    try:
        exe.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

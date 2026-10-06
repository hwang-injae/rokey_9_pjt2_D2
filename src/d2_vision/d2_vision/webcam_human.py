"""웹캠 사람 감지 (webcam_human) — 작업대 구역별 사람 있음/없음을 /d2/vision/human_zone 으로 낸다.

인터페이스는 mock_webcam_human 과 같다(IRD human_zone/1, RELIABLE·KEEP_LAST 1·VOLATILE).
1차(10/7)는 신호만 낸다. 받는 쪽(정지 노드)은 10/8 W066부터, 카메라 연결 신호(camera_status, 예전 이름 heartbeat)도 그때 더한다.

판정: MediaPipe Pose 의 어깨·팔꿈치·손목(점 11~16) 중 보임 정도 min_visibility 이상인 점이
하나라도 구역(robot.yaml webcam_zones, base_link m) 안이면 그 구역 = 있음. 높이 때문에 밀리는 오차는 보정하지 않는다
(W055 설치·보정 뒤 구역 경계·margin 으로 정한다).

필요한 것 (MediaPipe·OpenCV 는 vision 컨테이너(W101)에만 설치한다 — 호스트 pip 금지):
  - robot.yaml 의 webcam_zones 4구역 좌표 (W055 뒤), webcam_calib.py 가 만든 webcam_H.json
  - pose_landmarker_lite.task (git 에 올리지 않고 로컬 폴더를 -v 로 연결)

실행:
  ros2 run d2_vision webcam_human --ros-args \\
    -p config_file:=<robot.yaml> -p calib_file:=<webcam_H.json> -p model_path:=<pose_landmarker_lite.task> \\
    -p camera_index:=<번호>

설정·모델·카메라가 안 맞으면 ERROR 로그를 남기고 시작하지 않는다(종료 코드 1).
카메라 프레임을 못 읽는 동안은 발행하지 않고(마지막 값 재발행 없음), 다시 읽히면 이어서 처리한다.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from d2_vision.human_zone_detector import ConfigError, HumanZoneDetector

# mock_webcam_human 과 같은 QoS (10/4 범진님과 정함)
QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                 reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

ARM = (11, 12, 13, 14, 15, 16)   # MediaPipe Pose: 왼·오른 어깨, 팔꿈치, 손목 (arm_track.py 와 같다)
REOPEN_AFTER_S = 2.0             # 이만큼 계속 못 읽으면 카메라를 다시 연다 (USB 가 빠졌다 꽂힌 경우)


class StartupError(Exception):
    """시작 조건이 안 맞을 때. 이유는 이미 ERROR 로그로 냈다."""


class WebcamHuman(Node):
    """웹캠 프레임을 읽어 구역별 사람 있음/없음을 낸다.

    입력: 웹캠 영상, 보정 json, robot.yaml webcam_zones, 포즈 모델.
    출력: /d2/vision/human_zone (JSON human_zone/1) — 처리한 프레임마다 한 번, stamp = 프레임을 읽은 시각(초).
    바깥 영향: 메시지 발행뿐(로봇은 안 움직인다).
    실패: 설정·모델·카메라·MediaPipe 문제는 ERROR 후 StartupError(시작 안 함).
          실행 중 프레임을 못 읽으면 WARN 만 내고 발행을 멈춘다.
    """

    def __init__(self):
        """파라미터를 읽고 구역 설정·포즈 모델·카메라를 준비한 뒤 발행기와 프레임 타이머를 만든다.

        입력: config_file·calib_file·model_path 파라미터(경로), camera_index. 출력: 없음(타이머가 step 을 부른다).
        실패: 설정·모델·MediaPipe 문제는 ERROR 로그 후 StartupError. 카메라만 못 열면 step 이 다시 시도한다.
        """
        super().__init__('webcam_human')
        self.declare_parameter('config_file', '')       # robot.yaml 경로 (webcam_zones)
        self.declare_parameter('calib_file', '')        # webcam_H.json 경로
        self.declare_parameter('model_path', '')        # pose_landmarker_lite.task 경로
        self.declare_parameter('camera_index', 0)       # ls /dev/video* 로 확인
        self.declare_parameter('min_visibility', 0.5)   # arm_track.py 와 같은 기준
        self.declare_parameter('num_poses', 2)          # 동시에 볼 사람 수 (arm_track.py 와 같다)

        self.detector = self._load_detector()
        self.pose, self.mp, self.cv2 = self._load_mediapipe()
        self.cap = None
        self.fail_since = None      # 프레임을 못 읽기 시작한 시각 (monotonic)
        self.last_ts_ms = -1
        self.t0 = time.monotonic()
        self._open_camera()

        self.pub_zone = self.create_publisher(String, '/d2/vision/human_zone', QOS)
        # 프레임 읽기가 기다려 주므로 타이머는 짧게 — 처리 속도는 카메라·MediaPipe 가 정한다
        self.create_timer(0.005, self.step)
        self.get_logger().info('webcam_human 시작: 카메라 %d, 보정 해상도 %dx%d' % (
            self.get_parameter('camera_index').value, self.detector.width, self.detector.height))

    def _fail(self, msg):
        """시작 못 하는 이유를 ERROR 로 남기고 StartupError 를 낸다."""
        self.get_logger().error(msg)
        raise StartupError(msg)

    def _load_detector(self):
        """robot.yaml webcam_zones 와 보정 파일을 읽는다. 없거나 비었으면 임의 값 없이 시작을 막는다."""
        try:
            return HumanZoneDetector.from_files(self.get_parameter('config_file').value,
                                                self.get_parameter('calib_file').value)
        except ConfigError as e:
            self._fail('웹캠 구역 설정 실패 — 정상 감지를 시작하지 않는다: %s' % e)

    def _load_mediapipe(self):
        """MediaPipe 포즈 모델을 연다. 라이브러리·모델 파일이 없으면 시작을 막는다(컨테이너 vision 에서만 돈다)."""
        path = self.get_parameter('model_path').value
        if not path or not os.path.isfile(path):
            self._fail("포즈 모델 파일이 없다: '%s' (model_path 파라미터, pose_landmarker_lite.task)" % path)
        try:
            import cv2
            import mediapipe as mp
        except ImportError as e:
            self._fail('MediaPipe/OpenCV 를 불러올 수 없다 (%s). 호스트에 pip 하지 않는다 — vision 컨테이너(W101)에서 실행' % e)
        vision = mp.tasks.vision
        pose = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=path),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=self.get_parameter('num_poses').value))
        return pose, mp, cv2

    def _open_camera(self):
        """웹캠을 보정 때 해상도로 연다. 못 열면 ERROR 만 내고 step 이 다시 시도한다(처음 시작 때도 같다)."""
        cap = self.cv2.VideoCapture(self.get_parameter('camera_index').value)
        cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, self.detector.width)
        cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, self.detector.height)
        if not cap.isOpened():
            cap.release()
            self.get_logger().error('카메라를 열 수 없다 (camera_index=%d). 발행하지 않고 다시 시도한다'
                                    % self.get_parameter('camera_index').value, throttle_duration_sec=5.0)
            self.cap = None
            return
        self.cap = cap

    def now(self):
        """ROS 시계의 지금 시각(초). human_zone 의 stamp 에 쓴다."""
        return self.get_clock().now().nanoseconds / 1e9

    def step(self):
        """프레임 하나를 읽어 처리하고 발행한다. 못 읽으면 발행 없이 WARN, 오래 못 읽으면 카메라를 다시 연다."""
        if self.cap is None:
            self._open_camera()
            return
        ok, frame = self.cap.read()
        stamp = self.now()   # 웹캠은 하드웨어 시각을 안 줘서 읽은 직후 시각을 프레임 시각으로 쓴다
        if not ok:
            if self.fail_since is None:
                self.fail_since = time.monotonic()
            self.get_logger().warn('카메라 프레임을 못 읽는다 — human_zone 발행 중단', throttle_duration_sec=5.0)
            if time.monotonic() - self.fail_since > REOPEN_AFTER_S:
                self.cap.release()
                self.cap = None
                self.fail_since = None
            return
        self.fail_since = None

        h, w = frame.shape[:2]
        if (w, h) != (self.detector.width, self.detector.height):
            # 해상도가 다르면 픽셀→mm 가 틀리므로 신호를 내지 않는다
            self.get_logger().error('화면 %dx%d 가 보정 때 %dx%d 와 다르다 — 발행하지 않는다. calib 를 다시 하거나 카메라를 확인'
                                    % (w, h, self.detector.width, self.detector.height), throttle_duration_sec=5.0)
            return

        # VIDEO 모드는 시각(ms)이 계속 커져야 한다
        self.last_ts_ms = max(int((time.monotonic() - self.t0) * 1000), self.last_ts_ms + 1)
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB,
                            data=self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB))
        result = self.pose.detect_for_video(img, self.last_ts_ms)

        min_vis = self.get_parameter('min_visibility').value
        points = [(p[i].x * w, p[i].y * h)
                  for p in result.pose_landmarks for i in ARM if (p[i].visibility or 0) >= min_vis]
        msg = {
            'schema': 'human_zone/1',
            'stamp': stamp,
            'zones': self.detector.occupied(points),
        }
        self.pub_zone.publish(String(data=json.dumps(msg)))

    def destroy_node(self):
        """카메라와 포즈 모델을 먼저 놓는다."""
        if getattr(self, 'cap', None) is not None:
            self.cap.release()
        if getattr(self, 'pose', None) is not None:
            self.pose.close()
        super().destroy_node()


def main(args=None):
    """webcam_human 노드를 돌린다. Ctrl+C 로 끝나고, 시작 조건이 안 맞으면 종료 코드 1."""
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = WebcamHuman()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except StartupError:
        code = 1   # 이유는 이미 ERROR 로그로 냈다
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

"""가짜 손목 손 찾기 (mock_wrist_hand) — 골격, IRD 6장 안 기준.

손목 카메라 없이 /d2/vision/hand_wrist 와 /d2/vision/heartbeat 를 낸다.
받는 쪽: 작업 관리자(차례 끝), 장면 관리(출발 전 human_* 넣기), 정지 노드(heartbeat).

규칙(안):
  - hand_wrist 는 관측 자세에 있는 동안만 계속 낸다(at_observe=true). 이동 중에는 안 낸다.
  - heartbeat 는 at_observe 와 상관없이 늘 낸다(로봇 차례에 끊기면 정지되므로).

실행:
  ros2 run d2_vision mock_wrist_hand
  ros2 run d2_vision mock_wrist_hand --ros-args -p present:=true

돌리는 중에 바꾸기:
  ros2 param set /mock_wrist_hand present true         # 손 들어옴
  ros2 param set /mock_wrist_hand present false        # 손 뺌 → 작업 관리자 3초 세기 시험
  ros2 param set /mock_wrist_hand at_observe false     # 로봇 이동 중 흉내 (hand_wrist 멈춤)
  ros2 param set /mock_wrist_hand stop_heartbeat true  # VISION_LOST 시험

미정(확정되면 고침):
  - 손·팔 모양: 지금은 IRD 안대로 상자(center_m + size_m). 세교님(장면 관리) 답에 따라 원기둥으로 바꿀 수 있음.
  (정함: 관측 자세에서만 10 Hz, 작업 관리자는 0.5초 지난 값 무시, QoS = RELIABLE·KEEP_LAST 1·VOLATILE)
"""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# QoS (안): 최신 값만 의미 있음 → 마지막 1개. RELIABLE 은 받는 쪽이 RELIABLE·BEST_EFFORT 어느 쪽이어도 붙는다
QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                 reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


class MockWristHand(Node):

    def __init__(self):
        super().__init__('mock_wrist_hand')
        self.declare_parameter('rate_hz', 10.0)          # hand_wrist 주기 (안)
        self.declare_parameter('heartbeat_hz', 2.0)      # IRD 확정
        self.declare_parameter('at_observe', True)       # 관측 자세에 있는가 (false 면 hand_wrist 안 냄)
        self.declare_parameter('present', False)         # 손·팔 보임
        # 손·팔 상자 1개 (base_link, m) — IRD 예시 값. 모양 확정 전 임시
        self.declare_parameter('center_m', [0.45, -0.10, 0.25])
        self.declare_parameter('size_m', [0.10, 0.10, 0.30])
        self.declare_parameter('stop_heartbeat', False)

        self.pub_hand = self.create_publisher(String, '/d2/vision/hand_wrist', QOS)
        self.pub_hb = self.create_publisher(String, '/d2/vision/heartbeat', QOS)
        rate = self.get_parameter('rate_hz').value
        hb = self.get_parameter('heartbeat_hz').value
        self.create_timer(1.0 / rate, self.publish_hand)
        self.create_timer(1.0 / hb, self.publish_heartbeat)
        self.get_logger().info(
            'mock_wrist_hand 시작: hand_wrist %.1f Hz(관측 자세일 때), heartbeat %.1f Hz'
            % (rate, hb))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_hand(self):
        if not self.get_parameter('at_observe').value:
            return
        present = self.get_parameter('present').value
        boxes = []
        if present:
            boxes.append({
                'center_m': list(self.get_parameter('center_m').value),
                'size_m': list(self.get_parameter('size_m').value),
            })
        msg = {
            'schema': 'hand_wrist/1',
            'stamp': self.now(),
            'present': present,
            'boxes': boxes,
        }
        self.pub_hand.publish(String(data=json.dumps(msg)))

    def publish_heartbeat(self):
        if self.get_parameter('stop_heartbeat').value:
            return
        t = self.now()
        msg = {
            'schema': 'heartbeat/1',
            'stamp': t,
            'node': 'wrist_hand',
            'last_frame_stamp': t,
        }
        self.pub_hb.publish(String(data=json.dumps(msg)))


def main(args=None):
    rclpy.init(args=args)
    node = MockWristHand()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

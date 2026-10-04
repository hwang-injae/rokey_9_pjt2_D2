"""가짜 웹캠 사람 감지 (mock_webcam_human) — IRD 6장 human_zone/1 기준.

웹캠 없이 /d2/vision/human_zone 을 낸다.
1차(10/7)는 방송만 한다. 받는 쪽(정지 노드)은 10/8 W066부터 구독한다.
heartbeat 는 1차 범위에서 빠졌다(W066에서 다시 넣는다. 지운 코드는 git 이력 b7df006 에 있다).

실행:
  ros2 run d2_vision mock_webcam_human
  ros2 run d2_vision mock_webcam_human --ros-args -p occupied:=assembly,path

돌리는 중에 바꾸기 (다음 주기부터 바로 반영):
  ros2 param set /mock_webcam_human occupied "assembly"   # 로봇 작업 영역에 사람
  ros2 param set /mock_webcam_human occupied ""           # 모두 비움

10/4 범진님과 정함: human_zone 10 Hz, QoS = RELIABLE·KEEP_LAST 1·VOLATILE.
가짜라 타이머로 낸다. 진짜(webcam_human)는 처리한 프레임마다 내고 stamp = 프레임 시각.
"""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# QoS (안): 최신 값만 의미 있음 → 마지막 1개. RELIABLE 이면 받는 쪽이 어느 설정이어도 붙는다
QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                 reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

ZONES = ('assembly', 'robot_supply', 'path', 'human_supply')


class MockWebcamHuman(Node):

    def __init__(self):
        super().__init__('mock_webcam_human')
        self.declare_parameter('rate_hz', 10.0)   # human_zone 주기 (안)
        self.declare_parameter('occupied', '')    # 사람 있는 구역, 쉼표로: "assembly,path"

        self.pub_zone = self.create_publisher(String, '/d2/vision/human_zone', QOS)
        rate = self.get_parameter('rate_hz').value
        self.create_timer(1.0 / rate, self.publish_zone)
        self.get_logger().info('mock_webcam_human 시작: human_zone %.1f Hz' % rate)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_zone(self):
        raw = self.get_parameter('occupied').value
        occupied = {z.strip() for z in raw.split(',') if z.strip()}
        unknown = occupied - set(ZONES)
        if unknown:
            self.get_logger().warn('모르는 구역 이름 %s — %s 중에서 쓴다' % (sorted(unknown), ZONES),
                                   throttle_duration_sec=5.0)
        msg = {
            'schema': 'human_zone/1',
            'stamp': self.now(),
            'zones': {z: (z in occupied) for z in ZONES},
        }
        self.pub_zone.publish(String(data=json.dumps(msg)))


def main(args=None):
    rclpy.init(args=args)
    node = MockWebcamHuman()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-
"""정지 노드 safety_stop — 멈출지 판단하고, 로봇을 세우고, 잠근다. 로봇 PC 에서 늘 켜 둔다 (W040)."""
import json
import os
import time

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from d2_interfaces.srv import StopRequest
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from d2_safety.safe_stop import SafeStop, init_ros

ALARM_STATES = {3: 'SAFE_OFF', 5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 9: 'SAFE_STOP2', 10: 'SAFE_OFF2'}  # 두산 robot_state
ALARM_PERIOD_S = 0.5      # 로봇 알람 확인 주기. 알람이면 제어기가 이미 세웠으므로 잠금·알림만 늦지 않으면 된다
FIRST_WAIT_DEFAULT_S = 2.0
# safety/state 는 바뀔 때만 보낸다. TRANSIENT_LOCAL 이라 늦게 켠 노드도 마지막 상태를 바로 받는다 (IRD 4.1, 10/6 S-26)
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


class SafetyStopNode(Node):
    """정지 판단 + 서기 궤적 + 잠금을 맡는 유일한 노드 (1차 정지 입력 4개: 화면 버튼·키·Ctrl+C·로봇 알람).

    받는 것: /d2/safety/stop (d2_interfaces/StopRequest, 화면·키·작업 관리자), /d2/safety/resume (std_srvs/Trigger, 화면 '다시 시작'),
             두산 /dsr_controller2/system/get_robot_state (로봇 알람), Ctrl+C (이 프로그램 터미널)
    내보내는 것: 서기 궤적 (SafeStop), /d2/safety/state (JSON safety_state/1, 바뀔 때, TRANSIENT_LOCAL)
    한 번 멈추면 잠그고, 화면 '다시 시작'(safety/resume)으로만 푼다. 집기·놓기는 safety/state 를 보고 목표를 취소한다 (S-26).
    """

    def __init__(self):
        """서비스·토픽을 열고 SafeStop 을 준비한다. robot.yaml 이 없으면 first_wait_s 기본값 2 s 를 쓴다."""
        super().__init__('safety_stop')
        self.stopper = SafeStop(self, self._first_wait_s())
        self.state_pub = self.create_publisher(String, '/d2/safety/state', STATE_QOS)
        self.create_service(Trigger, '/d2/safety/resume', self._on_resume)
        self.create_service(StopRequest, '/d2/safety/stop', self._on_stop)
        self.pending = None               # 서비스·타이머 콜백이 정지 이유를 넣고, run() 이 세운다
        self.stopped, self.locked, self.reason = False, False, ''
        self.stopping = False             # 세우는 중에 들어온 입력은 새 정지로 보지 않는다
        self._alarm_cli, self._alarm_busy = None, False
        self.create_timer(ALARM_PERIOD_S, self._check_alarm)

    def _first_wait_s(self):
        """robot.yaml 의 stop.first_wait_s (s) 를 읽는다. 파일·키가 없으면 기본값을 쓰고 경고한다."""
        try:
            path = os.path.join(get_package_share_directory('d2_bringup'), 'config', 'robot.yaml')
            with open(path) as f:
                return float(yaml.safe_load(f)['stop']['first_wait_s'])
        except (PackageNotFoundError, OSError, KeyError, TypeError, ValueError):
            self.get_logger().warn(f'robot.yaml 에서 stop.first_wait_s 를 못 읽어 {FIRST_WAIT_DEFAULT_S} s 를 쓴다')
            return FIRST_WAIT_DEFAULT_S

    def _on_stop(self, req, res):
        """화면·키·작업 관리자 정지 요청을 받는다. 바로 '받았다'고 답하고, 실제로 세우는 것은 run() 이 한다.

        입력: source (web·key·task — task 는 move_to 가 timeout.move_to_s 를 넘을 때 reason TIMEOUT, 10/7 PL), reason.
        source 를 가리지 않고 받는다: 멈추라는 요청은 누가 보내든 받는 쪽이 안전하다(멈출지 판단은 이 노드 한 곳).
        출력: success = 정지를 시작하거나 이미 멈춰 있으면 True.
        """
        if not (self.locked or self.stopping):
            self.pending = req.reason or f'STOP_{req.source.upper()}'
        res.success = True
        res.message = '이미 멈춰 있다' if self.locked else '세우는 중'
        return res

    def _on_resume(self, _req, res):
        """화면 '다시 시작' — 잠금을 푼다. 세우는 중이면 거절한다 (다 선 뒤에만 풀 수 있다)."""
        if self.stopping:
            res.success, res.message = False, '아직 세우는 중이다'
            return res
        was_locked = self.locked
        self.stopped, self.locked, self.reason = False, False, ''
        self._publish_state()
        res.success, res.message = True, ('잠금을 풀었다' if was_locked else '잠겨 있지 않았다')
        self.get_logger().info(f'[다시 시작] {res.message}')
        return res

    def _check_alarm(self):
        """두산 로봇 상태를 비동기로 물어, 알람 상태면 정지 이유를 넣는다. 상태 서비스가 없으면(가상 등) 건너뛴다."""
        if self._alarm_busy or self.locked or self.stopping:
            return
        if self._alarm_cli is None:
            try:
                from dsr_msgs2.srv import GetRobotState
            except ImportError:
                return
            self._alarm_cli = (self.create_client(GetRobotState, '/dsr_controller2/system/get_robot_state'), GetRobotState)
        cli, srv = self._alarm_cli
        if not cli.service_is_ready():
            return
        self._alarm_busy = True
        cli.call_async(srv.Request()).add_done_callback(self._on_alarm_state)

    def _on_alarm_state(self, fut):
        """로봇 상태 답을 받는다. 알람이면 pending = 'ROBOT_ALARM:<상태 이름>'."""
        self._alarm_busy = False
        r = fut.result()
        if r is not None and r.success and r.robot_state in ALARM_STATES and not (self.locked or self.stopping):
            self.pending = f'ROBOT_ALARM:{ALARM_STATES[r.robot_state]}'

    def _publish_state(self):
        """/d2/safety/state 에 지금 상태를 JSON safety_state/1 로 보낸다."""
        self.state_pub.publish(String(data=json.dumps(
            {'schema': 'safety_state/1', 'stamp': time.time(), 'stopped': self.stopped,
             'locked': self.locked, 'reason': self.reason})))

    def halt(self, reason):
        """로봇을 세우고 잠근다.

        순서: safety/state {stopped, locked} -> 서기 궤적·설 때까지 확인 (SafeStop).
        바깥 영향: 로봇 팔이 선다. 못 세우면 SafeStop 이 '펜던트 비상정지!' 를 띄우고, 그래도 잠근 채로 둔다.
        """
        self.stopping = True
        # 상태를 먼저 알린다: 집기·놓기가 이것을 보고 목표를 취소해야 다음 궤적을 보내지 않는다
        self.stopped, self.locked, self.reason = True, True, reason
        self._publish_state()
        try:
            self.stopper.stop(reason)
        finally:
            self.stopping = False
            self.pending = None

    def run(self):
        """정지 이유가 들어올 때까지 노드를 돌리다가, 들어오면 세운다. Ctrl+C 를 받으면 세운 뒤 끝낸다."""
        self.stopper.prepare()
        self._publish_state()
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.pending and not self.locked and not self.stopping:
                    self.halt(self.pending)
        except KeyboardInterrupt:
            self.halt('CTRL_C')


def main():
    """정지 노드를 켠다. rclpy 는 init_ros 로 시작해 Ctrl+C 뒤에도 서기 궤적을 보낼 수 있게 한다."""
    init_ros()
    node = SafetyStopNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

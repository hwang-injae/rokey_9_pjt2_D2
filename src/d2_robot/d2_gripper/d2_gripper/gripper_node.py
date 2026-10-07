# -*- coding: utf-8 -*-
"""그리퍼 노드 gripper — OnRobot RG2 를 다루는 유일한 노드. 다 움직인 뒤 폭과 잡힘을 답한다 (W039, S-01)."""
import json
import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from d2_interfaces.srv import GripperCommand
from onrobot_rg_msgs.srv import SetCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

DRIVER_SRV = '/onrobot/sendCommand'        # onrobot_rg_control 드라이버 (실기) / 가상 그리퍼 — 브링업이 띄운다
FEEDBACK_TOPIC = '/onrobot_joint_states'   # 브링업이 드라이버의 joint_states 를 이 이름으로 바꿔 낸다
MAX_WIDTH_M = 0.110                        # RG2 최대 열림
FORCE_STEP_N = 2.5                         # 드라이버는 힘을 'i'/'d' 로 2.5 N 씩만 바꾼다 (켜질 때 40 N)
FORCE_RESET_STEPS = 16                     # 40 N / 2.5 N — 이만큼 'd' 를 보내면 확실히 0 N
STILL_M = 0.0002                           # 0.3 s 동안 폭 변화가 0.2 mm 미만이면 다 움직였다
STILL_S = 0.3
FEEDBACK_MAX_AGE_S = 1.0                   # 이보다 오래된 폭은 믿지 않는다
GRASP_MARGIN_M = 0.002                     # 명령 폭보다 이만큼 넘게 덜 닫혔으면 무언가에 막힌 것 (잡힘 후보)


def rg2_display_width_m(angle):
    """RG2 finger_joint 각 (rad) -> 표시 폭 (m). onrobot_rg_control 드라이버의 jointValueToWidth 와 같은 식."""
    L1, L3, th1, th3, dy = 0.108505, 0.055, 1.41371, 0.76794, -0.0144
    return (math.cos(angle + th3) * L3 + dy + L1 * math.cos(th1)) * 2


class GripperNode(Node):
    """RG2 그리퍼를 다루는 유일한 노드.

    받는 것: /d2/gripper/command (d2_interfaces/GripperCommand: 실제 손가락 끝 사이 폭 m, 힘 N)
    돌려주는 것: 다 움직인 뒤 실제 폭(m)과 잡힘 — 잡힘 = 명령보다 덜 닫혔고, 그 폭이 robot.yaml grasp_width_m 중 하나와 맞음
    내보내는 것: /d2/gripper/state (JSON gripper_state/1, 1초에 1번 + 바뀔 때)
    파라미터 virtual: 가상 그리퍼(폭이 블록에 안 막힘)면 true — 폭은 NaN, 잡힘 = 닫는 방향 명령이었는지로 답한다.
    """

    def __init__(self):
        """설정을 읽고 드라이버 클라이언트·서비스·상태 방송을 만든다."""
        super().__init__('gripper')
        with open(os.path.join(get_package_share_directory('d2_bringup'), 'config', 'robot.yaml')) as f:
            cfg = yaml.safe_load(f)
        g = cfg['gripper']
        self.offset_m, self.tol_m, self.settle_s = g['feedback_offset_m'], g['check_tolerance_m'], g['settle_timeout_s']
        self.cmd_offset_m = g['command_offset_m']
        self.grasp_widths = list(cfg['grasp_width_m'].values())
        self.virtual = self.declare_parameter('virtual', False).value
        cb = ReentrantCallbackGroup()
        self.drv = self.create_client(SetCommand, DRIVER_SRV, callback_group=cb)
        self.create_subscription(JointState, FEEDBACK_TOPIC, self._on_feedback, 10, callback_group=cb)
        self.state_pub = self.create_publisher(String, '/d2/gripper/state', 10)
        self.create_service(GripperCommand, '/d2/gripper/command', self._on_command, callback_group=cb)
        self.create_timer(1.0, self._publish_state, callback_group=cb)
        self.lock = threading.Lock()      # 명령은 하나씩 (드라이버가 'i'/'d' 때 직전 폭을 다시 보내서 섞이면 안 된다)
        self.angle = None                 # (finger_joint 각 rad, 받은 시각)
        self.force_n = None               # 드라이버에 지금 걸린 힘 (모르면 None -> 0 N 으로 내린 뒤 맞춘다)
        self.last_cmd_m = None
        self.width_m, self.grasped = float('nan'), False

    def _on_feedback(self, msg):
        """드라이버 joint_states 에서 손가락 각을 적는다."""
        if 'finger_joint' in msg.name:
            self.angle = (msg.position[msg.name.index('finger_joint')], time.monotonic())

    def read_width(self):
        """지금 실제 손가락 끝 사이 폭 (m) = 표시 폭 - 오프셋. 1 s 안에 받은 값이 없으면 NaN."""
        if self.angle is None or time.monotonic() - self.angle[1] > FEEDBACK_MAX_AGE_S:
            return float('nan')
        return rg2_display_width_m(self.angle[0]) - self.offset_m

    def _send(self, command):
        """드라이버에 글자 명령 하나를 보낸다 ('o'·'c'·폭 1/10 mm·'i'·'d'). 반환: 드라이버가 받았으면 True."""
        if not self.drv.wait_for_service(timeout_sec=2.0):
            return False
        fut = self.drv.call_async(SetCommand.Request(command=command))
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.005)
        return bool(fut.done() and fut.result() and fut.result().success)

    def _set_force(self, force_n):
        """드라이버 힘을 force_n 으로 맞춘다. 처음에는 0 N 까지 내린 뒤 2.5 N 씩 올린다 (드라이버가 숫자를 안 받아서).

        'i'/'d' 마다 직전 폭 명령이 새 힘으로 다시 나간다 — 그래서 폭 명령을 보낸 뒤에 부른다.
        가상 그리퍼는 'i'/'d' 를 모르므로 실패해도 넘어간다.
        """
        target = round(force_n / FORCE_STEP_N)
        if self.force_n is None:
            cmds = ['d'] * FORCE_RESET_STEPS + ['i'] * target
        else:
            diff = target - round(self.force_n / FORCE_STEP_N)
            cmds = ['i' if diff > 0 else 'd'] * abs(diff)
        sent = all([self._send(c) for c in cmds])
        if cmds and not sent and not self.virtual:
            self.get_logger().warn('힘 명령(i/d)을 드라이버가 받지 않았다')
        self.force_n = target * FORCE_STEP_N

    def _wait_still(self):
        """폭이 멈출 때까지 (STILL_S 동안 STILL_M 미만) 기다린다. 반환: 멈춘 폭 (m), 폭을 모르면 NaN."""
        time.sleep(STILL_S)
        end, prev = time.monotonic() + self.settle_s, self.read_width()
        while time.monotonic() < end:
            time.sleep(STILL_S)
            w = self.read_width()
            if not math.isnan(w) and not math.isnan(prev) and abs(w - prev) < STILL_M:
                return w
            prev = w
        return self.read_width()

    def _on_command(self, req, res):
        """폭 width_m 로 움직이고 다 멈춘 뒤 답한다.

        바깥 영향: 그리퍼가 움직인다. 반환: success = 드라이버가 받았는지, grasped·width_m = 멈춘 뒤 상태.
        드라이버가 없거나 답이 없으면 success=false, reason 에 이유.
        """
        with self.lock:
            # 요청은 실제 손가락 끝 사이 폭. 드라이버 명령은 그보다 command_offset_m 만큼 크게 줘야 그 폭이 된다 (10/6 캘리퍼스)
            cmd_m = req.width_m + self.cmd_offset_m if req.width_m > 0 else 0.0
            cmd = str(int(round(min(MAX_WIDTH_M, max(0.0, cmd_m)) * 10000)))   # 드라이버 단위 1/10 mm
            closing = self.last_cmd_m is not None and req.width_m < self.last_cmd_m
            if not self._send(cmd):
                res.success, res.reason, res.grasped, res.width_m = False, 'GRIPPER_NO_RESPONSE', False, float('nan')
                return res
            self.last_cmd_m = req.width_m
            if self.force_n is None or abs(self.force_n - req.force_n) >= FORCE_STEP_N / 2:
                self._set_force(req.force_n)
            w = self._wait_still()
            if self.virtual:
                # 가상 그리퍼는 블록에 막히지 않고 명령 폭까지 닫힌다: 폭으로는 잡힘을 알 수 없다
                w, grasped = float('nan'), closing
            else:
                grasped = (not math.isnan(w) and w > req.width_m + GRASP_MARGIN_M
                           and any(abs(w - gw) <= self.tol_m for gw in self.grasp_widths))
            res.success, res.reason, res.grasped, res.width_m = True, '', grasped, w
            if math.isnan(w) and not self.virtual:
                res.success, res.reason = False, 'NO_FEEDBACK'
            changed = grasped != self.grasped
            self.width_m, self.grasped = w, grasped
        if changed:
            self._publish_state()
        self.get_logger().info(f'폭 명령 {req.width_m * 1000:.1f} mm -> 실제 '
                               f'{"모름" if math.isnan(w) else f"{w * 1000:.1f} mm"}, 잡힘 {grasped}')
        return res

    def _publish_state(self):
        """/d2/gripper/state 에 지금 폭·잡힘을 JSON gripper_state/1 로 보낸다 (멈춘 뒤 쥔 블록 확인용, D-12)."""
        w = self.read_width() if not self.virtual else float('nan')
        self.state_pub.publish(String(data=json.dumps(
            {'schema': 'gripper_state/1', 'stamp': time.time(),
             'width_m': None if math.isnan(w) else round(w, 4), 'grasped': self.grasped})))


def main():
    """그리퍼 노드를 켠다. 서비스 안에서 드라이버 답을 기다리므로 여러 스레드 executor 로 돌린다."""
    rclpy.init()
    node = GripperNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

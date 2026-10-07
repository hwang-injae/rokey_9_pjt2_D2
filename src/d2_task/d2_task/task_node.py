# -*- coding: utf-8 -*-
"""task 노드 — TaskManager(작업 관리자 상태표)를 ROS 2 에 잇는다. 로봇 PC 에서 돈다 (W044).

웹 화면 · 음성은 웹 PC 에 있고 다리(bridge)가 ROS 이름 그대로 대신 부른다(E-26~E-28). 이 노드는 MQTT 를 모른다.
받는 것: /d2/hmi/command (HmiCommand 서비스), /d2/hmi/intent (JSON intent/1), /d2/safety/state (JSON safety_state/1),
        /d2/gripper/state (JSON gripper_state/1)
제공하는 것: /d2/task/check_design (JsonQuery — 검사 묶음 DesignChecker, 요청 = blocks/1 글자, 응답 = check_result/1 글자)
부르는 것: /d2/motion/move_to (MoveTo), /d2/vision/check_progress (CheckProgress), /d2/motion/pick_place (액션 PickPlace)
내보내는 것: /d2/task/state (JSON state/1), /d2/task/progress (JSON progress/1) — 늦게 붙는 쪽(다리)도 마지막 값을 받게 TRANSIENT_LOCAL
레시피(1차): ROS 파라미터 recipe_dir 아래 <design_id>.recipe.json (assembly.recipe/1.0). get_design 은 W119 뒤.
바깥 영향: pick_place · move_to 를 통해 로봇이 움직인다. 이 노드가 팔을 직접 움직이지는 않는다.
Ctrl+C: 기다리는 중인 모든 호출이 빠져나오고 진행 중인 pick_place 목표를 취소한다(서기는 pick_place 가 한다).
"""
import json
import logging
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from d2_interfaces.action import PickPlace
from d2_interfaces.srv import CheckProgress, HmiCommand, JsonQuery, MoveTo
from d2_safety.safe_stop import init_ros
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from d2_task.design_checker import DesignChecker
from d2_task.task_manager import TaskManager, wait_until

LOOP_S = 0.1          # 작업 스레드가 한 단계 하고 쉬는 간격. pick_place_node 의 main 반복과 같다(정책 숫자가 아니다)
CANCEL_WAIT_S = 1.0   # Ctrl+C 때 취소 요청이 나갈 때까지 기다리는 시간. pick_place_node 의 executor 종료 대기와 같다
# 정지 노드 · 다리와 같게: 마지막 값만 의미 있고, 늦게 붙는 쪽도 받는다 (IRD 4.1, MQTT retained 자리)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)


def load_robot_yaml():
    """d2_bringup 의 config/robot.yaml 을 읽는다. 없으면 예외(설정 없이 로봇을 움직이게 하지 않는다)."""
    path = os.path.join(get_package_share_directory('d2_bringup'), 'config', 'robot.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def to_pose(pose):
    """TaskPlanner 의 (xyz m, 쿼터니언 xyzw) 를 geometry_msgs/Pose 로 바꾼다."""
    (x, y, z), (qx, qy, qz, qw) = pose
    out = Pose()
    out.position.x, out.position.y, out.position.z = x, y, z
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = qx, qy, qz, qw
    return out


class TaskNode(Node):
    """TaskManager 의 io(바깥 일)를 ROS 로 구현하고, 콜백을 TaskManager 에 넘긴다.

    기다림은 모두 wait_until 을 거친다 — 정지 신호나 Ctrl+C 가 오면 future.result() 처럼 막히지 않고 바로 빠져나온다.
    """

    def __init__(self):
        """설정을 읽고 서비스 · 클라이언트 · 구독 · 방송을 만든다. 파라미터 recipe_dir 은 비어 있으면 설계를 못 고른다."""
        super().__init__('task')
        self.declare_parameter('recipe_dir', '')
        cb = ReentrantCallbackGroup()
        cfg = load_robot_yaml()
        self.manager = TaskManager(cfg, self)
        self.checker = DesignChecker(cfg)         # 변환기 ①(한세교 W110)이 정해지면 blocks_to_recipe 인자로 붙인다
        self._active = None                       # 진행 중인 pick_place 목표 핸들(취소용)
        self._active_lock = threading.Lock()
        self.move_cli = self.create_client(MoveTo, '/d2/motion/move_to', callback_group=cb)
        self.check_cli = self.create_client(CheckProgress, '/d2/vision/check_progress', callback_group=cb)
        self.pick_cli = ActionClient(self, PickPlace, '/d2/motion/pick_place', callback_group=cb)
        self.state_pub = self.create_publisher(String, '/d2/task/state', LATCHED_QOS)
        self.progress_pub = self.create_publisher(String, '/d2/task/progress', LATCHED_QOS)
        self.create_subscription(String, '/d2/safety/state', self._on_safety, LATCHED_QOS, callback_group=cb)
        self.create_subscription(String, '/d2/gripper/state', self._on_gripper, 10, callback_group=cb)
        self.create_subscription(String, '/d2/hmi/intent', self._on_intent, 10, callback_group=cb)
        self.create_service(HmiCommand, '/d2/hmi/command', self._on_command, callback_group=cb)
        # 검사 요청끼리는 순서대로, 안전 · 그리퍼 · 상태표 콜백과는 따로 — 계산이 길어도 그쪽을 막지 않는다
        self.create_service(JsonQuery, '/d2/task/check_design', self._on_check_design,
                            callback_group=MutuallyExclusiveCallbackGroup())

    # ---------- 콜백 → TaskManager ----------
    def _json(self, msg):
        """JSON 토픽 글자를 dict 로. 깨졌으면 로그만 남기고 None."""
        try:
            return json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f'JSON 이 아니라서 버린다: {msg.data[:80]!r}')
            return None

    def _on_safety(self, msg):
        """/d2/safety/state 를 TaskManager 에 넘긴다."""
        st = self._json(msg)
        if st is not None:
            self.manager.on_safety(st)

    def _on_gripper(self, msg):
        """/d2/gripper/state 를 TaskManager 에 넘긴다."""
        st = self._json(msg)
        if st is not None:
            self.manager.on_gripper(st)

    def _on_intent(self, msg):
        """/d2/hmi/intent(음성 의도)를 TaskManager 에 넘긴다."""
        d = self._json(msg)
        if d is not None:
            self.manager.on_intent(d.get('intent'), d.get('design_id'))

    def _on_command(self, req, res):
        """/d2/hmi/command 요청을 TaskManager 에 넘기고 바로 답한다."""
        res.success, res.reason = self.manager.command(req.cmd, req.design_id)
        return res

    def _on_check_design(self, req, res):
        """/d2/task/check_design 요청을 DesignChecker 에 넘기고 답한다. 작업 관리자 상태 · 로봇에는 손대지 않는다."""
        res.success, res.reason, res.response_json = self.checker.handle_json(req.request_json)
        return res

    # ---------- TaskManager 의 io ----------
    def load_recipe(self, design_id):
        """recipe_dir/<design_id>.recipe.json 을 읽는다. 파라미터가 비었거나 파일이 없거나 이름이 경로를 가리키면 None."""
        recipe_dir = self.get_parameter('recipe_dir').value
        if not recipe_dir or not design_id or any(c in design_id for c in '/\\') or design_id.startswith('.'):
            return None
        try:
            with open(os.path.join(recipe_dir, f'{design_id}.recipe.json'), encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def services_ready(self):
        """아직 안 떠 있는 서버 이름들. 기다리지 않고 지금 보이는 것만 본다."""
        waiting = []
        if not self.move_cli.service_is_ready():
            waiting.append('/d2/motion/move_to')
        if not self.check_cli.service_is_ready():
            waiting.append('/d2/vision/check_progress')
        if not self.pick_cli.server_is_ready():
            waiting.append('/d2/motion/pick_place')
        return waiting

    def _call(self, client, request, should_abort):
        """서비스를 부르고 답을 기다린다. 기다리는 중 중단 신호가 오면 요청을 버리고 None."""
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not wait_until(done, should_abort):
            client.remove_pending_request(future)
            return None
        return future.result()

    def move_to(self, target, should_abort):
        """/d2/motion/move_to. 반환: (성공, reason). 중단되면 (False, 'STOPPED')."""
        req = MoveTo.Request()
        req.target = target
        res = self._call(self.move_cli, req, should_abort)
        return (False, 'STOPPED') if res is None else (res.success, res.reason)

    def check_progress(self, block_ids, should_abort):
        """/d2/vision/check_progress. 반환: (성공, reason, {block_id: {state, dx_m, dy_m, dz_m, top_z_m}}). 중단되면 (False, 'STOPPED', {})."""
        req = CheckProgress.Request()
        req.block_ids = list(block_ids)
        res = self._call(self.check_cli, req, should_abort)
        if res is None:
            return False, 'STOPPED', {}
        if not res.success:
            return False, res.reason, {}
        n = len(res.block_ids)
        if not all(len(a) == n for a in (res.states, res.dx_m, res.dy_m, res.dz_m, res.top_z_m)):
            return False, 'ERROR', {}               # 결과 배열 길이가 서로 다르면 어느 칸이 어느 블록인지 모른다
        rows = {res.block_ids[i]: {'state': res.states[i], 'dx_m': res.dx_m[i], 'dy_m': res.dy_m[i],
                                   'dz_m': res.dz_m[i], 'top_z_m': res.top_z_m[i]} for i in range(n)}
        return True, '', rows

    def pick_place(self, goal, should_abort):
        """블록 1개 pick_place 액션을 보내고 결과를 기다린다. 반환: (성공, reason).

        기다리는 중 중단 신호가 오면 목표를 취소하고 (False, 'STOPPED') 또는 (False, 'CANCELED'). 취소가 늦게 받아져도 취소한다.
        """
        g = PickPlace.Goal()
        g.block_id, g.supply_slot, g.grasp = goal['block_id'], goal['supply_slot'], goal['grasp']
        g.pick_pose, g.place_pose = to_pose(goal['pick_pose']), to_pose(goal['place_pose'])
        sent = self.pick_cli.send_goal_async(g)
        accepted = threading.Event()
        sent.add_done_callback(lambda _f: accepted.set())
        try:
            if not wait_until(accepted, should_abort):
                sent.add_done_callback(self._cancel_late)      # 아직 답이 없던 목표는 받아지는 대로 바로 취소
                return False, 'STOPPED'
        except KeyboardInterrupt:                              # Ctrl+C: 받아지는 대로 취소하고 끝낸다
            sent.add_done_callback(self._cancel_late)
            raise
        handle = sent.result()
        if not handle.accepted:
            return False, 'ERROR'
        with self._active_lock:
            self._active = handle
        finished = threading.Event()
        result = handle.get_result_async()
        result.add_done_callback(lambda _f: finished.set())
        try:
            if not wait_until(finished, should_abort):
                handle.cancel_goal_async()
                return False, 'CANCELED'
            r = result.result().result
            return r.success, r.reason
        except KeyboardInterrupt:                              # Ctrl+C: _active 를 지우기 전에 목표를 취소한다
            self.cancel_active()
            raise
        finally:
            with self._active_lock:
                self._active = None

    @staticmethod
    def _cancel_late(sent):
        """보낼 때 답을 못 기다린 목표가 나중에 받아졌으면 곧바로 취소한다."""
        handle = sent.result()
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()

    def cancel_active(self):
        """Ctrl+C 때: 진행 중인 pick_place 목표의 취소 요청이 나갈 때까지 잠깐 기다린다."""
        with self._active_lock:
            handle = self._active
        if handle is None:
            return
        sent = threading.Event()
        handle.cancel_goal_async().add_done_callback(lambda _f: sent.set())
        sent.wait(CANCEL_WAIT_S)

    def publish_state(self, msg):
        """/d2/task/state 로 state/1 JSON 을 보낸다."""
        self.state_pub.publish(String(data=json.dumps(msg, ensure_ascii=False, allow_nan=False)))

    def publish_progress(self, msg):
        """/d2/task/progress 로 progress/1 JSON 을 보낸다(안 잰 값은 이미 null)."""
        self.progress_pub.publish(String(data=json.dumps(msg, ensure_ascii=False, allow_nan=False)))


def main():
    """task 노드를 켠다. executor 는 다른 스레드에서 돌리고, 이 스레드가 상태표를 한 단계씩 돌린다. Ctrl+C 면 진행 중인 집기·놓기를 취소한다."""
    logging.basicConfig(level=logging.INFO, format='[%(name)s] %(message)s')
    init_ros()
    node = TaskNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.get_logger().info('[작업 관리자] 준비 끝')
        while rclpy.ok():
            node.manager.run_once()
            time.sleep(LOOP_S)
    except KeyboardInterrupt:
        node.manager.shutdown()
        node.cancel_active()
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

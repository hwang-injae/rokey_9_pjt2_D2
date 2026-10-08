# -*- coding: utf-8 -*-
"""task 노드 — TaskManager(작업 관리자 상태표)를 ROS 2 에 잇는다. 로봇 PC 에서 돈다 (W044).

웹 화면 · 음성은 웹 PC 에 있고 다리(bridge)가 ROS 이름 그대로 대신 부른다(E-26~E-28). 이 노드는 MQTT 를 모른다.
받는 것: /d2/hmi/command (HmiCommand 서비스), /d2/hmi/intent (JSON intent/1), /d2/safety/state (JSON safety_state/1),
        /d2/gripper/state (JSON gripper_state/1), /d2/hmi/alive (웹 생존 신호, 로컬 개발은 감시하지 않음)
제공하는 것: /d2/task/check_design (JsonQuery — 검사 묶음 DesignChecker, 요청 = blocks/1 글자, 응답 = check_result/1 글자)
부르는 것: /d2/motion/move_to (MoveTo), /d2/vision/check_progress (CheckProgress), /d2/motion/pick_place (액션 PickPlace),
        /d2/safety/stop (StopRequest — 시간 초과 · Ctrl+C 때 먼저 정지 요청),
        /d2/vision/scan_capture · scan_infer (JsonQuery — 촬영 점군 수집과 추론),
        /d2/hmi/save_build (JsonQuery — 끝난 조립의 build/1 요약, 저장이 확인될 때까지 TaskManager 가 들고 있고 여기서 비동기로 보낸다)
내보내는 것: /d2/task/state (JSON state/1), /d2/task/progress (JSON progress/1) — 늦게 붙는 쪽(다리)도 마지막 값을 받게 TRANSIENT_LOCAL
        /d2/task/scan_result (JSON scan_result/1 — 구조 검사·저장은 HMI에서 진행)
기록(CSV): ROS 파라미터 log_dir 아래 <run_id>.csv — 기본은 홈 아래 d2_data/runs(저장소 밖), `~` 는 홈으로 바뀐다. 빈 값을 주면 파일 기록이 꺼지고 run_id 만 만든다
설계 조회: 설계 선택 때(출발은 받아 둔 설계, E-55 ①) /d2/hmi/get_design (JsonQuery, design/1) 을 비동기로 부르고 timeout.service_s 안에 답이 없으면 실패로 본다.
        원격 조회가 실패해도 로컬 파일로 몰래 대신하지 않는다. 웹 없이 개발할 때만 파라미터 design_source:=local 로 명시하고
        recipe_dir 아래 <design_id>_recipe.json · _structure.json 을 읽는다. 옮기는 동안 옛 .recipe.json 도 받는다(E-52).
바깥 영향: pick_place · move_to 를 통해 로봇이 움직인다. 이 노드가 팔을 직접 움직이지는 않는다.
Ctrl+C: 중단 플래그 · 목표 취소 · 정지 요청 뒤에만 파일 기록을 마무리한다. 시간 초과도 목표 취소와 정지 요청을 먼저 보낸다.
"""
import json
import logging
import os
import threading
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from d2_interfaces.action import PickPlace
from d2_interfaces.srv import CheckProgress, HmiCommand, JsonQuery, MoveTo, StopRequest
from d2_safety.safe_stop import init_ros
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from d2_task.build_sender import BuildSender
from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument
from d2_task.run_logger import RunLogger
from d2_task.task_manager import TaskManager, wait_until

LOOP_S = 0.1          # 작업 스레드가 한 단계 하고 쉬는 간격. pick_place_node 의 main 반복과 같다(정책 숫자가 아니다)
SAVE_POLL_S = 1.0       # 보낼 요약이 있는지 · 답이 늦었는지 살피는 간격. 제한 시간(timeout.service_s)이 아니라 확인 주기일 뿐이다
LOG_FLUSH_S = 2.0      # 끝낼 때 기록 파일에 남은 줄이 쓰일 때까지 기다리는 한도. 취소 요청을 보낸 뒤에만 기다린다
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
        """설정과 ROS 통신을 만든다. 로컬 설계는 design_source=local 과 recipe_dir 을 함께 주며, log_dir 이 비면 파일을 안 쓴다."""
        super().__init__('task')
        self.declare_parameter('recipe_dir', '')         # design_source:=local 일 때만 쓰는 레시피 폴더(개발용)
        self.declare_parameter('design_source', 'remote')  # remote = /d2/hmi/get_design(기본) · local = recipe_dir 파일(웹 없이 개발할 때 명시)
        self.declare_parameter('log_dir', str(Path.home() / 'd2_data' / 'runs'))   # 조립 기록(CSV) 폴더 — 저장소 밖. 빈 값 = 파일 기록 끔
        cb = ReentrantCallbackGroup()
        cfg = load_robot_yaml()
        logger = RunLogger(self.get_parameter('log_dir').value)
        if logger.enabled:
            self.get_logger().info(f'[작업 관리자] 조립 기록 폴더: {logger.log_dir}')
        else:
            self.get_logger().warn('[작업 관리자] 조립 기록 파일이 꺼져 있다(log_dir 이 비어 있음) — run_id 와 요약만 만든다')
        self.manager = TaskManager(cfg, self, logger,
                                   monitor_hmi=self.get_parameter('design_source').value != 'local')
        self.checker = DesignChecker(cfg)         # 변환기 ①(한세교 W110)이 정해지면 blocks_to_recipe 인자로 붙인다
        self._active = None                       # 진행 중인 pick_place 목표 핸들(취소용)
        self._active_lock = threading.Lock()
        self.move_cli = self.create_client(MoveTo, '/d2/motion/move_to', callback_group=cb)
        self.check_cli = self.create_client(CheckProgress, '/d2/vision/check_progress', callback_group=cb)
        self.pick_cli = ActionClient(self, PickPlace, '/d2/motion/pick_place', callback_group=cb)
        self.save_cli = self.create_client(JsonQuery, '/d2/hmi/save_build', callback_group=cb)
        self.design_cli = self.create_client(JsonQuery, '/d2/hmi/get_design', callback_group=cb)
        self.capture_cli = self.create_client(JsonQuery, '/d2/vision/scan_capture', callback_group=cb)
        self.infer_cli = self.create_client(JsonQuery, '/d2/vision/scan_infer', callback_group=cb)
        self.stop_cli = self.create_client(StopRequest, '/d2/safety/stop', callback_group=cb)
        self.service_s = cfg['timeout']['service_s']      # 서비스 한 번의 제한 시간 · save_build 다시 보내기 간격 (robot.yaml)
        self.move_to_s = cfg['timeout']['move_to_s']
        self.pick_place_s = cfg['timeout']['pick_place_s']
        self.sender = BuildSender(self.manager, self._call_save, self.save_cli.remove_pending_request,
                                  self.save_cli.service_is_ready, self.service_s)
        self.state_pub = self.create_publisher(String, '/d2/task/state', LATCHED_QOS)
        self.progress_pub = self.create_publisher(String, '/d2/task/progress', LATCHED_QOS)
        self.scan_pub = self.create_publisher(String, '/d2/task/scan_result', LATCHED_QOS)
        self.create_subscription(String, '/d2/safety/state', self._on_safety, LATCHED_QOS, callback_group=cb)
        self.create_subscription(String, '/d2/gripper/state', self._on_gripper, 10, callback_group=cb)
        self.create_subscription(String, '/d2/hmi/intent', self._on_intent, 10, callback_group=cb)
        self.create_subscription(String, '/d2/hmi/alive', self._on_hmi_alive, 10, callback_group=cb)
        self.create_service(HmiCommand, '/d2/hmi/command', self._on_command, callback_group=cb)
        self.create_timer(SAVE_POLL_S, self.sender.poll, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(LOOP_S, self.manager.poll_hmi, callback_group=MutuallyExclusiveCallbackGroup())
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

    def _on_hmi_alive(self, msg):
        """웹 생존 신호를 넘긴다. 깨진 JSON · 객체가 아닌 값은 갱신하지 않으며 이동을 중단하지 않는다."""
        body = self._json(msg)
        self.manager.on_hmi_alive(body)

    def _on_command(self, req, res):
        """/d2/hmi/command 를 처리한다. 설계 조회가 있으면 service_s 초 안에서 기다린 뒤 결과를 답한다."""
        res.success, res.reason = self.manager.command(req.cmd, req.design_id)
        return res

    def _on_check_design(self, req, res):
        """/d2/task/check_design 요청을 DesignChecker 에 넘기고 답한다. 작업 관리자 상태 · 로봇에는 손대지 않는다."""
        res.success, res.reason, res.response_json = self.checker.handle_json(req.request_json)
        return res

    # ---------- 결과 저장(save_build) ----------
    def _call_save(self, summary):
        """build/1 요약을 /d2/hmi/save_build 로 비동기로 보낸다. 반환: future."""
        req = JsonQuery.Request()
        req.request_json = json.dumps(summary, ensure_ascii=False, allow_nan=False)
        return self.save_cli.call_async(req)

    # ---------- TaskManager 의 io ----------
    def get_design(self, design_id, should_abort):
        """설계 조회. 반환: (ok, reason, design dict 또는 None). design_source 에 따라 원격(기본) 또는 로컬 파일 하나만 쓴다.

        원격: /d2/hmi/get_design 에 {"design_id"} 를 보내고 timeout.service_s 안에 답을 기다린다(정지 신호가 오면 바로 빠져나옴).
        시간이 지나면 요청을 버리고 (False, 'TIMEOUT'). 서버가 없으면 (False, 'ERROR'). 답이 늦게 와도 아무도 받지 않는다.
        """
        if self.get_parameter('design_source').value == 'local':
            return self._local_design(design_id)
        if not self.design_cli.service_is_ready():
            return False, 'ERROR', None
        req = JsonQuery.Request()
        req.request_json = json.dumps({'design_id': design_id}, ensure_ascii=False)
        future = self.design_cli.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not wait_until(done, should_abort, timeout_s=self.service_s):
            self.design_cli.remove_pending_request(future)
            return False, ('STOPPED' if should_abort() else 'TIMEOUT'), None
        res = future.result()
        if not res.success:
            return False, res.reason or 'ERROR', None
        try:
            return True, '', json.loads(res.response_json)
        except ValueError:
            return False, 'ERROR', None

    def _local_design(self, design_id):
        """개발용 레시피를 design/1 로 읽는다. 새 _recipe/_structure 우선, 없으면 옛 .recipe. 파일·형식 오류는 실패 반환."""
        recipe_dir = self.get_parameter('recipe_dir').value
        if not recipe_dir or not design_id or any(c in design_id for c in '/\\') or design_id.startswith('.'):
            return False, '', None
        try:
            return True, '', RecipeDocument.load(recipe_dir, design_id).design(design_id)
        except (OSError, ValueError):
            return False, '', None

    def services_ready(self, scan=False):
        """조립 또는 스캔에 필요한 서버 중 아직 안 떠 있는 이름들. 기다리지 않고 확인하며 바깥 메시지는 보내지 않는다."""
        waiting = []
        if not self.move_cli.service_is_ready():
            waiting.append('/d2/motion/move_to')
        if scan:
            for client, name in ((self.capture_cli, '/d2/vision/scan_capture'), (self.infer_cli, '/d2/vision/scan_infer')):
                if not client.service_is_ready():
                    waiting.append(name)
            return waiting
        if not self.check_cli.service_is_ready():
            waiting.append('/d2/vision/check_progress')
        if not self.pick_cli.server_is_ready():
            waiting.append('/d2/motion/pick_place')
        return waiting

    def _scan_query(self, client, body, should_abort):
        """스캔 JsonQuery를 service_s 초 안에 처리한다. 단위 없음, 반환 (success, reason, 응답 dict).

        JSON 깨짐·객체 아님·유한하지 않은 수는 ERROR. 서버 없음은 ERROR, 정지·시간 초과는 공통 reason으로 돌려준다.
        """
        if not client.service_is_ready():
            return False, 'ERROR', None
        req = JsonQuery.Request()
        req.request_json = json.dumps(body, allow_nan=False)
        res, why = self._call(client, req, should_abort, self.service_s)
        if res is None:
            return False, why or 'ERROR', None
        if not res.success:
            return False, res.reason or 'SCAN_FAILED', None
        try:
            response = json.loads(res.response_json)
            json.dumps(response, allow_nan=False)
            if not isinstance(response, dict):
                raise ValueError('스캔 응답이 JSON 객체가 아니다')
            return True, '', response
        except (ValueError, TypeError):
            return False, 'ERROR', None

    def scan_capture(self, pose_id, run_id, should_abort):
        """촬영 자세 이름·run_id를 비전에 보내 점군을 모은다. 단위 없음, 제한 시간·실패는 _scan_query와 같다."""
        return self._scan_query(self.capture_cli, dict(pose_id=pose_id, run_id=run_id), should_abort)

    def scan_infer(self, run_id, should_abort):
        """run_id의 점군 추론을 요청한다. 결과 좌표는 blocks/1의 mm, 제한 시간·실패는 _scan_query와 같다."""
        return self._scan_query(self.infer_cli, dict(run_id=run_id), should_abort)

    def publish_scan_result(self, body):
        """완성된 scan_result/1(mm)을 retained 성격의 ROS 토픽으로 방송한다. NaN·Infinity는 직렬화 오류로 거절한다."""
        self.scan_pub.publish(String(data=json.dumps(body, ensure_ascii=False, allow_nan=False)))

    def _call(self, client, request, should_abort, timeout_s):
        """서비스를 비동기로 부르고 timeout_s 초까지 기다린다. 반환: (응답, reason), 실패 때 응답은 None.

        중단 · 시간 초과 때 future 를 추적에서 지운다. 이동 서비스는 이 조작으로 로봇이 서지 않으므로 호출자가 정지 요청을 해야 한다.
        """
        if should_abort():
            return None, 'STOPPED'
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        try:
            if not wait_until(done, should_abort, timeout_s=timeout_s):
                client.remove_pending_request(future)
                return None, ('STOPPED' if should_abort() else 'TIMEOUT')
            return future.result(), ''
        except KeyboardInterrupt:
            client.remove_pending_request(future)
            raise
        except Exception as e:
            client.remove_pending_request(future)
            self.get_logger().error(f'서비스 응답 실패: {e!r}')
            return None, 'ERROR'

    def move_to(self, target, should_abort, speed_ratio=1.0):
        """정해진 자세로 이동한다. speed_ratio = 평소 속도 대비 비율. move_to_s 초 초과는 TIMEOUT(관리자가 정지 요청).

        반환: (성공, reason). 중단 때 STOPPED, 통신 실패 때 ERROR. ROS 요청의 시간 초과만으로 이동이 취소되지는 않는다.
        """
        req = MoveTo.Request()
        req.target, req.speed_ratio = target, speed_ratio
        res, why = self._call(self.move_cli, req, should_abort, self.move_to_s)
        return (False, why or 'ERROR') if res is None else (res.success, res.reason)

    def request_stop(self, reason):
        """/d2/safety/stop 에 source=task 와 reason 을 보낸다. 반환: (접수, 설명); 물리적 정지 완료 응답은 아니다.

        service_s 초 안에 답이 없거나 서버가 없으면 실패를 알린다. 종료 중에도 executor 가 살아 있는 동안 먼저 정지 요청을 보낸다.
        """
        if not self.stop_cli.service_is_ready():
            return False, '정지 서비스가 없다. 펜던트로 정지를 확인하세요'
        req = StopRequest.Request()
        req.source, req.reason = 'task', reason
        res, why = self._call(self.stop_cli, req, lambda: not rclpy.ok(), self.service_s)
        return (False, why or 'ERROR') if res is None else (res.success, res.message)

    def check_progress(self, block_ids, should_abort):
        """진행 확인을 service_s 초 안에 받는다. 반환: (성공, reason, 블록별 관측). 중단 · 시간 초과 · 실패 때 관측은 빈 dict."""
        req = CheckProgress.Request()
        req.design_id = self.manager.design_id     # 이름에서 _B 앞을 자르면 새 역할 이름(BACK · BEAM)이 설계 ID 를 망가뜨린다(E-52)
        req.run_id = self.manager.run_id or ''      # 손목 블록 인식은 run_id가 바뀔 때 설계를 다시 읽는다(E-60 ②). 조립 · 스캔 시작 때 정해지고 한 판 동안 같다
        req.block_ids = list(block_ids)
        res, why = self._call(self.check_cli, req, should_abort, self.service_s)
        if res is None:
            return False, why or 'ERROR', {}
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

        목표 수락부터 결과까지 합쳐 pick_place_s 초를 쓴다. 초과하면 먼저 취소하고 TIMEOUT(관리자가 정지 요청).
        중단 신호 때도 목표를 취소한다. 목표 수락이 늦어도 받아지는 즉시 취소한다.
        """
        if should_abort():
            return False, 'STOPPED'
        g = PickPlace.Goal()
        g.block_id, g.supply_slot, g.grasp = goal['block_id'], goal['supply_slot'], goal['grasp']
        g.pick_pose, g.place_pose = to_pose(goal['pick_pose']), to_pose(goal['place_pose'])
        deadline = time.monotonic() + self.pick_place_s
        sent = self.pick_cli.send_goal_async(g)
        accepted = threading.Event()
        sent.add_done_callback(lambda _f: accepted.set())
        try:
            if not wait_until(accepted, should_abort, timeout_s=max(0.0, deadline - time.monotonic())):
                sent.add_done_callback(self._cancel_late)      # 아직 답이 없던 목표는 받아지는 대로 바로 취소
                return False, ('STOPPED' if should_abort() else 'TIMEOUT')
        except KeyboardInterrupt:                              # Ctrl+C: 받아지는 대로 취소하고 끝낸다
            sent.add_done_callback(self._cancel_late)
            raise
        handle = sent.result()
        if not handle.accepted:
            return False, 'ERROR'
        with self._active_lock:
            self._active = handle
        finished = threading.Event()
        try:
            result = handle.get_result_async()
            result.add_done_callback(lambda _f: finished.set())
            if not wait_until(finished, should_abort, timeout_s=max(0.0, deadline - time.monotonic())):
                try:
                    handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().error(f'목표 취소 요청 실패: {e!r}')   # 취소 오류로 TIMEOUT 을 잃으면 정지 서비스도 건너뛴다
                return False, ('CANCELED' if should_abort() else 'TIMEOUT')
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
    """task 상태표와 executor 를 돌린다. Ctrl+C 는 목표 취소 · 정지 서비스 요청 뒤 제한 있는 기록 마무리를 한다."""
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
        node.manager.shutdown()                    # ① 중단 신호 → ② 진행 중인 목표 취소 요청 → ③ 기록 마무리(finally)
        node.cancel_active()
        ok, message = node.request_stop('CTRL_C')  # 이동 서비스는 future 를 버리는 것만으로 서지 않는다
        if not ok:
            node.get_logger().error(f'종료 때 정지 요청 실패: {message}')
    finally:
        node.manager.finalize(LOG_FLUSH_S)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

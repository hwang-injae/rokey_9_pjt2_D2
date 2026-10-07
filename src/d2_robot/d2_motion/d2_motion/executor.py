# -*- coding: utf-8 -*-
"""MoveIt2 실행기 MoveItExecutor — pick_place 노드 안에서 쓰는 클래스 (노드 아님, IRD 3장).

원본: 박진용 real_demo/jenga_lib.py Arm · recipe_demo/run_targets.py (10/6 LV1 11개 실기 성공 코드).
"""
import math
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PlanningOptions, PlanningSceneComponents, PositionConstraint, RobotState,
                             RobotTrajectory)
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from d2_motion.motion_math import column, flip_about_z, quat_from_axes, slot_block_pose

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GROUP = 'manipulator'
TILT_TOL_DEG = 15.0       # 자유 이동 중 공구가 아래 방향에서 기울 수 있는 최대 각 (쥔 블록이 쏟아지지 않게)
ARRIVE_TOL_DEG = 1.0      # 실행이 '성공'이어도 목표와 이만큼 넘게 다르면 실제로 안 움직인 것 (두산 알람·보호 정지)
ARRIVE_WAIT_S = 5.0       # 궤적이 끝난 뒤 팔이 멈출 때까지 기다리는 최대 시간
PATH_POINTS = 40          # RViz 경로 선에 쓰는 점 수 (FK 를 점마다 불러서 많으면 출발이 늦어진다)
ERROR_NAMES = {getattr(MoveItErrorCodes, n): n for n in dir(MoveItErrorCodes)
               if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)}


def make_pose(xyz, q=(0.0, 0.0, 0.0, 1.0)):
    """위치 (m) 와 쿼터니언 (x, y, z, w) -> geometry_msgs/Pose."""
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


class MoveItExecutor:
    """MoveIt2 move_group 에 붙어 IK·충돌 검사·경로 계획을 하고, 궤적을 실행하며 정지 요청을 감시한다.

    부르는 것: move_action (계획만 — 공중 이동 OMPL, 수직 직선 Pilz LIN), execute_trajectory (실행),
               compute_ik / compute_fk / check_state_validity / get_planning_scene
    받는 것: joint_states
    node 는 MultiThreadedExecutor 로 돌아야 한다 — 이 클래스는 future 가 끝나기를 기다리기만 하고 node 를 직접 돌리지 않는다.
    """

    def __init__(self, node, cfg, stopper, cb_group):
        """node: pick_place 노드, cfg: robot.yaml, stopper: SafeStop (실행 중 목표를 넣어 두면 정지 때 취소), cb_group: Reentrant."""
        self.node, self.cfg, self.stopper = node, cfg, stopper
        self.tcp = cfg['tcp_link']
        self.frame = cfg['frame_id']
        self.js = {}
        node.create_subscription(JointState, 'joint_states', self._on_js, 50, callback_group=cb_group)
        self.move_ac = ActionClient(node, MoveGroup, 'move_action', callback_group=cb_group)
        self.exec_ac = ActionClient(node, ExecuteTrajectory, 'execute_trajectory', callback_group=cb_group)
        self.fk_cli = node.create_client(GetPositionFK, 'compute_fk', callback_group=cb_group)
        self.ik_cli = node.create_client(GetPositionIK, 'compute_ik', callback_group=cb_group)
        self.sv_cli = node.create_client(GetStateValidity, 'check_state_validity', callback_group=cb_group)
        self.scene_cli = node.create_client(GetPlanningScene, 'get_planning_scene', callback_group=cb_group)
        self.path_pub = node.create_publisher(MarkerArray, '/d2/motion/path_markers', 1)
        self.q_down = None        # 홈 자세의 TCP 자세 = '공구 아래' 기준

    def _on_js(self, msg):
        """joint_states 를 받아 관절값을 적는다."""
        for n, p in zip(msg.name, msg.position):
            self.js[n] = p

    def wait_future(self, fut, timeout):
        """future 가 끝날 때까지 기다린다. 반환: 결과, timeout 초 안에 안 끝나면 None."""
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.005)
        return fut.result() if fut.done() else None

    def wait_ready(self, timeout=15.0):
        """move_group 과 joint_states 가 준비될 때까지 기다린다. 반환: 준비됐으면 True."""
        ok = self.move_ac.wait_for_server(timeout_sec=timeout) and self.exec_ac.wait_for_server(timeout_sec=timeout)
        ok = ok and all(c.wait_for_service(timeout_sec=timeout) for c in (self.fk_cli, self.ik_cli, self.sv_cli))
        end = time.monotonic() + timeout
        while ok and not all(j in self.js for j in JOINTS) and time.monotonic() < end:
            time.sleep(0.1)
        if not ok or not all(j in self.js for j in JOINTS):
            return False
        home = [math.radians(v) for v in self.cfg['home_pose']['joints_deg']]
        pose = self.fk(home)
        if pose is None:
            return False
        o = pose.orientation
        self.q_down = (o.x, o.y, o.z, o.w)
        return True

    def current(self):
        """지금 관절값 (rad, JOINTS 순서)."""
        return [self.js[j] for j in JOINTS]

    def _state(self, q):
        """관절값 q 의 RobotState. is_diff 라서 그리퍼 관절·쥔 블록은 지금 장면 그대로 쓴다."""
        st = RobotState()
        st.is_diff = True
        st.joint_state.name = list(JOINTS)
        st.joint_state.position = list(q)
        return st

    def fk(self, q):
        """관절값 q 에서 TCP 자세 (Pose). 실패하면 None."""
        req = GetPositionFK.Request()
        req.header.frame_id = self.frame
        req.fk_link_names = [self.tcp]
        req.robot_state = self._state(q)
        res = self.wait_future(self.fk_cli.call_async(req), 5.0)
        return res.pose_stamped[0].pose if res and res.pose_stamped else None

    def ik(self, xyz, quat, seed, avoid=False, timeout=0.1):
        """TCP 위치 xyz (m)·자세 quat 의 관절 해 (seed 근처). 해가 없으면 None. avoid=True 면 충돌 없는 해만."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = self.tcp
        r.avoid_collisions = avoid
        r.robot_state = self._state(seed)
        r.timeout = Duration(sec=0, nanosec=int(timeout * 1e9))
        r.pose_stamped = PoseStamped(pose=make_pose(xyz, quat))
        r.pose_stamped.header.frame_id = self.frame
        res = self.wait_future(self.ik_cli.call_async(req), 5.0)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [pos[j] for j in JOINTS]

    def solve(self, xyz, quat, seed):
        """같은 잡기의 두 대칭 자세 (TCP Z 축 180°) 중 관절이 덜 도는 충돌 없는 IK. 반환: (q, quat), 없으면 (None, None)."""
        best = None
        for q in (quat, flip_about_z(quat)):
            sol = self.ik(xyz, q, seed, avoid=True, timeout=0.2)
            if sol is not None:
                cost = sum(abs(a - b) for a, b in zip(sol, seed))
                if best is None or cost < best[0]:
                    best = (cost, sol, q)
        return (None, None) if best is None else (best[1], best[2])

    def _send_plan(self, req, plan_time, scene_objects=()):
        """MotionPlanRequest 를 move_group 에 계획만 시킨다 (scene_objects 는 이 계획에만 더하는 물체).

        반환: (시간까지 매겨진 JointTrajectory, 결과 이름). 실패면 궤적 None.
        궤적의 시간은 MoveIt 이 매긴 것 (OMPL = TOTG 응답 어댑터, Pilz = 사다리꼴 속도) 을 그대로 쓴다.
        """
        g = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=True))
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        g.planning_options.planning_scene_diff.world.collision_objects = list(scene_objects)
        gh = self.wait_future(self.move_ac.send_goal_async(g), 5.0)
        if gh is None or not gh.accepted:
            return None, '목표 거절'
        res = self.wait_future(gh.get_result_async(), plan_time + 10.0)
        if res is None:
            return None, '시간 초과'
        if res.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, ERROR_NAMES.get(res.result.error_code.val, str(res.result.error_code.val))
        return res.result.planned_trajectory.joint_trajectory, 'SUCCESS'

    def plan_line(self, b_xyz, quat, scale, start_q=None):
        """TCP 를 지금(또는 start_q) 자세에서 b_xyz·quat 까지 직선으로 가는 궤적을 Pilz LIN 으로 계획한다.

        수직으로 집기·놓기 높이에 오르내릴 때 쓴다. LIN 은 장애물을 돌아가지 않고 직선만 만든다 (막히면 실패).
        반환: (시간까지 매겨진 JointTrajectory, 결과 이름). 실패면 궤적 None.
        """
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = 'pilz_industrial_motion_planner'
        req.planner_id = 'LIN'
        req.num_planning_attempts = 1
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = scale
        req.start_state = self._state(start_q if start_q is not None else self.current())
        if start_q is not None:
            req.start_state.is_diff = False       # 미리 확인할 때는 지금 자세가 아닌 start_q 에서 출발
        target = PoseStamped(pose=make_pose(b_xyz, quat))
        target.header.frame_id = self.frame
        pc = PositionConstraint(link_name=self.tcp, weight=1.0)
        pc.header.frame_id = self.frame
        pc.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-4])]
        pc.constraint_region.primitive_poses = [target.pose]
        oc = OrientationConstraint(link_name=self.tcp, orientation=target.pose.orientation, weight=1.0,
                                   absolute_x_axis_tolerance=1e-3, absolute_y_axis_tolerance=1e-3, absolute_z_axis_tolerance=1e-3)
        oc.header.frame_id = self.frame
        req.goal_constraints = [Constraints(position_constraints=[pc], orientation_constraints=[oc])]
        return self._send_plan(req, 2.0)

    def valid(self, q, constraints=None):
        """관절값 q 가 장면(작업대·놓인 블록·쥔 블록)과 안 부딪히는지. 반환: (괜찮으면 True, 부딪힌 물체 목록)."""
        req = GetStateValidity.Request()
        req.robot_state = self._state(q)
        req.group_name = GROUP
        if constraints is not None:
            req.constraints = constraints
        res = self.wait_future(self.sv_cli.call_async(req), 2.0)
        if res is None:
            return False, ['응답 없음']
        return res.valid, sorted({f'{c.contact_body_1}|{c.contact_body_2}' for c in res.contacts})

    def down_constraint(self):
        """'공구가 아래를 향한 채' 경로 제약 (기울기 TILT_TOL_DEG 까지, 공구 축 둘레 회전은 자유)."""
        oc = OrientationConstraint()
        oc.header.frame_id = self.frame
        oc.link_name = self.tcp
        oc.orientation = Quaternion(x=self.q_down[0], y=self.q_down[1], z=self.q_down[2], w=self.q_down[3])
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = math.radians(TILT_TOL_DEG)
        oc.absolute_z_axis_tolerance = math.pi
        oc.weight = 1.0
        return Constraints(orientation_constraints=[oc])

    def padded_blocks(self, pad_m):
        """장면의 놓인 블록(blk_*)을 사방 pad_m 부풀린 상자 목록 (계획 요청에만 실어 보낸다 — 실제 장면은 안 바뀐다).

        MoveIt 은 겹치지만 않으면 아슬아슬한 길도 고른다. 장면과 실제 블록이 몇 mm 달라 등받이에 부딪힌 적이 있어서
        (10/6 실기) 공중 이동 계획 때만 여유를 둔다. 장면을 못 읽으면 빈 목록.
        """
        comp = PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        res = self.wait_future(self.scene_cli.call_async(GetPlanningScene.Request(components=comp)), 3.0)
        if res is None:
            return []
        out = []
        for o in res.scene.world.collision_objects:
            if not o.id.startswith('blk_') or not o.primitives:
                continue
            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[d + 2 * pad_m for d in o.primitives[0].dimensions])
            out.append(CollisionObject(id=f'pad_{o.id}', header=o.header, pose=o.pose, primitives=[box],
                                       primitive_poses=list(o.primitive_poses), operation=CollisionObject.ADD))
        return out

    def supply_boxes(self, pad_m, skip_slot=None):
        """공급 칸 p1~p6 에 블록이 있다고 보고, 칸마다 블록 상자를 사방 pad_m 부풀려 만든다 (계획 요청에만 싣는다).

        공급 칸 블록은 장면에 없어서, 안 넣으면 공중 이동이 다른 칸 블록(세운 블록 75 mm) 위를 낮게 지나갈 수 있다.
        skip_slot (1부터) = 지금 집으러 가거나 방금 집은 칸 — 그 칸으로는 내려가야 하므로 뺀다.
        """
        out = []
        size = [v + 2 * pad_m for v in self.cfg['block_actual_m']]
        for k in range(len(self.cfg['supply_slots'])):
            if k + 1 == skip_slot:
                continue
            center, rot = slot_block_pose(self.cfg, k + 1)
            o = CollisionObject(id=f'pad_supply_{k + 1}', operation=CollisionObject.ADD)
            o.header.frame_id = self.frame
            o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
            o.primitive_poses = [make_pose(center, quat_from_axes(*(column(rot, i) for i in range(3))))]
            out.append(o)
        return out

    def plan(self, goal, keep_down, planner, plan_time=5.0, pad_m=0.0, skip_slot=None):
        """지금 자세 -> 관절 목표 goal 을 OMPL 로 계획만 한다. 반환: (시간까지 매겨진 JointTrajectory, 결과 이름).

        속도·가속은 robot.yaml speed_scale 을 MoveIt 에 넘겨 MoveIt 이 매긴다 (joint_limits.yaml x 비율).
        pad_m > 0 이면 놓인 블록과 공급 칸 블록(skip_slot 칸 빼고)을 그만큼 부풀린 상자를 이 계획에만 더해
        여유 있는 길을 찾게 한다.
        """
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = 'ompl'
        req.planner_id = planner
        req.num_planning_attempts = 1
        req.allowed_planning_time = plan_time
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = self.cfg['speed_scale']
        req.start_state = self._state(self.current())
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v, tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0)
            for j, v in zip(JOINTS, goal)])]
        if keep_down:
            req.path_constraints = self.down_constraint()
        extra = self.padded_blocks(pad_m) + self.supply_boxes(pad_m, skip_slot) if pad_m > 0 else ()
        return self._send_plan(req, plan_time, extra)

    @staticmethod
    def positions(jt):
        """JointTrajectory -> JOINTS 순서의 관절값 목록 (rad)."""
        order = [jt.joint_names.index(j) for j in JOINTS]
        return [[p.positions[k] for k in order] for p in jt.points]

    def show_path(self, path, color):
        """관절 경로를 손가락 끝(TCP) 선으로 RViz 에 그린다 (/d2/motion/path_markers). color = (r, g, b). 실패해도 이동은 막지 않는다."""
        step = max(1, len(path) // PATH_POINTS)
        pts = []
        for q in path[::step] + [path[-1]]:
            p = self.fk(q)
            if p is not None:
                pts.append(Point(x=p.position.x, y=p.position.y, z=p.position.z))
        m = Marker(ns='path', id=0, type=Marker.LINE_STRIP, action=Marker.ADD, points=pts)
        m.header.frame_id = self.frame
        m.scale.x = 0.004
        m.color.r, m.color.g, m.color.b, m.color.a = float(color[0]), float(color[1]), float(color[2]), 1.0
        m.pose.orientation.w = 1.0
        self.path_pub.publish(MarkerArray(markers=[m]))

    def clear_path(self):
        """RViz 경로 선을 지운다."""
        self.path_pub.publish(MarkerArray(markers=[Marker(ns='path', id=0, action=Marker.DELETEALL)]))

    def execute(self, jt, halted):
        """MoveIt 이 계획한 궤적(시간 포함)을 그대로 execute_trajectory 로 실행한다.

        halted(): 멈춰야 하면 이유 글자, 아니면 None. 실행 중 계속 보고, 이유가 생기면 SafeStop 으로 바로 세운다.
        바깥 영향: 로봇 팔이 움직인다. RViz 에 경로 선을 그리고 (초록), 성공하면 지우고, 실패하면 빨강으로 남긴다.
        반환: (성공하면 True, 실패 이유 코드 또는 '').
        """
        path = self.positions(jt)
        self.show_path(path, (0.1, 0.9, 0.2))
        ok, why = self._run(jt, path[-1], halted)
        if ok:
            self.clear_path()
        else:
            self.show_path(path, (1.0, 0.1, 0.1))     # 멈춘 경로는 남겨 둔다 (어디로 가다 멈췄는지 보게)
        return ok, why

    def _run(self, jt, end_q, halted):
        """execute 의 실제 실행 부분: execute_trajectory -> 정지 감시 -> 도착 확인. 반환: (성공, 이유)."""
        last = jt.points[-1].time_from_start
        gh = self.wait_future(self.exec_ac.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=RobotTrajectory(joint_trajectory=jt))), 10.0)
        if gh is None or not gh.accepted:
            return False, 'PLAN_FAILED'
        self.stopper.gh = gh              # 정지 노드가 아니라 이 프로그램이 멈출 때 SafeStop 이 같이 취소한다
        res_f = gh.get_result_async()
        end = time.monotonic() + last.sec + last.nanosec * 1e-9 + 30.0
        while not res_f.done():
            why = halted()
            if why:
                self.stopper.stop(why)
                return False, why
            if time.monotonic() > end:
                self.stopper.stop('TIMEOUT')
                return False, 'TIMEOUT'
            time.sleep(0.01)
        self.stopper.gh = None
        if res_f.result().result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, 'PLAN_FAILED'
        # 두산 알람·보호 정지면 컨트롤러가 '성공'이라고 해도 팔이 안 움직인다: 실제 도착을 본다.
        # 제어기가 감속 구역 등으로 궤적보다 늦게 따라올 수 있어 (10/6 가상: 궤적 끝난 뒤에도 30도 넘게 따라오는 중)
        # 멈출 때까지 ARRIVE_WAIT_S 기다린 뒤 판정한다
        self.stopper.wait_still(ARRIVE_WAIT_S)
        err = max(abs(math.degrees(a - b)) for a, b in zip(self.current(), end_q))
        if err > ARRIVE_TOL_DEG:
            self.node.get_logger().error(f'목표에 도착하지 않음 (관절 차이 {err:.1f}도) — 펜던트 알람 확인')
            return False, 'TIMEOUT'
        return True, ''

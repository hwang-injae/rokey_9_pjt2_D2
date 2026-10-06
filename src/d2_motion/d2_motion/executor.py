# -*- coding: utf-8 -*-
"""MoveIt2 실행기 MoveItExecutor — pick_place 노드 안에서 쓰는 클래스 (노드 아님, IRD 3장).

원본: 박진용 real_demo/jenga_lib.py Arm · recipe_demo/run_targets.py (10/6 LV1 11개 실기 성공 코드).
"""
import math
import random
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PlanningOptions, RobotState, RobotTrajectory)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from d2_motion.motion_math import densify, flip_about_z, interp, time_parameterize

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GROUP = 'manipulator'
TILT_TOL_DEG = 15.0       # 자유 이동 중 공구가 아래 방향에서 기울 수 있는 최대 각 (쥔 블록이 쏟아지지 않게)
ARRIVE_TOL_DEG = 1.0      # 실행이 '성공'이어도 목표와 이만큼 넘게 다르면 실제로 안 움직인 것 (두산 알람·보호 정지)
JUMP_DEG = 10.0           # 직선 경로에서 이웃 IK 해가 이만큼 넘게 뛰면 자세가 뒤집힌 것으로 보고 버린다
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

    부르는 것: move_action (계획만), execute_trajectory (실행), compute_ik / compute_fk / check_state_validity
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

    def line(self, a_xyz, b_xyz, quat, seed, step=0.005):
        """a -> b 직선 (자세 quat 고정) 을 step (m) 마다 IK 로 이은 관절 경로. 이을 수 없으면 None."""
        n = max(1, int(math.ceil(math.dist(a_xyz, b_xyz) / step)))
        path, q = [], seed
        for k in range(n + 1):
            q2 = self.ik(interp(a_xyz, b_xyz, k / n), quat, q)
            if q2 is None or (path and max(abs(u - v) for u, v in zip(q2, q)) > math.radians(JUMP_DEG)):
                return None
            path.append(q2)
            q = q2
        return path

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

    def plan(self, goal, keep_down, planner, plan_time=5.0):
        """지금 자세 -> 관절 목표 goal 경로를 OMPL 로 계획만 한다. 반환: (관절 경로, 결과 이름). 실패면 경로 None."""
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
        g = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=True))
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        gh = self.wait_future(self.move_ac.send_goal_async(g), 5.0)
        if gh is None or not gh.accepted:
            return None, '목표 거절'
        res = self.wait_future(gh.get_result_async(), plan_time + 10.0)
        if res is None:
            return None, '시간 초과'
        if res.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, ERROR_NAMES.get(res.result.error_code.val, str(res.result.error_code.val))
        jt = res.result.planned_trajectory.joint_trajectory
        order = [jt.joint_names.index(j) for j in JOINTS]
        return [[p.positions[k] for k in order] for p in jt.points], 'SUCCESS'

    def _segment_ok(self, a, b, constraints, step=math.radians(1.0)):
        """관절 a -> b 직선 사이 점들이 모두 충돌·제약을 지키는지 (1° 마다 검사)."""
        n = max(1, int(math.ceil(max(abs(y - x) for x, y in zip(a, b)) / step)))
        return all(self.valid(interp(a, b, k / n), constraints)[0] for k in range(1, n))

    def shortcut(self, path, constraints, tries=15, budget=1.0):
        """OMPL 경로의 군더더기를 지름길로 다듬는다. budget 초를 넘기면 거기까지 한 결과를 돌려준다."""
        rng = random.Random(0)
        t_end = time.monotonic() + budget
        p = [list(x) for x in path]
        out, i = [p[0]], 0
        while i < len(p) - 1:
            j = len(p) - 1 if time.monotonic() < t_end else i + 1
            while j > i + 1 and not self._segment_ok(p[i], p[j], constraints):
                j = max(i + 1, (i + j) // 2) if j - i > 8 else j - 1
            out.append(p[j])
            i = j
        p = out
        for _ in range(tries):
            if len(p) < 3 or time.monotonic() > t_end:
                break
            i = rng.randrange(0, len(p) - 2)
            j = rng.randrange(i + 2, len(p))
            a, b = interp(p[i], p[i + 1], rng.random()), interp(p[j - 1], p[j], rng.random())
            if self._segment_ok(a, b, constraints):
                p = p[:i + 1] + [a, b] + p[j:]
        return p

    def execute(self, path, scale, halted):
        """관절 경로에 시간을 매겨 MoveIt 으로 실행한다.

        halted(): 멈춰야 하면 이유 글자, 아니면 None. 실행 중 계속 보고, 이유가 생기면 SafeStop 으로 바로 세운다.
        바깥 영향: 로봇 팔이 움직인다.
        반환: (성공하면 True, 실패 이유 코드 또는 '').
        """
        times, pos, vel, acc = time_parameterize(densify(path), scale)
        jt = JointTrajectory(joint_names=list(JOINTS))
        for t, p, v, a in zip(times, pos, vel, acc):
            jt.points.append(JointTrajectoryPoint(positions=list(p), velocities=list(v), accelerations=list(a),
                                                  time_from_start=Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))))
        gh = self.wait_future(self.exec_ac.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=RobotTrajectory(joint_trajectory=jt))), 10.0)
        if gh is None or not gh.accepted:
            return False, 'PLAN_FAILED'
        self.stopper.gh = gh              # 정지 노드가 아니라 이 프로그램이 멈출 때 SafeStop 이 같이 취소한다
        res_f = gh.get_result_async()
        end = time.monotonic() + times[-1] + 30.0
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
        # 두산 알람·보호 정지면 컨트롤러가 '성공'이라고 해도 팔이 안 움직인다: 실제 도착을 본다
        self.stopper.wait_still(2.0)
        err = max(abs(math.degrees(a - b)) for a, b in zip(self.current(), pos[-1]))
        if err > ARRIVE_TOL_DEG:
            self.node.get_logger().error(f'목표에 도착하지 않음 (관절 차이 {err:.1f}도) — 펜던트 알람 확인')
            return False, 'TIMEOUT'
        return True, ''

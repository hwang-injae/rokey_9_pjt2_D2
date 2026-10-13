# -*- coding: utf-8 -*-
"""MoveIt2 실행기 MoveItExecutor — pick_place 노드 안에서 쓰는 클래스 (노드 아님, IRD 3장).

원본: 박진용 real_demo/jenga_lib.py Arm · recipe_demo/run_targets.py (10/6 LV1 11개 실기 성공 코드).
"""
import math
import random
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PlanningOptions, PlanningSceneComponents, PositionConstraint, RobotState,
                             RobotTrajectory)
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
FINGER_JOINT = 'rg2_finger_joint'
# 손가락 링크 — 쥔 블록에 닿아도 충돌로 보지 않는다(장면 관리 노드의 쥔 블록 붙이기와 미리 계획해 보기가 같이 쓴다)
FINGER_LINKS = ['rg2_base_link', 'rg2_left_outer_knuckle', 'rg2_left_inner_knuckle', 'rg2_left_inner_finger',
                'rg2_right_outer_knuckle', 'rg2_right_inner_knuckle', 'rg2_right_inner_finger']
# IK 를 처음 시작점으로 못 풀면 다시 풀어 볼 시작점: 교시 자세들 + 고정된 무작위 6개(실행마다 같게 seed 0).
# 손목을 꺾은 자세는 시작점 하나로는 못 푸는 일이 있었다(10/11 가상)
EXTRA_SEED_POSES = ('home_pose', 'observe_pose', 'observe_supply_pose', 'observe_front_pose', 'observe_side_pose')
EXTRA_RANDOM_SEEDS = 6
# 특이점 · 큰 회전 막기(10/12 박진용 — 두산 매뉴얼 특이점 세 가지). IK 해 · 계획한 길의 모든 점에서 본다. 숫자는 매뉴얼에 없어 정한 여유 값.
# 팔꿈치 = 3번 관절 0° 근처(10/12 실기: J3 28° 로 가다 정지) · 손목 = 5번 관절 0° 근처 · 어깨 = 1번 축과 6번 축이 한 줄(손끝이 1번 축 가까이)
SING_J3_DEG = 15.0        # 10/12: 45° 로 두니 받침 놓기(J3 −41 ~ −43°) · 그 위 80 mm(J3 −19 ~ −24°)가 막힘 — 오전 실기에서 그 자세로 받침 경유 7번 됨
SING_J5_DEG = 15.0
SING_SHOULDER_M = 0.150
MAX_TURN_DEG = 200.0      # 자유 이동 한 번에 관절 1 ~ 6번 중 하나라도 길 따라 이보다 많이 돌면 그 길은 버린다(크게 도는 길 막기, 정한 값)
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
        rng = random.Random(0)
        self.extra_seeds = ([[math.radians(v) for v in cfg[k]['joints_deg']] for k in EXTRA_SEED_POSES if cfg.get(k)] +
                            [[rng.uniform(-math.pi, math.pi) for _ in JOINTS] for _ in range(EXTRA_RANDOM_SEEDS)])

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

    def singular(self, q, shoulder=True):
        """관절값 q 가 특이점 근처면 이유 글자, 아니면 None. shoulder=False 면 FK(서비스)가 드는 어깨 검사는 건너뛴다."""
        if abs(math.degrees(q[2])) < SING_J3_DEG:
            return f'팔꿈치 특이점(J3 {math.degrees(q[2]):.0f}°)'
        if abs(math.degrees(q[4])) < SING_J5_DEG:
            return f'손목 특이점(J5 {math.degrees(q[4]):.0f}°)'
        if shoulder:
            p = self.fk(q)
            if p is not None and math.hypot(p.position.x, p.position.y) < SING_SHOULDER_M:
                return f'어깨 특이점(손끝이 1번 축에서 {math.hypot(p.position.x, p.position.y) * 1000:.0f} mm)'
        return None

    def path_check(self, jt, line=False):
        """궤적 검사. line=True(직선) 면 모든 점이 특이점 근처가 아닌지(어깨는 3점마다 + 끝), 관절 이동이면 끝 점만 —
        두산 매뉴얼: 특이점은 직선(작업 공간) 이동에서만 문제, 관절 이동은 영향 없음. 둘 다 어느 관절이든 MAX_TURN_DEG 넘게 돌면 이유. 아니면 None."""
        idx = [jt.joint_names.index(j) for j in JOINTS]
        pts = [[p.positions[i] for i in idx] for p in jt.points]
        for k, q in enumerate(pts):
            if not line and k != len(pts) - 1:
                continue
            why = self.singular(q, shoulder=(k % 3 == 0 or k == len(pts) - 1))
            if why:
                return why
        # 관절 1 ~ 6번 모두(10/12 박진용: 손목도 크게 돌면 안 됨)
        turn = max(sum(abs(math.degrees(b[j] - a[j])) for a, b in zip(pts, pts[1:])) for j in range(6)) if len(pts) > 1 else 0.0
        if turn > MAX_TURN_DEG:
            return f'관절이 {turn:.0f}° 돌아가는 길'
        return None

    def _state(self, q, finger=None, held=None):
        """관절값 q 의 RobotState. is_diff 라서 주지 않은 그리퍼 관절·쥔 블록은 지금 장면 그대로 쓴다.

        finger = 손가락 관절 각 (rad, 주면 그 각으로), held = 쥔 블록 상자 {'size_m', 'offset_m'[, 'quat']} (TCP 기준, 주면 이 상태에만 붙임 —
        'quat' 가 있으면 그 방향, 장면 관리 held_quat 와 같음).
        둘 다 계획 요청 안에서만 쓰고 장면은 바꾸지 않는다(장면은 장면 관리 노드만 고친다).
        """
        st = RobotState()
        st.is_diff = True
        st.joint_state.name = list(JOINTS) + ([FINGER_JOINT] if finger is not None else [])
        st.joint_state.position = list(q) + ([float(finger)] if finger is not None else [])
        if held is not None:
            box = CollisionObject(id='held_check', operation=CollisionObject.ADD)
            box.header.frame_id = self.tcp
            box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in held['size_m']])]
            box.primitive_poses = [make_pose(held['offset_m'], held.get('quat') or (0.0, 0.0, 0.0, 1.0))]
            st.attached_collision_objects = [AttachedCollisionObject(link_name=self.tcp, object=box,
                                                                     touch_links=FINGER_LINKS + [self.tcp])]
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
        """같은 잡기의 두 대칭 자세 (TCP Z 축 180°) 중 관절이 덜 도는 충돌 없는 IK. 반환: (q, quat), 없으면 (None, None).

        먼저 seed 에서 풀고, 두 자세 다 못 풀 때만 extra_seeds(교시 자세 · 고정 무작위)로 다시 푼다 — 보통 자세는 첫 시도로
        끝나 시간이 늘지 않고, 손목을 꺾은 자세처럼 시작점에 따라 못 푸는 경우만 더 찾는다. 관절 이동량은 늘 seed 기준이다.
        """
        for seeds, timeout in (([seed], 0.2), (self.extra_seeds, 0.1)):
            best = None
            for q in (quat, flip_about_z(quat)):
                for s in seeds:
                    sol = self.ik(xyz, q, s, avoid=True, timeout=timeout)
                    if sol is not None and self.singular(sol) is None:
                        cost = sum(abs(a - b) for a, b in zip(sol, seed))
                        if best is None or cost < best[0]:
                            best = (cost, sol, q)
                        break
            if best is not None:
                return best[1], best[2]
        return None, None

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
        jt, why = self._send_plan(req, 2.0)
        bad = self.path_check(jt, line=True) if jt is not None else None
        return (None, bad) if bad else (jt, why)

    def valid(self, q, constraints=None, finger=None, held=None):
        """관절값 q 가 장면(작업대·놓인 블록·쥔 블록)과 안 부딪히는지. 반환: (괜찮으면 True, 부딪힌 물체 목록).

        finger · held 를 주면 그 손가락 각 · 쥔 블록 상자로 본다(_state — 미리 검사할 때, 장면은 그대로).
        """
        req = GetStateValidity.Request()
        req.robot_state = self._state(q, finger, held)
        req.group_name = GROUP
        if constraints is not None:
            req.constraints = constraints
        res = self.wait_future(self.sv_cli.call_async(req), 2.0)
        if res is None:
            return False, ['응답 없음']
        return res.valid, sorted({f'{c.contact_body_1}|{c.contact_body_2}' for c in res.contacts})

    def tcp_y_max(self):
        """자유 이동 중 손끝(TCP)이 넘지 말 base y (m) = 공급 영역 먼 쪽 끝 + motion.tcp_y_margin_m — 탑뷰에서 공급 영역 위쪽으로 넘어가지
        않게(10/13 박진용). 벽이 아니라 손끝만 본다(팔 몸통은 막지 않아 가장자리 블록도 집는다). 공급 영역이 없으면 None(제한 없음)."""
        ar = self.cfg.get('supply_area_m')
        return None if not ar else ar['y_m'][1] + self.cfg['motion'].get('tcp_y_margin_m', 0.0)

    def down_constraint(self, tol_deg=TILT_TOL_DEG):
        """'공구가 아래를 향한 채' 경로 제약 (기울기 tol_deg 까지 — 기본 TILT_TOL_DEG, 공구 축 둘레 회전은 자유)."""
        oc = OrientationConstraint()
        oc.header.frame_id = self.frame
        oc.link_name = self.tcp
        oc.orientation = Quaternion(x=self.q_down[0], y=self.q_down[1], z=self.q_down[2], w=self.q_down[3])
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = math.radians(tol_deg)
        oc.absolute_z_axis_tolerance = math.pi
        oc.weight = 1.0
        return Constraints(orientation_constraints=[oc])

    def padded_blocks(self, pad_m, skip=None):
        """장면의 놓인 블록(blk_*)을 사방 pad_m 부풀린 상자 목록 (계획 요청에만 실어 보낸다 — 실제 장면은 안 바뀐다).

        MoveIt 은 겹치지만 않으면 아슬아슬한 길도 고른다. 장면과 실제 블록이 몇 mm 달라 등받이에 부딪힌 적이 있어서
        (10/6 실기) 공중 이동 계획 때만 여유를 둔다. skip = 부풀리지 않을 물체 이름(지금 집는 블록 자신). 장면을 못 읽으면 빈 목록.
        """
        comp = PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        res = self.wait_future(self.scene_cli.call_async(GetPlanningScene.Request(components=comp)), 3.0)
        if res is None:
            return []
        out = []
        for o in res.scene.world.collision_objects:
            if not o.id.startswith('blk_') or not o.primitives or o.id == skip:
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

    def obstacle_boxes(self, obstacles, pad_m):
        """흩뿌린 공급의 주변 블록마다 블록 상자를 사방 pad_m 부풀려 만든다 (계획 요청에만 싣는다, E-53 ② B안).

        obstacles = PickPlace.obstacles (geometry_msgs/Pose 목록 — 집을 블록만 뺀 식별 블록의 중심 자세, 블록 자신의 축, base_link m).
        자세의 x·y·z 축이 블록의 길이·폭·두께라(slot_block_pose 와 같은 규약) 크기는 block_actual_m 그대로 쓴다.
        흩어진 블록은 카메라 오차만큼 어긋나 있어 장면에 넣지 않고 공중 이동 계획 때만 피한다(E-53 A안을 안 쓰는 이유).
        """
        size = [v + 2 * pad_m for v in self.cfg['block_actual_m']]
        out = []
        for k, pose in enumerate(obstacles):
            o = CollisionObject(id=f'pad_obstacle_{k}', operation=CollisionObject.ADD)
            o.header.frame_id = self.frame
            o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
            o.primitive_poses = [pose]
            out.append(o)
        return out

    def structure_box(self, box):
        """scan_structure_box 의 (중심 m, 크기 m) → 계획 요청에 실을 상자 하나 (부풀리지 않음 — 10/12 실기에서 이 크기로 검증)."""
        center, size = box
        o = CollisionObject(id='pad_scan_structure', operation=CollisionObject.ADD)
        o.header.frame_id = self.frame
        o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])]
        o.primitive_poses = [make_pose(center, (0.0, 0.0, 0.0, 1.0))]
        return o

    def plan(self, goal, keep_down, planner, plan_time=5.0, pad_m=0.0, skip_slot=None, scale=None, obstacles=None,
             structure=None, start_q=None, finger=None, held=None, skip_block=None):
        """지금 자세 -> 관절 목표 goal 을 OMPL 로 계획만 한다. 반환: (시간까지 매겨진 JointTrajectory, 결과 이름).

        속도·가속은 scale(없으면 robot.yaml speed_scale)을 MoveIt 에 넘겨 MoveIt 이 매긴다 (joint_limits.yaml x 비율).
        pad_m > 0 이면 놓인 블록과 공급 쪽 블록을 그만큼 부풀린 상자를 이 계획에만 더해 여유 있는 길을 찾게 한다.
        공급 쪽 블록 = obstacles 가 None 이면 공급 칸 가정 상자(skip_slot 칸 빼고), 목록이면 흩뿌림이라 그 블록들
        (빈 목록 = 주변 블록 없음 — 칸 가정 상자로 돌아가지 않는다, IRD 4.3).
        structure = 스캔 이동의 구조물 상자(motion_math.scan_structure_box) — 주면 pad_m 과 상관없이 이 계획에 더한다(W186).
        start_q · finger · held 를 주면 지금 자세 대신 그 출발 상태(관절 · 손가락 각 · 쥔 블록)로 계획한다 — 움직이기 전에
        '블록을 쥐고 옮기는 계획'을 미리 세워 볼 때 쓴다(장면은 안 바뀐다).
        skip_block = 여유 상자를 씌우지 않을 놓인 블록 이름(다시 집는 블록 자신 — 잡을 때 장면에서 지워진다).
        """
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = 'ompl'
        req.planner_id = planner
        req.num_planning_attempts = 1
        req.allowed_planning_time = plan_time
        # 자유 이동(공중 이동 · 자세 이동)은 robot.yaml motion.transit_speed_ratio 를 더 곱해 느리게 — 1번 관절이 크게 도는 이동 끝에서
        # 닿은 것 없이 충돌 판정 SAFE_STOP 이 두 번 났다(10/13 실기, 박진용: 이동 속도 줄이기). 집기 · 놓기 직선(plan_line)은 그대로
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = \
            (scale or self.cfg['speed_scale']) * self.cfg['motion'].get('transit_speed_ratio', 1.0)
        req.start_state = self._state(self.current() if start_q is None else start_q, finger, held)
        if start_q is not None:
            req.start_state.is_diff = False       # plan_line 과 같게: 미리 세워 볼 때는 지금 자세가 아닌 start_q 에서 출발
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v, tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0)
            for j, v in zip(JOINTS, goal)])]
        if keep_down:
            # True = 평소(TILT_TOL_DEG), 숫자 = 그 각까지(부르는 쪽이 정함 — 10/13 tool_tilt_max_deg 키는 지움)
            req.path_constraints = self.down_constraint() if keep_down is True else self.down_constraint(float(keep_down))
        extra = ()
        if pad_m > 0:
            supply = self.supply_boxes(pad_m, skip_slot) if obstacles is None else self.obstacle_boxes(obstacles, pad_m)
            extra = self.padded_blocks(pad_m, skip_block) + supply
        if structure is not None:
            extra = list(extra) + [self.structure_box(structure)]
        jt, why = self._send_plan(req, plan_time, extra)
        bad = (self.path_check(jt) or self.tcp_y_check(jt)) if jt is not None else None
        return (None, bad) if bad else (jt, why)

    def tcp_y_check(self, jt, samples=30):
        """계획한 자유 이동 경로에서 손끝(TCP) base y 가 tcp_y_max 를 넘으면 이유 글자, 아니면 None(제한 없으면 None).
        MoveIt 경로 제약(위치 상자)으로 걸었더니 쥔 블록 · 주변 블록까지 있을 때 OMPL 이 5 s 안에 길을 못 찾아(10/13 실기 — 제약 표본을
        IK 로 뽑느라) 계획 뒤 검사로 바꿈. 넘으면 이 길을 버린다 — 부르는 쪽이 다른 계획기 · 여유로 다시 찾는다."""
        ymax = self.tcp_y_max()
        if ymax is None:
            return None
        path = self.positions(jt)
        for q in path[::max(1, len(path) // samples)] + [path[-1]]:
            p = self.fk(q)
            if p is not None and p.position.y > ymax:
                return f'손끝 y {p.position.y * 1000:.0f} mm > 제한 {ymax * 1000:.0f} mm'
        return None

    @staticmethod
    def reversed_path(jt):
        """궤적 jt 를 끝 → 처음으로 되짚는 궤적(같은 시간표, 속도 부호만 반대). 내려간 직선을 그대로 거슬러 올라올 때 쓴다 —
        다시 계획하지 않으니 막힐 일이 없고, 지나간 자리라 충돌 검사도 다시 할 필요가 없다."""
        def sec(d):
            return d.sec + d.nanosec * 1e-9
        total = sec(jt.points[-1].time_from_start)
        out = JointTrajectory(joint_names=list(jt.joint_names))
        for p in reversed(jt.points):
            ns = max(0, round((total - sec(p.time_from_start)) * 1e9))
            out.points.append(JointTrajectoryPoint(positions=list(p.positions), velocities=[-v for v in p.velocities],
                                                   accelerations=list(p.accelerations),
                                                   time_from_start=Duration(sec=ns // 10 ** 9, nanosec=ns % 10 ** 9)))
        return out

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
        # 미리 계산한 궤적은 계획 때 출발 자세(앞 궤적의 끝)에서 시작한다. 실제 도착은 그와 ARRIVE_TOL_DEG 안에서 다를 수 있는데
        # MoveIt 은 출발 차이가 0.01 rad 를 넘으면 실행을 거절하므로 첫 점을 지금 관절값으로 맞춘다(그보다 멀면 엉뚱한 자세 — 안 움직임)
        now = self.current()
        if max(abs(math.degrees(a - b)) for a, b in zip(now, path[0])) > ARRIVE_TOL_DEG:
            self.node.get_logger().error('궤적 출발 자세와 지금 자세가 다르다 — 움직이지 않는다')
            return False, 'ERROR'
        jt.points[0].positions = [now[JOINTS.index(n)] if n in JOINTS else v for n, v in zip(jt.joint_names, jt.points[0].positions)]
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

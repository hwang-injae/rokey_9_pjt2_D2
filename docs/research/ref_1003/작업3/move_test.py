#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""작업 3 (V-07, V-08): MoveIt2로 이름 붙은 관절 목표까지 계획하고, 확인 뒤에 실행한다.

먼저 할 것: source <내 워크스페이스>/install/setup.bash, MoveIt2 브링업이 떠 있어야 한다.
실행 위치: 브링업을 띄운 PC의 새 터미널. 빌드는 필요 없다 (python3로 바로 실행).

예)
  python3 move_test.py --list                # 목표 이름과 관절값 보기
  python3 move_test.py C --plan-only         # 계획만 (V-08). 계획 시간을 출력
  python3 move_test.py J10                   # 계획 -> RViz에서 경로 확인 -> y 입력 시 실행 -> 오차 출력 (V-07)
  python3 move_test.py B --planner lin       # 직선 (Pilz LIN)
  python3 move_test.py C --scale 0.3         # 속도·가속도 비율 0.3 (0.1에서 같은 목표가 문제없이 끝난 뒤에만)

목표 관절값은 플랜지(link_6) 기준이다. RG2 그리퍼는 MoveIt2 모델에 없으므로
scene_setup.py로 책상과 '그리퍼 대신 원기둥'을 먼저 넣는다.
브링업의 name 인자를 바꿨다면 --ns 에 같은 값을 준다 (기본값은 빈 값).
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, DisplayTrajectory, MoveItErrorCodes)
from moveit_msgs.srv import GetPositionFK

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# 관절 1~6 (도). 플랜지 위치는 base_link 기준 (m). 공구가 아래를 보는 자세.
GOALS = {
    'H':   [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
    'J10': [10.0, 0.0, 90.0, 0.0, 90.0, 0.0],
    'A':   [-30.0, 21.0, 76.0, 0.0, 83.0, 0.0],
    'B':   [-25.0, 35.0, 55.0, 0.0, 90.0, 0.0],
    'C':   [30.0, 21.0, 76.0, 0.0, 83.0, 0.0],
}
GOAL_NOTE = {
    'H':   '홈. 플랜지 약 (0.37, 0.01, 0.42)',
    'J10': 'H에서 관절 1만 +10도',
    'A':   '플랜지 약 (0.45, -0.25, 0.35)',
    'B':   'A에서 x 방향으로 약 100 mm (직선 시험)',
    'C':   '플랜지 약 (0.44, 0.26, 0.35)',
}
PLANNERS = {  # 이름: (pipeline_id, planner_id). planner_id가 비면 그룹 기본값 (이 설정은 RRTConnect)
    'ompl': ('ompl', ''),
    'lin': ('pilz_industrial_motion_planner', 'LIN'),
    'ptp': ('pilz_industrial_motion_planner', 'PTP'),
}
ERROR_NAMES = {getattr(MoveItErrorCodes, n): n for n in dir(MoveItErrorCodes)
               if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)}
MAX_SCALE = 0.5


def err_name(code):
    return ERROR_NAMES.get(code, str(code))


class MoveItHelper:
    """move_group에 계획·실행을 요청하고, 도착 오차를 계산한다. 노드를 받아서 쓴다."""

    def __init__(self, node, group='manipulator'):
        self.node = node
        self.group = group
        self.last_js = None
        node.create_subscription(JointState, 'joint_states', self._on_js, 10)
        self.move_ac = ActionClient(node, MoveGroup, 'move_action')
        self.exec_ac = ActionClient(node, ExecuteTrajectory, 'execute_trajectory')
        self.fk_cli = node.create_client(GetPositionFK, 'compute_fk')
        self.display_pub = node.create_publisher(DisplayTrajectory, 'display_planned_path', 10)

    def _on_js(self, msg):
        self.last_js = msg

    def wait_ready(self, timeout=10.0):
        ok = self.move_ac.wait_for_server(timeout_sec=timeout)
        ok = ok and self.exec_ac.wait_for_server(timeout_sec=timeout)
        ok = ok and self.fk_cli.wait_for_service(timeout_sec=timeout)
        if not ok:
            print('[오류] move_group을 찾지 못했다. 브링업이 다 떴는지, --ns가 브링업 name과 같은지 본다.')
            print('       확인: ros2 action list | grep move_action')
            return False
        end = time.time() + timeout
        while self.last_js is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.last_js is None:
            print('[오류] joint_states를 받지 못했다. 확인: ros2 topic list | grep joint_states')
            return False
        return True

    def _spin_future(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def current_deg(self):
        """가장 최근 joint_states를 관절 1~6 순서의 도 값으로 돌려준다."""
        end = time.time() + 0.5
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        js = self.last_js
        pos = dict(zip(js.name, js.position))
        return [math.degrees(pos[j]) for j in JOINTS]

    def plan(self, goal_deg, planner='ompl', scale=0.1, attempts=1, plan_time=5.0):
        pipeline, planner_id = PLANNERS[planner]
        req = MotionPlanRequest()
        req.group_name = self.group
        req.pipeline_id = pipeline
        req.planner_id = planner_id
        req.num_planning_attempts = attempts
        req.allowed_planning_time = plan_time
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale
        req.start_state.is_diff = True  # 현재 상태(붙인 물체 포함)에서 시작
        c = Constraints()
        for name, deg in zip(JOINTS, goal_deg):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = math.radians(deg)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        gh = self._spin_future(self.move_ac.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            return None, '목표가 거절됨', 0.0
        res = self._spin_future(gh.get_result_async(), plan_time + 20.0)
        if res is None:
            return None, '응답 없음 (시간 초과)', 0.0
        r = res.result
        if r.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, err_name(r.error_code.val), r.planning_time
        disp = DisplayTrajectory()
        disp.trajectory_start = r.trajectory_start
        disp.trajectory = [r.planned_trajectory]
        self.display_pub.publish(disp)  # RViz MotionPlanning 표시에 경로가 나온다
        return r, 'SUCCESS', r.planning_time

    def execute(self, plan_result, timeout=120.0):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = plan_result.planned_trajectory
        gh = self._spin_future(self.exec_ac.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            return '실행 목표가 거절됨'
        res = self._spin_future(gh.get_result_async(), timeout)
        if res is None:
            return '응답 없음 (시간 초과)'
        return err_name(res.result.error_code.val)

    def flange_xyz(self, deg):
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['link_6']
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [math.radians(d) for d in deg]
        res = self._spin_future(self.fk_cli.call_async(req), 5.0)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)

    def arrival_error(self, goal_deg, settle=1.0):
        """도착 뒤 관절 차이(도)와 플랜지 위치 차이(mm)."""
        time.sleep(settle)
        actual = self.current_deg()
        dj = [a - g for a, g in zip(actual, goal_deg)]
        pg, pa = self.flange_xyz(goal_deg), self.flange_xyz(actual)
        dmm = None if (pg is None or pa is None) else 1000.0 * math.dist(pg, pa)
        return actual, dj, dmm


def traj_seconds(plan_result):
    pts = plan_result.planned_trajectory.joint_trajectory.points
    if not pts:
        return 0.0
    t = pts[-1].time_from_start
    return t.sec + t.nanosec * 1e-9


def ask_yes(prompt):
    try:
        return input(prompt).strip().lower() == 'y'
    except EOFError:
        return False


def fmt(vals, nd=2):
    return ', '.join(f'{v:.{nd}f}' for v in vals)


def main():
    ap = argparse.ArgumentParser(description='작업 3: MoveIt2 관절 목표 계획·실행')
    ap.add_argument('goal', nargs='?', help='목표 이름: ' + ', '.join(GOALS))
    ap.add_argument('--list', action='store_true', help='목표 이름과 관절값 보기')
    ap.add_argument('--plan-only', action='store_true', help='계획만 하고 실행하지 않는다 (V-08)')
    ap.add_argument('--planner', default='ompl', choices=list(PLANNERS), help='ompl(기본) / lin / ptp')
    ap.add_argument('--scale', type=float, default=0.1, help='속도·가속도 비율 (기본 0.1, 최대 0.5)')
    ap.add_argument('--attempts', type=int, default=1, help='계획 시도 수 (기본 1)')
    ap.add_argument('--ns', default='', help='브링업 name 인자와 같은 값 (기본: 빈 값)')
    args = ap.parse_args()

    if args.list or not args.goal:
        for k, v in GOALS.items():
            print(f'{k:4s} [{fmt(v, 1)}]  {GOAL_NOTE[k]}')
        return 0
    if args.goal not in GOALS:
        print(f'[오류] 목표 이름은 {", ".join(GOALS)} 중 하나다.')
        return 1
    if not (0.0 < args.scale <= MAX_SCALE):
        print(f'[오류] --scale은 0보다 크고 {MAX_SCALE} 이하로 준다.')
        return 1

    goal_deg = GOALS[args.goal]
    rclpy.init()
    node = rclpy.create_node('task3_move_test', namespace=args.ns)
    mv = MoveItHelper(node)
    try:
        if not mv.wait_ready():
            return 1
        print(f'목표 {args.goal} [{fmt(goal_deg, 1)}] / 계획기 {args.planner} / 속도 비율 {args.scale}')
        print(f'현재 관절 [{fmt(mv.current_deg(), 1)}]')
        res, status, ptime = mv.plan(goal_deg, args.planner, args.scale, args.attempts)
        if res is None:
            print(f'[계획 실패] {status} (계획 시간 {ptime:.3f} s)')
            return 2
        print(f'[계획 성공] 계획 시간 {ptime:.3f} s, 경로 점 {len(res.planned_trajectory.joint_trajectory.points)}개, '
              f'움직이는 시간 약 {traj_seconds(res):.1f} s')
        if args.plan_only:
            return 0
        print('RViz에서 경로 애니메이션을 끝까지 본다. 그리퍼 대신 원기둥이 책상·장애물에 가까이 가면 n.')
        if args.scale > 0.2:
            print(f'속도 비율 {args.scale}: 같은 목표를 0.1에서 문제없이 끝낸 뒤에만 실행한다.')
        if not ask_yes('실행할까요? 펜던트를 손에 들고(비상정지 가능), 작업 영역에 손이 없으면 y: '):
            print('실행하지 않았다.')
            return 0
        status = mv.execute(res)
        print(f'[실행 결과] {status}')
        actual, dj, dmm = mv.arrival_error(goal_deg)
        print(f'도착 관절 [{fmt(actual, 3)}]')
        print(f'관절 차이 (도) [{fmt(dj, 3)}]  최대 {max(abs(d) for d in dj):.3f}')
        if dmm is None:
            print('플랜지 위치 차이: 계산 못 함 (compute_fk 실패)')
        else:
            print(f'플랜지(link_6) 위치 차이 {dmm:.2f} mm')
        return 0 if status == 'SUCCESS' else 3
    except KeyboardInterrupt:
        print('\n중단했다. 로봇이 움직이고 있으면 비상정지.')
        return 4
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

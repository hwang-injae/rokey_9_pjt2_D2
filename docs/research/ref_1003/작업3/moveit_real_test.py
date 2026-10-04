#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MoveIt2 실기 시험: 두 자세(A, C) 사이에 장애물 상자를 두고, 피해 가는 경로가 실기에서 부드럽게 실행되는지 본다.

한 번 실행하면 이 순서로 한다. 실행 전마다 y 를 물어본다.
  1. 장면 넣기: scene_setup.py --tcp <TCP> --box
     (책상 + 그리퍼 대신 원기둥 + 장애물 상자 0.10 x 0.10 x 0.20 m, 중심 (0.55, 0) m — 실물은 두지 않는다)
  2. 시작 자세 A 로 이동
  3. A -> C: 상자를 피해 가는 경로를 여러 번 계획해 가장 짧은 것을 고르고, 실행하면서 관절 값을 기록
  4. (--back) C -> A 도 같은 방법으로
  5. 결과: 계획 시간, 예상·실제 시간, 관절 추종 오차, 떨림(속도 방향이 계획보다 더 자주 바뀜),
     joint_states 끊김, 도착 오차 -> 화면 + CSV + PNG(그래프)

먼저 할 것 (터미널마다 같은 ROS_DOMAIN_ID):
  터미널 1: source <워크스페이스>/install/setup.bash
            ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m0609 host:=192.168.1.100
  터미널 2: source <워크스페이스>/install/setup.bash
            cd <이 파일이 있는 폴더>   (scene_setup.py 가 같은 폴더에 있어야 한다)
            python3 moveit_real_test.py --tcp 228            # TCP 는 한석형이 잰 값(mm)으로
  가상으로 먼저 보려면 터미널 1 을 mode:=virtual host:=127.0.0.1 로 띄운다.

예)
  python3 moveit_real_test.py --tcp 228 --table-z -0.018    # 속도 비율 0.1, BiTRRT, 5번 계획 중 가장 짧은 것
  python3 moveit_real_test.py --tcp 228 --back              # C -> A 도
  python3 moveit_real_test.py --tcp 228 --scale 0.2 --back  # 0.1 에서 문제없이 끝난 뒤에만
  python3 moveit_real_test.py --tcp 228 --planner rrtconnect
안전:
  - 실행할 때마다 조작자가 티치 펜던트를 손에 든다. 로봇이 움직이는 동안 작업 영역에 손을 넣지 않는다.
  - 실행 중 Ctrl+C: 이 프로그램이 MoveIt2 실행을 취소하고 두산 move_stop(빠른 정지)까지 부른 뒤 끝난다.
    그래도 움직이면 펜던트 비상정지.
  - 장애물 상자는 MoveIt2 장면에만 있다(가상). 실물을 두려면 이 시험이 끝난 뒤 가벼운 상자로 같은 자리에.
"""
import argparse
import csv
import datetime
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions, DisplayTrajectory, MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

HERE = os.path.dirname(os.path.abspath(__file__))
JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
GOALS = {  # 관절 1~6 (도). 플랜지 위치는 base_link 기준
    'A': [-30.0, 21.0, 76.0, 0.0, 83.0, 0.0],   # 플랜지 약 (0.45, -0.25, 0.35) m
    'C': [30.0, 21.0, 76.0, 0.0, 83.0, 0.0],    # 플랜지 약 (0.44, 0.26, 0.35) m
}
BOX_XY = (0.55, 0.0)            # scene_setup.py --box 와 같은 자리 (m)
PLANNERS = {'bitrrt': 'BiTRRT', 'rrtconnect': ''}   # '' = 그룹 기본값(RRTConnect)
MAX_SCALE = 0.3
MOVE_DEG = 0.05                 # 이보다 크게 바뀌면 '움직이기 시작'으로 본다 (도)
GAP_S = 0.05                    # joint_states 간격이 이보다 길면 '끊김'으로 센다 (초)
REF = {'track_deg': 1.0, 'extra_rev': 0, 'gaps': 0, 'arrive_deg': 0.1}   # 참고 기준
ERR = {getattr(MoveItErrorCodes, n): n for n in dir(MoveItErrorCodes)
       if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)}


def tsec(d):
    return d.sec + d.nanosec * 1e-9


def ask_yes(msg):
    try:
        return input(msg).strip().lower() == 'y'
    except EOFError:
        return False


class Tester:
    def __init__(self, node):
        self.node = node
        self.js = None
        self.rec = None                       # 기록 중이면 [(t, [deg x6]), ...]
        node.create_subscription(JointState, 'joint_states', self._on_js, 50)
        self.move_ac = ActionClient(node, MoveGroup, 'move_action')
        self.exec_ac = ActionClient(node, ExecuteTrajectory, 'execute_trajectory')
        self.fk = node.create_client(GetPositionFK, 'compute_fk')
        self.disp = node.create_publisher(DisplayTrajectory, 'display_planned_path', 10)
        self.stop_cli = None
        self.hold_ac = ActionClient(node, FollowJointTrajectory, 'dsr_moveit_controller/follow_joint_trajectory')

    # ---------- 기본 ----------
    def _on_js(self, m):
        pos = dict(zip(m.name, m.position))
        if not all(j in pos for j in JOINTS):
            return
        deg = [math.degrees(pos[j]) for j in JOINTS]
        self.js = deg
        if self.rec is not None:
            self.rec.append((time.monotonic(), deg))

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_future(self, fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        return fut.result() if fut.done() else None

    def ready(self):
        ok = self.move_ac.wait_for_server(timeout_sec=10.0) and self.exec_ac.wait_for_server(timeout_sec=10.0)
        ok = ok and self.fk.wait_for_service(timeout_sec=10.0)
        self.spin_for(1.0)
        if not ok or self.js is None:
            print('[오류] move_group 이나 joint_states 를 못 찾았다. 브링업이 다 떴는지, ROS_DOMAIN_ID 가 같은지 본다.')
            print('       확인: ros2 action list | grep move_action / ros2 topic list | grep joint_states')
            return False
        self._find_stop()
        return True

    def _find_stop(self):
        try:
            from dsr_msgs2.srv import MoveStop
        except ImportError:
            print('[알림] dsr_msgs2 를 못 불러 두산 move_stop 은 쓰지 않는다 (MoveIt2 취소만). 워크스페이스를 source 했는지 본다.')
            return
        names = [n for n, t in self.node.get_service_names_and_types() if n.endswith('/motion/move_stop')]
        if names:
            self.stop_cli = (self.node.create_client(MoveStop, sorted(names, key=len)[0]), MoveStop)
            print(f'Ctrl+C 때 부를 두산 정지 서비스: {sorted(names, key=len)[0]}')
        else:
            print('[알림] motion/move_stop 서비스가 안 보인다. Ctrl+C 때 MoveIt2 취소만 한다. 펜던트를 꼭 손에 든다.')

    # ---------- 계획 ----------
    def plan(self, goal_deg, planner, scale, plan_time=5.0):
        req = MotionPlanRequest()
        req.group_name = 'manipulator'
        req.pipeline_id = 'ompl'
        req.planner_id = PLANNERS[planner]
        req.num_planning_attempts = 1
        req.allowed_planning_time = plan_time
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale
        req.start_state.is_diff = True
        c = Constraints()
        for name, d in zip(JOINTS, goal_deg):
            c.joint_constraints.append(JointConstraint(joint_name=name, position=math.radians(d),
                                                       tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        req.goal_constraints = [c]
        goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=True))
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        gh = self.wait_future(self.move_ac.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            return None, '목표 거절'
        res = self.wait_future(gh.get_result_async(), plan_time + 20.0)
        if res is None:
            return None, '응답 없음'
        r = res.result
        if r.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, ERR.get(r.error_code.val, str(r.error_code.val))
        return r, 'SUCCESS'

    def best_plan(self, goal_deg, planner, scale, tries):
        best, times = None, []
        for k in range(tries):
            r, st = self.plan(goal_deg, planner, scale)
            if r is None:
                print(f'  계획 {k + 1}/{tries}: 실패 ({st})')
                continue
            dur = tsec(r.planned_trajectory.joint_trajectory.points[-1].time_from_start)
            times.append(r.planning_time)
            print(f'  계획 {k + 1}/{tries}: 성공, 계획 {r.planning_time:.3f} s, 움직이는 시간 {dur:.1f} s')
            if best is None or dur < tsec(best.planned_trajectory.joint_trajectory.points[-1].time_from_start):
                best = r
        if best is not None:
            d = DisplayTrajectory(trajectory_start=best.trajectory_start, trajectory=[best.planned_trajectory])
            self.disp.publish(d)
        return best, times

    def flange_path(self, r, n=25):
        """경로를 n 점 골라 플랜지(link_6) 위치를 FK 로 구한다 -> [(x, y, z), ...] (m)"""
        pts = r.planned_trajectory.joint_trajectory.points
        names = list(r.planned_trajectory.joint_trajectory.joint_names)
        out = []
        for i in range(n):
            p = pts[round(i * (len(pts) - 1) / (n - 1))]
            req = GetPositionFK.Request()
            req.header.frame_id = 'base_link'
            req.fk_link_names = ['link_6']
            req.robot_state.joint_state.name = names
            req.robot_state.joint_state.position = list(p.positions)
            res = self.wait_future(self.fk.call_async(req), 5.0)
            if res is not None and res.pose_stamped:
                q = res.pose_stamped[0].pose.position
                out.append((q.x, q.y, q.z))
        return out

    # ---------- 실행 ----------
    def hold_here(self):
        """제어기에 '지금 자리에 0.3 초 안에 서라'는 궤적을 바로 보내 실행 중인 궤적을 덮어쓴다."""
        if self.js is None or not self.hold_ac.wait_for_server(timeout_sec=1.0):
            print('[정지] 궤적 제어기(dsr_moveit_controller)를 못 찾았다.')
            return
        jt = JointTrajectory(joint_names=list(JOINTS))
        jt.points = [JointTrajectoryPoint(positions=[math.radians(d) for d in self.js], velocities=[0.0] * 6,
                                          time_from_start=Duration(sec=0, nanosec=300_000_000))]
        gh = self.wait_future(self.hold_ac.send_goal_async(FollowJointTrajectory.Goal(trajectory=jt)), 2.0)
        print('[정지] 제어기에 "지금 자리에 서기" 궤적을 보냈다' + ('' if gh is not None and gh.accepted else ' (거절됨)'))

    def stop_robot(self, gh):
        print('\n[정지] 지금 자리에 서게 하고, MoveIt2 실행을 취소한다 ...')
        self.hold_here()
        if gh is not None:
            self.wait_future(gh.cancel_goal_async(), 2.0)
        if self.stop_cli is not None:
            cli, srv = self.stop_cli
            if cli.wait_for_service(timeout_sec=1.0):
                self.wait_future(cli.call_async(srv.Request(stop_mode=1)), 2.0)   # 1 = DR_QSTOP (빠른 정지)
                print('[정지] 두산 move_stop(빠른 정지)도 불렀다.')
        a = self.js
        self.spin_for(1.0)
        b = self.js
        moving = a is not None and b is not None and max(abs(x - y) for x, y in zip(a, b)) > MOVE_DEG
        print('[정지] 1초 동안 ' + ('아직 움직인다 -> 펜던트 비상정지!' if moving else '움직이지 않음'))

    def execute(self, r, timeout=180.0):
        """실행하면서 joint_states 를 기록한다. Ctrl+C 면 멈추고 KeyboardInterrupt 를 다시 올린다."""
        gh = None
        self.rec = []
        try:
            gh = self.wait_future(self.exec_ac.send_goal_async(ExecuteTrajectory.Goal(trajectory=r.planned_trajectory)), 10.0)
            if gh is None or not gh.accepted:
                return '실행 목표 거절', []
            fut = gh.get_result_async()
            end = time.monotonic() + timeout
            while not fut.done():
                rclpy.spin_once(self.node, timeout_sec=0.005)
                if time.monotonic() > end:
                    self.stop_robot(gh)
                    return '시간 초과 (멈춤)', self.rec
            last, still, end2 = self.js, 0.0, time.monotonic() + 8.0   # 완전히 멈출 때까지(0.5 s 그대로) 더 기록, 최대 8 s
            while still < 0.5 and time.monotonic() < end2:
                self.spin_for(0.1)
                moved = max(abs(x - y) for x, y in zip(self.js, last)) > 0.01
                still = 0.0 if moved else still + 0.1
                last = self.js
            code = fut.result().result.error_code.val
            return ERR.get(code, str(code)), self.rec
        except KeyboardInterrupt:
            self.stop_robot(gh)
            raise
        finally:
            rec, self.rec = self.rec, None
            self._last_rec = rec


# ---------- 분석 ----------
def interp(points, t):
    """계획 궤적의 시각 t(초) 관절값(도)"""
    if t <= tsec(points[0].time_from_start):
        return [math.degrees(v) for v in points[0].positions]
    for a, b in zip(points, points[1:]):
        ta, tb = tsec(a.time_from_start), tsec(b.time_from_start)
        if t <= tb:
            w = 0.0 if tb == ta else (t - ta) / (tb - ta)
            return [math.degrees(x + w * (y - x)) for x, y in zip(a.positions, b.positions)]
    return [math.degrees(v) for v in points[-1].positions]


def reversals(times, series, vmin=0.5):
    """속도(도/초)의 방향이 바뀐 횟수. 5점 평균으로 고른 뒤 계산, |v| < vmin 은 무시"""
    n = len(series)
    if n < 7:
        return 0
    sm = [sum(series[max(0, i - 2):i + 3]) / len(series[max(0, i - 2):i + 3]) for i in range(n)]
    sign, cnt = 0, 0
    for i in range(1, n):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            continue
        v = (sm[i] - sm[i - 1]) / dt
        if abs(v) < vmin:
            continue
        s = 1 if v > 0 else -1
        if sign and s != sign:
            cnt += 1
        sign = s
    return cnt


def analyze(r, rec, goal_deg, label, outdir):
    jt = r.planned_trajectory.joint_trajectory
    order = [jt.joint_names.index(j) for j in JOINTS]
    pts = jt.points
    for p in pts:                                     # JOINTS 순서로 맞춘다
        p.positions = [p.positions[i] for i in order]
    plan_dur = tsec(pts[-1].time_from_start)
    if len(rec) < 10:
        print('[분석] 기록이 너무 적다 (joint_states 가 안 왔다). 분석을 건너뛴다.')
        return None
    q0 = rec[0][1]
    ta0 = next((t for t, q in rec if max(abs(a - b) for a, b in zip(q, q0)) > MOVE_DEG), None)
    if ta0 is None:
        print('[분석] 로봇이 움직이지 않았다.')
        return None
    p0 = [math.degrees(v) for v in pts[0].positions]
    tp0 = 0.0
    for k in range(1, 2001):                          # 계획에서도 같은 기준으로 '움직이기 시작' 시각을 찾는다
        t = plan_dur * k / 2000
        if max(abs(a - b) for a, b in zip(interp(pts, t), p0)) > MOVE_DEG:
            tp0 = t
            break
    shift = ta0 - tp0                                 # 실제 시각 - shift = 계획 시각
    tend = next((t for t, q in rec if t > ta0 and max(abs(a - b) for a, b in zip(q, goal_deg)) < MOVE_DEG), rec[-1][0])
    act_dur = tend - shift
    rows, errs = [], []
    for t, q in rec:
        tp = t - shift
        if tp < 0 or t > tend + 0.5:
            continue
        qp = interp(pts, tp)
        e = [abs(a - b) for a, b in zip(q, qp)]
        errs.append((max(e), e.index(max(e)) + 1))
        rows.append([round(tp, 4)] + [round(v, 4) for v in qp] + [round(v, 4) for v in q])
    gaps = [b[0] - a[0] for a, b in zip(rec, rec[1:])]
    tt = [t for t, _ in rec]
    act_rev = sum(reversals(tt, [q[j] for _, q in rec]) for j in range(6))
    pt = [tsec(p.time_from_start) for p in pts]
    dense_t = [plan_dur * k / 400 for k in range(401)]
    plan_rev = sum(reversals(dense_t, [interp(pts, t)[j] for t in dense_t]) for j in range(6))
    final = rec[-1][1]
    arrive = max(abs(a - b) for a, b in zip(final, goal_deg))
    emax, ej = max(errs) if errs else (0.0, 0)
    res = {
        'plan_dur': plan_dur, 'act_dur': act_dur, 'track_max': emax, 'track_joint': ej,
        'track_mean': sum(e for e, _ in errs) / max(1, len(errs)),
        'hz': (len(rec) - 1) / max(1e-6, rec[-1][0] - rec[0][0]), 'gap_max': max(gaps) if gaps else 0.0,
        'gaps': sum(1 for g in gaps if g > GAP_S), 'plan_rev': plan_rev, 'act_rev': act_rev,
        'extra_rev': max(0, act_rev - plan_rev), 'arrive': arrive,
    }
    stamp = datetime.datetime.now().strftime('%H%M%S')
    base = os.path.join(outdir, f'moveit_real_{label}_{stamp}')
    with open(base + '.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_s'] + [f'계획_j{i}' for i in range(1, 7)] + [f'실제_j{i}' for i in range(1, 7)])
        w.writerows(rows)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        ts = [r_[0] for r_ in rows]
        for j in range(6):
            line, = ax[0].plot(ts, [r_[1 + j] for r_ in rows], '--', lw=1)
            ax[0].plot(ts, [r_[7 + j] for r_ in rows], '-', lw=1.6, color=line.get_color(), label=f'j{j + 1}')
        ax[0].set_ylabel('joint (deg)  dashed=plan, solid=actual')
        ax[0].legend(ncol=6, fontsize=8)
        ax[1].plot(ts, [max(abs(r_[1 + j] - r_[7 + j]) for j in range(6)) for r_ in rows], color='crimson')
        ax[1].axhline(REF['track_deg'], color='gray', ls=':')
        ax[1].set_ylabel('max |plan - actual| (deg)')
        ax[1].set_xlabel('time (s)')
        fig.suptitle(f'{label}: plan {plan_dur:.1f} s / actual {act_dur:.1f} s')
        fig.savefig(base + '.png', dpi=100)
        plt.close(fig)
        res['png'] = base + '.png'
    except Exception as e:                            # 그래프는 없어도 된다
        print(f'[알림] 그래프를 못 그렸다 ({e}). CSV 만 남긴다.')
    res['csv'] = base + '.csv'
    return res


def report(label, res):
    ok = (res['track_max'] <= REF['track_deg'] and res['extra_rev'] <= REF['extra_rev']
          and res['gaps'] <= REF['gaps'] and res['arrive'] <= REF['arrive_deg'])
    print(f'\n===== {label} 결과 =====')
    print(f'움직이는 시간: 계획 {res["plan_dur"]:.1f} s / 실제 {res["act_dur"]:.1f} s')
    print(f'관절 추종 오차: 최대 {res["track_max"]:.2f}° (관절 {res["track_joint"]}), 평균 {res["track_mean"]:.2f}°   (참고 기준 {REF["track_deg"]}° 이하)')
    print(f'떨림: 속도 방향 바뀜 계획 {res["plan_rev"]}회 / 실제 {res["act_rev"]}회 -> 더 생긴 것 {res["extra_rev"]}회   (참고 기준 0)')
    print(f'joint_states: {res["hz"]:.0f} Hz, 가장 긴 간격 {1000 * res["gap_max"]:.0f} ms, {int(1000 * GAP_S)} ms 넘는 끊김 {res["gaps"]}회   (참고 기준 0)')
    print(f'도착 오차: 최대 {res["arrive"]:.3f}°   (참고 기준 {REF["arrive_deg"]}° 이하)')
    print(f'숫자 참고 판정: {"부드러움" if ok else "확인 필요"}  (눈으로 본 끊김·떨림·소리도 같이 적는다)')
    print(f'기록: {res["csv"]}' + (f', {res["png"]}' if 'png' in res else ''))


def do_move(t, name, planner, scale, tries, outdir, check_box):
    goal = GOALS[name]
    print(f'\n--- {name} 로 이동: 관절 [{", ".join(f"{v:.1f}" for v in goal)}] / {planner} / 속도 비율 {scale} ---')
    r, ptimes = t.best_plan(goal, planner, scale, tries)
    if r is None:
        print('[계획 실패] 모두 실패했다. 장면(상자 위치)과 시작 자세를 본다.')
        return False
    if check_box:
        path = t.flange_path(r)
        if path:
            dmin = min(math.hypot(x - BOX_XY[0], y - BOX_XY[1]) for x, y, _ in path)
            zmax = max(z for _, _, z in path)
            print(f'  고른 경로: 플랜지가 상자 중심에서 수평 최소 {dmin:.2f} m, 가장 높은 곳 {zmax:.2f} m '
                  f'(상자는 높이 0.20 m, 반폭 0.05 m. 그리퍼 대신 원기둥이 플랜지 아래로 TCP 만큼 내려온다)')
    print('RViz 에서 경로 애니메이션을 끝까지 본다. 그리퍼 대신 원기둥이 상자·책상에 바짝 붙으면 n.')
    if not ask_yes('실행할까요? 펜던트를 손에 들고, 작업 영역에 손이 없으면 y: '):
        print('실행하지 않았다.')
        return False
    st, rec = t.execute(r)
    print(f'[실행 결과] {st}')
    res = analyze(r, rec, goal, f'to{name}', outdir)
    if res:
        report(f'{name} 로 이동', res)
    return st == 'SUCCESS'


def main():
    ap = argparse.ArgumentParser(description='MoveIt2 실기 시험: 장애물을 피해 가는 경로가 부드러운가')
    ap.add_argument('--tcp', type=float, help='TCP 길이 mm (플랜지 -> 손가락 끝). 한석형이 잰 값')
    ap.add_argument('--scale', type=float, default=0.1, help=f'속도·가속도 비율 (기본 0.1, 최대 {MAX_SCALE})')
    ap.add_argument('--planner', default='bitrrt', choices=list(PLANNERS), help='bitrrt(기본, 박진용 2차 채택) / rrtconnect')
    ap.add_argument('--tries', type=int, default=5, help='A->C 계획 횟수. 가장 짧은 경로를 고른다 (기본 5. BiTRRT는 가끔 실패한다)')
    ap.add_argument('--back', action='store_true', help='C -> A 도 한다')
    ap.add_argument('--table-z', type=float, default=0.0, help='책상 윗면 높이 (m, 로봇 바닥면 기준). scene_setup.py 로 넘긴다')
    ap.add_argument('--no-scene', action='store_true', help='장면을 이미 넣었으면 건너뛴다')
    ap.add_argument('--ns', default='', help='브링업 name 인자와 같은 값 (기본: 빈 값)')
    ap.add_argument('--out', default=os.getcwd(), help='CSV·PNG 저장 폴더 (기본: 지금 폴더)')
    a = ap.parse_args()
    if not (0.0 < a.scale <= MAX_SCALE):
        print(f'[오류] --scale 은 0 보다 크고 {MAX_SCALE} 이하.')
        return 1
    if not a.no_scene:
        if a.tcp is None:
            print('[오류] --tcp (mm) 를 준다. 그리퍼 대신 원기둥 길이에 쓴다. 모르면 228 로 하고 기록에 적는다.')
            return 1
        cmd = [sys.executable, os.path.join(HERE, 'scene_setup.py'), '--tcp', str(a.tcp), '--box', '--table-z', str(a.table_z)]
        if a.ns:
            cmd += ['--ns', a.ns]
        print('장면 넣기: ' + ' '.join(cmd[1:]))
        if subprocess.run(cmd).returncode != 0:
            print('[오류] scene_setup.py 가 실패했다. 같은 폴더에 있는지, 브링업이 떠 있는지 본다.')
            return 1

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C 뒤에도 ROS 가 살아 있어야 정지 명령을 보낼 수 있다
    node = rclpy.create_node('moveit_real_test', namespace=a.ns)
    t = Tester(node)
    try:
        if not t.ready():
            return 1
        print(f'현재 관절 [{", ".join(f"{v:.1f}" for v in t.js)}]')
        if a.scale > 0.1:
            print(f'속도 비율 {a.scale}: 0.1 에서 같은 시험이 문제없이 끝난 뒤에만 한다.')
        if max(abs(x - y) for x, y in zip(t.js, GOALS['A'])) > 0.5:
            if not do_move(t, 'A', a.planner, a.scale, a.tries, a.out, False):
                return 2
        ok = do_move(t, 'C', a.planner, a.scale, a.tries, a.out, True)
        if ok and a.back:
            ok = do_move(t, 'A', a.planner, a.scale, a.tries, a.out, True)
        return 0 if ok else 3
    except KeyboardInterrupt:
        print('\n중단했다.')
        return 4
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

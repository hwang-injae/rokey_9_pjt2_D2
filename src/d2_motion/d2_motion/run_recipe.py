# -*- coding: utf-8 -*-
"""시험 도구 run_recipe — 레시피를 읽어 블록마다 /d2/motion/pick_place 목표를 차례로 보낸다 (작업 관리자 대신, 로봇 파트 시험용).

원본: 박진용 recipe_demo/make_targets.py + run_targets.py 의 실행 순서 (10/6 LV1 11개 실기 성공).

  ros2 run d2_motion run_recipe <LV1.recipe.json> --slots 1,3 --check   # 목표만 계산해 보여 줌 (ROS·로봇 없음)
  ros2 run d2_motion run_recipe <LV1.recipe.json> --slots 1,3           # 블록마다 y 확인
  ros2 run d2_motion run_recipe <LV1.recipe.json> --slots 1,3 --auto    # 확인 없이 (가상 시험)
  ros2 run d2_motion run_recipe <lv2.json> <lv4.json> --auto             # 두 설계를 세트로 (의자 앞에 책상)
--slots 없음 = 블록 잡기에 정해진 칸(p1~p6)으로, --slots 1,3 = 공급 칸 1, 3, 1, 3 ... 번갈아 (사람이 가져간 칸을 다시 채운다). Ctrl+C = 지금 목표 취소 -> pick_place 가 세운다.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from d2_interfaces.action import PickPlace
from d2_interfaces.srv import GripperCommand, MoveTo
from rclpy.action import ActionClient
from std_msgs.msg import String

from d2_motion.executor import make_pose
from d2_motion.motion_math import (close_angle_deg, column, layout_designs, pick_place_tcp, quat_from_axes,
                                   slot_block_pose, up_axis)
from d2_safety.safe_stop import init_ros

HOME_TIMEOUT_S = 120.0
START_OPEN_M = 0.100       # 시작할 때 그리퍼를 여는 폭 (가장 큰 집는 폭 FLAT_LONG·EDGE_LONG 과 같음)


def parse_list(spec):
    """'1-3,5' -> [1, 2, 3, 5]."""
    out = []
    for tok in spec.split(','):
        a, _, b = tok.partition('-')
        out += list(range(int(a), int(b or a) + 1))
    return out


def plan_jobs(cfg, recipes, slots=None, only=None):
    """레시피(여럿이면 layout_designs 의 세트 배치) 블록마다 (블록, 공급 칸, 칸의 블록 중심, 칸의 블록 회전) 을 만든다.

    slots 를 주면 그 칸을 차례로 다시 쓴다. 안 주면 블록의 잡기로 정해진 칸(robot.yaml supply_slots grasp)을 쓴다
    (위에서 집으므로 자세를 못 바꾸고, 칸마다 그리퍼 방향이 고정). 맞는 칸이 없으면 ValueError.
    """
    jobs, turn = [], {}
    blocks = [b for design, _ in layout_designs(cfg, recipes) for b in design]
    for i, b in enumerate(blocks):
        if slots:
            slot = slots[i % len(slots)]
        else:
            # 블록의 잡기와 같은 잡기로 정해진 칸 (칸에 grasp 가 없으면 같은 자세의 칸) 을 차례로 쓴다
            up = up_axis(b['rot'])
            cand = [k + 1 for k, st in enumerate(cfg['supply_slots'])
                    if st.get('grasp', b['grasp']) == b['grasp'] and st.get('block_up', 'THICKNESS') == up]
            if not cand:
                raise ValueError(f'{b["block_id"]}: {b["grasp"]} 공급 칸이 robot.yaml 에 없다')
            slot = cand[turn.get(b['grasp'], 0) % len(cand)]
            turn[b['grasp']] = turn.get(b['grasp'], 0) + 1
        if only and i + 1 not in only:
            continue
        center, rot = slot_block_pose(cfg, slot)
        jobs.append((b, slot, center, rot))
    return jobs


def show(cfg, jobs):
    """블록마다 잡기·칸·집기·놓기 TCP 목표를 표로 보여 준다. 계산이 안 되는 블록이 있으면 False."""
    ok = True
    print(f'{"블록":<10} {"잡기":<12} {"칸":>2}  {"집기 TCP (m)":<24} {"닫힘°":>6}  {"놓기 TCP (m)":<24} {"닫힘°":>6}')
    for b, slot, center, rot in jobs:
        try:
            pxyz, pq, lxyz, lq, _ = pick_place_tcp(cfg, center, rot, b['center'], b['rot'], b['grasp'], slot)
        except ValueError as e:
            print(f'{b["block_id"]:<10} 계산 실패: {e}')
            ok = False
            continue
        print(f'{b["block_id"]:<10} {b["grasp"]:<12} {slot:>2}  ({", ".join(f"{v:.3f}" for v in pxyz)})'.ljust(52) +
              f'{close_angle_deg(pq):6.1f}  ({", ".join(f"{v:.3f}" for v in lxyz)})'.ljust(28) + f'{close_angle_deg(lq):6.1f}')
    return ok


def confirm(what):
    """y 를 받으면 True."""
    try:
        return input(f'  -> {what}. 펜던트를 손에 들고, 작업 영역에 손이 없으면 y: ').strip().lower() == 'y'
    except EOFError:
        return False


def main():
    """레시피를 읽어 목표를 계산하고, --check 가 아니면 홈 -> 그리퍼 초기화 -> 블록마다 PickPlace -> 홈, 결과를 CSV 로 남긴다."""
    ap = argparse.ArgumentParser(description='레시피 -> 블록마다 /d2/motion/pick_place (로봇 파트 시험)')
    ap.add_argument('recipes', nargs='+',
                    help='레시피 파일 (assembly.recipe/1.0 또는 m0609.jenga.cad_recipe/1.0). 여럿이면 세트로 나란히 (첫 설계의 −y 쪽에 다음)')
    ap.add_argument('--slots', default=None,
                    help='단계 순서대로 쓸 공급 칸 (예: 1,3). 없으면 놓을 자세와 같은 자세의 칸을 자동으로 돌려 쓴다')
    ap.add_argument('--steps', default=None, help='실행할 순번 (전체 블록 중 몇 번째, 예: 1-3,5). 없으면 전부')
    ap.add_argument('--check', action='store_true', help='목표만 계산해 보여 주고 끝낸다 (로봇 안 움직임)')
    ap.add_argument('--auto', action='store_true', help='y 확인 없이 (가상 시험용)')
    args = ap.parse_args()

    with open(os.path.join(get_package_share_directory('d2_bringup'), 'config', 'robot.yaml')) as f:
        cfg = yaml.safe_load(f)
    recipes = []
    for path in args.recipes:
        with open(path) as f:
            recipes.append(json.load(f))
    try:
        jobs = plan_jobs(cfg, recipes, parse_list(args.slots) if args.slots else None,
                         parse_list(args.steps) if args.steps else None)
    except ValueError as e:
        print(f'[레시피 계산 실패] {e}')
        return 1
    if not show(cfg, jobs) or args.check:
        return 0 if args.check else 1

    init_ros()                            # Ctrl+C 뒤에도 목표 취소를 보낼 수 있게
    node = rclpy.create_node('run_recipe')
    ac = ActionClient(node, PickPlace, '/d2/motion/pick_place')
    mv = node.create_client(MoveTo, '/d2/motion/move_to')
    grip = node.create_client(GripperCommand, '/d2/gripper/command')
    rows, gh, t_start = [], None, time.monotonic()

    def wait(fut, timeout=None):
        """future 가 끝날 때까지(또는 timeout s) spin_once 로 돌며 기다린다."""
        end = None if timeout is None else time.monotonic() + timeout
        while not fut.done() and (end is None or time.monotonic() < end):
            rclpy.spin_once(node, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def go_home():
        """/d2/motion/move_to home — 다 간 뒤 답. 반환: 도착했으면 True."""
        print('===== 홈 자세로')
        r = wait(mv.call_async(MoveTo.Request(target='home')), HOME_TIMEOUT_S)
        ok = r is not None and r.success
        print('   -> ' + ('도착' if ok else f'실패 {getattr(r, "reason", "응답 없음")}'))
        return ok

    try:
        if not ac.wait_for_server(timeout_sec=10.0) or not mv.wait_for_service(timeout_sec=5.0) \
                or not grip.wait_for_service(timeout_sec=5.0):
            print('[오류] pick_place·gripper 노드가 없다 (robot_nodes.launch.py 를 먼저 띄운다)')
            return 1
        used = set()
        if not args.auto and not confirm(f'시작: 공급 칸 {", ".join(sorted({str(s) for _, s, _, _ in jobs}))}번에 블록을 놓았으면'):
            return 1
        # 블록을 쥔 채 시작하면 홈에서 그리퍼를 열 때 떨어뜨린다 -> 사람이 먼저 빼게 하고 멈춘다
        gst = {}
        node.create_subscription(String, '/d2/gripper/state', lambda m: gst.update(json.loads(m.data)), 10)
        end = time.monotonic() + 3.0
        while not gst and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if gst.get('grasped'):
            print('[오류] 그리퍼가 블록을 쥐고 있다. 블록을 손으로 잡고 그리퍼를 연 뒤 다시 실행한다')
            return 1
        # 시작은 늘 같은 상태에서: 홈 자세 -> 그리퍼를 시작 폭으로 열고 힘을 맞춤 -> 블록 작업
        if not go_home():
            return 1
        r = wait(grip.call_async(GripperCommand.Request(width_m=START_OPEN_M, force_n=float(cfg['gripper']['force_n']))), 15.0)
        if r is None or not r.success:
            print(f'[오류] 그리퍼 초기화 실패 ({getattr(r, "reason", "응답 없음")})')
            return 1
        print(f'===== 그리퍼 시작 폭 {START_OPEN_M * 1000:.0f} mm, 힘 {cfg["gripper"]["force_n"]:.0f} N')
        for b, slot, center, rot in jobs:
            if slot in used and not args.auto and not confirm(f'{b["block_id"]}: 공급 칸 {slot}번에 블록을 다시 놓았으면'):
                break
            used.add(slot)
            goal = PickPlace.Goal(block_id=b['block_id'], supply_slot=str(slot), grasp=b['grasp'],
                                  pick_pose=make_pose(center, quat_from_axes(*(column(rot, k) for k in range(3)))),
                                  place_pose=make_pose(b['center'], b['quat']))
            print(f'===== {b["block_id"]} ({b["grasp"]}, 칸 {slot})')
            gh = wait(ac.send_goal_async(goal, feedback_callback=lambda m: print(f'   {m.feedback.step}')), 10.0)
            if gh is None or not gh.accepted:
                print('[오류] 목표를 받지 않았다')
                break
            r = wait(gh.get_result_async()).result
            gh = None
            rows.append([b['block_id'], b['grasp'], slot, r.success, r.reason,
                         '' if math.isnan(r.grip_width_m) else round(r.grip_width_m * 1000, 1), round(r.duration_s, 1)])
            print(f'   -> {"성공" if r.success else "실패 " + r.reason} ({r.duration_s:.1f} s)')
            if not r.success:
                break
        else:
            go_home()                     # 모든 블록을 놓았으면 홈으로 (실패하면 그 자리에 둔다 — 사람이 보고 판단)
    except KeyboardInterrupt:
        print('\n[Ctrl+C] 지금 목표를 취소한다 (pick_place 가 로봇을 세운다)')
        if gh is not None:
            wait(gh.cancel_goal_async(), 3.0)
    finally:
        if rows:
            model_id = '_'.join(r['model']['model_id'] if 'model' in r else r['model_id'] for r in recipes)
            out = f'run_{model_id}_{time.strftime("%m%d%H%M")}.csv'
            with open(out, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['block_id', 'grasp', 'slot', 'success', 'reason', 'grip_width_mm', 'duration_s'])
                w.writerows(rows)
            ok_n = sum(1 for r in rows if r[3])
            print(f'결과: {ok_n}/{len(jobs)} 성공, 전체 {time.monotonic() - t_start:.0f} s, 기록 {out}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

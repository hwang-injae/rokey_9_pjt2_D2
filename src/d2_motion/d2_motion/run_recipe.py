# -*- coding: utf-8 -*-
"""시험 도구 run_recipe — 레시피를 읽어 블록마다 /d2/motion/pick_place 목표를 차례로 보낸다 (작업 관리자 대신, 로봇 파트 시험용).

원본: 박진용 recipe_demo/make_targets.py + run_targets.py 의 실행 순서 (10/6 LV1 11개 실기 성공).

  ros2 run d2_motion run_recipe <001_CHAIR_BENCH.recipe.json> --check   # 목표만 계산해 보여 줌 (ROS·로봇 없음)
  ros2 run d2_motion run_recipe <001_CHAIR_BENCH.recipe.json>           # 블록마다 y 확인
  ros2 run d2_motion run_recipe <001_CHAIR_BENCH.recipe.json> --auto    # 확인 없이 (가상 시험)
--slots 없음 = 블록의 잡기(grasp)에 정해진 공급 칸(robot.yaml supply_slots 의 grasp)을 쓴다 (10/6 교시: 칸 하나 = 잡기 하나).
--slots 2,1 처럼 주면 그 칸을 차례로 쓰지만, 잡기가 다른 칸이면 계산 단계에서 거절한다.
같은 칸을 블록마다 다시 쓰므로 사람이 칸을 다시 채운다. Ctrl+C = 지금 목표 취소 -> pick_place 가 세운다.
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
from rclpy.action import ActionClient

from d2_motion.executor import make_pose
from d2_motion.motion_math import (check_in_area, close_angle_deg, column, pick_place_tcp, quat_from_axes,
                                   recipe_blocks, slot_block_pose)
from d2_safety.safe_stop import init_ros


def parse_list(spec):
    """'1-3,5' -> [1, 2, 3, 5]."""
    out = []
    for tok in spec.split(','):
        a, _, b = tok.partition('-')
        out += list(range(int(a), int(b or a) + 1))
    return out


def plan_jobs(cfg, recipe, slots=None, only=None):
    """레시피 블록마다 (블록, 공급 칸, 칸의 블록 중심, 칸의 블록 회전) 을 만든다.

    slots 를 주면 그 칸을 차례로 다시 쓴다. 안 주면 블록의 잡기와 같은 grasp 의 칸을 쓴다 (같은 잡기 칸이 여럿이면 돌려 씀).
    맞는 칸이 없거나 블록이 조립 작업공간 밖이면 ValueError.
    """
    jobs, turn = [], {}
    for i, b in enumerate(recipe_blocks(cfg, recipe)):
        check_in_area(cfg, b)
        if slots:
            slot = slots[i % len(slots)]
        else:
            cand = [k + 1 for k, st in enumerate(cfg['supply_slots']) if st.get('grasp') == b['grasp']]
            if not cand:
                raise ValueError(f'{b["block_id"]}: {b["grasp"]} 공급 칸이 robot.yaml 에 없다')
            slot = cand[turn.get(b['grasp'], 0) % len(cand)]
            turn[b['grasp']] = turn.get(b['grasp'], 0) + 1
        if only and b['sequence'] not in only:
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
    """레시피를 읽어 목표를 계산하고, --check 가 아니면 블록마다 PickPlace 를 보내 결과를 CSV 로 남긴다."""
    ap = argparse.ArgumentParser(description='레시피 -> 블록마다 /d2/motion/pick_place (로봇 파트 시험)')
    ap.add_argument('recipe', help='레시피 파일 (assembly.recipe/1.0)')
    ap.add_argument('--slots', default=None,
                    help='단계 순서대로 쓸 공급 칸 (예: 2,1). 없으면 블록의 잡기에 정해진 칸을 쓴다')
    ap.add_argument('--steps', default=None, help='실행할 sequence (예: 1-3,5). 없으면 전부')
    ap.add_argument('--check', action='store_true', help='목표만 계산해 보여 주고 끝낸다 (로봇 안 움직임)')
    ap.add_argument('--auto', action='store_true', help='y 확인 없이 (가상 시험용)')
    args = ap.parse_args()

    with open(os.path.join(get_package_share_directory('d2_bringup'), 'config', 'robot.yaml')) as f:
        cfg = yaml.safe_load(f)
    with open(args.recipe) as f:
        recipe = json.load(f)
    try:
        jobs = plan_jobs(cfg, recipe, parse_list(args.slots) if args.slots else None,
                         parse_list(args.steps) if args.steps else None)
    except ValueError as e:
        print(f'[레시피 계산 실패] {e}')
        return 1
    if not show(cfg, jobs) or args.check:
        return 0 if args.check else 1

    init_ros()                            # Ctrl+C 뒤에도 목표 취소를 보낼 수 있게
    node = rclpy.create_node('run_recipe')
    ac = ActionClient(node, PickPlace, '/d2/motion/pick_place')
    rows, gh, t_start = [], None, time.monotonic()

    def wait(fut, timeout=None):
        end = None if timeout is None else time.monotonic() + timeout
        while not fut.done() and (end is None or time.monotonic() < end):
            rclpy.spin_once(node, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    try:
        if not ac.wait_for_server(timeout_sec=10.0):
            print('[오류] /d2/motion/pick_place 가 없다. pick_place 노드를 먼저 띄운다')
            return 1
        used = set()
        if not args.auto and not confirm(f'시작: 공급 칸 {", ".join(sorted({str(s) for _, s, _, _ in jobs}))}번에 블록을 놓았으면'):
            return 1
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
    except KeyboardInterrupt:
        print('\n[Ctrl+C] 지금 목표를 취소한다 (pick_place 가 로봇을 세운다)')
        if gh is not None:
            wait(gh.cancel_goal_async(), 3.0)
    finally:
        if rows:
            out = f'run_{recipe["model"]["model_id"]}_{time.strftime("%m%d%H%M")}.csv'
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

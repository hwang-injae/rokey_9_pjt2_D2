#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""작업 3 (V-07, V-08, V-10): MoveIt2 장면에 책상, 그리퍼 대신 원기둥, 장애물을 넣는다.

이 브링업의 MoveIt2 모델은 팔(link_6 플랜지까지)뿐이다. RG2 그리퍼와 책상이 없어서
그대로 실기에서 실행하면 그리퍼가 책상에 닿는 경로가 나올 수 있다. 그래서 먼저
책상과 '그리퍼 대신 원기둥'(link_6에 붙임, 길이 = TCP + 여유)을 넣는다.

먼저 할 것: source <내 워크스페이스>/install/setup.bash, MoveIt2 브링업이 떠 있어야 한다.
길이는 m, --tcp만 mm. 좌표는 base_link 기준 (x: 로봇 앞, y: 로봇 왼쪽, z: 위).

예)
  python3 scene_setup.py --tcp 228                                     # 장면 1: 책상 + 그리퍼 대신 원기둥
  python3 scene_setup.py --tcp 228 --box                               # 장면 2: + 구조물 상자
  python3 scene_setup.py --tcp 228 --box --arm-x 0.38                  # 장면 3: + 사람 팔 원기둥
  python3 scene_setup.py --tcp 228 --box --arm-x 0.62 --arm-y 0.12     # 장면 4: 팔 원기둥을 옮김
  python3 scene_setup.py --clear                                       # 모두 지움
실행할 때마다 장면을 이 명령의 내용으로 맞춘다 (빠진 물체는 지운다).
"""
import argparse
import sys

import rclpy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (CollisionObject, AttachedCollisionObject, PlanningScene,
                             PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

FRAME = 'base_link'
TABLE_IDS = ['table_front', 'table_back', 'table_left', 'table_right']
BOX_ID = 'structure'
ARM_ID = 'human_arm'
GRIP_ID = 'gripper_standin'
HOLE = 0.15        # 로봇 바닥 둘레에 비워 두는 반폭 (m). 책상 상자가 로봇 바닥과 겹치지 않게
HOLE_RIGHT = 0.35  # 오른쪽(-y)은 바닥 케이블 연결부 모델이 나와 있어서 더 비운다 (m)
TABLE_T = 0.02     # 책상 상자 두께 (m)
BOX_XY = (0.55, 0.0)  # 구조물 상자 중심 (m)


def make_pose(x, y, z):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.w = 1.0
    return p


def make_obj(oid, frame, shape, dims, xyz):
    o = CollisionObject()
    o.header.frame_id = frame
    o.id = oid
    sp = SolidPrimitive()
    sp.type = shape
    sp.dimensions = [float(d) for d in dims]
    o.primitives = [sp]
    o.primitive_poses = [make_pose(*xyz)]
    o.operation = CollisionObject.ADD
    return o


def table_objects(z_top):
    """로봇 바닥 둘레만 비운 책상 판 4장. 윗면 높이 = z_top."""
    zc = z_top - TABLE_T / 2
    x0, x1, y0, y1 = -0.6, 1.0, -0.9, 0.9
    pieces = {
        'table_front': ((x1 - HOLE, y1 - y0), ((x1 + HOLE) / 2, 0.0)),
        'table_back': ((-HOLE - x0, y1 - y0), ((x0 - HOLE) / 2, 0.0)),
        'table_left': ((2 * HOLE, y1 - HOLE), (0.0, (y1 + HOLE) / 2)),
        'table_right': ((2 * HOLE, -HOLE_RIGHT - y0), (0.0, (y0 - HOLE_RIGHT) / 2)),
    }
    out = []
    for oid, ((sx, sy), (cx, cy)) in pieces.items():
        out.append(make_obj(oid, FRAME, SolidPrimitive.BOX, (sx, sy, TABLE_T), (cx, cy, zc)))
    return out


def gripper_standin(length, radius):
    """link_6에 붙이는 원기둥. link_6의 +z(공구 방향)로 length만큼 나온다."""
    aco = AttachedCollisionObject()
    aco.link_name = 'link_6'
    aco.object = make_obj(GRIP_ID, 'link_6', SolidPrimitive.CYLINDER, (length, radius),
                          (0.0, 0.0, length / 2))
    aco.touch_links = ['link_6', 'link_5']
    return aco


def call(node, cli, req, what):
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    if not fut.done() or fut.result() is None:
        print(f'[오류] {what}: 응답 없음')
        return None
    return fut.result()


def apply(node, cli, scene, what):
    req = ApplyPlanningScene.Request()
    req.scene = scene
    res = call(node, cli, req, what)
    ok = res is not None and res.success
    if not ok:
        print(f'[오류] {what}: 실패 (move_group 터미널의 오류 줄을 본다)')
    return ok


def empty_diff():
    s = PlanningScene()
    s.is_diff = True
    s.robot_state.is_diff = True
    return s


def main():
    ap = argparse.ArgumentParser(description='작업 3: MoveIt2 장면 설정')
    ap.add_argument('--tcp', type=float, help='작업 2가 보낸 TCP 길이 (플랜지 -> 손가락 끝, mm)')
    ap.add_argument('--margin', type=float, default=20.0, help='그리퍼 원기둥을 손가락 끝보다 더 길게 (mm, 기본 20)')
    ap.add_argument('--radius', type=float, default=0.07, help='그리퍼 원기둥 반지름 (m, 기본 0.07)')
    ap.add_argument('--table-z', type=float, default=0.0, help='책상 윗면 높이 (m, 로봇 바닥면 기준, 기본 0)')
    ap.add_argument('--box', action='store_true', help='구조물 상자 0.10 x 0.10 x 0.20 m를 (0.55, 0) 에 둔다')
    ap.add_argument('--arm-x', type=float, help='사람 팔 원기둥(지름 0.10, 높이 0.50 m)의 x 위치 (y는 --arm-y)')
    ap.add_argument('--arm-y', type=float, default=0.0, help='사람 팔 원기둥의 y 위치 (기본 0)')
    ap.add_argument('--clear', action='store_true', help='이 스크립트가 넣은 물체를 모두 지운다')
    ap.add_argument('--ns', default='', help='브링업 name 인자와 같은 값 (기본: 빈 값)')
    args = ap.parse_args()
    if not args.clear and args.tcp is None:
        print('[오류] --tcp (mm)를 준다. 작업 2가 보낸 TCP 값이다. 못 받았으면 230으로 하고 기록에 적는다.')
        return 1
    if args.tcp is not None and not (150.0 <= args.tcp <= 350.0):
        print('[오류] --tcp는 mm 단위다 (예: 228). 값을 다시 본다.')
        return 1

    rclpy.init()
    node = rclpy.create_node('task3_scene_setup', namespace=args.ns)
    try:
        apply_cli = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
        get_cli = node.create_client(GetPlanningScene, 'get_planning_scene')
        if not (apply_cli.wait_for_service(timeout_sec=10.0) and get_cli.wait_for_service(timeout_sec=10.0)):
            print('[오류] move_group 서비스를 찾지 못했다. 브링업이 다 떴는지, --ns가 브링업 name과 같은지 본다.')
            print('       확인: ros2 service list | grep apply_planning_scene')
            return 1

        # 지금 장면에 있는 물체 이름
        greq = GetPlanningScene.Request()
        greq.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                       | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        gres = call(node, get_cli, greq, '장면 읽기')
        if gres is None:
            return 1
        world_now = {o.id for o in gres.scene.world.collision_objects}
        attached_now = {a.object.id for a in gres.scene.robot_state.attached_collision_objects}

        want_world = []
        want_attached = None
        if not args.clear:
            want_world += table_objects(args.table_z)
            if args.box:
                want_world.append(make_obj(BOX_ID, FRAME, SolidPrimitive.BOX, (0.10, 0.10, 0.20),
                                           (BOX_XY[0], BOX_XY[1], args.table_z + 0.10)))
            if args.arm_x is not None:
                want_world.append(make_obj(ARM_ID, FRAME, SolidPrimitive.CYLINDER, (0.50, 0.05),
                                           (args.arm_x, args.arm_y, args.table_z + 0.25)))
            length = (args.tcp + args.margin) / 1000.0
            want_attached = gripper_standin(length, args.radius)
        want_ids = {o.id for o in want_world}

        # 1) 그리퍼 원기둥을 떼어 낼 때: 떼면 월드 물체로 남으므로 한 번 더 지운다
        if want_attached is None and GRIP_ID in attached_now:
            s = empty_diff()
            aco = AttachedCollisionObject()
            aco.link_name = 'link_6'
            aco.object.id = GRIP_ID
            aco.object.operation = CollisionObject.REMOVE
            s.robot_state.attached_collision_objects = [aco]
            if not apply(node, apply_cli, s, '그리퍼 원기둥 떼기'):
                return 1
            world_now.add(GRIP_ID)

        # 2) 필요 없는 월드 물체 지우기 (이 스크립트가 쓰는 이름만)
        ours = set(TABLE_IDS) | {BOX_ID, ARM_ID, GRIP_ID}
        remove = [oid for oid in world_now if oid in ours and oid not in want_ids]
        if remove:
            s = empty_diff()
            for oid in remove:
                o = CollisionObject()
                o.header.frame_id = FRAME
                o.id = oid
                o.operation = CollisionObject.REMOVE
                s.world.collision_objects.append(o)
            if not apply(node, apply_cli, s, '물체 지우기'):
                return 1

        # 3) 넣을 물체 넣기 (같은 이름이 있으면 바꾼다)
        if want_world or want_attached is not None:
            s = empty_diff()
            s.world.collision_objects = want_world
            if want_attached is not None:
                s.robot_state.attached_collision_objects = [want_attached]
            if not apply(node, apply_cli, s, '물체 넣기'):
                return 1

        if args.clear:
            print('[완료] 이 스크립트가 넣은 물체를 모두 지웠다.')
        else:
            parts = [f'책상(윗면 z={args.table_z:.3f} m)',
                     f'그리퍼 대신 원기둥(길이 {(args.tcp + args.margin) / 1000.0:.3f} m, 반지름 {args.radius:.2f} m)']
            if args.box:
                parts.append(f'구조물 상자 0.10x0.10x0.20 m @ ({BOX_XY[0]:.2f}, {BOX_XY[1]:.2f})')
            if args.arm_x is not None:
                parts.append(f'사람 팔 원기둥 d0.10 h0.50 m @ ({args.arm_x:.2f}, {args.arm_y:.2f})')
            print('[완료] ' + ', '.join(parts))
            print('RViz에서 장애물(초록)과 팔 끝의 원기둥이 보이는지 확인한다.')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

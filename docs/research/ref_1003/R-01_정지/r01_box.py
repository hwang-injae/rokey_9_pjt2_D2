#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[R-01 시험용] MoveIt 장면에 '가상 상자'(r01_box)를 넣거나 뺀다. 실물은 두지 않는다.

pick_place_avoid.py 는 관측 자세에서 본 장애물만 쓰므로, 움직이는 중에 실물을 놓아도 '막힘'이 생기지 않는다.
그래서 움직이는 중에 이 스크립트로 경로 위에 가상 상자를 넣어 '앞길 막힘 -> 정지'를 시험한다.

  python3 r01_box.py add 0.37 0.0 0.20 0.10     # 중심 x y z (m, base_link), 한 변 (m)
  python3 r01_box.py remove                      # 빼기 (빼면 로봇이 다시 길을 찾아 움직인다)
"""
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('add', 'remove') or (sys.argv[1] == 'add' and len(sys.argv) != 6):
        print(__doc__)
        return 1
    rclpy.init()
    node = rclpy.create_node('r01_box')
    cli = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
    if not cli.wait_for_service(timeout_sec=5.0):
        print('[오류] move_group 이 안 보인다.')
        return 1
    o = CollisionObject()
    o.header.frame_id = 'base_link'
    o.id = 'r01_box'
    if sys.argv[1] == 'add':
        x, y, z, s = (float(v) for v in sys.argv[2:6])
        o.operation = CollisionObject.ADD
        o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[s, s, s])]
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.0
        o.primitive_poses = [p]
    else:
        o.operation = CollisionObject.REMOVE
    sc = PlanningScene()
    sc.is_diff = True
    sc.robot_state.is_diff = True
    sc.world.collision_objects = [o]
    fut = cli.call_async(ApplyPlanningScene.Request(scene=sc))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    ok = fut.done() and fut.result().success
    print(f'{sys.argv[1]}: {"성공" if ok else "실패"}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

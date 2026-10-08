# -*- coding: utf-8 -*-
"""장면 관리 노드 scene_manager — MoveIt2 장면을 고치는 유일한 노드. 1차 장면 = 작업대 + 쌓인 블록 + 쥔 블록 (W036)."""
import json
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from d2_interfaces.srv import SceneAttach
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, ObjectColor, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, String

from d2_motion.executor import make_pose
from d2_motion.motion_math import layout_designs, load_recipe, pick_place_tcp, recipe_files, slot_block_pose, tcp_target

HELD = 'held'
TABLE_THICK_M = 0.02
PLACED_SHRINK_M = 0.0005     # 놓인 블록 상자를 면마다 줄인다 — 면끼리 맞닿은 블록(좌판 틈 0 mm)을 충돌로 보지 않게
FINGER_LINKS = ['rg2_base_link', 'rg2_left_outer_knuckle', 'rg2_left_inner_knuckle', 'rg2_left_inner_finger',
                'rg2_right_outer_knuckle', 'rg2_right_inner_knuckle', 'rg2_right_inner_finger']
PLACED_RGBA, HELD_RGBA = (0.85, 0.65, 0.3, 1.0), (0.25, 0.5, 0.95, 1.0)
PROGRESS_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST, depth=1)   # IRD 6장 progress/1 (10/4 확정)


class SceneManagerNode(Node):
    """MoveIt2 장면을 고치는 유일한 노드. 다른 노드는 apply_planning_scene 을 부르지 않는다.

    받는 것: /d2/motion/scene/attach (SceneAttach — 잡은 뒤 붙이기 / 놓은 뒤 떼기 = 놓인 블록으로),
             /d2/task/progress (JSON progress/1 — 놓인 블록 blk_<block_id> 를 관측 자세로 맞춤)
    부르는 것: MoveIt2 apply_planning_scene, get_planning_scene
    파라미터 recipe: 레시피 파일 경로 (여럿이면 쉼표로, 세트 배치) — 쥔 블록 상자 크기·위치와 놓은 자리를 여기서 계산한다.
                    비우면 share/d2_bringup/recipes/ 의 레시피(*_recipe.json)를 하나씩(세트 배치 없이) 모두 읽는다 — run_recipe 가 번호로 고를 때.
    """

    def __init__(self):
        """설정·레시피를 읽고 서비스·구독을 만든다. 장면 초기화는 run() 에서 move_group 이 뜬 뒤 한다."""
        super().__init__('scene_manager')
        share = get_package_share_directory('d2_bringup')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            self.cfg = yaml.safe_load(f)
        paths = [p for p in self.declare_parameter('recipe', '').value.split(',') if p.strip()]
        self.blocks = {}
        if paths:
            recipes = [load_recipe(path.strip()) for path in paths]
            # 여러 레시피면 run_recipe 와 같은 세트 배치 (layout_designs 는 계산만으로 정해져 같은 자리가 나온다)
            self.blocks = {b['block_id']: b for design, _ in layout_designs(self.cfg, recipes) for b in design}
            self.get_logger().info(f'레시피 {len(paths)}개, 블록 {len(self.blocks)}개: {", ".join(paths)}')
        else:
            # 설치된 레시피를 하나씩 읽는다: block_id 에 모델 이름이 붙어 있어(001_CHAIR_BENCH_LEG_001_01) 한 사전에 넣어도 겹치지 않는다.
            # 레시피 하나를 쌓는 자리는 layout_designs 의 설계 1개 = 조립 원점 그대로라, run_recipe 에서 하나를 골라도 자리가 같다
            for path in recipe_files(os.path.join(share, 'recipes')):
                self.blocks.update({b['block_id']: b for design, _ in layout_designs(self.cfg, [load_recipe(path)]) for b in design})
                self.get_logger().info(f'레시피: {path}')
            if not self.blocks:
                self.get_logger().warn('레시피가 없다 — 쥔 블록·놓은 블록을 장면에 넣지 못한다')
        cb = ReentrantCallbackGroup()
        self.scene_cli = self.create_client(ApplyPlanningScene, 'apply_planning_scene', callback_group=cb)
        self.get_cli = self.create_client(GetPlanningScene, 'get_planning_scene', callback_group=cb)
        self.create_service(SceneAttach, '/d2/motion/scene/attach', self._on_attach, callback_group=cb)
        self.create_subscription(String, '/d2/task/progress', self._on_progress, PROGRESS_QOS, callback_group=cb)
        self.placed = set()               # 장면에 넣은 놓인 블록 id

    def apply(self, scene):
        """장면 변경(diff)을 move_group 에 보낸다. 반환: 적용됐으면 True."""
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if not self.scene_cli.wait_for_service(timeout_sec=2.0):
            return False
        fut = self.scene_cli.call_async(ApplyPlanningScene.Request(scene=scene))
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.005)
        return bool(fut.done() and fut.result() and fut.result().success)

    def _box(self, oid, size, pose, op=CollisionObject.ADD):
        """base_link 기준 상자 충돌 물체."""
        o = CollisionObject(id=oid, operation=op)
        o.header.frame_id = self.cfg['frame_id']
        o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])]
        o.primitive_poses = [pose]
        return o

    def _remove(self, oid):
        """world 물체 oid 를 지우는 CollisionObject."""
        o = CollisionObject(id=oid, operation=CollisionObject.REMOVE)
        o.header.frame_id = self.cfg['frame_id']
        return o

    @staticmethod
    def _color(oid, rgba):
        """RViz 에 보일 물체 색."""
        return ObjectColor(id=oid, color=ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3]))

    def reset(self):
        """장면을 처음 상태로: 작업대 (로봇 바닥 둘레만 비움) 를 넣고, 지난 실행의 놓인 블록·쥔 블록을 지운다."""
        tb = self.cfg['table']
        (x0, x1), (y0, y1), h, z = tb['x_m'], tb['y_m'], tb['base_clear_m'], self.cfg['table_z_m']
        sc = PlanningScene()
        for i, (a, b, c, d) in enumerate([(x0, -h, y0, y1), (h, x1, y0, y1), (-h, h, h, y1), (-h, h, y0, -h)]):
            if b > a and d > c:
                # 0.5 mm 아래로: 작업대 윗면에 놓인 블록·손가락 끝 보정점이 작업대와 겹쳐 보이지 않게
                sc.world.collision_objects.append(self._box(
                    f'table_{i}', (b - a, d - c, TABLE_THICK_M), make_pose(((a + b) / 2, (c + d) / 2, z - TABLE_THICK_M / 2 - 0.0005))))
        # 지난 실행의 블록·쥔 블록은 지금 장면에 있는 것만 지운다: 없는 물체를 지우라고 하면 MoveIt 이 변경 전체를 실패로 답한다
        world, attached = self._current_objects()
        sc.world.collision_objects += [self._remove(oid) for oid in world if oid.startswith('blk_') or oid == HELD]
        if HELD in attached:
            aco = AttachedCollisionObject(link_name=self.cfg['tcp_link'])
            aco.object.id, aco.object.operation = HELD, CollisionObject.REMOVE
            sc.robot_state.attached_collision_objects = [aco]
            # 떼기만 하면 MoveIt 이 상자를 world 로 내려놓아 그 자리에 남는다(10/7 실기 — 다음 집기가 PLAN_FAILED).
            # 같은 변경 안에서 robot_state 가 world 보다 먼저 적용되므로 뗀 뒤 world 에서도 지운다(_on_attach 떼기와 같음)
            if HELD not in world:
                sc.world.collision_objects.append(self._remove(HELD))
        ok = self.apply(sc)
        self.placed.clear()
        return ok

    def _current_objects(self):
        """move_group 의 지금 장면에서 (world 물체 이름 목록, 붙은 물체 이름 목록). 못 읽으면 빈 목록."""
        comp = PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        fut = self.get_cli.call_async(GetPlanningScene.Request(components=PlanningSceneComponents(components=comp)))
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.005)
        if not (fut.done() and fut.result()):
            return [], []
        sc = fut.result().scene
        return ([o.id for o in sc.world.collision_objects],
                [a.object.id for a in sc.robot_state.attached_collision_objects])

    def add_placed(self, block_id, center, quat):
        """놓인 블록 blk_<block_id> 를 base 자세 (중심 m, 쿼터니언) 로 넣거나 옮긴다."""
        size = [max(0.001, v - 2 * PLACED_SHRINK_M) for v in self.cfg['block_actual_m']]
        oid = f'blk_{block_id}'
        sc = PlanningScene()
        sc.world.collision_objects = [self._box(oid, size, make_pose(center, quat))]
        sc.object_colors = [self._color(oid, PLACED_RGBA)]
        if self.apply(sc):
            self.placed.add(block_id)

    def _held_box(self, b):
        """블록 b 를 쥐었을 때 TCP 기준 상자 (크기·위치). 공급 칸은 잡기마다 하나로 고정이라, 그 칸의 교시 깊이까지
        반영한다 — 세운 블록은 계산식(11 mm)보다 깊게 물어서, 안 그러면 상자가 실제보다 아래로 붙어 작업대와 겹친다 (10/6 실기)."""
        slots = [k + 1 for k, st in enumerate(self.cfg['supply_slots']) if st.get('grasp') == b['grasp']]
        if not slots:
            return tcp_target(self.cfg, b['center'], b['rot'], b['grasp'], self.cfg['assembly_origin'])[2]
        center, rot = slot_block_pose(self.cfg, slots[0])
        return pick_place_tcp(self.cfg, center, rot, b['center'], b['rot'], b['grasp'], slots[0])[4]

    def _on_attach(self, req, res):
        """쥔 블록 붙이기 (attach=True: TCP 에 상자) / 떼기 (False: 쥔 상자를 지우고 레시피 자리에 놓인 블록으로).

        레시피에 없는 block_id 면 success=false, reason UNKNOWN_BLOCK.
        """
        b = self.blocks.get(req.block_id)
        if b is None:
            res.success, res.reason = False, 'UNKNOWN_BLOCK'
            return res
        tcp = self.cfg['tcp_link']
        sc = PlanningScene()
        aco = AttachedCollisionObject(link_name=tcp)
        aco.object.id = HELD
        if req.attach:
            held = self._held_box(b)
            aco.object.header.frame_id = tcp
            aco.object.operation = CollisionObject.ADD
            aco.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in held['size_m']])]
            aco.object.primitive_poses = [make_pose(held['offset_m'])]
            aco.touch_links = FINGER_LINKS + [tcp]      # 손가락이 쥔 블록에 닿는 것은 충돌이 아니다
            sc.object_colors = [self._color(HELD, HELD_RGBA)]
        else:
            aco.object.operation = CollisionObject.REMOVE
            sc.world.collision_objects = [self._remove(HELD)]
        sc.robot_state.attached_collision_objects = [aco]
        res.success = self.apply(sc)
        if res.success and not req.attach:
            self.add_placed(req.block_id, b['center'], b['quat'])
        res.reason = '' if res.success else 'TIMEOUT'
        return res

    def _on_progress(self, msg):
        """진행표 progress/1 를 따라 놓인 블록을 맞춘다: present 는 진행표의 center_m · quat(설계 자리)로 넣고, absent 는 뺀다.
        occluded · unknown 은 있는지 모르므로 그대로 둔다. state 이름은 IRD 2장 '블록 관측 states'(task_planner.STATES)와 같다.

        진행표에 없는 놓인 블록(앞 설계 것)도 뺀다 — 진행표는 지금 설계의 블록 전부라, 설계가 바뀌면 앞 설계 블록이
        조립 영역에 남아 새 설계의 놓기 경로를 막는다(10/7 실기: 벤치 11개가 남아 003 PLAN_FAILED).
        """
        blocks = json.loads(msg.data).get('blocks', [])
        gone = self.placed - {blk.get('block_id') for blk in blocks}
        for blk in blocks:
            bid, state = blk.get('block_id'), blk.get('state')
            if state == 'present' and blk.get('center_m') and blk.get('quat'):
                self.add_placed(bid, blk['center_m'], blk['quat'])
            elif state == 'absent' and bid in self.placed:
                gone.add(bid)
        if gone:
            sc = PlanningScene()
            sc.world.collision_objects = [self._remove(f'blk_{bid}') for bid in sorted(gone)]
            if self.apply(sc):
                self.placed -= gone


def main():
    """장면 관리 노드를 켠다. move_group 이 뜨기를 기다렸다가 장면을 처음 상태로 맞춘다."""
    rclpy.init()
    node = SceneManagerNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        if not node.scene_cli.wait_for_service(timeout_sec=30.0):
            node.get_logger().error('apply_planning_scene 이 없다. 브링업(real_moveit.launch.py)을 먼저 띄운다')
            return
        # reset() 은 서비스 답을 기다리므로 executor 가 도는 동안 다른 스레드에서 부른다
        threading.Thread(target=lambda: node.get_logger().info(
            '[장면] 작업대 넣음' if node.reset() else '[장면] 초기화 실패'), daemon=True).start()
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

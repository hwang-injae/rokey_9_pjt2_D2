# -*- coding: utf-8 -*-
"""집기·놓기 노드 pick_place — 블록 1개를 공급 칸에서 집어 조립 자리에 놓는다. 팔을 움직이라고 하는 유일한 노드 (W037)."""
import json
import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from d2_interfaces.action import PickPlace
from d2_interfaces.srv import GripperCommand, MoveTo, SceneAttach
from dsr_msgs2.srv import GetCurrentTcp, GetCurrentTool
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from d2_motion.executor import MoveItExecutor
from d2_motion.motion_math import GRASP_AXIS, matrix_from_quat, pick_place_tcp
from d2_safety.safe_stop import SafeStop, init_ros

PLANNERS = ('BiTRRT', 'RRTConnect')    # BiTRRT 가 먼저 (관절을 덜 돌리는 길), 실패하면 RRTConnect. 경로는 MoveIt 계획 그대로 쓴다
LINE_SCALE = 0.5                       # 수직 직선 이동은 블록 가까이라 자유 이동의 절반 속도 (Pilz 직선 속도 한계 x 비율)
GRIPPER_TIMEOUT_S = 10.0
SAFETY_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=1)   # 정지 노드와 같게 (IRD 4.1, S-26)


def load_config(name):
    """d2_bringup 의 config/<name> (robot.yaml · tcp.json · tool.json) 을 읽는다. 없으면 예외 (설정 없이 로봇을 움직이지 않는다)."""
    with open(os.path.join(get_package_share_directory('d2_bringup'), 'config', name)) as f:
        return yaml.safe_load(f) if name.endswith('.yaml') else json.load(f)


class PickPlaceNode(Node):
    """블록 1개 집기·놓기 액션 서버 + 관측·홈 자세 이동 서비스. MoveIt2 실행기는 같은 프로그램 안 클래스(MoveItExecutor).

    받는 것: /d2/motion/pick_place (액션 PickPlace), /d2/motion/move_to (MoveTo),
             /d2/safety/state (JSON safety_state/1 — stopped 면 지금 동작을 세우고 목표를 실패로 끝냄,
             locked 동안 새 목표를 받지 않음. S-26)
    부르는 것: /d2/gripper/command (GripperCommand), /d2/motion/scene/attach (SceneAttach), MoveIt2 move_group,
               켤 때 두산 tcp/get_current_tcp · tool/get_current_tool (config/tcp.json·tool.json 과 비교)
    PickPlace 의 pick_pose·place_pose 는 블록 중심 자세(base_link, m, 블록 자신의 축)로 받고, 손가락 끝 목표는 여기서 계산한다 (안).
    """

    def __init__(self):
        """설정을 읽고 실행기·정지 기능·서버·클라이언트를 만든다."""
        super().__init__('pick_place')
        self.cfg = load_config('robot.yaml')
        self.controller_error = None     # 제어기 TCP·툴이 tcp.json·tool.json 과 다르면 이유 (목표를 거절)
        cb = ReentrantCallbackGroup()
        self.stopper = SafeStop(self, self.cfg['stop']['first_wait_s'], spin_node=False)   # executor 는 다른 스레드
        self.exe = MoveItExecutor(self, self.cfg, self.stopper, cb)
        self.grip_cli = self.create_client(GripperCommand, '/d2/gripper/command', callback_group=cb)
        self.attach_cli = self.create_client(SceneAttach, '/d2/motion/scene/attach', callback_group=cb)
        self.create_subscription(String, '/d2/safety/state', self._on_safety, SAFETY_QOS, callback_group=cb)
        self.create_subscription(String, '/d2/gripper/state', self._on_gripper, 10, callback_group=cb)
        self.holding = False              # 그리퍼가 블록을 쥐고 있나 (gripper/state grasped)
        self.halt_reason = None           # 정지 노드가 멈춘 이유 (safety/state reason). 잠금이 풀리면 지운다
        self.locked = False
        self.busy = threading.Lock()      # 팔은 하나라 목표를 하나씩만 실행한다
        ActionServer(self, PickPlace, '/d2/motion/pick_place', self._execute, callback_group=cb,
                     cancel_callback=lambda _gh: CancelResponse.ACCEPT)
        self.create_service(MoveTo, '/d2/motion/move_to', self._on_move_to, callback_group=cb)

    # ---------- 입력 ----------
    def _on_safety(self, msg):
        """정지 노드의 safety_state/1 — stopped 면 이유를 적어 실행 중인 동작을 바로 세우게 하고, 잠금 상태를 따라간다.

        halt 이유는 잠금이 풀리는 순간(잠김 -> 풀림)에만 지운다.
        """
        st = json.loads(msg.data)
        locked = bool(st.get('locked'))
        if st.get('stopped'):
            self.halt_reason = st.get('reason') or 'STOPPED'
        elif self.locked and not locked:
            self.halt_reason = None
        self.locked = locked

    def _on_gripper(self, msg):
        """그리퍼 노드의 gripper_state/1 — 블록을 쥐고 있는지 적는다 (쥔 채로 새 목표를 시작하지 않게)."""
        self.holding = bool(json.loads(msg.data).get('grasped'))

    def _halted(self, gh=None):
        """멈춰야 할 이유 (정지 노드의 stopped, 목표 취소) 를 돌려준다. 없으면 None."""
        if self.halt_reason:
            return self.halt_reason
        if gh is not None and gh.is_cancel_requested:
            return 'CANCELED'
        return None

    # ---------- 동작 단위 ----------
    def go_free(self, goal, halted, keep_down=True, skip_slot=None):
        """지금 자세 -> 관절 목표 goal 로 MoveIt 경로를 찾아 간다. 반환: (성공, 실패 이유).

        skip_slot = 장애물로 넣지 않을 공급 칸 (집으러 가는 칸, 방금 집은 칸).
        """
        # 출발 자세가 이미 '공구 아래'를 어기면(예: 관절 0° 로 선 자세) 제약을 걸면 출발부터 실패한다 -> 이번 이동만 제약 없이
        if keep_down and not self.exe.valid(self.exe.current(), self.exe.down_constraint())[0]:
            self.get_logger().info('출발 자세가 공구 아래가 아니라 자세 제약 없이 계획한다')
            keep_down = False
        # 놓인 블록에서 transit_clearance_m 띄운 길을 먼저 찾고, 없으면 절반 여유로 한 번 더 (그래도 없으면 안 움직인다)
        clear = self.cfg['motion']['transit_clearance_m']
        jt = None
        for pad in (clear, clear / 2):
            for planner in PLANNERS:
                jt, err = self.exe.plan(goal, keep_down, planner, pad_m=pad, skip_slot=skip_slot)
                if jt is not None:
                    break
                self.get_logger().warn(f'길 찾기 실패 ({planner}, 여유 {pad * 1000:.0f} mm): {err}')
            if jt is not None:
                break
        if jt is None:
            return False, 'PLAN_FAILED'
        return self.exe.execute(jt, halted)

    def go_line(self, b_xyz, quat, halted):
        """TCP 를 지금 자세에서 b_xyz 까지 수직 직선으로 옮긴다 (MoveIt Pilz LIN). 실행 전에 궤적 점마다 충돌을 본다.

        반환: (성공, 실패 이유).
        """
        jt, err = self.exe.plan_line(b_xyz, quat, self.cfg['speed_scale'] * LINE_SCALE)
        if jt is None:
            self.get_logger().error(f'직선 경로를 못 만든다 (Pilz LIN): {err}')
            return False, 'PLAN_FAILED'
        path = self.exe.positions(jt)
        for k in range(0, len(path), 2):
            ok, hits = self.exe.valid(path[k])
            if not ok:
                self.get_logger().error(f'수직 경로 충돌 {k}/{len(path) - 1}: {", ".join(hits)}')
                return False, 'PLAN_FAILED'
        return self.exe.execute(jt, halted)

    def grip(self, width_m):
        """그리퍼 노드에 폭 width_m 로 움직이라고 하고 다 움직일 때까지 기다린다. 반환: 결과 (응답 없으면 None)."""
        if not self.grip_cli.wait_for_service(timeout_sec=2.0):
            return None
        req = GripperCommand.Request(width_m=float(width_m), force_n=float(self.cfg['gripper']['force_n']))
        return self.exe.wait_future(self.grip_cli.call_async(req), GRIPPER_TIMEOUT_S)

    def scene_attach(self, block_id, attach):
        """장면 관리 노드에 쥔 블록 붙이기(attach=True)·떼기를 부탁한다. 장면은 그 노드만 고친다. 반환: 성공하면 True."""
        if not self.attach_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('장면 관리 노드가 없다 — 쥔 블록 없이 계획한다')
            return False
        res = self.exe.wait_future(self.attach_cli.call_async(SceneAttach.Request(block_id=block_id, attach=attach)), 5.0)
        return bool(res and res.success)

    # ---------- 블록 1개 ----------
    def _targets(self, g):
        """목표의 블록 자세 -> 집기·놓기 TCP 목표와 IK. 반환: dict, 풀리지 않으면 None."""
        def pose(p):
            """geometry_msgs/Pose -> (위치 m 튜플, 3x3 회전 행렬)."""
            return ((p.position.x, p.position.y, p.position.z),
                    matrix_from_quat((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)))
        (pc, pr), (lc, lr) = pose(g.pick_pose), pose(g.place_pose)
        slot = int(g.supply_slot) if g.supply_slot.isdigit() else None
        try:
            pick_xyz, pick_q, place_xyz, place_q, _ = pick_place_tcp(self.cfg, pc, pr, lc, lr, g.grasp, slot)
        except (ValueError, KeyError) as e:
            self.get_logger().error(f'{g.block_id}: 목표 계산 실패 — {e}')
            return None
        m = self.cfg['motion']
        # IK 는 홈 자세에서 출발해 푼다: 지금 자세(예: 관절 0° 로 선 자세)에서 풀면 팔을 뒤로 꺾은 해(관절 1 이 154°)가
        # 나와 놓는 자리까지 길을 못 찾은 적이 있다 (10/6 가상). 놓기는 집기 해에서 출발한다 (원본 run_targets 와 같음)
        out, seed = {}, [math.radians(v) for v in self.cfg['home_pose']['joints_deg']]
        for kind, xyz, q, up in (('pick', pick_xyz, pick_q, m['pick_up_m']), ('place', place_xyz, place_q, m['place_up_m'])):
            low = (xyz[0], xyz[1], xyz[2] + up)
            high = (low[0], low[1], low[2] + m['approach_m'])
            qa, quat = self.exe.solve(high, q, seed)
            # 움직이기 전에 위 자세에서 수직으로 내려가는 직선이 풀리는지 미리 본다 (Pilz LIN, 위 자세에서 출발)
            down = None if qa is None else self.exe.plan_line(low, quat, self.cfg['speed_scale'] * LINE_SCALE, start_q=qa)[0]
            if down is None:
                self.get_logger().error(f'{g.block_id}: {kind} IK·직선 실패 ({", ".join(f"{v:.3f}" for v in high)})')
                return None
            out[kind] = {'qa': qa, 'quat': quat, 'high': high, 'low': low}
            seed = qa
        return out

    def _execute(self, gh):
        """PickPlace 목표 1개: 집기 위 -> 열기 -> 하강 -> 닫기·잡힘 확인 -> 상승 -> 놓기 위 -> 하강 -> 열기 -> 상승.

        바깥 영향: 로봇 팔·그리퍼가 움직이고, 장면 관리에 쥔 블록 붙이기·떼기를 부탁한다.
        실패·정지·취소 때는 세운 뒤 success=false 와 이유 코드(IRD 7장, 정지면 halt 이유)로 끝낸다.
        """
        g, t0 = gh.request, time.monotonic()
        res = PickPlace.Result()

        def finish(ok, reason='', width=float('nan')):
            """액션 결과를 채운다(성공·이유·잡은 폭 m·걸린 시간 s)."""
            res.success, res.reason, res.grip_width_m, res.duration_s = ok, reason, width, time.monotonic() - t0
            if ok:
                gh.succeed()
            elif reason == 'CANCELED':
                gh.canceled()
            else:
                gh.abort()
            self.get_logger().info(f'{g.block_id}: {"성공" if ok else "실패 " + reason} ({res.duration_s:.1f} s)')
            return res

        if self.locked:
            return finish(False, self.halt_reason or 'STOPPED')
        if self.controller_error:
            return finish(False, 'TCP_MISMATCH')
        if self.holding:
            # 블록을 쥔 채 출발하면 다른 칸으로 들고 가거나, 집는 폭 '열기' 가 오히려 조이는 명령이 된다 (10/6 실기)
            self.get_logger().error(f'{g.block_id}: 그리퍼가 블록을 쥐고 있다 — 사람이 블록을 빼고 연 뒤 다시')
            return finish(False, 'HOLDING_BLOCK')
        if g.grasp not in GRASP_AXIS:
            return finish(False, 'PLAN_FAILED')
        if not self.busy.acquire(blocking=False):
            return finish(False, 'BUSY')
        try:
            p = self._targets(g)
            if p is None:
                return finish(False, 'PLAN_FAILED')
            pk, pl = p['pick'], p['place']
            slot = int(g.supply_slot) if g.supply_slot.isdigit() else None
            halted = lambda: self._halted(gh)       # noqa: E731
            width = float('nan')

            def step(name):
                """중간 보고(step)를 보내고 로그에 남긴다."""
                gh.publish_feedback(PickPlace.Feedback(step=name))
                self.get_logger().info(f'{g.block_id}: {name}')

            step('approach')
            ok, why = self.go_free(pk['qa'], halted, skip_slot=slot)
            if not ok:
                return finish(False, why)
            # 집는 폭으로 여는 것은 집을 블록 바로 위에서 한다: 이동 중 벌린 손가락이 다른 것에 걸리지 않게 (10/6 실기)
            r = self.grip(self.cfg['grasp_open_pick_m'][g.grasp])
            if r is None or not r.success:
                return finish(False, 'GRASP_FAILED')
            ok, why = self.go_line(pk['low'], pk['quat'], halted)
            if not ok:
                return finish(False, why)
            step('grasp')
            r = self.grip(self.cfg['grasp_close_m'][g.grasp])
            if r is None or not r.success:
                return finish(False, 'GRASP_FAILED')
            width = r.width_m
            expect, tol = self.cfg['grasp_width_m'][g.grasp], self.cfg['gripper']['check_tolerance_m']
            # 폭을 모르면(가상 그리퍼) 그리퍼 노드의 잡힘만 믿는다. 알면 이 잡기의 블록 폭과 맞는지도 본다
            if not r.grasped or (not math.isnan(width) and abs(width - expect) > tol):
                self.get_logger().error(f'{g.block_id}: 잡힘 실패 (폭 {width * 1000:.1f} mm, 기대 {expect * 1000:.1f})')
                return finish(False, 'GRASP_FAILED', width)
            self.scene_attach(g.block_id, True)
            step('lift')
            ok, why = self.go_line(pk['high'], pk['quat'], halted)
            if not ok:
                return finish(False, why, width)
            step('move')
            ok, why = self.go_free(pl['qa'], halted, skip_slot=slot)
            if not ok:
                return finish(False, why, width)
            step('place')
            ok, why = self.go_line(pl['low'], pl['quat'], halted)
            if not ok:
                return finish(False, why, width)
            r = self.grip(self.cfg['grasp_open_place_m'][g.grasp])
            if r is None or not r.success:
                return finish(False, 'GRASP_FAILED', width)
            self.scene_attach(g.block_id, False)
            step('retreat')
            ok, why = self.go_line(pl['high'], pl['quat'], halted)
            return finish(ok, why, width)
        except Exception as e:            # 예상 못 한 오류: 궤적이 아직 돌고 있을 수 있으니 먼저 세운다
            self.get_logger().error(f'{g.block_id}: 오류 {e!r}')
            self.stopper.stop('ERROR')
            return finish(False, 'ERROR')
        finally:
            self.busy.release()

    def check_controller(self):
        """제어기에 지금 켜진 TCP·툴 이름을 물어 config/tcp.json·tool.json 과 맞는지 본다.

        툴(무게·무게중심)이 틀리면 두산 충돌 감지가 틀어지고, TCP 가 틀리면 펜던트 좌표가 손가락 끝이 아니다.
        다르면 controller_error 에 이유를 적고 이후 목표를 거절한다 (TCP_MISMATCH).
        이름이 비어 오면(에뮬레이터 등 등록 없음) 경고만 한다. 서비스가 없으면 확인하지 않는다.
        """
        want = {'tcp': load_config('tcp.json')['name'], 'tool': load_config('tool.json')['name']}
        for kind, srv in (('tcp', GetCurrentTcp), ('tool', GetCurrentTool)):
            cli = self.create_client(srv, f'/dsr_controller2/{kind}/get_current_{kind}')
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'제어기 {kind} 확인 서비스가 없다 — 확인하지 않는다')
                continue
            r = self.exe.wait_future(cli.call_async(srv.Request()), 3.0)
            got = r.info if r is not None and r.success else None
            if not got:
                self.get_logger().warn(f'제어기 활성 {kind} 이름을 못 받았다 (에뮬레이터면 정상) — {kind}.json {want[kind]}')
            elif got != want[kind]:
                self.controller_error = f'제어기 활성 {kind} "{got}" != {kind}.json "{want[kind]}"'
                self.get_logger().error(f'{self.controller_error} — 펜던트에서 맞춘 뒤 다시 켠다. 목표를 받지 않는다')
            else:
                self.get_logger().info(f'제어기 활성 {kind}: {got} (config/{kind}.json 과 같음)')

    def _on_move_to(self, req, res):
        """관측(observe)·홈(home) 자세로 간다. 다 간 뒤 답한다. 자세가 robot.yaml 에 없으면 실패."""
        pose = self.cfg.get(f'{req.target}_pose')
        if req.target not in ('observe', 'home') or not pose:
            res.success, res.reason = False, 'PLAN_FAILED'
            return res
        if self.locked:
            res.success, res.reason = False, self.halt_reason or 'STOPPED'
            return res
        if self.controller_error:
            res.success, res.reason = False, 'TCP_MISMATCH'
            return res
        if not self.busy.acquire(blocking=False):
            res.success, res.reason = False, 'BUSY'
            return res
        try:
            ok, why = self.go_free([math.radians(v) for v in pose['joints_deg']], self._halted, keep_down=False)
            res.success, res.reason = ok, why
            return res
        except Exception as e:            # 예상 못 한 오류: 먼저 세운다
            self.get_logger().error(f'move_to {req.target}: 오류 {e!r}')
            self.stopper.stop('ERROR')
            res.success, res.reason = False, 'ERROR'
            return res
        finally:
            self.busy.release()


def main():
    """집기·놓기 노드를 켠다. executor 는 다른 스레드에서 돌리고, 이 스레드는 Ctrl+C 를 기다렸다가 먼저 로봇을 세운다."""
    init_ros()
    node = PickPlaceNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        if not node.exe.wait_ready():
            node.get_logger().error('MoveIt 이 준비되지 않았다. 브링업(real_moveit.launch.py)을 먼저 띄운다')
            return
        node.stopper.prepare()
        node.check_controller()
        node.get_logger().info('[집기·놓기] 준비 끝')
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.halt_reason = 'CTRL_C'          # 실행 중인 목표가 다음 궤적을 보내지 않게 먼저 막는다
        node.stopper.stop('CTRL_C')
    finally:
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

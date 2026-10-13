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
from d2_motion.motion_math import (CENTER_PREFIX, GRASP_AXIS, TILT, TILT_AS, TILT_LESS_DEG, TIP, TIP_GRASP, TIP_LIFT_M, flip_about_z,
                                   held_finger_angle, matrix_from_quat, move_speed_scale, open_finger_angle, pick_open_width, pick_place_tcp,
                                   scan_structure_box, tcp_target, tilt_pick_place_tcp, tip_place_tcp, up_axis)
from d2_safety.safe_stop import SafeStop, init_ros

PLANNERS = ('BiTRRT', 'RRTConnect')    # BiTRRT 가 먼저 (관절을 덜 돌리는 길), 실패하면 RRTConnect. 경로는 MoveIt 계획 그대로 쓴다
LINE_SCALE = 1.0                       # 직선도 자유 이동과 같은 속도(10/12 박진용: 코드 안 속도는 모두 같게). 가상에서 공급 영역 728 · p1 12 ·
                                       # 레시피 4개 놓기 47 · 세우기 48 직선을 0.15 · 0.2 · 0.25 · 0.3 으로 계획해 실패가 모두 같았다(속도 탓 실패 0)
GRIPPER_TIMEOUT_S = 10.0
# 그리퍼 기본 상태 = 활짝(10/11 박진용): 관측 · 촬영 때 좁은 손가락이 손목 카메라 화면에 찍힌다(W134 · B34).
# RG2 최대 열림 — 그리퍼 노드가 최대 명령으로 잘라 보낸다. 집는 폭은 지금처럼 집을 블록 바로 위에서 맞춘다
OPEN_FULL_M = 0.110
# run_recipe --grasp-test 의 part_id 앞글자 — 레시피에 없어 장면에 붙일 수 없지만 집은 자리로 되돌아오는 시험이라 그냥 진행한다
TEST_BLOCK_PREFIX = 'GRASP_P'
# 수직 직선 충돌 검사 간격 (관절 rad). Pilz 점은 0.1 s 마다라 0.15 m/s 에서 약 15 mm 씩 벌어져 블록 두께(14.8 mm)를 건너뛸 수 있다.
# 팔 끝까지 약 0.9 m 이므로 관절 0.005 rad 는 TCP 4.5 mm 이하 (10/6 한세교 교차 검증)
LINE_CHECK_RAD = 0.005
TIP_MAX_TURN_DEG = 150.0    # 눕히기 이동 한 번에 어느 관절이든 이보다 많이 돌면 그 손목 방향은 안 쓴다(one_pick STAND_MAX_TURN — 10/13 가상 212° 해로 길 없음)
TIP_SETTLE_S = 0.5          # 눕히기: 손을 연 뒤 블록이 넘어지는 동안 기다린다
SAFETY_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=1)   # 정지 노드와 같게 (IRD 4.1, S-26)


def load_config(name):
    """d2_bringup 의 config/<name> (robot.yaml · tcp.json · tool.json) 을 읽는다. 없으면 예외 (설정 없이 로봇을 움직이지 않는다)."""
    with open(os.path.join(get_package_share_directory('d2_bringup'), 'config', name)) as f:
        return yaml.safe_load(f) if name.endswith('.yaml') else json.load(f)


def grip_failure(r):
    """그리퍼 명령 결과 -> 실패 이유 (성공이면 None).

    답이 없으면(서비스 없음 · 시간 안에 답 없음) GRIPPER_NO_RESPONSE, 그리퍼 노드가 실패로 답하면 그 이유
    (GRIPPER_NO_RESPONSE · NO_FEEDBACK)를 그대로 넘긴다 — GRASP_FAILED 로 바꾸면 작업 관리자가 고장 난 그리퍼로
    다시 집으러 간다. 두 코드는 작업 관리자가 재시도 없이 ERROR 로 받아 사람을 부른다(E-74, IRD 7장).
    """
    if r is None:
        return 'GRIPPER_NO_RESPONSE'
    if not r.success:
        return r.reason or 'GRIPPER_NO_RESPONSE'
    return None


class PickPlaceNode(Node):
    """블록 1개 집기·놓기 액션 서버 + 관측·홈 자세 이동 서비스. MoveIt2 실행기는 같은 프로그램 안 클래스(MoveItExecutor).

    받는 것: /d2/motion/pick_place (액션 PickPlace), /d2/motion/move_to (MoveTo),
             /d2/safety/state (JSON safety_state/1 — stopped 면 지금 동작을 세우고 목표를 CANCELED 로 끝냄,
             locked 동안 새 목표를 STOPPED 로 거절. S-26)
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
        """멈춰야 하면 'CANCELED' (정지 노드의 stopped · Ctrl+C · 목표 취소), 아니면 None.

        정지 이유(STOP_WEB · ROBOT_ALARM:… 등)는 결과에 넣지 않는다 — 작업 관리자는 STOPPED · CANCELED 만 정지로 알아듣고
        나머지는 ERROR 로 처리한다(IRD 7장, E-62). 자세한 이유는 /d2/safety/state 의 reason 과 이 노드 로그에 있다.
        """
        if self.halt_reason or (gh is not None and gh.is_cancel_requested):
            return 'CANCELED'
        return None

    # ---------- 동작 단위 ----------
    def plan_free(self, goal, keep_down=True, skip_slot=None, scale=None, obstacles=None, structure=None, **start):
        """관절 목표 goal 까지 공중 이동 경로를 계획만 한다. 반환: 궤적 (못 찾으면 None).

        skip_slot = 장애물로 넣지 않을 공급 칸 (집으러 가는 칸, 방금 집은 칸). scale = 속도 비율(없으면 robot.yaml speed_scale).
        obstacles = 흩뿌림의 주변 블록 자세 목록(PickPlace.obstacles) — 주면 공급 칸 가정 상자 대신 이것을 피한다(E-53 ②).
        structure = 스캔 이동의 구조물 상자(motion_math.scan_structure_box) — 사람이 쌓아 장면에 없는 구조물을 피한다(W186).
        start = 출발 상태(start_q · finger · held — executor.plan). 없으면 지금 자세에서.
        """
        # 놓인 블록에서 transit_clearance_m 띄운 길을 먼저 찾고, 없으면 절반 여유로 한 번 더 (그래도 없으면 안 움직인다)
        clear = self.cfg['motion']['transit_clearance_m']
        for pad in (clear, clear / 2):
            for planner in PLANNERS:
                jt, err = self.exe.plan(goal, keep_down, planner, pad_m=pad, skip_slot=skip_slot, scale=scale,
                                        obstacles=obstacles, structure=structure, **start)
                if jt is not None:
                    return jt
                self.get_logger().warn(f'길 찾기 실패 ({planner}, 여유 {pad * 1000:.0f} mm): {err}')
        return None

    def down_level(self, *qs):
        """관절값들이 모두 '공구 아래' 제약 안이면 True(평소 15°), 아니면 모두 공구 최대 기울기(robot.yaml motion.tool_tilt_max_deg) 안이면
        그 각(숫자), 그것도 아니면 False(제약 없음). 실기에서 공구가 30 ~ 35° 넘게 기울면 안전 모드가 걸려(10/12) 기울여 집기 앞뒤 이동도 묶는다."""
        if all(self.exe.valid(q, self.exe.down_constraint())[0] for q in qs):
            return True
        tmax = self.cfg['motion'].get('tool_tilt_max_deg')
        if tmax is not None and all(self.exe.valid(q, self.exe.down_constraint(tmax))[0] for q in qs):
            return float(tmax)
        return False

    def go_free(self, goal, halted, keep_down=True, skip_slot=None, scale=None, obstacles=None, structure=None):
        """지금 자세 -> 관절 목표 goal 로 MoveIt 경로를 찾아 간다(plan_free). 반환: (성공, 실패 이유).

        길이 안 나오면 먼저 수직으로 approach_m 만큼 올라간 뒤(rise) 한 번 더 찾는다 — 놓는 중 멈춰 손가락이 놓인 블록 가까이(여유 상자 안)
        있으면 출발 자세부터 충돌로 막힌다(10/11 실기: 조립 자리 위 낮은 곳에서 관측 · 홈 이동이 계속 PLAN_FAILED).
        """
        for attempt in range(2):
            # 출발 자세가 이미 '공구 아래'를 어기면(예: 관절 0° 로 선 자세) 제약을 걸면 출발부터 실패한다 -> 이번 이동만 제약 없이
            down = keep_down and self.down_level(self.exe.current())
            if keep_down and down is not True:
                self.get_logger().info(f'출발 자세가 공구 아래가 아니라 공구 기울기 {down:.0f}° 안으로 계획한다' if down
                                       else '출발 자세가 공구 최대 기울기도 넘어 자세 제약 없이 계획한다')
            jt = self.plan_free(goal, down, skip_slot, scale, obstacles, structure)
            if jt is not None:
                return self.exe.execute(jt, halted)
            if attempt == 0:
                ok, why = self.rise(halted)
                if not ok:
                    return False, why
        return False, 'PLAN_FAILED'

    def rise(self, halted):
        """지금 손끝 방향 그대로 수직으로 approach_m(내려올 때와 같은 높이) 올라간다(Pilz LIN — 여유 상자 없이 실제 장면으로 촘촘히 충돌 검사). 반환: (성공, 이유).

        바깥 영향: 팔이 움직인다. 직선이 안 풀리거나 부딪히는 자세가 있으면 움직이지 않고 PLAN_FAILED.
        """
        pose = self.exe.fk(self.exe.current())
        if pose is None:
            return False, 'PLAN_FAILED'
        p, o = pose.position, pose.orientation
        up = self.cfg['motion']['approach_m']
        jt, err = self.exe.plan_line((p.x, p.y, p.z + up), (o.x, o.y, o.z, o.w), self.cfg['speed_scale'] * LINE_SCALE)
        if jt is None:
            self.get_logger().error(f'수직으로 올라가는 직선도 안 된다: {err}')
            return False, 'PLAN_FAILED'
        if not self.line_clear(jt, 'rise'):
            return False, 'PLAN_FAILED'
        self.get_logger().warn(f'출발 자세에서 길이 없어 먼저 수직으로 {up * 1000:.0f} mm 올라간다')
        return self.exe.execute(jt, halted)

    def routes(self, p, part_id, grasp, open_pick, skip_slot, obstacles):
        """블록 1개의 길을 움직이기 전에 전부 계획 · 검사한다. 반환: 궤적 dict, 하나라도 안 되면 None.

        approach(지금 → 집기 위) · pick_down(집기 위 → 집는 높이, 연 손가락으로 충돌 검사) · lift(pick_down 을 거꾸로)
        · carry(쥔 블록 · 문 손가락으로 집기 위 → 놓기 위) · place_down(쥔 블록으로 충돌 검사) · retreat(place_down 을 거꾸로).
        잡은 뒤에는 이 궤적을 실행만 한다 — 다시 계획하지 않으니 잡힘을 확인한 뒤 '길 없음'으로 서지 않는다(10/11 박진용:
        잡았으면 들고 옮기고 놓기는 그냥 가면 된다). 올라오기는 내려간 길을 되짚으므로 쥔 블록 모양이 실제와 조금 달라도 막히지 않는다.
        바깥 영향: 계획만 한다(로봇 · 그리퍼 · 장면 그대로).
        """
        pk, pl = p['pick'], p['place']
        held_angle = held_finger_angle(self.cfg, grasp)
        # 출발 자세가 이미 '공구 아래'를 어기면(예: 관절 0° 로 선 자세) 제약을 걸면 출발부터 실패한다 -> 접근만 제약 없이.
        # (제약을 늘 빼면 플래너가 관절을 크게 돌리는 길을 내 가상 제어기가 못 따라갔다 — 10/12 가상 관절 차이 137°)
        # 기울여 집기(TILT)는 도착(집기 위) 자세가 이미 기울어 제약을 어긴다 — 출발 · 도착이 둘 다 제약 안일 때만 건다
        keep_down = self.down_level(self.exe.current(), pk['qa'])
        r = {'approach': self.plan_free(pk['qa'], keep_down, skip_slot, None, obstacles)}
        if r['approach'] is None:
            self.get_logger().error('집기 위로 가는 길이 없다 — 움직이지 않는다')
            return None
        # 다시 집기(받침 · 빈 곳에 놓았던 블록)면 장면에 그 블록 자신이 놓은 자세(blk_<part_id>)로 남아 있다 — 실제로 잰 자세가 조금만 달라도
        # 손가락이 '자기 자신'과 닿는다고 막힌다(10/12 가상: 받침 위 블록 방향이 놓은 방향과 달라 pick 수직 경로 충돌). 그 닿음은 뺀다
        if not self.line_clear(pk['down'], 'pick', finger=open_finger_angle(self.cfg, open_pick), ignore=f'blk_{part_id}'):
            return None
        # 들기: 보통은 내려간 직선을 되짚는다. 눕히기(TIP)는 TIP_LIFT_M 만 드는 직선(_tip_targets)
        r['pick_down'], r['lift'] = pk['down'], pk.get('lift') or self.exe.reversed_path(pk['down'])
        lift_q = pk.get('lift_q', pk['qa'])
        # 접근 때는 손가락이 넓게 벌어져 옆 블록 여유 상자 바깥을 지나가지만, 쥔 뒤에는 손가락이 닫히고 블록이 붙어
        # 여유 상자와 겹칠 수 있다 — 10/11 가상: 접근 · 집기는 됐는데 옮기기 계획이 출발 자세 충돌로 막혔다
        # 기울여 집기(TILT)는 집기 위 자세에서 공구가 이미 15° 넘게 기울어 있어 '공구 아래' 제약을 걸면 출발부터 실패한다
        # (10/12 가상: 20° 블록 → 쥐고 옮기는 길 0.5 s 만에 PLAN_FAILED) — 출발이 제약을 어기면 옮기기만 제약 없이
        carry_down = self.down_level(lift_q, pl['qa'])
        # 다시 집는 블록 자신(blk_<part_id>)에는 여유 상자를 씌우지 않는다 — 세운 블록(75 mm)을 STAND 로 쥐고 approach_m 들면
        # 쥔 블록 바닥과 놓인 자기 자신 윗면 사이가 5 mm 라 여유 10 · 20 mm 상자와 출발부터 겹쳤다(10/12 가상 p1 세운 뒤 다시 집기)
        r['carry'] = self.plan_free(pl['qa'], carry_down, skip_slot, None, obstacles,
                                    start_q=lift_q, finger=held_angle, held=p['held'], skip_block=f'blk_{part_id}')
        if r['carry'] is None:
            self.get_logger().error('쥐고 옮기는 길이 없다(집기 위에서 쥔 블록 · 손가락이 주변과 겹침 등) — 움직이지 않는다')
            return None
        # 같은 이름의 놓인 블록(blk_<part_id>)은 잡으면서 장면 관리가 지운다(다시 집기) — 미리 볼 때도 그것과의 닿음은 뺀다
        if not self.line_clear(pl['down'], 'place', finger=held_angle, held=p['held'], ignore=f'blk_{part_id}'):
            return None
        r['place_down'], r['retreat'] = pl['down'], self.exe.reversed_path(pl['down'])
        return r

    def line_clear(self, jt, kind, finger=None, held=None, ignore=None):
        """수직 직선 궤적 jt 를 촘촘히 나눠(dense) 손가락 각 finger · 쥔 블록 held 로 부딪히는 자세가 없는지. 반환: 괜찮으면 True.

        ignore = 이 물체 이름이 낀 닿음은 셈하지 않는다(실행 전에 장면에서 지워질 물체).
        """
        for k, q in enumerate(self.dense(self.exe.positions(jt))):
            ok, raw = self.exe.valid(q, finger=finger, held=held)
            hits = [h for h in raw if ignore not in h.split('|')]
            if not ok and (hits or not raw):         # 닿음 목록 없이 안 되는 것(관절 범위 · 응답 없음)은 그대로 실패
                self.get_logger().error(f'{kind} 수직 경로 충돌 (검사 {k}번째 자세): {", ".join(hits)} — 움직이지 않는다')
                return False
        return True

    @staticmethod
    def dense(path):
        """관절 경로(rad 목록) -> 이웃 자세 사이를 LINE_CHECK_RAD 이하로 나눈 자세 목록. 처음·끝 자세를 꼭 넣는다 (끝 = 제일 낮은 집기·놓기 자세)."""
        out = [path[0]]
        for a, b in zip(path, path[1:]):
            n = max(1, math.ceil(max(abs(y - x) for x, y in zip(a, b)) / LINE_CHECK_RAD))
            out += [[x + (y - x) * i / n for x, y in zip(a, b)] for i in range(1, n + 1)]
        return out

    def grip(self, width_m):
        """그리퍼 노드에 폭 width_m 로 움직이라고 하고 다 움직일 때까지 기다린다. 반환: 결과 (응답 없으면 None — grip_failure 로 이유를 낸다)."""
        if not self.grip_cli.wait_for_service(timeout_sec=2.0):
            return None
        req = GripperCommand.Request(width_m=float(width_m), force_n=float(self.cfg['gripper']['force_n']))
        return self.exe.wait_future(self.grip_cli.call_async(req), GRIPPER_TIMEOUT_S)

    def scene_attach(self, part_id, attach, held=None, placed=None):
        """장면 관리 노드에 쥔 블록 붙이기(attach=True)·떼기를 부탁한다. 장면은 그 노드만 고친다. 반환: 성공하면 True.

        held = 붙이기 때 쥔 블록 상자 {'size_m', 'offset_m'[, 'quat']}(TCP 기준 m, pick_place_tcp 가 실제 집은 자세로 계산 — 'quat' 는
        TCP 기준 상자 방향 xyzw, 공구를 기울여 잡아 블록 축이 TCP 축과 어긋날 때만 · 없으면 TCP 축과 나란함, 10/13 PL held_quat),
        placed = 떼기 때 실제로 놓은 블록 (중심 m, 쿼터니언 xyzw)(base_link). 둘 다 보내 장면 관리가 레시피 없이도 그린다
        (AI · 스캔 설계, 흩뿌림, 다시 집기 — 10/9 PL E-76). None 이면 칸을 비워 장면 관리가 레시피로 계산한다.
        """
        if not self.attach_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('장면 관리 노드가 없다 — 쥔 블록 없이 계획한다')
            return False
        req = SceneAttach.Request(part_id=part_id, attach=attach)
        if held is not None:
            req.held_size_m, req.held_offset_m = [float(v) for v in held['size_m']], [float(v) for v in held['offset_m']]
            if held.get('quat') is not None:
                req.held_quat = [float(v) for v in held['quat']]
        if placed is not None:
            req.placed_center_m, req.placed_quat = [float(v) for v in placed[0]], [float(v) for v in placed[1]]
        res = self.exe.wait_future(self.attach_cli.call_async(req), 5.0)
        return bool(res and res.success)

    # ---------- 블록 1개 ----------
    def _targets(self, g):
        """목표의 블록 자세 -> 집기·놓기 TCP 목표와 IK. 반환: dict, 풀리지 않으면 None."""
        def pose(p):
            """geometry_msgs/Pose -> (위치 m 튜플, 3x3 회전 행렬)."""
            return ((p.position.x, p.position.y, p.position.z),
                    matrix_from_quat((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)))
        (pc, pr), (lc, lr) = pose(g.pick_pose), pose(g.place_pose)
        if g.grasp == TILT:
            return self._tilt_targets(g, pc, pr, lc)
        if g.grasp == TIP:
            return self._tip_targets(g, pc, pr, lc)
        slot = int(g.supply_slot) if g.supply_slot.isdigit() else None
        try:
            pick_xyz, pick_q, place_xyz, place_q, held = pick_place_tcp(self.cfg, pc, pr, lc, lr, g.grasp, slot)
        except (ValueError, KeyError) as e:
            self.get_logger().error(f'{g.part_id}: 목표 계산 실패 — {e}')
            return None
        m = self.cfg['motion']
        # IK 는 홈 자세에서 출발해 푼다: 지금 자세(예: 관절 0° 로 선 자세)에서 풀면 팔을 뒤로 꺾은 해(관절 1 이 154°)가
        # 나와 놓는 자리까지 길을 못 찾은 적이 있다 (10/6 가상). 놓기는 집기 해에서 출발한다 (원본 run_targets 와 같음)
        # 다만 지금 공구가 아래를 향한 자세(관측 · 촬영 자세 등)면 지금 관절값에서 푼다 — 대칭 손목 방향 둘 중 덜 도는 쪽을 골라야
        # '공구 아래' 제약 안 접근 길이 나온다(10/12 실기 · 가상: 받침 위 촬영 자리(손목 90°)에서 홈 기준 해로 가려다 40 s 뒤 PLAN_FAILED)
        now = self.exe.current()
        down_now = self.exe.valid(now, self.exe.down_constraint())[0]
        out, seed = {}, (now if down_now else [math.radians(v) for v in self.cfg['home_pose']['joints_deg']])
        for kind, xyz, q, up in (('pick', pick_xyz, pick_q, m['pick_up_m']), ('place', place_xyz, place_q, m['place_up_m'])):
            low = (xyz[0], xyz[1], xyz[2] + up)
            high = (low[0], low[1], low[2] + m['approach_m'])
            qa, quat = self.exe.solve(high, q, seed)
            # 손가락 쌍 가운데가 rg2_tcp 에서 닫힘 축으로 비켜 있다(robot.yaml gripper.finger_center_close_m) — IK 가 고른 손목 방향(quat, 대칭 둘 중 하나)
            # 기준으로 TCP 를 그만큼 반대로 옮겨 손가락 가운데가 목표에 오게 하고, 같은 손목 방향으로 다시 푼다(10/13 실기: 넓은 폭이 한쪽 끝면에 붙음)
            fc = self.cfg['gripper'].get('finger_center_close_m') or 0.0
            if qa is not None and fc:
                ycol = [r[1] for r in matrix_from_quat(quat)]
                low = tuple(v - fc * c for v, c in zip(low, ycol))
                high = tuple(v - fc * c for v, c in zip(high, ycol))
                qb = self.exe.ik(high, quat, qa, avoid=True, timeout=0.2)
                qa = qb if qb is not None and self.exe.singular(qb) is None else None
            # 움직이기 전에 위 자세에서 수직으로 내려가는 직선이 풀리는지 미리 본다 (Pilz LIN, 위 자세에서 출발)
            # 놓기는 살살 — robot.yaml motion.place_speed_ratio(10/12 박진용: 조립할 때 절반 속도),
            # 집기도 느리게 — motion.pick_speed_ratio(10/13 박진용: 집기 하강 너무 빠름, 0.3 → 0.2). 올라오는 직선은 같은 궤적이라 같이 느려진다
            ratio = m.get('place_speed_ratio', 1.0) if kind == 'place' else m.get('pick_speed_ratio', 1.0)
            down = None if qa is None else self.exe.plan_line(low, quat, self.cfg['speed_scale'] * LINE_SCALE * ratio, start_q=qa)[0]
            if down is None:
                self.get_logger().error(f'{g.part_id}: {kind} IK·직선 실패 ({", ".join(f"{v:.3f}" for v in high)})')
                return None
            out[kind] = {'qa': qa, 'quat': quat, 'high': high, 'low': low, 'down': down}
            seed = qa
        # 장면 관리에 넘길 값(E-76): 쥔 블록 상자(실제 집는 칸 · 잡기 기준)와 놓을 블록 자세(목표 그대로)
        o = g.place_pose.orientation
        out['held'], out['placed'] = held, (lc, (o.x, o.y, o.z, o.w))
        return out

    def _tilt_line(self, xyz, quat, scale, start_q):
        """기울여 집기의 직선 하나 — 속도 scale 로 안 풀리면 그 직선만 절반 속도로 한 번 더(10/13 박진용).
        기울인 공구로 내려가는 직선은 0.3 에서 3번 관절 가속 한계(3.14 rad/s²)를 넘어 못 만들 때가 있다. 반환: 궤적 또는 None."""
        jt = self.exe.plan_line(xyz, quat, scale, start_q=start_q)[0]
        if jt is None:
            jt = self.exe.plan_line(xyz, quat, scale * 0.5, start_q=start_q)[0]
            if jt is not None:
                self.get_logger().info(f'기울여 집기 직선: 속도 {scale:.2f} 로 안 돼 {scale * 0.5:.3f} 로 만듦')
        return jt

    def _tilt_targets(self, g, pc, pr, lc):
        """잡기 TILT(기울어진 블록, 챌린지 A1 · W180): pick_pose 회전의 두께 축 = 윗면 법선으로 보고 공구를 기울여 문 뒤 place_pose 에 눕힌다.

        블록 기울기 그대로(less 0)부터 손이 안 닿으면 TILT_LESS_DEG 만큼씩 덜 기울여, 집기는 공구 축을 따라 · 놓기는 수직으로 내려가는
        직선이 풀리는 첫 조합을 고른다(시험 도구 one_pick tilt_move 와 같은 순서). 반환: _targets 와 같은 dict, 하나도 안 되면 None.
        바깥 영향: 계획만 한다.
        """
        m, scale = self.cfg['motion'], self.cfg['speed_scale'] * LINE_SCALE
        home = [math.radians(v) for v in self.cfg['home_pose']['joints_deg']]
        o = g.place_pose.orientation
        lr = matrix_from_quat((o.x, o.y, o.z, o.w))
        for less in TILT_LESS_DEG:
            try:
                t = tilt_pick_place_tcp(self.cfg, pc, pr, lc, lr, less)
            except (ValueError, KeyError) as e:
                self.get_logger().error(f'{g.part_id}: 기울여 집기 목표 계산 실패 — {e}')
                return None
            if t is None:
                break
            grips, place_xyz, place_q, held = t
            p_low = (place_xyz[0], place_xyz[1], place_xyz[2] + m['place_up_m'])
            p_high = (p_low[0], p_low[1], p_low[2] + m['approach_m'])
            for xyz, q, axis in grips:
                high = tuple(v + a * m['approach_m'] for v, a in zip(xyz, axis))
                qa, quat = self.exe.solve(high, q, home)
                down = None if qa is None else self._tilt_line(xyz, quat, scale * m.get('pick_speed_ratio', 1.0), qa)
                if down is None:
                    continue
                qb, quat_b = self.exe.solve(p_high, place_q, qa)
                pdown = None if qb is None else self._tilt_line(p_low, quat_b, scale * m.get('place_speed_ratio', 1.0), qb)
                if pdown is None:
                    continue
                if less:
                    self.get_logger().info(f'{g.part_id}: 블록보다 {less}° 덜 기울여 집는다(남은 기울기는 놓을 때 넘어가며 눕는다)')
                return {'pick': {'qa': qa, 'quat': quat, 'high': high, 'low': xyz, 'down': down},
                        'place': {'qa': qb, 'quat': quat_b, 'high': p_high, 'low': p_low, 'down': pdown},
                        'held': held, 'placed': (lc, (o.x, o.y, o.z, o.w))}
        self.get_logger().error(f'{g.part_id}: 기울여 집는 IK · 직선이 어느 기울기로도 안 풀림')
        return None

    def _ik_exact(self, xyz, quat, ref):
        """자세 그대로(손목 대칭 안 바꿈)의 충돌 없는 IK — ref · 홈 · extra_seeds 순으로 풀고 특이점 근처 해는 버린다. 없으면 None."""
        home = [math.radians(v) for v in self.cfg['home_pose']['joints_deg']]
        for seed in [ref, home] + self.exe.extra_seeds:
            sol = self.exe.ik(tuple(xyz), tuple(quat), seed, avoid=True, timeout=0.1)
            if sol is not None and self.exe.singular(sol) is None:
                return sol
        return None

    def _unwrap(self, q, ref):
        """관절마다 ±360° 같은 자세 중 ref 에 가장 가까운 값 — MoveIt 관절 한계(joint_limits.yaml) 안에서만(one_pick 과 같음)."""
        if not hasattr(self, '_limits'):
            path = os.path.join(get_package_share_directory('d2_bringup'), 'config', 'joint_limits.yaml')
            with open(path, encoding='utf-8') as f:
                jl = yaml.safe_load(f)['joint_limits']
            self._limits = [(jl[f'joint_{k}']['min_position'], jl[f'joint_{k}']['max_position']) for k in range(1, 7)]
        lim = self._limits
        return [min([v + k * 2 * math.pi for k in (-1, 0, 1) if lim[j][0] <= v + k * 2 * math.pi <= lim[j][1]] or [v],
                    key=lambda x: abs(x - r)) for j, (v, r) in enumerate(zip(q, ref))]

    def _tip_targets(self, g, pc, pr, lc):
        """잡기 TIP(받침 눕히기, 10/13 실기 — one_pick tip_plan 그대로): 받침 위 선 블록(pick_pose)을 TIP_GRASP 로 물어 TIP_LIFT_M 들고,
        닫는 축 둘레로 기울여 place_pose(누울 자리) 쪽에 가장 낮은 모서리를 받침 + place_up_m 에 내려놓는다(motion_math.tip_place_tcp).

        공구 대칭 두 방향을 다 보고 IK(특이점 아님) · 직선이 다 되고 관절이 TIP_MAX_TURN_DEG 넘게 돌지 않는 것 중 덜 도는 쪽.
        반환: _targets 와 같은 dict + pick['lift'] · pick['lift_q'](들기 직선 · 끝 관절값), 안 되면 None. 바깥 영향: 계획만 한다.
        """
        m, scale = self.cfg['motion'], self.cfg['speed_scale'] * LINE_SCALE
        now = self.exe.current()
        o = g.place_pose.orientation

        def turn(a, b):
            return max(abs(math.degrees(x - y)) for x, y in zip(a, b))
        best, why = None, '자세 없음'
        for flip in (False, True):
            try:
                t, why_t = tip_place_tcp(self.cfg, pc, pr, lc, flip)
            except (ValueError, KeyError) as e:
                self.get_logger().error(f'{g.part_id}: 눕히기 목표 계산 실패 — {e}')
                return None
            if t is None:
                why = why_t
                continue
            tcp0, q0, tcp1, q1 = t['tcp0'], t['q0'], t['tcp1'], t['q1']
            pre0 = (tcp0[0], tcp0[1], tcp0[2] + m['approach_m'])
            qa = self._ik_exact(pre0, q0, now)
            if qa is None:
                why = '집기 위 IK'
                continue
            qa = self._unwrap(qa, now)
            down = self.exe.plan_line(tcp0, q0, scale * m.get('pick_speed_ratio', 1.0), start_q=qa)[0]
            if down is None:
                why = '집기 내려가기 직선'
                continue
            lift = self.exe.plan_line((tcp0[0], tcp0[1], tcp0[2] + TIP_LIFT_M), q0, scale * m.get('pick_speed_ratio', 1.0),
                                      start_q=self.exe.positions(down)[-1])[0]
            if lift is None:
                why = '들기 직선'
                continue
            ql = self.exe.positions(lift)[-1]
            pre1 = (tcp1[0], tcp1[1], tcp1[2] + TIP_LIFT_M)
            qb = self._ik_exact(pre1, q1, ql)
            if qb is None:
                why = '기울여 놓기 위 IK'
                continue
            qb = self._unwrap(qb, ql)
            pdown = self._tilt_line(tcp1, q1, scale * m.get('place_speed_ratio', 1.0), qb)
            if pdown is None:
                why = '기울여 내려가기 직선'
                continue
            steps = [turn(now, qa), turn(ql, qb)]
            if max(steps) > TIP_MAX_TURN_DEG:
                why = f'관절 회전 {max(steps):.0f}°'
                continue
            if best is None or sum(steps) < best[0]:
                best = (sum(steps), {'pick': {'qa': qa, 'quat': q0, 'high': pre0, 'low': tcp0, 'down': down, 'lift': lift, 'lift_q': ql},
                                     'place': {'qa': qb, 'quat': q1, 'high': pre1, 'low': tcp1, 'down': pdown},
                                     'held': t['held'], 'placed': (lc, (o.x, o.y, o.z, o.w))})
                self.get_logger().info(f'{g.part_id}: 눕히기 {t["grasp"]} 로 물어 {t["phi_deg"]:+.0f}° 기울여 놓음 · 넘어지는 쪽 '
                                       f'({t["d"][0]:+.2f}, {t["d"][1]:+.2f}){" · 공구 180° 돌림" if flip else ""} · 관절 회전 '
                                       f'{steps[0]:.0f}° · {steps[1]:.0f}°')
        if best is None:
            self.get_logger().error(f'{g.part_id}: 눕히기 자세가 안 나옴({why})')
            return None
        return best[1]

    def _center_targets(self, g):
        """잡기 CENTER_<FLAT_SHORT|FLAT_LONG>(받침 맞추기, one_pick center_at_p1 그대로): 받침 위 눕힌 블록(pick_pose)을 그 폭으로 물었다
        바로 놓을 위 자세 · 내려가는 직선. 손가락이 닫히는 축은 쥘 때 손 가운데로 맞춰지므로, 레시피 잡기와 반대 폭으로 한 번 물면
        레시피 잡기가 맞춰 주지 않는 축이 잰 중심으로 온다(11:59 실기 W144: 닫히지 않는 축만 ±2 ~ 3 mm → 옆 위판을 밂).
        손목 대칭 두 방향을 다 풀어(지금 관절값에 가까운 ±360° 값으로) 지금 자세에서 관절이 덜 도는 쪽부터 — 10/13 실기 001 SEAT_001_01 ·
        004 BASE_001_01: IK 가 먼저 풀린 쪽만 써서 받침 촬영 자세에서 가는 길을 40 s 못 찾고(BiTRRT 시간 초과) 맞추기를 건너뜀.
        손가락 가운데 비킴(finger_center_close_m)은 집기와 같이 보정.
        반환: [{'grasp', 'qa', 'down', 'turn_deg'}](덜 도는 순) 또는 None. 바깥 영향: 계획만 한다."""
        gc = g.grasp[len(CENTER_PREFIX):]
        p, o = g.pick_pose.position, g.pick_pose.orientation
        try:
            xyz, qt, _ = tcp_target(self.cfg, (p.x, p.y, p.z), matrix_from_quat((o.x, o.y, o.z, o.w)), gc)
        except (ValueError, KeyError) as e:
            self.get_logger().error(f'{g.part_id}: 맞추기 목표 계산 실패 — {e}')
            return None
        m = self.cfg['motion']
        fc = self.cfg['gripper'].get('finger_center_close_m') or 0.0
        scale = self.cfg['speed_scale'] * LINE_SCALE * m.get('pick_speed_ratio', 1.0)
        now = self.exe.current()
        out = []
        for q in (qt, flip_about_z(qt)):
            ycol = [r[1] for r in matrix_from_quat(q)]
            low = tuple(v - fc * c for v, c in zip(xyz, ycol))
            qa = self._ik_exact((low[0], low[1], low[2] + m['approach_m']), q, now)
            if qa is None:
                continue
            qa = self._unwrap(qa, now)
            down = self.exe.plan_line(low, q, scale, start_q=qa)[0]
            if down is not None:
                out.append({'grasp': gc, 'qa': qa, 'down': down,
                            'turn_deg': max(abs(math.degrees(a - b)) for a, b in zip(qa, now))})
        if not out:
            self.get_logger().warn(f'{g.part_id}: 맞추기({gc}) IK · 직선이 안 나옴')
            return None
        return sorted(out, key=lambda c: c['turn_deg'])

    def _center(self, g, halted, step, finish):
        """받침 맞추기 실행: 위로 → 그 폭으로 열기 → 내려가기 → 닫기 → 열기 → 내려온 길로 올라오기 → 활짝. 블록은 들지 않는다.
        움직이기 전에 길이 안 나오면 PLAN_FAILED(팔 안 움직임 — 작업 관리자는 맞추기 없이 다시 집기로 간다). 반환: finish 결과."""
        cands = self._center_targets(g)
        if cands is None:
            return finish(False, 'PLAN_FAILED')
        open_pick = pick_open_width(self.cfg, cands[0]['grasp'], g.open_width_m)
        if open_pick is None:
            return finish(False, 'PLAN_FAILED')
        c, approach = None, None
        for cand in cands:                      # 덜 도는 손목 방향부터 — 길이 안 나오면 대칭 다른 쪽
            approach = self.plan_free(cand['qa'], self.down_level(self.exe.current(), cand['qa']), None, None, [])
            if approach is not None and self.line_clear(cand['down'], 'center', finger=open_finger_angle(self.cfg, open_pick),
                                                        ignore=f'blk_{g.part_id}'):
                c = cand
                break
            self.get_logger().warn(f'{g.part_id}: 맞추기 손목 방향(관절 최대 {cand["turn_deg"]:.0f}°) 길 없음')
        if c is None:
            return finish(False, 'PLAN_FAILED')
        step('approach')
        ok, why = self.exe.execute(approach, halted)
        if not ok:
            return finish(False, why)
        why = grip_failure(self.grip(open_pick))
        if why:
            return finish(False, why)
        ok, why = self.exe.execute(c['down'], halted)
        if not ok:
            return finish(False, why)
        step('grasp')
        r_grip = self.grip(self.cfg['grasp_close_m'][c['grasp']])
        why = grip_failure(r_grip)
        if why:
            return finish(False, why)
        width = r_grip.width_m
        self.get_logger().info(f'{g.part_id}: 맞추기 {c["grasp"]} 로 물었음 · 폭 {width * 1000:.1f} mm'
                               f'{"" if r_grip.grasped else " (잡힘 판정 아님 — 그대로 진행)"}')
        why = grip_failure(self.grip(open_pick))
        if why:
            return finish(False, why, width)
        step('retreat')
        ok, why = self.exe.execute(self.exe.reversed_path(c['down']), halted)
        if not ok:
            return finish(False, why, width)
        why = grip_failure(self.grip(OPEN_FULL_M))
        return finish(not why, why or '', width)

    def _execute(self, gh):
        """PickPlace 목표 1개: 집기 위 -> 집는 폭 -> 하강 -> 닫기·잡힘 확인 -> 상승 -> 놓기 위 -> 하강 -> 열기 -> 상승 -> 활짝.

        집기 전 여는 폭은 open_width_m(0 이면 robot.yaml grasp_open_pick_m, 블록 폭 이하면 움직이기 전에 PLAN_FAILED).
        흩뿌림(supply_slot 빈 값)은 공중 이동 때 obstacles 블록을 transit_clearance_m 부풀려 피한다(E-53 ②).
        움직이기 전에 길을 전부 계획 · 검사하고(routes), 하나라도 안 되면 아무것도 건드리지 않고 PLAN_FAILED.
        그 뒤로는 계획한 궤적을 실행만 한다 — 잡은 뒤 계획이 막혀 서는 일이 없다. 실행 중에는 정지 · 시간 초과만 멈춘다.
        그리퍼가 답이 없거나 폭을 못 읽으면 GRIPPER_NO_RESPONSE · NO_FEEDBACK 을 그대로 낸다(E-74 — 헛잡음 GRASP_FAILED 와 구분).
        헛잡음이면 다시 열고 내려온 길로 올라와 GRASP_FAILED(블록은 그 자리, 그리퍼는 연 채 — 부르는 쪽이 다시 보고 다시 집는다).
        공급 칸에서 블록 폭보다 더 닫혔으면(칸이 빔) 같은 동작 뒤 SLOT_EMPTY.
        잡은 뒤 장면에 쥔 블록을 못 붙이면(장면 관리 없음 · 장면 적용 실패) 그 자리에 다시 내려놓고 올라와 ERROR.
        쥔 상자 · 놓은 자리는 이 노드가 계산해 보내므로 레시피 파일에 없는 설계도 붙는다(E-76).
        grasp = TILT(기울어진 블록, W180)면 pick_pose 두께 축을 윗면 법선으로 보고 공구를 기울여 FLAT_SHORT 폭으로 문 뒤
        place_pose 에 눕힌다(_tilt_targets). 폭 · 닫기 · 놓기 값은 FLAT_SHORT 것.

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
            elif reason == 'CANCELED' and gh.is_cancel_requested:
                gh.canceled()                       # 취소 요청을 받은 목표만 canceled 로 끝낼 수 있다 (정지 노드가 세운 건 abort + CANCELED)
            else:
                gh.abort()
            why = f'{reason} ({self.halt_reason})' if reason in ('CANCELED', 'STOPPED') and self.halt_reason else reason
            self.get_logger().info(f'{g.part_id}: {"성공" if ok else "실패 " + why} ({res.duration_s:.1f} s)')
            return res

        if self.locked:
            return finish(False, 'STOPPED')          # 잠긴 동안 거절 (IRD 7장 — 정지 이유는 safety/state 에)
        if self.controller_error:
            return finish(False, 'TCP_MISMATCH')
        if self.holding:
            # 블록을 쥔 채 출발하면 다른 칸으로 들고 가거나, 집는 폭 '열기' 가 오히려 조이는 명령이 된다 (10/6 실기)
            self.get_logger().error(f'{g.part_id}: 그리퍼가 블록을 쥐고 있다 — 사람이 블록을 빼고 연 뒤 다시')
            return finish(False, 'ERROR')   # IRD 7장: 먼저 세움 → 사람 호출. 자세한 이유는 위 로그
        center = g.grasp.startswith(CENTER_PREFIX) and g.grasp[len(CENTER_PREFIX):] in ('FLAT_SHORT', 'FLAT_LONG')
        if g.grasp not in GRASP_AXIS and g.grasp not in (TILT, TIP) and not center:
            return finish(False, 'PLAN_FAILED')
        # 폭 · 닫기 · 놓기 값 이름 — TILT 는 FLAT_SHORT 폭, TIP 은 받침 위 블록 자세로 정한 잡기(옆으로 섬 EDGE_LONG · 섬 STAND_LONG)
        o = g.pick_pose.orientation
        up = up_axis(matrix_from_quat((o.x, o.y, o.z, o.w)))
        if g.grasp == TIP and up not in TIP_GRASP:
            self.get_logger().error(f'{g.part_id}: 눕히기(TIP)인데 pick_pose 가 누운 블록')
            return finish(False, 'PLAN_FAILED')
        gk = TILT_AS if g.grasp == TILT else TIP_GRASP[up] if g.grasp == TIP else g.grasp[len(CENTER_PREFIX):] if center else g.grasp
        open_pick = pick_open_width(self.cfg, gk, g.open_width_m)
        if open_pick is None:
            # 블록 폭 이하로 열면 손가락이 블록 위에 내려앉는다 — 움직이기 전에 거절
            self.get_logger().error(f'{g.part_id}: open_width_m {g.open_width_m * 1000:.1f} mm 가 블록 폭 이하')
            return finish(False, 'PLAN_FAILED')
        if not self.busy.acquire(blocking=False):
            return finish(False, 'BUSY')
        try:
            halted = lambda: self._halted(gh)       # noqa: E731

            def step(name):
                """중간 보고(step)를 보내고 로그에 남긴다."""
                gh.publish_feedback(PickPlace.Feedback(step=name))
                self.get_logger().info(f'{g.part_id}: {name}')

            if center:
                return self._center(g, halted, step, finish)
            p = self._targets(g)
            if p is None:
                return finish(False, 'PLAN_FAILED')
            slot = int(g.supply_slot) if g.supply_slot.isdigit() else None
            # 흩뿌림(supply_slot 빈 값)은 공급 칸 가정 상자 대신 주변 블록을 피한다(E-53 ② — 공중 이동 계획 때만, 수직 직선은 틈 기준 E-49)
            around = None if slot is not None else list(g.obstacles)
            width = float('nan')

            r = self.routes(p, g.part_id, gk, open_pick, slot, around)
            if r is None:
                return finish(False, 'PLAN_FAILED')
            step('approach')
            ok, why = self.exe.execute(r['approach'], halted)
            if not ok:
                return finish(False, why)
            # 집는 폭은 집을 블록 바로 위에서 맞춘다: 블록마다 폭이 달라 미리 맞출 수 없다
            why = grip_failure(self.grip(open_pick))
            if why:
                return finish(False, why)
            ok, why = self.exe.execute(r['pick_down'], halted)
            if not ok:
                return finish(False, why)
            step('grasp')
            r_grip = self.grip(self.cfg['grasp_close_m'][gk])
            why = grip_failure(r_grip)
            if why:
                return finish(False, why)
            width = r_grip.width_m
            expect, tol = self.cfg['grasp_width_m'][gk], self.cfg['gripper']['check_tolerance_m']
            # 폭을 모르면(가상 그리퍼) 그리퍼 노드의 잡힘만 믿는다. 알면 이 잡기의 블록 폭과 맞는지도 본다
            if not r_grip.grasped or (not math.isnan(width) and abs(width - expect) > tol):
                self.get_logger().error(f'{g.part_id}: 잡힘 실패 (폭 {width * 1000:.1f} mm, 기대 {expect * 1000:.1f}) — 열고 올라온다')
                # 손가락을 닫은 채 서 있으면 다음 이동 때 블록을 칠 수 있다 — 열고 내려온 길로 올라와 다시 볼 수 있게 한다
                self.grip(open_pick)
                ok, why = self.exe.execute(r['lift'], halted)
                # 공급 칸에서 블록 폭보다 더 닫혔으면 칸이 빈 것 — SLOT_EMPTY(IRD 7장: 작업 판단이 그 칸을 빔으로 적고 다음 칸으로)
                empty = slot is not None and not math.isnan(width) and width < expect - tol
                return finish(False, ('SLOT_EMPTY' if empty else 'GRASP_FAILED') if ok else why, width)
            # 쥔 블록이 장면에 안 붙으면 운반 중 충돌을 못 본다 → 들지 않는다.
            # 블록은 아직 바닥에 닿아 있으므로 그 자리에서 다시 열면 원래 자리에 그대로 남는다(공중 개방 아님)
            if not self.scene_attach(g.part_id, True, held=p['held']) and not g.part_id.startswith(TEST_BLOCK_PREFIX):
                self.get_logger().error(f'{g.part_id}: 장면에 쥔 블록을 못 붙였다 — 내려놓고 올라와 멈춘다')
                self.grip(open_pick)
                self.exe.execute(r['lift'], halted)
                return finish(False, 'ERROR', width)
            for name, key in (('lift', 'lift'), ('move', 'carry'), ('place', 'place_down')):
                step(name)
                ok, why = self.exe.execute(r[key], halted)
                if not ok:
                    return finish(False, why, width)
            why = grip_failure(self.grip(self.cfg['grasp_open_place_m'][gk]))
            if why:
                return finish(False, why, width)
            if not self.scene_attach(g.part_id, False, placed=p['placed']) and not g.part_id.startswith(TEST_BLOCK_PREFIX):
                # 떼기 실패는 장면에 블록이 남아 다음 계획이 더 조심스러워질 뿐이라 멈추지 않는다
                self.get_logger().warn(f'{g.part_id}: 장면에서 쥔 블록을 못 뗐다')
            if g.grasp == TIP:
                time.sleep(TIP_SETTLE_S)            # 기울여 놓은 블록이 넘어지는 동안 손가락을 그 자리에 둔다
            step('retreat')
            ok, why = self.exe.execute(r['retreat'], halted)
            if not ok:
                return finish(False, why, width)
            # 놓은 블록 위로 올라온 뒤 기본 상태(활짝)로 — 놓는 자리 옆 블록을 덜 치게 놓을 때는 좁게 연다
            why = grip_failure(self.grip(OPEN_FULL_M))
            return finish(not why, why or '', width)
        except Exception as e:            # 예상 못 한 오류: 궤적이 아직 돌고 있을 수 있으니 먼저 세운다
            self.get_logger().error(f'{g.part_id}: 오류 {e!r}')
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
        """robot.yaml 에 <target>_pose(joints_deg)가 있는 자세로 간다. 다 간 뒤 답한다(W186 — 고정 목록 대신 robot.yaml).

        도착한 뒤 블록을 쥐고 있지 않으면 그리퍼를 활짝 연다(기본 상태). 그리퍼가 답이 없으면 GRIPPER_NO_RESPONSE.

        자세 값은 robot.yaml <target>_pose. 값이 없거나(교시 전 · 모르는 이름) 관절 6개가 아니면 PLAN_FAILED.
        스캔 촬영 자세 사이를 오갈 때는 사람이 쌓은 구조물 자리 상자를 피해 계획한다(motion_math.scan_structure_box, W186).
        speed_ratio(0 = 평소, 0 < 값 ≤ 1 = 느리게, 다시 시작 뒤 첫 이동 0.5)를 speed_scale 에 곱한다. 범위 밖이면 PLAN_FAILED.
        observe_spare 만 '공구 아래' 경로 제약을 건다(출발 자세가 공구 아래일 때 — go_free).
        """
        pose = self.cfg.get(f'{req.target}_pose')
        scale = move_speed_scale(self.cfg, req.speed_ratio)
        joints = pose.get('joints_deg') if isinstance(pose, dict) else None
        if not isinstance(joints, list) or len(joints) != 6 or scale is None:
            self.get_logger().error(f'move_to {req.target}: 자세 없음 또는 speed_ratio {req.speed_ratio} 범위 밖')
            res.success, res.reason = False, 'PLAN_FAILED'
            return res
        if self.locked:
            res.success, res.reason = False, 'STOPPED'
            return res
        if self.controller_error:
            res.success, res.reason = False, 'TCP_MISMATCH'
            return res
        if not self.busy.acquire(blocking=False):
            res.success, res.reason = False, 'BUSY'
            return res
        try:
            box = scan_structure_box(self.cfg, req.target, [math.degrees(v) for v in self.exe.current()])
            if box is not None:
                self.get_logger().info(f'move_to {req.target}: 스캔 구조물 상자 {box[1]} m 를 피해 계획')
            # 받침 위 촬영 자세(observe_spare)는 공구 아래 자세라, 출발도 공구 아래면 '공구 아래' 제약을 걸고 계획한다(go_free 가 출발 자세를 본다) —
            # 제약 없이 찾으면 관절을 크게 돌리는 길이 나와 가상 제어기가 못 따라갔다(10/12 가상 TIMEOUT, one_pick look_spare 와 같음 · b1 대조 #15)
            ok, why = self.go_free([math.radians(v) for v in joints], self._halted, keep_down=req.target == 'observe_spare', scale=scale,
                                   structure=box)
            # 도착한 뒤 기본 상태(활짝)로 — 관측 · 촬영 사진에 손가락이 안 찍히게. 블록을 쥐고 있으면 떨어뜨리므로 열지 않는다
            if ok and not self.holding:
                why = grip_failure(self.grip(OPEN_FULL_M)) or ''
                ok = not why
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

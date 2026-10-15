# -*- coding: utf-8 -*-
"""작업 관리자 TaskManager — 자동 조립 상태표 · 실패 대응표 · 시작 점검 (ROS 없이 동작, SDD 5장 · 7.1).

task 는 로봇 PC 에서 돈다. 웹 화면 · 음성은 웹 PC 에 있고 다리(bridge)가 ROS 이름 그대로 대신 부르므로(E-26~E-28),
이 클래스는 MQTT 를 전혀 모른다. task 노드(task_node.py)가 콜백에서 on_* · command 를 부르고,
작업 스레드 하나가 run_once() 를 되풀이한다.
바깥 일(이동 · 관측 · 집기 · 방송)은 생성할 때 받은 io 객체로 한다. io 가 갖춰야 할 것(ROS 노드나 시험용 가짜):
  get_design(design_id, should_abort) -> (ok, reason, design)   설계 조회(/d2/hmi/get_design, design/4.0 dict). 제한 시간은 io 가 건다
  services_ready() -> [이름]                        아직 안 떠 있는 서버 이름들(없으면 빈 목록, 기다리지 않는다)
  move_to(target, should_abort, speed_ratio=1.0) -> (ok, reason)  복구 뒤 첫 이동만 비율 지정
  check_progress(part_ids, should_abort) -> (ok, reason, rows)   rows = {part_id: {state, dx_m, dy_m, dz_m, top_z_m}}
  pick_place(goal, should_abort, on_feedback, on_result) -> (ok, reason)    goal = TaskPlanner.next_block 의 FOUND dict. 피드백 step · 결과 수신 시각을 콜백으로 알린다
  request_stop(reason) -> (ok, message)             정지 노드에 먼저 세우라고 요청(응답은 접수 여부)
  publish_state(dict) · publish_progress(dict)
move_to · check_progress · pick_place 는 io 가 robot.yaml 의 제한 시간(service_s 3 · move_to_s 45 · pick_place_s 120 — 10/13 W188)을 건다.
시간 초과 때 request_stop(reason) 으로 정지 노드에 먼저 요청하고, stopped 신호를 확인하기 전에는 복구하지 않는다. 기다리는 동안
should_abort() 가 참이 되면(정지 신호 · Ctrl+C) 곧바로 빠져나와 (False, 'STOPPED') 로 돌려주고, 진행 중인 pick_place 목표는 취소한다.

범위: 자동 조립 상태 11개 + 기록(CSV · run_id, W065 — RunLogger), 설계 조회 · 결과 저장 · 시간 제한(W119 일부).
웹 끊김은 WAIT_HMI 로 기다린다(서기 궤적 아님). 스캔은 촬영 자세 이동 → 점군 수집 → 추론 → 검토 순서다.
run_id 는 조립 출발(READY → CHECK) 또는 스캔 시작(SCAN_MOVE) 때 새로 만든다. 조립 정지 · 복구 · 공급 보충은 같은 ID 를 쓴다.
끝난 조립의 build/2 요약은 last_build(최근 결과 표시용)와 pending_builds(run_id 별 미전송 요약)에 둔다.
/d2/hmi/save_build 로 보내는 일은 task 노드가 하고(네트워크 대기는 이 클래스의 잠금 밖), 이 클래스는 무엇을 보낼지(builds_to_send) ·
답이 오면 어떻게 할지(build_result)만 정한다. 저장이 확인된 run_id 만 목록에서 지운다 — 실패 · 시간 초과 · 잘못된 답은 요약을 남긴다.
WAIT_SUPPLY(공급 채우기 · [계속])는 05 작업분류 W119 ②의 일부지만 SDD 5장 상태표에 있어 W044 에서 먼저 구현했다 — **W119 완료가 아니다**
(W119 는 스캔 흐름이 더 남아 있다).
복구(RECOVER) 뒤 첫 성공한 move_to 는 robot.yaml recover.speed_ratio(0.5)로 움직인다. 조립 중이 아닐 때(IDLE · READY · DONE)
정지됐다 풀린 뒤 돌아갈 곳은 SDD 에
없어서 가장 가까운 규칙(비조립 RECOVER → IDLE)만 따른다(IRD 12장 W121 다시 확인 대상).
"""
import copy
import json
import logging
import math
import threading
import time

from d2_task.block_picker import BlockPicker
from d2_task.recipe_document import PLACEMENTS_SCHEMA, RECIPE_SCHEMA, same_major    # 형식은 정본(10/14 점검 ③-5)
from d2_task.recipe_to_parts import ORIS
from d2_task.run_logger import RunLogger, _reject_constant
from d2_task.scatter_pick import REOBSERVE, REPLAN_SAME, ScatterFlow, StepTracker, block_pose, contact_phase, grasped_now
from d2_motion.motion_math import matrix_from_quat
from d2_task.task_planner import TaskPlanner

PLAN_STREAK = 3                            # 받침 경유: 움직이기 전 길 계산 거절이 연달아 이만큼이면 사람(10/12 실기 — 장면에 남은 블록으로 20번 헛돎)

DESIGN_SCHEMA = 'design/4.0'

GRIPPER_POLL_S = 0.05                          # gripper 상태 대기 간격(s) — 정지가 끼면 이 안에 알아본다
LOG = logging.getLogger('d2_task')

# 조립이 진행 중인 상태 — 정지 · 실패 뒤 RECOVER 에서 CHECK 로 이어 간다
RUN_STATES = ('CHECK', 'SELECT', 'PICK_PLACE', 'WAIT_SUPPLY', 'WAIT_HMI', 'VERIFY', 'RECOVER', 'ERROR')
# 로봇이 서야 하는 결과 — 정지 노드가 세웠다(STOPPED · CANCELED). 그리퍼 응답 없음(GRIPPER_NO_RESPONSE) · 피드백 없음(NO_FEEDBACK)은
# 정지 노드가 잠그지 않으니 신호를 기다리지 않고 작업 관리자가 ERROR 로 멈춘다(E-74, 10/9 PL)
STOP_REASONS = ('STOPPED', 'CANCELED')
GRIPPER_OPEN_M = 0.0865   # 정지 중 그리퍼 열기 폭(m) — 복구 절차 4절 2 명령과 같은 값(10/13 16:30 PL). 쥔 블록(최대 75 mm)을 뺄 수 있게 연다
# 시작 점검 화면 글의 노드 이름표 — 서비스 경로는 로그에만 남기고 화면에는 사람이 알아볼 이름으로(10/14 점검 ③ 안내 정리)
SERVICE_LABEL = {'/d2/motion/move_to': '로봇 이동 · 집기 기능', '/d2/motion/pick_place': '로봇 이동 · 집기 기능',
                 '/d2/vision/check_progress': '손목 카메라 기능', '/d2/vision/scan_capture': '손목 카메라 기능',
                 '/d2/vision/scan_infer': '손목 카메라 기능', '/d2/vision/find_blocks': '손목 카메라 기능'}


def _finite(v):
    """유한한 수(bool · 글자 · 아주 큰 정수 제외)면 float, 아니면 None. 바깥 영향 없음."""
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        return None
    try:
        v = float(v)
    except OverflowError:
        return None
    return v if math.isfinite(v) else None


def placed_offset(center_m, part_id, ok, why, body):
    """W144 A4(E-119): 놓은 블록 다시 재기(find_blocks area assembly) 답 → 기록용 dict. ROS 없음 · 바깥 영향 없음.

    응답 꼴은 비전 #177(민범진, 10/14): 성공 {"ok": true, "part_id", "blocks": [b], "rejected": null, "detail": "", "measured_stamp"} ·
    측정 거절 · 블록 없음 = ok true · blocks [] · rejected 코드(블록 없음은 NO_PART) + detail · 요청 오류 = success false(SCAN_FAILED '요청 오류: …').
    measured_stamp = 쓴 깊이 중 마지막 장을 비전 노드가 받은 시각(초, 노드 시계 — 영상 header.stamp 아님).
    입력: 설계 중심 center_m([x, y, z] base_link m), 요청한 part_id, io.w144_measure 의 (ok, reason, 응답 dict).
    출력: {'status': 'MEASURED', 'x_m', 'y_m'(잰 값 m), 'target_x_m', 'target_y_m'(비교한 설계 중심 m), 'dx_mm', 'dy_mm', 'yaw_deg', 'up', 'measured_stamp'}
      — 'x · y 기록 가능'일 뿐 다시 집기 판정이 아니다. 그 밖은 {'status': 'UNMEASURED', 'reason', 'detail', 'measured_stamp'}:
      reason = 실패 reason(요청 오류 · ERROR · TIMEOUT …) | 거절 코드(rejected 글자 그대로 — NO_PART 포함) | BAD_RESPONSE(꼴이 계약과 다름).
      못 잰 것을 오차 0 으로 만들지 않는다(E-119 ②). 이상한 칸은 None.
    """
    def unmeasured(reason, detail, stamp=None):
        return {'status': 'UNMEASURED', 'reason': reason, 'detail': detail if isinstance(detail, str) else None, 'measured_stamp': stamp}

    if not ok or not isinstance(body, dict):
        return unmeasured(why or 'ERROR', body.get('detail') if isinstance(body, dict) else None)
    stamp = _finite(body.get('measured_stamp'))
    if body.get('ok') is not True or body.get('part_id') != part_id:     # 다른 블록의 답을 이 블록 값으로 적지 않는다
        return unmeasured('BAD_RESPONSE', f'ok · part_id 가 요청과 다름({body.get("part_id")!r})', stamp)
    blocks, rejected = body.get('blocks'), body.get('rejected')
    if blocks == []:
        if isinstance(rejected, str) and rejected:
            return unmeasured(rejected, body.get('detail'), stamp)        # 측정 거절 · 블록 없음(NO_PART) — 코드는 비전 정의 그대로
        return unmeasured('BAD_RESPONSE', 'blocks [] 인데 rejected 코드 없음', stamp)
    if not isinstance(blocks, list) or len(blocks) != 1 or not isinstance(blocks[0], dict) or rejected is not None:
        n = len(blocks) if isinstance(blocks, list) else None
        return unmeasured('BAD_RESPONSE', f'blocks {n}개 · rejected {rejected!r}', stamp)
    b = blocks[0]
    x, y = _finite(b.get('x_m')), _finite(b.get('y_m'))
    if x is None or y is None:
        return unmeasured('BAD_RESPONSE', 'x_m · y_m 없음 · 유한한 수 아님', stamp)
    dx_raw, dy_raw = (x - center_m[0]) * 1000.0, (y - center_m[1]) * 1000.0     # 판단용(반올림 전) — 기록용 dx_mm 은 소수 2자리
    dx, dy = round(dx_raw, 2), round(dy_raw, 2)
    if not (math.isfinite(dx) and math.isfinite(dy)):
        return unmeasured('BAD_RESPONSE', 'x_m · y_m 이 유한하지 않음', stamp)
    up = b.get('up') if b.get('up') in ('THICKNESS', 'WIDTH', 'LENGTH') else None
    tilted = b.get('tilted') if isinstance(b.get('tilted'), bool) else None
    corners = b.get('corners_z_m')
    # top_z_m = #177 윗면 점 가운데 값(잰 값). 다시 집기 높이는 이것과 자세 치수로 구한다(regrasp_decision) — 설계 z 로 대신하지 않음
    # MEASURED = 'x · y 를 기록할 수 있음'일 뿐 — 다시 집기 판정은 regrasp_decision(대상 · 자세 · 기울기 · 시각 · 횟수)이 따로 한다
    return {'status': 'MEASURED', 'x_m': round(x, 5), 'y_m': round(y, 5), 'target_x_m': round(float(center_m[0]), 5),
            'target_y_m': round(float(center_m[1]), 5), 'dx_mm': dx, 'dy_mm': dy, 'dx_raw_mm': dx_raw, 'dy_raw_mm': dy_raw,
            'yaw_deg': _finite(b.get('yaw_deg')), 'up': up,
            'tilted': tilted, 'top_z_m': _finite(b.get('top_z_m')), 'measured_stamp': stamp,
            # 모서리 안쪽 칸 높이 4곳(m, CORNER_SIGNS 순서, 못 잰 곳 None — 꼭짓점이 아님) — 응답에 칸이 있을 때만 싣는다(지금 비전 응답의 기록은 그대로).
            # 칸 이름 · 순서는 민범진과 맞출 제안(아직 계약 아님). 꼴이 다르면(4개 목록 아님) None
            **({'corners_z_m': [_finite(v) for v in corners] if isinstance(corners, list) and len(corners) == 4 else None}
               if 'corners_z_m' in b else {})}


A4_TOL_MM = 2.0            # 축별 이 값 이상이면 다시 집어 놓는다(10/14 20:37 PL 승인 — 일정표 W144 · E-119 의 3 mm 초과를 바꿈)
FLOAT_EPS_MM = 1e-9        # 부동소수 표현 오차만 흡수(예 0.402 − 0.4 = 1.9999999999996 mm) — 문턱을 바꾸는 값이 아님
A4_MAX_TRIES = 2           # 같은 블록 다시 집기 최대 횟수(W144 '최대 2번') — 다 쓰고도 크면 남은 오차를 적고 다음 블록으로(10/14 19:47 사용자 확정)
STAMP_ROUND_S = 0.0005     # 비전 #177 이 measured_stamp 를 소수 3자리(1 ms)로 반올림해 보낸다 — 비교에 그 반올림 폭만 둔다(새 문턱 아님)
UP_NAMES = ('LENGTH', 'WIDTH', 'THICKNESS')
A4_Z_TOL_MM = 5.0          # 모서리 안쪽 칸 높이: 가장 낮은 1곳을 뺀 3곳의 평균과 하나라도 이만큼 이상 다르면 Z 문제 → 사람 수정 대기(10/15 사용자 합의, 단일 블록)
# 모서리 순서 = 윗면 (긴 축 −, 짧은 축 −) · (−, +) · (+, −) · (+, +). 비전 corners_z_m 칸과 같은 순서 — 칸 이름 · 순서는 민범진과 맞출 제안(아직 계약 아님)
CORNER_SIGNS = ((-1, -1), (-1, 1), (1, -1), (1, 1))


def design_up(quat):
    """설계 자세(쿼터니언 xyzw) → 위를 향한 블록 축 이름. 블록 축 k 의 base z 성분이 가장 큰 것. 바깥 영향 없음."""
    R = matrix_from_quat(quat)
    return UP_NAMES[max(range(3), key=lambda k: abs(R[2][k]))]


def design_yaw_deg(quat):
    """설계 자세(쿼터니언 xyzw) → 블록 긴 축(LENGTH)이 base x 와 이루는 각(°, [−90, 90)) — 비전 yaw_deg 와 같은 규약(180° 대칭). 바깥 영향 없음."""
    R = matrix_from_quat(quat)
    return (math.degrees(math.atan2(R[1][0], R[0][0])) + 90.0) % 180.0 - 90.0


def corner_xy_errors(row, part, cfg):
    """중심과 윗면 모서리 4곳의 x · y 오차(mm, 잰 − 설계). 바깥 영향 없음.

    입력: row = placed_offset MEASURED(dx_raw_mm · dy_raw_mm · yaw_deg · up — 눕힘 THICKNESS · 옆세움 WIDTH) · part(quat) · cfg(block_actual_m).
    출력: [('center', ex, ey), ('c1', ex, ey), …, ('c4', ex, ey)] — 모서리 순서는 CORNER_SIGNS.
    모서리는 비전이 재지 않는다: 잰 중심 · 잰 yaw 와 실측 블록 치수로 만든 **계산값**(직사각형 가정)이다. 잰 yaw − 설계 yaw 는 블록이 180° 대칭이라
    [−90, 90) 로 감아 같은 모서리끼리 짝짓는다. 회전만 틀어졌을 때 모서리 한 축 2 mm ≈ 3.1°(긴 축 절반 37.2 mm).
    """
    L, W, T = (float(v) * 1000.0 for v in cfg['block_actual_m'])
    a, b = L / 2.0, (W if row['up'] == 'THICKNESS' else T) / 2.0
    yd = design_yaw_deg(part['quat'])
    ym = math.radians(yd + (row['yaw_deg'] - yd + 90.0) % 180.0 - 90.0)
    yd = math.radians(yd)
    dx, dy = row.get('dx_raw_mm', row['dx_mm']), row.get('dy_raw_mm', row['dy_mm'])
    out = [('center', dx, dy)]
    for k, (su, sv) in enumerate(CORNER_SIGNS, 1):
        u, v = su * a, sv * b
        out.append((f'c{k}', dx + (u * math.cos(ym) - v * math.sin(ym)) - (u * math.cos(yd) - v * math.sin(yd)),
                    dy + (u * math.sin(ym) + v * math.cos(ym)) - (u * math.sin(yd) + v * math.cos(yd))))
    return out


def corner_z_check(corners_z_m):
    """모서리 안쪽 칸 높이 4곳(m — 비전 corners_z_m, 꼭짓점이 아니라 잰 중심 · yaw 기준 윗면 안쪽 칸) → ('OK' | 'Z' | 'MISSING', 글). 바깥 영향 없음.

    10/15 사용자 합의: 가장 낮은 1곳을 빼고 남은 3곳의 평균과 하나라도 A4_Z_TOL_MM 이상 다르면 Z 문제(자동 다시 집기 안 함 · 사람 수정 대기).
    판정 문턱은 칸 높이 기준 '세 점 평균 대비 편차 ≥ 5 mm' 하나다. 예: 칸 하나만 h 높으면 편차 2h/3 → 칸 높이 7.5 mm 부터.
    칸이 꼭짓점 안쪽이라 강체로 기운 블록은 꼭짓점이 더 들려야 걸린다(긴 끝 들림 ≈ 9 mm) — 그 전에 비전 TILTED 가 먼저 거른다.
    4곳이 다 유한한 수가 아니면 'MISSING' — 못 얻은 것을 정상으로 보지 않는다.
    """
    if not isinstance(corners_z_m, (list, tuple)) or len(corners_z_m) != 4:
        return 'MISSING', '모서리 높이 4곳이 응답에 없어요'
    z = [_finite(v) for v in corners_z_m]
    if any(v is None for v in z):
        return 'MISSING', f'모서리 4곳 중 {sum(v is None for v in z)}곳 높이를 못 얻었어요'
    rest = sorted(z)[1:]                            # 가장 낮은 1곳을 뺀다
    mean = sum(rest) / 3.0
    dev = max(abs(v - mean) for v in rest) * 1000.0
    if dev >= A4_Z_TOL_MM - FLOAT_EPS_MM:
        return 'Z', f'모서리 높이 차 {dev:.1f} mm ≥ {A4_Z_TOL_MM:.0f} mm(가장 낮은 곳을 뺀 3곳 평균 기준)'
    return 'OK', f'모서리 높이 차 {dev:.1f} mm'


def regrasp_decision(row, t_req, t_resp, part, place_pose, cfg, tries, corners=False):
    """W144 A4 자동 다시 집기 판단(ROS 없음 · 바깥 영향 없음). 기록 모드와 따로 — w144_regrasp 가 켜졌을 때만 부른다.

    corners(기본 False — 선택 기능, task 파라미터 w144_corner_judge): False 면 지금 판정 그대로(중심 dx · dy 만, 모서리 칸은 보지 않음).
    True 면 모서리 판정(10/15 사용자 합의, 단일 블록): 모서리 안쪽 칸 높이 → 중심 · 모서리 x · y. 비전 TILTED 거절 · tilted True 는 사람 수정 대기(HOLD — 안 B′,
    #182 의 '기울어짐도 기록 · 다음 블록'과 달라 PL 확인 대기), 다른 거절은 지금처럼 SKIP. 비전이 모서리 높이(corners_z_m)를 내야 돈다 —
    칸이 없으면 '못 얻음'으로 다시 집지 않는다(지금 비전에서 켜면 다시 집기가 모두 멈춘다 — 비전 칸과 같이 켤 것).

    입력: row = placed_offset 결과(없으면 None) · t_req · t_resp = 측정을 부른 · 답을 받은 시각(time.time(), 초) ·
          part = 설계 부품(part_id · quat · grasp) · place_pose = 이 블록을 놓을 때 실제로 보낸 목표((xyz m, 쿼터니언) — 흩뿌림이면 같은 층
          1 mm 띄운 자리, 칸 방식이면 설계 자리. 모르면 None) · cfg = robot.yaml dict(block_actual_m) · tries = 이 블록을 이미 다시 집은 횟수.
    최신성: #177 은 요청 뒤에 받은 깊이만 쓰고(fresh_frames) measured_stamp 는 그중 마지막 장을 받은 시각(노드 시계)이다 —
          같은 로봇 PC 시계라는 가정 아래 t_req ≤ measured_stamp ≤ t_resp 일 때만 '이번 요청의 영상'으로 본다(밖이면 시계 · 영상을 믿지 않음).
          measured_stamp 는 1 ms 로 반올림돼 오므로 양 끝에 STAMP_ROUND_S(0.5 ms)만 둔다.
    판정 차례(10/15 사용자 합의 — 단일 블록): 측정 유효 → 모서리 안쪽 칸 높이(corner_z_check — 못 얻으면 SKIP, Z 문제면 HOLD) → 중심 · 모서리 4곳 x · y(corner_xy_errors).
    출력: ('OK', None) = 중심 · 모서리 모두 두 축 A4_TOL_MM 미만(반올림 전 값) — 그대로 이어 감 / ('HOLD', 글) = Z 문제 — 다시 집지 않고 사람 수정 대기 / ('EXHAUSTED', 글) = A4_TOL_MM 이상이지만 이미 A4_MAX_TRIES 번 다시 놓음 — 남은 오차를 적고
          이어 감(유효한 측정일 때만 — 측정 불가 · 기울어짐 등은 그 전에 SKIP) / ('REGRASP', 목표) = PickPlace 목표 {part_id, supply_slot '', grasp(레시피), pick_pose(잰 중심 ·
          잰 yaw · 잰 윗면에서 자세 치수로 구한 중심 높이 — block_pose), place_pose(설계), open_width_m 0, obstacles []} /
          ('SKIP', 까닭) = 측정을 못 씀 — 다시 집지 않고 까닭만 적고 다음 블록(사람 대기 없음, 10/14 21:5x 사용자 결정). 다시 집기 조건 = |dx| ≥ A4_TOL_MM 또는 |dy| ≥ A4_TOL_MM(반올림 전 값). 측정을 못 했거나 믿을 수 없으면(대상 · 자세 다름 · 기울어짐 · 좌표 · yaw · 윗면 없음 · 측정 시각이 요청 · 답 사이가 아님 ·
          원래 놓은 목표를 모름) 다시 집지 않는다. 놓는 목표는 원래 놓을 때 보낸 것 그대로(설계 중심으로 덮어쓰지 않음).
    """
    if row is None:
        return 'SKIP', '놓은 블록을 다시 재지 못했어요(측정 꺼짐 · 답 없음)'
    if row.get('status') != 'MEASURED':
        if corners and row.get('reason') == 'TILTED':   # 모서리 판정이 켜졌으면 비전 TILTED 거절도 Z 문제 — 자동 집기 없이 사람 수정 대기(안 B′, #182 와 다름 → PL 확인 대기)
            return 'HOLD', f'놓은 블록이 기울었어요(손목 카메라 TILTED{": " + row["detail"] if row.get("detail") else ""}) — 손으로 바로 놓은 뒤 [계속]을 누르면 다시 잽니다'
        return 'SKIP', f'놓은 블록을 재지 못했어요({row.get("reason")}{": " + row["detail"] if row.get("detail") else ""})'
    stamp = row.get('measured_stamp')
    if not isinstance(stamp, (int, float)) or isinstance(stamp, bool) or not math.isfinite(stamp) or not t_req - STAMP_ROUND_S <= stamp <= t_resp + STAMP_ROUND_S:
        return 'SKIP', '측정 영상이 이번 요청 뒤의 것인지 확인되지 않았어요(measured_stamp가 요청 · 답 시각 사이가 아님)'
    if corners and row.get('tilted') is True:      # 잰 블록이 기울었다고 온 경우도 TILTED 거절과 같이 사람 수정 대기(안 B′). 모름(None)은 SKIP
        return 'HOLD', '놓은 블록이 기울었어요(손목 카메라 tilted) — 손으로 바로 놓은 뒤 [계속]을 누르면 다시 잽니다'
    if row.get('tilted') is not False:
        return 'SKIP', '놓은 블록이 기울어졌거나 기울기를 모릅니다'
    want_up = design_up(part['quat'])
    if row.get('up') != want_up:
        return 'SKIP', f'놓은 블록 자세({row.get("up")})가 설계({want_up})와 다릅니다'
    if row.get('yaw_deg') is None or row.get('top_z_m') is None:
        return 'SKIP', '잰 yaw · 윗면 높이가 없습니다'
    if not corners:                                # 지금 판정(기본) — 중심만
        dx_raw, dy_raw = row.get('dx_raw_mm', row['dx_mm']), row.get('dy_raw_mm', row['dy_mm'])   # 반올림 전 값으로 판단(1.996 이 2.00 으로 보여도 다시 집지 않음)
        if abs(dx_raw) < A4_TOL_MM - FLOAT_EPS_MM and abs(dy_raw) < A4_TOL_MM - FLOAT_EPS_MM:
            return 'OK', None
        if tries >= A4_MAX_TRIES:                  # 다시 집기를 다 썼다 — 남은 오차만 적고 이어 감(기존 VERIFY 판정은 이미 통과, 사람 대기 없음)
            return 'EXHAUSTED', f'다시 집기 {A4_MAX_TRIES}번을 다 써도 남은 오차 dx {row["dx_mm"]} · dy {row["dy_mm"]} mm — 그대로 다음 블록'
        return _regrasp_goal(row, part, place_pose, cfg)
    if row.get('up') not in ('THICKNESS', 'WIDTH'):
        return 'SKIP', f'윗면 모서리를 만들 수 없는 자세({row.get("up")})예요'
    z_state, z_text = corner_z_check(row.get('corners_z_m'))
    if z_state == 'MISSING':                       # 못 얻은 것은 정상이 아니다 — 다시 집지 않고 까닭을 적고 다음 블록(#182 규칙, 사람 대기 여부는 결정 대기)
        return 'SKIP', f'모서리 높이를 못 얻어 판정하지 않아요({z_text})'
    if z_state == 'Z':
        return 'HOLD', f'놓은 블록이 기울었어요({z_text}) — 손으로 바로 놓은 뒤 [계속]을 누르면 다시 잽니다'
    errs = corner_xy_errors(row, part, cfg)        # 반올림 전 값으로 판단(1.996 이 2.00 으로 보여도 다시 집지 않음)
    if all(abs(ex) < A4_TOL_MM - FLOAT_EPS_MM and abs(ey) < A4_TOL_MM - FLOAT_EPS_MM for _, ex, ey in errs):
        return 'OK', None
    worst = max(errs, key=lambda e: max(abs(e[1]), abs(e[2])))
    if tries >= A4_MAX_TRIES:                      # 다시 집기를 다 썼다 — 남은 오차만 적고 이어 감(기존 VERIFY 판정은 이미 통과, 사람 대기 없음)
        return 'EXHAUSTED', (f'다시 집기 {A4_MAX_TRIES}번을 다 써도 남은 오차 dx {row["dx_mm"]} · dy {row["dy_mm"]} mm'
                             f'(가장 큰 곳 {worst[0]} {worst[1]:.2f} · {worst[2]:.2f} mm) — 그대로 다음 블록')
    return _regrasp_goal(row, part, place_pose, cfg)


def _regrasp_goal(row, part, place_pose, cfg):
    """다시 집기 목표(잰 자세에서 집어 처음 보낸 목표에 놓기). 처음 목표를 모르면 SKIP. 바깥 영향 없음."""
    if place_pose is None:
        return 'SKIP', '이 블록을 놓을 때 보낸 목표를 몰라 다시 놓을 자리를 정할 수 없어요'
    pick = block_pose(cfg, row['up'], row['yaw_deg'], row['x_m'], row['y_m'], row['top_z_m'])
    return 'REGRASP', {'part_id': part['part_id'], 'supply_slot': '', 'grasp': part['grasp'], 'pick_pose': pick,
                       'place_pose': copy.deepcopy(place_pose), 'open_width_m': 0.0, 'obstacles': []}


def wait_until(event, should_abort, poll_s=0.05, timeout_s=None):
    """event 가 켜질 때까지 기다리되, should_abort() 가 참이 되면 바로 빠져나온다. timeout_s 를 주면 그 시간(monotonic)이 지나도 빠져나온다.

    반환: True = event 가 켜졌다(답이 왔다), False = 중단 신호가 먼저 왔다(또는 timeout_s 가 지났다).
    poll_s 는 중단 신호를 확인하는 간격일 뿐 기다리는 시간 제한이 아니다. 완전 블로킹 대기(future.result())로
    정지 처리가 막히지 않게 하려고 task_node 의 모든 기다림이 이것을 거친다.
    """
    deadline = None if timeout_s is None else time.monotonic() + timeout_s
    while True:
        if should_abort() or (deadline is not None and time.monotonic() >= deadline):
            return False
        wait_s = poll_s if deadline is None else min(poll_s, max(0.0, deadline - time.monotonic()))
        if event.wait(wait_s):
            # 이미 켜진 이벤트도 취소 · 시간 초과보다 우선하지 않는다. 경계에서 늦은 성공을 적용하지 않기 위해서다
            return not should_abort() and (deadline is None or time.monotonic() < deadline)


class TaskManager:
    """자동 조립 상태표. 설계를 고르고(select_design) 출발(start)하면 진행 확인 → 다음 블록 → 집기·놓기 → 배치 확인을 되풀이한다.

    들어오는 것(콜백 스레드): command(HmiCommand) · on_intent(음성) · on_safety(정지 노드) · on_gripper · shutdown.
    나가는 것: io 로 /d2/task/state · /d2/task/progress 방송, move_to · check_progress · pick_place 호출.
    바깥 영향: 로봇은 pick_place · move_to 를 통해서만 움직인다(이 클래스는 팔을 직접 안 건드린다).
    실패: TIMEOUT 은 먼저 정지 요청 후 정지 상태를 기다린다(IRD 7장). ERROR · GRIPPER_NO_RESPONSE · NO_FEEDBACK 는 ERROR 로 간다(E-74).
    IRD 에 없는 reason 도 같은 공통 분기로 일반 실패 → ERROR 로 간다.
    정지 노드가 멈추면(safety/state stopped) 어느 상태에서든 STOPPED 로 가고, 잠금이 풀린 신호가 새로 오면 RECOVER 로 간다.
    """

    def __init__(self, cfg, io, logger=None, clock=time.monotonic, monitor_hmi=False, open_width_m=None):
        """cfg = robot.yaml dict, io = 위 설명의 바깥 일 담당, logger = RunLogger(없으면 파일 없이 run_id 만 만든다), clock = 단조 시계(시험용).
        monitor_hmi 는 웹 연결 신호 감시 여부. 이 클래스의 기본은 False(시험 · 웹 없이 동작)이고, task 노드는 design_source 와 따로 노드 파라미터 monitor_hmi(기본 true)로 정해 넘긴다(E-62).
        공급 방식은 cfg['supply_mode'](없으면 slots): slots = 공급 칸 6개(기본), scatter = 흩뿌린 공급(관측 → 후보 고르기, E-55). scatter 는 open_width_m(m)을 명시로 받아야
        집기 요청을 만든다 — task 노드는 0 을 넘기고, 0 은 'pick_place 가 robot.yaml grasp_open_pick_m.<잡기>를 쓴다'는 뜻이다(IRD PickPlace.srv). None(안 넘김)이면 scatter 선택은 ERROR 로 끝난다.
        만들 때 디스크에 보관돼 있던 미전송 요약을 되살려 다시 보낼 수 있게 한다(로봇 작업을 받기 전). 처음 상태는 IDLE."""
        self.cfg, self.io = cfg, io
        self.logger = logger if logger is not None else RunLogger()
        self.run_id = None                        # 지금(또는 방금 끝난) 조립의 run_id. 새 설계를 고르거나 IDLE 로 가면 비운다
        self.last_build = None                    # 가장 최근에 끝난 조립의 build/2 요약(표시용). 새 출발로는 지우지 않는다
        self.pending_builds = {}                  # run_id → 아직 DB 저장이 확인 안 된 build/2 요약(복사본)
        self._build_inflight = set()              # 지금 save_build 요청을 보내고 답을 기다리는 run_id
        self._build_last_try = {}                 # run_id → 마지막으로 보낸 시각(monotonic, 다시 보내기 간격 계산)
        self._build_ready = set()                 # 디스크 보관이 끝났거나(성공 · 실패) 보관하지 않는 run_id — 이것만 보낸다
        self._build_queued = {}                   # run_id → 대기 목록에 들어온 시각(monotonic). 디스크가 늦으면 이 시각으로 기다림을 끊는다
        self._rejected_unsaved = {}               # run_id → 웹이 영구 거절(retry:false)했는데 rejected/ 보관이 아직 안 된 요약. 네트워크로는 다시 보내지 않는다
        self._reject_inflight = set()             # 거절 보관을 쓰기 스레드에 맡기고 끝을 기다리는 run_id
        self._reject_last_try = {}                # run_id → 마지막으로 보관을 맡긴 시각(monotonic, retry_s 간격)
        self._clock = clock
        self.command_s = cfg['timeout']['command_s']   # 화면 명령(선택 · 출발)을 받은 뒤 확정해야 하는 시간(s) — 넘으면 실행하지 않고 TIMEOUT
        self._epoch = 0                           # 상태가 바뀔 때마다 +1 — 설계 조회 중에 상태가 바뀌었는지 알아보는 표
        self._steps = StepTracker()               # 집기 요청마다 마지막 피드백 step · 결과 수신 시각(흩뿌림 집기 실패 뒤 재시도 판단 _scatter_failed 가 쓴다)
        self._pick_seq = 0                        # pick_place 요청마다 새 번호
        self.last_pick = None                     # 가장 최근 pick_place 가 끝났을 때의 {request_id, ok, reason, step, result_at}
        self._stop_count = 0                      # stopped=true 신호를 받은 횟수 — 정지 뒤 [다시 시작]까지 끝나도 그 전에 보낸 요청의 늦은 답을 알아본다
        self._lookup_token = 0                    # 가장 최근 설계 조회 요청 번호 — 이전 요청의 늦은 답을 버리는 데 쓴다
        self._lock = threading.RLock()            # 콜백 스레드와 작업 스레드가 같이 만지는 값들
        self.state = 'IDLE'
        self.planner = None
        self.design_id = None
        self.part_id = None                      # 지금 다루는 블록
        self.goal = None                          # 보낼(보낸) pick_place 목표 = next_block 의 FOUND
        self.placed = None                        # 방금 놓은 블록(VERIFY 에서 확인)
        self.safety = None                        # 마지막 safety_state/1
        self.safety_count = 0                     # safety/state 를 받은 횟수 — 다시 시작 감지에 쓴다(시계가 다른 PC 의 stamp 는 안 믿는다)
        self.gripper = None                       # 마지막 gripper_state/1
        self._entered_count = 0                   # STOPPED · ERROR 에 들어갈 때의 safety_count
        self._assembling = False                  # 정지됐을 때 조립 중이었나(RECOVER 가 CHECK 로 이어 갈지 정한다)
        self._cleaned = False                     # 이번 정지에서 [초기화](정리 확인)를 받았나 — [정리 끝] 뒤 장면을 비울지(10/14 박진용)
        self._pending_start = False               # WAIT_SUPPLY 에서 받은 [계속]
        self._supply_moved = False                # WAIT_SUPPLY 에서 관측 자세에 이미 도착했나(도착해야 알림 · [계속]을 받는다)
        self._reobserved = False                  # UNKNOWN_BLOCK 재관측을 이미 한 번 했나
        self._plan_retry = 0                      # PLAN_FAILED 다시 계획 횟수
        self._grasp_stage = 0                     # GRASP_FAILED: 1 = 같은 칸 재시도 중, 2 = 다음 칸 시도 중
        self._shutdown = False
        self._timeout_pending = False             # 정지 요청의 성공 응답만으로는 서 있다고 볼 수 없다. stopped 신호가 필요하다
        self._recover_slow = False                # 복구 뒤 첫 이동이 성공할 때까지 저속을 유지한다
        self._monitor_hmi = monitor_hmi
        self.supply_mode = cfg.get('supply_mode', 'slots')
        if self.supply_mode not in ('slots', 'scatter'):
            raise ValueError(f"robot.yaml supply_mode 는 slots · scatter 중 하나다: {self.supply_mode!r}")
        self._scatter = (ScatterFlow(cfg, BlockPicker(cfg['find']['min_gap_mm'], cfg), self.supply_mode, open_width_m)
                         if self.supply_mode == 'scatter' else None)
        # 받침 경유(robot.yaml scatter.via_spare, 10/13): 공급 → 받침에 눕힘 → 받침 위에서 깊이로 재기 → 레시피 잡기로 곧게 다시 집어 조립
        self._spare_on = None                     # 받침에 놓아 둔 블록의 목표 part_id(있으면 다음 SELECT 가 공급 대신 받침부터 마무리한다)
        self._w144_due = None                     # 닫을 W144 기록(조립 결과 DONE 인가) — 잠금 밖 작업 스레드가 닫는다(10/14 #152 리뷰 3)
        self._w144_starting = False               # 출발했고 W144 기록을 아직 안 열었다 — CHECK 첫 단계가 잠금 밖에서 연다(#152 리뷰 3)
        self._w144_reopen = False                 # 조립 중 정지로 W144 녹화를 닫았다 — [다시 시작]으로 이어 가면 새로 연다(10/15 박진용)
        self._spare_centers = 0                   # 받침 위 이 블록에 받침 맞추기를 한 수 — 맞춘 뒤 다시 재서 2 mm 넘으면 다시(scatter_pick.CENTER_MAX 번까지)
        self._spare_failed = []                   # 이번 블록에서 받침으로 옮기다 안 된 시도 [(x_m, y_m, 잡기, 각도 °, 옮김 m)] — 같은 후보를 다시 고르지 않게
        self._plan_streak = 0                     # 움직이기 전 길 계산 거절이 연달아 몇 번(받침 경유) — PLAN_STREAK 번이면 사람
        self._spare_empty = 0                     # 받침에 놓았는데 안 보인 수(연달아) — 1번째는 사람이 보고 [계속](다시 잼), 2번째는 비었다고 보고 공급에서
        self._spare_seen = None                   # 받침 경유: 마지막 공급 관측 (목표, find_blocks 응답) — 움직이기 전 거절이면 다시 찍지 않고 같은 관측의 다음 후보
        self._no_match = 0                        # 받침 경유: 연달아 후보 없음 — 1번째는 다시 찍고 2번째에 사람(찍을 때마다 판정이 조금씩 다름, one_pick)
        self._bad_blocks = 0                      # 받침 경유: 연달아 블록 값 오류(장애물을 못 만듦) — RETRY 번까지 다시 찍고 사람
        self._fresh = False                       # 출발 직후 첫 CHECK 인가 — 흩뿌림이면 장면 정리 · 홈부터(one_pick 시작과 같음)
        self._hold_then = None                    # 받침 경유 사람 확인(WAIT_SUPPLY) 뒤 [계속]에 할 일: 'reset'(시도 수 비우고 이어 감) · 'verify'(다시 확인)
        self._tally = {'sent': 0, 'ok': 0}        # 이번 조립의 집기 · 놓기 요청 수 · 성공 수(one_pick 집계와 같음 — 끝날 때 기록)
        self._a4_tries = {}                       # W144 A4: 블록마다 자동으로 다시 집은 횟수(이번 조립)
        self._a4_last = None                      # W144 A4: 마지막 측정 (token, 기록 row, 부른 시각, 답 받은 시각)
        self._a4_place = {}                       # W144 A4: 블록마다 놓을 때 실제로 보낸 place_pose(흩뿌림은 1 mm 띄운 자리) — 다시 놓을 자리
        self._spread = None                       # 같은 층 1 mm 띄운 놓는 중심(part_id → [x, y, z] m) — 설계가 정해질 때 한 번 계산
        # 손목 그림 집기 순위(state/2 pick, 10/14 22시 PL) — 공급 find_blocks 결과를 적용할 때만 바뀌고(받침 · 조립 자리 재기는 공급 그림이
        # 아니라 그대로) 그 사이 모든 state/2 에 실린다. 조립 끝(DONE) · 취소 · IDLE · 새 출발이면 비운다(None = 키를 뺌)
        self._pick = None
        if self._scatter is not None:             # 흩뿌림 실패 뒤 재시도 설정(robot.yaml scatter.*) — 없거나 이상하면 만들 때 거절한다
            sc = cfg['scatter']
            self.max_attempts = sc['max_attempts']
            if type(self.max_attempts) is not int or self.max_attempts < 1:
                raise ValueError(f'robot.yaml scatter.max_attempts 는 1 이상의 정수다: {self.max_attempts!r}')
            self.gripper_max_age_s = self._positive(sc['gripper_max_age_s'], 'scatter.gripper_max_age_s')
            self.gripper_wait_s = self._positive(sc['gripper_wait_s'], 'scatter.gripper_wait_s')
        self._scatter_fail = {}                   # 흩뿌림 집기 시도 횟수(part_id → 번, PickPlace 요청을 보낼 때 +1). 재관측 · 정지 · 복구로 비우지 않고, VERIFY 에서 present 로 확인한 블록만 지운다
        self._sleep, self._wall = time.sleep, time.time    # 시험에서 바꾼다 — gripper 대기용 sleep · 최신성 판정용 벽시계(grasped_now 는 time.time 기준)
        self._hmi_seen_at = None                  # PC 사이 stamp 대신 이 PC 의 단조 시계로 수신 간격을 잰다
        self._hmi_pause = False                   # 잠깐 끊겼다 다시 붙어도 사람의 start 전에는 풀지 않는다
        self._hmi_recheck = False                 # WAIT_HMI 의 start 는 SELECT 에서 진행표부터 다시 확인한다
        self._scan_run = False                    # 스캔 CSV 는 남기되 조립 결과 build/2 로 보내지 않는다
        self._scan_poses, self._scan_index = (), 0
        self._in_handler = False                  # 작업 스레드가 STOPPED 밖의 단계(이동 · 집기 결과 처리 등)를 돌리는 중 — 그동안 정리 취소는 받지 않는다
        self._gripper_seq = 0                     # 그리퍼 열기 요청 번호 — 답이 한 번만 대기를 풀게 한다
        self._gripper_busy = False                # 정지 중 그리퍼 열기를 보냈고 서버의 실제 답이 아직 없다 — TIMEOUT 으로는 풀지 않는다(10/13 사용자 결정)
        for run_id, summary in self.logger.load_pending():
            self.pending_builds[run_id] = summary
            self._build_ready.add(run_id)         # 이미 디스크에 있다
        if self.pending_builds:
            LOG.warning('디스크에 보관된 미전송 요약 %d개를 되살렸다: %s', len(self.pending_builds), ', '.join(self.pending_builds))
        self._handlers = {'CHECK': self._check, 'SELECT': self._select, 'PICK_PLACE': self._pick_place,
                          'VERIFY': self._verify, 'WAIT_SUPPLY': self._wait_supply, 'STOPPED': self._stopped,
                          'RECOVER': self._recover, 'ERROR': self._error,
                          'SCAN_MOVE': self._scan_move, 'SCAN_CAPTURE': self._scan_capture, 'SCAN_INFER': self._scan_infer}

    # ---------- 콜백 스레드에서 들어오는 것 ----------
    def on_safety(self, st):
        """정지 노드의 safety_state/1(dict)을 적는다. 상태 전이는 작업 스레드가 run_once 에서 한다(콜백은 변수만).

        객체(dict)가 아니면 버리고 경고만 남긴다 — 적어 두면 halted() 등이 .get 에서 터져 작업 스레드(노드)가 멈춘다(10/14 점검 ③-5).
        """
        if not isinstance(st, dict):
            LOG.warning('safety/state 가 객체가 아니라 버린다: %r', st)
            return
        with self._lock:
            self.safety = st
            self.safety_count += 1
            if st.get('stopped'):
                self._stop_count += 1
                if self._scatter is not None:
                    self._scatter.invalidate()        # 관측 · 조회 중이던 흩뿌림 답은 정지 뒤에 적용하지 않는다

    def on_gripper(self, st):
        """그리퍼 노드의 gripper_state/1(dict)을 적는다. grasped 가 참이면 블록을 쥐고 있다."""
        with self._lock:
            self.gripper = st

    def on_hmi_alive(self, body):
        """웹 연결 JSON 의 alive 가 정확히 true 일 때 수신 시각을 갱신한다. 깨진 값은 무시하고 외부 stamp 는 사용하지 않는다.

        재접속은 대기를 풀지 않는다. 현재 블록 처리 중 끊겼다가 돌아온 경우도 사람의 계속 버튼을 기다린다.
        """
        with self._lock:
            self.poll_hmi()                       # 갱신 전 끊김을 기억해야 빠른 재접속으로 대기 규칙이 사라지지 않는다
            if isinstance(body, dict) and body.get('alive') is True:
                self._hmi_seen_at = self._clock()

    def _hmi_fresh(self):
        """(잠금 안) 웹 연결 신호가 lost_after_s 초 안에 왔는가. 감시 안 하는 로컬 개발은 항상 참."""
        return not self._monitor_hmi or (self._hmi_seen_at is not None and
                                         self._clock() - self._hmi_seen_at < self.cfg['mqtt']['lost_after_s'])

    def poll_hmi(self):
        """타이머가 조립 중 웹 끊김을 기억한다(메모리만, 이동 · 상태 방송 없음). 작업 스레드가 블록을 끝낸 뒤 WAIT_HMI 로 간다."""
        with self._lock:
            if self.run_id and self.state in RUN_STATES and not self._hmi_fresh():
                self._hmi_pause = True

    def shutdown(self):
        """Ctrl+C 등 끝낼 때 부른다. 기다리는 중인 호출이 곧바로 빠져나오고 pick_place 목표가 취소된다."""
        self._shutdown = True

    def finalize(self, flush_s=2.0):
        """끝낼 때(Ctrl+C · 종료) 맨 마지막에 부른다 — 진행 중인 목표 취소를 요청한 **뒤에**. 조립 중이었으면 기록을 닫고
        (ERROR 중이면 ERROR, 그 밖은 STOPPED) 큐에 남은 줄이 파일에 쓰일 때까지 flush_s 초 안에서 기다린다.

        요약 확정은 메모리에서 바로 되고, 기다리는 것은 파일 쓰기뿐이라 디스크가 멈춰도 flush_s 를 넘기지 않는다.
        시간 안에 못 끝나면 경고를 남긴다(flush 의 True 는 '다 처리했다'이지 '저장 성공'이 아니라서 file_ok 도 함께 본다).
        """
        with self._lock:
            if self.logger.active:
                self._finish_run('ERROR' if self.state == 'ERROR' else 'STOPPED')
        if not self.logger.flush(flush_s):
            LOG.warning('기록 파일에 아직 다 안 써졌다(%.1f초 안에 못 끝냄) — 마지막 줄이 CSV 에 없을 수 있다. 요약(last_build)은 메모리에 있다', flush_s)

    # ---------- 결과 저장(save_build) — 보내는 일은 task 노드, 정하는 일은 여기 ----------
    def builds_to_send(self, now, retry_s):
        """지금 /d2/hmi/save_build 로 보낼 요약들 [(run_id, 요약 복사본)]. 보내는 것으로 적어 두므로 호출한 쪽이 꼭 보내야 한다.

        답을 기다리는 중인 것 · 마지막으로 보낸 지 retry_s(monotonic 초)가 안 된 것 · 끝내는 중이면 내지 않는다.
        now = time.monotonic(). 잠금 안에서 메모리만 만진다(네트워크 대기 없음).
        """
        with self._lock:
            if self._shutdown:
                return []
            self._retry_reject_archive(self._clock(), retry_s)    # 보관 재시도 간격은 build_result 와 같은 시계(self._clock)로 잰다
            out = []
            for run_id, summary in self.pending_builds.items():
                last = self._build_last_try.get(run_id)
                if run_id in self._build_inflight or (last is not None and now - last < retry_s):
                    continue
                if run_id not in self._build_ready and self._clock() - self._build_queued.get(run_id, 0.0) < retry_s:
                    continue                      # 디스크 보관이 끝나길 잠깐 기다린다. 디스크가 늦으면 retry_s 뒤에는 기다리지 않고 보낸다
                self._build_inflight.add(run_id)
                self._build_last_try[run_id] = now
                out.append((run_id, copy.deepcopy(summary)))
            return out

    def build_result(self, run_id, success, response_json):
        """save_build 의 답을 그 요청의 run_id 에만 적용한다. 반환: 저장이 확인돼 대기 목록에서 지웠으면 True.

        서비스 success 가 참이고 응답 JSON 이 객체이며 ok 가 참(true)일 때만 지운다(True 는 'DB 저장 확인' 뜻 그대로). 그 밖(실패 · 깨진 JSON · ok 거짓)은 요약을 남긴다.
        단 success 가 참 · 응답이 객체 · ok 가 정확히 false · retry 가 정확히 false 이면 **영구 거절**이다(IRD 4.2 save_build): 대기 목록에서 빼
        다시 보내지 않고 log_dir/rejected/<run_id>.json 에 원기록을 보존한다(파일 I/O 는 기록 스레드, 이 경우도 반환은 False).
        retry 가 없거나 true · 0 · 'false' · null 이면 기존처럼 남겨 다시 보낸다.
        이미 지워졌거나 모르는 run_id 의 늦은 답은 아무것도 안 바꾼다 — 다른 조립의 요약은 건드리지 않는다.
        """
        with self._lock:
            self._build_inflight.discard(run_id)
            if run_id not in self.pending_builds:
                return False
            if self._permanently_rejected(success, response_json):
                LOG.warning('결과 저장이 영구 거절됐다(run %s) — 다시 보내지 않고 rejected/ 에 보관한다: %.80r', run_id, response_json)
                summary = self.pending_builds.pop(run_id)
                self._forget_send_state(run_id)
                self._rejected_unsaved[run_id] = summary
                self._archive_rejected(run_id, self._clock())
                return False
            if success is not True or not self._response_ok(response_json):
                LOG.warning('결과 저장 실패(run %s) — 요약을 남기고 나중에 다시 보낸다: success=%r %.80r', run_id, success, response_json)
                return False
            del self.pending_builds[run_id]
            self._forget_send_state(run_id)
            self.logger.delete_pending(run_id)      # 저장이 확인된 그 run 의 보관 파일만 지운다(기록 스레드가)
            return True

    def _forget_send_state(self, run_id):
        """(잠금 안) 그 run_id 의 전송 상태(마지막 시도 · 보관 끝남 · 대기 시각)를 비운다."""
        self._build_last_try.pop(run_id, None)
        self._build_ready.discard(run_id)
        self._build_queued.pop(run_id, None)

    @staticmethod
    def _permanently_rejected(success, response_json):
        """success 가 True, 응답이 객체, ok 가 정확히 False, retry 가 정확히 False 일 때만 참. ok 가 True 면(저장 성공) 항상 거짓.

        NaN · Infinity 같은 비표준 JSON 상수가 든 응답은 깨진 JSON 으로 보고 거절로 인정하지 않는다(재전송 유지 — 기존 payload 형식 계약)."""
        if success is not True:
            return False
        try:
            body = json.loads(response_json, parse_constant=_reject_constant)
        except (TypeError, ValueError):
            return False
        return isinstance(body, dict) and body.get('ok') is False and body.get('retry') is False

    def _archive_rejected(self, run_id, now):
        """(잠금 안) 거절된 요약의 rejected/ 보관을 기록 스레드에 맡긴다. 네트워크는 쓰지 않는다.

        log_dir 이 비어 있으면 파일로 보존할 수 없다 — 경고하고 요약은 메모리(_rejected_unsaved)에만 둔다(다시 시도하지 않는다).
        큐가 가득 차 못 맡기면 다음 builds_to_send 에서 retry_s 뒤 다시 맡긴다.
        """
        if run_id in self._reject_inflight:
            return
        if not self.logger.enabled:
            LOG.warning('log_dir 이 비어 거절된 요약 %s 을 파일로 보존하지 못한다 — 메모리에만 둔다', run_id)
            return
        self._reject_last_try[run_id] = now
        if self.logger.reject_pending(run_id, copy.deepcopy(self._rejected_unsaved[run_id]),
                                      lambda ok, r=run_id: self._reject_archived(r, ok)):
            self._reject_inflight.add(run_id)

    def _reject_archived(self, run_id, ok):
        """(기록 스레드가 부른다) 거절 보관이 끝났다. 성공이면 메모리 요약을 놓고, 실패면 남겨 두었다가 다시 맡긴다."""
        with self._lock:
            self._reject_inflight.discard(run_id)
            if ok:
                self._rejected_unsaved.pop(run_id, None)
                self._reject_last_try.pop(run_id, None)
            else:
                LOG.warning('거절된 요약 %s 의 파일 보관이 실패했다 — 메모리에 두고 다시 시도한다', run_id)

    def _retry_reject_archive(self, now, retry_s):
        """(잠금 안) 보관 못 한 거절 요약을 retry_s 간격으로 다시 맡긴다(큐 가득 · 쓰기 실패 뒤 회복). 네트워크 재전송과 따로다."""
        if not self.logger.enabled:
            return
        for run_id in list(self._rejected_unsaved):
            last = self._reject_last_try.get(run_id)
            if run_id not in self._reject_inflight and (last is None or now - last >= retry_s):
                self._archive_rejected(run_id, now)

    def _pending_persisted(self, run_id, ok):
        """(기록 스레드가 부른다) 미전송 요약의 디스크 보관이 끝났다. 성공 · 실패 모두 보낼 수 있게 한다(실패하면 메모리 재전송만)."""
        with self._lock:
            if run_id in self.pending_builds:       # 보관이 늦어 메모리로 보내 이미 저장 성공 처리한 run_id 는 다시 넣지 않는다
                self._build_ready.add(run_id)

    def build_failed(self, run_id, why):
        """save_build 요청이 못 나갔거나 시간 안에 답이 없었다. 요약은 남기고 다시 보낼 수 있게 한다."""
        with self._lock:
            self._build_inflight.discard(run_id)
        LOG.warning('결과 저장을 못 했다(run %s): %s — 요약을 남긴다', run_id, why)

    @staticmethod
    def _response_ok(response_json):
        """응답 JSON 글자가 객체이고 ok 가 true(참인 값이 아니라 정확히 true)인가."""
        try:
            body = json.loads(response_json)
        except (TypeError, ValueError):
            return False
        return isinstance(body, dict) and body.get('ok') is True

    def halted(self):
        """정지 신호 · 종료 · 시간 초과 뒤 정지 확인 대기면 참. io 가 새 동작을 막고 기다림을 중단할 때 쓴다."""
        st = self.safety
        return self._shutdown or self._timeout_pending or bool(st and st.get('stopped'))

    def command(self, cmd, design_id=''):
        """화면 버튼 명령(HmiCommand, 다리가 웹 화면 대신 부른다). 반환: (success, reason).

        select_design = 설계를 **조회**(/d2/hmi/get_design)하고 시작 점검 → READY. start = READY 에서 [설계 선택] 때 받아 둔 설계로 출발(점검 한 번 더) → CHECK(다시 조회하지 않는다),
        WAIT_SUPPLY 에서는 [계속](관측 자세 도착 뒤), WAIT_HMI 는 웹 재연결 뒤 진행 확인부터 SELECT 로 이어 간다(조회 없음).
        scan 은 IDLE·SCAN_REVIEW·DONE 에서 새 촬영을 시작한다. cancel 은 READY·ERROR·SCAN_REVIEW 를 IDLE 로 돌린다(음성 cancel 과 같다, S-16).
        조회는 이 호출 스레드가 잠금 **밖에서** 기다린다(최대 timeout.service_s). 조회 실패 · 시간 초과 · 잘못된 답이면 새 조립을 시작하지 않고
        거절한다(로컬 파일로 몰래 대신하지 않는다 — 개발용 로컬은 노드가 design_source=local 로 명시해야 한다).
        reason 은 IRD 7장에 이미 있는 코드만 쓴다: 운전 중 'BUSY', 정지 중 'STOPPED', 조회 시간 초과 'TIMEOUT'. 맞는 코드가 없는 거절(설계 없음 ·
        점검 실패 · 아직 없는 명령)은 reason 을 빈 값으로 두고 이유를 state/2 의 message 글자로만 알린다.
        """
        with self._lock:
            if self._gripper_busy and cmd in ('select_design', 'start', 'scan'):     # 팔을 움직이게 하는 요청 — 그리퍼가 실제로 다 열렸다는 답이 올 때까지 받지 않는다
                return self._reject('BUSY', '그리퍼가 열리는 중이에요. 완료 안내 뒤 다시 누르세요')
        if cmd == 'select_design':
            return self._lookup_then(design_id, 'select')
        if cmd == 'start':
            return self._lookup_then(design_id, 'start')
        if cmd == 'gripper_open':
            return self._gripper_open()
        with self._lock:
            if cmd == 'scan':
                return self._begin_scan()
            if cmd == 'cancel' and self.state == 'STOPPED':
                return self._cleanup_stopped()
            if cmd == 'cancel':
                if not self._cancel_to_idle():
                    return self._reject(self._busy_reason(), '지금은 [취소]할 수 없어요(준비 · 오류 · 스캔 검토에서만). 움직이는 중이면 [정지]를 누르세요')
                return True, ''
            return self._reject('', f'모르는 명령: {cmd}')

    def on_intent(self, intent, design_id=None):
        """음성 의도(intent/1, 다리가 ROS 토픽으로 낸다). start·select_design은 버튼과 같고 cancel은 READY·ERROR·SCAN_REVIEW와 STOPPED(정리 확인 — 화면 cancel과 같은 함수)에서 받는다.

        그 밖(request_design 등)은 무시. 출발할 수 없는 때의 음성 start 는 무시하고 알림 voice_start_ignored 만 낸다(IRD 8.5).
        설계 조회가 있는 동작은 잠금 밖에서 command 로 한다.
        """
        if intent == 'select_design' and design_id:
            self.command('select_design', design_id)
        elif intent == 'start':
            with self._lock:
                can = self.state in ('READY', 'WAIT_SUPPLY', 'WAIT_HMI') or (self.state == 'IDLE' and design_id)
                if not can:
                    self._publish('voice_start_ignored', '지금은 음성 출발을 받을 수 없어 무시했다')
                    return
            ok, _ = self.command('start', design_id or '')
            with self._lock:
                if not ok and self.state == 'WAIT_SUPPLY':      # 아직 관측 자세로 가는 중이라 못 받았다
                    self._publish('voice_start_ignored', '관측 자세로 가는 중이라 음성 출발을 무시했다')
        elif intent == 'cancel':
            with self._lock:
                if self.state == 'STOPPED':
                    self._cleanup_stopped()      # 화면 [초기화]와 같은 함수(10/13 19:45 PL) — 기록 닫기 · 잠금 유지, 팔 이동 없음
                else:
                    self._cancel_to_idle()

    def _cleanup_stopped(self):
        """(잠금 안) STOPPED 의 화면 cancel = 사람이 쥔 블록 · 구조물을 정리했다는 확인(W183, 10/13 16:30 PL). 반환: (success, reason).

        조립 기록을 STOPPED 로 닫고 조립 상태 · 설계 · 진행표 · 목표를 비운다. 상태는 STOPPED 그대로 두고 안전 잠금을 풀지 않으며 로봇을 움직이지 않는다.
        이어서 화면이 /d2/safety/resume 을 부르면 기존 RECOVER 의 비조립 경로로 IDLE 에 간다(조립이 비었으니 CHECK · 자동 재개로 갈 수 없다).
        다시 눌러도 기록은 이미 닫혀 새 기록 · 종료 줄을 만들지 않는다. 작업 단계가 아직 정지 결과를 처리하는 중이면 BUSY 로 거절한다(다시 누르면 된다).
        """
        if self._in_handler:
            return self._reject('BUSY', '멈추는 중인 동작을 정리하고 있어요. 잠시 뒤 다시 누르세요')
        self.logger.log('cleanup', None, 'STOPPED 정리 확인')     # 닫기 전에 남긴다(run 이 없으면 아무것도 안 함)
        self.planner, self.design_id, self.part_id, self.goal = None, None, None, None
        self._assembling = False
        self._cleaned = True
        self._clear_run()
        self._publish('stopped', '정리를 확인했어요. 다 치웠으면 [정리 끝]을 누르세요 — 대기로 돌아갑니다')
        return True, ''

    def _gripper_open(self):
        """화면 gripper_open: STOPPED 에서만 그리퍼 열기를 보낸다(사람이 쥔 블록을 빼기 위해, W183). 반환: (접수했나, reason). 팔은 움직이지 않는다.

        폭 · 힘은 복구 절차(docs 복구절차_정지뒤다시시작 4절 2)와 같은 GRIPPER_OPEN_M · robot.yaml gripper.force_n.
        접수되면 곧바로 (True, '')로 답하고(10/13 PL — 그리퍼가 다 멈출 때까지 화면 명령을 붙잡지 않는다), 실제 결과는 그리퍼 서버의 답이 올 때
        _gripper_done 이 gripper_done 알림으로 낸다. 그때까지 _gripper_busy 를 유지하며 state/2 는 모두 message_id gripper_opening 으로 나간다
        (retained 마지막 값으로 다시 붙은 화면도 대기 중임을 안다). 대기 중 반복 open · select_design · start · scan 은 BUSY,
        조립 RECOVER 는 CHECK 로 가지 않고 기다린다. 정지 · resume · cancel 은 막지 않는다. 상태 · 안전 잠금 · 조립 기록은 바꾸지 않는다.
        보내지 못하면(서버 없음 · 예외) 바로 gripper_done(실패)을 내고 (False, reason).
        답이 끝내 오지 않으면 대기(차단)를 스스로 풀지 않는다 — 사람이 그리퍼 장비를 확인한 뒤 task 를 다시 켠다(10/13 19:45 PL, 복구 절차 4-1).
        """
        with self._lock:
            if self.state != 'STOPPED':
                return self._reject(self._busy_reason(), '그리퍼 열기는 정지 상태에서만 할 수 있다')
            if self._gripper_busy:
                return self._reject('BUSY', '그리퍼가 이미 열리는 중이에요. 완료 안내를 기다리세요')
            self._gripper_busy = True
            self._gripper_seq += 1
            seq = self._gripper_seq
            self._publish('gripper_opening', '그리퍼를 여는 중이에요. 완료 안내까지 기다리세요')
        try:
            accepted, why = self.io.gripper_open(GRIPPER_OPEN_M, self.cfg['gripper']['force_n'],
                                                 lambda done_ok, done_why: self._gripper_done(seq, done_ok, done_why))
        except Exception as e:                 # noqa: BLE001 — 보내지도 못했다: 답이 올 일이 없으니 바로 푼다
            accepted, why = False, f'ERROR({e!r})'
        if not accepted:
            self._gripper_done(seq, False, why or 'ERROR')
            return False, ('ERROR' if (why or '').startswith('ERROR') else (why or 'ERROR'))
        return True, ''

    def _gripper_done(self, seq, ok, why):
        """그리퍼 서버의 실제 답(또는 보내기 실패)을 받았다 — 대기를 풀고 gripper_done 을 알린다. 한 요청에 한 번만, 늦게 와도 같다."""
        with self._lock:
            if not self._gripper_busy or seq != self._gripper_seq:
                return
            self._gripper_busy = False
            self.logger.log('gripper_open', None, 'ok' if ok else (why or 'ERROR'))
            self._publish('gripper_done', '그리퍼를 열었어요' if ok else f'그리퍼 열기 실패: {why or "이유 없음"}')

    def _cancel_to_idle(self):
        """(잠금 안) READY·ERROR·SCAN_REVIEW 에서 취소해 IDLE 로 간다. 반환: 취소했나. 화면 cancel 과 음성 cancel 이 같이 쓴다(S-16).

        ERROR 에서는 조립 기록을 ERROR 로 닫는다. 운전 중에는 받지 않는다. 로봇은 움직이지 않는다.
        """
        if self.state not in ('READY', 'ERROR', 'SCAN_REVIEW'):
            return False
        if self.state == 'ERROR':
            self._finish_run('ERROR')
        self.planner, self.design_id, self.part_id = None, None, None
        self._clear_run()
        self._set('IDLE', None, '취소했다')
        return True

    # ---------- 작업 스레드 ----------
    def run_once(self):
        """지금 상태의 일을 한 단계만 하고 돌아온다(작업 스레드가 되풀이해서 부른다).

        정지 신호를 매번 먼저 본다. WAIT_HMI 는 서기 정지가 아니므로 현재 액션은 중단하지 않는다.
        단계 안에서 예상 못 한 예외가 나면 로그를 남기고 ERROR 로 간다(팔은 pick_place 가 서 있게 한다).
        """
        self._apply_safety()
        self.poll_hmi()
        with self._lock:
            due, self._w144_due = self._w144_due, None
        if due is not None:
            # W144 녹화 닫기(rosbag 정상 종료 · ZIP 은 수십 초)는 잠금 밖에서 — 잠금 안에서 기다리면 그동안 정지 신호 · 명령 처리가 막힌다(#152 리뷰 3)
            self.io.w144_finish(due)
        with self._lock:
            if self._hmi_pause and self.state in ('CHECK', 'SELECT', 'PICK_PLACE', 'WAIT_SUPPLY'):
                self._enter_wait_hmi()
        with self._lock:
            handler = self._handlers.get(self.state)
            if handler is None:
                return
            # 정지 직후에도 이 단계는 끝까지 돈다(늦은 결과를 STOPPED 로 정리). 그 사이 정리 취소가 goal · planner 를 비우면 단계가 깨지므로 표시해 둔다
            self._in_handler = self.state != 'STOPPED'
        try:
            handler()
        except Exception as e:        # noqa: BLE001 — 어떤 오류든 멈추고 사람을 부른다
            LOG.exception('상태 %s 에서 예상 못 한 오류', self.state)
            self._to_error(f'예상 못 한 오류로 작업을 멈췄어요({type(e).__name__}). 로봇 담당자에게 알려 주세요')
        finally:
            with self._lock:
                self._in_handler = False

    # ---------- 상태별 일 ----------
    def _check(self):
        """CHECK: 관측 자세로 → 모든 블록을 진행 확인 → 어긋남이 있으면 ERROR, 없으면 SELECT.
        받침 경유 출발 직후에는 먼저 장면을 정리하고 홈으로 간다(_clean_start). W144 기록은 출발 직후 여기서(잠금 밖에서) 연다 —
        필수 자료가 없으면 움직이기 전에 ERROR(INTEGRATION.md). 출발 명령(잠금 안)에서 열면 토픽 확인 · 녹화 시작을 기다리는 동안
        정지 신호 처리가 막혔다(#152 리뷰 3)."""
        if self._w144_starting:
            self._w144_starting = False
            ok, why = self.io.w144_start(self.design_id, self.run_id)
            if not ok:
                # 덜 된 기록은 닫아 둔다(다음 run_once 가 잠금 밖에서 ZIP) — 열린 채 [다시 시작]으로 이어 가면 촬영이 반쯤 연 기록을 썼다(10/14 점검 ② 3)
                with self._lock:
                    self._w144_due = False
                return self._to_error(why)
        if self._fresh and self._scatter is not None and self._scatter.via_spare:
            self._fresh = False
            if not self._clean_start():
                return None
        ok, why = self._move_observe()
        if not ok:
            return self._failed(why, '관측 자세 이동')
        if not self._observe([b['part_id'] for b in self.planner.parts]):
            return None
        problems = self.planner.judge()
        if problems:
            return self._offset_over(problems)
        self._set('SELECT', None, '')
        return None

    def _clean_start(self):
        """흩뿌림 출발 직후(one_pick 시작과 같음): ① 장면에 남은 쥔 상자 떼기(앞 실행이 쥔 채 멈췄으면 모든 계획이 막혔다 — 10/12 실기)
        ② 홈 자세로 — 앞 실행의 놓인 블록이 남은 장면 그대로 계획한다(실물이 남아 있어도 피하게, #152 리뷰 2). 못 가면 평소 실패 처리(사람)이고
        장면을 비워 다시 가지 않는다 ③ 빈 진행표(parts [])로 장면 관리가 앞 실행의 놓인 블록을 지우게 하고 1 s 기다림(이 뒤 CHECK 가 진행을 다시 본다).
        반환: 이어 가도 되나(이동 실패면 처리 뒤 False)."""
        detach = getattr(self.io, 'scene_detach', None)
        if detach is not None and not detach():
            LOG.warning('출발: 장면 관리가 답하지 않아 쥔 상자를 못 뗐다 — 그대로 진행')
        ok, why = self.io.move_to('home', self.halted)
        if not ok:
            self._failed(why, '출발 홈 자세 이동')
            return ok
        self.io.publish_progress({'schema': 'progress/2.0', 'stamp': time.time(), 'design_id': self.design_id, 'run_id': self.run_id,
                                  'obs_stamp': time.time(), 'parts': []})
        self._sleep(1.0)
        return ok

    def _move_via(self, target):
        """받침 경유 자세 이동 — 길이 안 나오면(PLAN_FAILED) 홈을 거쳐 한 번 더(one_pick move). 반환: (ok, why)."""
        ok, why = self.io.move_to(target, self.halted)
        if not ok and why == 'PLAN_FAILED' and self._scatter.via_spare and not self.halted():
            self.logger.log('fail', self.part_id, f'{target} 이동 PLAN_FAILED — 홈 거쳐 다시')
            ok_h, why_h = self.io.move_to('home', self.halted)
            if not ok_h:
                return ok_h, why_h
            ok, why = self.io.move_to(target, self.halted)
        return ok, why

    def _find_via(self, **opts):
        """받침 경유 find_blocks — 실패하면 timeout.find_retries 번 다시(정지 · 중단이면 바로 끝). 반환: (ok, why, body)."""
        tries = 1 + int(self.cfg['timeout'].get('find_retries', 0)) if self._scatter.via_spare else 1
        for k in range(tries):
            ok, why, body = self.io.find_blocks(self.run_id, self.halted, **opts)
            if ok or self.halted() or why in STOP_REASONS:
                return ok, why, body
            if k + 1 < tries:
                self.logger.log('fail', self.part_id, f'find_blocks {why} — 다시({k + 1}/{tries - 1})')
        return ok, why, body

    def _hold(self, part_id, text, then=None, message_id='no_match_block'):
        """받침 경유: 서지 않고 사람을 부른다(one_pick hold) — WAIT_SUPPLY + 알림(기본 no_match_block), [계속]이면 then 을 한 뒤 이어 간다.

        message_id: 화면 알림 이름 — 받침 경유 · 공급은 기본 no_match_block 그대로, A4 모서리 판정의 사람 수정 대기만 placed_check(10/15 PL)."""
        self._hold_then = then
        self._enter_wait_supply(part_id, (message_id, text))
        return None

    def _move_observe(self):
        """관측 자세 이동 결과(ok, reason). 복구 뒤 첫 이동은 저속을 유지하고 성공한 뒤에만 해제한다."""
        if self._recover_slow:
            ok, why = self.io.move_to('observe', self.halted, speed_ratio=self.cfg['recover']['speed_ratio'])
        else:
            ok, why = self.io.move_to('observe', self.halted)
        if ok:
            self._recover_slow = False
        return ok, why

    def _select(self):
        """SELECT: 작업 판단에 다음 블록을 묻는다. 있으면 PICK_PLACE, 끝이면 DONE, 못 본 블록은 재관측 1번 뒤 ERROR."""
        if self._hmi_recheck:
            ok, why = self._move_observe()
            if not ok:
                return self._failed(why, '웹 재연결 뒤 관측 자세 이동')
            if not self._observe([b['part_id'] for b in self.planner.parts]):
                return
            # 받침 경유는 VERIFY 와 같은 엄격 판정(높이 place_dz_m · strict) — 놓은 뒤 확인을 기다리던 중 웹이 끊겼다 이어 가면
            # 느슨한 판정으로 고치지 않은 블록 위로 진행했다(10/14 점검 ② 2ⓐ). 걸리면 VERIFY 처럼 사람이 고치고 [계속]
            via = self._scatter is not None and self._scatter.via_spare
            dz = (self.cfg.get('scatter') or {}).get('place_dz_m') if via else None
            problems = self.planner.judge(expect_present=[self.placed] if self.placed else (), placed_dz_m=dz, strict=via)
            if problems and via and self.placed:
                self._hmi_recheck = False
                first = problems[0]
                return self._hold(self.placed, f'{first["part_id"]}: {first["detail"]} — 손으로 바로 놓은 뒤 [계속]을 누르면 다시 확인합니다',
                                  then='verify')
            if problems:
                return self._offset_over(problems)
            if self.placed:
                row = self.planner.progress[self.placed]
                self.logger.record_measure(self.placed, row['dx_m'], row['dy_m'], row['dz_m'])
            self._hmi_recheck = False
            self.poll_hmi()
            if self._hmi_pause:
                return self._enter_wait_hmi()
        r = self._next_goal()
        if r is None:                          # 흩뿌림은 전환까지 끝냈거나(집기 · 대기 · 오류) 이동 · 조회 실패를 처리했거나 정지 뒤 늦은 답을 버렸다
            return None
        status = r['status']
        if status == 'FOUND':
            self._start_goal(r)
            self._set('PICK_PLACE', None, f'{r["part_id"]} 를 집으러 갑니다')
        elif status == 'DONE':
            with self._lock:               # DONE 방송과 요약 확정 사이에 새 설계 선택 · 출발이 끼어들지 못하게 한 잠금 안에서. 요약 CSV 는 쓰기 스레드가 쓰지만, w144_record 가 켜져 있으면 w144_finish(녹화 종료 · ZIP)가 이 잠금 안에서 돈다(점검 ② 3번 — 박진용)
                self.part_id = self._pick = None      # 조립이 끝났다 — 손목 그림 집기 순위도 DONE 알림부터 뺀다(10/14 22시 PL)
                self._set('DONE', 'done', '조립이 끝났어요')
                self._finish_run('DONE')
        elif status == 'UNKNOWN_BLOCK' and not self._reobserved:
            self._reobserved = True
            self._set('CHECK', None, f'{r["part_id"]} 가 있는지 못 봐서 한 번 더 봅니다')
        elif status == 'WAIT_SUPPLY':
            self._enter_wait_supply(r['part_id'], r.get('alert'))
        else:                                  # NO_SUPPORT · 재관측 뒤에도 UNKNOWN_BLOCK
            self._to_error(f'{r["part_id"]}: 다음 블록을 정하지 못했어요({status}). 작업대를 확인하고 로봇 담당자에게 알려 주세요')

    def _next_goal(self, pick_message='{part} 를 집으러 갑니다'):
        """다음 집기 목표를 공급 방식에 맞게 정한다(목표 블록은 작업 판단이, 어디서 집을지는 방식이 정한다).

        slots: planner.next_block() 를 그대로 돌려준다(부르는 쪽이 목표를 적용한다).
        scatter: planner.next_target() 로 블록을 정하고, 집을 블록이 있으면 _scatter_goal 이 관측 → 후보 고르기 → **목표 적용과 PICK_PLACE 전환까지**
        한 잠금 안에서 끝내고 None 을 돌려준다. 블록이 없는 경우(DONE · NO_SUPPORT · UNKNOWN_BLOCK)는 그 status dict 를 돌려준다.
        None = 이미 처리했다(전환 · 이동/조회 실패 · 설정 미완 · 정지 뒤 늦은 답 버림) — 부르는 쪽은 아무것도 더 하지 않는다.
        """
        if self._scatter is None:
            return self.planner.next_block()
        target = self.planner.next_target()
        if target['status'] != 'FOUND':
            return target
        target = self._spread_target(target)          # 같은 층 1 mm 띄우기 — 흩뿌림이면 늘(one_pick)
        self._scatter_goal(target, pick_message)
        return None

    def _spread_target(self, target):
        """흩뿌림 조립이면(받침 경유 여부와 상관없이 — 호출 쪽 _select) 목표의 놓는 중심을 같은 층 1 mm 띄운 값으로 바꾼 복사본(pick_rank.spread_parts — 설계 25 mm 간격에 실제 폭 24.8 mm 라
        틈 0.2 mm 뿐이어서 놓을 때 옆 블록을 밀어 줄이 틀어졌다, 10/13 실기). 진행표 · 확인(check_progress)은 설계 중심 그대로."""
        if self._spread is None:
            from d2_task.pick_rank import spread_parts
            centers = spread_parts(self.planner.parts, self.cfg)
            self._spread = {p['part_id']: c for p, c in zip(self.planner.parts, centers)}
        c = self._spread.get(target['part_id'])
        if c is None:
            return target
        return dict(target, place_pose=(tuple(c), target['place_pose'][1]))

    def _scatter_goal(self, target, pick_message):
        """흩뿌린 공급에서 target 블록을 집을 후보를 고르고 결과를 적용한다: 설정 점검 → observe_supply 이동 → find_blocks → ScatterFlow 고르기.

        설정(열림 폭)이 안 정해졌으면 **로봇을 움직이기 전에** ERROR 로 끝낸다. 이동 · 조회는 잠금 밖에서 기다린다.
        적용(commit)과 그 결과(목표 적용 · PICK_PLACE 또는 WAIT_SUPPLY 전환)는 **한 잠금 안에서** 한다 — 정지 신호(on_safety)는 같은 잠금을 얻어야
        반영되므로 commit 확인 뒤 전환 전에 정지가 끼지 못한다. commit 은 요청 번호 · 상태 변화 · 정지 횟수 · 정지 중 여부를 그 안에서 다시 확인한다
        (정지가 끼었거나, 정지 뒤 [다시 시작]까지 끝났어도 그 전 요청의 늦은 답은 적용하지 않는다). 로봇은 move_to 외에 움직이지 않는다.
        공급 관측 결과를 적용하면 state/2 pick(손목 그림 집기 순위)도 그 결과의 것으로 바꾼다(받침 위 재기는 그대로 — 10/14 22시 PL).
        """
        if not self._scatter.configured:
            self._to_error('흩뿌린 공급 설정(열림 폭)이 없어 집을 수 없어요. 로봇 담당자에게 설정 확인을 부탁하세요')
            return
        stops_before = self._stop_count
        ticket = self._scatter.begin((self._epoch, stops_before))
        from_spare = self._scatter.via_spare and self._spare_on is not None
        # 받침에 블록이 있으면 받침 위 촬영 자세에서 그 블록만 깊이로 잰다(받침은 공급 영역 밖이라 공급 관측으로는 안 보인다)
        ok, why = self._move_via('observe_spare' if from_spare else 'observe_supply')
        if not ok:
            self._failed(why, '받침 위 촬영 자세 이동' if from_spare else '공급 관측 자세 이동')
            return
        if self._scatter.via_spare:
            # 받침: 깊이로 블록 하나 / 공급: 고르기가 손가락 자리를 깊이로 미리 거르게 높이 지도도 받는다
            opts = {'area': 'spare'} if from_spare else {'height_map': True}
            ok, why, body = self._find_via(**opts)
        else:
            ok, why, body = self.io.find_blocks(self.run_id, self.halted)
        if not ok or self.halted():
            if self._scatter.via_spare and not self.halted() and why not in STOP_REASONS:
                # 로봇은 서 있다(찍기만 실패) — 정지 요청 없이 사람이 카메라 · 인식 노드를 보고 [계속](one_pick hold)
                self._hold(target['part_id'], f'{"받침 위 블록" if from_spare else "공급 영역"}을 카메라로 못 쟀어요({why}) — 손목 카메라 · '
                                              '손목 블록 인식 노드를 확인하고 [계속]을 누르면 다시 잽니다')
                return
            self._failed('STOPPED' if self.halted() else why, 'find_blocks 조회')
            return
        if from_spare:
            result = self._scatter.prepare_from_spare(target, body, self._spare_centers)
            if 'offset_mm' in result:                # 맞춘 뒤 다시 잰 어긋남 — 왜 못 맞추는지 숫자로 보려고 늘 남긴다(10/14 실기)
                self.logger.log('spare', target['part_id'], f'받침 맞춤 {self._spare_centers}번 뒤 다시 잼: 받침 자리에서 {result["offset_mm"]} mm')
        elif self._scatter.via_spare:
            self._spare_seen = (target, body)
            result = self._scatter.prepare_spare(target, body, self._spare_failed)
        else:
            result = self._scatter.prepare(target, body)
        with self._lock:
            applied = self._scatter.commit(ticket, lambda: ((self._epoch, self._stop_count), self.halted()), result)
            if applied:
                if not from_spare:
                    # 공급 그림의 새 결과가 손목 그림 집기 순위를 바꾼다 — 순위로 고른 결과(SPARE)만 pick 이 있고, 못 고른 결과면 비운다.
                    # 받침 위 재기는 공급 그림이 아니라 그대로 둔다(10/14 22시 PL). 상태 전환 전에 바꿔야 그 state/2 부터 실린다
                    self._pick = copy.deepcopy(result.get('pick'))
                self._apply_scatter_result(target, result, pick_message)
                return
        if self._stop_count != stops_before:       # 조회 중에 정지가 있었다(이미 [다시 시작]까지 끝났어도) → 다른 정지 때처럼 STOPPED 절차로
            self._failed('STOPPED', 'find_blocks 조회 중 정지')

    def _apply_scatter_result(self, target, result, pick_message):
        """(잠금 안) commit 이 적용된 흩뿌림 결과를 상태에 반영한다: PICK → 목표 + PICK_PLACE, 비었음 · 맞는 블록 없음 → WAIT_SUPPLY, 그 밖은 ERROR."""
        status, part_id = result['status'], target['part_id']
        if status in ('PICK', 'SPARE', 'SPARE_TIP', 'SPARE_CENTER', 'SPARE_STAND'):
            self._w144_event('candidate_selected', part_id, status=status, tag=result.get('tag'), candidate=result.get('candidate'))
        if status == 'PICK' and self._spare_on is not None:      # 받침 경유 2단계 — 받침에서 레시피 잡기로 다시 집어 조립
            self._spare_empty = 0
            self._start_goal(dict(result['request'], status='FOUND', from_spare=True))
            self._set('PICK_PLACE', None, f'{part_id} 를 받침에서 다시 집어 조립합니다')
        elif status == 'PICK':
            self._start_goal(dict(result['request'], status='FOUND'))
            self._set('PICK_PLACE', None, pick_message.format(part=part_id))
        elif status == 'SPARE':                    # 받침 경유 1단계 — 집기 쉬운 블록을 받침에 눕힌다(조립으로 세지 않음)
            self._no_match = self._bad_blocks = 0
            self._start_goal(dict(result['request'], status='FOUND', attempt=result['attempt'], seen=result['candidate']))
            self._set('PICK_PLACE', None, f'{part_id} 에 쓸 블록을 받침에 옮깁니다({result["tag"]})')
        elif status == 'SPARE_TIP':                # 받침에 옆으로 · 세워 놓은 블록 — 잡기 TIP 으로 기울여 눕힌다(조립 아님, 눕힌 뒤 다시 잰다)
            self._spare_empty = 0
            self._start_goal(dict(result['request'], status='FOUND'))
            self._set('PICK_PLACE', None, f'{part_id} 에 쓸 받침 위 블록({result["candidate"].get("up")})을 눕힙니다')
        elif status == 'SPARE_CENTER':             # 받침 맞추기 — 잰 자리에서 반대 폭으로 집어 받침 가운데에 놓는다(끝나면 다시 잼)
            self._spare_empty = 0
            self._start_goal(dict(result['request'], status='FOUND'))
            self._set('PICK_PLACE', None, f'{part_id} 를 받침에서 가운데로 맞춥니다')
        elif status == 'SPARE_STAND':              # 받침 가운데에 맞은 눕힌 블록을 세우기 교시 동작으로 세운다(세운 뒤 다시 잼)
            self._spare_empty = 0
            self._start_goal(dict(result['request'], status='FOUND', from_spare=True))
            self._set('PICK_PLACE', None, f'{part_id} 에 쓸 받침 위 블록을 세웁니다')
        elif status == 'SPARE_EMPTY':
            # 방금 받침에 놓은 블록이 안 보인다 — 비었다고 보고 다음 블록을 그 위에 놓으면 부딪힌다(one_pick 10/13 가상). 사람이 본다:
            # 1번째는 받침을 확인하고 [계속]이면 다시 잰다, 연달아 2번째면 받침 기록을 지우고 [계속]이면 공급에서 다시 고른다
            self._spare_empty += 1
            if self._spare_empty >= 2:
                self.logger.log('spare', part_id, '받침이 두 번 연달아 비어 보임 — 사람 확인 뒤 공급에서 다시')
                self._spare_on, self._spare_empty = None, 0
                self._enter_wait_supply(part_id, ('no_match_block', '받침 위 블록이 그래도 안 보여요 — 받침이 정말 비었으면 [계속](공급에서 '
                                                                    '다시 고름), 블록이 있으면 치운 뒤 [계속]을 누르세요'))
            else:
                self.logger.log('spare', part_id, '받침 위 블록이 안 보임 — 사람 확인')
                self._enter_wait_supply(part_id, ('no_match_block', '받침에 놓은 블록이 안 보여요 — 받침을 확인하고 [계속]을 누르면 다시 잽니다'))
        elif status == 'SPARE_BLOCKED':            # 받침 둘레 손가락 자리에 뭔가 있다(깊이) — 사람이 비우고 [계속]이면 다시 잼(one_pick p1_to_recipe)
            self._hold(part_id, f'받침 둘레 손가락 자리에 물건이 있어요({result["message"]}) — 받침 둘레를 비우고 [계속]을 누르세요')
        elif status == 'STAND_NOT_READY':
            self._to_error(f'{part_id}: 이 자세(세움 · 옆세움)로 세우는 교시 동작이 robot.yaml standup_taught 에 없다(세우기 W131) — 사람이 확인해 주세요')
        elif status == 'SPARE_NOT_FLAT':
            self._to_error(f'{part_id}: 받침 위 블록 자세를 모른다({result["block"].get("up")}) — 받침 위 블록을 확인해 주세요')
        elif status == 'EMPTY':
            self._enter_wait_supply(part_id, ('supply_empty', '공급 영역에 블록이 없어요. 블록을 흩뿌린 뒤 [계속]을 누르세요'))
        elif status == 'NO_MATCH' and self._scatter.via_spare and self._no_match == 0:
            self._no_match = 1                     # 찍을 때마다 판정이 조금씩 달라 한 번은 다시 찍어 본다(one_pick)
            self._set('SELECT', None, f'{part_id}: 손가락이 들어갈 블록이 없어 한 번 더 찍습니다')
        elif status == 'NO_MATCH':
            self._no_match = 0
            if self._scatter.via_spare:
                self._spare_failed.clear()         # 사람이 흩은 뒤에는 처음 후보부터(one_pick '흩어 주세요' 뒤 failed 비움)
            counts = result['counts']
            # 기울어진 블록이 있고 제외 이유가 tilted · under 뿐이면 IRD 알림 tilted_block(E-68 — 덮인 블록이 같이 있어도 사람이 기울어진 것을 바로 놓으면 된다).
            # under 만이거나 다른 제외 이유가 섞이면 no_match_block(E-73) — 이유는 글자(message)로 같이 보낸다
            only_tilted = counts.get('tilted', 0) > 0 and all(n == 0 for k, n in counts.items() if k not in ('tilted', 'under'))
            self._enter_wait_supply(part_id, ('tilted_block' if only_tilted else 'no_match_block', result['message'] + ' — 정리한 뒤 [계속]을 누르세요'))
        elif status == 'NOT_CONFIGURED':
            self._to_error('흩뿌린 공급 설정(열림 폭)이 없어 집을 수 없어요. 로봇 담당자에게 설정 확인을 부탁하세요')
        elif status == 'BLOCKED' and self._scatter.via_spare and self._bad_blocks < PLAN_STREAK - 1:
            self._bad_blocks += 1                  # 블록 값이 잘못 나오면 다시 찍는다(one_pick — 3번째에 사람)
            self._set('SELECT', None, f'{result["message"]} — 다시 찍습니다')
        elif status == 'BLOCKED':                  # 장애물 블록 중 형식이 틀린 것이 있다(E-68) — 일부만 보내지 않고 서서 사람을 부른다
            self._bad_blocks = 0
            self._to_error(result['message'])
        else:                                      # LOOKUP_FAILED — 횟수 상한 정책이 정해지기 전이라 다시 시도하지 않고 사람을 부른다
            self._to_error(f'손목 카메라의 블록 찾기 결과를 쓸 수 없어요({result.get("reason", status)}). 손목 카메라 기능을 확인하고 비전 담당에게 알려 주세요')

    def _pick_place(self):
        """PICK_PLACE: 목표 하나를 pick_place 로 보내고 결과를 SDD 7.1 표대로 처리한다.

        요청마다 새 번호를 만들어 피드백 step · 결과 수신 시각(time.time())을 모으고, 끝나면 self.last_pick 에 남긴다
        ({request_id, ok, reason, step, result_at} — result_at 이 None 이면 결과 없이 끝난 것). 흩뿌림 집기가 실패하면 _scatter_failed 가 step(접촉 전후)과 result_at(gripper 상태가 결과 이후인지)으로 재시도 여부를 정한다.
        """
        a4 = self.goal.get('a4_regrasp')      # W144 A4 다시 집기('regrasp' 는 받침 경유 헛잡음 다시 집기가 이미 쓰는 키 — 섞지 않음)
        via = self._scatter is not None and self._scatter.via_spare and not a4
        if self._scatter is not None and not via and not a4:   # W144 A4 다시 집기는 따로 센다(_a4_tries)
            part = self.goal['part_id']
            with self._lock:
                if self._scatter_fail.get(part, 0) >= self.max_attempts:      # 복구 · 다른 실패 경로로 와도 한도를 넘는 요청은 보내지 않는다
                    return self._to_error(f'{part}: 집기 {self.max_attempts}번 시도를 이미 했다 — 사람이 확인해 주세요')
                self._scatter_fail[part] = self._scatter_fail.get(part, 0) + 1     # 요청을 보내기 시작할 때 센다(성공 · 실패 · 시간 초과 모두 한 번)
        aid = self._w144_before_pick()
        if aid is False:                       # W144 시도 기록이 안 됨 — 집기를 시작하지 않는다(INTEGRATION.md)
            return self._failed('ERROR', f'{self.goal["part_id"]} W144 시도 기록')
        self._pick_seq += 1
        rid = self._pick_seq
        self._steps.begin(rid)                 # 이 요청의 피드백 · 결과만 받는다. 이전 요청의 늦은 것은 번호가 달라 버려진다
        ok, why = self.io.pick_place(self.goal, self.halted,
                                     on_feedback=lambda step: self._steps.on_feedback(rid, step),
                                     on_result=lambda at: self._steps.on_result(rid, at))
        # 결과 처리에 넘길 수집값. result_at 이 None 이면 결과를 못 받고 끝난 것(거절 · 시간 초과 · 우리 쪽 취소) — 이유(why)로 어느 경우인지 가른다
        self.last_pick = {'request_id': rid, 'ok': ok, 'reason': why,
                          'step': self._steps.last_step(rid), 'result_at': self._steps.result_at(rid)}
        self._steps.end(rid)                   # 이 번호의 늦은 피드백이 다음 요청에 섞이지 않게 닫는다
        self._w144_after_pick(aid, ok, why)
        self._tally['sent'] += 1
        self._tally['ok'] += 1 if ok else 0
        self.logger.log('attempt', self.goal['part_id'], f'{self.goal.get("grasp")} {"성공" if ok else why}'
                        f'{"" if ok or self.last_pick["step"] is not None else "(움직이기 전)"} · 지금까지 {self._tally["ok"]}/{self._tally["sent"]}')
        if a4:
            return self._regrasp_result(ok, why)
        if via:
            handled = self._via_spare_result(ok, why)
            if handled is not False:
                return handled
        if ok:
            self.placed = self.goal['part_id']
            self._a4_place[self.placed] = copy.deepcopy(self.goal.get('place_pose'))     # W144 A4 다시 놓을 자리 = 이번에 실제로 보낸 목표
            self.logger.record_placed(self.placed, self.goal['supply_slot'])
            self.poll_hmi()
            if self._hmi_pause:
                return self._enter_wait_hmi()
            self._set('VERIFY', None, f'{self.placed} 를 놓았다. 확인합니다')
            return None
        self.logger.log('fail', self.goal['part_id'], f'pick_place {why}')
        if self._scatter is not None and why in ('PLAN_FAILED', 'GRASP_FAILED', 'SLOT_EMPTY'):
            return self._scatter_failed(why)
        if why == 'PLAN_FAILED':
            if self._plan_retry < 1:           # 같은 목표로 다시 계획 1번
                self._plan_retry += 1
            else:
                self._to_error(f'{self.goal["part_id"]}: 로봇이 갈 길을 찾지 못했어요(PLAN_FAILED, 다시 계획해도 실패). 작업대 위 물건을 확인하고 로봇 담당자에게 알려 주세요')
        elif why == 'GRASP_FAILED':
            self._grasp_failed()
        elif why == 'SLOT_EMPTY':
            self._slot_empty()
        else:
            # TIMEOUT 은 정지 노드에 요청하고 상태를 확인한다. 그 밖의 정식 코드 · 모르는 코드는 공통 분기를 탄다
            self._failed(why, f'{self.goal["part_id"]} 집기·놓기', log=False)

    def _via_spare_result(self, ok, why):
        """받침 경유 PickPlace 결과 처리(10/13 — one_pick 실기 규칙). 반환: False = 처리 안 함(부르는 쪽이 평소대로 — 조립 성공 · 모르는 이유),
        그 밖 = 이 함수가 상태를 정했다. 규칙: 팔이 움직인 실패만 센다(받침에 옮기기는 블록당 max_attempts 번 · 받침에서 다시 집기는 따로
        max_attempts 번), 움직이기 전 길 계산 거절은 세지 않고 다음 후보로 — 연달아 PLAN_STREAK 번이면 사람을 부른다."""
        part, goal = self.goal['part_id'], self.goal
        moved = self.last_pick['step'] is not None
        if goal.get('spare_center') and ok:
            # 맞추기 = 받침 가운데로 옮겨 놓기(10/14) — 다시 재서 2 mm 안인지 본다(scatter_pick.CENTER_TOL_M). 맞춘 뒤 다시 안 재고 세웠더니
            # 남은 어긋남을 몰랐다(10/14 실기: 맞추기 → 세우기 뒤 4.7 mm)
            self._spare_centers += 1
            self.poll_hmi()
            if self._hmi_pause:
                return self._enter_wait_hmi()
            return self._set('SELECT', None, f'{part} 받침 맞춘 자리를 다시 잽니다')
        if goal.get('spare_center') and why == 'PLAN_FAILED' and not moved:
            # 움직이기 전에 길이 안 나왔으면 블록은 잰 자리 그대로라 잰 값으로 레시피 잡기(skip). 세우기는 교시 경로가 받침 가운데 기준이라
            # 건너뛸 수 없다 → 사람
            if goal.get('skip') is None:
                return self._hold(part, f'{part}: 받침 위 블록을 가운데로 못 옮겨 세우지 못해요(움직이기 전 길 거절) — 블록을 받침 가운데에 '
                                        '두고 [계속]을 누르면 다시 잽니다')
            self.logger.log('spare', part, '받침 맞추기 길이 안 나와 건너뜀')
            self._start_goal(dict(goal['skip'], status='FOUND', from_spare=True))
            return self._set('PICK_PLACE', None, f'{part} 를 받침에서 다시 집어 조립합니다')
        if ok and goal.get('spare_stand'):         # 받침 위 블록을 세움(W131 교시 동작) — 조립이 아니다. 다음 SELECT 가 받침을 다시 재서 레시피 잡기로 집는다
            self._plan_streak = 0
            self.logger.log('spare', part, '받침 위 블록을 세움')
            self.poll_hmi()
            if self._hmi_pause:
                return self._enter_wait_hmi()
            return self._set('SELECT', None, f'{part} 에 쓸 받침 위 블록을 세웠다. 다시 잽니다')
        if ok and goal.get('spare_tip'):           # 받침 위 블록을 눕힘 — 조립이 아니다. 다음 SELECT 가 받침을 다시 잰다
            self._plan_streak, self._spare_centers = 0, 0
            self.logger.log('spare', part, '받침 위 블록을 눕힘')
            self.poll_hmi()
            if self._hmi_pause:
                return self._enter_wait_hmi()
            return self._set('SELECT', None, f'{part} 에 쓸 받침 위 블록을 눕혔다. 다시 잽니다')
        if ok and goal.get('via_spare'):           # 받침에 눕힘 — 조립이 아니다(placed · 기록 · VERIFY 없음). 다음 SELECT 가 받침을 잰다
            self._spare_on, self._plan_streak, self._spare_centers = part, 0, 0
            self._spare_failed.clear()
            self.logger.log('spare', part, '받침에 눕힘')
            self.poll_hmi()
            if self._hmi_pause:
                return self._enter_wait_hmi()
            return self._set('SELECT', None, f'{part} 에 쓸 블록을 받침에 놓았다. 받침 위에서 잽니다')
        if ok:                                     # 받침에서 다시 집어 조립 — 평소 성공 처리(placed · VERIFY)로
            self._spare_on, self._plan_streak, self._spare_centers = None, 0, 0
            return False
        self.logger.log('fail', part, f'pick_place {why}{"" if moved else "(움직이기 전)"}')
        if goal.get('via_spare') and self.last_pick['step'] in ('place', 'retreat'):
            # 받침에 내려놓는 중 · 뒤에 실패 · 정지 · 시간 초과 — 실물은 받침에 있을 수 있다. 기록을 남겨 다음 SELECT 가 받침부터 잰다
            # (비어 있으면 SPARE_EMPTY 사람 확인). 안 남기면 같은 자리에 또 놓을 수 있었다(#152 리뷰 1)
            self._spare_on = part
        if why == 'PLAN_FAILED' and not moved:
            self._plan_streak += 1
            if goal.get('via_spare'):
                self._spare_failed.append(tuple(goal['attempt']))
            if self._plan_streak >= PLAN_STREAK:
                self._plan_streak = 0          # 팔은 안 움직였다 — 서지 않고 사람이 보고 [계속]이면 다시 찍어 이어 감(one_pick hold)
                return self._hold(part, f'{part}: 움직이기 전 길 계산이 연달아 {PLAN_STREAK}번 거절됨 — 집기 노드 로그(놓는 자리 · 장면)를 '
                                        '확인하고 [계속]을 누르세요')
            if goal.get('via_spare') and self._spare_seen is not None:
                return self._next_spare_candidate()    # 장면이 그대로라 다시 찍지 않고 같은 관측의 다음 후보(one_pick)
            return self._set('SELECT', None, f'{part}: 길이 안 나와 다음 후보로')
        self._plan_streak = 0
        # 팔이 움직인 실패는 레시피 블록 하나에 합쳐 센다(공급 → 받침 · 받침에서 눕히기 · 다시 집기 모두, one_pick tries) — max_attempts 면 사람
        if goal.get('via_spare'):
            self._spare_failed.append(tuple(goal['attempt']))
        self._scatter_fail[part] = self._scatter_fail.get(part, 0) + (1 if moved else 0)
        if why in ('PLAN_FAILED', 'GRASP_FAILED', 'SLOT_EMPTY'):
            return self._scatter_failed(why)
        return self._failed(why, f'{part} 받침 경유 집기·놓기', log=False)

    def _next_spare_candidate(self):
        """받침 경유: 움직이기 전에 길이 안 나온 뒤 다시 찍지 않고 마지막 공급 관측(_spare_seen)에서 다음 후보를 골라 적용한다(one_pick — 장면이 그대로).
        안 된 후보는 _spare_failed 에 이미 있다. 바깥 영향: 다음 PickPlace 목표 · 상태(로봇은 아직 안 움직임) · state/2 pick(같은 그림의 새 순위 ·
        남은 후보가 없으면 뺌)."""
        target, body = self._spare_seen
        result = self._scatter.prepare_spare(target, body, self._spare_failed)
        with self._lock:
            ticket = self._scatter.begin((self._epoch, self._stop_count))
            if self._scatter.commit(ticket, lambda: ((self._epoch, self._stop_count), self.halted()), result):
                self._pick = copy.deepcopy(result.get('pick'))     # 같은 공급 그림에서 다시 고른 순위(안 된 후보 빠짐, 10/14 22시 PL)
                if result['status'] == 'SPARE':
                    self._apply_scatter_result(target, result, '')
                    return None
                return self._set('SELECT', None, f'{target["part_id"]}: 이 관측에 남은 후보가 없어 다시 찍습니다')
        return None

    def _spare_regrasp(self, goal):
        """받침 경유 헛잡음 뒤 그 블록을 다시 찍어 위치를 고쳐 같은 잡기로 한 번 더(one_pick try_goal · see_block): 집기 위(노드가 열고 올라온 자리)에서
        find_blocks → 온전한 같은 자세 블록(확신도 0.5 이상 · 덮이지 않음 · 40 mm 안)이 없으면 공급 관측 자세로 가서 한 번 더.
        반환: 적용했으면 True(PICK_PLACE — 이 관측을 _spare_seen 으로 바꿔 둠), observe_supply 이동 · find_blocks 가 실패하면 평소 실패 처리
        (_failed — TIMEOUT 은 먼저 세움)를 하고 True, 못 찾았거나 정지 중이면 False(부르는 쪽이 평소대로). 바깥 영향: 팔 이동(observe_supply) · find_blocks."""
        token = (self._epoch, self._stop_count)
        cand = goal.get('seen') or {}                  # 고를 때 본 블록(헛잡음 판단 뒤에는 ScatterFlow 후보가 비워진다)
        for go in (False, True):
            if go:
                ok, why = self.io.move_to('observe_supply', self.halted)
                if not ok:
                    if self.halted():
                        return False
                    # 이동 실패(TIMEOUT 포함)를 넘기고 다시 찍으러 가면 팔이 어디 있는지 모른 채 이어 갔다(#152 리뷰 4)
                    self._failed(why, f'{goal["part_id"]} 헛잡음 뒤 공급 관측 자세 이동')
                    return True
            ok, why, body = self.io.find_blocks(self.run_id, self.halted, height_map=True)
            if self.halted():
                return False
            if not ok:
                # 조회 실패(TIMEOUT 포함)도 다른 조회처럼 평소 실패 처리 — 예전엔 정지 요청 없이 SELECT 로 갔다(10/14 점검 ② 2ⓑ)
                self._failed(why, f'{goal["part_id"]} 헛잡음 뒤 find_blocks')
                return True
            req = self._scatter.reseen_request(goal, cand, body)
            if req is not None:
                with self._lock:
                    if self.halted() or (self._epoch, self._stop_count) != token:
                        return False
                    if self._spare_seen is not None:
                        # 이 다시 잡기가 움직이기 전에 거절되면 '같은 관측의 다음 후보'를 이 관측에서 고른다 — 밀리기 전 옛 관측으로 고르면
                        # 빈 자리로 갔다(10/14 점검 ② 2ⓒ)
                        self._spare_seen = (self._spare_seen[0], body)
                    self.logger.log('spare', goal['part_id'], '헛잡음 — 다시 찍은 자리로 같은 잡기 한 번 더')
                    self._start_goal(dict(req, status='FOUND', regrasp=True))
                    self._set('PICK_PLACE', None, f'{goal["part_id"]}: 헛잡음 — 다시 찍은 자리로 한 번 더 집습니다')
                return True
        return False

    # ---------- W144 원인 추적(#140 인계 — 기록만, 집기 · 재시도 정책은 안 바꿈) ----------
    def _w144_event(self, event, part_id, **fields):
        """io.w144_event 가 있으면 events.jsonl 한 줄(부품의 열린 action_id · 원래 조립 순서를 붙임)."""
        if getattr(self.io, 'w144_event', None) is None:
            return
        part = next((b for b in self.planner.parts if b['part_id'] == part_id), {}) if self.planner else {}
        aid = getattr(self, '_w144_open', {}).get(part_id)
        self.io.w144_event(event, part_id=part_id, action_id=aid, sequence=part.get('sequence'),
                           retry_of=getattr(self, '_w144_last', {}).get(part_id) if aid else None, **fields)

    def _w144_leg(self):
        """지금 목표가 받침 경유의 어느 단계인가 — 공급→받침 · 받침 눕히기 · 받침 맞추기 · 받침 세우기 · 받침→조립 · 바로 조립."""
        g = self.goal
        return ('to_spare' if g.get('via_spare') else 'spare_tip' if g.get('spare_tip') else 'spare_center' if g.get('spare_center')
                else 'spare_stand' if g.get('spare_stand')
                else 'from_spare' if g.get('from_spare') else 'direct')

    def _w144_before_pick(self):
        """집기 명령 직전: 이 부품의 열린 시도가 없으면 begin_attempt(재시도면 retry_of) → pick_requested 이벤트.
        반환: action_id · None(기록 꺼짐) · False(시도 기록 실패 — 집기를 시작하지 않는다)."""
        if getattr(self.io, 'w144_begin', None) is None or not hasattr(self, '_w144_open'):
            return None
        g, part_id = self.goal, self.goal['part_id']
        part = next((b for b in self.planner.parts if b['part_id'] == part_id), None)
        aid = self._w144_open.get(part_id)
        if aid is None and part is not None:
            evidence = {'source': {'candidate': g.get('seen'), 'leg': self._w144_leg()},
                        'grasp': {'grasp': g.get('grasp'), 'open_width_m': g.get('open_width_m')},
                        'runtime': {'run_id': self.run_id, 'design_id': self.design_id}}
            aid, why = self.io.w144_begin(part_id, part.get('sequence'), part['center'], part['quat'], evidence,
                                          retry_of=self._w144_last.get(part_id))
            if why:
                return False
            if aid is not None:
                self._w144_open[part_id], self._w144_legs[aid] = aid, []
        self._w144_event('pick_requested', part_id, leg=self._w144_leg(), grasp=g.get('grasp'), pick_pose=g.get('pick_pose'),
                         place_pose=g.get('place_pose'), open_width_m=g.get('open_width_m'), obstacles=len(g.get('obstacles') or []),
                         tcp_target=None, tcp_note='TCP 목표 · 실제 TCP 는 task 가 받지 않음 — 집기 노드 로그 · bag /tf 로 본다')
        return aid

    def _w144_after_pick(self, aid, ok, why):
        """집기 · 놓기 결과: place_result 이벤트 + 단계 기록. 실패면 failed_attempt(그 시도를 닫고 다음 시도는 retry_of 로 잇는다)."""
        if not hasattr(self, '_w144_open'):
            return
        part_id = self.goal['part_id']
        leg = {'leg': self._w144_leg(), 'grasp': self.goal.get('grasp'), 'pick_pose': self.goal.get('pick_pose'),
               'place_pose': self.goal.get('place_pose'), 'ok': ok, 'reason': why or None,
               'last_step': self.last_pick.get('step'), 'result_unix_s': self.last_pick.get('result_at')}
        if aid is not None:
            self._w144_legs.setdefault(aid, []).append(leg)
        self._w144_event('place_result', part_id, **leg)
        if not ok and aid is not None and getattr(self.io, 'w144_fail', None) is not None:
            part = next((b for b in self.planner.parts if b['part_id'] == part_id), {})
            self.io.w144_fail(aid, part_id, part.get('sequence'), why, self._w144_evidence(aid))
            self._w144_open.pop(part_id, None)
            self._w144_last[part_id] = aid

    def _w144_evidence(self, aid):
        """after_place · failed_attempt 에 넣을 실행 증거 — 단계별 요청 블록 자세 · 결과 · 마지막 피드백 단계. TCP 는 null(task 가 받지 않음)."""
        return {'motion': list(getattr(self, '_w144_legs', {}).get(aid, [])) if aid else [],
                'release': None, 'tcp': None, 'tcp_note': 'TCP 목표 · 실제 TCP 는 집기 노드 로그 · bag /tf 로 본다'}

    def _verify(self):
        """VERIFY: 관측 자세에서 방금 놓은 블록과 그 받침만 다시 보고, 없거나 어긋났으면 ERROR."""
        ok, why = self.io.move_to('observe', self.halted)
        if not ok:
            return self._failed(why, '관측 자세 이동')
        part = next(b for b in self.planner.parts if b['part_id'] == self.placed)
        if not self._observe([self.placed] + part['supports']):
            return None
        row = self.planner.progress[self.placed]      # 판정 전에 측정값부터 남긴다(어긋나서 ERROR 가 되어도 값은 기록)
        self.logger.record_measure(self.placed, row['dx_m'], row['dy_m'], row['dz_m'])
        via = self._scatter is not None and self._scatter.via_spare
        dz = (self.cfg.get('scatter') or {}).get('place_dz_m') if via else None
        problems = self.planner.judge(expect_present=[self.placed], placed_dz_m=dz, strict=via)
        if problems and via:
            # 받침 경유: 사람이 손으로 고치고 [계속]이면 다시 확인(one_pick placed_check — 서지 않음)
            first = problems[0]
            return self._hold(self.placed, f'{first["part_id"]}: {first["detail"]} — 손으로 바로 놓은 뒤 [계속]을 누르면 다시 확인합니다', then='verify')
        if problems:
            return self._offset_over(problems)
        if self.planner.progress[self.placed]['state'] == 'present':     # 오류가 없다는 것만으로는 부족하다 — 놓인 것이 확인된 블록의 시도 횟수만 지운다
            self._scatter_fail.pop(self.placed, None)
        if self._measure_placed(part):
            return None                            # 측정 시간 초과(정지 요청 → ERROR) 또는 측정 중 정지 — 다음 단계로 가지 않는다
        if self._a4_regrasp(part):
            return None                            # 자동 다시 집기(PICK_PLACE) 또는 사람 확인(WAIT_SUPPLY)으로 갔다
        if getattr(self.io, 'w144_after_place', None) is not None:
            # W144: 최종 자리 확인이 끝난 블록을 observe 에서 촬영(one_pick final_placed — 설계 중심 · 방향). 촬영 실패면 io 가 세우고 ERROR
            aid = self._w144_open.pop(self.placed, None) if hasattr(self, '_w144_open') else None
            ok, why = self.io.w144_after_place(self.placed, part['center'], part['quat'], action_id=aid, sequence=part.get('sequence'),
                                               evidence=self._w144_evidence(aid))
            if not ok:
                return self._failed(why, 'W144 촬영')
        self._set('SELECT', None, '')
        return None

    def _measure_placed(self, part):
        """W144 A4(E-119): VERIFY 를 통과한 방금 놓은 블록만 find_blocks area assembly 로 다시 재서 기록한다(기본 꺼짐 — io 가 None 을 주면 아무것도 안 함).

        기록만 한다 — 판정 · 다음 블록은 바꾸지 않고, 못 잰 것(실패 · 거절 · 이상 응답 · 예외)은 UNMEASURED 로 적고 이어 간다(다시 집기는 측정 검증 뒤 따로).
        시간 초과는 다른 조회와 같이 기존 TIMEOUT 경로(정지 요청 → 정지 확인 대기 ERROR, 10/14 16:32 확정)로 보낸다.
        조회 중 정지 · 다른 run 으로 바뀜이면 답을 버린다(새 run 에 옛 측정을 적지 않음).
        반환: True = 측정이 흐름을 멈췄다(TIMEOUT · 정지) — 부르는 쪽은 다음 단계로 가지 않는다. False = 이어 가도 된다.
        바깥 영향: find_blocks 조회 1번(로봇 안 움직임 — 손목은 이미 observe), CSV 'a4_measure' 한 줄 · W144 events 'placed_measured' 한 줄.
        """
        fn = getattr(self.io, 'w144_measure', None)
        if fn is None:
            return False
        with self._lock:
            token = (self._epoch, self._stop_count, self.run_id, self.placed)
            design_id = self.design_id
            self._a4_last = None
        t_req = time.time()                         # 측정 최신성 비교 기준(비전 measured_stamp 는 같은 로봇 PC 시계)
        try:
            res = fn(token[2], design_id, token[3], self.halted)     # run_id · part_id · design_id 를 같은 잠금 시점 값으로
        except Exception as e:                     # noqa: BLE001 — 기록용 측정이 조립을 멈추게 하지 않는다(못 잰 것으로 적음)
            res = (False, 'ERROR', {'detail': f'{type(e).__name__}: {e}'})
        t_resp = time.time()
        if res is None:                            # 꺼짐(task_node w144_measure false)
            return False
        if not (isinstance(res, tuple) and len(res) == 3):
            res = (False, 'ERROR', {'detail': f'io 답 꼴이 이상함: {type(res).__name__}'})
        row = placed_offset(part['center'], token[3], *res)
        try:
            text = json.dumps(row, ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError):
            row, text = {'status': 'UNMEASURED', 'reason': 'BAD_RESPONSE', 'detail': None}, '{"status": "UNMEASURED", "reason": "BAD_RESPONSE"}'
        with self._lock:                           # 확인과 기록을 한 잠금 안에서 — 확인 뒤 기록 전에 정지 · run 바뀜이 끼지 않게(commit 과 같은 방식)
            if (self._epoch, self._stop_count, self.run_id, self.placed) != token or self.halted():
                LOG.info('A4 측정 답을 버림(조회 중 정지 또는 run 바뀜): %s', token[3])
                return True                        # 늦은 답은 버리고 다음 run_once 가 정지 절차를 밟는다
            self.logger.log('a4_measure', self.placed, text)
            self._w144_event('placed_measured', self.placed, **row)
            self._a4_last = (token, row, t_req, t_resp)
        if res[1] == 'TIMEOUT':                    # 정지 요청은 잠금 밖에서(기존 _failed 규칙)
            self._failed('TIMEOUT', f'{self.placed} 놓은 블록 다시 재기')
            return True
        return False

    def _a4_regrasp(self, part):
        """W144 A4 자동 다시 집기(기본 꺼짐 — io.w144_regrasp_on() 이 참일 때만). 반환: True = 상태를 바꿨다(PICK_PLACE · WAIT_SUPPLY), False = 이어 감.

        판단은 regrasp_decision. 다시 집기는 같은 part_id 목표로 PICK_PLACE → 기존 성공 처리 → VERIFY(다시 잼). Z 문제(HOLD)와 사람 확인은 기존 _hold
        (WAIT_SUPPLY no_match_block → [계속]이면 다시 확인). 바깥 영향: CSV 'a4_regrasp' 한 줄, 상태 방송.
        """
        on = getattr(self.io, 'w144_regrasp_on', None)
        if on is None or not on():
            return False
        last = self._a4_last
        mine = bool(last) and last[0][3] == self.placed
        row, t_req, t_resp = (last[1], last[2], last[3]) if mine else (None, 0.0, 0.0)
        corner_on = getattr(self.io, 'w144_corner_on', None)
        corner_on = bool(corner_on and corner_on())          # 모서리 판정(선택, 기본 꺼짐 — w144_corner_judge)
        verdict, out = regrasp_decision(row, t_req, t_resp, part, self._a4_place.get(self.placed), self.cfg, self._a4_tries.get(self.placed, 0),
                                        corner_on)
        with self._lock:                           # 판단 뒤 적용 전에 정지 · 취소 · run 바뀜이 끼면 버린다(commit 과 같은 방식, OK 도) — 다음 run_once 가 정지 절차
            if not mine or (self._epoch, self._stop_count, self.run_id, self.placed) != last[0] or self.halted():
                LOG.info('A4 다시 집기 판단을 버림(정지 또는 상태 바뀜): %s', self.placed)
                return True
            if verdict == 'OK':                    # 모서리 판정이면 기준 안을 다시 집은 횟수와 함께 적어 소진 뒤 진행과 가른다(10/15 사용자 합의)
                if corner_on:
                    self.logger.log('a4_regrasp', self.placed, f'기준 안: 다시 집기 {self._a4_tries.get(self.placed, 0)}번 뒤 중심 · 모서리 2 mm 안')
                return False
            if verdict == 'HOLD':                  # Z 문제 — 자동으로 다시 집지 않고 사람이 고친 뒤 [계속]이면 다시 확인(VERIFY → 다시 잼)
                self.logger.log('a4_regrasp', self.placed, f'Z 문제 — 사람 수정 대기: {out}')
                self._w144_event('regrasp_hold', self.placed, reason=out)
                self._hold(self.placed, f'{self.placed}: {out}', then='verify', message_id='placed_check')   # 놓은 블록 확인 대기 — 공급 대기(no_match_block)와 구분(10/15 PL)
                return True
            if verdict == 'EXHAUSTED':
                self.logger.log('a4_regrasp', self.placed, f'보정 소진: {out}')
                self._w144_event('regrasp_exhausted', self.placed, dx_mm=row['dx_mm'], dy_mm=row['dy_mm'], tries=self._a4_tries.get(self.placed, 0))
                return False                       # 기존 흐름(W144 촬영 → SELECT)으로 다음 블록
            if verdict == 'SKIP':                   # 측정을 못 씀 — 다시 집지도, 사람을 부르지도 않고 까닭만 적고 다음 블록(10/14 21:5x 사용자 결정)
                self.logger.log('a4_regrasp', self.placed, f'측정 못 씀 — 다시 집기 안 함: {out}')
                self._w144_event('regrasp_skipped', self.placed, reason=out)
                return False                       # 기존 흐름(W144 촬영 → SELECT)으로 다음 블록
            n = self._a4_tries[self.placed] = self._a4_tries.get(self.placed, 0) + 1
            more = ''
            if corner_on:                          # 회전만이면 중심은 작고 모서리가 까닭 — 가장 큰 곳을 같이 적는다
                worst = max(corner_xy_errors(row, part, self.cfg), key=lambda e: max(abs(e[1]), abs(e[2])))
                more = f'가장 큰 곳 {worst[0]} {worst[1]:.2f} · {worst[2]:.2f} mm · '
            self.logger.log('a4_regrasp', self.placed, f'{n}번째 다시 집기: dx {row["dx_mm"]} · dy {row["dy_mm"]} mm(설계 중심 기준) · '
                                                       f'{more}놓을 자리 = 처음 보낸 목표')
            self._start_goal(dict(out, status='FOUND', a4_regrasp=n))
            self._set('PICK_PLACE', None, f'{self.placed} 가 설계 자리에서 벗어나 다시 집어 놓습니다({n}/{A4_MAX_TRIES})')
        return True

    def _regrasp_result(self, ok, why):
        """W144 A4 다시 집기 PickPlace 결과. 성공 = 기존 성공 처리(조립 수는 record_placed 가 블록당 한 번만 셈) → VERIFY 로 다시 잼.
        움직이기 전 PLAN_FAILED = 사람 확인(_hold, [계속]이면 다시 확인). 움직인 뒤 실패 · 정지 · 시간 초과 = 기존 _failed(정지 처리)."""
        part = self.goal['part_id']
        moved = self.last_pick['step'] is not None
        self.logger.log('a4_regrasp', part, f'다시 집기 결과: {"성공" if ok else why}{"" if ok or moved else "(움직이기 전)"}')
        if ok:
            self.placed = part
            self.logger.record_placed(self.placed, self.goal['supply_slot'])
            self.poll_hmi()
            if self._hmi_pause:
                return self._enter_wait_hmi()
            self._set('VERIFY', None, f'{self.placed} 를 다시 놓았다. 확인합니다')
            return None
        if why == 'PLAN_FAILED' and not moved:
            return self._hold(part, f'{part}: 다시 집을 길을 찾지 못했어요(PLAN_FAILED) — 손으로 바로 놓은 뒤 [계속]을 누르면 다시 확인합니다', then='verify')
        return self._failed(why or 'ERROR', f'{part} 다시 집기', log=False)

    def _wait_supply(self):
        """WAIT_SUPPLY: 먼저 관측 자세로 1번 간다. 도착한 뒤에야 '공급 채워 주세요'(supply_empty)를 방송하고 [계속]을 받는다.

        [계속](start)이 오면 공급 칸을 다 있음으로 되돌려 같은 블록으로 PICK_PLACE 를 다시 시작한다(SELECT 로 돌아가지 않는다).
        도착 전에 온 start 는 command 가 거절하므로 여기서는 진행하지 않는다.
        """
        if not self._supply_moved:
            ok, why = self.io.move_to('observe', self.halted)
            if not ok:
                return self._failed(why, '공급 대기 자세 이동')
            with self._lock:
                self._supply_moved = True
                self._publish('supply_empty', '공급 칸이 비었어요. 채운 뒤 [계속]을 누르세요')
            return None
        with self._lock:
            if not self._pending_start:
                return None
            self._pending_start = False
        then, self._hold_then = self._hold_then, None
        if then == 'reset':
            self._reset_spare_tries()
        if then == 'verify':                   # 놓은 뒤 확인이 안 됐던 블록 — 사람이 고친 뒤 다시 확인
            self._set('VERIFY', None, f'{self.placed} 를 다시 확인합니다')
            return None
        if self._scatter is not None:
            self._scatter.after_wait_start()        # 흩뿌림은 다시 관측하고 조회한다. 공급 칸 표시는 건드리지 않는다
            if self._scatter.via_spare:
                self._spare_failed.clear()          # 사람이 손본 뒤에는 처음 후보부터(one_pick)
        else:
            self.planner.supply_refilled()
        r = self._next_goal('공급을 채웠다. {part} 를 이어서 집습니다')
        if r is None:                          # 흩뿌림은 전환까지 끝냈다(다시 기다림 · 집기 · 오류)
            return None
        if r['status'] != 'FOUND':
            return self._to_error(f'공급을 채웠는데 다음 블록을 정하지 못했어요({r["status"]}). 로봇 담당자에게 알려 주세요')
        self._start_goal(r)
        self._set('PICK_PLACE', None, f'공급을 채웠다. {r["part_id"]} 를 이어서 집습니다')
        return None

    def _stopped(self):
        """STOPPED: 들어온 뒤 잠금이 풀린 safety/state 가 새로 오면 RECOVER 로 간다(SDD 5장)."""
        with self._lock:
            if self._unlocked_since(self._entered_count):
                self._set('RECOVER', None, '다시 시작 — 쥔 블록을 확인합니다')

    def _recover(self):
        """RECOVER: 조립 중이었으면 쥔 블록이 없는지 확인(gripper/state)하고 진행 확인부터 다시 한다. 쥐고 있거나 모르면 ERROR.

        조립 중이 아니었으면(IDLE · READY · DONE 에서 정지) IDLE 로 간다 — SDD 5장의 비조립 RECOVER → IDLE 규칙(IRD 12장 W121 다시 확인 대상).
        조립 중 복구는 첫 관측 이동에 recover.speed_ratio 를 쓰며, 실패했으면 다음 시도도 저속을 유지한다.
        """
        if not self._assembling:
            # [초기화] → [정리 끝]이면 사람이 조립 자리를 치웠다고 확인했으니 장면의 놓인 블록도 비운다(빈 진행표 → 장면 관리가 blk_* 전부 지움).
            # 그냥 정지 → [다시 시작](완성 · 대기 등)은 실물이 남았을 수 있어 그대로 둔다(10/14 석형님 #174 리뷰 ①) — 그때 장면은
            # 다음 [조립] 출발 때 빈 진행표로 비운다(조립 끝 DONE 에도 그대로 — DONE 뒤 [스캔]이 가구를 피해야 함, 10/14 PL)
            if self._cleaned:
                self._cleaned = False
                self.io.publish_progress({'schema': 'progress/2.0', 'stamp': time.time(), 'design_id': self.design_id, 'run_id': self.run_id,
                                          'obs_stamp': time.time(), 'parts': []})
            self.planner, self.design_id, self.part_id = None, None, None
            self._clear_run()
            self._set('IDLE', None, '정지가 풀렸어요. 설계를 다시 고르세요')
            return None
        with self._lock:
            if self._gripper_busy:                # 열기 답 전에는 gripper/state 도 믿을 수 없고 팔도 움직이지 않는다 — 답이 오면 다음 단계가 이어 본다
                return None
        grip = self.gripper
        # 안내는 화면(HMI GRIP_HELD)과 같은 길(10/14 PL — E-114 꼴). 쥔 채 [다시 시작]만 다시 누르면 RECOVER → ERROR 를 되풀이하므로
        # 빼는 길([정지] → [그리퍼 열기] — 정지에서만 받음)을 알린다. 상태를 모르면 그리퍼 연결부터
        if grip is None:
            return self._to_error('그리퍼 상태를 몰라 이어 갈 수 없어요 → 그리퍼 신호가 오는지 → 로봇 PC 그리퍼 노드와 그리퍼 전원 · 연결을 확인한 뒤 [다시 시작]')
        if grip.get('grasped'):
            return self._to_error('그리퍼가 블록을 쥐고 있어 이어 갈 수 없어요 → 쥔 블록을 먼저 빼야 해요 → 블록을 받칠 사람과 함께 [정지] → [그리퍼 열기] → [다시 시작]')
        self._recover_slow = True
        if self._w144_reopen:                      # 정지 때 닫은 W144 녹화를 CHECK 첫 단계가 새 폴더로 다시 연다(녹화 꺼짐이면 아무것도 안 함)
            self._w144_reopen, self._w144_starting = False, True
            # 새 녹화는 앞 녹화의 시도 번호를 모른다 — 남겨 두면 다시 집기 · 촬영이 옛 번호를 써서 'action_id 없음' → ERROR 를 되풀이했다(10/15 002 실기).
            # 정지 때가 아니라 여기서 비운다: 정지 직후 늦게 온 집기 결과가 옛 녹화(아직 열림)에 실패를 적고 옛 번호를 다시 남기기 때문
            self._w144_open, self._w144_last, self._w144_legs = {}, {}, {}
        if self._scatter is not None and self._scatter.via_spare:
            # 받침 경유: 사람이 확인하고 [다시 시작]하면 시도 수 · 안 된 후보를 비우고 이어 간다(one_pick hold → Enter 와 같음).
            # 받침 기록(_spare_on)은 남겨 받침 위 블록부터 다시 잰다
            self._reset_spare_tries()
        self._set('CHECK', None, '진행 확인부터 다시 합니다')
        return None

    def _reset_spare_tries(self):
        """받침 경유 시도 수 · 안 된 후보 · 연달아 센 값들을 비운다(사람이 확인한 뒤 이어 갈 때)."""
        self._scatter_fail.clear()
        self._spare_failed.clear()
        self._plan_streak = self._no_match = self._bad_blocks = 0

    def _error(self):
        """ERROR: 사람이 확인할 때까지 기다린다. 정지 확인 대기 중이면 stopped 신호 전에는 복구를 막는다.

        정지 노드의 resume 은 잠겨 있지 않아도 safety/state 를 다시 내보내므로(최신 main safety_stop_node), 그것이 신호다.
        """
        with self._lock:
            if self._timeout_pending:
                return                         # 정지 상태를 한 번도 못 받았다면 새 unlocked 신호도 복구 허가가 아니다
            if self._unlocked_since(self._entered_count):
                self._assembling = self.planner is not None and not self._scan_run
                self._set('RECOVER', None, '다시 시작 — 쥔 블록을 확인합니다')

    # ---------- 스캔: 네트워크 대기는 잠금 밖 ----------
    def _begin_scan(self):
        """잠금 안에서 IDLE·SCAN_REVIEW·DONE의 scan 명령을 점검하고 새 run으로 시작한다. 로봇 이동은 작업 스레드에서 한다.

        설정은 robot.yaml scan.poses, 단위 없음. 점검 실패는 현재 상태·기록을 보존하며 (False, reason)을 반환한다.
        """
        # DONE 도 받는다(10/13 PL): 끝난 조립 기록은 이미 닫혀 있고 last_build · pending_builds 는 그대로 두며 새 run_id 로 시작한다
        if self.state not in ('IDLE', 'SCAN_REVIEW', 'DONE') or self._shutdown:
            return self._reject(self._busy_reason(), '지금은 스캔을 시작할 수 없어요(대기 · 조립 끝 · 스캔 검토에서만). 하던 일이 끝난 뒤 누르세요')
        poses = self.cfg.get('scan', {}).get('poses')
        bad_poses = not isinstance(poses, list) or not poses or any(
            not isinstance(p, str) or f'{p}_pose' not in self.cfg for p in poses) or len(set(poses)) != len(poses)
        if bad_poses:
            LOG.warning('스캔 시작 거절: robot.yaml scan.poses %r 와 각 <이름>_pose 가 없거나 맞지 않는다', poses)   # 설정 키는 로그(담당자용)
            return self._reject('', '스캔 촬영 자세 설정이 없거나 맞지 않아 시작할 수 없어요. 로봇 담당에게 설정 확인을 부탁하세요')     # 맞는 코드 없는 거절 = 빈 값 + 글(E-111 A3)
        ok, code, text = self._precheck(scan=True)
        if not ok:
            return self._reject(code, f'스캔 시작 점검 실패: {text}')
        self._clear_run()
        self.planner, self.design_id, self.part_id, self.placed = None, None, None, None
        self._scan_run = True
        self._scan_poses, self._scan_index = tuple(poses), 0
        self.run_id = self.logger.start_run('', 0)     # 구조가 아직 설계가 아니므로 CSV 만 열고 build/2 은 만들지 않는다
        self.logger.log('scan', None, 'start')
        self._set('SCAN_MOVE', 'scan_running', '스캔 촬영 자세로 갑니다')
        return True, ''

    def _scan_move(self):
        """현재 scan.poses 자세로 이동한다(m·rad는 motion 담당). 실패는 공통 정지·오류 처리, 성공 뒤에만 촬영 상태로 간다."""
        pose = self._scan_poses[self._scan_index]
        ok, why = self.io.move_to(pose, self.halted)
        if not ok:
            return self._failed(why, '스캔 자세 이동')
        with self._lock:
            if self.halted():
                return self._failed('STOPPED', '스캔 자세 이동')
            self._set('SCAN_CAPTURE', 'scan_running', f'{pose}에서 촬영합니다')

    def _scan_capture(self):
        """현재 자세·run_id로 점군을 수집한다. points는 비음수 정수. 잘못된 답은 SCAN_FAILED, 정지 뒤 답은 버린다."""
        pose = self._scan_poses[self._scan_index]
        ok, why, body = self.io.scan_capture(pose, self.run_id, self.halted)
        if not ok or self.halted():
            return self._failed('STOPPED' if self.halted() else why, '스캔 촬영')
        if self._scan_data_failed(body, '스캔 실패', '스캔 촬영'):
            return None
        points = body.get('points') if isinstance(body, dict) else None
        if not isinstance(body, dict) or body.get('ok') is not True or isinstance(points, bool) or not isinstance(points, int) or points < 0:
            return self._failed('SCAN_FAILED', '스캔 촬영 응답')
        with self._lock:
            if self.halted():
                return self._failed('STOPPED', '스캔 촬영 결과 적용')
            self.logger.log('scan_capture', None, f'{pose}: {points}')
            self._scan_index += 1
            self._set('SCAN_MOVE' if self._scan_index < len(self._scan_poses) else 'SCAN_INFER', 'scan_running', '스캔 촬영을 마쳤습니다')

    def _scan_data_failed(self, body, title, where):
        """비전이 답은 냈지만 결과가 실패({"ok": false, "reason", "detail"} · success=true)면 화면에 '<title>\\n<detail>'로 알리고 ERROR. 반환: 처리했나.

        W182(10/13 PL · 민범진 계약): scan_capture 의 ok:false = '스캔 실패'(재촬영으로 안 됨), scan_infer 의 ok:false = '다시 스캔하세요'(자료 실패).
        detail 이 글자가 아니거나 ok 가 false 가 아니면 이 함수는 손대지 않는다 — 깨진 응답은 부르는 쪽의 기존 SCAN_FAILED 처리로 간다.
        success=false(TIMEOUT 정지 요청 포함)는 여기 오기 전에 기존 _failed 로 처리된다. 로봇 이동 없음.
        """
        if not isinstance(body, dict) or body.get('ok') is not False or not isinstance(body.get('detail'), str):
            return False
        detail = body['detail'].strip() or str(body.get('reason') or '이유 없음')
        with self._lock:
            if self.halted():
                self._failed('STOPPED', f'{where} 결과 적용')
                return True
        self.logger.log('fail', None, f'{where}: {body.get("reason") or ""} {detail}'.strip())
        self._to_error(f'{title}\n{detail}')
        return True

    def _scan_infer(self):
        """모은 점군을 추론해 scan_result/2.0 로 방송한 뒤 검토를 기다린다. 단위 mm, 로봇 이동 없음.

        구조 합격 검사는 HMI의 check_design 요청 때 한다. 여기서는 JSON 형식·필수 값만 확인하며 잘못된 답은 SCAN_FAILED.
        """
        ok, why, body = self.io.scan_infer(self.run_id, self.halted)
        if not ok or self.halted():
            return self._failed('STOPPED' if self.halted() else why, '스캔 추론')
        if self._scan_data_failed(body, '다시 스캔하세요', '스캔 추론'):
            return None
        try:
            blocks = body['blocks']
            if body.get('ok') is not True or not isinstance(blocks, dict) or not same_major(blocks.get('schema'), 'blocks/2'):
                raise ValueError('blocks/2 객체가 아니다')
            if any(not isinstance(blocks.get(k), str) or not blocks[k] for k in ('design_id', 'family')):
                raise ValueError('design_id · family 가 없다')
            rows = blocks.get('blocks')
            if not isinstance(rows, list) or not rows:
                raise ValueError('추론 블록이 없다')
            nums = set()
            for b in rows:
                num = b['scan_num']
                if isinstance(num, bool) or not isinstance(num, int) or num <= 0 or num in nums:
                    raise ValueError('블록 scan_num 이 잘못됐다')
                nums.add(num)
                if b.get('ori') not in ORIS or not isinstance(b.get('inferred', False), bool):
                    raise ValueError('블록 ori · inferred 가 잘못됐다')
                if any(isinstance(b[k], bool) or not isinstance(b[k], (int, float)) for k in ('x', 'y', 'z')):
                    raise ValueError('블록 좌표가 숫자가 아니다')
            count = body['inferred_count']
            if isinstance(count, bool) or not isinstance(count, int) or not 0 <= count <= len(rows) or not isinstance(body['image_path'], str):
                raise ValueError('inferred_count · image_path 가 잘못됐다')
            cloud = body.get('cloud_path', '')                # 화면 점군 창용 PLY 경로(로봇 PC). 선택 칸(10/8 PL, IRD 4.2 · 6장): 없거나 빈 글자 = 점군 없음 → 스캔은 그대로, 결과에서 뺀다
            if not isinstance(cloud, str):                    # 글자가 아닌 값(null · 숫자 · 목록 …)만 잘못된 응답이다 → SCAN_FAILED
                raise ValueError('cloud_path 가 글자가 아니다')
            result = dict(schema='scan_result/2.0', stamp=time.time(), run_id=self.run_id, blocks=copy.deepcopy(blocks),
                          inferred_count=count, image_path=body['image_path'], poses_used=list(self._scan_poses))
            if cloud:
                result['cloud_path'] = cloud                  # 비어 있지 않은 글자는 그대로 넘긴다(작업 관리자는 파일을 열지 않는다)
            nearest = body.get('nearest_base')                # 선택 칸(E-79): 추론기가 고른 가장 가까운 기본 설계 design_id — HMI 가 GPT 힌트로 쓴다.
            if isinstance(nearest, str) and nearest:          # 힌트일 뿐이라 없음 · null · 빈 글자 · 글자 아닌 값은 칸을 빼고 스캔은 그대로 간다(cloud_path 와 달리 SCAN_FAILED 로 막지 않는다)
                result['nearest_base'] = nearest
            json.dumps(result, allow_nan=False)       # NaN · Infinity 를 방송·저장 가능한 결과로 넘기지 않는다
        except (KeyError, TypeError, ValueError) as e:
            return self._to_error(f'SCAN_FAILED: 추론 응답이 잘못됐다({e})')
        with self._lock:
            if self.halted():
                return self._failed('STOPPED', '스캔 추론 결과 적용')
            self.io.publish_scan_result(result)
            self._finish_run('DONE')
            self._set('SCAN_REVIEW', 'scan_review', '스캔 결과를 확인하고 저장·다시 스캔·취소를 선택하세요')

    # ---------- 실패 · 공급 처리 ----------
    @staticmethod
    def _positive(value, name):
        """유한한 양의 실수(bool 아님)면 float 로 돌려주고, 아니면 ValueError."""
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'robot.yaml {name} 는 0 보다 큰 유한한 수다: {value!r}')
        return float(value)

    def _scatter_failed(self, why):
        """흩뿌림 집기가 실패한 뒤 다음 동작을 정한다(W178): ScatterFlow.on_failure 의 판단을 따르되 블록당 총 집기 시도를 max_attempts 로 막는다.

        입력: why(PickPlace 실패 이유), self.last_pick(마지막 step · 결과 수신 시각), self.gripper. 바깥 영향: REPLAN_SAME 은 같은 후보로 PICK_PLACE 를
        한 번 더, REOBSERVE 는 SELECT 로 돌아가 observe_supply → find_blocks 를 다시 한다(로봇을 새로 움직이는 건 그 경로뿐).
        RECOVER · ERROR · 시도 한도 · 후보가 없을 때 · WAIT(입력원이 없어 새 경로를 만들지 않음)는 서서 사람을 부른다(ERROR → [다시 시작] → 기존 RECOVER).
        손에 블록이 없음(grasped False)이 확인돼야만 다시 시도한다 — 모르면 failure_action 이 RECOVER 로 보낸다.
        gripper 상태를 기다리는 동안 정지 · 상태 변화 · 새 요청이 끼면(오래된 처리) 아무것도 적용하지 않고 지금 상태를 그대로 둔다.
        """
        part = self.goal['part_id']
        info = self.last_pick
        with self._lock:
            token = (self._epoch, self._stop_count)
        status, grasped = self._grasped_after_result(info, token)
        if status == 'STALE':
            return None
        with self._lock:
            if self._stale(info['request_id'], token):         # 대기 뒤 적용 직전에 다시 확인 — on_failure 도 이 안에서 부른다
                return None
            action = self._scatter.on_failure(why, contact_phase(info['step'], why), grasped, None)
            via = self._scatter.via_spare
            if via and action == REPLAN_SAME:
                action = REOBSERVE                 # 받침 경유: 팔이 움직인 뒤 실패는 같은 요청을 되풀이하지 않고 다시 찍는다(one_pick)
            n = self._scatter_fail.get(part, 0)
            check = ' — 받침 · 손 안을 확인해 주세요' if via else ''
            regrasp = False
            if action in (REPLAN_SAME, REOBSERVE):
                if n >= self.max_attempts:
                    if via:                    # 손은 비었다(REOBSERVE) — 서지 않고 사람이 보고 [계속]이면 시도 수를 비우고 이어 감(one_pick)
                        return self._hold(part, f'{part}: 팔이 움직인 시도 {n}번 모두 안 됨({why}) — 블록 · 받침 · 로봇을 확인하고 [계속]을 누르세요',
                                          then='reset')
                    return self._to_error(f'{part}: {why}(집기 {n}번 모두 실패{check} — 사람이 확인해 주세요)')
                if via and not self.goal.get('via_spare'):
                    # 받침 위 블록을 옮기다(눕히기 · 맞추기 · 다시 집기) 안 됨 — 받침 · 손 안을 사람이 보고 [계속]이면 받침을 다시 잰다(one_pick 'check')
                    return self._hold(part, f'{part}: 받침 위 블록을 옮기다 안 됨({why}) — 받침 · 손 안을 확인하고(손에 있으면 빼서 받침에 두기) '
                                            '[계속]을 누르면 다시 잽니다')
                req = self._scatter.reusable_request() if action == REPLAN_SAME else None
                if req is not None:
                    self._start_goal(dict(req, status='FOUND'))
                    return self._set('PICK_PLACE', None, f'{part} 를 같은 자리로 다시 집으러 갑니다')
                # 받침 경유 공급 헛잡음이면 그 블록을 다시 찍어 같은 잡기로 한 번 더(one_pick try_goal) — 찍기는 잠금 밖에서
                regrasp = (via and why == 'GRASP_FAILED' and self.goal.get('via_spare') and not self.goal.get('regrasp')
                           and self.goal.get('grasp') != 'TILT')
                if not regrasp:
                    return self._set('SELECT', None, f'{part} 를 다시 보고 집으러 갑니다')
            else:
                unknown = ' — 그리퍼 상태를 확인하지 못했다' if status == 'UNKNOWN' else ''
                return self._to_error(f'{part}: {why}(흩뿌림 집기 실패 — {action}{unknown}{check}, 사람이 확인해 주세요)')
        if regrasp and self._spare_regrasp(self.goal):
            return None
        with self._lock:
            if self._stale(info['request_id'], token):
                return None
            return self._set('SELECT', None, f'{part} 를 다시 보고 집으러 갑니다')

    def _stale(self, request_id, token):
        """(잠금 안) 대기 중에 정지 · 상태 변화 · 새 요청이 있었나. 있으면 이 실패 처리는 오래된 것이다."""
        return (self.halted() or (self._epoch, self._stop_count) != token
                or self.last_pick is None or self.last_pick['request_id'] != request_id)

    def _grasped_after_result(self, info, token):
        """PickPlace 결과 뒤 그리퍼가 블록을 쥐고 있나. 반환 (status, grasped): ('OK', True/False) · ('UNKNOWN', None) · ('STALE', None).

        gripper 상태는 1초마다 + 바뀔 때 나오므로 결과 직전 값(stamp < 결과 시각)은 grasped_now 가 거절한다. 이 함수는 그것을 완화하지 않고
        gripper_wait_s 동안만 0.05초 간격으로 새 값을 기다린다(기다리는 동안 잠금을 잡지 않는다). 결과를 못 받았으면(result_at 없음) 기다리지 않는다.
        UNKNOWN = 제한 시간 안에 믿을 값이 안 왔다(→ 기존 복구 정책). STALE = 기다리는 중 정지 · 상태 변화 · 새 요청이 끼었다(→ 아무것도 적용하지 않음).
        gripper_wait_s 는 운영 설정 제안값이지 실측으로 보장된 도착 시간이 아니다.
        """
        result_at, rid = info['result_at'], info['request_id']
        if result_at is None:
            return 'UNKNOWN', None
        deadline = self._clock() + self.gripper_wait_s
        while True:
            with self._lock:
                if self._stale(rid, token):
                    return 'STALE', None
                snapshot = copy.deepcopy(self.gripper)          # 잠금 안에서 복사한 값으로 판정한다
            grasped = grasped_now(snapshot, result_at, self._wall(), self.gripper_max_age_s)
            if grasped is not None:
                return 'OK', grasped
            remaining = deadline - self._clock()
            if remaining <= 0:
                return 'UNKNOWN', None
            self._sleep(min(GRIPPER_POLL_S, remaining))

    def _grasp_failed(self):
        """GRASP_FAILED: 같은 칸 1번 더 → 다음 칸 1개 → 거기서도 실패하면 ERROR. 모든 칸을 돌지 않는다.

        다음 칸이 없으면(next_block 이 WAIT_SUPPLY 로 알려도) 공급이 빈 것이 아니라 재시도 칸이 없는 것이므로
        '공급 채워 주세요'를 띄우지 않고 ERROR 로 간다(SLOT_EMPTY 의 WAIT_SUPPLY 와 섞지 않는다).
        """
        self._grasp_stage += 1
        part = self.goal['part_id']
        if self._grasp_stage == 1:             # 같은 칸으로 다시(상태 그대로)
            return None
        if self._grasp_stage == 2:
            r = self.planner.next_block(avoid_slots=(self.goal['supply_slot'],))
            if r['status'] == 'FOUND' and r['part_id'] == part:
                self.goal = r
                return None
            return self._to_error(f'{part}: GRASP_FAILED(같은 칸 재시도 뒤 쓸 다른 칸이 없다)')
        return self._to_error(f'{part}: GRASP_FAILED(다음 칸에서도 실패)')

    def _slot_empty(self):
        """SLOT_EMPTY: 그 칸을 빔으로 적고 같은 블록의 다음 칸으로. 맞는 칸이 다 비었으면 진짜 WAIT_SUPPLY 로 간다."""
        self.planner.mark_slot_empty(int(self.goal['supply_slot']))
        r = self.planner.next_block()
        if r['status'] == 'FOUND' and r['part_id'] == self.goal['part_id']:
            self.goal, self._grasp_stage = r, 0
        elif r['status'] == 'WAIT_SUPPLY':
            self._enter_wait_supply(r['part_id'])
        else:
            self._to_error(f'공급 칸을 바꾸는 중 예상 못 한 결과가 나왔어요({r["status"]}). 로봇 담당자에게 알려 주세요')

    def _enter_wait_supply(self, part_id, alert=None):
        """WAIT_SUPPLY 상태로 들어간다. 칸 방식은 알림(supply_empty)을 아직 안 낸다 — 관측 자세에 도착한 뒤 _wait_supply 가 낸다.

        흩뿌림은 이미 observe_supply 에 와 있으므로 alert=(message_id, 글자)를 주면 도착 확인 없이 그 알림으로 바로 들어간다.
        """
        with self._lock:
            self.part_id = part_id
            self._pending_start = False        # 이전에 눌린 [계속]은 버린다
            if alert is not None:
                self._supply_moved = True
                self._set('WAIT_SUPPLY', alert[0], alert[1])
            else:
                self._supply_moved = False
                self._set('WAIT_SUPPLY', None, '공급이 필요해요. 관측 자세로 갑니다')

    def _enter_wait_hmi(self):
        """현재 동작이 끝난 뒤 WAIT_HMI 를 알린다. 정지 요청 · 공급 초기화 · 조립 요약 확정은 하지 않는다."""
        with self._lock:
            if self.state != 'WAIT_HMI':
                self._pending_start = False
                self._set('WAIT_HMI', 'hmi_lost', '웹 연결이 끊겼어요. 다시 연결한 뒤 [계속]을 누르세요')

    def _failed(self, reason, where, log=True):
        """이동 · 관측 · 집기 실패를 처리한다. TIMEOUT 은 잠금 밖에서 정지 요청 후 상태 확인, 정지류는 STOPPED, BUSY 는 재시도.

        모르는 reason(IRD 7장에 없는 문자열)도 이름을 따로 다루지 않고 일반 실패로 ERROR 에 보낸다. 글자는 알림에 그대로 적는다.
        """
        if log:
            self.logger.log('fail', self.part_id, f'{where}: {reason or "이유 없음"}')
        if reason == 'TIMEOUT':
            with self._lock:
                self._timeout_pending = True
            try:
                ok, message = self.io.request_stop('TIMEOUT')    # 파일 기록 대기 · 상태 복구보다 먼저, 잠금 밖에서 보낸다
            except Exception as e:              # 정지 요청 자체가 실패해도 다음 이동을 막고 사람에게 알린다
                ok, message = False, repr(e)
            LOG.warning('TIMEOUT 정지 요청 %s: %s', '접수' if ok else '실패', message)
            with self._lock:
                if self.safety and self.safety.get('stopped'):
                    self._enter_stopped()
                else:
                    self._to_error(f'{where}: 시간 안에 끝나지 않아(TIMEOUT) 정지를 요청했지만(요청 {"접수" if ok else "실패"}) 아직 정지가 확인되지 않아 다시 시작할 수 없어요. '
                                   '로봇이 멈췄는지 확인하고 화면의 [정지]를 누르세요. 정지가 확인되면 [다시 시작]을 누르세요')
            return
        if reason in STOP_REASONS:             # 정지 노드의 신호를 기다린다
            self._enter_stopped()
        elif reason == 'BUSY':                 # 기다렸다 다시: 같은 상태에 머무르면 다음 단계가 다시 부른다 — 1 s 쉬고(one_pick, 쉼 없이 다시 보내지 않게)
            self._sleep(1.0)
            return
        elif reason in ('GRIPPER_NO_RESPONSE', 'NO_FEEDBACK'):     # 코드만 보이면 사람이 뭘 해야 할지 모른다 — 코드는 괄호로 남겨 기록과 맞춘다(E-74)
            what = '그리퍼가 응답하지 않아요' if reason == 'GRIPPER_NO_RESPONSE' else '그리퍼 폭을 읽지 못했어요'
            self._to_error(f'{where}: {what}({reason}) — 쥔 블록이 있으면 받친 채 그리퍼 전원 · 연결을 복구하고, 그리퍼를 열어 블록을 뺀 뒤 [다시 시작]을 누르세요')
        else:
            self._to_error(f'{where} 실패: {reason or "이유 없음"}')

    def _offset_over(self, problems):
        """판정에서 OFFSET_OVER 가 나왔다. 첫 문제를 알림에 적고 ERROR 로 간다."""
        first = problems[0]
        self._to_error(f'{first["part_id"]}: OFFSET_OVER — {first["detail"]}', 'offset_over')

    def _observe(self, part_ids):
        """블록들을 check_progress 로 보고 진행표에 넣고 progress 를 방송한다. 실패하면 알아서 상태를 바꾸고 False."""
        ok, why, rows = self.io.check_progress(part_ids, self.halted)
        if not ok:
            self._failed(why, '진행 확인')
            return False
        try:
            self.planner.update_progress(rows)
        except ValueError as e:
            self._to_error(f'손목 카메라의 진행 확인 답을 쓸 수 없어요({e}). 비전 담당에게 알려 주세요')
            return False
        self.io.publish_progress(self.planner.progress_message(self.run_id, time.time()))
        return True

    # ---------- 설계 고르기 · 출발 · 시작 점검 ----------
    def _start_lookup_target(self, design_id):
        """(잠금 안) 출발 명령이 설계 조회부터 해야 하면 그 design_id, 아니면 None(바로 _start 가 처리: [계속] · 거절).

        READY 에서는 [설계 선택] 때 받아 둔 설계를 그대로 쓴다(E-55 ①, 10/7 PL — 다시 조회하지 않는다). IDLE 에서 설계를 포함한 음성 start 만
        그 설계를 조회한다.
        """
        if self.state == 'IDLE' and design_id:
            return design_id
        return None

    def _lookup_then(self, design_id, mode):
        """설계를 조회한 뒤 mode 에 따라 적용한다(select = 고르기 → READY, start = 고르기(IDLE 이면) + 출발).

        ① 잠금 안: 지금 가능한 상태인지 보고 요청 번호 · 상태 표(epoch)를 적는다 ② **잠금 밖**: io.get_design 으로 기다린다
        ③ 잠금 안: 더 새 요청이 있거나 · 상태가 바뀌었거나 · 정지 · 종료 중이면 답을 버리고, 아니면 검증해서 적용한다.
        """
        received = self._clock()                       # 명령을 받은 순간 — timeout.command_s 안에 확정 못 하면 실행하지 않는다
        with self._lock:
            if mode == 'start':
                design_id = self._start_lookup_target(design_id)
                if design_id is None:
                    return self._start()
            if mode == 'select' and self.state not in ('IDLE', 'READY', 'DONE', 'SCAN_REVIEW'):
                return self._reject(self._busy_reason(), '작업 중에는 설계를 바꿀 수 없어요. 작업이 끝난 뒤 선택해 주세요')
            if not design_id:
                return self._reject('', f'설계 파일을 못 읽었다: {design_id!r} — 설계를 고르지 않았어요. 화면 설계 목록에서 고른 뒤 다시 누르세요')
            self._lookup_token += 1
            token, epoch, was_halted = self._lookup_token, self._epoch, self.halted()    # 이미 정지 중이면 시작 점검이 이유를 알린다
        try:
            ok, why, design = self.io.get_design(design_id, self.halted)
        except Exception as e:   # noqa: BLE001 — 조회가 어떻게 실패해도 새 조립을 시작하지 않는다
            ok, why, design = False, 'ERROR', None
            LOG.warning('설계 조회 중 예외: %r', e)
        with self._lock:
            if self._shutdown or (self.halted() and not was_halted):      # 조회하는 동안 정지 · 종료가 왔다
                return self._reject('STOPPED', '정지 또는 종료로 설계 조회를 중단했어요. 복구가 끝난 뒤 다시 선택해 주세요')
            if token != self._lookup_token or epoch != self._epoch:
                return self._reject('BUSY', '조회하는 동안 다른 요청이나 상태 변화가 있어 이 조회 결과를 버렸어요. 다시 눌러 주세요')
            if self._clock() - received > self.command_s:      # 늦은 출발 · 선택은 허용하지 않는다(웹은 이미 시간 초과로 보였다)
                return self._reject('TIMEOUT', f'명령을 받은 뒤 {self.command_s} 초 안에 끝내지 못해 실행하지 않았어요. 웹 PC와 로봇 PC 연결(다리 · 브로커)을 확인하고 다시 누르세요')
            if not ok:
                code = why if why in ('TIMEOUT', 'STOPPED', 'BUSY') else ''      # E-111 A3: 이 셋만 코드, 나머지(ERROR · 서버 이유)는 빈 값 + 글
                return self._reject(code, f'설계를 못 가져왔다: {design_id!r} ({why or "이유 없음"}) — 설계 목록에 이 설계가 있는지 확인하고 다시 골라 주세요. 계속 안 되면 설계 저장소(웹) 담당에게 알려 주세요')
            planner, problem = self._planner_from(design, design_id)
            if problem:
                return self._reject('', f'이 설계 레시피로는 조립 계획을 만들 수 없어요({problem}). 다른 설계를 고르세요')
            if mode == 'select':
                return self._apply_select(planner, design_id)
            if self.state == 'IDLE':
                ok, why = self._apply_select(planner, design_id)
                if not ok:
                    return ok, why
            return self._start()

    def _planner_from(self, design, design_id):
        """조회 답(design/4.0)을 검증해 TaskPlanner 를 만든다. 반환: (planner, 문제 글자). 문제가 없으면 문제 글자는 빈 값.

        검증: 객체 · schema design/4.0 · 요청한 design_id 와 같음 · recipe(구조)와 placements(조립 방법)가 각각 recipe/4.0 · placements/4.0 객체
        (두 문서는 block 이름과 recipe_sha256 으로 연결한다). 칸 이름이 아니라 schema 로 내용을 확인하고, 옛 형식(design/1 · cad_*)은 받은 schema 를
        적어 거절한다(변환하지 않음). 레시피 내용은 TaskPlanner 가 robot.yaml 과 맞는지 본다. 잘못된 입력은 조립하지 않고 문제 글자를 반환한다.
        """
        got = design.get('schema') if isinstance(design, dict) else None
        if not same_major(got, DESIGN_SCHEMA):
            return None, f'설계 조회 답이 {DESIGN_SCHEMA} 가 아니다(받은 schema: {got!r}) — 옛 형식은 거절한다'
        if design.get('design_id') != design_id:
            return None, f'조회 답의 design_id({design.get("design_id")!r})가 요청({design_id!r})과 다르다'
        for key, schema in (('recipe', RECIPE_SCHEMA), ('placements', PLACEMENTS_SCHEMA)):
            got = design.get(key, {}).get('schema') if isinstance(design.get(key), dict) else None
            if not same_major(got, schema):
                return None, f'조회 답의 {key} 가 {schema} 객체가 아니다(받은 schema: {got!r})'
        try:
            planner = TaskPlanner(self.cfg, design['recipe'], design['placements'])
        except (ValueError, KeyError, TypeError) as e:
            return None, f'레시피를 못 읽는다: {e}'
        if not planner.parts:
            return None, '레시피에 블록이 없다'
        return planner, ''

    def _apply_select(self, planner, design_id):
        """(잠금 안) 조회한 설계로 시작 점검을 통과하면 READY. IDLE · READY · DONE 에서만."""
        if self.state not in ('IDLE', 'READY', 'DONE', 'SCAN_REVIEW'):
            return self._reject(self._busy_reason(), '작업 중에는 설계를 바꿀 수 없어요. 작업이 끝난 뒤 선택해 주세요')
        ok, code, text = self._precheck()
        if not ok:
            return self._reject(code, f'시작 점검 실패: {text}')
        self.planner, self.design_id, self.part_id, self.placed = planner, design_id, None, None
        self._scatter_fail = {}
        self._clear_run()
        self._set('READY', 'ready_to_start', f'{design_id} 준비됐어요. 출발을 누르세요')
        return True, ''

    def _start(self):
        """(lock 안) READY 출발(→ CHECK), WAIT_SUPPLY · WAIT_HMI 에서 계속. 설계 조회는 _lookup_then 이 먼저 한다.

        WAIT_SUPPLY 에서는 관측 자세에 도착해 supply_empty 를 낸 뒤에만 받는다(도착 전 start 는 BUSY 로 거절, 진행시키지 않는다).
        """
        if self.state == 'WAIT_HMI':
            ok, code, text = self._precheck()
            if not ok:
                return self._reject(code, f'계속 점검 실패: {text}')
            self._hmi_pause, self._hmi_recheck = False, True
            self.logger.log('command', self.part_id, 'continue_hmi')
            self._set('SELECT', None, '웹이 연결됐어요. 진행 확인부터 이어갑니다')
            return True, ''
        if self.state == 'WAIT_SUPPLY':
            if not self._supply_moved:
                return self._reject('BUSY', '아직 관측 자세로 가는 중이에요. 도착한 뒤 [계속]을 누르세요')
            self._pending_start = True
            self.logger.log('command', self.part_id, 'continue')    # 같은 run_id 로 이어 간다
            return True, ''
        if self.state != 'READY':
            if self.state == 'IDLE':
                return self._reject('', '설계를 먼저 고르세요 — 화면 설계 목록에서 고른 뒤 [조립]')
            return self._reject(self._busy_reason(), '지금은 출발할 수 없어요(설계를 고른 준비 상태에서만). 하던 일이 끝난 뒤 누르세요')
        ok, code, text = self._precheck()
        if not ok:
            return self._reject(code, f'시작 점검 실패: {text}')
        self._reobserved = False
        self._spare_on, self._spare_failed, self._plan_streak, self._spread = None, [], 0, None
        self._spare_empty = self._no_match = self._bad_blocks = self._spare_centers = 0
        self._fresh, self._hold_then, self._tally = True, None, {'sent': 0, 'ok': 0}
        self._a4_tries, self._a4_last, self._a4_place = {}, None, {}
        self.placed = self.part_id = self._pick = None     # 새 출발 — 앞 조립의 손목 그림 집기 순위를 싣지 않는다(10/14 22시 PL)
        self.run_id = self.logger.start_run(self.design_id, len(self.planner.parts))   # 실제 출발 때만 새 run_id
        self._w144_open, self._w144_last, self._w144_legs = {}, {}, {}    # W144 시도: 부품 → 열린 action_id · 마지막 실패 action_id, action → 단계 기록
        # W144 기록(촬영 폴더 · 설정 복사 · 필수 토픽 확인 · rosbag)은 CHECK 첫 단계에서(잠금 밖) 연다 — 첫 이동 · find_blocks 보다 먼저
        self._w144_starting = getattr(self.io, 'w144_start', None) is not None
        self._set('CHECK', None, '진행 확인을 시작합니다')
        return True, ''

    def _precheck(self, scan=False):
        """시작 점검(M-06) 중 숫자 없는 항목. 반환: (통과, 거절 reason, 이유 글자).

        ① 조립 또는 scan=True의 촬영·추론 서버가 떠 있다(정지 서비스가 없으면 사람이 할 일 중심 글, 경로는 로그) ② 정지 노드 신호를 받았고 잠기지 않았다 ③ 그리퍼가 비었다.
        신호가 몇 초 안에 왔는지(신선도)는 숫자가 필요해서 보지 않는다. 사람이 영역 밖인지는 누르는 사람이 확인한다.
        """
        missing = self.io.services_ready(scan=True) if scan else self.io.services_ready()
        if missing:
            LOG.warning('시작 점검: 안 떠 있는 서버 %s', ', '.join(missing))     # 기술 경로는 로그에만(화면 글은 사람이 할 일 중심)
            if '/d2/safety/stop' in missing:
                # 10/14 점검 ③: 정지 서비스 경로만 보이면 무엇을 해야 할지 모른다 — 팀 실행 절차(복구 절차 3-2, robot_nodes.launch.py)로 안내
                LOG.warning('시작 점검: 정지 서비스 /d2/safety/stop 없음 — 로봇 PC 로봇 노드 실행(ros2 launch d2_bringup robot_nodes.launch.py) 확인(복구 절차 3-2)')
                return False, '', '로봇을 멈추는 기능(정지 노드)이 준비되지 않아 시작할 수 없어요. 로봇 PC의 로봇 노드가 켜져 있는지 확인한 뒤 다시 누르세요'
            names = sorted({SERVICE_LABEL.get(n, n) for n in missing})
            return False, '', f'서버가 안 떠 있다: {" · ".join(names)} 준비 안 됨 — 로봇 PC에서 해당 기능이 켜져 있는지 확인한 뒤 다시 누르세요'
        if self._timeout_pending:
            return False, 'STOPPED', '시간 초과 뒤 정지가 아직 확인되지 않아 시작할 수 없어요. 로봇이 멈췄는지 확인하고 화면의 [정지]를 누르세요. 정지가 확인되면 [다시 시작]을 누르세요'
        if not self._hmi_fresh():
            return False, '', '웹 연결 신호를 못 받았어요. 웹 PC의 웹 · 브로커와 로봇 PC의 다리(bridge)가 켜져 있는지 확인한 뒤 다시 누르세요'      # IRD 7장 BUSY 는 '기다렸다 다시'라 웹이 자동 재시도한다 → 맞는 코드가 없어 비움(E-62)
        st = self.safety
        if st is None:
            return False, '', '정지 노드 신호를 아직 못 받았어요. 로봇 PC의 로봇 노드가 켜져 있는지 확인한 뒤 다시 누르세요'
        if st.get('stopped') or st.get('locked'):
            return False, 'STOPPED', '정지 상태예요. 원인을 없앤 뒤 [다시 시작]을 먼저 누르세요'
        if self.gripper is None:
            return False, '', '그리퍼 신호를 아직 못 받았어요. 로봇 PC의 로봇 노드(그리퍼)와 그리퍼 전원 · 연결을 확인한 뒤 다시 누르세요'
        if self.gripper.get('grasped'):
            return False, '', '그리퍼가 블록을 쥐고 있어 시작할 수 없어요 → 조립 · 스캔 전에 블록을 빼야 해요 → 블록을 받칠 사람과 함께 [정지] → [그리퍼 열기] → [다시 시작]'   # 화면 GRIP_HELD 와 같은 길(10/14 PL)
        return True, '', ''

    # ---------- 상태 바꾸기 · 방송 ----------
    def _start_goal(self, found):
        """다음 블록 목표를 잡고 재시도 횟수를 비운다."""
        self.goal, self.part_id = found, found['part_id']
        self._plan_retry = self._grasp_stage = 0
        self._reobserved = False

    def _apply_safety(self):
        """정지 노드가 stopped 를 보냈으면 어느 상태에서든 STOPPED 로 간다."""
        with self._lock:
            if self.safety and self.safety.get('stopped') and self.state != 'STOPPED':
                self._enter_stopped()

    def _enter_stopped(self):
        """STOPPED 로 간다. 조립 중이었는지 적어 두고(RECOVER 가 쓴다), 눌려 있던 [계속]은 버린다."""
        with self._lock:
            self._timeout_pending = False
            if self.state == 'STOPPED':
                return
            self._assembling = self.state in RUN_STATES and not self._scan_run
            self._cleaned = False
            if self._assembling and getattr(self.io, 'w144_finish', None) is not None:
                # 조립 중 정지면 W144 녹화를 바로 닫는다(닫으면 초당 2장으로 줄이고 원본을 지운다) — 닫지 않으면 [다시 시작] · [초기화] 전까지
                # 원본(카메라 30 Hz)이 분당 약 2.6 GB 씩 쌓였다(10/14 V008 24 GB · V009 57 GB). [다시 시작]으로 이어 가면 RECOVER 가 새로 연다
                self._w144_due, self._w144_reopen = False, True
            self._entered_count = self.safety_count
            self._pending_start = False
            self.logger.count_stop()
            self._set('STOPPED', 'stopped', '멈췄어요. 원인을 없앤 뒤 [다시 시작]을 누르세요')
            # 웹 없이 돌릴 때 사람이 칠 명령(one_pick 안내와 같음) — 잠금 해제는 사람이 직접(10/12 PL)
            LOG.warning('정지 — 원인을 없앤 뒤 사람이 [다시 시작] 또는: ros2 service call /d2/safety/resume std_srvs/srv/Trigger')

    def _to_error(self, message, message_id=None):
        """ERROR 로 간다. 로봇은 서 있고 사람을 부른다. message_id 는 IRD 값만 쓰고 맞는 것이 없으면 null."""
        with self._lock:
            self._entered_count = self.safety_count
            self.logger.log('error', self.part_id, message)
            self._set('ERROR', message_id, message)

    def _unlocked_since(self, count):
        """(lock 안) count 번째 이후에 잠금이 풀린 safety/state 가 새로 왔나. [다시 시작]을 알아보는 규칙."""
        st = self.safety
        return bool(st) and self.safety_count > count and not st.get('stopped') and not st.get('locked')

    def _finish_run(self, result):
        """열린 CSV를 result로 닫는다. 조립만 build/2 요약을 보존하고 스캔은 DB 조립 목록에 넣지 않는다. run_id는 화면에 남긴다."""
        if not self._scan_run and self._tally['sent']:
            self.logger.log('tally', None, f'집기 · 놓기 요청 {self._tally["sent"]}번 중 성공 {self._tally["ok"]}번')   # one_pick 집계(W132)
        summary = self.logger.finish(result)
        if not self._scan_run and getattr(self.io, 'w144_finish', None) is not None:
            # W144 촬영 ZIP(one_pick finally — 멈춘 조립도 덜 된 기록을 보존). 여기는 잠금 안이라 닫기는 작업 스레드(run_once)가 잠금 밖에서 한다
            self._w144_due = result == 'DONE'
        if self._scan_run:
            return                                  # 스캔은 CSV 와 scan_result 로 남기고 조립 build/2 로 저장하지 않는다
        if summary is not None:
            with self._lock:
                self.last_build = summary
                run_id = summary['run_id']
                self.pending_builds[run_id] = copy.deepcopy(summary)    # 저장이 확인될 때까지 run_id 별로 남긴다
                self._build_queued[run_id] = self._clock()
                # 디스크 보관이 끝난 것만 보낸다(재시작 복구를 위해). 보관을 안 쓰는 설정이면 바로 보낼 수 있다(재시작 복구 안 됨)
                if not self.logger.save_pending(run_id, copy.deepcopy(summary), lambda ok, r=run_id: self._pending_persisted(r, ok)):
                    self._build_ready.add(run_id)

    def _clear_run(self):
        """run_id 를 비운다(새 설계 · IDLE · 취소 · 정리 확인 · 스캔 시작). 닫히지 않은 기록이 있으면 STOPPED 로 닫는다. state/2 pick 도 비운다."""
        if self.logger.active:
            self._finish_run('STOPPED')
        self.logger.clear()
        self.run_id = self._pick = None              # 조립이 끝났으니 손목 그림 집기 순위도 뺀다(10/14 22시 PL)
        self._w144_reopen = False
        if self._scatter is not None:
            self._scatter.invalidate()
        self._recover_slow = False
        self._hmi_pause = self._hmi_recheck = False
        self._scan_run = False

    def _busy_reason(self):
        """거절할 때 쓰는 기존 코드: 정지 중이면 STOPPED, 그 밖에는 BUSY."""
        return 'STOPPED' if self.state == 'STOPPED' else 'BUSY'

    def _reject(self, reason, message):
        """(lock 안) 명령을 거절하고 이유를 알림으로 낸다. 상태는 그대로."""
        LOG.info('거절(%s): %s', reason or '-', message)
        self._publish(None, message)
        return False, reason

    def _set(self, state, message_id=None, message=''):
        """상태를 바꾸고 /d2/task/state 로 알린다."""
        with self._lock:
            LOG.info('%s → %s %s', self.state, state, message)
            if self.state == 'WAIT_SUPPLY' and state != 'WAIT_SUPPLY':
                # 사람 확인 뒤 할 일(_hold_then)은 그 WAIT_SUPPLY 의 [계속]에만 — 웹 끊김 · 정지로 떠나면 비운다. 남겨 두면 나중의 다른
                # [계속]이 엉뚱하게 VERIFY 로 갔다(10/14 점검 ② 2ⓐ). [계속]이 쓰는 길은 바꾸기 전에 이미 꺼내 갔다
                self._hold_then = None
            self.state = state
            self._epoch += 1
            self.logger.log('state', self.part_id, state)
            self._publish(message_id, message)

    def _publish(self, message_id, message):
        """state/2 한 건을 보낸다. run_id 는 조립 중(끝난 직후 포함)에만 값, message_id 는 IRD 2장 값 또는 null.

        시간 초과 뒤 정지 확인 대기 ERROR 에서는 어떤 재발행(거절 알림 포함)이든 'stop_pending' 으로 낸다 — 화면이 이 id 로
        [다시 시작]을 막으므로 중간 알림 하나로 풀리면 안 된다(10/14 PL E-117). stopped 확인(_enter_stopped)이 대기를 풀면 붙지 않는다.
        그리퍼 열기 대기(STOPPED 에서만 시작)와 정지 확인 대기(ERROR 에서만, STOPPED 진입 때 풀림)는 시험한 상태 · 명령 경로에서 같이 성립하지 않았다
        (test_task_timeouts — 모든 비동기 순서를 증명한 것은 아니다). 둘 다면 기존 gripper_opening 이 먼저다.
        pick(손목 그림 집기 순위 {image_seq, index, ranks, reason})은 있을 때만 싣고, 없으면 키를 뺀다(null 아님 — 10/14 22시 PL).
        """
        if self._gripper_busy and message_id != 'gripper_done':
            message_id = 'gripper_opening'          # 그리퍼 열기 답 전에는 어떤 알림이든 대기 표시를 유지한다 — 화면은 이것으로 팔 이동 버튼을 끈다(10/13 PL)
        elif self.state == 'ERROR' and self._timeout_pending:
            message_id = 'stop_pending'
        msg = {'schema': 'state/2', 'stamp': time.time(), 'mode': 'auto', 'state': self.state,
               'run_id': self.run_id, 'design_id': self.design_id, 'part_id': self.part_id,
               'message_id': message_id, 'message': message}
        if self._pick is not None:
            msg['pick'] = copy.deepcopy(self._pick)     # 받는 쪽(다리 · 비전 노드)이 바꿔도 다음 알림에 섞이지 않게 복사
        self.io.publish_state(msg)

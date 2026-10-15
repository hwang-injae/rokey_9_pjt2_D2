# -*- coding: utf-8 -*-
"""집기·놓기 계산 (ROS 없이 동작) — 블록 자세 -> 손가락 끝(TCP) 목표, 설계도(레시피) 읽기, 여러 설계 배치.

pick_place 노드·scene_manager 노드·run_recipe 도구가 같이 쓴다. ROS 없이 시험할 수 있게 노드와 나눴다(팀 규칙 ②).
원본: 박진용 recipe_demo/recipe_tcp.py (10/4~10/6 실기로 확인한 식). 궤적 시간은 MoveIt 이 매긴다.

좌표 약속
  - 위치 m, 쿼터니언 (x, y, z, w), 기준 base_link. 설정은 robot.yaml(cfg dict)에서 읽는다.
  - 블록 자세 = 블록 중심 + 블록 자신의 축(x = LENGTH 75, y = WIDTH 25, z = THICKNESS 15 mm 방향).
  - TCP(rg2_tcp, 손가락 끝 중심): 접근 축 +Z 가 아래(base -Z), 닫힘 축 Y 가 블록의 잡는 축과 평행.
  - rg2_tcp 와 두산 posx(활성 TCP GripperDA_v1 = 손가락 가운데 — 10/10 값 편집 [0, 2.7, 228], config/tcp.json) 틀: 같은 점이고 방향은 손목 Z 로 -90° 다르다
    R_rg2tcp = R_posx · Rz(-90°) → x_rg2 = -y_posx, y_rg2 = +x_posx, z 같음. 그래서 닫힘 축 = posx 의 x 축.
    (10/7 W134 observe_supply 자세에서 MoveIt FK 와 실측 posx 비교, 잔차 0.3°.) 카메라 보정은 rg2_tcp 틀이다(config/T_rg2tcp2camera — 10/10 W156, 옛 posx 틀 T_gripper2camera 는 지움).
  - 잡기 이름 6가지(IRD 2장) = <바닥 상태>_<LONG|SHORT>. 그리퍼 폭은 '손가락 사이에 끼우는 블록 축'만으로 정해진다.
"""
import csv
import hashlib
import json
import math
import os

AXES = {'LENGTH': 0, 'WIDTH': 1, 'THICKNESS': 2}
# 잡기 이름 -> 손가락 사이에 끼우는 블록 축
GRASP_AXIS = {'FLAT_SHORT': 'WIDTH', 'FLAT_LONG': 'LENGTH', 'EDGE_SHORT': 'THICKNESS',
              'EDGE_LONG': 'LENGTH', 'STAND_SHORT': 'THICKNESS', 'STAND_LONG': 'WIDTH'}
# 위를 향하는 블록 축 -> 바닥 상태 이름 (눕힘 15 · 옆으로 세움 25 · 위로 세움 75 mm 높이)
STATE_OF_UP = {'THICKNESS': 'FLAT', 'WIDTH': 'EDGE', 'LENGTH': 'STAND'}
LEVEL_TOL_DEG = 5.0       # 잡는 축이 수평에서 이만큼까지 기울어도 위에서 집을 수 있다고 본다
# 기울여 집기(잡기 TILT, 챌린지 A1 · W180): 폭 · 닫기 · 놓기 값은 FLAT_SHORT 것을 쓴다(윗면에 수직으로 좁은 폭을 문다)
TILT = 'TILT'
TILT_AS = 'FLAT_SHORT'
TILT_DEG = (3.0, 45.0)              # 기울여 집는 윗면 기울기 범위 (°) — 10/11 가상 45° · 실기 15.3°
TILT_LESS_DEG = (0, 5, 10, 15, 20)  # 손이 안 닿으면 블록 기울기보다 이만큼 덜 기울여 집는다 (°) — 남은 기울기는 놓을 때 넘어가며 눕는다
# 받침 눕히기(잡기 TIP, 10/13 실기 옆으로 섬 3번 · 섬 1번 — 시험 도구 one_pick tip_plan 을 옮김)
TIP = 'TIP'
TIP_DEG = {'WIDTH': 35.0, 'LENGTH': 20.0}   # 놓는 기울기 (°) — 넘어가는 각 atan(두께/높이) 31° · 11° 보다 크게
TIP_GRASP = {'WIDTH': 'EDGE_LONG', 'LENGTH': 'STAND_LONG'}   # 손가락 닫는 축 = 돌림축이어야 손가락 없는 쪽(두께 면)으로 넘어진다
TIP_LIFT_M = 0.04                   # 문 뒤 들기 · 기울여 놓기 위 높이 (m)
FLOOR_CLEAR_M = 0.003               # 가장 낮은 손가락 모서리 ↔ 받침 윗면 여유 (m)
# 받침 맞추기(잡기 CENTER_FLAT_SHORT · CENTER_FLAT_LONG, 10/13 one_pick center_at_p1) — 그 폭으로 물었다 바로 놓아 반대 축을 손 가운데로
CENTER_PREFIX = 'CENTER_'


# ---------------- 회전 ----------------
def rpy_matrix(r, p, y):
    """고정 축 x -> y -> z 회전 (ROS rpy, rad) -> 3x3 행렬. 열 = 돌린 물체의 x, y, z 축."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def rot_z(m, yaw):
    """행렬 m 을 base z 축으로 yaw (rad) 만큼 더 돌린다."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [[c * m[0][j] - s * m[1][j] for j in range(3)],
            [s * m[0][j] + c * m[1][j] for j in range(3)],
            list(m[2])]


def column(m, i):
    """행렬 m 의 i 번째 열 (물체의 i 번째 축 방향)."""
    return [m[0][i], m[1][i], m[2][i]]


def quat_from_axes(x, y, z):
    """열 x, y, z 로 된 회전 행렬 -> 쿼터니언 (x, y, z, w), w >= 0."""
    m00, m11, m22 = x[0], y[1], z[2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((y[2] - z[1]) / s, (z[0] - x[2]) / s, (x[1] - y[0]) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        q = (0.25 * s, (y[0] + x[1]) / s, (z[0] + x[2]) / s, (y[2] - z[1]) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        q = ((y[0] + x[1]) / s, 0.25 * s, (z[1] + y[2]) / s, (z[0] - x[2]) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2
        q = ((z[0] + x[2]) / s, (z[1] + y[2]) / s, 0.25 * s, (x[1] - y[0]) / s)
    n = math.sqrt(sum(v * v for v in q))
    q = tuple(v / n for v in q)
    return q if q[3] >= 0 else tuple(-v for v in q)


def quat_axes(q):
    """쿼터니언 (x, y, z, w) -> 돌린 x, y, z 축 세 개."""
    x, y, z, w = q
    return ([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)],
            [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
            [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])


def matrix_from_quat(q):
    """쿼터니언 -> 3x3 행렬 (열 = 돌린 축)."""
    ax = quat_axes(q)
    return [[ax[j][i] for j in range(3)] for i in range(3)]


def flip_about_z(q):
    """TCP Z 축으로 180° 돌린 자세 (같은 잡기, 손가락 좌우만 바뀜). 둘 중 관절이 덜 도는 쪽을 실행 때 고른다."""
    x, y, z, w = q
    return (y, -x, w, -z) if -z >= 0 else (-y, x, -w, z)


def tcp_quat_down(psi):
    """TCP 자세: 접근 축 +Z = 아래, 닫힘 축 +Y = base 수평 방향 psi (rad)."""
    return quat_from_axes([-math.sin(psi), math.cos(psi), 0.0], [math.cos(psi), math.sin(psi), 0.0], [0.0, 0.0, -1.0])


def wrap_half(a):
    """닫힘 방향은 180° 대칭이라 [-90°, 90°) 로 정리한다 (rad)."""
    return (a + math.pi / 2) % math.pi - math.pi / 2


def close_angle_deg(q):
    """TCP 닫힘 축(Y)의 base 수평 방향 (도). 화면 표시용."""
    y = quat_axes(q)[1]
    return math.degrees(math.atan2(y[1], y[0]))


# ---------------- 블록 ----------------
def up_axis(rot):
    """위(base Z)를 향하는 블록 축 이름 (LENGTH / WIDTH / THICKNESS)."""
    return ['LENGTH', 'WIDTH', 'THICKNESS'][max(range(3), key=lambda k: abs(rot[2][k]))]


def grasp_name(rot, axis):
    """블록 회전과 끼우는 축 -> 잡기 이름 (예: FLAT_SHORT). 잡을 수 없는 조합이면 ValueError."""
    up = up_axis(rot)
    if axis == up:
        raise ValueError(f'위를 향하는 축({up})은 위에서 집을 때 끼울 수 없다')
    other = ({'LENGTH', 'WIDTH', 'THICKNESS'} - {up, axis}).pop()
    longer = AXES[axis] < AXES[other]          # 축 번호가 작을수록 긴 쪽 (75 > 25 > 15)
    return f'{STATE_OF_UP[up]}_{"LONG" if longer else "SHORT"}'


def half_height(rot, size):
    """블록 회전과 크기 (m) -> 위아래 반높이 (m)."""
    return sum(abs(rot[2][i]) * size[i] / 2 for i in range(3))


def slot_block_pose(cfg, slot):
    """공급 칸 slot (1부터) 에 놓인 블록의 중심 (m) 과 회전 행렬.

    칸의 block_up = 위를 향하는 블록 축, yaw_deg = 블록 긴 쪽(LENGTH) 방향. LENGTH 로 세우면 WIDTH 방향.
    칸 바닥 높이는 작업면 table_z_m 하나를 쓴다(10/8 PL E-62 — 칸 맞춤 틀을 깔아 바닥이 올라가면 다시 정함).
    칸 번호가 없으면 ValueError.
    """
    slots = cfg['supply_slots']
    if not 1 <= slot <= len(slots):
        raise ValueError(f'공급 칸 {slot} 이 없다 (1 ~ {len(slots)})')
    st = slots[slot - 1]
    up, yaw = st.get('block_up', 'THICKNESS'), math.radians(st['yaw_deg'])
    if up == 'THICKNESS':
        rot = rpy_matrix(0.0, 0.0, yaw)
    elif up == 'WIDTH':       # 긴 쪽(x) 축으로 90° 굴려 WIDTH 가 위로
        rot = rot_z(rpy_matrix(math.pi / 2, 0.0, 0.0), yaw)
    elif up == 'LENGTH':      # y 축으로 -90° 세워 LENGTH 가 위로. 그때 WIDTH 가 +y 라서 yaw - 90° 를 돌린다
        rot = rot_z(rpy_matrix(0.0, -math.pi / 2, 0.0), yaw - math.pi / 2)
    else:
        raise ValueError(f'공급 칸 {slot}: block_up 은 THICKNESS / WIDTH / LENGTH ({up})')
    return (st['x_m'], st['y_m'], cfg['table_z_m'] + half_height(rot, cfg['block_actual_m'])), rot


# ---------------- RG2 손가락 끝 높이 ----------------
# RG2 손가락은 호를 그리며 닫혀서 열림 폭에 따라 손가락 끝 높이가 바뀐다 (75 mm 블록을 물면 다 닫음보다 16.8 mm 위).
# URDF rg2_tcp 는 고정이라 폭별 끝 높이를 계산해 보정한다. 식·상수는 onrobot_rg_control 드라이버(RG2)와 같다.
RG2_L1, RG2_L3, RG2_TH1, RG2_TH3, RG2_DY, RG2_DZ = 0.108505, 0.055, 1.41371, 0.76794, -0.0144, 0.1095 + 0.0427


def rg2_finger_angle(display_width_m):
    """그리퍼 표시(피드백) 폭 (m) -> 손가락 관절 각 (rad). 드라이버 finger_joint = URDF rg2_finger_joint(브링업이 이름만 바꿔 넘김)."""
    c = (display_width_m / 2 - RG2_DY - RG2_L1 * math.cos(RG2_TH1)) / RG2_L3
    return math.acos(max(-1.0, min(1.0, c))) - RG2_TH3


def rg2_tip_height_m(display_width_m):
    """그리퍼 표시(피드백) 폭 (m) -> 그리퍼 바닥에서 손가락 끝까지 높이 (m)."""
    return RG2_L3 * math.sin(rg2_finger_angle(display_width_m) + RG2_TH3) + RG2_DZ


def held_finger_angle(cfg, grasp):
    """잡기 grasp 로 블록을 물었을 때 손가락 관절 각 (rad) — 쥔 상태로 미리 계획해 볼 때 MoveIt 손가락 자세로 쓴다."""
    return rg2_finger_angle(_grip_display_m(cfg, grasp))


def open_finger_angle(cfg, width_m):
    """손가락 사이 실제 폭 width_m (m) 로 열었을 때 손가락 관절 각 (rad) — 집기 전 연 폭으로 미리 충돌을 볼 때 쓴다."""
    return rg2_finger_angle(width_m + cfg['gripper']['feedback_offset_m'])


def _grip_display_m(cfg, grasp):
    """잡기 grasp 로 블록을 물었을 때 그리퍼 표시 폭 (m) = 실제 폭 + 표시 오프셋."""
    return cfg['grasp_width_m'][grasp] + cfg['gripper']['feedback_offset_m']


def _tcp_z_for_tip(cfg, tip_z, grasp):
    """블록을 문 폭에서 실제 손가락 끝이 tip_z 에 오게 하는 rg2_tcp z (m). 보정점이 없으면 ValueError."""
    fh = cfg['gripper'].get('finger_height') or {}
    if fh.get('touch_tcp_z_m') is None or fh.get('touch_display_width_m') is None:
        raise ValueError('손가락 끝 높이 보정점(gripper.finger_height)이 robot.yaml 에 없다')
    drop = rg2_tip_height_m(_grip_display_m(cfg, grasp)) - rg2_tip_height_m(fh['touch_display_width_m'])
    return tip_z + (fh['touch_tcp_z_m'] - cfg['table_z_m']) + drop


def _tcp_z_from_taught(cfg, slot_cfg, top_z, grasp):
    """교시한 집는 높이 기준 TCP z = 교시 높이 + (윗면 높이 차) + (무는 폭이 달라 생기는 손가락 끝 높이 차)."""
    size = dict(zip(('LENGTH', 'WIDTH', 'THICKNESS'), cfg['block_actual_m']))
    top_ref = cfg['table_z_m'] + size[slot_cfg.get('block_up', 'THICKNESS')]
    drop = rg2_tip_height_m(_grip_display_m(cfg, grasp)) - rg2_tip_height_m(slot_cfg['grip_display_width_m'])
    return slot_cfg['tcp_z_m'] + (top_z - top_ref) + drop


def tcp_target(cfg, center, rot, grasp, slot_cfg=None):
    """블록 중심 (m)·회전·잡기 이름 -> TCP 목표 (위치, 쿼터니언) 와 TCP 에 붙일 잡은 블록 상자.

    TCP 높이: 블록을 문 폭에서 실제 손가락 끝이 '윗면 - grasp_depth_m' 에 오게 한다.
    slot_cfg 에 교시 높이(tcp_z_m)가 있으면 그것을 기준으로 한다 (찍은 높이 우선).
    반환: (xyz, quat, held) — held = {'size_m': TCP 축별 길이, 'offset_m': TCP 기준 블록 중심}.
    잡는 축이 수평이 아니면 ValueError (위에서 수직으로 못 집음).
    """
    axis = GRASP_AXIS[grasp]
    gdir = column(rot, AXES[axis])
    tilt = math.degrees(math.asin(min(1.0, abs(gdir[2]))))
    if tilt > LEVEL_TOL_DEG:
        raise ValueError(f'잡는 축이 수평이 아님 (기울기 {tilt:.1f}도)')
    q = tcp_quat_down(wrap_half(math.atan2(gdir[1], gdir[0])))
    size = cfg['block_actual_m']          # 실제 블록 치수 (설계 치수보다 최대 0.5 mm 작다)
    top = center[2] + half_height(rot, size)
    if slot_cfg is not None and slot_cfg.get('tcp_z_m') is not None:
        tcp_z = _tcp_z_from_taught(cfg, slot_cfg, top, grasp)
    else:
        tcp_z = _tcp_z_for_tip(cfg, top - cfg['grasp_depth_m'], grasp)
    # 잡은 블록 상자: TCP 축마다 블록이 차지하는 길이. TCP z 는 아래를 향하므로 TCP 보다 위인 블록 중심은 음수
    cols = [column(rot, i) for i in range(3)]
    held_size = [sum(abs(sum(a * b for a, b in zip(t, cols[i]))) * size[i] for i in range(3)) for t in quat_axes(q)]
    held = {'size_m': held_size, 'offset_m': [0.0, 0.0, -(center[2] - tcp_z)]}
    return (center[0], center[1], tcp_z), q, held


def move_speed_scale(cfg, speed_ratio):
    """move_to 의 speed_ratio -> MoveIt 속도 비율 (robot.yaml speed_scale x speed_ratio, IRD 4.2 MoveTo · W121 C-9).

    입력: speed_ratio 0 = 평소(1.0배), 0 < 값 ≤ 1 = 그만큼 느리게(예: 다시 시작 뒤 첫 이동 0.5).
    반환: MoveIt 에 넘길 비율, 범위 밖(음수 · 1 초과 · NaN)이면 None — 평소보다 빠르게는 가지 않는다.
    """
    if speed_ratio == 0:
        return cfg['speed_scale']
    if not 0 < speed_ratio <= 1:          # NaN 도 여기서 걸린다
        return None
    return cfg['speed_scale'] * speed_ratio


def scan_structure_box(cfg, target, current_deg, tol_deg=1.0):
    """스캔 이동 때 사람이 쌓은 구조물 자리에 둘 상자 (계획 요청에만 싣는다 — 실제 장면은 안 바뀐다, W186).

    사람이 쌓은 구조물은 장면에 없어서 MoveIt 이 모르고, 대각선 촬영 자세는 구조물 가까이 지난다(10/12 실기 — 로봇 쪽
    대각선은 상자와 여유 20 mm). 그래서 scan.poses 의 두 번째 자세부터 그 자세로 갈 때와, 그 자세에서 떠날 때만 상자를 둔다.
    첫 자세(observe)로 갈 때는 두지 않는다 — 조립 중에도 observe 로 가며, 그때 손이 이 상자 안(놓은 블록 옆)에 있을 수 있다.
    입력: target = move_to 자세 이름 · current_deg = 지금 관절값(도, 6개) · tol_deg = '그 자세에 있음'으로 볼 관절 차이(도).
    반환: (중심 (x, y, z) m, 크기 (x, y, z) m) — 조립 원점 위, 바닥 = 작업면(table_z_m), 크기 = robot.yaml scan.structure_box_m.
          둘 필요가 없거나 scan.structure_box_m 이 없으면 None.
    """
    scan = cfg.get('scan') or {}
    size = scan.get('structure_box_m')
    later = [p for p in (scan.get('poses') or [])[1:] if f'{p}_pose' in cfg]
    if not size or not later:
        return None
    here = any(max(abs(a - b) for a, b in zip(cfg[f'{p}_pose']['joints_deg'], current_deg)) <= tol_deg for p in later)
    if target not in later and not here:
        return None
    o = cfg['assembly_origin']
    return (o['x_m'], o['y_m'], cfg['table_z_m'] + size[2] / 2), tuple(size)


def pick_open_width(cfg, grasp, open_width_m):
    """집기 전에 여는 실제 폭 (m) (IRD 4.2 PickPlace open_width_m · W121 C-10).

    입력: grasp = 잡기 이름, open_width_m 0 = robot.yaml grasp_open_pick_m.<grasp>, 값이 있으면 그 폭(흩어진 블록 틈에 맞출 때).
    반환: 열 폭 (m), 닫는 방향 블록 폭(grasp_width_m) 이하이면 None — 손가락이 블록 위에 내려앉는다.
    너무 큰 값은 그리퍼 노드가 RG2 최대 열림으로 자른다.
    """
    if open_width_m == 0:
        return cfg['grasp_open_pick_m'][grasp]
    if not open_width_m > cfg['grasp_width_m'][grasp]:   # NaN 도 여기서 걸린다
        return None
    return open_width_m


def pick_place_tcp(cfg, pick_center, pick_rot, place_center, place_rot, grasp, slot=None):
    """집을 블록·놓을 자리의 블록 자세 -> 집기·놓기 TCP 목표 둘과 잡은 블록 상자.

    놓는 높이는 assembly_origin 에 교시 높이가 있으면 그것을 기준으로 한다.
    공급 칸 slot 에 교시 높이가 있으면 블록을 표준보다 깊거나 얕게 문다. 놓을 때도 같은 깊이로 물고 있으므로
    놓는 TCP 를 그 차이만큼 옮긴다 (안 그러면 블록 바닥이 작업대를 누른다, 10/4 실기).
    반환: (pick_xyz, pick_q, place_xyz, place_q, held). 계산할 수 없거나 공급·놓기 자세가 다르면 ValueError.
    """
    # 위에서 집어 그대로 내려놓으므로 블록 자세(어느 면이 바닥인지)를 바꿀 수 없다: 공급 자세 = 놓을 자세여야 한다
    if up_axis(pick_rot) != up_axis(place_rot):
        raise ValueError(f'공급 블록은 {up_axis(pick_rot)} 가 위인데 놓을 자세는 {up_axis(place_rot)} 가 위 — 같은 자세의 칸이 필요하다')
    slot_cfg = cfg['supply_slots'][slot - 1] if slot else None
    # 공급 칸은 잡기가 하나로 고정이다: 다른 잡기로 집으면 손목을 돌려 찍은 방향과 달라진다 (10/6 실기)
    if slot_cfg is not None and slot_cfg.get('grasp') not in (None, grasp):
        raise ValueError(f'공급 칸 {slot} 은 {slot_cfg["grasp"]} 칸인데 잡기는 {grasp}')
    # 카메라로 잰 윗면이 낮게 나오면 블록 바닥이 작업대 아래로 계산되고, 그 자리에 붙인 쥔 블록 상자가 작업대와 겹쳐
    # 들어 올리기부터 막힌다(10/11 실기: 윗면 −4.2 mm → 바닥 1 mm 아래 → lift CheckStartStateCollision held-table).
    # 블록은 작업대를 뚫을 수 없으므로 바닥을 작업대 높이로 올린다 — 잡는 높이는 그만큼(수 mm 이내)만 바뀐다
    floor = cfg['table_z_m'] + half_height(pick_rot, cfg['block_actual_m'])
    if pick_center[2] < floor:
        pick_center = (pick_center[0], pick_center[1], floor)
    pick_xyz, pick_q, _ = tcp_target(cfg, pick_center, pick_rot, grasp, slot_cfg)
    # 놓는 높이는 조립 원점에서 교시한 높이(tcp_z_m, record_cell 의 m)가 있으면 그것 기준, 없으면 손가락 끝 보정식 (원본과 같음)
    place_xyz, place_q, held = tcp_target(cfg, place_center, place_rot, grasp, cfg['assembly_origin'])
    if slot_cfg is not None and slot_cfg.get('tcp_z_m') is not None:
        std_z = tcp_target(cfg, pick_center, pick_rot, grasp)[0][2]
        delta = pick_xyz[2] - std_z
        place_xyz = (place_xyz[0], place_xyz[1], place_xyz[2] + delta)
        held['offset_m'][2] += delta
    return pick_xyz, pick_q, place_xyz, place_q, held


def place_offset_m(cfg, place_rot, grasp):
    """조립 자리 놓기 보정(10/15 PL) — 잡기 FLAT_SHORT 로 놓는 블록만, 놓기 TCP 를 블록 긴 축(LENGTH) 방향으로 옮길 양.

    FLAT_SHORT 는 손가락이 폭만 물어 긴 축 어디를 잡았는지가 놓는 자리에 그대로 남는다(FLAT_LONG 은 긴 축을 물어 저절로 가운데).
    10/15 마스크 실기(R20261015_152525_8d83): FLAT_SHORT 로 놓은 다리 8개 dy −1.43 ~ −3.63 mm(긴 축 = base y) · FLAT_LONG 좌판 3개 긴 축 dx −0.13 ~ 0.5 mm,
    민범진 사진 22장 분석 −2.4 ± 0.8 mm. robot.yaml motion.place_offset_flat_short_m [x, y](base, m)를 긴 축에 내린 만큼만 쓴다 —
    긴 축이 base x 인 블록에는 y 보정이 0 이 된다(그 블록은 아직 잰 값이 없어 옮기지 않는다).
    입력: cfg(robot.yaml dict), place_rot(놓을 블록 회전 3x3), grasp(잡기 이름). 출력: (dx, dy, 0) m. 키가 없거나 [0, 0] 이면 (0, 0, 0) — 보정 끔.
    """
    off = (cfg.get('motion') or {}).get('place_offset_flat_short_m') or [0.0, 0.0]
    if grasp != 'FLAT_SHORT' or not any(off):
        return (0.0, 0.0, 0.0)
    u = column(place_rot, AXES['LENGTH'])
    k = off[0] * u[0] + off[1] * u[1]
    return (k * u[0], k * u[1], 0.0)


def tilt_pick_place_tcp(cfg, pick_center, pick_rot, place_center, place_rot, less_deg):
    """기울어진(걸친) 눕힌 블록을 윗면에 수직으로 기울인 공구로 FLAT_SHORT 폭으로 물어 눕혀 놓는 TCP 목표 (잡기 TILT).

    입력: pick_center · pick_rot = 기울어진 블록 중심 (m) · 회전(열 = 길이 · 폭 · 두께 축, 두께 축이 윗면 법선),
          place_center · place_rot = 눕혀 놓을 자리, less_deg = 블록 기울기보다 덜 기울일 각 (°, 손이 안 닿을 때).
    반환: (grips, place_xyz, place_q, held) — grips = [(TCP xyz, quat, 접근 축 단위 벡터)] 카메라(공구 −x)가 위로 가는 쪽 먼저.
          덜 기울일 게 없으면(less > 기울기) None. 눕힌 블록이 아니거나 기울기가 TILT_DEG 밖이면 ValueError.
    공구 축은 윗면 법선과 연직 사이 평면에서 연직으로부터 (기울기 − less), 닫는 방향 = 블록 폭 ± — 옆면 두 쪽을 문다.
    손 안에 남는 기울기 d 만큼 쥔 상자를 키우고, 놓을 때는 낮은 끝이 다른 블록처럼 place_up 위에 오게 TCP 를 올린다 —
    놓으면 남은 기울기만큼 넘어가 눕는다(10/11 가상 45° · 실기 15.3°, 시험 도구 one_pick 에서 옮김).
    """
    if up_axis(pick_rot) != 'THICKNESS' or up_axis(place_rot) != 'THICKNESS':
        raise ValueError('기울여 집기는 눕힌 블록을 눕혀 놓을 때만')
    n = column(pick_rot, 2)
    if n[2] < 0:
        n = [-v for v in n]
    w = column(pick_rot, 1)
    tilt = math.acos(max(-1.0, min(1.0, n[2])))
    if not TILT_DEG[0] <= math.degrees(tilt) <= TILT_DEG[1]:
        raise ValueError(f'기울기 {math.degrees(tilt):.1f}° 가 {TILT_DEG[0]:.0f} ~ {TILT_DEG[1]:.0f}° 밖')
    if less_deg and math.radians(less_deg) > tilt:
        return None
    L, W, T = cfg['block_actual_m']
    tip = _tcp_z_for_tip(cfg, 0.0, 'FLAT_SHORT')          # 문 손가락 끝 → rg2_tcp 공구 축 거리 (수직 집기 높이 식과 같은 값)
    top = [c + v * T / 2 for c, v in zip(pick_center, n)]
    hlen = math.hypot(n[0], n[1])
    phi = tilt - math.radians(less_deg)
    a = [0.0, 0.0, 1.0] if hlen < 1e-9 else [n[0] / hlen * math.sin(phi), n[1] / hlen * math.sin(phi), math.cos(phi)]
    tcp = [t - v * cfg['grasp_depth_m'] + av * tip for t, v, av in zip(top, n, a)]
    grips = []
    for sgn in (1, -1):
        z = [-v for v in a]
        y = [sgn * v for v in w]
        dot = sum(p * q for p, q in zip(y, z))
        y = [p - dot * q for p, q in zip(y, z)]
        ny = math.sqrt(sum(v * v for v in y))
        y = [v / ny for v in y]
        x = [y[1] * z[2] - y[2] * z[1], y[2] * z[0] - y[0] * z[2], y[0] * z[1] - y[1] * z[0]]
        grips.append((-x[2], (tuple(tcp), quat_from_axes(x, y, z), tuple(a))))
    grips.sort(key=lambda g: -g[0])
    d = tilt - phi
    held = {'size_m': [L * math.cos(d) + T * math.sin(d), W, L * math.sin(d) + T * math.cos(d)],
            'offset_m': [0.0, 0.0, tip + T / 2 - cfg['grasp_depth_m']]}
    place_xyz, place_q, _ = tcp_target(cfg, place_center, place_rot, 'FLAT_SHORT')
    place_xyz = (place_xyz[0], place_xyz[1], place_xyz[2] + L / 2 * math.sin(d) + T / 2 * (math.cos(d) - 1))
    return [g for _, g in grips], place_xyz, place_q, held


def _rot_axis(axis, ang):
    """단위 축 axis 둘레 ang (rad) 회전 행렬 (Rodrigues)."""
    x, y, z = axis
    c, s, v = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return [[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v]]


def _mul(a, b):
    """3x3 행렬 곱."""
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(m, v):
    """3x3 행렬 x 벡터."""
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def _transpose(m):
    """3x3 전치."""
    return [[m[j][i] for j in range(3)] for i in range(3)]


def tip_place_tcp(cfg, pick_center, pick_rot, land_center, flip):
    """받침에 옆으로 · 세워 놓은 블록을 손가락 축 둘레로 기울여 놓아 눕히는 TCP 목표 (잡기 TIP, 시험 도구 one_pick tip_plan 과 같은 식).

    입력: pick_center · pick_rot = 받침 위 선 블록 중심 (m) · 회전(열 = 길이 · 폭 · 두께 축), land_center = 넘어져 누울 블록 중심 (m — z 는
          받침 윗면 + 두께/2), flip = 공구를 수직축 180° 돌린 쪽(대칭 두 손목 방향 중 하나).
    잡기 = TIP_GRASP[위 축](옆으로 섬 EDGE_LONG · 섬 STAND_LONG). 공구를 닫는 축 둘레로 TIP_DEG 만큼, 블록 윗부분이 로봇 반대쪽(base → 받침
    방향)으로 가게 기울이고, 블록의 가장 낮은 모서리가 받침 + place_up_m 에 오게 둔다. 그 모서리를 축으로 넘어진 블록 중심이 land_center 에
    오게 모서리 자리 = land_center − 넘어지는 방향 × 높이/2.
    반환: (dict(grasp, tcp0 · q0 = 집는 TCP, tcp1 · q1 = 기울여 놓는 TCP, held, phi_deg, d), '') 또는 (None, 이유 글자) — 손가락 끝이
    받침 + FLOOR_CLEAR_M 아래면 None. 눕힌 블록이면 ValueError. 바깥 영향 없음.
    """
    up = up_axis(pick_rot)
    if up not in TIP_GRASP:
        raise ValueError('눕히기(TIP)는 옆으로 선 · 선 블록만')
    grasp = TIP_GRASP[up]
    L, W, T = cfg['block_actual_m']
    H = {'WIDTH': W, 'LENGTH': L}[up]
    xyz, qt, held = tcp_target(cfg, pick_center, pick_rot, grasp)
    rt = matrix_from_quat(qt)
    if flip:
        rt = [[-r[0], -r[1], r[2]] for r in rt]              # 공구 z 축 둘레 180°: x · y 열 부호 바꿈
    rn = math.hypot(land_center[0], land_center[1])
    radial = (land_center[0] / rn, land_center[1] / rn)
    a = column(rt, 1)                                         # 손가락 닫는 축(수평)
    d = [a[1], -a[0], 0.0]                                    # a x z — 양(+)으로 돌리면 윗부분이 가는 방향
    sgn = 1 if d[0] * radial[0] + d[1] * radial[1] > 0 else -1
    phi = math.radians(TIP_DEG[up]) * sgn
    d = [v * sgn for v in d]
    rr = _rot_axis(a, phi)
    rb1, rt1 = _mul(rr, pick_rot), _mul(rr, rt)
    corners = [_apply(rb1, [i * L / 2, j * W / 2, k * T / 2]) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]
    zmin = min(c[2] for c in corners)
    lows = [c for c in corners if c[2] < zmin + 1e-6]
    low = [sum(c[i] for c in lows) / len(lows) for i in range(3)]       # 가장 낮은 모서리(꼭짓점 둘) 가운데
    support_z = land_center[2] - T / 2
    pivot = [land_center[0] - d[0] * H / 2, land_center[1] - d[1] * H / 2, support_z + cfg['motion']['place_up_m']]
    c1 = [p - v for p, v in zip(pivot, low)]
    rel = _apply(_mul(rt1, _transpose(rt)), [t - c for t, c in zip(xyz, pick_center)])
    tcp1 = [c + v for c, v in zip(c1, rel)]
    reach, half = _tcp_z_for_tip(cfg, 0.0, grasp), cfg['finger']['width_m'] / 2
    z1, x1 = column(rt1, 2), column(rt1, 0)
    tip = min(tcp1[2] + z1[2] * reach + x1[2] * k * half for k in (1, -1))
    if tip < support_z + FLOOR_CLEAR_M:
        return None, f'손가락 끝이 받침 위 {(tip - support_z) * 1000:.1f} mm'
    return {'grasp': grasp, 'tcp0': tuple(xyz), 'q0': quat_from_axes(*[column(rt, i) for i in range(3)]),
            'tcp1': tuple(tcp1), 'q1': quat_from_axes(*[column(rt1, i) for i in range(3)]), 'held': held,
            'phi_deg': math.degrees(phi), 'd': d}, ''


# ---------------- 세우기 교시 동작 (W131 — 한세교 교시, 10/13 박진용 옮김) ----------------
# robot.yaml standup_taught.<case> 의 교시 자세 칸 = 펜던트로 교시한 순서
STANDUP_ORDER = ('entry', 'pick', 'lift_up', 'rotate', 'place')
# 잡기 이름 -> 교시 동작 (IRD 2장, 10/13): 세움(LENGTH 위) = high · 옆세움(WIDTH 위) = side
STANDUP_GRASPS = {'STANDUP_HIGH': 'high', 'STANDUP_SIDE': 'side'}
STANDUP_UP = {'high': 'LENGTH', 'side': 'WIDTH'}
# 쥔 블록 상자를 축마다 줄이는 길이 (면마다 0.5 mm, 놓인 블록과 같음) — 받침 윗면에 딱 붙어 있어 그대로면 붙이자마자 겹쳐 보인다
STANDUP_HELD_SHRINK_M = 0.001


def zyz_matrix(a_deg, b_deg, c_deg):
    """두산 펜던트 회전 A · B · C (도, ZYZ 오일러 = Rz(A) · Ry(B) · Rz(C)) -> 3x3 행렬 (열 = 돌린 축)."""
    def rz(t):
        """z 축 회전 행렬 (rad)."""
        return [[math.cos(t), -math.sin(t), 0.0], [math.sin(t), math.cos(t), 0.0], [0.0, 0.0, 1.0]]

    def ry(t):
        """y 축 회전 행렬 (rad)."""
        return [[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]]

    return _mul(_mul(rz(math.radians(a_deg)), ry(math.radians(b_deg))), rz(math.radians(c_deg)))


def posx_to_rg2tcp(task):
    """펜던트 posx [x, y, z mm, A, B, C 도] (BASE · 활성 TCP GripperDA_v1) -> rg2_tcp 목표 (위치 m, 쿼터니언 x y z w).

    위치는 같은 점이고 방향은 R_rg2tcp = R_posx · Rz(-90°): x_rg2 = -y_posx, y_rg2 = +x_posx, z 같음(한세교 가상 세움 표로 확인).
    값이 6개가 아니면 ValueError.
    """
    if len(task) != 6:
        raise ValueError(f'posx 는 6개여야 한다: {task}')
    r = zyz_matrix(*task[3:])
    x, y, z = (column(r, i) for i in range(3))
    return tuple(v / 1000.0 for v in task[:3]), quat_from_axes([-v for v in y], x, z)


def standup_case(cfg, pick_center, pick_rot, place_center, place_rot, grasp):
    """세우기 요청을 확인하고 교시 동작 이름을 낸다. 잡기 이름(STANDUP_HIGH · STANDUP_SIDE)이 동작을 정한다.

    입력: 블록 중심 (m) · 회전 (집기 · 놓기), 잡기 이름. robot.yaml standup_taught · spare_spots 첫 자리를 읽는다.
    반환: 'high' | 'side'. 눕힌 블록이 아니거나, 놓을 자세의 위 축이 잡기와 다르거나, 집기 · 놓기 블록 중심이 받침 자리에서
    spot_tolerance_m 넘게 떨어지면 ValueError — 교시 좌표는 그 자리 하나에서만 맞다. 바깥 영향 없음.
    """
    case = STANDUP_GRASPS.get(grasp)
    if case is None:
        raise ValueError(f'세우기 잡기가 아니다: {grasp}')
    if up_axis(pick_rot) != 'THICKNESS':
        raise ValueError(f'세우기는 눕힌 블록에서만 — 집을 블록은 {up_axis(pick_rot)} 가 위')
    if up_axis(place_rot) != STANDUP_UP[case]:
        raise ValueError(f'{grasp} 는 {STANDUP_UP[case]} 를 위로 세운다 — 놓을 자세는 {up_axis(place_rot)} 가 위')
    spot, tol = cfg['spare_spots'][0], cfg['standup_taught']['spot_tolerance_m']
    for name, c in (('집기', pick_center), ('놓기', place_center)):
        d = math.hypot(c[0] - spot['x_m'], c[1] - spot['y_m'])
        if d > tol:
            raise ValueError(f'{name} 블록 중심이 받침 자리에서 {d * 1000:.1f} mm 떨어짐 (허용 {tol * 1000:.0f} mm)')
    return case


def standup_targets(cfg, case):
    """교시 동작 case('high' · 'side')의 MoveL 목표 (rg2_tcp, 위치 m · 쿼터니언) 를 순서대로 만든다.

    반환: {이름: (xyz, quat)} — STANDUP_ORDER 다섯 자세 + 'release_up'(놓기 자세에서 Z+ 로 lift_after_place_m,
    방향 그대로 — 연 손가락을 세운 블록에서 위로 빼는 자리. 교시에 없어 이것만 계산한다, 10/13 한세교).
    교시 칸이 없으면 KeyError.
    """
    taught = cfg['standup_taught']
    out = {name: posx_to_rg2tcp(taught[case][name]) for name in STANDUP_ORDER}
    (x, y, z), q = out['place']
    out['release_up'] = ((x, y, z + taught['lift_after_place_m']), q)
    return out


def standup_held_box(cfg, pick_center, pick_rot, tcp_xyz, tcp_q):
    """교시 파지 자세에서 쥔 블록 상자 (SceneAttach held — 크기 m · TCP 기준 블록 중심 m · TCP 기준 방향 쿼터니언 'quat').

    기울여 잡아 블록 축이 TCP 축과 어긋나므로 방향(held_quat)을 같이 보낸다 — 장면에 실제 블록 모양 그대로 붙는다.
    크기 = 블록 실측에서 축마다 STANDUP_HELD_SHRINK_M 뺀 값. 블록 중심 = 요청 pick_pose 중심 그대로. 바깥 영향 없음.
    """
    axes = quat_axes(tcp_q)
    d = [c - p for c, p in zip(pick_center, tcp_xyz)]
    rel = [[sum(axes[i][k] * pick_rot[k][j] for k in range(3)) for j in range(3)] for i in range(3)]   # R_tcpᵀ · R_block
    return {'size_m': [v - STANDUP_HELD_SHRINK_M for v in cfg['block_actual_m']],
            'offset_m': [sum(a * b for a, b in zip(t, d)) for t in axes],
            'quat': quat_from_axes(*(column(rel, i) for i in range(3)))}


def held_block_pose(tcp_xyz, tcp_q, held):
    """TCP 자세 (m · 쿼터니언) 에서 쥔 블록 상자 held(standup_held_box)의 base 자세 -> (중심 m, 쿼터니언).

    놓는 순간 손에 붙은 블록이 실제로 있는 자리 — 떼기(SceneAttach 놓은 자리)에 쓰면 장면 블록이 열린 손가락과 같은 자리라
    목표 자세(place_pose)와의 몇 mm 차이로 손가락과 겹쳐 보이지 않는다.
    """
    r_tcp, r_rel = matrix_from_quat(tcp_q), matrix_from_quat(held['quat'])
    center = tuple(tcp_xyz[i] + sum(r_tcp[i][k] * held['offset_m'][k] for k in range(3)) for i in range(3))
    return center, quat_from_axes(*(column(_mul(r_tcp, r_rel), i) for i in range(3)))


# ---------------- 설계도(레시피) ----------------
RECIPE_SUFFIXES = ('_recipe.json',)    # 레시피 = 구조 파일 <모델ID>_recipe.json (E-69 — 옛 이름 .recipe.json 은 W139 뒤 뺌)
PLACEMENTS_SUFFIX = '_placements.csv'  # 조립 방법 원본 — 구조 파일과 같은 폴더 · 같은 모델 ID (E-69)
RECIPE_SCHEMA = 'recipe/4.0'          # 10/13 PLM: 자재 stocks + template.composition[역할].parts[그룹] 트리
PLACEMENTS_SCHEMA = 'placements/4.0'


def same_major(got, expected):
    """형식 이름이 앞자리까지 같은지(IRD 6장 형식 버전 규칙 — 뒷자리는 호환 변경). d2_task.recipe_document.same_major 와 같은 식이다 —
    d2_task 가 d2_motion 을 쓰므로 여기서 불러 쓸 수 없어 한 줄을 둔다."""
    if not isinstance(got, str) or '/' not in got:
        return False
    name, ver = got.split('/', 1)
    exp_name, exp_ver = expected.split('/', 1)
    return name == exp_name and ver.split('.')[0] == exp_ver.split('.')[0]


def recipe_files(folder):
    """folder 안의 레시피(구조) 파일(<모델ID>_recipe.json) 경로 목록 (이름 순).
    조립 방법 파일(_placements.csv)은 load_recipe 가 같이 읽으므로 목록에 넣지 않는다. 폴더가 없으면 빈 목록."""
    if not os.path.isdir(folder):
        return []
    return sorted(os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(RECIPE_SUFFIXES))


def recipe_name(path):
    """레시피 파일 경로 -> 보여 줄 이름 (모델 ID, 예 001_CHAIR_BENCH_V000)."""
    name = os.path.basename(path)
    for suf in RECIPE_SUFFIXES:
        if name.endswith(suf):
            return name[:-len(suf)]
    return name


def recipe_sha256(recipe):
    """짝 확인 해시 = 레시피(구조) 객체를 키 정렬 · 공백 없는 JSON(한글 그대로)의 UTF-8 바이트로 만든 sha256 (IRD 6장 E-69).
    정본은 d2_task.recipe_document.recipe_sha256(10/14 C4) — d2_task 가 d2_motion 을 쓰므로 여기서 불러 쓸 수 없어
    같은 두 줄을 두고, 값이 같은지는 tests/test_shared_tables.py 가 확인한다."""
    text = json.dumps(recipe, sort_keys=True, separators=(',', ':'), ensure_ascii=False)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def load_recipe(path):
    """레시피 파일 <모델ID>_recipe.json(구조, recipe/4.0)과 같은 폴더의 <모델ID>_placements.csv(조립 방법, 줄마다 placements/4.0)를 읽어
    {'schema', 'design_id', 'recipe_sha256', 'steps', 'structure'} 로 붙인다 — recipe_parts 가 둘을 블록 이름으로 잇는다(E-69).
    steps 는 CSV 의 조립 방법 칸(part_name · part_id · sequence · stage · grasp · grasp_axis · supports)만 쓰고, 표시 칸(중심 · 크기 · 방향)은
    구조 파일과 같은 값이라 읽지 않는다. sequence · stage 는 정수, supports 는 ';' 로 나눈 목록.
    실패: 조립 방법 파일이 없으면 FileNotFoundError. 형식이 recipe/4.0 · placements/4.0 이 아니거나(옛 cad_* 는 바꿔 읽지 않고 거절)
    recipe_sha256 이 구조와 다르면 ValueError — 구조가 바뀌었는데 옛 조립 방법으로 쌓으면 받침 · 잡기가 어긋난다."""
    with open(path) as f:
        recipe = json.load(f)
    if not same_major(recipe.get('schema'), RECIPE_SCHEMA):
        raise ValueError(f'{os.path.basename(path)}: 형식 {recipe.get("schema")!r} 는 읽지 않는다 ({RECIPE_SCHEMA} 앞자리만)')
    want = recipe_sha256(recipe)
    pp = os.path.join(os.path.dirname(path), recipe['design_id'] + PLACEMENTS_SUFFIX)
    with open(pp, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    steps = []
    for r in rows:
        if not same_major(r.get('schema'), PLACEMENTS_SCHEMA):
            raise ValueError(f'{os.path.basename(pp)}: 형식 {r.get("schema")!r} 는 읽지 않는다 ({PLACEMENTS_SCHEMA} 앞자리만)')
        if r.get('recipe_sha256') != want:
            raise ValueError(f'{os.path.basename(pp)} 의 recipe_sha256 이 {os.path.basename(path)} 와 다르다 (구조가 바뀜 — 레시피 도구로 다시 만든다)')
        steps.append({'part_name': r['part_name'], 'part_id': r['part_id'], 'sequence': int(r['sequence']), 'stage': int(r['stage']),
                      'grasp': r['grasp'], 'grasp_axis': r['grasp_axis'], 'supports': [k for k in r['supports'].split(';') if k]})
    return {'schema': PLACEMENTS_SCHEMA, 'design_id': recipe['design_id'], 'recipe_sha256': want, 'steps': steps, 'structure': recipe}


def recipe_design_id(recipe):
    """레시피의 모델 ID (= 기본 설계 design_id). 설계 이름은 블록 이름에서 잘라 내지 않는다(E-52 ④ — BACK · BEAM 에도 _B 가 있다).
    모델 ID 칸이 없으면 빈 글자."""
    return recipe['model']['design_id'] if 'model' in recipe else recipe.get('design_id', '')


def recipe_parts(cfg, recipe):
    """레시피 -> sequence 순서의 블록 목록 (설계 좌표 -> base 좌표).

    레시피 = E-69 두 파일: 조립 방법 placements/4.0 steps[](part_name · part_id · sequence · stage · grasp_axis · supports — 부품 이름)
    + 구조 recipe/4.0 (load_recipe 가 'structure' 칸에 붙임: stocks[](stock_id · size_mm) +
    template.composition[역할].parts[그룹][](part · stock_id · center_mm · R),
    블록 이름 = '<그룹>_<part>' 예 LEG_001_01 — 사람 칸 label · revision 은 읽지 않는다).
    part_id = 조립 방법의 part_id 칸, 없으면 '<design_id>_<블록 이름>' (예 001_CHAIR_BENCH_V000_LEG_001_01 — 노드 사이 전체 블록 이름).
    옛 한 파일 꼴(model.instances + steps)도 아직 읽는다 — task(TaskPlanner)가 두 파일을 RecipeDocument 로 이 꼴로 합쳐 넘기고
    (steps[].part_id), task 시험 예시는 이 꼴 파일이다(part_id 가 없으면 '<DESIGN_ID>_<instance_id>' — 10/14 C3, 옛 _B<번호> 꼴은 없앰). task 옮기기(W141) 뒤 뺀다.
    옛 blocks[] 형식 · 로봇 쪽 옛 파일 이름 .recipe.json 은 W139 뒤 뺐다(W138).
    조립 원점은 robot.yaml assembly_origin 하나만 쓴다 (레시피 T_base_from_cad 는 null — 10/4 합의).
    가로 위치는 설계 그대로, 높이는 받침의 실제 윗면 위에 실측 블록(block_actual_m)으로 쌓아 올린 값이다.
    반환: [{'part_id', 'sequence', 'stage', 'grasp', 'center': (m), 'rot': 3x3, 'quat'}]
    """
    o = cfg['assembly_origin']
    yaw = math.radians(o['yaw_deg'])
    c, s = math.cos(yaw), math.sin(yaw)
    top = {}                                         # part_id -> 실제 윗면 z (m, base). 받침을 따라 쌓아 올린다

    def part(part_id, seq, stage, size_mm, center_mm, R, axis, supports):
        """레시피 블록 하나 -> 목표 항목. 크기는 mm 입력, 설계 치수와 다르면 예외. 윗면 z 는 받침 실제 윗면을 따라 쌓는다."""
        size_m = [v / 1000.0 for v in size_mm]
        if any(abs(a - b) > 1e-4 for a, b in zip(size_m, cfg['block_size_m'])):
            raise ValueError(f'{part_id}: 블록 크기 {size_m} 가 robot.yaml block_size_m 와 다르다')
        x, y, z = (v / 1000.0 for v in center_mm)
        rot = rot_z(R, yaw)
        # 높이는 설계값(15 mm 층)이 아니라 실제 블록(약 14.8 mm)으로 쌓아 올린다: 받침들의 실제 윗면 위에 놓는다.
        # 설계값을 그대로 쓰면 위층일수록 실제 면보다 높은 곳에서 놓아 떨어지며 뒤틀린다 (10/6 실기, 10층에서 +1.8 mm)
        below = [top[k] for k in supports if k in top]
        if below:
            bottom = max(below)
        else:                                        # 받침이 없으면 작업대 위 (설계상 떠 있으면 설계 높이 그대로)
            bottom = o['z_m'] + max(0.0, z - half_height(R, size_m))
        h = 2 * half_height(rot, cfg['block_actual_m'])
        top[part_id] = bottom + h
        return {'part_id': part_id, 'sequence': seq, 'stage': stage, 'grasp': grasp_name(rot, axis),
                'center': (o['x_m'] + c * x - s * y, o['y_m'] + s * x + c * y, bottom + h / 2),
                'rot': rot, 'quat': quat_from_axes(*(column(rot, k) for k in range(3)))}

    out = []
    if 'model' in recipe:                           # 옛 한 파일 꼴 — task RecipeDocument.geometry · task 시험 예시(W141 전)
        model = recipe['model']
        sizes = {p['stock_id']: p['size_mm'] for p in model['stocks']}
        inst = {i['instance_id']: i for i in model['instances']}
        # 순서표에 part_id 가 없을 때만 정본 규칙(<설계 ID 대문자>_<부품 이름>)으로 — 옛 _B### 꼴은 없앰(10/14 PL C3, tests/test_shared_tables.py)
        bid = {st['instance_id']: st.get('part_id') or f'{model["design_id"].upper()}_{st["instance_id"]}' for st in recipe['steps']}
        for st in sorted(recipe['steps'], key=lambda k: k['sequence']):
            i = inst[st['instance_id']]
            out.append(part(bid[st['instance_id']], st['sequence'], st.get('stage'), sizes[i['stock_id']], i['center_mm'], i['R'],
                             st['grasp_axis'], [bid[k] for k in st.get('support_instance_ids') or [] if k in bid]))
        return out
    struct = recipe['structure']                     # 두 파일 (load_recipe 가 구조 _recipe.json 을 'structure' 칸에 붙임)
    sizes = {p['stock_id']: p['size_mm'] for p in struct['stocks']}
    geo = {f'{group}_{item["part"]}': item for comp in struct['template']['composition'].values()
           for group, items in comp['parts'].items() for item in items}
    full = {st['part_name']: st.get('part_id') or f'{recipe["design_id"].upper()}_{st["part_name"]}' for st in recipe['steps']}
    for st in sorted(recipe['steps'], key=lambda k: k['sequence']):
        g = geo[st['part_name']]
        out.append(part(full[st['part_name']], st['sequence'], st.get('stage'), sizes[g['stock_id']], g['center_mm'], g['R'],
                         st['grasp_axis'], [full[k] for k in st.get('supports') or [] if k in full]))
    return out


def footprint(cfg, parts):
    """블록들이 바닥에서 차지하는 xy 사각형 (min_x, min_y, max_x, max_y) (m, base)."""
    xs, ys = [], []
    for b in parts:
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    p = [sum(b['rot'][i][k] * s_ * h for k, (s_, h) in enumerate(zip((sx, sy, sz), [v / 2 for v in cfg['block_size_m']])))
                         for i in range(2)]
                    xs.append(b['center'][0] + p[0])
                    ys.append(b['center'][1] + p[1])
    return min(xs), min(ys), max(xs), max(ys)


SET_GAP_M = 0.02          # 여러 설계를 세트로 놓을 때 설계 사이 틈 (책상·의자처럼 가깝게, 손가락은 블록 폭 안이라 걸리지 않음)


def layout_designs(cfg, recipes):
    """여러 설계를 한 번에 쌓을 자리를 정한다. 반환: [(블록 목록, (dx, dy) m)] — 블록 center 는 옮긴 뒤 값.

    설계 하나면 assembly_origin 그대로. 여럿이면 준 순서대로 −y 쪽으로 SET_GAP_M 씩 띄워 나란히 놓고(책상·의자 세트:
    의자의 앞(−y)에 책상), 세트 전체의 가운데를 조립 중심에 맞춘다. 계산만으로 정해지므로 run_recipe 와 scene_manager 가
    같은 레시피 목록이면 같은 자리를 쓴다. 조립 작업공간(중심 ± assembly_area_half_m)을 넘으면 ValueError.
    """
    designs = [recipe_parts(cfg, r) for r in recipes]
    names = [recipe_design_id(r) or f'{k + 1}번' for k, r in enumerate(recipes)]
    fps = [footprint(cfg, b) for b in designs]
    shifts, y_top = [], None
    for fp in fps:                           # 앞 설계의 −y 끝에서 SET_GAP_M 띄워 다음 설계의 +y 끝을 둔다
        dy = 0.0 if y_top is None else (y_top - SET_GAP_M) - fp[3]
        shifts.append(dy)
        y_top = fp[1] + dy
    if len(designs) > 1:                     # 세트 전체 가운데를 조립 중심 y 에 맞춘다
        lo = min(fp[1] + dy for fp, dy in zip(fps, shifts))
        hi = max(fp[3] + dy for fp, dy in zip(fps, shifts))
        mid = cfg['assembly_origin']['y_m'] - (lo + hi) / 2
        shifts = [dy + mid for dy in shifts]
    o, half = cfg['assembly_origin'], cfg['assembly_area_half_m']
    out = []
    for parts, fp, dy, name in zip(designs, fps, shifts, names):
        if (fp[0] < o['x_m'] - half - 1e-6 or fp[2] > o['x_m'] + half + 1e-6
                or fp[1] + dy < o['y_m'] - half - 1e-6 or fp[3] + dy > o['y_m'] + half + 1e-6):
            raise ValueError(f'설계 {name} 가 조립 작업공간(중심 ± {half * 1000:.0f} mm)을 넘는다')
        for b in parts:
            b['center'] = (b['center'][0], b['center'][1] + dy, b['center'][2])
        out.append((parts, (0.0, dy)))
    return out

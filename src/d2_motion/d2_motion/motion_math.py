# -*- coding: utf-8 -*-
"""집기·놓기 계산 (ROS 없이 동작) — 블록 자세 -> 손가락 끝(TCP) 목표, 설계도(레시피) 읽기, 여러 설계 배치.

pick_place 노드·scene_manager 노드·run_recipe 도구가 같이 쓴다. ROS 없이 시험할 수 있게 노드와 나눴다(팀 규칙 ②).
원본: 박진용 recipe_demo/recipe_tcp.py (10/4~10/6 실기로 확인한 식). 궤적 시간은 MoveIt 이 매긴다.

좌표 약속
  - 위치 m, 쿼터니언 (x, y, z, w), 기준 base_link. 설정은 robot.yaml(cfg dict)에서 읽는다.
  - 블록 자세 = 블록 중심 + 블록 자신의 축(x = LENGTH 75, y = WIDTH 25, z = THICKNESS 15 mm 방향).
  - TCP(rg2_tcp, 손가락 끝 중심): 접근 축 +Z 가 아래(base -Z), 닫힘 축 Y 가 블록의 잡는 축과 평행.
  - 잡기 이름 6가지(IRD 2장) = <바닥 상태>_<LONG|SHORT>. 그리퍼 폭은 '손가락 사이에 끼우는 블록 축'만으로 정해진다.
"""
import math

AXES = {'LENGTH': 0, 'WIDTH': 1, 'THICKNESS': 2}
# 잡기 이름 -> 손가락 사이에 끼우는 블록 축
GRASP_AXIS = {'FLAT_SHORT': 'WIDTH', 'FLAT_LONG': 'LENGTH', 'EDGE_SHORT': 'THICKNESS',
              'EDGE_LONG': 'LENGTH', 'STAND_SHORT': 'THICKNESS', 'STAND_LONG': 'WIDTH'}
# 위를 향하는 블록 축 -> 바닥 상태 이름 (눕힘 15 · 옆으로 세움 25 · 위로 세움 75 mm 높이)
STATE_OF_UP = {'THICKNESS': 'FLAT', 'WIDTH': 'EDGE', 'LENGTH': 'STAND'}
LEVEL_TOL_DEG = 5.0       # 잡는 축이 수평에서 이만큼까지 기울어도 위에서 집을 수 있다고 본다


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
    return (st['x_m'], st['y_m'], st['surface_z_m'] + half_height(rot, cfg['block_actual_m'])), rot


# ---------------- RG2 손가락 끝 높이 ----------------
# RG2 손가락은 호를 그리며 닫혀서 열림 폭에 따라 손가락 끝 높이가 바뀐다 (75 mm 블록을 물면 다 닫음보다 16.8 mm 위).
# URDF rg2_tcp 는 고정이라 폭별 끝 높이를 계산해 보정한다. 식·상수는 onrobot_rg_control 드라이버(RG2)와 같다.
RG2_L1, RG2_L3, RG2_TH1, RG2_TH3, RG2_DY, RG2_DZ = 0.108505, 0.055, 1.41371, 0.76794, -0.0144, 0.1095 + 0.0427


def rg2_tip_height_m(display_width_m):
    """그리퍼 표시(피드백) 폭 (m) -> 그리퍼 바닥에서 손가락 끝까지 높이 (m)."""
    c = (display_width_m / 2 - RG2_DY - RG2_L1 * math.cos(RG2_TH1)) / RG2_L3
    a = math.acos(max(-1.0, min(1.0, c))) - RG2_TH3
    return RG2_L3 * math.sin(a + RG2_TH3) + RG2_DZ


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
    pick_xyz, pick_q, _ = tcp_target(cfg, pick_center, pick_rot, grasp, slot_cfg)
    # 놓는 높이는 조립 원점에서 교시한 높이(tcp_z_m, record_cell 의 m)가 있으면 그것 기준, 없으면 손가락 끝 보정식 (원본과 같음)
    place_xyz, place_q, held = tcp_target(cfg, place_center, place_rot, grasp, cfg['assembly_origin'])
    if slot_cfg is not None and slot_cfg.get('tcp_z_m') is not None:
        std_z = tcp_target(cfg, pick_center, pick_rot, grasp)[0][2]
        delta = pick_xyz[2] - std_z
        place_xyz = (place_xyz[0], place_xyz[1], place_xyz[2] + delta)
        held['offset_m'][2] += delta
    return pick_xyz, pick_q, place_xyz, place_q, held


# ---------------- 설계도(레시피) ----------------
def recipe_blocks(cfg, recipe):
    """레시피 -> sequence 순서의 블록 목록 (설계 좌표 -> base 좌표). 형식 두 가지를 읽는다.

    - assembly.recipe/1.0 (CAD_to_Recipe 출력, 예 src/recipe_manager/recipes/001_CHAIR_BENCH.recipe.json): model.instances + steps.
      steps[].block_id 를 쓴다 (규칙 '<model_id>_B<sequence 3자리>', 예 001_CHAIR_BENCH_B001 — 10/7 W105).
      block_id 가 없는 옛 파일은 같은 규칙으로 만들어 쓴다.
    - m0609.jenga.cad_recipe/1.0 (한세교 Advanced, 예 03_Recipes/lv4_table_standing.recipe.json): blocks[].
      block_id 그대로, 끼우는 축은 closing_axis_cad 와 나란한 블록 축.
    조립 원점은 robot.yaml assembly_origin 하나만 쓴다 (레시피 T_base_from_cad 는 null — 10/4 합의).
    가로 위치는 설계 그대로, 높이는 받침의 실제 윗면 위에 실측 블록(block_actual_m)으로 쌓아 올린 값이다.
    반환: [{'block_id', 'sequence', 'stage', 'grasp', 'center': (m), 'rot': 3x3, 'quat'}]
    """
    o = cfg['assembly_origin']
    yaw = math.radians(o['yaw_deg'])
    c, s = math.cos(yaw), math.sin(yaw)
    top = {}                                         # block_id -> 실제 윗면 z (m, base). 받침을 따라 쌓아 올린다

    def block(block_id, seq, stage, size_mm, center_mm, R, axis, supports):
        """레시피 블록 하나 -> 목표 항목. 크기는 mm 입력, 설계 치수와 다르면 예외. 윗면 z 는 받침 실제 윗면을 따라 쌓는다."""
        size_m = [v / 1000.0 for v in size_mm]
        if any(abs(a - b) > 1e-4 for a, b in zip(size_m, cfg['block_size_m'])):
            raise ValueError(f'{block_id}: 블록 크기 {size_m} 가 robot.yaml block_size_m 와 다르다')
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
        top[block_id] = bottom + h
        return {'block_id': block_id, 'sequence': seq, 'stage': stage, 'grasp': grasp_name(rot, axis),
                'center': (o['x_m'] + c * x - s * y, o['y_m'] + s * x + c * y, bottom + h / 2),
                'rot': rot, 'quat': quat_from_axes(*(column(rot, k) for k in range(3)))}

    out = []
    if 'blocks' in recipe:                          # m0609.jenga.cad_recipe/1.0
        for b in sorted(recipe['blocks'], key=lambda k: k['sequence']):
            R, ca = b['R_cad_from_block'], b['closing_axis_cad']
            axis = ['LENGTH', 'WIDTH', 'THICKNESS'][max(range(3), key=lambda k: abs(sum(R[i][k] * ca[i] for i in range(3))))]
            out.append(block(b['block_id'], b['sequence'], b.get('stage'), b['size_lwt_mm'], b['center_cad_mm'], R, axis,
                             b.get('support_block_ids') or []))
    else:                                           # assembly.recipe/1.0
        model = recipe['model']
        sizes = {p['part_id']: p['size_mm'] for p in model['parts']}
        inst = {i['instance_id']: i for i in model['instances']}
        bid = {st['instance_id']: st.get('block_id') or f'{model["model_id"]}_B{st["sequence"]:03d}' for st in recipe['steps']}
        for st in sorted(recipe['steps'], key=lambda k: k['sequence']):
            i = inst[st['instance_id']]
            out.append(block(bid[st['instance_id']], st['sequence'], st.get('stage'), sizes[i['part_id']], i['center_mm'], i['R'],
                             st['grasp_axis'], [bid[k] for k in st.get('support_instance_ids') or [] if k in bid]))
    return out


def footprint(cfg, blocks):
    """블록들이 바닥에서 차지하는 xy 사각형 (min_x, min_y, max_x, max_y) (m, base)."""
    xs, ys = [], []
    for b in blocks:
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
    designs = [recipe_blocks(cfg, r) for r in recipes]
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
    for blocks, fp, dy in zip(designs, fps, shifts):
        if (fp[0] < o['x_m'] - half - 1e-6 or fp[2] > o['x_m'] + half + 1e-6
                or fp[1] + dy < o['y_m'] - half - 1e-6 or fp[3] + dy > o['y_m'] + half + 1e-6):
            raise ValueError(f'설계 {blocks[0]["block_id"].split("_")[0]} 가 조립 작업공간(중심 ± {half * 1000:.0f} mm)을 넘는다')
        for b in blocks:
            b['center'] = (b['center'][0], b['center'][1] + dy, b['center'][2])
        out.append((blocks, (0.0, dy)))
    return out

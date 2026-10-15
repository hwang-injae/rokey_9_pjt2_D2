"""조립 자리에 방금 놓은 블록 하나 재기 — find_blocks 요청 {"area": "assembly", "design_id", "part_id"} (ROS 없는 계산, YOLO 없음).

W144(챌린지 A4 — 놓은 블록이 많이 비켰으면 다시 집어 놓기)의 측정 입력. 작업 관리자가 VERIFY 에서 방금 놓은 블록(위에 덮이기 전)을 묻는다.
받침 재기(spare_measure.measure_on_spare)와 같은 방식 — 설계 자리 둘레의 깊이 점 → 윗면 높이 · 윗면 점의 주축으로 yaw · 양 끝 백분위의 가운데.
받침과 다른 점: 둘레에 다른 블록이 있을 수 있어서, 블록 하나로 볼 수 없으면 좌표를 내지 않고 거절 코드를 낸다(잘못된 다시 집기를 막으려고).

거절 코드(reject): NO_PART(설계 자리에 그 높이의 윗면이 없음) · COVERED(설계 자리 위에 다른 물체가 덮음) · STANDING(세운 블록 — 윗면이
15 × 25 mm 라 지금은 재지 않음) · FEW_POINTS(윗면 점이 모자람) · TILTED(긴 축 양 끝 높이 차 5 mm · 긴 축 5° · 폭 방향 10° 중 하나 넘음, 또는 한쪽 끝만 들림 —
놓은 블록 전용 문턱, 10/15) · NEIGHBOR_MERGED(같은 높이
이웃과 붙어 윗면이 블록보다 큼 — 경계가 안 보임) · SIZE_MISMATCH(윗면이 블록보다 4 mm 넘게 작음 — 가장자리가 가려지거나 깊이 구멍).
정확도는 실기로 아직 안 봤다(10/14) — 작업 관리자는 실기 흔들림을 확인한 뒤에 다시 집기에 쓴다(PL · 한석형).
응답 corners_z_m(10/15 한석형 #190): 잰 블록 네 모서리 안쪽 칸 높이 — 모서리 판정(한 모서리가 뜨면 사람 대기)에 쓴다. 못 잰 모서리는 null.
단위: 입력 · 출력 좌표 m(base_link). 바깥 영향 없음.
"""
import math

import numpy as np
from scipy import ndimage

from d2_vision.block_finder import TOP_AXES, wrap_deg
from d2_vision.spare_measure import EDGE_PCT, MIN_PTS_FLAT, UP_DIM

REACH_M = 0.015            # 설계 자리(위에서 본 상자) 둘레로 이만큼 넓혀 점을 본다 — 놓기 어긋남(몇 mm)과 이웃 붙음을 함께 보려고
TOP_TOL_M = 0.005          # 설계 윗면 높이에서 이만큼 안의 점만 이 블록 윗면 — 아래층(15 mm 아래) · 위 블록(15 mm 위)과 가른다
COVER_ABOVE_M = 0.008      # 설계 자리 안에서 윗면 + 이 값보다 높은 점(손가락 · 위 블록) 비율이
COVER_FRAC = 0.2           # 이 이상이면 COVERED
SIZE_TOL_M = 0.004         # 잰 윗면 두 변이 블록 치수와 이보다 많이 다르면 NEIGHBOR_MERGED(큼) · SIZE_MISMATCH(작음)
MIN_FRAC = 0.5             # 윗면 점이 받침 재기 최소(MIN_PTS_FLAT 비율)의 이 배보다 적으면 FEW_POINTS — 둘레 이웃에 가려 반쯤 보일 수 있어서
SEG_CELL_M = 0.002         # 윗면 점을 이 칸으로 나눠 이어진 덩어리만 이 블록으로 본다 — 틈(2 칸 이상)이 보이는 이웃은 떨어진 덩어리라 빠진다
EDGE_KEEP = (EDGE_PCT[1] - EDGE_PCT[0]) / 100.0   # 양 끝 백분위로 잰 폭 = 실제 폭 × 이 비율(고르게 퍼진 점) — 크기는 이걸로 나눠 되돌린다
# 기울기 문턱 — 놓은 블록 전용(10/15, 한석형 건의 · 민범진). block_finder(흩뿌린 블록 집기 5 mm · 5°)에서 가져다 쓰던 것을 뗐다.
TILT_MM = 5.0             # 긴 축 양 끝 높이 차(mm) 이상이면 TILTED — block_finder · IRD E-43 과 같은 값
TILT_LONG_DEG = 5.0       # 긴 축 방향 기울기 각(°) — block_finder B1(10/14 PL)과 같은 값
TILT_WIDE_DEG = 10.0      # 폭 방향 기울기 각(°) — 깊이 카메라가 윗면 가장자리를 처지게 재서(가운데 5 mm 밖부터 3 ~ 5 mm) 반듯한 블록도
                          # 폭 방향 4 ~ 6° 로 재진다(실기 V008 · V009 · V001 15장 최대 4.1°, 다시 찍기 5 ~ 6°). 진짜 옆 기울어짐은
                          # 이웃(15 mm 위)에 걸칠 때라 10° 를 넘는다
TILT_DEG_MIN_SHORT_MM = 20.0   # 각도 기준은 윗면 짧은 변이 이 이상(눕힌 75 × 25 면)일 때만 — block_finder 와 같은 값
# 한쪽 끝 들림(10/15 PL): 짧은 끝이 길이 방향으로 5 ~ 10 mm 들리면 들린 끝 점이 높이 띠(±TOP_TOL_M) 밖으로 잘려 평면이 기울기를 못 잡는다
# → 설계 자리 긴 축 양 끝 띠에서 윗면보다 END_ABOVE_M 넘게 높은 점 비율을 본다. 합성 5 mm 들림 0.90 이상 · 3 mm 0.01 이하 ·
# 실기 정상 14장(V008 · V009 · V001 1층 다리) 최대 0.14 → 문턱 0.5
END_STRIP_M = 0.010       # 긴 축 끝에서 이만큼 안쪽 띠
END_ABOVE_M = 0.004       # 설계 윗면보다 이만큼 넘게 높으면 '위로 빠진 점'
END_FRAC = 0.5            # 한쪽 끝 띠에서 위로 빠진 점 비율이 이 이상이고 다른 끝은 아래면 TILTED(둘 다 높으면 기울기가 아니라 통째로 높음)
END_MARGIN_M = 0.004      # 끝 띠는 짧은 변 가장자리에서 이만큼 안쪽만(옆 블록 · 가장자리 튐 빼기)
# 모서리 높이(10/15 한석형 #190 모서리 판정 입력 — 응답 blocks[0].corners_z_m): 잰 블록 윗면 네 모서리 안쪽 칸의 깊이 z 가운데 값
CORNER_INSET_M = 0.004     # 모서리 칸은 윗면 끝에서 이만큼 안쪽부터(가장자리 깊이 처짐 · 튐을 피함)
CORNER_CELL_M = 0.006      # 칸 한 변 — 0.5 m 에서 약 7 px, 실기 다리 92개 모서리에서 칸마다 점 30 ~ 80개
CORNER_Z_LO_M = 0.008      # 칸 점은 설계 윗면보다 이만큼 아래까지(아래층 15 mm 아래 · 작업면은 뺌)
CORNER_Z_HI_M = 0.020      # 설계 윗면보다 이만큼 위까지 — 윗면 ±TOP_TOL_M 띠로 자르면 8 mm 넘게 들린 모서리가 null 이 된다(합성 8 · 10 · 15 mm 들림).
                           # 위에 손가락 · 블록이 있으면 COVERED 가 먼저 거르므로 위쪽은 넓게 둔다
CORNER_MIN_PTS = 5         # 칸 점이 이보다 적으면 그 모서리는 null(못 잼)
REJECTS = ('NO_PART', 'COVERED', 'STANDING', 'FEW_POINTS', 'TILTED', 'NEIGHBOR_MERGED', 'SIZE_MISMATCH')


def up_of(rot):
    """블록 회전(base←블록 3x3) → 위를 향한 블록 축 이름(LENGTH · WIDTH · THICKNESS). rot 열 = 블록 축의 base 방향."""
    k = int(np.argmax(np.abs(np.asarray(rot, float)[2])))
    return ('LENGTH', 'WIDTH', 'THICKNESS')[k]


def _connected_to_center(sel, center_xy):
    """윗면 점 중 설계 가운데와 이어진 덩어리만 남긴다(SEG_CELL_M 칸, 8방향 이웃). 가운데 칸이 비면 가운데에서 가장 가까운 점의 덩어리.
    틈이 보이는 이웃 블록 윗면은 떨어진 덩어리라 빠지고, 붙은 이웃은 한 덩어리로 남아 뒤 크기 검사에서 NEIGHBOR_MERGED 가 된다."""
    ij = np.floor(sel[:, :2] / SEG_CELL_M).astype(int)
    lo = ij.min(axis=0)
    ij -= lo
    grid = np.zeros(ij.max(axis=0) + 1, bool)
    grid[ij[:, 0], ij[:, 1]] = True
    labels, _ = ndimage.label(grid, structure=np.ones((3, 3), bool))
    lab = labels[ij[:, 0], ij[:, 1]]
    k = int(np.argmin(np.hypot(sel[:, 0] - center_xy[0], sel[:, 1] - center_xy[1])))
    return sel[lab == lab[k]]


def _end_lifted(p, local, top_z, up, size):
    """한쪽 끝만 들렸나 — 설계 자리 긴 축 양 끝 띠(END_STRIP_M) 안 점 중 설계 윗면 + END_ABOVE_M 보다 높은 점 비율을 끝마다 잰다.
    입력: p = base 점 (N, 3) m · local = 같은 점의 블록 좌표 · top_z = 설계 윗면 높이 m · up = 위 축 이름 · size = [길이, 폭, 두께] m.
    출력: 한쪽 끝 비율 ≥ END_FRAC 이고 다른 끝 < END_FRAC 이면 (앞 끝 비율, 뒤 끝 비율), 아니면 None(끝 띠에 점이 없어도 None).
    바깥 영향 없음."""
    names = ('LENGTH', 'WIDTH', 'THICKNESS')
    i, j = (names.index(k) for k in TOP_AXES[up])          # 윗면 긴 변 축 · 짧은 변 축
    across = np.abs(local[:, j]) <= size[j] / 2.0 - END_MARGIN_M
    fr = []
    for sgn in (-1.0, 1.0):
        z = p[across & (sgn * local[:, i] > size[i] / 2.0 - END_STRIP_M) & (sgn * local[:, i] <= size[i] / 2.0), 2]
        if len(z) == 0:
            return None
        fr.append(float(np.mean(z > top_z + END_ABOVE_M)))
    if max(fr) >= END_FRAC > min(fr):
        return tuple(round(f, 2) for f in fr)
    return None


def corner_heights(p, center_xy, yaw_deg, top_z, a, b):
    """잰 블록 윗면 네 모서리 안쪽 칸의 높이 — 응답 corners_z_m(한석형 #190 모서리 판정: 한 모서리가 뜨면 자동 다시 집기 대신 사람 대기).

    입력: p = base 점 (N, 3) m(NaN 뺀 것) · center_xy = 잰 블록 가운데 (x, y) m · yaw_deg = 잰 yaw(°) · top_z = 설계 윗면 높이 m ·
          a · b = 윗면 긴 변 · 짧은 변 m. 칸은 설계 자리가 아니라 **잰 가운데 · yaw** 기준 — 비켜 놓인 블록(10/15 실기 +7 mm)을 설계 자리로
          재면 끝 칸이 블록 밖(아래층)을 재서 'Z 문제'로 잘못 간다(한석형 XY 모서리도 잰 값 기준).
    출력: [z(−u,−v), z(−u,+v), z(+u,−v), z(+u,+v)] — u = 긴 변 방향(yaw) · v = 짧은 변, 값 = 칸 점(설계 윗면 −CORNER_Z_LO_M ~ +CORNER_Z_HI_M)
          z 가운데 값 m(base, 소수 4자리) 또는 None(점 CORNER_MIN_PTS 미만). 바깥 영향 없음."""
    th = math.radians(yaw_deg)
    u = np.array([math.cos(th), math.sin(th)])
    v = np.array([-u[1], u[0]])
    d = p[:, :2] - np.asarray(center_xy, float)
    su, sv = d @ u, d @ v
    zok = (p[:, 2] >= top_z - CORNER_Z_LO_M) & (p[:, 2] <= top_z + CORNER_Z_HI_M)
    out = []
    for si in (-1.0, 1.0):
        for sj in (-1.0, 1.0):
            cu = si * (a / 2.0 - CORNER_INSET_M - CORNER_CELL_M / 2.0)
            cv = sj * (b / 2.0 - CORNER_INSET_M - CORNER_CELL_M / 2.0)
            m = zok & (np.abs(su - cu) <= CORNER_CELL_M / 2.0) & (np.abs(sv - cv) <= CORNER_CELL_M / 2.0)
            out.append(round(float(np.median(p[m, 2])), 4) if int(np.count_nonzero(m)) >= CORNER_MIN_PTS else None)
    return out


def measure_placed(pts, part, block_actual_m):
    """설계 자리에 놓인 블록 하나를 깊이 점으로 잰다.

    입력: pts = base 점 (N, 3) 또는 (H, W, 3) m(NaN 은 뺌) · part = 설계 자리 {part_id, center (x, y, z) m, rot 3x3}(motion_math.recipe_parts
          · BlockChecker.parts 와 같은 칸) · block_actual_m = [길이, 폭, 두께] m.
    출력: (블록 dict, None) — {part_id, x_m, y_m, top_z_m, yaw_deg, up, tilted, confidence, size_mm [두 변], corners_z_m [모서리 4곳 z 또는 null]}
          또는 (None, (거절 코드, 설명)).
    순서: ① 세움이면 STANDING ② 설계 자리 둘레(REACH_M) 점 ③ 자리 안 높은 점 비율로 COVERED ④ 설계 윗면 ± TOP_TOL_M 점 — 없으면 NO_PART,
          가운데와 이어진 덩어리만(틈 보이는 이웃 뺌) — 모자라면 FEW_POINTS ⑤ 윗면 평면의 긴 축 양 끝 높이 차 · 긴 축 각 · 폭 방향 각, 또는 한쪽 끝 띠만 위로 빠짐(_end_lifted)으로 TILTED ⑥ 주축 yaw · 양 끝 백분위 가운데 ·
          두 변 길이(백분위 폭 ÷ EDGE_KEEP) → 크면 NEIGHBOR_MERGED, 작으면 SIZE_MISMATCH. 윗면 높이 = 윗면 점 가운데 값 ⑦ 잰 가운데 · yaw 로
          모서리 네 칸 높이(corner_heights).
    바깥 영향 없음. 실패: part 칸이 없으면 KeyError."""
    L, W, T = (float(v) for v in block_actual_m)
    size = (L, W, T)
    rot = np.asarray(part['rot'], float)
    up = up_of(rot)
    if up == 'LENGTH':
        return None, ('STANDING', '세운 블록은 윗면이 15 × 25 mm 라 지금은 재지 않는다')
    cx, cy, cz = (float(v) for v in part['center'])
    half = np.array(size) / 2.0
    top_z = cz + float(np.abs(rot[2]) @ np.array(size)) / 2.0           # 설계 윗면 높이(base z)
    a, b = (size[UP_DIM[k]] for k in TOP_AXES[up])                     # 윗면 두 변(긴 · 짧은)
    p = np.asarray(pts, float).reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    local = (p - np.array([cx, cy, cz])) @ rot                          # 블록 좌표(rot^T · d)
    flat = [i for i in range(3) if ('LENGTH', 'WIDTH', 'THICKNESS')[i] != up]
    reach = np.all(np.abs(local[:, flat]) <= half[flat] + REACH_M, axis=1)
    inside = np.all(np.abs(local[:, flat]) <= half[flat], axis=1)
    above_in = inside & (p[:, 2] > top_z - TOP_TOL_M)
    n_cover = int(np.count_nonzero(inside & (p[:, 2] > top_z + COVER_ABOVE_M)))
    if n_cover and n_cover >= COVER_FRAC * max(int(np.count_nonzero(above_in)), 1):
        return None, ('COVERED', f'설계 자리 안 점 중 윗면 {COVER_ABOVE_M * 1000:.0f} mm 위가 {n_cover}개 — 손가락 · 위 블록이 덮음')
    sel = p[reach & (np.abs(p[:, 2] - top_z) <= TOP_TOL_M)]
    need = MIN_PTS_FLAT * a * b / (L * W)
    if len(sel) < max(need * 0.1, 3):
        return None, ('NO_PART', f'설계 윗면 높이 ±{TOP_TOL_M * 1000:.0f} mm 점이 {len(sel)}개 — 블록이 없거나 다른 높이')
    sel = _connected_to_center(sel, (cx, cy))
    if len(sel) < max(need * MIN_FRAC, 3):
        return None, ('FEW_POINTS', f'윗면 점 {len(sel)}개 < {need * MIN_FRAC:.0f}개')
    A = np.c_[sel[:, 0], sel[:, 1], np.ones(len(sel))]
    coef = np.linalg.lstsq(A, sel[:, 2], rcond=None)[0]
    xy = sel[:, :2]
    mean = xy.mean(axis=0)
    w, vec = np.linalg.eigh(np.cov((xy - mean).T))
    u = vec[:, int(np.argmax(w))]
    v = np.array([-u[1], u[0]])
    lo_u, hi_u = np.percentile((xy - mean) @ u, EDGE_PCT)
    lo_v, hi_v = np.percentile((xy - mean) @ v, EDGE_PCT)
    span_u, span_v = (hi_u - lo_u) / EDGE_KEEP, (hi_v - lo_v) / EDGE_KEEP   # 백분위 폭 → 실제 폭
    tilt_mm = abs(coef[0] * u[0] + coef[1] * u[1]) * span_u              # 긴 축 양 끝 높이 차
    # 각도는 설계 자리 축으로 나눈다 — 기운 윗면이 높이 띠(±TOP_TOL_M)에 잘려 네모에 가까워지면 점 주축 u 가 폭 쪽으로 뒤집혀
    # 길이 방향 기울기가 느슨한 폭 문턱을 받는다(합성: 길이 방향 한 끝 5 · 8 mm 들림이 통과)
    ax = rot[:2, ('LENGTH', 'WIDTH', 'THICKNESS').index(TOP_AXES[up][0])]
    ax = ax / np.linalg.norm(ax)
    long_deg, wide_deg = (math.degrees(math.atan(abs(coef[0] * d[0] + coef[1] * d[1]))) for d in (ax, (-ax[1], ax[0])))
    if tilt_mm >= TILT_MM / 1000.0 or (b * 1000.0 >= TILT_DEG_MIN_SHORT_MM and (long_deg >= TILT_LONG_DEG or wide_deg >= TILT_WIDE_DEG)):
        return None, ('TILTED', f'윗면 긴 축 양 끝 높이 차 {tilt_mm * 1000:.1f} mm · 긴 축 {long_deg:.1f}° · 폭 방향 {wide_deg:.1f}°')
    lifted = _end_lifted(p, local, top_z, up, size)
    if lifted:
        return None, ('TILTED', f'한쪽 끝만 들림 — 긴 축 끝 {END_STRIP_M * 1000:.0f} mm 띠에서 윗면 +{END_ABOVE_M * 1000:.0f} mm 넘는 점 비율 '
                                f'{lifted[0]:.2f} · {lifted[1]:.2f}')
    if span_u > a + SIZE_TOL_M or span_v > b + SIZE_TOL_M:
        return None, ('NEIGHBOR_MERGED', f'윗면 {span_u * 1000:.0f} × {span_v * 1000:.0f} mm > 블록 {a * 1000:.0f} × {b * 1000:.0f} mm — 같은 높이 이웃과 붙어 경계가 안 보임')
    if span_u < a - SIZE_TOL_M or span_v < b - SIZE_TOL_M:
        return None, ('SIZE_MISMATCH', f'윗면 {span_u * 1000:.0f} × {span_v * 1000:.0f} mm < 블록 {a * 1000:.0f} × {b * 1000:.0f} mm — 가장자리가 안 보임')
    c = mean + u * (lo_u + hi_u) / 2 + v * (lo_v + hi_v) / 2
    yaw = round(wrap_deg(math.degrees(math.atan2(u[1], u[0]))), 1)
    yaw = yaw if yaw < 90.0 else -90.0
    return {
        'part_id': part['part_id'],
        'x_m': round(float(c[0]), 4),
        'y_m': round(float(c[1]), 4),
        'top_z_m': round(float(np.median(sel[:, 2])), 4),                # 윗면 점의 가운데 값 — 98 % 백분위는 깊이 잡음만큼 높게 나온다
        'yaw_deg': yaw,
        'up': up,
        'tilted': False,
        'confidence': 1.0,
        'size_mm': [round(span_u * 1000.0, 1), round(span_v * 1000.0, 1)],
        'corners_z_m': corner_heights(p, c, yaw, top_z, a, b),      # 한석형 #190 모서리 판정 입력(10/15) — 못 잰 모서리는 null
    }, None

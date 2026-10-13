"""받침(임시 적재 자리) 위 블록 하나 재기 — find_blocks 요청 {"area": "spare"} (ROS 없는 계산, YOLO 없음).

작업 관리자가 손목 카메라를 받침(robot.yaml spare_spots[0]) 위로 옮긴 뒤 부른다. 받침 위에는 블록이 많아야 하나라서
마스크 없이 깊이만 본다: 받침 가운데 둘레 점 → 윗면 높이로 자세(up) → 윗면 점의 주축으로 yaw · 가운데.
출력 블록 dict 는 find_blocks 응답 blocks 원소와 같은 칸 + "base_z_m"(받침 윗면 높이) — 받침 위에는 다른 블록이 없어서
틈은 넉넉한 값(OPEN_GAP_MM) · clear true · overlap none · tilted false · confidence 1.0 으로 둔다.
단위: 입력 · 출력 좌표 m(base_link), gap_mm 만 mm. 바깥 영향 없음.
"""
import math

import numpy as np

from d2_vision.block_finder import TOP_AXES, wrap_deg

NEAR_RADIUS_M = 0.07       # 받침 가운데에서 이 수평 거리 안 점만 본다 — 블록 반길이 37 mm + 놓기 어긋남 여유(받침 폭 80 mm 안)
ABOVE_SUPPORT_M = 0.004    # 받침 윗면 + 이 값보다 높은 점만 블록 — 받침 윗면 깊이 잡음(±2~3 mm)을 빼려고
ABOVE_TOP_M = 0.008        # 블록 가장 높은 자세(세움 = 길이) + 이 값보다 높은 점은 블록이 아니다(손가락 · 잡음)
MIN_PTS_FLAT = 200         # 눕힘 윗면(길이 × 폭)에 있어야 할 최소 점 수 — 다른 자세는 윗면 넓이 비율만큼 줄인다(시험 도구에서 옮긴 값)
TOP_PCT = 98               # 블록 높이 = near 점 z 의 이 백분위 − 받침 윗면(가장 높은 몇 점의 튐을 빼려고)
TOP_BAND_M = 0.005         # 윗면 점 = 높이 − 이 값보다 높은 점(옆면 · 받침 가장자리를 빼려고)
EDGE_PCT = (2, 98)         # 윗면 양 끝 = 주축 투영의 이 백분위(가장자리 튐을 빼려고) — 가운데 = 두 끝의 가운데
OPEN_GAP_MM = 99.0         # 받침 위 블록은 둘레에 다른 블록이 없다 — 틈은 이 값(find.min_gap_mm 22.2 보다 넉넉히)

UP_DIM = {'LENGTH': 0, 'WIDTH': 1, 'THICKNESS': 2}   # block_actual_m 안 순서 [길이, 폭, 두께]


def _spot(spot):
    """robot.yaml spare_spots 원소 {x_m, y_m, z_m(받침 윗면), yaw_deg} → (x, y, z) m. 칸이 없으면 KeyError."""
    return float(spot['x_m']), float(spot['y_m']), float(spot['z_m'])


def _block(x, y, yaw_deg, up, sz, size):
    """받침 위 블록 dict(find_blocks 칸 + base_z_m). 입력: 가운데 x · y m · yaw 도 · 자세 up · 받침 윗면 sz m · size = (길이, 폭, 두께) m."""
    yaw = round(float(yaw_deg), 1)
    axes = TOP_AXES[up]
    return {
        'x_m': round(float(x), 4),
        'y_m': round(float(y), 4),
        'top_z_m': round(sz + size[UP_DIM[up]], 4),     # 받침 위에 바로 놓였다고 보고 받침 윗면 + 그 자세 높이(깊이 잡음 대신 실측 치수)
        'yaw_deg': yaw if yaw < 90.0 else -90.0,        # 반올림으로 90.0 이 되면 경계 규칙(−90 ≤ yaw < 90)대로
        'up': up,
        'clear': {a: True for a in axes},
        'gap_mm': {a: OPEN_GAP_MM for a in axes},
        'overlap': 'none',
        'tilted': False,
        'confidence': 1.0,
        'base_z_m': round(sz, 4),
    }


def measure_on_spare(pts, spot, block_actual_m):
    """받침 위 블록 하나를 깊이 점으로 잰다. 블록이 없거나 점이 모자라면 None.

    입력: pts = base 점 (N, 3) 또는 (H, W, 3) m(NaN 점은 뺀다) · spot = robot.yaml spare_spots[0] {x_m, y_m, z_m(받침 윗면), yaw_deg} ·
          block_actual_m = [길이, 폭, 두께] m(robot.yaml block_actual_m).
    출력: 블록 dict {x_m, y_m, top_z_m, yaw_deg, up, clear, gap_mm, overlap, tilted, confidence, base_z_m} 또는 None.
    순서: ① 받침 가운데 NEAR_RADIUS_M 안 · 받침 윗면 위 ABOVE_SUPPORT_M ~ 길이 + ABOVE_TOP_M 인 점(near) — 세움 윗면 넓이만큼도 없으면 None
          ② 높이 = near z 의 TOP_PCT 백분위 − 받침 윗면 → 두께 · 폭 · 길이 중 가장 가까운 것이 up
          ③ 윗면 점(높이 − TOP_BAND_M 위) 수가 그 자세 윗면 넓이 비율만큼 모자라면 None
          ④ 윗면 xy 공분산의 주축 u → yaw, u · v 투영의 EDGE_PCT 양 끝 가운데 = 블록 가운데.
    바깥 영향 없음. 실패: spot 칸이 없으면 KeyError."""
    sx, sy, sz = _spot(spot)
    L, W, T = (float(v) for v in block_actual_m)
    size = (L, W, T)
    p = np.asarray(pts, float).reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    r = np.hypot(p[:, 0] - sx, p[:, 1] - sy)
    near = p[(r < NEAR_RADIUS_M) & (p[:, 2] > sz + ABOVE_SUPPORT_M) & (p[:, 2] < sz + L + ABOVE_TOP_M)]
    if len(near) < MIN_PTS_FLAT * (W * T) / (L * W):     # 가장 작은 윗면(세움 = 폭 × 두께)만큼도 없다 → 빈 받침
        return None
    top = float(np.percentile(near[:, 2], TOP_PCT)) - sz
    up = min(UP_DIM, key=lambda k: abs(size[UP_DIM[k]] - top))
    sel = near[near[:, 2] > sz + top - TOP_BAND_M]
    a, b = (size[UP_DIM[k]] for k in TOP_AXES[up])
    if len(sel) < max(MIN_PTS_FLAT * a * b / (L * W), 3):  # 3: 공분산을 낼 최소 점(넓이 비율이 아주 작아도)
        return None
    xy = sel[:, :2]
    mean = xy.mean(axis=0)
    w, vec = np.linalg.eigh(np.cov((xy - mean).T))
    u = vec[:, int(np.argmax(w))]
    v = np.array([-u[1], u[0]])
    lo_u, hi_u = np.percentile((xy - mean) @ u, EDGE_PCT)
    lo_v, hi_v = np.percentile((xy - mean) @ v, EDGE_PCT)
    c = mean + u * (lo_u + hi_u) / 2 + v * (lo_v + hi_v) / 2
    return _block(c[0], c[1], wrap_deg(math.degrees(math.atan2(u[1], u[0]))), up, sz, size)


def ideal_spare_block(spot, block_actual_m, up='THICKNESS'):
    """받침 가운데에 받침 yaw 로 놓인 블록 dict(measure_on_spare 와 같은 칸) — 가짜 노드(mock_wrist_block)의 기본 답.
    입력: spot = spare_spots[0] · block_actual_m = [길이, 폭, 두께] m · up = 자세(기본 눕힘). 바깥 영향 없음. 실패: 칸이 없으면 KeyError."""
    sx, sy, sz = _spot(spot)
    return _block(sx, sy, wrap_deg(float(spot.get('yaw_deg', 0.0))), up, sz, tuple(float(v) for v in block_actual_m))

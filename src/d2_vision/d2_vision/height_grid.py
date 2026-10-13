"""공급 영역 높이 지도 — find_blocks 요청 {"height_map": true} 의 응답 칸 (ROS 없는 계산).

작업 관리자(task)가 잡기 후보의 두 손가락 자리(직사각형)에 '손끝 높이 + 3 mm' 보다 높은 셀이 5 % 넘게 있는지 보려고 받는다.
영역: 공급은 supply_map_area(공급 영역 + 손가락 여유), 받침(area spare)은 spare_map_area(받침 가운데 둘레).
노드(wrist_block)는 find_blocks 와 같은 깊이 · 자세로 base 점을 만들어 여기 넘기고, 가짜 노드(mock_wrist_block)는 평평한 작업대 지도를 낸다.
응답 칸: {"x0_m","y0_m","res_m","nx","ny","z_m"} — x0_m · y0_m = 셀 (0, 0) 의 왼쪽 아래 모서리(base_link m),
  셀 (ix, iy) = x0 + ix·res ~ x0 + (ix+1)·res, z_m[iy·nx + ix] = 그 셀 안 점의 최고 base z(m, 소수 4자리) · 점이 없으면 null.
'최고 z' 인 이유: 블록 옆면 · 아래층이 섞여 보여도 손가락이 부딪히는 것은 가장 높은 면이다(block_checker.height_map 과 같은 규칙).
바깥 영향 없음.
"""
import math

import numpy as np

HEIGHT_MAP_RES_M = 0.002   # 셀 한 변 — 손가락 폭(약 20 mm) 안에 셀 10개가 들어가 5 % 비율을 셀 수 있게(틈 계산 GAP_CELL_M 과 같은 2 mm)


def grid_size(area, res_m=HEIGHT_MAP_RES_M):
    """영역 ((x0, x1), (y0, y1)) m → 셀 수 (nx, ny). 영역 끝까지 덮게 올림한다(마지막 셀은 영역 밖으로 조금 넘을 수 있음).
    실패: res_m ≤ 0 이면 ValueError."""
    if not res_m > 0:
        raise ValueError(f'res_m 은 0 보다 커야 한다: {res_m!r}')
    (x0, x1), (y0, y1) = area
    # 1e-9: 0.4186 / 0.002 같은 나눗셈의 부동소수 잡음으로 셀이 하나 더 생기지 않게
    return max(1, math.ceil((x1 - x0) / res_m - 1e-9)), max(1, math.ceil((y1 - y0) / res_m - 1e-9))


def height_grid(P, area, res_m=HEIGHT_MAP_RES_M):
    """base 점 → 공급 영역 높이 지도 dict(맨 위 칸 설명).

    입력: P = base 점 (H, W, 3) 또는 (N, 3) m — NaN 점은 뺀다(block_finder.base_image 출력 그대로 넘겨도 된다) ·
          area = ((x0, x1), (y0, y1)) m(finder_cfg 의 area_m — robot.yaml supply_area_m) · res_m = 셀 한 변 m.
    출력: {"x0_m","y0_m","res_m","nx","ny","z_m"} — 파이썬 기본형만(JSON 직렬화 가능, NaN 없음).
    영역 밖 점은 버린다(x1 · y1 경계 위 점은 마지막 셀). 바깥 영향 없음. 실패: res_m ≤ 0 이면 ValueError."""
    (x0, x1), (y0, y1) = area
    nx, ny = grid_size(area, res_m)
    pts = np.asarray(P, float).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    keep = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    ix = np.minimum(np.floor((x[keep] - x0) / res_m).astype(int), nx - 1)
    iy = np.minimum(np.floor((y[keep] - y0) / res_m).astype(int), ny - 1)
    grid = np.full(nx * ny, -np.inf)       # NaN 으로 시작하면 max(NaN, z) 가 NaN 이라 -inf 로 시작한다
    np.maximum.at(grid, iy * nx + ix, z[keep])
    return {'x0_m': float(x0), 'y0_m': float(y0), 'res_m': float(res_m), 'nx': int(nx), 'ny': int(ny),
            'z_m': [round(float(v), 4) if np.isfinite(v) else None for v in grid]}


def flat_grid(area, z_m, res_m=HEIGHT_MAP_RES_M):
    """모든 셀이 같은 높이 z_m(m)인 높이 지도 dict(height_grid 와 같은 칸) — 가짜 노드가 '블록 없는 평평한 작업대'로 낸다.
    입력: area = ((x0, x1), (y0, y1)) m · z_m = 작업대 높이 m(robot.yaml table_z_m). 바깥 영향 없음. 실패: res_m ≤ 0 이면 ValueError."""
    (x0, _), (y0, _) = area
    nx, ny = grid_size(area, res_m)
    return {'x0_m': float(x0), 'y0_m': float(y0), 'res_m': float(res_m), 'nx': int(nx), 'ny': int(ny),
            'z_m': [round(float(z_m), 4)] * (nx * ny)}


def finger_margin_m(cfg):
    """손가락 여유 (m) = 가장 넓게 연 폭의 절반 + 손가락 두께(robot.yaml grasp_open_pick_m · finger.thickness_m) — 영역 가장자리 블록의
    손가락 두 자리까지 높이 지도에 들게. 공급 영역 안만 내면 가장자리 블록 손가락 자리의 셀이 모자라 '모름 = 안 막힘'으로 지나갔다
    (10/13 b1 one_pick 대조 #3 — one_pick 은 화면 전체 점을 썼다). 실패: 키가 없으면 KeyError."""
    return max(float(v) for v in cfg['grasp_open_pick_m'].values()) / 2 + float(cfg['finger']['thickness_m'])


def supply_map_area(supply_area, cfg):
    """공급 높이 지도 영역 ((x0, x1), (y0, y1)) m = 공급 영역(((x0, x1), (y0, y1)) m)을 손가락 여유만큼 넓힌 것."""
    (x0, x1), (y0, y1) = supply_area
    m = finger_margin_m(cfg)
    return (x0 - m, x1 + m), (y0 - m, y1 + m)


def spare_map_area(cfg):
    """받침(spare_spots[0]) 높이 지도 영역 — 받침 가운데에서 블록 긴 변 절반 + 손가락 여유까지(받침 위 다시 잡기 전 손가락 자리 확인 —
    one_pick p1_to_recipe, 10/13 b1 대조 #20). 실패: 키가 없으면 KeyError · IndexError."""
    sp = cfg['spare_spots'][0]
    r = float(cfg['block_actual_m'][0]) / 2 + finger_margin_m(cfg)
    return (sp['x_m'] - r, sp['x_m'] + r), (sp['y_m'] - r, sp['y_m'] + r)

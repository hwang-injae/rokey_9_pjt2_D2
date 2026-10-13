"""높이 지도 시험 (pytest, ROS · 로봇 없음) — find_blocks {"height_map": true} 응답 칸(height_grid.py).

손 계산 점으로 셀 자리(z_m[iy·nx + ix]) · 셀마다 최고 z · 빈 셀 null · 영역 밖 버림 · 소수 4자리 · (H, W, 3) 입력을 본다.
"""
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from d2_vision.block_finder import finder_cfg
from d2_vision.height_grid import HEIGHT_MAP_RES_M, finger_margin_m, flat_grid, grid_size, height_grid, spare_map_area, supply_map_area

ROBOT_YAML = Path(__file__).resolve().parents[2] / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml'
AREA = ((0.100, 0.110), (0.200, 0.206))      # 10 × 6 mm → 2 mm 셀 5 × 3


def cell(g, ix, iy):
    return g['z_m'][iy * g['nx'] + ix]


def test_grid_size_rounds_up_and_matches_supply_area():
    assert grid_size(AREA) == (5, 3)
    assert grid_size(((0.0, 0.0101), (0.0, 0.002))) == (6, 1)               # 끝이 조금 넘으면 셀 하나 더(영역 끝까지 덮게)
    area = finder_cfg(yaml.safe_load(ROBOT_YAML.read_text(encoding='utf-8')))['area_m']
    nx, ny = grid_size(area)
    (x0, x1), (y0, y1) = area
    assert (nx - 1) * HEIGHT_MAP_RES_M < x1 - x0 <= nx * HEIGHT_MAP_RES_M + 1e-9
    assert (ny - 1) * HEIGHT_MAP_RES_M < y1 - y0 <= ny * HEIGHT_MAP_RES_M + 1e-9
    with pytest.raises(ValueError):
        grid_size(AREA, 0.0)


def test_cells_keep_max_z_and_null_for_empty():
    pts = np.array([[0.1005, 0.2005, 0.010],      # 셀 (0, 0)
                    [0.1015, 0.2015, 0.012],      # 셀 (0, 0) 더 높음 → 남음
                    [0.1050, 0.2030, 0.0151234],  # 셀 (2, 1) — 소수 4자리
                    [0.1100, 0.2060, 0.020],      # 영역 끝 모서리 → 마지막 셀 (4, 2)
                    [0.0990, 0.2030, 0.900],      # 영역 밖(x) → 버림
                    [0.1050, 0.2070, 0.900],      # 영역 밖(y) → 버림
                    [np.nan, 0.2030, 0.900]])     # 깊이 없음 → 버림
    g = height_grid(pts, AREA)
    assert set(g) == {'x0_m', 'y0_m', 'res_m', 'nx', 'ny', 'z_m'}
    assert (g['x0_m'], g['y0_m'], g['res_m'], g['nx'], g['ny']) == (0.1, 0.2, 0.002, 5, 3)
    assert len(g['z_m']) == 15
    assert cell(g, 0, 0) == 0.012 and cell(g, 2, 1) == 0.0151 and cell(g, 4, 2) == 0.02
    assert sum(v is not None for v in g['z_m']) == 3
    json.dumps(g, allow_nan=False)


def test_image_shaped_points_and_empty():
    """(H, W, 3) base 점(base_image 출력 꼴, NaN 포함)도 받는다. 점이 없으면 모두 null."""
    P = np.full((2, 2, 3), np.nan)
    P[0, 0] = [0.103, 0.203, -0.001]
    g = height_grid(P, AREA)
    assert cell(g, 1, 1) == -0.001 and sum(v is not None for v in g['z_m']) == 1
    assert height_grid(np.zeros((0, 3)), AREA)['z_m'] == [None] * 15


def test_flat_grid_same_fields():
    g = flat_grid(AREA, 0.00004)
    assert {k: g[k] for k in ('x0_m', 'y0_m', 'res_m', 'nx', 'ny')} == {'x0_m': 0.1, 'y0_m': 0.2, 'res_m': 0.002, 'nx': 5, 'ny': 3}
    assert g['z_m'] == [0.0] * 15


def test_공급_지도_영역은_손가락_여유만큼_넓다():
    """supply_map_area = 공급 영역 ± (가장 넓게 연 폭/2 + 손가락 두께) — 가장자리 블록 손가락 자리까지 덮는다(10/13 b1 대조 #3)."""
    cfg = {'grasp_open_pick_m': {'FLAT_SHORT': 0.0365, 'FLAT_LONG': 0.0865}, 'finger': {'thickness_m': 0.0114}}
    m = 0.0865 / 2 + 0.0114
    assert finger_margin_m(cfg) == pytest.approx(m)
    (x0, x1), (y0, y1) = supply_map_area(((0.2, 0.5), (0.1, 0.3)), cfg)
    assert (x0, x1, y0, y1) == pytest.approx((0.2 - m, 0.5 + m, 0.1 - m, 0.3 + m))


def test_받침_지도_영역은_받침_가운데_둘레():
    cfg = {'grasp_open_pick_m': {'FLAT_LONG': 0.0865}, 'finger': {'thickness_m': 0.0114},
           'spare_spots': [{'x_m': 0.3675, 'y_m': -0.285}], 'block_actual_m': [0.07445, 0.0248, 0.0148]}
    r = 0.07445 / 2 + 0.0865 / 2 + 0.0114
    (x0, x1), (y0, y1) = spare_map_area(cfg)
    assert (x0, x1, y0, y1) == pytest.approx((0.3675 - r, 0.3675 + r, -0.285 - r, -0.285 + r))

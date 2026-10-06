"""BlockChecker 시험 (pytest, ROS · 로봇 없음) — SDD 9장 "블록 빼기 · 더 놓기 · 어긋남".

가짜 점군: 벤치 11개 중 k 개를 상자로 쌓고 작업면을 깔아 base 점군을 만든다 (2 mm 간격, 윗면 + 작업면만 — 손목 카메라가 위에서 볼 때 보이는 면).
"""
import math

import numpy as np
import pytest

from d2_vision.block_checker import BlockChecker, depth_to_base_points, height_map

ORIGIN = {'x_m': 0.4261, 'y_m': -0.0725, 'z_m': -0.018, 'yaw_deg': 0.3}
HALF_M = 0.15
L, W, T = 75.0, 25.0, 15.0


def bench_recipe():
    """lv1_bench 와 같은 11개 (벽 2열 × 4층 FLAT_Y + 좌판 3개 FLAT_X). 한세교 cad_recipe/1.0 의 center_cad_mm · R 과 같은 값."""
    blocks = []
    R_y = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]      # 긴 쪽이 설계 y
    R_x = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]       # 긴 쪽이 설계 x
    n = 1
    for layer in range(4):
        for x in (-25.0, 25.0):
            blocks.append(dict(block_id=f'LV1_B{n:03d}', center_cad_mm=[x, 0.0, 7.5 + 15 * layer],
                               size_lwt_mm=[L, W, T], R_cad_from_block=R_y))
            n += 1
    for y in (-25.0, 0.0, 25.0):
        blocks.append(dict(block_id=f'LV1_B{n:03d}', center_cad_mm=[0.0, y, 67.5],
                           size_lwt_mm=[L, W, T], R_cad_from_block=R_x))
        n += 1
    return {'blocks': blocks}


def cad_to_base(p_mm):
    c, s = math.cos(math.radians(ORIGIN['yaw_deg'])), math.sin(math.radians(ORIGIN['yaw_deg']))
    x, y, z = np.array(p_mm) / 1000.0
    return ORIGIN['x_m'] + c * x - s * y, ORIGIN['y_m'] + s * x + c * y, ORIGIN['z_m'] + z


def scene_points(recipe, placed, extra_boxes=(), shift_mm=None, step=0.002):
    """placed 에 든 블록의 윗면 + 작업면 점군(base, m). extra_boxes = [(center_mm, half_mm)] 로 설계에 없는 물체를 더한다.
    shift_mm = {block_id: (dx, dy, dz)} 로 블록을 어긋나게 둔다."""
    pts = []
    g = np.arange(-HALF_M, HALF_M, step)
    X, Y = np.meshgrid(g, g, indexing='ij')
    table = np.stack([X.ravel() + ORIGIN['x_m'], Y.ravel() + ORIGIN['y_m'], np.full(X.size, ORIGIN['z_m'])], 1)
    pts.append(table)
    boxes = []
    for b in recipe['blocks']:
        if b['block_id'] not in placed:
            continue
        ext = np.abs(np.array(b['R_cad_from_block'])) @ np.array(b['size_lwt_mm']) / 2
        c = np.array(b['center_cad_mm'], float)
        if shift_mm and b['block_id'] in shift_mm:
            c = c + np.array(shift_mm[b['block_id']])
        boxes.append((c, ext))
    boxes += [(np.array(c, float), np.array(h, float)) for c, h in extra_boxes]
    for c, ext in boxes:
        gx = np.arange(-ext[0], ext[0], step * 1000)
        gy = np.arange(-ext[1], ext[1], step * 1000)
        GX, GY = np.meshgrid(gx, gy, indexing='ij')
        top = np.stack([GX.ravel() + c[0], GY.ravel() + c[1], np.full(GX.size, c[2] + ext[2])], 1)
        pts.append(np.array([cad_to_base(p) for p in top]))
    return np.vstack(pts)


@pytest.fixture
def checker():
    return BlockChecker(bench_recipe(), ORIGIN, HALF_M)


IDS = [f'LV1_B{i:03d}' for i in range(1, 12)]


def test_all_absent_on_empty_table(checker):
    res = checker.check(scene_points(bench_recipe(), set()), IDS)
    assert [r['state'] for r in res] == ['absent'] * 11
    assert all(math.isnan(r['top_z_m']) is False for r in res)          # 작업면 높이는 잰다
    assert abs(res[0]['top_z_m'] - ORIGIN['z_m']) < 0.001


def test_partial_stack(checker):
    placed = set(IDS[:6])                                                  # 3층까지
    res = checker.check(scene_points(bench_recipe(), placed), IDS)
    states = [r['state'] for r in res]
    assert states[:6] == ['present'] * 6
    assert states[6:8] == ['absent'] * 2                                   # 4층 자리: 3층 윗면이 보임 → 더 낮음
    assert states[8:] == ['absent'] * 3                                    # 좌판 자리: 벽 없는 곳은 작업면
    # 높이: 3층 윗면 = 원점 z + 45 mm, dz ≈ 0
    assert abs(res[4]['top_z_m'] - (ORIGIN['z_m'] + 0.045)) < 0.001
    assert abs(res[4]['dz_m']) < 0.001


def test_complete_bench(checker):
    res = checker.check(scene_points(bench_recipe(), set(IDS)), IDS)
    assert [r['state'] for r in res] == ['present'] * 11
    assert abs(res[10]['top_z_m'] - (ORIGIN['z_m'] + 0.075)) < 0.001
    assert math.isnan(res[0]['top_z_m']) and math.isnan(res[0]['dz_m'])    # 1층 벽은 가려져서 높이를 못 잼


def test_missing_support_under_seat(checker):
    """3층까지만 쌓고 4층 없이 좌판을 올린 (물리적으로 불가능한) 점군이 아니라, 4층 왼쪽(B007)만 빼고 좌판이 그 위에 떠 있는 경우:
    B007 자리는 좌판 윗면(75)이 보이고 설계상 위 블록 윗면과 맞으므로 present 로 추정된다 — 이 추정의 한계를 기록해 둔다."""
    placed = set(IDS) - {'LV1_B007'}
    res = checker.check(scene_points(bench_recipe(), placed), ['LV1_B007'])
    assert res[0]['state'] == 'present'                                    # 한계: 위 블록이 떠 있으면 못 잡는다


def test_extra_block_on_top_is_occluded(checker):
    """좌판 위에 설계에 없는 블록을 하나 더 놓으면 그 아래 좌판은 occluded (더 높게 보임)."""
    extra = [([0.0, 0.0, 82.5], [37.5, 12.5, 7.5])]                       # 좌판 가운데(B010) 위
    res = checker.check(scene_points(bench_recipe(), set(IDS), extra_boxes=extra), ['LV1_B010', 'LV1_B009'])
    assert res[0]['state'] == 'occluded'
    assert res[1]['state'] == 'present'


def test_small_shift_still_present(checker):
    """1 · 3 · 5 mm 옆으로 어긋나도 있음 (SDD 9장) — ±2 mm 판정은 안 한다."""
    for d in (1.0, 3.0, 5.0):
        res = checker.check(scene_points(bench_recipe(), {'LV1_B001', 'LV1_B002'}, shift_mm={'LV1_B001': (d, 0, 0)}),
                            ['LV1_B001'])
        assert res[0]['state'] == 'present', d
        assert math.isnan(res[0]['dx_m'])                                 # 1차: 측정 안 함


def test_unknown_id_and_hole(checker):
    res = checker.check(scene_points(bench_recipe(), set()), ['LV9_X999'])
    assert res[0]['state'] == 'unknown' and math.isnan(res[0]['top_z_m'])
    assert checker.check(np.zeros((0, 3)), ['LV1_B001'])[0]['state'] == 'unknown'   # 점이 없으면 unknown


def test_depth_to_base_roundtrip():
    """깊이 영상 → 점군: 카메라가 작업면 0.35 m 위에서 아래를 볼 때 작업면 점의 z 가 table z 로 나온다."""
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])                                 # 카메라 z 축이 base −z (아래)
    T[:3, 3] = [0.4, 0.0, ORIGIN['z_m'] + 0.35]
    depth = np.full((480, 640), 0.35)
    intr = (600.0, 600.0, 320.0, 240.0)
    pts = depth_to_base_points(depth, intr, T, stride=8)
    assert np.allclose(pts[:, 2], ORIGIN['z_m'], atol=1e-6)
    grid, x0, y0 = height_map(pts, (0.4, 0.0), 0.1)
    assert np.nanmax(grid) - np.nanmin(grid) < 1e-6

"""받침 위 블록 재기 시험 (pytest, ROS · 로봇 없음) — find_blocks {"area": "spare"} (spare_measure.py).

합성 점: 받침 윗면(spare_spots[0] z_m) 평면 + 블록 윗면 1 mm 격자(± 0.5 mm 잡음) + 옆면 몇 줄. 눕힘 · 옆세움 · 세움 · 빈 받침 · 점 모자람.
"""
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from d2_vision.spare_measure import OPEN_GAP_MM, ideal_spare_block, measure_on_spare

ROBOT = yaml.safe_load((Path(__file__).resolve().parents[2] / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml').read_text(encoding='utf-8'))
SPOT = ROBOT['spare_spots'][0]
SIZE = ROBOT['block_actual_m']               # [길이, 폭, 두께] m
L, W, T = SIZE
SX, SY, SZ = SPOT['x_m'], SPOT['y_m'], SPOT['z_m']
KEYS = {'x_m', 'y_m', 'top_z_m', 'yaw_deg', 'up', 'clear', 'gap_mm', 'overlap', 'tilted', 'confidence', 'base_z_m'}
# up → (윗면 yaw 쪽 변, 다른 변, 높이) — 세움 yaw 는 폭(25 mm) 변 기준(block_finder 와 같음)
POSE = {'THICKNESS': (L, W, T), 'WIDTH': (L, T, W), 'LENGTH': (W, T, L)}


def scene(up, cx, cy, yaw_deg, step=0.001, seed=0, support=True):
    """받침 윗면 점 + 블록(자세 up, 가운데 cx · cy, yaw) 윗면 · 옆면 점 (N, 3) m."""
    rng = np.random.default_rng(seed)
    pts = []
    if support:                                  # 받침 윗면 80 × 80 mm(블록 밑까지 — 위에서는 가려지지만 잡음 시험으로 둔다)
        g = np.arange(-0.04, 0.04, 0.002)
        X, Y = np.meshgrid(g, g)
        pts.append(np.c_[SX + X.ravel(), SY + Y.ravel(), np.full(X.size, SZ)])
    a, b, h = POSE[up]
    s, t = np.meshgrid(np.arange(-a / 2, a / 2, step), np.arange(-b / 2, b / 2, step))
    s, t = s.ravel(), t.ravel()
    c, sn = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    top = np.c_[cx + s * c - t * sn, cy + s * sn + t * c, SZ + h + rng.normal(0, 0.0005, s.size)]
    pts.append(top)
    # 옆면 한쪽(카메라가 조금 비스듬히 보는 쪽) — 윗면 띠 밖이라 yaw · 가운데에 섞이지 않아야 한다
    zs = np.arange(SZ + 0.002, SZ + h - 0.006, step)
    ss = np.arange(-a / 2, a / 2, step)
    S, Z = np.meshgrid(ss, zs)
    S, Z = S.ravel(), Z.ravel()
    pts.append(np.c_[cx + S * c - (b / 2) * sn, cy + S * sn + (b / 2) * c, Z])
    return np.vstack(pts)


@pytest.mark.parametrize('up,dx,dy,yaw', [('THICKNESS', 0.0, 0.0, 90.0), ('THICKNESS', 0.004, -0.003, 80.0),
                                          ('WIDTH', -0.002, 0.003, -85.0), ('LENGTH', 0.003, 0.002, 30.0)])
def test_measures_pose_centre_yaw(up, dx, dy, yaw):
    b = measure_on_spare(scene(up, SX + dx, SY + dy, yaw), SPOT, SIZE)
    assert b is not None and set(b) == KEYS
    json.dumps(b, allow_nan=False)
    assert b['up'] == up
    assert abs(b['x_m'] - (SX + dx)) < 0.001 and abs(b['y_m'] - (SY + dy)) < 0.001, b
    want = (yaw + 90.0) % 180.0 - 90.0
    assert abs((b['yaw_deg'] - want + 90.0) % 180.0 - 90.0) < 1.0, b
    assert -90.0 <= b['yaw_deg'] < 90.0
    assert b['top_z_m'] == round(SZ + POSE[up][2], 4) and b['base_z_m'] == round(SZ, 4)
    axes = {'THICKNESS': {'LENGTH', 'WIDTH'}, 'WIDTH': {'LENGTH', 'THICKNESS'}, 'LENGTH': {'WIDTH', 'THICKNESS'}}[up]
    assert set(b['gap_mm']) == axes and set(b['clear']) == axes
    assert all(v == OPEN_GAP_MM for v in b['gap_mm'].values()) and all(b['clear'].values())
    assert (b['overlap'], b['tilted'], b['confidence']) == ('none', False, 1.0)


def test_empty_support_and_too_few_points():
    """받침만 → None. 블록 점이 너무 적음(드문 격자) → None. 받침 가운데에서 먼 블록 → None. NaN 점은 무시."""
    g = np.arange(-0.04, 0.04, 0.001)
    X, Y = np.meshgrid(g, g)
    only = np.c_[SX + X.ravel(), SY + Y.ravel(), SZ + np.random.default_rng(1).normal(0, 0.001, X.size)]
    assert measure_on_spare(only, SPOT, SIZE) is None
    assert measure_on_spare(scene('THICKNESS', SX, SY, 90.0, step=0.008), SPOT, SIZE) is None
    assert measure_on_spare(scene('THICKNESS', SX + 0.12, SY, 90.0), SPOT, SIZE) is None
    assert measure_on_spare(np.zeros((0, 3)), SPOT, SIZE) is None
    pts = scene('THICKNESS', SX, SY, 90.0)
    pts = np.vstack([pts, np.full((10, 3), np.nan)])
    assert measure_on_spare(pts, SPOT, SIZE)['up'] == 'THICKNESS'


def test_ideal_block_matches_measure_fields():
    b = ideal_spare_block(SPOT, SIZE)
    assert set(b) == KEYS and b['up'] == 'THICKNESS' and (b['x_m'], b['y_m']) == (round(SX, 4), round(SY, 4))   # 0.1 mm 반올림
    assert b['yaw_deg'] == (SPOT['yaw_deg'] + 90.0) % 180.0 - 90.0 and b['top_z_m'] == round(SZ + T, 4)
    with pytest.raises(KeyError):
        measure_on_spare(np.zeros((0, 3)), {'x_m': 0.3}, SIZE)

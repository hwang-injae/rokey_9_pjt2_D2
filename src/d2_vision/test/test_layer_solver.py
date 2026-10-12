"""LayerSolver 단위 시험 (pytest, ROS · 로봇 · cv2 없음) — W115 확정 추론기 v12g 의 계산 부분.

1. _min_area_rect: 돌린 사각형 점 → 긴 변 · 짧은 변 · 방향 · 가운데. 점이 모자라면 None.
2. 두 층 판(위 3개 · 아래 1개)을 위에서만 찍으면 위는 보인 블록, 아래는 받침 규칙으로 넣은 inferred 블록.
3. 빈 입력(자세 없음 · 깊이 0)은 예외 없이 블록 [] + 진단의 점 수.
4. 출력 값은 JSON 으로 보낼 수 있는 Python float · bool · str.
5. cv2 를 막아도(sys.modules['cv2'] = None) layer_solver · structure_scanner 를 import 할 수 있다(CI 는 cv2 없음).
합성 깊이 · 가짜 YOLO 마스크는 test_structure_scanner 의 render · fake_masks 를 그대로 쓴다.
"""
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from d2_vision.layer_solver import LayerSolver, _min_area_rect

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_structure_scanner import ACTUAL_MM, CFG, INTR, fake_masks, render   # noqa: E402

O = CFG['assembly_origin']
ORIGIN_MM = np.array([O['x_m'], O['y_m'], O['z_m']]) * 1000.0
HALF_MM = CFG['assembly_area_half_m'] * 1000.0


def solver():
    """robot.yaml 블록 크기(설계 75 × 25 × 15 · 실측) · 조립 영역 반폭으로 만든 LayerSolver."""
    return LayerSolver([v * 1000.0 for v in CFG['block_size_m']], ACTUAL_MM, HALF_MM)


@pytest.mark.parametrize('deg', [0.0, 30.0, 100.0, 170.0])
def test_min_area_rect_rotated_rectangle(deg):
    """75 × 25 mm 사각형 안 점들을 deg 만큼 돌려 (10, −5) 로 옮김 → 긴 변 75 · 짧은 변 25 · 방향 deg · 가운데 (10, −5) (±0.5)."""
    rng = np.random.default_rng(1)
    p = np.c_[rng.uniform(-37.5, 37.5, 2000), rng.uniform(-12.5, 12.5, 2000)]
    p = np.r_[p, [[-37.5, -12.5], [37.5, -12.5], [37.5, 12.5], [-37.5, 12.5]]]       # 모서리를 넣어 크기를 딱 맞춤
    a = math.radians(deg)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    cx, cy, long_, short, ang = _min_area_rect(p @ R.T + [10.0, -5.0])
    assert (cx, cy) == pytest.approx((10.0, -5.0), abs=0.5)
    assert (long_, short) == pytest.approx((75.0, 25.0), abs=0.5)
    assert min(abs(ang - deg), 180 - abs(ang - deg)) < 0.5 and 0.0 <= ang < 180.0


def test_min_area_rect_degenerate():
    """점이 3개 미만이거나 한 줄 위면 None(윗면으로 쓰지 않음)."""
    assert _min_area_rect(np.zeros((2, 2))) is None
    assert _min_area_rect(np.c_[np.arange(10.0), np.arange(10.0)]) is None


def test_two_layer_plate_top_only_infers_lower():
    """75 × 75 두 층 판(아래 x 3개 · 위 y 3개)을 위에서 한 장만 찍음 → 위 y 3개는 보인 블록, 아래는 inferred(받침 규칙)."""
    plate = [dict(order=i + 1, x=0.0, y=y, z=0.0, ori='x') for i, y in enumerate((-25.0, 0.0, 25.0))]
    plate += [dict(order=4 + i, x=x, y=0.0, z=15.0, ori='y') for i, x in enumerate((-25.0, 0.0, 25.0))]
    depth, T, lab = render(plate, 'observe', offset=(0.0, 0.0), labels=True)
    blocks, diag = solver().solve([('observe', depth, INTR, T)], ORIGIN_MM, O['yaw_deg'], [fake_masks(lab)])
    top = [b for b in blocks if b['z'] == 15.0]
    low = [b for b in blocks if b['z'] == 0.0]
    assert sorted(b['x'] for b in top) == [-25.0, 0.0, 25.0], blocks
    assert all(b['ori'] == 'y' and not b['inferred'] for b in top)
    assert low and all(b['inferred'] for b in low), blocks
    assert len(blocks) == len(top) + len(low) and diag['faces'] >= 3


def test_empty_input_no_exception():
    """자세 없음 · 깊이가 모두 0 → 예외 없이 블록 [] 이고 진단 points 가 0."""
    s = solver()
    blocks, diag = s.solve([], ORIGIN_MM, 0.0, [])
    assert blocks == [] and diag['points'] == 0
    T = render([], 'observe')[1]
    blocks, diag = s.solve([('observe', np.zeros((480, 640), np.uint16), INTR, T)], ORIGIN_MM, 0.0, [[]])
    assert blocks == [] and diag['points'] == 0


def test_output_python_types():
    """블록 칸이 Python float · str · bool 이고 진단까지 JSON 으로 바꿀 수 있다(다리 · 화면으로 그대로 보냄)."""
    plate = [dict(order=1, x=0.0, y=0.0, z=0.0, ori='x'), dict(order=2, x=0.0, y=0.0, z=15.0, ori='y')]
    views, masks = [], []
    for i, p in enumerate(('observe', 'observe_front', 'observe_side')):
        depth, T, lab = render(plate, p, seed=i, labels=True)
        views.append((p, depth, INTR, T))
        masks.append(fake_masks(lab))
    blocks, diag = solver().solve(views, ORIGIN_MM, O['yaw_deg'], masks)
    assert blocks
    for b in blocks:
        assert set(b) == {'x', 'y', 'z', 'ori', 'inferred'}
        assert all(type(b[k]) is float for k in ('x', 'y', 'z'))
        assert type(b['ori']) is str and type(b['inferred']) is bool
    json.dumps(blocks, allow_nan=False)
    json.dumps(diag, default=str)


def test_import_without_cv2():
    """cv2 를 막은 새 파이썬에서 layer_solver · structure_scanner 를 import 하고 작은 풀이를 돌린다(CI 에 cv2 가 없음)."""
    src = str(Path(__file__).resolve().parents[1])
    code = ("import sys; sys.modules['cv2'] = None\n"
            "import numpy as np\n"
            "import d2_vision.layer_solver as m, d2_vision.structure_scanner\n"
            "print(m.LayerSolver((75, 25, 15), (74.45, 24.8, 14.8), 150).solve([], np.zeros(3), 0.0, [])[0])\n")
    env = {**os.environ, 'PYTHONPATH': src + os.pathsep + os.environ.get('PYTHONPATH', '')}
    r = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, env=env, timeout=120)
    assert r.returncode == 0, r.stderr
    assert r.stdout.strip() == '[]'

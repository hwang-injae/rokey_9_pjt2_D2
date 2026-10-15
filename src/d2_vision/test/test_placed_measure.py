"""조립 자리에 놓은 블록 하나 재기 시험 (pytest, ROS · 로봇 없음) — find_blocks {"area": "assembly"} (placed_measure.py, W144).

합성 점: 작업면(z = 0) 2 mm 격자 + 상자마다 윗면 1 mm 격자 + 옆면 한 줄(윗면 6 mm 아래까지) — 손목 카메라가 위에서 볼 때 보이는 면.
위 상자의 발자국 아래에 있는 점(작업면 · 아래층 윗면)은 위에서 안 보이므로 뺀다. 잡음은 z 에만 σ 0.5 mm(seed 고정).
설계 자리(part)는 recipe_parts() 출력 모양 {part_id, center m, rot 3x3 base←블록} 을 여기서 직접 만든다(test_block_checker 와 같은 방식).
"""
import json
import math

import numpy as np
import pytest

from d2_vision.placed_measure import CORNER_MIN_PTS, REJECTS, corner_heights, measure_placed, up_of

SIZE = [0.07445, 0.0248, 0.0148]          # 실측 블록 (robot.yaml block_actual_m) [길이, 폭, 두께] m
L, W, T = SIZE
CX, CY = 0.40, -0.05                      # 설계 자리 가운데(수평) m — 작업면 z = 0
KEYS = {'part_id', 'x_m', 'y_m', 'top_z_m', 'yaw_deg', 'up', 'tilted', 'confidence', 'size_mm', 'corners_z_m'}
R_FLAT = np.eye(3)                                                   # 눕힘: 길이 x · 폭 y · 두께 위
R_EDGE = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])   # 옆세움: 길이 x · 폭 위 · 두께 y
R_STAND = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])   # 세움: 길이 위 · 폭 x (test_block_checker 와 같음)


def rot_z(R, yaw_deg):
    """R 을 z 축으로 yaw_deg(도) 돌린 회전(base←블록)."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ np.asarray(R, float)


def rot_axis(axis, deg):
    """블록 좌표 축(0 x · 1 y) 둘레로 deg 도 돌리는 회전 — 기울어진 윗면을 만들 때 블록 회전 뒤에 곱한다."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == 0:
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def part(yaw_deg=0.0, layer=0, R=R_FLAT, xy=(CX, CY), part_id='001_CHAIR_BENCH_V000_SEAT_001_01'):
    """설계 자리 하나(recipe_parts 모양). layer = 그 아래 쌓인 두께 층 수 — 가운데 z = layer·T + 위 축 치수 / 2."""
    rot = rot_z(R, yaw_deg)
    h = float(np.abs(rot[2]) @ np.array(SIZE)) / 2.0
    return {'part_id': part_id, 'center': (xy[0], xy[1], layer * T + h), 'rot': rot}


def box(center, rot, top_step=0.001, keep=None):
    """상자 하나(가운데 m · 회전 base←블록 · 윗면 격자 간격 m · keep(local) → bool 배열로 윗면 점 고르기)."""
    return {'c': np.asarray(center, float), 'rot': np.asarray(rot, float), 'step': top_step, 'keep': keep}


def _axes(rot):
    """회전 → (위 축 k, 위 축 부호, 수평 두 축 i · j(i 가 긴 쪽))."""
    k = int(np.argmax(np.abs(rot[2])))
    i, j = (a for a in range(3) if a != k)
    return k, float(np.sign(rot[2, k])), i, j


def _footprint(p, b):
    """점 p (N, 3) 가 상자 b 의 위에서 본 발자국 안인지(수평 거리만)."""
    _, _, i, j = _axes(b['rot'])
    d = p - b['c']
    d[:, 2] = 0.0
    local = d @ b['rot']
    half = np.array(SIZE) / 2.0
    return (np.abs(local[:, i]) <= half[i]) & (np.abs(local[:, j]) <= half[j])


def scene(boxes, table=True, noise=0.0005, seed=0, sides=True):
    """작업면 + 상자들의 윗면(· 옆면 한 줄) 점 (N, 3) m. 높은 상자 발자국 아래 점(작업면 · 낮은 윗면)은 가려져 뺀다."""
    rng = np.random.default_rng(seed)
    half = np.array(SIZE) / 2.0
    pts, owner = [], []
    if table:                                     # 설계 자리 둘레 ±120 mm 작업면 2 mm 격자
        g = np.arange(-0.12, 0.12, 0.002)
        X, Y = np.meshgrid(g, g)
        pts.append(np.c_[CX + X.ravel(), CY + Y.ravel(), np.zeros(X.size)])
        owner.append(np.full(X.size, -1))
    for n, b in enumerate(boxes):
        k, sgn, i, j = _axes(b['rot'])
        st = b['step']
        s, t = np.meshgrid(np.arange(-half[i] + st / 2, half[i], st), np.arange(-half[j] + st / 2, half[j], st))
        local = np.zeros((s.size, 3))
        local[:, i], local[:, j], local[:, k] = s.ravel(), t.ravel(), sgn * half[k]
        if b['keep'] is not None:
            local = local[b['keep'](local)]
        pts.append(local @ b['rot'].T + b['c'])
        owner.append(np.full(len(local), n))
        if sides and 2 * half[k] > 0.008:            # 옆면(짧은 수평 축 + 쪽) — 바닥 + 2 mm ~ 윗면 − 6 mm, 윗면 띠 밖
            S, R = np.meshgrid(np.arange(-half[i], half[i], 0.001), np.arange(-half[k] + 0.002, half[k] - 0.006, 0.001))
            side = np.zeros((S.size, 3))
            side[:, i], side[:, j], side[:, k] = S.ravel(), half[j], sgn * R.ravel()
            pts.append(side @ b['rot'].T + b['c'])
            owner.append(np.full(len(side), n))
    p, own = np.vstack(pts), np.concatenate(owner)
    for n, b in enumerate(boxes):                    # 가림: 이 상자 발자국 안 · 이 상자 윗면보다 낮은 남의 점
        top = b['c'][2] + float(np.abs(b['rot'][2]) @ np.array(SIZE)) / 2.0
        hid = (own != n) & (p[:, 2] < top - 0.001) & _footprint(p.copy(), b)
        p, own = p[~hid], own[~hid]
    p[:, 2] += rng.normal(0.0, noise, len(p))
    return p


def yaw_diff(a, b):
    """두 yaw(도) 차 — 블록 180° 대칭이라 −90 ~ 90 으로 감는다."""
    return (a - b + 90.0) % 180.0 - 90.0


def design_top(pt):
    """설계 윗면 높이 m."""
    return pt['center'][2] + float(np.abs(pt['rot'][2]) @ np.array(SIZE)) / 2.0


def check_valid(res, pt, x, y, yaw, up='THICKNESS', top=None, xy_tol=0.001, yaw_tol=2.0):
    """정상 답(블록 dict, None)인지 · 칸 · 위치 · yaw · 윗면 높이를 본다."""
    blk, rej = res
    assert rej is None and blk is not None, rej
    assert set(blk) == KEYS
    json.dumps(blk, allow_nan=False)
    assert blk['part_id'] == pt['part_id'] and blk['up'] == up
    assert abs(blk['x_m'] - x) < xy_tol and abs(blk['y_m'] - y) < xy_tol, blk
    assert -90.0 <= blk['yaw_deg'] < 90.0
    assert abs(yaw_diff(blk['yaw_deg'], yaw)) < yaw_tol, blk
    assert abs(blk['top_z_m'] - (design_top(pt) if top is None else top)) < 0.0015, blk
    assert (blk['tilted'], blk['confidence']) == (False, 1.0)
    assert isinstance(blk['corners_z_m'], list) and len(blk['corners_z_m']) == 4
    return blk


def check_reject(res, code):
    """거절 답(None, (code, 설명))인지."""
    blk, rej = res
    assert blk is None and rej is not None
    assert rej[0] == code, rej
    assert rej[0] in REJECTS and isinstance(rej[1], str) and rej[1]


# ---------- 정상 ----------

@pytest.mark.parametrize('yaw', [0.0, 37.0, 90.0, -45.0])
def test_flat_block_at_design_pose(yaw):
    """1. 설계 자리 그대로 놓인 눕힌 블록 → 가운데 1 mm · yaw 2° · 윗면 1.5 mm 안, up THICKNESS."""
    pt = part(yaw)
    blk = check_valid(measure_placed(scene([box(pt['center'], pt['rot'])]), pt, SIZE), pt, CX, CY, yaw)
    assert len(blk['size_mm']) == 2


@pytest.mark.parametrize('yaw', [0.0, 37.0, 90.0])
def test_flat_block_shifted_and_turned(yaw):
    """2. 설계에서 (+3, −2) mm · +4° 어긋나 놓임 → 잰 가운데 · yaw 는 실제 놓인 자리(1 mm · 1.5°)."""
    pt = part(yaw)
    x, y = CX + 0.003, CY - 0.002
    actual = box((x, y, pt['center'][2]), rot_z(R_FLAT, yaw + 4.0))
    check_valid(measure_placed(scene([actual]), pt, SIZE), pt, x, y, yaw + 4.0, yaw_tol=1.5)


def test_block_on_lower_layer():
    """3. 아래층 두 블록(벽, 긴 쪽 y) 위에 가로로 놓인 좌판 → 15 mm 아래 아래층 윗면이 섞이지 않고 윗면 높이가 맞다."""
    pt = part(0.0, layer=1)
    lower = [box((CX + dx, CY, T / 2), rot_z(R_FLAT, 90.0)) for dx in (-0.025, 0.025)]
    pts = scene(lower + [box(pt['center'], pt['rot'])])
    check_valid(measure_placed(pts, pt, SIZE), pt, CX, CY, 0.0, top=2 * T)


@pytest.mark.parametrize('where,gap', [('side', 0.020), ('side', 0.030), ('end', 0.020)])
def test_neighbor_with_clear_gap_is_ok(where, gap):
    """5. 같은 높이 이웃이 20 mm 넘게 떨어져 있으면(REACH 15 mm 밖) 정상."""
    pt = part(0.0)
    off = (0.0, W + gap) if where == 'side' else (L + gap, 0.0)
    nb = box((CX + off[0], CY + off[1], T / 2), R_FLAT)
    check_valid(measure_placed(scene([box(pt['center'], pt['rot']), nb]), pt, SIZE), pt, CX, CY, 0.0)


def test_edge_block_width_up():
    """12. 옆세움(폭 위, 윗면 74.5 × 14.8 mm) 설계 자리 그대로 → up WIDTH, 윗면 = 폭 높이."""
    pt = part(30.0, R=R_EDGE)
    assert up_of(pt['rot']) == 'WIDTH'
    check_valid(measure_placed(scene([box(pt['center'], pt['rot'])]), pt, SIZE), pt, CX, CY, 30.0, up='WIDTH', top=W)


def test_nan_points_and_image_shape():
    """13. NaN 점은 빼고, (H, W, 3) 모양 입력도 (N, 3) 와 같은 답."""
    pt = part(20.0)
    pts = scene([box(pt['center'], pt['rot'])])
    want, _ = measure_placed(pts, pt, SIZE)
    assert want is not None
    n = len(pts)
    w = 100
    h = -(-(n + 50) // w)
    img = np.full((h * w, 3), np.nan)
    img[:n] = pts
    img[n:n + 50, 0] = 0.5                     # 한 칸만 NaN 인 점도 뺀다
    got, rej = measure_placed(img.reshape(h, w, 3), pt, SIZE)
    assert rej is None and got == want


# ---------- 거절 ----------

def _neighbor_merged(where='side', gap=0.0):
    pt = part(0.0)
    off = (0.0, W + gap) if where == 'side' else (L + gap, 0.0)
    return scene([box(pt['center'], pt['rot']), box((CX + off[0], CY + off[1], T / 2), R_FLAT)]), pt


def _empty():
    return scene([]), part(0.0)


def _layer_missing():
    pt = part(0.0, layer=1)                    # 설계는 2층인데 블록이 1층(15 mm 낮게)에 있음
    return scene([box((CX, CY, T / 2), R_FLAT)]), pt


def _covered_full():
    pt = part(0.0)
    return scene([box(pt['center'], pt['rot']), box((CX, CY, 1.5 * T), R_FLAT)]), pt


def _covered_cross():
    pt = part(0.0)                             # 위에 가로 블록 하나가 가운데 25 × 25 mm 를 덮음
    return scene([box(pt['center'], pt['rot']), box((CX, CY, 1.5 * T), rot_z(R_FLAT, 90.0))]), pt


def _standing():
    return np.zeros((0, 3)), part(0.0, R=R_STAND)


def _few_points():
    pt = part(0.0)                             # 윗면 6 mm 격자 = 13 × 4 = 52점(필요 200 × 0.5 = 100 미만, NO_PART 20 이상)
    return scene([box(pt['center'], pt['rot'], top_step=0.006)]), pt


def _few_points_hole():
    pt = part(0.0)                             # 윗면 대부분이 깊이 구멍 — 한쪽 끝 3 × 24 mm 띠(약 72점)만 남음
    keep = lambda lc: lc[:, 0] > L / 2 - 0.003
    return scene([box(pt['center'], pt['rot'], keep=keep)]), pt


def _tilted(axis, deg=7.0):
    pt = part(0.0)
    return scene([box(pt['center'], pt['rot'] @ rot_axis(axis, deg))]), pt


def _lifted(axis, h_mm, yaw=0.0):
    """한쪽 모서리를 h_mm 들어 올린 블록(그 모서리 밑에 무언가가 끼어 걸친 모양). axis 0 = 긴 모서리(폭 방향으로 기욺) · 1 = 짧은 끝(길이 방향)."""
    pt = part(yaw)
    span = W if axis == 0 else L
    c = np.array(pt['center'], float)
    c[2] += h_mm / 2000.0                      # 한쪽이 바닥에 닿은 채 다른 쪽이 h 만큼 들리면 가운데는 h/2 올라간다
    return scene([box(c, pt['rot'] @ rot_axis(axis, math.degrees(math.asin(h_mm / 1000.0 / span))))]), pt


def _half_hidden():
    pt = part(0.0)                             # 긴 축 한쪽 절반 윗면이 안 보임(가장자리 가려짐)
    keep = lambda lc: lc[:, 0] < 0.0
    return scene([box(pt['center'], pt['rot'], keep=keep)]), pt


REJECT_CASES = {
    'neighbor_side_flush': (lambda: _neighbor_merged('side', 0.0), 'NEIGHBOR_MERGED'),
    'neighbor_side_1mm': (lambda: _neighbor_merged('side', 0.001), 'NEIGHBOR_MERGED'),
    'neighbor_end_flush': (lambda: _neighbor_merged('end', 0.0), 'NEIGHBOR_MERGED'),
    'empty': (_empty, 'NO_PART'),
    'layer_missing': (_layer_missing, 'NO_PART'),
    'covered_full': (_covered_full, 'COVERED'),
    'covered_cross': (_covered_cross, 'COVERED'),
    'standing': (_standing, 'STANDING'),
    'few_points_grid': (_few_points, 'FEW_POINTS'),
    'few_points_hole': (_few_points_hole, 'FEW_POINTS'),
    'tilted_long': (lambda: _tilted(1), 'TILTED'),        # 짧은 축(폭) 둘레 7° — 긴 축 양 끝 높이 차 약 9 mm
    'tilted_side': (lambda: _lifted(0, 8.0), 'TILTED'),   # 긴 모서리 한쪽 8 mm 들림(약 19°) — 양 끝 차는 작고 폭 방향 각도 기준(10°)으로 걸림
    'half_hidden': (_half_hidden, 'SIZE_MISMATCH'),
}


@pytest.mark.parametrize('name', list(REJECT_CASES))
def test_reject_cases(name):
    """4 · 6 · 7 · 8 · 9 · 10 · 11. 이웃 붙음 · 빈 자리 · 한 층 빠짐 · 덮임 · 세움 · 점 모자람 · 기욺 · 절반 가려짐 → 각 거절 코드."""
    make, code = REJECT_CASES[name]
    pts, pt = make()
    check_reject(measure_placed(pts, pt, SIZE), code)


def test_rejects_tuple_covers_every_code():
    """14. 위 거절 시험이 내는 코드가 REJECTS 와 같은 묶음(빠짐 · 남는 것 없음)."""
    got = set()
    for make, _ in REJECT_CASES.values():
        pts, pt = make()
        blk, rej = measure_placed(pts, pt, SIZE)
        assert blk is None and rej[0] in REJECTS
        got.add(rej[0])
    assert got == set(REJECTS) and len(REJECTS) == len(set(REJECTS))


def test_side_tilt_below_wide_limit_is_ok():
    """폭 방향(긴 축 둘레) 7° 는 놓은 블록 전용 문턱(TILT_WIDE_DEG 10°) 아래라 정상으로 잰다(10/15 — 깊이 가장자리 처짐으로
    반듯한 블록도 폭 방향 4 ~ 6° 로 재지는 것을 막으려고). 예전(5°)에는 TILTED 였다."""
    pts, pt = _tilted(0, 7.0)
    blk, rej = measure_placed(pts, pt, SIZE)
    assert rej is None and blk['tilted'] is False, rej


@pytest.mark.parametrize('yaw', [0.0, 30.0, 90.0])
@pytest.mark.parametrize('h_mm', [5.0, 8.0])
def test_end_lift_along_length_is_tilted(h_mm, yaw):
    """짧은 끝이 길이 방향으로 5 · 8 mm 들린 블록은 TILTED(10/15 PL — 들린 끝이 높이 띠 밖으로 잘려 평면 기울기로는 못 잡던 것).
    10 mm 는 들린 끝이 설계 자리 위 8 mm 를 넘어 먼저 COVERED 로 거절될 수 있어 여기서는 뺀다(거절이면 다시 집지 않음은 같다)."""
    pts, pt = _lifted(1, h_mm, yaw)
    check_reject(measure_placed(pts, pt, SIZE), 'TILTED')


@pytest.mark.parametrize('yaw', [0.0, 30.0, 90.0])
def test_small_end_lift_is_measured(yaw):
    """3 mm 들림(양 끝 차가 TILT_MM 5 mm 아래)은 끝 들림 조건에 안 걸리고 정상으로 잰다."""
    pts, pt = _lifted(1, 3.0, yaw)
    blk, rej = measure_placed(pts, pt, SIZE)
    assert rej is None and blk['tilted'] is False, rej


# ---------- 10/14 고친 것(하위 검토에서 찾음 — 크기 되돌림 · 틈 보이는 이웃 빼기) ----------

def test_size_mm_reports_true_size():
    """다 보이는 윗면의 size_mm 은 실제 두 변(74.45 · 24.8 mm) ± 2 mm 여야 한다."""
    pt = part(0.0)
    blk, _ = measure_placed(scene([box(pt['center'], pt['rot'])]), pt, SIZE)
    assert abs(blk['size_mm'][0] - L * 1000) < 2.0 and abs(blk['size_mm'][1] - W * 1000) < 2.0, blk['size_mm']


def test_slightly_eroded_edges_are_ok():
    """긴 변 양 끝이 1 mm 씩 깊이 가장자리에서 빠져도(2 mm 작음 < SIZE_TOL 4 mm) 정상이어야 한다."""
    pt = part(0.0)
    keep = lambda lc: np.abs(lc[:, 0]) < L / 2 - 0.001
    check_valid(measure_placed(scene([box(pt['center'], pt['rot'], keep=keep)]), pt, SIZE), pt, CX, CY, 0.0)


def test_neighbor_with_visible_8mm_gap_is_ok():
    """같은 높이 이웃이 8 mm 떨어져 틈이 보이면 이 블록만 재야 한다."""
    pt = part(0.0)
    nb = box((CX, CY + W + 0.008, T / 2), R_FLAT)
    check_valid(measure_placed(scene([box(pt['center'], pt['rot']), nb]), pt, SIZE), pt, CX, CY, 0.0)


def test_missing_part_field_raises():
    with pytest.raises(KeyError):
        measure_placed(np.zeros((0, 3)), {'part_id': 'X', 'rot': np.eye(3)}, SIZE)


@pytest.mark.parametrize('sigma', [0.001, 0.002, 0.003])
def test_top_height_is_not_raised_by_depth_noise(sigma):
    """윗면 높이 = 윗면 점의 가운데 값 — 깊이 잡음 σ 1 ~ 3 mm 에서도 1 mm 안(98 % 백분위였을 때는 +2 ~ +4 mm 높게 나왔다, 10/14 하위 검토)."""
    pt = part(0.0)
    blk, why = measure_placed(scene([box(pt['center'], pt['rot'])], noise=sigma), pt, SIZE)
    assert why is None and abs(blk['top_z_m'] - (pt['center'][2] + T / 2)) < 0.001, (sigma, blk)


# ---------- 모서리 높이 corners_z_m (10/15 한석형 #190) ----------

def test_corners_flat_block_are_at_top():
    """반듯한 블록: 네 모서리 모두 값(null 없음)이고 설계 윗면 ±1.5 mm, 서로 1.5 mm 안."""
    pt = part(30.0)
    blk, _ = measure_placed(scene([box(pt['center'], pt['rot'])]), pt, SIZE)
    cz = blk['corners_z_m']
    assert all(v is not None for v in cz) and max(cz) - min(cz) < 0.0015
    assert all(abs(v - design_top(pt)) < 0.0015 for v in cz), cz


def test_corners_long_edge_lifted_3mm_shows_on_one_side():
    """긴 모서리 한쪽이 3 mm 들림(폭 방향 약 7° — 10° 문턱 아래라 잰다): +v 쪽 두 모서리가 −v 쪽보다 높다.
    모서리 칸은 끝에서 CORNER_INSET + CELL/2 = 7 mm 안쪽이라 칸 사이 차는 가장자리 들림 × (24.8 − 14) / 24.8 ≈ 3 × 0.44 = 1.3 mm
    (한석형 Z 문턱 5 mm 는 폭 방향 가장자리 들림 약 11 mm · 길이 방향 약 6 mm 에 해당). 차례 (−u,−v) (−u,+v) (+u,−v) (+u,+v) — v 가 뒤집히면 부호만 반대."""
    pts, pt = _lifted(0, 3.0)
    blk, rej = measure_placed(pts, pt, SIZE)
    assert rej is None, rej
    cz = blk['corners_z_m']
    assert all(v is not None for v in cz)
    expect = 0.003 * (W - 2 * 0.007) / W
    d1, d2 = cz[1] - cz[0], cz[3] - cz[2]
    assert abs(abs(d1) - expect) < 0.0006 and abs(abs(d2) - expect) < 0.0006 and d1 * d2 > 0, (cz, expect)


def test_corner_heights_window_keeps_lifted_corner_and_nulls_empty_cell():
    """corner_heights 자체: 한 모서리 칸만 12 mm 올려도 null 이 아니라 그 값(±5 mm 띠가 아니라 −8 ~ +20 mm 창), 점이 CORNER_MIN_PTS 미만인 칸은 null."""
    rng = np.random.default_rng(1)
    g = np.arange(-L / 2, L / 2, 0.001); h = np.arange(-W / 2, W / 2, 0.001)
    gx, gy = np.meshgrid(g, h)
    p = np.c_[gx.ravel(), gy.ravel(), np.zeros(gx.size)] + rng.normal(0, 0.0003, (gx.size, 3))
    corner = (p[:, 0] > L / 2 - 0.012) & (p[:, 1] > W / 2 - 0.012)              # (+u,+v) 모서리 둘레 12 mm 를 12 mm 올림
    p[corner, 2] += 0.012
    cz = corner_heights(p, (0.0, 0.0), 0.0, 0.0, L, W)
    assert cz[3] is not None and abs(cz[3] - 0.012) < 0.001 and all(abs(cz[i]) < 0.001 for i in range(3)), cz
    few = p[~((p[:, 0] < -L / 2 + 0.012) & (p[:, 1] < -W / 2 + 0.012))]      # (−u,−v) 칸 점을 지움
    cz = corner_heights(few, (0.0, 0.0), 0.0, 0.0, L, W)
    assert cz[0] is None and all(v is not None for v in cz[1:]), cz
    assert CORNER_MIN_PTS >= 3
    json.dumps({'corners_z_m': cz}, allow_nan=False)                           # null 은 JSON 으로 나간다


def test_corners_follow_measured_pose_not_design():
    """블록이 긴 축으로 8 mm 비켜 놓이면 모서리 칸도 잰 자리를 따라가 네 모서리 모두 윗면 높이(설계 자리 기준이면 끝 칸이 블록 밖 — 아래층을 잼)."""
    pt = part(0.0)
    c = np.array(pt['center']); c[0] += 0.008
    blk, rej = measure_placed(scene([box(c, pt['rot'])]), pt, SIZE)
    assert rej is None, rej
    cz = blk['corners_z_m']
    assert all(v is not None and abs(v - design_top(pt)) < 0.0015 for v in cz), cz

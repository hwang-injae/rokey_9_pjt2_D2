"""놓은 블록 하나를 마스크 + 광선으로 재기 시험 (pytest, ROS · 로봇 · cv2 없음) — find_blocks {"area": "assembly"} placed_method:=mask (placed_mask.py, W144).

합성 장면: 카메라가 설계 자리 바로 위 CAM_H 에서 똑바로 내려다본다(640×480, fx = fy = 600). 픽셀마다 광선을 쏴 작업면(z = 0)과
상자들(slab 교차 — 기울어진 상자도) 중 처음 맞는 점을 base 점 영상 (H, W, 3) 으로 만든다(block_finder.base_image 와 같은 꼴).
마스크 = 그 상자를 처음 맞힌 픽셀(YOLO 윤곽 대신 — 옆면 포함). 깊이 잡음은 z 에만 σ 0.5 mm(seed 고정).
'가장자리 처짐'은 윗면 가장자리 SAG_W 안 점의 z 를 SAG_DROP 내려 만든다(실기 깊이가 윗면 가장자리를 처지게 재는 것, 10/15 A4).
"""
import json
import math

import numpy as np
import pytest

from d2_vision.placed_mask import _project, measure_placed_mask, measure_placed_mask_detail
from d2_vision.placed_measure import REJECTS, measure_placed

SIZE = [0.07445, 0.0248, 0.0148]          # 실측 블록 (robot.yaml block_actual_m) [길이, 폭, 두께] m
L, W, T = SIZE
CX, CY = 0.40, -0.05                      # 설계 자리 가운데(수평) m — 작업면 z = 0
CAM_H = 0.30                              # 카메라 높이 m — 1 px ≈ 0.48 mm 라 픽셀 양자화(±반 픽셀씩 두 끝)가 ±0.5 mm 안(실기 0.5 m 는 0.8 mm/px)
INTR = (600.0, 600.0, 320.0, 240.0)       # (fx, fy, cx, cy) 픽셀
HW = (480, 640)
SAG_W, SAG_DROP = 0.004, 0.008            # 처짐: 윗면 가장자리 4 mm 띠가 8 mm 낮게 — 깊이 윗면 띠(±5 mm)에서 빠진다
KEYS = {'part_id', 'x_m', 'y_m', 'top_z_m', 'yaw_deg', 'up', 'tilted', 'confidence', 'size_mm', 'corners_z_m'}


def rot_z(yaw_deg):
    """z 축 둘레 yaw_deg(도) 회전(base←블록, 눕힘 — 길이 x · 폭 y · 두께 위)."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_axis(axis, deg):
    """블록 좌표 축(0 x · 1 y) 둘레로 deg 도 — 기울어진 윗면을 만들 때 블록 회전 뒤에 곱한다."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == 0:
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def part(yaw_deg=0.0, layer=0, xy=(CX, CY), part_id='001_CHAIR_BENCH_V000_SEAT_001_01'):
    """설계 자리 하나(recipe_parts 모양) — 눕힌 블록, layer = 그 아래 두께 층 수."""
    return {'part_id': part_id, 'center': (xy[0], xy[1], layer * T + T / 2.0), 'rot': rot_z(yaw_deg)}


def cam_pose(xy=(CX, CY)):
    """xy 위 CAM_H 에서 똑바로 내려다보는 카메라 → base 4x4 m(카메라 x = base x · 카메라 y = base −y · 광축 = base −z)."""
    Tc = np.eye(4)
    Tc[:3, :3] = np.diag([1.0, -1.0, -1.0])
    Tc[:3, 3] = [xy[0], xy[1], CAM_H]
    return Tc


def render(boxes, Tc=None, sag=False, noise=0.0005, seed=0):
    """상자 목록 [(가운데 m, 회전 base←블록)] → (base 점 영상 (H, W, 3) m, 마스크 목록 [bool (H, W)] — 상자마다 처음 맞힌 픽셀)."""
    Tc = cam_pose() if Tc is None else Tc
    fx, fy, cx, cy = INTR
    v, u = np.mgrid[0:HW[0], 0:HW[1]]
    d = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones(HW)], axis=-1).reshape(-1, 3) @ Tc[:3, :3].T
    o = Tc[:3, 3]
    t_best = -o[2] / d[:, 2]                       # 작업면 z = 0
    owner = np.full(len(d), -1)
    half = np.array(SIZE) / 2.0
    for n, (c, R) in enumerate(boxes):
        R = np.asarray(R, float)
        ol = R.T @ (o - np.asarray(c, float))
        dl = d @ R                                 # 블록 좌표 방향(R^T d)
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (-half - ol) / dl, (half - ol) / dl
        t_near = np.nanmax(np.minimum(t1, t2), axis=1)
        t_far = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (t_near <= t_far) & (t_far > 0) & (t_near < t_best)
        t_best[hit], owner[hit] = t_near[hit], n
    P = o + d * t_best[:, None]
    if sag:                                        # 윗면 가장자리 SAG_W 띠를 SAG_DROP 아래로
        for n, (c, R) in enumerate(boxes):
            lc = (P - np.asarray(c, float)) @ np.asarray(R, float)
            top = (owner == n) & (lc[:, 2] > half[2] - 1e-6)
            edge = np.minimum(half[0] - np.abs(lc[:, 0]), half[1] - np.abs(lc[:, 1])) < SAG_W
            P[top & edge, 2] -= SAG_DROP
    P[:, 2] += np.random.default_rng(seed).normal(0.0, noise, len(P))
    return P.reshape(HW + (3,)), [(owner == n).reshape(HW) for n in range(len(boxes))]


def placed(pt, dxy=(0.0, 0.0), dyaw=0.0):
    """설계 자리 pt 에서 dxy(m) · dyaw(도) 비킨 실제 블록 (가운데, 회전)."""
    c = np.array(pt['center'], float) + [dxy[0], dxy[1], 0.0]
    return c, rot_z(dyaw) @ pt['rot']


def yaw_of(R):
    """블록 회전의 긴 축 yaw(도, −90 ~ 90)."""
    return (math.degrees(math.atan2(R[1, 0], R[0, 0])) + 90.0) % 180.0 - 90.0


def run(P, masks, pt, shift=(0, 0), Tc=None):
    """measure_placed_mask_detail 을 장면 기본값으로."""
    return measure_placed_mask_detail(P, masks, pt, SIZE, INTR, cam_pose() if Tc is None else Tc, shift)


# ---------- ① 가운데 · 크기 · yaw ----------

@pytest.mark.parametrize('yaw, dxy, dyaw', [(0.0, (0.0, 0.0), 0.0), (0.0, (0.002, -0.0015), 3.0),
                                            (37.0, (-0.0025, 0.001), -2.0), (90.0, (0.001, 0.002), 1.5)])
def test_center_size_yaw_match_truth(yaw, dxy, dyaw):
    """1. 비킨 · 돈 블록 → 가운데 ±0.5 mm · yaw ±0.5° · 두 변 ±0.5 mm · 윗면 높이 ±1 mm, method mask."""
    pt = part(yaw)
    c, R = placed(pt, dxy, dyaw)
    P, masks = render([(c, R)])
    blk, rej, info = run(P, masks, pt)
    assert rej is None and info['method'] == 'mask', (rej, info)
    assert abs(blk['x_m'] - c[0]) <= 0.0005 and abs(blk['y_m'] - c[1]) <= 0.0005, blk
    assert abs((blk['yaw_deg'] - yaw_of(R) + 90.0) % 180.0 - 90.0) <= 0.5, blk
    assert abs(blk['size_mm'][0] - L * 1000) <= 0.5 and abs(blk['size_mm'][1] - W * 1000) <= 0.5, blk
    assert abs(blk['top_z_m'] - T) <= 0.001
    assert -90.0 <= blk['yaw_deg'] < 90.0 and blk['up'] == 'THICKNESS' and (blk['tilted'], blk['confidence']) == (False, 1.0)
    json.dumps(blk, allow_nan=False)
    json.dumps(info, allow_nan=False)


def test_second_layer_block():
    """1b. 아래층 위에 놓은 블록(가로로 걸침) — 윗면 높이 = 두 층, 크기 · 가운데 맞음."""
    low = part(90.0)
    pt = part(0.0, layer=1)
    c, R = placed(pt, (0.0015, 0.0), 0.0)
    P, masks = render([placed(low), (c, R)])
    blk, rej, info = run(P, masks, pt)
    assert rej is None and info['method'] == 'mask', (rej, info)
    assert abs(blk['x_m'] - c[0]) <= 0.0005 and abs(blk['y_m'] - c[1]) <= 0.0005, blk
    assert abs(blk['top_z_m'] - 2 * T) <= 0.001


# ---------- ② 가장자리 처짐 ----------

@pytest.mark.parametrize('yaw', [0.0, 30.0])
def test_sagging_edges_mask_size_ok_depth_mismatch(yaw):
    """2. 윗면 가장자리 4 mm 가 8 mm 처진 깊이 — measure_placed 는 SIZE_MISMATCH, 마스크 방식은 크기 · 가운데가 맞는다."""
    pt = part(yaw)
    c, R = placed(pt, (0.001, -0.001), 0.0)
    P, masks = render([(c, R)], sag=True)
    _, rej_d = measure_placed(P, pt, SIZE)
    assert rej_d is not None and rej_d[0] == 'SIZE_MISMATCH', rej_d
    blk, rej, info = run(P, masks, pt)
    assert rej is None and info['method'] == 'mask' and info['depth_reject'] == 'SIZE_MISMATCH', (rej, info)
    assert abs(blk['size_mm'][0] - L * 1000) <= 0.5 and abs(blk['size_mm'][1] - W * 1000) <= 0.5, blk
    assert abs(blk['x_m'] - c[0]) <= 0.0005 and abs(blk['y_m'] - c[1]) <= 0.0005, blk
    assert abs(blk['top_z_m'] - T) <= 0.001, blk               # 안쪽 점이라 처진 가장자리에 끌려 내려가지 않는다


# ---------- ③ 픽셀 옮김 ----------

@pytest.mark.parametrize('shift', [(4, 0), (0, -3), (-5, -1), (2.5, 1.5)])
def test_px_shift_moves_center(shift):
    """3. px_shift (du, dv) → 가운데가 du · dv × (카메라 높이 − 윗면) / f 만큼 — 이 카메라는 u → base +x, v → base −y."""
    pt = part(20.0)
    P, masks = render([placed(pt)])
    b0, _, _ = run(P, masks, pt)
    b1, rej, info = run(P, masks, pt, shift)
    du, dv = (int(round(float(v))) for v in shift)                     # 정수 픽셀로 반올림해 마스크 고르기 · 높이 · 광선에 같이 쓴다
    assert rej is None and info['method'] == 'mask' and info['shift_px'] == [du, dv]
    scale = (CAM_H - T) / INTR[0]
    assert abs((b1['x_m'] - b0['x_m']) - du * scale) <= 0.0002, (b0, b1)
    assert abs((b1['y_m'] - b0['y_m']) + dv * scale) <= 0.0002, (b0, b1)
    assert abs(b1['size_mm'][0] - b0['size_mm'][0]) <= 0.3 and abs(b1['yaw_deg'] - b0['yaw_deg']) <= 0.2


# ---------- ④ 마스크가 설계 가운데를 안 덮음 ----------

def test_no_mask_over_center_returns_depth_result():
    """4. 마스크가 다른 블록만 덮거나 · 마스크가 없으면 깊이 결과(measure_placed)를 그대로, method depth + 까닭."""
    pt = part(0.0)
    other = part(0.0, xy=(CX, CY + 0.06))
    P, masks = render([placed(pt), placed(other)])
    want = measure_placed(P, pt, SIZE)
    assert want[1] is None
    for ms in ([masks[1]], [], None, [np.zeros(HW, bool), None]):
        blk, rej, info = run(P, ms, pt)
        assert (blk, rej) == want and info['method'] == 'depth' and info['reason'], info
    assert measure_placed_mask(P, [masks[1]], pt, SIZE, INTR, cam_pose()) == want


def test_far_shift_moves_mask_off_center():
    """4b. 마스크를 블록 폭보다 멀리 옮기면 설계 자리와 40 % 넘게 안 겹쳐 깊이 결과로 돌아간다(옮긴 뒤 마스크로 고른다)."""
    pt = part(0.0)
    P, masks = render([placed(pt)])
    blk, rej, info = run(P, masks, pt, (0, 40))                 # 40 px ≈ 19 mm > 폭 절반 12.4 mm
    assert (blk, rej) == measure_placed(P, pt, SIZE) and info['method'] == 'depth'


# ---------- ⑤ 깊이 거절은 그대로 ----------

def _tilted():
    pt = part(0.0)
    c, R = placed(pt)
    return [(c + [0, 0, 0.004], R @ rot_axis(1, 7.0))], pt


def _covered():
    pt = part(0.0)
    top = part(90.0, layer=1)
    return [placed(pt), placed(top)], pt


def _empty():
    pt = part(0.0)
    return [placed(part(0.0, xy=(CX, CY + 0.08)))], pt


@pytest.mark.parametrize('make, code', [(_tilted, 'TILTED'), (_covered, 'COVERED'), (_empty, 'NO_PART')])
def test_depth_rejects_pass_through(make, code):
    """5. 기울어짐 · 덮임 · 없음은 깊이가 정한다 — 마스크가 있어도 measure_placed 의 거절 그대로, method depth."""
    boxes, pt = make()
    P, masks = render(boxes)
    want = measure_placed(P, pt, SIZE)
    assert want[1] is not None and want[1][0] == code, want
    blk, rej, info = run(P, masks, pt)
    assert (blk, rej) == want and info['method'] == 'depth' and info['depth_reject'] == code


# ---------- 이웃과 붙음 ----------

def test_flush_neighbor_depth_merged_mask_separates():
    """6. 옆에 바짝 붙은 같은 높이 이웃 — 깊이는 NEIGHBOR_MERGED, 블록마다 마스크가 따로면 마스크 방식은 그 블록만 잰다."""
    pt = part(0.0)
    nb = part(0.0, xy=(CX, CY + W))
    P, masks = render([placed(pt), placed(nb)])
    assert measure_placed(P, pt, SIZE)[1][0] == 'NEIGHBOR_MERGED'
    blk, rej, info = run(P, masks, pt)
    assert rej is None and info['method'] == 'mask', (rej, info)
    assert abs(blk['y_m'] - CY) <= 0.0005 and abs(blk['size_mm'][1] - W * 1000) <= 0.5, blk
    blk, rej, info = run(P, [masks[0] | masks[1]], pt)           # YOLO 가 두 블록을 한 마스크로 냈으면 마스크를 안 믿고 깊이 결과(여기선 깊이도 붙음)
    assert (blk, rej) == measure_placed(P, pt, SIZE) and rej[0] == 'NEIGHBOR_MERGED' and info['method'] == 'depth' and '붙음' in info['reason']


def test_partial_mask_falls_back_to_depth():
    """7. 마스크가 블록 절반만 덮으면 마스크를 안 믿고 깊이 결과 그대로 — 깊이가 통과한 블록을 거절로 바꾸지 않고, 잘못된 가운데도 내지 않는다."""
    pt = part(0.0)
    P, masks = render([placed(pt)])
    half = masks[0].copy()
    half[:, :int(INTR[2]) - 20] = False                         # 가운데 픽셀은 남기고 왼쪽을 지움
    want = measure_placed(P, pt, SIZE)
    assert want[1] is None
    blk, rej, info = run(P, [half], pt)
    assert (blk, rej) == want and info['method'] == 'depth' and '일부만' in info['reason'] and info['mask_size_mm'][0] < L * 1000 - 4


def test_picks_mask_overlapping_design_footprint_not_first():
    """7b. 붙은 이웃 마스크가 목록에 먼저 있고 설계 가운데 픽셀까지 살짝 덮어도, 설계 자리와 가장 많이 겹치는 이 블록 마스크를 고른다."""
    pt = part(0.0)
    nb = part(0.0, xy=(CX, CY + W))
    P, masks = render([placed(pt), placed(nb)])
    u0, v0 = (int(round(v)) for v in _project((CX, CY, T), INTR, cam_pose()))
    fat = masks[1].copy()
    fat[v0 - 2:v0 + 3, u0 - 2:u0 + 3] = True                    # 이웃 마스크가 설계 가운데 픽셀을 덮음(겹친 픽셀을 남기는 YOLO 중복)
    blk, rej, info = run(P, [fat, masks[0]], pt)
    assert rej is None and info['method'] == 'mask' and abs(blk['y_m'] - CY) <= 0.0005, (blk, info)


# ---------- ⑥ 꼴 ----------

def test_output_keys_and_shape_same_as_measure_placed():
    """8. 출력 꼴(둘) · 키 · 값 형이 measure_placed 와 같다. NaN 점이 섞여도 된다."""
    pt = part(15.0)
    P, masks = render([placed(pt, (0.001, 0.0))])
    P[::7, ::5] = np.nan                                         # 깊이 구멍
    want = measure_placed(P, pt, SIZE)
    got = measure_placed_mask(P, masks, pt, SIZE, INTR, cam_pose())
    assert len(got) == 2 and got[1] is None and want[1] is None
    assert set(got[0]) == set(want[0]) == KEYS
    assert {k: type(v) for k, v in got[0].items()} == {k: type(v) for k, v in want[0].items()}
    assert got[0]['part_id'] == pt['part_id'] and len(got[0]['size_mm']) == 2


# ---------- 모서리 높이 corners_z_m (10/15 한석형 #190 — 마스크 방식도 낸다) ----------

def test_mask_result_has_corners_at_measured_pose():
    """마스크 방식 답에도 corners_z_m 4개 — 칸은 마스크로 잰 가운데 · yaw 기준이라 8 mm 비켜 놓인 블록도 네 모서리 모두 윗면 높이(null 없음)."""
    pt = part(20.0)
    c = np.array(pt['center']); c[0] += 0.008
    P, masks = render([(c, placed(pt)[1])])
    blk, rej, info = run(P, masks, pt)
    assert rej is None and info['method'] == 'mask', (rej, info)
    cz = blk['corners_z_m']
    assert isinstance(cz, list) and len(cz) == 4 and all(v is not None for v in cz), cz
    assert all(abs(v - (c[2] + T / 2)) < 0.0015 for v in cz), cz
    assert set(blk) == KEYS

"""BlockFinder 시험 (pytest, ROS · 로봇 없음) — W086 흩어진 블록 찾기 ①, SDD §6.11 ②.

① 합성: 공급 관측 자세(observe_supply)의 실측 카메라(내부 파라미터 · T_base2cam = W114-v2 s06_001 자세 json 값)로
   블록 상자를 광선 추적해 깊이 영상 + 블록 번호 영상을 만든다. 정답 마스크(블록 번호 영상)를 넣어 뒷단(blocks_from_masks)만 보고,
   눕힘 1개 · 2개는 예비 마스크(masks_from_edges, cv2 필요)로도 본다.
② 실측(있을 때만): ~/Downloads/W114-v2(또는 환경 변수 D2_W114_DIR)의 흩뿌린 장면 몇 장에 예비 마스크 → 뒷단을 돌려 결과를 찍는다(pytest -s).
   D2_FIND_PREVIEW_DIR 를 주면 윤곽 · 번호 그림을 그 폴더에 저장한다(저장소 안에 두지 말 것). 정답 라벨이 로컬에 없어 판정은 하지 않는다.
CI 에는 cv2 가 없다 — cv2 가 필요한 시험은 importorskip 으로 건너뛴다(뒷단 시험은 numpy 만).
"""
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest
import yaml

from d2_vision.block_finder import BlockFinder, blocks_from_masks, finder_cfg, masks_from_edges, wrap_deg

ROBOT_YAML = Path(__file__).resolve().parents[2] / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml'
CFG = finder_cfg(yaml.safe_load(ROBOT_YAML.read_text(encoding='utf-8')))
SIZE = CFG['size_m']
TABLE = CFG['table_z_m']
# W114-v2 s06_001_a_pose.json (observe_supply 자세, 10/7) — 카메라는 작업대 위 약 496 mm, 화면 긴 쪽 = base x
INTR = (603.2089233398438, 602.9675903320312, 319.44287109375, 246.93531799316406)
T_B2C = np.array([[0.999833, 0.018095, -0.002353, 0.389811],
                  [0.018148, -0.99953, 0.024701, 0.231348],
                  [-0.001905, -0.02474, -0.999692, 0.477916],
                  [0.0, 0.0, 0.0, 1.0]])
H, W = 480, 640
POS_TOL_MM, YAW_TOL_DEG = 3.0, 3.0


# ---------------- 합성 장면 ----------------
def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# 블록 축(열 0 = LENGTH, 1 = WIDTH, 2 = THICKNESS) → base. up 축이 z 로 가게 둔다. 윗면 긴 변이 base x 를 향하게(yaw 0) 맞춘다.
R_UP = {
    'THICKNESS': np.eye(3),
    'WIDTH': np.array([[1.0, 0, 0], [0, 0, -1], [0, 1, 0]]),     # 폭 축이 위 · 길이 축이 x
    'LENGTH': np.array([[0, 1.0, 0], [0, 0, 1], [1, 0, 0]]),      # 길이 축이 위 · 폭 축(윗면 25 mm 변)이 x
}


def block(x_mm, y_mm, yaw_deg, up='THICKNESS', base_mm=0.0):
    """작업면(또는 base_mm 높이 받침) 위에 바로 놓인 블록 상자: (중심 m, 회전 3x3, 반치수 m)."""
    R = rz(math.radians(yaw_deg)) @ R_UP[up]
    hz = SIZE[['LENGTH', 'WIDTH', 'THICKNESS'].index(up)] / 2
    return (np.array([x_mm / 1000, y_mm / 1000, TABLE + base_mm / 1000 + hz]), R, np.array(SIZE) / 2)


def render(boxes):
    """상자들을 실측 카메라로 광선 추적 → (깊이 m (H, W), 블록 번호 (H, W) — 작업면 −1)."""
    v, u = np.mgrid[0:H, 0:W]
    d_cam = np.stack([(u - INTR[2]) / INTR[0], (v - INTR[3]) / INTR[1], np.ones((H, W))], axis=-1)
    R, o = T_B2C[:3, :3], T_B2C[:3, 3]
    d = d_cam @ R.T                                   # 광선 방향(base). 카메라 z 성분이 1 이라 t = 카메라 깊이
    t_best = (TABLE - o[2]) / d[..., 2]
    idx = np.full((H, W), -1)
    for i, (c, Rb, h) in enumerate(boxes):
        o_l = Rb.T @ (o - c)
        d_l = d @ Rb
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (-h - o_l) / d_l, (h - o_l) / d_l
            tmin = np.max(np.minimum(t1, t2), axis=-1)
            tmax = np.min(np.maximum(t1, t2), axis=-1)
            hit = (tmax >= tmin) & (tmin > 0) & (tmin < t_best)
        t_best[hit] = tmin[hit]
        idx[hit] = i
    return t_best, idx


def color_of(idx):
    """합성 컬러: 작업면 밝은 회색, 블록은 나무색(블록마다 밝기 조금 다르게)."""
    img = np.full((H, W, 3), 225, np.uint8)
    for i in range(idx.max() + 1):
        img[idx == i] = (120 + 8 * i, 170 + 5 * i, 205)
    return img


def largest_piece(m):
    """마스크의 가장 큰 4-연결 조각(가려져 두 조각이 된 아래 블록 — E-66 라벨 규칙: 큰 조각 하나만)."""
    lab = np.zeros(m.shape, int)
    n = 0
    for v0, u0 in zip(*np.nonzero(m)):
        if lab[v0, u0]:
            continue
        n += 1
        stack = [(v0, u0)]
        lab[v0, u0] = n
        while stack:
            v, u = stack.pop()
            for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = v + dv, u + du
                if 0 <= a < H and 0 <= b < W and m[a, b] and not lab[a, b]:
                    lab[a, b] = n
                    stack.append((a, b))
    sizes = [(lab == k).sum() for k in range(1, n + 1)]
    return lab == (int(np.argmax(sizes)) + 1)


def run_truth(boxes, masks=None):
    """정답 마스크(블록 번호 영상)로 뒷단만 돌린다. 출력: (블록 목록, info, 마스크 번호 → 블록 dict)."""
    depth, idx = render(boxes)
    if masks is None:
        masks = [idx == i for i in range(len(boxes))]
    info = {}
    out = blocks_from_masks(masks, depth, INTR, T_B2C, CFG, info=info)
    return out, info, dict(zip(info['mask_index'], out))


def check_pose(b, x_mm, y_mm, yaw_deg, up):
    """자리 ±3 mm · yaw ±3°(180° 대칭) · 자세."""
    assert b['up'] == up, b
    assert abs(b['x_m'] * 1000 - x_mm) <= POS_TOL_MM, b
    assert abs(b['y_m'] * 1000 - y_mm) <= POS_TOL_MM, b
    assert abs(wrap_deg(b['yaw_deg'] - yaw_deg)) <= YAW_TOL_DEG, b
    assert -90.0 <= b['yaw_deg'] < 90.0


def check_schema(blocks):
    """IRD find_blocks 칸 그대로 · JSON 직렬화 가능(NaN 없음) · 윗면 높이 순 · 틈 키 = 수평 두 축."""
    json.dumps(blocks, allow_nan=False)
    want = {'x_m', 'y_m', 'top_z_m', 'yaw_deg', 'up', 'clear', 'gap_mm', 'overlap', 'tilted', 'confidence'}
    for b in blocks:
        assert set(b) == want
        axes = {'THICKNESS': {'LENGTH', 'WIDTH'}, 'WIDTH': {'LENGTH', 'THICKNESS'}, 'LENGTH': {'WIDTH', 'THICKNESS'}}[b['up']]
        assert set(b['gap_mm']) == axes and set(b['clear']) == axes
        assert b['overlap'] in ('none', 'top', 'under') and isinstance(b['tilted'], bool)
        assert 0.0 <= b['confidence'] <= 1.0
    tops = [b['top_z_m'] for b in blocks]
    assert tops == sorted(tops, reverse=True)


# ---------------- 합성 — 정답 마스크로 뒷단 ----------------
def test_flat_single_yaw12():
    out, _, _ = run_truth([block(420, 210, 12)])
    check_schema(out)
    b, = out
    check_pose(b, 420, 210, 12, 'THICKNESS')
    assert abs(b['top_z_m'] - (TABLE + SIZE[2])) < 0.002
    assert b['clear'] == {'LENGTH': True, 'WIDTH': True} and b['overlap'] == 'none' and not b['tilted']


def test_edge_single():
    out, _, _ = run_truth([block(300, 260, -30, 'WIDTH')])
    check_schema(out)
    b, = out
    check_pose(b, 300, 260, -30, 'WIDTH')
    assert abs(b['top_z_m'] - (TABLE + SIZE[1])) < 0.002
    assert all(b['clear'].values()) and b['overlap'] == 'none' and not b['tilted']


def test_stand_single_yaw_by_width_edge():
    """세움 — yaw 는 윗면 25 mm 변(WIDTH) 기준(10/8 E-62). 카메라 가운데에서 약 100 mm 떨어져 옆면이 보여도 윗면만 쓴다."""
    out, _, _ = run_truth([block(480, 280, 45, 'LENGTH')])
    check_schema(out)
    b, = out
    check_pose(b, 480, 280, 45, 'LENGTH')
    assert abs(b['top_z_m'] - (TABLE + SIZE[0])) < 0.002
    assert all(b['clear'].values()) and b['overlap'] == 'none' and not b['tilted']


def _pair_8mm():
    """눕힘 두 개를 긴 변끼리 8 mm 틈으로 나란히(yaw 12°)."""
    yaw = math.radians(12)
    off = SIZE[1] * 1000 + 8.0                       # 중심 사이 = 폭 + 8 mm
    vx, vy = -math.sin(yaw) * off, math.cos(yaw) * off
    return [block(390, 200, 12), block(390 + vx, 200 + vy, 12)], (390 + vx, 200 + vy)


def test_two_flat_8mm_gap():
    boxes, (x2, y2) = _pair_8mm()
    out, _, by = run_truth(boxes)
    check_schema(out)
    check_pose(by[0], 390, 200, 12, 'THICKNESS')
    check_pose(by[1], x2, y2, 12, 'THICKNESS')
    for b in out:
        assert abs(b['gap_mm']['WIDTH'] - 8.0) <= 2.0, b      # 2 mm 격자 — ±1.4 mm + 여유
        assert b['clear'] == {'LENGTH': True, 'WIDTH': False}
        assert b['overlap'] == 'none' and not b['tilted']


def test_tilted_on_other_block():
    """B 가 한쪽 끝은 작업면, 다른 쪽은 A 의 끝 모서리에 걸쳐 기울어짐(E-43) → B tilted · top, A under(걸친 블록도 빼야 함)."""
    L, _, T = (s * 1000 for s in SIZE)
    a_x, a_y = 390.0, 230.0
    a_left = a_x - L / 2                             # A 의 왼쪽 끝 (x mm)
    x_low = a_left - 52.0                            # B 아래 모서리가 작업면에 닿는 곳
    th = math.atan2(T, a_left - x_low)               # B 밑면이 A 왼쪽 위 모서리에 닿는 기울기
    # B 중심 = 밑면 가운데 + 두께/2 × 윗면 쪽 법선(x-z 평면, 길이 축이 +x 로 올라감)
    bx = x_low + (L / 2) * math.cos(th) - (T / 2) * math.sin(th)
    bz = (L / 2) * math.sin(th) + (T / 2) * math.cos(th)
    b_y = a_y + 5.0
    B = (np.array([bx / 1000, b_y / 1000, TABLE + bz / 1000]), ry(-th), np.array(SIZE) / 2)
    out, info, by = run_truth([block(a_x, a_y, 0), B])
    check_schema(out)
    b = by[1]
    assert b['tilted'] and b['up'] == 'THICKNESS' and b['overlap'] == 'top', (b, info['detail'])
    top_cx = bx - (T / 2) * math.sin(th)              # 윗면 가운데의 x (기울어진 만큼 안쪽)
    assert abs(b['x_m'] * 1000 - top_cx) <= POS_TOL_MM and abs(b['y_m'] * 1000 - b_y) <= POS_TOL_MM
    assert abs(wrap_deg(b['yaw_deg'])) <= YAW_TOL_DEG
    a = by[0]
    assert a['overlap'] == 'under' and not a['tilted'] and a['up'] == 'THICKNESS', (a, info['detail'])


def test_stacked_cross():
    """포개짐: A 위에 B 가 90° 엇갈려 얹힘 → B up THICKNESS · top · 윗면 = 두께 2개, A under(보이는 큰 조각만 마스크 — E-66)."""
    T = SIZE[2] * 1000
    boxes = [block(330, 290, 0), block(345, 290, 90, base_mm=T)]
    _, idx = render(boxes)
    masks = [largest_piece(idx == 0), idx == 1]
    out, info, by = run_truth(boxes, masks)
    check_schema(out)
    b = by[1]
    check_pose(b, 345, 290, 90, 'THICKNESS')
    assert b['overlap'] == 'top' and not b['tilted']
    assert abs(b['top_z_m'] - (TABLE + 2 * SIZE[2])) < 0.002
    assert out[0] is b                                # 윗면 높이 순 — 위 블록이 먼저
    assert by[0]['overlap'] == 'under', (by[0], info['detail'])


def test_clipped_at_image_border():
    """영상 왼쪽 테두리에 잘린 블록(영역 밖이기도 함) → 자세 · yaw 를 못 믿으니 under + confidence ≤ 0.2 (10/8 PL)."""
    out, info, _ = run_truth([block(140, 230, 0)])
    b, = out
    assert b['overlap'] == 'under' and b['confidence'] <= 0.2, (b, info['detail'])


def test_empty_and_bad_masks():
    """빈 마스크 · 크기 다른 마스크 · 깊이 없는 영상은 빼고 빈 목록(예외 없음)."""
    depth, _ = render([block(420, 210, 0)])
    info = {}
    out = blocks_from_masks([np.zeros((H, W), bool), np.ones((10, 10), bool)], depth, INTR, T_B2C, CFG, info=info)
    assert out == [] and len(info['skipped']) == 2
    assert blocks_from_masks([np.ones((H, W), bool)], np.zeros((H, W)), INTR, T_B2C, CFG) == []


# ---------------- 합성 — 예비 마스크(엣지 + 깊이) ----------------
def test_edges_single_flat():
    pytest.importorskip('cv2')
    depth, idx = render([block(420, 210, 12)])
    out = BlockFinder(CFG).find(color_of(idx), depth, INTR, T_B2C)
    check_schema(out)
    b, = out
    check_pose(b, 420, 210, 12, 'THICKNESS')
    assert all(b['clear'].values())


def test_edges_two_flat_8mm():
    pytest.importorskip('cv2')
    boxes, (x2, y2) = _pair_8mm()
    depth, idx = render(boxes)
    masks = masks_from_edges(color_of(idx), depth, INTR, T_B2C, CFG)
    assert len(masks) == 2
    out = blocks_from_masks(masks, depth, INTR, T_B2C, CFG)
    check_schema(out)
    got = sorted(out, key=lambda b: b['y_m'])
    check_pose(got[0], 390, 200, 12, 'THICKNESS')
    check_pose(got[1], x2, y2, 12, 'THICKNESS')
    for b in got:
        assert b['clear'] == {'LENGTH': True, 'WIDTH': False} and abs(b['gap_mm']['WIDTH'] - 8.0) <= 3.0


# ---------------- 실측 장면 (있을 때만, 판정 없이 출력) ----------------
REAL_DIR = Path(os.environ.get('D2_W114_DIR', Path.home() / 'Downloads' / 'W114-v2'))
REAL_SCENES = ['s07_040_a', 's06_010_a', 's08_040_a', 's08_030_a']   # 드문 · 보통 · 세움 많음 · 빈 장면


def _draw(color, masks, info, blocks, path, cv2):
    """윤곽(판정 색) + 번호 + 자세 · yaw · 틈 글자. 초록 = 집을 수 있음(none · 기울지 않음 · 틈 하나 이상), 주황 = top, 빨강 = under · tilted."""
    img = color.copy()
    for n, (mi, b) in enumerate(zip(info['mask_index'], blocks)):
        m = masks[mi].astype(np.uint8)
        if b['tilted'] or b['overlap'] == 'under':
            col = (0, 0, 255)
        elif b['overlap'] == 'top':
            col = (0, 140, 255)
        else:
            col = (0, 200, 0) if any(b['clear'].values()) else (200, 200, 0)
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cs, -1, col, 2)
        vs, us = np.nonzero(m)
        cv2.putText(img, str(n), (int(us.mean()) - 6, int(vs.mean()) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    for i in info.get('skipped', []):
        m = masks[i[0]].astype(np.uint8) if i[0] < len(masks) else None
        if m is not None:
            cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, cs, -1, (128, 128, 128), 1)
    legend = np.full((24 + 18 * len(blocks), img.shape[1], 3), 255, np.uint8)
    cv2.putText(legend, f'table_z {info["table_z_m"] * 1000:.1f} mm  blocks {len(blocks)}', (4, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    for n, b in enumerate(blocks):
        g = ' '.join(f'{k[0]}{v:.0f}' for k, v in b['gap_mm'].items())
        txt = (f'{n}: {b["up"][:5]} yaw {b["yaw_deg"]:+.0f} top {b["top_z_m"] * 1000:+.0f}mm gap {g} '
               f'{b["overlap"]}{" TILT" if b["tilted"] else ""} c{b["confidence"]:.2f}')
        cv2.putText(legend, txt, (4, 34 + 18 * n), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
    cv2.imwrite(str(path), np.vstack([img, legend]))


@pytest.mark.parametrize('name', REAL_SCENES)
def test_real_scene_preview(name):
    """실측 흩뿌린 장면: 예비 마스크 → 뒷단. 결과를 찍고(pytest -s) D2_FIND_PREVIEW_DIR 가 있으면 그림 저장. 자세 json 의 T_base2cam 을 쓴다."""
    cv2 = pytest.importorskip('cv2')
    stem = REAL_DIR / name
    if not Path(f'{stem}_color.png').is_file():
        pytest.skip(f'실측 데이터 없음: {stem}_color.png')
    pose = json.loads(Path(f'{stem}_pose.json').read_text())
    ii = pose['intrinsics_px']
    intr = (ii['fx'], ii['fy'], ii['ppx'], ii['ppy'])
    T = np.array(pose['T_base2cam_m'])
    color = cv2.imread(f'{stem}_color.png')
    depth = cv2.imread(f'{stem}_depth.png', cv2.IMREAD_UNCHANGED).astype(float) / 1000.0
    finder = BlockFinder(CFG)
    blocks = finder.find(color, depth, intr, T)
    check_schema(blocks)
    info = finder.last_info
    print(f'\n[{name}] 작업면 {info["table_z_m"] * 1000:.1f} mm · 마스크 {len(finder.last_masks)} · 블록 {len(blocks)}'
          f' · 빠짐 {len(info["skipped"])}')
    for n, (mi, b) in enumerate(zip(info['mask_index'], blocks)):
        d = info['detail'][mi]
        print(f'  {n:2d} {b["up"]:9s} yaw {b["yaw_deg"]:+6.1f} ({b["x_m"] * 1000:5.0f},{b["y_m"] * 1000:5.0f}) '
              f'top {b["top_z_m"] * 1000:+5.1f} gap {b["gap_mm"]} {b["overlap"]:5s} tilt {b["tilted"]!s:5s} '
              f'c {b["confidence"]:.2f} | 윗면 {d["long_mm"]:.0f}x{d["short_mm"]:.0f} h {d["h_c"]:.1f} 기울 {d["tilt_mm"]:.1f}'
              f' 면적비 {d["area_ratio"]:.2f}')
    out_dir = os.environ.get('D2_FIND_PREVIEW_DIR')
    if out_dir:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        _draw(color, finder.last_masks, info, blocks, Path(out_dir) / f'{name}_find.png', cv2)


REAL1_DIR = Path(os.environ.get('D2_W114_V1_DIR', Path.home() / 'Downloads' / '데이터셋' / 'W114'))   # 첫 촬영(s01~s05)


def test_real_separated_blocks_have_clear_axis():
    """실측 회귀(있을 때만): s04_002 는 블록 7개가 흩어져 있다(서로 붙은 건 옆으로 8~10 mm 떨어진 한 쌍뿐).
    예비 마스크 → 뒷단에서 덮이지 않고 기울지 않은 블록 7개 중 6개 이상이 한 축 이상 clear 여야 한다.
    (10/8 고치기 전: 내 블록 둘레 깊이 번짐 · 마스크보다 긴 윗면 끝이 장애물로 세져 2개뿐 — 틈 4~7 mm)"""
    cv2 = pytest.importorskip('cv2')
    stem = REAL1_DIR / 's04_002_a'
    if not Path(f'{stem}_color.png').is_file():
        pytest.skip(f'실측 데이터 없음: {stem}_color.png')
    pose = json.loads(Path(f'{stem}_pose.json').read_text())
    ii = pose['intrinsics_px']
    color = cv2.imread(f'{stem}_color.png')
    depth = cv2.imread(f'{stem}_depth.png', cv2.IMREAD_UNCHANGED).astype(float) / 1000.0
    blocks = BlockFinder(CFG).find(color, depth, (ii['fx'], ii['fy'], ii['ppx'], ii['ppy']), np.array(pose['T_base2cam_m']))
    free = [b for b in blocks if b['overlap'] == 'none' and not b['tilted']]
    assert len(free) == 7, blocks
    assert sum(any(b['clear'].values()) for b in free) >= 6, [b['gap_mm'] for b in free]

"""StructureScanner 시험 (pytest, ROS · 로봇 없음) — W115 스캔 추론기 ③ · V-48 일치율.

1. 합성: 기본 설계 4개의 정답 blocks/1(변환기 ② RecipeToBlocks 로 레시피에서)을 실측 크기(74.45 × 24.8 × 14.8 mm) 상자로 놓고
   촬영 자세 3곳(W134 실측 카메라 위치와 같은 곳)에서 깊이 영상을 광선으로 그린다(깊이 잡음 ± 0.5 mm, 1 mm 단위 PNG 와 같음).
   기본 · 구조물 전체가 몇 mm 밀림 · 작업면 높이 1 mm 틀림 세 경우 모두 일치율 ≥ 0.9.
2. 실측(W114 s11 — 사람이 쌓은 001_CHAIR_BENCH, 자세 3곳): 파일이 있을 때만, 통과 기준 없이 숫자만 출력(`pytest -s`).
   폴더는 환경 변수 D2_W114_SCAN_DIR 또는 ~/Downloads/데이터셋/W114_scan.
3. save_cloud: PLY 머리말 · 크기 ≤ 2 MB · 큰 점군이면 복셀을 키움.
d2_task 는 정답을 만들 때만 쓴다(소스 폴더를 import 경로에 넣음 — CI 는 PYTHONPATH=src/d2_vision 로 돈다).
"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from d2_vision.mock_scan import load_recipe_files
from d2_vision.structure_scanner import StructureScanner, match_rate, ori_extents, read_ply_header

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'd2_task'))
from d2_task.recipe_to_blocks import RecipeToBlocks, ori_extents as task_ori_extents   # noqa: E402

CFG = yaml.safe_load((SRC / 'd2_robot/d2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
RECIPES = SRC / 'recipe_manager/recipes'
DESIGNS = ['001_CHAIR_BENCH', '002_CHAIR_BACK', '003_DESK_STAND', '004_DESK_PEDESTAL']
ACTUAL_MM = [v * 1000 for v in CFG['block_actual_m']]
INTR = {'fx': 603.2, 'fy': 603.0, 'ppx': 319.4, 'ppy': 246.9}          # W114 촬영 D435i 컬러 기준 내부값(자세 json 과 같은 크기)
# 촬영 자세 3곳의 카메라 위치(설계 좌표 mm, 조립 원점 기준) · 보는 방향 — s11 자세 json 의 T_base2cam 에서 읽은 값을 반올림
CAMS = {'observe': ((0, -1, 496), (0, 0, -1)),
        'observe_front': ((-228, 1, 472), (0.497, 0.0, -0.868)),
        'observe_side': ((1, -262, 451), (0.0, 0.574, -0.819))}
REAL_DIR = Path(os.environ.get('D2_W114_SCAN_DIR', Path.home() / 'Downloads' / '데이터셋' / 'W114_scan'))
REAL_STEMS = {'observe': 's11_001_a_observe', 'observe_front': 's11_002_a_front', 'observe_side': 's11_003_a_side'}


def truth(design_id):
    """레시피 두 파일(E-69) → 정답 블록 (변환기 ② — blocks/2.0, 자리 · 방향 칸은 blocks/1 과 같다)."""
    return RecipeToBlocks(design_id, 'test', [75, 25, 15]).convert(*load_recipe_files(str(RECIPES), design_id))


def cam_rot(look):
    """보는 방향 → 카메라 회전(열 = 카메라 x · y · z 축, 설계 좌표). y(영상 아래) = 아래쪽에 가깝게."""
    z = np.array(look, float)
    z /= np.linalg.norm(z)
    y = np.array([0, 0, -1.0]) - np.dot([0, 0, -1.0], z) * z
    if np.linalg.norm(y) < 1e-6:
        y = np.array([-1.0, 0, 0])
    y /= np.linalg.norm(y)
    return np.stack([np.cross(y, z), y, z], axis=1)


def render(blocks, pose, offset=(17.0, 27.0), z_off=0.0, noise=0.5, seed=0):
    """정답 블록을 실측 크기 상자로 놓고 자세 pose 의 깊이 영상(mm uint16) · T_base2cam(m)을 만든다.

    offset = 구조물을 설계 원점에서 옮긴 양(mm, 10/7 실기에서 x + 17 · y + 27 mm 치우쳐 놓였다) · z_off = 작업면 높이 오차(mm).
    """
    c, look = CAMS[pose]
    c = np.array(c, float)
    Rd = cam_rot(look)
    v, u = np.mgrid[0:480, 0:640]
    rays = np.stack([(u - INTR['ppx']) / INTR['fx'], (v - INTR['ppy']) / INTR['fy'], np.ones(u.shape)], -1).reshape(-1, 3) @ Rd.T
    with np.errstate(divide='ignore', invalid='ignore'):
        t_table = (z_off - c[2]) / rays[:, 2]
    t_best = np.where(t_table > 0, t_table, np.inf)
    ext = ori_extents(ACTUAL_MM)
    layer = ACTUAL_MM[2] / 15.0                                           # 실측 두께로 쌓인 높이
    for b in blocks:
        ex, ey, ez = ext[b['ori']]
        lo = np.array([b['x'] + offset[0] - ex / 2, b['y'] + offset[1] - ey / 2, b['z'] * layer + z_off])
        hi = lo + np.array([ex, ey, ez])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (lo - c) / rays, (hi - c) / rays
        tn = np.nanmax(np.minimum(t1, t2), 1)
        tf = np.nanmin(np.maximum(t1, t2), 1)
        hit = (tn < tf) & (tn > 0)
        t_best = np.where(hit & (tn < t_best), tn, t_best)
    depth = t_best.reshape(480, 640) + np.random.default_rng(seed).uniform(-noise, noise, (480, 640))
    depth = np.where(np.isfinite(depth) & (depth < 1000), np.round(depth), 0).astype(np.uint16)
    o = CFG['assembly_origin']
    yaw = math.radians(o['yaw_deg'])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Rd
    T[:3, 3] = Rz @ c / 1000.0 + np.array([o['x_m'], o['y_m'], o['z_m']])
    return depth, T


def scan(blocks, poses=tuple(CAMS), **kw):
    sc = StructureScanner(CFG, design_id='scan_test')
    for i, p in enumerate(poses):
        depth, T = render(blocks, p, seed=i, **kw)
        sc.add_capture(p, depth, INTR, T)
    return sc


def test_ori_table_same_as_recipe_to_blocks():
    """방향 코드 표가 변환기 ②(정답을 만드는 쪽)와 같아야 일치율 비교가 맞다."""
    assert ori_extents([75, 25, 15]) == task_ori_extents([75, 25, 15])
    assert ori_extents(ACTUAL_MM) == task_ori_extents(ACTUAL_MM)


@pytest.mark.parametrize('design_id', DESIGNS)
@pytest.mark.parametrize('case', [{}, {'offset': (24.0, 21.0)}, {'z_off': 1.0}], ids=['base', 'shift', 'z1mm'])
def test_synthetic_match_rate(design_id, case):
    """기본 설계 4개 × (기본 · 밀림 · 높이 1 mm 틀림) — 일치율 ≥ 0.9, ok, blocks/1 형식."""
    tb = truth(design_id)
    r = scan(tb['blocks'], **case).infer()
    rate, matched, detail = match_rate(r['blocks'], tb)
    print(f'{design_id} {case} rate {rate:.2f} ({matched}/{len(tb["blocks"])}) conf {r["confidence"].get("explained")}')
    assert r['ok'], r['reason']
    assert rate >= 0.9, detail
    body = r['blocks']
    assert body['schema'] == 'blocks/1' and body['design_id'] == 'scan_test' and body['family']
    orders = [b['order'] for b in body['blocks']]
    assert orders == list(range(1, len(orders) + 1))
    for b in body['blocks']:
        assert b['ori'] in ('x', 'y', 'xe', 'ye', 'zx', 'zy') and isinstance(b['inferred'], bool)
        assert all(type(b[k]) is float for k in ('x', 'y', 'z'))
    json.dumps(r, allow_nan=False)                                          # 다리 · 화면으로 그대로 보낼 수 있는 값


def test_top_view_only_infers_hidden_blocks():
    """위에서 한 장만 찍으면 다리가 안 보인다 → 받침 규칙으로 아래를 채우고 inferred 로 표시한다(SDD ⑥)."""
    tb = truth('001_CHAIR_BENCH')
    r = scan(tb['blocks'], poses=('observe',)).infer()
    blocks = r['blocks']['blocks']
    assert any(b['inferred'] for b in blocks)
    assert r['inferred_count'] == sum(b['inferred'] for b in blocks)
    assert min(b['z'] for b in blocks) == 0.0                              # 바닥까지 반복
    assert not r['ok'] and '추정' in r['reason']                           # 추정이 절반을 넘으면 다시 스캔 안내


def test_nearest_base_family():
    """bases 를 주면 블록 수 · 외곽이 가장 가까운 기본 설계의 family 를 쓴다."""
    tb = truth('003_DESK_STAND')
    bases = []
    for d, fam in zip(DESIGNS, ['chair', 'chair', 'desk', 'desk']):
        b = truth(d)
        b['family'] = fam
        bases.append(b)
    r = scan(tb['blocks']).infer(bases=bases)
    assert r['nearest_base'] == '003_DESK_STAND' and r['blocks']['family'] == 'desk'


def test_no_capture_fails():
    r = StructureScanner(CFG).infer()
    assert not r['ok'] and r['blocks']['blocks'] == []


def test_match_rate_rules():
    """E-45: 같은 격자 자리 + 같은 방향, 일치율 = 맞은 수 ÷ max(정답, 추론), 바닥 외곽 가운데끼리 맞춰 비교."""
    tb = truth('001_CHAIR_BENCH')['blocks']
    assert match_rate(tb, tb)[0] == 1.0
    moved = [dict(b, x=b['x'] + 40, y=b['y'] - 30) for b in tb]              # 통째로 옮겨도 같다
    assert match_rate(moved, tb)[0] == 1.0
    wrong = [dict(b, ori='y') if b['order'] == 10 else b for b in tb]      # 방향 하나 틀림(위층 — 바닥 외곽은 그대로)
    assert match_rate(wrong, tb)[:2] == (10 / 11, 10)
    extra = tb + [dict(tb[0], order=99, z=75.0)]                          # 하나 더 → 분모가 추론 수
    rate, matched, detail = match_rate(extra, tb)
    assert (rate, matched, len(detail['extra'])) == (11 / 12, 11, 1)
    off = [dict(b, x=b['x'] + 13) if b['order'] == 10 else b for b in tb]  # 반 칸(12.5 mm) 넘게 어긋남
    assert match_rate(off, tb)[1] == 10
    assert match_rate([], tb)[0] == 0.0


def test_save_cloud_ply(tmp_path):
    """IRD E-67 cloud_path: base_link · 복셀 3 mm · binary PLY ≤ 2 MB."""
    sc = scan(truth('002_CHAIR_BACK')['blocks'])
    path = tmp_path / 'cloud.ply'
    n = sc.save_cloud(str(path))
    fmt, count, head = read_ply_header(path)
    assert fmt == 'binary_little_endian' and count == n > 1000
    assert path.stat().st_size <= 2_000_000 and path.stat().st_size == head + 12 * n
    assert sc.cloud_voxel_m == pytest.approx(0.003)
    pts = np.frombuffer(path.read_bytes()[head:], '<f4').reshape(-1, 3)
    o = CFG['assembly_origin']
    assert abs(np.median(pts[:, 0]) - o['x_m']) < 0.2 and np.max(pts[:, 2]) < o['z_m'] + 0.3   # base_link m


def test_save_cloud_grows_voxel_when_too_big(tmp_path):
    """점이 많으면 복셀을 1.25 배씩 키워 2 MB 안에 맞춘다."""
    sc = StructureScanner(CFG)
    o = CFG['assembly_origin']
    rng = np.random.default_rng(1)
    p = np.c_[rng.uniform(-0.19, 0.19, 400000) + o['x_m'], rng.uniform(-0.19, 0.19, 400000) + o['y_m'],
              rng.uniform(0.0, 0.28, 400000) + o['z_m']]
    sc.captures.append({'pose_id': 'big', 'points_m': p, 'cam_m': np.zeros(3)})
    path = tmp_path / 'big.ply'
    n = sc.save_cloud(str(path))
    assert path.stat().st_size <= 2_000_000 and sc.cloud_voxel_m > 0.003 and n > 50000


@pytest.mark.skipif(not (REAL_DIR / 's11_001_a_observe_depth.png').is_file(), reason='W114 실측 촬영 파일이 없다(드라이브 1_원본_W114)')
def test_real_s11_bench_report():
    """실측 s11(사람이 쌓은 벤치) — 자세 하나씩 · 셋 합침 일치율을 출력만 한다(통과 기준 없음, V-48 은 실기 날 다시)."""
    cv2 = pytest.importorskip('cv2')
    tb = truth('001_CHAIR_BENCH')
    rows = [[p] for p in REAL_STEMS] + [list(REAL_STEMS)]
    for poses in rows:
        sc = StructureScanner(CFG, design_id='scan_s11')
        for p in poses:
            depth = cv2.imread(str(REAL_DIR / f'{REAL_STEMS[p]}_depth.png'), cv2.IMREAD_UNCHANGED)
            js = json.loads((REAL_DIR / f'{REAL_STEMS[p]}_pose.json').read_text(encoding='utf-8'))
            sc.add_capture(p, depth, js['intrinsics_px'], js['T_base2cam_m'])
        r = sc.infer()
        rate, matched, detail = match_rate(r['blocks'], tb)
        c = r['confidence']
        print(f'\n[s11 {"+".join(poses)}] 일치율 {rate:.2f} ({matched} 맞음 / 정답 {len(tb["blocks"])} · 추론 {len(r["blocks"]["blocks"])}) '
              f'ok={r["ok"]} {r["reason"]} 설명률 {c.get("explained")} 추정 {r["inferred_count"]} '
              f'자세 맞춤 {[pl.get("align_mm") for pl in c.get("planes", [])]}')
        print('   틀림(정답 중 못 맞춤):', [(t['order'], t['x'], t['y'], t['z'], t['ori']) for t in detail['missed']])
        print('   남음(추론 중 짝 없음):', [(b['order'], b['x'], b['y'], b['z'], b['ori'], b['inferred']) for b in detail['extra']])

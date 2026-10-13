# -*- coding: utf-8 -*-
"""W168 스캔 중 실시간 점군 미리보기 계산(LiveCloud, ROS 없음) 시험 — 점 자리 · 자르기 · 복셀 · 크기 · 알림 칸 · 언제 낼지 · 파일 돌려 쓰기.

합성: 카메라가 조립 원점 위 500 mm 에서 똑바로 내려다봄(손 계산이 되게) — 평평한 작업대 + 가운데 30 mm 높은 상자.
실측(있을 때만): 10/7 s11 깊이 + 자세 json 의 T 로 점 수 · 크기 · 시간을 찍는다(pytest -s).
"""
import json
import os
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from d2_vision.live_cloud import LIVE_MAX_BYTES, LiveCloud, live_target
from d2_vision.structure_scanner import read_ply_header

ROOT = Path(__file__).resolve().parents[3]
CFG = yaml.safe_load((ROOT / 'src/d2_robot/d2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
O = CFG['assembly_origin']
R_M = CFG.get('assembly_area_half_m', 0.15) + 0.05          # 자르기 반폭(save_cloud 와 같음)
INTR = (603.2, 603.0, 319.4, 246.9)                        # W114 D435i 컬러 내부값
PREVIEW_KEYS = {'schema', 'stamp', 'run_id', 'pose_id', 'index', 'kind', 'cloud_path', 'points'}   # IRD 6장 scan_preview/1.0


def down_camera(height_m=0.5):
    """조립 원점 바로 위 height_m 에서 똑바로 내려다보는 카메라 → base 4×4(m). 카메라 z(광축) = base −z, 카메라 x = base x, y = base −y."""
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = [O['x_m'], O['y_m'], O['z_m'] + height_m]
    return T


def synthetic_depth(height_m=0.5, box_mm=30.0):
    """평평한 작업대(깊이 = height) + 화면 가운데 80 × 80 px 가 box_mm 높은 상자. 출력 (480, 640) mm float."""
    d = np.full((480, 640), height_m * 1000.0)
    d[200:280, 280:360] = height_m * 1000.0 - box_mm
    return d


def test_points_land_where_hand_calc_says():
    """작업대 점은 base z = 원점 z, 상자 점은 + 30 mm. 화면 가운데 픽셀은 카메라 바로 아래(원점 x · y) — 손 계산과 1 mm 안."""
    live = LiveCloud(CFG)
    pts = live.points(synthetic_depth(), INTR, down_camera())
    z = pts[:, 2] - O['z_m']
    assert np.allclose(np.sort(np.unique(np.round(z, 4))), [0.0, 0.03], atol=1e-3)
    top = pts[z > 0.015]
    cx = O['x_m'] + ((280 + 360) / 2 - 0.5 - INTR[2]) * 0.47 / INTR[0]   # 상자 가운데 픽셀 → base x(깊이 470 mm)
    assert abs(np.mean(top[:, 0]) - cx) < 0.001


def test_crop_keeps_assembly_area_only():
    """카메라는 영역보다 넓게 본다(500 mm 에서 약 ±265 mm) — 자른 뒤에는 원점 ± (반폭 + 5 cm) 안 점만."""
    live = LiveCloud(CFG)
    pts = live.points(synthetic_depth(), INTR, down_camera())
    assert len(pts) > 1000
    assert np.all(np.abs(pts[:, 0] - O['x_m']) < R_M) and np.all(np.abs(pts[:, 1] - O['y_m']) < R_M)


def test_write_voxel_size_and_message(tmp_path):
    """live_1.ply: binary PLY · 복셀 5 mm · ≤ 500 KB, 알림 칸 = scan_preview/1.0 kind live(capture 와 같은 칸)."""
    live = LiveCloud(CFG)
    pts = live.points(synthetic_depth(), INTR, down_camera())
    index, path, n, voxel = live.write(tmp_path / 'R1', 'R1', pts)
    fmt, count, head = read_ply_header(path)
    assert (index, path.name, fmt, count, voxel) == (1, 'live_1.ply', 'binary_little_endian', n, pytest.approx(0.005))
    assert path.stat().st_size == head + 12 * n <= LIVE_MAX_BYTES
    assert b'voxel 0.0050' in path.read_bytes()[:head]
    msg = LiveCloud.message('R1', '', index, path, n, stamp=12.5)
    assert set(msg) == PREVIEW_KEYS and msg['kind'] == 'live' and msg['schema'] == 'scan_preview/1.0'
    assert (msg['run_id'], msg['pose_id'], msg['index'], msg['points'], msg['stamp']) == ('R1', '', 1, n, 12.5)
    json.dumps(msg)


def test_grows_voxel_when_over_limit(tmp_path):
    """크기 상한을 넘으면 복셀을 키워 맞춘다(save_cloud 와 같은 방식)."""
    live = LiveCloud(CFG, max_bytes=8000)
    pts = live.points(synthetic_depth(), INTR, down_camera())
    _, path, _, voxel = live.write(tmp_path, 'R1', pts)
    assert voxel > 0.005 and path.stat().st_size <= 8000


def test_files_rotate_atomically(tmp_path):
    """keep 개 파일을 돌려 쓴다 — 5번 쓰면 live_1 ~ 3 만 남고 5번째는 live_2. 임시 파일(.tmp)은 안 남는다. run 마다 순번은 따로."""
    live = LiveCloud(CFG, keep=3)
    pts = live.points(synthetic_depth(), INTR, down_camera())
    paths = [live.write(tmp_path / 'R1', 'R1', pts)[1].name for _ in range(5)]
    assert paths == ['live_1.ply', 'live_2.ply', 'live_3.ply', 'live_1.ply', 'live_2.ply']
    assert sorted(p.name for p in (tmp_path / 'R1').iterdir()) == ['live_1.ply', 'live_2.ply', 'live_3.ply']
    assert live.write(tmp_path / 'R2', 'R2', pts)[0] == 1 and live.counts['R1'] == 5


@pytest.mark.parametrize('state, only, want', [
    ({'state': 'SCAN_MOVE', 'run_id': 'R7'}, True, (True, 'R7')),
    ({'state': 'SCAN_CAPTURE', 'run_id': 'R7'}, True, (True, 'R7')),
    ({'state': 'PICK_PLACE', 'run_id': 'R7'}, True, (False, 'R7')),
    ({'state': 'IDLE', 'run_id': None}, True, (False, 'live')),
    (None, True, (False, 'live')),                       # state 를 아직 못 받음 — 스캔 아님
    ({'state': 'IDLE', 'run_id': None}, False, (True, 'live')),   # 늘 내기(시험용) — run_id 없으면 'live'
])
def test_only_during_scan(state, only, want):
    """기본(only_during_scan)은 작업 관리자 state 가 SCAN_* 일 때만 낸다 — 평소엔 계산하지 않는다."""
    assert live_target(state, only) == want


def test_step_reasons_and_skip(tmp_path):
    """한 틱: 스캔 아님 → 계산 안 함 · 깊이 없음 · 내부값 없음 · TF(자세) 없음 → 그 장 건너뜀(파일 없음) · 다 있으면 냄."""
    live = LiveCloud(CFG)
    scan = {'state': 'SCAN_MOVE', 'run_id': 'R1'}
    d, T = synthetic_depth(), down_camera()
    assert live.step({'state': 'IDLE'}, d, INTR, T, tmp_path)[2] == 'not_scanning'
    assert live.step(scan, None, INTR, T, tmp_path)[2] == 'no_depth'
    assert live.step(scan, d, None, T, tmp_path)[2] == 'no_intrinsics'
    assert live.step(scan, d, INTR, None, tmp_path)[2] == 'no_pose'
    assert not (tmp_path / 'R1').exists()                                 # 건너뛴 장은 파일을 안 쓴다
    msg, pts, why = live.step(scan, d, INTR, T, tmp_path)
    assert why == '' and msg['index'] == 1 and Path(msg['cloud_path']) == tmp_path / 'R1' / 'live_1.ply' and len(pts) > 1000


def test_real_s11_size_and_time(tmp_path):
    """실측(있을 때만): 10/7 s11 세 자세 깊이 + 자세 json T → 자른 점 · PLY 크기 · 걸린 시간(로봇 PC 는 더 느릴 수 있음)."""
    cv2 = pytest.importorskip('cv2')
    real = Path(os.environ.get('D2_W114_SCAN_DIR', Path.home() / 'Downloads' / '데이터셋' / 'W114_scan'))
    stems = ['s11_001_a_observe', 's11_002_a_front', 's11_003_a_side']
    if not (real / f'{stems[0]}_depth.png').is_file():
        pytest.skip('실측 s11 자료가 없다')
    cfg_old = {**CFG, 'assembly_origin': {**CFG['assembly_origin'], 'z_m': -0.018}}   # 10/7 자료는 옛 작업대 기준(#125 와 같음)
    live = LiveCloud(cfg_old)
    for st in stems:
        depth = cv2.imread(str(real / f'{st}_depth.png'), cv2.IMREAD_UNCHANGED)
        js = json.loads((real / f'{st}_pose.json').read_text(encoding='utf-8'))
        t0 = time.perf_counter()
        msg, pts, why = live.step({'state': 'SCAN_MOVE', 'run_id': 'R1'}, depth, js['intrinsics_px'], np.array(js['T_base2cam_m']), tmp_path)
        ms = (time.perf_counter() - t0) * 1000
        size = Path(msg['cloud_path']).stat().st_size
        print(f'[live s11 {st}] 자른 점 {len(pts)} → PLY {msg["points"]} 점 · {size / 1024:.0f} KB · {ms:.0f} ms')
        assert why == '' and size <= LIVE_MAX_BYTES and msg['points'] > 500

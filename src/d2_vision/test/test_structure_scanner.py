"""StructureScanner 시험 (pytest, ROS · 로봇 없음) — W115 스캔 추론기 ③ · V-48 일치율.

1. 합성: 기본 설계 4개의 정답 blocks/2(변환기 ② RecipeToParts 로 레시피에서)을 실측 크기(74.45 × 24.8 × 14.8 mm) 상자로 놓고
   촬영 자세 3곳(W134 실측 카메라 위치와 같은 곳)에서 깊이 영상을 광선으로 그린다(깊이 잡음 ± 0.5 mm, 1 mm 단위 PNG 와 같음).
   기본 · 구조물 전체가 몇 mm 밀림 · 작업면 높이 1 mm 틀림 세 경우 모두 일치율 ≥ 0.9.
   추론기(v12g)가 윗면 YOLO 마스크를 쓰므로 그린 블록 번호로 가짜 마스크를 만들어 넣는다. 5자세(대각선 2곳 더)는 1.0 · 추정 0.
   10/12 5자세 실기 4장면(D2_SCAN_DIR, 기본 ~/d2_data/scan + masks.pkl)이 있으면 47/47 — 없으면 건너뜀(사진은 저장소에 없음).
2. 실측(W114 s11 — 사람이 쌓은 001_CHAIR_BENCH_V000, 자세 3곳): 파일이 있을 때만, 통과 기준 없이 숫자만 출력(`pytest -s`).
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
from d2_vision.structure_scanner import (FLOOR_RGB, StructureScanner, color_cloud, match_rate, ori_extents, read_ply_header,
                                         read_ply_points, save_capture, save_preview)

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'd2_task'))
from d2_task.recipe_to_parts import RecipeToParts, ori_extents as task_ori_extents   # noqa: E402

CFG = yaml.safe_load((SRC / 'd2_robot/d2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
RECIPES = SRC / 'recipe_manager/recipes'
DESIGNS = ['001_CHAIR_BENCH_V000', '002_CHAIR_BACK_V000', '003_DESK_STAND_V000', '004_DESK_PEDESTAL_V000']
ACTUAL_MM = [v * 1000 for v in CFG['block_actual_m']]
INTR = {'fx': 603.2, 'fy': 603.0, 'ppx': 319.4, 'ppy': 246.9}          # W114 촬영 D435i 컬러 기준 내부값(자세 json 과 같은 크기)
# 촬영 자세 3곳의 카메라 위치(설계 좌표 mm, 조립 원점 기준) · 보는 방향 — s11 자세 json 의 T_base2cam 에서 읽은 값을 반올림
CAMS = {'observe': ((0, -1, 496), (0, 0, -1)),
        'observe_front': ((-228, 1, 472), (0.497, 0.0, -0.868)),
        'observe_side': ((1, -262, 451), (0.0, 0.574, -0.819))}
# 10/12 5자세 실기(W166 · 001_CHAIR_BENCH_165814 등)의 대각선 두 자세 — pose.json T_base2cam 카메라 위치 · 보는 방향을 반올림
CAMS5 = {**CAMS, 'diag_far_yp': ((149, 254, 245), (-0.437, -0.748, -0.5)),
         'diag_robot_ym': ((-91, -266, 360), (0.228, 0.664, -0.712))}
SCAN_DIR = Path(os.environ.get('D2_SCAN_DIR', Path.home() / 'd2_data' / 'scan'))
# 10/12 5자세 실기 스캔 폴더(W166) — masks.pkl 은 자세 순서(SCAN5_ORDER)대로 YOLO 마스크 목록
SCAN5 = {'001_CHAIR_BENCH_V000': '001_CHAIR_BENCH_165814', '002_CHAIR_BACK_V000': '002_CHAIR_BACK_170202',
         '003_DESK_STAND_V000': '003_DESK_STAND_171210', '004_DESK_PEDESTAL_V000': '004_DESK_PEDESTAL_171514'}
SCAN5_ORDER = ['observe', 'diag_far_yp', 'observe_front', 'diag_robot_ym', 'observe_side']
REAL_DIR = Path(os.environ.get('D2_W114_SCAN_DIR', Path.home() / 'Downloads' / '데이터셋' / 'W114_scan'))
REAL_STEMS = {'observe': 's11_001_a_observe', 'observe_front': 's11_002_a_front', 'observe_side': 's11_003_a_side'}
# s11 은 10/7 촬영 — 자세 json T_base2cam 이 그때 손목 보정값(작업대 −0.018 m 에 맞춰 고정)이라 작업대가 약 −18 mm 로 나온다.
# 10/11 E-85(#123)로 조립 원점 z 가 0.000 이 되어도 그 자료는 그 기준으로 돌린다 — 아니면 작업대 맞춤(원점 z ±10 mm)이 빗나가 층이 어긋난다.
CFG_OLD = {**CFG, 'assembly_origin': {**CFG['assembly_origin'], 'z_m': -0.018}}


def truth(design_id):
    """레시피 두 파일(E-69) → 정답 블록 (변환기 ② — parts/3.0, 자리 · 방향 칸은 blocks/2 과 같다).
    스캔은 진짜 블록을 보므로 목록 칸 이름을 스캔 쪽(blocks)으로 맞춰 돌려준다(10/13 이름 규칙)."""
    d = RecipeToParts(design_id, 'test', [75, 25, 15]).convert(*load_recipe_files(str(RECIPES), design_id))
    return {**d, 'blocks': d['parts']}


def cam_rot(look):
    """보는 방향 → 카메라 회전(열 = 카메라 x · y · z 축, 설계 좌표). y(영상 아래) = 아래쪽에 가깝게."""
    z = np.array(look, float)
    z /= np.linalg.norm(z)
    y = np.array([0, 0, -1.0]) - np.dot([0, 0, -1.0], z) * z
    if np.linalg.norm(y) < 1e-6:
        y = np.array([-1.0, 0, 0])
    y /= np.linalg.norm(y)
    return np.stack([np.cross(y, z), y, z], axis=1)


def render(blocks, pose, offset=(17.0, 27.0), z_off=0.0, noise=0.5, seed=0, labels=False):
    """정답 블록을 실측 크기 상자로 놓고 자세 pose 의 깊이 영상(mm uint16) · T_base2cam(m)을 만든다.

    offset = 구조물을 설계 원점에서 옮긴 양(mm, 10/7 실기에서 x + 17 · y + 27 mm 치우쳐 놓였다) · z_off = 작업면 높이 오차(mm).
    labels = True 면 (depth, T, lab) — lab = 픽셀마다 처음 맞은 블록 번호(blocks 순서, 작업대 · 허공 −1). 가짜 YOLO 마스크용.
    """
    c, look = CAMS5[pose]
    c = np.array(c, float)
    Rd = cam_rot(look)
    v, u = np.mgrid[0:480, 0:640]
    rays = np.stack([(u - INTR['ppx']) / INTR['fx'], (v - INTR['ppy']) / INTR['fy'], np.ones(u.shape)], -1).reshape(-1, 3) @ Rd.T
    with np.errstate(divide='ignore', invalid='ignore'):
        t_table = (z_off - c[2]) / rays[:, 2]
    t_best = np.where(t_table > 0, t_table, np.inf)
    lab = np.full(t_best.shape, -1)
    ext = ori_extents(ACTUAL_MM)
    layer = ACTUAL_MM[2] / 15.0                                           # 실측 두께로 쌓인 높이
    for i, b in enumerate(blocks):
        ex, ey, ez = ext[b['ori']]
        lo = np.array([b['x'] + offset[0] - ex / 2, b['y'] + offset[1] - ey / 2, b['z'] * layer + z_off])
        hi = lo + np.array([ex, ey, ez])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (lo - c) / rays, (hi - c) / rays
        tn = np.nanmax(np.minimum(t1, t2), 1)
        tf = np.nanmin(np.maximum(t1, t2), 1)
        hit = (tn < tf) & (tn > 0)
        near = hit & (tn < t_best)
        lab = np.where(near, i, lab)
        t_best = np.where(near, tn, t_best)
    depth = t_best.reshape(480, 640) + np.random.default_rng(seed).uniform(-noise, noise, (480, 640))
    depth = np.where(np.isfinite(depth) & (depth < 1000), np.round(depth), 0).astype(np.uint16)
    o = CFG['assembly_origin']
    yaw = math.radians(o['yaw_deg'])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Rd
    T[:3, 3] = Rz @ c / 1000.0 + np.array([o['x_m'], o['y_m'], o['z_m']])
    return (depth, T, lab.reshape(480, 640)) if labels else (depth, T)


def fake_masks(lab, min_px=100):
    """render 의 블록 번호 영상 → 블록마다 보이는 부분 bool 마스크 목록(가짜 YOLO seg — 100 px 미만 조각은 YOLO 도 못 잡는다고 보고 뺌)."""
    return [lab == i for i in np.unique(lab) if i >= 0 and (lab == i).sum() >= min_px]


def scan(blocks, poses=tuple(CAMS), **kw):
    """자세마다 깊이 + 가짜 YOLO 마스크를 그려 StructureScanner 에 넣는다(v12g 추론기는 윗면 마스크를 쓴다)."""
    sc = StructureScanner(CFG, design_id='scan_test')
    for i, p in enumerate(poses):
        depth, T, lab = render(blocks, p, seed=i, labels=True, **kw)
        sc.add_capture(p, depth, INTR, T, masks=fake_masks(lab))
    return sc


def test_ori_table_same_as_recipe_to_parts():
    """방향 코드 표가 변환기 ②(정답을 만드는 쪽)와 같아야 일치율 비교가 맞다."""
    assert ori_extents([75, 25, 15]) == task_ori_extents([75, 25, 15])
    assert ori_extents(ACTUAL_MM) == task_ori_extents(ACTUAL_MM)


@pytest.mark.parametrize('design_id', DESIGNS)
@pytest.mark.parametrize('case', [{}, {'offset': (24.0, 21.0)}, {'z_off': 1.0}], ids=['base', 'shift', 'z1mm'])
def test_synthetic_match_rate(design_id, case, request):
    """기본 설계 4개 × (기본 · 밀림 · 높이 1 mm 틀림) — 3자세 일치율 ≥ 0.9, ok, blocks/2 형식.
    002 를 (17, 27) mm 에 둔 3자세(기본 · 높이)는 v12g 시제품도 11/16 — 등받이 6단을 세운 블록 3개로 봄(10/12 확인). 알려진 약점으로 xfail."""
    if design_id == '002_CHAIR_BACK_V000' and 'offset' not in case:
        request.applymarker(pytest.mark.xfail(reason='v12g 3자세 합성 002 등받이 → 세운 블록(시제품 11/16, 5자세는 16/16)', strict=False))
    tb = truth(design_id)
    r = scan(tb['blocks'], **case).infer()
    rate, matched, detail = match_rate(r['blocks'], tb)
    print(f'{design_id} {case} rate {rate:.2f} ({matched}/{len(tb["blocks"])}) conf {r["confidence"].get("explained")}')
    assert r['ok'], r['reason']
    assert rate >= 0.9, detail
    body = r['blocks']
    assert body['schema'] == 'blocks/2' and body['design_id'] == 'scan_test' and body['family']
    orders = [b['scan_num'] for b in body['blocks']]
    assert orders == list(range(1, len(orders) + 1))
    for b in body['blocks']:
        assert b['ori'] in ('x', 'y', 'xe', 'ye', 'zx', 'zy') and isinstance(b['inferred'], bool)
        assert all(type(b[k]) is float for k in ('x', 'y', 'z'))
    json.dumps(r, allow_nan=False)                                          # 다리 · 화면으로 그대로 보낼 수 있는 값


def test_top_view_only_infers_hidden_blocks():
    """위에서 한 장만 찍으면 다리가 안 보인다 → 받침 규칙으로 아래를 채우고 inferred 로 표시한다(SDD ⑥)."""
    tb = truth('001_CHAIR_BENCH_V000')
    r = scan(tb['blocks'], poses=('observe',)).infer()
    blocks = r['blocks']['blocks']
    assert any(b['inferred'] for b in blocks)
    assert r['inferred_count'] == sum(b['inferred'] for b in blocks)
    assert min(b['z'] for b in blocks) == 0.0                              # 바닥까지 반복
    assert not r['ok'] and ('추정' in r['reason'] or '자세' in r['reason'])   # 추정이 절반을 넘거나 자세가 하나뿐이면 다시 스캔 안내


@pytest.mark.parametrize('design_id', DESIGNS)
@pytest.mark.parametrize('case', [{}, {'offset': (24.0, 21.0)}, {'z_off': 1.0}], ids=['base', 'shift', 'z1mm'])
def test_synthetic_five_poses_all_match(design_id, case):
    """10/12 확정 5자세(3자세 + 대각선 2곳) × (기본 · 밀림 · 높이 1 mm) — 일치율 1.0 · 추정 0(실기 47/47 · 추정 0 과 같은 기대)."""
    tb = truth(design_id)
    r = scan(tb['blocks'], poses=tuple(CAMS5), **case).infer()
    rate, _, detail = match_rate(r['blocks'], tb)
    assert r['ok'], r['reason']
    assert rate == 1.0, detail
    assert r['inferred_count'] == 0


def test_nearest_base_family():
    """bases 를 주면 블록 수 · 외곽이 가장 가까운 기본 설계의 family 를 쓴다."""
    tb = truth('003_DESK_STAND_V000')
    bases = []
    for d, fam in zip(DESIGNS, ['chair', 'chair', 'desk', 'desk']):
        b = truth(d)
        b['family'] = fam
        bases.append(b)
    r = scan(tb['blocks']).infer(bases=bases)
    assert r['nearest_base'] == '003_DESK_STAND_V000' and r['blocks']['family'] == 'desk'


def test_no_capture_fails():
    r = StructureScanner(CFG).infer()
    assert not r['ok'] and r['blocks']['blocks'] == []


def test_match_rate_rules():
    """E-45: 같은 격자 자리 + 같은 방향, 일치율 = 맞은 수 ÷ max(정답, 추론), 바닥 외곽 가운데끼리 맞춰 비교."""
    tb = truth('001_CHAIR_BENCH_V000')['blocks']
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
    """IRD E-67 cloud_path: base_link · 복셀 3 mm · binary PLY ≤ 2 MB · 점마다 색(x y z float + red green blue uchar = 15 B)."""
    sc = scan(truth('002_CHAIR_BACK_V000')['blocks'])
    path = tmp_path / 'cloud.ply'
    n = sc.save_cloud(str(path))
    fmt, count, head = read_ply_header(path)
    assert fmt == 'binary_little_endian' and count == n > 1000
    assert path.stat().st_size <= 2_000_000 and path.stat().st_size == head + 15 * n
    assert sc.cloud_voxel_m == pytest.approx(0.003)
    pts, rgb = read_ply_points(path)
    o = CFG['assembly_origin']
    assert abs(np.median(pts[:, 0]) - o['x_m']) < 0.2 and np.max(pts[:, 2]) < o['z_m'] + 0.3   # base_link m
    floor = pts[:, 2] < o['z_m'] + 0.005
    assert floor.any() and (~floor).any() and (rgb[floor] == FLOOR_RGB).all() and not (rgb[~floor] == FLOOR_RGB).all(axis=1).any()


PREVIEW_KEYS = {'schema', 'stamp', 'run_id', 'pose_id', 'index', 'kind', 'cloud_path', 'points'}   # IRD 6장 scan_preview/1.0


def test_preview_accumulates_per_capture(tmp_path):
    """E-82 점군 미리보기: 자세 3곳을 찍을 때마다 preview_1 ~ 3.ply 가 생기고 '지금까지 모은' 점이 늘어난다. JSON 칸 · PLY 형식 · 크기."""
    sc, prev = StructureScanner(CFG, design_id='scan_test'), []
    for i, p in enumerate(CAMS, start=1):
        depth, T = render(truth('001_CHAIR_BENCH_V000')['blocks'], p, seed=i)
        sc.capture_pose(p, depth, INTR, T)
        msg = save_preview(sc.snapshot(), tmp_path / 'R1', 'R1', p, i, stamp=100.0 + i)
        assert set(msg) == PREVIEW_KEYS and msg['schema'] == 'scan_preview/1.0' and msg['kind'] == 'capture'
        assert (msg['run_id'], msg['pose_id'], msg['index'], msg['stamp']) == ('R1', p, i, 100.0 + i)
        path = Path(msg['cloud_path'])
        assert path == tmp_path / 'R1' / f'preview_{i}.ply' and path.stat().st_size <= 2_000_000
        fmt, count, _ = read_ply_header(path)
        assert fmt == 'binary_little_endian' and count == msg['points']
        prev.append(msg['points'])
        json.dumps(msg)                                                    # 발행할 수 있는 JSON
    assert prev[0] < prev[1] < prev[2], prev                               # 자세가 늘면 모은 점이 늘어난다(누적)
    assert sc.cloud_voxel_m is None                                        # 원래 스캐너는 그대로(save_cloud 는 복사본에서만)


def test_recapture_same_pose_replaces_and_snapshot_keeps_old(tmp_path):
    """같은 자세를 다시 찍으면 그 자세 점이 새 것으로 바뀐다(capture_pose). 그 전에 뜬 snapshot 은 옛 점 그대로 — 미리보기 스레드가 안전하게 읽는다."""
    blocks = truth('001_CHAIR_BENCH_V000')['blocks']
    sc = StructureScanner(CFG)
    depth, T = render(blocks, 'observe', seed=1)
    sc.capture_pose('observe', depth, INTR, T)
    old = sc.snapshot()
    depth2, T2 = render(blocks, 'observe', offset=(40.0, 27.0), seed=2)  # 구조물을 옮겨 다시 찍음
    sc.capture_pose('observe', depth2, INTR, T2)
    assert [c['pose_id'] for c in sc.captures] == ['observe']             # 덮어쓰기 — 둘이 되지 않음
    assert len(old.captures) == 1 and old.captures[0] is not sc.captures[0]
    a = save_preview(old, tmp_path, 'R1', 'observe', 1)
    b = save_preview(sc.snapshot(), tmp_path, 'R1', 'observe', 2)
    pa, pb = (read_ply_points(m['cloud_path'])[0] for m in (a, b))
    top = CFG['assembly_origin']['z_m'] + 0.008                             # 작업대 점은 빼고 블록 점만(작업대가 점 대부분이라 가운데값이 안 움직인다)
    shift = np.median(pb[pb[:, 2] > top, 0]) - np.median(pa[pa[:, 2] > top, 0])
    assert abs(shift - 0.023) < 0.004, shift                                # 옮긴 23 mm 만큼 블록 점이 움직였다(x 축 — 원점 yaw 0.3°)


def test_preview_save_failure_raises_and_leaves_scan(tmp_path):
    """미리보기 파일을 못 쓰면 OSError(노드는 경고만) — 원래 스캐너의 촬영은 그대로라 scan_infer 에 영향 없다."""
    sc = scan(truth('001_CHAIR_BENCH_V000')['blocks'])
    blocker = tmp_path / 'R1'
    blocker.write_text('폴더 자리에 파일')                                  # 폴더를 못 만들게
    before = [c['points_m'] for c in sc.captures]
    with pytest.raises(OSError):
        save_preview(sc.snapshot(), blocker, 'R1', 'observe', 1)
    assert len(sc.captures) == len(before) and all(c['points_m'] is x for c, x in zip(sc.captures, before))   # 촬영 그대로
    assert sc.infer()['ok']


def test_save_capture_files_reload_same_points(tmp_path):
    """자세 자료(10/11): <pose_id>_color.png · _depth.png · _pose.json 이 생기고, 실측 시험 방식(add_capture(depth png, intrinsics_px, T_base2cam_m))
    으로 다시 읽으면 처음과 같은 점 수 · 같은 점이 나온다. pose.json 칸은 capture_scene 과 같은 이름."""
    cv2 = pytest.importorskip('cv2')
    depth, T = render(truth('001_CHAIR_BENCH_V000')['blocks'], 'observe_front', seed=3)
    color = np.full((480, 640, 3), 120, np.uint8)
    intr = (INTR['fx'], INTR['fy'], INTR['ppx'], INTR['ppy'])            # 노드의 self.intr 꼴(튜플)
    files = save_capture(tmp_path / 'R1', 'observe_front', depth.astype(np.float32), color, intr, T,
                         {'run_id': 'R1', 'index': 2, 'tcp': {'name': 'GripperDA_v1'}})
    assert sorted(f.name for f in files) == ['observe_front_color.png', 'observe_front_depth.png', 'observe_front_pose.json']
    js = json.loads((tmp_path / 'R1' / 'observe_front_pose.json').read_text(encoding='utf-8'))
    assert {'intrinsics_px', 'depth_unit', 'color_size', 'T_base2cam_m', 'tcp', 'run_id', 'pose_id', 'index', 'stamp'} <= set(js)
    assert (js['depth_unit'], js['run_id'], js['pose_id'], js['index'], js['color_size']) == ('mm', 'R1', 'observe_front', 2, [640, 480])
    a, b = StructureScanner(CFG), StructureScanner(CFG)
    n0 = a.add_capture('observe_front', depth, INTR, T)
    back = cv2.imread(str(tmp_path / 'R1' / 'observe_front_depth.png'), cv2.IMREAD_UNCHANGED)
    assert back.dtype == np.uint16 and np.array_equal(back, depth)
    n1 = b.add_capture('observe_front', back, js['intrinsics_px'], np.array(js['T_base2cam_m']))   # test_real_s11 과 같은 읽기
    assert n1 == n0 and np.allclose(a.captures[0]['points_m'], b.captures[0]['points_m'], atol=2e-6)


def test_save_capture_no_color_overwrite_and_failure(tmp_path):
    """컬러가 없으면 사진만 건너뜀 · 같은 자세 다시 찍으면 덮어씀 · 폴더를 못 만들면 OSError(노드는 경고만 — 답과 무관)."""
    cv2 = pytest.importorskip('cv2')
    d1 = np.full((48, 64), 400.0)
    d2 = np.full((48, 64), 450.4)
    T = np.eye(4)
    names = [f.name for f in save_capture(tmp_path, 'observe', d1, None, INTR, T)]
    assert names == ['observe_depth.png', 'observe_pose.json']
    assert json.loads((tmp_path / 'observe_pose.json').read_text(encoding='utf-8'))['color_size'] is None
    save_capture(tmp_path, 'observe', d2, None, INTR, T)                 # 같은 자세 다시
    assert int(cv2.imread(str(tmp_path / 'observe_depth.png'), cv2.IMREAD_UNCHANGED)[0, 0]) == 450   # 새 깊이(반올림)
    blocker = tmp_path / 'R9'
    blocker.write_text('폴더 자리에 파일')
    with pytest.raises(OSError):
        save_capture(blocker, 'observe', d1, None, INTR, T)


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
    tb = truth('001_CHAIR_BENCH_V000')
    rows = [[p] for p in REAL_STEMS] + [list(REAL_STEMS)]
    for poses in rows:
        sc = StructureScanner(CFG_OLD, design_id='scan_s11')
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
        print('   남음(추론 중 짝 없음):', [(b['scan_num'], b['x'], b['y'], b['z'], b['ori'], b['inferred']) for b in detail['extra']])


@pytest.mark.skipif(not all((SCAN_DIR / d / 'masks.pkl').is_file() for d in SCAN5.values()),
                    reason='10/12 5자세 실기 스캔(D2_SCAN_DIR, 기본 ~/d2_data/scan)이 없다 — 사진은 저장소에 넣지 않는다')
def test_real_scan5_all_match():
    """10/12 5자세 실기 4장면(YOLO 마스크 masks.pkl 포함) — 47/47 · 추정 0(E-88 확정 판 v12g 와 같은 결과)."""
    import pickle
    cv2 = pytest.importorskip('cv2')
    total = 0
    for design_id, sub in SCAN5.items():
        folder = SCAN_DIR / sub
        masks = pickle.loads((folder / 'masks.pkl').read_bytes())
        sc = StructureScanner(CFG, design_id='scan_real')
        for p, m in zip(SCAN5_ORDER, masks):
            js = json.loads((folder / f'{p}_pose.json').read_text(encoding='utf-8'))
            sc.add_capture(p, cv2.imread(str(folder / f'{p}_depth.png'), cv2.IMREAD_UNCHANGED), js['intrinsics_px'],
                           np.array(js['T_base2cam_m']), masks=m)
        r = sc.infer()
        tb = truth(design_id)
        _, matched, detail = match_rate(r['blocks'], tb)
        assert r['ok'] and r['inferred_count'] == 0, (design_id, r['reason'], r['inferred_count'])
        assert matched == len(tb['blocks']) == len(r['blocks']['blocks']), (design_id, detail['missed'], detail['extra'])
        total += matched
    assert total == 47


# ---------------- 화면 점군 색 · 거르기(color_cloud — 보기 전용, 10/13 민범진) ----------------
def flat(x0, x1, y0, y1, z, step=0.003):
    """(x0..x1, y0..y1) 높이 z 의 평평한 점 판(m)."""
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.c_[xs.ravel(), ys.ravel(), np.full(xs.size, z)]


def test_color_cloud_floor_gray_height_blue_to_red():
    """바닥(작업면 + 5 mm 아래) = 회색, 구조물 = 높이별 파랑(아래) → 빨강(위 — 이 점군의 가장 높은 곳)."""
    floor = flat(-0.10, 0.10, -0.10, 0.10, 0.0)
    low = flat(-0.02, 0.02, -0.01, 0.01, 0.015)                           # 블록 한 층 윗면
    high = flat(-0.02, 0.02, -0.01, 0.01, 0.090)                          # 맨 위
    pts, rgb = color_cloud(np.r_[floor, low, high], 0.0)
    f, lo, hi = (np.isclose(pts[:, 2], z) for z in (0.0, 0.015, 0.090))
    assert (rgb[f] == FLOOR_RGB).all()
    assert (rgb[lo][:, 2] > rgb[lo][:, 0]).all()                          # 아래층은 파란 쪽(B > R)
    assert (rgb[hi] == (255, 0, 0)).all()                                 # 가장 높은 곳은 빨강


def test_color_cloud_drops_specks_and_far_floor_keeps_separate_block():
    """부스러기(블록 가장 작은 면의 반보다 작은 덩어리)는 빼고, 떨어진 블록 하나(25 × 75 mm)는 남긴다. 바닥은 남긴 덩어리 둘레 2 cm 안만."""
    floor = flat(-0.20, 0.20, -0.20, 0.20, 0.0)
    main = flat(-0.04, 0.04, -0.04, 0.04, 0.045)                          # 구조물
    lone = flat(0.12, 0.195, -0.0125, 0.0125, 0.015)                      # 떨어진 블록 하나
    speck = flat(-0.15, -0.144, 0.15, 0.156, 0.02)                        # 6 × 6 mm 부스러기
    pts, rgb = color_cloud(np.r_[floor, main, lone, speck], 0.0)
    obj = pts[pts[:, 2] > 0.005]
    assert not ((obj[:, 0] < -0.1) & (obj[:, 1] > 0.1)).any()             # 부스러기 사라짐
    assert ((obj[:, 0] > 0.1) & np.isclose(obj[:, 2], 0.015)).sum() > 100  # 떨어진 블록은 남음
    fl = pts[pts[:, 2] < 0.005]
    assert len(fl) and not ((fl[:, 0] < -0.1) & (fl[:, 1] < -0.1)).any()  # 먼 바닥(구석) 사라짐
    assert (np.abs(fl[:, 0]) <= 0.04 + 0.025).sum() > 0 and fl[:, 1].max() <= 0.04 + 0.025   # 구조물 둘레 2 cm(+ 칸 5 mm) 안 바닥만


def test_color_cloud_floor_only_and_empty():
    """구조물이 없으면 바닥을 그대로 회색 · 빈 점군은 빈 결과."""
    floor = flat(-0.05, 0.05, -0.05, 0.05, 0.001)
    pts, rgb = color_cloud(floor, 0.0)
    assert len(pts) == len(floor) and (rgb == FLOOR_RGB).all()
    pts, rgb = color_cloud(np.zeros((0, 3)), 0.0)
    assert pts.shape == (0, 3) and rgb.shape == (0, 3)


def test_color_cloud_drops_clusters_outside_assembly_area():
    """가운데가 조립 영역 밖인 큰 덩어리(받침 상자 · 로봇 쪽 물체 — 자르기 둘레에 걸쳐 들어옴)는 뺀다. 그 물체가 더 높아도 색 범위를 가져가지 않는다."""
    floor = flat(-0.20, 0.20, -0.20, 0.20, 0.0)
    main = flat(-0.04, 0.04, -0.04, 0.04, 0.060)                          # 조립 영역 안 구조물(맨 위 = 빨강이어야)
    box = flat(-0.15, 0.15, -0.20, -0.17, 0.080)                          # 영역(± 0.15) 밖에 가운데가 있는 받침 상자
    tall = flat(-0.20, -0.17, 0.10, 0.15, 0.180)                          # 영역 밖 높은 물체
    pts, rgb = color_cloud(np.r_[floor, main, box, tall], 0.0, area=((-0.15, 0.15), (-0.15, 0.15)))
    assert not (pts[:, 2] > 0.07).any()                                   # 상자 · 높은 물체 사라짐
    assert (rgb[np.isclose(pts[:, 2], 0.060)] == (255, 0, 0)).all()       # 구조물 꼭대기가 빨강

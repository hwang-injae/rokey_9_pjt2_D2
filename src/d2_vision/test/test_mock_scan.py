"""MockScan 시험 (pytest, ROS · 로봇 없음) — 가짜 scan_capture · scan_infer · find_blocks 응답이 작업 관리자 검사를 통과하는 모양인지.

작업 관리자(task_manager._scan_capture · _scan_infer)가 응답에서 확인하는 칸을 여기서 그대로 확인한다(d2_task 를 import 하지 않고).
변환기 ②(d2_task RecipeToBlocks)와 같은 blocks 를 내는지는 d2_task 를 읽을 수 있을 때만 비교한다(없으면 건너뜀).
레시피는 저장소 src/recipe_manager/recipes 의 기본 설계 두 파일(E-69 _recipe.json + _placements.csv)을 읽는다. 파일은 pytest 임시 폴더에 쓴다.
"""
import json
import struct
import sys
import zlib
from pathlib import Path

import numpy as np
import pytest
import yaml

from d2_vision.mock_scan import DEFAULT_FIND_BLOCK, MAX_RUNS, MockScan, blocks_to_points, load_recipe_files, structure_to_blocks

ROOT = Path(__file__).resolve().parents[3]
RECIPES = ROOT / 'src/recipe_manager/recipes'
CFG = yaml.safe_load((ROOT / 'src/d2_robot/d2_bringup/config/robot.yaml').read_text(encoding='utf-8'))   # 노드와 같은 값
ORIGIN = CFG['assembly_origin']
BLOCK_MM = [v * 1000.0 for v in CFG['block_size_m']]
MIN_GAP = CFG['find']['min_gap_mm']
DESIGNS = [('001_CHAIR_BENCH_V000', 11), ('002_CHAIR_BACK_V000', 16), ('003_DESK_STAND_V000', 9), ('004_DESK_PEDESTAL_V000', 11)]


def scanner(tmp_path):
    return MockScan(ORIGIN, BLOCK_MM, MIN_GAP, out_root=tmp_path)


def load(design_id):
    """기본 설계 → (구조, 조립 방법). 두 파일이 없으면 건너뜀."""
    pair = load_recipe_files(str(RECIPES), design_id)
    if pair is None:
        pytest.skip(f'{design_id} 레시피 두 파일이 없다')
    return pair


def task_accepts_infer(body):
    """task_manager._scan_infer 의 형식 검사를 그대로 옮긴 것(+ PR #82 의 cloud_path: 있으면 비지 않은 글자). 틀리면 AssertionError."""
    blocks = body['blocks']
    assert body.get('ok') is True and isinstance(blocks, dict) and blocks.get('schema') == 'blocks/1'
    assert all(isinstance(blocks.get(k), str) and blocks[k] for k in ('design_id', 'family'))
    rows = blocks['blocks']
    assert isinstance(rows, list) and rows
    orders = set()
    for b in rows:
        assert not isinstance(b['order'], bool) and isinstance(b['order'], int) and b['order'] > 0 and b['order'] not in orders
        orders.add(b['order'])
        assert b.get('ori') in ('x', 'y', 'xe', 'ye', 'zx', 'zy') and isinstance(b.get('inferred', False), bool)
        assert all(not isinstance(b[k], bool) and isinstance(b[k], (int, float)) for k in ('x', 'y', 'z'))
    count = body['inferred_count']
    assert not isinstance(count, bool) and isinstance(count, int) and 0 <= count <= len(rows)
    assert isinstance(body['image_path'], str)
    assert isinstance(body['cloud_path'], str) and body['cloud_path']
    json.dumps(body, allow_nan=False)


def test_capture_answers_points_and_remembers_poses(tmp_path):
    """scan_capture: ok · points(음 아닌 정수, bool 아님) — task 의 검사 그대로. run_id 별로 자세를 기억한다."""
    m = scanner(tmp_path)
    for pose in ('observe_front', 'observe_side'):
        ok, reason, body = m.capture(json.dumps({'pose_id': pose, 'run_id': 'R1'}))
        assert ok and reason == '' and body['ok'] is True
        assert isinstance(body['points'], int) and not isinstance(body['points'], bool) and body['points'] >= 0
    assert m.runs['R1'] == ['observe_front', 'observe_side']


def test_capture_forgets_oldest_runs(tmp_path):
    m = scanner(tmp_path)
    for i in range(MAX_RUNS + 5):
        m.capture(json.dumps({'pose_id': 'observe_front', 'run_id': f'R{i}'}))
    assert len(m.runs) == MAX_RUNS and 'R0' not in m.runs and f'R{MAX_RUNS + 4}' in m.runs


@pytest.mark.parametrize('text', ['not json', '[]', '{}', '{"run_id": ""}', '{"run_id": 3, "pose_id": "a"}',
                                  '{"run_id": "../x", "pose_id": "a"}', '{"run_id": "R1"}'])
def test_bad_requests_fail_with_scan_failed(tmp_path, text):
    """JSON 아님 · 객체 아님 · run_id(또는 pose_id) 없음 · 폴더 밖을 가리키는 run_id → success=false, reason SCAN_FAILED."""
    ok, reason, body = scanner(tmp_path).capture(text)
    assert not ok and reason == 'SCAN_FAILED' and body['ok'] is False and body['reason'] == 'SCAN_FAILED'


@pytest.mark.parametrize('design_id, n', DESIGNS)
def test_infer_gives_blocks_task_accepts_and_real_files(tmp_path, design_id, n):
    """scan_infer: 기본 설계 4개 모두 task 형식 검사 통과 · 블록 수 = 레시피 · 사진 · 점군 파일이 실제로 있다."""
    load(design_id)                                                                # 두 파일이 없으면 건너뜀
    m = scanner(tmp_path)
    m.capture(json.dumps({'pose_id': 'observe_front', 'run_id': 'R20261011_101502_5b7e'}))
    ok, reason, body = m.infer(json.dumps({'run_id': 'R20261011_101502_5b7e'}), str(RECIPES), design_id, 'chair', 2)
    assert ok and reason == ''
    task_accepts_infer(body)
    assert len(body['blocks']['blocks']) == n and body['inferred_count'] == 2
    assert body['blocks']['design_id'] == 'scan_chair_01'
    png = Path(body['image_path']).read_bytes()
    assert png[:8] == b'\x89PNG\r\n\x1a\n'
    w, h = struct.unpack('>II', png[16:24])
    assert (w, h) == (320, 240)
    zlib.decompress(png[png.index(b'IDAT') + 4:png.index(b'IEND') - 8])          # 압축이 깨지지 않았다(CRC 4바이트 · 길이 4바이트 앞까지)
    lines = Path(body['cloud_path']).read_text(encoding='ascii').splitlines()
    nv = int(next(line for line in lines if line.startswith('element vertex')).split()[-1])
    pts = np.array([[float(v) for v in line.split()] for line in lines[lines.index('end_header') + 1:]])
    assert pts.shape == (nv, 3) and 100 <= nv <= 5000                              # '점 몇 백 개' — 너무 크지 않게
    assert np.all(np.abs(pts[:, 0] - ORIGIN['x_m']) < 0.15) and np.all(np.abs(pts[:, 1] - ORIGIN['y_m']) < 0.15)   # 조립 영역 안(m)
    assert pts[:, 2].min() > ORIGIN['z_m']                                         # 작업면 위


def test_infer_numbering_and_inferred_clamp(tmp_path):
    """scan 설계 이름은 성공할 때마다 번호 +1, inferred 수는 0 ~ 블록 수로 자른다."""
    m = scanner(tmp_path)
    m.capture(json.dumps({'pose_id': 'p', 'run_id': 'R1'}))
    load('001_CHAIR_BENCH_V000')
    _, _, a = m.infer('{"run_id": "R1"}', str(RECIPES), '001_CHAIR_BENCH_V000', 'chair', 99)
    _, _, b = m.infer('{"run_id": "R1"}', str(RECIPES), '001_CHAIR_BENCH_V000', 'chair', -3)
    assert a['inferred_count'] == 11 and b['inferred_count'] == 0
    assert (a['blocks']['design_id'], b['blocks']['design_id']) == ('scan_chair_01', 'scan_chair_02')


def test_infer_result_failures(tmp_path):
    """촬영 안 한 run_id · 레시피 없음 · scan_fail → success=true + ok:false SCAN_FAILED(task 는 SCAN_FAILED 로 다룬다)."""
    m = scanner(tmp_path)
    assert m.infer('{"run_id": "R9"}', str(RECIPES), '001_CHAIR_BENCH_V000', 'chair', 0)[2]['reason'] == 'SCAN_FAILED'
    m.capture('{"pose_id": "p", "run_id": "R1"}')
    for args in ((str(RECIPES), 'NO_SUCH', 'chair', 0), ('', '001_CHAIR_BENCH_V000', 'chair', 0),
                 (str(RECIPES), '../001_CHAIR_BENCH_V000', 'chair', 0)):
        ok, reason, body = m.infer('{"run_id": "R1"}', *args)
        assert ok and reason == '' and body == {'ok': False, 'reason': 'SCAN_FAILED', 'detail': body['detail']}
    ok, _, body = m.infer('{"run_id": "R1"}', str(RECIPES), '001_CHAIR_BENCH_V000', 'chair', 0, fail=True)
    assert ok and body['ok'] is False and body['reason'] == 'SCAN_FAILED'
    assert not list(tmp_path.iterdir())                                           # 실패 때는 파일을 쓰지 않는다


def test_bench_blocks_match_ird_example():
    """벤치 첫 블록 = IRD 6장 blocks/1 예시({"order":1,"x":-25,"y":0,"z":0,"ori":"y"}) — 설계 좌표 mm, z = 아랫면."""
    s, r = load('001_CHAIR_BENCH_V000')
    first = structure_to_blocks(s, r, 'scan_chair_01', 'chair', 0)['blocks'][0]
    assert (first['order'], first['x'], first['y'], first['z'], first['ori'], first['inferred']) == (1, -25.0, 0.0, 0.0, 'y', False)


@pytest.mark.parametrize('design_id, n', DESIGNS)
def test_same_blocks_as_converter_2(design_id, n):
    """변환기 ②(d2_task RecipeToBlocks)와 같은 자리 · 방향을 낸다. d2_task 를 못 읽으면 건너뜀.
    변환기 ②는 blocks/2.0(역할 · 단계 · 잡기 칸 더함, E-69)이고 스캔은 blocks/1 이라 order · x · y · z · ori 만 비교한다."""
    s, r = load(design_id)
    sys.path.insert(0, str(ROOT / 'src/d2_task'))
    try:
        rtb = pytest.importorskip('d2_task.recipe_to_blocks')
    finally:
        sys.path.remove(str(ROOT / 'src/d2_task'))
    want = rtb.RecipeToBlocks('scan_chair_01', 'chair', BLOCK_MM).convert(s, r)
    got = structure_to_blocks(s, r, 'scan_chair_01', 'chair', 0)
    keys = ('order', 'x', 'y', 'z', 'ori')
    assert [{k: b[k] for k in keys} for b in got['blocks']] == [{k: b[k] for k in keys} for b in want['blocks']]
    assert (got['design_id'], got['family'], len(got['blocks'])) == (want['design_id'], want['family'], n)


def test_points_follow_origin_yaw():
    """설계 (0,0) 윗면 점이 assembly_origin 으로 간다(yaw 0.3° 돌림 포함)."""
    blocks = {'blocks': [{'order': 1, 'x': 0.0, 'y': 0.0, 'z': 0.0, 'ori': 'x', 'inferred': False}]}
    pts = blocks_to_points(blocks, ORIGIN, BLOCK_MM)
    assert np.allclose(pts[:, :2].mean(axis=0), [ORIGIN['x_m'], ORIGIN['y_m']], atol=1e-4)
    assert np.allclose(pts[:, 2], ORIGIN['z_m'] + 0.015)


def test_find_default_is_ird_example(tmp_path):
    """find_blocks 기본 = IRD 예시 블록 하나. clear 는 gap_mm ≥ min_gap(지금 22.2): LENGTH 26 → true, WIDTH 8.5 → false."""
    ok, reason, body = scanner(tmp_path).find('{"run_id": "R1"}')
    assert ok and reason == '' and body['ok'] is True and len(body['blocks']) == 1
    b = body['blocks'][0]
    assert b['clear'] == {'LENGTH': True, 'WIDTH': False}
    keys = ('x_m', 'y_m', 'top_z_m', 'yaw_deg', 'up', 'clear', 'gap_mm', 'overlap', 'tilted', 'confidence')
    assert set(b) == set(keys)
    assert 'clear' not in DEFAULT_FIND_BLOCK                                       # 기본값을 바꾸지 않는다


def test_find_param_list_empty_and_sorted(tmp_path):
    """"[]" = 빈 목록(WAIT_SUPPLY 시험). 여러 개면 윗면 높이 높은 순, clear 는 다시 계산(파라미터 clear 를 믿지 않음)."""
    m = scanner(tmp_path)
    assert m.find('{"run_id": "R1"}', '[]')[2] == {'ok': True, 'blocks': []}
    rows = [dict(DEFAULT_FIND_BLOCK, top_z_m=-0.003, gap_mm={'LENGTH': MIN_GAP, 'WIDTH': 30.0}, clear={'LENGTH': False}),
            dict(DEFAULT_FIND_BLOCK, top_z_m=0.012, up='WIDTH', gap_mm={'LENGTH': 10.0, 'THICKNESS': 40.0}, overlap='top')]
    _, _, body = m.find('{"run_id": "R1"}', json.dumps(rows))
    assert [b['top_z_m'] for b in body['blocks']] == [0.012, -0.003]
    assert body['blocks'][0]['clear'] == {'LENGTH': False, 'THICKNESS': True}
    assert body['blocks'][1]['clear'] == {'LENGTH': True, 'WIDTH': True}


@pytest.mark.parametrize('param', ['{', '{"a": 1}', '[1]', '[{"x_m": 0.4}]'])
def test_find_bad_param_is_error(tmp_path, param):
    ok, reason, body = scanner(tmp_path).find('{"run_id": "R1"}', param)
    assert not ok and reason == 'ERROR' and body['ok'] is False


def test_find_bad_request(tmp_path):
    ok, reason, _ = scanner(tmp_path).find('{"pose_id": "x"}')
    assert not ok and reason == 'SCAN_FAILED'

"""wrist_block 의 scan_capture · scan_infer · find_blocks 가 쓰는 ROS 없는 함수 시험 (pytest, 로봇 · ROS 없음 — W115 · W086 · W148).

노드(wrist_block.py)는 rclpy · cv_bridge · 두산 메시지가 있어야 import 되므로, 노드가 부르는 계산만 여기서 본다:
요청 검사(parse_request) · 깊이 평균(mean_depth_mm) · YOLO 결과 → 마스크(masks_from_yolo) · 검출 그림(draw_found, cv2 있을 때) ·
scan_infer 응답 만들기(scan_response — 작업 관리자 task_manager._scan_infer 가 검사하는 칸) · 기본 설계 → family(family_of · structure_to_blocks).
CI 에는 cv2 가 없다 — cv2 가 필요한 시험은 importorskip 으로 건너뛴다.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from d2_vision.block_checker import mean_depth_mm
from d2_vision.block_finder import draw_found, masks_from_yolo
from d2_vision.mock_scan import parse_request, structure_to_blocks
from d2_vision.structure_scanner import family_of, scan_response

RECIPES = Path(__file__).resolve().parents[2] / 'recipe_manager' / 'recipes'


def test_parse_request_checks_run_id():
    """run_id · pose_id 는 비지 않은 글자, run_id 는 폴더 이름으로 쓸 수 있어야 한다(scan_dir/<run_id>/)."""
    assert parse_request('{"pose_id": "observe_front", "run_id": "R1"}', ('pose_id', 'run_id'))['pose_id'] == 'observe_front'
    for bad in ('not json', '[1]', '{"run_id": ""}', '{"run_id": "../etc"}', '{"run_id": 3}'):
        with pytest.raises(ValueError):
            parse_request(bad, ('run_id',))


def test_mean_depth_skips_holes():
    """구멍(0 · NaN)은 빼고 평균, 모든 장에서 구멍인 화소는 0(add_capture · BlockFinder 가 '없음'으로 읽는 값)."""
    a = np.array([[500.0, 0.0, np.nan]], np.float32)
    b = np.array([[502.0, 0.0, 400.0]], np.float32)
    m = mean_depth_mm([a, b])
    assert m.dtype == np.float32 and m.tolist() == [[501.0, 0.0, 400.0]]
    with pytest.raises(ValueError):
        mean_depth_mm([])


class _Arr:
    """torch 텐서 흉내(.cpu().numpy())."""

    def __init__(self, a):
        self.a = np.asarray(a)

    def cpu(self):
        return self

    def numpy(self):
        return self.a


class _Result:
    """ultralytics Results 흉내 — masks.data (N, h, w) · boxes.conf (N,)."""

    def __init__(self, data, conf):
        self.masks = None if data is None else type('M', (), {'data': _Arr(data)})()
        self.boxes = type('B', (), {'conf': _Arr(conf)})()


def test_masks_from_yolo_same_size_and_resized():
    """원본 크기 마스크는 그대로 bool, 작은 마스크는 가장 가까운 픽셀로 늘린다. 신뢰도는 float 목록."""
    data = np.zeros((2, 4, 6), np.float32)
    data[0, 1:3, 2:5] = 0.9
    data[1, 0, 0] = 0.4                              # 0.5 아래 → 마스크 아님
    masks, scores = masks_from_yolo(_Result(data, [0.8, 0.6]), (4, 6))
    assert len(masks) == 2 and masks[0].dtype == bool and masks[0].sum() == 6 and masks[1].sum() == 0
    assert scores == [pytest.approx(0.8), pytest.approx(0.6)]
    small = np.zeros((1, 2, 3), np.float32)
    small[0, 0, 0] = 1.0                             # 왼쪽 위 1/6 칸
    masks, _ = masks_from_yolo(_Result(small, [0.7]), (4, 6))
    assert masks[0].shape == (4, 6) and masks[0][:2, :2].all() and masks[0].sum() == 4


def test_masks_from_yolo_no_detection():
    """검출이 없으면(masks None · 0개) 빈 목록 — find(masks=[]) 는 빈 블록 목록(ok true → WAIT_SUPPLY)."""
    assert masks_from_yolo(_Result(None, []), (480, 640)) == ([], [])
    assert masks_from_yolo(_Result(np.zeros((0, 480, 640)), []), (480, 640)) == ([], [])


def test_draw_found_keeps_size():
    """wrist_image 그림은 컬러와 같은 크기(640×480 그대로 — IRD E-67)이고 윤곽 · 글자가 그려진다."""
    pytest.importorskip('cv2')
    color = np.full((480, 640, 3), 90, np.uint8)
    m = np.zeros((480, 640), bool)
    m[200:230, 250:350] = True
    blk = {'up': 'THICKNESS', 'yaw_deg': 12.0, 'overlap': 'none', 'tilted': False}
    img = draw_found(color, [blk], [m], [0])
    assert img.shape == color.shape and (img != color).any() and (color == 90).all()   # 원본은 그대로


def _infer_result(ok=True):
    """StructureScanner.infer() 모양(numpy 수가 섞인) 결과."""
    blocks = [{'order': np.int64(1), 'x': np.float64(-25.0), 'y': np.float64(0.0), 'z': np.float64(0.0), 'ori': 'y', 'inferred': np.bool_(True)},
              {'order': 2, 'x': 25.000001, 'y': 0.0, 'z': 15.0, 'ori': 'x', 'inferred': False}]
    return {'ok': ok, 'reason': '' if ok else '점 설명률 0.51 < 0.8', 'inferred_count': 1, 'confidence': {'explained': 0.9},
            'nearest_base': '001_CHAIR_BENCH',
            'blocks': {'schema': 'blocks/1', 'design_id': 'scan_chair_01', 'family': 'chair', 'blocks': blocks}}


def test_scan_response_ok_matches_task_checks():
    """ok: IRD 칸만 · 파이썬 기본형 · NaN 없음 · order int · inferred bool(task_manager._scan_infer 검사와 같은 조건)."""
    success, reason, body = scan_response(_infer_result(), '/tmp/x/color.png', '/tmp/x/cloud.ply')
    assert (success, reason) == (True, '')
    assert set(body) == {'ok', 'blocks', 'inferred_count', 'image_path', 'cloud_path'}
    text = json.dumps(body, allow_nan=False)
    back = json.loads(text)
    rows = back['blocks']['blocks']
    assert back['blocks']['schema'] == 'blocks/1' and back['inferred_count'] == 1
    assert all(type(b['order']) is int and type(b['inferred']) is bool for b in rows)
    assert rows[1]['x'] == 25.0


def test_scan_response_without_cloud_and_failed():
    """PLY 저장 실패(None) → cloud_path 칸을 뺀 ok:true(10/8 임시). 신뢰도 미달 → success true + ok:false SCAN_FAILED + 이유."""
    _, _, body = scan_response(_infer_result(), '', None)
    assert body['ok'] is True and 'cloud_path' not in body and body['image_path'] == ''
    success, reason, body = scan_response(_infer_result(ok=False), '', None)
    assert (success, reason, body['ok'], body['reason']) == (True, '', False, 'SCAN_FAILED') and '설명률' in body['detail']


def test_family_of_and_bases_from_recipes():
    """기본 설계 4개(레시피 두 파일) → blocks/1 + family(이름의 CHAIR · DESK) — scan_infer 의 nearest_base 후보."""
    assert [family_of(d) for d in ('001_CHAIR_BENCH', '004_DESK_PEDESTAL', 'scan_x')] == ['chair', 'desk', 'unknown']
    found = sorted(RECIPES.glob('*_structure.json'))
    if not found:
        pytest.skip('레시피 폴더가 없다')
    for s_path in found:
        did = s_path.name[:-len('_structure.json')]
        r_path = s_path.with_name(did + '_recipe.json')
        b = structure_to_blocks(json.loads(s_path.read_text(encoding='utf-8')), json.loads(r_path.read_text(encoding='utf-8')),
                                did, family_of(did), 0)
        assert b['family'] in ('chair', 'desk') and b['blocks'] and not any(x['inferred'] for x in b['blocks'])

"""가짜 스캔 · 흩뿌림 찾기 계산 MockScan — ROS 없음. 노드 mock_wrist_block 이 쓴다(W140 뒤 task 가상 시험용).

IRD 4.2 `/d2/vision/scan_capture` · `scan_infer` · `find_blocks`(JsonQuery) 의 응답을 카메라 없이 만든다.
노드와 나눈 이유: ROS 없이 pytest 로 응답 모양을 시험하려고(CLAUDE.md 2장 — 계산 파일은 같은 패키지 다른 파일로).
- scan_infer 의 blocks/1 은 레시피 두 파일(`<id>_structure.json` + `<id>_recipe.json`, E-52)에서 직접 만든다.
  d2_task 의 변환기 ②(RecipeToBlocks)를 import 하지 않는 이유: d2_vision 시험(CI)은 src/d2_vision 만 경로에 넣고,
  d2_task 의존을 package.xml 에 새로 더하지 않으려고. 같은 규칙(order = sequence, x · y = 중심, z = 아랫면, ori = 회전 + 크기)이며
  결과가 같은지는 시험(test_mock_scan.py)이 d2_task 가 있을 때 비교한다.
- image_path · cloud_path 는 실제 파일을 쓴다(다리가 읽어 보낼 수 있게): `<임시 폴더>/d2_scan/<run_id>/color.png` · `cloud.ply`.
  PNG 는 표준 라이브러리(zlib)로, PLY 는 글자(ASCII)로 쓴다 — cv2 · open3d 없이(CI 에 없음, 가짜라 가볍게).
단위: blocks/1 은 mm(설계 좌표계), PLY · find_blocks 는 m(base_link).
바깥 영향: infer 가 임시 폴더에 파일 두 개를 쓴다. 그 밖에는 계산만.
실패: 잘못된 요청 → (False, 코드, {"ok":false,"reason","detail"}) — 서비스 success=false 로 답한다.
      답은 냈지만 결과가 실패(촬영 안 한 run_id · 레시피 못 읽음 · scan_fail 흉내) → (True, '', {"ok":false,"reason":"SCAN_FAILED","detail"}).
      작업 관리자는 두 경우 모두 SCAN_FAILED(또는 받은 코드)로 다룬다(task_manager._scan_capture · _scan_infer).
"""
import json
import math
import re
import struct
import tempfile
import zlib
from collections import OrderedDict
from pathlib import Path

import numpy as np

from d2_vision.block_checker import recipe_path

MOCK_POINTS = 180000            # scan_capture 가 답하는 '모은 점 수'(가짜 — IRD 예시와 비슷한 크기)
MAX_RUNS = 20                   # 기억하는 run_id 수(오래된 것부터 지운다 — 노드를 오래 띄워도 메모리가 늘지 않게)
CLOUD_STEP_MM = 5.0             # PLY 점 간격 — 블록 윗면만 5 mm 격자(벤치 11개 ≈ 800점, IRD 복셀 3 mm · 2 MB 보다 훨씬 작다)
IMG_W, IMG_H = 320, 240         # 가짜 사진 크기(px). 1 px = 1 mm 위에서 본 모습
RUN_ID_RE = re.compile(r'^[A-Za-z0-9_-][A-Za-z0-9_.-]*$')   # run_id 는 폴더 이름이 된다 — '/' · 앞 '.' 을 막는다
ORI_CODES = ('x', 'y', 'xe', 'ye', 'zx', 'zy')

# IRD 4.2 · 245줄 find_blocks 예시 블록 하나(눕힘, 긴 쪽으로만 잡을 틈). clear 는 find() 가 gap_mm 으로 다시 계산한다
DEFAULT_FIND_BLOCK = {'x_m': 0.412, 'y_m': 0.221, 'top_z_m': -0.003, 'yaw_deg': 12.0, 'up': 'THICKNESS',
                      'gap_mm': {'LENGTH': 26.0, 'WIDTH': 8.5}, 'overlap': 'none', 'tilted': False, 'confidence': 0.91}


def _fail(code, detail):
    """잘못된 요청 → 서비스 success=false 응답 세 칸."""
    return False, code, {'ok': False, 'reason': code, 'detail': detail}


def _result_fail(detail):
    """요청은 처리했지만 스캔 결과가 실패 → success=true + ok:false + SCAN_FAILED(IRD 7장)."""
    return True, '', {'ok': False, 'reason': 'SCAN_FAILED', 'detail': detail}


def parse_request(text, keys):
    """요청 JSON 글자 → dict. keys 의 칸은 비지 않은 글자여야 한다. 어긋나면 ValueError(무엇이 틀렸는지)."""
    try:
        body = json.loads(text)
    except (TypeError, ValueError):
        raise ValueError('요청이 JSON 이 아니다')
    if not isinstance(body, dict):
        raise ValueError('요청이 JSON 객체가 아니다')
    for k in keys:
        if not isinstance(body.get(k), str) or not body[k]:
            raise ValueError(f'{k} 칸이 없거나 비었다')
    if not RUN_ID_RE.match(body['run_id']):
        raise ValueError('run_id 에 폴더 이름으로 쓸 수 없는 글자가 있다')
    return body


def ori_table(size_mm):
    """블록 크기(길이 · 폭 · 두께, mm) → 방향 코드마다 (x · y · z 방향 길이). IRD 2장 ori 표(변환기 ② ori_extents 와 같은 규칙)."""
    L, W, T = size_mm
    return {'x': (L, W, T), 'y': (W, L, T), 'xe': (L, T, W), 'ye': (T, L, W), 'zx': (T, W, L), 'zy': (W, T, L)}


def structure_to_blocks(structure, recipe, design_id, family, inferred_count):
    """레시피 두 파일 dict → blocks/1 dict(mm, 설계 좌표계). 바깥 영향 없음.

    order = 레시피 sequence, x · y = 블록 중심, z = 아랫면 높이(중심 − z 방향 길이 / 2), ori = |R| · 부품 크기가 맞는 방향 코드.
    inferred_count: 앞 순서(아래층)부터 이 수만큼 inferred: true — 스캔에서 가려진 블록 흉내(0 ~ 블록 수로 자른다).
    실패: 칸이 없거나 블록 · 부품 이름이 안 맞거나 방향을 못 고르면 KeyError · ValueError · TypeError."""
    parts = {p['part_id']: [float(v) for v in p['size_mm']] for p in structure['parts']}
    by_name = {b['block']: b for b in structure['blocks']}
    rows = []
    for st in sorted(recipe['steps'], key=lambda s: s['sequence']):
        b = by_name[st['block']]
        size = parts[b['part_id']]
        ext = tuple(sum(abs(b['R'][i][k]) * size[k] for k in range(3)) for i in range(3))
        hits = [o for o, e in ori_table(size).items() if e == ext]
        if len(hits) != 1:
            raise ValueError(f'{st["block"]}: 크기 {ext} 에 맞는 방향 코드가 {len(hits)}개다')
        x, y, cz = (float(v) for v in b['center_mm'])
        rows.append({'order': int(st['sequence']), 'x': x, 'y': y, 'z': cz - ext[2] / 2, 'ori': hits[0], 'inferred': False})
    if not rows:
        raise ValueError('레시피에 steps 가 없다')
    for r in rows[:max(0, min(int(inferred_count), len(rows)))]:
        r['inferred'] = True
    return {'schema': 'blocks/1', 'design_id': design_id, 'family': family, 'blocks': rows}


def _footprints(blocks, size_mm):
    """blocks/1 블록마다 (x0, x1, y0, y1, 윗면 z) mm — 설계 좌표계. size_mm = 블록 크기(robot.yaml block_size_m × 1000)."""
    table = ori_table(size_mm)
    out = []
    for b in blocks['blocks']:
        ex, ey, ez = table[b['ori']]
        out.append((b['x'] - ex / 2, b['x'] + ex / 2, b['y'] - ey / 2, b['y'] + ey / 2, b['z'] + ez))
    return out


def blocks_to_points(blocks, origin, size_mm, step_mm=CLOUD_STEP_MM):
    """blocks/1(mm, 설계) → 블록 윗면 점군 (N, 3) m, base_link. 바깥 영향 없음.

    size_mm = 블록 크기(길이 · 폭 · 두께). origin = robot.yaml assembly_origin(x_m · y_m · z_m · yaw_deg) — 설계 원점을 base 로 옮긴다(motion_math.recipe_blocks 와 같은 뜻:
    설계 좌표를 yaw 만큼 돌리고 원점만큼 민다). 위에서 보는 손목 카메라처럼 윗면만 둔다(가려진 아래 블록 윗면도 겹쳐 남는다 — 가짜라 그대로)."""
    yaw = math.radians(float(origin.get('yaw_deg', 0.0)))
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for x0, x1, y0, y1, top in _footprints(blocks, size_mm):
        xs = np.arange(x0 + step_mm / 2, x1, step_mm)
        ys = np.arange(y0 + step_mm / 2, y1, step_mm)
        gx, gy = np.meshgrid(xs, ys)
        gx, gy = gx.ravel() / 1000.0, gy.ravel() / 1000.0
        pts.append(np.stack([origin['x_m'] + c * gx - s * gy, origin['y_m'] + s * gx + c * gy,
                             np.full_like(gx, origin['z_m'] + top / 1000.0)], axis=1))
    return np.concatenate(pts) if pts else np.zeros((0, 3))


def write_ply(path, points):
    """점군 (N, 3) m → ASCII PLY 파일. 바깥 영향: path 에 파일 하나를 쓴다(폴더는 부르는 쪽이 만든다)."""
    head = ['ply', 'format ascii 1.0', 'comment d2 mock scan, frame base_link, unit m',
            f'element vertex {len(points)}', 'property float x', 'property float y', 'property float z', 'end_header']
    body = ['%.4f %.4f %.4f' % tuple(p) for p in points]
    Path(path).write_text('\n'.join(head + body) + '\n', encoding='ascii')


def write_png(path, rgb):
    """(H, W, 3) uint8 → PNG 파일(표준 라이브러리 zlib, 필터 없음). 바깥 영향: path 에 파일 하나를 쓴다."""
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[r].tobytes() for r in range(h))

    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))
    Path(path).write_bytes(png)


def top_view(blocks, size_mm):
    """blocks/1 · 블록 크기(mm) → 위에서 본 가짜 사진 (IMG_H, IMG_W, 3) uint8. 1 px = 1 mm, 가운데 = 설계 원점, 위층일수록 밝게.
    화면의 '실물 사진' 칸을 채워 보려는 그림이라 실제 카메라 모습과는 다르다."""
    img = np.full((IMG_H, IMG_W, 3), 40, np.uint8)          # 어두운 작업면
    for x0, x1, y0, y1, top in sorted(_footprints(blocks, size_mm), key=lambda f: f[4]):   # 낮은 것부터 칠해 위층이 위에 남게
        c0, c1 = int(round(IMG_W / 2 + x0)), int(round(IMG_W / 2 + x1))
        r0, r1 = int(round(IMG_H / 2 - y1)), int(round(IMG_H / 2 - y0))   # 그림의 위 = 설계 +y(뒤)
        c0, c1, r0, r1 = max(c0, 0), min(c1, IMG_W), max(r0, 0), min(r1, IMG_H)
        if c0 >= c1 or r0 >= r1:
            continue
        shade = int(min(255, 120 + top))                    # 높을수록 밝게(mm 그대로, 0~135 mm 를 120~255 로)
        img[r0:r1, c0:c1] = (shade, int(shade * 0.8), int(shade * 0.55))   # 나무색 비슷하게
        img[r0, c0:c1] = img[r1 - 1, c0:c1] = 20                          # 블록 테두리
        img[r0:r1, c0] = img[r0:r1, c1 - 1] = 20
    return img


class MockScan:
    """scan_capture · scan_infer · find_blocks 가짜 응답을 만든다. run_id 마다 촬영한 자세를 기억한다. ROS 없음.

    입력: origin = robot.yaml assembly_origin · block_mm = robot.yaml block_size_m × 1000(길이 · 폭 · 두께) ·
          min_gap_mm = robot.yaml find.min_gap_mm · out_root = 파일 폴더(기본 <임시>/d2_scan).
    각 메서드 출력: (success, reason, 응답 dict) — 노드가 JsonQuery 응답 세 칸에 그대로 넣는다."""

    def __init__(self, origin, block_mm, min_gap_mm, out_root=None):
        """설정만 담는다. 파일은 infer 때 쓴다."""
        self.origin = origin
        self.block_mm = [float(v) for v in block_mm]
        self.min_gap_mm = float(min_gap_mm)
        self.out_root = Path(out_root) if out_root else Path(tempfile.gettempdir()) / 'd2_scan'
        self.runs = OrderedDict()           # run_id → 촬영한 pose_id 목록
        self.n_designs = 0                  # 스캔 설계 번호(scan_<family>_<번호>, IRD 2장 design_id) — 노드가 뜬 뒤 성공한 추론 수

    def capture(self, text):
        """scan_capture: {"pose_id","run_id"} → {"ok":true,"points"}. run_id 별로 pose_id 를 기억한다(MAX_RUNS 개까지)."""
        try:
            body = parse_request(text, ('pose_id', 'run_id'))
        except ValueError as e:
            return _fail('SCAN_FAILED', str(e))
        rid = body['run_id']
        self.runs.setdefault(rid, []).append(body['pose_id'])
        self.runs.move_to_end(rid)
        while len(self.runs) > MAX_RUNS:
            self.runs.popitem(last=False)
        return True, '', {'ok': True, 'points': MOCK_POINTS}

    def infer(self, text, recipe_dir, design_id, family, inferred_count, fail=False):
        """scan_infer: {"run_id"} → {"ok":true,"blocks":blocks/1,"inferred_count","image_path","cloud_path"}.

        blocks 는 recipe_dir 의 design_id 레시피 두 파일로 만든다(그 설계를 '스캔했다'고 흉내). 설계 이름은 scan_<family>_<번호>.
        바깥 영향: out_root/<run_id>/color.png · cloud.ply 를 쓴다. 실패: 촬영 안 한 run_id · 레시피 못 읽음 · fail 흉내 → ok:false SCAN_FAILED."""
        try:
            body = parse_request(text, ('run_id',))
        except ValueError as e:
            return _fail('SCAN_FAILED', str(e))
        rid = body['run_id']
        if rid not in self.runs:
            return _result_fail(f'run_id {rid} 로 촬영한 자세가 없다(scan_capture 먼저)')
        if fail:
            return _result_fail('scan_fail 파라미터 — 격자 맞추기 실패 흉내')
        s_path = recipe_path(recipe_dir, design_id, ('_structure.json',))
        r_path = recipe_path(recipe_dir, design_id, ('_recipe.json',))
        if s_path is None or r_path is None:
            return _result_fail(f'레시피 {design_id} 두 파일을 못 찾음(recipe_dir={recipe_dir!r})')
        try:
            structure = json.loads(s_path.read_text(encoding='utf-8'))
            recipe = json.loads(r_path.read_text(encoding='utf-8'))
            blocks = structure_to_blocks(structure, recipe, f'scan_{family}_{self.n_designs + 1:02d}', family, inferred_count)
        except (OSError, KeyError, TypeError, ValueError, IndexError) as e:
            return _result_fail(f'레시피 {design_id} 를 blocks/1 로 못 바꿈: {e}')
        out = self.out_root / rid
        out.mkdir(parents=True, exist_ok=True)
        image, cloud = out / 'color.png', out / 'cloud.ply'
        write_png(image, top_view(blocks, self.block_mm))
        write_ply(cloud, blocks_to_points(blocks, self.origin, self.block_mm))
        self.n_designs += 1
        count = sum(1 for b in blocks['blocks'] if b['inferred'])
        return True, '', {'ok': True, 'blocks': blocks, 'inferred_count': count,
                          'image_path': str(image), 'cloud_path': str(cloud)}

    def find(self, text, blocks_json=''):
        """find_blocks: {"run_id"} → {"ok":true,"blocks":[…]}(IRD 4.2 칸, base_link m, 윗면 높이 높은 순).

        blocks_json = 파라미터 find_blocks_json — 빈 글자면 DEFAULT_FIND_BLOCK 하나, "[]" 면 빈 목록(WAIT_SUPPLY 시험).
        clear 는 늘 gap_mm ≥ min_gap_mm 로 다시 계산한다(IRD — clear 와 gap_mm 이 어긋나지 않게).
        실패: 요청이 틀리면 SCAN_FAILED, 파라미터가 블록 목록 JSON 이 아니면 ERROR(둘 다 success=false)."""
        try:
            parse_request(text, ('run_id',))
        except ValueError as e:
            return _fail('SCAN_FAILED', str(e))
        try:
            rows = json.loads(blocks_json) if blocks_json.strip() else [DEFAULT_FIND_BLOCK]
            if not isinstance(rows, list) or any(not isinstance(b, dict) or not isinstance(b.get('gap_mm'), dict) for b in rows):
                raise ValueError('블록 dict(gap_mm 칸 포함)의 목록이 아니다')
            out = []
            for b in rows:
                b = json.loads(json.dumps(b))                     # 파라미터 · 기본값을 바꾸지 않게 복사
                b['clear'] = {k: float(v) >= self.min_gap_mm for k, v in b['gap_mm'].items()}
                out.append(b)
            out.sort(key=lambda b: -float(b.get('top_z_m', 0.0)))
        except (TypeError, ValueError) as e:
            return _fail('ERROR', f'find_blocks_json 파라미터가 잘못됐다: {e}')
        return True, '', {'ok': True, 'blocks': out}

"""블록 있음·없음·높이 판정 — ROS 없는 계산 (W041, SDD §6.3). 노드 wrist_block 이 쓴다.

흐름: 깊이 영상 → base 점군 → 높이 지도(5 mm 격자, 셀마다 최고 z) → 블록마다 설계 자리(ROI)의 높이 중앙값
      → 설계 윗면 높이와 비교 → state(present · absent · occluded · unknown) + top_z_m · dz_m. dx_m · dy_m 은 NaN(10/8 W100 전).

입력 블록 목록은 **팀 공용 `d2_motion.motion_math.recipe_blocks(cfg, recipe)` 의 출력 형식**이다 — 레시피 형식(E-69 두 파일:
구조 recipe/2.0 + 조립 방법 placements/2.0)과 조립 원점·실측 높이 쌓기는 거기서 한 번만 계산한다. 이 파일은 그 결과(block_id · center(m, base) · rot 3x3)만 받는다.
block_id 는 레시피가 만든 전체 이름(예 001_CHAIR_BENCH_V000_LEG_001_01)과 글자 그대로 맞춘다. 설계는 요청의 design_id 칸으로만 고른다
(E-52 ④ — 블록 이름에서 설계 이름을 잘라 내지 않는다). 그 설계의 레시피 파일을 찾는 것(recipe_path)과 get_design 답(design/2.0)을
recipe_blocks 에 넣을 레시피로 바꾸는 것(recipe_from_design)도 여기 둔다 — wrist_block · mock_wrist_block 이
같이 쓰고, ROS · d2_motion 없이 시험하려고(CI 는 d2_vision 만 빌드한다).
단위: 이 파일 안은 모두 m · rad(레시피 dict 는 파일과 같은 mm 그대로 넘긴다). 좌표: base_link.
높이 지도는 스캔 추론기(StructureScanner, W115)도 같은 함수를 쓴다.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

NAN = float('nan')
# 형식 이름(IRD 6장, E-69) — task_manager · motion_math 와 같은 값. d2_task · d2_motion 을 import 하지 않으려고 여기 적는다
DESIGN_SCHEMA = 'design/2.0'
RECIPE_SCHEMA = 'recipe/2.0'            # 구조(무엇을 어디에)
PLACEMENTS_SCHEMA = 'placements/2.0'    # 조립 방법(순서 · 잡기 · 받침)
CELL_M = 0.005          # 높이 지도 셀 (SDD §6.9 ②: 5 mm)
ROI_SHRINK_M = 0.004    # 블록 자리를 사방 4 mm 줄여서 본다 — 가장자리 깊이 튐 · 1~3 mm 어긋남을 피한다
MIN_CELLS = 6           # 이보다 적은 셀이면 unknown (가려짐 · 깊이 구멍)
TOL_FRACTION = 0.5      # 높이 허용 = 두께의 절반 (SDD §6.3 "기준 안: 두께 절반")


def depth_to_base_points(depth_m, intr, T_base2cam, stride=2, z_min_m=0.1, z_max_m=1.0):
    """깊이 영상 → base 점군. 입력: depth_m (H, W) m, intr (fx, fy, ppx, ppy), T_base2cam 4x4 (m).
    stride 픽셀마다 하나씩 뽑고, 깊이 0 · NaN · 범위 밖은 버린다. 출력: (N, 3) m."""
    fx, fy, ppx, ppy = intr
    d = depth_m[::stride, ::stride]
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    ok = (d > z_min_m) & (d < z_max_m)
    z = d[ok]
    pts_cam = np.stack([(u[ok] - ppx) * z / fx, (v[ok] - ppy) * z / fy, z, np.ones_like(z)], axis=1)
    return (pts_cam @ T_base2cam.T)[:, :3]


def mean_depth_mm(frames):
    """깊이 프레임 여러 장(mm) → 화소마다 평균(mm). scan_capture · find_blocks 가 쓴다(SDD §6.9 '깊이 ~10장 평균').

    입력: frames = 같은 크기 (H, W) 깊이 목록(mm, 구멍은 0 또는 NaN). 출력: (H, W) float32 mm — 구멍은 빼고 평균,
    모든 장에서 구멍인 화소는 0(StructureScanner.add_capture · BlockFinder 가 '없음'으로 읽는 값).
    실패: 목록이 비거나 크기가 다르면 ValueError(np.stack). 바깥 영향 없음."""
    st = np.stack([np.asarray(f, np.float32) for f in frames])
    ok = np.isfinite(st) & (st > 0)
    n = ok.sum(axis=0)
    s = np.where(ok, st, 0.0).sum(axis=0)
    return np.where(n > 0, s / np.maximum(n, 1), 0.0).astype(np.float32)


def height_map(points, origin_xy, half_m, cell_m=CELL_M):
    """점군 → 높이 지도. origin_xy 를 가운데로 ± half_m 영역을 cell_m 격자로 나눠 셀마다 **최고 z** 를 둔다.
    입력 points (N, 3) m. 출력: (grid (n, n) m — 점 없는 셀은 NaN, x0, y0) — x0·y0 는 grid[0][0] 셀의 왼쪽 아래 모서리.
    '최고 z' 인 이유: 블록 옆면 · 아래층이 섞여 보여도 윗면이 가장 높아서 윗면 높이가 남는다."""
    n = int(round(2 * half_m / cell_m))
    x0, y0 = origin_xy[0] - half_m, origin_xy[1] - half_m
    grid = np.full((n, n), -np.inf)        # NaN 으로 시작하면 max(NaN, z) 가 NaN 이라 -inf 로 시작한다
    if len(points) > 0:
        ix = np.floor((points[:, 0] - x0) / cell_m).astype(int)
        iy = np.floor((points[:, 1] - y0) / cell_m).astype(int)
        ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
        np.maximum.at(grid, (ix[ok], iy[ok]), points[ok, 2])
    grid[np.isinf(grid)] = NAN             # 점이 없는 셀
    return grid, x0, y0


def recipe_path(recipe_dir, design_id, suffixes):
    """설계 이름 → recipe_dir 안의 조립 레시피 파일 경로(Path). 파일을 읽지는 않는다(읽기는 motion_math.load_recipe).

    입력: recipe_dir = 레시피 폴더(task 노드와 같은 파라미터) · design_id = 설계 이름(check_progress 요청의 design_id 칸) ·
          suffixes = 파일 이름 끝 후보, 앞쪽이 먼저(motion_math.RECIPE_SUFFIXES — E-69 구조 파일 '_recipe.json'.
          옆 '_placements.csv' 는 load_recipe 가 같이 읽는다).
    출력: 있는 첫 파일의 Path. recipe_dir · design_id 가 비었거나, design_id 가 경로를 가리키거나('/' · '\\' · 앞 '.'),
          후보 파일이 하나도 없으면 None.
    """
    if not recipe_dir or not design_id or any(c in design_id for c in '/\\') or design_id.startswith('.'):
        return None
    for suf in suffixes:
        p = Path(recipe_dir) / f'{design_id}{suf}'
        if p.is_file():
            return p
    return None


def recipe_sha256(recipe):
    """구조(recipe/2.0) 객체의 짝 확인 해시 — 키 정렬 · 공백 없는 JSON(한글 그대로, UTF-8)의 sha256 16진수 64자(IRD 6장, 한세교 정함).
    motion_math.recipe_sha256 · d2_task recipe_document.recipe_sha256 과 같은 식이다(같은 값인지는 시험이 본다). 바깥 영향 없음."""
    text = json.dumps(recipe, sort_keys=True, separators=(',', ':'), ensure_ascii=False)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def recipe_from_design(design, design_id):
    """get_design 답(design/2.0 dict) → motion_math.recipe_blocks 에 넣을 레시피 dict. 바깥 영향 없음(입력을 바꾸지 않는다).

    로컬 파일 길(motion_math.load_recipe)과 같은 모양을 만든다: 조립 방법(placements/2.0 — schema · model_id · recipe_sha256 · steps)에
    구조(recipe/2.0)를 'structure' 칸으로 붙인다. 블록 · 크기 · 받침 내용은 여기서 다시 읽지 않는다 — recipe_blocks 가 읽고, 틀리면 예외를 낸다.
    입력: design = get_design 응답 JSON 을 읽은 dict · design_id = 요청한 설계 이름. 레시피 단위는 파일과 같은 mm.
    출력: 레시피 dict(조립 방법 얕은 복사 + 'structure' 칸).
    실패: ValueError — 봉투 검사는 작업 관리자(task_manager._planner_from)와 같다: schema design/2.0 · design_id 가 요청과 같음 ·
      recipe 가 recipe/2.0 객체 · placements 가 placements/2.0 객체. 칸 이름이 아니라 schema 로 보고, 옛 형식(design/1 · cad_* ·
      assembly.recipe)은 받은 schema 를 적어 거절한다(변환하지 않음, E-69). 더해 두 문서의 model_id 가 다르거나 placements 의
      recipe_sha256 이 구조와 맞지 않을 때(load_recipe · task RecipeDocument 와 같은 거절 — 다른 구조에 대해 쓴 조립 방법으로 보면 받침 · 자리가 어긋난다).
    """
    got = design.get('schema') if isinstance(design, dict) else None
    if got != DESIGN_SCHEMA:
        raise ValueError(f'get_design 답이 {DESIGN_SCHEMA} 가 아니다(받은 schema: {got!r}) — 옛 형식은 거절한다')
    if design.get('design_id') != design_id:
        raise ValueError(f'get_design 답의 design_id({design.get("design_id")!r})가 요청({design_id!r})과 다르다')
    for key, schema in (('recipe', RECIPE_SCHEMA), ('placements', PLACEMENTS_SCHEMA)):
        got = design[key].get('schema') if isinstance(design.get(key), dict) else None
        if got != schema:
            raise ValueError(f'get_design 답의 {key} 가 {schema} 객체가 아니다(받은 schema: {got!r})')
    recipe, placements = design['recipe'], design['placements']
    if placements.get('model_id') != recipe.get('model_id'):
        raise ValueError('recipe 와 placements 의 model_id 가 다르다')
    if placements.get('recipe_sha256') != recipe_sha256(recipe):
        raise ValueError('placements 의 recipe_sha256 이 recipe(구조)와 맞지 않는다')
    out = dict(placements)
    out['structure'] = recipe
    return out


class BlockChecker:
    """블록마다 설계 자리의 높이를 보고 있음 · 없음을 판정한다 (로봇 · ROS 없음).

    입력: blocks = recipe_blocks() 출력 목록(dict: block_id · center (x, y, z) m base · rot 3x3 base←블록) ·
          origin_xy = 조립 원점 (x, y) m · half_m = 조립 작업공간 반폭 · size_m = 실측 블록 (길이, 폭, 두께) m (robot.yaml block_actual_m).
    check(points, block_ids) → 블록마다 dict(block_id, state, top_z_m, dz_m, dx_m, dy_m). dx · dy 는 늘 NaN(1차).
    바깥 영향 없음. 목록에 없는 block_id 는 state unknown, 높이 NaN."""

    def __init__(self, blocks, origin_xy, half_m, size_m, cell_m=CELL_M):
        self.origin_xy, self.half_m, self.cell_m = origin_xy, half_m, cell_m
        size = np.asarray(size_m, float)
        self.blocks = {}
        for b in blocks:
            rot = np.asarray(b['rot'], float)
            ext = np.abs(rot) @ size / 2.0                       # base x · y · z 축 방향 반폭 (m)
            cx, cy, cz = b['center']
            self.blocks[b['block_id']] = {
                'center': np.array([cx, cy, cz]), 'rot': rot, 'half': size / 2.0, 'ext': ext,
                'top_z': cz + ext[2], 'height': 2 * ext[2], 'above_tops': [],
            }
        # 설계상 이 블록 위에 올라가는 블록들의 윗면 높이 — 가려진 블록을 '받침 있음'으로 추정할 때 쓴다
        for me in self.blocks.values():
            for other in self.blocks.values():
                if other is me:
                    continue
                overlap = np.all(np.abs(other['center'][:2] - me['center'][:2]) < me['ext'][:2] + other['ext'][:2] - 1e-6)
                if overlap and other['center'][2] - other['ext'][2] >= me['top_z'] - 1e-6:
                    me['above_tops'].append(other['top_z'])

    def _roi_cells(self, grid, x0, y0, blk):
        """블록 자리(사방 ROI_SHRINK_M 줄임) 안에 들어가는 높이 지도 셀들의 z. 셀 중심을 블록 좌표로 돌려(rot 전치) 상자 안인지 본다."""
        n = grid.shape[0]
        xs = x0 + (np.arange(n) + 0.5) * self.cell_m
        X, Y = np.meshgrid(xs, y0 + (np.arange(n) + 0.5) * self.cell_m, indexing='ij')
        d = np.stack([X - blk['center'][0], Y - blk['center'][1], np.zeros_like(X)], axis=-1)   # 수평 오프셋 (셀, 3)
        local = d @ blk['rot']                                    # rot^T · d (블록 축 성분)
        inside = np.all(np.abs(local) <= blk['half'] - ROI_SHRINK_M, axis=-1)
        z = grid[inside]
        return z[~np.isnan(z)]

    def check(self, points, block_ids):
        """점군(base, m)으로 block_ids 를 판정한다. 출력은 요청 순서대로 같은 길이.
        판정: |측정 − 설계 윗면| ≤ 두께/2 → present · 더 낮음 → absent · 셀 부족 → unknown
              · 더 높음: 설계상 위에 올라가는 블록의 윗면과 맞으면 present(가려짐 — 위 블록이 그 높이에 있으면 받침도 있다,
                top_z · dz 는 못 재서 NaN) · 설계에 없는 높이면 occluded(설계 밖 물체 · 손)."""
        grid, x0, y0 = height_map(points, self.origin_xy, self.half_m, self.cell_m)
        out = []
        for bid in block_ids:
            blk = self.blocks.get(bid)
            if blk is None:
                out.append(dict(block_id=bid, state='unknown', top_z_m=NAN, dz_m=NAN, dx_m=NAN, dy_m=NAN))
                continue
            z = self._roi_cells(grid, x0, y0, blk)
            if len(z) < MIN_CELLS:
                out.append(dict(block_id=bid, state='unknown', top_z_m=NAN, dz_m=NAN, dx_m=NAN, dy_m=NAN))
                continue
            top = float(np.median(z))
            dz = top - blk['top_z']
            tol = blk['height'] * TOL_FRACTION
            if abs(dz) <= tol:
                out.append(dict(block_id=bid, state='present', top_z_m=top, dz_m=dz, dx_m=NAN, dy_m=NAN))
            elif dz < 0:
                out.append(dict(block_id=bid, state='absent', top_z_m=top, dz_m=NAN, dx_m=NAN, dy_m=NAN))
            elif any(abs(top - t) <= tol for t in blk['above_tops']):
                out.append(dict(block_id=bid, state='present', top_z_m=NAN, dz_m=NAN, dx_m=NAN, dy_m=NAN))   # 가려짐
            else:
                out.append(dict(block_id=bid, state='occluded', top_z_m=top, dz_m=NAN, dx_m=NAN, dy_m=NAN))
        return out

"""BlockChecker 시험 (pytest, ROS · 로봇 없음) — SDD 9장 "블록 빼기 · 더 놓기 · 어긋남" + W140(E-52) 설계 · 레시피 파일 고르기.

블록 목록은 recipe_blocks() 출력 형식(block_id · center m base · rot)을 여기서 직접 만든다(벤치 11개 — 001_CHAIR_BENCH 와 같은 배치).
d2_motion 을 import 하지 않는다: CI 는 d2_vision 만 빌드한다. 예외 하나 — 로컬 · get_design 두 길 비교 시험은 d2_motion · d2_task 를
pytest.importorskip 으로 읽어, 없으면(CI) 건너뛴다.
가짜 점군: 놓인 블록의 윗면 + 작업면만(2 mm 간격 — 손목 카메라가 위에서 볼 때 보이는 면).
"""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from d2_vision.block_checker import (BlockChecker, depth_to_base_points, design_of, height_map, recipe_from_design,
                                     recipe_path)

ORIGIN = {'x_m': 0.4261, 'y_m': -0.0725, 'z_m': -0.018, 'yaw_deg': 0.3}
HALF_M = 0.15
SIZE = [0.07445, 0.0248, 0.0148]          # 실측 블록 (robot.yaml block_actual_m)
T = SIZE[2]
IDS = [f'001_CHAIR_BENCH_B{i:03d}' for i in range(1, 12)]


def rot_z(R, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return Rz @ np.asarray(R, float)


def bench_blocks():
    """벤치 11개를 recipe_blocks() 출력 모양으로. 벽 2열 × 4층(긴 쪽 y) + 좌판 3개(긴 쪽 x). 높이는 실측 두께로 쌓는다."""
    yaw = math.radians(ORIGIN['yaw_deg'])
    c, s = math.cos(yaw), math.sin(yaw)
    R_y = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    R_x = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    out = []

    def add(bid, x_mm, y_mm, layer, R):
        x, y = x_mm / 1000.0, y_mm / 1000.0
        out.append({'block_id': bid, 'rot': rot_z(R, yaw),
                    'center': (ORIGIN['x_m'] + c * x - s * y, ORIGIN['y_m'] + s * x + c * y, ORIGIN['z_m'] + (layer + 0.5) * T)})
    n = 0
    for layer in range(4):
        for x in (-25.0, 25.0):
            add(IDS[n], x, 0.0, layer, R_y); n += 1
    for y in (-25.0, 0.0, 25.0):
        add(IDS[n], 0.0, y, 4, R_x); n += 1
    return out


def scene_points(blocks, placed, extra=(), shift_m=None, step=0.002):
    """placed 에 든 블록의 윗면 + 작업면 점군(base, m). extra = [(center_xyz, half_xyz)] 설계에 없는 상자. shift_m = {id: (dx, dy, dz)}."""
    g = np.arange(-HALF_M, HALF_M, step)
    X, Y = np.meshgrid(g, g, indexing='ij')
    pts = [np.stack([X.ravel() + ORIGIN['x_m'], Y.ravel() + ORIGIN['y_m'], np.full(X.size, ORIGIN['z_m'])], 1)]
    boxes = []
    for b in blocks:
        if b['block_id'] in placed:
            c = np.array(b['center'])
            if shift_m and b['block_id'] in shift_m:
                c = c + np.array(shift_m[b['block_id']])
            boxes.append((c, b['rot'], np.array(SIZE) / 2))
    boxes += [(np.array(c, float), np.eye(3), np.array(h, float)) for c, h in extra]
    for c, rot, half in boxes:
        gx = np.arange(-half[0], half[0], step)
        gy = np.arange(-half[1], half[1], step)
        GX, GY = np.meshgrid(gx, gy, indexing='ij')
        local = np.stack([GX.ravel(), GY.ravel(), np.full(GX.size, half[2])], 1)     # 블록 좌표의 윗면
        pts.append(local @ rot.T + c)
    return np.vstack(pts)


@pytest.fixture
def blocks():
    return bench_blocks()


@pytest.fixture
def checker(blocks):
    return BlockChecker(blocks, (ORIGIN['x_m'], ORIGIN['y_m']), HALF_M, SIZE)


def test_all_absent_on_empty_table(checker, blocks):
    res = checker.check(scene_points(blocks, set()), IDS)
    assert [r['state'] for r in res] == ['absent'] * 11
    assert abs(res[0]['top_z_m'] - ORIGIN['z_m']) < 0.001          # 보이는 면(작업면) 높이는 준다


def test_partial_stack(checker, blocks):
    res = checker.check(scene_points(blocks, set(IDS[:6])), IDS)      # 3층까지
    states = [r['state'] for r in res]
    assert states[:6] == ['present'] * 6
    assert states[6:] == ['absent'] * 5                                # 4층 자리 · 좌판 자리
    assert abs(res[4]['top_z_m'] - (ORIGIN['z_m'] + 3 * T)) < 0.001   # 3층 윗면
    assert abs(res[4]['dz_m']) < 0.001


def test_complete_bench(checker, blocks):
    res = checker.check(scene_points(blocks, set(IDS)), IDS)
    assert [r['state'] for r in res] == ['present'] * 11
    assert abs(res[10]['top_z_m'] - (ORIGIN['z_m'] + 5 * T)) < 0.001
    assert math.isnan(res[0]['top_z_m']) and math.isnan(res[0]['dz_m'])    # 1층 벽은 가려져서 높이를 못 잼


def test_extra_block_on_top_is_occluded(checker, blocks):
    """좌판 위에 설계에 없는 블록을 하나 더 놓으면 그 아래 좌판은 occluded (설계에 없는 높이)."""
    top = ORIGIN['z_m'] + 5 * T
    extra = [((ORIGIN['x_m'], ORIGIN['y_m'], top + T / 2), (0.0375, 0.0125, T / 2))]
    res = checker.check(scene_points(blocks, set(IDS), extra=extra), [IDS[9], IDS[8]])
    assert res[0]['state'] == 'occluded'
    assert res[1]['state'] == 'present'


def test_small_shift_still_present(checker, blocks):
    """1 · 3 · 5 mm 옆으로 어긋나도 있음 (SDD 9장) — ±2 mm 판정은 안 한다."""
    for d in (0.001, 0.003, 0.005):
        res = checker.check(scene_points(blocks, {IDS[0], IDS[1]}, shift_m={IDS[0]: (d, 0, 0)}), [IDS[0]])
        assert res[0]['state'] == 'present', d
        assert math.isnan(res[0]['dx_m'])


def test_missing_support_under_seat_limit(checker, blocks):
    """4층 왼쪽(B007)만 빼고 좌판이 그 위에 떠 있는 비현실적 점군: 좌판 윗면이 설계상 위 블록 윗면과 맞아 present 로 추정된다 — 한계 기록."""
    res = checker.check(scene_points(blocks, set(IDS) - {IDS[6]}), [IDS[6]])
    assert res[0]['state'] == 'present'


def test_unknown_id_and_hole(checker, blocks):
    res = checker.check(scene_points(blocks, set()), ['LV9_X999'])
    assert res[0]['state'] == 'unknown' and math.isnan(res[0]['top_z_m'])
    assert checker.check(np.zeros((0, 3)), [IDS[0]])[0]['state'] == 'unknown'


def test_depth_to_base_roundtrip():
    """깊이 영상 → 점군: 카메라가 작업면 0.35 m 위에서 아래를 볼 때 작업면 점의 z 가 table z 로 나온다. 구멍(NaN)은 빠진다."""
    Tm = np.eye(4)
    Tm[:3, :3] = np.diag([1.0, -1.0, -1.0])
    Tm[:3, 3] = [0.4, 0.0, ORIGIN['z_m'] + 0.35]
    depth = np.full((480, 640), 0.35)
    depth[:10, :10] = np.nan
    pts = depth_to_base_points(depth, (600.0, 600.0, 320.0, 240.0), Tm, stride=8)
    assert np.allclose(pts[:, 2], ORIGIN['z_m'], atol=1e-6)
    grid, x0, y0 = height_map(pts, (0.4, 0.0), 0.1)
    assert np.nanmax(grid) - np.nanmin(grid) < 1e-6


# ---------------- W140 (E-52): check_progress 요청 → 설계 · 레시피 파일 ----------------
SUFFIXES = ('_recipe.json', '.recipe.json')     # motion_math.RECIPE_SUFFIXES 와 같은 값 — d2_motion 을 import 하지 않으려고 여기 적는다
NEW_IDS = ['001_CHAIR_BENCH_' + n for n in [f'LEG_00{w}_0{k}' for k in range(1, 5) for w in (1, 2)]
           + [f'SEAT_001_0{k}' for k in range(1, 4)]]     # 작명 규칙 v2 4장 — 벤치 sequence 순서(B001 → LEG_001_01, B002 → LEG_002_01 …)


def test_design_of_uses_design_id_field():
    """design_id 칸이 있으면 블록 이름을 보지 않는다(E-52 ④) — 새 이름에 옛 '_B' 규칙을 쓰면 설계가 틀린다(아래 옛 결과)."""
    cases = [('001_CHAIR_BENCH', '001_CHAIR_BENCH_LEG_001_01', '001_CHAIR'),       # BENCH 의 '_B'
             ('002_CHAIR_BACK', '002_CHAIR_BACK_SEAT_001_01', '002_CHAIR'),        # BACK 의 '_B'
             ('003_DESK_STAND', '003_DESK_STAND_TOP_001_01', ''),                  # '_B' 없음
             ('004_DESK_PEDESTAL', '004_DESK_PEDESTAL_COLUMN_001_03', ''),
             ('001_CHAIR_BENCH', '001_CHAIR_BENCH_B001', '001_CHAIR_BENCH')]       # 옛 이름이 와도 칸이 먼저
    for design, bid, old in cases:
        assert design_of(design, bid) == design
        assert design_of('', bid) == old


def test_design_of_old_rule_when_empty():
    """design_id 가 비면 옛 방식(10/8 저녁까지): 마지막 '_B' 앞. '_B' 가 없으면 빈 글자(부르는 쪽 ERROR · unknown)."""
    assert design_of('', '001_CHAIR_BENCH_B003') == '001_CHAIR_BENCH'
    assert design_of('', '003_DESK_STAND_B009') == '003_DESK_STAND'
    assert design_of('', 'LEG_001_01') == ''
    assert design_of('', '') == ''


def test_recipe_path_prefers_new_name(tmp_path):
    """새 이름 _recipe.json 이 있으면 그것, 없으면 옛 .recipe.json. 둘 다 없으면 None."""
    (tmp_path / '001_CHAIR_BENCH.recipe.json').write_text('{}')
    assert recipe_path(str(tmp_path), '001_CHAIR_BENCH', SUFFIXES) == tmp_path / '001_CHAIR_BENCH.recipe.json'
    (tmp_path / '001_CHAIR_BENCH_recipe.json').write_text('{}')
    assert recipe_path(str(tmp_path), '001_CHAIR_BENCH', SUFFIXES) == tmp_path / '001_CHAIR_BENCH_recipe.json'
    assert recipe_path(str(tmp_path), '002_CHAIR_BACK', SUFFIXES) is None


def test_recipe_path_rejects_bad_input(tmp_path):
    """폴더 · 이름이 비었거나 이름이 경로를 가리키면 파일을 보지 않고 None (task 노드 로컬 읽기와 같은 규칙)."""
    (tmp_path / 'x_recipe.json').write_text('{}')
    sub = tmp_path / 'sub'
    sub.mkdir()
    assert recipe_path('', 'x', SUFFIXES) is None
    assert recipe_path(str(tmp_path), '', SUFFIXES) is None
    for bad in ('../x', 'sub/../x', '..\\x', '.x'):
        assert recipe_path(str(sub), bad, SUFFIXES) is None
    assert recipe_path(str(tmp_path), 'x_recipe.json', ('',)) == tmp_path / 'x_recipe.json'     # 끝 후보는 부르는 쪽이 정한다


def test_new_full_names_match_and_foreign_ids_unknown():
    """레시피가 새 전체 이름을 만들면 같은 이름으로 판정하고, 그 설계에 없는 이름(옛 이름 · 다른 설계)은 그 블록만 unknown."""
    blocks = [dict(b, block_id=n) for b, n in zip(bench_blocks(), NEW_IDS)]
    chk = BlockChecker(blocks, (ORIGIN['x_m'], ORIGIN['y_m']), HALF_M, SIZE)
    ask = [NEW_IDS[0], IDS[0], '002_CHAIR_BACK_BACK_001_01', NEW_IDS[10]]
    res = chk.check(scene_points(blocks, set(NEW_IDS[:2])), ask)
    assert [r['block_id'] for r in res] == ask
    assert [r['state'] for r in res] == ['present', 'unknown', 'unknown', 'absent']
    assert all(math.isnan(r['top_z_m']) and math.isnan(r['dz_m']) for r in res[1:3])


# ---------------- design_source remote (10/7 민범진 · 한석형): get_design 답(design/1) → recipe_blocks 에 넣을 레시피 ----------------
NEW_RECIPE = {'schema': 'cad_recipe/1.0', 'model_id': '001_CHAIR_BENCH', 'structure_sha256': '0' * 64, 'steps': []}
NEW_STRUCT = {'schema': 'cad_structure/1.0', 'model_id': '001_CHAIR_BENCH', 'parts': [], 'blocks': []}
OLD_RECIPE = {'schema': 'cad_recipe/1.0', 'model': {'model_id': '001_CHAIR_BENCH'}, 'steps': []}


def design(recipe, structure=None, design_id='001_CHAIR_BENCH', **extra):
    """시험용 design/1 dict. structure 가 None 이면 칸을 넣지 않는다(옛 한 파일 레시피의 design/1 모양)."""
    d = {'schema': 'design/1', 'design_id': design_id, 'recipe': recipe, **extra}
    if structure is not None:
        d['structure'] = structure
    return d


def test_recipe_from_design_new_attaches_structure():
    """새 형식(E-52): structure 를 레시피 'structure' 칸에 붙인다(load_recipe 와 같은 모양). 입력 design 은 바꾸지 않는다."""
    d = design(NEW_RECIPE, NEW_STRUCT, family='chair', blocks={'schema': 'blocks/1'})
    out = recipe_from_design(d, '001_CHAIR_BENCH')
    assert out['structure'] is NEW_STRUCT
    assert {k: v for k, v in out.items() if k != 'structure'} == NEW_RECIPE
    assert 'structure' not in d['recipe'] and 'structure' not in NEW_RECIPE


def test_recipe_from_design_old_as_is():
    """옛 한 파일(model 칸): 그대로(load_recipe 도 그대로 읽는다). 옛 이름 schema assembly.recipe/1.0 도 받는다(task 와 같음)."""
    assert recipe_from_design(design(OLD_RECIPE), '001_CHAIR_BENCH') == OLD_RECIPE
    old_name = dict(OLD_RECIPE, schema='assembly.recipe/1.0')
    assert recipe_from_design(design(old_name), '001_CHAIR_BENCH') == old_name


@pytest.mark.parametrize('bad', [
    None, [], {'schema': 'blocks/1'},                                            # design/1 아님
    design(NEW_RECIPE, NEW_STRUCT, design_id='002_CHAIR_BACK'),                  # 요청한 설계와 다른 답
    design(None), design(dict(NEW_RECIPE, schema='m0609.jenga.cad_recipe/1.0'), NEW_STRUCT),   # recipe 가 아니거나 schema 다름
    design(NEW_RECIPE), design(NEW_RECIPE, dict(NEW_STRUCT, schema='x')),        # 새 형식인데 structure 없음 · 형식 다름
    design(NEW_RECIPE, dict(NEW_STRUCT, model_id='002_CHAIR_BACK')),             # model_id 다름
    design(OLD_RECIPE, NEW_STRUCT),                                              # 옛 형식에 structure (task RecipeDocument 도 거절)
])
def test_recipe_from_design_rejects(bad):
    """형식이 틀린 get_design 답은 ValueError — wrist_block 은 ERROR 로 답하고 로컬 파일로 대신하지 않는다."""
    with pytest.raises(ValueError):
        recipe_from_design(bad, '001_CHAIR_BENCH')


@pytest.mark.parametrize('design_id, n', [('001_CHAIR_BENCH', 11), ('002_CHAIR_BACK', 16), ('003_DESK_STAND', 9)])
def test_local_and_get_design_paths_give_same_blocks(design_id, n):
    """한석형 부탁(10/7): 로컬 파일 길(load_recipe)과 get_design 길(task 의 RecipeDocument.load(...).design() 으로 만든 design/1 →
    recipe_from_design)이 같은 블록 이름 · 자리를 낸다. 작업 관리자(TaskPlanner)가 쓰는 블록 이름과도 같다.
    d2_motion · d2_task 가 import 되지 않으면(CI 의 d2_vision 시험은 d2_vision 만 경로에 넣는다) 건너뛴다 — 로컬에서 colcon 빌드 뒤 돈다."""
    mm = pytest.importorskip('d2_motion.motion_math')
    rd = pytest.importorskip('d2_task.recipe_document')
    tp = pytest.importorskip('d2_task.task_planner')
    yaml = pytest.importorskip('yaml')
    root = Path(__file__).resolve().parents[3]
    cfg = yaml.safe_load((root / 'src/d2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
    folders = [root / 'src/recipe_manager/recipes']                       # 옛 한 파일(.recipe.json) 셋
    if design_id == '001_CHAIR_BENCH':
        folders.append(root / 'src/d2_task/test/fixtures')                # 새 두 파일(_recipe · _structure) — 벤치만 있다
    for folder in folders:
        path = recipe_path(str(folder), design_id, mm.RECIPE_SUFFIXES)
        if path is None:
            pytest.skip(f'{folder} 에 {design_id} 레시피가 없다')
        local = mm.recipe_blocks(cfg, mm.load_recipe(str(path)))
        d = json.loads(json.dumps(rd.RecipeDocument.load(folder, design_id).design(design_id)))   # 서비스로 오가는 JSON 글자를 한 번 거친다
        remote = mm.recipe_blocks(cfg, recipe_from_design(d, design_id))
        names = [b['block_id'] for b in local]
        assert len(names) == len(set(names)) == n, path.name
        assert [b['block_id'] for b in remote] == names, path.name
        assert all(np.allclose(a['center'], b['center']) and np.allclose(a['rot'], b['rot']) for a, b in zip(local, remote))
        planner = tp.TaskPlanner(cfg, d['recipe'], d.get('structure'))
        assert [b['block_id'] for b in planner.blocks] == names, path.name

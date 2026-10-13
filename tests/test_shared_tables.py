"""여러 파트가 따로 들고 있는 같은 표 · 규칙이 정본과 같은지 본다(10/14 PL C1 ~ C4 — 모으기 + 시험).

정본: 설계 잡기 6개 · 부품 이름 규칙 · 레시피 지문 · 형식 앞자리 비교 = d2_task.recipe_document, 방향 코드 · 표 = d2_task.recipe_to_parts.
같은 패키지(d2_task) · 웹 backend 는 정본을 가져다 쓰고, 패키지 경계 때문에 가져올 수 없는 복사본(d2_motion · d2_vision · 화면 TypeScript ·
robot.yaml)은 여기서 같은지 본다 — 한쪽만 고치면 CI '파이썬 시험'이 잡는다. 바깥 영향 없음(파일 읽기만).
"""
import copy
import json
import re
import sys

import yaml

from conftest import ROBOT_YAML, ROOT

for _sub in ('src/d2_task', 'src/d2_vision', 'web/backend'):
    if str(ROOT / _sub) not in sys.path:
        sys.path.insert(0, str(ROOT / _sub))

from d2_motion import motion_math as mm  # noqa: E402
from d2_task import recipe_builder  # noqa: E402
from d2_task.design_checker import STATE_OF_ORI  # noqa: E402
from d2_task.pick_rank import PickRanker  # noqa: E402
from d2_task.recipe_document import GRASPS, RecipeDocument, part_id, part_name, recipe_sha256, same_major  # noqa: E402
from d2_task.recipe_to_parts import ORIS, ori_extents  # noqa: E402
from d2_vision import block_checker, layer_solver, mock_scan  # noqa: E402

RECIPES = ROOT / 'src' / 'recipe_manager' / 'recipes'
MODELS = ('001_CHAIR_BENCH_V000', '002_CHAIR_BACK_V000', '003_DESK_STAND_V000', '004_DESK_PEDESTAL_V000')
CFG = yaml.safe_load(ROBOT_YAML.read_text(encoding='utf-8'))
BLOCK_MM = tuple(v * 1000 for v in CFG['block_size_m'])


def _steps(steps):
    """순서표 줄 → 비교용 (part_name, part_id, sequence, stage, grasp, grasp_axis, supports) 목록(sequence 순)."""
    return [(s['part_name'], s['part_id'], s['sequence'], s['stage'], s['grasp'], s['grasp_axis'], sorted(s['supports']))
            for s in sorted(steps, key=lambda s: s['sequence'])]


def test_grasp_lists_match_canon():
    """C1: 설계 잡기 6개 — 레시피 도구는 정본을 가져다 쓰고, 집기 노드 표 · 블록 고르기 순서 · robot.yaml 폭 표는 같은 6개."""
    want = set(GRASPS)
    assert len(GRASPS) == 6 and recipe_builder.GRASPS is GRASPS
    assert set(mm.GRASP_AXIS) == want
    assert set(PickRanker.GRASP_ORDER) == want and len(PickRanker.GRASP_ORDER) == 6
    for key in ('grasp_width_m', 'grasp_open_pick_m', 'grasp_open_place_m', 'grasp_close_m'):
        assert set(CFG[key]) == want, key


def test_ori_tables_match_canon():
    """C2: 방향 코드 · 표 — 비전 두 곳 · 검사 묶음 상태 표 · 화면 Preview3D.tsx 가 정본 ori_extents 와 같다."""
    canon = ori_extents(BLOCK_MM)
    assert tuple(canon) == ORIS
    assert layer_solver.ori_extents(BLOCK_MM) == canon
    assert mock_scan.ori_table(BLOCK_MM) == canon
    assert set(STATE_OF_ORI) == set(ORIS)
    ts = (ROOT / 'web' / 'frontend' / 'components' / 'Preview3D.tsx').read_text(encoding='utf-8')
    body = re.search(r'const table: Record<string, Mm3> = \{([^}]*)\}', ts).group(1)
    sym = dict(zip('LWT', BLOCK_MM))
    screen = {k: tuple(sym[a] for a in (a1, a2, a3)) for k, a1, a2, a3 in re.findall(r'(\w+): \[(\w), (\w), (\w)\]', body)}
    assert screen == canon


def test_part_names_match_canon():
    """C3: 부품 이름(<그룹>_<번호>) · 노드 사이 이름(<설계 ID 대문자>_<부품 이름>) — 레시피 읽기 · 집기 노드(순서표에 이름이 없을 때 경로까지)
    · 가짜 스캔이 정본 part_name · part_id 와 같은 이름을 만든다."""
    for model in MODELS:
        recipe = json.loads((RECIPES / f'{model}_recipe.json').read_text(encoding='utf-8'))
        names = sorted(part_name(group, item['part']) for comp in recipe['template']['composition'].values()
                       for group, items in comp['parts'].items() for item in items)
        want = sorted(part_id(model, n) for n in names)
        doc = RecipeDocument.load(RECIPES, model)
        assert sorted(s['part_id'] for s in doc.placements['steps']) == want, model
        loaded = mm.load_recipe(str(RECIPES / f'{model}_recipe.json'))
        assert sorted(p['part_id'] for p in mm.recipe_parts(CFG, loaded)) == want, model
        bare = copy.deepcopy(loaded)                 # 순서표에 part_id 가 없을 때 집기 노드가 만드는 이름도 정본과 같게
        for st in bare['steps']:
            st.pop('part_id')
        assert sorted(p['part_id'] for p in mm.recipe_parts(CFG, bare)) == want, model
        geo = copy.deepcopy(doc.geometry)            # 작업 판단이 넘기는 내부 꼴(instance_id)도
        for st in geo['steps']:
            st.pop('part_id', None)
        assert sorted(p['part_id'] for p in mm.recipe_parts(CFG, geo)) == want, model
        rec, pl = mock_scan.load_recipe_files(RECIPES, model)
        assert sorted(s['part_name'] for s in pl['steps']) == names, model
        family = 'chair' if 'CHAIR' in model else 'desk'
        assert len(mock_scan.structure_to_blocks(rec, pl, 'scan', family, 0)['blocks']) == len(names), model


def test_fingerprint_and_schema_compare_match_canon():
    """C4: 레시피 지문(recipe_sha256) · 형식 앞자리 비교(same_major) — 집기 노드 · 비전 복사본이 정본과 같은 값, 웹은 정본 그대로."""
    import design_store
    for model in MODELS:
        recipe = json.loads((RECIPES / f'{model}_recipe.json').read_text(encoding='utf-8'))
        want = recipe_sha256(recipe)
        assert mm.recipe_sha256(recipe) == want == block_checker.recipe_sha256(recipe), model
    pairs = [('recipe/4.0', 'recipe/4.0'), ('recipe/4.1', 'recipe/4.0'), ('recipe/2.0', 'recipe/4.0'), ('placements/4.0', 'recipe/4.0'),
             ('state/2', 'state/2'), ('state/2.1', 'state/2'), (None, 'recipe/4.0'), ('recipe', 'recipe/4.0'), ('recipe/30', 'recipe/4.0')]
    canon = [same_major(a, b) for a, b in pairs]
    for copy_fn in (mm.same_major, block_checker.same_major):
        assert [copy_fn(a, b) for a, b in pairs] == canon
    assert design_store.same_major is same_major


def test_recipe_readers_match_canon():
    """C4: 레시피 두 파일 읽기 — 집기 노드(load_recipe) · 가짜 스캔(load_recipe_files)이 정본(RecipeDocument)과 같은 순서표를 읽는다."""
    for model in MODELS:
        canon = _steps(RecipeDocument.load(RECIPES, model).placements['steps'])
        assert _steps(mm.load_recipe(str(RECIPES / f'{model}_recipe.json'))['steps']) == canon, model
        _, pl = mock_scan.load_recipe_files(RECIPES, model)
        assert _steps(pl['steps']) == canon, model

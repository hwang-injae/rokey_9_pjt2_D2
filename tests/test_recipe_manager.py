"""recipe_manager(CAD → 레시피, E-52) — 기본 설계 4종을 DXF에서 다시 만들면 저장된 구조 · 조립 · 배치표와 같고,
이름 규칙 · 번호 순서 · 겹침 · 뜬 블록 · 순서 · 단계 · 잡기 상태 · 빠진 CAD 속성을 거부한다."""
import copy
import hashlib
import itertools
import json
import sys

import pytest

from conftest import ROOT

RM = ROOT / 'src' / 'recipe_manager'
sys.path.insert(0, str(RM))
from recipe_manager.recipe_builder import RecipeBuilder  # noqa: E402

DESIGNS = [('001_CHAIR_BENCH', '001_chair_bench'), ('002_CHAIR_BACK', '002_chair_back'), ('003_DESK_STAND', '003_desk_stand'),
           ('004_DESK_PEDESTAL', '004_desk_pedestal')]


def make_boxes(structure):
    """저장된 구조(중심 · 회전 · 치수)로 블록 꼭짓점 8개씩을 만든다 — DXF 없이 RecipeBuilder 를 시험한다."""
    sizes = {p['part_id']: p['size_mm'] for p in structure['parts']}
    boxes = []
    for b in structure['blocks']:
        R, c, half = b['R'], b['center_mm'], [s / 2 for s in sizes[b['part_id']]]
        corners = [tuple(c[i] + sum(R[i][k] * sg[k] * half[k] for k in range(3)) for i in range(3))
                   for sg in itertools.product((-1, 1), repeat=3)]
        boxes.append({'block': b['block'], 'handle': b['cad']['handle'], 'vertices': corners})
    return boxes


def load_saved(model_id):
    """저장된 구조 · 조립 파일 → (구조 dict, 조립 dict, 조립에서 되살린 CAD 속성 hints)."""
    structure = json.loads((RM / 'recipes' / f'{model_id}_structure.json').read_text(encoding='utf-8'))
    recipe = json.loads((RM / 'recipes' / f'{model_id}_recipe.json').read_text(encoding='utf-8'))
    hints = {s['block']: {'SEQ': str(s['sequence']), 'STAGE': str(s['stage']), 'GRASP': s['grasp']} for s in recipe['steps']}
    return structure, recipe, hints


@pytest.fixture
def bench():
    """벤치(001) 저장 레시피 → (builder, 구조, 조립, hints)."""
    return (RecipeBuilder(), *load_saved('001_CHAIR_BENCH'))


# 기본 설계 4종: DXF → 구조 · 조립 · 배치표가 저장된 파일과 같다 (블록 이름 · 순서 · 잡기는 DXF 에서만 온다 — 계획 파일 없음)
@pytest.mark.parametrize('model_id, cad', DESIGNS)
def test_dxf_rebuild_matches_saved_files(model_id, cad):
    pytest.importorskip('ezdxf')
    sys.path.insert(0, str(RM / 'recipe_manager'))
    from main import RecipeManager
    manager = RecipeManager(RM / 'recipes')
    structure, hints = manager.load_structure(RM / 'cads' / f'{cad}.dxf', model_id)
    text = manager.builder.make_json_text(structure)
    assert text == (RM / 'recipes' / f'{model_id}_structure.json').read_text(encoding='utf-8')
    recipe = manager.builder.make_recipe(structure, hashlib.sha256(text.encode('utf-8')).hexdigest(), hints)
    assert manager.builder.make_json_text(recipe) == (RM / 'recipes' / f'{model_id}_recipe.json').read_text(encoding='utf-8')
    csv_lines = (RM / 'recipes' / f'{model_id}_placements.csv').read_text(encoding='utf-8').splitlines()
    assert len(csv_lines) == len(recipe['steps']) + 1 and csv_lines[1].startswith(f'{model_id}_{recipe["steps"][0]["block"]},')


# 폴더: cads/ 에는 CAD 만(계획 파일 없음), recipes/ 에는 모형마다 구조 · 조립 · 배치표 3개만(블록 JSON 파일은 두지 않음 — 웹이 변환기 ②로 바꿈, E-59)
def test_folders_hold_only_cad_and_results():
    assert not list((RM / 'cads').glob('*plan*'))
    names = sorted(p.name for p in (RM / 'recipes').iterdir())
    assert names == sorted(f'{m}_{k}' for m, _ in DESIGNS for k in ('structure.json', 'recipe.json', 'placements.csv'))


# 조립 파일의 structure_sha256 = 저장된 구조 파일 바이트의 sha256 (읽는 쪽이 이 값으로 구조가 바뀌었는지 본다)
@pytest.mark.parametrize('model_id, cad', DESIGNS)
def test_structure_sha256_matches_saved_bytes(model_id, cad):
    raw = (RM / 'recipes' / f'{model_id}_structure.json').read_bytes()
    recipe = json.loads((RM / 'recipes' / f'{model_id}_recipe.json').read_text(encoding='utf-8'))
    assert recipe['schema'] == 'cad_recipe/1.0' and recipe['structure_sha256'] == hashlib.sha256(raw).hexdigest()


# 저장 구조로 다시 만든 구조 · 조립이 원본과 같다 (꼭짓점 → 치수 · 회전 · 받침 · 블록 이름)
def test_boxes_round_trip_to_same_recipe(bench):
    builder, structure, recipe, hints = bench
    rebuilt = builder.make_structure(make_boxes(structure), structure['model_id'],
                                     structure['source_cad']['filename'], structure['source_cad']['sha256'])
    assert rebuilt == structure
    assert builder.make_recipe(rebuilt, recipe['structure_sha256'], hints) == recipe
    assert recipe['steps'][-1]['supports'] == ['LEG_001_04', 'LEG_002_04']
    assert {s['grasp']: s['grasp_axis'] for s in recipe['steps']} == {'FLAT_SHORT': 'WIDTH', 'FLAT_LONG': 'LENGTH'}


# 이름 규칙: 꼴이 틀림 · 같은 이름 두 번 · 번호가 위치 순서와 다름(아래층부터 · 앞 → 뒤 · 왼 → 오른) · 번호 빠짐 → 거부
@pytest.mark.parametrize('rename, message', [
    ({'LEG_001_01': 'B001'}, 'block name must be'),
    ({'LEG_001_01': 'LEG_001_02', 'LEG_001_02': 'LEG_001_02'}, 'used twice'),
    ({'LEG_001_01': 'LEG_001_02', 'LEG_001_02': 'LEG_001_01'}, 'bottom → top'),
    ({'SEAT_001_01': 'SEAT_001_03', 'SEAT_001_03': 'SEAT_001_01'}, 'front → rear'),
    ({'LEG_001_01': 'LEG_002_05', 'LEG_001_02': 'LEG_002_06', 'LEG_001_03': 'LEG_002_07', 'LEG_001_04': 'LEG_002_08',
      'LEG_002_01': 'LEG_001_01', 'LEG_002_02': 'LEG_001_02', 'LEG_002_03': 'LEG_001_03', 'LEG_002_04': 'LEG_001_04'}, 'block numbers must run'),
    ({'SEAT_001_03': 'SEAT_001_04'}, 'must run 01..03'),
])
def test_bad_block_names_rejected(bench, rename, message):
    builder, structure, _, _ = bench
    boxes = [dict(b, block=rename.get(b['block'], b['block'])) for b in make_boxes(structure)]
    with pytest.raises(ValueError, match=message):
        builder.make_structure(boxes, 'X', 'x.dxf', '0')


# 형상 오류: 같은 자리에 두 블록(겹침) · 책상 아래 블록은 구조를 만들 때 거부한다
def test_overlap_and_below_table_rejected(bench):
    builder, structure, _, _ = bench
    boxes = make_boxes(structure)
    with pytest.raises(ValueError, match='overlap'):
        builder.make_structure(boxes + [dict(boxes[8], block='SEAT_002_01')], 'X', 'x.dxf', '0')
    sunk = [dict(b, vertices=[(x, y, z - 10) for x, y, z in b['vertices']]) for b in boxes]
    with pytest.raises(ValueError, match='below table'):
        builder.make_structure(sunk, 'X', 'x.dxf', '0')


# CAD 속성 오류: 받침보다 먼저 놓기(뜬 블록) · 순서 빠짐 · 단계 건너뜀 · 잡기 상태가 CAD와 다름 · 속성 빠짐 · 모르는 잡기 이름
@pytest.mark.parametrize('break_hints, message', [
    (lambda h: h['LEG_001_01'].update(SEQ='3', STAGE='2') or h['LEG_001_02'].update(SEQ='1', STAGE='1'), 'float'),
    (lambda h: h['SEAT_001_03'].update(SEQ='99'), 'SEQ must run'),
    (lambda h: h['SEAT_001_03'].update(STAGE='7'), 'STAGE must start'),
    (lambda h: h['LEG_001_01'].update(GRASP='EDGE_SHORT'), 'needs the part EDGE'),
    (lambda h: h['LEG_001_01'].pop('GRASP'), r'CAD attributes missing: LEG_001_01\(GRASP\)'),
    (lambda h: h['LEG_001_01'].update(GRASP='SIDE_99'), 'unknown CAD GRASP'),
    (lambda h: h['LEG_001_01'].update(SEQ='one'), 'must be integers'),
])
def test_bad_cad_attributes_rejected(bench, break_hints, message):
    builder, structure, recipe, hints = bench
    hints = copy.deepcopy(hints)
    break_hints(hints)
    with pytest.raises(ValueError, match=message):
        builder.make_recipe(structure, recipe['structure_sha256'], hints)


# 옛 GRASP 속성(SIDE_25 · END_75 — 지금 DXF 값)은 CAD 배치를 보고 6가지 이름으로, 수직 닫힘 축은 거부
def test_legacy_grasp_attributes(bench):
    builder, structure, recipe, hints = bench
    legacy = {k: dict(v, GRASP={'FLAT_SHORT': 'SIDE_25', 'FLAT_LONG': 'END_75'}[v['GRASP']]) for k, v in hints.items()}
    assert builder.make_recipe(structure, recipe['structure_sha256'], legacy) == recipe
    with pytest.raises(ValueError, match='vertical'):
        builder.make_recipe(structure, recipe['structure_sha256'], dict(legacy, LEG_001_01=dict(hints['LEG_001_01'], GRASP='THICKNESS')))


# 변환기 ①(W110 · W139): 기본 설계 4종을 블록 JSON(변환기 ②)으로 바꿨다가 다시 레시피로 — 로봇 목표(중심 · 회전 · 순서 · 단계 · 잡기)가
# CAD 레시피와 같다(V-45). 이름은 역할을 모르므로 BLOCK_001_<번호>. 검사 묶음에 붙이면 check_design 이 두 파일을 돌려준다.
@pytest.mark.parametrize('model_id, cad', DESIGNS)
def test_blocks_to_recipe_matches_cad_recipe(model_id, cad, robot_cfg):
    sys.path.insert(0, str(ROOT / 'src' / 'd2_task'))
    from d2_motion import motion_math as mm
    from d2_task.design_checker import DesignChecker
    from d2_task.recipe_document import RecipeDocument
    from d2_task.recipe_to_blocks import RecipeToBlocks
    from recipe_manager.blocks_to_recipe import BlocksToRecipe
    block_mm = [v * 1000.0 for v in robot_cfg['block_size_m']]
    doc = RecipeDocument.load(RM / 'recipes', model_id)
    request = RecipeToBlocks(model_id.lower(), 'test', block_mm).convert(doc.recipe, doc.structure)
    checker = DesignChecker(robot_cfg)
    checker.blocks_to_recipe = BlocksToRecipe(checker.grasp_options, block_mm).convert
    result = checker.check(request)
    assert result['ok'], result['errors']
    converted = {'structure': result['structure'], 'recipe': result['recipe']}
    assert converted['recipe']['steps'][0]['block'].startswith('BLOCK_001_')
    cad = mm.recipe_blocks(robot_cfg, mm.load_recipe(str(RM / 'recipes' / f'{model_id}_recipe.json')))
    gen = mm.recipe_blocks(robot_cfg, dict(converted['recipe'], structure=converted['structure']))
    assert [(b['sequence'], b['stage'], b['grasp'], b['rot']) for b in gen] == [(b['sequence'], b['stage'], b['grasp'], b['rot']) for b in cad]
    assert all(max(abs(p - q) for p, q in zip(g['center'], c['center'])) < 1e-9 for g, c in zip(gen, cad))


# 변환기 ①: 잡을 후보가 없는 블록 · order 빠짐 · 모르는 ori 는 거부한다
def test_blocks_to_recipe_rejects_bad_input(robot_cfg):
    from recipe_manager.blocks_to_recipe import BlocksToRecipe
    block_mm = [v * 1000.0 for v in robot_cfg['block_size_m']]
    one = {'design_id': 't', 'blocks': [{'order': 1, 'x': 0, 'y': 0, 'z': 0, 'ori': 'x'}]}
    with pytest.raises(ValueError, match='no grasp'):
        BlocksToRecipe(lambda blocks: {1: []}, block_mm).convert(one)
    with pytest.raises(ValueError, match='order must run'):
        BlocksToRecipe(lambda blocks: {2: ['FLAT_SHORT']}, block_mm).convert(dict(one, blocks=[dict(one['blocks'][0], order=2)]))
    with pytest.raises(ValueError, match='unknown ori'):
        BlocksToRecipe(lambda blocks: {1: ['FLAT_SHORT']}, block_mm).convert(dict(one, blocks=[dict(one['blocks'][0], ori='q')]))
    out = BlocksToRecipe(lambda blocks: {1: ['FLAT_LONG', 'FLAT_SHORT']}, block_mm).convert(one)
    assert out['recipe']['steps'][0]['grasp'] == 'FLAT_SHORT' and out['structure']['source_cad'] is None

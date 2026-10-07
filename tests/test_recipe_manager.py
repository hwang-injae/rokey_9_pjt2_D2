"""recipe_manager(CAD → 레시피) — 기본 설계 4종이 저장된 레시피와 같게 나오고, 겹침 · 뜬 블록 · 순서 · 잡기 상태 오류를 거부한다."""
import copy
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


def make_boxes(model):
    """저장된 레시피의 모델(중심 · 회전 · 치수)로 블록 꼭짓점 8개씩을 만든다 — DXF 없이 RecipeBuilder 를 시험한다."""
    sizes = {p['part_id']: p['size_mm'] for p in model['parts']}
    boxes = []
    for inst in model['instances']:
        R, c, half = inst['R'], inst['center_mm'], [s / 2 for s in sizes[inst['part_id']]]
        corners = [tuple(c[i] + sum(R[i][k] * sg[k] * half[k] for k in range(3)) for i in range(3))
                   for sg in itertools.product((-1, 1), repeat=3)]
        boxes.append({'id': inst['instance_id'], 'vertices': corners, 'hints': {}})
    return boxes


@pytest.fixture
def bench():
    """벤치(001) 저장 레시피 → (builder, 모델, 계획). 계획은 레시피 단계에서 사람이 쓰는 칸만 남긴 것."""
    recipe = json.loads((RM / 'recipes' / '001_CHAIR_BENCH_recipe.json').read_text(encoding='utf-8'))
    plan = json.loads((RM / 'cads' / '001_CHAIR_BENCH_plan.json').read_text(encoding='utf-8'))
    return RecipeBuilder(), recipe['model'], plan


# 기본 설계 4종: DXF → 계획 양식 · 레시피 · 배치표가 저장된 파일과 같다 (리팩터링 전후 결과 고정)
@pytest.mark.parametrize('model_id, cad', DESIGNS)
def test_dxf_rebuild_matches_saved_files(model_id, cad):
    pytest.importorskip('ezdxf')
    sys.path.insert(0, str(RM / 'recipe_manager'))
    from main import RecipeManager
    manager = RecipeManager(RM / 'recipes')
    model, hints = manager.load_model(RM / 'cads' / f'{cad}.dxf', model_id, '1')
    plan_text = (RM / 'cads' / f'{model_id}_plan.json').read_text(encoding='utf-8')
    assert manager.make_json_text(manager.builder.make_plan_template(model, hints)) == plan_text
    recipe = manager.builder.build_recipe(model, json.loads(plan_text))
    assert manager.make_json_text(recipe) == (RM / 'recipes' / f'{model_id}_recipe.json').read_text(encoding='utf-8')
    rows = manager.builder.make_placement_rows(recipe)
    csv_lines = (RM / 'recipes' / f'{model_id}_placements.csv').read_text(encoding='utf-8').splitlines()
    assert len(csv_lines) == len(rows) + 1 and csv_lines[1].startswith(f'{model_id}_B001,1,1,')


# recipes/ 에는 모형마다 결과 3개(_recipe.json · _placements.csv · 10/8 W117 변환기 ②가 만든 _blocks.json — HMI 가 이 폴더만 읽게)만 있다 — 계획은 cads/ 옆, 모델은 레시피 안에만
def test_recipes_folder_has_only_results():
    names = sorted(p.name for p in (RM / 'recipes').iterdir())
    assert names == sorted(f'{m}_{k}' for m, _ in DESIGNS for k in ('recipe.json', 'placements.csv', 'blocks.json'))


# 저장 레시피의 모델로 다시 만든 모델 · 레시피가 원본과 같다(꼭짓점 → 치수 · 회전 · 받침 · block_id)
def test_boxes_round_trip_to_same_recipe(bench):
    builder, model, plan = bench
    rebuilt = builder.make_model(make_boxes(model), model['model_id'], '1',
                                 model['source_cad']['filename'], model['source_cad']['sha256'])
    assert rebuilt == model
    recipe = builder.build_recipe(rebuilt, plan)
    assert [s['block_id'] for s in recipe['steps']] == [f'001_CHAIR_BENCH_B{n:03d}' for n in range(1, 12)]
    assert recipe['steps'][8]['support_block_ids'] == ['001_CHAIR_BENCH_B007', '001_CHAIR_BENCH_B008']
    assert {s['grasp']: s['grasp_axis'] for s in recipe['steps']} == {'FLAT_SHORT': 'WIDTH', 'FLAT_LONG': 'LENGTH'}


# 형상 오류: 같은 자리에 두 블록(겹침) · 책상 아래 블록은 모델을 만들 때 거부한다
def test_overlap_and_below_table_rejected(bench):
    builder, model, _ = bench
    boxes = make_boxes(model)
    with pytest.raises(ValueError, match='overlap'):
        builder.make_model(boxes + [dict(boxes[0], id='DUP')], 'X', '1', 'x.dxf', '0')
    sunk = [dict(b, vertices=[(x, y, z - 10) for x, y, z in b['vertices']]) for b in boxes]
    with pytest.raises(ValueError, match='below table'):
        builder.make_model(sunk, 'X', '1', 'x.dxf', '0')


# 계획 오류: 받침보다 먼저 놓기(뜬 블록) · 순서 빠짐 · 단계 건너뜀 · 잡기 상태가 CAD와 다름 · 잡기 빈칸 · 다른 CAD
@pytest.mark.parametrize('break_plan, message', [
    (lambda p: p['steps'][0].update(sequence=3, stage=2) or p['steps'][2].update(sequence=1, stage=1), 'float'),
    (lambda p: p['steps'][-1].update(sequence=99), 'sequence must run'),
    (lambda p: p['steps'][-1].update(stage=7), 'stage must start'),
    (lambda p: p['steps'][0].update(grasp='EDGE_SHORT'), 'needs the part EDGE'),
    (lambda p: p['steps'][0].update(grasp=None), 'grasp is not set'),
    (lambda p: p['steps'][0].update(grasp='SIDE_25'), 'grasp must be one of'),
    (lambda p: p.update(source_cad_sha256='0' * 64), 'sha256 mismatch'),
    (lambda p: p['steps'].pop(), 'exactly once'),
])
def test_bad_plan_rejected(bench, break_plan, message):
    builder, model, plan = bench
    plan = copy.deepcopy(plan)
    break_plan(plan)
    with pytest.raises(ValueError, match=message):
        builder.build_recipe(model, plan)


# 계획 양식: 힌트가 없으면 빈칸, 옛 GRASP 이름(SIDE_25 · END_75)은 CAD 배치를 보고 6가지 이름으로 바꾼다
def test_plan_template_hints_and_legacy_grasp(bench):
    builder, model, _ = bench
    empty = builder.make_plan_template(model)
    assert all(s['sequence'] is None and s['grasp'] is None for s in empty['steps'])
    legs, seat = model['instances'][0]['instance_id'], model['instances'][8]['instance_id']
    filled = builder.make_plan_template(model, {legs: {'GRASP': 'SIDE_25'}, seat: {'GRASP': 'END_75'}})
    grasp = {s['instance_id']: s['grasp'] for s in filled['steps']}
    assert (grasp[legs], grasp[seat]) == ('FLAT_SHORT', 'FLAT_LONG')
    with pytest.raises(ValueError, match='vertical'):
        builder.make_plan_template(model, {legs: {'GRASP': 'THICKNESS'}})


"""recipe_manager(CAD → 레시피 recipe/4.0 + 조립 방법 placements/4.0, 10/8 E-69 · 10/13 PLM 트리) — 기본 설계 4종을 DXF에서 다시 만들면
저장된 _recipe.json · _placements.csv 와 같고(사람이 HMI에서 쓴 칸은 저장 파일에서 옮김), 짝 해시 · 이름 규칙 · 번호 순서 · 겹침 · 뜬 블록 ·
순서 · 단계 · 잡기 상태 · 빠진 CAD 속성을 거부한다."""
import copy
import csv
import hashlib
import itertools
import json
import sys

import pytest

from conftest import ROOT

RM = ROOT / 'src' / 'recipe_manager'
sys.path.insert(0, str(ROOT / 'src' / 'd2_task'))
from d2_task.recipe_builder import PLACEMENT_COLUMNS, RecipeBuilder  # noqa: E402

DESIGNS = [('001_CHAIR_BENCH_V000', '001_chair_bench_v000'), ('002_CHAIR_BACK_V000', '002_chair_back_v000'), ('003_DESK_STAND_V000', '003_desk_stand_v000'),
           ('004_DESK_PEDESTAL_V000', '004_desk_pedestal_v000')]


HUMAN_TEMPLATE_KEYS = ('name', 'description', 'tags')    # 사람이 HMI에서 쓰는 칸 — template 위
HUMAN_ROLE_KEYS = ('label', 'revision')                   # 사람이 HMI에서 쓰는 칸 — 구성요소(역할)마다


def iter_part_items(recipe):
    """레시피 트리의 부품 항목을 (부품 이름, 항목) 으로 하나씩 돌려준다."""
    for comp in recipe['template']['composition'].values():
        for group, items in comp['parts'].items():
            for item in items:
                yield f'{group}_{item["part"]}', item


def strip_human_fields(recipe):
    """저장 레시피에서 사람 칸을 뺀 복사본 — RecipeBuilder 가 만드는 계산 값만의 모양."""
    out = copy.deepcopy(recipe)
    for key in HUMAN_TEMPLATE_KEYS:
        out['template'].pop(key, None)
    for comp in out['template']['composition'].values():
        for key in HUMAN_ROLE_KEYS:
            comp.pop(key, None)
    return out


def make_boxes(recipe):
    """저장된 레시피(중심 · 회전 · 치수)로 블록 꼭짓점 8개씩을 만든다 — DXF 없이 RecipeBuilder 를 시험한다."""
    sizes = {p['stock_id']: p['size_mm'] for p in recipe['stocks']}
    boxes = []
    for name, b in iter_part_items(recipe):
        R, c, half = b['R'], b['center_mm'], [s / 2 for s in sizes[b['stock_id']]]
        corners = [tuple(c[i] + sum(R[i][k] * sg[k] * half[k] for k in range(3)) for i in range(3))
                   for sg in itertools.product((-1, 1), repeat=3)]
        boxes.append({'part_name': name, 'handle': b['cad']['handle'], 'vertices': corners})
    return boxes


def load_saved(model_id):
    """recipes/ 의 두 파일 → (레시피 dict, CSV 에서 읽은 조립 steps, steps 에서 되살린 CAD 속성 hints).

    CSV 는 글자라 sequence · stage 는 정수로, supports 는 ';' 로 나눠 placements/4.0 steps 와 같은 모양으로 바꾼다.
    """
    recipe = json.loads((RM / 'recipes' / f'{model_id}_recipe.json').read_text(encoding='utf-8'))
    with open(RM / 'recipes' / f'{model_id}_placements.csv', encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))
    steps = [{'part_name': r['part_name'], 'part_id': r['part_id'], 'sequence': int(r['sequence']), 'stage': int(r['stage']),
              'grasp': r['grasp'], 'grasp_axis': r['grasp_axis'], 'supports': [x for x in r['supports'].split(';') if x]}
             for r in rows]
    hints = {s['part_name']: {'SEQ': str(s['sequence']), 'STAGE': str(s['stage']), 'GRASP': s['grasp']} for s in steps}
    return recipe, steps, hints


@pytest.fixture
def bench():
    """벤치(001) 저장 레시피 → (builder, 레시피, 조립 steps, hints)."""
    return (RecipeBuilder(), *load_saved('001_CHAIR_BENCH_V000'))


# 기본 설계 4종: DXF → 두 파일이 저장된 recipes/ 파일과 글자까지 같다 (블록 이름 · 순서 · 잡기는 DXF 에서만 온다 — 계획 파일 없음).
# 사람이 HMI에서 쓴 칸은 이미 저장된 레시피에 있다고 보고, 저장 파일을 먼저 두고 덮어쓰기([1])로 다시 만든다.
@pytest.mark.parametrize('model_id, cad', DESIGNS)
def test_dxf_build_matches_saved_files(model_id, cad, tmp_path, monkeypatch):
    pytest.importorskip('ezdxf')
    sys.path.insert(0, str(RM / 'recipe_manager'))
    from main import RecipeManager
    (tmp_path / f'{model_id}_recipe.json').write_bytes((RM / 'recipes' / f'{model_id}_recipe.json').read_bytes())
    monkeypatch.setattr('builtins.input', lambda prompt: '1')
    RecipeManager(tmp_path).build_recipe(RM / 'cads' / f'{cad}.dxf')
    names = [f'{model_id}_placements.csv', f'{model_id}_recipe.json']
    assert sorted(p.name for p in tmp_path.iterdir()) == names
    for name in names:
        assert (tmp_path / name).read_bytes() == (RM / 'recipes' / name).read_bytes(), name


# 폴더: cads/ 에는 CAD 만(계획 파일 없음), recipes/ 에는 모형마다 _recipe.json · _placements.csv 2개만(_structure.json 없앰 — E-69)
def test_folders_hold_only_cad_and_results():
    assert not list((RM / 'cads').glob('*plan*'))
    names = sorted(p.name for p in (RM / 'recipes').iterdir())
    assert names == sorted(f'{m}_{k}' for m, _ in DESIGNS for k in ('recipe.json', 'placements.csv'))


# 저장된 두 파일의 짝: 레시피 schema recipe/4.0, CSV 칸 = PLACEMENT_COLUMNS, 모든 줄 recipe_sha256 = 레시피 객체 해시 · schema placements/4.0
@pytest.mark.parametrize('model_id, cad', DESIGNS)
def test_saved_placements_match_recipe(model_id, cad):
    recipe = json.loads((RM / 'recipes' / f'{model_id}_recipe.json').read_text(encoding='utf-8'))
    with open(RM / 'recipes' / f'{model_id}_placements.csv', encoding='utf-8', newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    assert recipe['schema'] == 'recipe/4.0' and recipe['design_id'] == model_id
    assert tuple(reader.fieldnames) == PLACEMENT_COLUMNS
    assert {(r['recipe_sha256'], r['schema']) for r in rows} == {(RecipeBuilder().calculate_recipe_sha256(recipe), 'placements/4.0')}
    assert sorted(r['part_name'] for r in rows) == sorted(name for name, _ in iter_part_items(recipe))


# 저장된 기본 설계 4종: 자재 stocks(젠가 블록 하나 = STOCK_001)를 부품마다 stock_id 로 참조하고(부품에 size_mm 없음),
# 구성요소마다 사람 칸(label · revision)과 계산 칸(count = 그룹 수)이 있다. 최상위 parts 목록은 없다
@pytest.mark.parametrize('model_id, cad', DESIGNS)
def test_saved_recipe_has_template_tree(model_id, cad):
    recipe = json.loads((RM / 'recipes' / f'{model_id}_recipe.json').read_text(encoding='utf-8'))
    assert 'parts' not in recipe
    assert recipe['stocks'] == [{'stock_id': 'STOCK_001', 'size_mm': [75.0, 25.0, 15.0]}]
    assert all(item['stock_id'] == 'STOCK_001' and 'size_mm' not in item for _, item in iter_part_items(recipe))
    assert all(key in recipe['template'] for key in HUMAN_TEMPLATE_KEYS)
    for comp in recipe['template']['composition'].values():
        assert all(key in comp for key in HUMAN_ROLE_KEYS) and comp['count'] == len(comp['parts'])


# 사람 칸 옮기기: 저장 파일의 사람 칸은 그대로, 계산 칸(부품 항목)은 CAD에서 새로. 저장 파일이 없는 새 설계는 계산 값만 + 빠진 칸을 알린다
def test_copy_human_fields_keeps_human_and_replaces_computed(tmp_path, capsys):
    sys.path.insert(0, str(RM / 'recipe_manager'))
    from main import RecipeManager
    saved = json.loads((RM / 'recipes' / '001_CHAIR_BENCH_V000_recipe.json').read_text(encoding='utf-8'))
    computed = strip_human_fields(saved)
    stale = copy.deepcopy(saved)
    stale['template']['composition']['LEG']['parts']['LEG_001'][0]['center_mm'][0] += 5    # 옛 계산 값은 버려져야 한다
    (tmp_path / 'old_recipe.json').write_text(json.dumps(stale, ensure_ascii=False), encoding='utf-8')
    manager = RecipeManager(tmp_path)
    assert manager.copy_human_fields(computed, tmp_path / 'old_recipe.json') == saved
    assert capsys.readouterr().out == ''
    assert manager.copy_human_fields(computed, tmp_path / 'none_recipe.json') == computed
    assert 'LEG.label' in capsys.readouterr().out


# 짝 확인 해시: 키 정렬 · 공백 없는 JSON 의 sha256 — 키 순서 · 들여쓰기가 바뀌어도(DB · MQTT 로 다시 써도) 같고, 값이 바뀌면 달라진다.
# 값은 고정(10/8 한세교) — 계산 방법을 바꾸면 이 시험이 깨져야 한다.
def test_recipe_sha256_is_fixed_canonical_json(bench):
    builder, recipe, _, _ = bench
    rewritten = json.loads(json.dumps(dict(reversed(list(recipe.items()))), indent=4))
    assert builder.calculate_recipe_sha256(rewritten) == builder.calculate_recipe_sha256(recipe)
    canonical = json.dumps(recipe, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode('utf-8')
    assert builder.calculate_recipe_sha256(recipe) == hashlib.sha256(canonical).hexdigest()
    assert builder.calculate_recipe_sha256(recipe) == BENCH_RECIPE_SHA256
    moved = copy.deepcopy(recipe)
    moved['template']['composition']['LEG']['parts']['LEG_001'][0]['center_mm'][0] += 1
    assert builder.calculate_recipe_sha256(moved) != BENCH_RECIPE_SHA256


BENCH_RECIPE_SHA256 = '63b1a5389e133f79af0396784ac2ceade86464da5d9071e71b27feda04eed2b2'   # 벤치 레시피(recipe/4.0 — stocks + 트리) 해시 — 계산 방법이 바뀌면 깨진다


# 저장 레시피로 다시 만든 레시피 · 조립 방법이 원본과 같다 (꼭짓점 → 치수 · 회전 · 받침 · 블록 이름). builder 는 사람 칸을 만들지 않는다
def test_boxes_round_trip_to_same_recipe(bench):
    builder, recipe, steps, hints = bench
    rebuilt = builder.make_recipe(make_boxes(recipe), recipe['design_id'],
                                  recipe['source_cad']['filename'], recipe['source_cad']['sha256'])
    assert rebuilt == strip_human_fields(recipe)
    placements = builder.make_placements(recipe, hints)
    assert placements['schema'] == 'placements/4.0' and placements['recipe_sha256'] == builder.calculate_recipe_sha256(recipe)
    assert placements['steps'] == steps
    assert placements['steps'][0]['part_id'] == '001_CHAIR_BENCH_V000_' + placements['steps'][0]['part_name']
    assert placements['steps'][-1]['supports'] == ['LEG_001_04', 'LEG_002_04']
    assert {s['grasp']: s['grasp_axis'] for s in placements['steps']} == {'FLAT_SHORT': 'WIDTH', 'FLAT_LONG': 'LENGTH'}


# 이름 규칙: 꼴이 틀림 · 옵션 두 개(역할 한 단어 + 옵션 0~1) · 같은 이름 두 번 · 번호가 위치 순서와 다름(아래층부터 · 앞 → 뒤 · 왼 → 오른) · 번호 빠짐 → 거부
@pytest.mark.parametrize('rename, message', [
    ({'LEG_001_01': 'B001'}, 'part name must be'),
    ({'SEAT_001_01': 'SEAT_ARM_REST_001_01'}, 'part name must be'),
    ({'LEG_001_01': 'LEG_001_02', 'LEG_001_02': 'LEG_001_02'}, 'used twice'),
    ({'LEG_001_01': 'LEG_001_02', 'LEG_001_02': 'LEG_001_01'}, 'bottom → top'),
    ({'SEAT_001_01': 'SEAT_001_03', 'SEAT_001_03': 'SEAT_001_01'}, 'front → rear'),
    ({'LEG_001_01': 'LEG_002_05', 'LEG_001_02': 'LEG_002_06', 'LEG_001_03': 'LEG_002_07', 'LEG_001_04': 'LEG_002_08',
      'LEG_002_01': 'LEG_001_01', 'LEG_002_02': 'LEG_001_02', 'LEG_002_03': 'LEG_001_03', 'LEG_002_04': 'LEG_001_04'}, 'part numbers must run'),
    ({'SEAT_001_03': 'SEAT_001_04'}, 'must run 01..03'),
])
def test_bad_part_names_rejected(bench, rename, message):
    builder, recipe, _, _ = bench
    boxes = [dict(b, part_name=rename.get(b['part_name'], b['part_name'])) for b in make_boxes(recipe)]
    with pytest.raises(ValueError, match=message):
        builder.make_recipe(boxes, 'X', 'x.dxf', '0')


# 옵션 하나(LEG_WHEEL)는 받는다 — 부품 번호는 역할 + 옵션 조합마다 따로
def test_one_option_name_accepted(bench):
    builder, recipe, _, _ = bench
    rename = {f'LEG_002_0{k}': f'LEG_WHEEL_001_0{k}' for k in range(1, 5)}
    boxes = [dict(b, part_name=rename.get(b['part_name'], b['part_name'])) for b in make_boxes(recipe)]
    rebuilt = builder.make_recipe(boxes, 'X', 'x.dxf', '0')
    assert {name for name, _ in iter_part_items(rebuilt)} >= set(rename.values())
    assert rebuilt['template']['composition']['LEG_WHEEL']['count'] == 1


# 형상 오류: 같은 자리에 두 블록(겹침) · 책상 아래 블록은 레시피를 만들 때 거부한다
def test_overlap_and_below_table_rejected(bench):
    builder, recipe, _, _ = bench
    boxes = make_boxes(recipe)
    with pytest.raises(ValueError, match='overlap'):
        builder.make_recipe(boxes + [dict(boxes[8], part_name='SEAT_002_01')], 'X', 'x.dxf', '0')
    sunk = [dict(b, vertices=[(x, y, z - 10) for x, y, z in b['vertices']]) for b in boxes]
    with pytest.raises(ValueError, match='below table'):
        builder.make_recipe(sunk, 'X', 'x.dxf', '0')


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
    builder, recipe, _, hints = bench
    hints = copy.deepcopy(hints)
    break_hints(hints)
    with pytest.raises(ValueError, match=message):
        builder.make_placements(recipe, hints)


# 옛 GRASP 속성(SIDE_25 · END_75)은 CAD 배치를 보고 6가지 이름으로, 수직 닫힘 축은 거부
def test_legacy_grasp_attributes(bench):
    builder, recipe, steps, hints = bench
    legacy = {k: dict(v, GRASP={'FLAT_SHORT': 'SIDE_25', 'FLAT_LONG': 'END_75'}[v['GRASP']]) for k, v in hints.items()}
    assert builder.make_placements(recipe, legacy)['steps'] == steps
    with pytest.raises(ValueError, match='vertical'):
        builder.make_placements(recipe, dict(legacy, LEG_001_01=dict(hints['LEG_001_01'], GRASP='THICKNESS')))

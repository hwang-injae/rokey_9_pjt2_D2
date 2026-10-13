# -*- coding: utf-8 -*-
"""변환기 ① 시험 (W110, E-69) — parts/4.0 → recipe/4.0 + placements/4.0.

기본 설계 4종 왕복(V-45: 레시피 → 변환기 ② → parts/4.0 → 변환기 ① → 같은 레시피), 블록 번호 매기기,
AI 실수 세 가지(역할 목록 · 부품 면 맞닿음 · 고른 잡기) 거부, AI 값을 대신 채우지 않음을 본다. ROS · 로봇 없이 돈다."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from d2_task.parts_to_recipe import PartsToRecipe, DesignRejected
from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument, recipe_sha256
from d2_task.recipe_to_parts import RecipeToParts

SRC = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load((SRC / 'd2_robot/d2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
BLOCK_MM = [v * 1000.0 for v in CFG['block_size_m']]
RECIPES = Path(__file__).parent / 'fixtures'          # 기본 설계 4종 시험용 사본(recipe/4.0 template.composition — 레시피 도구가 새 구조를 내기 전까지 이 사본이 정답)
FAMILY = {'001_CHAIR_BENCH_V000': 'chair', '002_CHAIR_BACK_V000': 'chair', '003_DESK_STAND_V000': 'desk', '004_DESK_PEDESTAL_V000': 'desk'}


def make_converter():
    """검사 묶음의 진짜 손가락 규칙(grasp_options)을 붙인 변환기 ①."""
    return PartsToRecipe(DesignChecker(CFG).grasp_options, BLOCK_MM)


def load_base(model_id):
    """기본 설계 레시피 두 파일 → (RecipeDocument, 변환기 ② 로 만든 parts/4.0)."""
    document = RecipeDocument.load(RECIPES, model_id)
    blocks = RecipeToParts(model_id, FAMILY[model_id], BLOCK_MM).convert(document.recipe, document.placements)
    return document, blocks


def rejected_details(request):
    """변환기가 DesignRejected 로 거부해야 하는 요청 → 이유(detail) 목록."""
    with pytest.raises(DesignRejected) as caught:
        make_converter().convert(request)
    return [e['detail'] for e in caught.value.errors]


@pytest.fixture
def bench():
    """벤치(001) parts/4.0 — 고치며 시험할 복사본."""
    return copy.deepcopy(load_base('001_CHAIR_BENCH_V000')[1])


def part_names(recipe):
    """recipe/4.0 template.composition 의 부품 이름(`<그룹>_<part>`, 예 LEG_001_01) 목록."""
    return [f'{g}_{it["part"]}' for c in recipe['template']['composition'].values() for g, its in c['parts'].items() for it in its]


# V-45 왕복: 기본 설계 4종 레시피 → parts/4.0 → 변환기 ① 이 저장된 레시피와 같다. 기본 설계에만 있는 칸(source_cad · template 의
# name · description · tags · 구성요소 revision · 부품 cad)은 AI·스캔 설계에 없어 빠진다(설계 초안 6절). 조립 방법은 새 구조 recipe_sha256 까지
# 맞아 task RecipeDocument 가 받는다
@pytest.mark.parametrize('model_id', FAMILY)
def test_round_trip_matches_cad_recipe(model_id):
    document, blocks = load_base(model_id)
    out = make_converter().convert(blocks)
    expected = copy.deepcopy(document.recipe)
    expected['source_cad'] = None
    expected['template'] = {'composition': {
        role: {'label': c['label'], 'count': c['count'],
               'parts': {g: [{k: v for k, v in it.items() if k != 'cad'} for it in its] for g, its in c['parts'].items()}}
        for role, c in expected['template']['composition'].items()}}
    assert out['recipe'] == expected
    assert out['placements']['steps'] == document.placements['steps']
    assert out['placements']['recipe_sha256'] == recipe_sha256(out['recipe']) != document.placements['recipe_sha256']     # 기본 설계와 구조가 달라 해시도 다르다
    RecipeDocument(out['recipe'], out['placements'])


# 변환기 ① 이 내는 레시피에는 기본 설계 전용 칸(최상위 parts · cad)이 없고, 구성요소는 label · count · parts 만 있다
def test_output_has_only_composition_fields(bench):
    out = make_converter().convert(bench)
    recipe = out['recipe']
    assert set(recipe) == {'schema', 'design_id', 'source_cad', 'stocks', 'template'} and set(recipe['template']) == {'composition'}
    assert {r: set(c) for r, c in recipe['template']['composition'].items()} == {'LEG': {'label', 'count', 'parts'}, 'SEAT': {'label', 'count', 'parts'}}
    assert [recipe['template']['composition'][r]['label'] for r in ('LEG', 'SEAT')] == ['다리', '좌판']
    assert [recipe['template']['composition'][r]['count'] for r in ('LEG', 'SEAT')] == [2, 1]
    assert all(set(it) == {'part', 'stock_id', 'center_mm', 'R'} for c in recipe['template']['composition'].values()
               for its in c['parts'].values() for it in its)
    assert 'size_mm' not in json.dumps(recipe['template']) and 'cad_handle' not in json.dumps(out['placements'])


# 새 builder(한세교가 template 구조를 직접 내는 경우)를 흉내: 옛 builder 출력을 새 구조로 바꿔 돌려주고, steps 에 옛 칸 stock_id · cad_handle 을 붙인다.
# 변환기는 AI·스캔 계약 칸(구성요소 label · count · parts, 부품 part · stock_id · center_mm · R)만 남기고, 해시를 다시 맞추고, steps 의 옛 칸을 뗀다.
# 기본 설계용 칸(name · description · tags · revision · cad)은 내보내지 않는다
def test_new_builder_output_filtered_to_generated_contract(bench):
    converter = make_converter()
    old = converter.builder
    seen = {}

    class NewBuilder:
        """make_recipe 가 금지 칸이 섞인 template 구조를 직접 내는 가짜 builder."""

        def make_recipe(self, vertices, *args):
            self.vertices, self.args = vertices, args
            recipe = converter.to_template_recipe(old.make_recipe(vertices, *args))
            template = recipe['template']
            template.update(name='시험', description='설명', tags=['태그'])
            for comp in template['composition'].values():
                comp['revision'] = {'lay': 'FLAT_STACK'}
                for items in comp['parts'].values():
                    for item in items:
                        item['cad'] = {'handle': '3E'}
            seen['recipe'] = copy.deepcopy(recipe)
            return recipe

        def make_placements(self, recipe, hints):
            placements = old.make_placements(old.make_recipe(self.vertices, *self.args), hints)
            return dict(placements, steps=[dict(s, stock_id='PART_001', cad_handle='3E') for s in placements['steps']])

    converter.builder = NewBuilder()
    before = copy.deepcopy(bench)
    out = converter.convert(bench)
    assert bench == before                                                  # 요청 입력 불변
    template = out['recipe']['template']
    assert set(template) == {'composition'}                                 # name · description · tags 없음
    assert all(set(c) == {'label', 'count', 'parts'} for c in template['composition'].values())       # revision 없음
    assert all(set(it) == {'part', 'stock_id', 'center_mm', 'R'} for c in template['composition'].values()
               for its in c['parts'].values() for it in its)               # cad 없음
    assert 'name' in seen['recipe']['template'] and 'revision' in next(iter(seen['recipe']['template']['composition'].values()))   # 가짜 builder 가 금지 칸을 실제로 냈다
    assert all('stock_id' not in s and 'cad_handle' not in s for s in out['placements']['steps'])
    assert out['placements']['recipe_sha256'] == recipe_sha256(out['recipe'])
    RecipeDocument(out['recipe'], out['placements'])


# 새 구조 입력을 직접 걸러도 입력 객체(builder 출력)는 바뀌지 않는다
def test_to_template_recipe_filters_new_tree_without_mutating():
    converter = make_converter()
    box = {'part_name': 'LEG_001_01', 'handle': None, 'vertices': [(x, y, z) for x in (0, 75) for y in (0, 25) for z in (0, 15)]}
    built = converter.to_template_recipe(converter.builder.make_recipe([box], 'X', None, None))
    built['template'].update(name='벤치', tags=['의자'])
    built['template']['composition']['LEG']['revision'] = {'lay': 'FLAT_STACK'}
    built['template']['composition']['LEG']['parts']['LEG_001'][0]['cad'] = {'handle': '3E'}
    before = copy.deepcopy(built)
    out = converter.to_template_recipe(built)
    kept = {k: before['template']['composition']['LEG']['parts']['LEG_001'][0][k] for k in ('part', 'stock_id', 'center_mm', 'R')}
    assert built == before and out is not built
    assert out['template'] == {'composition': {'LEG': {'label': '다리', 'count': 1, 'parts': {'LEG_001': [kept]}}}}


@pytest.mark.parametrize('role,label,expected', [
    ('LEG', None, '다리'), ('LEG', '', '다리'),
    ('LEG_WHEEL', None, '다리'), ('UNKNOWN', None, 'UNKNOWN'),
    ('LEG', '사용자 이름', '사용자 이름'),
])
def test_to_template_recipe_completes_optional_label(role, label, expected):
    """새 생산자의 label 생략·빈 값은 역할 이름표로 보완하며 기존 값과 입력 원본을 보존한다. 바깥 영향 없음."""
    comp = {'count': 0, 'parts': {}}
    if label is not None:
        comp['label'] = label
    built = {'schema': 'recipe/4.0', 'design_id': 'X', 'source_cad': None,
             'template': {'composition': {role: comp}}}
    before = copy.deepcopy(built)
    out = make_converter().to_template_recipe(built)
    assert out['template']['composition'][role] == {'label': expected, 'count': 0, 'parts': {}}
    assert built == before


# to_template_recipe 는 입력(builder 출력)을 바꾸지 않는다
def test_to_template_recipe_does_not_mutate(bench):
    converter = make_converter()
    document, _ = load_base('001_CHAIR_BENCH_V000')
    built = {'schema': 'recipe/4.0', 'design_id': 'X', 'source_cad': None, 'stocks': [{'stock_id': 'PART_001', 'size_mm': BLOCK_MM}],
             'parts': [{'part_name': 'LEG_001_01', 'stock_id': 'PART_001', 'center_mm': [0.0, 0.0, 7.5],
                        'R': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]}]}
    before = copy.deepcopy(built)
    out = converter.to_template_recipe(built)
    assert built == before and out['template']['composition']['LEG']['parts']['LEG_001'][0]['part'] == '01'


# 블록 번호는 코드가: 벤치 → LEG_001_01~04(왼 다리) · LEG_002_01~04(오른 다리) · SEAT_001_01~03. AI 의 part 값(묶음 표시)은 번호가 아니다
def test_bench_block_names_from_positions(bench):
    for b in bench['parts']:
        b['group'] = {1: 7, 2: 3}[b['group']] if b['role'] == 'LEG' else 5      # 묶음 값만 바꿔도 이름은 위치로 같다
    names = part_names(make_converter().convert(bench)['recipe'])
    assert sorted(names) == [f'LEG_001_0{k}' for k in range(1, 5)] + [f'LEG_002_0{k}' for k in range(1, 5)] + [f'SEAT_001_0{k}' for k in range(1, 4)]


# 생성 설계 ID(<Template ID>_V<3자리>, E-84)는 레시피 model_id 그대로, part_id 는 대문자 — task RecipeDocument 와 같은 규칙(IRD 2장)
def test_generated_design_id_part_id_upper(bench):
    bench['design_id'] = '001_CHAIR_BENCH_V001'
    out = make_converter().convert(bench)
    assert out['recipe']['design_id'] == '001_CHAIR_BENCH_V001'
    assert out['placements']['steps'][0]['part_id'].startswith('001_CHAIR_BENCH_V001_LEG_')
    RecipeDocument(out['recipe'], out['placements'])


# AI 값은 그대로: 순서 · 단계 · 잡기를 옮기고, 둘 다 되는 블록에서 AI 가 고른 긴 쪽도 짧은 쪽으로 바꾸지 않는다(공식 채움 없음)
def test_ai_values_kept_as_written(bench):
    options = DesignChecker(CFG).grasp_options(bench['parts'])
    both = next(b for b in bench['parts'] if {'FLAT_SHORT', 'FLAT_LONG'} <= set(options[b['sequence']]))
    both['grasp'] = 'FLAT_LONG'
    steps = {s['sequence']: s for s in make_converter().convert(bench)['placements']['steps']}
    assert steps[both['sequence']]['grasp'] == 'FLAT_LONG'
    assert all((steps[b['sequence']]['stage'], steps[b['sequence']]['grasp']) == (b['stage'], b['grasp']) for b in bench['parts'])


# 칸이 빠지면 대신 채우지 않고 거부 — 어느 블록의 어느 칸인지 적는다
@pytest.mark.parametrize('key', ['role', 'group', 'stage', 'grasp'])
def test_missing_ai_field_rejected(bench, key):
    del bench['parts'][2][key]
    assert any(key in d and '3번' in d for d in rejected_details(bench))


# ① 역할 · 옵션이 목록에 없으면 거부(있는 것을 알려 줌). 목록에 있는 옵션(LEG_WHEEL)은 받는다
def test_role_not_in_list_rejected(bench):
    for b in bench['parts']:
        if b['role'] == 'SEAT':
            b['role'] = 'SEATING'
    details = rejected_details(bench)
    assert len(details) == 1 and 'SEATING' in details[0] and 'SEAT' in details[0]
    for b in bench['parts']:
        b['role'] = {'SEATING': 'SEAT', 'LEG': 'LEG_ROCKER'}[b['role']]
    assert any('ROCKER' in d for d in rejected_details(bench))
    for b in bench['parts']:
        b['role'] = 'LEG_WHEEL' if b['role'] == 'LEG_ROCKER' else b['role']
    recipe = make_converter().convert(bench)['recipe']
    assert any(n.startswith('LEG_WHEEL_002_') for n in part_names(recipe))
    assert recipe['template']['composition']['LEG_WHEEL']['label'] == '다리'       # 옵션 역할은 기본 역할의 화면 이름표를 쓴다
    RecipeDocument(*[make_converter().convert(bench)[k] for k in ('recipe', 'placements')])


# ② 떨어진 블록을 한 부품으로 묶으면 거부 — 벤치 왼 · 오른 다리를 같은 part 로
def test_separate_legs_in_one_group_rejected(bench):
    for b in bench['parts']:
        if b['role'] == 'LEG':
            b['group'] = 1
    details = rejected_details(bench)
    assert len(details) == 1 and 'LEG group 1' in details[0] and '맞닿아 있지 않다' in details[0]


# ② 맞닿음 = 면 접촉: 옆면이 붙으면 한 부품, 모서리만 닿으면(대각선 자리) 한 부품이 아니다
def test_face_contact_only():
    converter = make_converter()
    a = converter.calculate_box({'x': 0, 'y': 0, 'z': 0, 'ori': 'x'})                    # 75 × 25 × 15
    side = converter.calculate_box({'x': 0, 'y': 25, 'z': 0, 'ori': 'x'})                # 옆면 붙음
    on_top = converter.calculate_box({'x': 0, 'y': 0, 'z': 15, 'ori': 'y'})              # 위에 엇갈려 얹음
    corner = converter.calculate_box({'x': 75, 'y': 25, 'z': 0, 'ori': 'x'})             # 모서리(세로선)만 닿음
    edge_up = converter.calculate_box({'x': 0, 'y': 25, 'z': 15, 'ori': 'x'})            # 위층 옆 — 모서리(가로선)만 닿음
    assert converter.is_face_contact(a, side) and converter.is_face_contact(a, on_top)
    assert not converter.is_face_contact(a, corner) and not converter.is_face_contact(a, edge_up)


# ③ AI 가 고른 잡기로 손가락이 안 들어가면 거부하고 가능한 잡기를 알려 준다 · 놓인 자세와 다른 잡기(눕힌 블록에 STAND)도 거부
def test_blocked_or_wrong_state_grasp_rejected(bench):
    options = DesignChecker(CFG).grasp_options(bench['parts'])
    only = next(b for b in bench['parts'] if len(options[b['sequence']]) == 1)
    allowed = options[only['sequence']][0]
    only['grasp'] = 'FLAT_LONG' if allowed == 'FLAT_SHORT' else 'FLAT_SHORT'
    details = rejected_details(bench)
    assert len(details) == 1 and f'{only["sequence"]}번' in details[0] and f'가능: {allowed}' in details[0]
    only['grasp'] = allowed
    bench['parts'][0]['grasp'] = 'STAND_SHORT'
    assert any('1번 grasp STAND_SHORT' in d for d in rejected_details(bench))


# 실수 여러 개는 한 번에 모아 돌려준다(재생성 횟수를 줄이려고)
def test_all_mistakes_reported_together(bench):
    bench['parts'][0]['role'] = 'FOOT'
    for b in bench['parts']:
        if b['role'] == 'SEAT':
            b['group'] = b['sequence']                       # 좌판 3장 → 각자 다른 part(한 장짜리 부품, 맞닿음 문제 없음)
    bench['parts'][-1]['grasp'] = 'EDGE_LONG'
    details = rejected_details(bench)
    assert any('FOOT' in d for d in details) and any('EDGE_LONG' in d for d in details)


# 형식이 틀린 요청(schema · order 빠짐)은 레시피를 만들지 않고 거부
def test_bad_request_rejected(bench):
    assert rejected_details(dict(bench, schema='blocks/2'))
    bench['parts'][3]['sequence'] = 99
    assert any('sequence' in d for d in rejected_details(bench))


# 역할 목록 파일: 작명 규칙 v3 3장 — 역할 7개 + 옵션 WHEEL, 이름은 대문자 한 단어(밑줄 없음), 뜻이 비어 있지 않음
def test_role_list_file():
    data = json.loads((SRC / 'd2_task/d2_task/roles.json').read_text(encoding='utf-8'))
    assert set(data['roles']) == {'LEG', 'SEAT', 'BACK', 'BEAM', 'TOP', 'BASE', 'COLUMN'} and set(data['options']) == {'WHEEL'}
    assert all(name.isalpha() and name.isupper() and meaning for name, meaning in {**data['roles'], **data['options']}.items())


# 검사 묶음에 붙이면(task_node 가 할 연결) 기본 설계 4종의 check_design 이 합격 + recipe · placements 를 돌려준다
@pytest.mark.parametrize('model_id', FAMILY)
def test_attached_to_design_checker(model_id):
    checker = DesignChecker(CFG)
    checker.parts_to_recipe = PartsToRecipe(checker.grasp_options, BLOCK_MM).convert
    document, blocks = load_base(model_id)
    ok, reason, text = checker.handle_json(json.dumps(dict(blocks, base_recipe=document.recipe)))
    result = json.loads(text)
    assert (ok, reason, result['ok']) == (True, '', True), result['errors']
    assert result['recipe']['schema'] == 'recipe/4.0' and result['placements']['schema'] == 'placements/4.0'

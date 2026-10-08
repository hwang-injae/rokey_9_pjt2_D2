# -*- coding: utf-8 -*-
"""변환기 ① 시험 (W110, E-69) — blocks/2.0 → recipe/2.0 + placements/2.0.

기본 설계 4종 왕복(V-45: 레시피 → 변환기 ② → blocks/2.0 → 변환기 ① → 같은 레시피), 블록 번호 매기기,
AI 실수 세 가지(역할 목록 · 부품 면 맞닿음 · 고른 잡기) 거부, AI 값을 대신 채우지 않음을 본다. ROS · 로봇 없이 돈다."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from d2_task.blocks_to_recipe import BlocksToRecipe, DesignRejected
from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_blocks import RecipeToBlocks

SRC = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load((SRC / 'd2_robot/d2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
BLOCK_MM = [v * 1000.0 for v in CFG['block_size_m']]
RECIPES = SRC / 'recipe_manager' / 'recipes'          # 레시피 도구가 CAD 에서 만든 기본 설계 4종(한세교 — 정답)
FAMILY = {'001_CHAIR_BENCH': 'chair', '002_CHAIR_BACK': 'chair', '003_DESK_STAND': 'desk', '004_DESK_PEDESTAL': 'desk'}


def make_converter():
    """검사 묶음의 진짜 손가락 규칙(grasp_options)을 붙인 변환기 ①."""
    return BlocksToRecipe(DesignChecker(CFG).grasp_options, BLOCK_MM)


def load_base(model_id):
    """기본 설계 레시피 두 파일 → (RecipeDocument, 변환기 ② 로 만든 blocks/2.0)."""
    document = RecipeDocument.load(RECIPES, model_id)
    blocks = RecipeToBlocks(model_id, FAMILY[model_id], BLOCK_MM).convert(document.recipe, document.placements)
    return document, blocks


def rejected_details(request):
    """변환기가 DesignRejected 로 거부해야 하는 요청 → 이유(detail) 목록."""
    with pytest.raises(DesignRejected) as caught:
        make_converter().convert(request)
    return [e['detail'] for e in caught.value.errors]


@pytest.fixture
def bench():
    """벤치(001) blocks/2.0 — 고치며 시험할 복사본."""
    return copy.deepcopy(load_base('001_CHAIR_BENCH')[1])


# V-45 왕복: 기본 설계 4종 레시피 → blocks/2.0 → 변환기 ① 이 저장된 레시피와 같다(CAD 대응 칸 source_cad · cad.handle 만 빠짐),
# 조립 방법은 recipe_sha256 까지 맞아 task RecipeDocument 가 받는다
@pytest.mark.parametrize('model_id', FAMILY)
def test_round_trip_matches_cad_recipe(model_id):
    document, blocks = load_base(model_id)
    out = make_converter().convert(blocks)
    expected = copy.deepcopy(document.recipe)
    expected['source_cad'] = None
    for b in expected['blocks']:
        b.pop('cad')
    assert out['recipe'] == expected
    assert out['placements']['steps'] == document.placements['steps']
    RecipeDocument(out['recipe'], out['placements'])


# 블록 번호는 코드가: 벤치 → LEG_001_01~04(왼 다리) · LEG_002_01~04(오른 다리) · SEAT_001_01~03. AI 의 part 값(묶음 표시)은 번호가 아니다
def test_bench_block_names_from_positions(bench):
    for b in bench['blocks']:
        b['part'] = {1: 7, 2: 3}[b['part']] if b['role'] == 'LEG' else 5      # 묶음 값만 바꿔도 이름은 위치로 같다
    names = [b['block'] for b in make_converter().convert(bench)['recipe']['blocks']]
    assert sorted(names) == [f'LEG_001_0{k}' for k in range(1, 5)] + [f'LEG_002_0{k}' for k in range(1, 5)] + [f'SEAT_001_0{k}' for k in range(1, 4)]


# 생성 설계 ID(소문자 · 점)는 레시피 model_id 그대로, block_id 는 대문자 — task RecipeDocument 와 같은 규칙(IRD 2장)
def test_generated_design_id_block_id_upper(bench):
    bench['design_id'] = 'chair_v1.1'
    out = make_converter().convert(bench)
    assert out['recipe']['model_id'] == 'chair_v1.1'
    assert out['placements']['steps'][0]['block_id'].startswith('CHAIR_V1.1_')
    RecipeDocument(out['recipe'], out['placements'])


# AI 값은 그대로: 순서 · 단계 · 잡기를 옮기고, 둘 다 되는 블록에서 AI 가 고른 긴 쪽도 짧은 쪽으로 바꾸지 않는다(공식 채움 없음)
def test_ai_values_kept_as_written(bench):
    options = DesignChecker(CFG).grasp_options(bench['blocks'])
    both = next(b for b in bench['blocks'] if {'FLAT_SHORT', 'FLAT_LONG'} <= set(options[b['order']]))
    both['grasp'] = 'FLAT_LONG'
    steps = {s['sequence']: s for s in make_converter().convert(bench)['placements']['steps']}
    assert steps[both['order']]['grasp'] == 'FLAT_LONG'
    assert all((steps[b['order']]['stage'], steps[b['order']]['grasp']) == (b['stage'], b['grasp']) for b in bench['blocks'])


# 칸이 빠지면 대신 채우지 않고 거부 — 어느 블록의 어느 칸인지 적는다
@pytest.mark.parametrize('key', ['role', 'part', 'stage', 'grasp'])
def test_missing_ai_field_rejected(bench, key):
    del bench['blocks'][2][key]
    assert any(key in d and '3번' in d for d in rejected_details(bench))


# ① 역할 · 옵션이 목록에 없으면 거부(있는 것을 알려 줌). 목록에 있는 옵션(LEG_WHEEL)은 받는다
def test_role_not_in_list_rejected(bench):
    for b in bench['blocks']:
        if b['role'] == 'SEAT':
            b['role'] = 'SEATING'
    details = rejected_details(bench)
    assert len(details) == 1 and 'SEATING' in details[0] and 'SEAT' in details[0]
    for b in bench['blocks']:
        b['role'] = {'SEATING': 'SEAT', 'LEG': 'LEG_ROCKER'}[b['role']]
    assert any('ROCKER' in d for d in rejected_details(bench))
    for b in bench['blocks']:
        b['role'] = 'LEG_WHEEL' if b['role'] == 'LEG_ROCKER' else b['role']
    assert any(b['block'].startswith('LEG_WHEEL_002_') for b in make_converter().convert(bench)['recipe']['blocks'])


# ② 떨어진 블록을 한 부품으로 묶으면 거부 — 벤치 왼 · 오른 다리를 같은 part 로
def test_separate_legs_in_one_part_rejected(bench):
    for b in bench['blocks']:
        if b['role'] == 'LEG':
            b['part'] = 1
    details = rejected_details(bench)
    assert len(details) == 1 and 'LEG part 1' in details[0] and '맞닿아 있지 않다' in details[0]


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
    options = DesignChecker(CFG).grasp_options(bench['blocks'])
    only = next(b for b in bench['blocks'] if len(options[b['order']]) == 1)
    allowed = options[only['order']][0]
    only['grasp'] = 'FLAT_LONG' if allowed == 'FLAT_SHORT' else 'FLAT_SHORT'
    details = rejected_details(bench)
    assert len(details) == 1 and f'{only["order"]}번' in details[0] and f'가능: {allowed}' in details[0]
    only['grasp'] = allowed
    bench['blocks'][0]['grasp'] = 'STAND_SHORT'
    assert any('1번 grasp STAND_SHORT' in d for d in rejected_details(bench))


# 실수 여러 개는 한 번에 모아 돌려준다(재생성 횟수를 줄이려고)
def test_all_mistakes_reported_together(bench):
    bench['blocks'][0]['role'] = 'FOOT'
    for b in bench['blocks']:
        if b['role'] == 'SEAT':
            b['part'] = b['order']                       # 좌판 3장 → 각자 다른 part(한 장짜리 부품, 맞닿음 문제 없음)
    bench['blocks'][-1]['grasp'] = 'EDGE_LONG'
    details = rejected_details(bench)
    assert any('FOOT' in d for d in details) and any('EDGE_LONG' in d for d in details)


# 형식이 틀린 요청(schema · order 빠짐)은 레시피를 만들지 않고 거부
def test_bad_request_rejected(bench):
    assert rejected_details(dict(bench, schema='blocks/1'))
    bench['blocks'][3]['order'] = 99
    assert any('order' in d for d in rejected_details(bench))


# 역할 목록 파일: 작명 규칙 v3 3장 — 역할 7개 + 옵션 WHEEL, 이름은 대문자 한 단어(밑줄 없음), 뜻이 비어 있지 않음
def test_role_list_file():
    data = json.loads((SRC / 'd2_task/d2_task/roles.json').read_text(encoding='utf-8'))
    assert set(data['roles']) == {'LEG', 'SEAT', 'BACK', 'BEAM', 'TOP', 'BASE', 'COLUMN'} and set(data['options']) == {'WHEEL'}
    assert all(name.isalpha() and name.isupper() and meaning for name, meaning in {**data['roles'], **data['options']}.items())


# 검사 묶음에 붙이면(task_node 가 할 연결) 기본 설계 4종의 check_design 이 합격 + recipe · placements 를 돌려준다
@pytest.mark.parametrize('model_id', FAMILY)
def test_attached_to_design_checker(model_id):
    checker = DesignChecker(CFG)
    checker.blocks_to_recipe = BlocksToRecipe(checker.grasp_options, BLOCK_MM).convert
    ok, reason, text = checker.handle_json(json.dumps(load_base(model_id)[1]))
    result = json.loads(text)
    assert (ok, reason, result['ok']) == (True, '', True), result['errors']
    assert result['recipe']['schema'] == 'recipe/2.0' and result['placements']['schema'] == 'placements/2.0'

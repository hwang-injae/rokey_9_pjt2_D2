# -*- coding: utf-8 -*-
"""W110 연결 시험 — 실제 변환기 ①을 검사 묶음에 붙인 check_design 서비스 (ROS 없이, task_node 가 하는 연결과 같은 방식).

변환기 ① 자체의 시험은 test_parts_to_recipe.py(한세교). 여기서는 task 쪽 연결만 본다: 합격 → recipe · placements 응답,
AI 실수(DesignRejected) → CHECK_FAILED(success true, 재생성 대상), 그 밖의 예외 → ERROR(success false).
"""
import copy
import json

import pytest

from d2_task.parts_to_recipe import PartsToRecipe
from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_parts import RecipeToParts
from test_design_checker import CFG, NO_TEMPLATE
from test_recipe_to_parts import BLOCK_MM, IDS, RECIPES


def attached(**kw):
    """task_node 와 같은 방식으로 변환기 ①을 붙인 검사 묶음(진짜 Template 검사 포함). kw 는 격리 시험이 대역을 넣을 때만 쓴다."""
    checker = DesignChecker(CFG, **kw)
    checker.parts_to_recipe = PartsToRecipe(checker.grasp_options, BLOCK_MM).convert
    return checker


def blocks_of(model_id):
    doc = RecipeDocument.load(RECIPES, model_id)
    _, family, _ = IDS[model_id]
    request = RecipeToParts(model_id, family, BLOCK_MM).convert(doc.recipe, doc.placements)     # 실제 설계의 design_id = 모델 ID(변환기 ①이 model_id 로 쓴다)
    return doc, dict(request, base_recipe=copy.deepcopy(doc.recipe))        # 실제 요청처럼 그 Template 의 V000 레시피를 통째로 싣는다


@pytest.mark.parametrize('model_id', IDS)
def test_변환기2의_출력이_check_design을_거쳐_레시피_파일과_같은_recipe_placements로_돌아온다(model_id):
    """V-45 왕복: 레시피 → parts/4.0(②) → check_design(검사 + 변환기 ①) → 처음과 같은 블록 이름 · 중심 · 회전 · 조립 방법이 돌아온다."""
    doc, blocks = blocks_of(model_id)
    ok, reason, text = attached().handle_json(json.dumps(blocks))
    result = json.loads(text)
    assert (ok, reason, result['ok']) == (True, '', True), result['errors']
    assert result['schema'] == 'check_result/3.0'
    # CAD 에서만 오는 칸(cad.handle · source_cad)과 그 위에 계산한 recipe_sha256 은 parts/4.0 에 없으므로 비교에서 뺀다
    keep = lambda r: sorted((f'{g}_{x["part"]}', next(stock['size_mm'] for stock in r['stocks'] if stock['stock_id'] == x['stock_id']), x['center_mm'], x['R'])
                            for c in r['template']['composition'].values() for g, xs in c['parts'].items() for x in xs)
    assert keep(result['recipe']) == keep(doc.recipe)
    steps = lambda p: [{k: v for k, v in st.items()} for st in p['steps']]
    assert steps(result['placements']) == steps(doc.placements)
    assert RecipeDocument(result['recipe'], result['placements']).placements['recipe_sha256'] == result['placements']['recipe_sha256']


def test_AI가_역할을_틀리면_CHECK_FAILED로_돌리고_서비스는_성공이다():
    _, blocks = blocks_of('001_CHAIR_BENCH_V000')
    blocks['parts'][0]['role'] = 'NOPE'
    ok, reason, text = attached(template_checker=NO_TEMPLATE).handle_json(json.dumps(blocks))     # 변환기 ①의 역할 목록 검사만 보려고 21번은 대역
    result = json.loads(text)
    assert (ok, reason, result['ok']) == (True, '', False)                      # 서비스는 처리했고 설계가 불합격 — 생성 쪽이 재생성한다
    assert result['errors'] and {e['reason'] for e in result['errors']} == {'CHECK_FAILED'}
    assert 'recipe' not in result and 'placements' not in result


def test_변환기가_찾은_실수가_여러_개면_한_번에_모두_errors에_담는다():
    _, blocks = blocks_of('001_CHAIR_BENCH_V000')
    blocks['parts'][0]['role'] = 'NOPE'
    blocks['parts'][1]['role'] = 'NADA'
    result = json.loads(attached(template_checker=NO_TEMPLATE).handle_json(json.dumps(blocks))[2])
    assert len(result['errors']) >= 2 and all(e['reason'] == 'CHECK_FAILED' and e['detail'] for e in result['errors'])
    assert all(e['kind'] == 'ai_fields' for e in result['errors'])             # 변환기 ①이 찾은 AI 칸 실수는 ai_fields


def test_변환기의_그_밖의_예외는_ERROR로_남는다():
    _, blocks = blocks_of('001_CHAIR_BENCH_V000')
    checker = DesignChecker(CFG, parts_to_recipe=lambda b: 1 / 0)
    ok, reason, text = checker.handle_json(json.dumps(blocks))
    assert (ok, reason) == (False, 'ERROR') and json.loads(text)['errors'][0]['reason'] == 'ERROR'

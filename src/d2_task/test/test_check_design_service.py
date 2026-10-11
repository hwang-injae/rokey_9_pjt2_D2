# -*- coding: utf-8 -*-
"""W110 연결 시험 — 실제 변환기 ①을 검사 묶음에 붙인 check_design 서비스 (ROS 없이, task_node 가 하는 연결과 같은 방식).

변환기 ① 자체의 시험은 test_blocks_to_recipe.py(한세교). 여기서는 task 쪽 연결만 본다: 합격 → recipe · placements 응답,
AI 실수(DesignRejected) → CHECK_FAILED(success true, 재생성 대상), 그 밖의 예외 → ERROR(success false).
"""
import json

import pytest

from d2_task.blocks_to_recipe import BlocksToRecipe
from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_blocks import RecipeToBlocks
from test_design_checker import CFG
from test_recipe_to_blocks import BLOCK_MM, IDS, RECIPES


def attached():
    """task_node 와 같은 방식으로 변환기 ①을 붙인 검사 묶음."""
    checker = DesignChecker(CFG)
    checker.blocks_to_recipe = BlocksToRecipe(checker.grasp_options, BLOCK_MM).convert
    return checker


def blocks_of(model_id):
    doc = RecipeDocument.load(RECIPES, model_id)
    _, family, _ = IDS[model_id]
    return doc, RecipeToBlocks(model_id, family, BLOCK_MM).convert(doc.recipe, doc.placements)     # 실제 설계의 design_id = 모델 ID(변환기 ①이 model_id 로 쓴다)


@pytest.mark.parametrize('model_id', IDS)
def test_변환기2의_출력이_check_design을_거쳐_레시피_파일과_같은_recipe_placements로_돌아온다(model_id):
    """V-45 왕복: 레시피 → blocks/2.0(②) → check_design(검사 + 변환기 ①) → 처음과 같은 블록 이름 · 중심 · 회전 · 조립 방법이 돌아온다."""
    doc, blocks = blocks_of(model_id)
    ok, reason, text = attached().handle_json(json.dumps(blocks))
    result = json.loads(text)
    assert (ok, reason, result['ok']) == (True, '', True), result['errors']
    assert result['schema'] == 'check_result/2.0'
    # CAD 에서만 오는 칸(cad.handle · source_cad)과 그 위에 계산한 recipe_sha256 은 blocks/2.0 에 없으므로 비교에서 뺀다
    keep = lambda r: [(x['block'], x['part_id'], x['center_mm'], x['R']) for x in r['blocks']]
    assert keep(result['recipe']) == keep(doc.recipe)
    steps = lambda p: [{k: v for k, v in st.items()} for st in p['steps']]
    assert steps(result['placements']) == steps(doc.placements)
    assert RecipeDocument(result['recipe'], result['placements']).placements['recipe_sha256'] == result['placements']['recipe_sha256']


def test_AI가_역할을_틀리면_CHECK_FAILED로_돌리고_서비스는_성공이다():
    _, blocks = blocks_of('001_CHAIR_BENCH_V000')
    blocks['blocks'][0]['role'] = 'NOPE'
    ok, reason, text = attached().handle_json(json.dumps(blocks))
    result = json.loads(text)
    assert (ok, reason, result['ok']) == (True, '', False)                      # 서비스는 처리했고 설계가 불합격 — 생성 쪽이 재생성한다
    assert result['errors'] and {e['reason'] for e in result['errors']} == {'CHECK_FAILED'}
    assert 'recipe' not in result and 'placements' not in result


def test_변환기가_찾은_실수가_여러_개면_한_번에_모두_errors에_담는다():
    _, blocks = blocks_of('001_CHAIR_BENCH_V000')
    blocks['blocks'][0]['role'] = 'NOPE'
    blocks['blocks'][1]['role'] = 'NADA'
    result = json.loads(attached().handle_json(json.dumps(blocks))[2])
    assert len(result['errors']) >= 2 and all(e['reason'] == 'CHECK_FAILED' and e['detail'] for e in result['errors'])


def test_변환기의_그_밖의_예외는_ERROR로_남는다():
    _, blocks = blocks_of('001_CHAIR_BENCH_V000')
    checker = DesignChecker(CFG, blocks_to_recipe=lambda b: 1 / 0)
    ok, reason, text = checker.handle_json(json.dumps(blocks))
    assert (ok, reason) == (False, 'ERROR') and json.loads(text)['errors'][0]['reason'] == 'ERROR'

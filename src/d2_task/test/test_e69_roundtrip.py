# -*- coding: utf-8 -*-
"""E-69 새 레시피 4종 왕복 시험(task 쪽, ROS 없이): 두 문서 읽기 → 진행표 → 변환기 ② blocks/2.0 → 검사 묶음 → 작명 규칙 v3 일치.

변환기 ①(W110, 한세교, 10/10 오전)은 아직 없어서 '다시 레시피로' 방향은 이름 · 번호가 같아지는지로 확인한다: blocks/2.0 의 role · part 와 위치 순서로
작명 규칙 v3 이 부품 번호 · 블록 번호를 다시 매기면 레시피의 블록 이름과 같아야 한다(V-45 왕복의 task 쪽 절반).
"""
import pytest

from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_blocks import RecipeToBlocks
from d2_task.task_planner import TaskPlanner
from test_design_checker import CFG, DESIGNS
from test_recipe_to_blocks import BLOCK_MM, IDS, RECIPES


def blocks_of(model_id):
    doc = RecipeDocument.load(RECIPES, model_id)
    design_id, family, _ = IDS[model_id]
    return doc, RecipeToBlocks(design_id, family, BLOCK_MM).convert(doc.recipe, doc.placements)


@pytest.mark.parametrize('model_id', IDS)
def test_4종_읽기_진행표_검사가_이어진다(model_id):
    doc, blocks = blocks_of(model_id)
    assert doc.placements['recipe_sha256'] and doc.recipe['model_id'] == doc.placements['model_id'] == model_id
    planner = TaskPlanner(CFG, doc.recipe, doc.placements)
    assert len(planner.blocks) == len(blocks['blocks']) == IDS[model_id][2]
    assert all(b['block_id'].startswith(model_id + '_') for b in planner.blocks)
    result = DesignChecker(CFG).check(blocks)
    assert result['ok'] and result['errors'] == [] and result['schema'] == 'check_result/2.0'
    assert result['min_margin_mm'] == DESIGNS[model_id][2]                 # V-45: 12.5 / 12.5 / 7.5 / 12.5


@pytest.mark.parametrize('model_id', IDS)
def test_blocks_2_0의_role_part로_번호를_다시_매기면_레시피_블록_이름과_같다(model_id):
    """작명 규칙 v3: 부품 번호 = blocks/2.0 의 part 묶음, 블록 번호 = 그 부품 안에서 아래층부터 · 앞→뒤 · 왼→오른. 코드(변환기 ①)가 하는 일의 task 쪽 확인."""
    doc, blocks = blocks_of(model_id)
    name_of = {s['sequence']: s['block'] for s in doc.placements['steps']}
    groups = {}
    for b in blocks['blocks']:
        groups.setdefault((b['role'], b['part']), []).append(b)
    for (role, part), items in groups.items():
        ordered = sorted(items, key=lambda b: (b['z'], b['y'], b['x']))
        for k, b in enumerate(ordered, 1):
            assert name_of[b['order']] == f'{role}_{part:03d}_{k:02d}', (model_id, b)


@pytest.mark.parametrize('model_id', IDS)
def test_같은_역할의_부품_번호는_1부터_빈틈없이_이어진다(model_id):
    _, blocks = blocks_of(model_id)
    parts = {}
    for b in blocks['blocks']:
        parts.setdefault(b['role'], set()).add(b['part'])
    assert all(sorted(p) == list(range(1, len(p) + 1)) for p in parts.values())


@pytest.mark.parametrize('model_id', IDS)
def test_blocks_2_0_왕복으로_같은_구조가_다시_나온다(model_id):
    """blocks/2.0 → (이름 · 위치 규칙으로 되돌린 구조) → 변환기 ② 가 같은 blocks/2.0 을 다시 낸다 — 위치 · 방향 계산이 흔들리지 않는다."""
    doc, blocks = blocks_of(model_id)
    _, again = blocks_of(model_id)
    assert blocks == again
    assert all(set(b) == {'order', 'x', 'y', 'z', 'ori', 'role', 'part', 'stage', 'grasp'} for b in blocks['blocks'])

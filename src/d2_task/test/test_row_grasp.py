# -*- coding: utf-8 -*-
"""연짝 잡기 규칙(row_grasp) 시험 — 10/13 21시 PL: 연짝(같은 층에 긴 면을 맞댄 눕힌 블록)은 모두 FLAT_LONG. ROS · d2_motion 없이."""
import csv
import json
from pathlib import Path

import pytest

from d2_task.recipe_to_parts import RecipeToParts
from d2_task.row_grasp import fix_row_grasps, row_orders

BLOCK_MM = [75.0, 25.0, 15.0]
RECIPES = Path(__file__).resolve().parents[2] / 'recipe_manager' / 'recipes'
# 기본 설계 4개의 연짝(10/13 박진용 — 검사 묶음 grasp_options 로 첫 블록 FLAT_LONG 손가락이 들어감을 확인)
ROWS = {'001_CHAIR_BENCH_V000': {'SEAT_001_01', 'SEAT_001_02', 'SEAT_001_03'},
        '002_CHAIR_BACK_V000': {'SEAT_001_01', 'SEAT_001_02', 'SEAT_001_03'},
        '003_DESK_STAND_V000': {'TOP_001_01', 'TOP_001_02', 'TOP_001_03'},
        '004_DESK_PEDESTAL_V000': {'BASE_001_01', 'BASE_001_02', 'BASE_001_03', 'TOP_001_01', 'TOP_001_02', 'TOP_001_03'}}


def part(order, x, y, z, ori='x', grasp='FLAT_SHORT'):
    """parts/3.0 블록 하나(mm, 설계 좌표)."""
    return {'order': order, 'x': x, 'y': y, 'z': z, 'ori': ori, 'role': 'TOP', 'group': 1, 'stage': 1, 'grasp': grasp}


def base_parts(model_id):
    """기본 설계 레시피 두 파일 → parts/3.0 parts 와 order → 블록 이름."""
    recipe = json.loads((RECIPES / f'{model_id}_recipe.json').read_text(encoding='utf-8'))
    with open(RECIPES / f'{model_id}_placements.csv', encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))
    steps = [{'part_name': r['part_name'], 'part_id': r['part_id'], 'sequence': int(r['sequence']), 'stage': int(r['stage']),
              'grasp': r['grasp'], 'grasp_axis': r['grasp_axis'], 'supports': [x for x in r['supports'].split(';') if x]} for r in rows]
    placements = {'schema': 'placements/3.0', 'model_id': model_id, 'recipe_sha256': rows[0]['recipe_sha256'], 'steps': steps}
    parts = RecipeToParts(model_id, 'test', BLOCK_MM).convert(recipe, placements)['parts']
    return parts, {s['sequence']: s['part_name'] for s in steps}


def test_연짝_셋은_첫_블록도_FLAT_LONG():
    parts = [part(1, 0, -25, 0), part(2, 0, 0, 0, grasp='FLAT_LONG'), part(3, 0, 25, 0, grasp='FLAT_LONG')]
    assert row_orders(parts, BLOCK_MM) == {1, 2, 3}
    fixed, changed = fix_row_grasps(parts, BLOCK_MM)
    assert changed == [1] and [p['grasp'] for p in fixed] == ['FLAT_LONG'] * 3
    assert parts[0]['grasp'] == 'FLAT_SHORT'                                     # 입력은 그대로


@pytest.mark.parametrize('parts', [
    [part(1, 0, 0, 0), part(2, 0, 0, 15)],                                      # 위아래로 쌓음(다른 층)
    [part(1, 0, 0, 0), part(2, 75, 0, 0)],                                      # 끝과 끝(긴 축 방향으로 이어짐)
    [part(1, 0, 0, 0), part(2, 0, 25, 0, ori='y')],                             # 방향이 다름(가로 · 세로)
    [part(1, 0, 0, 0), part(2, 0, 40, 0)],                                      # 15 mm 떨어짐(맞대지 않음)
    [part(1, 0, 0, 0), part(2, 50, 25, 0)],                                     # 길이 방향으로 절반 넘게 어긋남
    [part(1, -25, 0, 0, ori='zx', grasp='STAND_SHORT'), part(2, 25, 0, 0, ori='zx', grasp='STAND_SHORT')],   # 세운 블록
])
def test_연짝이_아니면_그대로(parts):
    assert row_orders(parts, BLOCK_MM) == set()
    assert fix_row_grasps(parts, BLOCK_MM)[1] == []


def test_긴_축이_y_인_연짝():
    parts = [part(1, -25, 0, 0, ori='y'), part(2, 0, 0, 0, ori='y'), part(3, 26, 0, 0, ori='y')]   # 틈 1 mm 도 연짝
    assert row_orders(parts, BLOCK_MM) == {1, 2, 3}


@pytest.mark.parametrize('model_id', sorted(ROWS))
def test_기본_설계_연짝은_레시피_파일에서_모두_FLAT_LONG(model_id):
    """기본 설계 4개: 연짝을 정확히 찾고, 레시피 파일이 이미 규칙대로라 고칠 블록이 없다(파일과 규칙이 같다)."""
    parts, name = base_parts(model_id)
    assert {name[o] for o in row_orders(parts, BLOCK_MM)} == ROWS[model_id]
    assert fix_row_grasps(parts, BLOCK_MM)[1] == []
    assert all(p['grasp'] == 'FLAT_LONG' for p in parts if name[p['order']] in ROWS[model_id])

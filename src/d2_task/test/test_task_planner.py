# -*- coding: utf-8 -*-
"""TaskPlanner 시험 (SDD 9.5 '가짜 진행표 5가지'). 레시피는 한세교 LV1 벤치 11개(001_CHAIR_BENCH), 설정은 실제 robot.yaml."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from d2_task.task_planner import TaskPlanner

SRC = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load((SRC / 'd2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
RECIPE = json.loads((Path(__file__).parent / 'fixtures/001_CHAIR_BENCH.recipe.json').read_text(encoding='utf-8'))
IDS = [f'001_CHAIR_BENCH_B{n:03d}' for n in range(1, 12)]


def planner(present=0, **kw):
    """앞에서부터 present 개는 놓였고 나머지는 없는 진행표를 가진 TaskPlanner."""
    p = TaskPlanner(CFG, kw.get('recipe', RECIPE))
    p.update_progress({b: {'state': 'present' if i < present else 'absent'} for i, b in enumerate(IDS)})
    return p


def test_빈_작업대면_첫_블록():
    r = planner(0).next_block()
    assert r['status'] == 'FOUND'
    assert r['block_id'] == IDS[0]
    assert r['grasp'] == 'FLAT_SHORT'          # 한세교 레시피 sequence 1 (다리 벽)
    assert r['supply_slot'] == '1'             # 눕힘 칸은 1 · 3 번
    assert len(r['pick_pose'][0]) == 3 and len(r['pick_pose'][1]) == 4
    assert r['place_pose'][0][2] == pytest.approx(CFG['assembly_origin']['z_m'] + 0.0075)   # 바닥 층 블록 중심 높이


def test_놓인_만큼_건너뛰고_칸을_돌려_쓴다():
    assert [planner(n).next_block()['supply_slot'] for n in (0, 1, 2, 3)] == ['1', '3', '1', '3']
    assert planner(4).next_block()['block_id'] == IDS[4]


def test_다_놓이면_DONE():
    assert planner(11).next_block() == {'status': 'DONE'}


def test_받침이_없으면_NO_SUPPORT():
    bad = copy.deepcopy(RECIPE)
    step = next(s for s in bad['steps'] if s['sequence'] == 1)
    step['support_instance_ids'] = [next(s['instance_id'] for s in bad['steps'] if s['sequence'] == 5)]
    assert planner(0, recipe=bad).next_block() == {'status': 'NO_SUPPORT', 'block_id': IDS[0]}


@pytest.mark.parametrize('state', ['occluded', 'unknown'])
def test_못_본_블록은_UNKNOWN_BLOCK(state):
    p = planner(2)
    p.update_progress({IDS[2]: {'state': state}})
    assert p.next_block() == {'status': 'UNKNOWN_BLOCK', 'block_id': IDS[2]}


def test_처음_관측_전에는_UNKNOWN_BLOCK():
    assert TaskPlanner(CFG, RECIPE).next_block()['status'] == 'UNKNOWN_BLOCK'


def test_빈_칸은_피하고_다_비면_WAIT_SUPPLY_채우면_이어_감():
    p = planner(0)
    p.mark_slot_empty(1)
    assert p.next_block()['supply_slot'] == '3'
    p.mark_slot_empty(3)
    assert p.next_block() == {'status': 'WAIT_SUPPLY', 'block_id': IDS[0]}
    p.supply_refilled()
    assert p.next_block()['status'] == 'FOUND'


def test_다른_자세_칸이_비어도_눕힘_블록은_그대로():
    p = planner(0)
    for slot in (2, 4, 5, 6):                  # 눕힘(THICKNESS) 칸이 아닌 칸
        p.mark_slot_empty(slot)
    assert p.next_block()['status'] == 'FOUND'


def test_잘못된_관측은_아무것도_안_바꾼다():
    p = planner(0)
    with pytest.raises(ValueError):
        p.update_progress({IDS[0]: {'state': 'present'}, 'NOPE_B001': {'state': 'present'}})
    with pytest.raises(ValueError):
        p.update_progress({IDS[0]: {'state': '있음'}})
    assert p.next_block()['block_id'] == IDS[0]


def test_없는_공급_칸_번호는_거부():
    with pytest.raises(ValueError):
        planner(0).mark_slot_empty(7)

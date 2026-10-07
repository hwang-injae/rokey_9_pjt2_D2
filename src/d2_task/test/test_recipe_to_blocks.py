# -*- coding: utf-8 -*-
"""변환기 ② 시험 (W117) — main 의 실제 레시피 3종(001 벤치 · 002 의자 · 003 책상 세운 다리)을 blocks/1 로 바꿔 본다."""
import copy
import json
from pathlib import Path

import pytest

from d2_task.recipe_to_blocks import RecipeToBlocks, ori_extents

RECIPES = Path(__file__).resolve().parents[2] / 'recipe_manager/recipes'
BLOCK_MM = [75.0, 25.0, 15.0]   # robot.yaml block_size_m × 1000
IDS = {'001_CHAIR_BENCH': ('bench', 'chair', 11), '002_CHAIR_BACK': ('chair_back', 'chair', 16),
       '003_DESK_STAND': ('desk_stand', 'desk', 9)}


def load(model_id):
    return json.loads((RECIPES / f'{model_id}.recipe.json').read_text(encoding='utf-8'))


def convert(model_id):
    design_id, family, _ = IDS[model_id]
    return RecipeToBlocks(design_id, family, BLOCK_MM).convert(load(model_id))


@pytest.mark.parametrize('model_id', IDS)
def test_3종_블록_수와_머리말(model_id):
    out = convert(model_id)
    design_id, family, n = IDS[model_id]
    assert (out['schema'], out['design_id'], out['family']) == ('blocks/1', design_id, family)
    assert [b['order'] for b in out['blocks']] == list(range(1, n + 1))
    assert all(b['inferred'] is False and b['ori'] in ori_extents(BLOCK_MM) for b in out['blocks'])


def test_벤치_실제_레시피와_같다():
    b = {x['order']: x for x in convert('001_CHAIR_BENCH')['blocks']}
    assert (b[1]['x'], b[1]['y'], b[1]['z'], b[1]['ori']) == (-25, 0, 0, 'y')
    assert (b[2]['x'], b[2]['y'], b[2]['z'], b[2]['ori']) == (25, 0, 0, 'y')
    assert (b[9]['x'], b[9]['y'], b[9]['z'], b[9]['ori']) == (0, -25, 60, 'x')
    assert [b[n]['z'] for n in range(1, 9)] == [0, 0, 15, 15, 30, 30, 45, 45]


def test_책상_세운_다리는_zx_이고_아랫면_높이():
    b = {x['order']: x for x in convert('003_DESK_STAND')['blocks']}
    assert (b[1]['x'], b[1]['y'], b[1]['z'], b[1]['ori']) == (-30, -25, 0, 'zx')   # 중심 37.5 − 길이 75/2
    assert (b[5]['z'], b[5]['ori']) == (75, 'x')
    assert (b[7]['x'], b[7]['z'], b[7]['ori']) == (-25, 90, 'y')


def test_의자_등받이_위층():
    b = {x['order']: x for x in convert('002_CHAIR_BACK')['blocks']}
    assert b[16]['z'] == 135 and b[16]['ori'] == 'x'


def test_steps_순서가_섞여도_sequence_순():
    r = load('001_CHAIR_BENCH')
    r['steps'].reverse()
    out = RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r)
    assert [b['order'] for b in out['blocks']] == list(range(1, 12))
    assert out == convert('001_CHAIR_BENCH')


def test_원본_레시피를_바꾸지_않는다():
    r = load('001_CHAIR_BENCH')
    before = copy.deepcopy(r)
    RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r)
    assert r == before


def test_출력은_JSON으로_직렬화된다():
    json.dumps(convert('003_DESK_STAND'))


def test_design_id_family_는_명시_입력():
    for args in (('', 'chair'), ('bench', ''), (None, None)):
        with pytest.raises(ValueError):
            RecipeToBlocks(*args, BLOCK_MM)


def bad(mutate):
    r = load('001_CHAIR_BENCH')
    mutate(r)
    with pytest.raises(ValueError):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r)


def test_잘못된_schema():
    bad(lambda r: r.update(schema='m0609.jenga.cad_recipe/1.0'))
    with pytest.raises(ValueError):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert({})


def test_단위가_mm_아님():
    bad(lambda r: r['model']['frame'].update(units='m'))


def test_없는_instance():
    bad(lambda r: r['steps'][0].update(instance_id='NOPE'))


def test_없는_part():
    bad(lambda r: r['model']['instances'][0].update(part_id='PART_999'))


def test_sequence_중복():
    bad(lambda r: r['steps'][1].update(sequence=1))


def test_steps_없음():
    bad(lambda r: r.update(steps=[]))


def test_45도_회전은_방향_복원_불가():
    s = 0.7071
    bad(lambda r: r['model']['instances'][0].update(R=[[s, -s, 0], [s, s, 0], [0, 0, 1]]))


def test_크기가_75_25_15가_아니면_방향_복원_불가():
    bad(lambda r: r['model']['parts'][0].update(size_mm=[80.0, 25.0, 15.0]))


def test_거울_반사_회전은_거부():
    bad(lambda r: r['model']['instances'][0].update(R=[[0, -1, 0], [1, 0, 0], [0, 0, -1]]))


def test_좌표는_레시피_원본_값_그대로():
    r = load('003_DESK_STAND')
    out = RecipeToBlocks('desk_stand', 'desk', BLOCK_MM).convert(r)
    inst = {i['instance_id']: i for i in r['model']['instances']}
    for step, b in zip(sorted(r['steps'], key=lambda s: s['sequence']), out['blocks']):
        cx, cy, _ = inst[step['instance_id']]['center_mm']
        assert (b['x'], b['y']) == (cx, cy)

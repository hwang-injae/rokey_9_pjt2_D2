# -*- coding: utf-8 -*-
"""변환기 ② 시험 (W117) — main 의 실제 레시피 3종(001 벤치 · 002 의자 · 003 책상 세운 다리)을 blocks/1 로 바꿔 본다."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from d2_task.recipe_to_blocks import RecipeToBlocks, ori_extents
from d2_task.recipe_document import RecipeDocument

RECIPES = Path(__file__).resolve().parents[2] / 'recipe_manager/recipes'
BLOCK_MM = [75.0, 25.0, 15.0]   # robot.yaml block_size_m × 1000 (아래 시험이 같은지 확인한다)
IDS = {'001_CHAIR_BENCH': ('bench', 'chair', 11), '002_CHAIR_BACK': ('chair_back', 'chair', 16),
       '003_DESK_STAND': ('desk_stand', 'desk', 9), '004_DESK_PEDESTAL': ('desk_pedestal', 'desk', 11)}


def load(model_id):
    """main 레시피 두 파일(_recipe.json · _structure.json)을 (recipe, structure) 복사본으로 읽는다. 형식 오류는 시험 실패로 알린다."""
    doc = RecipeDocument.load(RECIPES, model_id)
    return copy.deepcopy(doc.recipe), copy.deepcopy(doc.structure)


def convert(model_id):
    design_id, family, _ = IDS[model_id]
    return RecipeToBlocks(design_id, family, BLOCK_MM).convert(*load(model_id))


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
    r, st = load('001_CHAIR_BENCH')
    r['steps'].reverse()
    out = RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r, st)
    assert [b['order'] for b in out['blocks']] == list(range(1, 12))
    assert out == convert('001_CHAIR_BENCH')


def test_원본_레시피를_바꾸지_않는다():
    r, st = load('001_CHAIR_BENCH')
    before = copy.deepcopy((r, st))
    RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r, st)
    assert (r, st) == before


def test_출력은_JSON으로_직렬화된다():
    json.dumps(convert('003_DESK_STAND'))


def test_design_id_family_는_명시_입력():
    for args in (('', 'chair'), ('bench', ''), (None, None)):
        with pytest.raises(ValueError):
            RecipeToBlocks(*args, BLOCK_MM)


def bad(mutate):
    """recipe · structure 복사본을 mutate(recipe, structure) 로 망가뜨리면 변환이 ValueError 여야 한다."""
    r, st = load('001_CHAIR_BENCH')
    mutate(r, st)
    with pytest.raises(ValueError):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r, st)


def test_옛_한_파일_레시피는_거부():
    """E-52: model 이 들어 있는 한 파일 레시피(옛 형식)는 더 받지 않는다."""
    old = {'schema': 'cad_recipe/1.0', 'model': {'model_id': '001_CHAIR_BENCH'}, 'steps': []}
    with pytest.raises(ValueError, match='옛 한 파일'):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(old, None)
    bad(lambda r, st: r.update(schema='assembly.recipe/1.0'))


def test_structure가_없으면_거부():
    r, _ = load('001_CHAIR_BENCH')
    with pytest.raises(ValueError):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r, None)


def test_잘못된_schema():
    bad(lambda r, st: r.update(schema='m0609.jenga.cad_recipe/1.0'))
    bad(lambda r, st: st.update(schema='other/1'))
    with pytest.raises(ValueError):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert({}, {})


def test_없는_block():
    bad(lambda r, st: r['steps'][0].update(block='NOPE_001_01'))


def test_없는_part():
    bad(lambda r, st: st['blocks'][0].update(part_id='PART_999'))


def test_sequence_중복():
    bad(lambda r, st: r['steps'][1].update(sequence=1))


def test_steps_없음():
    bad(lambda r, st: r.update(steps=[]))


def test_45도_회전은_방향_복원_불가():
    s = 0.7071
    bad(lambda r, st: st['blocks'][0].update(R=[[s, -s, 0], [s, s, 0], [0, 0, 1]]))


def test_크기가_75_25_15가_아니면_방향_복원_불가():
    bad(lambda r, st: st['parts'][0].update(size_mm=[80.0, 25.0, 15.0]))


def test_거울_반사_회전은_거부():
    bad(lambda r, st: st['blocks'][0].update(R=[[0, -1, 0], [1, 0, 0], [0, 0, -1]]))


def test_좌표는_레시피_원본_값_그대로():
    r, st = load('003_DESK_STAND')
    out = RecipeToBlocks('desk_stand', 'desk', BLOCK_MM).convert(r, st)
    center = {b['block']: b['center_mm'] for b in st['blocks']}
    for step, b in zip(sorted(r['steps'], key=lambda s: s['sequence']), out['blocks']):
        cx, cy, _ = center[step['block']]
        assert (b['x'], b['y']) == (cx, cy)


def test_블록_크기_두_곳은_허용_오차_없이_같다():
    """방향 계산(레시피 parts[].size_mm)과 방향 표(robot.yaml block_size_m × 1000)가 같아야 방향이 정해진다(한세교 W117 확인 요청)."""
    root = Path(__file__).resolve().parents[2]
    cfg = yaml.safe_load(next(root.glob('d2_robot/d2_bringup/config/robot.yaml')).read_text(encoding='utf-8'))
    assert [v * 1000.0 for v in cfg['block_size_m']] == BLOCK_MM
    for model_id in IDS:
        _, structure = load(model_id)
        assert all(p['size_mm'] == BLOCK_MM for p in structure['parts']), model_id

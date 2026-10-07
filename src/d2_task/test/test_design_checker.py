# -*- coding: utf-8 -*-
"""검사 묶음 DesignChecker 시험 (W109) — 실제 robot.yaml + main 의 기본 설계 3종(레시피 → 변환기 ②) + 손으로 만든 나쁜 설계들."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from d2_task.design_checker import DesignChecker
from d2_task.recipe_to_blocks import RecipeToBlocks

SRC = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load((SRC / 'd2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
BLOCK_MM = [v * 1000.0 for v in CFG['block_size_m']]
DESIGNS = {'001_CHAIR_BENCH': ('bench', 'chair', 12.5), '002_CHAIR_BACK': ('chair_back', 'chair', 12.5),
           '003_DESK_STAND': ('desk_stand', 'desk', 7.5)}


def base_design(model_id):
    design_id, family, _ = DESIGNS[model_id]
    recipe = json.loads((SRC / f'recipe_manager/recipes/{model_id}.recipe.json').read_text(encoding='utf-8'))
    return RecipeToBlocks(design_id, family, BLOCK_MM).convert(recipe)


def design(*items):
    """(x, y, z, ori) 들 → order 1.. 의 blocks/1."""
    return {'schema': 'blocks/1', 'design_id': 't', 'family': 'test',
            'blocks': [{'order': i, 'x': x, 'y': y, 'z': z, 'ori': o, 'inferred': False} for i, (x, y, z, o) in enumerate(items, 1)]}


def check(req, **kw):
    return DesignChecker(CFG, **kw).check(req)


def reasons(res):
    return [(e['block'], e['reason']) for e in res['errors']]


@pytest.mark.parametrize('model_id', DESIGNS)
def test_기본_설계_3종_통과(model_id):
    res = check(base_design(model_id))
    assert res['ok'] and res['errors'] == [] and res['schema'] == 'check_result/1'
    assert res['min_margin_mm'] == DESIGNS[model_id][2]


def test_계단_20mm씩_3번_내밀면_불안정():
    res = check(design(*[(20 * i, 0, 15 * i, 'x') for i in range(4)]))
    assert not res['ok']
    assert res['min_margin_mm'] == -2.5
    assert reasons(res) == [(2, 'CHECK_FAILED')]
    assert '2번 위 덩어리' in res['errors'][0]['detail'] and '-2.5' in res['errors'][0]['detail']


def test_5mm_내밀면_통과():
    assert check(design((0, 0, 0, 'x'), (5, 0, 15, 'x'), (10, 0, 30, 'x')))['ok']


def test_쌓는_도중_불안정은_순서에_따라_다르다():
    a, b, c = (-25, 0, 0, 'y'), (25, 0, 0, 'y'), (0, 0, 15, 'x')
    assert check(design(a, b, c))['ok']
    res = check(design(a, c, b))                  # b 가 없을 때 c 가 한 쪽 기둥에만 얹혀 쓰러진다
    assert not res['ok'] and res['min_margin_mm'] == -12.5 and reasons(res) == [(1, 'CHECK_FAILED'), (2, 'CHECK_FAILED')]   # 1번은 위 덩어리 무게중심이 가장자리(여유 0)


def test_파고듦():
    res = check(design((0, 0, 0, 'x'), (10, 0, 5, 'x')))
    assert any('1번 ↔ 2번 겹침' in e['detail'] for e in res['errors'])


def test_면이_닿기만_하면_파고듦_아님():
    assert not any('겹침' in e['detail'] for e in check(design((0, 0, 0, 'x'), (0, 25, 0, 'x')))['errors'])


def test_공중에_뜬_블록():
    res = check(design((0, 0, 0, 'x'), (0, 0, 20, 'x')))
    assert reasons(res) == [(2, 'CHECK_FAILED')] and '공중' in res['errors'][0]['detail'] and res['min_margin_mm'] is None


def test_받침이_더_늦게_놓이면_공중():
    res = check(design((0, 0, 15, 'x'), (0, 0, 0, 'x')))
    assert '받침 2번이 더 늦게 놓임' in res['errors'][0]['detail']


def test_작업면_아래():
    assert not check(design((0, 0, -1, 'x')))['ok']


def test_양쪽_막힌_칸():
    pocket = [(0, -25, 0, 'x'), (0, 25, 0, 'x'), (-50, 0, 0, 'y'), (50, 0, 0, 'y'), (0, 0, 0, 'x')]
    res = check(design(*pocket))
    assert reasons(res) == [(5, 'CHECK_FAILED')] and '잡을 면 없음' in res['errors'][0]['detail']
    assert check(design(*pocket[:2], pocket[4]))['ok']                  # 한 방향만 막히면 다른 방향으로 잡는다


def test_잡기_후보는_놓는_시점_기준():
    bench = base_design('001_CHAIR_BENCH')
    opts = DesignChecker(CFG).grasp_options(bench['blocks'])
    assert sorted(opts[1]) == ['FLAT_LONG', 'FLAT_SHORT']
    assert opts[10] == ['FLAT_LONG']                                    # 9번이 옆에 있어 좁은 쪽(FLAT_SHORT)은 막힘
    stand = DesignChecker(CFG).grasp_options(base_design('003_DESK_STAND')['blocks'])
    assert sorted(stand[1]) == ['STAND_LONG', 'STAND_SHORT']


def test_작업영역_밖():
    assert check(design((100, 0, 0, 'x')))['ok']
    res = check(design((120, 0, 0, 'x')))                              # x 끝 157.5 mm > 150 mm
    assert reasons(res) == [(1, 'CHECK_FAILED')] and '작업영역 밖' in res['errors'][0]['detail']


@pytest.mark.parametrize('mutate, text', [
    (lambda r: r.pop('blocks'), 'blocks'),
    (lambda r: r.update(blocks=[]), 'blocks'),
    (lambda r: r['blocks'][0].pop('ori'), 'ori 없음'),
    (lambda r: r['blocks'][0].update(ori='q'), 'ori='),
    (lambda r: r['blocks'][0].update(x='1'), 'x 가 숫자가 아니다'),
    (lambda r: r['blocks'][0].update(z=True), 'z 가 숫자가 아니다'),
    (lambda r: r['blocks'][1].update(order=5), 'order'),
    (lambda r: r['blocks'][1].update(order=True), 'order'),
])
def test_형식_오류(mutate, text):
    req = design((0, 0, 0, 'x'), (0, 0, 15, 'x'))
    mutate(req)
    res = check(req)
    assert not res['ok'] and res['min_margin_mm'] is None and res['errors'][0]['detail'].startswith('형식')
    assert text in res['errors'][0]['detail']


def test_요청이_객체가_아님():
    assert not check([])['ok']


def test_블록_수_상한():
    res = check({'blocks': [{'order': i, 'x': 0, 'y': 0, 'z': 0, 'ori': 'x'} for i in range(1, 56)]})
    assert reasons(res) == [(None, 'OUT_OF_SCOPE')]
    assert check({'blocks': [{'order': i, 'x': 0, 'y': 0, 'z': 15 * (i - 1), 'ori': 'x'} for i in range(1, 3)]})['ok']


def test_order_순서가_섞여_와도_order로_검사():
    req = design((0, 0, 0, 'x'), (0, 0, 15, 'x'))
    req['blocks'].reverse()
    assert check(req)['ok']


def test_변환기_연결():
    bench = base_design('001_CHAIR_BENCH')
    calls = []
    res = check(bench, blocks_to_recipe=lambda blocks: calls.append(blocks) or {'schema': 'cad_recipe/1.0'})
    assert res['ok'] and res['recipe'] == {'schema': 'cad_recipe/1.0'} and calls == [bench]
    assert 'recipe' not in check(bench)
    bad = check(design((0, 0, 0, 'x'), (0, 0, 20, 'x')), blocks_to_recipe=lambda b: calls.append(b))
    assert not bad['ok'] and len(calls) == 1                           # 통과 못 하면 변환기를 부르지 않는다
    boom = check(bench, blocks_to_recipe=lambda b: 1 / 0)
    assert not boom['ok'] and boom['errors'][0]['reason'] == 'ERROR'


def test_robot_yaml에_키가_없으면_만들_때_알림():
    cfg = copy.deepcopy(CFG)
    del cfg['check']
    with pytest.raises(ValueError, match='check'):
        DesignChecker(cfg)


def test_결과는_JSON으로_직렬화된다():
    json.dumps(check(base_design('002_CHAIR_BACK'), blocks_to_recipe=lambda b: {}))

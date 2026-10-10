# -*- coding: utf-8 -*-
"""검사 묶음 DesignChecker 시험 (W109) — 실제 robot.yaml + main 의 기본 설계 3종(레시피 → 변환기 ②) + 손으로 만든 나쁜 설계들."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from d2_task.design_checker import DesignChecker
from d2_task.recipe_to_blocks import RecipeToBlocks
from d2_task.recipe_document import RecipeDocument

SRC = Path(__file__).resolve().parents[2]
ROBOT = SRC / 'd2_robot' if (SRC / 'd2_robot/d2_bringup').is_dir() else SRC
CFG = yaml.safe_load((ROBOT / 'd2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
BLOCK_MM = [v * 1000.0 for v in CFG['block_size_m']]
DESIGNS = {'001_CHAIR_BENCH_V000': ('bench', 'chair', 12.5), '002_CHAIR_BACK_V000': ('chair_back', 'chair', 12.5),
           '003_DESK_STAND_V000': ('desk_stand', 'desk', 7.5), '004_DESK_PEDESTAL_V000': ('desk_pedestal', 'desk', 12.5)}   # 004 = 10/7 가운데 기둥형 책상(W117)


FIXTURES = Path(__file__).parent / 'fixtures'
_DOC = RecipeDocument.load(FIXTURES, '001_CHAIR_BENCH_V000')
CONVERTED = {'recipe': _DOC.recipe, 'placements': _DOC.placements}      # 변환기 ①이 돌려주는 두 문서(가짜 변환기의 답, E-69)


def base_design(model_id):
    design_id, family, _ = DESIGNS[model_id]
    """레시피 두 파일(시험용 사본)을 읽어 blocks/2.0 으로 바꾼다. 읽기 실패는 시험에 알린다."""
    document = RecipeDocument.load(FIXTURES, model_id)
    return RecipeToBlocks(design_id, family, BLOCK_MM).convert(document.recipe, document.placements)


def design(*items):
    """(x, y, z, ori) 들 → order 1.. 의 blocks/2.0 (AI 칸은 형식만 맞는 값: 역할 LEG · 부품 1 · 단계 1 · 잡기 FLAT_SHORT)."""
    return {'schema': 'blocks/2.0', 'design_id': 't', 'family': 'test',
            'blocks': [{'order': i, 'x': x, 'y': y, 'z': z, 'ori': o, 'role': 'LEG', 'part': 1, 'stage': 1, 'grasp': 'FLAT_SHORT'}
                       for i, (x, y, z, o) in enumerate(items, 1)]}


def check(req, **kw):
    return DesignChecker(CFG, **kw).check(req)


def reasons(res):
    return [(e['block'], e['reason']) for e in res['errors']]


@pytest.mark.parametrize('model_id', DESIGNS)
def test_기본_설계_3종_통과(model_id):
    res = check(base_design(model_id))
    assert res['ok'] and res['errors'] == [] and res['schema'] == 'check_result/2.0'
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
    bench = base_design('001_CHAIR_BENCH_V000')
    opts = DesignChecker(CFG).grasp_options(bench['blocks'])
    assert sorted(opts[1]) == ['FLAT_LONG', 'FLAT_SHORT']
    assert opts[10] == ['FLAT_LONG']                                    # 9번이 옆에 있어 좁은 쪽(FLAT_SHORT)은 막힘
    stand = DesignChecker(CFG).grasp_options(base_design('003_DESK_STAND_V000')['blocks'])
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
    res = check(design(*[(0, 0, 0, 'x')] * 55))
    assert reasons(res) == [(None, 'OUT_OF_SCOPE')]
    assert check(design(*[(0, 0, 15 * (i - 1), 'x') for i in range(1, 3)]))['ok']


def test_order_순서가_섞여_와도_order로_검사():
    req = design((0, 0, 0, 'x'), (0, 0, 15, 'x'))
    req['blocks'].reverse()
    assert check(req)['ok']


def test_변환기_연결():
    """변환기 ①은 {recipe, placements} 두 문서를 돌려준다. 한 문서만 준 답은 ERROR."""
    bench = base_design('001_CHAIR_BENCH_V000')
    calls = []
    res = check(bench, blocks_to_recipe=lambda blocks: calls.append(blocks) or CONVERTED)
    assert res['ok'] and (res['recipe'], res['placements']) == (CONVERTED['recipe'], CONVERTED['placements']) and calls == [bench]
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
    json.dumps(check(base_design('002_CHAIR_BACK_V000'), blocks_to_recipe=lambda b: {}))


# ---------------- check_design 서비스 한 번 처리(handle_json) ----------------
def handle(req_text, **kw):
    ok, reason, text = DesignChecker(CFG, **kw).handle_json(req_text)
    return ok, reason, json.loads(text)


def test_서비스_합격은_success_true와_레시피():
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), blocks_to_recipe=lambda b: CONVERTED)
    assert (ok, reason) == (True, '') and res['ok'] and (res['recipe'], res['placements']) == (CONVERTED['recipe'], CONVERTED['placements'])


def test_서비스_설계_불합격은_success_true_ok_false():
    ok, reason, res = handle(json.dumps(design((0, 0, 0, 'x'), (0, 0, 20, 'x'))), blocks_to_recipe=lambda b: {})
    assert (ok, reason) == (True, '') and not res['ok'] and res['errors']


def test_서비스_변환기_미연결이면_합격이어도_success_false():
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')))
    assert (ok, reason) == (False, 'ERROR') and not res['ok'] and 'recipe' not in res
    assert res['min_margin_mm'] == 12.5 and '변환기' in res['errors'][0]['detail']


def test_서비스_변환기_미연결이어도_불합격은_검사_이유_그대로():
    ok, reason, res = handle(json.dumps(design((0, 0, 0, 'x'), (0, 0, 20, 'x'))))
    assert (ok, reason) == (True, '') and not res['ok'] and res['errors'][0]['reason'] == 'CHECK_FAILED'


@pytest.mark.parametrize('text', ['{깨진', '[]', '"글자"', 'null'])
def test_서비스_JSON이_아니거나_객체가_아니면_ERROR(text):
    ok, reason, res = handle(text)
    assert (ok, reason) == (False, 'ERROR') and res['errors'][0]['reason'] == 'ERROR'


def test_서비스_안쪽_예외도_ERROR():
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), blocks_to_recipe=lambda b: float('nan'))
    assert (ok, reason) == (False, 'ERROR')           # NaN 은 allow_nan=False 로 직렬화에서 걸린다


def test_서비스_응답은_순수_JSON():
    _, _, text = DesignChecker(CFG, blocks_to_recipe=lambda b: CONVERTED).handle_json(json.dumps(base_design('002_CHAIR_BACK_V000')))
    assert json.loads(text, parse_constant=lambda c: pytest.fail(c))['ok']


def test_서비스_변환기_예외는_success_false():
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), blocks_to_recipe=lambda b: 1 / 0)
    assert (ok, reason) == (False, 'ERROR') and not res['ok'] and '변환기' in res['errors'][0]['detail']


@pytest.mark.parametrize('bad_recipe', [None, [], 'x', {}, {'schema': 'other/1'}, {'schema': 'assembly.recipe/1.0'}])
def test_서비스_변환기_결과가_레시피_객체가_아니면_ERROR(bad_recipe):
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), blocks_to_recipe=lambda b: bad_recipe)
    assert (ok, reason) == (False, 'ERROR') and not res['ok'] and 'recipe' not in res


@pytest.mark.parametrize('schema', ['other/1', 'blocks/1', 'blocks/2', 'blocks/3.0', None, 5])
def test_서비스_schema가_blocks_1이_아니면_ERROR(schema):
    req = base_design('001_CHAIR_BENCH_V000')
    req['schema'] = schema
    ok, reason, _ = handle(json.dumps(req), blocks_to_recipe=lambda b: CONVERTED)
    assert (ok, reason) == (False, 'ERROR')


def test_서비스_schema_없으면_ERROR():
    req = base_design('001_CHAIR_BENCH_V000')
    del req['schema']
    assert handle(json.dumps(req), blocks_to_recipe=lambda b: {})[:2] == (False, 'ERROR')


@pytest.mark.parametrize('literal', ['NaN', 'Infinity', '-Infinity'])
def test_서비스_유한하지_않은_수는_불합격이_아니라_ERROR(literal):
    text = json.dumps(base_design('001_CHAIR_BENCH_V000')).replace('"x": -25', f'"x": {literal}', 1)
    assert literal in text
    ok, reason, res = handle(text, blocks_to_recipe=lambda b: CONVERTED)
    assert (ok, reason) == (False, 'ERROR') and literal in res['errors'][0]['detail']


def test_서비스_오류_응답도_순수_JSON():
    _, _, text = DesignChecker(CFG).handle_json('{"schema":"blocks/1","blocks":[{"x":NaN}]}')
    json.loads(text, parse_constant=lambda c: pytest.fail(c))


def test_서비스_ori가_이상한_값이어도_ERROR로_돌려준다():
    req = base_design('001_CHAIR_BENCH_V000')
    req['blocks'][0]['ori'] = []
    ok, reason, res = handle(json.dumps(req), blocks_to_recipe=lambda b: CONVERTED)
    assert (ok, reason) == (False, 'ERROR') and not res['ok']


@pytest.mark.parametrize('literal', ['1e999', '-1e999', '1E400'])
def test_서비스_읽으면_무한대가_되는_수도_ERROR(literal):
    text = json.dumps(base_design('001_CHAIR_BENCH_V000')).replace('"x": -25', f'"x": {literal}', 1)
    assert literal in text
    ok, reason, res = handle(text, blocks_to_recipe=lambda b: CONVERTED)
    assert (ok, reason) == (False, 'ERROR') and literal in res['errors'][0]['detail']


def test_변환기가_옛_한_파일만_돌려주면_ERROR():
    """recipe(구조)만 있는 답(placements 없음)은 합격 결과로 내보내지 않는다."""
    res = check(base_design('001_CHAIR_BENCH_V000'), blocks_to_recipe=lambda b: CONVERTED['recipe'])
    assert not res['ok'] and res['errors'][0]['reason'] == 'ERROR'


# ---------- blocks/2.0 의 AI 칸(role · part · stage · grasp) 형식 검사 (E-69) ----------
def ai_blocks(n=3):
    """쌓인 블록 n개(아랫면 0, 15, 30)의 정상 blocks/2.0."""
    return design(*[(0, 0, 15 * i, 'x') for i in range(n)])


def one_error_detail(req):
    res = check(req)
    assert not res['ok'] and res['errors'] and all(e['reason'] == 'CHECK_FAILED' for e in res['errors']), res
    return res['errors'][0]['detail']


@pytest.mark.parametrize('key', ['role', 'part', 'stage', 'grasp'])
def test_AI_칸이_빠지면_CHECK_FAILED이고_코드가_대신_채우지_않는다(key):
    req = ai_blocks()
    del req['blocks'][1][key]
    assert key in one_error_detail(req) and '없음' in one_error_detail(req)


@pytest.mark.parametrize('role', ['leg', 'ARM_REST_X', 'LEG_', '_LEG', 'LEG1', '', 5, None])
def test_역할_형식이_틀리면_거절(role):
    req = ai_blocks()
    req['blocks'][0]['role'] = role
    assert 'role' in one_error_detail(req)


@pytest.mark.parametrize('role', ['LEG', 'LEG_WHEEL', 'SEAT'])
def test_역할_한_단어_옵션_0_1개는_통과(role):
    req = ai_blocks()
    req['blocks'][0]['role'] = role
    assert check(req)['ok']


@pytest.mark.parametrize('key, value', [('part', 0), ('part', -1), ('part', True), ('part', 1.5), ('part', '1'),
                                        ('stage', 0), ('stage', True), ('stage', None), ('grasp', 'SIDE_25'), ('grasp', 'flat_short'), ('grasp', None)])
def test_part_stage_grasp_값이_틀리면_거절(key, value):
    req = ai_blocks()
    req['blocks'][1][key] = value
    assert key in one_error_detail(req)


@pytest.mark.parametrize('stages', [[2, 2, 3], [1, 3, 3], [1, 2, 1], [2, 3, 4]])
def test_단계는_1부터_같거나_1씩_늘어야_한다(stages):
    req = ai_blocks()
    for b, st in zip(req['blocks'], stages):
        b['stage'] = st
    assert 'stage' in one_error_detail(req)


@pytest.mark.parametrize('stages', [[1, 1, 1], [1, 1, 2], [1, 2, 3], [1, 2, 2]])
def test_단계가_같거나_1씩_늘면_통과(stages):
    req = ai_blocks()
    for b, st in zip(req['blocks'], stages):
        b['stage'] = st
    assert check(req)['ok']


def test_inferred는_선택이고_불리언이어야_한다():
    req = ai_blocks()
    req['blocks'][2]['inferred'] = True
    assert check(req)['ok']
    req['blocks'][2]['inferred'] = 'yes'
    assert 'inferred' in one_error_detail(req)


def test_옛_blocks_1은_AI_칸이_없어_서비스에서_거절():
    """E-69: blocks/1 은 스캔 추론기 출력(위치 · 방향만)이라 check_design 요청으로 받지 않는다 — 받은 schema 를 이유에 적어 ERROR."""
    req = ai_blocks()
    req['schema'] = 'blocks/1'
    for b in req['blocks']:
        for k in ('role', 'part', 'stage', 'grasp'):
            del b[k]
    ok, reason, text = DesignChecker(CFG).handle_json(json.dumps(req))
    assert (ok, reason) == (False, 'ERROR') and 'blocks/1' in text


def test_변환기_결과의_짝_해시가_안_맞으면_ERROR():
    from d2_task.recipe_document import recipe_sha256  # noqa: F401  (해시 규칙은 test_recipe_document 가 본다)
    wrong = {'recipe': CONVERTED['recipe'], 'placements': dict(CONVERTED['placements'], recipe_sha256='0' * 64)}
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), blocks_to_recipe=lambda b: wrong)
    assert (ok, reason) == (False, 'ERROR') and not res['ok'] and 'recipe_sha256' in res['errors'][0]['detail']

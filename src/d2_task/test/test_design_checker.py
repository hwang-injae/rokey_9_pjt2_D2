# -*- coding: utf-8 -*-
"""검사 묶음 DesignChecker 시험 (W109) — 실제 robot.yaml + main 의 기본 설계 3종(레시피 → 변환기 ②) + 손으로 만든 나쁜 설계들."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from d2_task.design_checker import DesignChecker
from d2_task.recipe_to_parts import RecipeToParts
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
    """레시피 두 파일(시험용 사본)을 읽어 실제 check_design 요청(parts/4.0 + base_recipe = 그 Template 의 V000 레시피 통째)으로 바꾼다. 읽기 실패는 시험에 알린다."""
    _, family, _ = DESIGNS[model_id]
    document = RecipeDocument.load(FIXTURES, model_id)
    request = RecipeToParts(model_id, BLOCK_MM).convert(document.recipe, document.placements)
    return dict(request, base_recipe=copy.deepcopy(document.recipe))


def design(*items):
    """(x, y, z, ori) 들 → order 1.. 의 parts/4.0 (AI 칸은 형식만 맞는 값: 역할 LEG · 부품 1 · 단계 1 · 잡기 FLAT_SHORT). base_recipe 는 없다 — NO_TEMPLATE 과 같이 쓴다."""
    return {'schema': 'parts/4.0', 'design_id': 't', 'family': 'test',
            'parts': [{'sequence': i, 'x': x, 'y': y, 'z': z, 'ori': o, 'role': 'LEG', 'group': 1, 'stage': 1, 'grasp': 'FLAT_SHORT'}
                       for i, (x, y, z, o) in enumerate(items, 1)]}


class NoTemplate:
    """기하(1~20번) 단독 시험용 Template 검사 대역 — 21번을 일부러 건너뛴다. 실제 서비스는 늘 진짜 TemplateChecker 를 쓰고(기본 생성자),
    이 대역은 시험이 template_checker= 로 직접 넣을 때만 켜진다. 21번 시험은 test_template_checker.py."""

    def read_base(self, request):
        return None

    def check(self, rules, parts):
        return []


NO_TEMPLATE = NoTemplate()


def check(req, **kw):
    """기하 시험용: 늘 NO_TEMPLATE 대역을 쓴다(kw 로 바꿀 수 있음)."""
    return DesignChecker(CFG, **{'template_checker': NO_TEMPLATE, **kw}).check(req)


def reasons(res):
    return [(e['part'], e['reason']) for e in res['errors']]


@pytest.mark.parametrize('model_id', DESIGNS)
def test_기본_설계_3종_통과(model_id):
    res = check(base_design(model_id))
    assert res['ok'] and res['errors'] == [] and res['schema'] == 'check_result/3.0'
    assert res['min_margin_mm'] == DESIGNS[model_id][2]


def test_계단_20mm씩_3번_내밀면_불안정():
    res = check(design(*[(20 * i, 0, 15 * i, 'x') for i in range(4)]))
    assert not res['ok']
    assert res['min_margin_mm'] == -2.5
    # 어떤 순서로도 안 되는 배치 — W201(E-121 ②)이 '로봇 조립 불가(R3)'를 position 으로(막힌 블록 = 4번을 놓을 때 2번 위 덩어리가 기움)
    assert reasons(res) == [(4, 'CHECK_FAILED')] and res['errors'][0]['kind'] == 'position' and res['feasible_order'] is None
    assert '로봇 조립 불가(R3)' in res['errors'][0]['detail'] and '2번 위 덩어리' in res['errors'][0]['detail'] and '-2.5' in res['errors'][0]['detail']


def test_5mm_내밀면_통과():
    assert check(design((0, 0, 0, 'x'), (5, 0, 15, 'x'), (10, 0, 30, 'x')))['ok']


def test_쌓는_도중_불안정은_순서에_따라_다르다():
    a, b, c = (-25, 0, 0, 'y'), (25, 0, 0, 'y'), (0, 0, 15, 'x')
    assert check(design(a, b, c))['ok']
    res = check(design(a, c, b))                  # b 가 없을 때 c 가 한 쪽 기둥에만 얹혀 쓰러진다
    assert not res['ok'] and res['min_margin_mm'] == -12.5
    # 다른 순서(1 → 3 → 2)로는 된다 — AI 순서 실패 두 줄(1 · 2번 위 덩어리)을 ai_fields 한 줄로 바꾸고 찾은 순서를 싣는다(W201 · E-121 ⑦)
    assert reasons(res) == [(1, 'CHECK_FAILED')] and res['errors'][0]['kind'] == 'ai_fields' and res['feasible_order'] == [1, 3, 2]
    assert '1 → 3 → 2' in res['errors'][0]['detail'] and '1번 위 덩어리' in res['errors'][0]['detail'] and '외 1줄' in res['errors'][0]['detail']


def test_파고듦():
    res = check(design((0, 0, 0, 'x'), (10, 0, 5, 'x')))
    assert any('1번 ↔ 2번 겹침' in e['detail'] for e in res['errors'])


def test_면이_닿기만_하면_파고듦_아님():
    assert not any('겹침' in e['detail'] for e in check(design((0, 0, 0, 'x'), (0, 25, 0, 'x')))['errors'])


def test_겹침_기준은_백만분의_1_mm():
    """0.01 mm 파고듦은 이제 걸리고(옛 기준 0.1 이면 통과), 계산 오차 수준(1e-9 mm)의 면접촉은 통과한다 (E-84)."""
    assert any('겹침' in e['detail'] for e in check(design((0, 0, 0, 'x'), (0, 24.99, 0, 'x')))['errors'])
    assert not any('겹침' in e['detail'] for e in check(design((0, 0, 0, 'x'), (0, 25 - 1e-9, 0, 'x')))['errors'])


def test_공중에_뜬_블록():
    res = check(design((0, 0, 0, 'x'), (0, 0, 20, 'x')))
    assert reasons(res) == [(2, 'CHECK_FAILED')] and '공중' in res['errors'][0]['detail'] and res['min_margin_mm'] is None


def test_받침이_더_늦게_놓이면_공중():
    res = check(design((0, 0, 15, 'x'), (0, 0, 0, 'x')))
    assert '받침 2번이 더 늦게 놓임' in res['errors'][0]['detail']


def test_작업면_아래():
    assert not check(design((0, 0, -1, 'x')))['ok']


def test_양쪽_막힌_칸():
    # 같은 층에 눕힘 3개 L 자 — AI 순서는 모서리(3번)가 마지막이라 두 방향 다 막혀 '잡을 면 없음', 모서리를 먼저 놓으면 된다(W201 · E-121 ⑦)
    corner = [(37.5, 0, 0, 'x'), (-37.5, 25, 0, 'x'), (-37.5, 0, 0, 'x')]
    res = check(design(*corner))
    assert reasons(res) == [(3, 'CHECK_FAILED')] and '잡을 면 없음' in res['errors'][0]['detail'] and res['errors'][0]['kind'] == 'ai_fields'
    assert sorted(res['feasible_order']) == [1, 2, 3] and res['feasible_order'].index(3) < 2
    assert check(design(corner[2], *corner[:2]))['ok']                  # 모서리를 먼저 쓰면 그대로 합격
    pocket = [(0, -25, 0, 'x'), (0, 25, 0, 'x'), (0, 0, 0, 'x')]
    assert check(design(*pocket))['ok']                                 # 한 방향만 막히면 다른 방향으로 잡는다


def test_잡기_후보는_놓는_시점_기준():
    bench = base_design('001_CHAIR_BENCH_V000')
    opts = DesignChecker(CFG).grasp_options(bench['parts'])
    assert sorted(opts[1]) == ['FLAT_LONG', 'FLAT_SHORT']
    assert opts[10] == ['FLAT_LONG']                                    # 9번이 옆에 있어 좁은 쪽(FLAT_SHORT)은 막힘
    stand = DesignChecker(CFG).grasp_options(base_design('003_DESK_STAND_V000')['parts'])
    assert sorted(stand[1]) == ['STAND_LONG', 'STAND_SHORT']


def test_작업영역_밖():
    assert check(design((100, 0, 0, 'x')))['ok']
    res = check(design((120, 0, 0, 'x')))                              # x 끝 157.5 mm > 150 mm
    assert reasons(res) == [(1, 'CHECK_FAILED')] and '작업영역 밖' in res['errors'][0]['detail']


@pytest.mark.parametrize('mutate, text', [
    (lambda r: r['parts'][0].pop('ori'), 'ori 없음'),
    (lambda r: r['parts'][0].update(ori='q'), 'ori='),
    (lambda r: r['parts'][0].update(x='1'), 'x 가 숫자가 아니다'),
    (lambda r: r['parts'][0].update(z=True), 'z 가 숫자가 아니다'),
    (lambda r: r['parts'][1].update(sequence=5), 'sequence'),
    (lambda r: r['parts'][1].update(sequence=True), 'sequence'),
])
def test_형식_오류(mutate, text):
    req = design((0, 0, 0, 'x'), (0, 0, 15, 'x'))
    mutate(req)
    res = check(req)
    assert not res['ok'] and res['min_margin_mm'] is None and res['errors'][0]['detail'].startswith('형식')
    assert text in res['errors'][0]['detail']


@pytest.mark.parametrize('mutate', [lambda r: r.pop('parts'), lambda r: r.update(parts=[]), lambda r: r.update(parts={}),
                                    lambda r: r.update(parts='x'), lambda r: r.pop('design_id'), lambda r: r.update(design_id=5),
                                    lambda r: r.update(design_id='')])
def test_봉투_오류는_REQUEST_INVALID(mutate):
    """design_id · parts 목록 누락 · 타입 · 빈 목록 = 웹 쪽 잘못(D10). 대역을 써도 봉투 확인은 늘 한다."""
    req = design((0, 0, 0, 'x'))
    mutate(req)
    res = check(req)
    assert [(e['reason'], 'kind' in e) for e in res['errors']] == [('REQUEST_INVALID', False)] and not res['ok']


def test_요청이_객체가_아님():
    res = check([])
    assert not res['ok'] and res['errors'][0]['reason'] == 'REQUEST_INVALID'


def test_블록_수_상한():
    res = check(design(*[(0, 0, 0, 'x')] * 55))
    assert reasons(res) == [(None, 'OUT_OF_SCOPE')]
    assert check(design(*[(0, 0, 15 * (i - 1), 'x') for i in range(1, 3)]))['ok']


def test_order_순서가_섞여_와도_order로_검사():
    req = design((0, 0, 0, 'x'), (0, 0, 15, 'x'))
    req['parts'].reverse()
    assert check(req)['ok']


def test_변환기_연결():
    """변환기 ①은 {recipe, placements} 두 문서를 돌려준다. 한 문서만 준 답은 ERROR."""
    bench = base_design('001_CHAIR_BENCH_V000')
    calls = []
    res = check(bench, parts_to_recipe=lambda blocks: calls.append(blocks) or CONVERTED)
    assert res['ok'] and (res['recipe'], res['placements']) == (CONVERTED['recipe'], CONVERTED['placements'])
    assert calls == [{k: v for k, v in bench.items() if k != 'base_recipe'}]       # 변환기 ①에는 base_recipe 를 뗀 사본이 간다
    assert 'recipe' not in check(bench)
    bad = check(design((0, 0, 0, 'x'), (0, 0, 20, 'x')), parts_to_recipe=lambda b: calls.append(b))
    assert not bad['ok'] and len(calls) == 1                           # 통과 못 하면 변환기를 부르지 않는다
    boom = check(bench, parts_to_recipe=lambda b: 1 / 0)
    assert not boom['ok'] and boom['errors'][0]['reason'] == 'ERROR'


def test_robot_yaml에_키가_없으면_만들_때_알림():
    cfg = copy.deepcopy(CFG)
    del cfg['check']
    with pytest.raises(ValueError, match='check'):
        DesignChecker(cfg)


def test_결과는_JSON으로_직렬화된다():
    json.dumps(check(base_design('002_CHAIR_BACK_V000'), parts_to_recipe=lambda b: {}))


# ---------------- check_design 서비스 한 번 처리(handle_json) ----------------
def handle(req_text, **kw):
    ok, reason, text = DesignChecker(CFG, **kw).handle_json(req_text)
    return ok, reason, json.loads(text)


def test_서비스_합격은_success_true와_레시피():
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), parts_to_recipe=lambda b: CONVERTED)
    assert (ok, reason) == (True, '') and res['ok'] and (res['recipe'], res['placements']) == (CONVERTED['recipe'], CONVERTED['placements'])


def test_서비스_설계_불합격은_success_true_ok_false():
    ok, reason, res = handle(json.dumps(design((0, 0, 0, 'x'), (0, 0, 20, 'x'))), parts_to_recipe=lambda b: {})
    assert (ok, reason) == (True, '') and not res['ok'] and res['errors']


def test_서비스_변환기_미연결이면_합격이어도_success_false():
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')))
    assert (ok, reason) == (False, 'ERROR') and not res['ok'] and 'recipe' not in res
    assert res['min_margin_mm'] == 12.5 and '변환기' in res['errors'][0]['detail']


def test_서비스_변환기_미연결이어도_불합격은_검사_이유_그대로():
    ok, reason, res = handle(json.dumps(design((0, 0, 0, 'x'), (0, 0, 20, 'x'))), template_checker=NO_TEMPLATE)
    assert (ok, reason) == (True, '') and not res['ok'] and res['errors'][0]['reason'] == 'CHECK_FAILED'


@pytest.mark.parametrize('text', ['{깨진', '[]', '"글자"', 'null'])
def test_서비스_JSON이_아니거나_객체가_아니면_REQUEST_INVALID(text):
    """봉투 오류는 웹 쪽 잘못 — 서비스는 처리했으므로 success true, kind 없음(재생성 안 함)."""
    ok, reason, res = handle(text)
    assert (ok, reason, res['ok']) == (True, '', False) and [(e['reason'], 'kind' in e) for e in res['errors']] == [('REQUEST_INVALID', False)]


def test_서비스_안쪽_예외도_ERROR():
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), parts_to_recipe=lambda b: float('nan'))
    assert (ok, reason) == (False, 'ERROR')           # NaN 은 allow_nan=False 로 직렬화에서 걸린다


def test_서비스_응답은_순수_JSON():
    _, _, text = DesignChecker(CFG, parts_to_recipe=lambda b: CONVERTED).handle_json(json.dumps(base_design('002_CHAIR_BACK_V000')))
    assert json.loads(text, parse_constant=lambda c: pytest.fail(c))['ok']


def test_서비스_변환기_예외는_success_false():
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), parts_to_recipe=lambda b: 1 / 0)
    assert (ok, reason) == (False, 'ERROR') and not res['ok'] and '변환기' in res['errors'][0]['detail']


@pytest.mark.parametrize('bad_recipe', [None, [], 'x', {}, {'schema': 'other/1'}, {'schema': 'assembly.recipe/1.0'}])
def test_서비스_변환기_결과가_레시피_객체가_아니면_ERROR(bad_recipe):
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), parts_to_recipe=lambda b: bad_recipe)
    assert (ok, reason) == (False, 'ERROR') and not res['ok'] and 'recipe' not in res


@pytest.mark.parametrize('schema', ['other/1', 'blocks/2', 'blocks/2.5', 'blocks/3.0', None, 5])
def test_서비스_schema가_parts_3이_아니면_REQUEST_INVALID(schema):
    req = base_design('001_CHAIR_BENCH_V000')
    req['schema'] = schema
    ok, reason, res = handle(json.dumps(req), parts_to_recipe=lambda b: CONVERTED)
    assert (ok, reason, res['ok']) == (True, '', False) and res['errors'][0]['reason'] == 'REQUEST_INVALID'
    assert repr(schema) in res['errors'][0]['detail']                   # 받은 schema 를 이유에 적는다


def test_서비스_parts_2_0도_REQUEST_INVALID():
    req = dict(base_design('001_CHAIR_BENCH_V000'), schema='parts/2.0')
    ok, reason, res = handle(json.dumps(req))
    assert (ok, reason) == (True, '') and res['errors'][0]['reason'] == 'REQUEST_INVALID' and 'parts/2.0' in res['errors'][0]['detail']


def test_서비스_schema_없으면_REQUEST_INVALID():
    req = base_design('001_CHAIR_BENCH_V000')
    del req['schema']
    assert handle(json.dumps(req), parts_to_recipe=lambda b: {})[:2] == (True, '')


@pytest.mark.parametrize('literal', ['NaN', 'Infinity', '-Infinity'])
def test_서비스_유한하지_않은_수는_설계_불합격이_아니라_REQUEST_INVALID(literal):
    text = json.dumps(base_design('001_CHAIR_BENCH_V000')).replace('"x": -25', f'"x": {literal}', 1)
    assert literal in text
    ok, reason, res = handle(text, parts_to_recipe=lambda b: CONVERTED)
    assert (ok, reason, res['ok']) == (True, '', False) and res['errors'][0]['reason'] == 'REQUEST_INVALID' and literal in res['errors'][0]['detail']


def test_서비스_오류_응답도_순수_JSON():
    _, _, text = DesignChecker(CFG).handle_json('{"schema":"blocks/2","blocks":[{"x":NaN}]}')
    json.loads(text, parse_constant=lambda c: pytest.fail(c))


@pytest.mark.parametrize('value', [[], {}, None, 5, 'q', [1]])
def test_서비스_ori가_이상한_타입이어도_TypeError_없이_부품_칸_오류(value):
    """부품별 칸 오류는 봉투 오류가 아니다 — CHECK_FAILED + position, 서비스는 성공."""
    req = base_design('001_CHAIR_BENCH_V000')
    req['parts'][0]['ori'] = value
    ok, reason, res = handle(json.dumps(req), parts_to_recipe=lambda b: CONVERTED)
    assert (ok, reason, res['ok']) == (True, '', False) and [(e['reason'], e['kind']) for e in res['errors']] == [('CHECK_FAILED', 'position')]


@pytest.mark.parametrize('literal', ['1e999', '-1e999', '1E400'])
def test_서비스_읽으면_무한대가_되는_수도_REQUEST_INVALID(literal):
    text = json.dumps(base_design('001_CHAIR_BENCH_V000')).replace('"x": -25', f'"x": {literal}', 1)
    assert literal in text
    ok, reason, res = handle(text, parts_to_recipe=lambda b: CONVERTED)
    assert (ok, reason, res['ok']) == (True, '', False) and res['errors'][0]['reason'] == 'REQUEST_INVALID' and literal in res['errors'][0]['detail']


def test_변환기가_옛_한_파일만_돌려주면_ERROR():
    """recipe(구조)만 있는 답(placements 없음)은 합격 결과로 내보내지 않는다."""
    res = check(base_design('001_CHAIR_BENCH_V000'), parts_to_recipe=lambda b: CONVERTED['recipe'])
    assert not res['ok'] and res['errors'][0]['reason'] == 'ERROR'


# ---------- parts/4.0 의 AI 칸(role · part · stage · grasp) 형식 검사 (E-69) ----------
def ai_blocks(n=3):
    """쌓인 블록 n개(아랫면 0, 15, 30)의 정상 parts/4.0."""
    return design(*[(0, 0, 15 * i, 'x') for i in range(n)])


def one_error_detail(req):
    res = check(req)
    assert not res['ok'] and res['errors'] and all(e['reason'] == 'CHECK_FAILED' for e in res['errors']), res
    return res['errors'][0]['detail']


@pytest.mark.parametrize('key', ['role', 'group', 'stage', 'grasp'])
def test_AI_칸이_빠지면_CHECK_FAILED이고_코드가_대신_채우지_않는다(key):
    req = ai_blocks()
    del req['parts'][1][key]
    assert key in one_error_detail(req) and '없음' in one_error_detail(req)


@pytest.mark.parametrize('role', ['leg', 'ARM_REST_X', 'LEG_', '_LEG', 'LEG1', '', 5, None])
def test_역할_형식이_틀리면_거절(role):
    req = ai_blocks()
    req['parts'][0]['role'] = role
    assert 'role' in one_error_detail(req)


@pytest.mark.parametrize('role', ['LEG', 'LEG_WHEEL', 'SEAT'])
def test_역할_한_단어_옵션_0_1개는_통과(role):
    req = ai_blocks()
    req['parts'][0]['role'] = role
    assert check(req)['ok']


@pytest.mark.parametrize('key, value', [('group', 0), ('group', -1), ('group', True), ('group', 1.5), ('group', '1'),
                                        ('stage', 0), ('stage', True), ('stage', None), ('grasp', 'SIDE_25'), ('grasp', 'flat_short'), ('grasp', None)])
def test_part_stage_grasp_값이_틀리면_거절(key, value):
    req = ai_blocks()
    req['parts'][1][key] = value
    assert key in one_error_detail(req)


@pytest.mark.parametrize('stages', [[2, 2, 3], [1, 3, 3], [1, 2, 1], [2, 3, 4]])
def test_단계는_1부터_같거나_1씩_늘어야_한다(stages):
    req = ai_blocks()
    for b, st in zip(req['parts'], stages):
        b['stage'] = st
    assert 'stage' in one_error_detail(req)


@pytest.mark.parametrize('stages', [[1, 1, 1], [1, 1, 2], [1, 2, 3], [1, 2, 2]])
def test_단계가_같거나_1씩_늘면_통과(stages):
    req = ai_blocks()
    for b, st in zip(req['parts'], stages):
        b['stage'] = st
    assert check(req)['ok']


def test_inferred는_선택이고_불리언이어야_한다():
    req = ai_blocks()
    req['parts'][2]['inferred'] = True
    assert check(req)['ok']
    req['parts'][2]['inferred'] = 'yes'
    assert 'inferred' in one_error_detail(req)


def test_옛_blocks_1은_AI_칸이_없어_서비스에서_거절():
    """E-69: blocks/2 는 스캔 추론기 출력(위치 · 방향만)이라 check_design 요청으로 받지 않는다 — 받은 schema 를 이유에 적어 REQUEST_INVALID(서비스는 성공)."""
    req = ai_blocks()
    req['schema'] = 'blocks/2'
    for b in req['parts']:
        for k in ('role', 'group', 'stage', 'grasp'):
            del b[k]
    ok, reason, text = DesignChecker(CFG).handle_json(json.dumps(req))
    assert (ok, reason) == (True, '') and 'blocks/2' in text and json.loads(text)['errors'][0]['reason'] == 'REQUEST_INVALID'


def test_변환기_결과의_짝_해시가_안_맞으면_ERROR():
    from d2_task.recipe_document import recipe_sha256  # noqa: F401  (해시 규칙은 test_recipe_document 가 본다)
    wrong = {'recipe': CONVERTED['recipe'], 'placements': dict(CONVERTED['placements'], recipe_sha256='0' * 64)}
    ok, reason, res = handle(json.dumps(base_design('001_CHAIR_BENCH_V000')), parts_to_recipe=lambda b: wrong)
    assert (ok, reason) == (False, 'ERROR') and not res['ok'] and 'recipe_sha256' in res['errors'][0]['detail']


# ---------------- W201 로봇 조립 가능(E-121) ----------------
def test_같은_층_2x2는_어떤_순서로도_조립_불가_position():
    square = [(-37.5, -12.5, 0, 'x'), (37.5, -12.5, 0, 'x'), (-37.5, 12.5, 0, 'x'), (37.5, 12.5, 0, 'x')]
    res = check(design(*square))
    assert not res['ok'] and res['feasible_order'] is None
    assert res['errors'] and all(e['kind'] == 'position' and '로봇 조립 불가(R5)' in e['detail'] for e in res['errors'])   # 스캔 [저장]은 다시 채우지 않고 멈춤


def test_한_층에_방향이_섞이면_R0_position():
    res = check(design((0, 0, 0, 'x'), (0, 100, 0, 'y')))
    assert [(e['part'], e['kind']) for e in res['errors']] == [(None, 'position')] and '로봇 조립 불가(R0)' in res['errors'][0]['detail']


@pytest.mark.parametrize('model_id', DESIGNS)
def test_기본_설계는_조립_가능_순서도_그대로(model_id):
    res = check(base_design(model_id))
    assert res['ok'] and res['feasible_order'] == sorted(p['sequence'] for p in base_design(model_id)['parts'])


def test_스캔은_격자_맞춤으로_판정만():
    bench = base_design('001_CHAIR_BENCH_V000')
    shaken = dict(bench, made_by='scan', parts=[dict(p, x=p['x'] + 0.1) for p in bench['parts']])
    res = check(shaken)
    assert res['feasible_order'] is not None                            # 0.1 mm 흔들림이어도 격자에 맞춰 '조립 가능'
    assert not any('로봇 조립 불가' in e['detail'] for e in res['errors'])

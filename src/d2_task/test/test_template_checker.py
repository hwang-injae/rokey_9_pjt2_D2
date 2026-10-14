# -*- coding: utf-8 -*-
"""Template 검사(21번, W162) 시험 — ROS 없이.

두 층: ① TemplateChecker 를 손으로 만든 작은 설계로 직접(식 · 맞닿은 묶음 · 오차 · 같은 크기 · 모양), ② 실제 기본 설계 4종을 DesignChecker 서비스 계층으로(요청 확인 · kind ·
기존 1~20번과 같이 모으기). 21번은 늘 켜져 있으므로 여기서는 대역(NO_TEMPLATE)을 쓰지 않는다.
"""
import copy
import json

import pytest

from d2_task import template_checker as tcm
from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_parts import ori_extents
from d2_task.template_checker import RequestInvalid, TemplateChecker
from test_design_checker import BLOCK_MM, CFG, CONVERTED, DESIGNS, FIXTURES, NO_TEMPLATE, base_design, design

BENCH = '001_CHAIR_BENCH_V000'


def real(**kw):
    """진짜 21번을 켠 검사 묶음(기본 생성자 그대로)."""
    return DesignChecker(CFG, **kw)


def reasons(res):
    return [e['reason'] for e in res['errors']]


def revision(req, role):
    """요청 안 base_recipe 의 그 역할 revision(고쳐 쓰는 용)."""
    return req['base_recipe']['template']['composition'][role]['revision']


# ---------------- 기본 4종 ----------------
@pytest.mark.parametrize('model_id', DESIGNS)
def test_새_fixture_4종은_schema_검사를_통과하고_규칙으로_읽힌다(model_id):
    document = RecipeDocument.load(FIXTURES, model_id)                 # recipe/4.0 · placements/4.0 · 짝 해시
    rules = TemplateChecker(ori_extents(BLOCK_MM)).read_base(dict(base_design(model_id)))
    composition = document.recipe['template']['composition']
    assert set(rules) == set(composition)
    assert {r: v['count'] for r, v in rules.items()} == {r: v['count'] for r, v in composition.items()}
    assert all(v['label'] == composition[r]['label'] for r, v in rules.items())


@pytest.mark.parametrize('model_id', DESIGNS)
def test_기본_설계_4종은_21번까지_통과한다(model_id):
    req = base_design(model_id)
    res = real().check(req)
    assert res['ok'] and res['errors'] == [] and res['min_margin_mm'] == DESIGNS[model_id][2]


@pytest.mark.parametrize('model_id', DESIGNS)
def test_서비스_응답에는_base_recipe가_없고_요청은_바뀌지_않는다(model_id):
    req = base_design(model_id)
    before = copy.deepcopy(req)
    ok, reason, text = real(parts_to_recipe=lambda r: CONVERTED).handle_json(json.dumps(req))
    assert (ok, reason) == (True, '') and 'base_recipe' not in json.loads(text)
    assert req == before
    res = real().check(req)                                            # 합격 · 불합격 모두
    assert req == before and 'base_recipe' not in res
    req['parts'][0]['z'] += 100
    snapshot = copy.deepcopy(req)
    assert 'base_recipe' not in real().check(req) and req == snapshot


def test_요청의_template_칸은_읽지_않는다():
    req = base_design(BENCH)
    req['template'] = {'composition': {'ZZZ': {'count': 9}}}
    assert real().check(req)['ok']
    req['base_recipe']['template']['name'] = None                      # 이름 · 설명 · 태그도 읽지 않는다
    assert real().check(req)['ok']


def test_역할_옵션은_앞_단어로_본다():
    req = base_design(BENCH)
    for p in req['parts']:
        if p['role'] == 'LEG' and p['group'] == 1:       # 묶음 전체의 옵션을 바꾼다(묶음 신원 = 전체 역할 + 번호이므로 일부만 바꾸면 다른 묶음이 된다)
            p['role'] = 'LEG_WHEEL'
    assert real().check(req)['ok']


# ---------------- 요청 확인 (REQUEST_INVALID) ----------------
def drop_base(req):
    del req['base_recipe']


def parent_mismatch(req):
    req['design_id'] = '002_CHAIR_BACK_V001'


@pytest.mark.parametrize('mutate', [
    drop_base,
    lambda r: r.update(base_recipe=None),
    lambda r: r.update(base_recipe=[]),
    lambda r: r.update(base_recipe='recipe'),
    lambda r: r['base_recipe'].update(schema='recipe/2.0'),
    lambda r: r['base_recipe'].update(schema='blocks/2'),
    lambda r: r['base_recipe'].pop('schema'),
    lambda r: r['base_recipe'].update(design_id='001_CHAIR_BENCH_V001'),
    lambda r: r['base_recipe'].update(design_id=None),
    lambda r: r['base_recipe'].update(design_id='_V000'),
    parent_mismatch,
    lambda r: r.update(design_id='001_CHAIR_BENCH'),
    lambda r: r.update(design_id='001_CHAIR_BENCH_V01'),
    lambda r: r.update(design_id=None),
    lambda r: r['base_recipe'].pop('template'),
    lambda r: r['base_recipe'].update(template=[]),
    lambda r: r['base_recipe']['template'].pop('composition'),
    lambda r: r['base_recipe']['template'].update(composition={}),
    lambda r: r['base_recipe']['template'].update(composition=[]),
], ids=['없음', 'null', '목록', '글자', 'recipe2', 'blocks2', 'schema없음', '파생model_id', 'model_id없음', '앞부분없음',
        'parent다름', 'design_id버전없음', 'design_id두자리', 'design_id없음', 'template없음', 'template목록', 'composition없음',
        'composition빈', 'composition목록'])
def test_사전_확인이_틀리면_REQUEST_INVALID_하나로_끝낸다(mutate):
    """서비스는 처리했으므로 success true, 설계 탓이 아니므로 재생성하지 않는다(kind 없음). 형식 오류가 같이 있어도 사전 확인이 먼저다."""
    req = base_design(BENCH)
    mutate(req)
    del req['parts'][0]['role']                                        # 뒤 검사가 있었다면 걸렸을 형식 오류
    ok, reason, text = real().handle_json(json.dumps(req))
    res = json.loads(text)
    assert (ok, reason, res['ok']) == (True, '', False)
    assert [(e['part'], e['reason']) for e in res['errors']] == [(None, 'REQUEST_INVALID')] and 'kind' not in res['errors'][0]
    assert res['errors'][0]['detail'] and res['min_margin_mm'] is None and 'recipe' not in res


def test_사전_확인이_틀리면_변환기와_21번은_부르지_않는다():
    req = base_design(BENCH)
    drop_base(req)
    calls = []

    def converter(request):
        calls.append(request)
        return CONVERTED
    assert real(parts_to_recipe=converter).check(req)['errors'][0]['reason'] == 'REQUEST_INVALID' and calls == []


def test_요청이_객체가_아니면_REQUEST_INVALID():
    assert reasons(real().check([])) == ['REQUEST_INVALID']


@pytest.mark.parametrize('mutate', [
    lambda c: c['LEG'].pop('count'),
    lambda c: c['LEG'].update(count=0),
    lambda c: c['LEG'].update(count=True),
    lambda c: c['LEG'].update(count='2'),
    lambda c: c['LEG'].pop('revision'),
    lambda c: c['LEG'].update(revision=[]),
    lambda c: c.update(leg=c.pop('LEG')),
    lambda c: c.update(LEG=[]),
    lambda c: c['LEG']['revision'].update(lay='FLAT'),
    lambda c: c['LEG']['revision'].update(lay=[]),
    lambda c: c['LEG']['revision'].update(lay={}),
    lambda c: c['LEG']['revision'].update(long_edge={'relation': [], 'to': 'SEAT'}),
    lambda c: c['LEG']['revision'].update(long_edge={'relation': {}, 'to': 'SEAT'}),
    lambda c: c['LEG']['revision'].update(long_edge={'relation': 'PARALLEL', 'to': []}),
    lambda c: c['LEG']['revision'].update(long_edge=[]),
    lambda c: c['LEG']['revision'].update(support=[[]]),
    lambda c: c['LEG']['revision'].update(support=[{}]),
    lambda c: c['LEG']['revision']['attach'].update(at=[]),
    lambda c: c['LEG']['revision']['attach'].update(at={}),
    lambda c: c['LEG']['revision']['attach'].update(to=[]),
    lambda c: c['LEG']['revision']['attach'].update(direction=[]),
    lambda c: c['LEG']['revision'].update(attach=[]),
    lambda c: c['LEG']['revision'].update(height_mm={}),
    lambda c: c['LEG']['revision'].update(height_mm=[{}, 1]),
    lambda c: c['LEG']['revision'].update(height_mm=[10 ** 400, 10 ** 401]),
    lambda c: c['SEAT']['revision'].update(mirror=[]),
    lambda c: c['SEAT']['revision'].update(mirror={}),
    lambda c: c['LEG'].update(count=[]),
    lambda c: c['LEG']['revision'].update(long_edge={'relation': 'SKEW', 'to': 'SEAT'}),
    lambda c: c['LEG']['revision'].update(long_edge={'relation': 'PARALLEL', 'to': 'NOPE'}),
    lambda c: c['LEG']['revision'].update(long_edge={'relation': 'PARALLEL', 'to': 'LEG'}),
    lambda c: c['LEG']['revision'].update(support=[]),
    lambda c: c['LEG']['revision'].update(support=['FLOOR']),
    lambda c: c['LEG']['revision']['attach'].update(at='MIDDLE'),
    lambda c: c['LEG']['revision']['attach'].update(direction='OUTWARD'),
    lambda c: c['LEG']['revision']['attach'].update(to='NOPE'),
    lambda c: c['LEG']['revision']['attach'].update(d_mm=[0]),
    lambda c: c['LEG']['revision']['attach'].update(d_mm=[5, 1]),
    lambda c: c['LEG']['revision']['attach'].update(d_mm=[True, 1]),
    lambda c: c['LEG']['revision']['attach'].update(d_mm=[0, None]),
    lambda c: c['LEG']['revision'].update(height_mm=[30]),
    lambda c: c['LEG']['revision'].update(height_mm='= NOPE.depth'),
    lambda c: c['LEG']['revision'].update(height_mm='= SEAT.size'),
    lambda c: c['LEG']['revision'].update(height_mm='SEAT.depth'),
    lambda c: c['LEG']['revision'].update(height_mm=30),
    lambda c: c['LEG']['revision'].update(same_size='yes'),
    lambda c: c['SEAT']['revision'].update(mirror='Z'),
    lambda c: c['LEG']['revision']['attach'].update(d_mm=[0, float('nan')]),
    lambda c: c['LEG']['revision']['attach'].update(d_mm=[0, float('inf')]),
])
def test_revision_모양이_틀리면_REQUEST_INVALID(mutate):
    req = base_design(BENCH)
    mutate(req['base_recipe']['template']['composition'])
    assert reasons(real().check(req)) == ['REQUEST_INVALID']


@pytest.mark.parametrize('expr', [
    '__import__("os").system("echo")', 'open("/etc/passwd")', 'SEAT.width ** 2', 'SEAT.width // 2', 'SEAT.width % 2',
    'abs(1)', 'SEAT.__class__', 'SEAT.width.real', 'SEAT', 'NOPE.width', 'seat.width', 'SEAT.weight', 'SEAT["width"]',
    '[1, 2][0]', '(lambda: 1)()', 'True', '1 if 1 else 2', '"75"', 'f"{1}"', '1 < 2', '1j', '10**400', '1e999', 'SEAT.width; 1', '',
    '(' * 150 + '1' + ')' * 150, 'x' * 300, '1 +', '-' * 500 + '1', '[SEAT.width]', 'LEG.height', 'SEAT.width if SEAT else 1',
])
def test_위험하거나_허용되지_않는_식은_거절한다(expr):
    req = base_design(BENCH)
    revision(req, 'LEG')['attach']['d_mm'] = [0, expr]
    assert reasons(real().check(req)) == ['REQUEST_INVALID']


def test_식은_eval을_쓰지_않는다():
    source = (tcm.__file__ and open(tcm.__file__, encoding='utf-8').read())
    assert 'eval(' not in source.replace('_eval(', '').replace('def _eval', '') and 'exec(' not in source


@pytest.mark.parametrize('expr, value', [('SEAT.width / 2 - 25', 12.5), ('-SEAT.depth + 100', 25.0), ('(SEAT.width + 25) * 2 / 4', 50.0),
                                         ('+5', 5.0), ('SEAT.depth*SEAT.depth/75', 75.0)])
def test_허용된_식은_계산한다(expr, value):
    req = base_design(BENCH)
    revision(req, 'LEG')['attach']['d_mm'] = [0, expr]                   # 다리는 좌판 끝에 붙어 d = 0 → 식이 0 이상이면 통과
    assert real().check(req)['ok']
    revision(req, 'LEG')['attach']['d_mm'] = [expr, 100]                 # 최소 = 식 값 > 0 이라 d = 0 이 벗어난다 — 문구에 계산한 값이 나온다
    res = real().check(req)
    assert reasons(res) == ['TEMPLATE_RANGE'] * 2 and all(f'범위 {tcm._n(value)} ~ 100 mm' in e['detail'] for e in res['errors'])


@pytest.mark.parametrize('expr', ['SEAT.width / 0', '10 / (5 - 5)', '1 / 0.0', 'SEAT.width / (2 - 2)', '1e308 * 10', '1e308 * 1e308 + SEAT.width',
                                  '10 / (SEAT.depth - 75)', 'SEAT.width * 1e308 * 1e308'])
def test_계산되지_않는_식은_AI_탓이_아니라_잘못된_규칙_REQUEST_INVALID(expr):
    """읽을 때 알 수 있는 0 나눔 · 넘침은 읽기에서, 설계 값이 있어야 아는 것(분모가 좌판 깊이 - 75 = 0 등)은 계산에서 — 둘 다 같은 REQUEST_INVALID, 재생성하지 않는다."""
    req = base_design(BENCH)
    revision(req, 'LEG')['attach']['d_mm'] = [0, expr]
    ok, reason, text = real().handle_json(json.dumps(req))
    res = json.loads(text)
    assert (ok, reason, res['ok']) == (True, '', False)
    assert [(e['reason'], 'kind' in e) for e in res['errors']] == [('REQUEST_INVALID', False)]


def test_모르는_선택_칸은_무시하고_아는_칸_검사는_그대로():
    """형식 버전 규칙: recipe/4.1 + 모르는 선택 칸(note 등)은 호환 변경. 새 검사로 해석하지 않는다."""
    req = base_design(BENCH)
    req['base_recipe']['schema'] = 'recipe/4.1'
    revision(req, 'LEG')['note'] = 'optional'
    revision(req, 'LEG')['speed'] = 1
    revision(req, 'SEAT')['future'] = {'a': [1, 2]}
    req['base_recipe']['template']['composition']['LEG']['extra'] = None
    assert real().check(req)['ok']
    revision(req, 'LEG')['height_mm'] = [30, 45]                          # 아는 칸은 그대로 걸린다
    assert reasons(real().check(req)) == ['TEMPLATE_RANGE'] * 2
    revision(req, 'LEG')['lay'] = []                                      # 아는 칸의 타입 오류는 여전히 요청 오류
    assert reasons(real().check(req)) == ['REQUEST_INVALID']


# ---------------- 범위 · 같음 · 대칭 (TEMPLATE_RANGE) ----------------
def test_범위_밖은_TEMPLATE_RANGE이고_화면_문구가_label과_범위를_담는다():
    req = base_design(BENCH)
    revision(req, 'LEG')['height_mm'] = [30, 45]                        # 다리 높이는 60
    res = real().check(req)
    assert not res['ok'] and reasons(res) == ['TEMPLATE_RANGE'] * 2     # 다리 묶음 둘 다
    first = res['errors'][0]
    assert first['kind'] == 'template' and first['part'] is None
    assert '다리' in first['detail'] and '높이' in first['detail'] and '60 mm' in first['detail'] and '30 ~ 45 mm' in first['detail']


def test_범위_경계는_오차_1e_6_mm_안이면_통과():
    req = base_design(BENCH)
    revision(req, 'LEG')['height_mm'] = [30, 59.9999995]                 # 0.5e-6 mm 위 — 같은 값으로 본다
    assert real().check(req)['ok']
    revision(req, 'LEG')['height_mm'] = [30, 59.999998]                  # 2e-6 mm 위 — 벗어났다
    assert reasons(real().check(req)) == ['TEMPLATE_RANGE'] * 2
    revision(req, 'LEG')['height_mm'] = [60.0000005, 90]
    assert real().check(req)['ok']
    revision(req, 'LEG')['height_mm'] = [60.000002, 90]
    assert not real().check(req)['ok']


def test_같음_규칙은_크기와_양_끝_맞춤을_둘_다_본다():
    req = base_design('004_DESK_PEDESTAL_V000')
    assert revision(req, 'BASE')['depth_mm'] == '= TOP.depth'
    shifted = copy.deepcopy(req)
    for p in shifted['parts']:
        if p['role'] == 'BASE':
            p['y'] += 5                                                 # 크기는 같지만 앞뒤 끝이 5 mm 어긋남
    res = real().check(shifted)
    assert any(e['reason'] == 'TEMPLATE_RANGE' and '양 끝' in e['detail'] and '바닥 받침' in e['detail'] for e in res['errors'])
    revision(req, 'BASE')['depth_mm'] = '= COLUMN.width'                  # 기둥 폭과 같아야 한다고 바꾸면 크기가 다르다
    res = real().check(req)
    assert any('같아야 한다' in e['detail'] for e in res['errors'])


def test_대칭_규칙():
    req = base_design(BENCH)
    for p in req['parts']:
        if p['role'] == 'LEG' and p['group'] == 2:
            p['x'] += 3                                                 # 다리 한 쪽만 3 mm 밀면 좌판 가운데 기준 좌우 대칭이 깨진다
    res = real().check(req)
    assert any(e['reason'] == 'TEMPLATE_RANGE' and '대칭' in e['detail'] and '좌우' in e['detail'] for e in res['errors'])


def test_XY_대칭은_앞뒤도_본다():
    req = base_design('003_DESK_STAND_V000')
    assert revision(req, 'TOP')['mirror'] == 'XY'
    for p in req['parts']:
        if p['role'] == 'BEAM' and p['group'] == 2:
            p['y'] += 3
    res = real().check(req)
    assert any('앞뒤' in e['detail'] and '대칭' in e['detail'] for e in res['errors'])


# ---------------- 모양 (TEMPLATE_SHAPE) ----------------
def shape_details(req):
    res = real().check(req)
    assert not res['ok'], res
    return res, [e['detail'] for e in res['errors'] if e['reason'] == 'TEMPLATE_SHAPE']


def test_역할_목록이_다르면_SHAPE():
    req = base_design(BENCH)
    req['parts'][0]['role'] = 'BACK'
    res, details = shape_details(req)
    assert any('없는 역할' in d and 'BACK' in d for d in details)
    req = base_design(BENCH)
    req['base_recipe']['template']['composition']['BEAM'] = {'label': '보', 'count': 1, 'revision': {}}
    assert any('보(BEAM) 부품이 없다' in d for d in shape_details(req)[1])


def test_개수가_다르면_SHAPE():
    req = base_design(BENCH)
    req['base_recipe']['template']['composition']['LEG']['count'] = 3
    assert any('다리 개수 2개 — Template 은 3개' in d for d in shape_details(req)[1])


@pytest.mark.parametrize('role, lay', [('SEAT', 'FLAT_STACK'), ('LEG', 'FLAT_SIDE_BY_SIDE'), ('LEG', 'STAND_STACK')])
def test_놓는_방향과_쌓는_모양이_다르면_SHAPE(role, lay):
    req = base_design(BENCH)
    revision(req, role)['lay'] = lay
    assert shape_details(req)[1]


def test_세운_다리_Template에_눕힌_설계를_내면_SHAPE():
    req = base_design('003_DESK_STAND_V000')
    revision(req, 'LEG')['lay'] = 'FLAT_STACK'
    assert any('놓는 방향이 눕혀 위로 쌓기 가 아니다' in d for d in shape_details(req)[1])


def test_받침이_맞지_않으면_SHAPE():
    req = base_design(BENCH)
    revision(req, 'SEAT')['support'] = ['TABLE']                         # 좌판은 바닥이 아니라 다리 위
    assert any('좌판 받침이 맞지 않다' in d and '작업면' in d for d in shape_details(req)[1])
    req = base_design(BENCH)
    revision(req, 'LEG')['support'] = ['SEAT']                           # 다리는 좌판 위가 아니다
    assert any('다리' in d and '받침이 맞지 않다' in d for d in shape_details(req)[1])


@pytest.mark.parametrize('relation, role, to', [('PARALLEL', 'LEG', 'SEAT'), ('PERPENDICULAR', 'BACK', 'SEAT')])
def test_긴_변_방향이_다르면_SHAPE(relation, role, to):
    req = base_design('002_CHAIR_BACK_V000')
    revision(req, role)['long_edge'] = {'relation': relation, 'to': to}
    assert any('긴 변' in d for d in shape_details(req)[1])


def test_붙는_자리_TOP이_안_맞으면_SHAPE():
    req = base_design(BENCH)
    revision(req, 'SEAT')['attach'] = {'to': 'LEG', 'at': 'TOP'}
    assert real().check(req)['ok']
    for p in req['parts']:
        if p['role'] == 'SEAT':
            p['z'] += 15                                                # 좌판이 뜬다 — 물리 실패와 21번 실패가 같이 나온다
    res = real().check(req)
    assert {'CHECK_FAILED', 'TEMPLATE_SHAPE'} <= set(reasons(res))


# ---------------- 물리 실패와 21번을 같이 모으기 + kind ----------------
def test_물리_실패가_있어도_21번을_건너뛰지_않고_둘_다_모은다():
    req = base_design(BENCH)
    req['parts'][-1]['z'] += 40                                          # 좌판 부품 하나가 뜬다(공중) → position
    revision(req, 'LEG')['height_mm'] = [30, 45]                         # 그리고 다리 높이도 범위 밖 → template
    res = real().check(req)
    assert not res['ok']
    kinds = {e['reason']: e['kind'] for e in res['errors']}
    assert kinds['CHECK_FAILED'] == 'position' and kinds['TEMPLATE_RANGE'] == 'template'
    assert reasons(res).index('CHECK_FAILED') < reasons(res).index('TEMPLATE_RANGE')      # 기존 검사 → 21번 순


def test_REQUEST_INVALID이면_1_20번도_돌리지_않는다():
    req = base_design(BENCH)
    req['parts'][-1]['z'] += 40
    drop_base(req)
    assert reasons(real().check(req)) == ['REQUEST_INVALID']


def test_형식이_틀리면_21번은_읽을_수_없어_형식_실패만_돌려준다():
    req = base_design(BENCH)
    del req['parts'][0]['ori']
    res = real().check(req)
    assert reasons(res) == ['CHECK_FAILED'] and res['errors'][0]['kind'] == 'position'


@pytest.mark.parametrize('mutate, kind', [
    (lambda r: r['parts'][0].update(x='1'), 'position'),
    (lambda r: r['parts'][0].update(ori='q'), 'position'),
    (lambda r: r['parts'][0].pop('ori'), 'position'),
    (lambda r: r['parts'][0].pop('role'), 'ai_fields'),
    (lambda r: r['parts'][0].update(role='leg'), 'ai_fields'),
    (lambda r: r['parts'][0].update(group=0), 'ai_fields'),
    (lambda r: r['parts'][0].update(stage=True), 'ai_fields'),
    (lambda r: r['parts'][0].update(grasp='SIDE'), 'ai_fields'),
    (lambda r: r['parts'][1].update(sequence=99), 'ai_fields'),
    (lambda r: r.update(parts=r['parts'] * 6), 'scope'),
])
def test_기존_형식_실패에도_kind가_붙는다(mutate, kind):
    req = base_design(BENCH)
    mutate(req)
    res = real().check(req)
    assert not res['ok'] and {e['kind'] for e in res['errors']} == {kind}


def test_물리_실패_kind():
    kinds = lambda r: {e['kind'] for e in real().check(r)['errors']}
    floating = base_design(BENCH)
    floating['parts'][-1]['z'] += 40
    assert 'position' in kinds(floating)                                  # 공중
    outside = base_design(BENCH)
    for p in outside['parts']:
        p['x'] += 400
    assert 'position' in kinds(outside)                                   # 작업영역 밖 · 작업대 밖
    overlap = base_design(BENCH)
    overlap['parts'][1].update(x=overlap['parts'][0]['x'], y=overlap['parts'][0]['y'], z=overlap['parts'][0]['z'])
    assert 'position' in kinds(overlap)                                   # 겹침
    tipping = base_design('003_DESK_STAND_V000')
    for p in tipping['parts']:
        if p['role'] == 'TOP':
            p['x'] += 60
    assert 'position' in kinds(tipping)                                   # 쌓는 도중 안정성


def test_잡을_면_없음은_ai_fields이다():
    corner = [(37.5, 0, 0, 'x'), (-37.5, 25, 0, 'x'), (-37.5, 0, 0, 'x')]     # 같은 층 L 자 — 모서리가 마지막이면 막힘, 다른 순서는 됨(W201)
    res = DesignChecker(CFG, template_checker=NO_TEMPLATE).check(design(*corner))
    assert [(e['reason'], e['kind']) for e in res['errors']] == [('CHECK_FAILED', 'ai_fields')]


def test_변환기_예외와_REQUEST_INVALID에는_kind가_없다():
    res = real(parts_to_recipe=lambda r: 1 / 0).check(base_design(BENCH))
    assert reasons(res) == ['ERROR'] and 'kind' not in res['errors'][0]


# ---------------- 서비스 계층 ----------------
def test_서비스_21번_불합격은_success_true_ok_false():
    req = base_design(BENCH)
    revision(req, 'LEG')['height_mm'] = [30, 45]
    ok, reason, text = real(parts_to_recipe=lambda r: CONVERTED).handle_json(json.dumps(req))
    res = json.loads(text)
    assert (ok, reason, res['ok']) == (True, '', False) and 'recipe' not in res and 'base_recipe' not in res


def test_서비스_21번_안의_예기치_않은_예외는_숨기지_않고_ERROR():
    class Boom:
        def read_base(self, request):
            return {}

        def check(self, rules, parts):
            raise ZeroDivisionError('boom')
    req = base_design(BENCH)
    with pytest.raises(ZeroDivisionError):
        DesignChecker(CFG, template_checker=Boom()).check(req)           # check() 는 삼키지 않는다
    ok, reason, text = DesignChecker(CFG, template_checker=Boom()).handle_json(json.dumps(req))
    assert (ok, reason) == (False, 'ERROR') and 'boom' in text


def test_서비스_revision_모양이_틀려도_ERROR가_아니라_REQUEST_INVALID():
    req = base_design(BENCH)
    revision(req, 'LEG')['lay'] = 'DIAGONAL'
    ok, reason, text = real().handle_json(json.dumps(req))
    assert (ok, reason) == (True, '') and json.loads(text)['errors'][0]['reason'] == 'REQUEST_INVALID'


def test_서비스_21번_합격_응답은_recipe_placements를_담고_base_recipe는_없다():
    req = base_design(BENCH)
    ok, reason, text = real(parts_to_recipe=lambda r: CONVERTED).handle_json(json.dumps(req))
    res = json.loads(text)
    assert (ok, reason, res['ok']) == (True, '', True) and 'recipe' in res and 'base_recipe' not in res


# ---------------- TemplateChecker 직접: 작은 설계 ----------------
def rig(composition, design_id='X_V001'):
    """composition(역할 → {count, revision, label?}) 으로 만든 규칙."""
    base = {'schema': 'recipe/4.0', 'design_id': 'X_V000', 'template': {'composition': composition}}
    return TemplateChecker(ori_extents(BLOCK_MM)).read_base({'design_id': design_id, 'base_recipe': base})


def put(order, x, y, z, ori, role, group):
    return {'sequence': order, 'x': x, 'y': y, 'z': z, 'ori': ori, 'role': role, 'group': group}


def measure(composition, parts):
    return TemplateChecker(ori_extents(BLOCK_MM)).check(rig(composition), parts)


def test_같은_역할_그룹이_여럿이면_맞닿은_상대_그룹으로_잰다():
    """POST 둘은 폭이 같지만, 각자 맞닿은 CAP 의 폭(75 · 25)과 비교한다. 첫 번째 CAP 만 쓰면 둘째가 잘못 통과한다."""
    composition = {'POST': {'count': 2, 'label': '기둥', 'revision': {'width_mm': [0, 'CAP.width']}}, 'CAP': {'count': 2, 'label': '덮개', 'revision': {}}}
    parts = [put(1, -100, 0, 0, 'x', 'POST', 1), put(2, 100, 0, 0, 'x', 'POST', 2),            # 덮개는 같은 방향(y) — 그룹 2 는 부품 셋을 나란히 놓아 더 넓다
             put(3, -100, 0, 15, 'y', 'CAP', 1), put(4, 75, 0, 15, 'y', 'CAP', 2), put(5, 100, 0, 15, 'y', 'CAP', 2), put(6, 125, 0, 15, 'y', 'CAP', 2)]
    errors = measure(composition, parts)
    assert [(e['reason'], e['kind']) for e in errors] == [('TEMPLATE_RANGE', 'template')]
    assert '기둥(그룹 1)' in errors[0]['detail'] and '75 mm' in errors[0]['detail'] and '0 ~ 25 mm' in errors[0]['detail']


def test_맞닿은_상대가_없고_상대_그룹이_여럿이면_한_번만_알린다():
    composition = {'POST': {'count': 1, 'label': '기둥', 'revision': {'width_mm': [0, 'CAP.width'], 'depth_mm': [0, 'CAP.depth']}},
                   'CAP': {'count': 2, 'label': '덮개', 'revision': {}}}
    parts = [put(1, 0, 0, 0, 'x', 'POST', 1), put(2, -200, 0, 0, 'x', 'CAP', 1), put(3, 200, 0, 0, 'x', 'CAP', 2)]
    errors = measure(composition, parts)
    assert [e['reason'] for e in errors] == ['TEMPLATE_SHAPE'] and '맞닿은 덮개' in errors[0]['detail']


def test_같은_크기_실패는_먼저_알리고_그_역할을_쓰는_식은_건너뛴다():
    composition = {'POST': {'count': 2, 'label': '기둥', 'revision': {'same_size': True}},
                   'CAP': {'count': 1, 'label': '덮개', 'revision': {'width_mm': '= POST.width', 'depth_mm': [0, 'POST.depth / 2']}}}
    parts = [put(1, -100, 0, 0, 'x', 'POST', 1), put(2, 100, 0, 0, 'x', 'POST', 2), put(3, 100, 25, 0, 'x', 'POST', 2),     # 그룹 2 는 두 부품(75 × 50)
             put(4, -100, 0, 15, 'x', 'CAP', 1)]
    errors = measure(composition, parts)
    assert [e['reason'] for e in errors] == ['TEMPLATE_RANGE'] and '기둥 크기가 서로 다르다' in errors[0]['detail']


def test_작업면_받침은_아랫면_z가_0이어야_한다():
    composition = {'POST': {'count': 1, 'label': '기둥', 'revision': {'support': ['TABLE']}}}
    assert measure(composition, [put(1, 0, 0, 0, 'x', 'POST', 1)]) == []
    errors = measure(composition, [put(1, 0, 0, 15, 'x', 'POST', 1)])
    assert [e['reason'] for e in errors] == ['TEMPLATE_SHAPE'] and '작업면' in errors[0]['detail']
    assert measure(composition, [put(1, 0, 0, 0.0000005, 'x', 'POST', 1)]) == []        # 오차 안


def test_같은_역할_묶음_모양_STACK은_이어진_층_SIDE_BY_SIDE는_한_층():
    stacked = [put(1, 0, 0, 0, 'x', 'POST', 1), put(2, 0, 0, 15, 'x', 'POST', 1)]
    side = [put(1, 0, 0, 0, 'x', 'POST', 1), put(2, 0, 25, 0, 'x', 'POST', 1)]
    for lay, ok_parts, bad_parts in (('FLAT_STACK', stacked, side), ('FLAT_SIDE_BY_SIDE', side, stacked)):
        composition = {'POST': {'count': 1, 'label': '기둥', 'revision': {'lay': lay}}}
        assert measure(composition, ok_parts) == []
        assert [e['reason'] for e in measure(composition, bad_parts)] == ['TEMPLATE_SHAPE']


def test_21번_실패_줄은_모두_template_kind이고_part는_비어_있다():
    composition = {'POST': {'count': 2, 'label': '기둥', 'revision': {'lay': 'STAND_STACK', 'height_mm': [0, 1]}}}
    errors = measure(composition, [put(1, 0, 0, 0, 'x', 'POST', 1)])
    assert errors and all(e['kind'] == 'template' and e['part'] is None and e['detail'].startswith('21번') for e in errors)


def test_읽기_함수의_예외_이름():
    with pytest.raises(RequestInvalid, match='base_recipe'):
        TemplateChecker(ori_extents(BLOCK_MM)).read_base({'design_id': 'X_V001'})


# ---------------- 독립 리뷰 재현 (10/13) ----------------
def test_한_역할의_부품_방향이_섞이면_SHAPE와_template_kind():
    """003 다리 4그룹 중 1그룹만 zy → zx. 예전에는 크기 RANGE 만 나왔다. 물리 실패가 이미 있어도 21번 실패를 같이 모은다."""
    req = base_design('003_DESK_STAND_V000')
    legs = [p for p in req['parts'] if p['role'] == 'LEG']
    assert len({p['ori'] for p in legs}) == 1
    original = legs[0]['ori']
    flipped = 'zy' if original == 'zx' else 'zx'                              # 한 그룹만 다른 세움 방향으로
    for p in legs:
        if p['group'] == 2:
            p['ori'] = flipped
    res = real().check(req)
    shape = [e for e in res['errors'] if e['reason'] == 'TEMPLATE_SHAPE' and '놓는 방향이 모두 같지 않다' in e['detail']]
    assert len(shape) == 1 and shape[0]['kind'] == 'template' and original in shape[0]['detail'] and flipped in shape[0]['detail']
    req2 = copy.deepcopy(req)
    req2['parts'][-1]['z'] += 40                                              # 물리(공중) 실패를 더해도 SHAPE 줄은 그대로 나온다
    res2 = real().check(req2)
    assert {e['reason'] for e in res2['errors']} >= {'CHECK_FAILED', 'TEMPLATE_SHAPE'}
    assert any('놓는 방향이 모두 같지 않다' in e['detail'] for e in res2['errors'])


# ---------------- 역할 옵션이 다른 묶음은 번호가 같아도 다른 묶음 ----------------
def with_wheel_leg(req, group_numbers=(1, 1)):
    """벤치의 두 다리를 (LEG, 번호 a) · (LEG_WHEEL, 번호 b) 로 바꾼다 — 기하는 그대로(변환기 ①은 (전체 역할, 번호)로 묶는다)."""
    for p in req['parts']:
        if p['role'] == 'LEG' and p['group'] == 2:
            p['role'], p['group'] = 'LEG_WHEEL', group_numbers[1]
        elif p['role'] == 'LEG':
            p['group'] = group_numbers[0]
    return req


def real_service():
    """task_node 와 같은 배선(진짜 변환기 ① + 진짜 21번)의 check_design 서비스."""
    from d2_task.parts_to_recipe import PartsToRecipe
    checker = DesignChecker(CFG)
    checker.parts_to_recipe = PartsToRecipe(checker.grasp_options, BLOCK_MM).convert
    return checker


@pytest.mark.parametrize('numbers', [(1, 1), (1, 2), (2, 1)], ids=['번호같음', '번호다름', '뒤바뀜'])
def test_옵션이_다른_두_다리는_번호가_같아도_두_묶음이다(numbers):
    req = with_wheel_leg(base_design(BENCH), numbers)
    ok, reason, text = real_service().handle_json(json.dumps(req))
    res = json.loads(text)
    assert (ok, reason, res['ok']) == (True, '', True), res['errors']       # 변환기 ① · 21번 모두 통과, recipe · placements 도 나온다
    assert res['recipe'] and res['placements'] and 'base_recipe' not in res
    assert real().check(req)['ok']


@pytest.mark.parametrize('role', ['LEG', 'LEG_WHEEL'])
def test_같은_역할_또는_같은_옵션의_두_묶음도_그대로_통과(role):
    req = base_design(BENCH)
    for p in req['parts']:
        if p['role'] == 'LEG':
            p['role'] = role
    assert real().check(req)['ok']


def test_옵션이_섞인_묶음은_실패_문구에_전체_역할도_적는다():
    req = with_wheel_leg(base_design(BENCH))
    revision(req, 'LEG')['height_mm'] = [30, 45]
    details = [e['detail'] for e in real().check(req)['errors']]
    assert any('그룹 LEG 1' in d for d in details) and any('그룹 LEG_WHEEL 1' in d for d in details)


def test_옵션이_섞여도_개수는_Template_역할_기준이고_부족하면_SHAPE():
    req = with_wheel_leg(base_design(BENCH))
    revision(req, 'LEG')['height_mm'] = [30, 120]
    req['base_recipe']['template']['composition']['LEG']['count'] = 3
    details = [e['detail'] for e in real().check(req)['errors'] if e['reason'] == 'TEMPLATE_SHAPE']
    assert details == ['21번 형태: 다리 개수 2개 — Template 은 3개']


# ---------------- 부품 단위 접촉 vs 묶음 상자 접촉 (실험) ----------------
def bbox_touches(run, a, b):
    """예전 판정(묶음 상자끼리)을 시험 안에서 되살린 비교용 — 오목한 묶음의 빈 곳도 닿은 것으로 본다."""
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        face = abs(a.hi[axis] - b.lo[axis]) <= tcm.TOL_MM or abs(b.hi[axis] - a.lo[axis]) <= tcm.TOL_MM
        if face and all(run.overlap(a, b, i) > tcm.TOL_MM for i in others):
            return True
    return False


def l_shaped_post_and_floating_cap():
    """계단 모양(오목한) 묶음 POST(같은 방향 눕힌 부품 둘)와, 그 묶음 상자 윗면 높이에 있지만 실제 부품 위는 아닌 CAP(세운 블록)."""
    return [put(1, 0, 0, 0, 'x', 'POST', 1), put(2, -50, 25, 0, 'x', 'POST', 1),      # P1 x[-37.5,37.5] y[-12.5,12.5] · P2 x[-87.5,-12.5] y[12.5,37.5] → 상자 안 x[-12.5,37.5] y[12.5,37.5] 가 빈 곳
            put(3, 25, 25, 15, 'zx', 'CAP', 1)]                                        # CAP x[17.5,32.5] y[12.5,37.5] — 빈 곳 위, 아랫면 z 15 = 상자 윗면


def test_부품_접촉은_L자_묶음의_빈_곳에_닿은_가짜_접촉을_막는다():
    composition = {'POST': {'count': 1, 'label': '받침', 'revision': {}}, 'CAP': {'count': 1, 'label': '덮개', 'revision': {'support': ['POST']}}}
    parts = l_shaped_post_and_floating_cap()
    run = tcm._Measure(TemplateChecker(ori_extents(BLOCK_MM)), rig(composition), parts)
    post, cap = run.by_role['POST'][0], run.by_role['CAP'][0]
    assert bbox_touches(run, cap, post) and abs(cap.lo[2] - post.hi[2]) <= tcm.TOL_MM        # 예전 판정: 닿았다(가짜)
    assert not run.touches(cap, post) and not run.rests_on(cap, post)                       # 부품 판정: 안 닿았다
    errors = measure(composition, parts)
    assert [(e['reason'], e['kind']) for e in errors] == [('TEMPLATE_SHAPE', 'template')] and '받침이 맞지 않다' in errors[0]['detail']


def test_가짜_접촉으로_상대_그룹을_잘못_고르지_않는다():
    """CAP 이 진짜로 얹힌 POST 는 그룹 2. 상자 접촉이면 그룹 1(L 자)도 후보라 번호 순으로 먼저 골라 잘못 잰다."""
    composition = {'POST': {'count': 2, 'label': '받침', 'revision': {}}, 'CAP': {'count': 1, 'label': '덮개', 'revision': {}}}
    parts = l_shaped_post_and_floating_cap()[:2] + [put(3, 70, 25, 15, 'x', 'POST', 2),           # 그룹 2 는 CAP 의 옆면에 진짜로 닿는다
                                                    put(4, 25, 25, 15, 'zx', 'CAP', 1)]            # 그룹 1 의 상자 윗면 높이에는 가짜로만 닿는다
    run = tcm._Measure(TemplateChecker(ori_extents(BLOCK_MM)), rig(composition), parts)
    cap = run.by_role['CAP'][0]
    assert run.partner(cap, 'POST').number == 2


def test_기본_4종의_부품_접촉은_묶음_상자_접촉과_같은_상대를_고른다():
    for model_id in DESIGNS:
        req = base_design(model_id)
        checker = TemplateChecker(ori_extents(BLOCK_MM))
        run = tcm._Measure(checker, checker.read_base(req), req['parts'])
        groups = [g for gs in run.by_role.values() for g in gs]
        assert all(run.touches(a, b) == bbox_touches(run, a, b) for a in groups for b in groups if a is not b), model_id


def support_case(support, positions):
    """독립 받침 그룹들과 보 한 그룹을 만든다. 위치 mm, ROS/로봇 영향 없음."""
    composition={'LEG':{'count':len(positions),'revision':{}},
                 'BEAM':{'count':1,'revision':{'support':support}}}
    parts=[put(i,x,0,0,'zy','LEG',i) for i,x in enumerate(positions,1)]
    parts.append(put(len(parts)+1,0,0,75,'x','BEAM',1))
    return composition,parts


@pytest.mark.parametrize('positions,ok',[([-25,25],True),([0,100],False),([-25,0,25],False)])
def test_per_group는_정확한_받침_그룹_개수(positions,ok):
    """정확히2개면 통과, T자1개/초과3개면 SHAPE template. 물리 안정성 검사와 독립 확인."""
    composition,parts=support_case({'to':'LEG','per_group':2},positions)
    errors=measure(composition,parts)
    assert (errors==[])==ok
    if not ok:
        assert all((e['reason'],e['kind'])==('TEMPLATE_SHAPE','template') for e in errors)
        assert '정확히 2개' in errors[0]['detail']


def test_목록_support는_역할_그룹_전부():
    """3개 중2개 받침만 닿으면 목록은 실패하고 per_group2는 통과한다."""
    composition,parts=support_case(['LEG'],[-25,25,100])
    errors=measure(composition,parts)
    assert len(errors)==1 and '전부 3개' in errors[0]['detail'] and '실제 2개' in errors[0]['detail']
    composition['BEAM']['revision']['support']={'to':'LEG','per_group':2}
    assert measure(composition,parts)==[]


def test_받침_부품_여럿과_상층_부품을_그룹_수로_세지_않는다():
    """한 받침 그룹 부품2개는1그룹, 보 상층은 받침에 직접 닿을 필요 없다."""
    composition,parts=support_case({'to':'LEG','per_group':2},[-25,25])
    parts.extend([put(4,-10,0,0,'zy','LEG',1),put(5,0,0,90,'x','BEAM',1)])
    composition['BEAM']['revision']['lay']='FLAT_STACK'
    assert measure(composition,parts)==[]


def test_받침_개수는_각_받쳐지는_그룹에서_따로_확인한다():
    """첫 보가2개 위라도 둘째 보가1개 위면 둘째만 실패한다."""
    composition={'LEG':{'count':4,'revision':{}},
                 'BEAM':{'count':2,'revision':{'support':{'to':'LEG','per_group':2}}}}
    parts=[put(1,-25,-50,0,'zy','LEG',1),put(2,25,-50,0,'zy','LEG',2),
           put(3,-25,50,0,'zy','LEG',3),put(4,100,50,0,'zy','LEG',4),
           put(5,0,-50,75,'x','BEAM',1),put(6,0,50,75,'x','BEAM',2)]
    errors=measure(composition,parts)
    assert len(errors)==1 and '그룹 2' in errors[0]['detail'] and '실제 1개' in errors[0]['detail']


@pytest.mark.parametrize('support',[{'to':'LEG','per_group':v} for v in (True,0,-1,1.5,'2',None)]+
                         [{'to':'TABLE','per_group':2},{'to':'BEAM','per_group':2},
                          {'to':'LEG','per_part':2}])
def test_불량_받침_객체는_REQUEST_INVALID(support):
    """불량 개수/역할/옛 제안 이름은 요청 오류이며 내부 ERROR로 바꾸지 않는다."""
    composition,_=support_case(support,[-25,25])
    with pytest.raises(RequestInvalid):rig(composition)


def test_004_TOP의_XY_대칭은_좌우와_앞뒤를_확인한다():
    """PL 추가 XY 규칙 자체가 두 방향 비대칭을 잡는지 검사한다."""
    req=base_design('004_DESK_PEDESTAL_V000')
    assert revision(req,'TOP')['mirror']=='XY'
    for axis in ('x','y'):
        shifted=copy.deepcopy(req)
        for part in shifted['parts']:
            if part['role']=='BASE':part[axis]+=5
        errors=real().check(shifted)['errors']
        assert any(e['reason']=='TEMPLATE_RANGE' and e['kind']=='template' and '대칭' in e['detail'] for e in errors)


# ---------------- 스캔 설계는 21번만 건너뜀 (E-106 · W191) ----------------
def real_converter():
    """진짜 21번 + 진짜 변환기 ①(task_node 와 같은 연결) — 대역 없이 made_by 경로를 본다."""
    from d2_task.parts_to_recipe import PartsToRecipe
    c = real()
    c.parts_to_recipe = PartsToRecipe(c.grasp_options, BLOCK_MM).convert
    return c


def free_shape():
    """벤치 V000 + 설계도에 없는 좌판 한 장을 맨 위에 더한 자유 모양(물리적으로는 받쳐짐)."""
    req = base_design(BENCH)
    req['parts'].append(dict(req['parts'][-1], sequence=len(req['parts']) + 1, z=req['parts'][-1]['z'] + 15, stage=req['parts'][-1]['stage'] + 1))
    return req


def test_같은_자유_모양_생성은_21번_불합격_스캔은_변환기까지_통과():
    gen = real_converter().check(free_shape())
    assert not gen['ok'] and set(reasons(gen)) == {'TEMPLATE_SHAPE'}
    scan = real_converter().check(dict(free_shape(), made_by='scan'))
    assert scan['ok'] and scan['errors'] == [] and 'recipe' in scan and 'placements' in scan


@pytest.mark.parametrize('value', [None, 'web', 'Scan', ' scan', 'scan ', 1, True, ['scan']])
def test_made_by_가_정확히_scan_이_아니면_21번은_그대로(value):
    req = free_shape()
    if value is not None:
        req['made_by'] = value
    res = real_converter().check(req)
    assert not res['ok'] and 'TEMPLATE_SHAPE' in reasons(res)


def test_스캔이어도_물리_불합격은_그대로_거절():
    req = dict(free_shape(), made_by='scan')
    req['parts'][-1]['z'] += 15                                         # 맨 위 좌판이 뜬다 — 받침 실패(1~20번)
    res = real_converter().check(req)
    assert not res['ok'] and 'CHECK_FAILED' in reasons(res) and 'TEMPLATE_SHAPE' not in reasons(res)
    assert 'recipe' not in res


@pytest.mark.parametrize('mutate', [lambda r: r.pop('base_recipe'),
                                    lambda r: r['base_recipe'].update(design_id='002_CHAIR_BACK_V000'),
                                    lambda r: r.update(design_id='bench_scan_1')])
def test_스캔이어도_요청_확인은_공통으로_REQUEST_INVALID(mutate):
    """base_recipe 있음 · Template 일치 · design_id 모양(<Template>_V3자리)은 스캔에도 그대로 본다(요청 계약)."""
    req = dict(free_shape(), made_by='scan')
    mutate(req)
    res = real_converter().check(req)
    assert not res['ok'] and reasons(res) == ['REQUEST_INVALID']

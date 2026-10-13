"""AI 설계 생성(web/backend/design_gen.py — DesignGenerator, W108)이 SDD 6.6 · IRD 8.3 · E-72 · E-84 대로 도는지 지킨다.

OpenAI 없이 돈다(가짜 클라이언트가 정해 둔 답을 차례로 준다). 검사는 진짜 검사 묶음(d2_task DesignChecker + 변환기 ① — task_node 와 같은 연결).
지키는 것: 부모 = 사람이 고른 설계(10/13 PL) · 바탕 설계를 요청 글에 넣음 · 도구는 같은 Template 의 다른 설계만(enum, 읽기는 골라서) ·
후보에 검사 전 새 ID(Template 다음 V) · 합격 후보만 저장(AI 판단 기록은 비움) · 모자라면 검사 detail 을 피드백으로 더(최대 2번) ·
범위 밖 OUT_OF_SCOPE · 검사 서비스 실패(로봇 PC)는 다시 만들지 않음 · 도구 단계 실패면 예시를 코드가 넣음 · 기록(NFR-17) · 중계(claude_bridge) 꺼짐 · 사용 한도 ·
스캔 [저장](W177 scan_fill → 검사 → 저장).
"""
import copy
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / 'web' / 'backend', ROOT / 'src' / 'd2_task', ROOT / 'src' / 'd2_bridge'):
    sys.path.insert(0, str(p))

from d2_task.parts_to_recipe import PartsToRecipe  # noqa: E402
from d2_task.design_checker import DesignChecker  # noqa: E402
import design_gen  # noqa: E402
from design_gen import MAX_REGEN, MODEL, DesignGenerator, GenError, scan_actions  # noqa: E402
from design_store import DesignStore, split_design_id  # noqa: E402

RECIPES = ROOT / 'src' / 'd2_robot' / 'd2_bringup' / 'recipes'
CFG = yaml.safe_load((ROOT / 'src' / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml').read_text(encoding='utf-8'))
RULES = {'block_size_m': CFG['block_size_m'], 'margin_mm': CFG['check']['margin_mm'], 'max_blocks': CFG['check']['max_blocks'],
         'finger_thickness_m': CFG['finger']['thickness_m'], 'finger_width_m': CFG['finger']['width_m'],
         'assembly_area_half_m': CFG['assembly_area_half_m']}      # app.load_rules 와 같은 키(E-78)
TEMPLATE, BENCH = '001_CHAIR_BENCH', '001_CHAIR_BENCH_V000'


class FakeOpenAI:
    """chat.completions.create 흉내 — 정해 둔 답(메시지 · 예외)을 차례로 돌려주고, 받은 인자를 그때 모습 그대로 모은다."""

    def __init__(self, replies):
        self.replies, self.calls = list(replies), []
        self.chat = SimpleNamespace(completions=SimpleNamespace(create=self.create))

    def create(self, **kw):
        self.calls.append(json.loads(json.dumps(kw, ensure_ascii=False)))
        reply = self.replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return SimpleNamespace(choices=[SimpleNamespace(message=reply)], usage=SimpleNamespace(total_tokens=100))


def tool_call(design_id, cid='c1'):
    """AI 가 도구 get_design 을 부른 답."""
    fn = SimpleNamespace(name='get_design', arguments=json.dumps({'design_id': design_id}))
    return SimpleNamespace(content=None, refusal=None, tool_calls=[SimpleNamespace(id=cid, function=fn)])


def answer(cands, out=False, reason=''):
    """AI 의 후보 답(구조화 출력 JSON)."""
    body = {'out_of_scope': out, 'reason': reason,
            'candidates': [{'idea': f'안 {i}', 'plan': 'z0: …', 'parts': b} for i, b in enumerate(cands)]}
    return SimpleNamespace(content=json.dumps(body, ensure_ascii=False), refusal=None, tool_calls=None)


def real_check():
    """로봇 PC task_node 와 같은 검사 묶음(DesignChecker + 변환기 ①) — MqttClient.request 가 돌려주는 응답 꼴로."""
    checker = DesignChecker(CFG)
    checker.parts_to_recipe = PartsToRecipe(checker.grasp_options, [v * 1000 for v in CFG['block_size_m']]).convert

    def check(blocks):
        success, reason, rj = checker.handle_json(json.dumps(blocks, ensure_ascii=False))
        return {'success': success, 'reason': reason, **json.loads(rj)}
    return check


@pytest.fixture
def store(tmp_path):
    """임시 폴더 저장소 + 기본 설계 4개."""
    s = DesignStore(tmp_path, RECIPES, [75, 25, 15])
    s.register_bases()
    return s


@pytest.fixture
def good(store):
    """검사에 합격하는 블록 목록(벤치 V000 그대로)."""
    return store.get_design(BENCH)['parts']['parts']


@pytest.fixture
def bad(good):
    """검사에 떨어지는 블록 목록 — 좌판 하나(9번)를 공중에 띄움."""
    b = copy.deepcopy(good)
    b[8]['z'] = 75.0
    return b


def derive(store, parent=BENCH):
    """같은 Template 에 파생 설계 하나를 저장한다(부모 블록 그대로 · 다음 번호, made_by web) — 도구로 읽을 '다른 설계'가 있게."""
    template = split_design_id(parent)[0]
    design_id, _ = store.new_design_id(template, parent)
    parts = {**store.get_design(parent)['parts'], 'design_id': design_id}
    res = real_check()({**parts, 'base_recipe': store.base_recipe(template)})
    assert res['ok'], res['errors']
    return store.save_design(parts, res['recipe'], res['placements'], parent, 'web', check={'ok': True})['design_id']


def make(store, replies, check=None):
    """가짜 OpenAI · 진짜 검사(또는 주어진 check)로 생성기를 만든다."""
    client = FakeOpenAI(replies)
    return DesignGenerator(store, check or real_check(), RULES, client=client), client


def test_generate_reads_reference_checks_and_saves_chosen(store, good, bad):
    """고른 설계(부모)를 바탕으로 주고 → 같은 Template 의 다른 설계를 도구로 참고 → 후보 3개에 새 ID → 진짜 검사 → 떨어진 1개만큼 더 만들어
    통과 3개 → 고른 1개만 저장. 진행 · 기록(NFR-17)도. 검사 요청에는 V000 레시피 통째(base_recipe — 10/13 PL)를 싣고, 후보 · 저장 설계에는
    안 넣는다. 저장하면 AI 기록 번호(#14), AI 판단 기록 3칸은 비움(글 · 음성 — 10/13 PL)."""
    v1 = derive(store)
    sent, check = [], real_check()
    gen, client = make(store, [tool_call(v1), answer([good, bad, good]), answer([good])],
                       check=lambda req: sent.append(req) or check(req))
    events = []
    r = gen.generate('벤치를 만들어 줘', BENCH, on_progress=lambda stage, info: events.append(stage))
    assert r['design_id'] == '001_CHAIR_BENCH_V002' and r['parent_id'] == BENCH and r['base_id'] == BENCH and r['reference'] == [v1]
    assert [c['check']['ok'] for c in r['candidates']] == [True, True, True] and r['attempts'] == 2   # 통과한 것만 화면에
    assert r['candidates'][0]['parts']['design_id'] == '001_CHAIR_BENCH_V002'
    assert len(sent) == 4 and all(req['base_recipe'] == store.get_design(BENCH)['recipe'] for req in sent)
    assert all('made_by' not in req for req in sent)      # AI 생성 검사 요청에는 made_by 를 싣지 않는다(10/14 PL A1)
    assert all('base_recipe' not in c['parts'] for c in r['candidates'])
    assert events == ['reading', 'candidates', 'checked', 'checked', 'checked', 'retry', 'candidates', 'checked']
    assert '후보 2개는 통과해 남겨 둔다' in client.calls[2]['messages'][-1]['content'] and '새 후보 1개만' in client.calls[2]['messages'][-1]['content']
    first, second, _ = client.calls
    assert first['model'] == MODEL and first['temperature'] == 0 and first['tool_choice'] == 'auto'   # 바탕은 줬으니 참고는 골라서
    assert first['parallel_tool_calls'] is False and first['response_format']['json_schema']['strict'] is True
    assert first['tools'][0]['function']['parameters']['properties']['design_id']['enum'] == [v1]   # 같은 Template 의 다른 설계만
    request = first['messages'][1]['content']
    assert '[바탕 설계 — 사용자가 고름]\n- 001_CHAIR_BENCH_V000 |' in request and json.dumps(good, ensure_ascii=False, separators=(',', ':')) in request
    assert f'- {v1} |' in request.split('[DB 목록')[1]
    assert second['messages'][-1]['role'] == 'tool' and '"parts"' in second['messages'][-1]['content']
    rec = gen.save(r, 0)
    assert rec['design_id'] == '001_CHAIR_BENCH_V002' and rec['parent_id'] == BENCH and rec['made_by'] == 'web'
    assert rec['prompt'] == '벤치를 만들어 줘' and rec['recipe']['design_id'] == '001_CHAIR_BENCH_V002'
    assert (rec.get('ai_template'), rec.get('ai_reason'), rec.get('template_changed')) == (None, None, None)
    assert 'base_recipe' not in rec['parts'] and 'revision' not in json.dumps(rec['recipe'])   # AI 레시피 = composition 만(레시피 초안 6절)
    with pytest.raises(ValueError):
        gen.save(r, 3)                                       # 없는 번호(떨어진 후보는 아예 안 돌려줌)
    logs = list(store.gen_logs_dir.glob('*_generate.json'))
    assert len(logs) == 1 and r['gen_log'] == logs[0].name
    log = json.loads(logs[0].read_text(encoding='utf-8'))
    assert log['messages'][0]['role'] == 'system' and len(log['calls']) == 3 and log['result']['design_id'] == '001_CHAIR_BENCH_V002'
    assert log['design_id'] == '001_CHAIR_BENCH_V002' and log['base_id'] == BENCH     # 고른 후보를 저장할 때 채움(설계_DB #14)
    assert 'sk-' not in logs[0].read_text(encoding='utf-8')


def test_parent_is_chosen_design_not_what_gpt_read(store, good):
    """부모 = 사람이 고른 설계(10/13 PL): 파생 V001 을 고르고 GPT 가 V000 을 읽어도 부모는 V001, 새 ID 는 Template 다음 번호.
    같은 Template 에 다른 설계가 없으면(기본 설계만) 도구 없이 바로 후보를 받는다."""
    v1 = derive(store)
    gen, client = make(store, [tool_call(BENCH), answer([good] * 3)])
    r = gen.generate('좌판을 넓혀 줘', v1)
    assert (r['parent_id'], r['reference'], r['design_id'], r['template']) == (v1, [BENCH], '001_CHAIR_BENCH_V002', TEMPLATE)
    assert client.calls[0]['tools'][0]['function']['parameters']['properties']['design_id']['enum'] == [BENCH]
    assert gen.save(r, 1)['parent_id'] == v1
    back = store.get_design('002_CHAIR_BACK_V000')['parts']['parts']
    gen, client = make(store, [answer([back] * 3)])
    r = gen.generate('다리를 늘려 줘', '002_CHAIR_BACK_V000')
    assert r['parent_id'] == '002_CHAIR_BACK_V000' and r['design_id'] == '002_CHAIR_BACK_V001' and 'tools' not in client.calls[0]
    assert '- 없음(이 Template 의 설계는 바탕 설계뿐)' in client.calls[0]['messages'][1]['content']


def test_regenerates_with_check_feedback_keeps_passed_and_fills_shortfall(store, good, bad):
    """다 떨어지면 검사 detail 을 피드백으로 3개 다시 → 1개 통과면 남기고 모자란 2개만 더 → 처음 + 2번 뒤 안 차면 통과한 것만(같은 ID)."""
    gen, client = make(store, [answer([bad] * 3), answer([good, bad, bad]), answer([bad, bad])])
    r = gen.generate('벤치', BENCH)
    assert r['attempts'] == 3 and [c['check']['ok'] for c in r['candidates']] == [True] and r['design_id'] == '001_CHAIR_BENCH_V001'
    retry = client.calls[1]
    assert 'tools' not in retry and retry['messages'][-1]['content'].startswith('[검사 결과]')
    fb = retry['messages'][-1]['content']
    assert '후보 3개를 다시' in fb and '9번' in fb and '공중' in fb and '고칠 것' in fb and '후보 1(안 0)' in fb   # 후보별로 묶고 고치는 법
    assert '아랫면 z 75.0' in fb and '가장 높아야 z 60.0' in fb                        # 어느 층이 비었는지(다리 꼭대기 60)
    assert '후보 1개는 통과해 남겨 둔다' in client.calls[2]['messages'][-1]['content'] and '새 후보 2개만' in client.calls[2]['messages'][-1]['content']


def test_refill_call_failure_keeps_passed(store, good, bad):
    """더 만들기 호출만 실패하면(네트워크 등) 이미 통과한 후보는 버리지 않고 돌려준다."""
    gen, _ = make(store, [answer([good, bad, bad]), RuntimeError('network')])
    r = gen.generate('벤치', BENCH)
    assert [c['check']['ok'] for c in r['candidates']] == [True]


@pytest.mark.parametrize('reasons, code', [(['TEMPLATE_RANGE', 'TEMPLATE_SHAPE'], 'TEMPLATE_SHAPE'),
                                           (['TEMPLATE_RANGE', 'CHECK_FAILED'], 'TEMPLATE_RANGE'),
                                           (['CHECK_FAILED'], 'GEN_FAILED')])
def test_none_passed_is_told_by_kind(store, good, reasons, code):
    """하나도 통과 못 하면 실패 종류로 알린다(레시피 초안 7-1): 구성 형태 먼저 → 범위(21번 detail 의 범위 글을 붙임) → 그 밖."""
    errs = [{'part': None, 'reason': r, 'kind': 'template' if r.startswith('TEMPLATE') else 'position',
             'detail': '21번 범위: 다리 높이 150 mm — 범위 30 ~ 120' if r == 'TEMPLATE_RANGE' else r} for r in reasons]
    gen, client = make(store, [answer([good] * 3)] * (MAX_REGEN + 1),
                       check=lambda req: {'success': True, 'reason': '', 'ok': False, 'errors': errs})
    with pytest.raises(GenError) as e:
        gen.generate('다리 높은 벤치', BENCH)
    assert e.value.code == code and len(client.calls) == 1 + MAX_REGEN
    assert ('다리 높이 150 mm — 범위 30 ~ 120' in e.value.message) == (code == 'TEMPLATE_RANGE') and '21번' not in e.value.message


def test_request_invalid_is_not_regenerated(store, good):
    """REQUEST_INVALID(웹 쪽 잘못 — base_recipe 등)는 다시 만들어도 같아 바로 끝낸다(D10)."""
    bad_req = {'success': True, 'reason': '', 'ok': False, 'errors': [{'part': None, 'reason': 'REQUEST_INVALID', 'detail': 'base_recipe 가 없다'}]}
    gen, client = make(store, [answer([good] * 3)], check=lambda req: bad_req)
    with pytest.raises(GenError) as e:
        gen.generate('벤치', BENCH)
    assert e.value.code == 'REQUEST_INVALID' and len(client.calls) == 1


def test_gives_up_after_two_regenerations(store, bad):
    """더 만들기는 최대 2번 — 그래도 하나도 통과 못 하면 실패 종류로(마지막 errors 를 detail 로). 좌판을 띄운 후보는 진짜 검사에서
    공중(position)과 구성 형태(21번 — 좌판이 다리 위가 아님)가 같이 떨어져, 구성 형태 먼저(PL 4-2-추가) = TEMPLATE_SHAPE."""
    gen, client = make(store, [answer([bad] * 3)] * (MAX_REGEN + 1))
    with pytest.raises(GenError) as e:
        gen.generate('벤치', BENCH)
    kinds = {x.get('kind') for x in e.value.detail}
    assert e.value.code == 'TEMPLATE_SHAPE' and {'position', 'template'} <= kinds
    assert len(client.calls) == 1 + MAX_REGEN


def test_out_of_scope(store):
    """AI 가 범위 밖이라고 하면 OUT_OF_SCOPE(이유 그대로) — 검사 · 저장 없음."""
    gen, _ = make(store, [answer([], out=True, reason='벤치로는 2층 침대를 못 만들어요')])
    with pytest.raises(GenError) as e:
        gen.generate('2층 침대', BENCH)
    assert e.value.code == 'OUT_OF_SCOPE' and '침대' in e.value.message
    assert [r['design_id'] for r in store.list_designs(template=TEMPLATE)] == [BENCH]


def test_tool_answers_only_template_designs_and_stops_after_two(store, good):
    """목록 밖 설계(다른 Template · 바탕 설계 자신)를 부르면 이유만 돌려주고(읽지 않음), 2번 읽으면 그다음은 도구를 못 부르게(none)."""
    v1 = derive(store)
    gen, client = make(store, [tool_call('002_CHAIR_BACK_V000'), tool_call(v1, 'c2'), tool_call(v1, 'c3'), answer([good] * 3)])
    r = gen.generate('벤치', BENCH)                            # 3라운드 내내 도구만 부름 → 예시를 코드가 넣는 길로(E-72)
    assert r['reference'] == [] and r['parent_id'] == BENCH and 'tools' not in client.calls[-1]
    gen, client = make(store, [tool_call(v1), tool_call(v1, 'c2'), answer([good] * 3)])
    r = gen.generate('벤치', BENCH)
    assert [c['tool_choice'] for c in client.calls] == ['auto', 'auto', 'none'] and r['reference'] == [v1, v1]
    gen, client = make(store, [tool_call(BENCH), tool_call(v1, 'c2'), answer([good] * 3)])
    r = gen.generate('벤치', BENCH)
    assert 'error' in client.calls[1]['messages'][-1]['content'] and r['reference'] == [v1] and r['parent_id'] == BENCH


def test_falls_back_to_examples_when_tool_phase_fails(store, good):
    """도구 단계 호출이 실패하면 같은 Template 예시를 코드가 넣어 한 번 더(E-72 자동 전환) — 부모는 그대로 고른 설계, 바탕 설계는 예시에서 뺌."""
    v1 = derive(store)
    gen, client = make(store, [RuntimeError('network'), answer([good] * 3)])
    r = gen.generate('벤치', v1)
    assert r['parent_id'] == v1 and r['reference'] == []
    text = client.calls[-1]['messages'][1]['content']
    examples = text.split('[참고 설계 — 도구 대신 코드가 넣음]')[1]
    assert 'tools' not in client.calls[-1] and f'({BENCH},' in examples and f'({v1},' not in examples


def test_check_service_failure_is_not_regenerated(store, good):
    """검사 서비스가 답을 못 하면(로봇 PC · 다리 — TIMEOUT · ERROR) AI 탓이 아니라 다시 만들지 않고 그 이유로 끝낸다."""
    gen, client = make(store, [answer([good] * 3)],
                       check=lambda b: {'success': False, 'reason': 'TIMEOUT', 'message': '5초 안에 답이 없다'})
    with pytest.raises(GenError) as e:
        gen.generate('벤치', BENCH)
    assert e.value.code == 'TIMEOUT' and len(client.calls) == 1


def test_unknown_base_and_bad_answer(store):
    """고른 설계가 없거나(모르는 Template · 저장 안 된 번호 · ID 꼴 아님) 부르기 전에 ERROR, AI 답이 JSON 이 아니면 GEN_FAILED(고쳐 읽지 않음)."""
    gen, client = make(store, [])
    for base in ('009_SOFA_LONG_V000', '001_CHAIR_BENCH_V007', '001_CHAIR_BENCH', None):
        with pytest.raises(GenError) as e:
            gen.generate('소파', base)
        assert e.value.code == 'ERROR'
    assert client.calls == []
    gen, _ = make(store, [SimpleNamespace(content='블록을 이렇게…', refusal=None, tool_calls=None)])
    with pytest.raises(GenError) as e:
        gen.generate('벤치', BENCH)
    assert e.value.code == 'GEN_FAILED'


def test_request_names_template_from_v000_recipe(store, good):
    """요청 글의 [Template] 줄 = V000 레시피 template 칸의 이름 — 설명(코드 표 없음, 10/13 PL). 모델은 MODEL(Claude 별칭 — 10/13 중계 claude_bridge)."""
    gen, client = make(store, [answer([good] * 3)])
    gen.generate('벤치', BENCH)
    bench = store.get_design(BENCH)['recipe']['template']
    assert f"[Template] {TEMPLATE} — {bench['name']} — {bench['description']} (" in client.calls[0]['messages'][1]['content']
    assert client.calls[0]['model'] == MODEL


def test_prompt_takes_rules_from_robot_yaml_and_roles(store):
    """프롬프트 숫자는 robot.yaml(E-78 키)에서, 역할 이름은 roles.json 에서 — 빈 자리($) 없이."""
    gen, _ = make(store, [])
    p = gen.design_prompt
    assert '75 × 25 × 15' in p and f"{CFG['check']['margin_mm']} mm" in p and 'COLUMN' in p and '$' not in p
    role_enum = gen._candidates_format()['json_schema']['schema']['properties']['candidates']['items']['properties']['parts'][
        'items']['properties']['role']['enum']
    assert 'LEG' in role_enum and 'LEG_WHEEL' in role_enum


def test_bridge_down_is_gen_failed(store, monkeypatch):
    """중계(claude_bridge)가 꺼져 있으면(또는 openai 라이브러리가 없으면) 웹은 켜져 있고 생성만 GEN_FAILED + 중계를 켜라는 안내."""
    pytest.importorskip('openai')
    monkeypatch.setattr(design_gen, 'LLM_URL', 'http://127.0.0.1:9/v1')   # 아무도 안 듣는 자리 — 진짜 중계 · Claude 를 부르지 않게
    gen = DesignGenerator(store, real_check(), RULES)
    with pytest.raises(GenError) as e:
        gen.generate('벤치', BENCH)
    assert e.value.code == 'GEN_FAILED' and 'claude_bridge' in e.value.message


def test_bridge_down_and_usage_limit_messages(store):
    """중계(claude_bridge)가 꺼져 있거나 Claude 사용 한도면 '잠시 뒤 다시' 대신 그 이유를 알린다(기다려도 안 풀려서)."""
    class APIConnectionError(Exception):   # openai 의 같은 이름 예외 흉내 — 이름으로 가른다
        pass
    gen, _ = make(store, [APIConnectionError('Connection error.'),
                          RuntimeError("Error code: 502 - {'error': {'message': \"claude 실패: You've hit your usage limit\"}}")])
    with pytest.raises(GenError) as e:
        gen.generate('벤치', BENCH)
    assert e.value.code == 'GEN_FAILED' and 'claude_bridge' in e.value.message
    with pytest.raises(GenError) as e:
        gen.generate('벤치', BENCH)
    assert e.value.code == 'GEN_FAILED' and '사용 한도' in e.value.message



# ---------- 스캔 [저장](IRD 8.4 ⑦) — scan_fill 은 W177(민범진) 자리라 시험에서는 정해 둔 답으로 ----------
RUN = 'R20261013_100000_a1b2'


def scan_of(parts, nearest=BENCH):
    """설계를 스캔한 것처럼 — blocks/2(scan_num · 위치 · 방향)만, AI 칸 없음. 순서는 일부러 뒤집음(scan_num 은 쌓는 순서가 아님)."""
    blocks = [{'scan_num': i + 1, **{k: p[k] for k in ('x', 'y', 'z', 'ori')}} for i, p in enumerate(reversed(parts))]
    return {'schema': 'scan_result/2.0', 'run_id': RUN, 'inferred_count': 0, 'nearest_base': nearest,
            'blocks': {'schema': 'blocks/2', 'blocks': blocks}}


def fills(gen, *answers):
    """gen.scan_fill 을 정해 둔 답(dict 또는 예외)으로 바꾼다. 반환: 받은 feedback 목록(부를 때마다 하나)."""
    got = []

    def fill(scan_result, feedback=None):
        got.append(feedback)
        a = answers[len(got) - 1]
        if isinstance(a, Exception):
            raise a
        return a
    gen.scan_fill = fill
    return got


def filled(parts, parent=BENCH, ai=TEMPLATE, reason='다리 둘 위 좌판'):
    """scan_fill 답 모양(W177 약속)."""
    return {'parts': copy.deepcopy(parts), 'parent_id': parent, 'ai_template': ai, 'ai_reason': reason}


def test_save_scan_with_real_scan_fill(store, good):
    """진짜 scan_fill(가짜 GPT)로 스캔 [저장] 끝까지 — 채움 → 검사(base_recipe) → 저장(made_by scan · AI 판단 기록). GPT 가 못 채우면
    GEN_FAILED, 화면 버튼은 [다시 스캔] · [취소]."""
    gen, _ = make(store, [tool_call(BENCH), scan_answer(good, BENCH)])
    rec = gen.save_scan(scan_of(good))
    assert rec['made_by'] == 'scan' and rec['parent_id'] == BENCH and rec['design_id'] == '001_CHAIR_BENCH_V001'
    assert rec['ai_template'] == TEMPLATE and rec['ai_reason'] == '블록 수 · 다리 벽이 같다'
    gen, _ = make(store, [RuntimeError('network'), RuntimeError('network')])
    with pytest.raises(GenError) as e:
        gen.save_scan(scan_of(good))
    assert e.value.code == 'GEN_FAILED' and scan_actions(e.value.detail) == ['rescan', 'cancel']


def test_save_scan_checks_with_base_recipe_and_saves_as_scan(store, good):
    """채움 → 부모 Template 의 다음 V → 검사(base_recipe = V000 레시피 통째) → 저장(made_by scan · AI 판단 기록, base_recipe 없음)."""
    sent, check = [], real_check()
    gen, _ = make(store, [], check=lambda req: sent.append(req) or check(req))
    fills(gen, filled(good))
    events = []
    rec = gen.save_scan(scan_of(good), on_progress=lambda stage, info: events.append(stage))
    assert rec['design_id'] == '001_CHAIR_BENCH_V001' and rec['made_by'] == 'scan' and rec['parent_id'] == BENCH
    assert (rec['ai_template'], rec['ai_reason'], rec['template_changed']) == (TEMPLATE, '다리 둘 위 좌판', False)
    assert sent[0]['base_recipe'] == store.get_design(BENCH)['recipe'] and 'base_recipe' not in rec['parts']
    assert sent[0]['made_by'] == 'scan' and 'made_by' not in rec['parts']   # 스캔 [저장]만 21번 건너뜀 표시(E-106 · 10/14 PL A1)
    assert events == ['filled', 'checked']


def test_save_scan_refills_when_ai_moved_parts_or_filled_wrong(store, good):
    """AI 가 위치를 바꾸면(E-79) 검사 전에 떨어뜨리고 피드백으로 다시 채운다(ai_fields). 다시 채우기는 최대 2번 → 그래도면 CHECK_FAILED."""
    moved = copy.deepcopy(good)
    moved[0]['x'] += 25
    gen, _ = make(store, [])
    got = fills(gen, filled(moved), filled(good))
    rec = gen.save_scan(scan_of(good))
    assert rec['made_by'] == 'scan' and got[0] is None and got[1]['errors'][0]['kind'] == 'ai_fields'
    assert got[1]['parts'] == moved                          # 앞 답의 parts 도 같이(오류 'N번'이 어느 부품인지 — 10/13 PL)
    gen, _ = make(store, [])
    got = fills(gen, filled(moved), filled(moved), filled(moved))
    with pytest.raises(GenError) as e:
        gen.save_scan(scan_of(good))
    assert e.value.code == 'CHECK_FAILED' and len(got) == 3 and scan_actions(e.value.detail) == ['rescan', 'cancel']


def test_save_scan_stops_at_position_and_scope(store, good, bad):
    """놓인 자리 실패(position — 공중 등)는 다시 채우지 않고 바로 [다시 스캔] · [취소], 블록 수 초과(scope)는 [취소]만."""
    gen, _ = make(store, [])
    got = fills(gen, filled(bad), filled(bad))
    with pytest.raises(GenError) as e:
        gen.save_scan(scan_of(bad))
    assert len(got) == 1 and any(x.get('kind') == 'position' for x in e.value.detail)
    assert scan_actions(e.value.detail) == ['rescan', 'cancel']
    many = [dict(p, z=p['z'] + 90 * k) for k in range(5) for p in good]       # 55개 > 54(robot.yaml check.max_blocks)
    gen, _ = make(store, [])
    got = fills(gen, filled(many), filled(many))
    with pytest.raises(GenError) as e:
        gen.save_scan(scan_of(many))
    assert len(got) == 1 and e.value.code == 'OUT_OF_SCOPE' and scan_actions(e.value.detail) == ['cancel']


def test_four_bases_pass_real_check_with_their_base_recipe(store):
    """W163 완료 기준(기본 4개 통과): 기본 설계 4개를 새 ID 로 검사 요청하면 각자 V000 레시피(base_recipe)로 진짜 검사 묶음(21번 포함)을 통과한다.
    다른 Template 의 base_recipe 를 실으면 REQUEST_INVALID(웹 쪽 잘못 — 21번이 model_id 로 대조)."""
    gen, _ = make(store, [])
    for t in ('001_CHAIR_BENCH', '002_CHAIR_BACK', '003_DESK_STAND', '004_DESK_PEDESTAL'):
        design_id, _ = store.new_design_id(t, f'{t}_V000')
        parts = {**store.get_design(f'{t}_V000')['parts'], 'design_id': design_id}
        chk = gen._check(parts)
        assert chk['ok'] and chk['recipe']['design_id'] == design_id and 'revision' not in json.dumps(chk['recipe']), (t, chk['errors'])
    wrong = real_check()({**parts, 'base_recipe': store.base_recipe('001_CHAIR_BENCH')})
    assert wrong['ok'] is False and [e['reason'] for e in wrong['errors']] == ['REQUEST_INVALID']


# ---------- 스캔 [저장] 채우기(W177 · IRD 8.4 ⑦ · E-79) ----------
def scan_answer(parts, parent, reason='블록 수 · 다리 벽이 같다', drop=0):
    """GPT 의 스캔 채우기 답(W177 진짜 scan_fill 시험용) — scan_num 마다 채울 칸만(scan_of 의 번호 규칙: 거꾸로 = n − order + 1).
    drop 개만큼 뒤에서 뺀다(모양 틀린 답)."""
    n = len(parts)
    rows = [{'scan_num': n - p['sequence'] + 1, **{k: p[k] for k in ('sequence', 'role', 'group', 'stage', 'grasp')}} for p in parts]
    body = {'parent_id': parent, 'reason': reason, 'plan': 'z0: …', 'parts': rows[:len(rows) - drop]}
    return SimpleNamespace(content=json.dumps(body, ensure_ascii=False), refusal=None, tool_calls=None)


def test_scan_fill_keeps_scan_shape_and_passes_check(store, good):
    """도구로 비슷한 설계를 읽고(DB 전체) → scan_num 마다 칸만 받아 위치 · 방향은 스캔 그대로 → 진짜 검사 합격 · 기록."""
    gen, client = make(store, [tool_call(BENCH), scan_answer(good, BENCH)])
    r = gen.scan_fill(scan_of(good))
    assert r['parent_id'] == BENCH and r['ai_template'] == TEMPLATE and r['ai_reason'] == '블록 수 · 다리 벽이 같다'
    assert [{k: p[k] for k in ('sequence', 'x', 'y', 'z', 'ori', 'role', 'group', 'stage', 'grasp')} for p in r['parts']] == \
        [{k: p[k] for k in ('sequence', 'x', 'y', 'z', 'ori', 'role', 'group', 'stage', 'grasp')} for p in good]
    assert all('inferred' not in p for p in r['parts'])                     # 스캔에 없는 칸은 만들지 않음
    design_id, _ = store.new_design_id(TEMPLATE, r['parent_id'])
    chk = gen._check({'schema': 'parts/4.0', 'design_id': design_id, 'parts': r['parts']})
    assert chk['ok'], chk['errors']
    first = client.calls[0]
    assert first['tool_choice'] == 'required' and first['tools'][0]['function']['parameters']['properties']['design_id']['enum'] == \
        ['001_CHAIR_BENCH_V000', '002_CHAIR_BACK_V000', '003_DESK_STAND_V000', '004_DESK_PEDESTAL_V000']   # DB 전체(E-79)
    item = first['response_format']['json_schema']['schema']['properties']['parts']['items']['properties']
    assert 'x' not in item and 'ori' not in item and 'scan_num' in item           # AI 는 위치 · 방향을 쓰지 못한다
    log = json.loads((store.gen_logs_dir / r['gen_log']).read_text(encoding='utf-8'))
    assert log['kind'] == 'scan_fill' and log['result']['parent_id'] == BENCH and log['run_id'] == RUN


def test_scan_fill_answer_must_cover_every_scan_num(store, good):
    """AI 답에서 블록이 빠지면(또는 겹치면) 코드가 채우지 않고 GEN_FAILED — 기록에 오류."""
    gen, _ = make(store, [tool_call(BENCH), scan_answer(good, BENCH, drop=1)])
    with pytest.raises(GenError) as e:
        gen.scan_fill(scan_of(good))
    assert e.value.code == 'GEN_FAILED'
    log = json.loads(next(store.gen_logs_dir.glob('*_scan_fill.json')).read_text(encoding='utf-8'))
    assert log['error']['code'] == 'GEN_FAILED'


def test_scan_fill_parent_other_family_or_none_is_replaced(store, good):
    """부모는 같은 family 만(E-79): 다른 family 를 고르면 nearest_base 로, NONE 이고 nearest_base 도 없으면
    (family 를 알면 같은 family) 기본 설계 중 부품 수가 가장 가까운 것으로. ai_template 은 AI 가 고른 그대로(template_changed 는 저장소가 계산)."""
    gen, _ = make(store, [tool_call(BENCH), scan_answer(good, '003_DESK_STAND_V000')])
    r = gen.scan_fill(scan_of(good))
    assert r['parent_id'] == BENCH and r['ai_template'] == '003_DESK_STAND'
    gen, _ = make(store, [tool_call(BENCH), scan_answer(good, 'NONE', reason='맞는 설계가 없다')])
    r = gen.scan_fill(scan_of(good, nearest=None))                 # family 모름 · 부품 11개 → 기본 설계 중 벤치(11)가 가장 가깝다
    assert r['parent_id'] == BENCH and r['ai_template'] == 'NONE' and r['ai_reason'] == '맞는 설계가 없다'


def test_scan_fill_feedback_and_tool_failure_fallback(store, good):
    """다시 채울 때 {errors, 앞 parts} 를 받아 앞 답(scan_num 붙여) · 오류 · 고치는 법을 넣고, 도구 단계(부모 고르기)가 실패하면
    대신할 부모 부품 목록을 코드가 넣어 칸만 채우게 한 번 더 — 부모 = nearest_base, ai_template NONE · ai_reason '실패: <코드>'(저장은 계속)."""
    first, _ = make(store, [tool_call(BENCH), scan_answer(good, BENCH)])
    prev = first.scan_fill(scan_of(good))['parts']
    feedback = {'errors': [{'part': 9, 'reason': 'unsupported', 'detail': '9번 블록이 공중', 'kind': 'ai_fields'}], 'parts': prev}
    gen, client = make(store, [RuntimeError('network'), scan_answer(good, '002_CHAIR_BACK_V000')])
    r = gen.scan_fill(scan_of(good), feedback=feedback)
    assert r['parent_id'] == BENCH and r['ai_template'] == 'NONE' and r['ai_reason'] == '실패: GEN_FAILED'
    last = client.calls[-1]
    text = last['messages'][1]['content']
    assert 'tools' not in last and '도구 대신 코드가 넣음' in text and BENCH in text
    assert '[앞 검사 결과]' in text and '9번 블록이 공중' in text and '고칠 것' in text
    n = len(good)
    shown = json.loads(text.split('[앞 답]\n')[1].split('\n')[0])
    assert {'scan_num': n - 9 + 1, 'sequence': 9} == {k: shown[8][k] for k in ('scan_num', 'sequence')}   # 앞 sequence 9 = 어느 스캔 블록인지


def test_scan_fill_rejects_bad_input_before_calling_ai(store):
    """스캔 모양이 틀리면(blocks 없음 · scan_num 겹침) AI 를 부르기 전에 ERROR."""
    gen, client = make(store, [])
    for bad in ({}, {'blocks': {'blocks': []}},
                {'blocks': {'blocks': [{'scan_num': 1, 'x': 0, 'y': 0, 'z': 0, 'ori': 'x'}] * 2}}):
        with pytest.raises(GenError) as e:
            gen.scan_fill(bad)
        assert e.value.code == 'ERROR'
    assert client.calls == []


@pytest.mark.parametrize('weird', ['그냥 글자', SimpleNamespace(choices=[]), SimpleNamespace(choices=[SimpleNamespace(message='글자')])])
def test_weird_response_shape_is_gen_failed(store, weird):
    """응답 모양이 다르면(글자뿐 · choices 비었음 · 메시지가 글자) AttributeError 가 아니라 GEN_FAILED(10/13 민범진 보고) — 기록엔 원문 일부."""
    gen, client = make(store, [])
    client.chat.completions.create = lambda **kw: weird
    with pytest.raises(GenError) as e:
        gen.generate('벤치', BENCH)
    assert e.value.code == 'GEN_FAILED' and '모양' in e.value.message
    log = json.loads(next(store.gen_logs_dir.glob('*_generate.json')).read_text(encoding='utf-8'))
    assert 'raw' in log['calls'][0]


class RelayError(Exception):
    """openai 의 상태 코드 예외 흉내 — 중계가 보낸 HTTP 코드(status_code)와 {'error': {...}} 안쪽(body)."""

    def __init__(self, status, message):
        super().__init__(f'Error code: {status}')
        self.status_code, self.body = status, {'message': message, 'type': 'claude_bridge'}


class APITimeoutError(Exception):   # openai 의 같은 이름 예외 흉내(이쪽 CALL_TIMEOUT_S 초과) — 이름으로 가른다
    pass


@pytest.mark.parametrize('err, words', [
    (APITimeoutError('Request timed out.'), '시간 안에'),
    (RelayError(504, 'claude 가 220초 안에 답하지 않았다'), '시간 안에'),
    (RelayError(429, 'claude 실패: 사용 한도'), '사용 한도'),
])
def test_timeout_and_limit_skip_example_fallback(store, good, err, words):
    """시간 초과 · 한도는 도구 단계에서 나도 예시를 넣어 한 번 더 부르지 않는다(다시 불러도 같아 화면만 두 배로 멈춤) — 생성 · 스캔 채우기 둘 다."""
    v1 = derive(store)
    gen, client = make(store, [err, answer([good] * 3)])
    with pytest.raises(GenError) as e:
        gen.generate('벤치', v1)
    assert e.value.code == 'GEN_FAILED' and words in e.value.message and len(client.calls) == 1
    gen, client = make(store, [err, scan_answer(good, BENCH)])
    with pytest.raises(GenError):
        gen.scan_fill(scan_of(good))
    assert len(client.calls) == 1


def test_relay_reason_is_shown(store):
    """중계가 실패 까닭을 보냈으면(502 {'error': {'message'}}) 라이브러리 예외 이름 대신 그 글을 화면에 — 그 밖 실패라 final 은 아님."""
    gen, _ = make(store, [RelayError(502, 'claude 가 정해진 형식으로 답하지 않았다: 블록 3개')])
    with pytest.raises(GenError) as e:
        gen.generate('벤치', BENCH)
    assert e.value.message == '설계 AI 실패 — claude 가 정해진 형식으로 답하지 않았다: 블록 3개' and not e.value.final

"""AI 설계 생성(web/backend/design_gen.py — DesignGenerator, W108)이 SDD 6.6 · IRD 8.3 · E-72 · E-84 대로 도는지 지킨다.

OpenAI 없이 돈다(가짜 클라이언트가 정해 둔 답을 차례로 준다). 검사는 진짜 검사 묶음(d2_task DesignChecker + 변환기 ① — task_node 와 같은 연결).
지키는 것: 첫 호출은 도구 get_design 을 꼭 부르게 함(RAG) · 도구는 확인된 Template 설계만(enum) · 읽은 설계 = 부모 · 후보에 검사 전 새 ID
(Template 다음 V) · 합격 후보만 저장 · 다 떨어지면 검사 detail 을 피드백으로 다시(최대 2번) → GEN_FAILED · 범위 밖 OUT_OF_SCOPE ·
검사 서비스 실패(로봇 PC)는 다시 만들지 않음 · 도구 단계 실패면 예시를 코드가 넣음 · 기록(NFR-17) · Template 고르기 · 키 없음.
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

from d2_task.blocks_to_recipe import BlocksToRecipe  # noqa: E402
from d2_task.design_checker import DesignChecker  # noqa: E402
from design_gen import MAX_REGEN, DesignGenerator, GenError  # noqa: E402
from design_store import DesignStore  # noqa: E402

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
    body = {'out_of_scope': out, 'reason': reason, 'candidates': [{'idea': f'안 {i}', 'blocks': b} for i, b in enumerate(cands)]}
    return SimpleNamespace(content=json.dumps(body, ensure_ascii=False), refusal=None, tool_calls=None)


def real_check():
    """로봇 PC task_node 와 같은 검사 묶음(DesignChecker + 변환기 ①) — MqttClient.request 가 돌려주는 응답 꼴로."""
    checker = DesignChecker(CFG)
    checker.blocks_to_recipe = BlocksToRecipe(checker.grasp_options, [v * 1000 for v in CFG['block_size_m']]).convert

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
    return store.get_design(BENCH)['blocks']['blocks']


@pytest.fixture
def bad(good):
    """검사에 떨어지는 블록 목록 — 좌판 하나(9번)를 공중에 띄움."""
    b = copy.deepcopy(good)
    b[8]['z'] = 75.0
    return b


def make(store, replies, check=None):
    """가짜 OpenAI · 진짜 검사(또는 주어진 check)로 생성기를 만든다."""
    client = FakeOpenAI(replies)
    return DesignGenerator(store, check or real_check(), RULES, client=client), client


def test_generate_reads_reference_checks_and_saves_chosen(store, good, bad):
    """도구로 참고 설계를 읽고(부모) → 후보 3개에 새 ID → 진짜 검사 → 합격 후보 1개만 저장. 진행 표시 · 기록(NFR-17)도."""
    gen, client = make(store, [tool_call(BENCH), answer([good, bad, good])])
    events = []
    r = gen.generate('벤치를 만들어 줘', TEMPLATE, on_progress=lambda stage, info: events.append(stage))
    assert r['design_id'] == '001_CHAIR_BENCH_V001' and r['parent_id'] == BENCH and r['reference'] == [BENCH]
    assert [c['check']['ok'] for c in r['candidates']] == [True, False, True]
    assert r['candidates'][1]['check']['errors'] and r['candidates'][0]['blocks']['design_id'] == '001_CHAIR_BENCH_V001'
    assert events == ['reading', 'candidates', 'checked', 'checked', 'checked']
    first, second = client.calls
    assert first['model'] == 'gpt-4o' and first['temperature'] == 0 and first['tool_choice'] == 'required'
    assert first['parallel_tool_calls'] is False and first['response_format']['json_schema']['strict'] is True
    assert first['tools'][0]['function']['parameters']['properties']['design_id']['enum'] == [BENCH]   # 이 Template 설계만
    assert second['messages'][-1]['role'] == 'tool' and '"blocks"' in second['messages'][-1]['content']
    rec = gen.save(r, 0)
    assert rec['design_id'] == '001_CHAIR_BENCH_V001' and rec['parent_id'] == BENCH and rec['made_by'] == 'web'
    assert rec['prompt'] == '벤치를 만들어 줘' and rec['recipe']['model_id'] == '001_CHAIR_BENCH_V001'
    with pytest.raises(ValueError):
        gen.save(r, 1)                                       # 떨어진 후보는 못 고름
    logs = list(store.gen_logs_dir.glob('*_generate.json'))
    assert len(logs) == 1
    log = json.loads(logs[0].read_text(encoding='utf-8'))
    assert log['messages'][0]['role'] == 'system' and len(log['calls']) == 2 and log['result']['design_id'] == '001_CHAIR_BENCH_V001'
    assert 'sk-' not in logs[0].read_text(encoding='utf-8')


def test_regenerates_with_check_feedback_then_succeeds(store, good, bad):
    """3개 다 떨어지면 검사 detail 을 피드백으로 다시 — 두 번째에 합격하면 그것을 돌려준다(같은 ID)."""
    gen, client = make(store, [tool_call(BENCH), answer([bad] * 3), answer([good, bad, bad])])
    r = gen.generate('벤치', TEMPLATE)
    assert r['attempts'] == 2 and r['candidates'][0]['check']['ok'] and r['design_id'] == '001_CHAIR_BENCH_V001'
    retry = client.calls[-1]
    assert retry['tool_choice'] == 'none' and retry['messages'][-1]['content'].startswith('[검사 결과]')
    assert '9번 블록' in retry['messages'][-1]['content']


def test_gives_up_after_two_regenerations(store, bad):
    """다시 만들기는 최대 2번 — 그래도 다 떨어지면 GEN_FAILED(마지막 errors 를 detail 로)."""
    gen, client = make(store, [tool_call(BENCH)] + [answer([bad] * 3)] * (MAX_REGEN + 1))
    with pytest.raises(GenError) as e:
        gen.generate('벤치', TEMPLATE)
    assert e.value.code == 'GEN_FAILED' and e.value.detail
    assert len(client.calls) == 1 + 1 + MAX_REGEN


def test_out_of_scope(store):
    """AI 가 범위 밖이라고 하면 OUT_OF_SCOPE(이유 그대로) — 검사 · 저장 없음."""
    gen, _ = make(store, [tool_call(BENCH), answer([], out=True, reason='벤치로는 2층 침대를 못 만들어요')])
    with pytest.raises(GenError) as e:
        gen.generate('2층 침대', TEMPLATE)
    assert e.value.code == 'OUT_OF_SCOPE' and '침대' in e.value.message
    assert [r['design_id'] for r in store.list_designs(template=TEMPLATE)] == [BENCH]


def test_tool_answers_only_template_designs_and_stops_after_two(store, good):
    """목록 밖 설계를 부르면 이유만 돌려주고(읽지 않음), 2번 읽으면 그다음은 도구를 못 부르게(tool_choice none)."""
    gen, client = make(store, [tool_call('002_CHAIR_BACK_V000'), tool_call(BENCH, 'c2'), tool_call(BENCH, 'c3'),
                               answer([good] * 3)])
    r = gen.generate('벤치', TEMPLATE)                         # 3라운드 내내 도구만 부름 → 예시를 코드가 넣는 길로(E-72)
    assert r['reference'] == [] and r['parent_id'] == BENCH and 'tools' not in client.calls[-1]
    gen, client = make(store, [tool_call(BENCH), tool_call(BENCH, 'c2'), answer([good] * 3)])
    r = gen.generate('벤치', TEMPLATE)
    assert [c['tool_choice'] for c in client.calls] == ['required', 'auto', 'none'] and r['reference'] == [BENCH, BENCH]
    gen, client = make(store, [tool_call('002_CHAIR_BACK_V000'), tool_call(BENCH, 'c2'), answer([good] * 3)])
    r = gen.generate('벤치', TEMPLATE)
    assert 'error' in client.calls[1]['messages'][-1]['content'] and r['reference'] == [BENCH]


def test_falls_back_to_examples_when_tool_phase_fails(store, good):
    """도구 단계 호출이 실패하면 같은 Template 예시를 코드가 넣어 한 번 더(E-72 자동 전환) — 부모 = 기본 설계."""
    gen, client = make(store, [RuntimeError('network'), answer([good] * 3)])
    r = gen.generate('벤치', TEMPLATE)
    assert r['parent_id'] == BENCH and r['reference'] == []
    assert 'tools' not in client.calls[-1] and '도구 대신 코드가 넣음' in client.calls[-1]['messages'][1]['content']


def test_check_service_failure_is_not_regenerated(store, good):
    """검사 서비스가 답을 못 하면(로봇 PC · 다리 — TIMEOUT · ERROR) AI 탓이 아니라 다시 만들지 않고 그 이유로 끝낸다."""
    gen, client = make(store, [tool_call(BENCH), answer([good] * 3)],
                       check=lambda b: {'success': False, 'reason': 'TIMEOUT', 'message': '5초 안에 답이 없다'})
    with pytest.raises(GenError) as e:
        gen.generate('벤치', TEMPLATE)
    assert e.value.code == 'TIMEOUT' and len(client.calls) == 2


def test_unknown_template_and_bad_answer(store):
    """모르는 Template 은 부르기 전에 거절, AI 답이 JSON 이 아니면 GEN_FAILED(고쳐 읽지 않음)."""
    gen, client = make(store, [])
    with pytest.raises(GenError):
        gen.generate('소파', '009_SOFA_LONG')
    assert client.calls == []
    gen, _ = make(store, [tool_call(BENCH), SimpleNamespace(content='블록을 이렇게…', refusal=None, tool_calls=None)])
    with pytest.raises(GenError) as e:
        gen.generate('벤치', TEMPLATE)
    assert e.value.code == 'GEN_FAILED'


def test_classify_template_or_none(store):
    """Template 고르기(E-84 ③): 등록된 4개 + NONE 중 하나. NONE 이면 template None — 화면은 범위 밖 안내."""
    reply = SimpleNamespace(content=json.dumps({'template': '003_DESK_STAND', 'reason': '다리가 선 책상'}), refusal=None, tool_calls=None)
    none = SimpleNamespace(content=json.dumps({'template': 'NONE', 'reason': '소파는 없어요'}), refusal=None, tool_calls=None)
    gen, client = make(store, [reply, none])
    assert gen.classify('다리가 긴 책상') == {'template': '003_DESK_STAND', 'reason': '다리가 선 책상'}
    assert gen.classify('소파') == {'template': None, 'reason': '소파는 없어요'}
    call = client.calls[0]
    assert call['model'] == 'gpt-4o'                     # 팀 키는 gpt-4o · whisper-1 만(10/10)
    assert call['response_format']['json_schema']['schema']['properties']['template']['enum'] == \
        ['001_CHAIR_BENCH', '002_CHAIR_BACK', '003_DESK_STAND', '004_DESK_PEDESTAL', 'NONE']


def test_prompt_takes_rules_from_robot_yaml_and_roles(store):
    """프롬프트 숫자는 robot.yaml(E-78 키)에서, 역할 이름은 roles.json 에서 — 빈 자리($) 없이."""
    gen, _ = make(store, [])
    p = gen.design_prompt
    assert '75 × 25 × 15' in p and f"{CFG['check']['margin_mm']} mm" in p and 'COLUMN' in p and '$' not in p
    role_enum = gen._candidates_format()['json_schema']['schema']['properties']['candidates']['items']['properties']['blocks'][
        'items']['properties']['role']['enum']
    assert 'LEG' in role_enum and 'LEG_WHEEL' in role_enum


def test_no_key_is_gen_failed(store, monkeypatch):
    """키가 없으면(또는 openai 라이브러리가 없으면) 웹은 켜져 있고 생성만 GEN_FAILED."""
    monkeypatch.delenv('OPENAI_API_KEY', raising=False)
    gen = DesignGenerator(store, real_check(), RULES)
    with pytest.raises(GenError) as e:
        gen.generate('벤치', TEMPLATE)
    assert e.value.code == 'GEN_FAILED'

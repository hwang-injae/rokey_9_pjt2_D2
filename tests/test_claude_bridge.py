# -*- coding: utf-8 -*-
"""claude_bridge(웹 PC 호스트 중계) — claude 실행 없이 가짜 실행 함수로 요청 → claude 인자 · 응답 모양을 본다.

진짜 design_gen 생성기를 중계 앞에 붙여(HTTP 없이) 도구 읽기 → 후보 → 검사가 끝까지 도는지도 본다.
"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / 'web', ROOT / 'web' / 'backend', ROOT / 'src' / 'd2_task', ROOT / 'src' / 'd2_bridge'):
    sys.path.insert(0, str(p))

from claude_bridge import BridgeError, ClaudeBridge  # noqa: E402

TOOL = {'type': 'function', 'function': {'name': 'get_design', 'strict': True, 'description': '설계 읽기',
        'parameters': {'type': 'object', 'additionalProperties': False, 'required': ['design_id'],
                       'properties': {'design_id': {'type': 'string', 'enum': ['001_CHAIR_BENCH_V000']}}}}}
FMT = {'type': 'json_schema', 'json_schema': {'name': 'x', 'strict': True, 'schema': {
    'type': 'object', 'required': ['a'], 'properties': {'a': {'type': 'integer'}}}}}


def fake(reply):
    """claude 대신 — 받은 (argv, 입력)을 모으고 정해 둔 JSON 출력(dict)을 돌려준다."""
    calls = []

    def run(argv, prompt):
        calls.append((argv, prompt))
        return json.dumps(reply)
    return run, calls


def flag(argv, name):
    """argv 에서 name 뒤 값."""
    return argv[argv.index(name) + 1]


def test_schema_answer_becomes_json_content():
    """response_format json_schema → --json-schema 로 넘기고 structured_output 을 content(JSON 글자)로. 도구 · MCP 는 끈다."""
    run, calls = fake({'is_error': False, 'result': '', 'structured_output': {'a': 3},
                       'usage': {'input_tokens': 10, 'cache_read_input_tokens': 5, 'output_tokens': 7}})
    res = ClaudeBridge('claude', 'sonnet', run).complete({
        'model': 'opus', 'response_format': FMT,
        'messages': [{'role': 'system', 'content': '규칙'}, {'role': 'user', 'content': '벤치'}]})
    msg = res['choices'][0]['message']
    assert json.loads(msg['content']) == {'a': 3} and msg['refusal'] is None and res['choices'][0]['finish_reason'] == 'stop'
    assert res['usage'] == {'prompt_tokens': 15, 'completion_tokens': 7, 'total_tokens': 22}
    argv, prompt = calls[0]
    assert flag(argv, '--model') == 'opus' and flag(argv, '--system-prompt') == '규칙' and flag(argv, '--tools') == ''
    assert json.loads(flag(argv, '--json-schema')) == FMT['json_schema']['schema']
    assert '--no-session-persistence' in argv and '--strict-mcp-config' in argv
    assert prompt.startswith('[사용자]\n벤치') and '규칙' not in prompt          # system 은 입력이 아니라 --system-prompt 로


def test_forced_tool_returns_tool_call_with_parameter_schema():
    """tool_choice required → 도구 인자 스키마로 물어 tool_calls 로. 'auto' 면 도구는 무시하고 답을 쓴다(참고 설계 1개만 읽음)."""
    run, calls = fake({'is_error': False, 'structured_output': {'design_id': '001_CHAIR_BENCH_V000'}, 'usage': {}})
    bridge = ClaudeBridge('claude', 'sonnet', run)
    res = bridge.complete({'model': 'sonnet', 'tools': [TOOL], 'tool_choice': 'required', 'response_format': FMT,
                           'messages': [{'role': 'user', 'content': '벤치'}]})
    msg = res['choices'][0]['message']
    assert msg['content'] is None and res['choices'][0]['finish_reason'] == 'tool_calls'
    tc = msg['tool_calls'][0]
    assert tc['function']['name'] == 'get_design' and json.loads(tc['function']['arguments']) == {'design_id': '001_CHAIR_BENCH_V000'}
    assert json.loads(flag(calls[0][0], '--json-schema')) == TOOL['function']['parameters']
    # 다음 라운드(auto): 도구 호출 · 결과가 글로 펼쳐지고, 답 스키마로 묻는다
    run2, calls2 = fake({'is_error': False, 'structured_output': {'a': 1}, 'usage': {}})
    bridge.run = run2
    bridge.complete({'model': 'sonnet', 'tools': [TOOL], 'tool_choice': 'auto', 'response_format': FMT, 'messages': [
        {'role': 'user', 'content': '벤치'},
        {'role': 'assistant', 'content': None, 'tool_calls': [tc]},
        {'role': 'tool', 'tool_call_id': tc['id'], 'content': '{"blocks":[]}'}]})
    argv, prompt = calls2[0]
    assert json.loads(flag(argv, '--json-schema')) == FMT['json_schema']['schema']
    assert '(도구 get_design 호출: {"design_id": "001_CHAIR_BENCH_V000"})' in prompt and '[도구 결과]\n{"blocks":[]}' in prompt


def test_model_alias_or_default():
    """gpt-4o 같은 모르는 이름(시험 안 한 haiku 포함)은 기본 모델, 별칭 · claude- 전체 ID 는 그대로."""
    b = ClaudeBridge('claude', 'sonnet', lambda *_: '')
    assert b.model_of('gpt-4o') == 'sonnet' and b.model_of('haiku') == 'sonnet'
    assert b.model_of('opus') == 'opus' and b.model_of('claude-opus-5-5') == 'claude-opus-5-5'


@pytest.mark.parametrize('reply, words', [
    ({'is_error': True, 'result': "You've hit your usage limit"}, 'limit'),
    ({'is_error': False, 'result': '형식 없이 글만', 'structured_output': None}, '형식'),
])
def test_claude_failure_is_502(reply, words):
    """claude 오류(사용 한도 등) · 형식 없는 답 → 502 + 이유 그대로(design_gen 이 화면에 알림)."""
    run, _ = fake(reply)
    with pytest.raises(BridgeError) as e:
        ClaudeBridge('claude', 'sonnet', run).complete({'response_format': FMT, 'messages': [{'role': 'user', 'content': 'x'}]})
    assert e.value.status == 502 and words in e.value.message


def test_design_gen_runs_through_bridge(tmp_path):
    """진짜 생성기 → (HTTP 대신 바로) 중계 → 가짜 claude: 도구 읽기 → 후보 → 진짜 검사 → 저장까지. 생성기 코드는 그대로."""
    import yaml
    from d2_task.blocks_to_recipe import BlocksToRecipe
    from d2_task.design_checker import DesignChecker
    from design_gen import DesignGenerator
    from design_store import DesignStore
    from test_design_gen import RECIPES, RULES, BENCH, TEMPLATE

    cfg = yaml.safe_load((ROOT / 'src' / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml').read_text(encoding='utf-8'))
    checker = DesignChecker(cfg)
    checker.blocks_to_recipe = BlocksToRecipe(checker.grasp_options, [v * 1000 for v in cfg['block_size_m']]).convert

    def check(blocks):
        ok, reason, rj = checker.handle_json(json.dumps(blocks, ensure_ascii=False))
        return {'success': ok, 'reason': reason, **json.loads(rj)}

    store = DesignStore(tmp_path, RECIPES, [v * 1000 for v in cfg['block_size_m']])
    store.register_bases()
    bench = store.get_design(BENCH)['blocks']['blocks']
    answers = iter([{'design_id': BENCH},
                    {'out_of_scope': False, 'reason': '', 'candidates': [{'idea': '그대로', 'plan': 'p', 'blocks': bench}] * 3}])

    def run(argv, prompt):
        return json.dumps({'is_error': False, 'structured_output': next(answers), 'usage': {'output_tokens': 1}})
    bridge = ClaudeBridge('claude', 'sonnet', run)

    def create(**kw):   # OpenAI 라이브러리 대신 — 요청 dict 를 중계에 그대로, 응답 dict 를 속성 모양으로
        res = json.loads(json.dumps(bridge.complete(json.loads(json.dumps(kw)))))
        m = res['choices'][0]['message']
        calls = [SimpleNamespace(id=t['id'], function=SimpleNamespace(**t['function'])) for t in m.get('tool_calls') or []] or None
        return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content=m['content'], refusal=None, tool_calls=calls))],
                               usage=SimpleNamespace(total_tokens=res['usage']['total_tokens']))
    client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create)))
    r = DesignGenerator(store, check, RULES, client=client).generate('벤치', TEMPLATE)
    assert r['reference'] == [BENCH] and r['attempts'] == 1 and all(c['check']['ok'] for c in r['candidates'])

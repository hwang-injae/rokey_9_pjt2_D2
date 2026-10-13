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
    """진짜 생성기 → (HTTP 대신 바로) 중계 → 가짜 claude 로 끝까지. 생성기 코드는 그대로.
    ① 설계 생성(바탕 = 고른 V000 — 다른 설계가 없어 도구 없이 바로 후보) → 진짜 검사 합격
    ② 스캔 [저장] 채우기 scan_fill(W177 — 첫 호출은 도구 강제 required → 중계가 도구 호출 모양으로) → 읽은 설계 · 칸 채움 → 진짜 검사 합격."""
    from design_gen import DesignGenerator
    from design_store import DesignStore
    from test_design_gen import BENCH, RECIPES, RULES, real_check, scan_of

    store = DesignStore(tmp_path, RECIPES, [75, 25, 15])
    store.register_bases()
    bench = store.get_design(BENCH)['parts']['parts']
    n = len(bench)
    fill = [{'scan_num': n - p['order'] + 1, **{k: p[k] for k in ('order', 'role', 'group', 'stage', 'grasp')}} for p in bench]
    answers = iter([
        {'out_of_scope': False, 'reason': '', 'candidates': [{'idea': '그대로', 'plan': 'p', 'parts': bench}] * 3},   # ① 후보
        {'design_id': BENCH},                                                                                            # ② 도구 읽기
        {'parent_id': BENCH, 'reason': '같은 벤치', 'plan': 'p', 'parts': fill}])                                         # ② 채우기

    def run(argv, prompt):
        return json.dumps({'is_error': False, 'structured_output': next(answers), 'usage': {'output_tokens': 1}})
    bridge = ClaudeBridge('claude', 'opus', run)

    def create(**kw):   # OpenAI 라이브러리 대신 — 요청 dict 를 중계에 그대로, 응답 dict 를 속성 모양으로
        res = json.loads(json.dumps(bridge.complete(json.loads(json.dumps(kw)))))
        m = res['choices'][0]['message']
        calls = [SimpleNamespace(id=t['id'], function=SimpleNamespace(**t['function'])) for t in m.get('tool_calls') or []] or None
        return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content=m['content'], refusal=None, tool_calls=calls))],
                               usage=SimpleNamespace(total_tokens=res['usage']['total_tokens']))
    client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create)))
    gen = DesignGenerator(store, real_check(), RULES, client=client)
    r = gen.generate('벤치', BENCH)
    assert r['parent_id'] == BENCH and r['attempts'] == 1 and r['candidates'] and all(c['check']['ok'] for c in r['candidates'])
    f = gen.scan_fill(scan_of(bench))
    assert f['parent_id'] == BENCH and f['ai_reason'] == '같은 벤치'
    design_id, _ = store.new_design_id('001_CHAIR_BENCH', BENCH)
    assert gen._check({'schema': 'parts/3.0', 'design_id': design_id, 'parts': f['parts']})['ok']


# ---------- 웹 PC 준비 점검(--check, 10/13) — claude · 네트워크 · docker 는 모두 가짜 ----------
from claude_bridge import SetupCheck  # noqa: E402

HELP_ALL = ' '.join(['--output-format', '--model', '--tools', '--no-session-persistence', '--strict-mcp-config',
                     '--disable-slash-commands', '--system-prompt', '--json-schema'])


def fake_cmd(help_text=HELP_ALL, container='web-web-1', seen_from_container=True):
    """claude --version · --help · docker ps · docker exec 대신 — 정해 둔 (종료 코드, 출력)."""
    def cmd(argv, timeout):
        if argv[-1] == '--version':
            return 0, '2.1.295 (Claude Code)\n'
        if argv[-1] == '--help':
            return 0, help_text
        if argv[:2] == ['docker', 'ps']:
            return 0, container + '\n' if container else ''
        if argv[:2] == ['docker', 'exec']:
            return (0, '{"ok": true}') if seen_from_container else (1, 'URLError: timed out')
        raise AssertionError(argv)
    return cmd


def good_claude(argv, prompt):
    """로그인 · 모델이 되는 claude — 정해진 형식 답."""
    return json.dumps({'is_error': False, 'structured_output': {'n': 1}, 'usage': {}})


def check(**kw):
    """가짜로 채운 SetupCheck — 바꿀 것만 kw 로."""
    args = dict(host='127.0.0.1', port=8790, model='opus', claude='/x/claude', cmd=fake_cmd(),
                fetch=lambda url, t: {'ok': True, 'default_model': 'opus'}, bridge_run=good_claude)
    args.update(kw)
    return SetupCheck(**args)


def states(results):
    """결과 목록 → 상태만."""
    return [r[0] for r in results]


def test_check_all_ok_and_flags_follow_bridge_code():
    """다 되면 6개 ✅ · 종료 코드 0 쪽 요약. 옵션 목록은 중계 _ask 가 실제로 넘기는 것(코드를 따라감)."""
    c = check()
    r = c.run()
    assert states(r) == ['ok'] * 6 and '준비 끝' in SetupCheck.report(r)
    assert set(c.used_flags()) == set(HELP_ALL.split())


def test_check_missing_flag_skips_login_and_says_upgrade():
    """옛 판(옵션 없음)이면 ② ❌ + 새 판으로, 진짜 호출 ③은 건너뜀."""
    r = check(cmd=fake_cmd(help_text=HELP_ALL.replace('--json-schema', ''))).run()
    assert states(r)[:3] == ['ok', 'fail', 'skip'] and '--json-schema' in r[1][2] and '새 판' in r[1][3]


@pytest.mark.parametrize('result, todo', [("You've hit your usage limit", '한도'), ('Please run /login', '로그인')])
def test_check_login_failure_tells_what_to_do(result, todo):
    """claude 오류 → ③ ❌ + 이유별 할 일(사용 한도 · 로그인)."""
    r = check(bridge_run=lambda argv, prompt: json.dumps({'is_error': True, 'result': result})).run()
    assert r[2][0] == 'fail' and todo in r[2][3]


def test_check_bridge_off_and_no_container():
    """중계가 꺼져 있으면 ⑤ ❌ + 켜는 명령(⑥은 ⏭), 웹 컨테이너가 없으면 ⑥ ⏭. --no-call 이면 ③ ⏭ · claude 가 없으면 ①②③."""
    def refuse(url, t):
        raise ConnectionRefusedError
    r = check(fetch=refuse, cmd=fake_cmd(container=''), call=False).run()
    assert states(r) == ['ok', 'ok', 'skip', 'ok', 'fail', 'skip'] and '--host 127.0.0.1' in r[4][3]
    r = check(fetch=refuse).run()                                       # 꺼져 있으면 ⑥은 방화벽 탓으로 몰지 않고 ⏭
    assert r[5][:3] == ('skip', '웹 컨테이너에서', '중계가 꺼져 있어 못 봄')
    assert states(check(claude='').run())[:3] == ['fail', 'skip', 'skip']


def test_check_container_cannot_see_bridge():
    """웹 컨테이너에서 안 보이면 ⑥ ❌ + 방화벽 · extra_hosts 확인."""
    r = check(cmd=fake_cmd(seen_from_container=False)).run()
    assert r[5][0] == 'fail' and 'ufw' in r[5][3] and 'extra_hosts' in r[5][3] and 'timed out' in r[5][2]
    assert '❌ 1개' in SetupCheck.report(r)

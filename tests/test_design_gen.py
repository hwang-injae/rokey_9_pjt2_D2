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

from d2_task.parts_to_recipe import PartsToRecipe  # noqa: E402
from d2_task.design_checker import DesignChecker  # noqa: E402
from design_gen import MAX_REGEN, DesignGenerator, GenError, scan_actions  # noqa: E402
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


def make(store, replies, check=None):
    """가짜 OpenAI · 진짜 검사(또는 주어진 check)로 생성기를 만든다."""
    client = FakeOpenAI(replies)
    return DesignGenerator(store, check or real_check(), RULES, client=client), client


def test_generate_reads_reference_checks_and_saves_chosen(store, good, bad):
    """도구로 참고 설계를 읽고(부모) → 후보 3개에 새 ID → 진짜 검사 → 합격 후보 1개만 저장. 진행 표시 · 기록(NFR-17)도.
    검사 요청에는 V000 레시피 통째(base_recipe — 10/13 PL)를 싣고, 후보 · 저장 설계에는 안 넣는다. 저장하면 AI 판단 기록 · AI 기록 번호(#14)."""
    sent, check = [], real_check()
    gen, client = make(store, [tool_call(BENCH), answer([good, bad, good])], check=lambda req: sent.append(req) or check(req))
    events = []
    r = gen.generate('벤치를 만들어 줘', TEMPLATE, on_progress=lambda stage, info: events.append(stage))
    assert r['design_id'] == '001_CHAIR_BENCH_V001' and r['parent_id'] == BENCH and r['reference'] == [BENCH]
    assert [c['check']['ok'] for c in r['candidates']] == [True, False, True]
    assert r['candidates'][1]['check']['errors'] and r['candidates'][0]['parts']['design_id'] == '001_CHAIR_BENCH_V001'
    assert len(sent) == 3 and all(req['base_recipe'] == store.get_design(BENCH)['recipe'] for req in sent)
    assert all('base_recipe' not in c['parts'] for c in r['candidates'])
    assert events == ['reading', 'candidates', 'checked', 'checked', 'checked']
    first, second = client.calls
    assert first['model'] == 'gpt-4o' and first['temperature'] == 0 and first['tool_choice'] == 'required'
    assert first['parallel_tool_calls'] is False and first['response_format']['json_schema']['strict'] is True
    assert first['tools'][0]['function']['parameters']['properties']['design_id']['enum'] == [BENCH]   # 이 Template 설계만
    assert second['messages'][-1]['role'] == 'tool' and '"parts"' in second['messages'][-1]['content']
    rec = gen.save(r, 0, ai_template='002_CHAIR_BACK', ai_reason='등받이가 있을 것 같아서')    # 사람이 확인 화면에서 바꾼 경우
    assert rec['design_id'] == '001_CHAIR_BENCH_V001' and rec['parent_id'] == BENCH and rec['made_by'] == 'web'
    assert rec['prompt'] == '벤치를 만들어 줘' and rec['recipe']['model_id'] == '001_CHAIR_BENCH_V001'
    assert (rec['ai_template'], rec['ai_reason'], rec['template_changed']) == ('002_CHAIR_BACK', '등받이가 있을 것 같아서', True)
    assert 'base_recipe' not in rec['parts'] and 'revision' not in json.dumps(rec['recipe'])   # AI 레시피 = composition 만(레시피 초안 6절)
    with pytest.raises(ValueError):
        gen.save(r, 1)                                       # 떨어진 후보는 못 고름
    logs = list(store.gen_logs_dir.glob('*_generate.json'))
    assert len(logs) == 1 and r['gen_log'] == logs[0].name
    log = json.loads(logs[0].read_text(encoding='utf-8'))
    assert log['messages'][0]['role'] == 'system' and len(log['calls']) == 2 and log['result']['design_id'] == '001_CHAIR_BENCH_V001'
    assert log['design_id'] == '001_CHAIR_BENCH_V001'        # 고른 후보를 저장할 때 채움(설계_DB #14)
    assert 'sk-' not in logs[0].read_text(encoding='utf-8')


def test_regenerates_with_check_feedback_then_succeeds(store, good, bad):
    """3개 다 떨어지면 검사 detail 을 피드백으로 다시 — 두 번째에 합격하면 그것을 돌려준다(같은 ID)."""
    gen, client = make(store, [tool_call(BENCH), answer([bad] * 3), answer([good, bad, bad])])
    r = gen.generate('벤치', TEMPLATE)
    assert r['attempts'] == 2 and r['candidates'][0]['check']['ok'] and r['design_id'] == '001_CHAIR_BENCH_V001'
    retry = client.calls[-1]
    assert retry['tool_choice'] == 'none' and retry['messages'][-1]['content'].startswith('[검사 결과]')
    fb = retry['messages'][-1]['content']
    assert '9번' in fb and '공중' in fb and '고칠 것' in fb and '후보 1(안 0)' in fb     # 후보별로 묶고 고치는 법을 붙임
    assert '아랫면 z 75.0' in fb and '가장 높아야 z 60.0' in fb                        # 어느 층이 비었는지(다리 꼭대기 60)


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
    bench = store.get_design(BENCH)['recipe']['template']   # 설명 = V000 레시피 template 칸(코드 표 없음, 10/13 PL)
    assert f"- {TEMPLATE}: {bench['name']} — {bench['description']} (" in call['messages'][0]['content']
    assert call['response_format']['json_schema']['schema']['properties']['template']['enum'] == \
        ['001_CHAIR_BENCH', '002_CHAIR_BACK', '003_DESK_STAND', '004_DESK_PEDESTAL', 'NONE']


def test_prompt_takes_rules_from_robot_yaml_and_roles(store):
    """프롬프트 숫자는 robot.yaml(E-78 키)에서, 역할 이름은 roles.json 에서 — 빈 자리($) 없이."""
    gen, _ = make(store, [])
    p = gen.design_prompt
    assert '75 × 25 × 15' in p and f"{CFG['check']['margin_mm']} mm" in p and 'COLUMN' in p and '$' not in p
    role_enum = gen._candidates_format()['json_schema']['schema']['properties']['candidates']['items']['properties']['parts'][
        'items']['properties']['role']['enum']
    assert 'LEG' in role_enum and 'LEG_WHEEL' in role_enum


def test_no_key_is_gen_failed(store, monkeypatch):
    """키가 없으면(또는 openai 라이브러리가 없으면) 웹은 켜져 있고 생성만 GEN_FAILED."""
    monkeypatch.delenv('OPENAI_API_KEY', raising=False)
    gen = DesignGenerator(store, real_check(), RULES)
    with pytest.raises(GenError) as e:
        gen.generate('벤치', TEMPLATE)
    assert e.value.code == 'GEN_FAILED'


def test_no_credit_message(store):
    """크레딧이 없으면(429 insufficient_quota) 기다리라는 말 대신 크레딧을 채우라고 알린다(10/10 실제로 겪음)."""
    gen, _ = make(store, [RuntimeError("Error code: 429 - {'error': {'code': 'insufficient_quota'}}")])
    with pytest.raises(GenError) as e:
        gen.classify('벤치')
    assert e.value.code == 'GEN_FAILED' and '크레딧' in e.value.message



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
        assert chk['ok'] and chk['recipe']['model_id'] == design_id and 'revision' not in json.dumps(chk['recipe']), (t, chk['errors'])
    wrong = real_check()({**parts, 'base_recipe': store.base_recipe('001_CHAIR_BENCH')})
    assert wrong['ok'] is False and [e['reason'] for e in wrong['errors']] == ['REQUEST_INVALID']
# ---------- 스캔 [저장] 채우기(W177 · IRD 8.4 ⑦ · E-79) ----------
def scan_answer(parts, parent, reason='블록 수 · 다리 벽이 같다', drop=0):
    """GPT 의 스캔 채우기 답(W177 진짜 scan_fill 시험용) — scan_num 마다 채울 칸만(scan_of 의 번호 규칙: 거꾸로 = n − order + 1).
    drop 개만큼 뒤에서 뺀다(모양 틀린 답)."""
    n = len(parts)
    rows = [{'scan_num': n - p['order'] + 1, **{k: p[k] for k in ('order', 'role', 'group', 'stage', 'grasp')}} for p in parts]
    body = {'parent_id': parent, 'reason': reason, 'plan': 'z0: …', 'parts': rows[:len(rows) - drop]}
    return SimpleNamespace(content=json.dumps(body, ensure_ascii=False), refusal=None, tool_calls=None)


def test_scan_fill_keeps_scan_shape_and_passes_check(store, good):
    """도구로 비슷한 설계를 읽고(DB 전체) → scan_num 마다 칸만 받아 위치 · 방향은 스캔 그대로 → 진짜 검사 합격 · 기록."""
    gen, client = make(store, [tool_call(BENCH), scan_answer(good, BENCH)])
    r = gen.scan_fill(scan_of(good))
    assert r['parent_id'] == BENCH and r['ai_template'] == TEMPLATE and r['ai_reason'] == '블록 수 · 다리 벽이 같다'
    assert [{k: p[k] for k in ('order', 'x', 'y', 'z', 'ori', 'role', 'group', 'stage', 'grasp')} for p in r['parts']] == \
        [{k: p[k] for k in ('order', 'x', 'y', 'z', 'ori', 'role', 'group', 'stage', 'grasp')} for p in good]
    assert all('inferred' not in p for p in r['parts'])                     # 스캔에 없는 칸은 만들지 않음
    design_id, _ = store.new_design_id(TEMPLATE, r['parent_id'])
    chk = gen._check({'schema': 'parts/3.0', 'design_id': design_id, 'parts': r['parts']})
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
    assert {'scan_num': n - 9 + 1, 'order': 9} == {k: shown[8][k] for k in ('scan_num', 'order')}   # 앞 order 9 = 어느 스캔 블록인지


def test_scan_fill_rejects_bad_input_before_calling_ai(store):
    """스캔 모양이 틀리면(blocks 없음 · scan_num 겹침) AI 를 부르기 전에 ERROR."""
    gen, client = make(store, [])
    for bad in ({}, {'blocks': {'blocks': []}},
                {'blocks': {'blocks': [{'scan_num': 1, 'x': 0, 'y': 0, 'z': 0, 'ori': 'x'}] * 2}}):
        with pytest.raises(GenError) as e:
            gen.scan_fill(bad)
        assert e.value.code == 'ERROR'
    assert client.calls == []

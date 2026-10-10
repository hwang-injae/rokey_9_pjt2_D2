# -*- coding: utf-8 -*-
"""DesignGenerator — AI 설계 생성에서 GPT 를 부르는 한 곳 (SDD 6.6 · IRD 8.3, W108). ROS 없음.

흐름(E-69 · E-67 · E-72 · E-84):
  classify(text)           → 어느 Template 인지 AI 가 고름 → 화면이 사용자에게 한 번 확인받음(W163). 의자 · 책상 밖이면 None
  generate(text, template) → ① 그 Template 설계 목록 요약을 주고 도구 get_design(최대 2번 — 그 Template 설계만)으로 참고 설계를 읽게 함
                             ② 같은 대화에서 후보 3개(블록 목록)를 받음(구조화 출력 · temperature 0)
                             ③ 새 ID(store.new_design_id)를 붙여 check_design 3번(쌓을 만한지는 검사 묶음 — 로봇 PC task)
                             ④ 3개 다 불합격이면 errors[].detail 을 피드백으로 다시(최대 2번) → 그래도 실패 GEN_FAILED
  save(result, index)      → 사람이 고른 합격 후보 1개만 store.save_design. 안 고른 후보는 버린다(E-84 ⑥)
AI 가 빠뜨리거나 틀린 칸을 코드가 대신 채우지 않는다(E-69) — 검사에서 떨어져 다시 만든다.
기록(NFR-17): 프롬프트 · 응답 원문 · 시간 · 검사 결과를 store.save_gen_log 로. OpenAI 키는 환경 변수에서만 읽고 어디에도 적지 않는다.
"""
import hashlib
import json
import string
import time
from pathlib import Path

from d2_task.recipe_document import GRASPS
from design_store import SCHEMA_BLOCKS, build_summary, family_of

MODEL = 'gpt-4o'                 # 설계 작성(SDD 6.6 · TR-12)
MODEL_CLASSIFY = 'gpt-4o'        # Template 고르기 — 팀 키 프로젝트는 gpt-4o · whisper-1 만 열려 있다(10/10 모델 목록 확인 — mini 는 403)
CALL_TIMEOUT_S = 30              # 호출당 시간 제한(TR-12)
CALL_RETRIES = 1                 # 호출 실패 때 다시 1번(TR-12)
MAX_READS = 2                    # 도구 get_design 최대 횟수(E-72)
MAX_REGEN = 2                    # 3개 다 불합격일 때 다시 만드는 횟수(SDD 6.6)
N_CANDIDATES = 3                 # 후보 수(E-67)
FEEDBACK_MAX = 30                # 피드백 줄 수 상한 — 블록이 많이 틀려도 프롬프트가 지나치게 길어지지 않게
ORIS = ('x', 'y', 'xe', 'ye', 'zx', 'zy')   # IRD 2장 방향 코드
PROMPTS = Path(__file__).parent / 'prompts'
ROLES = Path(__file__).resolve().parents[2] / 'src' / 'd2_task' / 'd2_task' / 'roles.json'   # 변환기 ①과 같은 목록 파일
# Template 설명(AI 가 고를 때 · 사용자 확인 화면). 10/11 V000 레시피의 template 칸(E-84 ③)이 정해지면 그 설명으로 바꾼다
TEMPLATE_HINTS = {
    '001_CHAIR_BENCH': '벤치 — 등받이 없는 의자(다리 벽 둘 + 좌판)',
    '002_CHAIR_BACK': '등받이 의자 — 벤치 + 뒤쪽 등받이',
    '003_DESK_STAND': '세운 다리 책상 — 세운 블록 다리 + 보 + 상판',
    '004_DESK_PEDESTAL': '가운데 기둥 책상 — 바닥 받침 + 가운데 기둥 + 상판',
}
MADE_KO = {'cad': '도면', 'web': '웹 글', 'voice': '음성', 'scan': '스캔'}


class GenError(Exception):
    """생성이 끝까지 못 간 이유. code = OUT_OF_SCOPE · GEN_FAILED(IRD 7장) · ERROR · TIMEOUT(검사 서비스). 화면은 message 를 보여 준다."""

    def __init__(self, code, message, detail=None):
        """code · 사람에게 보일 한 줄 · detail(마지막 검사 errors 등 목록)."""
        super().__init__(message)
        self.code, self.message, self.detail = code, message, detail or []


class DesignGenerator:
    """요청 문장 → Template 고르기 · 후보 3개 · 검사 · 저장. 상태 없음(요청마다 새 대화) — 후보는 부르는 쪽(작업 job)이 들고 있다.

    입력: store(DesignStore), check(blocks/2.0 dict → check_design 응답 dict — MqttClient.request 로 만든 함수),
          rules(app.load_rules — robot.yaml 설계 규칙 E-78), client(OpenAI 클라이언트 — 시험 때 가짜, 비우면 처음 부를 때 만듦).
    바깥 영향: OpenAI 호출(돈 · 시간), check_design 요청(로봇은 안 움직임), store 에 기록 · 저장. 로봇을 움직이지 않는다.
    실패: GenError(code, message) — 부르는 쪽이 화면에 그대로 알린다.
    """

    def __init__(self, store, check, rules, client=None, clock=time.monotonic, roles_path=ROLES, prompts_dir=PROMPTS):
        """역할 목록을 읽고 프롬프트에 robot.yaml 숫자를 넣어 둔다(프롬프트 파일은 시작 때 한 번 읽음)."""
        self.store, self.check, self.clock = store, check, clock
        self._client = client
        roles = json.loads(Path(roles_path).read_text(encoding='utf-8'))
        self.role_names = sorted(roles['roles']) + sorted(f'{r}_{o}' for r in roles['roles'] for o in roles.get('options', {}))
        role_lines = [f'- {k}: {v}' for k, v in roles['roles'].items()]
        role_lines += [f'- (옵션) {k}: {v} — 역할 뒤에 붙임(예 LEG_{k})' for k, v in roles.get('options', {}).items()]
        L, W, T = (round(v * 1000) for v in rules['block_size_m'])
        self.design_prompt = string.Template((Path(prompts_dir) / 'design_system.txt').read_text(encoding='utf-8')).substitute(
            L=L, W=W, T=T, area_half=round(rules['assembly_area_half_m'] * 1000), margin=rules['margin_mm'],
            max_blocks=rules['max_blocks'], finger_t=round(rules['finger_thickness_m'] * 1000, 1),
            finger_w=round(rules['finger_width_m'] * 1000, 1), roles='\n'.join(role_lines))
        self.template_prompt = string.Template((Path(prompts_dir) / 'template_system.txt').read_text(encoding='utf-8'))

    @property
    def client(self):
        """OpenAI 클라이언트. 처음 부를 때 만든다 — 키(OPENAI_API_KEY 환경 변수)가 없어도 웹(화면 · 로봇 제어)은 켜져야 해서."""
        if self._client is None:
            try:
                from openai import OpenAI
                self._client = OpenAI(timeout=CALL_TIMEOUT_S, max_retries=CALL_RETRIES)
            except Exception as e:   # noqa: BLE001 — 키 없음 · 라이브러리 없음 모두 '생성 못 함'으로
                raise GenError('GEN_FAILED', f'AI 를 부를 수 없어요(OpenAI 준비 실패: {type(e).__name__}) — .env 키 확인') from e
        return self._client

    # ---------- Template 고르기(E-84 ③) ----------
    def classify(self, text):
        """요청 문장 → {'template': Template ID 또는 None, 'reason'}. None = 의자 · 책상 밖이거나 맞는 모양 없음(화면은 OUT_OF_SCOPE 안내).

        Template = 저장소에 V000 이 등록된 기본 설계. 화면이 이 결과를 사용자에게 한 번 확인받은 뒤 generate 를 부른다.
        """
        templates = self._templates()
        system = self.template_prompt.substitute(templates='\n'.join(f'- {t}: {h}' for t, h in templates.items()))
        fmt = self._format('template_choice', {
            'type': 'object', 'additionalProperties': False, 'required': ['template', 'reason'],
            'properties': {'template': {'type': 'string', 'enum': list(templates) + ['NONE']}, 'reason': {'type': 'string'}}})
        log = self._new_log('classify', text, system)
        messages = [{'role': 'system', 'content': system}, {'role': 'user', 'content': text}]
        try:
            data = self._parse(self._chat(log, MODEL_CLASSIFY, messages, fmt))
            log['result'] = data
        finally:
            self._save_log(log, messages)
        return {'template': None if data['template'] == 'NONE' else data['template'], 'reason': data['reason']}

    def _templates(self):
        """{Template ID: 설명} — 저장소에 V000 이 있는 기본 설계만(E-84 ② Template 은 기본 4개로 고정)."""
        out = {}
        for row in self.store.list_designs():
            if row['made_by'] == 'cad' and row['design_id'].endswith('_V000'):
                t = row['design_id'][:-len('_V000')]
                x, y, z = row['size_mm']
                out[t] = f'{TEMPLATE_HINTS.get(t, t)} (기본 설계 블록 {row["block_count"]}개, 외곽 {x:g}×{y:g}×{z:g} mm)'
        return out

    # ---------- 후보 3개 ----------
    def generate(self, text, template, on_progress=None):
        """요청 문장 · 확인된 Template → 후보 3개 + 검사 결과. 하나라도 합격이면 돌려준다(사람이 고름 → save).

        on_progress(stage, info): 화면 진행 표시용 — 'reading'(참고 설계 읽음) · 'candidates'(후보 3개 — 미리보기를 바로 그림) ·
        'checked'(후보 하나 검사 끝) · 'retry'(다 떨어져 다시). 출력: {text, template, family, design_id, parent_id, reference,
        candidates[{idea, blocks, check}], attempts, elapsed_s, log}. 실패: GenError.
        """
        progress = on_progress or (lambda *_: None)
        rows = self.store.list_designs(template=template)          # E-84 ③ — 이 Template 설계만 보여 주고 읽게 한다
        allowed = [r['design_id'] for r in rows]
        if f'{template}_V000' not in allowed:
            raise GenError('ERROR', f'모르는 Template: {template}')
        family = family_of(template)
        t0 = self.clock()
        log = self._new_log('generate', text, self.design_prompt, template=template)
        messages = [{'role': 'system', 'content': self.design_prompt},
                    {'role': 'user', 'content': self._request_text(text, template, rows)}]
        read = []
        try:
            try:
                msg = self._read_and_answer(log, messages, allowed, read, progress)
            except GenError:
                # 도구 단계가 실패하면 같은 Template 예시를 코드가 넣어 한 번 더(E-72 자동 전환)
                log['fallback'] = True
                read.clear()
                messages[1:] = [{'role': 'user', 'content': self._request_text(text, template, rows)
                                 + '\n\n' + self._examples_text(template)}]
                msg = self._chat(log, MODEL, messages, self._candidates_format())
            parent = read[0] if read else f'{template}_V000'    # 읽은 설계 = 부모(둘이면 처음 것), 자동 전환이면 기본 설계(E-72)
            design_id = None
            last = []
            for attempt in range(1, MAX_REGEN + 2):
                data = self._parse(msg)
                if data.get('out_of_scope') is True:
                    raise GenError('OUT_OF_SCOPE', data.get('reason') or '지금 만들 수 있는 모양이 아니에요')
                raw = data.get('candidates')
                if not isinstance(raw, list) or not all(isinstance(c, dict) and isinstance(c.get('blocks'), list) for c in raw):
                    raise GenError('GEN_FAILED', 'AI 답의 모양이 틀렸어요(candidates)')
                if design_id is None:
                    design_id, _ = self.store.new_design_id(template, parent)   # 검사 전에 — 변환기 ①이 이 ID 로 레시피를 만든다
                cands = [{'idea': str(c.get('idea') or ''), 'blocks': {'schema': SCHEMA_BLOCKS, 'design_id': design_id,
                                                                       'family': family, 'blocks': c['blocks']}}
                         for c in raw[:N_CANDIDATES]]
                progress('candidates', {'attempt': attempt, 'design_id': design_id, 'candidates': cands})
                for i, c in enumerate(cands):
                    c['check'] = self._check(c['blocks'])
                    progress('checked', {'attempt': attempt, 'index': i, 'check': c['check']})
                log['attempts'].append([{'idea': c['idea'], 'blocks': len(c['blocks']['blocks']),
                                         'check': {k: c['check'][k] for k in ('ok', 'reason', 'min_margin_mm', 'errors')}} for c in cands])
                if any(c['check']['ok'] for c in cands):
                    result = {'text': text, 'template': template, 'family': family, 'design_id': design_id, 'parent_id': parent,
                              'reference': list(read), 'candidates': cands, 'attempts': attempt,
                              'elapsed_s': round(self.clock() - t0, 1)}
                    log['result'] = {k: result[k] for k in ('design_id', 'parent_id', 'reference', 'attempts', 'elapsed_s')}
                    return result
                if cands and all(c['check']['service_failed'] for c in cands):
                    first = cands[0]['check']
                    raise GenError(first['reason'] or 'ERROR', '설계 검사를 못 했어요 — 로봇 PC · 다리 연결을 확인하세요',
                                   [{'block': None, 'reason': first['reason'], 'detail': first['message'] or ''}])
                last = [e for c in cands for e in c['check']['errors']]
                if attempt > MAX_REGEN:
                    break
                progress('retry', {'attempt': attempt + 1})
                messages += [{'role': 'assistant', 'content': msg.content}, {'role': 'user', 'content': self._feedback(cands)}]
                msg = self._chat(log, MODEL, messages, self._candidates_format(),
                                 **({'tools': [self._tool(allowed)], 'tool_choice': 'none'} if read else {}))
            raise GenError('GEN_FAILED', 'AI 후보가 검사를 통과하지 못했어요 — 기본 설계를 골라 보세요', last)
        except GenError as e:
            log['error'] = {'code': e.code, 'message': e.message}
            raise
        finally:
            log['elapsed_s'] = round(self.clock() - t0, 1)
            self._save_log(log, messages)

    def save(self, result, index, made_by='web'):
        """사람이 고른 후보 1개를 저장한다(IRD 8.3 ⑥). 반환: 저장한 설계 기록. 불합격 후보 · 없는 번호는 ValueError(저장 안 함).

        그 사이 다른 요청이 같은 ID 를 먼저 저장했으면 store 가 거절(ValueError) — 부르는 쪽은 처음부터 다시 만든다.
        """
        if not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < len(result['candidates']):
            raise ValueError(f'후보 번호가 틀렸다: {index!r}')
        cand = result['candidates'][index]
        chk = cand['check']
        if not chk['ok']:
            raise ValueError('검사에 떨어진 후보는 고를 수 없다')
        return self.store.save_design(cand['blocks'], chk['recipe'], chk['placements'], result['parent_id'], made_by,
                                      prompt=result['text'],
                                      check={'ok': True, 'min_margin_mm': chk['min_margin_mm'], 'errors': chk['errors']})

    # ---------- 대화 ----------
    def _read_and_answer(self, log, messages, allowed, read, progress):
        """참고 설계 읽기(도구) → 후보 답. 첫 번째는 도구를 꼭 부르게 하고(RAG — 예시 없이 쓰지 않게), 2번 읽으면 더는 못 부르게 한다.

        병렬 도구 호출은 끈다(구조화 출력과 같이 쓸 때 OpenAI 권고). 끝까지 답이 없으면 GenError.
        """
        tools = [self._tool(allowed)]
        fmt = self._candidates_format()
        for rnd in range(MAX_READS + 1):
            choice = 'required' if rnd == 0 else ('auto' if len(read) < MAX_READS else 'none')
            msg = self._chat(log, MODEL, messages, fmt, tools=tools, tool_choice=choice, parallel_tool_calls=False)
            calls = msg.tool_calls or []
            if not calls:
                return msg
            messages.append({'role': 'assistant', 'content': msg.content, 'tool_calls': [
                {'id': tc.id, 'type': 'function', 'function': {'name': tc.function.name, 'arguments': tc.function.arguments}}
                for tc in calls]})
            for tc in calls:
                messages.append({'role': 'tool', 'tool_call_id': tc.id, 'content': self._tool_answer(tc, allowed, read)})
                if read:
                    progress('reading', {'design_id': read[-1]})
        raise GenError('GEN_FAILED', 'AI 가 참고 설계만 읽고 후보를 쓰지 않았어요')

    def _tool_answer(self, tc, allowed, read):
        """도구 get_design 한 번 — 이 Template 설계의 블록 목록 + 최근 조립 한 줄(JSON 글자). 목록 밖 · 한도 넘음은 이유만 돌려준다."""
        try:
            design_id = json.loads(tc.function.arguments or '{}').get('design_id')
        except (ValueError, AttributeError):
            design_id = None
        if tc.function.name != 'get_design' or design_id not in allowed:
            return json.dumps({'error': '이 Template 목록에 없는 설계다'}, ensure_ascii=False)
        if len(read) >= MAX_READS:
            return json.dumps({'error': f'읽기는 {MAX_READS}번까지다 — 이제 후보를 써라'}, ensure_ascii=False)
        d = self.store.get_design(design_id)
        read.append(design_id)
        builds = self.store.builds_for(design_id)
        return json.dumps({'design_id': design_id, 'made_by': d.get('made_by'), 'parent_id': d.get('parent_id'),
                           'last_build': build_summary(builds[0]) if builds else None, 'blocks': d['blocks']['blocks']},
                          ensure_ascii=False, separators=(',', ':'))

    def _chat(self, log, model, messages, response_format, **kw):
        """GPT 를 한 번 부르고 메시지를 돌려준다(temperature 0). 응답 원문 · 걸린 시간 · 토큰 수를 log['calls'] 에 남긴다.

        실패(네트워크 · 시간 초과 · 키 · 거절)는 GenError(GEN_FAILED). 클라이언트가 재시도 1번을 한다(CALL_RETRIES).
        """
        t0 = self.clock()
        try:
            resp = self.client.chat.completions.create(model=model, temperature=0, messages=messages,
                                                       response_format=response_format, **kw)
        except GenError:
            raise
        except Exception as e:   # noqa: BLE001 — 어떤 실패든 화면에는 '생성 실패' 한 줄로
            log['calls'].append({'model': model, 'elapsed_s': round(self.clock() - t0, 2), 'error': f'{type(e).__name__}: {e}'})
            raise GenError('GEN_FAILED', f'AI 호출 실패({type(e).__name__}) — 잠시 뒤 다시') from e
        msg = resp.choices[0].message
        usage = getattr(resp, 'usage', None)
        log['calls'].append({'model': model, 'elapsed_s': round(self.clock() - t0, 2), 'content': msg.content,
                             'refusal': getattr(msg, 'refusal', None),
                             'tool_calls': [{'name': tc.function.name, 'arguments': tc.function.arguments} for tc in (msg.tool_calls or [])],
                             'tokens': getattr(usage, 'total_tokens', None)})
        return msg

    @staticmethod
    def _parse(msg):
        """구조화 출력 답 → dict. 거절 · 빈 답 · 깨진 JSON 은 GenError(GEN_FAILED) — 고쳐 읽지 않는다."""
        if getattr(msg, 'refusal', None):
            raise GenError('GEN_FAILED', f'AI 가 답을 거절했어요: {msg.refusal}')
        try:
            data = json.loads(msg.content or '')
        except ValueError as e:
            raise GenError('GEN_FAILED', 'AI 답을 읽지 못했어요(JSON 아님)') from e
        if not isinstance(data, dict):
            raise GenError('GEN_FAILED', 'AI 답이 객체가 아니에요')
        return data

    def _check(self, blocks):
        """check_design 한 번 → {ok, service_failed, reason, message, min_margin_mm, errors, recipe, placements}.

        ok = 서비스 성공 + 합격 + recipe · placements 둘 다(IRD 4.2). 서비스 실패(ERROR · TIMEOUT — 로봇 PC · 다리 · 변환기 고장)는
        AI 탓이 아니라 service_failed 로 나눠 다시 만들지 않는다. 부르는 함수가 예외를 내도 ERROR 로 바꾼다.
        """
        try:
            res = self.check(blocks)
        except Exception as e:   # noqa: BLE001 — 검사 경로 고장도 화면에 알릴 실패로
            res = {'success': False, 'reason': 'ERROR', 'message': f'{type(e).__name__}: {e}'}
        served = res.get('success') is True
        ok = served and res.get('ok') is True and isinstance(res.get('recipe'), dict) and isinstance(res.get('placements'), dict)
        return {'ok': ok, 'service_failed': not served, 'reason': res.get('reason') or '', 'message': res.get('message'),
                'min_margin_mm': res.get('min_margin_mm'), 'errors': res.get('errors') or [],
                'recipe': res.get('recipe') if ok else None, 'placements': res.get('placements') if ok else None}

    @staticmethod
    def _feedback(cands):
        """후보마다 검사 errors 를 한 줄씩 — 다음 답에서 고치게(SDD 6.6 ⑤). 너무 길면 앞 FEEDBACK_MAX 줄만."""
        lines = []
        for i, c in enumerate(cands, 1):
            for e in c['check']['errors']:
                where = f"{e['block']}번 블록 — " if e.get('block') is not None else ''
                lines.append(f"후보 {i}: {where}{e.get('detail') or e.get('reason')}")
        if not lines:
            lines = ['후보가 없거나 검사 결과를 받지 못했다']
        return ('[검사 결과] 후보가 모두 검사에서 떨어졌다. 아래를 고쳐 후보 3개를 처음부터 다시 써라(규칙 · 칸은 그대로).\n'
                + '\n'.join(lines[:FEEDBACK_MAX]))

    # ---------- 프롬프트 조각 ----------
    def _request_text(self, text, template, rows):
        """사용자 메시지: 요청 · 확인된 Template · 이 Template 설계 목록 요약(E-72 — 한 줄 = 설계 하나, 최근 조립 결과 포함)."""
        lines = []
        for r in rows:
            x, y, z = r['size_mm']
            b = r.get('last_build')
            build = (f"{b['result']} {b['placed']}/{b['total']}" + (f", 최대 오차 {b['max_err_mm']} mm" if b['max_err_mm'] is not None else '')
                     if b else '없음')
            lines.append(f"- {r['design_id']} | {MADE_KO.get(r['made_by'], r['made_by'])} | 부모 {r['parent_id'] or '없음'} | "
                         f"블록 {r['block_count']} | 외곽 {x:g}×{y:g}×{z:g} mm | 최근 조립: {build}")
        return (f'[요청] {text}\n[Template] {template} — {TEMPLATE_HINTS.get(template, template)} (사용자가 확인함)\n'
                f'[DB 목록 — 이 Template 의 설계]\n' + '\n'.join(lines))

    def _examples_text(self, template):
        """자동 전환 예시(E-72): 그 V000 + 최근 파생 3개의 블록 목록(도구 단계가 실패했을 때만)."""
        parts = ['[참고 설계 — 도구 대신 코드가 넣음]']
        for e in self.store.examples_for(template, n=3):
            parts.append(f"({e['design_id']}, 최근 조립 {e['last_build'] or '없음'})\n"
                         + json.dumps(e['blocks']['blocks'], ensure_ascii=False, separators=(',', ':')))
        return '\n'.join(parts)

    @staticmethod
    def _tool(allowed):
        """도구 get_design 정의 — design_id 는 이 Template 설계 목록 안에서만 고르게 enum 으로 묶는다(E-84 ③)."""
        return {'type': 'function', 'function': {
            'name': 'get_design', 'strict': True,
            'description': 'DB 목록의 설계 하나를 읽는다 — 블록 목록(blocks/2.0)과 최근 조립 결과. 참고할 설계를 1 ~ 2개 읽는다.',
            'parameters': {'type': 'object', 'additionalProperties': False, 'required': ['design_id'],
                           'properties': {'design_id': {'type': 'string', 'enum': list(allowed)}}}}}

    def _candidates_format(self):
        """후보 3개 구조화 출력 형식(JSON Schema strict). role 은 역할 목록 파일의 이름만(enum — 목록 밖 이름은 변환기 ①이 어차피 거절)."""
        block = {'type': 'object', 'additionalProperties': False,
                 'required': ['order', 'x', 'y', 'z', 'ori', 'role', 'part', 'stage', 'grasp'],
                 'properties': {'order': {'type': 'integer'}, 'x': {'type': 'number'}, 'y': {'type': 'number'},
                                'z': {'type': 'number'}, 'ori': {'type': 'string', 'enum': list(ORIS)},
                                'role': {'type': 'string', 'enum': self.role_names}, 'part': {'type': 'integer'},
                                'stage': {'type': 'integer'}, 'grasp': {'type': 'string', 'enum': list(GRASPS)}}}
        return self._format('design_candidates', {
            'type': 'object', 'additionalProperties': False, 'required': ['out_of_scope', 'reason', 'candidates'],
            'properties': {'out_of_scope': {'type': 'boolean'}, 'reason': {'type': 'string'},
                           'candidates': {'type': 'array', 'items': {
                               'type': 'object', 'additionalProperties': False, 'required': ['idea', 'blocks'],
                               'properties': {'idea': {'type': 'string'}, 'blocks': {'type': 'array', 'items': block}}}}}})

    @staticmethod
    def _format(name, schema):
        """OpenAI response_format(json_schema, strict)."""
        return {'type': 'json_schema', 'json_schema': {'name': name, 'strict': True, 'schema': schema}}

    # ---------- 기록(NFR-17) ----------
    def _new_log(self, kind, text, system, **kw):
        """기록 하나 시작 — 종류 · 요청 · 모델 · 시스템 프롬프트 sha256 앞 12자리(같은 프롬프트였는지 비교용)."""
        return {'kind': kind, 'text': text, 'started': round(time.time(), 3), 'model': MODEL if kind == 'generate' else MODEL_CLASSIFY,
                'prompt_sha': hashlib.sha256(system.encode('utf-8')).hexdigest()[:12], 'calls': [], 'attempts': [], **kw}

    def _save_log(self, log, messages):
        """보낸 메시지 전부(프롬프트 원문) + 응답 원문을 저장소에 남긴다. 기록 실패는 생성 결과를 막지 않는다(로그만)."""
        log['messages'] = messages
        try:
            log['file'] = self.store.save_gen_log(log)
        except Exception as e:   # noqa: BLE001 — 기록 때문에 사용자 요청이 실패하지 않게
            import logging
            logging.getLogger('web.gen').warning('[생성] 기록 저장 실패: %s', e)

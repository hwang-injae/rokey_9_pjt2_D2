# -*- coding: utf-8 -*-
"""DesignGenerator — AI 설계 생성에서 GPT 를 부르는 한 곳 (SDD 6.6 · IRD 8.3, W108). ROS 없음.

흐름(E-69 · E-67 · E-72 · E-84 · 10/13 PL 화면 결정):
  generate(text, base_id)  → ① 사람이 설계 목록에서 고른 설계(base_id = 부모)를 바탕으로 주고, 같은 Template 의 다른 설계는 목록 요약 +
                               도구 get_design(최대 2번 — 그 Template 설계만)으로 참고만 읽게 함(E-72). Template = base_id 앞부분
                               (AI 모양 추천 classify 는 없앰 — 사람이 설계를 고르니까, 10/13 PL)
                             ② 같은 대화에서 후보 3개(블록 목록)를 받음(구조화 출력 · temperature 0)
                             ③ 새 ID(store.new_design_id)를 붙여 check_design(쌓을 만한지는 검사 묶음 — 로봇 PC task)
                             ④ 통과한 후보만 남기고 모자란 개수만큼 떨어진 후보의 검사 결과를 알려 더 만듦(처음 + 2번, 10/11 PL N3 · A5)
                                → 끝까지 안 차면 통과한 것만(1 ~ 2개), 하나도 없으면 실패 종류(구성 형태 · 범위 · 그 밖)로 알림
  save(result, index)      → 사람이 고른 합격 후보 1개만 store.save_design. 안 고른 후보는 버린다(E-84 ⑥)
  save_scan(scan_result)   → 스캔 [저장](IRD 8.4 ⑦): scan_fill(GPT 채우기 — W177 민범진이 안을 채움) → 검사 → 저장(made_by scan)
AI 가 빠뜨리거나 틀린 칸을 코드가 대신 채우지 않는다(E-69) — 검사에서 떨어져 다시 만든다.
기록(NFR-17): 프롬프트 · 응답 원문 · 시간 · 검사 결과를 store.save_gen_log 로. OpenAI 키는 환경 변수에서만 읽고 어디에도 적지 않는다.
"""
import hashlib
import json
import re
import string
import time
from pathlib import Path

from d2_task.recipe_document import GRASPS
from design_store import AI_NONE, SCHEMA_PARTS, build_summary, family_of, split_design_id

MODEL = 'gpt-4o'                 # 설계 작성 · 스캔 채우기(SDD 6.6 · TR-12) — 팀 키 프로젝트는 gpt-4o · whisper-1 만 열려 있다(10/10 — mini 는 403)
CALL_TIMEOUT_S = 30              # 호출당 시간 제한(TR-12)
CALL_RETRIES = 1                 # 호출 실패 때 다시 1번(TR-12)
MAX_READS = 2                    # 도구 get_design 최대 횟수(E-72)
MAX_REGEN = 2                    # 통과 후보가 3개 안 될 때 모자란 만큼 더 만드는 횟수(처음 + 2번 — 10/11 PL N3 · A5)
SCAN_REFILLS = 2                 # 스캔 [저장]이 AI 칸 · Template 탓으로 떨어지면 GPT 가 다시 채우는 횟수(10/11 PL A3 — 레시피 초안 7-1)
N_CANDIDATES = 3                 # 후보 수(E-67)
FEEDBACK_MAX = 30                # 피드백 줄 수 상한 — 블록이 많이 틀려도 프롬프트가 지나치게 길어지지 않게
# 검사 errors[].detail 의 낱말 → 고치는 법(다시 만들 때 AI 가 같은 실수를 되풀이하지 않게 — 10/10 시험: '공중'만 알려 주니 3번 다 같은 실수)
FIX_HINTS = (
    ('공중', '아래에 받치는 블록이 없다 — 받침 블록 윗면 위(x · y 가 겹치게)로 옮기거나, 그 아래 받침(다리 · 기둥)을 그쪽으로 넓혀라'),
    ('여유', '무게중심이 받침 가장자리에 너무 가깝다 — 안쪽으로 옮기거나 받침을 그쪽으로 넓혀라(이어 붙인 다리 벽 등)'),
    ('겹침', '두 블록이 파고든다 — 하나를 옮겨라(면끼리 닿는 것은 괜찮다)'),
    ('grasp', '잡기를 괄호 안 \'가능\'한 것으로 바꿔라'),
    ('잡을 면 없음', '옆이 막혀 손가락이 못 들어간다 — 옆 블록과 띄우거나 놓는 순서(order)를 바꿔라'),
    ('작업영역 밖', '작업 영역을 넘는다 — 가운데 쪽으로 옮기거나 크기를 줄여라'),
    ('작업대 밖', '작업대를 넘는다 — 가운데 쪽으로 옮기거나 크기를 줄여라'),
    ('z<0', 'z 는 0 이상이다'),
    ('역할 목록', '역할은 [역할 목록]의 이름만 쓴다'),
    ('group', '같은 role · group 블록은 면으로 이어져야 한다 — 떨어진 덩어리는 group 번호를 나눠라'),
)
ORIS = ('x', 'y', 'xe', 'ye', 'zx', 'zy')   # IRD 2장 방향 코드
PROMPTS = Path(__file__).parent / 'prompts'
ROLES = Path(__file__).resolve().parents[2] / 'src' / 'd2_task' / 'd2_task' / 'roles.json'   # 변환기 ①과 같은 목록 파일
MADE_KO = {'cad': '도면', 'web': '웹 글', 'voice': '음성', 'scan': '스캔'}


class GenError(Exception):
    """생성이 끝까지 못 간 이유. code = OUT_OF_SCOPE · GEN_FAILED(IRD 7장) · ERROR · TIMEOUT(검사 서비스). 화면은 message 를 보여 준다."""

    def __init__(self, code, message, detail=None):
        """code · 사람에게 보일 한 줄 · detail(마지막 검사 errors 등 목록)."""
        super().__init__(message)
        self.code, self.message, self.detail = code, message, detail or []


def scan_actions(errors):
    """스캔 [저장]이 실패한 뒤 화면에 둘 버튼(레시피 초안 7-1 · 10/11 PL A2): 블록 수 초과(kind scope)가 있으면 [취소]만,
    그 밖(위치 · 다시 채워도 실패 · 호출 실패)은 [다시 스캔] · [취소]. 입력: 마지막 검사 errors. 출력: ['cancel'] 또는 ['rescan', 'cancel']."""
    return ['cancel'] if any(isinstance(e, dict) and e.get('kind') == 'scope' for e in errors or []) else ['rescan', 'cancel']


class DesignGenerator:
    """요청 문장 + 고른 설계 → 후보 3개 · 검사 · 저장, 스캔 [저장]. 상태 없음(요청마다 새 대화) — 후보는 부르는 쪽(작업 job)이 들고 있다.

    입력: store(DesignStore), check(parts/3.0 dict → check_design 응답 dict — MqttClient.request 로 만든 함수),
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
        self.extent = {'x': (L, W, T), 'y': (W, L, T), 'xe': (L, T, W), 'ye': (T, L, W), 'zx': (T, W, L), 'zy': (W, T, L)}   # IRD 2장 ori 표 — 피드백 설명용
        self.design_prompt = string.Template((Path(prompts_dir) / 'design_system.txt').read_text(encoding='utf-8')).substitute(
            L=L, W=W, T=T, area_half=round(rules['assembly_area_half_m'] * 1000), margin=rules['margin_mm'],
            edge=f"{L / 2 - rules['margin_mm']:g}", half_L=f'{L / 2:g}', two_L=2 * L,     # 받침 예시 숫자(다리 끝 안쪽 · 이어 붙이기)
            max_blocks=rules['max_blocks'], finger_t=round(rules['finger_thickness_m'] * 1000, 1),
            finger_w=round(rules['finger_width_m'] * 1000, 1), roles='\n'.join(role_lines))

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

    # ---------- Template 이름 · 스캔 AI 판단 기록 ----------
    def templates(self):
        """{Template ID: {name, description, label}} — 저장소에 V000 이 있는 기본 설계만(E-84 ② Template 은 기본 4개로 고정).

        name · description = V000 레시피의 template 칸(10/13 PL — 코드에 설명 표를 두지 않음). label = 'name — description
        (기본 설계 부품 n개, 외곽 …)' — 생성 요청 글의 [Template] 줄 · 화면 설계 이름(앞 ' — ' 앞이 이름)에 쓴다.
        """
        out = {}
        for t in self.store.templates():
            x, y, z = t['size_mm']
            head = ' — '.join(s for s in (t['name'], t['description']) if s)
            out[t['template_id']] = {'name': t['name'], 'description': t['description'],
                                     'label': f"{head} (기본 설계 부품 {t['block_count']}개, 외곽 {x:g}×{y:g}×{z:g} mm)"}
        return out

    @staticmethod
    def ai_judgment(choice=None, code=None):
        """스캔 부모 고르기 결과 → AI 판단 기록 {ai_template, ai_reason}(설계_DB 1절 — 스캔만 채움, 글 · 음성 설계는 비움 10/13 PL).

        choice = {'template', 'reason'}(GPT 가 고른 것), code = 고르기 실패 코드. 못 고름(NONE) · 실패면 ai_template 'NONE',
        까닭은 GPT 가 말한 것 또는 '실패: <코드>'.
        """
        if code is not None:
            return {'ai_template': AI_NONE, 'ai_reason': f'실패: {code}'}
        return {'ai_template': choice.get('template') or AI_NONE, 'ai_reason': choice.get('reason') or ''}

    # ---------- 후보 3개 ----------
    def generate(self, text, base_id, on_progress=None):
        """요청 문장 · 사람이 고른 설계(base_id) → 검사를 통과한 후보만(최대 3개). 사람이 그중 하나를 고른다(→ save).

        바탕(10/13 PL): 부모 = 고른 설계(GPT 가 읽은 설계가 아님), Template = base_id 앞부분. GPT 에는 고른 설계의 블록 목록을
        바탕으로 주고, 같은 Template 의 다른 설계는 목록 요약 + 도구 get_design 으로 참고만 읽게 한다(안 읽어도 됨 — E-72).

        후보 규칙(10/11 PL N3 · A5 — 레시피 초안 7-1): 3개 검사 → 통과한 후보만 남기고 모자란 개수만큼 AI 가 더 만든다(떨어진 후보의
        검사 결과를 알려 줌). 더 만들기는 처음 + MAX_REGEN 번까지(35 ~ 60초 걸려도 그대로, 먼저 보여 주기 없음 — 화면은 끝난 뒤 통과한
        후보만 그림). 끝까지 3개가 안 차면 통과한 1 ~ 2개만. REQUEST_INVALID(웹 쪽 잘못)는 더 만들지 않는다.
        on_progress(stage, info): 화면 진행 표시용 — 'reading'(참고 설계 읽음) · 'candidates'(이번에 검사할 후보 수 count — 그림은 안 줌) ·
        'checked'(후보 하나 검사 끝 — passed = 지금까지 통과 수) · 'retry'(모자라서 더 — need). 출력: {text, base_id, template, family,
        design_id, parent_id(= base_id), reference(GPT 가 읽은 설계), candidates[{idea, parts, check}](통과한 것만), attempts, elapsed_s,
        gen_log(이 생성의 AI 호출 기록 이름 — 고른 후보를 저장할 때 씀)}.
        실패: GenError — 하나도 못 통과하면 code = TEMPLATE_SHAPE('만들 수 없는 모양') · TEMPLATE_RANGE('지금 만들 수 있는 범위가 아님'
        + 범위) · GEN_FAILED(그 밖), 여러 실패가 같이면 구성 형태 먼저(PL 4-2-추가). REQUEST_INVALID · OUT_OF_SCOPE · 검사 서비스 실패 ·
        고른 설계가 없음(ERROR)도 GenError.
        """
        result = None
        progress = on_progress or (lambda *_: None)
        try:
            template = split_design_id(base_id)[0]
        except ValueError:
            raise GenError('ERROR', f'고른 설계 ID 가 틀렸어요: {base_id}') from None
        rows = self.store.list_designs(template=template)          # E-84 ③ — 이 Template 설계만 보여 주고 읽게 한다
        base_row = next((r for r in rows if r['design_id'] == base_id), None)
        if base_row is None:
            raise GenError('ERROR', f'고른 설계가 저장소에 없어요: {base_id}')
        others = [r for r in rows if r['design_id'] != base_id]
        allowed = [r['design_id'] for r in others]                 # 도구로 읽을 수 있는 것 = 같은 Template 의 다른 설계
        base_parts = self.store.get_design(base_id)['parts']['parts']
        family = family_of(template)
        parent = base_id
        t0 = self.clock()
        log = self._new_log('generate', text, self.design_prompt, template=template, base_id=base_id)
        request = self._request_text(text, template, base_row, base_parts, others)
        messages = [{'role': 'system', 'content': self.design_prompt}, {'role': 'user', 'content': request}]
        read = []
        try:
            if not allowed:                                         # 읽을 다른 설계가 없음(기본 설계뿐) — 도구 없이 바로 후보
                msg = self._chat(log, MODEL, messages, self._candidates_format())
            else:
                try:
                    msg = self._read_and_answer(log, messages, allowed, read, progress, first='auto')
                except GenError:
                    # 도구 단계가 실패하면 같은 Template 예시를 코드가 넣어 한 번 더(E-72 자동 전환)
                    log['fallback'] = True
                    read.clear()
                    messages[1:] = [{'role': 'user', 'content': request + '\n\n' + self._examples_text(template, base_id)}]
                    msg = self._chat(log, MODEL, messages, self._candidates_format())
            design_id = None
            passed, failed, need, attempt = [], [], N_CANDIDATES, 0
            for attempt in range(1, MAX_REGEN + 2):
                data = self._parse(msg)
                if data.get('out_of_scope') is True:
                    raise GenError('OUT_OF_SCOPE', data.get('reason') or '지금 만들 수 있는 모양이 아니에요')
                raw = data.get('candidates')
                if not isinstance(raw, list) or not all(isinstance(c, dict) and isinstance(c.get('parts'), list) for c in raw):
                    raise GenError('GEN_FAILED', 'AI 답의 모양이 틀렸어요(candidates)')
                if design_id is None:
                    design_id, _ = self.store.new_design_id(template, parent)   # 검사 전에 — 변환기 ①이 이 ID 로 레시피를 만든다
                cands = [{'idea': str(c.get('idea') or ''), 'parts': {'schema': SCHEMA_PARTS, 'design_id': design_id,
                                                                      'parts': c['parts']}}
                         for c in raw[:need]]
                progress('candidates', {'attempt': attempt, 'design_id': design_id, 'count': len(cands)})
                for i, c in enumerate(cands):
                    c['check'] = self._check(c['parts'])
                    progress('checked', {'attempt': attempt, 'index': i, 'check': c['check'],
                                         'passed': len(passed) + sum(x['check']['ok'] for x in cands[:i + 1])})
                log['attempts'].append([{'idea': c['idea'], 'block_count': len(c['parts']['parts']),
                                         'check': {k: c['check'][k] for k in ('ok', 'reason', 'min_margin_mm', 'errors')}} for c in cands])
                if cands and all(c['check']['service_failed'] for c in cands):
                    first = cands[0]['check']
                    raise GenError(first['reason'] or 'ERROR', '설계 검사를 못 했어요 — 로봇 PC · 다리 연결을 확인하세요',
                                   [{'part': None, 'reason': first['reason'], 'detail': first['message'] or ''}])
                invalid = [e for c in cands for e in c['check']['errors'] if e.get('reason') == 'REQUEST_INVALID']
                if invalid:                                  # 웹 쪽 잘못(base_recipe 등) — 다시 만들어도 같다(D10)
                    raise GenError('REQUEST_INVALID', '검사 요청 오류(웹) — 다시 만들지 않아요', invalid)
                passed += [c for c in cands if c['check']['ok']]
                failed = [c for c in cands if not c['check']['ok']]
                need = N_CANDIDATES - len(passed)
                if need <= 0 or attempt > MAX_REGEN:
                    break
                progress('retry', {'attempt': attempt + 1, 'need': need})
                messages += [{'role': 'assistant', 'content': msg.content},
                             {'role': 'user', 'content': self._feedback(failed, self.extent, need, len(passed))}]
                try:
                    msg = self._chat(log, MODEL, messages, self._candidates_format(),
                                     **({'tools': [self._tool(allowed)], 'tool_choice': 'none'} if read else {}))
                except GenError:
                    if passed:          # 더 만들기만 실패 — 이미 통과한 후보는 버리지 않고 보여 준다
                        break
                    raise
            if passed:
                result = {'text': text, 'base_id': base_id, 'template': template, 'family': family, 'design_id': design_id, 'parent_id': parent,
                          'reference': list(read), 'candidates': passed[:N_CANDIDATES], 'attempts': attempt,
                          'elapsed_s': round(self.clock() - t0, 1)}
                log['result'] = {k: result[k] for k in ('design_id', 'parent_id', 'reference', 'attempts', 'elapsed_s')}
                return result
            raise self._no_pass_error([e for c in failed for e in c['check']['errors']])
        except GenError as e:
            log['error'] = {'code': e.code, 'message': e.message}
            raise
        finally:
            log['elapsed_s'] = round(self.clock() - t0, 1)
            self._save_log(log, messages)
            if result is not None:
                result['gen_log'] = log.get('file')    # 기록을 남긴 뒤에야 이름을 안다 — 돌려주기 전에 붙인다

    def save(self, result, index, made_by='web'):
        """사람이 고른 후보 1개를 저장한다(IRD 8.3 ⑥). 반환: 저장한 설계 기록. 불합격 후보 · 없는 번호는 ValueError(저장 안 함).

        부모 = 사람이 고른 설계(result['parent_id']). AI 판단 기록 3칸은 비운다(글 · 음성은 사람이 설계를 고름 — 10/13 PL, 스캔만 채움).
        저장하면 이 생성의 AI 호출 기록에 design_id 를 적는다(설계_DB #14). 그 사이 다른 요청이 같은 ID 를 먼저 저장했으면 store 가
        거절(ValueError) — 부르는 쪽은 처음부터 다시 만든다.
        """
        if not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < len(result['candidates']):
            raise ValueError(f'후보 번호가 틀렸다: {index!r}')
        cand = result['candidates'][index]
        chk = cand['check']
        if not chk['ok']:
            raise ValueError('검사에 떨어진 후보는 고를 수 없다')
        return self.store.save_design(cand['parts'], chk['recipe'], chk['placements'], result['parent_id'], made_by,
                                      prompt=result['text'],
                                      check={'ok': True, 'min_margin_mm': chk['min_margin_mm'], 'errors': chk['errors']},
                                      gen_log=result.get('gen_log'))

    # ---------- 스캔 [저장](IRD 8.4 ⑦) ----------
    def scan_fill(self, scan_result, feedback=None):
        """스캔 추론 결과에 GPT 가 AI 칸을 채우고 부모를 고른다 — W177(민범진, 10/13 PL). GPT 대화 한 번(도구 읽기 포함).

        입력: scan_result(scan_result/2.0 — blocks = blocks/2 {blocks[{scan_num, x, y, z, ori, inferred?}]} mm 설계 좌표 ·
              nearest_base(선택 — 추론기가 본 가장 가까운 기본 설계)), feedback(다시 채울 때만, 처음엔 None —
              {'errors': 앞 시도의 검사 errors[{part, reason, detail, kind}], 'parts': 앞 시도에서 이 함수가 돌려준 parts 그대로}.
              errors[].part · detail 의 'N번' = 그 parts 의 order(위치 · 방향을 바꾼 줄은 part null). 10/13 PL — 오류가 어느 부품인지
              GPT 가 알게 같이 넘긴다. 다시 채울 때 위치 · 방향은 그대로 두고 AI 칸만 고친다).
        출력: {'parts': [{order, x, y, z, ori, role, group, stage, grasp, inferred?}] — x · y · z · ori 는 스캔 그대로(바꾸면 save_scan 이
                         검사 전에 떨어뜨림), order = 쌓는 순서(스캔 번호 scan_num 아님) · role · group · stage · grasp = 생성과 같은 규칙,
               'parent_id': 최종 부모(DB 에 있는 설계 ID — 새 설계 ID 의 Template 이 이것으로 정해짐),
               'ai_template': GPT 가 고른 부모의 Template ID(못 고르면 'NONE'), 'ai_reason': 고른 까닭 한 줄(실패면 '실패: <코드>'),
               'gen_log': 이 호출의 AI 기록 이름(_save_log 가 남긴 log['file'] — 저장하면 그 기록에 design_id 를 적음, 설계_DB #14)}.
              부모(E-79): AI 생성과 같은 방식(E-72)으로 DB 목록 요약을 주고 GPT 가 도구 get_design 으로 고름 — DB 전체 후보(기본 · 파생 ·
              예전 스캔) · 같은 family 만. 다르거나 GPT 가 못 고르면 코드가 바꾼다(같은 family 기본 설계 · nearest_base — 사람 고르기
              화면 없음, 10/13 PL) — 그러면 최종 parent_id 의 Template 과 ai_template 이 달라 template_changed 가 true 가 된다(설계_DB 1절).
              부모 고르기만 실패하고 AI 칸 채우기가 됐으면 ai_reason = '실패: <코드>' 로 적고 parts 를 돌려준다(저장은 계속 — 생성의
              ai_judgment 와 같은 규칙). AI 칸 채우기 자체가 실패할 때만 GenError.
        구현: GPT 는 scan_num 마다 order · role · group · stage · grasp 만 답하고 x · y · z · ori · inferred 는 코드가 스캔에서 옮긴다
              (모양이 바뀔 수 없음). 대신할 부모 = nearest_base, 없으면 같은 family(nearest_base 의 family, 없으면 blocks.family) 기본 설계 중
              부품 수가 가장 가까운 것. '부모 고르기 실패' = 도구 단계 실패 — 그때는 대신할 부모의 부품 목록을 코드가 넣고 칸만 채우게 한 번 더.
              GPT 가 틀린 role · stage · grasp 를 코드가 고치지 않는다(E-69) — 검사에서 떨어져 feedback 으로 다시 온다.
        바깥 영향: OpenAI 호출(gpt-4o, temperature 0, 구조화 출력) · AI 호출 기록(kind scan_fill — 설계_DB 3절). 로봇은 움직이지 않는다.
        실패: GenError — GEN_FAILED(호출 실패 · 답의 scan_num 이 스캔과 다름) · ERROR(스캔 모양이 틀림 · 부모 후보 기본 설계 없음).
        """
        body = scan_result.get('blocks') if isinstance(scan_result, dict) else None
        scan = body.get('blocks') if isinstance(body, dict) else None
        if not isinstance(scan, list) or not scan or not all(
                isinstance(b, dict) and all(k in b for k in ('scan_num', 'x', 'y', 'z', 'ori')) for b in scan):
            raise GenError('ERROR', '스캔 결과 모양이 틀렸어요(blocks/2 — blocks[scan_num · x · y · z · ori])')
        by_num = {b['scan_num']: b for b in scan}
        if len(by_num) != len(scan):
            raise GenError('ERROR', '스캔 블록 번호(scan_num)가 겹쳐요')
        rows = self.store.list_designs()
        allowed = [r['design_id'] for r in rows]
        family_by = {r['design_id']: r['family'] for r in rows}
        nearest = scan_result.get('nearest_base') if scan_result.get('nearest_base') in family_by else None
        family = family_by[nearest] if nearest else (body.get('family') if body.get('family') in ('chair', 'desk') else None)
        bases = [r for r in rows if r['made_by'] == 'cad' and family in (None, r['family'])]
        fallback = nearest or (min(bases, key=lambda r: abs(r['block_count'] - len(scan)))['design_id'] if bases else None)
        if fallback is None:
            raise GenError('ERROR', '부모로 삼을 기본 설계가 저장소에 없어요')

        # 규칙 · 칸 설명(블록 · 방향 · 받침 · role · grasp)은 생성 프롬프트를 그대로 쓰고, 할 일만 스캔 채우기로 바꾼다
        system = self.design_prompt + (
            '\n\n[이번 일 — 스캔 채우기. 위 [참고 설계 읽기] · [출력] 대신 이것을 따른다]\n'
            '사람이 쌓은 가구를 스캔해 블록마다 위치(x · y · z)와 방향(ori)을 얻었다. 모양은 정해졌다 — 블록을 더하거나 빼거나 옮기지 않는다.\n'
            '1. [DB 목록](한 줄 = 설계 하나 — 기본 · 파생 · 예전 스캔)을 보고 도구 get_design 으로 스캔과 가장 비슷한 설계를 1 ~ 2개 읽는다.\n'
            '   가장 비슷한 설계 = 부모(parent_id): 블록 수 · 외곽 크기 · 층 구성이 가깝고 같은 가구 종류(의자 · 책상)인 것. 맞는 것이 없으면 NONE.\n'
            '2. 스캔 블록마다(scan_num) role · group · order · stage · grasp 를 위 규칙대로 쓴다. 읽은 설계에서 같은 자리 블록의 칸을 참고한다.\n'
            '   order 는 scan_num 과 상관없이 실제로 쌓는 순서다(1부터, 겹치지 않게, 받치는 블록 먼저).\n'
            '   inferred = true 는 가려져서 추정한 블록이다 — 다른 블록과 똑같이 채운다.\n'
            '3. parts 에는 스캔 블록 하나마다 정확히 한 줄(scan_num 이 빠지거나 겹치면 안 된다).\n'
            '4. reason: 그 부모를 고른 까닭 한국어 한 줄. plan: parts 를 쓰기 전에 층마다 한 줄 — 그 층 z, scan_num, role · stage.')
        lines = []
        for r in rows:
            x, y, z = r['size_mm']
            b = r.get('last_build')
            build = f"{b['result']} {b['placed']}/{b['total']}" if b else '없음'
            lines.append(f"- {r['design_id']} | {MADE_KO.get(r['made_by'], r['made_by'])} | 부모 {r['parent_id'] or '없음'} | "
                         f"블록 {r['block_count']} | 외곽 {x:g}×{y:g}×{z:g} mm | 최근 조립: {build}")
        shown = [{k: b[k] for k in ('scan_num', 'x', 'y', 'z', 'ori', 'inferred') if k in b}
                 for b in sorted(scan, key=lambda b: (b['z'], b['y'], b['x']))]    # 아래층부터 — 층 구성을 읽기 쉽게
        text = (f"[스캔] 블록 {len(scan)}개" + (f" · 추론기가 고른 가까운 기본 설계 {nearest}" if nearest else '') + '\n'
                + json.dumps(shown, ensure_ascii=False, separators=(',', ':')) + '\n[DB 목록 — 전체 설계]\n' + '\n'.join(lines))
        if feedback:
            # 앞 답은 이 대화에 없다(요청마다 새 대화) — 앞 parts 를 scan_num 과 함께 보여 줘 오류의 'N번'(= 앞 order)이 어느 블록인지 알게
            fb = feedback if isinstance(feedback, dict) else {'errors': feedback}

            def shape(b):                                      # save_scan._moved 와 같은 비교 키(0.1 mm · ori)
                return round(float(b['x']), 1), round(float(b['y']), 1), round(float(b['z']), 1), str(b['ori'])
            num_of = {shape(b): b['scan_num'] for b in scan}
            prev = [{'scan_num': num_of.get(shape(p)), **{k: p.get(k) for k in ('order', 'role', 'group', 'stage', 'grasp')}}
                    for p in fb.get('parts') or [] if isinstance(p, dict) and all(k in p for k in ('x', 'y', 'z', 'ori'))]
            fix = []
            for e in (fb.get('errors') or [])[:FEEDBACK_MAX]:
                detail = str(e.get('detail') or e.get('reason')) if isinstance(e, dict) else str(e)
                hint = next((h for key, h in FIX_HINTS if key in detail), '')
                fix.append(f'- {detail}' + (f' → 고칠 것: {hint}' if hint else ''))
            text += ('\n\n[앞 검사 결과] 앞서 채운 칸이 검사에서 떨어졌다. 오류의 N번 = 아래 [앞 답]의 order. 위치 · 방향은 그대로 두고 '
                     'role · group · order · stage · grasp 를 고쳐 다시 채워라.\n[앞 답]\n'
                     + json.dumps(prev, ensure_ascii=False, separators=(',', ':')) + '\n[오류]\n' + '\n'.join(fix))
        part = {'type': 'object', 'additionalProperties': False,
                'required': ['scan_num', 'order', 'role', 'group', 'stage', 'grasp'],
                'properties': {'scan_num': {'type': 'integer'}, 'order': {'type': 'integer'},
                               'role': {'type': 'string', 'enum': self.role_names}, 'group': {'type': 'integer'},
                               'stage': {'type': 'integer'}, 'grasp': {'type': 'string', 'enum': list(GRASPS)}}}
        fmt = self._format('scan_fill', {
            'type': 'object', 'additionalProperties': False, 'required': ['parent_id', 'reason', 'plan', 'parts'],
            'properties': {'parent_id': {'type': 'string', 'enum': allowed + [AI_NONE]}, 'reason': {'type': 'string'},
                           'plan': {'type': 'string'}, 'parts': {'type': 'array', 'items': part}}})

        t0 = self.clock()
        log = self._new_log('scan_fill', f'스캔 {scan_result.get("run_id")} 블록 {len(scan)}개', system,
                            run_id=scan_result.get('run_id'), nearest_base=nearest, feedback=feedback or None)
        messages = [{'role': 'system', 'content': system}, {'role': 'user', 'content': text}]
        read = []
        fail_code = None                                       # 부모 고르기(도구 단계) 실패 코드
        try:
            try:
                msg = self._read_and_answer(log, messages, allowed, read, lambda *_: None, fmt)
            except GenError as e:
                # 도구 단계가 실패하면 대신할 부모의 블록 목록을 코드가 넣어 칸만 채우게 한 번 더(E-72 자동 전환과 같은 방식)
                fail_code = e.code
                log['fallback'] = True
                read.clear()
                messages[1:] = [{'role': 'user', 'content': text + f'\n\n[참고 설계 — 도구 대신 코드가 넣음] ({fallback})\n'
                                 + json.dumps(self.store.get_design(fallback)['parts']['parts'], ensure_ascii=False,
                                              separators=(',', ':'))}]
                msg = self._chat(log, MODEL, messages, fmt)
            data = self._parse(msg)
            got = data.get('parts')
            if not isinstance(got, list) or not all(isinstance(p, dict) for p in got) or \
                    sorted(str(p.get('scan_num')) for p in got) != sorted(map(str, by_num)):
                raise GenError('GEN_FAILED', 'AI 답의 블록이 스캔과 맞지 않아요(scan_num 빠짐 · 겹침)')
            by_text = {str(k): b for k, b in by_num.items()}      # 비교와 같은 글자 키 — 숫자 · 글자 scan_num 이 섞여도 같은 블록
            parts = []
            for p in sorted(got, key=lambda p: p['order']):
                b = by_text[str(p['scan_num'])]
                parts.append({'order': p['order'], 'x': b['x'], 'y': b['y'], 'z': b['z'], 'ori': b['ori'],
                              'role': p['role'], 'group': p['group'], 'stage': p['stage'], 'grasp': p['grasp'],
                              **({'inferred': bool(b['inferred'])} if 'inferred' in b else {})})
            choice = data.get('parent_id')
            if fail_code is not None:                          # 부모 고르기만 실패 — 칸은 채웠으니 부모는 코드가, 저장은 계속(ai_judgment 규칙)
                parent, judged = fallback, self.ai_judgment(code=fail_code)
            else:
                try:
                    ai_template = split_design_id(choice)[0] if choice in family_by else AI_NONE
                except ValueError:                             # E-84 꼴이 아닌 옛 이름 설계 — Template 을 모름
                    ai_template = AI_NONE
                parent = choice if choice in family_by and family in (None, family_by[choice]) else fallback
                judged = {'ai_template': ai_template, 'ai_reason': str(data.get('reason') or '').strip()}
            result = {'parts': parts, 'parent_id': parent, **judged}
            log['result'] = {'parent_id': parent, 'ai_choice': choice, **judged, 'reference': list(read)}
        except GenError as e:
            log['error'] = {'code': e.code, 'message': e.message}
            raise
        finally:
            log['elapsed_s'] = round(self.clock() - t0, 1)
            self._save_log(log, messages)
        result['gen_log'] = log.get('file')
        return result

    def save_scan(self, scan_result, on_progress=None):
        """스캔 [저장] 한 번(IRD 8.4 ⑦): scan_fill → 새 ID(부모 Template 의 다음 V) → check_design(base_recipe 실음) → 합격이면 저장.

        떨어지면 kind 로 나눈다(레시피 초안 7-1 · 10/11 PL A2 · A3): scope(블록 수 초과) · position(놓인 자리 — GPT 는 위치를 안 바꿈,
        E-79)이 있으면 바로 끝 · ai_fields · template 만이면 {errors, 앞 parts} 를 피드백으로 GPT 다시 채우기(최대 SCAN_REFILLS 번).
        REQUEST_INVALID(웹 쪽 잘못) · 검사 서비스 실패(ERROR · TIMEOUT)는 다시 채우지 않는다.
        on_progress(stage, info): 'filled'(채움 끝 — attempt · design_id · parent_id) · 'checked'(검사 끝 — check) · 'retry'.
        출력: 저장한 설계 기록(design/3.0, made_by scan + AI 판단 기록). 저장 뒤 hmi/command select_design 은 화면이 보낸다(IRD 8.4 ⑧).
        실패: GenError(code, message, detail = 마지막 errors) — 화면 버튼은 scan_actions(detail).
        바깥 영향: OpenAI 호출(scan_fill) · check_design 요청 · 저장. 로봇은 움직이지 않는다.
        """
        progress = on_progress or (lambda *_: None)
        scan = scan_result.get('blocks') if isinstance(scan_result, dict) else None
        if not isinstance(scan, dict) or not isinstance(scan.get('blocks'), list) or not scan['blocks']:
            raise GenError('ERROR', '저장할 스캔 결과가 없어요')
        feedback, errors = None, []
        for attempt in range(1, SCAN_REFILLS + 2):
            fill = self.scan_fill(scan_result, feedback)
            parent = fill.get('parent_id')
            try:
                design_id, _ = self.store.new_design_id(split_design_id(parent)[0], parent)
            except (KeyError, ValueError) as e:
                raise GenError('GEN_FAILED', f'AI 가 고른 부모 설계를 쓸 수 없어요({parent})') from e
            parts = {'schema': SCHEMA_PARTS, 'design_id': design_id, 'parts': fill.get('parts')}
            progress('filled', {'attempt': attempt, 'design_id': design_id, 'parent_id': parent})
            moved = self._moved(scan['blocks'], fill.get('parts'))
            chk = ({'ok': False, 'service_failed': False, 'reason': '', 'min_margin_mm': None, 'errors': [moved]}
                   if moved else self._check(parts))
            progress('checked', {'attempt': attempt, 'check': {k: chk[k] for k in ('ok', 'reason', 'min_margin_mm', 'errors')}})
            if chk['ok']:
                return self.store.save_design(parts, chk['recipe'], chk['placements'], parent, 'scan',
                                              check={'ok': True, 'min_margin_mm': chk['min_margin_mm'], 'errors': chk['errors']},
                                              ai_template=fill.get('ai_template'), ai_reason=fill.get('ai_reason'),
                                              gen_log=fill.get('gen_log'))
            errors = chk['errors']
            if chk['service_failed']:
                raise GenError(chk['reason'] or 'ERROR', '설계 검사를 못 했어요 — 로봇 PC · 다리 연결을 확인하세요',
                               [{'part': None, 'reason': chk['reason'], 'detail': chk['message'] or ''}])
            if any(e.get('reason') == 'REQUEST_INVALID' for e in errors):
                raise GenError('REQUEST_INVALID', '검사 요청 오류(웹) — 다시 채우지 않아요', errors)
            kinds = {e.get('kind') for e in errors}
            if not errors or not kinds <= {'ai_fields', 'template'} or attempt > SCAN_REFILLS:
                break
            progress('retry', {'attempt': attempt + 1})
            feedback = {'errors': errors, 'parts': fill.get('parts')}     # 오류의 'N번'(order)이 어느 부품인지 GPT 가 알게(10/13 PL)
        scope = any(e.get('kind') == 'scope' for e in errors)
        raise GenError('OUT_OF_SCOPE' if scope else 'CHECK_FAILED',
                       '블록이 너무 많아 저장할 수 없어요' if scope else '스캔한 설계가 검사를 통과하지 못했어요', errors)

    @staticmethod
    def _moved(scan_blocks, parts):
        """AI 가 스캔한 위치 · 방향 · 개수를 바꿨으면 검사 errors 한 줄(kind ai_fields — 다시 채움), 그대로면 None.
        E-79: 모양(blocks/2 위치 · 방향)은 바꾸지 않고 AI 칸만 채운다. 비교는 x · y · z(0.1 mm) · ori 묶음 — 순서는 상관없음."""
        def key(b):
            return round(float(b['x']), 1), round(float(b['y']), 1), round(float(b['z']), 1), str(b['ori'])
        try:
            same = sorted(map(key, scan_blocks)) == sorted(map(key, parts))
        except (KeyError, TypeError, ValueError):
            same = False
        return None if same else {'part': None, 'reason': 'CHECK_FAILED', 'kind': 'ai_fields',
                                  'detail': 'AI 가 스캔한 부품의 위치 · 방향 · 개수를 바꿨다 — 그대로 두고 역할 · 그룹 · 순서 · 단계 · 잡기만 채워라'}

    # ---------- 대화 ----------
    def _read_and_answer(self, log, messages, allowed, read, progress, fmt=None, first='required'):
        """참고 설계 읽기(도구) → 후보 답. 2번 읽으면 더는 못 부르게 한다. 병렬 도구 호출은 끈다(구조화 출력과 같이 쓸 때 OpenAI 권고).

        first = 첫 번째 도구 선택 — 'required'(꼭 읽게: scan_fill 부모 고르기) · 'auto'(generate — 바탕 설계는 이미 줬으니 참고는 골라서).
        fmt = 답 형식(비우면 후보 3개 — generate). scan_fill 은 스캔 채우기 형식을 넘긴다. 끝까지 답이 없으면 GenError.
        """
        tools = [self._tool(allowed)]
        fmt = fmt or self._candidates_format()
        for rnd in range(MAX_READS + 1):
            choice = first if rnd == 0 else ('auto' if len(read) < MAX_READS else 'none')
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
                           'last_build': build_summary(builds[0]) if builds else None, 'parts': d['parts']['parts']},
                          ensure_ascii=False, separators=(',', ':'))

    def _chat(self, log, model, messages, response_format, **kw):
        """GPT 를 한 번 부르고 메시지를 돌려준다(temperature 0). 응답 원문 · 걸린 시간 · 토큰 수를 log['calls'] 에 남긴다.

        실패(네트워크 · 시간 초과 · 키 · 거절)는 GenError(GEN_FAILED). 클라이언트가 재시도 1번을 한다(CALL_RETRIES).
        응답 모양이 다르면(choices 없음 · 메시지가 글자뿐 등 — 10/13 민범진 보고 AttributeError) 그것도 GEN_FAILED 로(화면이 '오류'에 머물지 않게).
        """
        t0 = self.clock()
        try:
            resp = self.client.chat.completions.create(model=model, temperature=0, messages=messages,
                                                       response_format=response_format, **kw)
        except GenError:
            raise
        except Exception as e:   # noqa: BLE001 — 어떤 실패든 화면에는 '생성 실패' 한 줄로
            log['calls'].append({'model': model, 'elapsed_s': round(self.clock() - t0, 2), 'error': f'{type(e).__name__}: {e}'})
            if 'insufficient_quota' in str(e) or 'no credits' in str(e):   # 429 중 '크레딧 없음'은 기다려도 안 풀린다 — 화면에 그대로
                raise GenError('GEN_FAILED', 'OpenAI 크레딧이 없어요 — 키 담당(PL)이 크레딧을 채워야 해요') from e
            raise GenError('GEN_FAILED', f'AI 호출 실패({type(e).__name__}) — 잠시 뒤 다시') from e
        try:
            msg = resp.choices[0].message
            call = {'model': model, 'elapsed_s': round(self.clock() - t0, 2), 'content': msg.content,
                    'refusal': getattr(msg, 'refusal', None),
                    'tool_calls': [{'name': tc.function.name, 'arguments': tc.function.arguments} for tc in (msg.tool_calls or [])],
                    'tokens': getattr(getattr(resp, 'usage', None), 'total_tokens', None)}
        except (AttributeError, IndexError, KeyError, TypeError) as e:
            log['calls'].append({'model': model, 'elapsed_s': round(self.clock() - t0, 2), 'error': f'{type(e).__name__}: {e}',
                                 'raw': repr(resp)[:2000]})
            raise GenError('GEN_FAILED', 'AI 답의 모양이 틀렸어요(응답) — 잠시 뒤 다시') from e
        log['calls'].append(call)
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

    def _check(self, parts):
        """check_design 한 번 → {ok, service_failed, reason, message, min_margin_mm, errors, recipe, placements}.

        요청 = parts/3.0 + base_recipe(설계 ID 앞부분 Template 의 V000 레시피 통째 — 10/13 PL, IRD 4.2). parts 자체에는 붙이지 않는다
        (저장하는 설계에 섞이지 않게). ok = 서비스 성공 + 합격 + recipe · placements 둘 다(IRD 4.2). 서비스 실패(ERROR · TIMEOUT —
        로봇 PC · 다리 · 변환기 고장)는 AI 탓이 아니라 service_failed 로 나눠 다시 만들지 않는다. 부르는 함수가 예외를 내도 ERROR 로 바꾼다.
        """
        try:
            template = split_design_id(parts['design_id'])[0]
            res = self.check({**parts, 'base_recipe': self.store.base_recipe(template)})
        except Exception as e:   # noqa: BLE001 — 검사 경로 고장도 화면에 알릴 실패로
            res = {'success': False, 'reason': 'ERROR', 'message': f'{type(e).__name__}: {e}'}
        served = res.get('success') is True
        ok = served and res.get('ok') is True and isinstance(res.get('recipe'), dict) and isinstance(res.get('placements'), dict)
        return {'ok': ok, 'service_failed': not served, 'reason': res.get('reason') or '', 'message': res.get('message'),
                'min_margin_mm': res.get('min_margin_mm'), 'errors': res.get('errors') or [],
                'recipe': res.get('recipe') if ok else None, 'placements': res.get('placements') if ok else None}

    @staticmethod
    def _no_pass_error(errors):
        """후보가 하나도 통과 못 했을 때 화면에 알릴 실패(레시피 초안 7-1 · 10/11 PL C5 · C6). 여러 실패가 같이면 구성 형태 먼저(PL 4-2-추가).

        TEMPLATE_SHAPE = '만들 수 없는 모양'(취소하고 다른 요청으로) · TEMPLATE_RANGE = '지금 만들 수 있는 범위가 아님' + 21번 detail(범위 글)
        · 그 밖 = GEN_FAILED '통과하지 못했어요'. 입력: 마지막 시도의 떨어진 후보 errors. 출력: GenError(detail = 그 errors).
        """
        reasons = {e.get('reason') for e in errors}
        if 'TEMPLATE_SHAPE' in reasons:
            return GenError('TEMPLATE_SHAPE', '만들 수 없는 모양이에요 — 취소하고 다시 요청해 주세요', errors)
        if 'TEMPLATE_RANGE' in reasons:     # detail 앞 '21번 범위:' 같은 검사 번호는 떼고 사람이 읽는 범위 글만(화면 규칙 10/10)
            ranges = list(dict.fromkeys(re.sub(r'^21번[^:]*:\s*', '', str(e.get('detail') or ''))
                                        for e in errors if e.get('reason') == 'TEMPLATE_RANGE'))
            return GenError('TEMPLATE_RANGE', '지금 만들 수 있는 범위가 아니에요 — ' + ' / '.join(r for r in ranges[:3] if r), errors)
        return GenError('GEN_FAILED', 'AI 후보가 검사를 통과하지 못했어요 — 다르게 요청하거나 기본 설계를 골라 보세요', errors)

    @staticmethod
    def _feedback(cands, extent, need=N_CANDIDATES, n_passed=0):
        """떨어진 후보마다 검사 errors 를 묶고 오류마다 고치는 법(FIX_HINTS) · 그 블록 위치를 붙인다 — 다음 답에서 고치게(SDD 6.6 ⑤).

        need = 더 만들 후보 수(통과한 n_passed 개는 남겨 두고 모자란 만큼만 — 10/11 PL N3). 위치 설명은 판정이 아니다(합격 · 불합격은
        검사 묶음이 정함) — AI 가 어느 층이 비었는지 알게 할 뿐(10/10 시험: 다리를 2층만 쌓고 좌판을 4층 위에). 너무 길면 앞 FEEDBACK_MAX 줄만.
        """
        lines = []
        for i, c in enumerate(cands, 1):
            errs = c['check']['errors']
            if not errs:
                continue
            lines.append(f"후보 {i}({c['idea']}):")
            by_order = {b.get('order'): b for b in c['parts']['parts'] if isinstance(b, dict)}
            for e in errs:
                detail = str(e.get('detail') or e.get('reason'))
                hint = next((h for key, h in FIX_HINTS if key in detail), '')
                where = DesignGenerator._where(detail, by_order, extent)
                lines.append(f'  - {detail}{where}' + (f' → 고칠 것: {hint}' if hint else ''))
        if not lines:
            lines = ['후보가 없거나 검사 결과를 받지 못했다']
        head = (f'[검사 결과] 후보 {n_passed}개는 통과해 남겨 둔다. 아래 떨어진 후보의 문제를 고쳐 새 후보 {need}개만 써라'
                if n_passed else f'[검사 결과] 후보가 모두 검사에서 떨어졌다. 아래 문제를 고쳐 후보 {need}개를 다시 써라')
        return (head + ' — 생각(idea)은 살리되, 받침이 모자라면 받침(다리 · 기둥)을 넓힌다. 규칙 · 칸은 그대로.\n'
                + '\n'.join(lines[:FEEDBACK_MAX]))

    @staticmethod
    def _where(detail, by_order, extent):
        """오류 글에 나온 블록의 위치 설명. '공중'이면 그 x · y 아래 블록 윗면의 가장 높은 z 도(어느 층이 비었는지). 모르면 ''."""
        nums = [int(n) for n in re.findall(r'(\d+)번', detail)]
        if not nums or nums[0] not in by_order:
            return ''
        found = []
        for n in nums[:2]:
            b = by_order.get(n)
            if b and b.get('ori') in extent:
                found.append(f"{n}번 ({b.get('x')}, {b.get('y')}, z {b.get('z')}, {b.get('ori')})")
        text = ' [' + ' · '.join(found) + ']' if found else ''
        if '공중' in detail:
            a = by_order[nums[0]]
            try:
                ax, ay, _ = extent[a['ori']]
                tops = []
                for b in by_order.values():
                    bx, by, bz = extent[b['ori']]
                    if b is not a and abs(b['x'] - a['x']) < (ax + bx) / 2 and abs(b['y'] - a['y']) < (ay + by) / 2 and b['z'] + bz <= a['z']:
                        tops.append(b['z'] + bz)
                text += f" — 이 블록 아랫면 z {a['z']}, 그 아래 블록 윗면은 가장 높아야 z {max(tops) if tops else '없음(바닥까지 비어 있음)'}"
            except (KeyError, TypeError):
                pass
        return text

    # ---------- 프롬프트 조각 ----------
    def _request_text(self, text, template, base_row, base_parts, others):
        """사용자 메시지: 요청 · Template · 바탕 설계(사람이 고름 — 요약 + 블록 목록) · 같은 Template 의 다른 설계 목록 요약(E-72 —
        한 줄 = 설계 하나, 최근 조립 결과 포함. 도구 get_design 으로 참고만)."""
        label = self.templates().get(template, {}).get('label', template)
        lines = [self._row_line(r) for r in others] or ['- 없음(이 Template 의 설계는 바탕 설계뿐)']
        return (f'[요청] {text}\n[Template] {template} — {label}\n'
                f'[바탕 설계 — 사용자가 고름]\n{self._row_line(base_row)}\n'
                + json.dumps(base_parts, ensure_ascii=False, separators=(',', ':'))
                + '\n[DB 목록 — 같은 Template 의 다른 설계(참고만)]\n' + '\n'.join(lines))

    @staticmethod
    def _row_line(r):
        """설계 목록 한 줄 — ID · 만든 방법 · 부모 · 블록 수 · 외곽 · 최근 조립 결과."""
        x, y, z = r['size_mm']
        b = r.get('last_build')
        build = (f"{b['result']} {b['placed']}/{b['total']}" + (f", 최대 오차 {b['max_err_mm']} mm" if b['max_err_mm'] is not None else '')
                 if b else '없음')
        return (f"- {r['design_id']} | {MADE_KO.get(r['made_by'], r['made_by'])} | 부모 {r['parent_id'] or '없음'} | "
                f"블록 {r['block_count']} | 외곽 {x:g}×{y:g}×{z:g} mm | 최근 조립: {build}")

    def _examples_text(self, template, base_id):
        """자동 전환 예시(E-72): 그 V000 + 최근 파생 3개의 블록 목록(도구 단계가 실패했을 때만). 바탕 설계는 이미 줬으니 뺀다."""
        lines = ['[참고 설계 — 도구 대신 코드가 넣음]']
        for e in self.store.examples_for(template, n=3):
            if e['design_id'] == base_id:
                continue
            lines.append(f"({e['design_id']}, 최근 조립 {e['last_build'] or '없음'})\n"
                         + json.dumps(e['parts']['parts'], ensure_ascii=False, separators=(',', ':')))
        return '\n'.join(lines)

    @staticmethod
    def _tool(allowed):
        """도구 get_design 정의 — design_id 는 이 Template 설계 목록 안에서만 고르게 enum 으로 묶는다(E-84 ③)."""
        return {'type': 'function', 'function': {
            'name': 'get_design', 'strict': True,
            'description': 'DB 목록의 설계 하나를 읽는다 — 블록 목록(parts/3.0)과 최근 조립 결과. 2개까지 읽을 수 있다.',
            'parameters': {'type': 'object', 'additionalProperties': False, 'required': ['design_id'],
                           'properties': {'design_id': {'type': 'string', 'enum': list(allowed)}}}}}

    def _candidates_format(self):
        """후보 3개 구조화 출력 형식(JSON Schema strict). role 은 역할 목록 파일의 이름만(enum — 목록 밖 이름은 변환기 ①이 어차피 거절)."""
        part = {'type': 'object', 'additionalProperties': False,
                 'required': ['order', 'x', 'y', 'z', 'ori', 'role', 'group', 'stage', 'grasp'],
                 'properties': {'order': {'type': 'integer'}, 'x': {'type': 'number'}, 'y': {'type': 'number'},
                                'z': {'type': 'number'}, 'ori': {'type': 'string', 'enum': list(ORIS)},
                                'role': {'type': 'string', 'enum': self.role_names}, 'group': {'type': 'integer'},
                                'stage': {'type': 'integer'}, 'grasp': {'type': 'string', 'enum': list(GRASPS)}}}
        return self._format('design_candidates', {
            'type': 'object', 'additionalProperties': False, 'required': ['out_of_scope', 'reason', 'candidates'],
            'properties': {'out_of_scope': {'type': 'boolean'}, 'reason': {'type': 'string'},
                           'candidates': {'type': 'array', 'items': {
                               'type': 'object', 'additionalProperties': False, 'required': ['idea', 'plan', 'parts'],
                               'properties': {'idea': {'type': 'string'}, 'plan': {'type': 'string'},     # plan 먼저 — 층별 계산을 쓰고 블록을 쓰게
                                              'parts': {'type': 'array', 'items': part}}}}}})

    @staticmethod
    def _format(name, schema):
        """OpenAI response_format(json_schema, strict)."""
        return {'type': 'json_schema', 'json_schema': {'name': name, 'strict': True, 'schema': schema}}

    # ---------- 기록(NFR-17) ----------
    def _new_log(self, kind, text, system, **kw):
        """기록 하나 시작 — 종류 · 요청 · 모델 · 시스템 프롬프트 sha256 앞 12자리(같은 프롬프트였는지 비교용)."""
        return {'kind': kind, 'text': text, 'started': round(time.time(), 3), 'model': MODEL,
                'prompt_sha': hashlib.sha256(system.encode('utf-8')).hexdigest()[:12], 'calls': [], 'attempts': [], **kw}

    def _save_log(self, log, messages):
        """보낸 메시지 전부(프롬프트 원문) + 응답 원문을 저장소에 남긴다. 기록 실패는 생성 결과를 막지 않는다(로그만)."""
        log['messages'] = messages
        try:
            log['file'] = self.store.save_gen_log(log)
        except Exception as e:   # noqa: BLE001 — 기록 때문에 사용자 요청이 실패하지 않게
            import logging
            logging.getLogger('web.gen').warning('[생성] 기록 저장 실패: %s', e)

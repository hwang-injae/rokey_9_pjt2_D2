# -*- coding: utf-8 -*-
"""ClaudeBridge — 웹 PC 호스트에서 OpenAI 형식 요청(/v1/chat/completions)을 받아 로그인된 Claude Code(`claude -p`)로 답한다.

왜(10/13 PL 승인): 설계 생성(design_gen)을 GPT-4o 에서 Claude 로 바꾼다. Claude 는 웹 PC 에 로그인된 Claude Code 로 부르므로
컨테이너(web) 안이 아니라 호스트에서 돈다. design_gen 은 OpenAI 라이브러리 그대로 주소(D2_LLM_URL)만 여기로 돌린다.
- response_format json_schema → `claude -p --json-schema`(구조화 출력)
- tools + tool_choice 'required' → 도구 인자 스키마로 한 번 물어 도구 호출 모양으로 돌려준다(참고 설계 읽기, E-72)
- tool_choice 'auto' · 'none' → 도구는 무시하고 답(후보)을 쓴다 — 참고 설계는 1개만 읽는다
Claude 는 도구 · MCP · 슬래시 명령 없이, 빈 임시 폴더에서 부른다(파일 · 명령 실행 없음). 대화 기록은 남기지 않는다.
키는 쓰지 않는다(로그인 계정의 구독 한도를 쓴다). 들어온 Authorization 은 보지 않는다.

실행(웹 PC 호스트): python3 web/claude_bridge.py [--host 172.17.0.1] [--port 8790] [--model opus]
"""
import argparse
import glob
import json
import logging
import os
import shutil
import subprocess
import tempfile
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

log = logging.getLogger('claude_bridge')
MODELS = ('sonnet', 'opus', 'haiku', 'fable')   # claude --model 별칭. 그 밖은 'claude-' 로 시작하는 전체 ID 만 넘긴다
RUN_TIMEOUT_S = 300                              # claude 한 번 상한 — design_gen 호출 제한(CALL_TIMEOUT_S)보다 길게(먼저 끊는 쪽은 design_gen)
ROLE_KO = {'user': '사용자', 'assistant': 'AI 답(이전)', 'tool': '도구 결과'}


class BridgeError(Exception):
    """claude 실행 실패. status = 돌려줄 HTTP 코드, message = 화면 · 기록에 남길 한 줄."""

    def __init__(self, status, message):
        """HTTP 코드와 한 줄 설명."""
        super().__init__(message)
        self.status, self.message = status, message


def find_claude():
    """claude 실행 파일 경로. 환경 변수 CLAUDE_BIN → PATH → Claude 데스크톱 앱이 깐 Claude Code(가장 새 판). 없으면 None."""
    found = os.environ.get('CLAUDE_BIN') or shutil.which('claude')
    if found:
        return found
    cands = glob.glob(os.path.expanduser('~/.config/Claude/claude-code/*/*/claude'))
    return max(cands, key=os.path.getmtime) if cands else None


class ClaudeBridge:
    """OpenAI chat.completions 요청 dict → claude -p 실행 → OpenAI 응답 dict. ROS · 네트워크 없이 시험할 수 있게 실행 함수를 바꿔 끼운다.

    입력: claude(실행 파일 경로), default_model(요청 model 이 별칭 · claude- ID 가 아닐 때), run(시험용 — (argv, stdin) → stdout 글자).
    바깥 영향: claude 프로세스 실행(로그인 계정 구독 사용량). 파일 · 명령 실행은 막아 둔다(--tools "").
    실패: BridgeError(400 = 요청 모양 틀림, 502 = claude 실패 · 시간 초과 · 사용 한도).
    """

    def __init__(self, claude, default_model='opus', run=None, timeout_s=RUN_TIMEOUT_S):
        """실행 파일 · 기본 모델 · 실행 함수를 정한다."""
        self.claude, self.default_model, self.timeout_s = claude, default_model, timeout_s
        self.run = run or self._run

    def complete(self, req):
        """요청 dict → OpenAI chat.completion dict. 도구 강제면 tool_calls, 아니면 content(JSON 글자 또는 글)."""
        messages = req.get('messages')
        if not isinstance(messages, list) or not messages:
            raise BridgeError(400, 'messages 가 없다')
        model = self.model_of(req.get('model'))
        system = '\n\n'.join(m.get('content') or '' for m in messages if m.get('role') == 'system')
        prompt = self.transcript([m for m in messages if m.get('role') != 'system'])
        tools = req.get('tools') or []
        choice = req.get('tool_choice')
        forced = self._forced_tool(tools, choice)
        if forced:
            fn = forced['function']
            prompt += (f"\n\n[지금 할 일] 답을 쓰기 전에 도구 {fn['name']}({fn.get('description', '')})를 한 번 부른다. "
                       '부를 인자만 JSON 으로 답하라.')
            out = self._ask(model, system, prompt, fn.get('parameters') or {'type': 'object'})
            args = json.dumps(out['structured'], ensure_ascii=False)
            message = {'role': 'assistant', 'content': None, 'tool_calls': [
                {'id': f'call_{uuid.uuid4().hex[:12]}', 'type': 'function', 'function': {'name': fn['name'], 'arguments': args}}]}
            return self._response(model, message, 'tool_calls', out)
        fmt = req.get('response_format') or {}
        schema = (fmt.get('json_schema') or {}).get('schema') if fmt.get('type') == 'json_schema' else None
        if schema is not None:
            prompt += '\n\n[지금 할 일] 위 대화에 이어 정해진 JSON 형식으로만 답하라.'
        out = self._ask(model, system, prompt, schema)
        content = json.dumps(out['structured'], ensure_ascii=False) if schema is not None else out['text']
        return self._response(model, {'role': 'assistant', 'content': content}, 'stop', out)

    def model_of(self, name):
        """요청 model → claude --model 값. 별칭(sonnet · opus …) · claude- 전체 ID 는 그대로, 그 밖(gpt-4o 등)은 기본 모델."""
        name = (name or '').strip()
        return name if name in MODELS or name.startswith('claude-') else self.default_model

    @staticmethod
    def transcript(messages):
        """system 을 뺀 대화 → 한 덩어리 글. 도구 호출 · 결과도 글로 펼친다(claude -p 는 한 번의 입력만 받는다)."""
        parts = []
        for m in messages:
            role = m.get('role')
            text = m.get('content') or ''
            if isinstance(text, list):   # [{type: text, text}] 모양도 받는다
                text = '\n'.join(p.get('text', '') for p in text if isinstance(p, dict))
            for tc in m.get('tool_calls') or []:
                fn = tc.get('function') or {}
                text += f"\n(도구 {fn.get('name')} 호출: {fn.get('arguments')})"
            parts.append(f'[{ROLE_KO.get(role, role)}]\n{text.strip()}')
        return '\n\n'.join(parts)

    @staticmethod
    def _forced_tool(tools, choice):
        """도구를 꼭 불러야 하는 요청이면 그 도구 정의. 'required'(도구 하나일 때) · {'type': 'function', 'function': {'name'}}만."""
        if not tools:
            return None
        if choice == 'required' and len(tools) == 1:
            return tools[0]
        if isinstance(choice, dict):
            name = (choice.get('function') or {}).get('name')
            return next((t for t in tools if (t.get('function') or {}).get('name') == name), None)
        if choice == 'required':
            raise BridgeError(400, '도구가 여럿일 때 required 는 아직 못 한다')
        return None

    def _ask(self, model, system, prompt, schema):
        """claude -p 한 번 → {'structured', 'text', 'usage'}. schema 가 있으면 구조화 출력이 꼭 있어야 한다."""
        argv = [self.claude, '-p', '--output-format', 'json', '--model', model, '--tools', '',
                '--no-session-persistence', '--strict-mcp-config', '--disable-slash-commands']
        if system:
            argv += ['--system-prompt', system]
        if schema is not None:
            argv += ['--json-schema', json.dumps(schema, ensure_ascii=False)]
        raw = self.run(argv, prompt)
        try:
            env = json.loads(raw)
        except ValueError as e:
            raise BridgeError(502, f'claude 출력이 JSON 이 아니다: {raw[:200]}') from e
        if env.get('is_error'):
            raise BridgeError(502, f"claude 실패: {str(env.get('result') or env.get('subtype'))[:300]}")
        structured = env.get('structured_output')
        if schema is not None and structured is None:
            raise BridgeError(502, f"claude 가 정해진 형식으로 답하지 않았다: {str(env.get('result'))[:200]}")
        return {'structured': structured, 'text': env.get('result') or '', 'usage': env.get('usage') or {}}

    def _run(self, argv, prompt):
        """claude 를 빈 임시 폴더에서 실행(저장소 CLAUDE.md · 파일이 끼어들지 않게) → stdout. 시간 초과 · 실패는 BridgeError(502)."""
        with tempfile.TemporaryDirectory(prefix='claude_bridge_') as cwd:
            try:
                p = subprocess.run(argv, input=prompt, text=True, capture_output=True, cwd=cwd, timeout=self.timeout_s,
                                   check=False)
            except subprocess.TimeoutExpired as e:
                raise BridgeError(502, f'claude 가 {self.timeout_s}초 안에 답하지 않았다') from e
        if p.returncode != 0 and not p.stdout.strip():
            raise BridgeError(502, f'claude 종료 코드 {p.returncode}: {p.stderr.strip()[-300:]}')
        return p.stdout

    @staticmethod
    def _response(model, message, finish, out):
        """OpenAI chat.completion 모양(design_gen 이 읽는 칸: choices[0].message · usage.total_tokens)."""
        u = out['usage']
        prompt_t = sum(int(u.get(k) or 0) for k in ('input_tokens', 'cache_read_input_tokens', 'cache_creation_input_tokens'))
        done_t = int(u.get('output_tokens') or 0)
        message.setdefault('refusal', None)
        return {'id': f'chatcmpl-{uuid.uuid4().hex[:16]}', 'object': 'chat.completion', 'created': int(time.time()), 'model': model,
                'choices': [{'index': 0, 'message': message, 'finish_reason': finish}],
                'usage': {'prompt_tokens': prompt_t, 'completion_tokens': done_t, 'total_tokens': prompt_t + done_t}}


def make_handler(bridge):
    """HTTP 처리기 — POST /v1/chat/completions 만. GET /health 는 살아 있는지(claude 경로 · 기본 모델)."""

    class Handler(BaseHTTPRequestHandler):
        """요청 하나 = claude 실행 하나. 오류는 OpenAI 오류 모양({'error': {'message'}})으로."""

        def do_GET(self):
            """살아 있는지 확인용."""
            if self.path.rstrip('/') == '/health':
                self._send(200, {'ok': True, 'claude': bridge.claude, 'default_model': bridge.default_model})
            else:
                self._send(404, {'error': {'message': 'not found'}})

        def do_POST(self):
            """chat.completions → claude. 걸린 시간 · 모델만 로그에 남긴다(프롬프트 · 답은 design_gen 기록에 있다)."""
            if self.path.rstrip('/') not in ('/v1/chat/completions', '/chat/completions'):
                self._send(404, {'error': {'message': 'not found'}})
                return
            t0 = time.time()
            try:
                req = json.loads(self.rfile.read(int(self.headers.get('Content-Length') or 0)) or b'{}')
                res = bridge.complete(req)
                log.info('[중계] %s %s %.1fs', res['model'], res['choices'][0]['finish_reason'], time.time() - t0)
                self._send(200, res)
            except BridgeError as e:
                log.warning('[중계] 실패 %.1fs: %s', time.time() - t0, e.message)
                self._send(e.status, {'error': {'message': e.message, 'type': 'claude_bridge'}})
            except ValueError as e:
                self._send(400, {'error': {'message': f'요청 JSON 이 틀렸다: {e}'}})

        def _send(self, status, body):
            """JSON 응답."""
            data = json.dumps(body, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, fmt, *args):
            """기본 접속 로그는 끈다(위 [중계] 한 줄로 충분)."""

    return Handler


def main():
    """인자를 읽고 서버를 띄운다. claude 를 못 찾으면 시작하지 않는다."""
    ap = argparse.ArgumentParser(description='OpenAI 형식 → claude -p 중계(설계 생성용)')
    ap.add_argument('--host', default='127.0.0.1',
                    help='듣는 주소. compose 의 web 컨테이너가 부르면 docker0 주소(보통 172.17.0.1) — LAN 에는 열지 않는다')
    ap.add_argument('--port', type=int, default=8790)
    ap.add_argument('--model', default=os.environ.get('D2_LLM_MODEL') or 'opus', help='요청 model 이 별칭이 아닐 때 쓸 모델')
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    claude = find_claude()
    if not claude:
        raise SystemExit('claude 실행 파일을 못 찾았다 — Claude Code 를 깔고 로그인하거나 CLAUDE_BIN 을 준다')
    bridge = ClaudeBridge(claude, a.model)
    log.info('[중계] %s:%d → %s (기본 모델 %s)', a.host, a.port, claude, a.model)
    ThreadingHTTPServer((a.host, a.port), make_handler(bridge)).serve_forever()


if __name__ == '__main__':
    main()

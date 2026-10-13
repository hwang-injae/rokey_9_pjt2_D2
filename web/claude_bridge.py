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
준비 점검(웹 PC 호스트): python3 web/claude_bridge.py --check [--host 172.17.0.1] [--no-call] — 항목마다 ✅ · ❌(할 일) · ⏭, 다 되면 종료 코드 0
"""
import argparse
import errno
import glob
import json
import logging
import os
import shutil
import socket
import subprocess
import tempfile
import time
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

log = logging.getLogger('claude_bridge')
MODELS = ('opus', 'sonnet')   # 10/13 설계 생성으로 시험한 claude --model 별칭. 그 밖은 'claude-' 로 시작하는 전체 ID 만 넘긴다
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
            for tc in m.get('tool_calls') or []:
                fn = tc.get('function') or {}
                text += f"\n(도구 {fn.get('name')} 호출: {fn.get('arguments')})"
            parts.append(f'[{ROLE_KO.get(role, role)}]\n{text.strip()}')
        return '\n\n'.join(parts)

    @staticmethod
    def _forced_tool(tools, choice):
        """도구를 꼭 불러야 하는 요청(tool_choice 'required')이면 그 도구 정의 — design_gen 은 도구 get_design 하나만 쓴다."""
        if choice == 'required' and len(tools) == 1:
            return tools[0]
        if choice == 'required':
            raise BridgeError(400, 'required 는 도구가 하나일 때만 받는다')
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
            if self.path.rstrip('/') != '/v1/chat/completions':
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


class SetupCheck:
    """웹 PC 준비 점검(--check) — 중계가 설계 생성에 쓰일 수 있는지 차례로 본다(체크리스트 A · B 를 명령 하나로, 10/13).

    항목: ① claude 실행 파일 ② 판 · 중계가 쓰는 옵션(_ask 가 실제로 넘기는 것을 모아 --help 와 대조 — 중계 코드가 바뀌어도 따라감)
          ③ 로그인 · 모델(진짜 claude 한 번, call=False 면 건너뜀) ④ 듣는 주소가 이 PC 에 있는지 ⑤ 중계가 켜져 있는지(/health)
          ⑥ 웹 컨테이너에서 중계가 보이는지(compose service web 이 떠 있고 ⑤가 됐을 때만).
    입력: host · port · model(중계 인자와 같음), call(③을 할지), cmd((argv, timeout) → (종료 코드, 출력) — 시험 때 가짜),
          fetch((url, timeout) → dict — 시험 때 가짜), claude(실행 파일 — 비우면 find_claude), bridge_run(③의 claude 실행 — 시험 때 가짜).
    출력: run() → [(상태 'ok' · 'fail' · 'skip', 이름, 한 줄 설명, 할 일)]. 앞 항목이 실패해 볼 수 없는 항목은 skip.
    바깥 영향: claude --version · --help 실행, ③이면 claude 한 번(구독 사용량 조금), /health 요청, docker ps · exec. 아무것도 바꾸지 않는다.
    """

    def __init__(self, host, port, model, call=True, cmd=None, fetch=None, claude=None, bridge_run=None):
        """점검 대상과 바꿔 끼울 실행 함수들을 받는다."""
        self.host, self.port, self.model, self.call = host, port, model, call
        self.cmd, self.fetch, self.bridge_run = cmd or self._cmd, fetch or self._fetch, bridge_run
        self.claude = claude if claude is not None else find_claude()

    def run(self):
        """6 항목을 차례로 → 결과 목록."""
        out = []
        if not self.claude:
            out.append(('fail', 'claude 실행 파일', '못 찾음(CLAUDE_BIN · PATH · 데스크톱 앱 폴더)',
                        'Claude Code(데스크톱 앱 또는 claude 명령)를 깔거나 CLAUDE_BIN=<경로> 를 준다'))
            out += [('skip', n, 'claude 가 없어 못 봄', '') for n in ('판 · 옵션', '로그인 · 모델')]
        else:
            out.append(('ok', 'claude 실행 파일', self.claude, ''))
            out.append(self._flags())
            out.append(self._login() if self.call and out[-1][0] == 'ok'
                       else ('skip', '로그인 · 모델', '--no-call' if not self.call else '옵션이 모자라 못 봄', ''))
        out.append(self._address())
        out.append(self._health())
        out.append(self._container() if out[-1][0] == 'ok' else ('skip', '웹 컨테이너에서', '중계가 꺼져 있어 못 봄', ''))
        return out

    def used_flags(self):
        """중계(_ask)가 claude 에 실제로 넘기는 옵션 이름들 — 가짜 실행으로 인자를 모은다(구조화 · system 둘 다 쓰는 길)."""
        seen = []

        def capture(argv, prompt):
            seen.extend(argv)
            return json.dumps({'is_error': False, 'structured_output': {}, 'result': '', 'usage': {}})
        ClaudeBridge(self.claude or 'claude', self.model, capture)._ask(self.model, 'system', 'prompt', {'type': 'object'})
        return sorted({a for a in seen if a.startswith('--')})

    def _flags(self):
        """② claude --version · --help 에 중계가 쓰는 옵션이 다 있는지."""
        code, ver = self.cmd([self.claude, '--version'], 20)
        code2, help_text = self.cmd([self.claude, '--help'], 20)
        if code != 0 or code2 != 0:
            return ('fail', '판 · 옵션', f'claude 를 실행하지 못함(종료 코드 {code} · {code2})', '실행 파일 권한 · 경로를 확인한다')
        missing = [f for f in self.used_flags() if f not in help_text]
        if missing:
            return ('fail', '판 · 옵션', f'{ver.strip()} — 없는 옵션 {missing}', 'Claude Code 를 새 판으로 올린다')
        return ('ok', '판 · 옵션', f'{ver.strip()} — 중계 옵션 {len(self.used_flags())}개 다 있음', '')

    def _login(self):
        """③ 진짜 claude 한 번(정해진 형식 답) — 로그인 · 모델 · 구조화 출력이 되는지. 걸린 시간도."""
        fmt = {'type': 'json_schema', 'json_schema': {'name': 'ping', 'schema': {
            'type': 'object', 'required': ['n'], 'properties': {'n': {'type': 'integer'}}}}}
        t0 = time.time()
        try:
            res = ClaudeBridge(self.claude, self.model, self.bridge_run, timeout_s=120).complete(
                {'model': self.model, 'response_format': fmt, 'messages': [{'role': 'user', 'content': '숫자 1 을 n 에 넣어 답하라'}]})
            n = json.loads(res['choices'][0]['message']['content']).get('n')
        except (BridgeError, ValueError, KeyError) as e:
            text = getattr(e, 'message', str(e))
            low = text.lower()
            todo = ('구독 사용 한도 — 풀릴 때까지 기다리거나 시연 계정을 바꾼다' if 'limit' in low else
                    '그 PC 에서 claude 를 한 번 실행해 로그인한다(또는 데스크톱 앱 로그인)' if 'log' in low or 'auth' in low else
                    f'--model {"sonnet" if self.model != "sonnet" else "opus"} 로 다시 점검해 보고, 되면 web/.env 의 D2_LLM_MODEL 을 바꾼다'
                    if 'model' in low else '위 오류를 보고 claude 를 직접 한 번 실행해 본다')
            return ('fail', '로그인 · 모델', f'{self.model}: {text[:160]}', todo)
        return ('ok', '로그인 · 모델', f'{self.model} 답 n={n} · {time.time() - t0:.1f} s', '')

    def _address(self):
        """④ 듣는 주소가 이 PC 에 있는지(0번 포트로 묶어 봄 — 중계가 이미 그 포트를 써도 상관없게)."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind((self.host, 0))
        except OSError as e:
            return ('fail', '듣는 주소', f'{self.host} 가 이 PC 에 없음({e.strerror})',
                    'ip -4 addr show docker0 의 주소로 --host 를 준다(보통 172.17.0.1)')
        return ('ok', '듣는 주소', f'{self.host} 있음', '')

    def _health(self):
        """⑤ 중계가 켜져 있는지 — GET /health."""
        url = f'http://{self.host}:{self.port}/health'
        try:
            body = self.fetch(url, 3)
        except Exception as e:   # noqa: BLE001 — 연결 거부 · 시간 초과 모두 '꺼져 있음'으로
            return ('fail', '중계 켜짐', f'{url} 응답 없음({type(e).__name__})',
                    f'다른 터미널에서 python3 web/claude_bridge.py --host {self.host}'
                    + (f' --port {self.port}' if self.port != 8790 else '') + ' 를 켜 둔다')
        return ('ok', '중계 켜짐', f"{url} — 기본 모델 {body.get('default_model')}", '') if body.get('ok') else \
            ('fail', '중계 켜짐', f'{url} 답이 이상함: {str(body)[:120]}', '그 포트를 다른 프로그램이 쓰는지 본다')

    def _container(self):
        """⑥ 웹 컨테이너(compose service web)에서 host.docker.internal 로 중계가 보이는지. 컨테이너가 없으면 skip."""
        code, names = self.cmd(['docker', 'ps', '--filter', 'label=com.docker.compose.service=web', '--format', '{{.Names}}'], 10)
        name = names.split()[0] if code == 0 and names.strip() else None
        if not name:
            return ('skip', '웹 컨테이너에서', '웹 컨테이너가 안 떠 있음(docker compose up 뒤 다시 점검)', '')
        probe = (f"import urllib.request as u; print(u.urlopen('http://host.docker.internal:{self.port}/health', timeout=5).read().decode())")
        code, out = self.cmd(['docker', 'exec', name, 'python', '-c', probe], 20)
        if code == 0 and '"ok": true' in out:
            return ('ok', '웹 컨테이너에서', f'{name} → host.docker.internal:{self.port} 보임', '')
        last = (out.strip().splitlines() or [''])[-1]          # 파이썬 오류는 마지막 줄만(이유)
        return ('fail', '웹 컨테이너에서', f'{name} 에서 안 보임: {last[-120:]}',
                f'방화벽(sudo ufw status)이 docker(172.17.0.0/16)에서 {self.port} 를 막는지 본다(PL 확인 뒤 허용) · '
                'compose 의 extra_hosts host.docker.internal:host-gateway 확인')

    @staticmethod
    def _cmd(argv, timeout):
        """명령 실행 → (종료 코드, 출력). 실행 파일이 없거나 시간 초과면 (127, 이유)."""
        try:
            p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, check=False)
            return p.returncode, p.stdout + p.stderr
        except (OSError, subprocess.TimeoutExpired) as e:
            return 127, f'{type(e).__name__}: {e}'

    @staticmethod
    def _fetch(url, timeout):
        """GET → JSON dict."""
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode('utf-8'))

    @staticmethod
    def report(results):
        """결과 목록 → 사람이 읽을 글(항목마다 한 줄 + 실패면 할 일) · 끝에 요약."""
        mark = {'ok': '✅', 'fail': '❌', 'skip': '⏭ '}
        lines = []
        for i, (state, name, detail, todo) in enumerate(results, 1):
            lines.append(f'{mark[state]} {i}. {name} — {detail}')
            if state == 'fail' and todo:
                lines.append(f'      → 할 일: {todo}')
        fails = sum(1 for r in results if r[0] == 'fail')
        lines.append('준비 끝 — 웹 화면에서 [생성]을 한 번 해 본다' if not fails else f'❌ {fails}개를 고친 뒤 다시 --check')
        return '\n'.join(lines)


def main():
    """인자를 읽고 서버를 띄운다(--check 면 준비 점검만 하고 끝). claude 를 못 찾으면 시작하지 않는다."""
    ap = argparse.ArgumentParser(description='OpenAI 형식 → claude -p 중계(설계 생성용)')
    ap.add_argument('--host', default='127.0.0.1',
                    help='듣는 주소. compose 의 web 컨테이너가 부르면 docker0 주소(보통 172.17.0.1) — LAN 에는 열지 않는다')
    ap.add_argument('--port', type=int, default=8790)
    ap.add_argument('--model', default=os.environ.get('D2_LLM_MODEL') or 'opus', help='요청 model 이 별칭이 아닐 때 쓸 모델')
    ap.add_argument('--check', action='store_true', help='웹 PC 준비 점검만 하고 끝(중계는 켜지 않음) — 다 되면 종료 코드 0')
    ap.add_argument('--no-call', action='store_true', help='--check 때 진짜 claude 호출(로그인 · 모델)을 건너뜀(구독 사용량 안 씀)')
    a = ap.parse_args()
    if a.check:
        results = SetupCheck(a.host, a.port, a.model, call=not a.no_call).run()
        print(SetupCheck.report(results))
        raise SystemExit(1 if any(r[0] == 'fail' for r in results) else 0)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    claude = find_claude()
    if not claude:
        raise SystemExit('claude 실행 파일을 못 찾았다 — Claude Code 를 깔고 로그인하거나 CLAUDE_BIN 을 준다')
    bridge = ClaudeBridge(claude, a.model)
    try:
        server = ThreadingHTTPServer((a.host, a.port), make_handler(bridge))
    except OSError as e:
        if e.errno == errno.EADDRINUSE:   # 흔한 경우 = 이미 다른 터미널에서 켜 둠 — 긴 오류 대신 할 일만
            raise SystemExit(f'{a.host}:{a.port} 를 이미 쓰고 있다 — 중계가 벌써 켜져 있으면 그대로 쓰면 된다 '
                             f'(확인: curl -s http://{a.host}:{a.port}/health)') from e
        raise
    log.info('[중계] %s:%d → %s (기본 모델 %s)', a.host, a.port, claude, a.model)
    server.serve_forever()


if __name__ == '__main__':
    main()

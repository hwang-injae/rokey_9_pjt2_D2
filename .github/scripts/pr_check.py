#!/usr/bin/env python3
"""PR 자동 검사 (팀 규칙 반영). PR 코드는 읽기만 하고 실행하지 않는다.

검사:
  1. main 반영 — 최신 main이 내 브랜치에 들어 있는가(= main을 merge했는가)
  2. 충돌 — main과 합칠 때 충돌이 없는가
  3. 문법 — .py(컴파일만) · .yaml/.yml · .json · .xml/.urdf/.xacro/.srdf/.launch
  4. 비밀값 — OpenAI API 키 모양 · .env · 키 파일 (팀 규칙 4)
  5. 변경 파일 — PR 본문 '변경 파일' 칸이 채워져 있는가(실패). 개별 커밋의 '변경 파일:' 줄은 경고만 (팀 규칙 8)
     squash merge 커밋 메시지를 'PR 제목 + 본문'으로 두면 main 기록에 변경 파일 목록이 남는다.
  6. 문서 이름 — docs/ 아래 .md는 이름_v<버전>_<MMDDHH>.md (README.md 제외, 팀 규칙 9)

사용: python pr_check.py --repo <PR 작업 폴더> --base origin/main
"""
import argparse
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

KEY_RE = re.compile(r'sk-(proj-|svcacct-)?[A-Za-z0-9_\-]{20,}')
DOC_RE = re.compile(r'_v\d+(-\d+)?_\d{6}\.md$')
SECRET_FILES = re.compile(r'(^|/)(\.env(\..+)?|.*\.(pem|key|p12|crt)|credentials.*|secrets\.ya?ml)$')
XML_EXT = ('.xml', '.urdf', '.xacro', '.srdf', '.launch')


class PrCheck:
    def __init__(self, repo, base):
        self.repo = Path(repo)
        self.base = base
        self.fails = []
        self.notes = []
        self.warns = []

    def git(self, *args, check=True):
        r = subprocess.run(['git', '-c', 'core.quotePath=false', *args], cwd=self.repo, capture_output=True, text=True)
        if check and r.returncode != 0:
            raise RuntimeError(f"git {' '.join(args)}: {r.stderr.strip()}")
        return r

    def check_main_merged(self):
        if self.git('merge-base', '--is-ancestor', self.base, 'HEAD', check=False).returncode == 0:
            self.notes.append('main 반영: 최신 main이 들어 있음 → 합칠 때 충돌 없음')
            return True
        self.fails.append('main 반영: 최신 main이 내 브랜치에 없다. `git fetch origin && git merge origin/main` 으로 합치고(충돌은 내 PC에서 풀고) 다시 push 한다.')
        r = self.git('merge-tree', '--write-tree', self.base, 'HEAD', check=False)
        if r.returncode != 0:
            self.fails.append('충돌: main과 합치면 충돌이 난다 — 위 방법으로 합치면서 충돌을 푼다.')
        return False

    def changed_files(self):
        out = self.git('diff', '--name-only', '--diff-filter=ACMR', f'{self.base}...HEAD').stdout
        return [f for f in out.splitlines() if f]

    def check_syntax(self, files):
        bad = []
        for f in files:
            p = self.repo / f
            if not p.is_file():
                continue
            try:
                if f.endswith('.py'):
                    compile(p.read_text(encoding='utf-8'), f, 'exec')
                elif f.endswith(('.yaml', '.yml')):
                    import yaml
                    list(yaml.safe_load_all(p.read_text(encoding='utf-8')))
                elif f.endswith('.json'):
                    json.loads(p.read_text(encoding='utf-8'))
                elif f.endswith(XML_EXT):
                    ET.parse(p)
            except Exception as e:  # 어떤 오류든 문법 실패로 본다
                bad.append(f'{f}: {type(e).__name__}: {e}')
        if bad:
            self.fails.append('문법 오류:\n  - ' + '\n  - '.join(bad))
        else:
            self.notes.append(f'문법: 바뀐 파일 {len(files)}개 통과')

    def check_secrets(self, files):
        bad = [f for f in files if SECRET_FILES.search(f) and not f.endswith('.env.example')]
        diff = self.git('diff', '-U0', f'{self.base}...HEAD').stdout
        hits = [l[:12] + '…' for l in diff.splitlines() if l.startswith('+') and KEY_RE.search(l)]
        if bad:
            self.fails.append('비밀 파일이 들어 있다(팀 규칙 4): ' + ', '.join(bad))
        if hits:
            self.fails.append(f'API 키 모양의 문자열이 {len(hits)}곳 있다(팀 규칙 4). 지우고, 이미 올렸다면 PL에게 알려 키를 폐기한다.')
        if not bad and not hits:
            self.notes.append('비밀값: 키·.env 없음')

    def check_commit_messages(self):
        out = self.git('log', '--no-merges', '--format=%h%x1f%s%x1f%b%x1e', f'{self.base}..HEAD').stdout
        missing = []
        for rec in out.split('\x1e'):
            rec = rec.strip()
            if not rec:
                continue
            h, subject, body = (rec.split('\x1f') + ['', ''])[:3]
            if not re.search(r'^변경 파일\s*:', body, re.M):
                missing.append(f'{h} {subject}')
        if missing:
            self.warns.append("커밋 메시지에 '변경 파일:' 줄이 없는 커밋(경고, 다음 커밋부터 넣기):\n  - " + '\n  - '.join(missing))
        else:
            self.notes.append("커밋 메시지: 모두 '변경 파일:' 있음")

    def check_pr_body(self):
        body = os.environ.get('PR_BODY')
        if body is None:
            self.notes.append('PR 본문: (로컬 실행이라 건너뜀)')
            return
        m = re.search(r'^##\s*변경 파일[^\n]*\n(.*?)(?=^##\s|\Z)', body, re.M | re.S)
        items = [l.strip()[1:].strip() for l in (m.group(1).splitlines() if m else []) if l.strip().startswith('-')]
        if not any(items):
            self.fails.append("PR 본문의 '변경 파일' 칸이 비어 있다(팀 규칙 8). 바뀐 파일 경로를 '- 경로' 줄로 적는다.")
        else:
            self.notes.append(f"PR 본문 '변경 파일': {len([i for i in items if i])}개 적힘")

    def check_doc_names(self, files):
        bad = [f for f in files if f.startswith('docs/') and f.endswith('.md')
               and Path(f).name != 'README.md' and not DOC_RE.search(f)]
        if bad:
            self.fails.append('문서 이름 규칙(팀 규칙 9: 이름_v<버전>_<MMDDHH>.md)에 안 맞는다:\n  - ' + '\n  - '.join(bad))
        else:
            self.notes.append('문서 이름: 규칙에 맞음')

    def run(self):
        self.check_main_merged()
        files = self.changed_files()
        self.check_syntax(files)
        self.check_secrets(files)
        self.check_commit_messages()
        self.check_pr_body()
        self.check_doc_names(files)
        ok = not self.fails
        lines = ['## PR 검사 결과: ' + ('통과 ✅' if ok else '실패 ❌'), '']
        lines += [f'- ✅ {n}' for n in self.notes]
        lines += [f'- ⚠️ {w}' for w in self.warns]
        lines += [f'- ❌ {f}' for f in self.fails]
        report = '\n'.join(lines)
        print(report)
        summary = os.environ.get('GITHUB_STEP_SUMMARY')
        if summary:
            with open(summary, 'a', encoding='utf-8') as fh:
                fh.write(report + '\n')
        return 0 if ok else 1


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default='.')
    ap.add_argument('--base', default='origin/main')
    a = ap.parse_args()
    sys.exit(PrCheck(a.repo, a.base).run())

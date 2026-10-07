#!/usr/bin/env python3
"""Claude 검토 결과로 PR을 판정해 GitHub에 남긴다: 자동 승인 / 수정 요청 / 코멘트만.

판정은 Claude가 아니라 이 스크립트가 정한다 — ①~⑧ 중 '막음'이 하나라도 있으면 수정 요청,
8개 항목이 다 오지 않았거나 검토가 돌지 못했으면 승인하지 않고 사람에게 넘긴다.
막음이 없어도 PR 본문에 체크 안 된 'PL(황인재) 확인' 항목(`- [ ] … PL 확인 필요`)이 있으면 승인하지 않고 코멘트만 남긴다
(10/7 PL — PR #45가 PL 확인 전에 자동 merge됨). PL이 확인한 뒤 직접 승인하면 auto-merge 된다.

입력(환경 변수):
  REVIEW_RESULT  검토 job 결과 (success · failure · skipped · cancelled)
  REVIEW_JSON    Claude 구조화 결과 {"items": [{"no", "result", "where", "reason"}], "notes": [...], "summary"}
  REPO, PR, AUTHOR, RUN_URL
  GH_TOKEN       승인 계정 토큰 (D2_APPROVER_TOKEN, 없으면 GITHUB_TOKEN)
바깥 영향: gh 로 PR 리뷰(승인·수정 요청) 또는 코멘트를 하나 남긴다. 실패해도 job은 실패시키지 않는다(코멘트로 알림).
"""
import json
import os
import re
import subprocess
import tempfile

ITEMS = {
    1: '정지 처리',
    2: '장면·멈춤·차례·진행표 담당',
    3: '/d2/ 이름·IRD',
    4: 'robot.yaml·개인 경로',
    5: '클래스로 묶기',
    6: '남의 파트 파일',
    7: '키·원본 자료',
    8: 'docstring (규칙 10)',
}
MARK = {'통과': '✅ 통과', '막음': '❌ 막음', '해당 없음': '— 해당 없음'}
# 체크 안 된 체크박스 줄 중 'PL 확인' · '황인재 확인'을 말하는 줄 (예: '- [ ] 다른 파트 영향 없음 — PL 확인 필요')
PL_CHECK = re.compile(r'^\s*[-*]\s*\[ \]\s*(.*(?:PL|황인재)\s*확인.*)$', re.M)


def find_pl_checks(body):
    """PR 본문에서 체크 안 된 'PL(황인재) 확인' 항목을 찾는다. 입력: 본문 글자(없으면 빈 글자). 출력: 걸린 줄 목록(앞 120자)."""
    return [m.group(1).strip()[:120] for m in PL_CHECK.finditer(body or '')]


APPROVERS = {'hwang-injae': '황인재 @hwang-injae', 'hansaekyo': '한세교 @hansaekyo'}


class Verdict:
    """환경 변수를 읽어 판정하고, 리뷰 본문을 만들어 gh 로 남긴다."""

    def __init__(self):
        env = os.environ
        self.repo, self.pr, self.author = env['REPO'], env['PR'], env['AUTHOR']
        self.run_url = env.get('RUN_URL', '')
        self.review_result = env.get('REVIEW_RESULT', '')
        self.raw = env.get('REVIEW_JSON', '')
        # 올린 사람은 자기 PR을 승인할 수 없으므로 안내에서 뺀다
        self.approvers = ' · '.join(n for login, n in APPROVERS.items() if login != self.author)

    def gh(self, *args):
        """gh 명령을 실행하고 성공 여부를 돌려준다(실패 내용은 로그에만)."""
        r = subprocess.run(['gh', *args], capture_output=True, text=True)
        if r.returncode != 0:
            print(f"gh {' '.join(args[:3])}… 실패: {r.stderr.strip()}")
        return r.returncode == 0

    def pl_checks(self):
        """이 PR 본문의 체크 안 된 PL 확인 항목. 본문을 못 읽으면 빈 목록(지금까지처럼 판정) — 실패는 로그에만 남긴다."""
        r = subprocess.run(['gh', 'pr', 'view', self.pr, '--repo', self.repo, '--json', 'body', '--jq', '.body'],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f'PR 본문을 못 읽음: {r.stderr.strip()}')
            return []
        return find_pl_checks(r.stdout)

    def approver(self):
        r = subprocess.run(['gh', 'api', 'user', '--jq', '.login'], capture_output=True, text=True)
        return r.stdout.strip() if r.returncode == 0 else 'github-actions[bot]'

    def parse(self):
        """Claude 결과를 {번호: 항목}으로. 형식이 틀리거나 8개가 다 없으면 None."""
        try:
            data = json.loads(self.raw)
            items = {int(i['no']): i for i in data.get('items', [])}
        except (ValueError, TypeError, KeyError, AttributeError):
            return None
        if set(items) != set(ITEMS) or any(i.get('result') not in MARK for i in items.values()):
            return None
        return data, items

    @staticmethod
    def cell(text):
        return str(text or '').replace('|', '\\|').replace('\n', ' ').strip()

    def body(self, data, items, blocked):
        lines = ['## Claude 검토 결과: ' + ('수정 요청 ❌' if blocked else '통과 ✅'), '',
                 '| 항목 | 결과 | 파일:줄 | 이유 |', '|---|---|---|---|']
        for no, name in ITEMS.items():
            i = items[no]
            lines.append(f"| {'①②③④⑤⑥⑦⑧'[no - 1]} {name} | {MARK[i['result']]} | {self.cell(i.get('where'))} | {self.cell(i.get('reason'))} |")
        if data.get('summary'):
            lines += ['', data['summary'].strip()]
        notes = [n for n in data.get('notes') or [] if str(n).strip()]
        if notes:
            lines += ['', '**참고 의견** (막는 사유 아님)'] + [f'- {self.cell(n)}' for n in notes]
        lines += ['', f'<sub>기준: `.claude/commands/pr-review.md` ①~⑧ · [실행 기록]({self.run_url}) · '
                      f'Claude 판정은 틀릴 수 있다 — 승인자가 `/pr-review {self.pr}`로 다시 볼 수 있다.</sub>']
        if blocked:
            lines += ['', '❌ 항목을 고쳐 다시 push 하면 검토가 다시 돈다.']
        return '\n'.join(lines)

    def post(self, kind, text):
        """kind: approve · request-changes · comment."""
        with tempfile.NamedTemporaryFile('w', suffix='.md', delete=False, encoding='utf-8') as fh:
            fh.write(text)
        if kind == 'comment':
            return self.gh('pr', 'comment', self.pr, '--repo', self.repo, '--body-file', fh.name)
        return self.gh('pr', 'review', self.pr, '--repo', self.repo, f'--{kind}', '--body-file', fh.name)

    def run(self):
        if self.review_result == 'skipped':
            self.post('comment', 'PR 검사 통과 ✅ — 팀원(협업자)이 아닌 계정이 연 PR이라 Claude 검토·자동 승인을 하지 않습니다. '
                                 f'승인자({self.approvers})가 직접 확인합니다.')
            return
        parsed = self.parse() if self.review_result == 'success' else None
        if parsed is None:
            self.post('comment', f'PR 검사는 통과 ✅, **Claude 검토를 못 했습니다**([실행 기록]({self.run_url})). '
                                 f'자동 승인하지 않았습니다 — 승인자({self.approvers})가 `/pr-review {self.pr}`로 확인해 주세요.')
            return
        data, items = parsed
        blocked = any(i['result'] == '막음' for i in items.values())
        text = self.body(data, items, blocked)
        if self.approver() == self.author:
            # GitHub은 본인 PR에 승인·수정 요청을 못 하게 막는다 → 결과만 남기고 다른 승인자에게 넘긴다
            self.post('comment', text + f'\n\n올린 사람이 자동 승인 계정과 같아서 자동 판정을 남기지 못합니다. '
                                        f'다른 승인자({self.approvers})가 승인해 주세요.')
            return
        waits = [] if blocked else self.pl_checks()
        if waits:
            head = text.replace('## Claude 검토 결과: 통과 ✅', '## Claude 검토 결과: 통과 ✅ — PL 확인 대기(자동 승인 안 함)', 1)
            self.post('comment', head + '\n\n⏸ 본문에 체크 안 된 PL 확인 항목이 있어 **자동 승인하지 않았습니다**:\n'
                      + '\n'.join(f'- {w}' for w in waits)
                      + '\n\nPL(황인재 @hwang-injae)이 확인한 뒤 직접 승인(Approve)하면 auto-merge 됩니다. (10/7 PL — PR #45)')
            return
        if not self.post('request-changes' if blocked else 'approve', text):
            self.post('comment', text + f'\n\n⚠️ 자동 판정을 남기지 못했습니다(승인 토큰·설정 확인 필요). 승인자({self.approvers})가 직접 판정해 주세요.')


if __name__ == '__main__':
    Verdict().run()

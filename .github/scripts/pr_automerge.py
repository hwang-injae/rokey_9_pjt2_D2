#!/usr/bin/env python3
"""PR에 GitHub auto-merge(Squash)를 걸어 둔다 — 승인(자동·사람)과 필수 검사(PR 검사·CI)가 끝나면 GitHub이 merge하고 브랜치를 지운다.

merge 조건을 정하는 것은 이 스크립트가 아니라 브랜치 보호(승인 1명 + Code Owners + 필수 검사 3개)다.
이 스크립트는 '조건이 되면 merge해 달라'는 예약만 건다(10/6 저녁 PL 결정 — 사람이 merge 버튼을 누르지 않는다).

입력(환경 변수):
  REPO, PR     저장소 · PR 번호
  GH_TOKEN     판정 계정 토큰(D2_APPROVER_TOKEN, 없으면 GITHUB_TOKEN)
바깥 영향: `gh pr merge --auto --squash` 한 번. squash 커밋 메시지는 저장소 설정(PR 제목 + 본문)을 따른다.
실패 때: 이미 걸려 있거나(다시 push) 조건이 이미 다 끝난 PR(clean — auto-merge를 걸 수 없음)은 그때만 바로 merge한다.
       그 밖의 실패는 로그에만 남기고 job을 실패시키지 않는다 — 사람이 PR 화면에서 'Enable auto-merge'를 누르면 된다.
"""
import os
import subprocess


def gh(*args):
    """gh 명령을 실행해 (성공 여부, 출력 또는 오류 문장)을 돌려준다."""
    r = subprocess.run(['gh', *args], capture_output=True, text=True)
    return r.returncode == 0, (r.stdout if r.returncode == 0 else r.stderr).strip()


def main():
    repo, pr = os.environ['REPO'], os.environ['PR']
    ok, msg = gh('pr', 'merge', pr, '--repo', repo, '--auto', '--squash')
    if ok:
        print(f'PR #{pr}: auto-merge(squash) 걸었다 — 승인 + 필수 검사가 끝나면 GitHub이 merge한다.')
        return
    print(f'PR #{pr}: auto-merge 걸기 실패 — {msg}')
    # 승인·검사가 이미 다 끝난 PR은 GitHub이 auto-merge 예약을 거부한다(clean status). 그때만 바로 merge한다.
    ok, state = gh('pr', 'view', pr, '--repo', repo, '--json', 'mergeStateStatus', '--jq', '.mergeStateStatus')
    if ok and state == 'CLEAN':
        ok, msg = gh('pr', 'merge', pr, '--repo', repo, '--squash')
        print(f'PR #{pr}: 조건이 이미 끝나 바로 merge — ' + ('성공' if ok else f'실패: {msg}'))
    else:
        print(f'PR #{pr}: 상태 {state!r} — 그대로 둔다(조건이 되면 사람이 Enable auto-merge를 누르거나 다음 push 때 다시 건다).')


if __name__ == '__main__':
    main()

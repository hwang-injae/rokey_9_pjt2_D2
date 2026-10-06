#!/usr/bin/env python3
"""Claude 검토 자료(review_input/)를 만든다 — 데이터 파일은 diff 대신 '이름·크기'만 넣어 토큰을 아낀다(10/7 PL).

PR #15에서 CAD(dxf·step)·레시피 json 48,000줄이 diff 그대로 Claude에게 갔다. 검토 ①~⑧에서 데이터 파일은
⑦(키·원본 자료 — 이름·크기·출처)만 보면 되므로 내용은 빼고 목록만 준다. 코드·설정·문서는 그대로 numbered_diff로 준다.

사용: python3 review_input.py --repo pr --base origin/main --out review_input
만드는 파일:
  files.txt    바뀐 파일 전부 (git diff --name-status)
  changes.txt  코드·설정·문서의 diff(실제 줄 번호 붙임) + 맨 끝에 '데이터 파일(내용 생략)' 표(이름 · 크기 · +줄/-줄)
데이터 파일 판정: 확장자가 DATA_EXT 이거나, 바뀐 줄이 BIG_LINES 를 넘는 파일(큰 json·csv 등). 바이너리는 numstat 이 '-' 라 자동으로 데이터.
입력은 git 명령 결과만 쓰고(PR 코드는 실행하지 않음), 바깥 영향은 --out 폴더에 파일 두 개를 쓰는 것뿐이다. git 이 실패하면 그대로 예외로 멈춘다.
"""
import argparse
import subprocess
import sys
from pathlib import Path

DATA_EXT = {'.dxf', '.step', '.stp', '.stl', '.obj', '.png', '.jpg', '.jpeg', '.gif', '.bmp', '.svg', '.npy', '.npz',
            '.pdf', '.xlsx', '.docx', '.pptx', '.zip', '.bin', '.pt', '.onnx', '.bag', '.db', '.mp4', '.wav', '.drawio'}
BIG_LINES = 1500  # 코드 파일이 이보다 크게 바뀌는 일은 거의 없다 — 넘으면 데이터(큰 json·csv)로 본다


def git(repo, *args):
    return subprocess.run(['git', '-C', repo, '-c', 'core.quotePath=false', *args], capture_output=True, text=True, check=True).stdout


def classify(repo, base):
    """(코드 파일 목록, 데이터 파일 표 줄 목록)을 돌려준다. numstat 한 줄 = 'added\\tdeleted\\tpath' (바이너리는 '-')."""
    code, data = [], []
    for line in git(repo, 'diff', '--numstat', f'{base}...HEAD').splitlines():
        added, deleted, path = line.split('\t', 2)
        binary = added == '-'
        big = not binary and int(added) + int(deleted) > BIG_LINES
        if binary or big or Path(path).suffix.lower() in DATA_EXT:
            size = (Path(repo) / path).stat().st_size if (Path(repo) / path).exists() else 0
            shown = f'{size} B' if size < 1024 else f'{size / 1024:.0f} KB'  # 256 B 같은 작은 파일이 '0 KB'로 보이지 않게
            data.append(f'{path}\t{shown}\t' + ('바이너리' if binary else f'+{added}/-{deleted}'))
        else:
            code.append(path)
    return code, data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default='pr')
    ap.add_argument('--base', default='origin/main')
    ap.add_argument('--out', default='review_input')
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'files.txt').write_text(git(a.repo, 'diff', '--name-status', f'{a.base}...HEAD'), encoding='utf-8')
    code, data = classify(a.repo, a.base)
    numbered = ''
    if code:
        diff = git(a.repo, 'diff', f'{a.base}...HEAD', '--', *code)
        r = subprocess.run([sys.executable, str(Path(__file__).with_name('numbered_diff.py'))], input=diff, capture_output=True, text=True, check=True)
        numbered = r.stdout
    tail = ''
    if data:
        tail = ('\n=== 데이터 파일 (내용 생략 — ⑦은 이름·크기·출처로 판정. 꼭 필요하면 pr/<경로>를 열 수 있다)\n'
                + '\n'.join(data) + '\n')
    (out / 'changes.txt').write_text(numbered + tail, encoding='utf-8')
    print(f'코드·문서 {len(code)}개 diff, 데이터 {len(data)}개 목록만')


if __name__ == '__main__':
    main()

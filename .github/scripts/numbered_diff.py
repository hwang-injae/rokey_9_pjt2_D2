#!/usr/bin/env python3
"""git diff 출력(표준 입력)을 '새 파일의 실제 줄 번호'가 붙은 글로 바꾼다 — Claude 검토 자료용.

Claude가 결과표 파일:줄에 diff 자체의 줄 번호를 적는 일이 있어서(PR #6), 줄마다 실제 번호를 붙여 준다.
출력 모양:
  === src/pkg/node.py
     24 + class Node:        ← 더한 줄(새 파일 24줄)
     25     def run(self):   ← 바뀌지 않은 주변 줄
        - old_line()         ← 지운 줄(새 파일에 없어서 번호 없음)
바깥 영향 없음(읽고 쓰기만). 형식을 모르는 줄은 그대로 건너뛴다.

사용: git diff origin/main...HEAD | python3 numbered_diff.py > changes.txt
"""
import re
import sys

HUNK = re.compile(r'^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@')


def main():
    n = None  # 지금 hunk에서 다음 '새 파일' 줄 번호
    for raw in sys.stdin:
        line = raw.rstrip('\n')
        if line.startswith('diff --git '):
            n = None
            print('\n=== ' + line.split(' b/', 1)[-1])
        elif line.startswith(('index ', '--- ', '+++ ', 'similarity ', 'rename ', 'new file', 'deleted file', 'old mode', 'new mode')):
            continue
        elif line.startswith('Binary files'):
            print('   (바이너리 파일 — 내용 없음)')
        elif (m := HUNK.match(line)):
            n = int(m.group(1))
            print('   ...')
        elif n is None or line.startswith('\\'):
            continue
        elif line.startswith('+'):
            print(f'{n:6d} + {line[1:]}')
            n += 1
        elif line.startswith('-'):
            print(f'       - {line[1:]}')
        else:
            print(f'{n:6d}   {line[1:]}')
            n += 1


if __name__ == '__main__':
    main()

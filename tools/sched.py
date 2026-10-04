#!/usr/bin/env python3
"""공유 드라이브의 일정표(정본)를 받아 터미널에 보여 준다. 로그인 필요 없음(링크 공유), 표준 라이브러리만 쓴다.

사용:
  python3 tools/sched.py                  오늘 작업 전체
  python3 tools/sched.py 황인재            오늘 내 작업 ('전원' 작업 포함)
  python3 tools/sched.py 황인재 --date 10/6   그날 내 작업
  python3 tools/sched.py 황인재 --all       내 작업 전체(완료 포함)
  python3 tools/sched.py W041             작업 하나 — 담당·상태·날짜 칸
  python3 tools/sched.py --robot          오늘 로봇 쓰는 작업([로봇 n] 순서)
  --file 일정표.xlsx                       내려받지 않고 PC의 파일로 보기
"""
import argparse
import datetime
import io
import re
import sys
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

FILE_ID = '1_vY90Xp4M3X1jWHVP-20tiBJsvhWXnU4'
URL = f'https://drive.google.com/uc?export=download&id={FILE_ID}'
NS = '{http://schemas.openxmlformats.org/spreadsheetml/2006/main}'
REL = '{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id'
SLOTS = ('오전', '오후', '저녁')


class Schedule:
    """Time Line 시트를 읽어 작업 줄과 날짜 칸을 다룬다."""

    def __init__(self, data):
        z = zipfile.ZipFile(io.BytesIO(data))
        shared = []
        if 'xl/sharedStrings.xml' in z.namelist():
            for si in ET.fromstring(z.read('xl/sharedStrings.xml')).iter(NS + 'si'):
                shared.append(''.join(t.text or '' for t in si.iter(NS + 't')))
        wb = ET.fromstring(z.read('xl/workbook.xml'))
        rels = {r.get('Id'): r.get('Target') for r in ET.fromstring(z.read('xl/_rels/workbook.xml.rels'))}
        sheet = next(s for s in wb.iter(NS + 'sheet') if s.get('name') == 'Time Line')
        path = 'xl/' + rels[sheet.get(REL)].lstrip('/').replace('xl/', '', 1)
        self.rows = {}
        for row in ET.fromstring(z.read(path)).iter(NS + 'row'):
            cells = {}
            for c in row.iter(NS + 'c'):
                v = c.find(NS + 'v')
                if c.get('t') == 's' and v is not None:
                    val = shared[int(v.text)]
                elif c.get('t') == 'inlineStr':
                    val = ''.join(t.text or '' for t in c.iter(NS + 't'))
                else:
                    val = v.text if v is not None else ''
                cells[self._col(c.get('r'))] = val
            self.rows[int(row.get('r'))] = cells
        self.slot_cols = self._slot_columns()
        self.tasks = [self._task(r) for r, cells in sorted(self.rows.items())
                      if r >= 5 and re.fullmatch(r'W\d{3}', cells.get(1, ''))]

    @staticmethod
    def _col(ref):
        n = 0
        for ch in re.match(r'[A-Z]+', ref).group():
            n = n * 26 + ord(ch) - 64
        return n

    def _slot_columns(self):
        """열 번호 → ('10/6', '오전') — 3행 날짜(병합 첫 칸만 글자)를 오른쪽으로 이어 준다."""
        day, out = None, {}
        for c in sorted(self.rows.get(4, {})):
            head = self.rows.get(3, {}).get(c, '')
            m = re.match(r'(\d+/\d+)', head)
            if m:
                day = m.group(1)
            if self.rows[4][c] in SLOTS and day:
                out[c] = (day, self.rows[4][c])
        return out

    def _task(self, r):
        cells = self.rows[r]
        slots = [(self.slot_cols[c], cells[c]) for c in sorted(self.slot_cols) if cells.get(c, '') not in ('', '0')]
        return dict(tid=cells.get(1, ''), name=cells.get(2, ''), role=cells.get(3, ''), who=cells.get(4, ''),
                    robot=cells.get(5, ''), status=cells.get(6, ''), slots=slots)

    def select(self, name=None, day=None, robot=False, include_done=False):
        out = []
        for t in self.tasks:
            if name and name not in t['who'] and t['who'] != '전원':
                continue
            if robot and t['robot'] != '사용':
                continue
            if day and not any(d == day for (d, _), _ in t['slots']):
                continue
            if not include_done and t['status'] == '완료':
                continue
            out.append(t)
        return out

    @staticmethod
    def line(t, day=None):
        when = [s for (d, s), _ in t['slots'] if d == day] if day else [f'{d} {s}' for (d, s), _ in t['slots']]
        urgent = ' ★급함' if any(v == '2' and (not day or d == day) for (d, _), v in t['slots']) else ''
        return f"{t['tid']} [{'·'.join(when)}]{urgent} {t['name']}  — {t['who'] or t['role']} · {t['status']}"


def main():
    ap = argparse.ArgumentParser(description='협동2 일정표 보기 (공유 드라이브 정본)')
    ap.add_argument('who', nargs='?', help='이름(예: 황인재) 또는 작업 번호(예: W041)')
    ap.add_argument('--date', help='날짜 M/D (기본: 오늘)')
    ap.add_argument('--all', action='store_true', help='날짜 상관없이 전체(완료 포함)')
    ap.add_argument('--robot', action='store_true', help='로봇 쓰는 작업만')
    ap.add_argument('--file', help='내려받지 않고 이 xlsx를 읽는다')
    a = ap.parse_args()
    try:
        data = open(a.file, 'rb').read() if a.file else urllib.request.urlopen(URL, timeout=30).read()
        sch = Schedule(data)
    except Exception as e:  # 인터넷·공유 설정 문제를 사람에게 알린다
        sys.exit(f'일정표를 못 읽었습니다: {e}\n드라이브 링크 공유가 켜져 있는지, 인터넷이 되는지 확인하세요.')

    if a.who and re.fullmatch(r'W\d{3}', a.who):
        t = next((x for x in sch.tasks if x['tid'] == a.who), None)
        print(sch.line(t) + f"\n  역할: {t['role']} · 로봇: {t['robot'] or '-'}" if t else f'{a.who} 없음')
        return
    today = datetime.date.today()
    day = None if a.all else (a.date or f'{today.month}/{today.day}')
    tasks = sch.select(name=a.who, day=day, robot=a.robot, include_done=a.all)
    if day:  # 그날 오전 → 오후 → 저녁 순서
        tasks.sort(key=lambda t: min(SLOTS.index(s) for (d, s), _ in t['slots'] if d == day))
    title = f"{day or '전체'} {a.who or '모두'}{' · 로봇' if a.robot else ''}"
    if day and not any(d == day for d, _ in sch.slot_cols.values()):
        print(f'{title}: 일정표에 없는 날입니다.')
        return
    print(f'{title} — {len(tasks)}개' + ('' if a.all else ' (완료는 뺌)'))
    for t in tasks:
        print('  ' + sch.line(t, day))
    if not tasks:
        print('  (할 일 없음)')


if __name__ == '__main__':
    main()

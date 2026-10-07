"""pr_verdict.py 의 판정 규칙을 고정한다 — 체크 안 된 'PL 확인' 항목이 있거나 IRD 내용 · d2_interfaces를 바꿨으면 자동 승인하지 않는다(10/7 PL, PR #45).

gh 를 부르지 않게 post · approver · pl_checks 를 바꿔 끼운다(네트워크 없음).
"""
import importlib.util
import json
from pathlib import Path

import pytest

PR_VERDICT_PY = Path(__file__).resolve().parents[1] / '.github' / 'scripts' / 'pr_verdict.py'


@pytest.fixture(scope='module')
def mod():
    """.github/scripts/pr_verdict.py 를 모듈로 읽는다(폴더 이름에 점이 있어 import 문으로는 못 읽는다)."""
    spec = importlib.util.spec_from_file_location('pr_verdict', PR_VERDICT_PY)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


FLAGGED = ['- [ ] 다른 파트 기능에 영향 없음을 확인함 — **IRD 기본 설계 이름 변경이라 PL 확인 필요**',
           '* [ ] 황인재 확인 받기 (robot.yaml 공용 값)']
NOT_FLAGGED = ['- [x] 다른 파트 영향 없음 — PL 확인 받음',
               '- [ ] 다른 파트 기능에 영향 없음을 확인함 (인터페이스를 바꿨으면 받는 파트 확인 받음)',
               '- 같은 run_id 재전송의 중복 저장 처리는 황인재 확인 대기']


@pytest.mark.parametrize('line', FLAGGED)
def test_unchecked_pl_check_is_found(mod, line):
    """체크 안 된 체크박스에 'PL 확인' · '황인재 확인'이 있으면 잡는다."""
    assert mod.find_pl_checks('본문\n' + line + '\n끝') != []


@pytest.mark.parametrize('line', NOT_FLAGGED)
def test_checked_or_plain_lines_are_ignored(mod, line):
    """체크한 줄 · PL 이 없는 체크박스 · 체크박스가 아닌 글은 잡지 않는다."""
    assert mod.find_pl_checks(line) == []


def _verdict(mod, monkeypatch, results, waits):
    """판정 객체를 만들고 gh 부르는 곳을 바꿔 끼운다. 반환: (객체, 남긴 판정 목록)."""
    items = [{'no': n, 'result': r, 'where': '-', 'reason': '-'} for n, r in enumerate(results, 1)]
    for k, v in {'REPO': 'o/r', 'PR': '1', 'AUTHOR': 'someone', 'REVIEW_RESULT': 'success',
                 'REVIEW_JSON': json.dumps({'items': items, 'notes': [], 'summary': 's'})}.items():
        monkeypatch.setenv(k, v)
    v = mod.Verdict()
    posted = []
    monkeypatch.setattr(v, 'approver', lambda: 'hwang-injae')
    monkeypatch.setattr(v, 'pl_checks', lambda: waits)
    monkeypatch.setattr(v, 'ird_waits', lambda: [])
    monkeypatch.setattr(v, 'post', lambda kind, text: posted.append((kind, text)) or True)
    return v, posted


def test_pl_check_holds_approval(mod, monkeypatch):
    """막음이 없어도 PL 확인 항목이 있으면 승인 대신 코멘트를 남긴다."""
    v, posted = _verdict(mod, monkeypatch, ['통과'] * 8, ['PL 확인 필요'])
    v.run()
    assert [k for k, _ in posted] == ['comment'] and 'PL 확인 대기' in posted[0][1]


def test_no_pl_check_approves(mod, monkeypatch):
    """막음도 PL 확인 항목도 없으면 지금처럼 자동 승인한다."""
    v, posted = _verdict(mod, monkeypatch, ['통과'] * 7 + ['해당 없음'], [])
    v.run()
    assert [k for k, _ in posted] == ['approve']


def test_blocked_still_requests_changes(mod, monkeypatch):
    """막음이 있으면 PL 확인 항목과 상관없이 수정 요청이다."""
    v, posted = _verdict(mod, monkeypatch, ['막음'] + ['통과'] * 7, ['PL 확인 필요'])
    v.run()
    assert [k for k, _ in posted] == ['request-changes']


IRD = 'docs/02_인터페이스_IRD_v3_100716.md'


def test_ird_content_change_is_found(mod):
    """IRD 내용(이름 · 칸)이 바뀌면 사람 확인 대상이다."""
    patch = '@@ -52 +52 @@\n-| 블록 번호 `block_id` | `LV1_B001` |\n+| 블록 이름 `block_id` | `LEG_001_01` |'
    assert mod.ird_changes([{'filename': IRD, 'status': 'modified', 'patch': patch}]) == [IRD]


def test_ird_link_stamp_only_is_ignored(mod):
    """다른 문서 이름이 바뀌어 IRD 안 링크의 월일시만 바뀐 것은 내용 변경이 아니다(docver touch)."""
    patch = ('@@ -3 +3 @@\n-[SDD](03_설계_SDD_v3_100714.md) · [05](05_작업분류_파트별_v2_100714.md)\n'
             '+[SDD](03_설계_SDD_v3_100716.md) · [05](05_작업분류_파트별_v2_100719.md)')
    assert mod.ird_changes([{'filename': IRD, 'status': 'modified', 'patch': patch}]) == []


def test_ird_rename_only_and_other_files_are_ignored(mod):
    """IRD 파일 이름만 바뀐 것 · IRD가 아닌 파일은 대상이 아니다."""
    files = [{'filename': IRD, 'status': 'renamed', 'previous_filename': 'docs/02_인터페이스_IRD_v3_100715.md'},
             {'filename': 'docs/03_설계_SDD_v3_100716.md', 'status': 'modified', 'patch': '-a\n+b'}]
    assert mod.ird_changes(files) == []


def test_ird_without_patch_counts_as_change(mod):
    """diff가 너무 커서 patch가 없으면 내용이 바뀐 것으로 본다."""
    assert mod.ird_changes([{'filename': IRD, 'status': 'modified'}]) == [IRD]


def test_ird_change_holds_approval(mod, monkeypatch):
    """막음이 없고 본문 체크도 없어도 IRD를 바꿨으면 승인 대신 코멘트를 남긴다."""
    v, posted = _verdict(mod, monkeypatch, ['통과'] * 8, [])
    monkeypatch.setattr(v, 'ird_waits', lambda: ['IRD(이름 · 칸의 정본)를 바꿈'])
    v.run()
    assert [k for k, _ in posted] == ['comment'] and 'IRD' in posted[0][1]


def test_iface_change_is_found(mod):
    """d2_interfaces 아래 파일은 더함 · 고침 · 지움 · 이름 바꿈 모두 사람 확인 대상이다."""
    files = [{'filename': 'src/d2_interfaces/srv/CheckProgress.srv', 'status': 'modified'},
             {'filename': 'src/d2_interfaces/srv/New.srv', 'status': 'added'},
             {'filename': 'src/d2_task/Old.srv', 'status': 'renamed', 'previous_filename': 'src/d2_interfaces/srv/Old.srv'}]
    assert mod.iface_changes(files) == [f['filename'] for f in files]


def test_other_packages_are_not_iface(mod):
    """다른 패키지 파일은 대상이 아니다."""
    files = [{'filename': 'src/d2_task/d2_task/task_node.py', 'status': 'modified'},
             {'filename': 'docs/d2_interfaces_note.md', 'status': 'added'}]
    assert mod.iface_changes(files) == []

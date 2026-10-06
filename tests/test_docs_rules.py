"""문서 규칙 — docs/ 아래 .md 이름이 팀 규칙 9(이름_v<버전>_<MMDDHH>.md)에 맞고, 저장소 .md 의 상대 링크가 깨지지 않았는지.

깨진 링크는 모두 모아 한 번에 보여 준다. 문서는 사람이 고친다 — 깨진 링크가 나와도 이 시험을 느슨하게 하지 않는다.
"""
import re
import urllib.parse

from conftest import ROOT

DOCS = ROOT / 'docs'
# [글](대상) · ![그림](대상) · (대상 "제목"). 대상에 공백이 있으면 <대상> 으로 감싼 것만 본다
LINK_RE = re.compile(r'!?\[[^\]]*\]\(\s*<?([^()<>\s]+)>?(?:\s+["\'][^"\']*["\'])?\s*\)')
# 링크로 보지 않는 곳: HTML 주석, 울타리 코드 블록, 인라인 코드. 이 구간에 '대상'이 걸치는 링크는 건너뛴다
# (예: 설명으로 쓴 [계속](`start`)). 링크 글자 쪽에 코드가 있는 [`robot.yaml`](경로) 는 대상이 밖에 있으니 검사한다
NOT_LINK_RE = re.compile(r'<!--.*?-->|```.*?```|~~~.*?~~~|`[^`\n]*`', re.S)
SKIP_PREFIX = ('http://', 'https://', 'mailto:', '#')


def broken_links(md):
    """마크다운 파일 하나의 깨진 상대 링크 목록. 입력: Path. 출력: ['파일:줄 -> 대상', ...]. 못 읽는 파일이면 예외."""
    text = md.read_text(encoding='utf-8')
    protected = [(m.start(), m.end()) for m in NOT_LINK_RE.finditer(text)]
    bad = []
    for m in LINK_RE.finditer(text):
        target = m.group(1)
        if target.startswith(SKIP_PREFIX) or any(a < m.end(1) and m.start(1) < b for a, b in protected):
            continue
        path = urllib.parse.unquote(target.split('#', 1)[0])
        if not path:
            continue
        where = (ROOT / path.lstrip('/')) if path.startswith('/') else (md.parent / path)
        if not where.exists():
            line = text.count('\n', 0, m.start()) + 1
            bad.append(f'{md.relative_to(ROOT)}:{line} -> {target}')
    return bad


def test_docs_md_names_follow_rule(pr_check_mod):
    """docs/ 아래 모든 .md(README.md 제외)는 이름_v<버전>_<MMDDHH>.md 다 (pr_check 의 DOC_RE 와 같은 기준)."""
    bad = sorted(str(p.relative_to(ROOT)) for p in DOCS.rglob('*.md')
                 if p.name != 'README.md' and not pr_check_mod.DOC_RE.search(p.name))
    assert not bad, '문서 이름 규칙(팀 규칙 9)에 안 맞음:\n  ' + '\n  '.join(bad)


def test_markdown_links_resolve(repo_files):
    """저장소의 .md 파일 안 상대 링크([..](경로) · ![..](그림))가 가리키는 파일·폴더가 있다. 깨진 것은 모두 모아 보여 준다."""
    files = repo_files(('.md',))
    assert files, '.md 파일을 하나도 못 찾았다 (git 또는 저장소 위치 확인)'
    bad = [b for md in files for b in broken_links(md)]
    assert not bad, f'깨진 링크 {len(bad)}개:\n  ' + '\n  '.join(bad)

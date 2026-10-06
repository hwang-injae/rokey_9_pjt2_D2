"""비밀값·개인 경로가 저장소에 없다 (팀 규칙 4 · 경로 규칙). 공개 저장소라 push 전에 PR 검사와 같은 기준으로 한 번 더 본다.

키 모양은 pr_check.py 의 KEY_RE 를 그대로 쓴다(기준을 두 곳에 두지 않는다).
"""
from conftest import ROOT

TEXT_SUFFIXES = ('.py', '.md', '.yaml', '.yml', '.json', '.txt', '.srv', '.action', '.xml',
                 '.toml', '.cfg', '.sh', '.ini', '.env.example')          # .launch.py 는 .py 에 포함
CODE_SUFFIXES = ('.py', '.yaml', '.yml')                                   # /home/ 를 보는 src/ 파일 (launch 는 .py)


def test_no_api_key_like_strings(repo_files, pr_check_mod):
    """텍스트 파일 어디에도 OpenAI 키 모양(KEY_RE) 문자열이 없다. 걸린 곳은 파일:줄 로 모두 보여 준다."""
    hits = []
    for p in repo_files(TEXT_SUFFIXES):
        for n, line in enumerate(p.read_text(encoding='utf-8', errors='replace').splitlines(), 1):
            if pr_check_mod.KEY_RE.search(line):
                hits.append(f'{p.relative_to(ROOT)}:{n}')
    assert not hits, '키 모양 문자열(팀 규칙 4) — 지우고, 진짜 키면 PL에게 알려 폐기한다:\n  ' + '\n  '.join(hits)


def test_no_secret_files_tracked(repo_files, pr_check_mod):
    """.env(예시 .env.example 제외)·.pem·.key·credentials*·secrets.yaml 같은 비밀 파일이 저장소에 없다."""
    bad = [str(p.relative_to(ROOT)) for p in repo_files()
           if pr_check_mod.SECRET_FILES.search(p.relative_to(ROOT).as_posix()) and not p.name.endswith('.env.example')]
    assert not bad, '비밀 파일(팀 규칙 4): ' + ', '.join(bad)


def test_no_personal_paths_in_src(repo_files):
    """src/ 아래 .py(launch 포함)·.yaml 에 개인 절대 경로 /home/ 가 없다 — get_package_share_directory()·Path(__file__).parent 로."""
    hits = []
    for p in repo_files(CODE_SUFFIXES):
        if p.relative_to(ROOT).parts[0] != 'src':
            continue
        for n, line in enumerate(p.read_text(encoding='utf-8', errors='replace').splitlines(), 1):
            if '/home/' in line:
                hits.append(f'{p.relative_to(ROOT)}:{n}')
    assert not hits, '개인 경로 /home/ :\n  ' + '\n  '.join(hits)

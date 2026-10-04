하루 시작 루틴을 대신 진행한다. 인자 $ARGUMENTS = 내 이름(예: `황인재`), 또는 `이름 W041`처럼 오늘 먼저 할 작업 번호까지. 이름이 없으면 `CLAUDE.local.md`(내 프롬프트)에서 찾고, 그래도 없으면 묻는다.

1. `git status`·`git branch --show-current`로 지금 상태를 본다. 커밋 안 한 변경이 있으면 커밋할지 stash할지 먼저 묻는다.
2. `git fetch origin && git checkout main && git pull --ff-only origin main`. 받은 커밋은 AGENTS.md 1장 방식으로만 본다 — `git log --format='%h %s%n%b' ORIG_HEAD..HEAD`, `git diff --stat ORIG_HEAD..HEAD`. **내 파트 파일이 바뀐 것만** 짚어 준다(전체 diff는 읽지 않는다).
3. `python3 tools/sched.py <이름>`으로 오늘 내 작업(W번호·칸·상태)을 보여 주고, `python3 tools/sched.py --robot`으로 오늘 로봇 순서를 한 줄로 덧붙인다. 일정표 정본은 공유 드라이브이고 이 도구는 로그인 없이 받는다. 못 받으면 PL에게 알리라고 한다.
4. 어떤 작업부터 할지 정한다(인자에 W번호가 있으면 그것). 그 작업의 완료 기준을 `docs/05_작업분류_파트별_*.md`의 내 파트 표에서 찾아 한 줄로 보여 준다. 다른 파트에서 받아야 하는 것(가짜 노드·인터페이스)이 있으면 `docs/02_인터페이스_IRD_*.md`의 해당 줄만 짚는다.
5. 브랜치를 `{GitHub ID 소문자}/{오늘 YYYYMMDD}-{기능}-{영문 kebab 설명}`으로 만든다(`git checkout -b`). merge된 브랜치는 이어 쓰지 않고 새로 판다.
6. 로봇을 쓰는 작업이면 운영 규칙을 한 줄로 상기시킨다: 펜던트를 든 사람·저속·짝과 함께·정지 확인 먼저.
7. 끝으로 "작업을 끝내면 `/wrap W번호`"라고 안내한다.

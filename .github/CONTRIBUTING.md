# GitHub 협업 규칙 (협동2)

팀원 5명이 같은 방식으로 Branch / PR / Issue를 다루기 위한 규칙이다. 팀 전체 규칙(역할·코드·보안·컨테이너·로봇 안전·문서 이름)은 [팀 협업 규칙](../docs/06_팀협업규칙_v1_100414.md)에 있다. 저장소: https://github.com/hwang-injae/rokey_9_pjt2_D2 (공개)

## 1. Branch
```
{이름}/{날짜 YYYYMMDD}-{기능}-{간단설명}   예: injae/20261005-hmi-web-buttons
```
- 설명은 영문 kebab-case. 같은 기능을 여러 날 하면 날짜만 바꿔 새 브랜치.
- `main` 보호(Settings → Branches, 10/4 적용): PR 필수, 승인 1명 이상, Code Owners 승인 필수, 직접 push 금지. 관리자(황인재)만 예외 — 문서 정리용, 코드는 황인재도 PR.
- 이 저장소는 공개 저장소라 위 보호가 실제로 강제된다. 필수 상태 검사로 **'PR 검사 / 문법·main 반영·충돌·규칙 검사'** 를 넣는다.

## 2. Merge
- 작업 중 오전·오후 한 번씩 `git fetch origin` → 많이 뒤처졌으면 `git merge origin/main`.
- PR 전: `git fetch origin && git merge origin/main` 으로 충돌을 내 PC에서 먼저 푼다.
- `Squash and merge`. squash 커밋 메시지는 **'PR 제목 + 본문'**으로 둔다(Settings → General → Pull Requests) — main 기록에 '변경 파일' 목록이 남아 에이전트가 그것만 읽는다.
- merge된 브랜치는 기록으로 남기되(`Automatically delete head branches` 끔) **더 이어서 쓰지 않는다.** 이어서 할 일은 `main`에서 날짜를 바꾼 새 브랜치를 판다.

## 3. 충돌 예방
- 작업 시작 전 오늘 만질 파일·기능을 노션에 적는다. 파트·패키지 단위로 나누면 충돌이 줄어든다.
- 브랜치는 짧게, 커밋은 작게. 하루 넘는 일은 쪼갤 수 있는지 먼저 본다.
- 공통 파일(README, 설정 파일, 인터페이스·메시지, launch, `package.xml`, `CMakeLists.txt`)은 먼저 알리고 바꾼다.
- 진행 중인 일은 Draft PR로 미리 연다.
- `build/`, `install/`, `log/`, `__pycache__/`는 커밋하지 않는다(`git add -f` 금지). 공유 브랜치에서 rebase·force-push 금지.

## 4. Commit 메시지 (변경 파일을 꼭 적는다)
```
feat: MoveIt 실행기에 서기 정지 추가

- 실행 중 '서라'를 받으면 현재 자리 0.3초 궤적을 보냄
변경 파일: src/jenga_motion/jenga_motion/executor.py, config/robot.yaml
영향: 로봇 동작(실행기)
```
| 접두어 | 쓸 때 |
|---|---|
| `feat` | 새 기능 |
| `fix` | 버그 수정 |
| `refactor` | 구조 개선 |
| `chore` | 설정·패키지 구성 |
| `docs` | 문서 |

- `변경 파일:`과 `영향:`은 에이전트가 그 파일만 읽게 하려는 것이다(토큰 절약, `AGENTS.md`). 하루를 마칠 때 `CHANGES.md` 맨 위에 파트별 한 줄 요약을 더한다.
- '수정', 'update', '123' 같은 메시지 금지. 커밋 하나에 작업 하나.

## 5. Pull Request
- 양식(`PULL_REQUEST_TEMPLATE.md`)의 변경 내용·**변경 파일(필수)**·영향·**완료한 일정표 작업**(`완료: W041`)·확인 사항을 채운다.
- **자동 검사·승인:** PR을 올리거나 새로 push하면 `PR 검사`가 돈다(`.github/workflows/pr_check.yml`).
  1. main 반영: 최신 main이 내 브랜치에 merge돼 있는가 → 없으면 실패(`git fetch origin && git merge origin/main` 뒤 다시 push)
  2. 충돌: main과 합칠 때 충돌이 없는가
  3. 문법: 바뀐 .py(컴파일만)·.yaml·.json·.xml/.urdf/.xacro
  4. 비밀값: OpenAI 키 모양 문자열·`.env`·키 파일이 없는가
  5. 변경 파일: PR 본문 '변경 파일' 칸이 채워져 있는가(개별 커밋의 `변경 파일:` 줄은 경고)
  6. 문서 이름: `docs/` 아래 새 문서가 `이름_v<버전>_<MMDDHH>.md`인가
  모두 통과하면 **자동 승인**된다(황인재 계정 — 황인재가 올린 PR은 한세교가 직접 승인). 승인 권한자는 **황인재(@hwang-injae)·한세교(@hansaekyo)**(CODEOWNERS)이고, 사람이 직접 승인·거절할 수도 있다. 자동 승인 계정 본인이 올린 PR은 다른 승인자가 승인한다.
- **작업 완료 알림:** 일정표 작업을 끝냈으면 PR 본문에 `완료: W번호`를 적는다. merge되면 PL의 문서담당 에이전트가 확인해 일정표(드라이브)의 상태를 '완료'로 바꾼다.
- merge 전: Files changed 확인, 충돌 확인, 다른 파트 영향 확인, 키·`.env`·개인 경로 없음 확인.

## 6. Issue
- 할 일 관리가 아니라 트러블슈팅 기록용. 오래 걸린 에러·팀이 알아야 할 버그만.
- 제목 `[문제] 어떤 상황에서 어떤 에러`, 본문은 `ISSUE_TEMPLATE/bug_report.md`. 해결하면 해결 방법을 댓글로 남기고 닫는다.

## 7. 보안
- **OpenAI API 키는 절대 GitHub에 올리지 않는다.** PC별 `.env`에 두고 환경 변수로 읽는다.
- GitHub 토큰·비밀번호·API 키·인증서·개인 정보 금지. 커밋 전 `git status`로 다시 확인.
- 실수로 올렸으면 바로 PL에게 알리고 **그 키를 먼저 폐기**한 뒤 새로 받는다(커밋을 지워도 기록에 남음). 컨테이너에는 `--env-file .env`로 넘긴다.

## 8. 경로
- 코드·launch·설정에 `/home/이름/...`을 쓰지 않는다. `get_package_share_directory()`나 `Path(__file__).parent`를 쓴다.
- 실행 명령은 README에 그대로 복사해 쓸 수 있게 적는다.

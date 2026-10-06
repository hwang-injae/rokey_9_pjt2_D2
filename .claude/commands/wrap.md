작업(또는 하루) 마무리 루틴을 대신 진행한다. 인자 $ARGUMENTS = 끝낸 일정표 작업 번호(여럿이면 쉼표, 예: `W041,W043`). 없으면 묻는다. 덜 끝났으면 `진행:`으로 적는다.
**W번호가 없거나 사용자가 모르면:** `python3 tools/sched.py <이름>`(이름은 `CLAUDE.local.md`에서)으로 오늘 내 작업과 '진행 중'·'지연' 작업을 보고, 바뀐 파일·브랜치 이름·커밋 메시지·`HANDOFF.local.md`와 맞춰 가장 맞는 W번호를 1~3개 골라 이유와 함께 제안한 뒤 확인받는다. 맞는 작업이 없으면 PR 본문에 `W번호 없음 — <한 일>`으로 적고 PL에게 알리라고 한다.
**황인재(10/6 PL — PR 없이 main에 바로 push):** 4~7단계 대신 ① `git fetch origin && git merge origin/main` ② **`python3 .github/scripts/pr_check.py --repo . --base origin/main`를 돌려 통과해야 push한다**(실패면 고친 뒤 다시) ③ 코드면 `.claude/commands/pr-review.md`의 ①~⑧로 스스로 한 번 본다 ④ 커밋 메시지에 `완료: W번호`(덜 끝났으면 `진행: W번호`) 줄을 넣는다 — 문서담당이 커밋 메시지에서 읽어 일정표에 반영한다 ⑤ `CHANGES.md` 한 줄 ⑥ `git push origin main`.
인자가 `메모`이면 커밋·PR 없이 **10단계(메모)만** 한다 — 작업 중간에 큰 주제를 바꿔 새 세션을 열 때 쓴다.

1. `git status`로 바뀐 파일을 보여 준다. `build/ install/ log/`, 영상·rosbag·DB 파일, `.env`, 키 파일이 섞여 있으면 빼고 왜 뺐는지 말한다. diff에 키 모양 문자열(`sk-…`)이나 개인 절대 경로(`/home/…`)가 있으면 멈추고 알린다.
2. 로봇을 움직이는 코드가 바뀌었으면 Ctrl+C·막힘·실패 때 먼저 서는 처리(서기 궤적, `SignalHandlerOptions.NO`)가 있는지 확인한다. 없으면 커밋 전에 알린다. 새로 만들거나 고친 클래스·함수에 설명 주석(docstring — 팀 규칙 10)이 있는지도 보고, 없으면 커밋 전에 넣자고 한다.
3. 커밋 메시지를 AGENTS.md 4장 양식으로 제안한다: `type: 요약` + 이유 2~3줄(필요하면) + `변경 파일: 경로, 경로` + `영향: 파트·묶음`. 사용자가 확인하면 `git add <파일들>`·`git commit`.
4. `git fetch origin && git merge origin/main`. 충돌이 나면 같이 푼다(PR 검사가 main 반영을 확인한다). 그다음 `git push -u origin <현재 브랜치>`.
5. `CHANGES.md` 맨 위 오늘 날짜 아래 내 파트 줄에 한 줄을 적어 같은 PR에 넣는다(날짜가 없으면 맨 위에 새로 만든다).
6. PR 본문을 `.github/PULL_REQUEST_TEMPLATE.md`대로 채운 초안을 만든다: 변경 내용, **변경 파일(필수)**, 영향, **완료한 일정표 작업**(`완료: W041` / `진행: W043`), 체크리스트. 사용자가 확인하면 `gh pr create --base main --title "<type: 요약>" --body-file <초안>`. gh가 없으면 본문을 보여 주고 GitHub 화면에서 만들게 안내한다.
7. `gh pr checks <번호>`로 PR 검사·CI 결과를 본다. 실패하면 Checks에 적힌 이유대로 고쳐 다시 push 한다(CI 시험은 로컬에서 `python3 -m pytest tests -q`로 먼저 돌려 본다). 승인은 황인재·한세교가 직접 한다. Claude 검토가 필요하면 라벨 `claude-review`를 붙인다 — '수정 요청'이면 적힌 파일:줄을 고쳐 다시 push 하고, 자동 승인되면 Squash and merge는 사람이 누른다. merge 뒤 일정표는 문서담당이 `완료:` 줄을 보고 고친다.
8. 로봇 실기 시험이면 결과(성공·실패·걸린 시간·이상 동작)를 PR 본문이나 `docs/test-reports/<ID>_<내용>_<이름>_v1_<MMDDHH>.md`에 남기게 돕는다.
9. 끝으로 팀 공유용 3줄을 만든다: 한 일(W번호) / 막힌 것 / 다음에 할 일.
10. 이어갈 메모를 저장소 맨 위 `HANDOFF.local.md`에 **덮어쓴다**(20줄 이내, `.gitignore`에 있어 올라가지 않는다. 커밋하지 않는다): 날짜·이름·브랜치 / 한 것(W번호·PR 번호) / 하던 중인 것(**W번호**, 어디까지, 다음에 칠 명령·열 파일) / 막힌 것·해 본 것 / 다른 파트에 필요한 것 / 다음 할 일. 쓴 내용을 보여 주고 "이 대화는 닫고, 다음은 새 세션에서 `/today 이름`"이라고 안내한다.

# tests — ROS 없이 도는 자동 시험

저장소 루트에서 `python3 -m pytest tests -q` 로 돈다(개인 venv 면 `~/venvs/<이름>/bin/python -m pytest tests -q`). 도구는 `pip install -r tests/requirements.txt`(pytest · pyyaml · ruff). CI(`.github/workflows/ci.yml`)가 PR(main 대상)과 main push 마다 같은 것과 `src/d2_task/test` · `src/d2_vision/test`(ROS 없이 pytest)를 돌린다 — 토큰(Claude API)은 쓰지 않는다.

| 파일 | 무엇을 지키나 |
|---|---|
| `test_pr_check_rules.py` | `.github/scripts/pr_check.py` 정규식 — 키 모양 · 문서 이름 · 비밀 파일 · 로봇 이동 코드 · 정지 설정 단어 |
| `test_pr_verdict_rules.py` | `.github/scripts/pr_verdict.py` 자동 승인 규칙 — 체크 안 된 'PL 확인' 줄이 있거나 IRD 내용 · `d2_interfaces`를 바꿨으면 자동 승인하지 않음(10/7) |
| `test_docs_rules.py` | `docs/` 문서 이름 규칙(팀 규칙 9) · 저장소 `.md` 의 상대 링크가 깨지지 않음(깨진 것은 모두 모아 보여 줌) |
| `test_robot_yaml.py` | `src/d2_robot/d2_bringup/config/robot.yaml` 필수 키 · 형식 · 공급 칸 6개 · 잡기 폭 순서(닫기 < 폭 < 놓기 열림 ≤ 집기 열림) · 개인 경로 없음 · 같은 키 두 번 없음 |
| `test_interfaces.py` | `d2_interfaces` 의 `.srv`/`.action` 구분선 수 · 칸 줄 형식 · 약속한 칸 이름(IRD 5장) · CMakeLists 등록 |
| `test_motion_math.py` | `d2_motion.motion_math` — 회전 왕복 · TCP 자세 · 잡기 이름 · robot.yaml 공급 칸 자세와 잡기가 맞음 · RG2 손가락 끝 높이 · 집기·놓기 TCP · 레시피(실측 두께로 쌓기) |
| `test_recipe_manager.py` | `src/recipe_manager`(E-52) — 기본 설계 4종을 DXF에서 다시 만들면 저장된 구조 · 조립 · 배치표와 같음 · `structure_sha256` = 구조 파일 바이트 · `cads/`엔 계획 파일 없음 · `recipes/`엔 모형마다 3개만 · 블록 이름 꼴 · 번호 순서 · 겹침 · 책상 아래 · 뜬 블록 · 순서 · 단계 · 잡기 상태 · 빠진 CAD 속성 거부 · 옛 잡기 이름(SIDE_25 · END_75) 변환 · **변환기 ① `BlocksToRecipe`: 4종 블록 JSON → 레시피가 CAD 레시피와 같은 로봇 목표(V-45), 잡기 짧은 쪽 우선, 잘못된 입력 거부** |
| `test_no_secrets.py` | 텍스트 파일에 키 모양 없음 · `.env`/`.pem` 등 비밀 파일 없음 · `src/` 에 `/home/` 경로 없음 |

`conftest.py` 가 저장소 루트(`ROOT`)·`robot.yaml`(`robot_cfg`)·`pr_check.py`(`pr_check_mod`)·저장소 파일 목록(`repo_files`)을 fixture 로 준다. `src/d2_robot/d2_motion` 을 `sys.path` 에 넣어 `d2_motion.motion_math` 를 ROS 없이 import 한다.

## 새 시험을 더할 때

- **ROS 없는 계산·설정·문서 규칙**은 여기 `tests/` 에, 파일 이름은 `test_<무엇>.py`. 파일 맨 위 docstring 에 무엇을 지키는지 한 줄.
- **ROS 노드 시험**은 `src/<패키지>/test/` 에 둔다(`colcon test`). rclpy 없이 도는 `d2_task` · `d2_vision` 시험은 CI가 pytest로 바로 돌린다(`ci.yml`). 그 패키지 `setup.py` 에 `tests_require=['pytest']` 가 있어야 colcon 이 pytest 로 돌린다(없으면 `setup.py test` 로 떨어져 시험이 돌지 않는다). CI 는 **두산 의존이 없는 패키지만** colcon build·test 한다(`ci.yml` 의 `CI_PKGS`) — 두산·OnRobot 패키지가 필요한 `d2_bringup`·`d2_motion`·`d2_gripper`·`d2_safety` 는 실기 PC에서 빌드한다.
- 숫자·좌표는 지어내지 않고 `robot_cfg` fixture 로 `robot.yaml` 을 읽는다. 관계(순서·합·일치)를 본다.
- 키 모양 문자열이 필요하면 파일에 쓰지 않고 실행 때 이어 붙인다(`'sk-' + 'proj-' + 'A' * 30`) — PR 검사가 파일 안의 키 모양을 막는다.
- 저장소 루트에 `pytest.ini`·`pyproject.toml` 을 두지 않는다 — `colcon test` 가 그것을 rootdir 로 잡는다. 실행은 늘 `python3 -m pytest tests`.

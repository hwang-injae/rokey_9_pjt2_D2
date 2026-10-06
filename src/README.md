# src 패키지

ROS 2 Jazzy 패키지 8개(로봇 PC) + 레시피 도구 **(안)** + 저장소 루트 `web/`(웹 PC, ROS 아님). **10/6 18시 PC 배치(E-26~E-33):** 웹 PC에는 ROS가 없고, `d2_hmi` 대신 **`d2_bridge`(ROS ↔ MQTT 다리)** 가 로봇 PC에서 웹 자리를 대신한다. **10/6 주제 개편(E-01~E-25)** — 설계는 블록 JSON으로 오가고 로봇에는 레시피로 간다. 로봇 동작 노드 5개는 10/6 corecode에서 옮김, `d2_interfaces` 7개는 main. 노드는 파트별로 나뉘어 있고, 서로 `/d2/` 아래 토픽·서비스·액션으로 주고받는다. 요청-결과는 전용 메시지 패키지 `d2_interfaces`, 상태 방송은 JSON 문자열이다. 이름·칸·단위 약속은 [인터페이스 문서](../docs/02_인터페이스_IRD_v3_100708.md), 구조는 [설계 문서](../docs/03_설계_SDD_v3_100708.md)에 있다.

| 패키지 (안) | 빌드 타입 | 담당 파트 | 역할 | 노드 (실행 이름, 안) | 돌리는 곳 |
|---|---|---|---|---|---|
| `d2_interfaces` | ament_cmake (rosidl) | 황인재 (W027, 10/6 PL이 로봇 동작에서 옮김) | 전용 메시지 7개 — `PickPlace.action` · `GripperCommand` · `CheckProgress` · `HmiCommand` · `SceneAttach` · `StopRequest` · `MoveTo` (`.srv`, 10/6 NextBlock 빠짐). 노드 없음 | — | 모든 PC |
| `d2_bringup` | ament_python | 로봇 동작 (인프라·통합) | 팀 브링업(박진용 `real_moveit.launch.py`를 옮김, `mode:=virtual`·`real`) + 전체 실행 launch(진짜·가짜 고르기) + 설정 파일 `src/d2_bringup/config/robot.yaml` | — (launch만) | 로봇 PC 호스트 |
| `d2_motion` | ament_python | 로봇 동작 | 블록 1개 집기·놓기 액션 서버(실행기는 같은 프로그램 안 클래스) + MoveIt2 장면을 고치는 유일한 노드 | `pick_place` · `scene_manager` | 로봇 PC 호스트 |
| `d2_gripper` | ament_python | 로봇 동작 (로봇 셀) | RG2를 다루는 유일한 노드. (폭, 힘)을 받아 다 움직인 뒤 폭을 보고 **잡힘까지** 답한다(잡힘 확인을 따로 두지 않음, S-01) | `gripper` | 로봇 PC 호스트 |
| `d2_safety` | ament_python | 로봇 동작 (안전 감시) | 정지 판단 + 제어기에 서기 궤적 + 잠금. 늘 켜 둔다. 1차 정지 입력 = 키·화면 버튼·Ctrl+C·로봇 알람 4개(카메라 끊김 · 음성 '멈춰' 연결은 나중에. 웹캠은 없음 E-19) | `safety_stop` | 로봇 PC 호스트 (고정) |
| `d2_vision` | ament_python | 비전 | 손목 카메라 블록 인식(있음 · 없음 · 높이 · 오차 측정값) + **스캔 추론기**(점군 → 격자 → 블록 JSON, `structure_scanner.py`) + 흩어진 블록 찾기(`find_blocks`, E-36). 웹캠 · 손 찾기는 뺌(E-19) | `wrist_block` · `mock_wrist_block` | 로봇 PC 호스트 |
| `d2_task` | ament_python | 비전 | 작업 관리자(상태표: 조립 · 스캔 · 공급 채우기 · 정지 · 다시 시작 + CSV → `builds`) + 작업 판단(레시피 · 진행표 · 다음 블록 · 집을 블록 고르기(E-36)) + **검사 묶음 `DesignChecker`(`/d2/task/check_design`, 안정성 7 mm · 받침 · 잡기 · 막힌 칸 · 작업영역 → 변환기 ①로 레시피)** + 변환기 ② | `task` · `mock_task` | 로봇 PC 호스트(10/6 E-26) |
| **`d2_bridge`**(새, E-33) | ament_python | HMI(안 — W121 확정, 대안 박진용) | **ROS ↔ MQTT 다리 노드 하나.** 웹 대신 `/d2/hmi/command` · `/d2/task/check_design` · `/d2/safety/stop` · `resume` 호출, `/d2/hmi/get_design` · `save_build` 제공, `/d2/hmi/intent` 발행, 상태 토픽 5개 → MQTT retained, 생존 신호. 규칙은 [IRD 10장](../docs/02_인터페이스_IRD_v3_100708.md#10-pc-사이-통신--mqtt-다리-e-27e-30-안) | `bridge` · `mock_bridge`(옛 `mock_web_ui`) | 로봇 PC 호스트 |
| (ROS 아님) `web/` | — | HMI | **`frontend/`**(Next.js 정적 + three.js, 브라우저에서 실행): 웹 화면(글상자 · 설계 선택 · 출발 · 정지 · 다시 시작 · 스캔 · 3D 미리보기 · 버전 트리) — backend와 REST · WebSocket만. **`backend/`**(FastAPI :8000 · paho-mqtt): REST · WebSocket · 정적 서빙 + 음성(웨이크워드 · Whisper STT · 의도) + **AI 설계 생성 `DesignGenerator`** + **저장소 `DesignStore`(JSON 파일 → DB)**. `web/compose.yaml`(`mosquitto` · `db` · `web`) — 10/7 E-41 | `backend`(frontend 정적 파일 포함) · `voice.py` · `mock_robot.py` | 웹 PC(compose · 호스트) · 화면은 브라우저 |

- 10/6 개편으로 ROS 노드는 **8개**(웹캠 사람 감지 `webcam_human` · 손목 손 찾기 뺌 — `mock_webcam_human.py`는 지우거나 두되 켜지 않는다; 웹 자리는 `d2_bridge`). `web/backend/`에 AI 설계 생성 `design_gen.py` · 저장소 `design_store.py`, `d2_task`에 검사 묶음 `design_checker.py` · 변환기 ② `recipe_to_blocks.py`, `d2_vision`에 스캔 추론기 `structure_scanner.py`가 ROS 없는 계산 파일로 들어간다. `d2_interfaces`에 `JsonQuery.srv`를 더한다(W121). 자세한 것은 [결정 기록 §9](../docs/decisions/결정기록_시나리오_역할_인터페이스_1004_v1_100609.md#9-104-15시-30분-간소화-결정-s-01s-13--대비책).
- 패키지 이름과 실행 이름은 이 문서에서 제안한 **안**이다. 만들 때 담당이 바꿀 수 있고, 바꾸면 이 표와 인터페이스 문서 3장을 같이 고친다.
- 가짜 노드(`mock_*`)는 각 패키지 안에 둔다. 누가 무엇을 만드는지는 [인터페이스 문서 11장](../docs/02_인터페이스_IRD_v3_100708.md)에 있다.
- CAD → 레시피 도구(한세교)는 노드가 아니라 라이브러리다. **변환기 ①(블록 JSON → 레시피, `blocks_to_recipe`)** 도 여기 들어가고 `d2_task`의 `DesignChecker`가 import 해서 쓴다. 지금 위치는 [`src/recipe_manager/`](recipe_manager/)이고 `COLCON_IGNORE`로 빌드에서 뺐다(10/6). 패키지 이름 · 최종 위치는 로봇 동작이 정한다(W110).

## 구조 규칙

- **관련 기능은 클래스로 묶는다(팀 규칙 1).** **노드 하나 = 노드 클래스 하나 = 파일 하나.** 노드가 쓰는 계산 클래스(ROS 없이 도는 것)는 같은 패키지의 다른 파일로 나눠도 된다 — 다른 사람이 맡거나 ROS 없이 시험할 때만(규칙 2, 10/5 S-14). 예: 그리퍼 노드 = `gripper_node.py` 하나, 그 안의 `GripperNode`에 열기·닫기·폭 읽기. 집기·놓기의 실행기는 노드가 아니라 `pick_place` 프로그램이 쓰는 클래스다(인터페이스 문서 3장).
- **일회성 참조·과도한 구조화를 하지 않는다(팀 규칙 2).** 한 번만 쓰는 값·함수를 따로 빼서 여기저기서 참조하게 만들지 않는다. '나중에 쓸지도 모르는' 추상 클래스·계층·설정 단계를 미리 만들지 않는다. 지금 필요한 만큼만 짠다.
- **숫자는 `src/d2_bringup/config/robot.yaml`에.** 좌표·높이·힘·시간은 코드에 쓰지 않고 이 파일 하나에 둔다(보정 값만 따로 파일). 키 이름은 10/4에 정했다 — 인터페이스 문서 9장.
- **경로를 하드코딩하지 않는다.** `/home/이름/...` 같은 개인 경로를 코드·launch·설정에 쓰지 않는다. `get_package_share_directory('d2_bringup')`이나 `Path(__file__).parent`를 쓴다.
- **로봇을 움직이는 코드는 Ctrl+C·막힘·실패 때 서기 정지를 먼저 한다.** rclpy는 `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`로 시작한다. 그래야 Ctrl+C에 ROS가 먼저 꺼지지 않고, 서기 궤적(지금 관절값, 0.3초)을 제어기에 보낸 뒤 끝낼 수 있다. 프로그램이 끝나도 이미 보낸 궤적은 계속 움직이기 때문이다(10/3 가상 시험). 예시는 R-01 정지 수정본의 `init_ros()`·`SafeStop.stop()` — [TS-01](../docs/troubleshooting/TS-01_정지가_안_들음_R-01_v1_100519.md).
- **MoveIt2 장면은 장면 관리 노드(`scene_manager`)만 고친다.** 다른 노드는 `apply_planning_scene`을 부르지 않는다. 쥔 블록은 `/d2/motion/scene/attach`로 부탁한다. 멈출지는 정지 노드, 차례는 작업 관리자, 진행표는 작업 판단이 정한다.
- **약속은 파일이 정본.** `d2_interfaces`의 `.action`·`.srv` 파일이 인터페이스 문서와 다르면 파일이 정본이다. 이름·칸을 바꿀 때는 PR을 올리고 **받는 파트 사람이 확인해야** merge한다. 커밋 `영향:`에 파트를 적고 `CHANGES.md`에 한 줄 남긴다.
- **가짜 노드(`mock_*`).** 보내는 쪽(토픽은 내보내는 쪽, 서비스·액션은 답하는 쪽)이 10/6 오전에 만든다(W034). 진짜와 같은 이름·형식·JSON 칸을 쓰고, 로봇·카메라·마이크 없이 돌며, 실패 코드를 일부러 낼 수 있다. 진짜와 동시에 켜지 않는다.
- **Docker (10/5 S-15 · S-17 → 10/6 E-19 → 10/6 18시 E-31).** 컨테이너는 **웹 PC에만** — `web/compose.yaml`의 `mosquitto`(MQTT 브로커) · `db`(PostgreSQL 16, E-39) · `web`(backend FastAPI + frontend 정적 파일, 안). 셋 다 황인재. 로봇 PC(ROS 노드 전부 + 다리)는 호스트. `hmi` · `db-hmi` · `vision` 컨테이너는 없다. 공통 설정은 [환경 설정 6-1 · 7장](../docs/env/README.md).
- **PC 사이 통신은 MQTT 다리 한 곳(`d2_bridge`).** 로봇 PC의 다른 노드는 MQTT를 모르고, `web/`은 ROS를 모른다. 토픽 · payload 규칙은 IRD 10장 표 하나.
- **키는 `.env`.** OpenAI 키는 PC마다 `.env`에 두고 환경 변수로 읽는다. 코드·설정·커밋에 쓰지 않는다(팀 규칙 4).

## 빌드 · 시험

지금 있는 패키지는 `d2_interfaces` · `d2_bringup` · `d2_motion` · `d2_gripper` · `d2_safety` · `d2_vision` · `d2_task`다(`d2_bridge`는 아직 없음). 아래처럼 빌드한다.

```bash
# 저장소 루트(rokey_9_pjt2_D2)에서
source /opt/ros/jazzy/setup.bash
source <두산 워크스페이스>/install/setup.bash      # 두산 드라이버·MoveIt 설정 (경로는 PC마다 다름)
colcon build --symlink-install
source install/setup.bash

# 메시지 패키지만 먼저 (10/4 오후)
colcon build --packages-select d2_interfaces
ros2 interface show d2_interfaces/action/PickPlace

# 자동 시험 — ROS 패키지
colcon test
colcon test-result --verbose

# 자동 시험 — ROS 없는 공통 시험(저장소 루트 tests/, CI가 PR마다 돈다)
python3 -m pytest tests -q
```

- **CI**(`.github/workflows/ci.yml`, 토큰 없음): PR · main push마다 `tests/` pytest + `ros:jazzy` 컨테이너에서 두산 의존이 없는 패키지(`d2_interfaces` · `d2_vision` → 생기면 `d2_task` · `d2_bridge`)만 colcon build · test + ruff 치명 오류. 두산 · OnRobot 패키지가 필요한 `d2_bringup` · `d2_motion` · `d2_gripper` · `d2_safety`는 실기 PC에서 빌드한다. 시험을 더하는 규칙은 [tests/README.md](../tests/README.md).
- 시험 수·결과는 코드가 생기면 이 절에 적는다.
- 로봇을 움직이는 시험은 펜던트를 든 사람이 있을 때, 저속·짝과 함께, 정지가 되는지부터 본다([팀 협업 규칙](../docs/06_팀협업규칙_v1_100708.md)).

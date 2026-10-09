# 아키텍처 질문 답변 — JsonQuery · 서비스 vs 액션 · MQTT 인터페이스

> 2026-10-07 · 황인재(HMI) 질문 정리(10/9 링크만 지금 판으로 — 내용은 10/7 기준, 바뀐 것은 IRD가 맞다). 이름 · 칸의 정본은 [IRD](02_인터페이스_IRD_v3_100917.md)다(이 문서와 다르면 IRD가 맞다).
> 근거: IRD 1장 원칙 2 · 4장 · 10장, [결정 기록 10/4](decisions/결정기록_시나리오_역할_인터페이스_1004_v1_100821.md), [SDD](03_설계_SDD_v3_100917.md), `src/d2_interfaces/srv/JsonQuery.srv`, 그림 [시스템 아키텍처 v6](images/시스템아키텍처_v6_100902.png)(drawio 정본: [드라이브](https://drive.google.com/file/d/1CvG9c2oQh0LLbekSjJwvdxX5RHRqAyMf/view?usp=drive_link)), 비교 저장소 https://github.com/rokey-c2/cobot3-ws-c2 (10/4 커밋 `4f3aa66`).

## 1. JsonQuery는 왜 있나

`JsonQuery`는 **JSON 글자 하나를 보내고 JSON 글자 하나를 받는 공용 서비스**다.

```text
string request_json      # 요청 JSON 글자
---
bool success
string reason            # IRD 7장 실패 코드
string response_json     # 응답 JSON 글자, 맨 앞 "schema": "<이름>/1"
```

쓰는 곳 6개: `check_design` · `get_design` · `save_build` · `scan_capture` · `scan_infer` · `find_blocks`.

### 만든 이유

1. **주고받는 내용이 크고 자주 바뀐다.** 블록 JSON · 레시피 · 검사 결과 · `find_blocks` 블록 목록은 여러 겹으로 들어 있고 길이도 매번 다르다. `.srv` 칸으로 하나하나 적으면 칸이 바뀔 때마다 `d2_interfaces`를 모든 PC에서 다시 빌드해야 한다. JSON 글자로 두면 빌드 없이 고친다.
2. **팀 규칙 ② (과도한 구조화 금지).** 비슷한 srv 6개를 따로 만들지 않고 하나로 같이 쓴다.
3. **MQTT 다리가 단순해진다.** 웹 PC에는 ROS가 없어서 MQTT로는 어차피 JSON이 오간다. 다리는 `request_json` 안의 객체에 `req_id`만 더해 그대로 넘긴다. 칸이 바뀌어도 다리 코드는 대개 그대로다(IRD 10.1).
4. **그래도 '묻고 답하기'는 지킨다.** 토픽이 아니라 서비스라서 답이 오는지 · 실패 코드(`success` / `reason`) · 시간 제한(5초)을 ROS가 처리한다.

**대신 포기한 것:** 빌드할 때 칸을 검사하지 못한다. 그래서 JSON 맨 앞에 `"schema": "이름/1"`을 붙이고 시험(`tests/`)으로 막는다. 로봇을 움직이는 요청은 틀리면 위험해서 전용 메시지로 남겼다(`PickPlace` · `MoveTo` · `GripperCommand` · `StopRequest`). 10/4 결정 원칙이 **"자주 바뀌는 건 JSON, 틀리면 위험한 건 전용 메시지"** 다.

### 다른 팀 저장소(cobot3-ws-c2)와 다른 점

저장소를 받아 아키텍처 그림(SVG)의 글자와 코드를 직접 확인했다.

| | 그 팀 | 우리 |
|---|---|---|
| JSON을 담는 곳 | **`std_msgs/String` 토픽 두 개를 짝으로** 쓴다(명령 토픽 + 상태 토픽). 예: `/arm_a/pick_place_command` ↔ `/arm_a/pick_place_status`, `/amr_a/lift_command` ↔ `lift_state` · `lift_result` | **서비스 하나**(`JsonQuery`)에 요청과 응답을 같이 담는다 |
| 전용 메시지 | `PickPlace.action` · `LocateBox.srv` · `ConfirmGrasp.srv` · `SetRoute.srv`가 있다. 그림의 주 흐름은 String 토픽이고, 그림 안 PickPlace 서버에 "Action ↔ String"이라고 적혀 있다 | 로봇을 움직이는 건 전용 메시지, JSON 덩어리는 `JsonQuery` |
| MQTT 다리 | **어댑터 4개**를 기능별로 나눴다(이동 · 리프트 / 수동 속도 / 공정 / 자세 동기화). 토픽 이름도 따로 지었다(`controltower/command/amr/{코드}/navigate` 등) | **다리 1개**(`d2_bridge`). 토픽 이름은 ROS 이름에서 맨 앞 `/`만 뺀 것(`d2/…`)으로 정해져 있다 |

두 팀 다 JSON 글자를 보낸다. 차이는 **무엇에 담느냐**다.

- 토픽 짝으로 하면 보낸 요청과 돌아온 답을 직접 짝지어야 하고 시간 제한도 직접 만들어야 한다. 대신 오래 걸리는 일의 상태를 계속 받기에는 자연스럽다.
- 그 팀은 Isaac Sim 쪽과 주로 토픽으로 잇다 보니 이렇게 된 것으로 보인다(그림만 보고 한 추측).
- 우리는 시뮬레이터 없이 짧은 묻고 답하기가 많아서 서비스로 했다.

## 2. 액션이 거의 없고 서비스만 쓰는 이유

액션은 `/d2/motion/pick_place` 하나뿐이다. 1~3은 문서에 근거가 있고, 4는 문서에 따로 적혀 있지 않은 추론이다.

1. **오래 걸리고 중간 보고가 필요한 일이 그것뿐이다.** 블록 1개 집기 · 놓기는 최대 90초, 중간에 `approach → grasp → lift → move → place → retreat` 단계를 보고한다(결정 기록 10/4 '블록 1개 전체 액션', SDD).
2. **나머지는 짧다.** 검사 3초 목표, 그리퍼는 다 움직인 뒤 답, 장면 붙이기 · 진행 확인 · 정지 · 다시 시작도 금방 끝난다. 이런 일에 액션(목표 · 중간 보고 · 취소 · 결과)을 쓰면 코드만 는다(팀 규칙 ②).
3. **멈추는 길이 '액션 취소'가 아니다.** 정지 노드가 서기 궤적을 **토픽** `/dsr_moveit_controller/joint_trajectory`로 제어기에 직접 보낸다. 실기에서 `FollowJointTrajectory` 액션으로는 서지 않았다(10/4 R-01, 10/6 박진용). 액션을 쓰는 가장 큰 이유인 취소가 우리에게는 해당이 안 된다.
4. **(추론) MQTT 다리는 `req` / `res` 한 쌍만 다룬다.** 액션을 MQTT로 넘기려면 목표 · 중간 보고 · 취소 · 결과 토픽을 다 만들어야 한다. 웹과 로봇 사이에 오가는 건 모두 짧은 요청이라 서비스로 충분하다.

`move_to`는 최대 30초 걸리지만 서비스다. 중간 보고가 필요 없고 멈추는 건 정지 노드가 하니까 서비스로 둔 것으로 보인다.

> ⚠️ 확인할 점: `scan_infer`(점군 추론)는 ROS 서비스 시간 제한 5초 안에 안 끝날 수 있다. 실제로 재 보고 넘으면 robot.yaml `timeout.*` 값을 늘리는 쪽으로 비전 파트와 맞춘다.

## 3. MQTT로 오가는 인터페이스 전부 (IRD 10장, 10/7 W121 임시 정함)

규칙: 토픽 = ROS 이름에서 맨 앞 `/`만 뺀다. 상태는 payload를 그대로 보낸다. 서비스는 `…/req` → `…/res` + `req_id`(UUID4), 5초(`mqtt.req_timeout_s`) 안에 답이 없으면 `TIMEOUT`. 다리는 변환만 하고 판단하지 않는다.

### 3.1 로봇 → 웹: 상태 방송

retained라서 화면이 늦게 붙어도 마지막 값을 받는다(ROS의 TRANSIENT_LOCAL 자리).

| MQTT 토픽 | ROS 이름 | 내용 | QoS |
|---|---|---|---|
| `d2/task/state` | `/d2/task/state` | `state/1` | 1 |
| `d2/task/progress` | `/d2/task/progress` | `progress/1` | 1 |
| `d2/task/scan_result` | `/d2/task/scan_result` | `scan_result/1` | 1 |
| `d2/safety/state` | `/d2/safety/state` | `safety_state/1` | 1 |
| `d2/gripper/state` | `/d2/gripper/state` | `gripper_state/1` | 0 |

### 3.2 웹 → 로봇: 토픽

| MQTT 토픽 | ROS 이름 | 내용 | retained |
|---|---|---|---|
| `d2/hmi/intent` | `/d2/hmi/intent` | 음성 `intent/1` — 다리가 ROS 토픽으로 낸다 | ✗ |

### 3.3 웹이 부르는 서비스

| MQTT | ROS 서비스 | 요청 → 응답 |
|---|---|---|
| `d2/hmi/command/req` · `res` | `/d2/hmi/command` (HmiCommand) | `{"req_id","cmd","design_id","mode"}` → `{"req_id","success","reason"}` |
| `d2/safety/stop/req` · `res` | `/d2/safety/stop` (StopRequest) | `{"req_id","source","reason"}` → `{"req_id","success","message"}` |
| `d2/safety/resume/req` · `res` | `/d2/safety/resume` (std_srvs/Trigger) | `{"req_id"}` → `{"req_id","success","message"}` |
| `d2/task/check_design/req` · `res` | `/d2/task/check_design` (JsonQuery) | `blocks/1` + `req_id` → `check_result/1` + `req_id` · `success` · `reason` |

### 3.4 로봇이 부르는 서비스 (웹 backend가 답한다)

| MQTT | ROS 서비스 | 요청 → 응답 |
|---|---|---|
| `d2/hmi/get_design/req` · `res` | `/d2/hmi/get_design` (JsonQuery) | `{"req_id","design_id"}` → `design/1` + `req_id` · `success` · `reason` |
| `d2/hmi/save_build/req` · `res` | `/d2/hmi/save_build` (JsonQuery, 안 ⓐ) | builds 칸 + `req_id` → `{"req_id","success","reason"}` |

### 3.5 생존 신호 (1초마다, retained, QoS 1)

| MQTT 토픽 | 방향 | 끊기면 |
|---|---|---|
| `d2/bridge/alive` | 로봇 → 웹 (LWT `{"alive":false}`) | 화면에 "로봇 PC 연결 끊김" 크게 표시, [출발] · [스캔] 막음, 정지는 키 · 펜던트 안내 |
| `d2/web/alive` → ROS `/d2/hmi/alive` | 웹 → 로봇 | 작업 관리자: 조립 중이면 지금 블록까지 놓고 WAIT(`hmi_lost`), IDLE이면 그대로. 정지 아님 |

### 3.6 아직 '안'

| MQTT 토픽 | 방향 | 내용 |
|---|---|---|
| `d2/vision/scan_image` | 로봇 → 웹 | JPEG 바이트(≤ 500 KB) — 다리가 `scan_result`의 경로를 읽어 보냄 |
| `d2/vision/scan_cloud` | 로봇 → 웹 | 다운샘플 PLY 바이트(복셀 3 mm, ≤ 2 MB) |

크기 상한 · 다운샘플 간격은 10/10 비전 + HMI가 정한다(IRD 12장).

### 3.7 MQTT로 안 넘어가는 것

- **로봇 PC 안에서만:** `pick_place` · `move_to` · `check_progress` · `gripper/command` · `scene/attach` · `scan_capture` · `scan_infer` · `find_blocks` · `camera_status` · `path_markers`.
- **웹 PC 안에서만:** 브라우저(frontend) ↔ backend는 REST(`/api/designs` · `/api/robot`) + WebSocket `/ws`. 브라우저는 브로커에 직접 붙지 않는다(E-41).

> 참고: `src/d2_bridge`는 10/7 현재 저장소에 아직 없다. 위 표는 IRD에 적힌 약속이고, 다리 코드(W127)는 이 표대로 만든다.

# web — HMI 설계 노트 (10/8 · 10/9 황인재, PL 결정)

> 정본은 [IRD](../docs/02_인터페이스_IRD_v3_100916.md) 2 · 4 · 6 · 8 · 10장과 [SDD](../docs/03_설계_SDD_v3_100916.md) 3.1.1 · 6.6 · 6.8 · 6.10장, 결정 E-67 · E-69 · E-70 ~ E-77이다. 이 노트는 **웹 안**(화면 · REST · WebSocket · 저장소 · AI 흐름)을 코드로 옮길 때의 세부만 적는다. ROS · MQTT 이름 · 칸은 IRD가 정본이고(E-57 '정함') 여기서 새로 정하지 않는다 — 더할 것은 8장에 모아 PL 확인 PR로 올린다. 파일 이름은 SDD 3.1.1을 따른다. [backend/README.md](backend/README.md)는 10/7 판이라 템플릿 길 · `tune_system` · `base_designs/`가 옛것(E-69)이다 — 지금 기준은 이 노트 4장.

구조(E-32 · E-41): 브라우저 `frontend`(Next.js 정적 `out/`, three.js) ↔ REST + WebSocket `/ws` ↔ `backend`(FastAPI :8000, paho-mqtt) ↔ MQTT 브로커 ↔ 로봇 PC `d2_bridge`. 브라우저는 브로커 · DB · OpenAI · ROS를 모른다. DB는 10/12까지 JSON 파일 폴더, 그 뒤 PostgreSQL(W088).

## 1. 화면 — 페이지 3개 + 공통 띠 (SDD 3.1.1 `app/`)

| 어디 | 무엇 | 쓰는 데이터 | 언제 |
|---|---|---|---|
| **공통 띠**(모든 페이지 위, `ConnectionBadge` · `RobotPanel`) | ① 상태 알림 줄: `state` 한글 + `message` + 정지 때 **누가 · 왜**(`safety_state.reason` = `STOP_WEB` · `STOP_KEY` · `STOP_TASK` · `ROBOT_ALARM:<상태>` · `CTRL_C`, E-62) ② **로봇 PC 끊김 배너**(`bridge/alive` 3초 없음 → 크게, [출발] · [스캔] 막음, "정지는 키 · 펜던트") ③ 그리퍼 표시(폭 · 잡힘) ④ 음성 의도 표시("음성: 출발") ⑤ [정지] · [다시 시작] — 늘 보임 | `task/state` · `safety/state` · `bridge/alive` · `gripper/state` · `hmi/intent` | W047 |
| **메인** `app/page.tsx` | ⑥ 설계 목록 · 트리(`DesignTree` — 기본 4개 → 파생본, `parent_id` · `version` · `made_by`) → 1개 고르면 ⑦ 3D 미리보기(`Preview3D`, E-40) ⑧ **진행도**: 놓은 블록 / 전체 = %, 3D에서 놓인 블록 초록 · 다음 블록 노랑 · 없는 블록 빨강 ⑨ 버튼 [설계 선택] [출발] [계속] [스캔] [취소] — 켜짐은 3장 ⑩ 글상자 + [생성](`RequestBox` — 음성 `request_design` 문장이 오면 채우고 사람이 [생성], E-07) ⑪ 생성 중 → **"디자인을 고르세요"**(후보 3개 3D 나란히 + 검사 결과 · `min_margin_mm`, 불합격 회색) → **"설계도 만드는 중"** → "저장됨 · 출발을 누르세요" / `GEN_FAILED` 이유 + 기본 설계 권함 / `OUT_OF_SCOPE` 안내 + "AI가 참고한 설계"(E-72) ⑫ 상태 로그 패널(시각 · state · message) | `design/2.0.blocks` · `task/progress` · `/ws` 생성 이벤트 | ⑥~⑨ W047 · ⑩ ⑪ W112 · ⑫ 10/11 |
| **설계 상세** `app/designs/page.tsx?id=…` | 미리보기 · 검사 결과 · 버전 트리 · 조립 기록(builds: 결과 · 놓은 수/전체 · 시간 · 정지 횟수 · 블록별 dz) | `/api/designs` | W112 |
| **스캔 비교** `app/scan/page.tsx`(`SCAN_REVIEW`일 때 자동, `ScanCompare`) | 실물 사진(`scan_image`) ↔ 추론 설계 3D(`inferred` 블록 반투명) + 추정 블록 수 · **점군 창**(`scan_cloud` PLY, 닫을 수 있음, E-35 — `cloud_path`가 없거나 빈 글자면 점군 창만 안 띄움) · 버튼 [그대로 저장] [AI로 고치기] [다시 스캔] [취소] | `task/scan_result` · `vision/scan_image` · `vision/scan_cloud` | W116 |
| 메인 안(나중) | **손목 카메라 검출 화면** — 비전이 YOLO-seg 결과를 그려 보낸 JPEG를 그대로(겹쳐 그리기 없음) | `vision/wrist_image` | W150 뒤 |

- **설계 상세는 `designs/[id]`가 아니라 `designs?id=…`:** Next.js 정적 내보내기(`output: 'export'`)는 동적 경로를 빌드 때 아는 ID만 만든다 — AI가 새로 만든 설계가 열리지 않는다(SDD 3.1.1 고칠 것, 8장).
- 3D 블록 크기 = `robot.yaml` `block_size_m`(backend가 `GET /api/designs/rules`로 줌 — 10/9 PL 결정 ①). `blocks/2.0`(저장 설계) · `blocks/1`(스캔 추론, 위치 · 방향 + `inferred`) 둘 다 같은 칸(`x` · `y` · `z` · `ori`)으로 같은 `Preview3D`가 그린다.
- 진행도 색은 `progress/1.1.blocks[].state`(`present` 초록 · `absent` 빨강) + 다음 블록(`state/1.block_id`) 노랑. **판정은 하지 않는다**(dx · dy는 측정값만 — 지금은 NaN). % = `present` 수 / 설계 블록 수, `DONE`이면 100 %.
- 카메라 표시기는 넣지 않는다 — `camera_status`는 MQTT로 넘기지 않는다(IRD 10.1).

## 2. 기능 ↔ 통신 대응표

브라우저 → REST → backend → MQTT(`…/req`) → 다리 → ROS 서비스 → 답이 거꾸로. 상태는 MQTT retained → backend → `/ws`로 브라우저에 밀어 준다. REST는 SDD 3.1.1 `routes/` 세 파일로 나눈다(`robot.py` · `designs.py` · `ws.py`).

| 화면 기능 | REST(backend) | MQTT(IRD 10.1) | ROS(IRD 4장) |
|---|---|---|---|
| 설계 선택 | `POST /api/robot/command {cmd: select_design, design_id}` | `d2/hmi/command/req` → `/res` | `/d2/hmi/command` |
| 출발 · [계속] | 같음 `{cmd: start, design_id, mode: auto}` | 같음 | 같음(`WAIT_SUPPLY` · `WAIT_HMI`에서 start = 계속) |
| 스캔 · 다시 스캔 | 같음 `{cmd: scan}` | 같음 | 같음 |
| 취소 | 같음 `{cmd: cancel}` | 같음 | 같음(`READY` · `ERROR` · `SCAN_REVIEW`) |
| 정지 | `POST /api/robot/stop` | `d2/safety/stop/req` (`source: web`, **reason 빈 값** → 정지 노드가 `STOP_WEB`) | `/d2/safety/stop` |
| 다시 시작 | `POST /api/robot/resume` | `d2/safety/resume/req` | `/d2/safety/resume` |
| 상태 · 진행도 · 정지 · 그리퍼 · 스캔 결과 | `GET /api/robot/state`(지금 값) + `/ws` | `d2/task/state` · `progress` · `scan_result` · `d2/safety/state` · `d2/gripper/state`(retained) | 토픽 |
| 로봇 PC 끊김 | `/ws` `{type: bridge_alive, alive}` | `d2/bridge/alive`(LWT `alive: false`) | — |
| 웹 연결 신호 | backend가 1초마다 냄 | `d2/web/alive` | 다리 → `/d2/hmi/alive` |
| 음성 문장 · 출발 | `/ws` `{type: intent, …}` | `d2/hmi/intent`(voice.py → 브로커, backend도 구독) | 다리 → `/d2/hmi/intent` |
| 설계 규칙 숫자 | `GET /api/designs/rules` | — | — (backend가 `robot.yaml`을 읽음, 10/9 ①) |
| 설계 생성 | `POST /api/designs/generate {text}` → `{job_id}`, 진행은 `/ws` `{type: gen, job_id, stage, candidates, reference}` | 검사만 `d2/task/check_design/req` × 3 | `/d2/task/check_design` |
| 후보 고르기 → 저장 | `POST /api/designs/generate/{job_id}/pick {index}` → `design/2.0` | — (검사 결과를 이미 받아 둠) | — |
| 설계 목록 · 트리 · 1개 | `GET /api/designs` · `GET /api/designs/tree` · `GET /api/designs/{id}` | — | — |
| 조립 기록 | `GET /api/designs/{id}/builds` | — | — |
| 스캔 [그대로 저장] | `POST /api/designs/from_scan {run_id}` → 4-C → check_design → 저장(`made_by scan`) → `select_design` | `check_design/req` · `hmi/command/req` | 같음 |
| 스캔 [AI로 고치기] | `POST /api/designs/generate {text, parent_id: <스캔 설계>}` — 4-B | 검사만 | 같음 |
| 사진 · 점군 | `GET /api/robot/scan/{run_id}/image.jpg` · `/cloud.ply`(backend가 MQTT로 받아 파일로 둠) | `d2/vision/scan_image` · `scan_cloud` | 다리가 파일을 읽어 보냄 |
| 손목 검출 화면 | `GET /api/robot/wrist.jpg`(1~2 Hz) | `d2/vision/wrist_image` | `/d2/vision/wrist_image` |
| (로봇이 부름) 설계 꺼내기 · 결과 저장 | backend가 답함 | `d2/hmi/get_design/req` → `/res` · `d2/hmi/save_build/req` → `/res` | `/d2/hmi/get_design` · `save_build` |

- 요청 시간 제한 5초(IRD `mqtt.req_timeout_s`). **자동 재전송은 어디에도 없다** — 시간 초과 · `BUSY` · 웹 연결 신호 없음 거절(`success: false` + reason 빈 값, #83) 모두 사람이 다시 누른다. 실제 상태는 `state/1` retained로 보여 준다. 버튼은 응답이나 시간 초과 뒤 바로 풀린다.
- `get_design`은 작업 관리자 `timeout.service_s` 3초 안에 답해야 한다 → backend는 파일 폴더에서 바로 읽는다(LLM · 검사 호출 없음). `save_build`는 같은 `run_id`가 또 오면 한 번만 저장하고 `ok: true`.

## 3. 상태별 버튼 (SDD 5.1 상태표)

| `state/1.state` | 켜지는 버튼 | 화면 글 |
|---|---|---|
| `IDLE` | 설계 선택 · 스캔 · 생성 | 설계를 고르세요 |
| `READY` | 출발 · 취소 · 설계 선택 | 출발을 누르세요 — "사람이 영역 밖인지 확인하고 누르세요" |
| `CHECK` · `SELECT` · `PICK_PLACE` · `VERIFY` | (정지) | 조립 중 n/N (%) |
| `WAIT_SUPPLY` | 계속(= start) | `supply_empty` · `tilted_block` · `no_match_block`("맞는 블록이 없어요. 블록을 정리하거나 더 넣고 [계속]", E-73) |
| `WAIT_HMI` | 계속(= start) | 웹 연결이 끊겼었어요. [계속]을 누르세요(`hmi_lost`) |
| `SCAN_MOVE` · `SCAN_CAPTURE` · `SCAN_INFER` | (정지) | 스캔 중(`scan_running`) |
| `SCAN_REVIEW` | 그대로 저장 · AI로 고치기 · 다시 스캔 · 취소 | 비교 화면(`scan_review`) |
| `STOPPED` | 다시 시작 | 멈췄어요 — `reason` + "로봇 작업 영역에서 손을 빼고 누르세요" |
| `RECOVER` | (정지) | 다시 시작 중(저속) |
| `ERROR` | 다시 시작 · 취소 | 오류 — `message`(그리퍼 응답 없음 · 피드백 없음도 여기, E-74 · E-77) |
| `DONE` | 설계 선택 · 스캔 · 생성 | 완성(`done`) · 100 % |
| 로봇 PC 끊김(어느 상태나) | 정지만(못 가면 키 · 펜던트 안내) | 배너 |

[정지]는 어느 상태에서나 켜져 있다(크고 빨강).

## 4. AI 흐름 (E-67 · E-69 · E-72, 10/9 PL)

### 4-A. 생성 — 후보 3개 → 사람이 1개

```text
①  글상자 [생성]  또는  음성 request_design → 글상자에 문장 → 사람이 [생성]          화면 "디자인 만드는 중"
②  RAG(E-72): store.list_designs 요약(design_id · family · version · parent_id · made_by · 블록 수 · 외곽 · 최근 builds 한 줄)을 주고
    GPT-4o에 도구 get_design(design_id) 하나(function calling, 최대 2번) → 읽은 설계 = 예시 · 새 설계의 parent_id(둘이면 처음 것)
    → 화면 "AI가 참고한 설계: 002_CHAIR_BACK v1.2 — 조립 성공, 오차 2.1 mm". 도구를 안 부르거나 실패하면 examples_for(family)
③  GPT-4o 1번 호출 → 후보 3개(blocks/2.0 — 블록마다 order · x · y · z · ori + role · part · stage · grasp, 구조화 출력 candidates[3])
    프롬프트: 역할 · 옵션 목록(src/d2_task/d2_task/roles.json) · 잡기 규칙(SDD 6.6) · 설계 규칙 숫자(robot.yaml — 10/9 ①) · RAG 예시
    칸이 빠지거나 틀리면 check_design이 CHECK_FAILED — 코드가 대신 채우지 않음(E-69 ②). OUT_OF_SCOPE → 안내, 끝
④  화면에 3개 3D + "디자인을 고르세요" — 동시에 check_design × 3(차례로, 각 3초 목표) → 오는 대로 합격 · min_margin / 불합격 회색 + detail
⑤  3개 다 불합격 → errors[].detail 피드백으로 ③부터(최대 2번) → 그래도 실패 GEN_FAILED(이유 + 기본 설계 권함)
⑥  사람이 1개 고름 → "설계도 만드는 중" → 그 후보의 check_result/2.0(recipe · placements)로 save_design
    (아직 검사 중이면 기다림) → 고른 1개만 designs · 나머지 2개는 rejected 기록(NFR-17) → "저장됨 · 출발을 누르세요"
⑦  [설계 선택] → hmi/command select_design → 작업 관리자가 get_design → READY → [출발]
```

- 시간: LLM(≤ 30초 — 후보 3개 × 칸 9개라 늘 수 있음, W108에서 잼) + 검사 3번(≈ 10초) → 60초 목표. 후보는 backend 메모리(`job_id`)에만 — 새로고침하면 사라져도 된다(저장 전).
- `design_id` = `<family>_v<version>`, 블록 이름의 설계 ID는 대문자(E-60). 저장 설계는 바뀌지 않는다(E-55 — 고치면 새 버전). `gen_path` 칸 없음(E-71).
- 검사 요청 · 응답은 IRD 그대로(`blocks/2.0`만 → `check_result/2.0`). **형식 버전 규칙(IRD 6장):** 앞자리 같으면 받고 모르는 칸 무시, 다르면 거절(이유에 받은 schema). 형식 이름 상수는 `design_store.py` 한 곳. 옛 `cad_structure/1.0` · `cad_recipe/1.0` · `blocks/1` 설계는 거절(변환 안 함, E-69 ⑧).

### 4-B. 스캔 → [AI로 고치기] (W125 챌린지, 10/9 PL ③)

스캔 비교 화면의 **[AI로 고치기]를 누를 때만** 돈다(스캔마다 자동으로 돌리지 않음 — 핵심 W116 복제가 GPT 시간 · 비용에 묶이지 않게). 스캔 원본을 먼저 4-C로 저장한 뒤 그 설계를 부모로 4-A를 그대로 돈다(요청 문장은 비어도 됨 — "검사에 맞게 고쳐 줘" · "한 층 높여 줘"). 화면 = 스캔 원본 3D 1개 + 후보 3개. 고른 1개는 `made_by web` · `parent_id` = 스캔 설계. 스캔 원본이 검사에 떨어졌을 때도 같은 길(`detail` 피드백 → 후보 3개).

### 4-C. 스캔 → [그대로 저장] (IRD 8.4 ⑦, 10/9 PL ②)

```text
scan_result/1.x 의 blocks/1(위치 · 방향 + inferred, family = 추론기 값) + nearest_base(추론기가 고른 기본 설계, 선택 칸 — 8장)
→ design_gen이 GPT-4o 1번 호출: 4-A ②와 같은 RAG(DB 목록 요약 + 도구 get_design) → 가장 비슷한 설계를 골라 읽음
   → 그 설계를 참고해 role · part · order · stage · grasp만 채움(inferred는 '추측' 힌트). 블록 위치 · 방향은 바꾸지 않는다(복제가 목적)
   → 고른 설계 = parent_id, family = GPT 판단
→ 코드 규칙: 부모는 같은 family만 — 다르거나 도구 실패면 nearest_base(추론기 값)를 부모로, family도 그것을 따름
→ blocks/2.0 → check_design → 저장(made_by scan) → select_design → READY
```

## 5. backend 파일 (SDD 3.1.1)

```
web/backend/
├── app.py            WebApp — 앱 만들기 · routes 등록 · frontend/out 정적 서빙 · 시작 점검(브로커 · DB · 키)
│                     · robot.yaml 설계 규칙 키 읽기(한 곳, 읽기 전용 — 10/9 ①: block_size_m · check.margin_mm · check.max_blocks ·
│                       finger.thickness_m · finger.width_m · assembly_area_half_m, 측정값은 안 읽음) → DesignGenerator · DesignStore에 넘김
├── routes/
│   ├── robot.py      /api/robot   명령 · 정지 · 다시 시작 · 상태 · 스캔 사진 · 점군 · 손목 영상 → MqttClient
│   ├── designs.py    /api/designs 목록 · 상세 · 트리 · 규칙 숫자 · 생성 · 고르기 · 스캔 저장 → DesignGenerator · DesignStore
│   └── ws.py         /ws          마지막 값 먼저 보낸 뒤 바뀔 때마다 {"type": …} 한 겹
├── mqtt_client.py    MqttClient — 다리와 통신 한 곳: req/res(req_id · 5초) · retained 상태 보관 · d2/web/alive 1초 · bridge/alive 감시 ·
│                     get_design · save_build 답하기(DesignStore 호출)
├── design_gen.py     DesignGenerator — GPT-4o 호출 한 곳(4-A · 4-C)
├── design_store.py   DesignStore — 저장 한 곳(JSON 폴더 → PostgreSQL) + 형식 이름 상수 · 기본 설계 등록(d2_task 변환기 ② import, E-59)
├── prompts/          design_system.txt(설계 직접 작성 + 역할 목록 + 잡기 규칙 · 스캔 채우기 절) · blocks_schema.json(blocks/2.0 후보 3개)
├── voice.py          VoiceListener — 호스트(마이크 → Whisper → 의도 → MQTT d2/hmi/intent)
├── mock_robot.py     로봇 PC 없이: 상태 내기 · req에 res 답하기(IRD 11장)
└── data/             designs/ · builds/ · rejected/ · scan/<run_id>/ (gitignore)
```

## 6. 다리 `d2_bridge`(로봇 PC, W127)

IRD 10.1 표 그대로. 구독 QoS는 IRD 4.1(`task/state` · `progress` · `scan_result` · `safety/state` = TRANSIENT_LOCAL depth 1, 나머지 기본 VOLATILE). `/d2/hmi/alive` 1초. 설정은 `robot.yaml` `mqtt.host` · `port` · `qos` · `req_timeout_s` · `alive_s` · `lost_after_s`(IRD 9장 — W127 PR에서 더함). 다리는 변환만 한다(판단 없음). 변환 계산은 ROS · paho 없는 파일로 떼어 `tests/`에서 pytest(CI는 `d2_bridge`를 colcon 빌드하지 않는다). 로컬 개발 PC에는 두산 패키지가 없어 정지 노드를 못 띄운다 — 정지 노드와의 확인은 로봇 PC(W129).

## 7. 일정 (일정표 v10_100916)

| 언제 | 무엇 |
|---|---|
| 10/9 저녁 | W127 다리 최소형 + `mock_bridge` → W126 backend MQTT 층(`mqtt_client.py` · `routes/robot.py` · `ws.py` · `mock_robot.py`) → W047 화면(가능한 데까지) |
| 10/10 낮(로봇) | W149 손목 TCP · W150 손목 노드 실기 · W129 PC 2대 MQTT 확인(한세교) · W151 화면 출발 첫 실기(화면이 없으면 예비 절차 — 명령 출발) · 18시 W158 보고 |
| 10/10 저녁 | W127 나머지(`get_design` · `save_build` · `check_design` · `intent` · `gripper/state` · `progress/1.1` · `scan_result/1.1`) · W111 저장소 · W108 · W112 |
| 10/11 | W108 · W112 · W122 생성 흐름 통합(한석형) · W045 음성 · W127 사진 · 점군 · 손목 영상 — 그 뒤는 10/10 18시 보고 뒤 PL이 다시 짬 |
| 10/12 ~ 13 오전 | compose(W102 — `robot.yaml` 파일 하나를 읽기 전용으로 연결) · PostgreSQL(W088) |

## 8. 문서 · 다른 파트에 걸린 것

| 무엇 | 누구 | 상태 |
|---|---|---|
| SDD 3.4 "웹 PC 코드는 robot.yaml을 읽지 않는다" → 설계 규칙 키만 읽기 전용(10/9 PL ①) | PM | 10/9 전달 |
| `scan_result`에 `nearest_base` 선택 칸(→ `scan_result/1.2`) — task_manager가 `scan_infer` 응답의 값을 넘김 + IRD 4.1 · 6장 | 한석형 · PM(PL 확인 PR) | 10/9 전달 |
| IRD 8.4 ⑦ · SDD 6.9 스캔 설계 부모 = GPT가 DB 목록에서 고른 같은 family 설계(실패 땐 `nearest_base`) · [AI로 고치기] 버튼(10/9 PL ② ③) | PM | 10/9 전달 |
| SDD 3.1.1 `designs/[id]/page.tsx` → `designs/page.tsx?id=`(정적 내보내기 제약) | PM | W047 · W112 때 전달 |
| FR-G02 · `count = 2` 처리 | PL — W108 결과 보고 | 10/10 이후 |

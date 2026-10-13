# 박진용 객체 인식 조립 코드에 추가할 촬영 기능

공급칸 실행기(build_capture/run.sh)는 사용하지 않는다. 기존 one_pick 객체 인식·고르기·집기·p1·재시도 정책을 대체하지 않는다.

1. 시작 때 PlacementRecorder(folder, move_observe, stop_robot)를 만든다.
2. move_observe 콜백은 기존 MoveTo(target='observe')를 사용하고 실제 도착 성공 bool을 반환한다. 기존 시간 제한·Ctrl+C·먼저 안전정지 처리를 유지한다. stop_robot는 기존 안전정지 함수다.
3. 최종 설계 자리에 배치 성공한 뒤 after_place(part_id, target_center_m, target_quaternion_xyzw)를 호출한다. p1 중간 배치에는 호출하지 않는다. 넘기는 좌표는 실제 블록의 설계 배치 중심/방향이며 틀기·옮김 그리퍼 목표가 아니다.
4. after_place 실패는 위로 전파해 다음 블록을 집지 않는다. 정지 후 재시작/알람해제는 기존 사람 조건을 유지한다.
5. 실행 finally에서 finish(completed)를 호출하고 ZIP경로를 출력한다. 중단자료도압축한다. 완료True는기존조립성공조건과전체11촬영이모두성공한경우만.
6. 기존 실행기가 사용한 robot.yaml/레시피/실제 TCP→카메라 보정 파일·단위·버전도 결과폴더에 복사해 주세요. 키/.env는 넣지 않는다.

결과: 최종배치11개마다observe하나만,사진11장. 블록마다 RGB PNG 한 장과 같은 시점의 정렬 깊이 한 장·카메라실측내참수·촬영시각TF. placements.json은목표,관측자료는실제좌표추정용. ZIP하나전달.

촬영기는표준RealSense /camera/camera 접두사용,다르면capture.py CLI --prefix에기존접두를넘기도록통합. 정렬깊이/색/내참수크기·시각불일치거절. 카메라TF없어도누락기록하며실제handeye쌍확인후분석. ROS환경과카메라기동은기존실행기책임.

검증: recorder pytest3통과(11이동→11촬영→ZIP,observe실패시정지/촬영금지/실패ZIP). 이동·촬영은대역. capture.py import/문법/CLI와기존observe가시성시뮬은확인했지만박진용실행기에통합한실기/모션시험은미완료. 연결후기존가상환경에서pytest/조립·촬영개수/정지실패검증결과와수정코드를팀코로반환.

## 배치별 원인 추적 — 10/13 사용자 요청

촬영은 최종 놓기 한 번마다 RGB·정렬 깊이 **한 쌍**이다. 같은 자리를 20번 촬영하지 않는다.
재배치도 별도 시도 번호로 한 쌍만 찍는다. `placements.json.sequence`는 관측 번호이고,
`attempts.json.sequence`는 원래 조립 순서다. 재배치 시 관측 번호만 증가한다.

### 기존 실행기에 연결할 지점

1. 실제 집기 명령 **직전** `begin_attempt(part_id, sequence, target_center_m,
   target_quaternion_xyzw, evidence=...)`로 시도를 저장하고 `action_id`를 받는다.
2. 기존 집기·중간 틀기·놓기는 그대로 실행한다. 아래 실행 증거를 기존 출력의 실제 값에서 모은다.
3. 놓기/후퇴 성공 뒤 `after_place(part_id, target_center_m, target_quaternion_xyzw,
   action_id=action_id, evidence=...)`를 한 번 호출한다. 기존 observe 도착 후 사진 한 쌍과 시도를 연결한다.
4. 집기/놓기 실패는 기존 정지 처리를 먼저 수행한 뒤 `failed_attempt(action_id, reason,
   evidence=...)`에 기록한다. 배치 성공으로 취급하거나 사진을 찍지 않는다.
5. 재배치는 원래 `sequence`와 설계 목표를 유지하고 새 `begin_attempt(..., retry_of=action_id)`로
   별도 기록한다. 이 기능은 재배치를 실행하거나 기존 재시도 횟수를 바꾸지 않는다.

기존 호출도 유지되지만 action_id가 없으면 `trace_available:false`다. 그 사진에서 파지 원인을 추정하지 않는다.
잘못된 부품/목표/시도를 연결하면 관측 이동 전에 ValueError로 거절하므로 실행기는 기존 오류·정지 경로로 보낸다.
`begin_attempt` 실패 때는 집기를 시작하지 않는다. 사진과 기록 저장 실패 뒤에도 다음 블록을 진행하지 않는다.

### 증거 객체에 담을 값

아래 이름은 이 도구의 로컬 진단 자료이며 ROS 인터페이스/IRD를 바꾸는 제안이 아니다.
입력은 표준 JSON 객체이고 길이 단위 m, 방향 xyzw, 좌표계 base_link를 명시한다.
없는 값은 null로 남긴다. 명령값과 관측값을 서로 대신 넣지 않는다.

- `source`: 인식 후보 ID, 실제 인식 중심/방향, 인식 시각, 자료 파일 또는 로그 참조.
- `grasp`: 선택한 면·grasp_axis·파지 방식, 그리퍼 목표 폭/실제 폭, 파지 결과.
- `motion`: 단계별 이름(최초 집기·p1·틀기·최종 놓기·후퇴), 요청 블록 자세,
  실행기가 계산한 TCP 목표, 실제 로봇 TCP와 읽은 시각, 성공/실패. 실제 출력에 없는 TCP는 null.
- `release`: 열기 명령/피드백, 내려놓기·후퇴 완료 시각 및 성공/실패.
- `runtime`: 실행 소스 버전, 실제 읽은 hand-eye 경로/해시·좌표 방향·단위,
  TCP 설정/레시피 해시. 동봉 파일만 보고 실제 사용 파일이라고 쓰지 않는다.

처음 알려진 값은 `begin_attempt`의 evidence에, 실행 중 쌓인 증거는 after_place/failed_attempt의
evidence에 담는다. 입력 객체를 복사하므로 호출 뒤 원본 로그 객체 변경이 이전 기록을 바꾸지 않는다.
`attempts.json`은 집기 전 저장되므로 중단 시 `started` 상태도 남는다.

### 분석과 보정의 순서

`action_id → capture_folder → 관측한 실제 블록 중심/방향 → 목표 대비 dx/dy/dz → 판정`
으로 연결한다. 목표 자세, PickPlace의 블록 중심 자세, TCP 목표/피드백, 카메라 측정을 각각 보관한다.
TCP가 목표에 도착했다는 사실만으로 블록이 정확하게 놓였다고 판단하지 않는다.

같은 파지 방식의 여러 배치를 비교해 방향/집기 위치별 편향을 찾는다. TCP 목표와 피드백의 차이,
파지 위치의 추정 편향, 중간 틀기 유무, 놓기·후퇴, 카메라 변환을 나누어 원인 후보를 표시한다.
사진만으로 미끄러짐/파지 편심/로봇 오차를 확정하지 않으며 증거가 없으면 원인은 UNKNOWN이다.

기존 7개 배치·140쌍은 그대로 분석에 사용한다. 오래된 자료에 새 action_id/파지 증거를 지어내지 않는다.
같은 배치의 20쌍은 관측 흔들림을 평가하는 자료이며 20개의 독립 배치 실험으로 세지 않는다.
확정된 실제 중심만 기존 축별 3mm·최대 2회 정책에 전달하고, 가림·접합/보정 불확실성은 재집기로 보내지 않는다.
자동 재집기는 실제 방향·주변/받침 확인과 팀 전달 계약이 확인된 뒤 연결한다.

추가 검증: recorder pytest 7건 통과(파지/놓기/재시도-사진 연결, 집기 실패 보존,
시도 불일치의 이동 전 거절, NaN 증거 거절 포함). 이동·촬영은 대역이며 실제 실행기 통합은 미실행.

## rosbag 전체 기록 — 박진용 실행기 담당 에이전트 인계

### 목적과 담당

실행기 담당은 아래 기록을 기존 one_pick 조립 흐름에 연결하고, 연결 코드와 검증 결과를 PR로 반환한다.
한석형은 받은 bag/시도 로그에서 정지한 observe의 RGB·깊이·촬영 시각 TF를 추출해 배치 오차를 분석한다.
이 절은 **연결 요구사항**이다. 이 PR에 rosbag 실행·추출 도구가 구현됐다는 뜻은 아니다.
관측당 한 쌍 촬영 도구는 즉시 확인용으로 유지하며, 연속 bag에서 원인 추적과 추가 추출을 수행한다.
손목 카메라의 잡기 중 가림은 observe의 최종 배치 측정을 막지 않는다.

### 어디부터 어디까지

1. 카메라·TF·기존 로봇 상태가 준비되고 실제 토픽/QoS를 확인한 뒤, **첫 후보 인식/find_blocks와
   첫 집기 명령보다 먼저** bag을 시작한다. 최초 인식 원본부터 있어야 집기 후보와 명령을 비교할 수 있다.
2. bag에 필수 토픽 데이터가 실제로 들어온 것을 확인한 뒤 조립을 시작한다.
   이미 있는 첫 observe 정지 구간의 작업대/배치 전 장면도 남긴다. 이를 위해 새 관측 자세를 만들지 않는다.
   첫 배치 전 동일 시점 자료가 없으면 baseline_missing을 기록하고 이후 배치 간 비교는 유지한다.
3. 후보 인식 → 집기 → p1/틀기 등 기존 중간 동작 → 최종 놓기 → 후퇴 → observe 도착/정지 →
   RGB·깊이 한 쌍 확인을 **모든 배치와 재시도에 걸쳐 끊지 않고** 기록한다.
4. 마지막 배치도 observe 정지 후 자료가 들어온 것을 확인하고 run_end 이벤트를 저장한 뒤 종료한다.
   실패/STOP/Ctrl+C 때는 기존 안전정지를 먼저 수행하고 실패 단계·미완료 상태를 남긴 다음 bag을 정상 종료한다.
5. 녹화 프로세스의 종료/저장 완료를 기다려 metadata와 데이터 파일을 보존한다. 완료 전 압축/전달하지 않는다.
   비정상 종료 bag은 incomplete로 표시하고 가능한 원자료와 실패 이유를 함께 남긴다.

### 무엇을 왜 기록하는가

아래는 현재 코드의 이름을 기준으로 한 목록이다. 실행기의 실제 이름/타입/QoS를 조회하고 manifest에 확정한다.
특히 joint_states와 실제 TCP 이름은 추측하지 않는다.

|자료|현재 이름 또는 연결 방법|필요한 이유|
|---|---|---|
|색 영상|`/camera/camera/color/image_raw`|블록 경계와 배치 전후 변화 확인|
|색 영상에 정렬된 원시 깊이|`/camera/camera/aligned_depth_to_color/image_raw`|화소를 실제 3차원 위치로 변환; 영상 캡처/색칠된 깊이 그림으로 대체하지 않음|
|색 카메라 내참수|`/camera/camera/color/camera_info`|실제 투영·왜곡·해상도 재현|
|동적/정적 TF|`/tf`, `/tf_static`|각 영상의 정확한 촬영 시각에 base←camera/base←TCP 좌표 변환; 정적 변환도 bag에 포함|
|실제 관절 상태|실행 중 조회한 joint_states 토픽|로봇의 실제 움직임과 정지 구간 확인; 명령 자세와 구별|
|그리퍼 상태|`/d2/gripper/state`|폭·파지 결과·열기/놓기 시각을 집기 단계와 비교|
|조립 상태/진행|`/d2/task/state`, `/d2/task/progress` (기존 실행기가 발행하는 경우)|어떤 부품/단계의 배치인지 연결. 없는 토픽을 필수로 가장하지 않음|
|인식 후보/선택 결과|실제 인식 서비스 반환값을 시도 로그에 복사|사진 속 어느 후보를 pick 대상으로 골랐는지 확인|
|집기/놓기 명령·단계·결과|아래 실행기 JSONL 로그|pick_pose/place_pose/grasp와 실제 실행/재시도 대응; bag만 켰다고 서비스/액션 요청이 저장됐다고 가정하지 않음|
|실제 TCP 피드백|기존 TCP TF 또는 실제 발행 상태/실행기 피드백|계산한 TCP 목표와 실제 도착 차이를 확인. 얻을 수 없으면 null+이유|

카메라 접두가 다르면 실제 접두를 사용한다. 필요한 TF 경로에 있는 모든 변환을 기록한다.
지원되는 배포판과 introspection 설정을 확인하지 않고 action/service 기록 옵션을 사용하지 않는다.
서비스/액션 내용은 실행기가 이미 가진 입력·피드백·결과를 **수동적 로컬 JSONL 기록**으로 남기면 충분하다.
팀 ROS 인터페이스를 새로 만들거나 motion 담당 외 파일을 바꾸지 않는다.

### 실행기 로그의 연결 키와 시점

`events.jsonl` 각 줄에 `run_id`, `action_id`, 원래 조립 `sequence`, 부품 ID, `retry_of`,
`event`, `ros_stamp_ns`, `unix_stamp_ns`를 넣는다. 실제 ROS 메시지 header stamp도 값 옆에 그대로 보관한다.
단조 시계는 제한시간 계산에만 쓰고, bag/이미지와 연결 가능한 ROS 시각을 따로 남긴다.
새 PLM 실행기는 part_id, 기존 one_pick은 block_id를 실제 버전대로 기록하고 manifest에 이름을 명시한다.

이벤트는 `run_start`, `candidate_selected`, `pick_requested`, 실제 단계 전환,
`place_result`, `retreat_complete`, `observe_requested`, `observe_arrived`, `observation_saved`,
`attempt_failed`, `run_end`를 해당 시점에 남긴다. 없는 피드백 단계/시각은 추측하지 않는다.
첫 pick_requested 이전에 begin_attempt로 action_id를 만들고 after_place의 사진에도 같은 action_id를 연결한다.
기존 시도 기록의 unix 시각만으로는 ROS 시뮬 시각과 대응이 보장되지 않으므로 JSONL의 ROS 시각을 함께 사용한다.

명령에는 원래 블록 중심 pick_pose/place_pose, grasp/grasp_axis/폭, 공급 후보,
실제 계산된 TCP 목표와 중간 틀기 자세를 담는다. 피드백에는 실제 읽은 TCP·그리퍼 상태와 시각을 담는다.
블록 중심 자세와 TCP 자세는 별도 필드로 저장한다. observe_arrived는 단순 요청 전송이 아니라
기존 도착 성공/정지 확인 후에 기록하고, observation_saved에는 실제 RGB/깊이 stamp와 자료 경로를 넣는다.
로그 파일 쓰기를 로봇 제어 콜백에서 오래 막지 않는다. 기존 파일 쓰기 방식/큐를 사용하고 실패는 드러낸다.

### 품질 확인·전달 자료·합격 기준

- 시작 전에 조회한 토픽/타입/QoS 목록과 실제 bag 기록 설정을 `manifest.json`에 저장한다.
  센서 best-effort와 `/tf_static`의 transient-local을 실제 발행 QoS에 맞춰 수신하고,
  시작 전에 발행된 정적 TF가 들어오는지도 확인한다.
- bag은 카메라 원래 발행 시각/해상도를 유지한다. 디스크 여유/쓰기 속도를 확인하고
  지원되는 lost-message 통계가 있으면 함께 남긴다. 분할 저장 시 모든 조각을 전달한다.
- 각 observe에는 after-arrival RGB·깊이 한 쌍이 있고, 현재 capture.py의 시각차/크기/인코딩 검사와
  촬영 시각 TF 조건을 충족해야 한다. 수집 실패/필수 자료 누락이면 기록을 불완전으로 표시하고
  기존 오류·정지 경로로 보내며 다음 집기를 진행하지 않는다. 임의 sleep을 도착 확인으로 대신하지 않는다.
- 전달 폴더: `bag/` 전체(metadata와 모든 데이터 조각), `events.jsonl`, `attempts.json`,
  `placements.json`, `run.json`, `manifest.json`, 배치별 observe 한 쌍, 실행기 변경 코드/커밋 및 검증 결과.
  실제 로드한 robot.yaml·TCP·hand-eye 원본/단위/변환 방향·레시피/순서표·파일 해시도 포함한다.
  키/.env/환경변수 전체는 넣지 않는다. 동봉 파일이 실제 로드 파일인지 manifest에 확인한다.
- 실행기는 로봇 이동/카메라를 대역으로 하여 녹화가 첫 인식/집기보다 먼저 시작하고
  마지막 observe 자료보다 나중에 종료하는지, 실패 시 안전정지 우선/불완전 보존,
  재시도 action_id 구분, 기록기 종료/실패/필수 토픽 누락을 확인한다.
- 실제 bag을 ROS 명령 전송 없이 오프라인으로 열어 필수 메시지 존재, 시각 범위,
  모든 배치의 observe-영상-TF-로그 대응을 확인한다. 이 기록 검증과 실제 로봇 모션 검증을 구분한다.

분석은 bag 파일을 직접 읽는 오프라인 추출로 한다. 정지 observe의 한 쌍을 배치별 측정에 쓰고,
집기/중간 동작 구간은 원인 후보 확인에 쓴다. bag 명령/액션을 실제 로봇에 재생하지 않는다.
기존 7배치 사진은 계속 분석하고, 위 전체 기록은 이후 부족한 실물 검증 때 수집한다.

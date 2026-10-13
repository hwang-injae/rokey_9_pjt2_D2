# 박진용 객체 인식 조립 코드에 추가할 촬영 기능

공급칸 실행기(build_capture/run.sh)는 사용하지 않는다. 기존 one_pick 객체 인식·고르기·집기·p1·재시도 정책을 대체하지 않는다.

1. 시작 때 PlacementRecorder(folder, move_observe, stop_robot)를 만든다.
2. move_observe 콜백은 기존 MoveTo(target='observe')를 사용하고 실제 도착 성공 bool을 반환한다. 기존 시간 제한·Ctrl+C·먼저 안전정지 처리를 유지한다. stop_robot는 기존 안전정지 함수다.
3. 최종 설계 자리에 배치 성공한 뒤 after_place(block_id, target_center_m, target_quaternion_xyzw)를 호출한다. p1 중간 배치에는 호출하지 않는다. 넘기는 좌표는 실제 블록의 설계 배치 중심/방향이며 틀기·옮김 그리퍼 목표가 아니다.
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

# W144 배치 측정·보정 계산 준비

## 현재 범위

이 도구는 저장된 RGB·정렬 깊이·촬영 시각 TF를 오프라인으로 분석한다.
PlacementCorrection은 검증된 관측에서 3mm 판정과 내부 재배치안을 계산한다.
로봇 명령을 보내지 않고 TaskManager/CheckProgress/ROS 인터페이스에도 아직 연결하지 않았다.
PR #140은 촬영/rosbag 실행기 인계이며 이 변경은 그 자료를 받아 계산할 별도 작업이다.

## 측정 흐름

1. 표준 CameraInfo의 실제 내참수/왜곡, 깊이 단위 m, 강체 TF를 확인한다.
2. 직전 배치 관측을 현재 카메라로 3D 재투영한다. z-buffer로 가림을 처리하고 빈 화소는 unknown으로 남긴다.
3. 목재 색과 새 깊이/색 변화로 새 블록 후보를 분리한다. 같은 높이에서 붙은 블록도 전후 차분 대상이다.
4. 내부 깊이에서 표면 높이와 평면 기울기/잔차를 확인하고 실제 경계 광선을 관측 표면에 교차시킨다.
5. 관측 외곽의 중심/yaw/크기를 측정한다. 목표는 검색 범위에만 사용하며 관측 중심을 목표로 되돌리지 않는다.
6. 후보 두 개, 변화 없음, 기울기/깊이 잡음, 크기 불일치, 불완전 직사각형은 UNKNOWN이다.

현재는 수평 눕힘과 목재 색만 지원한다. 색 문턱/형상 문턱은 탐색 설정이며 현물 정확도 보장이 아니다.
모든 결과는 EXPLORATORY/UNKNOWN과 accuracy_validated:false로 반환하고 자동 보정에 직접 넘기지 않는다.
반복 촬영의 흔들림은 실제 위치 정답 대비 절대 오차와 다르다.

## 계산 정책과 내부 입력

assess(part_id, target_center_m, observation)는 아래 정보가 확인된 내부 관측만 받는다.

- state:present, frame:base_link, 동일 part_id, identity_confirmed:true, accuracy_validated:true
- 실제 center_m과 독립 실측 시험으로 검증된 축별 오차 상한 uncertainty_m (m, 비음수)

|판정|조건|
|---|---|
|PASS|모든 축에서 abs(delta)+uncertainty <= 3mm|
|CORRECT|한 축이라도 abs(delta)-uncertainty > 3mm, 재배치 예산 남음|
|HUMAN|재배치 2회 예산을 이미 소비함|
|UNMEASURED|문턱을 걸친 불확실성/미검증 관측/잘못된 입력|

prepare_goal은 실측 quaternion_xyzw와 pose_confirmed, support_confirmed, scene_clear,
regrasp_allowed 및 실제 주변 obstacles도 확인한다. 현재는 FLAT_SHORT/FLAT_LONG만 준비한다.
pick_pose는 실제 중심/방향, place_pose는 원래 설계 목표다. 목표 방향을 실제 방향으로 대신 넣지 않는다.
관측·계획은 예산을 소비하지 않으며, 송신 담당은 정지 잠금 안에서 최신 run/part/관측 시각/안전 상태를
재확인하고 reserve 성공 뒤에만 요청해야 한다. STOP/실패로 예산을 환불하지 않고 새 run에서만 초기화한다.
이 이름들은 내부 계산안이며 IRD/서비스 확정 필드가 아니다. 실제 전송 연결은 계약 검토 뒤 진행한다.

## 실행과 의존성

기존 ROS 카메라 환경의 numpy/OpenCV를 사용한다. 설치를 실행하지 않았다.
OpenCV는 팀 CI의 공통 시험 의존성에 없으므로 영상 시험은 tools 아래에서 별도로 실행한다.
일반 CI task 시험은 OpenCV를 import하지 않는 정책 시험을 포함한다. 영상 시험을 CI에서 실행했다고 표시하지 않는다.

```bash
python3 -m pytest -q src/d2_task/test tools/w144_measure/test_measurement.py
python3 tools/w144_measure/analyze_received.py <받은_폴더> \
  --out <원자료_밖_분석_폴더> --size-m 0.07445 0.0248 0.0148 \
  --tcp-camera <실제_TCP_camera_보정.npy> --calibration-units mm
```

TCP←camera 외참수의 이동 단위 m/mm를 명시한다. CLI의 크기는 동봉 robot.yaml의 실측 값이며
다른 촬영에는 그 실행의 실제 크기를 사용한다. 실제 로드 보정 파일 여부는 별도 확인해야 한다.
원자료 내부를 출력 폴더로 허용하지 않는다. 분석 JSON과 대표 경계 그림만 새 폴더에 생성한다.

## 검증과 실제 자료 결과

- 최신 main 34368f2 위에서 task 전체+영상 시험 925 통과: task 915, 별도 영상 10.
- 새 정책/영상 관련 48 통과. 합성 RGB/깊이는 축별 실제 이동, 같은 높이 접합,
  카메라 이동, 정적 장면, 다중 후보, 절반/모서리 가림, 깊이 누락/잘못된 변환을 포함한다.
- 합성 영상의 정확한 위치는 채점에만 쓰고 측정기의 검색 목표와 구분했다.
- 실제 7배치·140쌍에 최종 엄격한 경계/평면 조건을 적용하면 3번 배치만 3/20 탐색 측정,
  나머지는 UNKNOWN이다. 3번 탐색 중앙값 목표 오차 mm는 약 [-1.365,-0.968,-0.077].
  이 값은 기준 실측 없음/보정 실제 사용 경로 미확인으로 정확도 PASS/배치 PASS가 아니다.
- 기존 느슨한 RGB 분석에서 얻은 1/4/6/7 탐색값을 새 코드의 통과로 합치지 않았다.
  경계가 불완전하거나 표면 잡음/기울기가 크면 기존 탐색값이 있어도 자동 보정 입력을 만들지 않는다.

## 남은 연결

현물의 독립 위치 기준으로 uncertainty를 확인하고, 색 경계/표면 조건의 현실 적합성을 비교해야 한다.
CheckProgress는 현재 실제 자세/관측 신원·시각/신뢰도를 전달하지 않으므로 PM·비전 담당과
최소 전달 계약을 확인한 뒤 TaskManager·장면·PickPlace를 연결한다. 인터페이스를 임의로 바꾸지 않는다.
기존 회전/받침/정지/충돌 검사와 새 PLM part_id 이름을 유지한다. 자동 재배치·ROS/실기 완료가 아니다.

# 로봇 시스템

M0609 팔·RG2 그리퍼·손목카메라가 장착된 로봇 시스템의 구성·동작·정지를 모은다.
`d2_robot/`은 소스 폴더 묶음이며 별도 ROS 패키지는 아니다. 각 패키지 이름·노드·통신 계약은 유지한다.

| 패키지 | 책임 |
|---|---|
| [d2_bringup](d2_bringup/) | 모델·장착 관계·TCP·공통 설정·launch |
| [d2_motion](d2_motion/) | 집기·놓기, MoveIt2 계획·실행, 충돌 장면 관리 |
| [d2_gripper](d2_gripper/) | RG2 제어·파지 확인 |
| [d2_safety](d2_safety/) | 정지 판단·서기 궤적·잠금·재개 |

카메라의 장착 모델·좌표는 bringup에 둔다. 관측 해석은 [d2_vision](../d2_vision/),
조립 순서·완료 판단은 [d2_task](../d2_task/), 공통 메시지는 [d2_interfaces](../d2_interfaces/),
설계·레시피 변환은 [recipe_manager](../recipe_manager/)에서 담당한다.

공통 설정 정본은 [robot.yaml](d2_bringup/config/robot.yaml)이다. 실행 시에는 계속
`get_package_share_directory('d2_bringup')`에서 설치된 설정·모델을 읽는다.
팔 명령은 집기·놓기, 실제 Scene 변경은 장면 관리, 정지 판단은 정지 노드가 각각 담당한다.

## 이동 뒤 빌드

저장소 루트에서 ROS 2와 두산 워크스페이스 환경을 불러온 뒤 실행한다.
기존 설치의 소스 링크를 새 경로로 갱신하려면 네 패키지를 다시 빌드한다.

```bash
colcon build --symlink-install --packages-select d2_bringup d2_motion d2_gripper d2_safety
source install/setup.bash
```

가상·실물 launch 명령의 패키지 이름은 기존과 같다. 가상 에뮬레이터 실행·종료 시
컨테이너 삭제 처리는 [저장소 규칙](../../AGENTS.md)을 먼저 확인한다.

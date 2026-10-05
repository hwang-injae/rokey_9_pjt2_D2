# 개발 환경 설정 (env)

| 항목 | 내용 |
|---|---|
| OS · ROS | Ubuntu 24.04 LTS + ROS 2 Jazzy (Python 3.12). 팀 PC 모두 같은 버전 |
| 로봇 | 두산 M0609 + 두산 드라이버(doosan-robot2) + MoveIt2 2.12. 팀 브링업 = 박진용 `real_moveit.launch.py` (`mode:=virtual`·`real`) |
| 그리퍼 | OnRobot RG2 (고무 패드 뺌, 10/4) |
| 카메라 | 손목 RealSense D435i(깊이 있음) — 로봇 PC / 웹캠(깊이 없음) — 운영 PC |
| ROS 2 통신 | **CycloneDDS**(`rmw_cyclonedds_cpp`), `ROS_DOMAIN_ID` = 60 (팀 60번대) |
| 컨테이너 | Docker (필수). 1차 컨테이너는 운영 PC의 `vision`(웹캠 사람 감지) · `hmi`(웹 화면) 2개. 음성 · 작업 관리자 · 작업 판단과 로봇 PC 노드는 호스트. DB(`db-hmi`)는 1차 뒤 (10/5 S-15) |
| PC | 2대로 시작: 로봇 PC + 운영 PC |

이 저장소 워크스페이스(`rokey_9_pjt2_D2`)는 두산 드라이버 워크스페이스 위에 겹쳐 쓴다. ROS 2 Jazzy와 두산 드라이버를 처음부터 까는 순서는 강의 설치 안내를 따른다. 이 문서는 **팀이 맞춰야 하는 것**만 적는다.

## 목차

| 절 | 내용 |
|---|---|
| 1. PC 2대 역할 | 로봇 PC · 운영 PC에 무엇을 켜나, 늘리는 기준 |
| 2. 저장소 받기 | `git clone`, 빌드 순서 |
| 3. 버전 확인 | OS · Python · ROS 2 |
| 4. 로봇 브링업 | 팀 브링업(가상 · 실기), 유선 IP, 정지 준비 확인 |
| 5. RG2 · RealSense D435i · 웹캠 | 장치 연결과 확인 |
| 6. CycloneDDS · ROS_DOMAIN_ID | 설치 · `.bashrc` · PC 2대 통신 확인 |
| 7. Docker | 컨테이너 2개(`vision` · `hmi`) · 설치 · host 네트워크 · `.env` · 로컬 폴더 연결 · 지우기 전 확인 |
| 8. 키 (`.env`) | OpenAI 키를 두는 규칙 |
| 9. 작업 전 확인 | 매일 켤 때 보는 점검표 |

## 1. PC 2대 역할

| PC | 켜는 것 | 연결 장치 | 돌리는 곳 |
|---|---|---|---|
| **로봇 PC** | 브링업(두산 드라이버·제어기·MoveIt2 move_group), 집기·놓기 + 실행기, 장면 관리, 그리퍼 노드, **정지 노드**, 손목 블록 인식, 손목 손 찾기 | 로봇·그리퍼(유선 랜), RealSense D435i(USB) | 호스트 |
| **운영 PC** | 웹 화면, 음성, 웹캠 사람 감지, 작업 관리자(1차 CSV 기록 포함), 작업 판단 · (1차 뒤) 음성 멈춰, DB | 웹캠(USB), 마이크 | 컨테이너 `vision`: 웹캠 사람 감지 · 컨테이너 `hmi`: 웹 화면 · 호스트: 음성 · 작업 관리자 · 작업 판단(10/5 S-15). DB 컨테이너 `db-hmi`는 1차 뒤. 음성 멈춰는 HMI가 정함. 기록기 노드는 없다(10/4 간소화 S-10) |

- 어느 PC가 로봇 PC·운영 PC인지는 (미정)이다. 정하면 여기에 적는다.
- 모든 노드는 어느 PC에서나 돌게 짠다. 옮기지 않는 것은 브링업·정지 노드(로봇 PC)이고, 카메라는 그 카메라를 처리하는 PC에 꽂는다.
- **늘리는 기준:** 10/7 자동 모드 실기 때 잰 부하가 CPU 70%를 넘거나 카메라 처리가 초당 15장 아래면 비전을 3번째 PC로 옮긴다(W053).
- **컨테이너는 성능을 바꾸지 않는다(S-15).** 같은 PC에서 도는 프로그램이라 CPU 차이가 거의 없고, host 네트워크라 ROS 통신 지연도 없다. 과부하는 노드를 다른 PC로 옮기거나 처리량(해상도 · Hz)을 줄여 푼다.

## 2. 저장소 받기

```bash
# 위치는 자유. 저장소 폴더 = 우리 ROS 2 워크스페이스
git clone https://github.com/hwang-injae/rokey_9_pjt2_D2.git
cd rokey_9_pjt2_D2
```

- 브랜치·커밋·PR 규칙은 [팀 협업 규칙](../06_팀협업규칙_v1_100523.md)에 있다.
- source 순서는 늘 같다: `/opt/ros/jazzy` → 두산 워크스페이스 → 이 저장소. 빌드 명령은 [src/README](../../src/README.md) '빌드 · 시험'.

## 3. 버전 확인

```bash
lsb_release -ds            # Ubuntu 24.04.x
python3 --version          # Python 3.12.x
printenv ROS_DISTRO        # jazzy (source 뒤)
```

## 4. 로봇 브링업

팀 브링업은 박진용 `real_moveit.launch.py` 하나로 통일한다(D-09). M0609 + RG2 + 손목 카메라 모델, 두산 드라이버, MoveIt2를 한 번에 켠다. **10/6 오전에 `d2_bringup` 패키지로 옮긴다**(로봇 동작, 인프라·통합, W035). 옮긴 뒤의 실행 명령은 그때 여기에 적는다.

```bash
# 가상 (에뮬레이터) — 로봇 없이
ros2 launch <real_moveit.launch.py 경로> mode:=virtual

# 실기
ros2 launch <real_moveit.launch.py 경로> mode:=real host:=192.168.1.100
```

- **실기 PC 유선 IP:** IPv4를 수동으로 `192.168.1.x`, 넷마스크 255.255.255.0. x는 로봇(.100)·그리퍼(.1)와 겹치지 않게 한다. 연결 확인은 `ping -c 3 192.168.1.100`, `ping -c 3 192.168.1.1`.
- 실기 모드 사전 설정(재부팅하면 풀린다): `sudo sysctl -w net.ipv4.ip_unprivileged_port_start=0`
- 티치펜던트와 ROS가 로봇을 동시에 제어하지 않게 한다.
- 정지 처리가 들어간 프로그램은 시작할 때 아래 두 줄이 나와야 한다. `없음`이나 `안 보인다`가 나오면 실기를 하지 않는다(R-01).

  ```
  [정지 준비] 서기 궤적 컨트롤러: 있음
  [정지 준비] 두산 move_stop 서비스: …/motion/move_stop
  ```

## 5. RG2 · RealSense D435i · 웹캠

| 장치 | 연결 | 확인 | 주의 |
|---|---|---|---|
| RG2 | 로봇 네트워크(192.168.1.1), Modbus | 그리퍼 노드(`d2_gripper`)로 열기·닫기 | **그리퍼 노드 하나만** 그리퍼에 명령한다. 고무 패드를 빼서 치수·TCP·표시폭을 다시 재는 중 — 값은 ＿＿ (측정 전) |
| RealSense D435i | 로봇 PC USB 3 | `ros2 launch realsense2_camera rs_launch.py` 뒤 `ros2 topic list`에 카메라 토픽 | 깊이 최소 거리 약 28 cm → 관측 자세는 30~40 cm 위. 손목 보정이 4.8 cm 어긋나 있어 10/6에 다시 한다(R-05) |
| 웹캠 | 운영 PC USB | `ls /dev/video*`로 번호 찾기 | 한 번에 한 프로그램만 웹캠을 연다. 작업대 앞쪽 긴 변 밖 가운데에 두고, 옮기면 보정을 다시 한다. 컨테이너에서는 `--device /dev/video<번호>` |

```bash
# RealSense ROS 패키지 (없으면)
sudo apt install ros-jazzy-realsense2-camera
```

- 파이썬 주의: MediaPipe는 numpy 2, ROS(cv_bridge)는 numpy 1.26을 써서 부딪친 기록이 있다. 그래서 웹캠 사람 감지는 컨테이너 `vision`으로 호스트와 떼어 둔다(10/5 S-15). 컨테이너 안에서 둘을 같이 쓰면 numpy<2와 맞는 MediaPipe 판을 고르거나 cv_bridge 없이 변환한다(비전 파트가 정함).

## 6. CycloneDDS · ROS_DOMAIN_ID

모든 PC·컨테이너가 **같은 RMW(CycloneDDS)와 같은 `ROS_DOMAIN_ID`**를 써야 서로 보인다. 우리 팀은 **60번대**(60~69)를 쓴다. GPU PC = **60**. 번호가 같아야 서로 보이므로 함께 돌리는 PC·컨테이너는 모두 **60**으로 맞춘다(혼자 시험할 때 61~69를 각자 쓰는 것은 (안)).

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

`~/.bashrc` 맨 아래에 더한다.

```bash
# --- D2 협동2 ---
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=60          # 팀 60번대. 함께 돌리는 PC·컨테이너는 모두 60
```

- 바꾼 뒤 새 터미널을 열고 `ros2 daemon stop`을 한 번 한다(예전 설정으로 떠 있던 데몬을 끈다).
- 지난 프로젝트 설정(Fast DDS 설정 파일, `ROS_DISCOVERY_SERVER`, 다른 `ROS_DOMAIN_ID`)이 `.bashrc`에 남아 있으면 주석 처리한다. 같은 변수가 두 번 있으면 아래 것이 이긴다.
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`는 혼자 가상 시험할 때만 쓴다. 켜 두면 PC 2대가 서로 안 보인다.
- **PC 2대 통신 확인:** PC A에서 `ros2 run demo_nodes_cpp talker`, PC B에서 `ros2 run demo_nodes_cpp listener`. 글이 안 넘어오면 두 PC의 `echo $RMW_IMPLEMENTATION`·`echo $ROS_DOMAIN_ID`부터 본다.
- 강의실 무선망이 ROS 2 자동 탐색(멀티캐스트)을 막으면 CycloneDDS 설정 파일(`CYCLONEDDS_URI`)에 상대 PC 주소를 적어야 한다. 필요한지와 설정 내용은 PC 2대를 처음 함께 돌릴 때 보고 정한다(늦어도 10/7 실기 전, 미정).
- 다른 팀 노드가 섞이는지: 브링업 뒤 `ros2 node list`에 다른 팀 `/dsr01` 등이 보이면 번호를 다시 본다.

## 7. Docker

```bash
sudo apt install docker.io
sudo usermod -aG docker $USER      # 한 번 로그아웃했다 들어오면 sudo 없이 쓴다
docker --version
```

1차 컨테이너는 **운영 PC의 2개**다(10/5 S-15). 나머지는 호스트에서 바로 돌린다.

| 컨테이너 | 안에서 도는 것 | 따로 두는 이유 | 만드는 사람 (S-17) |
|---|---|---|---|
| `vision` | 웹캠 사람 감지 | MediaPipe의 numpy 2와 ROS cv_bridge의 numpy 1.26이 부딪친 기록이 있다. 웹캠은 `--device`로 넘긴다 | 한석형 · 10/6 오후 (W101) |
| `hmi` | 웹 화면 | 웹 서버 라이브러리를 ROS와 떼어 둔다 | 황인재 · 10/6 오후~저녁 (W102, `d2_interfaces` W027 뒤) |
| `db-hmi` (1차 뒤) | DB | DB 프로그램은 컨테이너로 띄우는 것이 가장 쉽다 | 황인재 · 1차 뒤 (W088) |

| 호스트에서 바로 | 이유 |
|---|---|
| 음성 (운영 PC) | 마이크 장치를 컨테이너에 넘기기가 번거롭다 |
| 작업 관리자 · 작업 판단 (운영 PC) | rclpy와 순수 파이썬뿐이라 부딪칠 것이 없다 |
| 로봇 PC 전부 | 10/3에 시험한 호스트 환경(두산 드라이버 · MoveIt2 · RealSense)을 그대로 쓴다 |

**컨테이너는 그 안에 넣는 노드의 담당이 만든다(10/5 S-17).** Dockerfile은 `docker/<컨테이너 이름>/`(안)에 두고, 공통 설정은 아래 명령과 규칙 표를 그대로 따른다. 10/6 저녁 기능별 통합 확인 ②(W097)부터 컨테이너로 돌린다. 1시간 넘게 막히면 일단 호스트로 돌리고 10/8에 마저 한다. 컨테이너 ↔ 로봇 PC 통신 확인은 로봇 동작(W029)이 한다.

컨테이너는 성능을 바꾸지 않는다(1장). DB 제품·시기는 HMI가 1차 뒤 10/8 오전 화면 설계 뒤에 정한다(W051, 10/4 간소화 S-09 — 1차 기록은 CSV). 띄우는 모양은 아래와 같다(이미지 이름은 미정).

```bash
docker run -d --name vision-<이름> \
  --net=host --ipc=host \
  --device /dev/video<번호> \
  --env-file .env \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=60 \
  -v <저장소 폴더>:/ws/rokey_9_pjt2_D2 \
  -v <원본 자료 폴더>:/data:ro \
  <이미지 이름>
```

| 규칙 | 내용 |
|---|---|
| host 네트워크 | `--net=host`로 띄운다. 그래야 호스트 노드와 같은 DDS 통신을 쓴다. 이미지 안에도 `ros-jazzy-rmw-cyclonedds-cpp`를 깐다 |
| 키는 `--env-file .env` | 키를 Dockerfile·이미지에 넣지 않는다 |
| 원본 자료는 로컬 연결 | 녹화(rosbag)·대용량 CAD·DB 데이터는 이미지에 넣지 않고(`COPY` 금지) 로컬 폴더를 `-v`로 연결한다. 읽기만 할 원본은 `:ro`. 설정·레시피도 복사하지 않고 로컬 저장소 폴더를 연결해 읽는다. DB 데이터는 이름 있는 볼륨이나 로컬 폴더에 둔다 |
| 이름 | 컨테이너 이름에 파트·이름을 붙인다. 예: `vision-<이름>`, `db-hmi` |
| 지우기 전 확인 | 지우기 전에 `docker ps -a`로 이름·이미지·만든 사람을 본다. 내가 만든 것이 아니면 먼저 묻는다. `docker rm -f $(docker ps -aq)`, `docker system prune` 같은 한꺼번에 지우는 명령은 쓰지 않는다 |
| 장치 | 웹캠은 `vision` 컨테이너에 `--device /dev/video<번호>`로 넘긴다. 마이크는 넘기지 않는다 — 음성은 호스트에서 돈다(S-15) |

## 8. 키 (`.env`)

- OpenAI 키는 **PC마다 `.env` 파일에만** 둔다. 코드는 환경 변수로 읽는다.
- `.env`는 `.gitignore`에 들어 있다. 커밋 전에 `git status`로 `.env`가 없는지 본다. PR 검사도 키 모양 글자와 `.env`를 막는다.
- 키를 코드·설정·커밋 메시지·이슈·노션·채팅에 붙이지 않는다. 실수로 올렸으면 바로 PL에게 알리고 그 키를 폐기한다([팀 협업 규칙](../06_팀협업규칙_v1_100523.md)).

## 9. 작업 전 확인

| 확인 | 명령 | 정상 |
|---|---|---|
| RMW | `echo $RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| 도메인 | `echo $ROS_DOMAIN_ID` | `60` |
| 다른 팀 섞임 | `ros2 node list` | 우리 노드만 |
| 로봇 연결(실기) | `ping -c 3 192.168.1.100` | 응답 |
| 정지 준비 | 프로그램 시작 화면 | '[정지 준비] … 있음' 두 줄 |
| 컨테이너 | `docker ps -a` | 내가 띄운 것만 내 이름으로 |
| 키 | `git status` | `.env` 없음 |
| 안전 | 펜던트 | 든 사람이 있다. 두산 작업 공간 제한·협동 속도 제한이 켜져 있다 |

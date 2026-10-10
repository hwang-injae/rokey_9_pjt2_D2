# 개발 환경 설정 (env)

| 항목 | 내용 |
|---|---|
| OS · ROS | Ubuntu 24.04 LTS + ROS 2 Jazzy (Python 3.12). 팀 PC 모두 같은 버전 |
| 로봇 | 두산 M0609 + 두산 드라이버(doosan-robot2) + MoveIt2 2.12. 팀 브링업 = 박진용 `real_moveit.launch.py` (`mode:=virtual`·`real`) |
| 그리퍼 | OnRobot RG2 |
| 카메라 | 손목 RealSense D435i(깊이 있음) — 로봇 PC. **웹캠은 쓰지 않는다**(10/6 주제 개편 E-19) |
| ROS 2 통신 | **로봇 PC 안에서만**: CycloneDDS(`rmw_cyclonedds_cpp`), `ROS_DOMAIN_ID` = 60 (팀 60번대). **PC 사이는 MQTT**(10/6 E-27) |
| 컨테이너 | Docker compose, **웹 PC에만**(10/6 E-31): `mosquitto`(MQTT 브로커 1883) · `db`(PostgreSQL 16) · `web`(backend FastAPI :8000 + frontend 정적 파일 · AI 생성 · 저장소, 안). 셋 다 황인재. 로봇 PC는 전부 호스트 |
| PC | 2대: **로봇 PC**(ROS 2 노드 전부) + **웹 PC**(ROS 없음 — 브로커 · DB · 웹 · 음성). 유선 LAN으로 연결 |

이 저장소 워크스페이스(`rokey_9_pjt2_D2`)는 두산 드라이버 워크스페이스 위에 겹쳐 쓴다. ROS 2 Jazzy와 두산 드라이버를 처음부터 까는 순서는 강의 설치 안내를 따른다. 이 문서는 **팀이 맞춰야 하는 것**만 적는다.

## 목차

| 절 | 내용 |
|---|---|
| 1. PC 2대 역할 | 로봇 PC · 웹 PC에 무엇을 켜나, 늘리는 기준 |
| 2. 저장소 받기 | `git clone`, 빌드 순서 |
| 3. 버전 확인 | OS · Python · ROS 2 · **3-1 로봇 PC 패키지 판**(두산 드라이버 · 브링업 바탕) |
| 4. 로봇 브링업 | 팀 브링업(가상 · 실기), 유선 IP, 정지 준비 확인 |
| 5. RG2 · RealSense D435i | 장치 연결과 확인 |
| 6. CycloneDDS · ROS_DOMAIN_ID | 로봇 PC 안 설치 · `.bashrc` (PC 사이 DDS는 안 씀) |
| 6-1. MQTT | 브로커(웹 PC 컨테이너) · 클라이언트 설치 · PC 2대 연결 확인 |
| 7. Docker | 웹 PC compose(`mosquitto` · `db` · `web`) · 설치 · `.env` · 로컬 폴더 연결 · 지우기 전 확인 |
| 8. 키 (`.env`) | OpenAI 키를 두는 규칙 |
| 9. 작업 전 확인 | 매일 켤 때 보는 점검표 |

## 1. PC 2대 역할

| PC | 켜는 것 | 연결 장치 | 돌리는 곳 |
|---|---|---|---|
| **로봇 PC** | 브링업(두산 드라이버·제어기·MoveIt2 move_group), 집기·놓기 + 실행기, 장면 관리, 그리퍼 노드, **정지 노드**, 손목 블록 인식 + **스캔 추론기**, **작업 관리자 · 작업 판단 · 검사 묶음(task)**, **다리 `bridge`(ROS ↔ MQTT)** | 로봇·그리퍼(유선 랜), RealSense D435i(USB), 웹 PC(유선 LAN) | 호스트(ROS 2 Jazzy) |
| **웹 PC**(**ROS 없음**) | 웹 화면(`web/frontend`, 브라우저에서 실행) · **AI 설계 생성 · 저장소**(`web/backend`), **DB**, **MQTT 브로커**, 음성(마이크) | 마이크 · 화면 · 로봇 PC(유선 LAN) | compose `mosquitto` · `db` · `web` + 호스트 음성 |

- 어느 PC가 로봇 PC·웹 PC인지는 (미정)이다. 정하면 여기에 적는다(웹 PC IP는 `robot.yaml` `mqtt.host`에).
- 로봇 PC 노드는 어느 PC에서나 돌게 짠다. 옮기지 않는 것은 브링업·정지 노드(로봇 PC)이고, 카메라는 그 카메라를 처리하는 PC에 꽂는다. **웹 PC에는 ROS 2·두산 환경을 깔지 않는다**(10/6 E-26).
- **늘리는 기준:** 책상 가운데 기둥 조립 실기(W118) 때 잰 부하가 CPU 70%를 넘거나 카메라 처리가 초당 15장 아래면 task · 다리를 3번째 PC로 옮긴다(W053).
- **컨테이너는 성능을 바꾸지 않는다(S-15).** 과부하는 노드를 다른 PC로 옮기거나 처리량(해상도 · Hz)을 줄여 푼다. 컨테이너는 필요한 것만(브로커 · DB · 웹 서버) 쓴다(PL 10/6).


## 2. 저장소 받기

```bash
# 위치는 자유. 저장소 폴더 = 우리 ROS 2 워크스페이스
git clone https://github.com/hwang-injae/rokey_9_pjt2_D2.git
cd rokey_9_pjt2_D2
```

- 브랜치·커밋·PR 규칙은 [팀 협업 규칙](../06_팀협업규칙_v1_100821.md)에 있다.
- source 순서는 늘 같다: `/opt/ros/jazzy` → 두산 워크스페이스 → 이 저장소. 빌드 명령은 [src/README](../../src/README.md) '빌드 · 시험'.

## 3. 버전 확인

```bash
lsb_release -ds            # Ubuntu 24.04.x
python3 --version          # Python 3.12.x
printenv ROS_DISTRO        # jazzy (source 뒤)
```

### 3-1. 로봇 PC 패키지 판 (박진용, 10/6)

로봇 PC의 두산 드라이버 · 브링업 바탕 패키지는 아래 판으로 맞춘다. 둘 다 이 저장소 밖(두산 워크스페이스)에 있다.

| 패키지 | 저장소 · 브랜치 | 판(커밋) | 주의 |
|---|---|---|---|
| 두산 드라이버 `doosan-robot2` | `ROKEY-SPARK/doosan-robot2_jazzy` · `main` | `31750d6` (7/10) | 교육 과정 배포본(포크) |
| 브링업 바탕 `m0609_rg2_integration` | `ROKEY-SPARK/m0609_rg2_integration` · `jazzy` | `e80512d` (8/11) | **`a152736`(7/21) 이전 판은 쓰지 않는다** — 모델 파일에 `ros2_control`이 없어 `real_moveit` 제어기가 안 붙는다(민범진 PC에서 나옴. R-01 시험 기록 3장의 현상과 같다) |

```bash
# 판 확인 — 두산 워크스페이스 src 안에서
git -C <doosan-robot2 경로> log -1 --oneline                # 31750d6
git -C <m0609_rg2_integration 경로> log -1 --oneline        # e80512d
git -C <m0609_rg2_integration 경로> branch --show-current   # jazzy
```

## 4. 로봇 브링업

팀 브링업은 박진용 `real_moveit.launch.py` 하나로 통일한다(D-09). M0609 + RG2 + 손목 카메라 모델, 두산 드라이버, MoveIt2를 한 번에 켠다. **`d2_bringup` 패키지로 옮겼다(10/6)**(로봇 동작, 인프라·통합, W035).

```bash
# 가상 (에뮬레이터) — 로봇 없이
ros2 launch d2_bringup real_moveit.launch.py mode:=virtual

# 실기
ros2 launch d2_bringup real_moveit.launch.py mode:=real host:=192.168.1.100
```

- **실기 PC 유선 IP:** IPv4를 수동으로 `192.168.1.x`, 넷마스크 255.255.255.0. x는 로봇(.100)·그리퍼(.1)와 겹치지 않게 한다. 연결 확인은 `ping -c 3 192.168.1.100`, `ping -c 3 192.168.1.1`.
- 실기 모드 사전 설정(재부팅하면 풀린다): `sudo sysctl -w net.ipv4.ip_unprivileged_port_start=0`
- 티치펜던트와 ROS가 로봇을 동시에 제어하지 않게 한다. **실기 중 펜던트 안전 설정 화면에 들어가지 않는다** — 서보가 꺼지고(SAFE_OFF) ROS 서보 명령이 무시돼 브링업을 다시 켜야 풀린다([복구 절차](../복구절차_정지뒤다시시작_v1_101008.md) §3-2, 10/7 실기).
- 런치는 **패키지 이름으로** 실행한다(`ros2 launch d2_bringup …`). `src/…` 경로로 실행하면 `__pycache__`가 생긴다.
- 정지 처리가 들어간 프로그램은 시작할 때 아래 두 줄이 나와야 한다. `없음`이나 `안 보인다`가 나오면 실기를 하지 않는다(R-01).

  ```
  [정지 준비] 서기 궤적 컨트롤러: 있음
  [정지 준비] 두산 move_stop 서비스: …/motion/move_stop
  ```

## 5. RG2 · RealSense D435i

| 장치 | 연결 | 확인 | 주의 |
|---|---|---|---|
| RG2 | 로봇 네트워크(192.168.1.1), Modbus | 그리퍼 노드(`d2_gripper`)로 열기·닫기 | **그리퍼 노드 하나만** 그리퍼에 명령한다. 치수·TCP·표시폭 값은 `robot.yaml`(10/6 측정) |
| RealSense D435i | 로봇 PC USB 3 | `ros2 launch realsense2_camera rs_launch.py` 뒤 `ros2 topic list`에 카메라 토픽 | 깊이 최소 거리 약 28 cm → 관측 자세는 30~40 cm 위. 손목 보정 4.8 cm 어긋남(R-05)은 10/6 W058에서 고쳤다(z +49.4 mm → 재측정 −0.5 mm) |

```bash
# RealSense ROS 패키지 (없으면)
sudo apt install ros-jazzy-realsense2-camera
```

- 파이썬 주의: 웹캠(MediaPipe)을 빼서(10/6 E-19) numpy 충돌 걱정이 줄었다. 스캔 추론기 · 검사기는 numpy · 표준 라이브러리로, OpenAI 라이브러리는 웹 PC `web/backend`(컨테이너 `web`)와 호스트 `voice.py`에서만 쓴다.

## 6. CycloneDDS · ROS_DOMAIN_ID (로봇 PC 안)

**로봇 PC**의 노드들이 **같은 RMW(CycloneDDS)와 같은 `ROS_DOMAIN_ID`**를 써야 서로 보인다. 우리 팀은 **60번대**(60~69)를 쓴다 — 로봇 PC = **60**. **PC 사이에는 DDS를 쓰지 않는다**(10/6 E-27 — 웹 PC는 ROS가 없고, 로봇 PC ↔ 웹 PC는 6-1 MQTT). 혼자 가상 시험하는 개발 PC는 61~69를 각자 써도 된다(안).

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

`~/.bashrc` 맨 아래에 더한다(로봇 쪽 개발 PC도 같게).

```bash
# --- D2 협동2 ---
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=60          # 팀 60번대. 로봇 PC는 60
```

- 바꾼 뒤 새 터미널을 열고 `ros2 daemon stop`을 한 번 한다(예전 설정으로 떠 있던 데몬을 끈다).
- **🆕 로봇에 붙을 수 있는 PC는 모두(한세교 · 박진용) `.bashrc` 기본값을 위 값으로 둔다(10/7 PL, W029).** 설정 스크립트(`rokey4_set` 등)를 실행해야만 60 · CycloneDDS가 잡히고 새 터미널 기본값이 협동1(Fast DDS · 50)이면, 그 터미널에서 띄운 노드(특히 **정지 노드**)가 다른 노드와 서로 안 보여 정지 명령이 닿지 않는다. 확인: 새 터미널에서 `echo $RMW_IMPLEMENTATION $ROS_DOMAIN_ID` → `rmw_cyclonedds_cpp 60`.
- 지난 프로젝트 설정(Fast DDS 설정 파일, `ROS_DISCOVERY_SERVER`, 다른 `ROS_DOMAIN_ID`)이 `.bashrc`에 남아 있으면 주석 처리한다. 같은 변수가 두 번 있으면 아래 것이 이긴다.
- 한 PC 안에서만 돌므로 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`를 켜 두어도 된다(다른 팀 노드가 섞이는 것을 막음 — 안). 강의실 무선망 멀티캐스트 · `CYCLONEDDS_URI` 걱정은 없어졌다.
- 다른 팀 노드가 섞이는지: 브링업 뒤 `ros2 node list`에 다른 팀 `/dsr01` 등이 보이면 번호를 다시 본다.

## 6-1. MQTT (PC 사이 — 웹 PC 브로커 ↔ 로봇 PC 다리)

PC 사이 통신은 **MQTT**다(10/6 E-27, 규칙은 [IRD 10장](../02_인터페이스_IRD_v3_101013.md#10-pc-사이-통신--mqtt-다리-e-27e-30)). 웹 PC의 컨테이너 `mosquitto`(포트 1883)가 브로커이고, 로봇 PC의 ROS 노드 `bridge`(`d2_bridge`, paho-mqtt)가 ROS ↔ MQTT를 바꾼다. 웹 백엔드 · 음성도 paho-mqtt로 브로커에 붙는다.

```bash
# 두 PC 모두 — 확인용 클라이언트
sudo apt install mosquitto-clients
# 로봇 PC — 다리 노드가 쓰는 파이썬 라이브러리 (ROS 파이썬과 같은 인터프리터)
sudo apt install python3-paho-mqtt        # 없으면 pip3 install --user paho-mqtt (venv 밖에서 numpy는 건드리지 않는다)
# 웹 PC — web/backend 의존성에 paho-mqtt 포함(compose 이미지 안)
```

| 확인 | 명령(어디서) | 정상 |
|---|---|---|
| 브로커가 떠 있나 | 웹 PC `docker compose -f web/compose.yaml ps` | `mosquitto` Up, 1883 |
| 로봇 PC에서 브로커가 보이나 | 로봇 PC `mosquitto_pub -h <웹 PC IP> -t d2/test -m hi` + 웹 PC `mosquitto_sub -t 'd2/#' -v` | 웹 PC 창에 `d2/test hi` |
| 다리가 붙었나 | 웹 PC `mosquitto_sub -t 'd2/bridge/alive' -v` | 1초마다 `{"alive":true,…}` |
| 끝까지 | 화면 [출발] → `d2/hmi/command/req` → 로봇 PC → `d2/task/state`가 바뀜 | `mosquitto_sub -t 'd2/#' -v`로 전부 보임 |

- 웹 PC IP는 `robot.yaml` `mqtt.host`(로봇 PC가 읽음). 브로커는 유선 LAN 안에서만 연다(인터넷 노출 없음). 인증은 compose(W102) 때 정함(안: 없음 — IRD 12장, 일정표 기준).
- 로봇 쪽 개발 PC는 웹 PC 없이 `mock_bridge`(ROS — 아직 없음)로, 웹 쪽 개발은 로봇 PC 없이 로컬 `docker run -p 1883:1883 eclipse-mosquitto` + `web/backend/mock_robot.py`로 한다. 다리 없이 task를 가상으로 돌릴 때는 웹 연결 감시를 끈다 — `ros2 run d2_task task --ros-args -p monitor_hmi:=false`(기본 true라 안 끄면 `/d2/hmi/alive`가 없어 출발 · 스캔이 거절됨, 10/8 E-62). 설계도 웹 없이 읽으려면 `-p design_source:=local -p recipe_dir:=<레시피 폴더>`([src/README](../../src/README.md) '빌드 · 시험').


## 7. Docker

```bash
sudo apt install docker.io
sudo usermod -aG docker $USER      # 한 번 로그아웃했다 들어오면 sudo 없이 쓴다
docker --version
```

컨테이너는 **웹 PC에만** 둔다(10/5 S-15 → 10/6 E-19 → **10/6 18시 E-31**). 로봇 PC(ROS · 두산 · RealSense)는 전부 호스트에서 바로 돌린다. 정의는 `web/compose.yaml` 하나(황인재).

| 컨테이너(서비스 이름) | 안에서 도는 것 | 따로 두는 이유 | 만드는 사람 (S-17) |
|---|---|---|---|
| `mosquitto` | MQTT 브로커(eclipse-mosquitto, 1883) — `web/mosquitto/mosquitto.conf` | PC 사이 통신의 가운데 서버. 설치 없이 이미지 1줄 | 황인재 · 10/6 저녁~10/7 (W102) |
| `db` | PostgreSQL 16(`designs` · `builds`, 10/6 E-39) | DB 프로그램은 컨테이너로 띄우는 것이 가장 쉽다. 데이터는 볼륨 · 로컬 폴더 | 황인재 · W088(일정표 기준) |
| `web`(안) | backend(FastAPI :8000 — REST · WebSocket `/ws` · paho-mqtt · **AI 설계 생성(GPT-4o) · 저장소 인터페이스**) 한 프로세스 + frontend 정적 파일(Next.js + three.js — 브라우저에서 실행, 10/7 E-41) | 웹 서버 · OpenAI 라이브러리를 한 이미지에. `env_file .env`로 키. compose(W102) 전에는 호스트로 띄워도 된다 | 황인재 · backend 최소형(W126) 10/9 · 저장소(W111, JSON 폴더) · 화면 10/10 저녁부터 · compose(W102) — 일정표 기준(10/8에 잡은 날짜는 못 지킴) |

| 호스트에서 바로 | 이유 |
|---|---|
| 음성 (웹 PC) | 마이크 장치를 컨테이너에 넘기기가 번거롭다. `web/backend/voice.py` → Whisper API → MQTT |
| 로봇 PC 전부(브링업 · 동작 · 그리퍼 · 정지 · 비전 · task · 다리) | 10/3에 시험한 호스트 환경(두산 드라이버 · MoveIt2 · RealSense)을 그대로 쓴다. 다리는 paho-mqtt만 더 깐다 |

**컨테이너는 그 안에 넣는 것의 담당이 만든다(10/5 S-17).** 브로커는 W102로 먼저 띄워 쓰고, 두 PC 연결 확인은 W129(일정표). 1시간 넘게 막히면 일단 호스트로 돌리고 compose(10/12) 때 마저 한다.

컨테이너는 성능을 바꾸지 않는다(1장). DB는 PostgreSQL 16(10/6 21시 E-39) — W088 전에는 저장소 인터페이스가 JSON 파일 폴더로 돈다. 띄우는 모양은 아래와 같다(안).

```bash
cd <저장소>/web
cp .env.example .env          # OPENAI_API_KEY=… · MQTT_HOST=mosquitto · DB 접속값 (값은 적지 않는다)
docker compose up -d          # mosquitto · db · web
docker compose ps
docker compose logs -f web    # 키 읽힘(값은 안 찍음) · 브로커 연결 · DB 연결 확인
```

| 규칙 | 내용 |
|---|---|
| 네트워크 | compose 기본 네트워크. `mosquitto`만 `ports: "1883:1883"`으로 LAN에 연다(로봇 PC 다리가 붙음). `web`은 `8000`(안) — backend가 frontend 정적 파일 · REST · WebSocket `/ws`를 같은 포트로 낸다(브라우저 `http://<웹 PC IP>:8000`). ROS가 없으므로 `--net=host` · DDS 설정이 필요 없다 |
| 키는 `env_file: .env` | 키를 Dockerfile·이미지·compose 파일에 넣지 않는다 |
| 원본 자료는 로컬 연결 | 녹화(rosbag)·대용량 CAD·DB 데이터·스캔 점군·사진은 이미지에 넣지 않고(`COPY` 금지) 로컬 폴더를 `volumes:`로 연결한다. 읽기만 할 원본은 `:ro`. 설정·레시피도 복사하지 않고 로컬 저장소 폴더를 연결해 읽는다. DB 데이터는 이름 있는 볼륨이나 로컬 폴더에 둔다 |
| 이름 | compose 서비스 이름 `mosquitto` · `db` · `web`. 따로 이름을 붙이면 파트·이름(예: `web-<이름>`) |
| 지우기 전 확인 | 지우기 전에 `docker ps -a`로 이름·이미지·만든 사람을 본다. 내가 만든 것이 아니면 먼저 묻는다. `docker rm -f $(docker ps -aq)`, `docker system prune` 같은 한꺼번에 지우는 명령은 쓰지 않는다 |
| 장치 | 컨테이너에 넘기는 장치는 없다(웹캠 없음). 마이크는 넘기지 않는다 — 음성은 호스트에서 돈다(S-15) |


## 8. 키 (`.env`)

- OpenAI 키는 **PC마다 `.env` 파일에만** 둔다. 코드는 환경 변수로 읽는다.
- `.env`는 `.gitignore`에 들어 있다. 커밋 전에 `git status`로 `.env`가 없는지 본다. PR 검사도 키 모양 글자와 `.env`를 막는다.
- 키를 코드·설정·커밋 메시지·이슈·노션·채팅에 붙이지 않는다. 실수로 올렸으면 바로 PL에게 알리고 그 키를 폐기한다([팀 협업 규칙](../06_팀협업규칙_v1_100821.md)).

## 9. 작업 전 확인

| 확인 | 명령 | 정상 |
|---|---|---|
| RMW(로봇 PC) | `echo $RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| 도메인(로봇 PC) | `echo $ROS_DOMAIN_ID` | `60` |
| 브로커 · 다리 | 웹 PC `mosquitto_sub -t 'd2/bridge/alive' -v` | 1초마다 `alive: true` |
| 다른 팀 섞임 | `ros2 node list` | 우리 노드만 |
| 로봇 연결(실기) | `ping -c 3 192.168.1.100` | 응답 |
| 정지 준비 | 프로그램 시작 화면 | '[정지 준비] … 있음' 두 줄 |
| 컨테이너(웹 PC) | `docker compose -f web/compose.yaml ps` | `mosquitto` · `db`(· `web`) Up |
| 키 | `git status` | `.env` 없음 |
| 안전 | 펜던트 | 든 사람이 있다. 두산 작업 공간 제한·TCP 속도 제한이 켜져 있다 |
| 키 · 인터넷 | 웹 PC `web` 시작 로그 | OpenAI 키 읽힘(값은 안 찍음), 브로커 · DB 연결 |

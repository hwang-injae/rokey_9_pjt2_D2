# vision 컨테이너 (W101)

웹캠 사람 감지 노드 `webcam_human`(d2_vision)을 MediaPipe와 함께 돌리는 컨테이너. 운영 PC에서 쓴다.
MediaPipe의 numpy 2가 호스트 ROS의 numpy 1.26과 부딪치는 것을 피하려고 따로 둔다(`docs/env/README.md` 7장).

## 이미지 만들기 (한 번)
```bash
docker build -t d2-vision:dev docker/vision
```
이미지에는 저장소·모델·`.env`가 들어가지 않는다(COPY 없음). 실행할 때 `-v` / `--env-file`로 연결한다.

## 쓴 버전과 이유
- 베이스 `ros:jazzy-ros-base` (Ubuntu 24.04 · Python 3.12 · ROS 2 Jazzy). RMW `rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=60`이 기본값.
- **mediapipe 1.0.1** — pip에 올라온 버전 중 최신(1.0.1, 1.0.0, 0.10.13~0.10.35)이고 Python 3.12에서 설치됨을 이미지 빌드로 확인했다.
  노드가 쓰는 `mp.tasks.vision.PoseLandmarker`(Tasks API)가 이 버전에서 있고, `arm_track.py`(V-06)도 1.0.1로 확인한 것이다. OpenCV(cv2 5.0.0)·numpy 2.5.3은 같이 깔린다.
- `cv_bridge`는 `webcam_human`이 안 쓰므로 넣지 않았다.
- apt의 numpy 1.26은 pip가 못 지운다(RECORD 없음). 그래서 `--ignore-installed numpy`로 `/usr/local`에 numpy 2를 깔아 가렸다. numpy에 기대는 ROS 패키지를 이 컨테이너에서 쓰려면 다시 확인한다.

## 실행 (카메라 번호는 W055 때 `ls /dev/video*`로 확인해 넣는다)
```bash
docker run -it --rm --name vision-<이름> \
  --net=host --ipc=host \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  --device /dev/video<번호> \
  -v <저장소 폴더>:/ws/rokey_9_pjt2_D2 \
  -v <모델 폴더>:/data/models:ro \
  d2-vision:dev bash
```
컨테이너 안에서:
```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/l build --packages-select d2_vision --build-base /tmp/b --install-base /tmp/i
source /tmp/i/setup.bash
ros2 run d2_vision webcam_human --ros-args \
  -p config_file:=/ws/rokey_9_pjt2_D2/src/d2_bringup/config/robot.yaml \
  -p calib_file:=/data/models/webcam_H.json \
  -p model_path:=/data/models/pose_landmarker_lite.task \
  -p camera_index:=0
```
- `--user`는 호스트 파일이 root 소유가 되는 것을 막는다. 빌드 결과는 컨테이너 안 `/tmp`에 두어 호스트 `build/`·`install/`과 섞이지 않는다.
- `pose_landmarker_lite.task`와 `webcam_H.json`은 git에 올리지 않는 로컬 폴더에 둔다(위 예는 같은 `/data/models`에 둠).
- `config_file`의 `webcam_zones` 좌표는 W055 보정 뒤 `robot.yaml`에 들어온다. 비어 있으면 노드가 ERROR를 내고 시작하지 않는다.
- 컨테이너를 지울 때는 `docker ps -a`로 내 것인지 보고 이름으로 하나씩 지운다. `docker system prune`·전체 삭제 금지.

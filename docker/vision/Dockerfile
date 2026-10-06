# vision 컨테이너 (W101) — 웹캠 사람 감지 노드 webcam_human 전용. 운영 PC에서 돈다.
# 따로 두는 이유: MediaPipe 의 numpy 2 와 호스트 ROS(cv_bridge)의 numpy 1.26 이 부딪친 기록이 있다.
# 저장소·모델·원본 자료·.env 는 이미지에 넣지 않는다(COPY 없음). 실행할 때 -v / --env-file 로 연결한다. 실행법은 README.md.
FROM ros:jazzy-ros-base

# cv_bridge 는 webcam_human 이 안 쓰므로 넣지 않는다. cv2 용 공유 라이브러리(libgl1·libglib2.0-0)만 더한다.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
        python3-pip \
        python3-yaml \
        libgl1 \
        libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# MediaPipe(OpenCV·numpy 2 가 같이 깔린다)는 이 컨테이너 안에만 깐다. Ubuntu 24.04 는 시스템 pip 를 막아 두어 옵션이 필요하다.
# apt 로 깔린 numpy 1.26 은 pip 가 지우지 못한다(RECORD 없음). 그래서 지우지 않고 /usr/local 에 numpy 2 를 따로 깔아
# 그 위에 가린다(--ignore-installed). webcam_human 은 numpy 를 쓰는 ROS 패키지(cv_bridge 등)를 안 쓴다.
# mediapipe 1.0.1 = 2026-10-06 현재 최신, arm_track.py(V-06)가 이 버전의 Tasks API 로 확인됨. 선택 이유는 README.md.
RUN pip install --no-cache-dir --break-system-packages --ignore-installed numpy "mediapipe==1.0.1"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=60

WORKDIR /ws/rokey_9_pjt2_D2

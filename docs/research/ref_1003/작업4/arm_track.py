"""V-06 웹캠 팔·손 추적 속도 재기. MediaPipe Tasks API 를 쓴다 (mediapipe 1.0.1 에서 확인).
mediapipe 0.10.30 부터는 mp.solutions(예전 Pose/Hands 예제)가 없다. 그래서 mp.tasks 를 쓴다.

필요: pip install "mediapipe==1.0.1"   (OpenCV 는 같이 깔린다. opencv-python 을 따로 깔지 않는다)
      이 파일과 같은 폴더에 pose_landmarker_lite.task, hand_landmarker.task
실행: python3 arm_track.py --label 천천히                 # 웹캠 0번, 30초
      python3 arm_track.py --cam 1 --label 두사람         # 다른 카메라 (ls /dev/video* 로 번호 확인)
      python3 arm_track.py --video 시험.mp4 --no-show     # 영상 파일로 시험
      python3 arm_track.py --cam 1 --homography webcam_H.json --label 손뻗기  # 손목의 책상 좌표(mm)도 화면에 표시
        (webcam_H.json 은 webcam_calib.py calib 로 만든다. 손목이 책상 면에 닿아 있다고 보고 구한 좌표다.
         떠 있는 손목은 카메라에서 먼 쪽(+y)으로 밀려 보인다. 해상도는 따로 안 주면 calib 때 값을 쓴다)
화면: 어깨·팔꿈치·손목 = 초록, 손 = 노랑, 왼쪽 위에 처리 속도. q 를 누르면 일찍 끝난다.
끝나면 한 줄을 출력하고 arm_track_log.csv 에도 쌓는다.
  처리 속도(Hz) = 처리한 프레임 수 / 걸린 시간 (카메라 읽기 포함)
  손목 놓침(%) = 손목 점이 하나도 안 나온 프레임 비율 (보임 정도 0.5 미만은 안 나온 것으로 본다)
  시험하는 동안 손목이 화면 밖으로 나가지 않게 한다. 그래야 '손목 놓침'이 '놓친 프레임'이 된다.
"""
import argparse
import csv
import datetime
import os
import time

import cv2
import mediapipe as mp

HERE = os.path.dirname(os.path.abspath(__file__))
V = mp.tasks.vision
ARM = (11, 12, 13, 14, 15, 16)   # 왼·오른 어깨, 팔꿈치, 손목
WRISTS = (15, 16)


def main():
    ap = argparse.ArgumentParser(description="V-06 팔·손 추적 속도")
    ap.add_argument("--cam", type=int, default=0, help="웹캠 번호 (기본 0)")
    ap.add_argument("--video", default="", help="웹캠 대신 영상 파일")
    ap.add_argument("--secs", type=float, default=30.0, help="시험 시간(초), 기본 30")
    ap.add_argument("--label", default="", help="상황 이름 (기록용)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--no-hands", action="store_true", help="손 추적을 끈다 (속도 비교용)")
    ap.add_argument("--no-show", action="store_true", help="화면을 띄우지 않는다")
    ap.add_argument("--homography", default="", help="webcam_calib.py 가 만든 webcam_H.json. 주면 손목의 책상 좌표(mm)를 표시")
    a = ap.parse_args()

    H = cal = None
    if a.homography:
        from webcam_calib import load_calib, px_to_mm   # 같은 폴더의 webcam_calib.py
        cal = load_calib(a.homography)
        H = cal["H"]
        if not a.video and (a.width, a.height) == (640, 480):   # 해상도를 안 줬으면 calib 때 값으로 연다
            a.width, a.height = int(cal["width"]), int(cal["height"])

    pose = V.PoseLandmarker.create_from_options(V.PoseLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=os.path.join(HERE, "pose_landmarker_lite.task")),
        running_mode=V.RunningMode.VIDEO, num_poses=2))
    hands = None
    if not a.no_hands:
        hands = V.HandLandmarker.create_from_options(V.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=os.path.join(HERE, "hand_landmarker.task")),
            running_mode=V.RunningMode.VIDEO, num_hands=4))

    cap = cv2.VideoCapture(a.video if a.video else a.cam)
    if not a.video:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    if not cap.isOpened():
        raise SystemExit("카메라나 영상을 열 수 없다. --cam 번호를 바꿔 본다 (ls /dev/video*)")

    n = wrist_seen = hand_seen = max_people = 0
    last_ts = -1
    t0 = time.monotonic()
    win_t, win_n, shown_hz = t0, 0, 0.0
    size = ""
    while True:
        ok, frame = cap.read()
        now = time.monotonic()
        if not ok or now - t0 > a.secs:
            break
        h, w = frame.shape[:2]
        if cal is not None and not size and (w, h) != (cal["width"], cal["height"]):
            print("[주의] 화면 %dx%d 가 calib 때(%sx%s)와 다르다. 책상 좌표가 틀린다. --width --height 를 calib 때와 같게 준다."
                  % (w, h, cal["width"], cal["height"]))
        size = "%dx%d" % (w, h)
        ts = max(int((now - t0) * 1000), last_ts + 1)   # VIDEO 모드는 시각(ms)이 계속 커져야 한다
        last_ts = ts
        img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        pr = pose.detect_for_video(img, ts)
        hr = hands.detect_for_video(img, ts) if hands else None
        n += 1
        max_people = max(max_people, len(pr.pose_landmarks))
        if any((p[i].visibility or 0) >= 0.5 for p in pr.pose_landmarks for i in WRISTS):
            wrist_seen += 1
        if hr is not None and hr.hand_landmarks:
            hand_seen += 1
        for p in pr.pose_landmarks:
            for i in ARM:
                if (p[i].visibility or 0) >= 0.5:
                    cv2.circle(frame, (int(p[i].x * w), int(p[i].y * h)), 6, (0, 200, 0), -1)
                    if H is not None and i in WRISTS:   # 손목의 책상 좌표 (손목이 책상 면에 있다고 볼 때)
                        x, y = px_to_mm(H, p[i].x * w, p[i].y * h)
                        cv2.putText(frame, "%.0f,%.0f mm" % (x, y), (int(p[i].x * w) + 8, int(p[i].y * h) - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if hr is not None:
            for hand in hr.hand_landmarks:
                for lm in hand:
                    cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 3, (0, 220, 255), -1)
        win_n += 1
        if now - win_t >= 1.0:
            shown_hz, win_t, win_n = win_n / (now - win_t), now, 0
        if not a.no_show:
            cv2.putText(frame, "%.1f Hz  people %d" % (shown_hz, len(pr.pose_landmarks)), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow("arm_track (q: quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    elapsed = time.monotonic() - t0
    cap.release()
    cv2.destroyAllWindows()
    if n == 0:
        raise SystemExit("프레임을 하나도 못 읽었다")
    hz = n / elapsed
    miss = 100.0 * (n - wrist_seen) / n
    hand_pct = 100.0 * hand_seen / n if hands else float("nan")
    print("[%s] 처리 속도 %.1f Hz | 손목 놓침 %.1f %% | 손 검출 %.1f %% | 최대 사람 수 %d | 프레임 %d개, %.1f초, %s"
          % (a.label or "-", hz, miss, hand_pct, max_people, n, elapsed, size))
    log = os.path.join(os.getcwd(), "arm_track_log.csv")
    new = not os.path.exists(log)
    with open(log, "a", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        if new:
            wr.writerow(["시각", "상황", "입력", "해상도", "프레임", "초", "Hz", "손목놓침%", "손검출%", "최대사람수"])
        wr.writerow([datetime.datetime.now().strftime("%H:%M:%S"), a.label, a.video or "cam%d" % a.cam, size, n,
                     round(elapsed, 1), round(hz, 1), round(miss, 1), round(hand_pct, 1), max_people])


if __name__ == "__main__":
    main()

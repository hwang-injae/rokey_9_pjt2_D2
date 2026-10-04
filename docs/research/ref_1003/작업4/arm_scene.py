"""V-06 웹캠 팔 추적 -> MoveIt2 장면 연결. 웹캠으로 찾은 팔꿈치·손목 자리에 원기둥을 세워 MoveIt2 장면에 넣고 계속 옮긴다.
그래서 V-10 처럼 '장애물이 생기면 경로를 다시 짜는' 시험을 진짜 사람 팔로 할 수 있다.

먼저 할 것:
  1. python3 webcam_calib.py calib 로 webcam_H.json 을 만든다 (카메라를 옮기면 다시 한다).
  2. source <내 워크스페이스>/install/setup.bash
  3. MoveIt2 브링업을 띄운다. 처음 시험은 가상(virtual) 모드로 한다.
  4. python3 ../작업3/scene_setup.py --tcp <TCP>   # 책상과 그리퍼 대신 원기둥을 먼저 넣는다
필요: pip install "mediapipe==1.0.1". 이 파일과 같은 폴더에 pose_landmarker_lite.task, webcam_calib.py
      mediapipe 를 venv 에 깔았으면 그 venv 에서 pip install pyyaml 도 한다 (rclpy 가 쓴다).
실행: python3 arm_scene.py --h webcam_H.json --dry-run       # ROS 없이 좌표만 출력. 먼저 이것으로 시험
      python3 arm_scene.py --h webcam_H.json --cam 1         # MoveIt2 장면에 human_0 ~ human_7 원기둥
      python3 arm_scene.py --h webcam_H.json --hz 2 --radius 0.15 --no-show
화면: 책상 격자, 팔꿈치·손목 = 초록 점과 x,y mm. q 또는 Ctrl+C 로 끝내면 넣은 원기둥을 지우고 끝난다.
  2초마다 출력: 카메라+MediaPipe 속도(Hz), 장면 갱신 속도(Hz), 마지막 서비스 호출 시간(ms), 물체 수.
원기둥: 보이는(보임 정도 0.5 이상) 팔꿈치·손목마다 1개, 세로로 세운다. 반지름 --radius, 높이 --height (m), base_link 기준.
  이번에 안 보인 점의 원기둥은 지운다. human_0 ~ human_7 만 넣고 지운다.
  다른 물체(책상, structure, gripper_standin, scene_setup.py 의 human_arm)는 건드리지 않는다.
  좌표는 '점이 책상 면에 있다'고 보고 구한다. 떠 있는 팔은 카메라에서 먼 쪽(+y)으로 밀려 보인다
  (100 mm 위 -> 약 85 mm, webcam_calib.py selftest 값). 그래서 반지름을 넉넉히 둔다.
안전: 실기에서 이 스크립트는 '편의 기능'일 뿐이다. 이것만 믿지 않는다.
  로봇의 충돌 감지와 느린 속도는 그대로 켜 둔다. 조작자는 티치 펜던트를 손에 들고 있는다.
  로봇이 움직이는 동안 손을 작업 영역에 넣지 않는다.
"""
import argparse
import os
import sys
import time

try:
    import cv2
    import mediapipe as mp
except ImportError as e:
    raise SystemExit('mediapipe 를 못 찾았다 (%s). pip install "mediapipe==1.0.1" 로 깐 환경에서 실행한다' % e)

from webcam_calib import draw_grid, load_calib, px_to_mm   # 같은 폴더의 webcam_calib.py

HERE = os.path.dirname(os.path.abspath(__file__))
V = mp.tasks.vision
JOINTS = (13, 14, 15, 16)                   # 왼·오른 팔꿈치, 손목
IDS = ["human_%d" % i for i in range(8)]    # 사람 2명 x 점 4개. 이 이름만 넣고 지운다
FRAME = "base_link"


def find_arms(poses, w, h, H):
    """보이는 팔꿈치·손목 -> {이름: (x, y) m}. 사람 k 의 j 번째 점 = human_(4k+j)."""
    out = {}
    for k, p in enumerate(poses[:2]):
        for j, i in enumerate(JOINTS):
            if (p[i].visibility or 0) < 0.5:
                continue
            x, y = px_to_mm(H, p[i].x * w, p[i].y * h)
            if not (abs(x) <= 3000 and abs(y) <= 3000):   # 화면 위쪽 끝(지평선 근처)의 믿을 수 없는 값
                continue
            out[IDS[4 * k + j]] = (x / 1000.0, y / 1000.0)
    return out


def ros_open(ns, radius, height, table_z):
    """MoveIt2 장면 연결 (--dry-run 이 아닐 때만 부른다). scene_setup.py 와 같은 서비스, 같은 방식(is_diff=True).
    (send, scene_ids, close) 함수 셋을 돌려준다."""
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from shape_msgs.msg import SolidPrimitive
        from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    except ImportError as e:
        raise SystemExit("ROS 2 를 못 찾았다 (%s).\n"
                         "  먼저 source <내 워크스페이스>/install/setup.bash 를 한다.\n"
                         "  venv 에서 'No module named yaml' 이 나오면 그 venv 에서 pip install pyyaml.\n"
                         "  ROS 없이 시험하려면 --dry-run 을 붙인다." % e)
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C 뒤에도 원기둥을 지울 수 있게
    except ImportError:
        rclpy.init()
    node = rclpy.create_node("arm_scene", namespace=ns)
    apply_cli = node.create_client(ApplyPlanningScene, "apply_planning_scene")
    get_cli = node.create_client(GetPlanningScene, "get_planning_scene")

    def close():
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not (apply_cli.wait_for_service(timeout_sec=10.0) and get_cli.wait_for_service(timeout_sec=10.0)):
        close()
        raise SystemExit("[오류] move_group 서비스를 찾지 못했다. 브링업이 다 떴는지, --ns 가 브링업 name 과 같은지 본다.\n"
                         "       확인: ros2 service list | grep apply_planning_scene")

    def call(cli, req, timeout):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def make_obj(oid, x, y):
        o = CollisionObject()
        o.header.frame_id = FRAME
        o.id = oid
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.CYLINDER
        sp.dimensions = [float(height), float(radius)]
        o.primitives = [sp]
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(table_z + height / 2)
        p.orientation.w = 1.0
        o.primitive_poses = [p]
        o.operation = CollisionObject.ADD   # 같은 이름이 있으면 바꾼다 (= 옮긴다)
        return o

    def remove_obj(oid):
        o = CollisionObject()
        o.header.frame_id = FRAME
        o.id = oid
        o.operation = CollisionObject.REMOVE
        return o

    def scene_ids():
        """지금 장면에 있는 human_0 ~ human_7. 못 읽으면 None."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        res = call(get_cli, req, 2.0)
        return None if res is None else {o.id for o in res.scene.world.collision_objects if o.id in IDS}

    def send(add, remove):
        """add = {이름: (x, y) m} 를 넣거나 옮기고, remove = [이름] 을 지운다. 성공하면 True."""
        s = PlanningScene()
        s.is_diff = True
        s.robot_state.is_diff = True
        s.world.collision_objects = [make_obj(k, x, y) for k, (x, y) in add.items()] + [remove_obj(k) for k in remove]
        req = ApplyPlanningScene.Request()
        req.scene = s
        res = call(apply_cli, req, 1.0)
        return res is not None and res.success

    return send, scene_ids, close


def main():
    ap = argparse.ArgumentParser(description="V-06 웹캠 팔 -> MoveIt2 장면 원기둥")
    ap.add_argument("--h", default="webcam_H.json", help="webcam_calib.py calib 가 만든 파일 (기본 webcam_H.json)")
    ap.add_argument("--cam", type=int, default=0, help="웹캠 번호 (기본 0)")
    ap.add_argument("--video", default="", help="웹캠 대신 영상 파일 (시험용)")
    ap.add_argument("--hz", type=float, default=5.0, help="장면 갱신 최대 횟수/초 (기본 5)")
    ap.add_argument("--radius", type=float, default=0.12, help="원기둥 반지름 (m, 기본 0.12)")
    ap.add_argument("--height", type=float, default=0.6, help="원기둥 높이 (m, 기본 0.6). 책상 윗면부터 세운다")
    ap.add_argument("--table-z", type=float, default=0.0, help="책상 윗면 높이 (m, 로봇 바닥면 기준, 기본 0)")
    ap.add_argument("--ns", default="", help="브링업 name 인자와 같은 값 (기본: 빈 값)")
    ap.add_argument("--no-show", action="store_true", help="화면을 띄우지 않는다 (Ctrl+C 로 끝낸다)")
    ap.add_argument("--dry-run", action="store_true", help="ROS 없이 좌표만 출력한다 (MoveIt2 없이 시험)")
    a = ap.parse_args()
    if a.hz <= 0 or not (0.0 < a.radius <= 0.5) or not (0.0 < a.height <= 2.0):
        raise SystemExit("--hz 는 0 보다 크게, --radius 와 --height 는 m 단위로 준다 (예: --radius 0.12 --height 0.6)")

    cal = load_calib(a.h)
    H = cal["H"]
    model = os.path.join(HERE, "pose_landmarker_lite.task")
    if not os.path.exists(model):
        raise SystemExit("pose_landmarker_lite.task 가 없다. arm_track.py 가 쓰는 파일을 이 폴더에 둔다: %s" % HERE)
    pose = V.PoseLandmarker.create_from_options(V.PoseLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=model),
        running_mode=V.RunningMode.VIDEO, num_poses=2))
    cap = cv2.VideoCapture(a.video if a.video else a.cam)
    if not a.video:   # calib 때와 같은 해상도로 연다
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cal["width"]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cal["height"]))
    if not cap.isOpened():
        raise SystemExit("카메라나 영상을 열 수 없다. --cam 1 처럼 번호를 바꿔 본다 (번호 확인: ls /dev/video*)")

    send = scene_ids = close = None
    if not a.dry_run:
        send, scene_ids, close = ros_open(a.ns, a.radius, a.height, a.table_z)
        stale = scene_ids()
        if stale:
            send({}, sorted(stale))
            print("지난번에 남은 원기둥 %d개를 지웠다: %s" % (len(stale), ", ".join(sorted(stale))))
    print("시작. %s q 또는 Ctrl+C 로 끝낸다." % ("ROS 없이 좌표만 출력한다 (--dry-run)." if a.dry_run else
                                          "MoveIt2 장면에 human_* 원기둥을 넣는다."))

    present = set()   # 지금 장면에 넣어 둔 이름
    t0 = time.monotonic()
    last_ts, last_send, last_ms = -1, 0.0, None
    win_t, win_frames, win_sends, fails = t0, 0, 0, 0
    size_checked = False
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("영상이 끝났거나 카메라가 끊겼다.")
                break
            now = time.monotonic()
            h, w = frame.shape[:2]
            if not size_checked:
                size_checked = True
                if (w, h) != (cal["width"], cal["height"]):
                    print("[주의] 화면 %dx%d 가 calib 때(%sx%s)와 다르다. 좌표가 틀린다. calib 를 이 해상도로 다시 한다."
                          % (w, h, cal["width"], cal["height"]))
            ts = max(int((now - t0) * 1000), last_ts + 1)   # VIDEO 모드는 시각(ms)이 계속 커져야 한다
            last_ts = ts
            img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            pr = pose.detect_for_video(img, ts)
            win_frames += 1
            arms = find_arms(pr.pose_landmarks, w, h, H)

            if now - last_send >= 1.0 / a.hz and (arms or present):
                last_send = now
                remove = sorted(present - set(arms))
                if a.dry_run:
                    print("  " + ("  ".join("%s (%.3f, %.3f)" % (k, x, y) for k, (x, y) in sorted(arms.items()))
                                  or "보이는 팔 없음") + ("  | 지움: " + ", ".join(remove) if remove else ""))
                    present = set(arms)
                else:
                    t1 = time.monotonic()
                    if send(arms, remove):
                        present = set(arms)
                    else:   # 실패하면 장면에 실제로 있는 이름을 다시 읽어 맞춘다
                        fails += 1
                        ids = scene_ids()
                        present = ids if ids is not None else present | set(arms)
                    last_ms = (time.monotonic() - t1) * 1000.0
                win_sends += 1

            if now - win_t >= 2.0:
                dt = now - win_t
                print("[%5.0fs] 카메라+MediaPipe %.1f Hz | 장면 갱신 %.1f Hz | 서비스 %s | 물체 %d개%s"
                      % (now - t0, win_frames / dt, win_sends / dt,
                         "-" if last_ms is None else "%.0f ms" % last_ms, len(present),
                         " | 실패 %d번 (move_group 터미널을 본다)" % fails if fails else ""))
                win_t, win_frames, win_sends, fails = now, 0, 0, 0

            if not a.no_show:
                draw_grid(frame, H)
                for p in pr.pose_landmarks[:2]:
                    for i in JOINTS:
                        if (p[i].visibility or 0) >= 0.5:
                            u, v = int(p[i].x * w), int(p[i].y * h)
                            x, y = px_to_mm(H, p[i].x * w, p[i].y * h)
                            cv2.circle(frame, (u, v), 6, (0, 200, 0), -1)
                            cv2.putText(frame, "%.0f,%.0f" % (x, y), (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                        (255, 255, 255), 2)
                cv2.putText(frame, "objects %d  %s" % (len(present), "dry-run" if a.dry_run else "MoveIt2"),
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("arm_scene (q: quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("Ctrl+C. 끝낸다.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if not a.dry_run:
            try:
                ids = scene_ids()
                left = sorted(ids if ids is not None else present)
                if left and send({}, left):
                    print("원기둥 %d개를 지웠다: %s" % (len(left), ", ".join(left)))
                elif left:
                    print("[주의] 원기둥을 다 지우지 못했다. arm_scene.py 를 다시 켰다 끄면 지워진다 (시작할 때 남은 것을 지운다).")
            except Exception as e:   # 끝낼 때는 최대한 지워 보고, 안 되면 알리기만 한다
                print("[주의] 원기둥 지우기 실패 (%s). arm_scene.py 를 다시 켰다 끄면 지워진다." % e)
            close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

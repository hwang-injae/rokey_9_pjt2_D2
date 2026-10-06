#!/usr/bin/env python3
"""손목 카메라 보정값(T_gripper2camera.npy)이 지금도 맞는지 — 로봇을 움직이지 않고 — 확인한다.

무엇을 하나
  관측 자세에서 손목 깊이 카메라로 빈 작업대를 찍고, 카메라가 계산한 '책상면 높이'를
  로봇 base 좌표로 바꿔 robot.yaml 의 table_z_m(그리퍼 끝으로 직접 짚은 값, -0.018 m)과 비교한다.
  차이가 거의 0 이면 보정이 맞고, 수십 mm 면 어긋난 것이다(문서 R-05 는 4.8 cm 높게 보인다고 적혀 있다).

입력
  --calib   T_gripper2camera.npy (카메라 → 그리퍼(TCP) 4x4, mm). 보정 때 활성이던 TCP 와 지금 TCP 가 같아야 한다.
  --posx    펜던트에 보이는 현재 posx 6개 (mm, deg ZYZ). 주지 않으면 두산 ROS 서비스로 읽는다.
  --table-z 작업면 높이 (m). 기본 robot.yaml 의 table_z_m.
  --rs      카메라를 ROS 대신 pyrealsense2 로 직접 연다 (카메라 노드가 안 떠 있을 때).
출력
  구역 5곳(가운데 + 네 귀퉁이)의 base 기준 높이(mm)와 table_z 와의 차이(mm). JSON 기록 한 줄도 남긴다.
바깥 영향
  없음 — 로봇 이동·그리퍼 명령·설정 변경을 하지 않는다. 자세와 깊이를 읽기만 한다.
실패 때
  카메라 프레임이나 posx 를 못 받으면 메시지를 내고 끝낸다(0 이 아닌 코드).

쓰는 법 (로봇 PC, 브링업과 카메라 노드가 떠 있는 터미널, 저장소 맨 위에서)
  python3 src/d2_vision/d2_vision/check_wrist_calib.py
  python3 src/d2_vision/d2_vision/check_wrist_calib.py --posx 400 0 350 0 180 0   # 펜던트 값으로
  python3 src/d2_vision/d2_vision/check_wrist_calib.py --rs                       # 카메라 직접 열기
기록은 실행한 폴더의 check_wrist_calib_log.jsonl 에 쌓인다(git 에 안 올림).
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# 보정 파일은 이 패키지 config/ 에 둔다(재보정하면 덮어쓴다). 소스 트리에서 바로 돌릴 수 있게 __file__ 기준.
CALIB_DEFAULT = Path(__file__).resolve().parents[1] / "config" / "T_gripper2camera.npy"
RECORD = Path.cwd() / "check_wrist_calib_log.jsonl"


def robot_yaml_path():
    """d2_bringup 패키지의 robot.yaml 경로. 빌드 전(소스 트리)이면 src/ 안의 파일로 대신한다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("d2_bringup")) / "config" / "robot.yaml"
    except Exception:
        return Path(__file__).resolve().parents[2] / "d2_bringup" / "config" / "robot.yaml"   # src/d2_vision/d2_vision → src/

# 깊이 평균을 낼 구역(픽셀 상자 한 변)과 모을 프레임 수.
# 10/3 V-17 경험: 정지 상태에서 10 프레임 중앙값이면 ±0.5 mm 로 안정적이다.
ROI_PX = 40
N_FRAMES = 10


def posx_to_matrix(x, y, z, rx, ry, rz):
    """두산 posx(mm, ZYZ deg) → T_base2gripper (그리퍼 점을 base 로 옮기는 4x4, mm).
    이전 프로젝트 robot_control.py 와 같은 규약(intrinsic ZYZ)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def read_table_z_from_yaml(path):
    """robot.yaml 에서 table_z_m 한 줄만 읽는다(yaml 모듈 없이). 못 찾으면 None."""
    try:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            if line.strip().startswith("table_z_m:"):
                return float(line.split(":", 1)[1].split("#")[0].strip())
    except OSError:
        pass
    return None


def roi_points(depth_mm, fx, fy, ppx, ppy):
    """깊이 영상에서 구역 5곳의 대표점을 카메라 좌표(mm)로 낸다.
    입력 depth_mm: (H, W) mm 단위. 출력: {이름: [x, y, z] 또는 None(깊이 없음)}."""
    h, w = depth_mm.shape
    centers = {
        "가운데": (w // 2, h // 2),
        "왼위": (w // 4, h // 4), "오른위": (3 * w // 4, h // 4),
        "왼아래": (w // 4, 3 * h // 4), "오른아래": (3 * w // 4, 3 * h // 4),
    }
    out = {}
    r = ROI_PX // 2
    for name, (u, v) in centers.items():
        patch = depth_mm[v - r:v + r, u - r:u + r]
        valid = patch[patch > 0]
        if valid.size < patch.size // 2:        # 절반 넘게 비면 믿지 않는다
            out[name] = None
            continue
        z = float(np.median(valid))
        out[name] = [(u - ppx) * z / fx, (v - ppy) * z / fy, z]
    return out


# ---------- 카메라 읽기: pyrealsense2 직접 ----------
def grab_rs():
    """pyrealsense2 로 깊이 N_FRAMES 장을 모아 중앙값 깊이(mm)와 내부 파라미터를 돌려준다."""
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    try:
        for _ in range(15):                     # 자동 노출 안정
            pipe.wait_for_frames()
        stack = []
        intr = None
        for _ in range(N_FRAMES):
            fr = align.process(pipe.wait_for_frames())
            d = fr.get_depth_frame()
            intr = d.profile.as_video_stream_profile().intrinsics
            stack.append(np.asanyarray(d.get_data()).astype(np.float32) * scale * 1000.0)
    finally:
        pipe.stop()
    depth = np.median(np.stack(stack), axis=0)
    return depth, (intr.fx, intr.fy, intr.ppx, intr.ppy)


# ---------- ROS 2: 카메라 토픽 · 두산 posx 서비스 ----------
def grab_ros(prefix, want_depth, want_posx, timeout_s=10.0):
    """ROS 에서 깊이 N_FRAMES 장 + camera_info(want_depth) 와 두산 posx(want_posx)를 읽는다.
    돌려주는 것: (depth_mm 또는 None, (fx, fy, ppx, ppy) 또는 None, posx 또는 None).
    시간 안에 못 받으면 RuntimeError. 로봇에는 아무 명령도 보내지 않는다."""
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("check_wrist_calib")
    frames, info = [], {}
    cli = None

    if want_depth:
        from sensor_msgs.msg import CameraInfo, Image
        from cv_bridge import CvBridge
        bridge = CvBridge()

        def on_depth(msg):
            if len(frames) < N_FRAMES:
                img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
                # realsense 노드: 16UC1 이면 mm, 32FC1 이면 m
                frames.append(img if msg.encoding == "16UC1" else img * 1000.0)

        def on_info(msg):
            info.update(fx=msg.k[0], fy=msg.k[4], ppx=msg.k[2], ppy=msg.k[5])

        node.create_subscription(Image, f"{prefix}/aligned_depth_to_color/image_raw", on_depth, 10)
        node.create_subscription(CameraInfo, f"{prefix}/color/camera_info", on_info, 10)

    if want_posx:
        from dsr_msgs2.srv import GetCurrentPosx
        # 팀 브링업(real_moveit.launch.py)은 두산 서비스를 컨트롤러 이름 아래에 둔다: /dsr_controller2/aux_control/...
        cli = node.create_client(GetCurrentPosx, "/dsr_controller2/aux_control/get_current_posx")

    posx, fut = None, None
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cli is not None and fut is None and cli.service_is_ready():
            req = GetCurrentPosx.Request()
            req.ref = 0                          # DR_BASE
            fut = cli.call_async(req)
        if fut is not None and fut.done() and posx is None:
            res = fut.result()
            posx = list(res.task_pos_info[0].data[:6]) if res and res.task_pos_info else None
        depth_ok = (not want_depth) or (len(frames) >= N_FRAMES and info)
        posx_ok = (not want_posx) or posx is not None
        if depth_ok and posx_ok:
            break
    node.destroy_node()
    rclpy.shutdown()

    if want_depth and (len(frames) < N_FRAMES or not info):
        raise RuntimeError(
            f"카메라 토픽이 안 들어온다: {prefix}/aligned_depth_to_color/image_raw "
            f"(받은 프레임 {len(frames)}장). `ros2 topic list | grep depth` 로 이름을 확인하고 "
            "카메라는 align_depth.enable:=true 로 띄운다.")
    if want_posx and posx is None:
        raise RuntimeError("두산 posx 서비스(/dsr_controller2/aux_control/get_current_posx)가 답하지 않는다. "
                           "브링업이 떠 있는지 보거나 --posx 로 펜던트 값을 넣는다.")
    depth = np.median(np.stack(frames), axis=0) if want_depth else None
    intr = (info["fx"], info["fy"], info["ppx"], info["ppy"]) if want_depth else None
    return depth, intr, posx


def main():
    ap = argparse.ArgumentParser(description="손목 카메라 보정 확인 (로봇 안 움직임)")
    ap.add_argument("--calib", default=str(CALIB_DEFAULT), help="T_gripper2camera.npy")
    ap.add_argument("--posx", type=float, nargs=6, metavar=("X", "Y", "Z", "A", "B", "C"),
                    help="펜던트 posx (mm, deg). 없으면 두산 서비스로 읽음")
    ap.add_argument("--table-z", type=float, default=None, help="작업면 높이 m (기본 robot.yaml table_z_m)")
    ap.add_argument("--rs", action="store_true", help="ROS 대신 pyrealsense2 로 카메라 직접 열기")
    ap.add_argument("--cam-prefix", default="/camera/camera", help="realsense 토픽 접두 (Jazzy 기본 /camera/camera)")
    a = ap.parse_args()

    yaml_path = robot_yaml_path()
    table_z_m = a.table_z if a.table_z is not None else read_table_z_from_yaml(yaml_path)
    if table_z_m is None:
        sys.exit(f"table_z_m 을 모른다 — --table-z 로 주거나 {yaml_path} 을 확인")
    T_g2c = np.load(a.calib)
    if T_g2c.shape != (4, 4):
        sys.exit(f"보정 파일 모양이 4x4 가 아니다: {T_g2c.shape}")

    # 1) 깊이 + posx. 카메라는 --rs 면 직접, 아니면 ROS 토픽. posx 는 --posx 가 없으면 두산 서비스.
    try:
        if a.rs:
            depth, intr = grab_rs()
            _, _, posx = (None, None, a.posx) if a.posx is not None else grab_ros(a.cam_prefix, False, True)
        else:
            depth, intr, posx_ros = grab_ros(a.cam_prefix, True, a.posx is None)
            posx = a.posx if a.posx is not None else posx_ros
    except RuntimeError as e:
        sys.exit(f"읽기 실패: {e}")

    # 2) 카메라 점 → base
    T_b2g = posx_to_matrix(*posx)
    T_b2c = T_b2g @ T_g2c
    cam_pts = roi_points(depth, *intr)
    rows, zs = [], []
    for name, p in cam_pts.items():
        if p is None:
            rows.append((name, None, None))
            continue
        pb = T_b2c @ np.array([p[0], p[1], p[2], 1.0])
        zs.append(pb[2])
        rows.append((name, p, pb[:3]))

    # 3) 보고
    table_mm = table_z_m * 1000.0
    print(f"\n보정 파일: {a.calib}")
    print(f"posx: {[round(v, 2) for v in posx]}  (mm, deg)")
    print(f"기준 작업면 높이 table_z: {table_mm:.1f} mm (base)\n")
    print(f"{'구역':<6} {'카메라 거리 mm':>14} {'base 높이 mm':>13} {'차이 mm':>9}")
    for name, p, pb in rows:
        if p is None:
            print(f"{name:<6} {'(깊이 없음)':>14}")
        else:
            print(f"{name:<6} {p[2]:>14.1f} {pb[2]:>13.1f} {pb[2] - table_mm:>+9.1f}")
    if zs:
        med = float(np.median(zs))
        spread = float(max(zs) - min(zs))
        print(f"\n중앙값 차이: {med - table_mm:+.1f} mm   (구역 사이 퍼짐 {spread:.1f} mm — 크면 카메라가 작업면에 비해 기울어진 것)")
        verdict = ("보정이 맞다(|차이| ≤ 5 mm)" if abs(med - table_mm) <= 5 else
                   "어긋남 — 재보정 필요" if abs(med - table_mm) > 15 else "애매함 — 한 번 더 재기")
        print(f"판정: {verdict}")
        with open(RECORD, "a", encoding="utf-8") as f:
            f.write(json.dumps({
                "time": time.strftime("%Y-%m-%d %H:%M:%S"), "calib": a.calib, "posx": posx,
                "table_z_mm": table_mm, "base_z_mm": {n: (None if pb is None else round(float(pb[2]), 2)) for n, _, pb in rows},
                "median_diff_mm": round(med - table_mm, 2), "spread_mm": round(spread, 2),
            }, ensure_ascii=False) + "\n")
        print(f"기록: {RECORD}")
    else:
        sys.exit("깊이가 있는 구역이 없다 — 카메라가 작업면을 보고 있는지, 거리가 28 cm 넘는지 확인")


if __name__ == "__main__":
    main()

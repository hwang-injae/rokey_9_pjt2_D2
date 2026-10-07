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
  --fix-tilt OUT.npy  (10/7 추가) 작업면 평면 맞춤으로 잰 기울기를 보정값의 **회전**에 반영하고, 높이 차이도 0 이 되게 맞춘
            새 보정 파일을 OUT 에 쓴다. 원본 --calib 는 건드리지 않는다(OUT 이 이미 있으면 --overwrite 가 있어야 덮어쓴다).
출력
  구역 5곳(가운데 + 네 귀퉁이)의 base 기준 높이(mm)와 table_z 와의 차이(mm). JSON 기록 한 줄도 남긴다.
  (10/7 추가) 작업면 전체 점으로 맞춘 평면의 기울기 — x · y 로 100 mm 갈 때 몇 mm 오르내리는지, 각도(°), 맞춤 잔차.
  10/6 W058 기록은 구역 중앙값이 −0.5 mm 였지만 구역마다 +0.5 ~ −3.1 mm 로 화면 아래쪽이 낮게 보였다(기울기 약 1.5°) —
  10/7 박진용 실기에서 블록 윗면이 −2.8 mm 로 나온 원인. 평면 맞춤이 이것을 숫자로 보여 주고 --fix-tilt 가 고친다.
바깥 영향
  없음 — 로봇 이동·그리퍼 명령·설정 변경을 하지 않는다. 자세와 깊이를 읽기만 한다.
실패 때
  카메라 프레임이나 posx 를 못 받으면 메시지를 내고 끝낸다(0 이 아닌 코드).

쓰는 법 (로봇 PC, 브링업과 카메라 노드가 떠 있는 터미널, 저장소 맨 위에서)
  python3 src/d2_vision/d2_vision/check_wrist_calib.py
  python3 src/d2_vision/d2_vision/check_wrist_calib.py --posx 400 0 350 0 180 0   # 펜던트 값으로
  python3 src/d2_vision/d2_vision/check_wrist_calib.py --rs                       # 카메라 직접 열기
  python3 src/d2_vision/d2_vision/check_wrist_calib.py --fix-tilt /tmp/T_fix.npy   # 빈 작업대를 보는 관측 자세에서: 기울기 보정본 만들기
    → 같은 자세에서 --calib /tmp/T_fix.npy 로 다시 확인해 기울기 ≈ 0 · 차이 ≈ 0 이면 config/T_gripper2camera.npy 에 복사(json 에 기록)
기록은 실행한 폴더의 check_wrist_calib_log.jsonl 에 쌓인다(git 에 안 올림).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# 보정 파일은 이 패키지 config/ 에 둔다(재보정하면 덮어쓴다). 소스 트리에서 바로 돌릴 수 있게 __file__ 기준.
CALIB_DEFAULT = Path(__file__).resolve().parents[1] / "config" / "T_gripper2camera.npy"
RECORD = Path.cwd() / "check_wrist_calib_log.jsonl"


def robot_yaml_path():
    """robot.yaml 경로를 반환한다. 입력 없음·출력 Path이며 빌드 전에는 src/d2_robot/ 경로를 사용한다.

    로봇·메시지에 영향은 없고 설치 경로 조회 실패 때 소스 설정 경로를 반환한다.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("d2_bringup")) / "config" / "robot.yaml"
    except Exception:
        return Path(__file__).resolve().parents[2] / "d2_robot" / "d2_bringup" / "config" / "robot.yaml"

# 깊이 평균을 낼 구역(픽셀 상자 한 변)과 모을 프레임 수.
# 10/3 V-17 경험: 정지 상태에서 10 프레임 중앙값이면 ±0.5 mm 로 안정적이다.
ROI_PX = 40
N_FRAMES = 10
# 평면 맞춤(10/7): 작업면 근처 점만 쓴다 — table_z ± PLANE_BAND_MM 밖(블록 · 케이블 · 그리퍼)은 뺀다.
# 1차 맞춤 뒤 PLANE_RESID_MM 보다 먼 점(가장자리 번짐 · 작은 물체)을 빼고 한 번 더 맞춘다. 깊이 잡음 ±0.5 mm(V-17)에 비해 넉넉한 값.
PLANE_BAND_MM = 15.0
PLANE_RESID_MM = 3.0
PLANE_STRIDE = 4            # 640×480 을 4픽셀마다 → 약 19,000 점. 평면 셋 계수에 충분
PLANE_MIN_PTS = 200


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


def depth_to_base(depth_mm, fx, fy, ppx, ppy, T_b2c, stride=PLANE_STRIDE):
    """깊이 영상 전체(stride 픽셀마다) → base 점 (N, 3) mm. 깊이 0 · NaN 은 버린다. wrist_block 의 depth_to_base_points 와 같은 계산."""
    d = depth_mm[::stride, ::stride]
    v, u = np.mgrid[0:depth_mm.shape[0]:stride, 0:depth_mm.shape[1]:stride]
    ok = np.isfinite(d) & (d > 0)
    z = d[ok]
    pc = np.stack([(u[ok] - ppx) * z / fx, (v[ok] - ppy) * z / fy, z, np.ones_like(z)], axis=1)
    return (pc @ T_b2c.T)[:, :3]


def fit_table_plane(pts_base, table_mm):
    """작업면 근처 점으로 평면 z = a·x + b·y + c (mm) 를 맞춘다. 두 번 맞춘다(먼 점을 빼고 다시).
    돌려줌: dict(a, b, c, n, rms_mm, tilt_deg) 또는 점이 모자라면 None.
    a · b 는 x · y 로 1 mm 갈 때 높이 변화(기울기). 카메라가 작업면에 비해 기울어져 있으면 0 이 아니다."""
    sel = pts_base[np.abs(pts_base[:, 2] - table_mm) <= PLANE_BAND_MM]
    if len(sel) < PLANE_MIN_PTS:
        return None
    for _ in range(2):
        A = np.column_stack([sel[:, 0], sel[:, 1], np.ones(len(sel))])
        coef, *_ = np.linalg.lstsq(A, sel[:, 2], rcond=None)
        resid = sel[:, 2] - A @ coef
        keep = np.abs(resid) <= PLANE_RESID_MM
        if keep.sum() < PLANE_MIN_PTS:
            break
        sel, resid = sel[keep], resid[keep]
    a, b, c = (float(v) for v in coef)
    return {"a": a, "b": b, "c": c, "n": len(sel), "rms_mm": float(np.sqrt(np.mean(resid ** 2))),
            "tilt_deg": float(np.degrees(np.arctan(np.hypot(a, b))))}


def fix_tilt(T_g2c, T_b2g, depth_mm, intr, table_mm):
    """보정값의 회전을 돌려 작업면 평면이 수평이 되게 하고, 카메라 광축 방향으로 밀어 높이 차이를 0 으로 맞춘 새 T_gripper2camera 를 만든다.
    입력: 지금 보정값(4x4 mm) · T_base2gripper(posx) · 빈 작업대 깊이(mm) · 내부 파라미터 · 작업면 높이(mm).
    돌려줌: (새 4x4, 보정 전 평면, 보정 뒤 평면, 돌린 각도 °, 민 거리 mm). 평면을 못 맞추면 None.
    왜 회전을 카메라 좌표에서 돌리나: 오차의 원인이 카메라가 그리퍼에 붙은 각도이므로, 어느 posx 에서 재든 같은 보정이 되게 하려고."""
    T_b2c = T_b2g @ T_g2c
    before = fit_table_plane(depth_to_base(depth_mm, *intr, T_b2c), table_mm)
    if before is None:
        return None
    R_b2c = T_b2c[:3, :3]
    n_meas = np.array([-before["a"], -before["b"], 1.0])            # 맞춘 평면의 법선(base)
    n_meas /= np.linalg.norm(n_meas)
    n_cam_meas = R_b2c.T @ n_meas                                   # 카메라 좌표에서 본 법선
    n_cam_want = R_b2c.T @ np.array([0.0, 0.0, 1.0])                # 수평 작업면이면 보여야 할 법선
    axis = np.cross(n_cam_meas, n_cam_want)
    ang = float(np.arctan2(np.linalg.norm(axis), np.dot(n_cam_meas, n_cam_want)))
    R_c = np.eye(3) if ang < 1e-9 else Rotation.from_rotvec(axis / np.linalg.norm(axis) * ang).as_matrix()
    T_new = T_g2c.copy()
    T_new[:3, :3] = T_g2c[:3, :3] @ R_c
    # 회전 뒤 남는 높이 차이는 카메라 광축(카메라 z)으로 민다 — 카메라가 작업면을 내려다보므로 base z 에 거의 그대로 반영된다
    T_b2c_new = T_b2g @ T_new
    mid = fit_table_plane(depth_to_base(depth_mm, *intr, T_b2c_new), table_mm)
    if mid is None:
        return None
    cam_xy = T_b2c_new[:2, 3]
    off = mid["a"] * cam_xy[0] + mid["b"] * cam_xy[1] + mid["c"] - table_mm   # 카메라 바로 아래의 평면 높이 − 기준
    ez_base_z = T_b2c_new[2, 2]                                     # 카메라 z 축이 base z 로 얼마나 향하나(내려다보면 ≈ −1)
    shift = -off / ez_base_z if abs(ez_base_z) > 0.3 else 0.0
    T_new[:3, 3] += T_new[:3, :3] @ np.array([0.0, 0.0, shift])
    after = fit_table_plane(depth_to_base(depth_mm, *intr, T_b2g @ T_new), table_mm)
    return T_new, before, after, float(np.degrees(ang)), float(shift)


def plane_lines(fit, table_mm, label):
    """평면 맞춤 결과를 사람이 읽을 줄들로."""
    if fit is None:
        return [f"{label}: 작업면 근처 점이 모자라 평면을 못 맞춤"]
    line = (f"{label}: x 로 100 mm 가면 {100 * fit['a']:+.1f} mm · y 로 100 mm 가면 {100 * fit['b']:+.1f} mm "
            f"(기울기 {fit['tilt_deg']:.2f}°, 점 {fit['n']}, 잔차 {fit['rms_mm']:.2f} mm)")
    return [line]


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
    ap.add_argument("--fix-tilt", metavar="OUT.npy", default=None,
                    help="작업면 평면 기울기 · 높이 차이를 보정한 새 보정 파일을 여기에 쓴다(원본은 안 건드림)")
    ap.add_argument("--overwrite", action="store_true", help="--fix-tilt 의 OUT 이 이미 있어도 덮어쓴다")
    a = ap.parse_args()
    if a.fix_tilt and Path(a.fix_tilt).exists() and not a.overwrite:
        sys.exit(f"{a.fix_tilt} 이 이미 있다 — 다른 이름을 쓰거나 --overwrite")

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
    # 3-1) 작업면 전체 점으로 평면 맞춤 — 구역 5곳의 중앙값은 기울기를 숨긴다(10/6: 중앙값 −0.5 인데 구역은 +0.5 ~ −3.1)
    plane = fit_table_plane(depth_to_base(depth, *intr, T_b2c), table_mm)
    for line in plane_lines(plane, table_mm, "작업면 평면"):
        print(line)
    if plane is not None and plane["tilt_deg"] > 0.3:
        print("  → 0.3° 넘으면 자리에 따라 높이가 달라진다(100 mm 에 0.5 mm 이상). --fix-tilt 로 고친다")
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
                "plane": None if plane is None else {k: round(v, 4) if isinstance(v, float) else v for k, v in plane.items()},
            }, ensure_ascii=False) + "\n")
        print(f"기록: {RECORD}")

    # 4) 기울기 보정본 쓰기 (--fix-tilt). 원본 --calib 는 그대로 둔다
    if a.fix_tilt:
        got = fix_tilt(T_g2c, T_b2g, depth, intr, table_mm)
        if got is None:
            sys.exit("평면을 못 맞춰 보정본을 만들지 못했다 — 빈 작업대가 화면 대부분을 차지하는 관측 자세에서 다시")
        T_new, before, after, ang, shift = got
        np.save(a.fix_tilt, T_new)
        print(f"\n기울기 보정본: {a.fix_tilt}")
        print(f"  회전 {ang:.2f}° 돌리고 카메라 광축으로 {shift:+.1f} mm 밈 (카메라 위치 mm {T_g2c[:3, 3].round(1).tolist()} → {T_new[:3, 3].round(1).tolist()})")
        for line in plane_lines(before, table_mm, "  보정 전"):
            print(line)
        for line in plane_lines(after, table_mm, "  보정 뒤"):
            print(line)
        if after is not None:
            cam_xy = (T_b2g @ T_new)[:2, 3]
            print(f"  보정 뒤 카메라 아래 작업면 높이 차이: {after['a'] * cam_xy[0] + after['b'] * cam_xy[1] + after['c'] - table_mm:+.2f} mm")
        print("  같은 자세에서 `--calib " + a.fix_tilt + "` 로 다시 확인한 뒤 config/T_gripper2camera.npy 에 복사하고 json 에 적는다")
    else:
        sys.exit("깊이가 있는 구역이 없다 — 카메라가 작업면을 보고 있는지, 거리가 28 cm 넘는지 확인")


if __name__ == "__main__":
    main()

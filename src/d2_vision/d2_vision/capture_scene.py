"""촬영 저장 도구 (capture_scene) — W114 흩뿌린 블록 10장면 · 스캔 촬영용. 로봇을 움직이지 않는다.

Enter 를 누를 때마다 손목 카메라의 **컬러 PNG + 컬러에 맞춘 깊이 16비트 PNG(mm) + 자세 json** 을 같은 번호로 저장한다
(docs/YOLO데이터_수집라벨링규칙 §2.1 · §2.3: `s<장면 2자리>_<번호 3자리>_<조명 a/b>_<color|depth|pose>`).
스캔 촬영(기본 설계 × 자세 3곳)은 자세 태그를 켜면 이름에 자세가 붙는다: `s11_001_a_front_color.png`(10/7 민범진 · 한석형 — 번호 약속 대신
이름으로 구분, PL 10/7 허락 — 규칙 §2.3). 태그를 안 켜면 이름은 그대로다. 장면 번호: 01~10 흩뿌린 장면 · 11~13 스캔(벤치 · 의자 Lv2 · 책상 Lv4) · 21~30 V-52 시험.
스캔 원본은 드라이브 `1_원본_W114/스캔`에 따로 올린다(Roboflow에 섞이지 않게).
자세 json = **MoveIt TF base_link → rg2_tcp(mm, 10/10 W156 — 손목 자세의 기준, E-70 ③)** · 두산 posx(mm · deg, 기록용) · 팔 관절값(/joint_states, deg)
· 카메라 내부 파라미터 · TF × 보정값(rg2_tcp 틀)으로 계산한 T_base2cam(m) · **그때 쓴 보정값(파일 · 수정 시각 · 카메라 위치 mm · 지문)**
· 제어기 TCP(config/tcp.json 이름 · 값 — 10/10 E-81 뒤 같은 이름 GripperDA_v1 이 값만 바뀜) · 해상도 · 시각.
브링업이 없으면 TF · 관절값 · T_base2cam 은 빈 값으로 저장된다(경고). `--posx` 로 펜던트 값을 적을 수 있다(기록용).
(W156 확인용, 다시 계산 뒤 지움) 옛 posx 틀 보정값(T_gripper2camera)이 있으면 posx 길로 낸 카메라 위치와 TF 길의 차이(mm)도 적는다.
보정값은 촬영 중간에 바뀔 수 있다(10/7 수평 약 1 cm 재보정). 그래서 뒤에 쓰는 코드(스캔 추론기 · find_blocks)는 json 의 T_base2cam 을
그대로 믿지 말고 **T_base_tcp_mm × 최신 보정값**으로 다시 계산한다(PL 10/7 → 10/10 TF 기준). json 의 calib 칸은 어느 값으로 계산했는지 되짚는 용도.

입력(인자)
  --out      저장 폴더 (예: ~/d2_data/W114). 없으면 만든다. 저장소에는 넣지 않는다(드라이브 YOLO_흩어진블록/1_원본_W114 로 올림)
  --scene    시작 장면 번호(1~), --light a|b 조명 표시, --cam-prefix realsense 토픽 접두, --posx X Y Z A B C 펜던트 값(브링업 없을 때)
  --pose     시작 자세 태그(observe · front · side, 스캔 촬영용). 비우면 태그 없음(흩뿌린 장면)
키: Enter = 저장 · s = 다음 장면(번호 1부터) · l = 조명 a/b 바꿈 · p = 자세 태그 돌리기(없음 → observe → front → side) · q = 끝
    observe · front · side 를 그대로 치면 그 태그로 바로 바뀐다(저장은 안 함)
바깥 영향: 파일 저장뿐. 실패 때: 프레임이 1초 안에 없으면 그 장은 저장하지 않고 이유를 찍는다. TF · posx 를 못 받으면 저장은 하고 경고를 찍는다.

카메라는 규칙대로 **640×480** 으로 띄운다(기본 1280×720 아님):
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30
실행:
  ros2 run d2_vision capture_scene --out ~/d2_data/W114 --scene 1 --light a
  ros2 run d2_vision capture_scene --out ~/d2_data/W114_scan --scene 11 --pose observe   # 스캔: 11 벤치 · 12 의자 · 13 책상, 자세마다 p → Enter
"""
import argparse
import hashlib
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState

import yaml

from d2_vision.tcp_pose import TcpPose, default_calib_path, load_calib
from dsr_msgs2.srv import GetCurrentPosx

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']   # 두산 팔 관절 (joint_states 에 그리퍼 관절도 섞여 온다)
FRESH_S = 1.0
POSES = ['', 'observe', 'front', 'side']    # 자세 태그 순서(p 키). '' = 태그 없음. robot.yaml 의 observe · observe_front · observe_side 에 대응


def calib_info(path, T):
    """pose json 에 적을 보정값 기록: 파일 이름 · 수정 시각(ISO) · 카메라 위치 mm(T 의 이동) · 4x4 지문(sha1 앞 12자리).
    T 가 None(보정 파일 없음)이면 None. 같은 지문이면 같은 보정값으로 계산한 사진이다."""
    if T is None:
        return None
    path = Path(path)
    return {
        'file': path.name,
        'mtime': time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(path.stat().st_mtime)) if path.exists() else None,
        'cam_pos_mm': [round(float(v), 2) for v in T[:3, 3]],
        'sha1_12': hashlib.sha1(np.ascontiguousarray(T, dtype=np.float64).round(6).tobytes()).hexdigest()[:12],
    }


def stem_for(scene, index, light, pose=''):
    """파일 이름 줄기. 태그가 없으면 규칙 그대로 `s01_003_a`, 있으면 `s11_001_a_front`(조명 뒤 · 종류 앞)."""
    stem = f's{scene:02d}_{index:03d}_{light}'
    return f'{stem}_{pose}' if pose else stem


def posx_to_matrix(x, y, z, rx, ry, rz):
    """두산 posx(mm, ZYZ deg) → T_base2gripper 4x4 (mm). W156 확인용(posx 길과 TF 길 비교)으로만 쓴다."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


class CaptureScene(Node):
    """카메라 · 관절값을 받아 두고, 저장 요청이 오면 최신 프레임과 자세를 파일로 쓴다."""

    def __init__(self, args):
        """구독(컬러 · 깊이 · camera_info · joint_states) · TF 리스너 · posx 클라이언트를 만들고 보정값 · tcp.json 을 읽는다."""
        super().__init__('capture_scene')
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.color = self.depth = None
        self.t_color = self.t_depth = 0.0
        self.intr = None
        self.joints = None
        self.calib_path = default_calib_path()
        try:
            self.T_tcp2c = load_calib(self.calib_path)
        except (OSError, ValueError):
            self.T_tcp2c = None
            self.get_logger().warn('보정값 %s 없음 — pose json 에 T_base2cam 을 못 넣는다' % self.calib_path)
        old = Path(self.calib_path).with_name('T_gripper2camera.npy')        # W156 확인용(posx 틀) — 다시 계산 뒤 지움
        self.T_posx2c = load_calib(old) if old.exists() else None
        bring = Path(get_package_share_directory('d2_bringup')) / 'config'
        cfg = yaml.safe_load((bring / 'robot.yaml').read_text(encoding='utf-8'))
        self.tcp = TcpPose(self, cfg.get('frame_id', 'base_link'), cfg.get('tcp_link', 'rg2_tcp'))
        try:
            t = json.loads((bring / 'tcp.json').read_text(encoding='utf-8'))
            self.tcp_json = {'name': t.get('name'), 'pos': t.get('pos'), 'verified_at': t.get('verified_at')}
        except (OSError, ValueError):
            self.tcp_json = None
        pre = args.cam_prefix
        self.create_subscription(Image, f'{pre}/color/image_raw', self.on_color, 5)
        self.create_subscription(Image, f'{pre}/aligned_depth_to_color/image_raw', self.on_depth, 5)
        self.create_subscription(CameraInfo, f'{pre}/color/camera_info', self.on_info, 5)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 20)
        self.cli_posx = self.create_client(GetCurrentPosx, '/dsr_controller2/aux_control/get_current_posx')

    def now(self):
        """노드 시계의 지금 시각(초)."""
        return self.get_clock().now().nanoseconds / 1e9

    def on_color(self, msg):
        """컬러 프레임(bgr8) 보관."""
        with self.lock:
            self.color, self.t_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8'), self.now()

    def on_depth(self, msg):
        """정렬된 깊이 프레임을 mm 16비트로 보관(32FC1 이면 m → mm)."""
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if msg.encoding != '16UC1':
            img = np.nan_to_num(img * 1000.0).astype(np.uint16)
        with self.lock:
            self.depth, self.t_depth = img, self.now()

    def on_info(self, msg):
        """camera_info → fx, fy, ppx, ppy (픽셀)."""
        self.intr = dict(fx=msg.k[0], fy=msg.k[4], ppx=msg.k[2], ppy=msg.k[5], width=msg.width, height=msg.height)

    def on_joints(self, msg):
        """joint_states 에서 팔 관절 6개만 deg 로 보관(그리퍼 관절은 뺀다)."""
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.joints = [float(np.degrees(pos[j])) for j in JOINTS]

    def read_posx(self):
        """posx 6개(mm · deg). --posx 가 있으면 그것, 아니면 두산 서비스(3초). 못 받으면 None."""
        if self.args.posx:
            return list(self.args.posx)
        if not self.cli_posx.wait_for_service(timeout_sec=3.0):
            return None
        req = GetCurrentPosx.Request()
        req.ref = 0
        fut = self.cli_posx.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.02)                       # 실행기는 다른 스레드에서 돈다
        r = fut.result() if fut.done() else None
        if r is None or not r.success or not r.task_pos_info:
            return None
        return list(r.task_pos_info[0].data[:6])

    def save(self, out, scene, index, light, pose=''):
        """최신 컬러 · 깊이 · 자세를 `s<scene>_<index>_<light>[_<pose>]_*` 로 저장한다. 성공하면 True, 아니면 이유를 찍고 False.
        누른 **뒤** 들어온 프레임을 최대 FRESH_S 기다려 쓴다(카메라를 막 켰거나 손을 뺀 직후의 옛 프레임을 피한다)."""
        t_req = self.now()
        while True:
            with self.lock:
                color, depth = self.color, self.depth
                ok = self.t_color > t_req and self.t_depth > t_req
            if ok or self.now() - t_req > FRESH_S:
                break
            time.sleep(0.02)
        if not ok:
            print('  ✗ 카메라 프레임이 %.1f초 안에 안 들어옴 — 카메라 노드를 확인' % FRESH_S)
            return False
        T_b2t = self.tcp.matrix_mm(3.0)
        if T_b2t is None:
            print('  ⚠ TF base_link → rg2_tcp 를 못 받음 — 브링업(real_moveit)이 떠 있는지 확인. 사진은 저장하지만 손목 보정 측정에는 못 쓴다')
        posx = self.read_posx()
        if posx is None:
            print('  ⚠ 두산 posx 를 못 받음(기록용 — 빈 값으로 저장)')
        stem = stem_for(scene, index, light, pose)
        cv2.imwrite(str(out / f'{stem}_color.png'), color)
        cv2.imwrite(str(out / f'{stem}_depth.png'), depth.astype(np.uint16))
        pose = {
            'scene': scene, 'index': index, 'light': light, 'pose_id': pose or None, 'stamp': time.time(),
            'T_base_tcp_mm': None if T_b2t is None else T_b2t.round(4).tolist(),
            'posx_mm_deg': posx, 'posj_deg': self.joints, 'intrinsics_px': self.intr,
            'depth_unit': 'mm', 'color_size': [int(color.shape[1]), int(color.shape[0])],
            'T_base2cam_m': None, 'tcp': self.tcp_json,
            'calib': calib_info(self.calib_path, self.T_tcp2c),    # 이 사진의 T_base2cam 을 어느 보정값으로 계산했나(PL 10/7)
        }
        if self.T_tcp2c is not None and T_b2t is not None:
            T = T_b2t @ self.T_tcp2c
            if self.T_posx2c is not None and posx is not None:   # W156 확인용: posx 길과 TF 길의 카메라 위치 차이(mm)
                pose['check_cam_posx_minus_tf_mm'] = ((posx_to_matrix(*posx) @ self.T_posx2c)[:3, 3] - T[:3, 3]).round(2).tolist()
            T[:3, 3] /= 1000.0
            pose['T_base2cam_m'] = T.round(6).tolist()
        (out / f'{stem}_pose.json').write_text(json.dumps(pose, ensure_ascii=False, indent=1), encoding='utf-8')
        print(f'  ✓ {stem}  ({color.shape[1]}x{color.shape[0]}, TF {"있음" if T_b2t is not None else "없음"}, '
              f'관절 {"있음" if self.joints else "없음"}, posx {"있음" if posx else "없음"})')
        return True


def main(args=None):
    """인자를 읽고 노드를 띄운 뒤 키 입력(Enter · s · l · q)을 받아 저장한다. 로봇에 명령은 없다."""
    ap = argparse.ArgumentParser(description='촬영 저장 (컬러 · 깊이 · 자세)')
    ap.add_argument('--out', required=True, help='저장 폴더')
    ap.add_argument('--scene', type=int, default=1, help='시작 장면 번호')
    ap.add_argument('--light', default='a', choices=['a', 'b'], help='조명 표시')
    ap.add_argument('--cam-prefix', default='/camera/camera')
    ap.add_argument('--posx', type=float, nargs=6, metavar=('X', 'Y', 'Z', 'A', 'B', 'C'), help='펜던트 posx(기록용 — 손목 자세는 TF)')
    ap.add_argument('--pose', default='', choices=POSES, help='시작 자세 태그(스캔 촬영). 비우면 태그 없음')
    a, ros_args = ap.parse_known_args(args)
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = CaptureScene(a)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    scene, light, pose = a.scene, a.light, a.pose
    index = 1 + len(list(out.glob(f's{scene:02d}_*_color.png')))     # 같은 장면에 이미 있으면 이어서 번호(태그가 있어도 같은 glob 에 걸린다)
    print(f'저장 폴더 {out}  장면 {scene:02d} 번호 {index:03d} 조명 {light} 자세 태그 {pose or "없음"}')
    print('Enter = 저장 · s = 다음 장면 · l = 조명 바꿈 · p = 자세 태그 돌리기(observe/front/side 를 쳐도 됨) · q = 끝')
    try:
        while True:
            key = sys.stdin.readline()
            if not key:
                break
            key = key.strip().lower()
            if key == 'q':
                break
            if key == 's':
                scene, index = scene + 1, 1
                print(f'→ 장면 {scene:02d} (블록을 새로 흩뿌린 뒤 Enter)')
            elif key == 'l':
                light = 'b' if light == 'a' else 'a'
                print(f'→ 조명 {light}')
            elif key == 'p':
                pose = POSES[(POSES.index(pose) + 1) % len(POSES)]
                print(f'→ 자세 태그 {pose or "없음"}')
            elif key in POSES[1:]:
                pose = key
                print(f'→ 자세 태그 {pose} (로봇을 그 자세로 보낸 뒤 Enter)')
            else:
                if node.save(out, scene, index, light, pose):
                    index += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f'끝. 파일 {len(list(out.glob("*_color.png")))}장 — 드라이브 YOLO_흩어진블록/1_원본_W114 에 올린다')


if __name__ == '__main__':
    main()

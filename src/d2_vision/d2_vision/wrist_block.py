"""손목 블록 인식 노드 (wrist_block) — IRD 4.2 `/d2/vision/check_progress` · 4.1 `camera_status/1` (W041).

작업 관리자가 관측 자세에서 `check_progress`(block_ids)를 부르면:
  손목 깊이 영상 10장(중앙값) + 두산 posx + hand-eye 보정값 → base 점군 → BlockChecker(block_checker.py)
  → 블록마다 state(present·absent·occluded·unknown) · top_z_m · dz_m (dx·dy 는 1차 NaN) 로 답한다.
카메라 연결 신호 `/d2/vision/camera_status` 를 2 Hz 로 낸다(마지막 프레임 시각).

입력(파라미터)
  recipe_dir   레시피 폴더(task 노드와 같은 파라미터, 예 src/recipe_manager/recipes). 요청 block_id 의 앞글자(`001_CHAIR_BENCH_B003` → `001_CHAIR_BENCH`)로
               `<recipe_dir>/<design_id>.recipe.json` 을 찾아 팀 공용 `d2_motion.motion_math.recipe_blocks()` 로 base 블록 목록을 만든다(설계마다 한 번, 캐시).
               설계를 `get_design` 으로 받는 길은 W121 뒤.
  calib_path   T_gripper2camera.npy(카메라 → TCP 4x4, mm). 비우면 이 패키지 share/config 의 것
  cam_prefix   realsense 토픽 접두 (Jazzy 기본 /camera/camera)
  posx         시험용 posx 6개(mm·deg, **실수로** 예: [460.5, -157.0, 294.8, 154.8, 180.0, 154.5]). 비우면 두산 서비스로 읽는다
  n_frames     중앙값 낼 깊이 프레임 수(10 — V-17 경험: 정지 상태 10장이면 ±0.5 mm)
robot.yaml(d2_bringup)에서 assembly_origin · assembly_area_half_m 을 읽는다.

바깥 영향: 서비스 답과 camera_status 발행뿐. 로봇·카메라·그리퍼에 명령을 보내지 않는다(관측 자세 이동은 작업 관리자가 move_to 로).
답할 때: 요청이 온 **뒤에** 들어온 깊이 프레임 n_frames 장(최대 1.5초 기다림)의 중앙값을 쓴다 — 로봇이 막 멈춘 직후의 움직이던 프레임을 섞지 않으려고.
실패 때: 새 프레임이 시간 안에 안 오거나 posx 를 못 받으면 success=false, reason=TIMEOUT.
         보정값·레시피를 못 읽으면 노드는 뜨되 success=false, reason=ERROR (배열은 요청 길이, state unknown).
NaN 규칙(IRD 5장): dx·dy 는 1차 늘 NaN. absent·unknown 은 dz·top_z 도 NaN. 가려진 present(위 블록 때문에 못 잼)도 NaN.
보정값은 TCP 기준이라(config/T_gripper2camera.json) 켤 때 제어기 활성 TCP 가 d2_bringup config/tcp.json 과 다르면 경고한다(한 번).

실행 (저장소 맨 위에서):
  ros2 run d2_vision wrist_block --ros-args -p recipe_dir:=src/recipe_manager/recipes
시험 호출:
  ros2 service call /d2/vision/check_progress d2_interfaces/srv/CheckProgress "{block_ids: [001_CHAIR_BENCH_B001, 001_CHAIR_BENCH_B002]}"
"""
import json
import threading
import warnings
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from d2_interfaces.srv import CheckProgress
from d2_motion.motion_math import recipe_blocks
from d2_vision.block_checker import BlockChecker, depth_to_base_points
from dsr_msgs2.srv import GetCurrentPosx, GetCurrentTcp

NAN = float('nan')
QOS_STATUS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
FRESH_WAIT_S = 1.5         # 요청 뒤 새 프레임 n장을 이만큼 기다린다(30 Hz 면 10장에 0.33초 — 여유 포함). 넘으면 TIMEOUT
SERVICE_WAIT_S = 3.0       # 두산 서비스 기다림(없음 3초 + 답 3초 = 최대 6초 막힘)


def posx_to_matrix(x, y, z, rx, ry, rz):
    """두산 posx(mm, ZYZ deg) → T_base2gripper 4x4 (mm). check_wrist_calib.py 와 같은 규약(intrinsic ZYZ)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


class WristBlock(Node):
    """check_progress 에 답하고 camera_status 를 내는 손목 블록 인식 노드."""

    def __init__(self):
        """파라미터 · 설정 · 보정값 · 레시피를 읽고 구독 · 서비스 · 타이머를 만든다. 레시피 · 보정이 없어도 노드는 뜬다(경고)."""
        super().__init__('wrist_block')
        self.declare_parameter('recipe_dir', '')
        self.declare_parameter('calib_path', '')
        self.declare_parameter('cam_prefix', '/camera/camera')
        self.declare_parameter('posx', [0.0] * 6)
        self.declare_parameter('n_frames', 10)
        self.declare_parameter('status_hz', 2.0)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = []                      # (받은 시각 s, 깊이 mm float32) 최근 n_frames 개
        self.intr = None                      # (fx, fy, ppx, ppy) 픽셀 단위
        self.last_frame_t = 0.0
        self.n_frames = int(self.get_parameter('n_frames').value)

        self.cfg = self._load_robot_yaml()
        self.T_g2c = self._load_calib()
        self.checkers = {}                    # design_id → BlockChecker (레시피는 설계마다 한 번만 읽는다)
        if not self.get_parameter('recipe_dir').value:
            self.get_logger().warn('recipe_dir 파라미터가 비었다 — check_progress 는 ERROR 로 답한다')

        group = ReentrantCallbackGroup()
        prefix = self.get_parameter('cam_prefix').value
        self.create_subscription(Image, f'{prefix}/aligned_depth_to_color/image_raw', self.on_depth, 10, callback_group=group)
        self.create_subscription(CameraInfo, f'{prefix}/color/camera_info', self.on_info, 10, callback_group=group)
        self.cli_posx = self.create_client(GetCurrentPosx, '/dsr_controller2/aux_control/get_current_posx', callback_group=group)
        self.cli_tcp = self.create_client(GetCurrentTcp, '/dsr_controller2/tcp/get_current_tcp', callback_group=group)
        self.create_service(CheckProgress, '/d2/vision/check_progress', self.on_check, callback_group=group)
        self.pub_status = self.create_publisher(String, '/d2/vision/camera_status', QOS_STATUS)
        self.create_timer(1.0 / self.get_parameter('status_hz').value, self.publish_status, callback_group=group)
        self._tcp_timer = self.create_timer(2.0, self.check_tcp_once, callback_group=group)   # 브링업이 늦게 뜰 수 있어 2초 뒤 한 번(안에서 끈다)
        self.get_logger().info('wrist_block 시작: /d2/vision/check_progress 대기 (카메라 %s)' % prefix)

    # ---------- 설정 읽기 ----------
    def _load_robot_yaml(self):
        """d2_bringup config/robot.yaml → dict. 없으면 예외(설정 없이 돌지 않는다)."""
        p = Path(get_package_share_directory('d2_bringup')) / 'config' / 'robot.yaml'
        return yaml.safe_load(p.read_text(encoding='utf-8'))

    def _load_calib(self):
        """hand-eye 4x4(mm). calib_path 가 비면 d2_vision share/config. 못 읽으면 None(모든 답 unknown)."""
        p = self.get_parameter('calib_path').value or str(Path(get_package_share_directory('d2_vision')) / 'config' / 'T_gripper2camera.npy')
        try:
            T = np.load(p)
            if T.shape != (4, 4):
                raise ValueError('4x4 아님')
            self.get_logger().info('보정값 %s (카메라 위치 mm %s)' % (p, T[:3, 3].round(1).tolist()))
            return T
        except (OSError, ValueError) as e:
            self.get_logger().error('보정값을 못 읽음 %s: %s — 모든 블록을 unknown 으로 답한다' % (p, e))
            return None

    def checker_for(self, design_id):
        """design_id 의 BlockChecker(캐시). `<recipe_dir>/<design_id>.recipe.json` → recipe_blocks(robot.yaml, 레시피) → BlockChecker.
        이름에 경로 문자가 있거나 파일 · 형식이 틀리면 None (task 노드 load_recipe 와 같은 규칙)."""
        if design_id in self.checkers:
            return self.checkers[design_id]
        recipe_dir = self.get_parameter('recipe_dir').value
        if not recipe_dir or not design_id or any(c in design_id for c in '/\\') or design_id.startswith('.'):
            return None
        try:
            recipe = json.loads((Path(recipe_dir) / f'{design_id}.recipe.json').read_text(encoding='utf-8'))
            blocks = recipe_blocks(self.cfg, recipe)
            o = self.cfg['assembly_origin']
            chk = BlockChecker(blocks, (o['x_m'], o['y_m']), float(self.cfg['assembly_area_half_m']), self.cfg['block_actual_m'])
            self.get_logger().info('레시피 %s: 블록 %d개' % (design_id, len(chk.blocks)))
        except (OSError, KeyError, TypeError, ValueError) as e:
            self.get_logger().error('레시피 %s 를 못 읽음(recipe_dir=%r): %s' % (design_id, recipe_dir, e))
            chk = None
        self.checkers[design_id] = chk
        return chk

    # ---------- 카메라 ----------
    def on_depth(self, msg):
        """정렬된 깊이 프레임을 mm float 로 모은다(최근 n_frames 개)."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        if msg.encoding != '16UC1':           # 32FC1 이면 m
            img *= 1000.0
        img[img <= 0] = np.nan                # 깊이 0 = 구멍. 중앙값에 섞이지 않게 NaN
        with self.lock:
            self.frames.append((self.now(), img))
            del self.frames[:-self.n_frames]
            self.last_frame_t = self.frames[-1][0]

    def on_info(self, msg):
        """camera_info → 내부 파라미터 (fx, fy, ppx, ppy) 픽셀 단위."""
        self.intr = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def now(self):
        """노드 시계의 지금 시각(초)."""
        return self.get_clock().now().nanoseconds / 1e9

    def publish_status(self):
        """`/d2/vision/camera_status`(camera_status/1) 2 Hz. last_frame_stamp 는 마지막 깊이 프레임을 받은 시각(초)."""
        msg = {'schema': 'camera_status/1', 'stamp': self.now(), 'node': 'wrist_block', 'last_frame_stamp': self.last_frame_t}
        self.pub_status.publish(String(data=json.dumps(msg)))

    # ---------- 로봇 ----------
    def _call(self, cli, req):
        """서비스를 부르고 답을 기다린다(멀티스레드 실행기 + 재진입 그룹이라 콜백 중에도 됨). 최대 SERVICE_WAIT_S×2 초 막힌다.
        시간 안에 못 받으면 보낸 요청을 치우고 None."""
        if not cli.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            return None
        done = threading.Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(SERVICE_WAIT_S):
            cli.remove_pending_request(fut)
            return None
        return fut.result()

    def check_tcp_once(self):
        """켤 때 한 번(타이머를 바로 끈다): 제어기 활성 TCP 가 d2_bringup config/tcp.json 과 같은지. 다르면 보정값이 안 맞으니 경고(답은 계속 한다)."""
        self._tcp_timer.cancel()
        try:
            want = json.loads((Path(get_package_share_directory('d2_bringup')) / 'config' / 'tcp.json').read_text(encoding='utf-8'))['name']
        except (OSError, KeyError, ValueError):
            return
        r = self._call(self.cli_tcp, GetCurrentTcp.Request())
        got = r.info if r is not None and r.success else ''
        if not got:
            self.get_logger().warn('제어기 TCP 를 못 읽음(에뮬레이터면 정상) — 보정값은 %s 기준' % want)
        elif got != want:
            self.get_logger().error('제어기 활성 TCP "%s" != tcp.json "%s" — hand-eye 보정값이 맞지 않는다. 펜던트에서 맞춘 뒤 다시 켠다' % (got, want))
        else:
            self.get_logger().info('제어기 활성 TCP %s (보정값 기준과 같음)' % got)

    def read_posx(self):
        """posx 6개(mm·deg). 파라미터 posx 가 0 이 아니면 그것(시험용), 아니면 두산 서비스. 못 받으면 None."""
        p = list(self.get_parameter('posx').value)
        if any(abs(v) > 1e-9 for v in p):
            return p
        req = GetCurrentPosx.Request()
        req.ref = 0                            # DR_BASE
        r = self._call(self.cli_posx, req)
        if r is None or not r.success or not r.task_pos_info:
            return None
        return list(r.task_pos_info[0].data[:6])

    def fresh_frames(self, t_req):
        """t_req 뒤에 들어온 깊이 프레임이 n_frames 장 모일 때까지(최대 FRESH_WAIT_S) 기다렸다가 돌려준다. 부족하면 None."""
        deadline = t_req + FRESH_WAIT_S
        while True:
            with self.lock:
                fresh = [img for t, img in self.frames if t > t_req]
            if len(fresh) >= self.n_frames:
                return fresh
            if self.now() > deadline:
                return None
            threading.Event().wait(0.02)

    # ---------- 서비스 ----------
    def _fill(self, res, ids, state='unknown', success=False, reason=''):
        """결과 배열을 요청 길이로 채운다(모두 같은 길이 — IRD 5장)."""
        n = len(ids)
        res.block_ids = list(ids)
        res.states = [state] * n
        res.dx_m, res.dy_m, res.dz_m, res.top_z_m = [NAN] * n, [NAN] * n, [NAN] * n, [NAN] * n
        res.success, res.reason = success, reason
        return res

    def on_check(self, req, res):
        """요청 뒤 새 깊이 프레임 n장 중앙값 + posx → 점군 → BlockChecker → 답. 설정 없음 → ERROR, 프레임 · posx 없음 → TIMEOUT."""
        ids = list(req.block_ids)
        # block_id = '<design_id>_B<순번>' (IRD 2장) → 앞글자로 설계를 찾는다. 한 요청은 한 설계라고 본다(섞이면 나머지는 unknown)
        design_ids = [i.rsplit('_B', 1)[0] for i in ids if '_B' in i]
        checker = self.checker_for(design_ids[0]) if design_ids else None
        if checker is None or self.T_g2c is None:
            self.get_logger().error('보정값 또는 레시피(%s)가 없어 답할 수 없다 → ERROR' % (design_ids[:1] or ids[:1]))
            return self._fill(res, ids, reason='ERROR')
        t_req = self.now()
        posx = self.read_posx()                                           # 로봇은 멈춰 있으니 프레임 모으기와 순서는 무관
        if posx is None:
            self.get_logger().error('두산 posx 를 못 받음 → TIMEOUT')
            return self._fill(res, ids, reason='TIMEOUT')
        frames = self.fresh_frames(t_req)
        if frames is None or self.intr is None:
            self.get_logger().error('요청 뒤 새 깊이 프레임이 %.1f초 안에 %d장 안 모임(카메라 끊김?) → TIMEOUT' % (FRESH_WAIT_S, self.n_frames))
            return self._fill(res, ids, reason='TIMEOUT')

        with warnings.catch_warnings():                                    # 10장 모두 구멍인 화소는 numpy 가 'All-NaN slice' 를 알리지만 결과(NaN → 점에서 빠짐)는 의도한 것
            warnings.simplefilter('ignore', RuntimeWarning)
            depth_m = np.nanmedian(np.stack(frames), axis=0) / 1000.0    # 구멍(NaN)은 빼고 중앙값. 전부 구멍이면 NaN → 점에서 빠짐
        T_b2c = posx_to_matrix(*posx) @ self.T_g2c
        T_b2c[:3, 3] /= 1000.0                                            # 노드 안은 m
        pts = depth_to_base_points(depth_m, self.intr, T_b2c, stride=2)
        results = checker.check(pts, ids)

        res.block_ids = ids
        res.states = [r['state'] for r in results]
        res.dx_m = [r['dx_m'] for r in results]
        res.dy_m = [r['dy_m'] for r in results]
        res.dz_m = [r['dz_m'] for r in results]
        # IRD 5장: absent·unknown 은 top_z 도 NaN (BlockChecker 는 absent 때 보이는 면 높이를 주지만 서비스 답에서는 뺀다)
        res.top_z_m = [r['top_z_m'] if r['state'] in ('present', 'occluded') else NAN for r in results]
        res.success, res.reason = True, ''
        # 로그에는 absent 때 보이는 면(작업면) 높이도 적는다 — 보정 기울기 진단용(10/7 박진용 실기: 윗면 −2.8 mm). 서비스 답은 IRD 5장대로 NaN
        self.get_logger().info('check_progress %d개 (점 %d, posx z %.0f mm) → %s' % (
            len(ids), len(pts), posx[2], {r['block_id']: (r['state'], None if np.isnan(r['top_z_m']) else round(r['top_z_m'] * 1000, 1)) for r in results}))
        return res


def main(args=None):
    """노드를 멀티스레드 실행기로 돌린다(서비스 콜백 안에서 두산 서비스를 부르기 위해). 로봇에 명령이 없어 서기 처리는 없다."""
    rclpy.init(args=args)
    node = WristBlock()
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(node)
    try:
        exe.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

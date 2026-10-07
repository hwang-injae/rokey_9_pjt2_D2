"""손목 블록 인식 노드 (wrist_block) — IRD 4.2 `/d2/vision/check_progress` · 4.1 `camera_status/1` (W041 · W140 E-52).

작업 관리자가 관측 자세에서 `check_progress`(design_id · block_ids)를 부르면:
  손목 깊이 영상 10장(중앙값) + 두산 posx + hand-eye 보정값 → base 점군 → BlockChecker(block_checker.py)
  → 블록마다 state(present·absent·occluded·unknown) · top_z_m · dz_m (dx·dy 는 1차 NaN) 로 답한다.
카메라 연결 신호 `/d2/vision/camera_status` 를 2 Hz 로 낸다(마지막 프레임 시각).

설계 · 블록 이름 (E-52, 10/7)
  요청 design_id 칸으로 설계를 고른다(블록 이름에서 잘라 내지 않는다). block_ids 는 전체 블록 이름
  `<design_id>_<역할>_<부품 3자리>_<블록 2자리>`(예 001_CHAIR_BENCH_LEG_001_01)를 그대로 받아 레시피가 만든 이름과 맞춘다.
  design_id 가 비면 옛 방식(10/8 저녁까지만): 첫 block_id 의 '_B' 앞(001_CHAIR_BENCH_B003 → 001_CHAIR_BENCH).
  한 요청은 한 설계다 — 그 설계 레시피에 없는 블록은 그 블록만 unknown.

설계 읽기 — task 와 같은 파라미터 design_source (10/7 민범진 · 한석형 합의). **task 와 손목에 같은 design_source 값을 준다**
  (다르면 task 는 웹 설계로 쌓는데 손목은 파일 설계로 보는 식으로 어긋난다).
  remote(기본)  설계를 늘 `/d2/hmi/get_design`(JsonQuery, 요청 {"design_id"} → 응답 design/1)으로 받는다 — 생성 · 스캔 설계도 같은 길.
                design/1 의 structure + recipe 를 load_recipe 와 같은 모양으로 붙여(block_checker.recipe_from_design)
                팀 공용 `d2_motion.motion_math.recipe_blocks()` 로 base 블록 목록을 만든다. 설계마다 받아 캐시(실패는 캐시 안 함).
                제한 시간 robot.yaml timeout.service_s. 실패하면 로컬 파일로 대신하지 않고 check_progress 를 실패로 답한다(아래).
  local         파일만(웹 없이 개발할 때 명시): `<recipe_dir>/<design_id>_recipe.json`(새, E-52 — 같은 폴더 `_structure.json` 도 읽음),
                없으면 `<design_id>.recipe.json`(옛)을 `load_recipe()` · `recipe_blocks()` 로(설계마다 캐시).
  캐시는 한 판(run) 안에서만(E-55 ① 10/7 PL — 기본 설계는 같은 design_id 로 다시 등록될 수 있다): 설계 블록 **전부**를 묻는 요청
  (= 작업 관리자의 시작 확인 CHECK)이 오면 새 판으로 보고 그 설계를 다시 읽는다. 블록 몇 개만 묻는 배치 확인(VERIFY)은 캐시를 쓴다.

입력(파라미터)
  design_source  'remote'(기본) · 'local' — task 노드와 같은 이름 · 기본값 · 뜻. 'local' 이 아니면 모두 remote 로 본다(task 와 같음)
  recipe_dir   레시피 폴더(task 노드와 같은 파라미터, 예 src/recipe_manager/recipes). design_source:=local 일 때만 쓴다
  calib_path   T_gripper2camera.npy(카메라 → TCP 4x4, mm). 비우면 이 패키지 share/config 의 것
  cam_prefix   realsense 토픽 접두 (Jazzy 기본 /camera/camera)
  posx         시험용 posx 6개(mm·deg, **실수로** 예: [460.5, -157.0, 294.8, 154.8, 180.0, 154.5]). 비우면 두산 서비스로 읽는다
  n_frames     중앙값 낼 깊이 프레임 수(10 — V-17 경험: 정지 상태 10장이면 ±0.5 mm)
robot.yaml(d2_bringup)에서 assembly_origin · assembly_area_half_m · block_actual_m · timeout.service_s 를 읽는다.

바깥 영향: 서비스 답 · camera_status 발행 · get_design 조회(remote, 설계마다 · 판마다 한 번)뿐. 로봇·카메라·그리퍼에 명령을 보내지 않는다
  (관측 자세 이동은 작업 관리자가 move_to 로).
답할 때: 요청이 온 **뒤에** 들어온 깊이 프레임 n_frames 장(최대 1.5초 기다림)의 중앙값을 쓴다 — 로봇이 막 멈춘 직후의 움직이던 프레임을 섞지 않으려고.
  remote 에서 처음 보는 설계이거나 시작 확인(블록 전부)이면 그 앞에 get_design 을 최대 timeout.service_s(3초) 기다린다.
실패 때: 새 프레임이 시간 안에 안 오거나 posx 를 못 받으면 success=false, reason=TIMEOUT.
         get_design 이 timeout.service_s 안에 답하지 않으면 success=false, reason=TIMEOUT (task_node.get_design 과 같은 코드).
         get_design 서버 없음 · success=false · 응답 형식 오류, 보정값·레시피를 못 읽음, 설계를 못 고름 → 노드는 뜬 채 success=false, reason=ERROR.
         (배열은 모두 요청 길이, state unknown, 값 NaN.)
NaN 규칙(CheckProgress.srv · IRD 5장 W121 C-5): dx·dy 는 1차 늘 NaN. dz_m 은 present 일 때만 값.
  top_z_m 은 present · occluded(설계 밖 물체 윗면, 참고값) 일 때 값. absent·unknown · 위 블록에 가려진 present 는 dz·top_z 둘 다 NaN.
보정값은 TCP 기준이라(config/T_gripper2camera.json) 켤 때 제어기 활성 TCP 가 d2_bringup config/tcp.json 과 다르면 경고한다(한 번).

실행 (저장소 맨 위에서). **task 와 같은 design_source 값을 준다:**
  웹 · 다리와 함께(기본 remote — task 도 기본 remote):
    ros2 run d2_vision wrist_block
  웹 없이 파일로(task 도 -p design_source:=local -p recipe_dir:=… 로 띄운다):
    ros2 run d2_vision wrist_block --ros-args -p design_source:=local -p recipe_dir:=src/recipe_manager/recipes
시험 호출 (새 — design_id 칸 + 전체 블록 이름):
  ros2 service call /d2/vision/check_progress d2_interfaces/srv/CheckProgress "{design_id: 001_CHAIR_BENCH, block_ids: [001_CHAIR_BENCH_LEG_001_01, 001_CHAIR_BENCH_SEAT_001_03]}"
시험 호출 (옛 — design_id 빈 값, 레시피가 아직 옛 이름일 때. 10/8 저녁까지):
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

from d2_interfaces.srv import CheckProgress, JsonQuery
from d2_motion.motion_math import RECIPE_SUFFIXES, load_recipe, recipe_blocks
from d2_vision.block_checker import BlockChecker, depth_to_base_points, design_of, recipe_from_design, recipe_path
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
        """파라미터 · 설정 · 보정값을 읽고 구독 · 서비스 · 클라이언트(posx · tcp · get_design) · 타이머를 만든다.
        설계는 요청이 올 때 읽는다(설계마다 한 번). 레시피 폴더 · 보정이 없어도 노드는 뜬다(경고)."""
        super().__init__('wrist_block')
        self.declare_parameter('recipe_dir', '')           # design_source:=local 일 때만 쓰는 레시피 폴더(task 와 같음)
        self.declare_parameter('design_source', 'remote')  # task 와 같은 값: remote = /d2/hmi/get_design(기본) · local = recipe_dir 파일
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
        self.service_s = float(self.cfg['timeout']['service_s'])   # get_design 한 번의 제한 시간(task 와 같은 값)
        self.checkers = {}                    # (design_source, design_id) → BlockChecker — 설계마다 한 번만 읽는다
        if self.get_parameter('design_source').value == 'local':
            self.get_logger().info('설계 읽기: local — recipe_dir 파일만 (task 도 design_source:=local 이어야 한다)')
            if not self.get_parameter('recipe_dir').value:
                self.get_logger().warn('recipe_dir 파라미터가 비었다 — check_progress 는 ERROR 로 답한다')
        else:
            self.get_logger().info('설계 읽기: remote — /d2/hmi/get_design (제한 %.1f초, task 도 remote 여야 한다)' % self.service_s)

        group = ReentrantCallbackGroup()
        prefix = self.get_parameter('cam_prefix').value
        self.create_subscription(Image, f'{prefix}/aligned_depth_to_color/image_raw', self.on_depth, 10, callback_group=group)
        self.create_subscription(CameraInfo, f'{prefix}/color/camera_info', self.on_info, 10, callback_group=group)
        self.cli_posx = self.create_client(GetCurrentPosx, '/dsr_controller2/aux_control/get_current_posx', callback_group=group)
        self.cli_tcp = self.create_client(GetCurrentTcp, '/dsr_controller2/tcp/get_current_tcp', callback_group=group)
        self.cli_design = self.create_client(JsonQuery, '/d2/hmi/get_design', callback_group=group)
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

    def checker_for(self, design_id, block_ids):
        """설계 이름 → (그 설계의 BlockChecker 또는 None, 실패 코드). design_source 에 따라 원격(기본) 또는 로컬 파일 하나만 쓴다.

        입력: design_id · block_ids(이번 요청의 블록 이름 — 캐시한 설계의 블록을 전부 담고 있으면 새 판의 시작 확인으로 보고 다시 읽는다, E-55 ①).
        출력: 성공 (BlockChecker, '') · 실패 (None, 'ERROR' 또는 'TIMEOUT') — 부르는 쪽이 그 코드로 check_progress 를 답한다.
        바깥 영향: 로그, remote 면 get_design 조회(설계마다 · 판마다 한 번). 캐시 열쇠는 (design_source, design_id) —
        돌리는 중에 design_source 를 바꿔도 다른 길로 읽은 설계를 쓰지 않는다. 다시 읽기가 실패하면 옛 캐시도 버린다(옛 설계로 답하지 않는다).
        - remote: _remote_checker. 실패는 캐시하지 않는다(웹 · 다리가 늦게 떠도 다음 요청에 다시 받는다).
        - local: `<recipe_dir>/<design_id>_recipe.json`(새 — load_recipe 가 옆 `_structure.json` 도 붙임), 없으면 `.recipe.json`(옛)
          → recipe_blocks(robot.yaml, 레시피) → BlockChecker. block_id 는 recipe_blocks 가 만든 이름(새 형식은 '<model_id>_<블록 이름>').
          파일을 못 찾으면(이름 비었음 · 경로 문자 · 파일 없음) 캐시하지 않아 다음 요청에 다시 찾고, 찾은 파일을 못 읽으면
          (구조 파일 없음 · 형식 틀림 · 블록 크기 다름) None 을 캐시한다(같은 오류를 매번 읽지 않게)."""
        if not design_id:                     # 설계를 못 고름(design_id 칸이 비었고 '_B' 이름도 없음) — 로그는 on_check 가 남긴다
            return None, 'ERROR'
        source = 'local' if self.get_parameter('design_source').value == 'local' else 'remote'
        key = (source, design_id)
        if key in self.checkers:
            chk = self.checkers[key]
            # E-55 ①(10/7 PL): 캐시는 한 번의 조립(run) 안에서만 — 기본 설계는 레시피 파일을 고치면 같은 design_id 로 다시 등록된다.
            # 손목은 [설계 선택]을 못 받으므로 '설계 블록 전부를 묻는 요청'(= 작업 관리자의 시작 확인 CHECK, SDD §6.3)을 새 판의 시작으로 보고
            # 그때 다시 받는다. 블록 1~2개를 묻는 배치 확인(VERIFY)은 캐시를 쓴다(블록이 2개뿐인 설계는 VERIFY 도 전부라 매번 받지만 해롭지 않다).
            if chk is not None and set(chk.blocks) <= set(block_ids):
                self.get_logger().info('설계 %s: 블록 전부를 묻는 요청(시작 확인) — 새 판으로 보고 다시 읽는다' % design_id)
                del self.checkers[key]
            else:
                return chk, ('' if chk is not None else 'ERROR')
        if source == 'remote':
            chk, reason = self._remote_checker(design_id)
            if chk is not None:
                self.checkers[key] = chk
            return chk, reason
        recipe_dir = self.get_parameter('recipe_dir').value
        path = recipe_path(recipe_dir, design_id, RECIPE_SUFFIXES)
        if path is None:
            self.get_logger().error('레시피 %r 파일을 못 찾음(recipe_dir=%r, 끝 %s)' % (design_id, recipe_dir, ' · '.join(RECIPE_SUFFIXES)))
            return None, 'ERROR'
        try:
            chk = self._make_checker(load_recipe(str(path)))
            self.get_logger().info('레시피 %s (%s): 블록 %d개' % (design_id, path.name, len(chk.blocks)))
        except (OSError, KeyError, TypeError, ValueError) as e:
            self.get_logger().error('레시피 %s 를 못 읽음(%s): %s' % (design_id, path, e))
            chk = None
        self.checkers[key] = chk
        return chk, ('' if chk is not None else 'ERROR')

    def _make_checker(self, recipe):
        """레시피 dict(load_recipe 모양, mm) → 팀 공용 recipe_blocks(robot.yaml) → BlockChecker. 실패는 recipe_blocks 의 예외 그대로."""
        o = self.cfg['assembly_origin']
        return BlockChecker(recipe_blocks(self.cfg, recipe), (o['x_m'], o['y_m']),
                            float(self.cfg['assembly_area_half_m']), self.cfg['block_actual_m'])

    def _remote_checker(self, design_id):
        """/d2/hmi/get_design 으로 design/1 을 받아 BlockChecker 를 만든다. 출력: (BlockChecker, '') 또는 (None, 코드).

        요청 {"design_id"} → 최대 timeout.service_s 기다림(서비스 콜백 안이라 _call — 멀티스레드 실행기 + 재진입 그룹).
        실패 코드는 task_node.get_design 과 맞춘다: 서버 없음(지금 안 보임) → ERROR · 시간 초과(요청을 치움) → TIMEOUT ·
        success=false → ERROR(서버 이유는 로그에만 — 10/7 합의) · JSON · design/1 형식 · 레시피 내용 오류 → ERROR.
        로컬 파일로 몰래 대신하지 않는다. 바깥 영향: get_design 요청 하나 · 로그."""
        if not self.cli_design.service_is_ready():
            self.get_logger().error('get_design 서버가 없다(다리 · 웹?) — 설계 %s 를 못 받음 → ERROR' % design_id)
            return None, 'ERROR'
        req = JsonQuery.Request()
        req.request_json = json.dumps({'design_id': design_id}, ensure_ascii=False)
        res = self._call(self.cli_design, req, self.service_s)
        if res is None:
            self.get_logger().error('get_design 이 %.1f초 안에 답하지 않음 — 설계 %s → TIMEOUT' % (self.service_s, design_id))
            return None, 'TIMEOUT'
        if not res.success:
            self.get_logger().error('get_design 실패(%r) — 설계 %s → ERROR' % (res.reason, design_id))
            return None, 'ERROR'
        try:
            chk = self._make_checker(recipe_from_design(json.loads(res.response_json), design_id))
        except (AttributeError, IndexError, KeyError, TypeError, ValueError) as e:   # JSON 깨짐(ValueError) · design/1 형식 · 레시피 내용 — 바깥 입력이라 넓게 잡아 콜백이 죽지 않게
            self.get_logger().error('get_design 답으로 설계 %s 를 못 읽음: %s → ERROR' % (design_id, e))
            return None, 'ERROR'
        self.get_logger().info('설계 %s (get_design): 블록 %d개' % (design_id, len(chk.blocks)))
        return chk, ''

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
    def _call(self, cli, req, wait_s=SERVICE_WAIT_S):
        """서비스를 부르고 답을 기다린다(멀티스레드 실행기 + 재진입 그룹이라 콜백 중에도 됨). 서버 기다림 wait_s + 답 기다림 wait_s
        (초 — 두산 서비스는 SERVICE_WAIT_S, get_design 은 timeout.service_s)만큼 막힌다. 시간 안에 못 받으면 보낸 요청을 치우고 None."""
        if not cli.wait_for_service(timeout_sec=wait_s):
            return None
        done = threading.Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(wait_s):
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
        """check_progress 콜백. 입력 req.design_id(빈 값 가능) · req.block_ids(전체 블록 이름). 출력 res(배열은 모두 요청 길이).

        보정값 확인 → 설계 고르기 · 읽기(checker_for — remote 면 처음 보는 설계 · 시작 확인(블록 전부)만 get_design) → 요청 뒤 새 깊이 프레임 n장 중앙값
        + posx → 점군 → BlockChecker → 답. 바깥 영향: get_design · 두산 posx 조회 · 로그.
        실패: 보정값 없음 · 설계를 못 고름 · 설계를 못 읽음 → ERROR(get_design 시간 초과만 TIMEOUT), 프레임 · posx 없음 → TIMEOUT
        (모두 state unknown, 값 NaN)."""
        ids = list(req.block_ids)
        if self.T_g2c is None:                # 보정값이 없으면 설계를 받아도 답할 수 없다 — get_design 을 부르지 않는다
            self.get_logger().error('보정값이 없어 답할 수 없다 → ERROR')
            return self._fill(res, ids, reason='ERROR')
        # 설계 = design_id 칸(E-52). 비었으면 옛 방식으로 첫 '_B' block_id 의 앞(10/8 저녁까지 — remote 면 그 이름으로 get_design).
        # 한 요청은 한 설계라고 본다 — 그 설계에 없는 블록은 BlockChecker 가 그 블록만 unknown 으로 답한다
        design_id = next((d for d in (design_of(req.design_id, i) for i in ids) if d), '')
        checker, reason = self.checker_for(design_id, ids)
        if checker is None:
            self.get_logger().error('설계(%r, design_id 칸 %r)를 못 읽어 답할 수 없다 → %s' % (design_id or ids[:1], req.design_id, reason))
            return self._fill(res, ids, reason=reason)
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
        self.get_logger().info('check_progress %s %d개 (점 %d, posx z %.0f mm) → %s' % (
            design_id, len(ids), len(pts), posx[2], {r['block_id']: (r['state'], None if np.isnan(r['top_z_m']) else round(r['top_z_m'] * 1000, 1)) for r in results}))
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

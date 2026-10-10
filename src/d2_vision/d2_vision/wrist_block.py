"""손목 블록 인식 노드 (wrist_block) — IRD 4.2 `/d2/vision/check_progress` · `scan_capture` · `scan_infer` · `find_blocks`
· 4.1 `camera_status/1` · `wrist_image` (W041 · W140 E-52 → E-69 · W115 · W086 · W148).

작업 관리자가 관측 자세에서 `check_progress`(design_id · block_ids)를 부르면:
  손목 깊이 영상 10장(중앙값) + 로봇 자세(TF base_link → rg2_tcp, 10/9 E-70 ③) + hand-eye 보정값(rg2_tcp 틀) → base 점군 → BlockChecker(block_checker.py)
  → 블록마다 state(present·absent·occluded·unknown) · top_z_m · dz_m (dx·dy 는 1차 NaN) 로 답한다.
카메라 연결 신호 `/d2/vision/camera_status` 를 2 Hz 로 낸다(마지막 프레임 시각).
스캔 · 흩뿌림 찾기도 이 노드 하나가 맡는다(IRD 84줄 '노드를 나눌지는 비전이 정함' → 카메라 구독 · 로봇 자세 · 보정값을 같이 쓰려고 한 노드, 10/8).
  `/d2/vision/scan_capture`(JsonQuery) {"pose_id","run_id"} → 요청 뒤 깊이 scan_frames 장 평균(구멍 0 은 빼고) + 지금 TF 자세 × 보정
      → StructureScanner.add_capture(structure_scanner.py) → {"ok":true,"points":n}. run_id 마다 따로 모으고, 같은 run_id 에 같은
      pose_id 가 다시 오면 덮어쓴다. 그때의 컬러 한 장도 둔다(scan_infer 의 사진).
  `/d2/vision/scan_infer`(JsonQuery) {"run_id"} → infer() → `<scan_dir>/<run_id>/` 에 color.png · cloud.ply(≤ 2 MB) · blocks.json
      → {"ok":true,"blocks":blocks/1,"inferred_count","image_path","cloud_path"}. 설계 이름 scan_<family>_<번호>(IRD 2장) —
      family 는 recipe_dir 의 기본 설계(이름에 CHAIR · DESK) 중 블록 수 · 외곽이 가장 가까운 것, 못 구하면 unknown.
      번호 = scan_dir 에 이미 있는 blocks.json 수 + 1(노드를 다시 켜도 이름이 겹치지 않게). 추론이 끝난 run_id 는 메모리에서 지운다.
  `/d2/vision/find_blocks`(JsonQuery) {"run_id"} → 요청 뒤 컬러 · 깊이(scan_frames 장 평균) + TF 자세 × 보정 → BlockFinder(block_finder.py)
      → {"ok":true,"blocks":[…]}(IRD 4.2 칸, 윗면 높이 순). 블록 0개도 ok true(빈 목록 — 작업 관리자가 WAIT_SUPPLY).
      마스크: yolo_model 파라미터가 있고 ultralytics 를 쓸 수 있으면 YOLO seg(본 방법 E-38), 아니면 예비(엣지 + 깊이, E-47).
  `/d2/vision/wrist_image`(sensor_msgs/CompressedImage, JPEG 품질 70, 640×480 그대로) — find_blocks 때마다 검출 결과
      (윤곽 · 번호 · up · yaw)를 그린 그림 한 장(IRD E-67 '때만 내도 됨'). 매 프레임 내지 않는다.

설계 · 블록 이름 (E-52, 10/7)
  요청 design_id 칸으로 설계를 고른다(블록 이름에서 잘라 내지 않는다). block_ids 는 전체 블록 이름
  `<design_id>_<역할>_<부품 3자리>_<블록 2자리>`(예 001_CHAIR_BENCH_LEG_001_01)를 그대로 받아 레시피가 만든 이름과 맞춘다.
  design_id 칸이 비면 설계를 고를 수 없어 실패로 답한다(success=false, reason=ERROR — 아래 '실패 때').
  한 요청은 한 설계다 — 그 설계 레시피에 없는 블록은 그 블록만 unknown.

설계 읽기 — task 와 같은 파라미터 design_source (10/7 민범진 · 한석형 합의). **task 와 손목에 같은 design_source 값을 준다**
  (다르면 task 는 웹 설계로 쌓는데 손목은 파일 설계로 보는 식으로 어긋난다).
  remote(기본)  설계를 늘 `/d2/hmi/get_design`(JsonQuery, 요청 {"design_id"} → 응답 design/2.0)으로 받는다 — 생성 · 스캔 설계도 같은 길.
                design/2.0 의 recipe(구조) + placements(조립 방법)를 load_recipe 와 같은 모양으로 붙여(block_checker.recipe_from_design)
                팀 공용 `d2_motion.motion_math.recipe_blocks()` 로 base 블록 목록을 만든다. 설계마다 받아 캐시(실패는 캐시 안 함).
                제한 시간 robot.yaml timeout.service_s. 실패하면 로컬 파일로 대신하지 않고 check_progress 를 실패로 답한다(아래).
  local         파일만(웹 없이 개발할 때 명시): `<recipe_dir>/<design_id>_recipe.json`(구조, E-69 — 같은 폴더 `_placements.csv` 도 읽음)을
                `load_recipe()` · `recipe_blocks()` 로(설계마다 캐시).
  캐시는 한 판(run) 안에서만(E-55 ① 10/7 PL — 기본 설계는 같은 design_id 로 다시 등록될 수 있다): 설계 블록 **전부**를 묻는 요청
  (= 작업 관리자의 시작 확인 CHECK)이 오면 새 판으로 보고 그 설계를 다시 읽는다. 블록 몇 개만 묻는 배치 확인(VERIFY)은 캐시를 쓴다.

입력(파라미터)
  design_source  'remote'(기본) · 'local' — task 노드와 같은 이름 · 기본값 · 뜻. 'local' 이 아니면 모두 remote 로 본다(task 와 같음)
  recipe_dir   레시피 폴더(task 노드와 같은 파라미터, 예 src/recipe_manager/recipes). design_source:=local 일 때 check_progress 설계,
               그리고 scan_infer 의 family 고르기(기본 설계 4개)에 쓴다 — 비우면 스캔 family 는 unknown
  calib_path   T_rg2tcp2camera.npy(카메라 → rg2_tcp 4x4, mm — 10/10 W156). 비우면 이 패키지 share/config 의 것
  cam_prefix   realsense 토픽 접두 (Jazzy 기본 /camera/camera) — 깊이 · camera_info · 컬러(<접두>/color/image_raw)
  tcp_pose_mm  시험용 rg2_tcp 자세 7개(x · y · z mm + 쿼터니언 x · y · z · w, **실수로**). 모두 0 이면 TF(robot.yaml frame_id → tcp_link)로 읽는다
  n_frames     check_progress 가 중앙값 낼 깊이 프레임 수(10 — V-17 경험: 정지 상태 10장이면 ±0.5 mm)
  scan_frames  scan_capture · find_blocks 가 평균 낼 깊이 프레임 수(10 — 30 fps 면 0.33초, task 의 3초 제한 안. 45장까지 1.5초 안)
  scan_dir     스캔 결과 폴더(비우면 ~/d2_data/scan) — <run_id>/color.png · cloud.ply · blocks.json
  yolo_model   YOLO seg 모델 파일(.pt) 경로. 비우거나 못 읽으면 예비 마스크(엣지 + 깊이)로 간다(켤 때 로그 한 번)
  yolo_conf    YOLO 검출 신뢰도 문턱(0.55 — 10/8 확정 모델 26s판: 검증 장면 재현율 ≥ 0.95 를 지키는 가장 높은 값, 0.5 보다 헛것만 줄고 놓침은 같음)
robot.yaml(d2_bringup)에서 assembly_origin · assembly_area_half_m · block_size_m · block_actual_m · timeout.service_s ·
  table_z_m · find.min_gap_mm · grasp_depth_m · finger.width_m 를 읽는다.

바깥 영향: 서비스 답 · camera_status · wrist_image 발행 · get_design 조회(remote, 설계마다 · 판마다 한 번) · scan_infer 의 파일 쓰기
  (scan_dir 아래)뿐. 로봇·카메라·그리퍼에 명령을 보내지 않는다(관측 · 촬영 자세 이동은 작업 관리자가 move_to 로).
답할 때: 요청이 온 **뒤에** 들어온 깊이 프레임 n_frames 장(최대 1.5초 기다림)의 중앙값을 쓴다 — 로봇이 막 멈춘 직후의 움직이던 프레임을 섞지 않으려고.
  remote 에서 처음 보는 설계이거나 시작 확인(블록 전부)이면 그 앞에 get_design 을 최대 timeout.service_s(3초) 기다린다.
실패 때: 새 프레임이 시간 안에 안 오거나 TF 자세(브링업 robot_state_publisher)를 못 받으면 success=false, reason=TIMEOUT.
         get_design 이 timeout.service_s 안에 답하지 않으면 success=false, reason=TIMEOUT (task_node.get_design 과 같은 코드).
         get_design 서버 없음 · success=false · 응답 형식 오류(옛 design/1 포함), 보정값·레시피를 못 읽음, design_id 칸이 빔
         → 노드는 뜬 채 success=false, reason=ERROR.
         (배열은 모두 요청 길이, state unknown, 값 NaN.)
스캔 · 찾기 실패 때(JsonQuery 두 층 — mock_wrist_block 과 같음. 응답 JSON 은 늘 {"ok":false,"reason","detail"}):
  요청 JSON 이 틀림(run_id · pose_id 없음 · 폴더 이름으로 못 쓰는 글자) → success=false, SCAN_FAILED.
  보정값 없음 · 계산 예외 → success=false, ERROR. TF 자세 못 받음 · 요청 뒤 새 깊이(find_blocks 는 컬러도)가 시간 안에 안 옴 → success=false, TIMEOUT.
  촬영 안 한 run_id · 추론 신뢰도 미달(점 설명률 · 추정 블록 비율 · 자세 수 — StructureScanner ⑧) → success=true + ok:false SCAN_FAILED.
  PLY 저장만 실패하면 cloud_path 칸을 빼고 ok:true 로 답하고 경고 로그(IRD 4.2 cloud_path 선택 칸, 10/8 PL).
NaN 규칙(CheckProgress.srv · IRD 5장 W121 C-5): dx·dy 는 1차 늘 NaN. dz_m 은 present 일 때만 값.
  top_z_m 은 present · occluded(설계 밖 물체 윗면, 참고값) 일 때 값. absent·unknown · 위 블록에 가려진 present 는 dz·top_z 둘 다 NaN.
로봇 자세는 MoveIt 모델(관절값 + config/tcp.json 으로 만든 rg2_tcp)에서 나오므로 펜던트 활성 TCP 와 상관없다(10/10 W156 — 예전 posx × T_gripper2camera 길은 뺌).

실행 (저장소 맨 위에서). **task 와 같은 design_source 값을 준다:**
  웹 · 다리와 함께(기본 remote — task 도 기본 remote):
    ros2 run d2_vision wrist_block
  웹 없이 파일로(task 도 -p design_source:=local -p recipe_dir:=… 로 띄운다):
    ros2 run d2_vision wrist_block --ros-args -p design_source:=local -p recipe_dir:=src/recipe_manager/recipes
시험 호출 (design_id 칸 + 전체 블록 이름):
  ros2 service call /d2/vision/check_progress d2_interfaces/srv/CheckProgress "{design_id: 001_CHAIR_BENCH, block_ids: [001_CHAIR_BENCH_LEG_001_01, 001_CHAIR_BENCH_SEAT_001_03]}"
스캔(촬영 자세마다 scan_capture → 마지막에 scan_infer) · 흩뿌림 찾기 · 검출 그림 보기:
  ros2 run d2_vision wrist_block --ros-args -p recipe_dir:=src/recipe_manager/recipes -p yolo_model:=<모델 .pt 경로>
  ros2 service call /d2/vision/scan_capture d2_interfaces/srv/JsonQuery "{request_json: '{\\"pose_id\\": \\"observe_front\\", \\"run_id\\": \\"R1\\"}'}"
  ros2 service call /d2/vision/scan_infer d2_interfaces/srv/JsonQuery "{request_json: '{\\"run_id\\": \\"R1\\"}'}"
  ros2 service call /d2/vision/find_blocks d2_interfaces/srv/JsonQuery "{request_json: '{\\"run_id\\": \\"R1\\"}'}"
  ros2 topic echo --once /d2/vision/wrist_image --field format     (find_blocks 를 부른 뒤 'jpeg' 이 한 번 온다)
"""
import json
import threading
import time
import warnings
from collections import OrderedDict
from pathlib import Path

import cv2
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
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from d2_interfaces.srv import CheckProgress, JsonQuery
from d2_motion.motion_math import RECIPE_SUFFIXES, load_recipe, recipe_blocks
from d2_vision.block_checker import BlockChecker, depth_to_base_points, mean_depth_mm, recipe_from_design, recipe_path
from d2_vision.block_finder import BlockFinder, draw_found, finder_cfg, masks_from_yolo
# 요청 검사(run_id 를 폴더 이름으로 써도 되는지까지) · 레시피 두 파일 → blocks/1 은 가짜 노드와 같은 함수를 쓴다(ROS 없는 계산, 시험 있음)
from d2_vision.mock_scan import load_recipe_files, parse_request, structure_to_blocks
from d2_vision.structure_scanner import StructureScanner, family_of, scan_response
from d2_vision.tcp_pose import TcpPose, default_calib_path, load_calib

NAN = float('nan')
QOS_STATUS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
FRESH_WAIT_S = 1.5         # 요청 뒤 새 프레임 n장을 이만큼 기다린다(30 Hz 면 10장에 0.33초 — 여유 포함). 넘으면 TIMEOUT
SERVICE_WAIT_S = 3.0       # TF 자세 · 서비스(get_design 은 timeout.service_s) 기본 기다림 — 10/10 W156 뒤 두산 서비스는 안 부름
MAX_SCAN_RUNS = 5          # 추론 전 run_id 를 이만큼만 기억한다(scan_infer 가 안 온 판이 쌓여 메모리가 늘지 않게 — 한 판 점군 약 2~5 MB)
JPEG_QUALITY = 70          # wrist_image (IRD E-67 — 640×480 · 품질 70 ≈ 65 KB)


def posx_to_matrix(x, y, z, rx, ry, rz):
    """두산 posx(mm, ZYZ deg) → T_base2gripper 4x4 (mm). 노드는 이제 TF 를 쓰고(W156), 이 함수는 저장소 밖 시험 스크립트(fs_rotate_test 등)가 가져다 쓴다."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


class WristBlock(Node):
    """check_progress · scan_capture · scan_infer · find_blocks 에 답하고 camera_status · wrist_image 를 내는 손목 블록 인식 노드."""

    def __init__(self):
        """파라미터 · 설정 · 보정값 · YOLO 모델 · 기본 설계(스캔 family 용)를 읽고 구독 · 서비스 · 클라이언트(get_design) · TF 리스너 ·
        발행 · 타이머를 만든다. check_progress 설계는 요청이 올 때 읽는다(설계마다 한 번). 레시피 폴더 · 보정 · 모델이 없어도 노드는 뜬다(경고)."""
        super().__init__('wrist_block')
        self.declare_parameter('recipe_dir', '')           # design_source:=local 설계 · 스캔 family 고르기에 쓰는 레시피 폴더(task 와 같음)
        self.declare_parameter('design_source', 'remote')  # task 와 같은 값: remote = /d2/hmi/get_design(기본) · local = recipe_dir 파일
        self.declare_parameter('calib_path', '')
        self.declare_parameter('cam_prefix', '/camera/camera')
        self.declare_parameter('tcp_pose_mm', [0.0] * 7)
        self.declare_parameter('n_frames', 10)
        self.declare_parameter('scan_frames', 10)
        self.declare_parameter('scan_dir', '')
        self.declare_parameter('yolo_model', '')
        self.declare_parameter('yolo_conf', 0.55)
        self.declare_parameter('status_hz', 2.0)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = []                      # (받은 시각 s, 깊이 mm float32) 최근 max(n_frames, scan_frames) 개
        self.color = None                     # (받은 시각 s, 컬러 BGR uint8, frame_id) 마지막 한 장
        self.intr = None                      # (fx, fy, ppx, ppy) 픽셀 단위
        self.last_frame_t = 0.0
        self.n_frames = int(self.get_parameter('n_frames').value)
        self.scan_frames = int(self.get_parameter('scan_frames').value)
        self.keep_frames = max(self.n_frames, self.scan_frames)
        self.scan_dir = Path(self.get_parameter('scan_dir').value or Path.home() / 'd2_data' / 'scan')
        self.scans = OrderedDict()            # run_id → {'scanner': StructureScanner, 'color': BGR 또는 None} — 추론 전 판만
        self.scan_lock = threading.Lock()
        self.find_lock = threading.Lock()     # BlockFinder 가 마지막 마스크를 들고 있어 한 번에 하나씩

        self.cfg = self._load_robot_yaml()
        self.T_tcp2c = self._load_calib()
        self.tcp = TcpPose(self, self.cfg.get('frame_id', 'base_link'), self.cfg.get('tcp_link', 'rg2_tcp'))
        self.finder = BlockFinder(finder_cfg(self.cfg))
        self.yolo = self._load_yolo()
        self.bases = self._load_bases()
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
        self.create_subscription(Image, f'{prefix}/color/image_raw', self.on_color, 10, callback_group=group)
        self.cli_design = self.create_client(JsonQuery, '/d2/hmi/get_design', callback_group=group)
        self.create_service(CheckProgress, '/d2/vision/check_progress', self.on_check, callback_group=group)
        self.create_service(JsonQuery, '/d2/vision/scan_capture', self.on_scan_capture, callback_group=group)
        self.create_service(JsonQuery, '/d2/vision/scan_infer', self.on_scan_infer, callback_group=group)
        self.create_service(JsonQuery, '/d2/vision/find_blocks', self.on_find_blocks, callback_group=group)
        self.pub_status = self.create_publisher(String, '/d2/vision/camera_status', QOS_STATUS)
        self.pub_image = self.create_publisher(CompressedImage, '/d2/vision/wrist_image', 10)   # IRD E-67 QoS 기본(VOLATILE)
        self.create_timer(1.0 / self.get_parameter('status_hz').value, self.publish_status, callback_group=group)
        self.get_logger().info('wrist_block 시작: check_progress · scan_capture · scan_infer · find_blocks 대기 (카메라 %s, 스캔 폴더 %s)'
                               % (prefix, self.scan_dir))

    # ---------- 설정 읽기 ----------
    def _load_robot_yaml(self):
        """d2_bringup config/robot.yaml → dict. 없으면 예외(설정 없이 돌지 않는다)."""
        p = Path(get_package_share_directory('d2_bringup')) / 'config' / 'robot.yaml'
        return yaml.safe_load(p.read_text(encoding='utf-8'))

    def _load_calib(self):
        """hand-eye 4x4(mm, 카메라 → rg2_tcp). calib_path 가 비면 d2_vision share/config. 못 읽으면 None(모든 답 unknown)."""
        p = self.get_parameter('calib_path').value or str(default_calib_path())
        try:
            T = load_calib(p)
            self.get_logger().info('보정값 %s (카메라 위치 mm %s)' % (p, T[:3, 3].round(1).tolist()))
            return T
        except (OSError, ValueError) as e:
            self.get_logger().error('보정값을 못 읽음 %s: %s — 모든 블록을 unknown 으로 답한다' % (p, e))
            return None

    def _load_yolo(self):
        """yolo_model 파라미터의 YOLO seg 모델을 읽는다. 비었거나 ultralytics · 파일이 없거나 못 읽으면 None(예비 마스크) — 로그는 여기서 한 번.
        켤 때 읽는 이유: 처음 읽기는 몇 초 걸려 find_blocks 의 3초 제한(timeout.service_s) 안에 넣지 않으려고.
        읽은 뒤 빈 640×480 그림으로 한 번 미리 돌린다 — 첫 추론 준비(10/8 실측 약 0.9초)를 첫 find_blocks 에서 빼려고.
        미리 돌리기가 실패해도 None(예비 마스크) — 실제 호출에서도 실패할 것이라."""
        path = self.get_parameter('yolo_model').value
        if not path:
            self.get_logger().info('find_blocks 마스크: 예비(엣지 + 깊이, E-47) — yolo_model 파라미터가 비었다')
            return None
        try:
            from ultralytics import YOLO      # 로봇 PC 에만 있다 — 없으면 예비로(노드는 뜬다)
            if not Path(path).is_file():
                raise FileNotFoundError(path)
            model = YOLO(path)
            t0 = time.monotonic()
            model(np.zeros((480, 640, 3), np.uint8), imgsz=640, conf=float(self.get_parameter('yolo_conf').value), verbose=False,
                  retina_masks=True)             # _yolo_masks 와 같은 인자로 미리 한 번
            warm_s = time.monotonic() - t0
        except Exception as e:                # ImportError · 파일 없음 · 모델 형식 · 미리 돌리기 실패 — 무엇이든 예비로 간다
            self.get_logger().error('YOLO 모델을 못 씀(%s: %s) — find_blocks 는 예비 마스크(엣지 + 깊이)로 간다' % (type(e).__name__, e))
            return None
        self.get_logger().info('find_blocks 마스크: YOLO seg %s (conf %.2f) · 미리 돌리기 %.2f초' % (
            path, self.get_parameter('yolo_conf').value, warm_s))
        return model

    def _load_bases(self):
        """recipe_dir 의 기본 설계(이름에 CHAIR · DESK, 레시피 두 파일 `_recipe.json` + `_placements.csv`, E-69) → blocks/1 목록. scan_infer 가 family 를 고르는 데 쓴다(nearest_base).
        폴더가 비었거나 못 읽은 설계는 빼고 경고 — 하나도 없으면 스캔 family 는 unknown."""
        recipe_dir = self.get_parameter('recipe_dir').value
        if not recipe_dir:
            self.get_logger().warn('recipe_dir 이 비었다 — scan_infer 의 family 는 unknown 으로 답한다')
            return []
        bases = []
        for r_path in sorted(Path(recipe_dir).glob('*_recipe.json')):
            design_id = r_path.name[:-len('_recipe.json')]
            if family_of(design_id) == 'unknown':
                continue
            try:
                pair = load_recipe_files(recipe_dir, design_id)
                if pair is None:                  # 조립 방법 파일이 없다 — 후보에서 뺀다
                    continue
                bases.append(structure_to_blocks(*pair, design_id, family_of(design_id), 0))
            except (OSError, KeyError, TypeError, ValueError, IndexError) as e:
                self.get_logger().warn('기본 설계 %s 를 blocks/1 로 못 바꿈: %s — 스캔 family 후보에서 뺀다' % (design_id, e))
        self.get_logger().info('스캔 family 후보(기본 설계): %s' % [b['design_id'] for b in bases])
        return bases

    def checker_for(self, design_id, block_ids):
        """설계 이름 → (그 설계의 BlockChecker 또는 None, 실패 코드). design_source 에 따라 원격(기본) 또는 로컬 파일 하나만 쓴다.

        입력: design_id · block_ids(이번 요청의 블록 이름 — 캐시한 설계의 블록을 전부 담고 있으면 새 판의 시작 확인으로 보고 다시 읽는다, E-55 ①).
        출력: 성공 (BlockChecker, '') · 실패 (None, 'ERROR' 또는 'TIMEOUT') — 부르는 쪽이 그 코드로 check_progress 를 답한다.
        바깥 영향: 로그, remote 면 get_design 조회(설계마다 · 판마다 한 번). 캐시 열쇠는 (design_source, design_id) —
        돌리는 중에 design_source 를 바꿔도 다른 길로 읽은 설계를 쓰지 않는다. 다시 읽기가 실패하면 옛 캐시도 버린다(옛 설계로 답하지 않는다).
        - remote: _remote_checker. 실패는 캐시하지 않는다(웹 · 다리가 늦게 떠도 다음 요청에 다시 받는다).
        - local: `<recipe_dir>/<design_id>_recipe.json`(구조 — load_recipe 가 옆 `_placements.csv` 도 붙임)
          → recipe_blocks(robot.yaml, 레시피) → BlockChecker. block_id 는 조립 방법의 block_id 칸('<model_id>_<블록 이름>').
          파일을 못 찾으면(이름 비었음 · 경로 문자 · 파일 없음) 캐시하지 않아 다음 요청에 다시 찾고, 찾은 파일을 못 읽으면
          (조립 방법 파일 없음 · 형식 틀림(옛 cad_* 포함) · recipe_sha256 다름 · 블록 크기 다름) None 을 캐시한다(같은 오류를 매번 읽지 않게)."""
        if not design_id:                     # design_id 칸이 비었다 — 블록 이름에서 설계를 잘라 내지 않는다(E-52 ④). 로그는 on_check 가 남긴다
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
        """/d2/hmi/get_design 으로 design/2.0 을 받아 BlockChecker 를 만든다. 출력: (BlockChecker, '') 또는 (None, 코드).

        요청 {"design_id"} → 최대 timeout.service_s 기다림(서비스 콜백 안이라 _call — 멀티스레드 실행기 + 재진입 그룹).
        실패 코드는 task_node.get_design 과 맞춘다: 서버 없음(지금 안 보임) → ERROR · 시간 초과(요청을 치움) → TIMEOUT ·
        success=false → ERROR(서버 이유는 로그에만 — 10/7 합의) · JSON · design/2.0 형식(옛 design/1 은 거절) · 짝 해시 · 레시피 내용 오류 → ERROR.
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
        except (AttributeError, IndexError, KeyError, TypeError, ValueError) as e:   # JSON 깨짐(ValueError) · design/2.0 형식 · 레시피 내용 — 바깥 입력이라 넓게 잡아 콜백이 죽지 않게
            self.get_logger().error('get_design 답으로 설계 %s 를 못 읽음: %s → ERROR' % (design_id, e))
            return None, 'ERROR'
        self.get_logger().info('설계 %s (get_design): 블록 %d개' % (design_id, len(chk.blocks)))
        return chk, ''

    # ---------- 카메라 ----------
    def on_depth(self, msg):
        """정렬된 깊이 프레임을 mm float 로 모은다(최근 max(n_frames, scan_frames) 개)."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        if msg.encoding != '16UC1':           # 32FC1 이면 m
            img *= 1000.0
        img[img <= 0] = np.nan                # 깊이 0 = 구멍. 중앙값 · 평균에 섞이지 않게 NaN
        with self.lock:
            self.frames.append((self.now(), img))
            del self.frames[:-self.keep_frames]
            self.last_frame_t = self.frames[-1][0]

    def on_color(self, msg):
        """컬러 프레임 마지막 한 장을 BGR 로 둔다(scan_infer 사진 · find_blocks 입력 · wrist_image 바탕). 깊이는 이 영상에 맞춰 정렬돼 있다."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.color = (self.now(), img, msg.header.frame_id)

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

    def read_tcp(self):
        """지금 rg2_tcp 자세 T_base←rg2_tcp 4x4 (mm). 파라미터 tcp_pose_mm 가 0 이 아니면 그것(시험용), 아니면 TF(최대 SERVICE_WAIT_S).
        못 받으면 None(브링업 · robot_state_publisher 없음). 바깥 영향 없음(/tf 구독뿐)."""
        p = list(self.get_parameter('tcp_pose_mm').value)
        if any(abs(v) > 1e-9 for v in p):
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(p[3:7]).as_matrix()
            T[:3, 3] = p[:3]
            return T
        return self.tcp.matrix_mm(SERVICE_WAIT_S)

    def fresh_frames(self, t_req, n=None):
        """t_req 뒤에 들어온 깊이 프레임이 n 장(기본 n_frames) 모일 때까지(최대 FRESH_WAIT_S) 기다렸다가 최근 n 장을 돌려준다. 부족하면 None."""
        n = self.n_frames if n is None else n
        deadline = t_req + FRESH_WAIT_S
        while True:
            with self.lock:
                fresh = [img for t, img in self.frames if t > t_req]
            if len(fresh) >= n:
                return fresh[-n:]
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
        """check_progress 콜백. 입력 req.design_id(설계 이름 — 빈 값이면 ERROR) · req.block_ids(전체 블록 이름). 출력 res(배열은 모두 요청 길이).

        보정값 확인 → 설계 고르기 · 읽기(checker_for — remote 면 처음 보는 설계 · 시작 확인(블록 전부)만 get_design) → 요청 뒤 새 깊이 프레임 n장 중앙값
        + TF 자세 → 점군 → BlockChecker → 답. 바깥 영향: get_design · 로그.
        실패: 보정값 없음 · design_id 칸이 빔 · 설계를 못 읽음 → ERROR(get_design 시간 초과만 TIMEOUT), 프레임 · TF 자세 없음 → TIMEOUT
        (모두 state unknown, 값 NaN)."""
        ids = list(req.block_ids)
        if self.T_tcp2c is None:              # 보정값이 없으면 설계를 받아도 답할 수 없다 — get_design 을 부르지 않는다
            self.get_logger().error('보정값이 없어 답할 수 없다 → ERROR')
            return self._fill(res, ids, reason='ERROR')
        # 설계 = design_id 칸만(E-52 ④ — 블록 이름에서 잘라 내지 않는다. BACK · BASE · BEAM 에도 '_B' 가 있다).
        # 한 요청은 한 설계라고 본다 — 그 설계에 없는 블록은 BlockChecker 가 그 블록만 unknown 으로 답한다
        design_id = req.design_id
        if not design_id:
            self.get_logger().error('check_progress 요청의 design_id 칸이 비었다(블록 %s …) — 설계를 고를 수 없다 → ERROR' % ids[:1])
            return self._fill(res, ids, reason='ERROR')
        checker, reason = self.checker_for(design_id, ids)
        if checker is None:
            self.get_logger().error('설계 %r 를 못 읽어 답할 수 없다 → %s' % (design_id, reason))
            return self._fill(res, ids, reason=reason)
        t_req = self.now()
        T_b2t = self.read_tcp()                                           # 로봇은 멈춰 있으니 프레임 모으기와 순서는 무관
        if T_b2t is None:
            self.get_logger().error('TF base_link → rg2_tcp 를 못 받음(브링업?) → TIMEOUT')
            return self._fill(res, ids, reason='TIMEOUT')
        frames = self.fresh_frames(t_req)
        if frames is None or self.intr is None:
            self.get_logger().error('요청 뒤 새 깊이 프레임이 %.1f초 안에 %d장 안 모임(카메라 끊김?) → TIMEOUT' % (FRESH_WAIT_S, self.n_frames))
            return self._fill(res, ids, reason='TIMEOUT')

        with warnings.catch_warnings():                                    # 10장 모두 구멍인 화소는 numpy 가 'All-NaN slice' 를 알리지만 결과(NaN → 점에서 빠짐)는 의도한 것
            warnings.simplefilter('ignore', RuntimeWarning)
            depth_m = np.nanmedian(np.stack(frames), axis=0) / 1000.0    # 구멍(NaN)은 빼고 중앙값. 전부 구멍이면 NaN → 점에서 빠짐
        T_b2c = T_b2t @ self.T_tcp2c
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
        self.get_logger().info('check_progress %s %d개 (점 %d, 손끝 z %.0f mm) → %s' % (
            design_id, len(ids), len(pts), T_b2t[2, 3], {r['block_id']: (r['state'], None if np.isnan(r['top_z_m']) else round(r['top_z_m'] * 1000, 1)) for r in results}))
        return res

    # ---------- 스캔 · 흩뿌림 찾기 (JsonQuery) ----------
    @staticmethod
    def _fail(success, code, detail):
        """실패 응답 (success, reason, {"ok":false,"reason","detail"}). success=True 는 '답은 냈지만 결과가 나쁨'(mock_scan 과 같은 두 층)."""
        return success, ('' if success else code), {'ok': False, 'reason': code, 'detail': detail}

    def _answer(self, res, name, out, t0):
        """(success, reason, 응답 dict) → JsonQuery 응답 res. NaN 은 보내지 않는다(allow_nan=False — 걸리면 ERROR).
        바깥 영향: 로그 한 줄(처리 시간 s — task 는 timeout.service_s 만 기다린다)."""
        success, reason, body = out
        try:
            text = json.dumps(body, ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError) as e:
            success, reason, body = self._fail(False, 'ERROR', '응답을 JSON 으로 못 바꿈: %s' % e)
            text = json.dumps(body, ensure_ascii=False)
        res.success, res.reason, res.response_json = success, reason, text
        dt = time.monotonic() - t0
        if success and body.get('ok') is True:
            self.get_logger().info('%s 답 (%.2f초)' % (name, dt))
        elif success:                         # 답은 냈지만 결과가 나쁨(다시 스캔 안내 등) — 노드 고장은 아니다
            self.get_logger().warn('%s 결과 실패 (%.2f초): %s' % (name, dt, text[:300]))
        else:
            self.get_logger().error('%s 실패 (%.2f초): reason=%s %s' % (name, dt, reason, text[:300]))
        return res

    def _snapshot(self, n, need_color):
        """요청 뒤 새 깊이 n 장 평균 + 지금 TF 자세 × 손목 보정 + (need_color 면) 요청 뒤 컬러 한 장. on_check 와 같은 규칙.

        출력: ((깊이 (H, W) mm 평균 · 0 = 없음, 컬러 BGR 또는 None, frame_id, T_base2cam 4x4 m), None) 또는 (None, 실패 응답 셋).
        실패: 보정값 없음 → ERROR · TF 자세 못 받음 · 새 프레임(컬러)이 FRESH_WAIT_S 안에 안 옴 → TIMEOUT. 바깥 영향 없음."""
        if self.T_tcp2c is None:
            return None, self._fail(False, 'ERROR', '손목 보정값이 없다')
        t_req = self.now()
        T_b2t = self.read_tcp()                                           # 로봇은 멈춰 있으니 프레임 모으기와 순서는 무관
        if T_b2t is None:
            return None, self._fail(False, 'TIMEOUT', 'TF base_link → rg2_tcp 를 못 받음(브링업?)')
        frames = self.fresh_frames(t_req, n)
        if frames is None or self.intr is None:
            return None, self._fail(False, 'TIMEOUT', '요청 뒤 새 깊이 프레임이 %.1f초 안에 %d장 안 모임(카메라 끊김?)' % (FRESH_WAIT_S, n))
        with self.lock:
            color = self.color
        if color is not None and color[0] <= t_req:
            color = None                                                  # 요청 앞의 컬러(로봇이 움직이던 때일 수 있다)는 쓰지 않는다
        if need_color and color is None:
            return None, self._fail(False, 'TIMEOUT', '요청 뒤 새 컬러 프레임이 없다(카메라 끊김?)')
        T_b2c = T_b2t @ self.T_tcp2c
        T_b2c[:3, 3] /= 1000.0                                            # 노드 안은 m
        img, frame_id = (color[1], color[2]) if color is not None else (None, '')
        return (mean_depth_mm(frames), img, frame_id, T_b2c), None

    def on_scan_capture(self, req, res):
        """scan_capture 콜백: {"pose_id","run_id"} → 깊이 scan_frames 장 평균 → StructureScanner.add_capture → {"ok":true,"points"}.

        run_id 마다 StructureScanner 하나(최근 MAX_SCAN_RUNS 개만), 같은 pose_id 가 다시 오면 그 자세 촬영을 바꾼다. 컬러 한 장도 둔다.
        실패는 맨 위 '스캔 · 찾기 실패 때'. 바깥 영향: 로그."""
        t0 = time.monotonic()
        try:
            body = parse_request(req.request_json, ('pose_id', 'run_id'))
        except ValueError as e:
            return self._answer(res, 'scan_capture', self._fail(False, 'SCAN_FAILED', str(e)), t0)
        snap, err = self._snapshot(self.scan_frames, need_color=False)
        if snap is None:
            return self._answer(res, 'scan_capture', err, t0)
        depth_mm, color, _, T = snap
        rid, pose_id = body['run_id'], body['pose_id']
        try:
            with self.scan_lock:
                run = self.scans.get(rid)
                if run is None:
                    run = self.scans[rid] = {'scanner': StructureScanner(self.cfg), 'color': None}
                self.scans.move_to_end(rid)
                while len(self.scans) > MAX_SCAN_RUNS:
                    old, _ = self.scans.popitem(last=False)
                    self.get_logger().warn('scan_infer 가 안 온 run_id %s 의 촬영을 버린다(최근 %d판만 기억)' % (old, MAX_SCAN_RUNS))
                sc = run['scanner']
                sc.captures = [c for c in sc.captures if c['pose_id'] != pose_id]   # 같은 자세 다시 찍음 → 덮어쓰기
                n = sc.add_capture(pose_id, depth_mm, self.intr, T)
                if color is not None:
                    run['color'] = color
                poses = [c['pose_id'] for c in sc.captures]
        except ValueError as e:
            return self._answer(res, 'scan_capture', self._fail(True, 'SCAN_FAILED', '점군을 못 만듦: %s' % e), t0)
        self.get_logger().info('scan_capture %s %s: 점 %d (모은 자세 %s, 카메라 z %.0f mm)' % (rid, pose_id, n, poses, T[2, 3] * 1000))
        return self._answer(res, 'scan_capture', (True, '', {'ok': True, 'points': int(n)}), t0)

    def _next_scan_number(self):
        """스캔 설계 번호 = scan_dir 에 이미 저장된 blocks.json 수 + 1 (scan_<family>_<번호>, IRD 2장 — 노드를 다시 켜도 안 겹치게)."""
        try:
            return len(list(self.scan_dir.glob('*/blocks.json'))) + 1
        except OSError:
            return 1

    def on_scan_infer(self, req, res):
        """scan_infer 콜백: {"run_id"} → StructureScanner.infer(기본 설계로 family) → 파일 저장 → scan_response.

        저장: <scan_dir>/<run_id>/color.png(마지막 촬영 컬러) · cloud.ply(save_cloud, ≤ 2 MB) · blocks.json(번호 세기 · 기록).
        PLY 저장이 실패하면 cloud_path 를 빼고 ok:true + 경고(IRD 4.2 선택 칸, 10/8 PL). 사진을 못 쓰면 image_path ''.
        추론이 끝난 run_id 는 성공 · 실패 모두 메모리에서 지운다(다시 스캔 = 새 run_id). 바깥 영향: 파일 쓰기 · 로그."""
        t0 = time.monotonic()
        try:
            rid = parse_request(req.request_json, ('run_id',))['run_id']
        except ValueError as e:
            return self._answer(res, 'scan_infer', self._fail(False, 'SCAN_FAILED', str(e)), t0)
        with self.scan_lock:
            run = self.scans.pop(rid, None)
        if run is None:
            return self._answer(res, 'scan_infer', self._fail(True, 'SCAN_FAILED', 'run_id %s 로 촬영한 자세가 없다(scan_capture 먼저)' % rid), t0)
        sc = run['scanner']
        try:
            result = sc.infer(bases=self.bases or None)
        except Exception as e:                # 계산 예외로 노드가 죽지 않게 — 원인은 로그로
            self.get_logger().error('scan_infer %s 계산 예외: %r' % (rid, e))
            return self._answer(res, 'scan_infer', self._fail(False, 'ERROR', '추론 계산 예외: %s' % e), t0)
        conf = result.get('confidence', {})
        self.get_logger().info('scan_infer %s: ok=%s %s · 블록 %d (추정 %d) · 설명률 %s · 가까운 기본 설계 %s' % (
            rid, result['ok'], result['reason'], len(result['blocks']['blocks']), result['inferred_count'],
            conf.get('explained'), result.get('nearest_base')))
        if not result['ok']:
            return self._answer(res, 'scan_infer', scan_response(result, '', None), t0)
        family = result['blocks'].get('family') or 'unknown'
        result['blocks']['family'] = family
        result['blocks']['design_id'] = 'scan_%s_%02d' % (family, self._next_scan_number())
        out_dir = self.scan_dir / rid
        image_path, cloud_path = '', None
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
            if run['color'] is not None and cv2.imwrite(str(out_dir / 'color.png'), run['color']):
                image_path = str(out_dir / 'color.png')
            else:
                self.get_logger().warn('scan_infer %s: 촬영 때 컬러가 없어(또는 못 써서) image_path 를 비운다' % rid)
        except OSError as e:
            self.get_logger().warn('scan_infer %s: 폴더 · 사진을 못 씀 %s — image_path 를 비운다' % (rid, e))
        try:
            # IRD E-67: PLY 가 없으면 화면 점군 창만 빈다 — 블록 결과는 그대로 넘긴다(cloud_path 칸 뺌). cloud_path 는 선택 칸(IRD 4.2, 10/8 PL)
            n_pts = sc.save_cloud(str(out_dir / 'cloud.ply'))
            cloud_path = str(out_dir / 'cloud.ply')
            self.get_logger().info('scan_infer %s: 점군 %d점 · %.0f KB (복셀 %.1f mm)' % (
                rid, n_pts, (out_dir / 'cloud.ply').stat().st_size / 1024, sc.cloud_voxel_m * 1000))
        except (OSError, ValueError) as e:
            self.get_logger().warn('scan_infer %s: 점군 PLY 를 못 씀 %s — cloud_path 를 빼고 답한다(선택 칸 — 화면 점군 창만 빈다)' % (rid, e))
        out = scan_response(result, image_path, cloud_path)
        try:
            (out_dir / 'blocks.json').write_text(json.dumps(out[2]['blocks'], ensure_ascii=False, indent=1), encoding='utf-8')
        except OSError as e:
            self.get_logger().warn('scan_infer %s: blocks.json 을 못 씀 %s (응답은 그대로)' % (rid, e))
        return self._answer(res, 'scan_infer', out, t0)

    def _yolo_masks(self, color):
        """YOLO seg 로 블록 마스크 · 신뢰도. 모델이 없거나 추론 예외면 (None, None) → 예비 마스크(엣지 + 깊이)."""
        if self.yolo is None:
            return None, None
        try:
            r = self.yolo(color, imgsz=640, conf=float(self.get_parameter('yolo_conf').value), verbose=False,
                          retina_masks=True)[0]                          # retina_masks: 마스크를 원본 크기(640×480)로 받는다
            return masks_from_yolo(r, color.shape[:2])
        except Exception as e:                # 추론이 실패해도 찾기는 예비로 계속한다
            self.get_logger().error('YOLO 추론 실패(%r) — 이번 find_blocks 는 예비 마스크로' % (e,))
            return None, None

    def publish_wrist_image(self, color, blocks, frame_id):
        """find_blocks 검출 결과를 그린 그림 한 장을 `/d2/vision/wrist_image`(JPEG 70, 640×480 그대로)로 낸다. 실패는 경고만."""
        try:
            img = draw_found(color, blocks, self.finder.last_masks, self.finder.last_info.get('mask_index', []))
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ok:
                raise ValueError('JPEG 인코딩 실패')
        except (ValueError, IndexError, cv2.error) as e:
            self.get_logger().warn('wrist_image 를 못 만듦: %s' % e)
            return
        msg = CompressedImage(format='jpeg', data=buf.tobytes())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        self.pub_image.publish(msg)

    def on_find_blocks(self, req, res):
        """find_blocks 콜백: {"run_id"} → 요청 뒤 컬러 · 깊이(scan_frames 장 평균) → BlockFinder → {"ok":true,"blocks":[…]} + wrist_image 한 장.

        블록 0개도 ok true(빈 목록 → 작업 관리자 WAIT_SUPPLY). 계산 예외 → success=false ERROR. 나머지 실패는 맨 위.
        바깥 영향: wrist_image 발행 · 로그. run_id 는 기록용(로그)."""
        t0 = time.monotonic()
        try:
            rid = parse_request(req.request_json, ('run_id',))['run_id']
        except ValueError as e:
            return self._answer(res, 'find_blocks', self._fail(False, 'SCAN_FAILED', str(e)), t0)
        snap, err = self._snapshot(self.scan_frames, need_color=True)
        if snap is None:
            return self._answer(res, 'find_blocks', err, t0)
        depth_mm, color, frame_id, T = snap
        try:
            with self.find_lock:
                masks, scores = self._yolo_masks(color)
                blocks = self.finder.find(color, depth_mm / 1000.0, self.intr, T, masks=masks, scores=scores)
                self.publish_wrist_image(color, blocks, frame_id)
                info = self.finder.last_info
        except Exception as e:                # 계산 예외로 노드가 죽지 않게 — 원인은 로그로
            self.get_logger().error('find_blocks %s 계산 예외: %r' % (rid, e))
            return self._answer(res, 'find_blocks', self._fail(False, 'ERROR', '찾기 계산 예외: %s' % e), t0)
        self.get_logger().info('find_blocks %s (%s): 블록 %d · 작업면 %.1f mm · 뺀 마스크 %d → %s' % (
            rid, 'YOLO' if masks is not None else '예비', len(blocks), info.get('table_z_m', NAN) * 1000, len(info.get('skipped', [])),
            [(b['up'][0], b['yaw_deg'], b['overlap'], b['confidence']) for b in blocks]))
        return self._answer(res, 'find_blocks', (True, '', {'ok': True, 'blocks': blocks}), t0)


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

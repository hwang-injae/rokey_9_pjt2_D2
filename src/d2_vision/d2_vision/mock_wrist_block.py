"""가짜 손목 블록 인식 (mock_wrist_block) — IRD 4.2 `/d2/vision/check_progress` · `scan_capture` · `scan_infer` · `find_blocks`
· 4.1 `camera_status/1` (W034 · W140 E-52 → E-69).

카메라·로봇 없이 작업 관리자(task)가 진행 확인 · 스캔 · 흩뿌림 찾기 흐름을 돌릴 수 있게 한다.
- `/d2/vision/check_progress`(CheckProgress): 요청한 block_id 마다 state 와 높이를 답한다.
  설계는 요청 design_id 칸으로만 고른다(E-52 ④ — 블록 이름에서 잘라 내지 않는다). 비었으면 success=false, reason ERROR(wrist_block 과 같음).
  block_ids 는 전체 블록 이름(예 001_CHAIR_BENCH_LEG_001_01)을 그대로 맞춘다.
  설계 읽기는 wrist_block 과 같다(**task · wrist_block 과 같은 design_source 값을 준다**, W157):
    remote(기본)  `/d2/hmi/get_design`(JsonQuery, 요청 {"design_id"} → 응답 design/2.0)으로 받는다 — AI 생성 · 스캔 설계(DB에만 있음)도 같은 길.
                  design/2.0 → block_checker.recipe_from_design → motion_math.recipe_blocks 로 블록 이름 · 윗면 높이를 만든다.
    local         `<recipe_dir>/<design_id>_recipe.json`(구조, E-69 — 옆 `_placements.csv` 도)만(웹 없이 개발할 때 명시).
  캐시는 한 판(run) 안에서만(design_cache.RunDesignCache — wrist_block 과 같은 규칙): 요청 run_id 가 지난 요청과 다르면 설계를 다시 읽고,
  같으면(블록을 전부 묻든 몇 개 묻든) 읽어 둔 설계를 쓴다. 빈 run_id 는 판 구분 없음(CheckProgress.srv) — 빈 값끼리는 캐시, 이름 있는 판과 오가면 다시 읽는다.
  기본은 전부 present. 파라미터로 absent·occluded 블록을 고를 수 있다.
  top_z_m = 팀 공용 `d2_motion.motion_math.recipe_blocks()` 가 계산한 블록 윗면 높이(base, m — 실측 두께로 쌓은 값). dz_m = 0.
  dx_m·dy_m 은 1차 규칙대로 늘 NaN(안 잼). absent·unknown 블록은 dz·top_z 도 NaN.
  CheckProgress.srv NaN 규칙과 다른 점(가짜라서 — 그대로 둔다, W140 질문): occluded 도 top_z NaN(가린 물체가 없어 윗면 값이 없다) ·
  '위 블록에 가려진 present' 를 구분하지 않고 present 는 모두 윗면 값(가짜는 아직 안 놓은 블록도 present 라 가려짐을 따지면 거의 다 NaN 이 된다).
- `/d2/vision/camera_status`(camera_status/1, 2 Hz): 카메라가 살아 있다는 신호. 정지 노드 연결은 챌린지 공통.
- `/d2/vision/scan_capture`(JsonQuery): {"pose_id","run_id"} → {"ok":true,"points"}. run_id 별로 촬영한 자세를 기억한다.
- `/d2/vision/scan_infer`(JsonQuery): {"run_id"} → {"ok":true,"blocks":blocks/1,"inferred_count","image_path","cloud_path"}.
  blocks = 파라미터 scan_design_id 레시피 두 파일(recipe_dir)을 '스캔했다'고 흉내(설계 이름 scan_<scan_family>_<번호>,
  앞 순서 scan_inferred 개는 inferred: true). image_path · cloud_path 는 실제 파일(<임시 폴더>/d2_scan/<run_id>/color.png · cloud.ply,
  PLY 는 base_link m) — 다리가 읽어 보낼 수 있다. 그 run_id 로 촬영(scan_capture)을 안 했으면 ok:false SCAN_FAILED.
- `/d2/vision/find_blocks`(JsonQuery): {"run_id"} → {"ok":true,"blocks":[…]}(IRD 4.2 칸). 파라미터 find_blocks_json 이 비면
  IRD 예시 블록 하나, "[]" 면 빈 목록(WAIT_SUPPLY 시험). clear 는 gap_mm ≥ robot.yaml find.min_gap_mm 으로 다시 계산한다.
  세 서비스의 계산(응답 만들기 · 파일 쓰기)은 mock_scan.MockScan(ROS 없음, pytest 로 시험). 잘못된 요청(JSON 아님 · run_id 없음)은
  success=false + 응답 {"ok":false,"reason","detail"}(reason: 요청 오류 SCAN_FAILED, find_blocks_json 파라미터 오류 ERROR).

실행 (저장소 맨 위에서. **task 와 같은 design_source 값을 준다**. recipe_dir 은 scan_infer 가 흉내 낼 설계 파일을 읽는 데도 쓴다):
  웹 · 다리(또는 mock_bridge)와 함께(기본 remote — task 도 기본 remote):
    ros2 run d2_vision mock_wrist_block --ros-args -p recipe_dir:=src/recipe_manager/recipes
  웹 없이 파일로(task 도 -p design_source:=local -p recipe_dir:=… 로 띄운다):
    ros2 run d2_vision mock_wrist_block --ros-args -p design_source:=local -p recipe_dir:=src/recipe_manager/recipes
돌리는 중에 바꾸기 (다음 요청부터 반영, 전체 블록 이름으로):
  ros2 param set /mock_wrist_block absent "001_CHAIR_BENCH_LEG_001_04,001_CHAIR_BENCH_LEG_002_04"   # 이 블록은 없음
  ros2 param set /mock_wrist_block occluded "001_CHAIR_BENCH_SEAT_001_01"                          # 이 블록은 가려짐
  ros2 param set /mock_wrist_block fail true                                                       # success=false, reason ERROR (카메라 고장 흉내)
  ros2 param set /mock_wrist_block scan_design_id 002_CHAIR_BACK                                   # 스캔이 이 설계를 본 것처럼
  ros2 param set /mock_wrist_block scan_fail true                                                  # scan_infer ok:false SCAN_FAILED
  ros2 param set /mock_wrist_block find_blocks_json "'[]'"                                         # 공급 빔 → 작업 관리자 WAIT_SUPPLY
시험 호출 (design_id 칸 + 전체 블록 이름):
  ros2 service call /d2/vision/check_progress d2_interfaces/srv/CheckProgress "{design_id: 001_CHAIR_BENCH, block_ids: [001_CHAIR_BENCH_LEG_001_01, 001_CHAIR_BENCH_SEAT_001_03]}"
  ros2 service call /d2/vision/scan_capture d2_interfaces/srv/JsonQuery "{request_json: '{\\"pose_id\\": \\"observe_front\\", \\"run_id\\": \\"R1\\"}'}"
  ros2 service call /d2/vision/scan_infer d2_interfaces/srv/JsonQuery "{request_json: '{\\"run_id\\": \\"R1\\"}'}"
  ros2 service call /d2/vision/find_blocks d2_interfaces/srv/JsonQuery "{request_json: '{\\"run_id\\": \\"R1\\"}'}"

바깥 영향: 서비스 답 · camera_status 발행 · 로그 · remote 면 get_design 요청(설계마다 · 판마다 한 번) · scan_infer 때 임시 폴더에 PNG · PLY 쓰기.
  로봇·카메라에 아무것도 보내지 않는다.
실패 때: design_id 칸이 비면 success=false, reason ERROR(블록 모두 unknown). 설계를 못 읽으면(get_design 서버 없음 · success=false · 형식 오류 ·
  local 파일 없음) success=false, reason ERROR(get_design 시간 초과는 TIMEOUT)에 블록 모두 unknown, 높이 NaN — wrist_block 과 같다.
  로컬 파일로 몰래 대신하지 않고, 다시 읽다 실패하면 지난 판의 설계로도 답하지 않는다(노드는 안 죽는다). 설계에 없는 블록은 그 블록만 unknown.
  스캔 · 찾기 실패는 위 각 서비스 설명.
"""
import json
import threading
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from d2_interfaces.srv import CheckProgress, JsonQuery
from d2_motion.motion_math import RECIPE_SUFFIXES, half_height, load_recipe, recipe_blocks
from d2_vision.block_checker import recipe_from_design, recipe_path
from d2_vision.design_cache import RunDesignCache
from d2_vision.mock_scan import MockScan

NAN = float('nan')
QOS_STATUS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def recipe_tops(cfg, recipe):
    """레시피 dict(motion_math.load_recipe 모양, mm) → {block_id: 윗면 높이 m(base)}. 바깥 영향 없음.

    recipe_blocks 가 읽고 실측 두께(robot.yaml block_actual_m)로 쌓는다. block_id 는 조립 방법의 block_id 칸('<model_id>_<블록 이름>').
    로컬 파일(load_recipe)과 get_design 답(block_checker.recipe_from_design)이 같은 모양이라 두 길이 같은 함수를 쓴다.
    실패: recipe_blocks 의 예외(KeyError · TypeError · ValueError · 조립 방법 파일 없음 · 형식 틀림(옛 cad_* 포함) · recipe_sha256 다름)를 그대로 낸다."""
    return {b['block_id']: b['center'][2] + half_height(b['rot'], cfg['block_actual_m']) for b in recipe_blocks(cfg, recipe)}


class MockWristBlock(Node):
    """check_progress · scan_capture · scan_infer · find_blocks 에 답하고 camera_status 를 내는 가짜 노드. 로봇·카메라에 아무것도 하지 않는다."""

    def __init__(self):
        """파라미터 선언 → 서비스·토픽·타이머·get_design 클라이언트 만들기. 설계는 요청 때 읽는다 — 못 읽어도 노드는 뜬다(경고만)."""
        super().__init__('mock_wrist_block')
        self.declare_parameter('recipe_dir', '')         # 레시피 폴더(task 와 같음). local 설계 읽기 · scan_infer 가 흉내 낼 설계 파일용
        self.declare_parameter('design_source', 'remote')  # task · wrist_block 과 같은 값: remote = /d2/hmi/get_design(기본) · local = recipe_dir 파일
        self.declare_parameter('absent', '')             # 없음으로 답할 block_id, 쉼표
        self.declare_parameter('occluded', '')           # 가려짐으로 답할 block_id, 쉼표
        self.declare_parameter('fail', False)            # True 면 success=false (카메라 고장 흉내)
        self.declare_parameter('status_hz', 2.0)         # camera_status 주기 (IRD 2 Hz)
        self.declare_parameter('scan_design_id', '001_CHAIR_BENCH')   # scan_infer 가 '스캔했다'고 흉내 낼 설계(recipe_dir 의 두 파일)
        self.declare_parameter('scan_family', 'chair')   # 스캔 설계 family(blocks/1 칸 · 이름 scan_<family>_<번호>)
        self.declare_parameter('scan_inferred', 2)       # inferred: true 로 표시할 블록 수(앞 순서부터 — 가려진 아래층 흉내)
        self.declare_parameter('scan_fail', False)       # True 면 scan_infer 가 ok:false SCAN_FAILED (격자 맞추기 실패 흉내)
        self.declare_parameter('find_blocks_json', '')   # find_blocks 블록 목록 JSON. 빈 값 = IRD 예시 블록 하나, "[]" = 공급 빔

        p = Path(get_package_share_directory('d2_bringup')) / 'config' / 'robot.yaml'
        self.cfg = yaml.safe_load(p.read_text(encoding='utf-8'))
        self.service_s = float(self.cfg['timeout']['service_s'])   # get_design 한 번의 제한 시간(wrist_block · task 와 같은 값)
        self.designs = RunDesignCache()                  # (design_source, design_id) → {block_id: 윗면 z m} — 설계마다 · 판(run_id)마다 한 번만 읽는다
        if self.get_parameter('design_source').value == 'local':
            self.get_logger().info('설계 읽기: local — recipe_dir 파일만 (task · wrist_block 도 design_source:=local 이어야 한다)')
            if not self.get_parameter('recipe_dir').value:
                self.get_logger().warn('recipe_dir 가 비었다 — check_progress 는 ERROR 로 답한다')
        else:
            self.get_logger().info('설계 읽기: remote — /d2/hmi/get_design (제한 %.1f초, task · wrist_block 도 remote 여야 한다)' % self.service_s)

        self.scan = MockScan(self.cfg['assembly_origin'], [v * 1000.0 for v in self.cfg['block_size_m']],
                             self.cfg['find']['min_gap_mm'])
        # 서비스 콜백 안에서 get_design 을 기다리려면 멀티스레드 실행기가 필요하고, 응답(client)은 서비스와 다른 그룹이어야 한다.
        # 네 서비스는 한 직렬 그룹에 둔다 — 요청이 겹치지 않아 ① 설계 캐시(RunDesignCache)를 같은 판에서 두 번 읽거나 느린 지난 판의 답이 새 판 캐시를 덮지 않고
        # ② MockScan 의 공유 상태(run 별 촬영 · 번호)를 동시에 건드리지 않는다(옛 단일 스레드 실행기와 같은 직렬 동작). 대기 중인 요청은 스레드를 잡지 않는다.
        services = MutuallyExclusiveCallbackGroup()
        group = ReentrantCallbackGroup()                 # get_design 응답 · camera_status 타이머
        self.cli_design = self.create_client(JsonQuery, '/d2/hmi/get_design', callback_group=group)
        self.srv = self.create_service(CheckProgress, '/d2/vision/check_progress', self.on_check, callback_group=services)
        self.create_service(JsonQuery, '/d2/vision/scan_capture', self.on_scan_capture, callback_group=services)
        self.create_service(JsonQuery, '/d2/vision/scan_infer', self.on_scan_infer, callback_group=services)
        self.create_service(JsonQuery, '/d2/vision/find_blocks', self.on_find_blocks, callback_group=services)
        self.pub_status = self.create_publisher(String, '/d2/vision/camera_status', QOS_STATUS)
        self.create_timer(1.0 / self.get_parameter('status_hz').value, self.publish_status, callback_group=group)
        self.get_logger().info('mock_wrist_block 시작: /d2/vision/check_progress · scan_capture · scan_infer · find_blocks 대기')

    def now(self):
        """노드 시계의 지금 시각(초, float). JSON stamp 칸에 쓴다."""
        return self.get_clock().now().nanoseconds / 1e9

    def _ids(self, name):
        """쉼표로 적힌 문자열 파라미터(name) → block_id 집합. 빈 문자열이면 빈 집합."""
        raw = self.get_parameter(name).value
        return {s.strip() for s in raw.split(',') if s.strip()}

    def _call(self, cli, req, wait_s):
        """서비스를 부르고 답을 기다린다(멀티스레드 실행기 + 재진입 그룹이라 콜백 중에도 됨). 서버 기다림 wait_s + 답 기다림 wait_s 초만큼 막힌다.
        시간 안에 못 받으면 보낸 요청을 치우고 None. wrist_block._call 과 같다."""
        if not cli.wait_for_service(timeout_sec=wait_s):
            return None
        done = threading.Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(wait_s):
            cli.remove_pending_request(fut)
            return None
        return fut.result()

    def tops_for(self, design_id, run_id):
        """설계 이름 → ({block_id: 윗면 z m} 또는 None, 실패 코드). design_source 에 따라 원격(기본) 또는 로컬 파일 하나만 쓴다.

        같은 판(run_id)에서는 읽어 둔 설계를 쓰고, run_id 가 지난 요청과 다르면 다시 읽는다(빈 값 = 판 구분 없음 — RunDesignCache). 실패는 캐시하지 않는다.
        출력: 성공 (tops, '') · 실패 (None, 'ERROR' 또는 'TIMEOUT'). 다시 읽다 실패해도 지난 판의 설계로 답하지 않는다.
        바깥 영향: 로그, remote 면 get_design 요청 하나(설계마다 · 판마다)."""
        source = 'local' if self.get_parameter('design_source').value == 'local' else 'remote'
        load = self._remote_tops if source == 'remote' else self._local_tops
        return self.designs.get((source, design_id), run_id, lambda: load(design_id))

    def _local_tops(self, design_id):
        """design_source:=local — `<recipe_dir>/<design_id>_recipe.json`(옆 `_placements.csv` 도) → 윗면 높이. 출력: (tops, '') 또는 (None, 'ERROR').
        실패: 파일 없음 · 이름 문자 이상 · 조립 방법 파일 없음 · 형식 틀림(옛 cad_* 포함) · recipe_sha256 다름. 바깥 영향: 파일 읽기 · 로그."""
        recipe_dir = self.get_parameter('recipe_dir').value
        path = recipe_path(recipe_dir, design_id, RECIPE_SUFFIXES)
        if path is None:
            self.get_logger().error('레시피 %r 파일을 못 찾음(recipe_dir=%r, 끝 %s)' % (design_id, recipe_dir, ' · '.join(RECIPE_SUFFIXES)))
            return None, 'ERROR'
        try:
            tops = recipe_tops(self.cfg, load_recipe(str(path)))
        except (OSError, KeyError, TypeError, ValueError) as e:
            self.get_logger().error('레시피 %s 를 못 읽음(%s): %s' % (design_id, path, e))
            return None, 'ERROR'
        self.get_logger().info('레시피 %s (%s): 블록 %d개' % (design_id, path.name, len(tops)))
        return tops, ''

    def _remote_tops(self, design_id):
        """/d2/hmi/get_design 으로 design/2.0 을 받아 윗면 높이를 만든다. 출력: (tops, '') 또는 (None, 코드) — wrist_block._remote_checker 와 같은 코드:
        서버 없음 → ERROR · timeout.service_s 안에 답 없음 → TIMEOUT · success=false → ERROR · JSON · design/2.0 형식(옛 design/1 은 거절) · 짝 해시 ·
        레시피 내용 오류 → ERROR. 로컬 파일로 몰래 대신하지 않는다. 바깥 영향: get_design 요청 하나 · 로그."""
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
            tops = recipe_tops(self.cfg, recipe_from_design(json.loads(res.response_json), design_id))
        except (AttributeError, IndexError, KeyError, TypeError, ValueError) as e:   # JSON 깨짐(ValueError) · design/2.0 형식 · 레시피 내용 — 바깥 입력이라 넓게 잡는다
            self.get_logger().error('get_design 답으로 설계 %s 를 못 읽음: %s → ERROR' % (design_id, e))
            return None, 'ERROR'
        self.get_logger().info('설계 %s (get_design): 블록 %d개' % (design_id, len(tops)))
        return tops, ''

    def on_check(self, req, res):
        """check_progress 콜백. 입력 req.design_id(빈 값 가능) · req.run_id(조립 한 판의 ID — 바뀌면 설계를 다시 읽음, 빈 값 = 판 구분 없음) ·
        req.block_ids(전체 블록 이름). 출력 res — 배열 길이는 모두 요청과 같다(IRD 5장). 블록마다 state·높이를 채운다.
        바깥 영향: 로그, remote 면 get_design 요청(새 판 · 처음 보는 설계일 때만).
        실패: design_id 칸이 비면 · 설계를 못 읽으면(tops_for 의 ERROR · TIMEOUT) · 실패 흉내(fail 파라미터)면 success=false —
        앞의 둘은 블록이 모두 unknown, NaN(wrist_block 과 같음)."""
        absent, occluded = self._ids('absent'), self._ids('occluded')
        res.block_ids = list(req.block_ids)
        res.states, res.dx_m, res.dy_m, res.dz_m, res.top_z_m = [], [], [], [], []
        design_id = req.design_id                        # design_id 칸만(E-52 ④ — 블록 이름에서 잘라 내지 않는다)
        tops, reason = None, 'ERROR'
        if not design_id:
            self.get_logger().error('check_progress 요청의 design_id 칸이 비었다 — 설계를 고를 수 없다 → ERROR')
        else:
            tops, reason = self.tops_for(design_id, req.run_id)
            if tops is None:
                self.get_logger().error('설계 %r 를 못 읽어 답할 수 없다 → %s' % (design_id, reason))
        for bid in req.block_ids:
            if tops is None:
                state, top = 'unknown', NAN            # 설계를 모르면 모든 블록을 모른다 — 지난 판의 설계로 대신 답하지 않는다
            elif bid in absent:
                state, top = 'absent', NAN
            elif bid in occluded:
                state, top = 'occluded', NAN           # 가린 물체가 없는 가짜라 윗면 값이 없다(srv 는 참고값 — 모듈 설명 '다른 점')
            elif bid in tops:
                state, top = 'present', tops[bid]
            else:
                state, top = 'unknown', NAN            # 레시피에 없는 블록 — 작업 판단이 UNKNOWN_BLOCK 으로 다룬다
            res.states.append(state)
            res.dx_m.append(NAN)                       # 1차: dx·dy 는 안 잼
            res.dy_m.append(NAN)
            res.dz_m.append(0.0 if state == 'present' else NAN)
            res.top_z_m.append(top)
        if self.get_parameter('fail').value:
            res.success, res.reason = False, 'ERROR'
        elif tops is None:
            res.success, res.reason = False, reason
        else:
            res.success, res.reason = True, ''
        self.get_logger().info('check_progress (design_id 칸 %r, run_id %r) %d개 → %s' % (
            req.design_id, req.run_id, len(req.block_ids), dict(zip(res.block_ids, res.states))))
        return res

    def _answer(self, name, res, out):
        """MockScan 결과 (success, reason, 응답 dict) → JsonQuery 응답 res. 바깥 영향: 로그(실패는 경고)."""
        res.success, res.reason = out[0], out[1]
        res.response_json = json.dumps(out[2], ensure_ascii=False, allow_nan=False)
        if not res.success or out[2].get('ok') is not True:
            self.get_logger().warn('%s 실패 흉내 · 잘못된 요청: %s' % (name, res.response_json))
        return res

    def on_scan_capture(self, req, res):
        """scan_capture 콜백. 입력 request_json {"pose_id","run_id"}. 출력 {"ok","points"}(가짜 점 수). 바깥 영향: 로그."""
        out = self.scan.capture(req.request_json)
        if out[0]:
            self.get_logger().info('scan_capture %s' % req.request_json)
        return self._answer('scan_capture', res, out)

    def on_scan_infer(self, req, res):
        """scan_infer 콜백. 입력 request_json {"run_id"}. 출력 blocks/1 · inferred_count · image_path · cloud_path.
        바깥 영향: 임시 폴더에 color.png · cloud.ply 쓰기 · 로그. 실패는 MockScan.infer 설명(ok:false SCAN_FAILED 등)."""
        out = self.scan.infer(req.request_json, self.get_parameter('recipe_dir').value,
                              self.get_parameter('scan_design_id').value, self.get_parameter('scan_family').value,
                              self.get_parameter('scan_inferred').value, self.get_parameter('scan_fail').value)
        if out[2].get('ok') is True:
            self.get_logger().info('scan_infer %s → %s 블록 %d개 (추정 %d) · %s' % (
                req.request_json, out[2]['blocks']['design_id'], len(out[2]['blocks']['blocks']),
                out[2]['inferred_count'], out[2]['cloud_path']))
        return self._answer('scan_infer', res, out)

    def on_find_blocks(self, req, res):
        """find_blocks 콜백. 입력 request_json {"run_id"}. 출력 {"ok","blocks":[…]}(find_blocks_json 파라미터 또는 IRD 예시). 바깥 영향: 로그."""
        out = self.scan.find(req.request_json, self.get_parameter('find_blocks_json').value)
        if out[0]:
            self.get_logger().info('find_blocks %s → 블록 %d개' % (req.request_json, len(out[2]['blocks'])))
        return self._answer('find_blocks', res, out)

    def publish_status(self):
        """타이머마다 `/d2/vision/camera_status`(camera_status/1)를 낸다. 가짜라 last_frame_stamp = 지금 시각(초)."""
        t = self.now()
        msg = {'schema': 'camera_status/1', 'stamp': t, 'node': 'wrist_block', 'last_frame_stamp': t}
        self.pub_status.publish(String(data=json.dumps(msg)))


def main(args=None):
    """노드를 멀티스레드 실행기로 띄우고 Ctrl+C 까지 돈다(서비스 콜백 안에서 get_design 을 기다리기 위해). 로봇·카메라에 명령을 보내지 않으므로 서기 처리는 없다."""
    rclpy.init(args=args)
    node = MockWristBlock()
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

"""가짜 손목 블록 인식 (mock_wrist_block) — IRD 4.2 `/d2/vision/check_progress` · 4.1 `camera_status/1` (W034 · W140 E-52).

카메라·로봇 없이 작업 관리자(task)가 진행 확인 흐름을 돌릴 수 있게 한다.
- `/d2/vision/check_progress`(CheckProgress): 요청한 block_id 마다 state 와 높이를 답한다.
  설계는 요청 design_id 칸으로 고른다(E-52 — 블록 이름에서 잘라 내지 않는다). 비었으면 옛 방식(10/8 저녁까지만):
  block_id 마다 '_B' 앞(001_CHAIR_BENCH_B003 → 001_CHAIR_BENCH). block_ids 는 전체 블록 이름(예 001_CHAIR_BENCH_LEG_001_01)을 그대로 맞춘다.
  레시피 파일은 wrist_block 과 같다: `<recipe_dir>/<design_id>_recipe.json`(새, 옆 `_structure.json` 도) → 없으면 `.recipe.json`(옛).
  기본은 전부 present. 파라미터로 absent·occluded 블록을 고를 수 있다.
  top_z_m = 팀 공용 `d2_motion.motion_math.recipe_blocks()` 가 계산한 블록 윗면 높이(base, m — 실측 두께로 쌓은 값). dz_m = 0.
  dx_m·dy_m 은 1차 규칙대로 늘 NaN(안 잼). absent·unknown 블록은 dz·top_z 도 NaN.
  CheckProgress.srv NaN 규칙과 다른 점(가짜라서 — 그대로 둔다, W140 질문): occluded 도 top_z NaN(가린 물체가 없어 윗면 값이 없다) ·
  '위 블록에 가려진 present' 를 구분하지 않고 present 는 모두 윗면 값(가짜는 아직 안 놓은 블록도 present 라 가려짐을 따지면 거의 다 NaN 이 된다).
- `/d2/vision/camera_status`(camera_status/1, 2 Hz): 카메라가 살아 있다는 신호. 정지 노드 연결은 챌린지 공통.
- `scan_capture` · `scan_infer` 는 `JsonQuery.srv` 가 main 에 들어온 뒤(W121, 10/7 오전) 더한다.

실행 (저장소 맨 위에서. 레시피 폴더는 task 노드와 같은 파라미터 recipe_dir):
  ros2 run d2_vision mock_wrist_block --ros-args -p recipe_dir:=src/recipe_manager/recipes
돌리는 중에 바꾸기 (다음 요청부터 반영, 전체 블록 이름으로):
  ros2 param set /mock_wrist_block absent "001_CHAIR_BENCH_LEG_001_04,001_CHAIR_BENCH_LEG_002_04"   # 이 블록은 없음
  ros2 param set /mock_wrist_block occluded "001_CHAIR_BENCH_SEAT_001_01"                          # 이 블록은 가려짐
  ros2 param set /mock_wrist_block fail true                                                       # success=false, reason ERROR (카메라 고장 흉내)
시험 호출 (새 — design_id 칸 + 전체 블록 이름):
  ros2 service call /d2/vision/check_progress d2_interfaces/srv/CheckProgress "{design_id: 001_CHAIR_BENCH, block_ids: [001_CHAIR_BENCH_LEG_001_01, 001_CHAIR_BENCH_SEAT_001_03]}"
시험 호출 (옛 — design_id 빈 값, 레시피가 아직 옛 이름일 때. 10/8 저녁까지):
  ros2 service call /d2/vision/check_progress d2_interfaces/srv/CheckProgress "{block_ids: [001_CHAIR_BENCH_B001, 001_CHAIR_BENCH_B011]}"

바깥 영향: 서비스 답 · camera_status 발행 · 로그뿐. 로봇·카메라에 아무것도 보내지 않는다.
실패 때: 설계를 못 고르거나 레시피 파일이 없거나 블록을 못 찾으면 그 블록은 state unknown, 높이 NaN 으로 답한다(노드는 안 죽는다).
"""
import json
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from d2_interfaces.srv import CheckProgress
from d2_motion.motion_math import RECIPE_SUFFIXES, half_height, load_recipe, recipe_blocks
from d2_vision.block_checker import design_of, recipe_path

NAN = float('nan')
QOS_STATUS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def load_recipe_tops(recipe_dir, design_id, cfg):
    """설계 이름 → {block_id: 윗면 높이 m(base)}. 바깥 영향 없음(파일 읽기만).

    파일은 wrist_block 과 같은 규칙(recipe_path): `<recipe_dir>/<design_id>_recipe.json`(새 — load_recipe 가 옆 `_structure.json` 도 붙임)
    → 없으면 `<design_id>.recipe.json`(옛). recipe_blocks 가 형식을 가려 읽고 실측 두께(robot.yaml block_actual_m)로 쌓는다.
    block_id 는 recipe_blocks 가 만든 이름(새 형식 '<model_id>_<블록 이름>', 옛 '<model_id>_B<순서>').
    실패(이름 비었음 · 경로 문자 · 파일 없음 · 구조 파일 없음 · 형식 틀림) → 빈 dict(요청 블록은 unknown)."""
    path = recipe_path(recipe_dir, design_id, RECIPE_SUFFIXES)
    if path is None:
        return {}
    try:
        return {b['block_id']: b['center'][2] + half_height(b['rot'], cfg['block_actual_m'])
                for b in recipe_blocks(cfg, load_recipe(str(path)))}
    except (OSError, KeyError, TypeError, ValueError):
        return {}


class MockWristBlock(Node):
    """check_progress 에 답하고 camera_status 를 내는 가짜 노드. 로봇·카메라에 아무것도 하지 않는다."""

    def __init__(self):
        """파라미터 선언 → 레시피 읽기 → 서비스·토픽·타이머 만들기. 레시피를 못 읽어도 노드는 뜬다(경고만)."""
        super().__init__('mock_wrist_block')
        self.declare_parameter('recipe_dir', '')         # 레시피 폴더(task 와 같음). 없으면 높이는 NaN · state unknown
        self.declare_parameter('absent', '')             # 없음으로 답할 block_id, 쉼표
        self.declare_parameter('occluded', '')           # 가려짐으로 답할 block_id, 쉼표
        self.declare_parameter('fail', False)            # True 면 success=false (카메라 고장 흉내)
        self.declare_parameter('status_hz', 2.0)         # camera_status 주기 (IRD 2 Hz)

        p = Path(get_package_share_directory('d2_bringup')) / 'config' / 'robot.yaml'
        self.cfg = yaml.safe_load(p.read_text(encoding='utf-8'))
        self.tops = {}                                   # design_id → {block_id: 윗면 z m}. 요청 때 설계마다 한 번 읽는다
        if not self.get_parameter('recipe_dir').value:
            self.get_logger().warn('recipe_dir 가 비었다 — 높이는 전부 NaN, state 는 unknown 으로 답한다')

        self.srv = self.create_service(CheckProgress, '/d2/vision/check_progress', self.on_check)
        self.pub_status = self.create_publisher(String, '/d2/vision/camera_status', QOS_STATUS)
        self.create_timer(1.0 / self.get_parameter('status_hz').value, self.publish_status)
        self.get_logger().info('mock_wrist_block 시작: /d2/vision/check_progress 대기')

    def now(self):
        """노드 시계의 지금 시각(초, float). JSON stamp 칸에 쓴다."""
        return self.get_clock().now().nanoseconds / 1e9

    def _ids(self, name):
        """쉼표로 적힌 문자열 파라미터(name) → block_id 집합. 빈 문자열이면 빈 집합."""
        raw = self.get_parameter(name).value
        return {s.strip() for s in raw.split(',') if s.strip()}

    def on_check(self, req, res):
        """check_progress 콜백. 입력 req.design_id(빈 값 가능) · req.block_ids(전체 블록 이름). 출력 res — 배열 길이는 모두 요청과 같다(IRD 5장).
        블록마다 state·높이를 채운다. 바깥 영향: 로그뿐. 실패 흉내(fail 파라미터)면 success=false, reason ERROR."""
        absent, occluded = self._ids('absent'), self._ids('occluded')
        res.block_ids = list(req.block_ids)
        res.states, res.dx_m, res.dy_m, res.dz_m, res.top_z_m = [], [], [], [], []
        for bid in req.block_ids:
            design_id = design_of(req.design_id, bid)    # design_id 칸(E-52), 비었으면 옛 방식 '_B' 앞(10/8 저녁까지)
            if design_id and design_id not in self.tops:
                self.tops[design_id] = load_recipe_tops(self.get_parameter('recipe_dir').value, design_id, self.cfg)
                self.get_logger().info('레시피 %s: 블록 %d개' % (design_id, len(self.tops[design_id])))
            tops = self.tops.get(design_id, {})
            if bid in absent:
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
        else:
            res.success, res.reason = True, ''
        self.get_logger().info('check_progress (design_id 칸 %r) %d개 → %s' % (
            req.design_id, len(req.block_ids), dict(zip(res.block_ids, res.states))))
        return res

    def publish_status(self):
        """타이머마다 `/d2/vision/camera_status`(camera_status/1)를 낸다. 가짜라 last_frame_stamp = 지금 시각(초)."""
        t = self.now()
        msg = {'schema': 'camera_status/1', 'stamp': t, 'node': 'wrist_block', 'last_frame_stamp': t}
        self.pub_status.publish(String(data=json.dumps(msg)))


def main(args=None):
    """노드를 띄우고 Ctrl+C 까지 돈다. 로봇·카메라에 명령을 보내지 않으므로 서기 처리는 없다."""
    rclpy.init(args=args)
    node = MockWristBlock()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""가짜 손목 블록 인식 (mock_wrist_block) — IRD 4.2 `/d2/vision/check_progress` · 4.1 `camera_status/1`.

카메라·로봇 없이 작업 관리자(task)가 진행 확인 흐름을 돌릴 수 있게 한다(W034).
- `/d2/vision/check_progress`(CheckProgress): 요청한 block_id 마다 state 와 높이를 답한다.
  기본은 전부 present. 파라미터로 absent·occluded 블록을 고를 수 있다.
  top_z_m = 레시피의 블록 윗면 높이(설계 z, mm) → base(m): assembly_origin z + 설계 z. dz_m = 0.
  dx_m·dy_m 은 1차 규칙대로 늘 NaN(안 잼). absent·unknown 블록은 dz·top_z 도 NaN.
- `/d2/vision/camera_status`(camera_status/1, 2 Hz): 카메라가 살아 있다는 신호. 정지 노드 연결은 챌린지 공통.
- `scan_capture` · `scan_infer` 는 `JsonQuery.srv` 가 main 에 들어온 뒤(W121, 10/7 오전) 더한다.

실행 (저장소 맨 위에서, 레시피는 한세교 cad_recipe/1.0 파일):
  ros2 run d2_vision mock_wrist_block --ros-args -p recipe_path:=<lv1_bench.recipe.json>
돌리는 중에 바꾸기 (다음 요청부터 반영):
  ros2 param set /mock_wrist_block absent "LV1_B005,LV1_B006"   # 이 블록은 없음
  ros2 param set /mock_wrist_block occluded "LV1_B009"          # 이 블록은 가려짐
  ros2 param set /mock_wrist_block fail true                     # success=false, reason ERROR (카메라 고장 흉내)
시험 호출:
  ros2 service call /d2/vision/check_progress d2_interfaces/srv/CheckProgress "{block_ids: [LV1_B001, LV1_B011]}"

실패 때: 레시피 파일이 없거나 블록을 못 찾으면 그 블록은 state unknown, 높이 NaN 으로 답한다(노드는 안 죽는다).
"""
import json
import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from d2_interfaces.srv import CheckProgress

NAN = float('nan')
QOS_STATUS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def block_top_z_mm(block):
    """레시피 블록 하나의 윗면 높이(설계 좌표 z, mm). 입력: cad_recipe/1.0 의 blocks[i].
    중심 z + 블록 축을 설계 z 에 투영한 반높이 — 눕힘·세움 어느 방향이든 맞는다."""
    cz = block['center_cad_mm'][2]
    size = block['size_lwt_mm']
    r_z = [row[2] for row in block['R_cad_from_block']]   # 설계 z 축에 실린 블록 축 성분 (열 벡터)
    half_h = sum(abs(r_z[k]) * size[k] for k in range(3)) / 2.0
    return cz + half_h


def load_recipe_tops(path):
    """레시피 파일 → {block_id: 윗면 높이 mm}. 파일이 없거나 형식이 다르면 빈 dict."""
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        return {b['block_id']: block_top_z_mm(b) for b in data['blocks']}
    except (OSError, KeyError, TypeError, ValueError):
        return {}


class MockWristBlock(Node):
    """check_progress 에 답하고 camera_status 를 내는 가짜 노드. 로봇·카메라에 아무것도 하지 않는다."""

    def __init__(self):
        """파라미터 선언 → 레시피 읽기 → 서비스·토픽·타이머 만들기. 레시피를 못 읽어도 노드는 뜬다(경고만)."""
        super().__init__('mock_wrist_block')
        self.declare_parameter('recipe_path', '')        # cad_recipe/1.0 파일. 없으면 높이는 NaN
        self.declare_parameter('origin_z_m', -0.018)     # robot.yaml assembly_origin.z_m (작업면 높이)
        self.declare_parameter('absent', '')             # 없음으로 답할 block_id, 쉼표
        self.declare_parameter('occluded', '')           # 가려짐으로 답할 block_id, 쉼표
        self.declare_parameter('fail', False)            # True 면 success=false (카메라 고장 흉내)
        self.declare_parameter('status_hz', 2.0)         # camera_status 주기 (IRD 2 Hz)

        self.tops_mm = load_recipe_tops(self.get_parameter('recipe_path').value)
        if self.tops_mm:
            self.get_logger().info('레시피 블록 %d개 읽음' % len(self.tops_mm))
        else:
            self.get_logger().warn('레시피를 못 읽음(recipe_path) — 높이는 전부 NaN, state 는 unknown 으로 답한다')

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
        """요청 block_ids 마다 state·높이를 채운다. 배열 길이는 모두 요청과 같다(IRD 5장)."""
        absent, occluded = self._ids('absent'), self._ids('occluded')
        origin_z = self.get_parameter('origin_z_m').value
        res.block_ids = list(req.block_ids)
        res.states, res.dx_m, res.dy_m, res.dz_m, res.top_z_m = [], [], [], [], []
        for bid in req.block_ids:
            if bid in absent:
                state, top = 'absent', NAN
            elif bid in occluded:
                state, top = 'occluded', NAN
            elif bid in self.tops_mm:
                state, top = 'present', origin_z + self.tops_mm[bid] / 1000.0
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
        self.get_logger().info('check_progress %d개 → %s' % (len(req.block_ids), dict(zip(res.block_ids, res.states))))
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

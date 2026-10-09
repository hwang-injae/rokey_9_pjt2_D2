# -*- coding: utf-8 -*-
"""가짜 다리 mock_bridge — 웹 PC · 브로커 없이 로봇 PC 혼자 시험할 때 다리 자리에 선다 (IRD 10.6 · 11장, 옛 mock_web_ui, W127).

다리와 같은 ROS 이름 · 형식을 쓴다 — 작업 관리자 · 정지 노드는 진짜 다리와 구별하지 못한다.
- /d2/hmi/alive 를 alive_s 마다 낸다 → 작업 관리자가 웹이 붙은 것으로 본다(monitor_hmi 를 끄지 않아도 출발 · 스캔이 된다)
- /d2/hmi/get_design: recipe_dir 의 레시피 두 파일(<id>_recipe.json + _placements.csv) → design/2.0(recipe · placements · blocks)
- /d2/hmi/save_build: 받은 build/1 을 로그로 찍고 ok true(DB 없음)
- 터미널 명령으로 화면 버튼 · 음성 흉내(help 로 목록)
로봇을 직접 움직이지 않는다 — 출발 · 정지는 작업 관리자 · 정지 노드에 요청만 한다.
"""
import json
import os
import sys
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from d2_interfaces.srv import HmiCommand, JsonQuery, StopRequest
from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_blocks import RecipeToBlocks
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

HELP = """명령: select <design_id> · start [design_id] · scan · cancel · stop · resume
      intent <start|select_design|cancel|ask_progress> [design_id] · say <문장>(request_design) · alive on|off · help"""


def load_robot_yaml():
    """d2_bringup 의 config/robot.yaml 을 읽는다. 없으면 예외."""
    path = os.path.join(get_package_share_directory('d2_bringup'), 'config', 'robot.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def family_of(design_id):
    """기본 설계 이름 → family(IRD 2장 chair · desk). 이름에 CHAIR · DESK 가 없으면 ValueError(지어내지 않는다)."""
    name = design_id.upper()
    if 'CHAIR' in name:
        return 'chair'
    if 'DESK' in name:
        return 'desk'
    raise ValueError(f'{design_id}: 이름으로 family 를 알 수 없다(CHAIR · DESK)')


class MockBridge(Node):
    """다리 흉내 노드. 웹 연결 신호 · 설계 꺼내기 · 결과 저장에 답하고, 터미널 명령을 화면 요청으로 바꿔 보낸다.

    입력: 파라미터 recipe_dir(비우면 d2_bringup/recipes) · 터미널 한 줄 명령. 출력: ROS 서비스 호출 · 토픽.
    실패: 레시피가 없거나 틀리면 get_design 에 success false · ERROR 로 답한다(웹 backend 와 같은 꼴).
    """

    def __init__(self):
        """robot.yaml(블록 크기 · alive_s)을 읽고 서비스 · 클라이언트 · 연결 신호 타이머를 연다."""
        super().__init__('mock_bridge')
        self.declare_parameter('recipe_dir', '')
        cfg = load_robot_yaml()
        self.recipe_dir = (self.get_parameter('recipe_dir').value
                           or os.path.join(get_package_share_directory('d2_bringup'), 'recipes'))
        self.block_mm = [v * 1000.0 for v in cfg['block_size_m']]
        self.alive_on = True
        self.design_id = ''           # select 로 고른 설계 — start 에 design_id 를 안 주면 이것을 쓴다
        cb = ReentrantCallbackGroup()
        self.alive_pub = self.create_publisher(String, '/d2/hmi/alive', 10)
        self.intent_pub = self.create_publisher(String, '/d2/hmi/intent', 10)
        self.command_cli = self.create_client(HmiCommand, '/d2/hmi/command', callback_group=cb)
        self.stop_cli = self.create_client(StopRequest, '/d2/safety/stop', callback_group=cb)
        self.resume_cli = self.create_client(Trigger, '/d2/safety/resume', callback_group=cb)
        self.create_service(JsonQuery, '/d2/hmi/get_design', self._on_get_design, callback_group=cb)
        self.create_service(JsonQuery, '/d2/hmi/save_build', self._on_save_build, callback_group=cb)
        self.create_timer(float(cfg['mqtt']['alive_s']), self._send_alive, callback_group=cb)
        self.get_logger().info(f'[가짜 다리] 레시피 폴더 {self.recipe_dir}\n{HELP}')

    def _send_alive(self):
        """웹 연결 신호를 낸다. 'alive off' 면 멈춰 웹 끊김(WAIT_HMI)을 흉내 낸다."""
        if self.alive_on:
            self.alive_pub.publish(String(data=json.dumps({'alive': True, 'stamp': round(time.time(), 3)})))

    # ---------- 웹 backend 흉내 ----------
    def design(self, design_id):
        """레시피 두 파일 → design/2.0 dict(웹 backend design_store 의 기본 설계와 같은 칸). 실패는 OSError · ValueError."""
        family = family_of(design_id)
        doc = RecipeDocument.load(self.recipe_dir, design_id)
        out = doc.design(design_id)
        out.update(family=family, version='1.0', parent_id=None, made_by='cad',
                   blocks=RecipeToBlocks(design_id, family, self.block_mm).convert(doc.recipe, doc.placements))
        return out

    def _on_get_design(self, req, res):
        """/d2/hmi/get_design — {"design_id"} → design/2.0. 못 찾거나 틀리면 success false · ERROR."""
        try:
            design_id = json.loads(req.request_json)['design_id']
            res.response_json = json.dumps(self.design(design_id), ensure_ascii=False)
            res.success, res.reason = True, ''
            self.get_logger().info(f'[가짜 다리] get_design {design_id} — 블록 {len(json.loads(res.response_json)["blocks"]["blocks"])}개')
        except (OSError, ValueError, KeyError, TypeError) as e:
            res.success, res.reason, res.response_json = False, 'ERROR', ''
            self.get_logger().warn(f'[가짜 다리] get_design 실패: {e}')
        return res

    def _on_save_build(self, req, res):
        """/d2/hmi/save_build — build/1 을 로그로 찍고 {"ok": true}(DB 없음 — 두 층 모두 참, IRD 4.2)."""
        self.get_logger().info(f'[가짜 다리] save_build {req.request_json}')
        res.success, res.reason, res.response_json = True, '', json.dumps({'ok': True})
        return res

    # ---------- 화면 · 음성 흉내 ----------
    def run_command(self, line):
        """터미널 한 줄을 화면 요청 · 음성 의도로 바꾼다. 모르는 명령은 도움말을 찍는다."""
        words = line.split()
        if not words:
            return
        cmd, args = words[0], words[1:]
        if cmd in ('select', 'start', 'scan', 'cancel'):
            if cmd == 'select' and args:
                self.design_id = args[0]
            design_id = args[0] if args else (self.design_id if cmd in ('select', 'start') else '')
            name = 'select_design' if cmd == 'select' else cmd
            self._call(self.command_cli, HmiCommand.Request(cmd=name, design_id=design_id, mode='auto'), f'hmi/command {name}')
        elif cmd == 'stop':
            self._call(self.stop_cli, StopRequest.Request(source='web', reason=''), 'safety/stop')   # reason 비움 → STOP_WEB
        elif cmd == 'resume':
            self._call(self.resume_cli, Trigger.Request(), 'safety/resume')
        elif cmd == 'intent' and args:
            self._intent(args[0], args[1] if len(args) > 1 else self.design_id, '')
        elif cmd == 'say' and args:
            self._intent('request_design', None, ' '.join(args))
        elif cmd == 'alive' and args in (['on'], ['off']):
            self.alive_on = args[0] == 'on'
            self.get_logger().info(f'[가짜 다리] 웹 연결 신호 {"켬" if self.alive_on else "끔(3초 뒤 WAIT_HMI)"}')
        else:
            print(HELP)

    def _call(self, client, req, label):
        """서비스를 부르고 답을 로그로 찍는다. 서비스가 없으면 바로 알린다(기다리지 않음)."""
        if not client.service_is_ready():
            self.get_logger().warn(f'[가짜 다리] {label}: 서비스가 안 떠 있다')
            return
        client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(f'[가짜 다리] {label} → {f.result()}'))

    def _intent(self, intent, design_id, text):
        """음성 의도 intent/1 을 낸다(confidence 1.0 — 가짜라 확실하다고 둔다)."""
        body = {'schema': 'intent/1', 'stamp': round(time.time(), 3), 'intent': intent, 'design_id': design_id or None,
                'mode': None, 'text': text, 'confidence': 1.0}
        self.intent_pub.publish(String(data=json.dumps(body, ensure_ascii=False)))
        self.get_logger().info(f'[가짜 다리] intent {body}')


def read_stdin(node):
    """터미널 명령을 한 줄씩 읽어 노드에 넘긴다(데몬 스레드). 입력이 닫히면 끝난다."""
    for line in sys.stdin:
        node.run_command(line.strip())


def main():
    """가짜 다리를 켠다. 터미널이면 명령을 받는다. 로봇을 움직이지 않으므로 rclpy 기본 Ctrl+C 처리를 쓴다."""
    rclpy.init()
    node = MockBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    if sys.stdin.isatty():
        threading.Thread(target=read_stdin, args=(node,), daemon=True).start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

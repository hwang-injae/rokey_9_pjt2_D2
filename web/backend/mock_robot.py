# -*- coding: utf-8 -*-
"""가짜 로봇 PC mock_robot — 로봇 PC · 다리 없이 웹(화면 · backend)을 개발할 때 쓴다 (IRD 10.6 · 11장, W126). MQTT 만 쓴다(ROS 없음).

- d2/bridge/alive 를 1초마다(LWT alive false), 처음 상태를 retained 로(task/state IDLE · safety/state · gripper/state)
- …/req 에 …/res 로 답한다(IRD 10.1 칸 그대로):
  hmi/command — select_design → READY · start → 가짜 조립(블록마다 progress/1.1) → DONE · scan → 스캔 단계 → SCAN_REVIEW + scan_result/1.1 · cancel
  safety/stop → 잠금 + STOPPED(reason STOP_WEB) · check_design → 늘 ok + 벤치 recipe · placements
  safety/resume → 풀림. 진짜 작업 관리자와 같이 조립 중에 멈췄으면 RECOVER → CHECK → 놓인 블록은 건너뛰고 이어서,
  조립 중이 아니었으면 IDLE(설계 다시 고름) — task_manager._recover · 복구 절차 문서 5장
- 가짜 조립 중 블록마다 가짜 손목 검출 그림(d2/vision/wrist_image, 640×480 JPEG — 진짜는 find_blocks 때 손목 비전이 그림). PIL 이 없으면 그림만 안 보냄
- 설계는 저장소 레시피 파일만 안다(웹 저장소 W111 전). 로봇을 움직이는 코드는 없다.
실행: python3 web/backend/mock_robot.py [--host localhost] [--step 1.0]
"""
import argparse
import io
import json
import random
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
for _pkg in ('d2_bridge', 'd2_task'):       # 같은 저장소의 ROS 없는 파일(app.py 와 같은 방식)
    sys.path.insert(0, str(REPO / 'src' / _pkg))

import yaml  # noqa: E402
from d2_bridge.bridge_codec import alive_payload, parse_request, req_topic, res_topic  # noqa: E402
from d2_task.recipe_document import RecipeDocument  # noqa: E402
from d2_task.recipe_to_blocks import RecipeToBlocks  # noqa: E402
from mqtt_client import make_mqtt_client  # noqa: E402

RECIPES = REPO / 'src' / 'd2_robot' / 'd2_bringup' / 'recipes'
ROBOT_YAML = REPO / 'src' / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml'
BENCH = '001_CHAIR_BENCH'           # check_design 응답 · 스캔 결과에 쓰는 설계(IRD 11장 '벤치 레시피')
SERVICES = ('/d2/hmi/command', '/d2/safety/stop', '/d2/safety/resume', '/d2/task/check_design')
CANCEL_OK = ('READY', 'ERROR', 'SCAN_REVIEW')     # 취소를 받는 상태(SDD 5.1, 10/8 E-62)
RUN_STATES = ('CHECK', 'SELECT', 'PICK_PLACE', 'WAIT_SUPPLY', 'WAIT_HMI', 'VERIFY', 'RECOVER', 'ERROR')   # 조립 중(task_manager 와 같음)


def fake_wrist_jpeg(seed, text):
    """가짜 손목 검출 그림(640×480 JPEG, IRD 10.5 크기) — 회색 바탕에 블록 3개 + 윤곽 · 점수 글자. PIL 이 없으면 None."""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return None
    img = Image.new('RGB', (640, 480), (70, 72, 76))
    draw = ImageDraw.Draw(img)
    rng = random.Random(seed)
    for i in range(3):
        x, y = rng.randint(40, 440), rng.randint(60, 300)
        w, h = (150, 50) if rng.random() < 0.5 else (50, 150)
        draw.rectangle([x, y, x + w, y + h], fill=(196, 160, 112), outline=(0, 230, 0) if i == 0 else (255, 200, 0), width=3)
        draw.text((x, y - 14), f'block {0.93 - i * 0.07:.2f}' + (' <- pick' if i == 0 else ''), fill=(255, 255, 255))
    draw.text((10, 10), text, fill=(255, 255, 255))
    buf = io.BytesIO()
    img.save(buf, 'JPEG', quality=70)
    return buf.getvalue()


class MockRobot:
    """로봇 PC 흉내. 상태 하나(state/1)와 잠금 하나(safety_state/1)를 들고, 명령에 따라 바꾸고 MQTT 로 방송한다.

    입력: 화면 요청(…/req). 출력: 응답(…/res) · retained 상태 토픽. 실패: 모르는 설계 select 는 success false · ERROR.
    """

    def __init__(self, host, port, step_s):
        """벤치 레시피를 읽어 두고(검사 응답 · 스캔 결과) MQTT 를 준비한다. step_s = 가짜 조립 · 스캔 한 단계 시간(초)."""
        cfg = yaml.safe_load(ROBOT_YAML.read_text(encoding='utf-8'))
        self.block_mm = [v * 1000 for v in cfg['block_size_m']]
        self.step_s = step_s
        self.lock = threading.Lock()
        self.st = dict(state='IDLE', run_id=None, design_id=None, block_id=None, message_id=None, message='대기')
        self.locked = False
        self.assembling = False       # 멈췄을 때 조립 중이었나 — 다시 시작 뒤 이어 갈지 정한다(진짜 작업 관리자와 같음)
        self.placed = []              # 이번 조립(run)에서 놓은 블록 이름 — 다시 시작 뒤 건너뛴다
        self.worker = None
        self.abort = threading.Event()
        self.c = make_mqtt_client('d2_mock_robot')
        self.c.on_connect = self._on_connect
        self.c.on_message = self._on_message
        self.c.will_set('d2/bridge/alive', alive_payload(False), qos=1, retain=True)
        self.host, self.port = host, port

    # ---------- 설계 ----------
    def design(self, design_id):
        """레시피 두 파일 → (doc, blocks/2.0). 없으면 OSError · ValueError."""
        family = 'chair' if 'CHAIR' in design_id.upper() else 'desk'
        doc = RecipeDocument.load(RECIPES, design_id)
        return doc, RecipeToBlocks(design_id, family, self.block_mm).convert(doc.recipe, doc.placements)

    # ---------- 방송 ----------
    def pub(self, topic, body, qos=1):
        """상태 JSON 을 retained 로 보낸다(다리가 하는 것과 같음)."""
        self.c.publish(topic, json.dumps({**body, 'stamp': round(time.time(), 3)}, ensure_ascii=False), qos=qos, retain=True)

    def set_state(self, state, message='', message_id=None, **kw):
        """작업 관리자 상태를 바꾸고 state/1 을 보낸다."""
        with self.lock:
            self.st.update(state=state, message=message, message_id=message_id, **kw)
            body = dict(schema='state/1', mode='auto', **self.st)
        self.pub('d2/task/state', body)
        print(f'[가짜 로봇] {state} {message}')

    def set_safety(self, locked, reason=''):
        """정지 상태를 바꾸고 safety_state/1 을 보낸다."""
        self.locked = locked
        self.pub('d2/safety/state', dict(schema='safety_state/1', stopped=locked, locked=locked, reason=reason))

    # ---------- 가짜 흐름 ----------
    def _assemble(self, design_id, run_id, placed, recovering=False):
        """블록마다 PICK_PLACE → VERIFY → progress 를 step_s 간격으로. placed 에 있는 블록은 건너뛴다. 정지(abort)면 바로 멈춘다.

        recovering = 다시 시작 뒤: RECOVER(쥔 블록 확인) → CHECK(진행 확인) 를 거친 뒤 남은 블록부터(진짜 작업 관리자 순서).
        """
        doc, _ = self.design(design_id)
        steps = sorted(doc.placements['steps'], key=lambda s: s['sequence'])
        ids = [s['block_id'] for s in steps]
        if recovering:
            for state, text in (('RECOVER', '다시 시작 — 쥔 블록을 확인합니다'),
                                ('CHECK', f'진행 확인부터 다시 합니다 — 놓인 블록 {len(placed)}개는 건너뜀')):
                self.set_state(state, text, block_id=None)
                if self.abort.wait(self.step_s):
                    return
        for n, bid in enumerate(ids, 1):
            if bid in placed:
                continue
            jpeg = fake_wrist_jpeg(n, f'MOCK wrist  {n}/{len(ids)}  {bid}')
            if jpeg:
                self.c.publish('d2/vision/wrist_image', jpeg, qos=0)   # 진짜와 같이 retained 아님(IRD 10.1)
            for state in ('PICK_PLACE', 'VERIFY'):
                self.set_state(state, f'{len(placed) + 1}/{len(ids)} {bid}', block_id=bid)
                if self.abort.wait(self.step_s):
                    return
            placed.append(bid)
            self.pub('d2/task/progress', {'schema': 'progress/1.1', 'design_id': design_id, 'run_id': run_id,
                                          'obs_stamp': round(time.time(), 3),
                                          'blocks': [{'block_id': b, 'state': 'present' if b in placed else 'absent',
                                                      'by': 'robot'} for b in ids]})
        self.set_state('DONE', '완성했어요', 'done', block_id=None)

    def _scan(self, run_id):
        """SCAN_MOVE · CAPTURE 를 자세 3곳 → SCAN_INFER → scan_result(벤치 blocks/1, 추정 2개) → SCAN_REVIEW."""
        for pose in ('observe', 'observe_front', 'observe_side'):
            for state in ('SCAN_MOVE', 'SCAN_CAPTURE'):
                self.set_state(state, f'스캔 중 — {pose}', 'scan_running')
                if self.abort.wait(self.step_s):
                    return
        self.set_state('SCAN_INFER', '스캔 결과 계산 중', 'scan_running')
        if self.abort.wait(self.step_s):
            return
        _, blocks = self.design(BENCH)
        items = [{'order': b['order'], 'x': b['x'], 'y': b['y'], 'z': b['z'], 'ori': b['ori'], 'inferred': i < 2}
                 for i, b in enumerate(blocks['blocks'])]     # 가려져 추측한 블록 2개를 흉내(화면 반투명 표시 시험용)
        self.pub('d2/task/scan_result', {'schema': 'scan_result/1.1', 'run_id': run_id, 'inferred_count': 2,
                                         'blocks': {'schema': 'blocks/1', 'design_id': 'scan_chair_01', 'family': 'chair',
                                                    'blocks': items},
                                         'image_path': '', 'poses_used': ['observe', 'observe_front', 'observe_side']})
        self.set_state('SCAN_REVIEW', '스캔 결과를 확인하고 [저장] · [다시 스캔] · [취소]', 'scan_review')

    def _run(self, target, *args):
        """가짜 흐름 스레드를 하나만 돌린다(앞 흐름은 멈춘 뒤)."""
        self.abort.set()
        if self.worker and self.worker.is_alive():
            self.worker.join()
        self.abort.clear()
        self.worker = threading.Thread(target=target, args=args, daemon=True)
        self.worker.start()

    # ---------- 요청 ----------
    def command(self, b):
        """hmi/command 처리 → (success, reason). 실제 작업 관리자 규칙을 간단히 흉내(잠금 · 상태 · 설계)."""
        cmd, design_id, state = b.get('cmd'), b.get('design_id') or '', self.st['state']
        if self.locked:
            return False, 'STOPPED'
        if cmd == 'select_design':
            try:
                self.design(design_id)
            except (OSError, ValueError):
                return False, 'ERROR'
            self.set_state('READY', f'{design_id} 선택 — 출발을 누르세요', 'ready_to_start', design_id=design_id)
            return True, ''
        if cmd == 'start':
            design_id = design_id or self.st['design_id']
            if state not in ('READY', 'WAIT_SUPPLY', 'WAIT_HMI') or not design_id:
                return False, 'BUSY'
            run_id = time.strftime('R%Y%m%d_%H%M%S_') + 'm0ck'
            self.placed = []
            self.set_state('CHECK', '출발 — 관측', run_id=run_id, design_id=design_id)
            self._run(self._assemble, design_id, run_id, self.placed)
            return True, ''
        if cmd == 'scan':
            if state not in ('IDLE', 'DONE', 'SCAN_REVIEW'):
                return False, 'BUSY'
            run_id = time.strftime('R%Y%m%d_%H%M%S_') + 'scan'
            self.set_state('SCAN_MOVE', '스캔 시작', 'scan_running', run_id=run_id)
            self._run(self._scan, run_id)
            return True, ''
        if cmd == 'cancel':
            if state not in CANCEL_OK:
                return False, 'BUSY'
            self.set_state('IDLE', '취소했어요', design_id=None, run_id=None)
            return True, ''
        return False, 'ERROR'

    def answer(self, name, b):
        """서비스 하나에 답할 칸을 만든다(IRD 10.1 응답 칸)."""
        if name == '/d2/hmi/command':
            ok, reason = self.command(b)
            return {'success': ok, 'reason': reason}
        if name == '/d2/safety/stop':
            self.abort.set()
            if not self.locked:
                self.assembling = self.st['state'] in RUN_STATES
            self.set_safety(True, b.get('reason') or 'STOP_' + (b.get('source') or 'web').upper())
            self.set_state('STOPPED', '멈췄어요. 원인을 없앤 뒤 [다시 시작]을 누르세요', 'stopped')
            return {'success': True, 'message': '세우는 중'}
        if name == '/d2/safety/resume':
            was = self.locked
            self.set_safety(False)
            if was and self.assembling and self.st['design_id'] and self.st['run_id']:
                self._run(self._assemble, self.st['design_id'], self.st['run_id'], self.placed, True)   # 같은 run_id 로 이어서
            elif was:
                self.set_state('IDLE', '정지가 풀렸어요. 설계를 다시 고르세요', design_id=None, run_id=None, block_id=None)
            return {'success': True, 'message': '잠금을 풀었다' if was else '잠겨 있지 않았다'}
        doc, _ = self.design(BENCH)                           # check_design — 늘 합격 + 벤치 레시피(IRD 11장)
        return {'success': True, 'reason': '', 'schema': 'check_result/2.0', 'ok': True, 'min_margin_mm': 12.5, 'errors': [],
                'recipe': doc.recipe, 'placements': doc.placements}

    # ---------- MQTT ----------
    def _on_connect(self, client, _u, _f, rc):
        """요청 토픽 구독 + 처음 상태를 retained 로."""
        client.subscribe([(req_topic(n), 1) for n in SERVICES])
        self.set_safety(self.locked)
        self.set_state(self.st['state'], self.st['message'], self.st['message_id'])
        self.pub('d2/gripper/state', {'schema': 'gripper_state/1', 'width_m': None, 'grasped': False}, qos=0)

    def _on_message(self, client, _u, msg):
        """요청 하나에 답한다. retained 로 남은 옛 요청 · 깨진 요청은 버린다(다리와 같은 규칙)."""
        if not msg.payload or msg.retain:
            return
        try:
            b = parse_request(msg.payload)
        except ValueError:
            return
        name = '/' + msg.topic[:-len('/req')]
        client.publish(res_topic(name), json.dumps({**self.answer(name, b), 'req_id': b['req_id']}, ensure_ascii=False), qos=1)

    def run(self):
        """브로커에 붙고 연결 신호를 1초마다 보낸다. Ctrl+C 면 alive false 를 남기고 끝."""
        self.c.connect(self.host, self.port, 5)
        self.c.loop_start()
        print(f'[가짜 로봇] {self.host}:{self.port} — 화면에서 버튼을 눌러 보세요(Ctrl+C 로 끝)')
        try:
            while True:
                self.c.publish('d2/bridge/alive', alive_payload(True), qos=1, retain=True)
                time.sleep(1)
        except KeyboardInterrupt:
            self.abort.set()
            self.c.publish('d2/bridge/alive', alive_payload(False), qos=1, retain=True).wait_for_publish(1)
            self.c.disconnect()
            self.c.loop_stop()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='가짜 로봇 PC (MQTT)')
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=1883)
    ap.add_argument('--step', type=float, default=1.0, help='가짜 조립 · 스캔 한 단계 시간(초)')
    a = ap.parse_args()
    MockRobot(a.host, a.port, a.step).run()

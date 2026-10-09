"""웹 backend MQTT 층(web/backend — mqtt_client · routes, W126)이 IRD 10장대로 요청 · 응답 · 상태 · 연결 신호를 다루는지 지킨다.

브로커 없이 가짜 paho 클라이언트로 돈다. 지키는 것: 요청 = …/req + req_id, 답을 req_id 로 짝지음 · 시간 초과 TIMEOUT · 자동 재전송 없음,
상태는 마지막 값 + 화면 이벤트, 다리 연결 신호 3초 끊김, retained 로 남은 옛 요청에는 답하지 않음, 로봇 PC 가 끊기면 출발 · 스캔을 막음.
"""
import json
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / 'web' / 'backend', ROOT / 'src' / 'd2_bridge', ROOT / 'src' / 'd2_task'):
    sys.path.insert(0, str(p))

from mqtt_client import MqttClient  # noqa: E402


class FakePaho:
    """paho 클라이언트 흉내 — 보낸 것을 모아 두고, 붙어 있는지만 정한다."""

    def __init__(self):
        self.sent, self.subs, self.up = [], [], True
        self.on_publish_hook = None

    def will_set(self, *a, **k):
        self.will = (a, k)

    def is_connected(self):
        return self.up

    def publish(self, topic, payload, qos=0, retain=False):
        self.sent.append((topic, json.loads(payload), retain))
        if self.on_publish_hook:
            self.on_publish_hook(topic, json.loads(payload))
        return SimpleNamespace(rc=0, wait_for_publish=lambda *_: None)

    def subscribe(self, topics):
        self.subs.extend(t for t, _ in topics)


def msg(topic, body, retain=False):
    """paho 메시지 흉내."""
    payload = body if isinstance(body, bytes) else json.dumps(body).encode()
    return SimpleNamespace(topic=topic, payload=payload, retain=retain)


@pytest.fixture
def mc():
    """가짜 시계 · 가짜 paho 로 만든 MqttClient(req_timeout 0.3 초 — 시험을 빨리)."""
    now = [100.0]
    m = MqttClient('localhost', 1883, req_timeout_s=0.3, alive_s=1, lost_after_s=3, client=FakePaho(), clock=lambda: now[0])
    m.now = now
    m.events = []
    m.on_event = m.events.append
    return m


def test_subscribes_states_alive_intent_responses_and_robot_requests(mc):
    """붙으면 상태 5종 · 다리 연결 신호 · 의도 · 응답 4개 · 로봇 요청 2개를 구독한다(IRD 10.1)."""
    mc._on_connect(mc.client, None, None, 0)
    subs = set(mc.client.subs)
    assert {'d2/task/state', 'd2/task/progress', 'd2/task/scan_result', 'd2/safety/state', 'd2/gripper/state',
            'd2/bridge/alive', 'd2/hmi/intent', 'd2/hmi/command/res', 'd2/safety/stop/res', 'd2/safety/resume/res',
            'd2/task/check_design/res', 'd2/hmi/get_design/req', 'd2/hmi/save_build/req'} <= subs


def test_request_pairs_by_req_id(mc):
    """요청 = …/req + req_id. 같은 req_id 의 …/res 가 오면 그 칸을 돌려준다(req_id 는 뺌)."""
    def robot_answers(topic, body):
        if topic == 'd2/hmi/command/req':
            threading.Timer(0.02, mc._on_message, args=(mc.client, None, msg(
                'd2/hmi/command/res', {'req_id': body['req_id'], 'success': False, 'reason': ''}))).start()
    mc.client.on_publish_hook = robot_answers
    out = mc.request('/d2/hmi/command', {'cmd': 'start', 'design_id': '001_CHAIR_BENCH', 'mode': 'auto'})
    assert out == {'success': False, 'reason': ''}                 # 빈 reason 거절도 그대로(#83) — 고치지 않는다
    topic, body, retain = mc.client.sent[0]
    assert topic == 'd2/hmi/command/req' and body['cmd'] == 'start' and body['req_id'] and retain is False


def test_request_timeout_and_no_broker_without_resend(mc):
    """답이 없으면 TIMEOUT, 브로커가 없으면 ERROR. 어느 쪽도 다시 보내지 않는다."""
    assert mc.request('/d2/safety/stop', {'source': 'web', 'reason': ''})['reason'] == 'TIMEOUT'
    assert len(mc.client.sent) == 1
    mc.client.up = False
    assert mc.request('/d2/safety/resume', {})['reason'] == 'ERROR'
    assert len(mc.client.sent) == 1


def test_state_kept_and_emitted_bad_json_ignored(mc):
    """상태는 마지막 값을 들고 화면 이벤트로 알린다. 깨진 JSON · 빈 payload · 객체 아님은 버린다."""
    mc._on_message(mc.client, None, msg('d2/task/state', {'schema': 'state/1', 'state': 'READY'}))
    mc._on_message(mc.client, None, msg('d2/task/state', b'{broken'))
    mc._on_message(mc.client, None, msg('d2/task/progress', b''))
    mc._on_message(mc.client, None, msg('d2/safety/state', [1, 2]))
    assert mc.snapshot()['state']['state'] == 'READY' and 'progress' not in mc.snapshot()
    assert mc.events == [{'type': 'state', 'data': {'schema': 'state/1', 'state': 'READY'}}]


def test_bridge_alive_and_lost_after(mc):
    """다리 연결 신호가 오면 연결, lost_after_s(3초) 넘게 안 오면 끊김 이벤트. alive false(LWT)면 바로 끊김."""
    mc._on_message(mc.client, None, msg('d2/bridge/alive', {'alive': True, 'stamp': 1.0}))
    assert mc.bridge_connected() and mc.events[-1] == {'type': 'bridge_alive', 'data': {'alive': True}}
    mc.now[0] += 2.9
    mc.check_bridge()
    assert mc.bridge_connected()
    mc.now[0] += 0.2
    mc.check_bridge()
    assert not mc.bridge_connected() and mc.events[-1] == {'type': 'bridge_alive', 'data': {'alive': False}}
    mc._on_message(mc.client, None, msg('d2/bridge/alive', {'alive': True}))
    mc._on_message(mc.client, None, msg('d2/bridge/alive', {'alive': False}))
    assert not mc.bridge_connected()


def test_robot_request_answered_by_handler_once(mc):
    """로봇이 부른 get_design 은 등록한 함수가 답한다(req_id 그대로). 같은 req_id 두 번째 · retained 옛 요청에는 안 답한다."""
    mc.serve('/d2/hmi/get_design', lambda b: {'success': True, 'reason': '', 'schema': 'design/2.0', 'design_id': b['design_id']})
    req = {'req_id': 'g1', 'design_id': '001_CHAIR_BENCH'}
    mc._on_message(mc.client, None, msg('d2/hmi/get_design/req', req))
    mc._on_message(mc.client, None, msg('d2/hmi/get_design/req', req))
    mc._on_message(mc.client, None, msg('d2/hmi/get_design/req', {'req_id': 'g2', 'design_id': 'X'}, retain=True))
    assert mc.client.sent == [('d2/hmi/get_design/res', {'success': True, 'reason': '', 'schema': 'design/2.0',
                                                         'design_id': '001_CHAIR_BENCH', 'req_id': 'g1'}, False)]


def test_robot_request_without_store_is_error(mc):
    """저장소가 아직 없으면(W111 전) success false · ERROR 로 바로 답한다 — 다리가 TIMEOUT 까지 기다리지 않게."""
    mc._on_message(mc.client, None, msg('d2/hmi/save_build/req', {'req_id': 's1', 'schema': 'build/1'}))
    topic, body, _ = mc.client.sent[0]
    assert topic == 'd2/hmi/save_build/res' and body['success'] is False and body['reason'] == 'ERROR' and body['req_id'] == 's1'


# ---------- REST · WebSocket (FastAPI) ----------
fastapi = pytest.importorskip('fastapi')
from fastapi.testclient import TestClient  # noqa: E402


class FakeMqtt:
    """routes 시험용 MqttClient 흉내 — 보낸 요청을 모으고 정해 둔 답을 돌려준다."""

    def __init__(self, bridge=True):
        self.calls, self.bridge, self.on_event = [], bridge, None
        self.host, self.port = 'localhost', 1883

    def request(self, name, fields):
        self.calls.append((name, fields))
        return {'success': True, 'reason': ''}

    def bridge_connected(self):
        return self.bridge

    def snapshot(self):
        return {'state': {'state': 'IDLE'}, 'bridge_alive': self.bridge, 'broker': True}

    def start(self):
        pass

    def stop(self):
        pass


RULES = {'block_size_m': [0.075, 0.025, 0.015], 'margin_mm': 7, 'max_blocks': 54, 'finger_thickness_m': 0.0114,
         'finger_width_m': 0.02, 'assembly_area_half_m': 0.15, 'req_timeout_s': 5, 'alive_s': 1, 'lost_after_s': 3}


def make_client(bridge=True):
    """가짜 MQTT 로 앱을 만든다(lifespan 은 돌리지 않음)."""
    import app as web_app
    fake = FakeMqtt(bridge)
    return TestClient(web_app.create_app(mqtt=fake, rules=RULES)), fake


def test_command_stop_resume_map_to_ird_names():
    """버튼 → IRD ROS 이름 · 칸: 명령은 /d2/hmi/command, 정지는 source web + 빈 reason(→ STOP_WEB), 다시 시작은 빈 칸."""
    client, fake = make_client()
    assert client.post('/api/robot/command', json={'cmd': 'select_design', 'design_id': '001_CHAIR_BENCH'}).json()['success']
    client.post('/api/robot/stop')
    client.post('/api/robot/resume')
    assert fake.calls == [('/d2/hmi/command', {'cmd': 'select_design', 'design_id': '001_CHAIR_BENCH', 'mode': 'auto'}),
                          ('/d2/safety/stop', {'source': 'web', 'reason': ''}),
                          ('/d2/safety/resume', {})]


def test_unknown_cmd_rejected_and_bridge_lost_blocks_start_scan_not_stop():
    """모르는 cmd 는 422. 로봇 PC 가 끊기면 출발 · 스캔은 보내지 않지만 정지 · 취소는 보낸다(IRD 10.3)."""
    client, fake = make_client(bridge=False)
    assert client.post('/api/robot/command', json={'cmd': 'turn_done'}).status_code == 422
    for cmd in ('start', 'scan'):
        body = client.post('/api/robot/command', json={'cmd': cmd}).json()
        assert body['success'] is False and '끊김' in body['message']
    client.post('/api/robot/command', json={'cmd': 'cancel'})
    client.post('/api/robot/stop')
    assert [n for n, _ in fake.calls] == ['/d2/hmi/command', '/d2/safety/stop']


def test_ws_sends_snapshot_first():
    """화면이 /ws 에 붙으면 들고 있는 값을 type 별로 먼저 보낸다({"type", "data"})."""
    client, _ = make_client()
    with client.websocket_connect('/ws') as ws:
        got = [ws.receive_json() for _ in range(3)]
    assert {'type': 'state', 'data': {'state': 'IDLE'}} in got
    assert {'type': 'bridge_alive', 'data': {'alive': True}} in got
    assert {'type': 'broker', 'data': {'connected': True}} in got

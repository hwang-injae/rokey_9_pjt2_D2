# -*- coding: utf-8 -*-
"""MqttClient — 웹 backend 가 브로커(→ 로봇 PC 다리)와 통신하는 한 곳 (IRD 10장, SDD 3.1.1, W126).

routes · voice 는 이 클래스만 부르고, 화면(frontend)은 MQTT 를 모른다(E-41). ROS 도 모른다 — 이름은 IRD 의 ROS 이름을 쓰지만 MQTT 토픽으로만 오간다.
토픽 이름 · req/res 짝 맞추기는 다리와 같은 d2_bridge/bridge_codec.py 를 쓴다(두 쪽 규칙을 한 곳에).
"""
import json
import threading
import time
import uuid

import paho.mqtt.client as mqtt
from d2_bridge.bridge_codec import PendingReplies, SeenIds, alive_payload, parse_request, req_topic, res_topic

STATE_TOPICS = {            # MQTT 상태 토픽(retained) → /ws 이벤트 type
    'd2/task/state': 'state',
    'd2/task/progress': 'progress',
    'd2/task/scan_result': 'scan_result',
    'd2/safety/state': 'safety',
    'd2/gripper/state': 'gripper',
}
INTENT_TOPIC = 'd2/hmi/intent'
BRIDGE_ALIVE_TOPIC = 'd2/bridge/alive'
WEB_ALIVE_TOPIC = 'd2/web/alive'
WEB_CALLS = ('/d2/hmi/command', '/d2/safety/stop', '/d2/safety/resume', '/d2/task/check_design')   # 웹이 부름
ROBOT_CALLS = ('/d2/hmi/get_design', '/d2/hmi/save_build')   # 로봇이 부르고 웹(저장소)이 답함
CLIENT_ID = 'd2_web'        # 웹 backend 는 하나 — 같은 이름으로 다시 붙으면 브로커가 옛 연결을 끊는다
MQTT_KEEPALIVE_S = 5        # 웹 PC 가 갑자기 꺼지면 약 1.5배 뒤 브로커가 LWT(alive false)를 남긴다. 작업 관리자는 그 전에 3초로 먼저 안다
CLOSE_WAIT_S = 1.0          # 끝낼 때 'alive false' 가 나갈 때까지 기다리는 한도


def make_mqtt_client(client_id):
    """paho 클라이언트를 만든다. apt 1.6 과 pip 2.x 둘 다 같은 콜백 꼴(VERSION1)로 쓴다(다리와 같음)."""
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    except AttributeError:
        return mqtt.Client(client_id=client_id)


class MqttClient:
    """브로커 연결 하나. 화면 요청 보내기 · 로봇 상태 들고 있기 · 연결 신호 · 로봇이 부른 요청에 답하기.

    입력: host · port(.env), req_timeout_s · alive_s · lost_after_s(robot.yaml mqtt.*). client 는 시험 때 가짜를 넣는다.
    바깥 영향: d2/web/alive 를 alive_s 마다 retained 로 보낸다(작업 관리자는 3초 끊기면 WAIT_HMI — IRD 10.3).
    실패: 브로커가 없으면 paho 가 1~5초 간격으로 다시 붙는다. 요청은 자동으로 다시 보내지 않는다(실제 상태는 state/1 로 본다).
    """

    def __init__(self, host, port, req_timeout_s, alive_s, lost_after_s, client=None, clock=time.monotonic):
        """연결은 start() 에서. on_event 는 /ws 허브가 넣는다(상태가 바뀔 때마다 {"type", "data"} 를 받음)."""
        self.host, self.port = host, int(port)
        self.req_timeout_s, self.alive_s, self.lost_after_s = float(req_timeout_s), float(alive_s), float(lost_after_s)
        self.clock = clock
        self.on_event = None
        self.handlers = {}               # 로봇이 부르는 요청: ROS 이름 → fn(body dict) → 응답 dict(success · reason · 칸들)
        self.pending, self.seen = PendingReplies(), SeenIds()
        self._lock = threading.Lock()
        self.state = {}                  # /ws type → 마지막 상태 dict
        self._bridge_alive, self._bridge_seen_at = False, None
        self._stop = threading.Event()
        self.client = client or make_mqtt_client(CLIENT_ID)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.will_set(WEB_ALIVE_TOPIC, alive_payload(False), qos=1, retain=True)

    # ---------- 연결 ----------
    def start(self):
        """브로커에 붙기 시작하고(기다리지 않음) 연결 신호 스레드를 띄운다."""
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.client.connect_async(self.host, self.port, keepalive=MQTT_KEEPALIVE_S)
        self.client.loop_start()
        threading.Thread(target=self._alive_loop, daemon=True).start()

    def stop(self):
        """끝낸다: 연결 신호 false 를 retained 로 남기고 나간다(깨끗한 종료에는 LWT 가 안 나가서 직접 보낸다)."""
        self._stop.set()
        if self.client.is_connected():
            info = self.client.publish(WEB_ALIVE_TOPIC, alive_payload(False), qos=1, retain=True)
            try:
                info.wait_for_publish(CLOSE_WAIT_S)
            except (TypeError, RuntimeError, ValueError):
                pass
        self.client.disconnect()
        self.client.loop_stop()

    def connected(self):
        """브로커에 붙어 있나."""
        return self.client.is_connected()

    def bridge_connected(self):
        """로봇 PC 다리의 연결 신호가 lost_after_s 안에 alive true 로 왔나(IRD 10.3 — 끊기면 화면 배너 · [출발] · [스캔] 막음)."""
        with self._lock:
            return self._fresh()

    def _fresh(self):
        """(잠금 안) 다리 연결 신호가 살아 있나."""
        return self._bridge_alive and self._bridge_seen_at is not None and \
            self.clock() - self._bridge_seen_at < self.lost_after_s

    def snapshot(self):
        """지금 값 전부(화면이 처음 붙을 때 · GET /api/robot/state): 상태 5종 + 브로커 · 다리 연결."""
        with self._lock:
            out = {k: v for k, v in self.state.items()}
            out['bridge_alive'] = self._fresh()
        out['broker'] = self.connected()
        return out

    def _on_connect(self, client, _userdata, _flags, rc):
        """(paho 스레드) 붙으면 상태 · 다리 연결 신호 · 의도 · 응답 · 로봇 요청 토픽을 구독한다."""
        if rc != 0:
            return
        topics = list(STATE_TOPICS) + [BRIDGE_ALIVE_TOPIC, INTENT_TOPIC] + \
            [res_topic(n) for n in WEB_CALLS] + [req_topic(n) for n in ROBOT_CALLS]
        client.subscribe([(t, 1) for t in topics])
        self._emit('broker', {'connected': True})

    def _alive_loop(self):
        """웹 연결 신호를 alive_s 마다 보내고, 다리 연결 신호가 끊겼는지 본다(끊기면 화면에 bridge_alive false)."""
        while not self._stop.wait(self.alive_s):
            if self.client.is_connected():
                self.client.publish(WEB_ALIVE_TOPIC, alive_payload(True), qos=1, retain=True)
            self.check_bridge()

    def check_bridge(self):
        """다리 연결 신호가 lost_after_s 넘게 없으면 끊김으로 바꾸고 알린다(alive 토픽이 아예 안 오는 경우 — LWT 전)."""
        with self._lock:
            lost = self._bridge_alive and not self._fresh()
            if lost:
                self._bridge_alive = False
        if lost:
            self._emit('bridge_alive', {'alive': False})

    # ---------- 화면 요청 ----------
    def request(self, ros_name, fields):
        """웹 → 로봇 요청(…/req)을 보내고 …/res 를 req_timeout_s 동안 기다린다. 반환: 응답 dict(req_id 뺌).

        브로커가 없으면 바로 {success False, reason ERROR}, 시간이 지나면 {success False, reason TIMEOUT}. 자동 재전송 없음(IRD 10.1).
        """
        if not self.client.is_connected():
            return {'success': False, 'reason': 'ERROR', 'message': '브로커 연결 없음'}
        req_id = str(uuid.uuid4())
        self.pending.open(req_id)
        self.client.publish(req_topic(ros_name), json.dumps({**fields, 'req_id': req_id}, ensure_ascii=False), qos=1)
        body = self.pending.wait(req_id, self.req_timeout_s)
        if body is None:
            return {'success': False, 'reason': 'TIMEOUT', 'message': f'{self.req_timeout_s:g}초 안에 답이 없다 — 실제 상태는 화면 상태 줄로'}
        return {k: v for k, v in body.items() if k != 'req_id'}

    def serve(self, ros_name, fn):
        """로봇이 부르는 요청(get_design · save_build)에 답할 함수를 등록한다(저장소 W111). fn(body) → 응답 dict."""
        self.handlers[ros_name] = fn

    # ---------- 받기 ----------
    def _on_message(self, client, _userdata, msg):
        """(paho 스레드) 토픽마다 나눈다. 깨진 JSON · 빈 payload(retained 지우기)는 버린다. 예외는 삼켜 paho 스레드를 살린다."""
        if not msg.payload:
            return
        try:
            if msg.topic in STATE_TOPICS:
                self._on_state(STATE_TOPICS[msg.topic], json.loads(msg.payload))
            elif msg.topic == BRIDGE_ALIVE_TOPIC:
                self._on_bridge_alive(json.loads(msg.payload))
            elif msg.topic == INTENT_TOPIC:
                self._emit('intent', json.loads(msg.payload))
            elif msg.topic.endswith('/res'):
                body = parse_request(msg.payload)
                self.pending.resolve(body['req_id'], body)
            elif msg.topic.endswith('/req') and not msg.retain:   # retained 로 남은 옛 요청에는 답하지 않는다(다리와 같은 규칙)
                self._answer_robot(client, msg.topic, parse_request(msg.payload))
        except (ValueError, KeyError, TypeError):
            pass

    def _on_state(self, kind, data):
        """상태 JSON 을 들고 있고 화면에 알린다. 객체가 아니면 버린다(판단 · 고치기 없음)."""
        if not isinstance(data, dict):
            return
        with self._lock:
            self.state[kind] = data
        self._emit(kind, data)

    def _on_bridge_alive(self, data):
        """다리 연결 신호. alive 가 정확히 true 일 때만 수신 시각을 갱신(받은 시각 기준 — 두 PC 시계를 맞추지 않는다)."""
        alive = isinstance(data, dict) and data.get('alive') is True
        with self._lock:
            changed = alive != self._fresh()
            self._bridge_alive = alive
            if alive:
                self._bridge_seen_at = self.clock()
        if changed:
            self._emit('bridge_alive', {'alive': alive})

    def _answer_robot(self, client, topic, body):
        """로봇이 부른 요청에 등록된 함수로 답한다. 같은 req_id 두 번째는 버리고, 함수가 없거나 실패하면 success false · ERROR."""
        if not self.seen.first(body['req_id']):
            return
        name = '/' + topic[:-len('/req')]
        fn = self.handlers.get(name)
        try:
            out = fn(body) if fn else {'success': False, 'reason': 'ERROR', 'message': f'{name} 를 답할 저장소가 아직 없다'}
        except Exception as e:   # noqa: BLE001 — 저장소 오류도 로봇에는 실패로 알린다(다리가 TIMEOUT 까지 기다리지 않게)
            out = {'success': False, 'reason': 'ERROR', 'message': repr(e)}
        client.publish(res_topic(name), json.dumps({**out, 'req_id': body['req_id']}, ensure_ascii=False), qos=1)

    def _emit(self, kind, data):
        """화면 이벤트 {"type", "data"} 를 on_event 에 넘긴다(paho · 연결 신호 스레드에서 불림 — 받는 쪽이 스레드 안전해야 함)."""
        if self.on_event:
            self.on_event({'type': kind, 'data': data})

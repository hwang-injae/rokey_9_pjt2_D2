# -*- coding: utf-8 -*-
"""다리 노드 bridge — 로봇 PC 의 ROS 2 ↔ 웹 PC 의 MQTT 브로커를 잇는 유일한 곳 (IRD 10장, E-27 ~ E-29, W127).

웹 PC 에는 ROS 가 없다. 이 노드가 웹 대신 /d2/hmi/* 이름을 쓰고, 로봇 PC 의 다른 노드는 MQTT 를 모른다.
변환만 한다 — 멈출지 · 진행표 · 설계 검사 판단은 하지 않는다. 이 노드가 죽어도 로봇 PC 안 정지(키 · Ctrl+C · 알람 · 펜던트)는 그대로다(E-30 ⓑ).
로봇을 직접 움직이지 않는다(출발 · 정지 요청을 작업 관리자 · 정지 노드에 전달만 한다) — 그래서 서기 궤적 처리는 없다.
"""
import os
import threading
import uuid
from functools import partial

import paho.mqtt.client as mqtt
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from d2_interfaces.srv import HmiCommand, JsonQuery, StopRequest
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from d2_bridge.bridge_codec import (PendingReplies, SeenIds, alive_payload, mqtt_topic, parse_request,
                                    query_from_mqtt_response, query_mqtt_request, query_request_json, query_response,
                                    req_topic, res_topic, string_fields, typed_response)

# 상태 토픽을 내는 쪽(task · 정지 노드)과 같아야 받는다. VOLATILE 로 내는 토픽을 이것으로 구독하면 아무것도 못 받는다(IRD 4.1)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
# 로봇 → 웹 상태 토픽: (ROS 이름, ROS 구독 QoS, MQTT QoS). 모두 retained — 늦게 붙은 화면도 마지막 값을 받는다(IRD 10.1)
STATE_TOPICS = (
    ('/d2/task/state', LATCHED_QOS, 1),
    ('/d2/task/progress', LATCHED_QOS, 1),
    ('/d2/task/scan_result', LATCHED_QOS, 1),
    ('/d2/safety/state', LATCHED_QOS, 1),
    ('/d2/gripper/state', 10, 0),       # 1 Hz 로 계속 와서 하나 잃어도 다음 것이 곧 온다 — QoS 0 (IRD 10.1)
)
# 웹이 부르는 서비스: ROS 이름 → (형식, 요청 글자 칸, 응답 칸). JsonQuery 는 칸 대신 객체를 그대로 옮긴다(IRD 10.1)
WEB_CALLS = {
    '/d2/hmi/command': (HmiCommand, ('cmd', 'design_id', 'mode'), ('success', 'reason')),
    '/d2/safety/stop': (StopRequest, ('source', 'reason'), ('success', 'message')),
    '/d2/safety/resume': (Trigger, (), ('success', 'message')),
    '/d2/task/check_design': (JsonQuery, None, None),
}
ROBOT_CALLS = ('/d2/hmi/get_design', '/d2/hmi/save_build')   # 로봇 노드가 부르고 웹 backend(design_store)가 답한다
INTENT = '/d2/hmi/intent'
HMI_ALIVE = '/d2/hmi/alive'
WEB_ALIVE_TOPIC = 'd2/web/alive'
BRIDGE_ALIVE_TOPIC = 'd2/bridge/alive'
CLIENT_ID = 'd2_bridge'       # 다리는 하나뿐 — 같은 이름으로 다시 붙으면 브로커가 옛 연결을 끊는다
MQTT_KEEPALIVE_S = 5          # 다리 PC 가 갑자기 꺼지면 약 1.5배 뒤 브로커가 LWT(alive false)를 남긴다. 화면은 그 전에 연결 신호 3초로 먼저 안다
CLOSE_WAIT_S = 1.0            # 끝낼 때 'alive false' 가 나갈 때까지 기다리는 한도


def load_robot_yaml():
    """d2_bringup 의 config/robot.yaml 을 읽는다. 없으면 예외 — 브로커 주소 없이 켜지 않는다."""
    path = os.path.join(get_package_share_directory('d2_bringup'), 'config', 'robot.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def make_mqtt_client(client_id):
    """paho 클라이언트를 만든다. 우분투 24.04 apt 의 1.6 과 pip 의 2.x 둘 다 같은 콜백 꼴(VERSION1)로 쓴다."""
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    except AttributeError:
        return mqtt.Client(client_id=client_id)


class MqttBridge(Node):
    """ROS ↔ MQTT 변환 하나(IRD 10.1 표).

    로봇 → 웹: 상태 토픽 5개 → MQTT retained, 연결 신호 d2/bridge/alive(alive_s 마다, LWT alive false).
    웹 → 로봇: d2/hmi/intent → /d2/hmi/intent, d2/web/alive → /d2/hmi/alive, …/req → 서비스 호출 → …/res(req_id 그대로).
    로봇이 부름: /d2/hmi/get_design · save_build 를 제공하고, MQTT …/req 로 웹에 묻고 …/res 를 req_timeout_s 동안 기다린다.
    실패: 브로커가 없으면 혼자 다시 붙는다. 서비스가 안 떠 있거나 요청이 틀리면 success false 로 답한다(자동 재시도 없음).
    """

    def __init__(self):
        """robot.yaml mqtt.* 를 읽고 ROS 쪽 구독 · 발행 · 서비스를 연다. 브로커 연결은 start() 에서 시작한다."""
        super().__init__('bridge')
        self.declare_parameter('mqtt_host', '')    # 비우면 robot.yaml mqtt.host. W129 처럼 웹 PC IP 를 실행 때 줄 때 쓴다
        cfg = load_robot_yaml()['mqtt']
        self.host = self.get_parameter('mqtt_host').value or cfg['host']
        self.port = int(cfg['port'])
        self.qos = int(cfg['qos'])
        self.req_timeout_s = float(cfg['req_timeout_s'])
        self.seen, self.pending = SeenIds(), PendingReplies()
        self.last_state = {}         # MQTT 토픽 → (payload, qos). 다시 붙으면 다시 보낸다 — 브로커가 재시작해 retained 를 잃어도 화면이 상태를 받게
        self._lock = threading.Lock()
        cb = ReentrantCallbackGroup()   # get_design · save_build 응답을 기다리는 동안 상태 전달 · 연결 신호가 막히지 않게

        for name, ros_qos, mqtt_qos in STATE_TOPICS:
            self.create_subscription(String, name, partial(self._on_state, mqtt_topic(name), mqtt_qos), ros_qos,
                                     callback_group=cb)
        self.intent_pub = self.create_publisher(String, INTENT, 10)
        self.alive_pub = self.create_publisher(String, HMI_ALIVE, 10)
        self.calls = {}              # MQTT 요청 토픽 → (ROS 이름, 클라이언트, 형식, 요청 칸, 응답 칸)
        for name, (srv, in_fields, out_fields) in WEB_CALLS.items():
            self.calls[req_topic(name)] = (name, self.create_client(srv, name, callback_group=cb), srv, in_fields, out_fields)
        for name in ROBOT_CALLS:
            self.create_service(JsonQuery, name, partial(self._ask_web, name), callback_group=cb)
        self.replies = {res_topic(name) for name in ROBOT_CALLS}

        self.mqtt = make_mqtt_client(CLIENT_ID)
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_disconnect = self._on_disconnect
        self.mqtt.on_message = self._on_message
        self.mqtt.will_set(BRIDGE_ALIVE_TOPIC, alive_payload(False), qos=self.qos, retain=True)
        self.mqtt.reconnect_delay_set(min_delay=1, max_delay=5)
        self.create_timer(float(cfg['alive_s']), self._send_alive, callback_group=cb)

    # ---------- 브로커 연결 ----------
    def start(self):
        """브로커에 붙기 시작한다(기다리지 않음). 브로커가 아직 없으면 paho 가 1~5초 간격으로 다시 붙는다."""
        self.get_logger().info(f'[다리] 브로커 {self.host}:{self.port} 에 붙는 중')
        self.mqtt.connect_async(self.host, self.port, keepalive=MQTT_KEEPALIVE_S)
        self.mqtt.loop_start()

    def close(self):
        """끝낸다: 연결 신호 false 를 retained 로 남기고 브로커에서 나간다(깨끗한 종료에는 LWT 가 안 나가서 직접 보낸다)."""
        if self.mqtt.is_connected():
            info = self.mqtt.publish(BRIDGE_ALIVE_TOPIC, alive_payload(False), qos=self.qos, retain=True)
            try:
                info.wait_for_publish(CLOSE_WAIT_S)
            except (TypeError, RuntimeError, ValueError):
                pass
        self.mqtt.disconnect()
        self.mqtt.loop_stop()

    def _on_connect(self, client, _userdata, _flags, rc):
        """(paho 스레드) 붙으면 요청 · 의도 · 웹 연결 신호 · 응답 토픽을 구독하고, 들고 있던 상태와 연결 신호를 다시 보낸다."""
        if rc != 0:
            self.get_logger().warn(f'[다리] 브로커 연결 거절 rc={rc}')
            return
        topics = list(self.calls) + list(self.replies) + [mqtt_topic(INTENT), WEB_ALIVE_TOPIC]
        client.subscribe([(t, self.qos) for t in topics])
        with self._lock:
            states = list(self.last_state.items())
        for topic, (payload, qos) in states:
            client.publish(topic, payload, qos=qos, retain=True)
        self._send_alive()
        self.get_logger().info(f'[다리] 브로커 연결됨 — 구독 {len(topics)}개, 상태 {len(states)}개 다시 보냄')

    def _on_disconnect(self, _client, _userdata, rc):
        """(paho 스레드) 끊김을 알린다. 뜻하지 않은 끊김이면 paho 가 혼자 다시 붙는다."""
        if rc != 0:
            self.get_logger().warn(f'[다리] 브로커 연결 끊김 rc={rc} — 다시 붙는 중. 그동안 화면 정지는 안 닿는다(키 · 펜던트)')

    def _publish(self, topic, payload, qos, retain=False):
        """MQTT 로 보낸다. 끊겨 있으면 버린다(쌓아 두면 다시 붙을 때 옛 값이 몰려 나간다 — 상태는 _on_connect 가 마지막 값만 다시 보냄)."""
        if not self.mqtt.is_connected():
            return False
        return self.mqtt.publish(topic, payload, qos=qos, retain=retain).rc == mqtt.MQTT_ERR_SUCCESS

    def _send_alive(self):
        """다리 연결 신호 d2/bridge/alive 를 retained 로 보낸다(alive_s 마다). 화면은 3초 끊기면 '로봇 PC 연결 끊김'(IRD 10.3)."""
        self._publish(BRIDGE_ALIVE_TOPIC, alive_payload(True), self.qos, retain=True)

    # ---------- 로봇 → 웹 ----------
    def _on_state(self, topic, mqtt_qos, msg):
        """상태 JSON 글자를 내용 그대로 MQTT retained 로 옮긴다(판단 없음). 마지막 값은 다시 붙을 때를 위해 들고 있는다."""
        with self._lock:
            self.last_state[topic] = (msg.data, mqtt_qos)
        self._publish(topic, msg.data, mqtt_qos, retain=True)

    # ---------- 웹 → 로봇 ----------
    def _on_message(self, _client, _userdata, msg):
        """(paho 스레드) 토픽마다 나눈다. 예외는 여기서 잡아 로그만 남긴다 — paho 스레드가 죽으면 다리 전체가 멈춘다."""
        try:
            if not msg.payload:
                return                 # 빈 payload = retained 지우기 — 옮길 내용이 없다
            if msg.topic == WEB_ALIVE_TOPIC:
                self._forward(self.alive_pub, msg.payload)
            elif msg.retain:
                # 요청 · 의도가 retained 로 남아 있으면 다리가 다시 붙을 때마다 옛 [출발] 이 다시 들어간다 — 버린다
                self.get_logger().warn(f'[다리] retained 로 남은 {msg.topic} 를 버림(요청 · 의도는 retained 로 보내지 않는다)')
            elif msg.topic == mqtt_topic(INTENT):
                self._forward(self.intent_pub, msg.payload)
            elif msg.topic in self.calls:
                self._call_robot(msg.topic, msg.payload)
            elif msg.topic in self.replies:
                self._on_reply(msg.payload)
        except Exception as e:   # noqa: BLE001 — 어떤 오류든 paho 스레드를 살려 둔다
            self.get_logger().error(f'[다리] {msg.topic} 처리 실패: {e!r}')

    def _forward(self, pub, payload):
        """MQTT payload(JSON 글자)를 내용 그대로 ROS String 토픽으로 낸다. UTF-8 이 아니면 버린다."""
        try:
            pub.publish(String(data=payload.decode('utf-8')))
        except UnicodeDecodeError:
            self.get_logger().warn('[다리] UTF-8 이 아닌 payload 를 버림')

    def _call_robot(self, topic, payload):
        """웹 요청(…/req)을 ROS 서비스로 부르고, 답은 _on_robot_answer 가 …/res 로 보낸다. 같은 req_id 두 번째는 버린다."""
        name, client, srv, in_fields, out_fields = self.calls[topic]
        try:
            body = parse_request(payload)
        except ValueError as e:
            self.get_logger().warn(f'[다리] {topic} 요청을 버림(답할 req_id 없음): {e}')
            return
        req_id = body['req_id']
        if not self.seen.first(req_id):
            self.get_logger().info(f'[다리] {topic} 같은 req_id {req_id} 두 번째 — 버림')
            return
        if not client.service_is_ready():
            self._reply_fail(name, req_id, f'{name} 서비스가 안 떠 있다')
            return
        req = srv.Request()
        try:
            if srv is JsonQuery:
                req.request_json = query_request_json(body)
            else:
                for key, value in string_fields(body, in_fields).items():
                    setattr(req, key, value)
        except ValueError as e:
            self._reply_fail(name, req_id, str(e))
            return
        client.call_async(req).add_done_callback(partial(self._on_robot_answer, name, req_id, srv, out_fields))

    def _on_robot_answer(self, name, req_id, srv, out_fields, future):
        """(ROS 실행기 스레드) 서비스 답을 MQTT …/res 로 보낸다. JsonQuery 는 response_json 객체를 펼친다(IRD 10.1)."""
        try:
            res = future.result()
        except Exception as e:   # noqa: BLE001 — 서비스 쪽 예외도 웹에는 실패로 알린다
            self._reply_fail(name, req_id, f'서비스 오류: {e!r}')
            return
        if srv is JsonQuery:
            payload = query_response(req_id, res.success, res.reason, res.response_json)
        else:
            payload = typed_response(req_id, **{f: getattr(res, f) for f in out_fields})
        self._publish(res_topic(name), payload, self.qos)

    def _reply_fail(self, name, req_id, message):
        """다리에서 막힌 요청을 success false · reason ERROR + 글로 답한다(자동 재시도 없음 — 사람이 다시 누른다)."""
        self.get_logger().warn(f'[다리] {name} 실패: {message}')
        self._publish(res_topic(name), typed_response(req_id, success=False, reason='ERROR', message=message), self.qos)

    # ---------- 로봇이 부름 ----------
    def _ask_web(self, name, req, res):
        """(ROS 실행기 스레드) get_design · save_build 요청을 MQTT 로 웹에 묻고 req_timeout_s 동안 답을 기다린다.

        출력: 웹 응답의 success · reason · 나머지 칸(response_json). 브로커 없음 · 요청 오류는 ERROR, 답이 안 오면 TIMEOUT.
        """
        req_id = str(uuid.uuid4())
        try:
            payload = query_mqtt_request(req_id, req.request_json)
        except ValueError as e:
            self.get_logger().warn(f'[다리] {name} 요청 오류: {e}')
            res.success, res.reason = False, 'ERROR'
            return res
        if not self.mqtt.is_connected():
            self.get_logger().warn(f'[다리] {name}: 브로커 연결 없음')
            res.success, res.reason = False, 'ERROR'
            return res
        self.pending.open(req_id)
        self._publish(req_topic(name), payload, self.qos)
        body = self.pending.wait(req_id, self.req_timeout_s)
        if body is None:
            self.get_logger().warn(f'[다리] {name}: 웹 응답이 {self.req_timeout_s} 초 안에 안 옴')
            res.success, res.reason = False, 'TIMEOUT'
            return res
        res.success, res.reason, res.response_json = query_from_mqtt_response(body)
        return res

    def _on_reply(self, payload):
        """(paho 스레드) 웹 응답(…/res)을 기다리는 _ask_web 에 넘긴다. 시간이 지난 뒤 온 응답 · 모르는 req_id 는 버린다."""
        try:
            body = parse_request(payload)
        except ValueError as e:
            self.get_logger().warn(f'[다리] 응답을 버림: {e}')
            return
        if not self.pending.resolve(body['req_id'], body):
            self.get_logger().info(f"[다리] 늦거나 모르는 응답 req_id {body['req_id']} — 버림")


def main():
    """다리를 켠다. 로봇을 움직이지 않으므로 rclpy 기본 Ctrl+C 처리를 쓴다 — 끝낼 때 연결 신호 false 를 남기고 브로커에서 나간다."""
    rclpy.init()
    node = MqttBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

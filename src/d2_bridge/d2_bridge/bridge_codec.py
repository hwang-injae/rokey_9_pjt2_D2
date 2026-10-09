# -*- coding: utf-8 -*-
"""다리 계산 — ROS 이름 ↔ MQTT 토픽, MQTT 요청 · 응답 JSON 만들기 · 읽기, 같은 req_id 거르기, 웹 응답 기다리기 (IRD 10장).

ROS · paho 를 import 하지 않는다 — tests/test_bridge_codec.py 가 로봇 없이 시험한다(CI 는 d2_bridge 를 colcon 빌드하지 않는다).
다리는 변환만 한다: 칸의 뜻을 판단하지 않고 JSON 내용은 그대로 옮긴다(IRD 10.1 규칙). 틀린 요청은 고치지 않고 거절한다.
"""
import json
import threading
import time
from collections import deque

SEEN_KEEP = 256     # 기억하는 req_id 개수. 요청은 사람이 버튼을 누를 때만 와서 몇 시간 치로도 충분하다
ROS_RESPONSE_KEYS = ('req_id', 'success', 'reason')   # JsonQuery 응답의 ROS 칸 — 나머지 칸은 response_json 객체를 펼친 것


def mqtt_topic(ros_name):
    """ROS 이름 → MQTT 토픽. 맨 앞 '/'만 뺀다(IRD 10.1 규칙). 예: '/d2/task/state' → 'd2/task/state'."""
    return ros_name[1:] if ros_name.startswith('/') else ros_name


def req_topic(ros_name):
    """서비스의 MQTT 요청 토픽. 예: '/d2/hmi/command' → 'd2/hmi/command/req'."""
    return mqtt_topic(ros_name) + '/req'


def res_topic(ros_name):
    """서비스의 MQTT 응답 토픽. 예: '/d2/hmi/command' → 'd2/hmi/command/res'."""
    return mqtt_topic(ros_name) + '/res'


def alive_payload(alive, stamp=None):
    """연결 신호 JSON 글자 {"alive", "stamp"}(stamp = 보낸 시각, 초). LWT(갑자기 끊김)는 alive False 로 만든다(IRD 10.3)."""
    return json.dumps({'alive': bool(alive), 'stamp': round(time.time() if stamp is None else stamp, 3)})


def parse_request(payload):
    """MQTT 요청 payload(바이트 · 글자) → dict.

    받는 것: 비어 있지 않은 글자 req_id 가 있는 JSON 객체(IRD 10.1).
    실패: 깨진 JSON · 객체 아님 · req_id 없음 → ValueError. 답할 req_id 가 없으면 부른 쪽이 응답을 짝지을 수 없어 버린다.
    """
    text = payload.decode('utf-8') if isinstance(payload, (bytes, bytearray)) else payload
    body = json.loads(text)
    if not isinstance(body, dict):
        raise ValueError('요청이 JSON 객체가 아니다')
    req_id = body.get('req_id')
    if not isinstance(req_id, str) or not req_id:
        raise ValueError('req_id 가 없다')
    return body


def string_fields(body, names):
    """요청 dict 에서 ROS 서비스의 글자 칸만 뽑는다. 없는 칸은 빈 글자(예: 정지 reason 을 비워 보내면 정지 노드가 STOP_WEB 으로 적음).

    출력: {칸 이름: 글자}. 실패: 글자가 아닌 값 → ValueError(다리가 형식을 바꿔 주지 않는다).
    """
    out = {}
    for name in names:
        value = body.get(name, '')
        if value is None:
            value = ''
        if not isinstance(value, str):
            raise ValueError(f'{name} 이 글자가 아니다')
        out[name] = value
    return out


def query_request_json(body):
    """웹 → 로봇 JsonQuery 요청(check_design): req_id 를 뺀 객체를 request_json 글자로(IRD 10.1 — MQTT 요청 = request_json 객체 + req_id)."""
    return json.dumps({k: v for k, v in body.items() if k != 'req_id'}, ensure_ascii=False)


def typed_response(req_id, **fields):
    """전용 서비스(HmiCommand · StopRequest · Trigger) 응답 → MQTT 응답 JSON 글자 {"req_id", 응답 칸들}."""
    return json.dumps({'req_id': req_id, **fields}, ensure_ascii=False)


def query_response(req_id, success, reason, response_json):
    """JsonQuery 응답 → MQTT 응답 JSON 글자: ROS 칸(req_id · success · reason) + response_json 객체를 펼친 것(IRD 10.1).

    response_json 이 비면 펼칠 것이 없다. 깨졌거나 객체가 아니면 내용을 고치지 않고 success False · reason ERROR 로 알린다.
    객체 안에 ROS 칸 이름이 또 있으면 ROS 칸이 이긴다(부른 쪽이 success 를 잘못 읽지 않게).
    """
    extra = {}
    if response_json:
        try:
            extra = json.loads(response_json)
        except ValueError:
            extra = None
        if not isinstance(extra, dict):
            return json.dumps({'req_id': req_id, 'success': False, 'reason': 'ERROR',
                               'message': '응답 JSON 이 객체가 아니다'}, ensure_ascii=False)
    out = {k: v for k, v in extra.items() if k not in ROS_RESPONSE_KEYS}
    out.update(req_id=req_id, success=bool(success), reason=reason or '')
    return json.dumps(out, ensure_ascii=False)


def query_mqtt_request(req_id, request_json):
    """로봇 → 웹 JsonQuery(get_design · save_build): request_json 의 객체 + req_id → MQTT 요청 JSON 글자.

    실패: request_json 이 JSON 객체가 아니면 ValueError(웹에 보내지 않고 부른 노드에 바로 실패로 답한다).
    """
    body = json.loads(request_json) if request_json else {}
    if not isinstance(body, dict):
        raise ValueError('request_json 이 JSON 객체가 아니다')
    return json.dumps({**body, 'req_id': req_id}, ensure_ascii=False)


def query_from_mqtt_response(body):
    """웹의 MQTT 응답 dict → (success, reason, response_json 글자). ROS 칸을 뺀 나머지를 response_json 으로 다시 묶는다.

    save_build 의 두 층(서비스 success = 요청 처리 · 응답 ok = DB 저장, IRD 4.2)은 ok 를 response_json 에 그대로 둔다.
    """
    rest = {k: v for k, v in body.items() if k not in ROS_RESPONSE_KEYS}
    reason = body.get('reason')
    return body.get('success') is True, reason if isinstance(reason, str) else '', json.dumps(rest, ensure_ascii=False)


class SeenIds:
    """최근 req_id 를 기억해 같은 요청이 두 번 오면 두 번째를 거른다(IRD 10.1 '같은 req_id 가 두 번 오면 두 번째는 버린다').

    MQTT QoS 1 은 같은 메시지를 두 번 줄 수 있다 — [출발]이 두 번 들어가지 않게 한다. 스레드 하나(paho)에서만 부른다.
    """

    def __init__(self, keep=SEEN_KEEP):
        """keep = 기억하는 개수. 넘으면 가장 오래된 것부터 잊는다."""
        self._order = deque()
        self._ids = set()
        self._keep = keep

    def first(self, req_id):
        """처음 보는 req_id 면 기억하고 True, 이미 본 것이면 False."""
        if req_id in self._ids:
            return False
        self._ids.add(req_id)
        self._order.append(req_id)
        if len(self._order) > self._keep:
            self._ids.discard(self._order.popleft())
        return True


class PendingReplies:
    """로봇 노드가 부른 요청(get_design · save_build)의 웹 응답을 req_id 로 기다린다.

    paho 스레드가 resolve 하고 ROS 실행기 스레드가 wait 한다 — 잠금으로 보호한다. 기다리지 않는 req_id(늦은 응답 · 모르는 것)는 버린다.
    """

    def __init__(self):
        """비어 있는 대기표를 만든다."""
        self._lock = threading.Lock()
        self._waiting = {}

    def open(self, req_id):
        """보내기 전에 req_id 를 대기표에 올린다(응답이 wait 보다 먼저 와도 잃지 않게)."""
        with self._lock:
            self._waiting[req_id] = [threading.Event(), None]

    def resolve(self, req_id, body):
        """응답 dict 를 넘긴다. 기다리는 req_id 면 True, 아니면(늦음 · 모름) False — 버린다."""
        with self._lock:
            slot = self._waiting.get(req_id)
            if slot is None:
                return False
            slot[1] = body
            slot[0].set()
            return True

    def wait(self, req_id, timeout_s):
        """timeout_s 초 안에 온 응답 dict, 안 오면 None. 어느 쪽이든 대기표에서 지운다(늦게 오면 resolve 가 버린다)."""
        with self._lock:
            slot = self._waiting.get(req_id)
        if slot is None:
            return None
        slot[0].wait(timeout_s)
        with self._lock:
            self._waiting.pop(req_id, None)
        return slot[1]

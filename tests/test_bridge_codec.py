"""다리 변환(d2_bridge/bridge_codec.py)이 IRD 10장 규칙대로 토픽 이름 · 요청 · 응답 JSON 을 만들고, 틀린 요청을 고치지 않고 거절하는지 지킨다.

ROS · paho 없이 돈다(CI 는 d2_bridge 를 colcon 빌드하지 않는다). bridge_codec 이 ROS · paho 를 import 하지 않는 것도 본다.
"""
import json
import sys
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'd2_bridge'))

from d2_bridge import bridge_codec as bc  # noqa: E402

CODEC_PY = ROOT / 'src' / 'd2_bridge' / 'd2_bridge' / 'bridge_codec.py'


def test_codec_has_no_ros_or_paho_import():
    """변환 파일은 ROS · paho 없이 돈다 — 그래야 이 시험이 CI 에서 돈다."""
    text = CODEC_PY.read_text(encoding='utf-8')
    for name in ('rclpy', 'paho', 'd2_interfaces', 'std_msgs'):
        assert f'import {name}' not in text and f'from {name}' not in text


@pytest.mark.parametrize('ros_name, topic', [
    ('/d2/task/state', 'd2/task/state'),
    ('/d2/task/progress', 'd2/task/progress'),
    ('/d2/task/scan_result', 'd2/task/scan_result'),
    ('/d2/safety/state', 'd2/safety/state'),
    ('/d2/gripper/state', 'd2/gripper/state'),
    ('/d2/hmi/intent', 'd2/hmi/intent'),
])
def test_topic_is_ros_name_without_leading_slash(ros_name, topic):
    """IRD 10.1: MQTT 토픽 = ROS 이름에서 맨 앞 '/'만 뺀 것."""
    assert bc.mqtt_topic(ros_name) == topic


@pytest.mark.parametrize('ros_name', ['/d2/hmi/command', '/d2/safety/stop', '/d2/safety/resume',
                                      '/d2/task/check_design', '/d2/hmi/get_design', '/d2/hmi/save_build'])
def test_service_topics_are_req_and_res(ros_name):
    """IRD 10.1: 서비스 = <토픽>/req · <토픽>/res."""
    assert bc.req_topic(ros_name) == ros_name[1:] + '/req'
    assert bc.res_topic(ros_name) == ros_name[1:] + '/res'


def test_alive_payload():
    """연결 신호 = {"alive", "stamp"} — LWT 는 alive false."""
    body = json.loads(bc.alive_payload(True, stamp=12.3456))
    assert body == {'alive': True, 'stamp': 12.346}
    assert json.loads(bc.alive_payload(False))['alive'] is False


@pytest.mark.parametrize('payload', [b'{not json', b'[1, 2]', b'"start"', b'{"cmd": "start"}', b'{"req_id": ""}',
                                     b'{"req_id": 3}'])
def test_parse_request_rejects_bad(payload):
    """깨진 JSON · 객체 아님 · req_id 없음 · 빈 req_id · 글자가 아닌 req_id 는 거절(답할 짝이 없음)."""
    with pytest.raises(ValueError):
        bc.parse_request(payload)


def test_parse_request_accepts_bytes_and_text():
    """바이트 · 글자 둘 다 받고 칸을 그대로 둔다."""
    body = {'req_id': 'a1', 'cmd': 'start', 'design_id': '001_CHAIR_BENCH', 'mode': 'auto'}
    assert bc.parse_request(json.dumps(body).encode()) == body
    assert bc.parse_request(json.dumps(body)) == body


def test_string_fields_defaults_and_rejects_non_text():
    """없는 칸 · null 은 빈 글자(정지 reason 을 비우면 정지 노드가 STOP_WEB), 글자가 아닌 값은 거절."""
    assert bc.string_fields({'req_id': 'x', 'source': 'web'}, ('source', 'reason')) == {'source': 'web', 'reason': ''}
    assert bc.string_fields({'reason': None}, ('reason',)) == {'reason': ''}
    with pytest.raises(ValueError):
        bc.string_fields({'cmd': 1}, ('cmd',))


def test_query_request_drops_req_id_keeps_rest():
    """check_design 요청: req_id 만 빼고 blocks/2.0 객체 그대로(한글 포함)."""
    body = {'req_id': '9a', 'schema': 'blocks/2.0', 'design_id': 'chair_v1.1', 'family': 'chair',
            'blocks': [{'order': 1, 'role': 'LEG'}], 'note': '의자'}
    assert json.loads(bc.query_request_json(body)) == {k: v for k, v in body.items() if k != 'req_id'}
    assert '의자' in bc.query_request_json(body)


def test_typed_response():
    """전용 서비스 응답 = {"req_id", 응답 칸들} — HmiCommand 는 success · reason(빈 reason 도 그대로, #83)."""
    assert json.loads(bc.typed_response('3f', success=False, reason='')) == {'req_id': '3f', 'success': False, 'reason': ''}


def test_query_response_spreads_object():
    """JsonQuery 응답 = ROS 칸 + response_json 객체를 펼침(IRD 10.2 check_design 예시)."""
    rj = json.dumps({'schema': 'check_result/2.0', 'ok': True, 'min_margin_mm': 12.5, 'errors': []})
    out = json.loads(bc.query_response('9a', True, '', rj))
    assert out == {'req_id': '9a', 'success': True, 'reason': '', 'schema': 'check_result/2.0', 'ok': True,
                   'min_margin_mm': 12.5, 'errors': []}


def test_query_response_ros_fields_win_and_empty_ok():
    """객체 안의 success · reason · req_id 는 ROS 칸이 덮는다. response_json 이 비면 ROS 칸만."""
    out = json.loads(bc.query_response('r1', False, 'TIMEOUT', json.dumps({'success': True, 'req_id': 'x', 'ok': 1})))
    assert out == {'req_id': 'r1', 'success': False, 'reason': 'TIMEOUT', 'ok': 1}
    assert json.loads(bc.query_response('r2', False, 'ERROR', '')) == {'req_id': 'r2', 'success': False, 'reason': 'ERROR'}


@pytest.mark.parametrize('rj', ['{broken', '[1]', '"x"'])
def test_query_response_bad_json_is_error(rj):
    """response_json 이 깨졌거나 객체가 아니면 고치지 않고 success false · ERROR."""
    out = json.loads(bc.query_response('r3', True, '', rj))
    assert out['success'] is False and out['reason'] == 'ERROR' and out['req_id'] == 'r3'


def test_robot_called_round_trip_keeps_two_layers():
    """save_build: 요청 = build/1 객체 + req_id, 응답의 ok 는 response_json 에 그대로(두 층 — IRD 4.2)."""
    build = {'schema': 'build/1', 'run_id': 'R20261010_143512_a3f9', 'result': 'DONE'}
    req = json.loads(bc.query_mqtt_request('c2', json.dumps(build)))
    assert req == {**build, 'req_id': 'c2'}
    success, reason, rj = bc.query_from_mqtt_response({'req_id': 'c2', 'success': True, 'reason': '', 'ok': False,
                                                       'message': 'DB 쓰기 실패'})
    assert success is True and reason == ''
    assert json.loads(rj) == {'ok': False, 'message': 'DB 쓰기 실패'}


def test_robot_called_rejects_non_object_and_reads_failure():
    """request_json 이 객체가 아니면 보내지 않는다. 웹의 실패 응답은 success false · reason 그대로."""
    with pytest.raises(ValueError):
        bc.query_mqtt_request('c3', '[1]')
    success, reason, _ = bc.query_from_mqtt_response({'req_id': 'c3', 'success': False, 'reason': 'ERROR'})
    assert success is False and reason == 'ERROR'
    success, _, _ = bc.query_from_mqtt_response({'req_id': 'c4', 'success': 'yes'})
    assert success is False                                     # 참 거짓이 아니면 성공으로 보지 않는다


def test_seen_ids_drops_repeat_and_forgets_oldest():
    """같은 req_id 두 번째는 거른다. 기억 개수를 넘으면 가장 오래된 것을 잊는다."""
    seen = bc.SeenIds(keep=2)
    assert seen.first('a') and not seen.first('a')
    assert seen.first('b') and seen.first('c')
    assert seen.first('a')                                      # 'a' 는 밀려나 잊혔다
    assert not seen.first('c')


def test_pending_replies_resolve_wait_timeout():
    """기다리는 req_id 만 받는다. 시간이 지나면 None, 늦게 온 응답은 버린다."""
    pending = bc.PendingReplies()
    pending.open('p1')
    threading.Timer(0.05, pending.resolve, args=('p1', {'success': True})).start()
    assert pending.wait('p1', 2.0) == {'success': True}
    pending.open('p2')
    assert pending.wait('p2', 0.05) is None
    assert pending.resolve('p2', {'success': True}) is False    # 시간이 지난 뒤 온 응답
    assert pending.resolve('unknown', {}) is False
    assert pending.wait('never_opened', 0.01) is None

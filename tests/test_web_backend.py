"""웹 backend(web/backend — mqtt_client · design_store · routes, W126 · W111)가 IRD 6 · 10장대로 동작하는지 지킨다.

브로커 없이 가짜 paho 클라이언트로 돈다. 지키는 것: 요청 = …/req + req_id, 답을 req_id 로 짝지음 · 시간 초과 TIMEOUT · 자동 재전송 없음,
상태는 마지막 값 + 화면 이벤트, 다리 연결 신호 3초 끊김, retained 로 남은 옛 요청에는 답하지 않음, 로봇 PC 가 끊기면 출발 · 스캔을 막음,
손목 검출 그림 · 스캔 사진 · 점군(바이트)은 마지막 하나 + 번호만 알림 · 스캔 사진 · 점군은 직전 scan_result 와 짝, 저장소는 기본 설계 4개를 design/2.0 으로 등록 · 옛 형식 거절 · 같은 run_id 는 한 번만 저장.
"""
import json
import os
import sys
import tempfile
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / 'web' / 'backend', ROOT / 'src' / 'd2_bridge', ROOT / 'src' / 'd2_task'):
    sys.path.insert(0, str(p))

from design_gen import GenError  # noqa: E402
from mqtt_client import MqttClient  # noqa: E402

# app.py 는 불러오는 순간 저장소를 열고 기본 설계를 등록한다 — 시험이 실제 web/backend/data(웹 PC 데이터)를 바꾸지 않게 임시 폴더로
os.environ['DATA_DIR'] = tempfile.mkdtemp(prefix='d2_web_test_')


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
    out = mc.request('/d2/hmi/command', {'cmd': 'start', 'design_id': '001_CHAIR_BENCH_V000', 'mode': 'auto'})
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
    req = {'req_id': 'g1', 'design_id': '001_CHAIR_BENCH_V000'}
    mc._on_message(mc.client, None, msg('d2/hmi/get_design/req', req))
    mc._on_message(mc.client, None, msg('d2/hmi/get_design/req', req))
    mc._on_message(mc.client, None, msg('d2/hmi/get_design/req', {'req_id': 'g2', 'design_id': 'X'}, retain=True))
    assert mc.client.sent == [('d2/hmi/get_design/res', {'success': True, 'reason': '', 'schema': 'design/2.0',
                                                         'design_id': '001_CHAIR_BENCH_V000', 'req_id': 'g1'}, False)]


def test_snapshot_tells_screen_the_request_timeout(mc):
    """화면이 붙을 때 받는 값에 요청 시간 제한(robot.yaml mqtt.req_timeout_s)이 있다 — 화면 코드에 숫자를 두지 않게."""
    assert mc.snapshot()['timing'] == {'req_timeout_s': 0.3}


def test_wrist_image_kept_as_bytes_and_only_seq_announced(mc):
    """손목 검출 그림은 JSON 이 아닌 JPEG 바이트 — 마지막 한 장을 들고 /ws 에는 번호만 알린다. 구독은 QoS 0(IRD 10.1)."""
    mc._on_connect(mc.client, None, None, 0)
    assert 'd2/vision/wrist_image' in mc.client.subs
    assert mc.blob('wrist_image') is None
    mc._on_message(mc.client, None, msg('d2/vision/wrist_image', b'\xff\xd8jpeg-1'))
    mc._on_message(mc.client, None, msg('d2/vision/wrist_image', b'\xff\xd8jpeg-2'))
    assert mc.blob('wrist_image') == b'\xff\xd8jpeg-2'
    seqs = [e['data']['seq'] for e in mc.events if e['type'] == 'wrist_image']
    assert seqs == [1, 2] and mc.snapshot()['wrist_image']['seq'] == 2
    assert 'run_id' not in mc.snapshot()['wrist_image']        # 손목 그림은 스캔과 짝짓지 않는다


def test_scan_image_cloud_paired_with_scan_result_and_cleared_on_new_scan(mc):
    """스캔 사진 · 점군(IRD 10.5)은 QoS 1 로 구독하고, 직전 scan_result 의 run_id 를 붙인다.
    새 스캔이 오면 앞 스캔 것을 지운다 — 점군이 없는 스캔(cloud_path 선택 칸)에 옛 점군이 남아 보이지 않게."""
    mc._on_connect(mc.client, None, None, 0)
    assert {'d2/vision/scan_image', 'd2/vision/scan_cloud'} <= set(mc.client.subs)
    mc._on_message(mc.client, None, msg('d2/task/scan_result', {'schema': 'scan_result/1.2', 'run_id': 'R1'}))
    mc._on_message(mc.client, None, msg('d2/vision/scan_image', b'\xff\xd8scan-1'))
    mc._on_message(mc.client, None, msg('d2/vision/scan_cloud', b'ply\nR1'))
    snap = mc.snapshot()
    assert snap['scan_image']['run_id'] == 'R1' and snap['scan_image']['bytes'] == len(b'\xff\xd8scan-1')
    assert snap['scan_cloud']['run_id'] == 'R1' and mc.blob('scan_cloud') == b'ply\nR1'
    mc._on_message(mc.client, None, msg('d2/task/scan_result', {'schema': 'scan_result/1.2', 'run_id': 'R1'}))
    assert mc.blob('scan_image') == b'\xff\xd8scan-1'          # 같은 스캔이 다시 오면(다리 재연결) 그대로
    mc._on_message(mc.client, None, msg('d2/task/scan_result', {'schema': 'scan_result/1.2', 'run_id': 'R2'}))
    assert mc.blob('scan_image') is None and mc.blob('scan_cloud') is None
    assert 'scan_image' not in mc.snapshot() and 'scan_cloud' not in mc.snapshot()
    mc._on_message(mc.client, None, msg('d2/vision/scan_image', b'\xff\xd8scan-2'))
    assert mc.snapshot()['scan_image']['run_id'] == 'R2' and mc.snapshot()['scan_image']['seq'] == 2
    assert mc.blob('scan_cloud') is None                       # R2 는 점군 없음 — 옛 R1 점군을 내주지 않는다


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
        self.handlers, self.blobs = {}, {}

    def serve(self, name, fn):
        self.handlers[name] = fn

    def blob(self, kind):
        return self.blobs.get(kind)

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


RECIPES = ROOT / 'src' / 'd2_robot' / 'd2_bringup' / 'recipes'
BASES = ['001_CHAIR_BENCH_V000', '002_CHAIR_BACK_V000', '003_DESK_STAND_V000', '004_DESK_PEDESTAL_V000']


@pytest.fixture
def store(tmp_path):
    """임시 폴더 저장소 + 저장소의 기본 설계 레시피 4개 등록."""
    from design_store import DesignStore
    s = DesignStore(tmp_path, RECIPES, [75, 25, 15])
    s.registered = s.register_bases()
    return s


class FakeGen:
    """DesignGenerator 흉내 — 생성은 release 가 열릴 때까지 기다린다(작업이 '도는 중'인 때를 시험하려고)."""

    def __init__(self):
        self.release, self.saved = threading.Event(), []

    def classify(self, text):
        return {'template': '001_CHAIR_BENCH', 'reason': '벤치 모양'}

    def templates(self):
        return {'001_CHAIR_BENCH': '벤치', '002_CHAIR_BACK': '등받이 의자'}

    def generate(self, text, template, on_progress=None):
        self.release.wait(2)
        cand = {'idea': '안 0', 'blocks': {'schema': 'blocks/2.0', 'design_id': '001_CHAIR_BENCH_V001', 'blocks': []},
                'check': {'ok': True, 'reason': '', 'min_margin_mm': 9.0, 'errors': [], 'recipe': {'big': 1}, 'placements': {'big': 1}}}
        bad = {**cand, 'check': {**cand['check'], 'ok': False, 'errors': [{'block': 9, 'reason': 'CHECK_FAILED', 'detail': '받침 없음'}]}}
        on_progress('reading', {'design_id': '001_CHAIR_BENCH_V000'})
        on_progress('candidates', {'attempt': 1, 'design_id': '001_CHAIR_BENCH_V001',
                                   'candidates': [{k: cand[k] for k in ('idea', 'blocks')}, {k: bad[k] for k in ('idea', 'blocks')}]})
        on_progress('checked', {'attempt': 1, 'index': 0, 'check': cand['check']})
        return {'text': text, 'template': template, 'design_id': '001_CHAIR_BENCH_V001', 'parent_id': '001_CHAIR_BENCH_V000',
                'reference': ['001_CHAIR_BENCH_V000'], 'candidates': [cand, bad], 'attempts': 1, 'elapsed_s': 1.0}

    def save(self, result, index, made_by='web'):
        if not result['candidates'][index]['check']['ok']:
            raise ValueError('검사에 떨어진 후보는 고를 수 없다')
        self.saved.append((index, made_by))
        return {'design_id': result['design_id'], 'version': 'V001', 'parent_id': result['parent_id']}


def make_client(store, bridge=True, gen=None):
    """가짜 MQTT · 임시 저장소(· 가짜 생성기)로 앱을 만든다(lifespan 은 돌리지 않음)."""
    import app as web_app
    fake = FakeMqtt(bridge)
    return TestClient(web_app.create_app(mqtt=fake, rules=RULES, store=store, gen=gen or FakeGen())), fake


def test_generate_routes_job_flow(store):
    """Template 고르기 → 생성 시작(job, 한 번에 하나) → 진행 · 결과 조회(레시피는 화면에 안 보냄) → 고르기 저장 → 다시 고르기 거절."""
    gen = FakeGen()
    client, _ = make_client(store, gen=gen)
    assert client.post('/api/designs/template', json={'text': '벤치'}).json() == \
        {'success': True, 'template': '001_CHAIR_BENCH', 'reason': '벤치 모양',
         'templates': [{'id': '001_CHAIR_BENCH', 'label': '벤치'}, {'id': '002_CHAIR_BACK', 'label': '등받이 의자'}]}
    assert client.post('/api/designs/template', json={'text': '  '}).status_code == 400
    gen.classify = lambda text: (_ for _ in ()).throw(GenError('GEN_FAILED', 'OpenAI 크레딧이 없어요'))
    fail = client.post('/api/designs/template', json={'text': '벤치'}).json()
    assert fail['success'] is False and '크레딧' in fail['message'] and len(fail['templates']) == 2     # 사람이 직접 고르게
    job = client.post('/api/designs/generate', json={'text': '벤치', 'template': '001_CHAIR_BENCH'}).json()['job_id']
    assert client.post('/api/designs/generate', json={'text': '또', 'template': '001_CHAIR_BENCH'}).status_code == 409   # 하나씩
    assert client.post(f'/api/designs/generate/{job}/pick', json={'index': 0}).status_code == 409                      # 아직 도는 중
    gen.release.set()
    for _ in range(100):
        view = client.get(f'/api/designs/generate/{job}').json()
        if view['state'] != 'running':
            break
        threading.Event().wait(0.02)
    assert view['state'] == 'ready' and view['design_id'] == '001_CHAIR_BENCH_V001' and view['reference'] == ['001_CHAIR_BENCH_V000']
    assert [c['check']['ok'] for c in view['candidates']] == [True, False] and 'recipe' not in view['candidates'][0]['check']
    assert view['reading'] == ['001_CHAIR_BENCH_V000'] and view['attempt'] == 1
    assert client.post(f'/api/designs/generate/{job}/pick', json={'index': 1}).status_code == 409                      # 떨어진 후보
    saved = client.post(f'/api/designs/generate/{job}/pick', json={'index': 0, 'made_by': 'voice'}).json()
    assert saved == {'design_id': '001_CHAIR_BENCH_V001', 'version': 'V001', 'parent_id': '001_CHAIR_BENCH_V000', 'index': 0}
    assert gen.saved == [(0, 'voice')] and client.get(f'/api/designs/generate/{job}').json()['state'] == 'saved'
    assert client.post(f'/api/designs/generate/{job}/pick', json={'index': 0}).status_code == 409                      # 한 번만
    assert client.get('/api/designs/generate/nope').status_code == 404


def test_command_stop_resume_map_to_ird_names(store):
    """버튼 → IRD ROS 이름 · 칸: 명령은 /d2/hmi/command, 정지는 source web + 빈 reason(→ STOP_WEB), 다시 시작은 빈 칸."""
    client, fake = make_client(store)
    assert client.post('/api/robot/command', json={'cmd': 'select_design', 'design_id': '001_CHAIR_BENCH_V000'}).json()['success']
    client.post('/api/robot/stop')
    client.post('/api/robot/resume')
    assert fake.calls == [('/d2/hmi/command', {'cmd': 'select_design', 'design_id': '001_CHAIR_BENCH_V000', 'mode': 'auto'}),
                          ('/d2/safety/stop', {'source': 'web', 'reason': ''}),
                          ('/d2/safety/resume', {})]


def test_unknown_cmd_rejected_and_bridge_lost_blocks_start_scan_not_stop(store):
    """모르는 cmd 는 422. 로봇 PC 가 끊기면 출발 · 스캔은 보내지 않지만 정지 · 취소는 보낸다(IRD 10.3)."""
    client, fake = make_client(store, bridge=False)
    assert client.post('/api/robot/command', json={'cmd': 'turn_done'}).status_code == 422
    for cmd in ('start', 'scan'):
        body = client.post('/api/robot/command', json={'cmd': cmd}).json()
        assert body['success'] is False and '끊김' in body['message']
    client.post('/api/robot/command', json={'cmd': 'cancel'})
    client.post('/api/robot/stop')
    assert [n for n, _ in fake.calls] == ['/d2/hmi/command', '/d2/safety/stop']


def test_ws_sends_snapshot_first(store):
    """화면이 /ws 에 붙으면 들고 있는 값을 type 별로 먼저 보낸다({"type", "data"})."""
    client, _ = make_client(store)
    with client.websocket_connect('/ws') as ws:
        got = [ws.receive_json() for _ in range(3)]
    assert {'type': 'state', 'data': {'state': 'IDLE'}} in got
    assert {'type': 'bridge_alive', 'data': {'alive': True}} in got
    assert {'type': 'broker', 'data': {'connected': True}} in got


# ---------- 저장소(DesignStore, W111 최소형) ----------
def test_store_registers_four_bases_as_design_2(store):
    """기본 설계 4개 = 레시피 두 파일 → 변환기 ② → design/2.0(v1.0 · made_by cad · 부모 없음 · recipe · placements · blocks/2.0)."""
    assert store.registered == BASES
    d = store.get_design('001_CHAIR_BENCH_V000')
    assert d['schema'] == 'design/2.0' and d['family'] == 'chair' and d['version'] == 'V000'
    assert d['made_by'] == 'cad' and d['parent_id'] is None
    assert d['recipe']['schema'] == 'recipe/2.0' and d['placements']['schema'] == 'placements/2.0'
    assert d['blocks']['schema'] == 'blocks/2.0' and len(d['blocks']['blocks']) == 11
    # 3D 진행도 색을 잇는 길: placements 의 sequence = blocks 의 order
    assert sorted(s['sequence'] for s in d['placements']['steps']) == sorted(b['order'] for b in d['blocks']['blocks'])


def test_store_list_is_summary_for_tree(store):
    """목록 = 트리용 요약(블록 수 · 바깥 크기 mm), family 로 거름."""
    rows = store.list_designs()
    assert [r['design_id'] for r in rows] == BASES
    bench = rows[0]
    assert bench['block_count'] == 11 and all(v > 0 for v in bench['size_mm'])
    assert [r['design_id'] for r in store.list_designs('desk')] == BASES[2:]


def test_store_rejects_old_schema_and_unsafe_ids(store):
    """옛 형식(design/1)은 변환하지 않고 거절 · 목록에서 뺀다(E-69). 경로 글자가 든 ID 는 없는 설계와 같다."""
    (store.designs_dir / 'old_v1.0.json').write_text('{"schema": "design/1", "design_id": "old_v1.0"}', encoding='utf-8')
    with pytest.raises(ValueError):
        store.get_design('old_v1.0')
    assert 'old_v1.0' not in [r['design_id'] for r in store.list_designs()]
    for bad in ('../secret', 'a/b', '', None):
        with pytest.raises(KeyError):
            store.get_design(bad)


def test_get_design_answer_for_robot(store):
    """로봇의 get_design: 있으면 success true + design/2.0 칸을 펼침, 없으면 success false · ERROR(IRD 10.1)."""
    ok = store.answer_get_design({'req_id': 'r1', 'design_id': '003_DESK_STAND_V000'})
    assert ok['success'] is True and ok['reason'] == '' and ok['schema'] == 'design/2.0' and ok['design_id'] == '003_DESK_STAND_V000'
    bad = store.answer_get_design({'req_id': 'r2', 'design_id': 'nope'})
    assert bad['success'] is False and bad['reason'] == 'ERROR'


def test_save_build_once_per_run_id(store):
    """같은 run_id 가 또 오면(작업 관리자는 확인될 때까지 다시 보냄) 한 번만 저장하고 ok true. 형식이 다르면 ok false."""
    build = {'schema': 'build/1', 'run_id': 'R20261010_143512_a3f9', 'design_id': '001_CHAIR_BENCH_V000', 'result': 'DONE',
             'placed': 11, 'total': 11, 'duration_s': 300.0, 'stop_count': 0, 'blocks': []}
    assert store.answer_save_build({**build, 'req_id': 'a'}) == {'success': True, 'reason': '', 'ok': True}
    (store.builds_dir / 'R20261010_143512_a3f9.json').write_text('{"keep": true}', encoding='utf-8')
    assert store.answer_save_build({**build, 'req_id': 'b'})['ok'] is True
    assert json.loads((store.builds_dir / 'R20261010_143512_a3f9.json').read_text()) == {'keep': True}   # 덮어쓰지 않음
    bad = store.answer_save_build({'schema': 'build/9', 'run_id': 'x', 'req_id': 'c'})
    assert bad['success'] is True and bad['ok'] is False


BENCH, BACK = '001_CHAIR_BENCH_V000', '002_CHAIR_BACK_V000'


def as_new(store, parent, design_id):
    """부모 레시피의 ID 만 design_id 로 바꾼 (blocks, recipe, placements) — check_design 이 주는 꼴(recipe_sha256 · block_id 다시 계산)."""
    from d2_task.recipe_document import recipe_sha256
    base = store.get_design(parent)
    recipe = {**base['recipe'], 'model_id': design_id}
    steps = [{**st, 'block_id': f"{design_id}_{st['block']}"} for st in base['placements']['steps']]
    placements = {**base['placements'], 'model_id': design_id, 'recipe_sha256': recipe_sha256(recipe), 'steps': steps}
    return {**base['blocks'], 'design_id': design_id}, recipe, placements


def derived(store, parent=BENCH, made_by='web'):
    """new_design_id 로 ID 를 받아 파생 설계 하나를 저장한다(AI 생성 · 스캔 저장과 같은 차례)."""
    template = parent.rsplit('_V', 1)[0]
    design_id, _ = store.new_design_id(template, parent)
    blocks, recipe, placements = as_new(store, parent, design_id)
    return store.save_design(blocks, recipe, placements, parent, made_by, prompt='벤치 높게',
                             check={'ok': True, 'min_margin_mm': 9.5, 'errors': [], 'recipe': recipe})


def test_new_design_id_next_number_in_template(store):
    """E-84: <Template ID>_V<3자리>, 번호 = 그 Template 의 가장 큰 V + 1(부모와 상관없이 만든 차례). 다른 Template 은 따로 센다."""
    assert store.new_design_id('001_CHAIR_BENCH', BENCH) == ('001_CHAIR_BENCH_V001', 'V001')
    assert derived(store)['design_id'] == '001_CHAIR_BENCH_V001'
    assert derived(store, '001_CHAIR_BENCH_V001')['design_id'] == '001_CHAIR_BENCH_V002'   # V001 을 고침
    assert store.new_design_id('001_CHAIR_BENCH', BENCH) == ('001_CHAIR_BENCH_V003', 'V003')   # 부모가 V000 이어도 다음 번호
    assert store.new_design_id('002_CHAIR_BACK') == ('002_CHAIR_BACK_V001', 'V001')
    with pytest.raises(ValueError):
        store.new_design_id('002_CHAIR_BACK', BENCH)        # 부모는 같은 Template 만
    for bad in ('chair', '005_SOFA_LONG', '001_chair_bench'):
        with pytest.raises(ValueError):
            store.new_design_id(bad)                        # Template 은 등록된 기본 4개만(E-84 ②)
    with pytest.raises(KeyError):
        store.new_design_id('001_CHAIR_BENCH', '001_CHAIR_BENCH_V009')


def test_save_design_keeps_record_and_never_overwrites(store):
    """검사 합격한 설계만 design/2.0(version = V 번호 · family = Template 에서) + prompt · check · created 로. 같은 ID 는 다시 저장하지 않는다."""
    rec = derived(store)
    got = store.get_design('001_CHAIR_BENCH_V001')
    assert got == rec and got['parent_id'] == BENCH and got['made_by'] == 'web'
    assert got['version'] == 'V001' and got['family'] == 'chair'
    assert got['check'] == {'ok': True, 'min_margin_mm': 9.5, 'errors': []} and got['prompt'] == '벤치 높게'   # check 는 세 칸만
    assert [r['design_id'] for r in store.children(BENCH)] == ['001_CHAIR_BENCH_V001']
    assert store.answer_get_design({'design_id': '001_CHAIR_BENCH_V001'})['placements']['model_id'] == '001_CHAIR_BENCH_V001'
    with pytest.raises(ValueError):                        # 같은 ID 두 번째(다른 요청이 먼저 저장한 경우)
        store.save_design(got['blocks'], got['recipe'], got['placements'], BENCH, 'web', check={'ok': True})


@pytest.mark.parametrize('broken', ['not_ok', 'cad', 'sha', 'v000', 'old_name', 'family', 'parent_template', 'no_parent'])
def test_save_design_refuses_what_robot_should_not_get(store, broken):
    """저장하지 않는 것: 검사 불합격(SR-09) · made_by cad · 레시피 짝(sha) 틀림 · V000(기본 설계 몫) · 옛 이름 꼴 ·
    family 가 Template 과 다름 · 부모가 다른 Template(마지막 방어선) · 부모 없음."""
    design_id, parent, made_by, ok = '001_CHAIR_BENCH_V001', BENCH, 'web', True
    if broken == 'v000':
        design_id = '001_CHAIR_BENCH_V000'
    elif broken == 'old_name':
        design_id = 'chair_v1.1'
    blocks, recipe, placements = as_new(store, BENCH, design_id)
    if broken == 'not_ok':
        ok = False
    elif broken == 'cad':
        made_by = 'cad'
    elif broken == 'sha':
        placements['recipe_sha256'] = '0' * 64
    elif broken == 'family':
        blocks['family'] = 'desk'
    elif broken == 'parent_template':
        parent = BACK
    elif broken == 'no_parent':
        parent = '001_CHAIR_BENCH_V009'
    with pytest.raises(ValueError):
        store.save_design(blocks, recipe, placements, parent, made_by, check={'ok': ok})
    assert not (store.designs_dir / '001_CHAIR_BENCH_V001.json').exists()


def test_list_has_last_build_and_builds_for(store):
    """목록 요약에 가장 최근 조립 한 줄(E-72 — RAG 목록 요약 · 화면). 오차는 잰 값 중 가장 큰 것(mm), null 은 뺀다."""
    def build(run_id, result, blocks):
        return {'schema': 'build/1', 'run_id': run_id, 'design_id': BENCH, 'result': result,
                'placed': 11 if result == 'DONE' else 4, 'total': 11, 'duration_s': 300.0, 'stop_count': 0, 'blocks': blocks}
    store.save_build(build('R20261010_100000_aaaa', 'STOPPED', []))
    store.save_build(build('R20261010_140000_bbbb', 'DONE', [{'block_id': 'X', 'dz_m': -0.0021, 'dx_m': None, 'dy_m': None},
                                                              {'block_id': 'Y', 'dz_m': 0.0008, 'dx_m': None, 'dy_m': None}]))
    rows = {r['design_id']: r for r in store.list_designs()}
    assert rows[BENCH]['last_build'] == {'run_id': 'R20261010_140000_bbbb', 'result': 'DONE', 'placed': 11,
                                         'total': 11, 'max_err_mm': 2.1}
    assert rows[BACK]['last_build'] is None
    assert [b['run_id'] for b in store.builds_for(BENCH)] == ['R20261010_140000_bbbb', 'R20261010_100000_aaaa']
    assert store.builds_for(BACK) == []


def test_template_filter_for_rag_list_and_examples(store):
    """확인된 Template 의 설계만 GPT 에 보여 준다(E-84 ③ — 다른 Template 설계를 읽고 만든 후보는 저장에서 거절되므로).
    examples_for = 그 V000 + 그 Template 의 최근 파생 n 개(최신 먼저), 각각 blocks 포함."""
    derived(store)                                          # 001_CHAIR_BENCH_V001
    newer = derived(store, '001_CHAIR_BENCH_V001')          # V002 — 더 최근
    derived(store, BACK)                                    # 002_CHAIR_BACK_V001 — 다른 Template
    assert [r['design_id'] for r in store.list_designs(template='001_CHAIR_BENCH')] == \
        [BENCH, '001_CHAIR_BENCH_V001', '001_CHAIR_BENCH_V002']
    ex = store.examples_for('001_CHAIR_BENCH', n=1)
    assert [e['design_id'] for e in ex] == [BENCH, newer['design_id']]
    assert all(e['blocks']['schema'] == 'blocks/2.0' for e in ex)
    assert [e['design_id'] for e in store.examples_for('003_DESK_STAND')] == ['003_DESK_STAND_V000']
    with pytest.raises(ValueError):
        store.examples_for('chair')


def test_designs_rest_and_robot_handlers(store):
    """REST: 목록 · 규칙(블록 크기 mm) · 하나 · 없으면 404. 앱이 get_design · save_build 를 저장소에 잇는다.
    손목 그림 · 스캔 사진 · 점군은 들고 있는 바이트 그대로, 없으면 404."""
    client, fake = make_client(store)
    assert [r['design_id'] for r in client.get('/api/designs').json()] == BASES
    assert [r['design_id'] for r in client.get('/api/designs?family=').json()] == BASES      # 빈 값 = 거르지 않음
    assert client.get('/api/designs/rules').json() == {'block_size_mm': [75.0, 25.0, 15.0], 'assembly_area_half_mm': 150.0}
    assert client.get('/api/designs/002_CHAIR_BACK_V000').json()['design_id'] == '002_CHAIR_BACK_V000'
    assert client.get('/api/designs/nope').status_code == 404
    assert set(fake.handlers) == {'/d2/hmi/get_design', '/d2/hmi/save_build'}
    assert client.get('/api/designs/001_CHAIR_BENCH_V000/builds').json() == []
    assert client.get('/api/designs/nope/builds').status_code == 404
    assert client.get('/api/designs/templates').json()[0] == {'id': '001_CHAIR_BENCH', 'name': '벤치', 'label': '벤치'}
    for url in ('/api/robot/wrist.jpg', '/api/robot/scan.jpg', '/api/robot/scan_cloud.ply'):
        assert client.get(url).status_code == 404
    fake.blobs = {'wrist_image': b'\xff\xd8jpeg', 'scan_image': b'\xff\xd8scan', 'scan_cloud': b'ply\n'}
    res = client.get('/api/robot/wrist.jpg')
    assert res.content == b'\xff\xd8jpeg' and res.headers['content-type'] == 'image/jpeg'
    res = client.get('/api/robot/scan.jpg')
    assert res.content == b'\xff\xd8scan' and res.headers['content-type'] == 'image/jpeg'
    res = client.get('/api/robot/scan_cloud.ply')
    assert res.content == b'ply\n' and res.headers['cache-control'] == 'no-store'

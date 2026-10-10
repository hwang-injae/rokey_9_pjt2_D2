# -*- coding: utf-8 -*-
"""W157 동시 요청 회귀 시험 — 실제 ROS 서비스로 check_progress 를 겹쳐 보낸다 (mock_wrist_block + 가짜 get_design, 로봇 없음).

순차 시험(test_design_cache.py)으로는 안 보이는 두 문제를 본다.
  ① 같은 판의 요청이 겹쳐도 get_design 을 한 번만 부르고 모두 답한다(스레드 4개 실행기에 6개를 겹쳐도 TIMEOUT 없음).
  ② 느린 지난 판의 응답이 늦게 와도 새 판 캐시를 덮지 않는다.
rclpy · d2_interfaces · d2_motion · d2_task · ament 의 d2_bringup 이 없으면 건너뛴다(CI 의 d2_vision pytest 는 ROS 가 없다) — 로컬에서
`source install/setup.bash` 뒤 돈다. 통신은 이 PC 안으로만 한정한다(ROS_LOCALHOST_ONLY, 도메인 197). 클래스가 한 번만 쓰는 값이라 위 상수로 둔다.
"""
import json
import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('d2_interfaces.srv')
pytest.importorskip('d2_motion.motion_math')
pytest.importorskip('d2_task.recipe_document')
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

from d2_interfaces.srv import CheckProgress, JsonQuery  # noqa: E402
from test_design_cache import generated_design  # noqa: E402

DOMAIN_ID = '197'


class FakeHmi(Node):
    """/d2/hmi/get_design 가짜. delay_for(n) = n 번째 호출이 늦게 답하는 초. calls = 받은 횟수."""

    def __init__(self, doc):
        super().__init__('d2_cache_test_hmi')
        self.doc, self.calls, self.delay_for, self._lock = doc, 0, (lambda n: 0.0), threading.Lock()
        self.create_service(JsonQuery, '/d2/hmi/get_design', self.answer, callback_group=ReentrantCallbackGroup())

    def reset(self, delay_for):
        self.calls, self.delay_for = 0, delay_for

    def answer(self, request, response):
        with self._lock:
            self.calls += 1
            n = self.calls
        time.sleep(self.delay_for(n))
        response.success, response.reason, response.response_json = True, '', json.dumps(self.doc)
        return response


@pytest.fixture(scope='module')
def ros():
    """ROS 한 번 켜기 + 가짜 HMI · 클라이언트(8 스레드 실행기 — 시험 대상 노드와 따로 둔다. 운영에서도 get_design 서버는 다른 프로세스다)."""
    mp = pytest.MonkeyPatch()
    mp.setenv('ROS_LOCALHOST_ONLY', '1')
    mp.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    mp.setenv('ROS_DOMAIN_ID', DOMAIN_ID)
    rclpy.init()
    doc = generated_design('001_CHAIR_BENCH', 'chair', 'ai_chair_ros', 'ai')
    hmi, client = FakeHmi(doc), Node('d2_cache_test_client')
    side = MultiThreadedExecutor(num_threads=8)
    side.add_node(hmi)
    side.add_node(client)
    threading.Thread(target=side.spin, daemon=True).start()
    cli = client.create_client(CheckProgress, '/d2/vision/check_progress', callback_group=ReentrantCallbackGroup())
    block = doc['design_id'].upper() + '_' + doc['placements']['steps'][0]['block']
    yield SimpleNamespace(hmi=hmi, cli=cli, design_id=doc['design_id'], block=block)
    side.shutdown(timeout_sec=2)
    hmi.destroy_node()
    client.destroy_node()
    rclpy.shutdown()
    mp.undo()


@pytest.fixture
def mock(ros):
    """시험마다 새 mock_wrist_block(빈 캐시) — 운영 main() 과 같은 스레드 4개 실행기. 설계 읽기는 remote(기본)."""
    from ament_index_python.packages import PackageNotFoundError
    try:
        from d2_vision.mock_wrist_block import MockWristBlock
        node = MockWristBlock()
    except (ImportError, PackageNotFoundError) as e:
        pytest.skip(f'mock_wrist_block 을 띄울 수 없다: {e}')
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(node)
    threading.Thread(target=exe.spin, daemon=True).start()
    assert ros.cli.wait_for_service(timeout_sec=10) and node.cli_design.wait_for_service(timeout_sec=10)
    yield node
    exe.shutdown(timeout_sec=2)
    node.destroy_node()


def send(ros, run_id):
    """check_progress 요청을 비동기로 보낸다. 반환 (future, 끝났다는 Event)."""
    req = CheckProgress.Request()
    req.design_id, req.run_id, req.block_ids = ros.design_id, run_id, [ros.block]
    done = threading.Event()
    fut = ros.cli.call_async(req)
    fut.add_done_callback(lambda _: done.set())
    return fut, done


def answers(sent, wait_s=20):
    """보낸 요청들의 응답 → [(success, reason, states)]. 시간 안에 못 받으면 (None, 'NO_ANSWER', None)."""
    out = []
    for fut, done in sent:
        r = fut.result() if done.wait(wait_s) else None
        out.append((r.success, r.reason, list(r.states)) if r else (None, 'NO_ANSWER', None))
    return out


def test_같은_판_동시_요청은_get_design을_한_번만_부르고_모두_답한다(ros, mock):
    """실행기 스레드(4)보다 많은 6개를 겹쳐도: 읽기 1회, 모두 present, TIMEOUT 없음. 끝난 뒤 같은 판을 또 물어도 다시 읽지 않는다."""
    ros.hmi.reset(lambda n: 0.5)
    got = answers([send(ros, 'R1') for _ in range(6)])
    assert got == [(True, '', ['present'])] * 6
    assert ros.hmi.calls == 1
    assert answers([send(ros, 'R1')]) == [(True, '', ['present'])] and ros.hmi.calls == 1


def test_느린_지난_판의_응답이_새_판_캐시를_덮지_않는다(ros, mock):
    """R1 의 get_design 이 늦다(1.5초). 그 사이 R2 가 도착해도 R1 이 끝난 뒤 차례로 R2 를 읽는다(2회). 끝난 뒤 R2 를 또 물으면 캐시(R2)를 쓴다 —
    지난 판(R1)이 캐시를 덮었다면 R2 를 다시 읽어 3회가 된다."""
    ros.hmi.reset(lambda n: 1.5 if n == 1 else 0.0)
    first = send(ros, 'R1')
    time.sleep(0.2)
    second = send(ros, 'R2')
    assert answers([first, second]) == [(True, '', ['present'])] * 2
    assert ros.hmi.calls == 2
    assert answers([send(ros, 'R2')]) == [(True, '', ['present'])] and ros.hmi.calls == 2

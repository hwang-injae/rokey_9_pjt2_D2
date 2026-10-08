"""W119 스캔 상태·검토 명령·중단·잘못된 응답을 가짜 비전과 로봇으로 검증한다."""
import copy
import json
import threading
from types import SimpleNamespace

import pytest

from d2_task.run_logger import RunLogger
from d2_task.task_manager import TaskManager
from test_task_manager import CFG, SAFE_OK, SAFE_STOP, FakeIO, drive
from test_task_timeouts import Client, completed, node  # noqa: F401 — 실제 노드 통신 메서드의 pytest fixture


class ScanIO(FakeIO):
    """실물 없이 점군 수와 추론 문서를 반환하고 스캔 호출을 기록한다."""

    def __init__(self):
        """한 블록 정상 응답과 빈 호출 기록을 준비한다."""
        super().__init__()
        self.captures, self.infers, self.results = [], [], []
        self.capture = {'ok': True, 'points': 100}
        self.inference = {'ok': True, 'inferred_count': 0, 'image_path': '/data/image.png',
                          'blocks': {'schema': 'blocks/1', 'design_id': 'scan_chair_1', 'family': 'chair',
                                     'blocks': [{'order': 1, 'x': 0, 'y': 0, 'z': 0, 'ori': 'x', 'inferred': False}]}}

    def services_ready(self, scan=False):
        """필요한 가짜 서비스 누락 목록을 반환한다. 외부 영향 없음."""
        return self.missing

    def scan_capture(self, pose_id, run_id, should_abort):
        """자세·실행 ID를 기록한다. 시험에서 응답을 붙잡거나 손상시킬 수 있다."""
        self.captures.append((pose_id, run_id))
        return self.capture() if callable(self.capture) else (True, '', copy.deepcopy(self.capture))

    def scan_infer(self, run_id, should_abort):
        """추론 요청 ID를 기록하고 준비된 응답을 돌려준다."""
        self.infers.append(run_id)
        return self.inference() if callable(self.inference) else (True, '', copy.deepcopy(self.inference))

    def publish_scan_result(self, result):
        """방송 결과를 보관한다. 실제 ROS 메시지는 보내지 않는다."""
        self.results.append(copy.deepcopy(result))


def make(log_dir=''):
    """점검을 통과한 작업 관리자와 가짜 비전을 준비한다."""
    io = ScanIO()
    m = TaskManager(CFG, io, RunLogger(str(log_dir)))
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def review(m):
    """스캔 명령부터 검토 대기까지 모든 실제 상태 핸들러를 실행한다."""
    assert m.command('scan') == (True, '')
    drive(m, 'SCAN_REVIEW')


def test_세자세_같은_run_검토대기와_CSV(tmp_path):
    """지정된 세 자세를 한 run으로 촬영하고 CSV만 닫는다. 조립 DB 요약을 만들지 않는다."""
    m, io = make(tmp_path)
    review(m)
    rid = m.run_id
    assert io.captures == [(p, rid) for p in CFG['scan']['poses']]
    assert io.infers == [rid] and io.results[0]['poses_used'] == CFG['scan']['poses']
    assert io.results[0]['schema'] == 'scan_result/1'
    assert not m.pending_builds and m.last_build is None
    m.finalize()
    text = (tmp_path / f'{rid}.csv').read_text()
    assert text.count('scan_capture') == 3 and 'end' in text and 'DONE' in text


def test_검토_저장선택_출발_재스캔_취소():
    """저장 뒤 select_design으로 READY, 출발은 별도 start. 재스캔은 새 ID, 취소는 IDLE."""
    m, io = make()
    review(m)
    first = m.run_id
    assert m.command('start') == (False, 'BUSY')
    assert m.command('scan') == (True, '') and m.run_id != first
    drive(m, 'SCAN_REVIEW')
    assert m.command('cancel') == (True, '') and m.state == 'IDLE' and m.run_id is None
    review(m)
    assert m.command('select_design', 'bench') == (True, '') and m.state == 'READY'
    assert m.command('start') == (True, '')
    drive(m, 'DONE')
    assert m.last_build['placed'] == 11


@pytest.mark.parametrize('mutate', [
    lambda m, io: m.on_gripper({'grasped': True}),
    lambda m, io: m.on_safety(SAFE_STOP),
    lambda m, io: setattr(io, 'missing', ['/d2/vision/scan_capture']),
])
def test_시작점검_실패는_이동과_run생성_없음(mutate):
    """쥔 블록·정지·없는 서버에서는 촬영을 시작하지 않는다."""
    m, io = make()
    mutate(m, io)
    assert not m.command('scan')[0] and m.state == 'IDLE' and m.run_id is None
    assert not io.calls and not io.captures


@pytest.mark.parametrize('phase', ['SCAN_CAPTURE', 'SCAN_INFER'])
def test_정지나_종료뒤_늦은_성공을_버린다(phase):
    """다른 스레드의 비전 응답을 지연시켜 중단 뒤 촬영·검토로 전진하지 않는지 확인한다."""
    m, io = make()
    assert m.command('scan')[0]
    drive(m, phase)
    entered, release = threading.Event(), threading.Event()
    response = copy.deepcopy(io.capture if phase == 'SCAN_CAPTURE' else io.inference)

    def delayed():
        """실제 대기 창에서 다른 콜백이 중단 신호를 넣도록 한다."""
        entered.set()
        assert release.wait(2)
        return True, '', response

    if phase == 'SCAN_CAPTURE':
        io.capture = delayed
    else:
        io.inference = delayed
    t = threading.Thread(target=m.run_once)
    t.start()
    assert entered.wait(1)
    if phase == 'SCAN_CAPTURE':
        m.on_safety(SAFE_STOP)
    else:
        m.shutdown()
    release.set()
    t.join(2)
    assert not t.is_alive() and m.state == 'STOPPED' and not io.results
    m.finalize()
    assert not m.pending_builds


@pytest.mark.parametrize('points', [-1, True, '100', None])
def test_잘못된_점군응답은_다음_자세로_넘어가지_않는다(points):
    """점 수가 비음수 정수가 아니면 오류로 대기한다."""
    m, io = make()
    io.capture['points'] = points
    assert m.command('scan')[0]
    drive(m, 'ERROR')
    assert len(io.captures) == 1 and not io.infers
    m.finalize()
    assert not m.pending_builds


@pytest.mark.parametrize('mutate', [
    lambda b: b.update(ok='true'),
    lambda b: b.update(blocks=[]),
    lambda b: b['blocks'].update(schema='other/1'),
    lambda b: b['blocks'].update(schema='blocks/2.0'),        # 스캔 추론기는 위치 · 방향만(blocks/1) — AI 칸이 든 blocks/2.0 은 scan_infer 응답이 아니다(E-69)
    lambda b: b['blocks'].update(blocks=[]),
    lambda b: b['blocks']['blocks'][0].update(x=float('inf')),
    lambda b: b['blocks']['blocks'][0].update(ori='other'),
    lambda b: b.update(inferred_count=True),
    lambda b: b.update(inferred_count=2),
    lambda b: b.update(image_path=None),
    lambda b: b.update(cloud_path=5),
    lambda b: b.update(cloud_path=None),                # 칸은 있는데 글자가 아님
    lambda b: b.update(cloud_path=['/data/cloud.ply']),
])
def test_추론_손상은_결과를_방송하지_않는다(mutate):
    """형식·유한 좌표·필수 값이 틀리면 검토 가능한 성공 결과로 내보내지 않는다."""
    m, io = make()
    mutate(io.inference)
    assert m.command('scan')[0]
    drive(m, 'ERROR')
    assert not io.results
    m.finalize()
    assert not m.pending_builds


def test_cloud_path는_scan_infer_응답값_그대로_scan_result에_실린다(tmp_path):
    """W147(IRD 6장): 점군 PLY 경로를 작업 관리자가 열지 않고 image_path 옆에 그대로 넘긴다."""
    m, io = make(tmp_path)
    io.inference['cloud_path'] = '/data/scan/R1/cloud.ply'
    review(m)
    assert io.results[0]['cloud_path'] == '/data/scan/R1/cloud.ply' and io.results[0]['image_path'] == '/data/image.png'
    m.finalize()


@pytest.mark.parametrize('present, value', [(False, None), (True, '')])
def test_cloud_path가_없거나_빈_글자면_스캔은_계속되고_결과에서_빠진다(tmp_path, present, value):
    """PL 답(10/8, IRD 4.2 · 6장): cloud_path 는 선택 칸이다. 키 없음 · 빈 글자 = 점군 없음 → SCAN_REVIEW 까지 가고 scan_result 에 칸을 만들지 않는다."""
    m, io = make(tmp_path)
    if present:
        io.inference['cloud_path'] = value
    review(m)
    assert m.state == 'SCAN_REVIEW' and 'cloud_path' not in io.results[0] and io.results[0]['image_path'] == '/data/image.png'
    m.finalize()


def test_이동_시간초과_정지확인_뒤_스캔은_IDLE로():
    """스캔은 부분 점군으로 자동 재개하거나 조립 CHECK로 가지 않는다."""
    m, io = make()
    io.move_script = [(False, 'TIMEOUT')]
    assert m.command('scan')[0]
    m.run_once()
    assert m.state == 'ERROR' and ('stop', 'TIMEOUT') in io.calls
    m.on_safety(SAFE_OK)
    m.run_once()
    assert m.state == 'ERROR'
    m.on_safety(SAFE_STOP)
    m.run_once()
    m.on_safety(SAFE_OK)
    drive(m, 'IDLE')
    assert not io.captures and not m.pending_builds


def test_실제_노드_스캔요청_필드와_제한시간(node):
    """ROS 없는 실제 노드 메서드 시험. pose_id·run_id를 전달하고 답 없는 추론은 TIMEOUT으로 끝낸다."""
    node.capture_cli = Client(completed(SimpleNamespace(success=True, reason='', response_json='{"ok":true,"points":100}')))
    assert node.scan_capture('observe_front', 'R20261007_120000_abcd', lambda: False)[0]
    assert json.loads(node.capture_cli.requests[0].request_json) == dict(pose_id='observe_front', run_id='R20261007_120000_abcd')
    node.infer_cli = Client()
    assert node.scan_infer('run', lambda: False) == (False, 'TIMEOUT', None)
    assert node.infer_cli.removed == [node.infer_cli.future]


@pytest.mark.parametrize('text', ['{', '[]', '{"ok":true,"points":NaN}', '{"ok":true,"points":1e999}'])
def test_실제_노드_깨진JSON과_무한대_응답거절(node, text):
    """서버의 형식 오류를 성공 응답이나 다음 상태로 전달하지 않는다."""
    node.infer_cli = Client(completed(SimpleNamespace(success=True, reason='', response_json=text)))
    assert node.scan_infer('run', lambda: False) == (False, 'ERROR', None)


def test_스캔_상태_알림은_IRD_이름을_쓴다(tmp_path):
    """E-62: 촬영 중(SCAN_MOVE · CAPTURE · INFER)은 scan_running, 검토 화면은 scan_review (message_id null 아님)."""
    m, io = make(tmp_path)
    review(m)
    got = {s['state']: s['message_id'] for s in io.states if s['state'].startswith('SCAN_')}
    assert got == {'SCAN_MOVE': 'scan_running', 'SCAN_CAPTURE': 'scan_running', 'SCAN_INFER': 'scan_running', 'SCAN_REVIEW': 'scan_review'}
    m.finalize()


def test_화면_cancel은_READY_ERROR_SCAN_REVIEW에서_받고_운전_중에는_거절(tmp_path):
    """E-62 ㉮: 음성 cancel 과 같은 상태에서 받는다(S-16 음성 = 버튼). ERROR 는 기록을 ERROR 로 닫는다."""
    m, io = make(tmp_path)
    review(m)                                                        # SCAN_REVIEW
    assert m.command('cancel') == (True, '') and m.state == 'IDLE'
    m, io = make(tmp_path)
    assert m.command('select_design', 'bench') == (True, '') and m.state == 'READY'     # READY
    assert m.command('cancel') == (True, '') and m.state == 'IDLE' and m.planner is None
    m, io = make(tmp_path)
    assert m.command('scan')[0]
    io.move_script = [(False, 'ERROR')]
    drive(m, 'ERROR')                                                # ERROR
    assert m.command('cancel') == (True, '') and m.state == 'IDLE' and m.run_id is None
    m, io = make(tmp_path)
    assert m.command('scan')[0] and m.state == 'SCAN_MOVE'            # 운전 중
    assert m.command('cancel') == (False, 'BUSY') and m.state == 'SCAN_MOVE'
    m.finalize()

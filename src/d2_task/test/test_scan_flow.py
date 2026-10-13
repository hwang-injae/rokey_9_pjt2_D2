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
                          'blocks': {'schema': 'blocks/2', 'design_id': 'scan_chair_1', 'family': 'chair',
                                     'blocks': [{'scan_num': 1, 'x': 0, 'y': 0, 'z': 0, 'ori': 'x', 'inferred': False}]}}

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
    """robot.yaml scan.poses 자세(10/12 W186 부터 5개)를 한 run으로 촬영하고 CSV만 닫는다. 조립 DB 요약을 만들지 않는다."""
    m, io = make(tmp_path)
    review(m)
    rid = m.run_id
    assert io.captures == [(p, rid) for p in CFG['scan']['poses']]
    assert io.infers == [rid] and io.results[0]['poses_used'] == CFG['scan']['poses']
    assert io.results[0]['schema'] == 'scan_result/2.0' and isinstance(io.results[0]['stamp'], float) and io.results[0]['stamp'] > 1.7e9    # 보낸 시각(epoch s, E-75)
    assert not m.pending_builds and m.last_build is None
    m.finalize()
    text = (tmp_path / f'{rid}.csv').read_text()
    assert text.count('scan_capture') == len(CFG['scan']['poses']) and 'end' in text and 'DONE' in text


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


def test_DONE_에서_scan_은_새_run으로_시작하고_끝난_조립_기록은_그대로(tmp_path):
    """조립이 끝난(DONE) 뒤 [스캔]: 새 run_id 로 SCAN_MOVE. 끝난 조립의 last_build · pending_builds · CSV 는 바꾸지 않는다(10/13 PL)."""
    m, io = make(tmp_path)
    assert m.command('select_design', 'bench') == (True, '') and m.command('start') == (True, '')
    drive(m, 'DONE')
    done_run, build, pending = m.run_id, copy.deepcopy(m.last_build), copy.deepcopy(m.pending_builds)
    assert build['result'] == 'DONE' and build['placed'] == 11
    assert m.command('scan') == (True, '')
    assert m.state == 'SCAN_MOVE' and m.run_id not in (None, done_run)
    assert m.last_build == build and m.pending_builds == pending
    drive(m, 'SCAN_REVIEW')
    assert io.captures and all(rid == m.run_id for _, rid in io.captures)
    assert m.last_build == build                                  # 스캔은 조립 결과(build/2)를 만들지 않는다
    m.finalize()
    assert 'DONE' in (tmp_path / f'{done_run}.csv').read_text()


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
    lambda b: b['blocks'].update(schema='parts/4.0'),        # 스캔 추론기는 위치 · 방향만(blocks/2) — AI 칸이 든 parts/4.0 은 scan_infer 응답이 아니다(E-69)
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


def test_nearest_base는_scan_infer_응답값_그대로_scan_result에_실린다(tmp_path):
    """E-79: 추론기가 고른 가장 가까운 기본 설계 design_id 를 작업 관리자가 해석하지 않고 그대로 넘긴다(HMI 가 GPT 힌트로 씀)."""
    m, io = make(tmp_path)
    io.inference['nearest_base'] = '001_CHAIR_BENCH_V000'
    review(m)
    assert io.results[0]['nearest_base'] == '001_CHAIR_BENCH_V000' and m.state == 'SCAN_REVIEW'
    m.finalize()


@pytest.mark.parametrize('present, value', [(False, None), (True, ''), (True, None), (True, 5)])
def test_nearest_base가_없거나_글자가_아니면_스캔은_계속되고_결과에서_빠진다(tmp_path, present, value):
    """선택 칸이자 힌트일 뿐이라 없음 · 빈 글자 · null · 글자 아닌 값은 칸을 빼고 SCAN_REVIEW 까지 간다(cloud_path 처럼 SCAN_FAILED 로 막지 않는다)."""
    m, io = make(tmp_path)
    if present:
        io.inference['nearest_base'] = value
    review(m)
    assert m.state == 'SCAN_REVIEW' and 'nearest_base' not in io.results[0] and io.results[0]['image_path'] == '/data/image.png'
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


def test_W182_촬영_ok_false_는_스캔_실패_detail():
    """success=true + scan_capture ok:false(민범진 계약) → '스캔 실패\\n{detail}', ERROR, 다음 자세로 안 감."""
    m, io = make()
    io.capture = {'ok': False, 'reason': 'SCAN_FAILED', 'detail': '점군을 못 만듦: 점 0개'}
    assert m.command('scan') == (True, '')
    drive(m, 'ERROR')
    assert io.states[-1]['message'] == '스캔 실패\n점군을 못 만듦: 점 0개'
    assert len(io.captures) == 1 and not io.infers and not io.results


def test_W182_추론_ok_false_는_다시_스캔하세요_detail():
    """success=true + scan_infer ok:false → '다시 스캔하세요\\n{detail}', ERROR, 결과 방송 없음."""
    m, io = make()
    io.inference = {'ok': False, 'reason': 'SCAN_FAILED', 'detail': '가려진 블록이 절반을 넘음'}
    assert m.command('scan') == (True, '')
    drive(m, 'ERROR')
    assert io.states[-1]['message'] == '다시 스캔하세요\n가려진 블록이 절반을 넘음' and not io.results


@pytest.mark.parametrize('body', [{'ok': False}, {'ok': False, 'detail': 3}, {'ok': 'no', 'detail': 'x'}, {'ok': True}])
def test_W182_깨진_추론_응답은_기존_잘못된_응답_처리(body):
    """detail 이 글자가 아니거나 ok 가 false 가 아니면 정상 실패로 보지 않는다 — 기존 'SCAN_FAILED: 추론 응답이 잘못됐다'."""
    m, io = make()
    io.inference = body
    assert m.command('scan') == (True, '')
    drive(m, 'ERROR')
    assert io.states[-1]['message'].startswith('SCAN_FAILED: 추론 응답이 잘못됐다') and not io.results


def test_W182_success_false_는_이유_표시와_기존_처리_TIMEOUT_은_정지_요청():
    """success=false 는 기존 _failed 그대로: 일반 실패는 이유를 적은 ERROR, TIMEOUT 은 정지 요청(request_stop)."""
    m, io = make()
    io.capture = lambda: (False, 'ERROR', None)
    assert m.command('scan') == (True, '')
    drive(m, 'ERROR')
    assert 'ERROR' in io.states[-1]['message'] and '스캔 실패\n' not in io.states[-1]['message']
    m2, io2 = make()
    io2.capture = lambda: (False, 'TIMEOUT', None)
    assert m2.command('scan') == (True, '')
    for _ in range(20):
        m2.run_once()
        if ('call', 'stop', 'TIMEOUT') in [e[:3] for e in io2.events]:
            break
    assert ('stop', 'TIMEOUT') in io2.calls

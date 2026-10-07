# -*- coding: utf-8 -*-
"""TaskManager 시험 (SDD 5장 상태표 · 7.1 실패 대응). ROS · 로봇 없이 가짜 io 로 돈다.

레시피는 한세교 LV1 벤치 11개(001_CHAIR_BENCH, assembly.recipe/1.0), 설정은 실제 robot.yaml(공급 칸은 잡기마다 하나).
'칸이 둘' 시험은 FLAT_SHORT 칸을 하나 더한 복사본을 쓴다.
"""
import copy
import json
import re
import threading
import time
from pathlib import Path

import pytest
import yaml

from d2_task.task_manager import TaskManager, wait_until

SRC = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load((SRC / 'd2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
RECIPE = json.loads((Path(__file__).parent / 'fixtures/001_CHAIR_BENCH.recipe.json').read_text(encoding='utf-8'))
IDS = [f'001_CHAIR_BENCH_B{n:03d}' for n in range(1, 12)]
OK = (True, '')
SAFE_OK = {'stopped': False, 'locked': False, 'reason': ''}
SAFE_STOP = {'stopped': True, 'locked': True, 'reason': 'STOP_REQUEST'}
# IRD 2장 '화면 알림 message_id' 중 이번에 쓰는 값. 이 밖의 값은 새로 만든 이름이라 나오면 안 된다
IRD_MESSAGE_IDS = {'ready_to_start', 'supply_empty', 'offset_over', 'stopped', 'done', 'voice_start_ignored'}


class FakeIO:
    """TaskManager 가 바깥 일을 맡기는 io 의 가짜. 부른 일 · 방송을 일어난 순서대로 events 에 적는다.

    *_script 는 호출마다 하나씩 꺼내 쓰는 결과 대기열(없으면 성공). pick_script 의 원소는 함수여도 된다(io 를 받아 결과를 돌려줌).
    world = 지금 작업대에 놓인 블록. dz = 놓인 블록의 높이 어긋남(m).
    """

    def __init__(self, recipe=RECIPE):
        self.recipe, self.manager = recipe, None
        self.events, self.states, self.progress = [], [], []
        self.world, self.missing, self.dz = set(), [], 0.0
        self.move_script, self.check_script, self.pick_script = [], [], []

    @property
    def calls(self):
        """부른 일만 순서대로: ('load', id) · ('move_to', target) · ('check', ids) · ('pick', block, slot)."""
        return [e[1:] for e in self.events if e[0] == 'call']

    def load_recipe(self, design_id):
        self.events.append(('call', 'load', design_id))
        return self.recipe if design_id == 'bench' else None

    def services_ready(self):
        return list(self.missing)

    def move_to(self, target, should_abort):
        self.events.append(('call', 'move_to', target))
        return self.move_script.pop(0) if self.move_script else OK

    def check_progress(self, block_ids, should_abort):
        self.events.append(('call', 'check', tuple(block_ids)))
        script = self.check_script.pop(0) if self.check_script else None
        if isinstance(script, tuple):                      # (False, reason): 호출 실패
            return False, script[1], {}
        rows = {}
        for b in block_ids:
            state = (script or {}).get(b, 'present' if b in self.world else 'absent')
            here = state == 'present'
            rows[b] = {'state': state, 'dx_m': float('nan'), 'dy_m': float('nan'),
                       'dz_m': self.dz if here else float('nan'), 'top_z_m': 0.05 if here else float('nan')}
        return True, '', rows

    def pick_place(self, goal, should_abort):
        self.events.append(('call', 'pick', goal['block_id'], goal['supply_slot']))
        step = self.pick_script.pop(0) if self.pick_script else OK
        if callable(step):
            step = step(self)
        if step[0]:
            self.world.add(goal['block_id'])
        return step

    def publish_state(self, msg):
        self.events.append(('state', msg['state'], msg['message_id']))
        self.states.append(msg)

    def publish_progress(self, msg):
        self.progress.append(msg)


def make(cfg=CFG):
    """정지 노드 · 그리퍼 신호를 받은 TaskManager 와 가짜 io."""
    io = FakeIO()
    m = TaskManager(cfg, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def ready(cfg=CFG, **scripts):
    """bench 를 골라 READY 로 만든다. scripts 는 io 의 *_script 를 미리 채운다."""
    m, io = make(cfg)
    for name, value in scripts.items():
        setattr(io, name, value)
    assert m.command('select_design', 'bench') == OK
    assert m.state == 'READY'
    return m, io


def go(cfg=CFG, **scripts):
    """READY 에서 출발해 CHECK 로 만든다."""
    m, io = ready(cfg, **scripts)
    assert m.command('start') == OK
    assert m.state == 'CHECK'
    return m, io


def drive(m, until, limit=500):
    """until(상태 이름 하나 또는 여러 개)에 닿을 때까지 한 단계씩 돌린다. 못 닿으면 실패."""
    until = {until} if isinstance(until, str) else set(until)
    for _ in range(limit):
        if m.state in until:
            return
        m.run_once()
    raise AssertionError(f'{sorted(until)} 에 못 갔다: 지금 {m.state}')


def picks(io):
    """pick_place 로 보낸 (block_id, 칸) 들."""
    return [c[1:] for c in io.calls if c[0] == 'pick']


def seen(io):
    """방송한 상태 이름들(순서대로)."""
    return [s['state'] for s in io.states]


def two_flat_short_slots():
    """FLAT_SHORT 칸이 둘(2번, 7번)인 robot.yaml 복사본."""
    cfg = copy.deepcopy(CFG)
    cfg['supply_slots'].append(dict(cfg['supply_slots'][1], x_m=cfg['supply_slots'][1]['x_m'] + 0.05))
    return cfg


def assert_ird_only(io):
    """방송한 state/1 이 IRD 에 있는 이름만 쓰는지: message_id 는 IRD 값 또는 null, run_id 는 null 또는 R<날짜>_<uuid>, mode 는 auto."""
    for s in io.states:
        assert s['schema'] == 'state/1' and s['mode'] == 'auto'
        assert s['run_id'] is None or re.fullmatch(r'R\d{8}_[0-9a-f]{32}', s['run_id']), s
        assert s['message_id'] is None or s['message_id'] in IRD_MESSAGE_IDS, s


# ---------- 정상 흐름 ----------
def test_벤치_11개_정상_흐름():
    m, io = go()
    drive(m, 'DONE')
    assert io.world == set(IDS)
    assert picks(io) == [(b, '2') for b in IDS[:8]] + [(b, '1') for b in IDS[8:]]      # 벽 FLAT_SHORT 칸 2, 좌판 FLAT_LONG 칸 1
    assert io.calls[:6] == [('load', 'bench'), ('move_to', 'observe'), ('check', tuple(IDS)),
                            ('pick', IDS[0], '2'), ('move_to', 'observe'), ('check', (IDS[0],))]
    assert ('check', (IDS[2], IDS[0])) in io.calls                                   # 3번째: 놓은 블록 + 받침
    assert len(io.progress) == 12 and all(json.dumps(p, allow_nan=False) for p in io.progress)
    order = seen(io)
    assert order[0] == 'READY' and order[-1] == 'DONE'
    assert all(s in order for s in ('CHECK', 'SELECT', 'PICK_PLACE', 'VERIFY'))
    assert io.states[-1]['message_id'] == 'done'
    assert_ird_only(io)


# ---------- 집기 · 놓기 실패 (SDD 7.1) ----------
def test_PLAN_FAILED_다시_계획_1번에_성공하면_이어_간다():
    m, io = go(pick_script=[(False, 'PLAN_FAILED'), OK])
    drive(m, 'DONE')
    assert picks(io)[:2] == [(IDS[0], '2'), (IDS[0], '2')] and io.world == set(IDS)


def test_PLAN_FAILED_또_실패하면_ERROR():
    m, io = go(pick_script=[(False, 'PLAN_FAILED')] * 2)
    drive(m, 'ERROR')
    assert len(picks(io)) == 2 and 'PLAN_FAILED' in io.states[-1]['message']
    assert_ird_only(io)


def test_GRASP_FAILED_같은_칸_1번_다음_칸_1개_그_뒤_ERROR():
    m, io = go(two_flat_short_slots(), pick_script=[(False, 'GRASP_FAILED')] * 5)
    drive(m, 'ERROR')
    assert [slot for _, slot in picks(io)] == ['2', '2', '7']      # 세 번째는 반드시 다른 칸, 네 번째는 없다
    assert 'WAIT_SUPPLY' not in seen(io)
    assert_ird_only(io)


def test_GRASP_FAILED_다음_칸에서_성공하면_이어_간다():
    m, io = go(two_flat_short_slots(), pick_script=[(False, 'GRASP_FAILED')] * 2 + [OK])
    drive(m, 'DONE')
    assert [slot for _, slot in picks(io)][:3] == ['2', '2', '7'] and io.world == set(IDS)


def test_GRASP_FAILED_다른_칸이_없으면_공급_요청_없이_ERROR():
    m, io = go(pick_script=[(False, 'GRASP_FAILED')] * 2)
    drive(m, 'ERROR')
    assert [slot for _, slot in picks(io)] == ['2', '2']
    assert 'WAIT_SUPPLY' not in seen(io) and not any(s['message_id'] == 'supply_empty' for s in io.states)


def test_SLOT_EMPTY_칸이_다_비면_관측_자세에_도착한_뒤_알림_계속_누르면_PICK_PLACE():
    m, io = go(pick_script=[(False, 'SLOT_EMPTY')])
    drive(m, 'WAIT_SUPPLY')
    assert m.command('start') == (False, 'BUSY')                                      # 아직 관측 자세로 가는 중 — 진행시키지 않는다
    assert not any(s['message_id'] == 'supply_empty' for s in io.states)
    m.run_once()                                                                      # 관측 자세로 이동한 뒤 알림
    ev = io.events
    i_pick = max(i for i, e in enumerate(ev) if e[:2] == ('call', 'pick'))
    i_move = next(i for i, e in enumerate(ev) if i > i_pick and e[1:] == ('move_to', 'observe'))
    i_note = ev.index(('state', 'WAIT_SUPPLY', 'supply_empty'))
    assert i_pick < i_move < i_note                                                   # 도착(이동 성공) 뒤에야 '채워 주세요'
    assert m.command('start') == OK
    drive(m, 'PICK_PLACE')
    assert 'SELECT' not in seen(io)[i_note and len(seen(io)) - 2:]                    # 재개는 SELECT 를 거치지 않는다
    drive(m, 'DONE')
    assert picks(io)[:2] == [(IDS[0], '2'), (IDS[0], '2')] and io.world == set(IDS)   # 같은 블록 · 같은 칸부터


def test_SLOT_EMPTY_다른_칸이_있으면_WAIT_SUPPLY_없이_다음_칸():
    m, io = go(two_flat_short_slots(), pick_script=[(False, 'SLOT_EMPTY')])
    drive(m, 'DONE')
    assert [slot for _, slot in picks(io)][:2] == ['2', '7'] and 'WAIT_SUPPLY' not in seen(io)


def test_BUSY는_PICK_PLACE에_머물며_다시_시도한다():
    m, io = go(pick_script=[(False, 'BUSY')] * 3 + [OK])
    drive(m, 'PICK_PLACE')
    for _ in range(3):
        m.run_once()
        assert m.state == 'PICK_PLACE'
    m.run_once()
    assert m.state == 'VERIFY'
    drive(m, 'DONE')
    assert 'ERROR' not in seen(io)


@pytest.mark.parametrize('reason', ['TIMEOUT', 'ERROR', 'GRIPPER_NO_RESPONSE'])
def test_IRD_정식_실패_코드는_ERROR로_간다(reason):
    """TIMEOUT · ERROR · GRIPPER_NO_RESPONSE 는 IRD 7장의 정식 코드이고 SDD 7.1 이 ERROR 로 정했다(TIMEOUT 은 '모르는 reason' 이 아니다)."""
    m, io = go(pick_script=[(False, reason)])
    drive(m, 'ERROR')
    assert reason in io.states[-1]['message'] and 'STOPPED' not in seen(io)
    assert_ird_only(io)


def test_IRD에_없는_reason도_일반_실패로_ERROR():
    m, io = go(pick_script=[(False, 'SOMETHING_NEW')])
    drive(m, 'ERROR')
    assert 'SOMETHING_NEW' in io.states[-1]['message'] and 'STOPPED' not in seen(io)
    assert_ird_only(io)


@pytest.mark.parametrize('reason', ['TIMEOUT', 'SOMETHING_NEW'])      # 정식 코드 TIMEOUT 과 IRD 에 없는 글자 모두 ERROR
@pytest.mark.parametrize('script', ['move_script', 'check_script'])
def test_이동_관측_실패도_ERROR(script, reason):
    m, io = go(**{script: [(False, reason)]})
    drive(m, 'ERROR')
    assert reason in io.states[-1]['message']


# ---------- 정지 · 복구 ----------
@pytest.mark.parametrize('reason', ['STOPPED', 'CANCELED', 'NO_FEEDBACK'])
def test_정지류_결과는_STOPPED_풀리면_RECOVER_CHECK_이어_간다(reason):
    m, io = go(pick_script=[(False, reason)])
    drive(m, 'STOPPED')
    assert io.states[-1]['message_id'] == 'stopped'
    for _ in range(5):
        m.run_once()
    assert m.state == 'STOPPED'                          # 풀린 신호가 오기 전에는 그대로
    m.on_safety(SAFE_STOP)
    m.run_once()
    assert m.state == 'STOPPED'                          # 잠긴 신호가 와도 그대로
    m.on_safety(SAFE_OK)
    drive(m, 'CHECK')
    drive(m, 'DONE')
    assert 'RECOVER' in seen(io) and io.world == set(IDS)
    assert_ird_only(io)


def test_운전_중_정지_신호는_어느_상태에서든_STOPPED():
    m, io = go()
    drive(m, 'SELECT')
    m.on_safety(SAFE_STOP)
    m.run_once()
    assert m.state == 'STOPPED'
    m.on_safety(SAFE_OK)
    drive(m, 'DONE')
    assert io.world == set(IDS)


def test_기다리는_중_정지_신호가_오면_바로_빠져나온다():
    def waits_for_stop(io):
        """pick_place 가 끝없이 걸리는 상황 — 중단 신호(halted)로만 빠져나온다."""
        return (True, '') if wait_until(threading.Event(), io.manager.halted) else (False, 'STOPPED')

    m, io = go(pick_script=[waits_for_stop])
    drive(m, 'PICK_PLACE')
    timer = threading.Timer(0.2, m.on_safety, [SAFE_STOP])
    start = time.monotonic()
    timer.start()
    m.run_once()                                         # 정지 신호가 오기 전에는 여기서 기다리고, 오면 빠져나와 STOPPED
    assert time.monotonic() - start < 2.0
    assert m.state == 'STOPPED' and io.states[-1]['message_id'] == 'stopped'
    timer.join()


def test_복구_때_쥔_블록이_있으면_ERROR():
    m, io = go(pick_script=[(False, 'STOPPED')])
    drive(m, 'STOPPED')
    m.on_gripper({'grasped': True})
    m.on_safety(SAFE_OK)
    drive(m, 'ERROR')
    assert '쥐고' in io.states[-1]['message']


def test_ERROR는_잠금이_풀린_신호가_새로_와야_RECOVER():
    m, io = go(pick_script=[(False, 'PLAN_FAILED')] * 2)
    drive(m, 'ERROR')
    for _ in range(30):
        m.run_once()
    assert m.state == 'ERROR'                            # ERROR 에 들어오기 전의 신호로는 안 넘어간다
    assert m.command('start') == (False, 'BUSY')         # 새 start 규칙은 없다
    m.on_safety(SAFE_OK)                                 # [다시 시작] = safety/resume 이 같은 내용을 다시 내보냄
    drive(m, 'CHECK')
    drive(m, 'DONE')
    assert 'RECOVER' in seen(io) and io.world == set(IDS)


def test_조립_중이_아닐_때_정지는_풀리면_IDLE_W121_확인():
    m, io = ready()
    m.on_safety(SAFE_STOP)
    m.run_once()
    assert m.state == 'STOPPED'
    m.on_safety(SAFE_OK)
    drive(m, 'IDLE')
    assert m.planner is None and m.design_id is None


def test_WAIT_SUPPLY_중_정지하면_눌린_계속은_버린다():
    m, io = go(pick_script=[(False, 'SLOT_EMPTY')])
    drive(m, 'WAIT_SUPPLY')
    m.run_once()
    assert m.command('start') == OK                      # 도착 뒤라 받는다
    m.on_safety(SAFE_STOP)
    m.run_once()
    assert m.state == 'STOPPED'
    m.on_safety(SAFE_OK)
    drive(m, 'CHECK')                                    # 진행 확인부터 — 바로 PICK_PLACE 로 가지 않는다


# ---------- 판정 ----------
def test_VERIFY에서_놓은_블록이_없으면_OFFSET_OVER():
    m, io = go(check_script=[None, {IDS[0]: 'absent'}])
    drive(m, 'ERROR')
    assert io.states[-1]['message_id'] == 'offset_over'


def test_VERIFY에서_높이가_두께_절반보다_어긋나면_OFFSET_OVER():
    m, io = go()
    io.dz = CFG['block_actual_m'][2] / 2 + 0.001
    drive(m, 'ERROR')
    assert io.states[-1]['message_id'] == 'offset_over'


def test_못_본_블록은_재관측_1번_뒤_이어_간다():
    m, io = go(check_script=[None, {IDS[0]: 'occluded'}])
    drive(m, 'DONE')
    assert io.calls.count(('check', tuple(IDS))) == 2   # 처음 + 재관측 1번


def test_계속_못_보이면_ERROR():
    m, io = go(check_script=[None, {IDS[0]: 'occluded'}, {IDS[0]: 'occluded'}])
    drive(m, 'ERROR')
    assert io.calls.count(('check', tuple(IDS))) == 2 and 'UNKNOWN_BLOCK' in io.states[-1]['message']


# ---------- 명령 · 음성 · 시작 점검 ----------
def test_운전_중에는_명령을_거절한다():
    m, io = go()
    assert m.command('start') == (False, 'BUSY')
    assert m.command('select_design', 'bench') == (False, 'BUSY')
    assert m.command('scan') == (False, '')              # 스캔은 W119
    assert m.command('모르는 것') == (False, '')
    assert m.state == 'CHECK'
    m.on_safety(SAFE_STOP)
    m.run_once()
    assert m.command('start') == (False, 'STOPPED')
    assert_ird_only(io)


@pytest.mark.parametrize('why, mutate, reason', [
    ('서버가 안 떠 있다', lambda m, io: setattr(io, 'missing', ['/d2/motion/pick_place']), ''),
    ('정지 노드 신호 없음', lambda m, io: setattr(m, 'safety', None), ''),
    ('정지 상태', lambda m, io: m.on_safety(SAFE_STOP), 'STOPPED'),
    ('그리퍼 신호 없음', lambda m, io: setattr(m, 'gripper', None), ''),
    ('그리퍼가 블록을 쥠', lambda m, io: m.on_gripper({'grasped': True}), ''),
])
def test_시작_점검에_실패하면_고르지_않는다(why, mutate, reason):
    m, io = make()
    mutate(m, io)
    assert m.command('select_design', 'bench') == (False, reason)
    assert m.state == 'IDLE' and m.planner is None
    assert '시작 점검 실패' in io.states[-1]['message']


def test_설계를_못_읽으면_거절():
    m, io = make()
    assert m.command('select_design', 'nope') == (False, '')
    assert m.command('select_design', '') == (False, '')
    io.recipe = copy.deepcopy(RECIPE)
    io.recipe['model']['parts'][0]['size_mm'] = [80.0, 25.0, 15.0]     # robot.yaml 블록 크기와 다른 레시피
    assert m.command('select_design', 'bench') == (False, '')
    assert m.state == 'IDLE'


def test_출발_때도_점검을_한_번_더_한다():
    m, io = ready()
    m.on_gripper({'grasped': True})
    assert m.command('start') == (False, '')
    assert m.state == 'READY'


def test_음성_의도는_화면_버튼과_똑같이():
    m, io = ready()
    m.on_intent('request_design')                        # 화면이 처리 — 무시
    m.on_intent('ask_progress')
    assert m.state == 'READY'
    m.on_intent('start')
    assert m.state == 'CHECK'
    m.on_intent('start')                                 # 운전 중 음성 출발은 무시 + 알림
    assert m.state == 'CHECK' and io.states[-1]['message_id'] == 'voice_start_ignored'
    m.on_intent('cancel')                                # 운전 중 취소는 받지 않는다
    assert m.state == 'CHECK'


def test_음성으로_설계_고르고_출발하고_취소():
    m, io = make()
    m.on_intent('start')                                 # 설계 없이는 무시
    assert m.state == 'IDLE' and io.states[-1]['message_id'] == 'voice_start_ignored'
    m.on_intent('select_design', 'bench')
    assert m.state == 'READY'
    m.on_intent('cancel')
    assert m.state == 'IDLE' and m.planner is None
    m.on_intent('start', 'bench')                        # 설계를 포함한 음성 start
    assert m.state == 'CHECK'


def test_ERROR에서_음성_취소는_IDLE():
    m, io = go(pick_script=[(False, 'PLAN_FAILED')] * 2)
    drive(m, 'ERROR')
    m.on_intent('cancel')
    assert m.state == 'IDLE'


def test_종료_신호는_기다림을_중단시킨다():
    m, io = make()
    assert not m.halted()
    m.shutdown()
    assert m.halted()


# ---------- wait_until ----------
def test_wait_until_이벤트가_켜지면_True():
    event = threading.Event()
    event.set()
    assert wait_until(event, lambda: False) is True


def test_wait_until_중단_신호가_오면_이벤트를_안_기다리고_False():
    start = time.monotonic()
    assert wait_until(threading.Event(), lambda: True) is False
    assert time.monotonic() - start < 1.0


def test_wait_until_기다리다가_중단_신호가_나중에_와도_빠져나온다():
    flag = threading.Event()
    threading.Timer(0.15, flag.set).start()
    start = time.monotonic()
    assert wait_until(threading.Event(), flag.is_set) is False
    assert 0.1 < time.monotonic() - start < 2.0

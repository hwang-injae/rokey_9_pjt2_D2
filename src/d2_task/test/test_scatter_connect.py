# -*- coding: utf-8 -*-
"""흩뿌린 공급(supply_mode: scatter) 연결 시험 — FakeIO 로 관측 → 후보 선택 → 대기·계속 → 정지 뒤 늦은 응답 무시까지 (W130, 로봇 없음).

실제 로봇 실행 · 열림 폭 확정 · W142 장애물(obstacles) 계약은 이 시험의 범위 밖이다. 이 시험은 TaskManager 의 목표 선택이
공급 칸(slots)과 흩뿌림(scatter)으로 갈라지고, 흩뿌림이 observe_supply 이동 → find_blocks 조회 → 후보 고르기 → PICK_PLACE 로 이어지며
정지 · 새 요청 뒤에 도착한 답은 적용되지 않는 것을 확인한다.
"""
import ast
import copy
import threading
import time
from pathlib import Path

import pytest

from d2_task.task_manager import TaskManager
from test_block_picker import blk
from test_task_manager import CFG, IDS, SAFE_OK, SAFE_STOP, FakeIO, drive, picks, seen

SCATTER_CFG = dict(copy.deepcopy(CFG), supply_mode='scatter')


class ScatterIO(FakeIO):
    """find_blocks 가짜: find_script 를 한 칸씩 쓰고(함수면 부른다), 없으면 늘 정상 블록 몇 개를 돌려준다."""

    def __init__(self):
        super().__init__()
        self.find_script = []
        self.pick_info = []                  # 집기마다 하나씩: {'step': 마지막 피드백, 'grasped': 결과 뒤 gripper 값(없으면 상태를 안 보냄)}
        self.goals = []

    def find_blocks(self, run_id, should_abort):
        self.events.append(('call', 'find_blocks', run_id))
        step = self.find_script.pop(0) if self.find_script else (True, '', good())
        return step(self) if callable(step) else step


def _pick_place(self, goal, should_abort, on_feedback=None, on_result=None):
    """FakeIO.pick_place 에 피드백 · 결과 수신 시각 · 결과 뒤 gripper 상태를 더한다(실패 뒤 판단 재료)."""
    info = self.pick_info.pop(0) if self.pick_info else None
    self.goals.append(copy.deepcopy(goal))
    res = FakeIO.pick_place(self, goal, should_abort, on_feedback, on_result)
    if info is not None:
        on_feedback(info['step'])
        at = self.last_at = time.time()
        on_result(at)
        if 'grasped' in info:
            self.manager.on_gripper({'stamp': at, 'grasped': info['grasped']})
    return res


ScatterIO.pick_place = _pick_place


def good():
    """눕힘 블록 둘(긴 방향 · 짧은 방향 틈이 모두 넉넉함) — 어느 FLAT 잡기에도 맞는다."""
    return {'ok': True, 'blocks': [blk(x=0.30, length=40, width=40), blk(x=0.35, length=40, width=40)]}


def make(open_width_m=0.0865, cfg=SCATTER_CFG):
    io = ScatterIO()
    m = TaskManager(cfg, io, open_width_m=open_width_m)
    io.manager = m
    fast_wait(m)
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    return m, io


def start(m):
    assert m.command('select_design', 'bench') == (True, '') and m.state == 'READY'
    assert m.command('start') == (True, '') and m.state == 'CHECK'


def calls(io, name):
    return [c for c in io.calls if c[0] == name]


def test_slots가_기본이고_이상한_supply_mode는_만들_때_거절():
    m = TaskManager(CFG, FakeIO())
    assert m.supply_mode == 'slots' and m._scatter is None
    with pytest.raises(ValueError, match='supply_mode'):
        TaskManager(dict(CFG, supply_mode='mixed'), FakeIO())


def test_scatter는_블록마다_관측_조회_고르기_집기로_끝까지_간다():
    m, io = make()
    start(m)
    drive(m, 'DONE')
    assert m.last_build['placed'] == len(IDS)
    assert [p[1] for p in picks(io)] == [''] * len(IDS)                        # 공급 칸 번호 없음 = 흩뿌림
    assert len(calls(io, 'find_blocks')) == len(IDS)
    assert ('move_to', 'observe_supply') in [c[:2] for c in io.calls] and len(calls(io, 'move_to')) >= len(IDS)
    goal = m.goal
    assert goal['supply_slot'] == '' and goal['open_width_m'] == 0.0865 and 'pick_pose' in goal and 'place_pose' in goal


def test_열림_폭_0은_정상_값이다_모든_집기_요청이_0을_싣는다():
    """task 노드가 넘기는 값(0 = pick_place 가 잡기마다 robot.yaml grasp_open_pick_m 을 씀). 0 이어도 ERROR 없이 끝까지 가고, 잡기가 달라도 한 값(0)으로 간다."""
    m, io = make(open_width_m=0.0)
    widths, original = [], io.pick_place
    io.pick_place = lambda goal, *a, **k: (widths.append(goal['open_width_m']), original(goal, *a, **k))[1]
    start(m)
    drive(m, 'DONE')
    assert m.last_build['placed'] == len(IDS) and widths == [0.0] * len(IDS)


def test_task_노드가_열림_폭_0을_TaskManager에_넘긴다():
    """노드가 안 넘기면(None) scatter 로 바꾸는 순간 첫 블록에서 '열림 폭 미정' ERROR 가 난다(10/10 W130 점검 B30). ROS 없이 소스에서 확인한다."""
    tree = ast.parse((Path(__file__).parents[1] / 'd2_task/task_node.py').read_text(encoding='utf-8'))
    calls_ = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'TaskManager']
    assert len(calls_) == 1
    kw = {k.arg: k.value for k in calls_[0].keywords}
    assert isinstance(kw.get('open_width_m'), ast.Constant) and kw['open_width_m'].value == 0.0


def test_목표_블록은_작업_판단이_정하고_어디서_집을지만_방식이_정한다():
    """scatter 도 블록 순서 · 잡기 · 놓을 자세는 slots 와 같다(planner.next_target)."""
    m, io = make()
    start(m)
    drive(m, 'PICK_PLACE')
    t = m.planner.next_target()
    assert (m.goal['part_id'], m.goal['grasp'], m.goal['place_pose']) == (t['part_id'], t['grasp'], t['place_pose'])


def test_공급에_블록이_없으면_supply_empty로_기다리고_계속하면_다시_관측한다():
    m, io = make()
    io.find_script = [(True, '', {'ok': True, 'blocks': []})]
    start(m)
    drive(m, 'WAIT_SUPPLY')
    assert io.states[-1]['message_id'] == 'supply_empty' and not picks(io)
    finds = len(calls(io, 'find_blocks'))
    assert m.command('start') == (True, '')                                  # [계속]
    drive(m, 'PICK_PLACE')
    assert len(calls(io, 'find_blocks')) == finds + 1 and m.part_id == IDS[0]
    drive(m, 'DONE')


def test_계속은_알림을_낸_뒤에야_받는다():
    m, io = make()
    io.find_script = [(True, '', {'ok': True, 'blocks': []})]
    start(m)
    drive(m, 'WAIT_SUPPLY')
    assert m._supply_moved is True                                            # 이미 observe_supply 에 와 있어 도착 확인 없이 알림이 나갔다


@pytest.mark.parametrize('blocks, message_id', [
    ([blk(overlap='under')], 'no_match_block'),                                # 덮인 것뿐 → 맞는 블록 없음 알림(E-73)
    ([blk(tilted=True)], 'tilted_block'),                                      # 기울어진 것만 남음 → IRD 알림
    ([blk(tilted=True), blk(x=0.40, overlap='under')], 'tilted_block'),        # 기울어진 것 + 덮인 것뿐(E-68) → 같은 알림
    ([blk(tilted=True), blk(x=0.40, up='LENGTH')], 'no_match_block'),          # 다른 제외 이유(목표와 다른 자세)가 섞이면 맞는 블록 없음 알림
    ([blk(tilted=True), blk(x=0.40, overlap='under'), {'x_m': 'abc'}], 'no_match_block'),  # 잘못된 값이 섞여도 맞는 블록 없음 알림
])
def test_맞는_블록이_없으면_이유를_알리고_기다린다(blocks, message_id):
    m, io = make()
    io.find_script = [(True, '', {'ok': True, 'blocks': blocks})]
    start(m)
    drive(m, 'WAIT_SUPPLY')
    last = io.states[-1]
    assert last['message_id'] == message_id and '맞는 블록이 없다' in last['message'] and not picks(io)


def test_기울어진_블록이_있어도_다른_블록을_집고_기울어진_것은_장애물이다():
    """E-68: 집을 후보가 있으면 tilted 블록 때문에 멈추지 않는다. tilted · under 는 PickPlace 장애물로 실린다."""
    m, io = make()
    io.find_script = [(True, '', {'ok': True, 'blocks': [blk(x=0.30, length=40, width=40), blk(x=0.40, tilted=True), blk(x=0.45, overlap='under')]})]
    start(m)
    drive(m, 'PICK_PLACE')
    assert len(m.goal['obstacles']) == 2 and m.goal['supply_slot'] == ''


def test_장애물_블록의_형식이_틀리면_일부만_보내지_않고_ERROR():
    m, io = make()
    io.find_script = [(True, '', {'ok': True, 'blocks': [blk(x=0.30, length=40, width=40), {'x_m': 'abc'}]})]
    start(m)
    drive(m, 'ERROR')
    assert '장애물 블록' in io.states[-1]['message'] and not picks(io)


def test_조회_응답이_이상하면_다시_시도하지_않고_ERROR():
    m, io = make()
    io.find_script = [(True, '', {'ok': False})]
    start(m)
    drive(m, 'ERROR')
    assert len(calls(io, 'find_blocks')) == 1 and not picks(io)


def test_열림_폭이_정해지지_않으면_ERROR로_집기를_시작하지_않는다():
    m, io = make(open_width_m=None)
    start(m)
    drive(m, 'ERROR')
    assert '열림 폭' in io.states[-1]['message'] and not picks(io)


def test_설정이_미완이면_로봇을_움직이기_전에_멈춘다():
    """설정 점검은 observe_supply 이동 · find_blocks 조회보다 앞이다 — 쓸 수 없는 설정으로 팔을 공급 영역에 보내지 않는다."""
    m, io = make(open_width_m=None)
    start(m)
    drive(m, 'ERROR')
    assert ('move_to', 'observe_supply') not in [c[:2] for c in io.calls] and not calls(io, 'find_blocks')


def test_조회가_정지로_끝나면_STOPPED():
    m, io = make()
    io.find_script = [(False, 'STOPPED', None)]
    start(m)
    drive(m, 'STOPPED')
    assert not picks(io)


def test_정지가_끼면_늦게_온_정상_응답을_적용하지_않는다():
    """find_blocks 를 기다리는 사이 정지 신호가 오고, 정상 응답은 그 뒤에 온다 → 목표를 만들지 않고 STOPPED."""
    m, io = make()

    def stop_then_answer(io_):
        m.on_safety(SAFE_STOP)
        return True, '', good()

    io.find_script = [stop_then_answer]
    start(m)
    drive(m, 'STOPPED')
    assert not picks(io) and m.goal is None and m._scatter.candidate is None


def test_정지_뒤_다시_시작까지_끝난_다음에_온_늦은_응답도_적용하지_않는다():
    """정지 → 잠금 해제까지 조회 중에 다 지나가도(지금은 정지 중이 아님) 정지 전에 보낸 요청의 답은 버리고 STOPPED 절차를 탄다."""
    m, io = make()

    def stop_resume_then_answer(io_):
        m.on_safety(SAFE_STOP)
        m.on_safety(SAFE_OK)
        return True, '', good()

    io.find_script = [stop_resume_then_answer]
    start(m)
    drive(m, 'STOPPED')
    assert not picks(io) and m.goal is None


def fast_wait(m):
    """gripper 대기를 실제로 기다리지 않게 한다: sleep 은 가짜 시간(off)만 늘리고, 단조 시계·벽시계는 그 시간만큼 앞서 간다. 반환: [누적 가짜 시간, sleep 호출 수]."""
    state = [0.0, 0]
    base_clock, base_wall = m._clock, m._wall

    def sleep(s):
        state[0] += s
        state[1] += 1
        if m.sleep_hook:
            m.sleep_hook(state[1])
    m.sleep_hook = None
    m._sleep = sleep
    m._clock = lambda: base_clock() + state[0]
    m._wall = lambda: base_wall() + state[0]
    m.fake = state
    return state


def fail_then_ok(m, io, script, info):
    """집기 결과 script · 피드백/그리퍼 info 를 쓰고 끝까지(VERIFY 나 ERROR) 돌린다."""
    io.pick_script, io.pick_info = list(script), list(info)
    start(m)
    drive(m, ('VERIFY', 'ERROR'))


def test_접촉_전_계획_실패는_같은_후보로_다시_집는다():
    m, io = make()
    fail_then_ok(m, io, [(False, 'PLAN_FAILED')], [{'step': 'approach', 'grasped': False}])
    assert m.state == 'VERIFY' and len(picks(io)) == 2 and len(calls(io, 'find_blocks')) == 1
    assert io.goals[0]['pick_pose'] == io.goals[1]['pick_pose']      # 같은 관측 후보


def test_집기_실패_뒤_손이_비었으면_다시_관측해서_다시_집는다():
    m, io = make()
    fail_then_ok(m, io, [(False, 'GRASP_FAILED')], [{'step': 'grasp', 'grasped': False}])
    assert m.state == 'VERIFY' and len(picks(io)) == 2 and len(calls(io, 'find_blocks')) == 2
    assert [c[1] for c in io.calls if c[0] == 'move_to'].count('observe_supply') == 2


@pytest.mark.parametrize('info', [[{'step': 'grasp', 'grasped': True}], [{'step': 'grasp'}], []],
                         ids=['쥐고_있음', '결과_뒤_상태_없음', '결과_시각_없음'])
def test_쥐었거나_모르면_새_집기_없이_ERROR(info):
    m, io = make()
    fail_then_ok(m, io, [(False, 'GRASP_FAILED')], info)
    assert m.state == 'ERROR' and len(picks(io)) == 1 and len(calls(io, 'find_blocks')) == 1


def test_블록당_총_집기_2번이면_사람을_부른다():
    m, io = make()
    fail_then_ok(m, io, [(False, 'GRASP_FAILED')] * 3, [{'step': 'grasp', 'grasped': False}] * 3)
    assert m.state == 'ERROR' and len(picks(io)) == 2 and len(calls(io, 'find_blocks')) == 2
    assert '2번' in io.states[-1]['message']


def test_횟수는_재관측과_정지_복구로_비우지_않는다():
    m, io = make()
    fail_then_ok(m, io, [(False, 'GRASP_FAILED')] * 3, [{'step': 'grasp', 'grasped': False}] * 3)
    block = m.goal['part_id']
    assert m._scatter_fail == {block: 2}
    m.on_safety(SAFE_STOP)
    m.on_safety(SAFE_OK)
    assert m._scatter_fail == {block: 2}          # 정지 · 해제로 지워지지 않는다


def test_놓기_성공_직후에는_횟수를_두고_배치_확인_뒤에_지운다():
    m, io = make()
    fail_then_ok(m, io, [(False, 'GRASP_FAILED')], [{'step': 'grasp', 'grasped': False}])
    block = m.goal['part_id']
    assert m.state == 'VERIFY' and m._scatter_fail == {block: 2}      # PickPlace 성공 직후(실패 1번 + 성공 1번 = 시도 2번)
    m.run_once()                                                        # VERIFY 통과
    assert m.state == 'SELECT' and block not in m._scatter_fail


def test_새_설계_선택은_횟수를_비운다():
    m, io = make()
    m._scatter_fail = {'X': 1}
    assert m.command('select_design', 'bench') == (True, '')
    assert m._scatter_fail == {}


def test_TIMEOUT은_기존_처리로_재시도하지_않는다():
    m, io = make()
    io.pick_script, io.pick_info = [(False, 'TIMEOUT')], [{'step': 'move', 'grasped': False}]
    start(m)
    drive(m, ('ERROR', 'STOPPED'))
    assert len(picks(io)) == 1 and list(m._scatter_fail.values()) == [1]      # 시간 초과도 시도 한 번으로 센다


def test_scatter_설정_키가_없으면_만들_때_거절():
    cfg = copy.deepcopy(SCATTER_CFG)
    del cfg['scatter']
    with pytest.raises(KeyError):
        TaskManager(cfg, ScatterIO(), open_width_m=0.0)
    cfg = copy.deepcopy(SCATTER_CFG)
    cfg['scatter']['max_attempts'] = 0
    with pytest.raises(ValueError):
        TaskManager(cfg, ScatterIO(), open_width_m=0.0)


def test_공급_칸_방식은_find_blocks를_부르지_않는다():
    io = ScatterIO()
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    start(m)
    drive(m, 'DONE')
    assert not calls(io, 'find_blocks') and all(p[1] != '' for p in picks(io))


def test_commit과_PICK_PLACE_전환_사이에는_정지가_끼지_못한다():
    """commit 확인 → 목표 적용 → PICK_PLACE 전환이 한 잠금 안이라, 그 사이에 오는 정지 신호는 전환이 끝난 뒤에야 반영된다(Codex 검토).

    전환 도중(_start_goal 안)에 다른 스레드가 정지 신호를 보낸다. 잠금 때문에 전환이 끝날 때까지 막혀 있다가(그 사이 m.safety 는 그대로),
    전환 뒤 첫 run_once 가 STOPPED 로 보내고 집기 요청은 나가지 않는다. 예전처럼 잠금 밖에서 전환했다면 m.safety 가 이미 바뀌어 있다.
    """
    m, io = make()
    start(m)
    seen = {}
    real = m._start_goal

    def start_goal_with_stop(found):
        t = threading.Thread(target=m.on_safety, args=(SAFE_STOP,))
        t.start()
        time.sleep(0.15)                                    # 정지 스레드가 잠금에서 막혀 있을 시간
        seen['blocked'] = m.safety == SAFE_OK               # 아직 정지 신호가 반영되지 않았다
        seen['thread'] = t
        real(found)

    m._start_goal = start_goal_with_stop
    drive(m, 'PICK_PLACE')
    seen['thread'].join(2)
    assert seen['blocked'] is True and not seen['thread'].is_alive()
    assert m.safety == SAFE_STOP and not picks(io)
    m.run_once()                                            # 다음 단계 맨 앞에서 정지를 반영 → 집기 요청 전에 STOPPED
    assert m.state == 'STOPPED' and not picks(io)


# ---------- W178: 시도 한도 · 배치 확인 · gripper 대기 ----------
def one_fail(m, io, hook=None, info=None, why='GRASP_FAILED'):
    """첫 집기 하나가 why 로 실패하게 하고, sleep 때마다 hook(n)을 부르며 PICK_PLACE 실패 처리까지 한 단계씩 돌린다."""
    io.pick_script = [(False, why)]
    io.pick_info = [info if info is not None else {'step': 'grasp'}]     # 'grasped' 가 없으면 결과 뒤 gripper 상태는 안 온다
    m.sleep_hook = hook
    start(m)
    drive(m, 'PICK_PLACE')
    m.run_once()


def test_한도를_채운_뒤_복구로_와도_새_집기_요청을_보내지_않는다():
    m, io = make()
    fail_then_ok(m, io, [(False, 'GRASP_FAILED')] * 3, [{'step': 'grasp', 'grasped': False}] * 3)
    assert m.state == 'ERROR' and len(picks(io)) == 2
    m.on_safety(SAFE_STOP)
    m.on_safety(SAFE_OK)
    drive(m, 'PICK_PLACE')
    m.run_once()                                    # 같은 블록 — 한도를 넘는 요청은 보내지 않는다
    assert m.state == 'ERROR' and len(picks(io)) == 2


def test_배치가_present로_확인되지_않으면_횟수를_지우지_않는다():
    m, io = make()
    fail_then_ok(m, io, [(False, 'GRASP_FAILED')], [{'step': 'grasp', 'grasped': False}])
    block = m.goal['part_id']
    m.planner.judge = lambda **k: []                   # 오류가 없어도
    m._observe = lambda ids: m.planner.progress[m.placed].__setitem__('state', 'absent') or True    # 놓인 것이 확인되지 않으면
    m.run_once()
    assert m._scatter_fail.get(block) == 2


def test_결과_뒤_gripper_상태가_대기_중_도착하면_그_값으로_판단한다():
    m, io = make()
    arrive = lambda n: n == 2 and m.on_gripper({'stamp': io.last_at, 'grasped': False})
    one_fail(m, io, hook=arrive)
    assert m.fake[1] == 2 and m.fake[0] < 1.5            # 도착 즉시 끝, 상한까지 기다리지 않음
    assert m.state == 'SELECT'                           # 손이 비었음 → 다시 관측


def test_믿을_상태가_끝내_안_오면_상한_뒤_모르는_것으로_ERROR():
    m, io = make()
    one_fail(m, io)
    assert m.state == 'ERROR' and 1.45 <= m.fake[0] <= 1.55 and len(picks(io)) == 1       # 새 요청 없음, 마지막 sleep 은 남은 시간 이하
    assert '확인하지 못했다' in io.states[-1]['message']


@pytest.mark.parametrize('state', [
    lambda at: {'stamp': at - 5.0, 'grasped': False},          # 결과보다 이전 값
    lambda at: {'stamp': at + 1e6, 'grasped': False},          # 미래 stamp(시계 이상)
    lambda at: {'stamp': at, 'grasped': 'False'},              # bool 이 아님
], ids=['결과_이전', '미래', 'bool_아님'])
def test_믿을_수_없는_상태는_기다린_뒤에도_완화하지_않는다(state):
    m, io = make()
    m.sleep_hook = lambda n: m.on_gripper(state(io.last_at))
    one_fail(m, io, hook=m.sleep_hook)
    assert m.state == 'ERROR' and len(picks(io)) == 1


def test_이미_믿을_상태가_있으면_기다리지_않는다():
    m, io = make()
    one_fail(m, io, info={'step': 'grasp', 'grasped': False})
    assert m.fake[1] == 0 and m.state == 'SELECT'


def test_대기_중_정지가_끼면_ERROR나_새_목표를_적용하지_않고_STOPPED로_간다():
    m, io = make()
    one_fail(m, io, hook=lambda n: m.on_safety(SAFE_STOP) if n == 1 else None)
    assert m.state == 'PICK_PLACE' and 'ERROR' not in seen(io)         # 오래된 처리는 아무것도 적용하지 않는다
    m.run_once()
    assert m.state == 'STOPPED' and len(picks(io)) == 1 and len(calls(io, 'find_blocks')) == 1


def test_대기_중_요청이_바뀌거나_상태가_바뀌면_오래된_처리를_버린다():
    for hook in (lambda n: m.last_pick.__setitem__('request_id', 999) if n == 1 else None,
                 lambda n: setattr(m, '_epoch', m._epoch + 1) if n == 1 else None):
        m, io = make()
        one_fail(m, io, hook=hook)
        assert m.state == 'PICK_PLACE' and 'ERROR' not in seen(io) and len(picks(io)) == 1


def test_대기가_끝난_뒤_적용_직전에도_토큰을_다시_확인한다():
    m, io = make()
    orig = m._grasped_after_result

    def late_change(info, token):
        out = orig(info, token)
        m._epoch += 1                                     # 대기는 정상 반환했지만 그 직후 상태가 바뀜
        return out

    m._grasped_after_result = late_change
    one_fail(m, io, info={'step': 'grasp', 'grasped': False})
    assert m.state == 'PICK_PLACE' and 'ERROR' not in seen(io) and len(picks(io)) == 1 and len(calls(io, 'find_blocks')) == 1


def test_대기_중에는_잠금을_잡지_않아_다른_스레드가_막히지_않는다():
    m, io = make()
    done = []

    def other():
        t = threading.Thread(target=lambda: (m.on_gripper({'stamp': 0.0, 'grasped': False}), done.append(1)))
        t.start()
        t.join(timeout=1.0)
    one_fail(m, io, hook=lambda n: other() if n == 1 else None)
    assert done == [1]


@pytest.mark.parametrize('key,value', [
    ('max_attempts', True), ('max_attempts', 2.0), ('max_attempts', 1.5), ('max_attempts', 0), ('max_attempts', '2'),
    ('gripper_wait_s', 0), ('gripper_wait_s', True), ('gripper_wait_s', float('inf')), ('gripper_wait_s', float('nan')), ('gripper_wait_s', -1),
    ('gripper_max_age_s', 0), ('gripper_max_age_s', float('inf')), ('gripper_max_age_s', float('nan')), ('gripper_max_age_s', False)])
def test_scatter_설정_값이_정수나_유한한_양수가_아니면_거절(key, value):
    cfg = copy.deepcopy(SCATTER_CFG)
    cfg['scatter'][key] = value
    with pytest.raises(ValueError):
        TaskManager(cfg, ScatterIO(), open_width_m=0.0)


def test_scatter_wait_키가_없으면_거절():
    cfg = copy.deepcopy(SCATTER_CFG)
    del cfg['scatter']['gripper_wait_s']
    with pytest.raises(KeyError):
        TaskManager(cfg, ScatterIO(), open_width_m=0.0)

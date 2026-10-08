# -*- coding: utf-8 -*-
"""흩뿌린 공급(supply_mode: scatter) 연결 시험 — FakeIO 로 관측 → 후보 선택 → 대기·계속 → 정지 뒤 늦은 응답 무시까지 (W130, 로봇 없음).

실제 로봇 실행 · 열림 폭 확정 · W142 장애물(obstacles) 계약은 이 시험의 범위 밖이다. 이 시험은 TaskManager 의 목표 선택이
공급 칸(slots)과 흩뿌림(scatter)으로 갈라지고, 흩뿌림이 observe_supply 이동 → find_blocks 조회 → 후보 고르기 → PICK_PLACE 로 이어지며
정지 · 새 요청 뒤에 도착한 답은 적용되지 않는 것을 확인한다.
"""
import copy

import pytest

from d2_task.task_manager import TaskManager
from test_block_picker import blk
from test_task_manager import CFG, IDS, SAFE_OK, SAFE_STOP, FakeIO, drive, picks

SCATTER_CFG = dict(copy.deepcopy(CFG), supply_mode='scatter')


class ScatterIO(FakeIO):
    """find_blocks 가짜: find_script 를 한 칸씩 쓰고(함수면 부른다), 없으면 늘 정상 블록 몇 개를 돌려준다."""

    def __init__(self):
        super().__init__()
        self.find_script = []

    def find_blocks(self, run_id, should_abort):
        self.events.append(('call', 'find_blocks', run_id))
        step = self.find_script.pop(0) if self.find_script else (True, '', good())
        return step(self) if callable(step) else step


def good():
    """눕힘 블록 둘(긴 방향 · 짧은 방향 틈이 모두 넉넉함) — 어느 FLAT 잡기에도 맞는다."""
    return {'ok': True, 'blocks': [blk(x=0.30, length=40, width=40), blk(x=0.35, length=40, width=40)]}


def make(open_width_m=0.0865, cfg=SCATTER_CFG):
    io = ScatterIO()
    m = TaskManager(cfg, io, open_width_m=open_width_m)
    io.manager = m
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


def test_목표_블록은_작업_판단이_정하고_어디서_집을지만_방식이_정한다():
    """scatter 도 블록 순서 · 잡기 · 놓을 자세는 slots 와 같다(planner.next_target)."""
    m, io = make()
    start(m)
    drive(m, 'PICK_PLACE')
    t = m.planner.next_target()
    assert (m.goal['block_id'], m.goal['grasp'], m.goal['place_pose']) == (t['block_id'], t['grasp'], t['place_pose'])


def test_공급에_블록이_없으면_supply_empty로_기다리고_계속하면_다시_관측한다():
    m, io = make()
    io.find_script = [(True, '', {'ok': True, 'blocks': []})]
    start(m)
    drive(m, 'WAIT_SUPPLY')
    assert io.states[-1]['message_id'] == 'supply_empty' and not picks(io)
    finds = len(calls(io, 'find_blocks'))
    assert m.command('start') == (True, '')                                  # [계속]
    drive(m, 'PICK_PLACE')
    assert len(calls(io, 'find_blocks')) == finds + 1 and m.block_id == IDS[0]
    drive(m, 'DONE')


def test_계속은_알림을_낸_뒤에야_받는다():
    m, io = make()
    io.find_script = [(True, '', {'ok': True, 'blocks': []})]
    start(m)
    drive(m, 'WAIT_SUPPLY')
    assert m._supply_moved is True                                            # 이미 observe_supply 에 와 있어 도착 확인 없이 알림이 나갔다


@pytest.mark.parametrize('blocks, message_id', [
    ([blk(overlap='under')], None),                                            # 덮인 것뿐 → 맞는 블록 없음 안내(이름 미정이라 message_id 없음)
    ([blk(tilted=True)], 'tilted_block'),                                      # 기울어진 것만 남음 → IRD 알림
])
def test_맞는_블록이_없으면_이유를_알리고_기다린다(blocks, message_id):
    m, io = make()
    io.find_script = [(True, '', {'ok': True, 'blocks': blocks})]
    start(m)
    drive(m, 'WAIT_SUPPLY')
    last = io.states[-1]
    assert last['message_id'] == message_id and '맞는 블록이 없다' in last['message'] and not picks(io)


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


def test_흩뿌림_집기가_실패하면_재시도_연결_전이라_ERROR():
    m, io = make()
    io.pick_script = [(False, 'GRASP_FAILED')]
    start(m)
    drive(m, 'ERROR')
    assert 'GRASP_FAILED' in io.states[-1]['message'] and len(picks(io)) == 1


def test_공급_칸_방식은_find_blocks를_부르지_않는다():
    io = ScatterIO()
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    start(m)
    drive(m, 'DONE')
    assert not calls(io, 'find_blocks') and all(p[1] != '' for p in picks(io))

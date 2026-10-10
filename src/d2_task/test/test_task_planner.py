# -*- coding: utf-8 -*-
"""TaskPlanner 시험 (SDD 9.5 '가짜 진행표 5가지'). 레시피는 한세교 LV1 벤치 11개(001_CHAIR_BENCH), 설정은 실제 robot.yaml."""
import copy
import json
import time
from pathlib import Path

import pytest
import yaml

from d2_task.recipe_document import RecipeDocument
from d2_task.task_planner import TaskPlanner

SRC = Path(__file__).resolve().parents[2]
ROBOT = SRC / 'd2_robot' if (SRC / 'd2_robot/d2_bringup').is_dir() else SRC
CFG = yaml.safe_load((ROBOT / 'd2_bringup/config/robot.yaml').read_text(encoding='utf-8'))
_DOC = RecipeDocument.load(Path(__file__).parent / 'fixtures', '001_CHAIR_BENCH')
RECIPE, PLACEMENTS = _DOC.recipe, _DOC.placements       # 구조(recipe/2.0) · 조립 방법(placements/2.0)
IDS = [f'001_CHAIR_BENCH_{s["block"]}' for s in sorted(PLACEMENTS['steps'], key=lambda s: s['sequence'])]   # E-52 역할 블록 이름


def planner(present=0, **kw):
    """앞에서부터 present 개는 놓였고 나머지는 없는 진행표를 가진 TaskPlanner."""
    p = TaskPlanner(CFG, kw.get('recipe', RECIPE), PLACEMENTS)
    p.update_progress({b: {'state': 'present' if i < present else 'absent'} for i, b in enumerate(IDS)})
    return p


def test_빈_작업대면_첫_블록():
    r = planner(0).next_block()
    assert r['status'] == 'FOUND'
    assert r['block_id'] == IDS[0]
    assert r['grasp'] == 'FLAT_SHORT'          # 한세교 레시피 sequence 1 (다리 벽)
    assert r['supply_slot'] == '2'             # robot.yaml p2 = FLAT_SHORT 칸
    assert len(r['pick_pose'][0]) == 3 and len(r['pick_pose'][1]) == 4


def test_블록마다_잡기가_같은_칸을_준다():
    for n in range(11):
        r = planner(n).next_block()
        assert CFG['supply_slots'][int(r['supply_slot']) - 1]['grasp'] == r['grasp'], r['block_id']
    assert [planner(n).next_block()['supply_slot'] for n in (0, 8, 9)] == ['2', '2', '1']   # FLAT_SHORT p2(앞 9개), 마지막 둘 FLAT_LONG p1


def test_높이는_실측_블록으로_쌓아_올린다():
    z0, t = CFG['assembly_origin']['z_m'], CFG['block_actual_m'][2]
    assert planner(0).next_block()['place_pose'][0][2] == pytest.approx(z0 + t / 2)        # 1층 벽
    assert planner(2).next_block()['place_pose'][0][2] == pytest.approx(z0 + 1.5 * t)      # 2층 벽
    assert planner(8).next_block()['place_pose'][0][2] == pytest.approx(z0 + 4.5 * t)      # 벽 4층 위 좌판


def test_다_놓이면_DONE():
    assert planner(11).next_block() == {'status': 'DONE'}


def test_받침이_없으면_NO_SUPPORT():
    """레시피 읽기가 '받침은 먼저'를 보장해서 파일로는 만들 수 없다 — 진행표 쪽 방어를 만든 뒤 받침을 뒤 블록으로 바꿔 시험한다."""
    p = planner(0)
    p.blocks[0]['supports'] = [IDS[4]]
    assert p.next_block() == {'status': 'NO_SUPPORT', 'block_id': IDS[0]}


@pytest.mark.parametrize('state', ['occluded', 'unknown'])
def test_못_본_블록은_UNKNOWN_BLOCK(state):
    p = planner(2)
    p.update_progress({IDS[2]: {'state': state}})
    assert p.next_block() == {'status': 'UNKNOWN_BLOCK', 'block_id': IDS[2]}


def test_처음_관측_전에는_UNKNOWN_BLOCK():
    assert TaskPlanner(CFG, RECIPE, PLACEMENTS).next_block()['status'] == 'UNKNOWN_BLOCK'


def test_칸이_비면_WAIT_SUPPLY_채우면_이어_감():
    p = planner(0)
    p.mark_slot_empty(2)
    assert p.next_block() == {'status': 'WAIT_SUPPLY', 'block_id': IDS[0]}
    p.supply_refilled()
    assert p.next_block()['supply_slot'] == '2'


def test_다른_잡기_칸이_비어도_그대로():
    p = planner(0)
    for slot in (1, 3, 4, 5, 6):               # FLAT_SHORT 가 아닌 칸
        p.mark_slot_empty(slot)
    assert p.next_block()['supply_slot'] == '2'


def test_잘못된_관측은_아무것도_안_바꾼다():
    p = planner(0)
    with pytest.raises(ValueError):
        p.update_progress({IDS[0]: {'state': 'present'}, 'NOPE_B001': {'state': 'present'}})
    with pytest.raises(ValueError):
        p.update_progress({IDS[0]: {'state': '있음'}})
    assert p.next_block()['block_id'] == IDS[0]


def test_없는_공급_칸_번호는_거부():
    with pytest.raises(ValueError):
        planner(0).mark_slot_empty(7)


def cfg_two_flat_short_slots():
    """FLAT_SHORT 칸이 둘인 robot.yaml 복사본 (실제 robot.yaml 은 잡기마다 칸이 하나)."""
    cfg = copy.deepcopy(CFG)
    extra = dict(cfg['supply_slots'][1], x_m=cfg['supply_slots'][1]['x_m'] + 0.05)
    cfg['supply_slots'].append(extra)                      # 7번 칸, p2 와 같은 FLAT_SHORT
    return cfg


def test_avoid_slots로_다른_칸을_받는다():
    p = TaskPlanner(cfg_two_flat_short_slots(), RECIPE, PLACEMENTS)
    p.update_progress({b: {'state': 'absent'} for b in IDS})
    first = p.next_block()
    second = p.next_block(avoid_slots=(int(first['supply_slot']),))
    assert first['status'] == 'FOUND'
    assert second['status'] == 'FOUND'
    assert second['block_id'] == first['block_id']          # 같은 블록, 칸만 다름
    assert second['supply_slot'] != first['supply_slot']
    assert second['grasp'] == first['grasp']
    assert p.next_block()['supply_slot'] == first['supply_slot']   # 피하기는 그 호출 한 번뿐(빔으로 적지 않음)


def test_avoid_slots는_문자열_번호도_받는다():
    p = TaskPlanner(cfg_two_flat_short_slots(), RECIPE, PLACEMENTS)
    p.update_progress({b: {'state': 'absent'} for b in IDS})
    first = p.next_block()
    assert p.next_block(avoid_slots=[first['supply_slot']])['supply_slot'] != first['supply_slot']


def test_후보가_하나뿐이면_avoid_slots에서_WAIT_SUPPLY():
    p = planner(0)
    first = p.next_block()
    result = p.next_block(avoid_slots=(int(first['supply_slot']),))
    assert result == {'status': 'WAIT_SUPPLY', 'block_id': IDS[0]}   # '공급 없음'이 아니라 '다른 칸 없음' — TaskManager 가 구분


def test_judge_정상이면_문제없음():
    p = planner(3)
    p.update_progress({IDS[2]: {'state': 'present', 'dz_m': 0.001}})
    assert p.judge(expect_present=[IDS[2]]) == []


def test_judge_놓은_블록이_없으면_OFFSET_OVER():
    p = planner(2)
    assert [x['reason'] for x in p.judge(expect_present=[IDS[2]])] == ['OFFSET_OVER']


def test_judge_높이는_두께_절반이_경계():
    half = CFG['block_actual_m'][2] / 2
    p = planner(1)
    p.update_progress({IDS[0]: {'state': 'present', 'dz_m': half}})
    assert p.judge() == []                                         # 경계값은 통과
    p.update_progress({IDS[0]: {'state': 'present', 'dz_m': half + 0.0001}})
    assert p.judge()[0]['block_id'] == IDS[0]


def test_judge_받침_없이_위층만_있으면_OFFSET_OVER():
    p = planner(0)
    p.update_progress({IDS[0]: {'state': 'absent'}, IDS[2]: {'state': 'present'}})   # 3번(받침 1번)만 있음
    assert [x['block_id'] for x in p.judge()] == [IDS[2]]


def test_judge_못_본_블록은_문제로_안_본다():
    p = planner(2)
    p.update_progress({IDS[1]: {'state': 'occluded'}, IDS[2]: {'state': 'unknown'}})
    assert p.judge(expect_present=[IDS[1], IDS[2]]) == []


def test_judge_없는_블록_이름은_거부():
    with pytest.raises(ValueError):
        planner(0).judge(expect_present=['NOPE'])


def test_progress_message는_NaN_없는_JSON():
    p = planner(2)
    p.update_progress({IDS[0]: {'state': 'present', 'top_z_m': 0.0123}})
    before = time.time()
    msg = p.progress_message(None, 12.5)
    text = json.dumps(msg, allow_nan=False)                        # NaN 이 남아 있으면 ValueError
    got = json.loads(text)
    assert got['schema'] == 'progress/1.1' and got['run_id'] is None and got['obs_stamp'] == 12.5
    assert before <= got['stamp'] <= time.time()                   # 보낸 시각(E-75) — 관측한 시각(obs_stamp)과 따로다
    assert len(got['blocks']) == 11 and got['blocks'][0]['by'] == 'robot'
    assert got['blocks'][0]['top_z_m'] == 0.0123 and got['blocks'][0]['dx_m'] is None


def test_빔으로_적은_칸은_다음_호출에서도_계속_제외_채우면_복귀():
    p = TaskPlanner(cfg_two_flat_short_slots(), RECIPE, PLACEMENTS)
    p.update_progress({b: {'state': 'absent'} for b in IDS})
    first = p.next_block()
    p.mark_slot_empty(int(first['supply_slot']))
    for _ in range(3):                                        # 몇 번을 불러도 빈 칸은 안 나온다
        again = p.next_block()
        assert again['status'] == 'FOUND' and again['supply_slot'] != first['supply_slot']
    p.supply_refilled()
    assert p.next_block()['supply_slot'] == first['supply_slot']   # 채운 뒤에는 다시 후보


def test_빈_칸이_다_차면_WAIT_SUPPLY_채우면_FOUND():
    p = TaskPlanner(cfg_two_flat_short_slots(), RECIPE, PLACEMENTS)
    p.update_progress({b: {'state': 'absent'} for b in IDS})
    for slot in (2, 7):                                       # FLAT_SHORT 칸 둘 다 빔
        p.mark_slot_empty(slot)
    assert p.next_block() == {'status': 'WAIT_SUPPLY', 'block_id': IDS[0]}
    p.supply_refilled()
    assert p.next_block()['status'] == 'FOUND'


def test_next_target은_어디서_집을지_빼고_next_block과_같은_블록을_고른다():
    """목표 선택(블록 · 잡기 · 놓을 자세)과 공급 칸 선택을 가른 것: next_block = next_target + 칸."""
    for n in (0, 3, 8):
        p = planner(n)
        t, b = p.next_target(), p.next_block()
        assert t['status'] == b['status'] == 'FOUND'
        assert (t['block_id'], t['grasp'], t['place_pose']) == (b['block_id'], b['grasp'], b['place_pose'])
        assert 'supply_slot' not in t and 'pick_pose' not in t


def test_next_target은_진행표를_바꾸지_않고_DONE_UNKNOWN_NO_SUPPORT도_같다():
    assert planner(11).next_target() == {'status': 'DONE'}
    p = planner(2)
    p.update_progress({IDS[2]: {'state': 'unknown'}})
    assert p.next_target() == p.next_block() == {'status': 'UNKNOWN_BLOCK', 'block_id': IDS[2]}
    before = copy.deepcopy(p.progress)
    p.next_target()
    assert p.progress == before

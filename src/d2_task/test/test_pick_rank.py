# -*- coding: utf-8 -*-
"""흩뿌린 블록 집기 순서(PickRanker · BlockPicker.rank)와 같은 층 1 mm 띄우기(spread_parts) 시험 — ROS 없이(10/13 박진용 확정안).

실기 장면은 10/13 11:59 실기(레시피 4번, 시험 도구 one_pick.py — 블록 11/11 · 집기 24/24 · 헛시도 0)의 '찾은 블록' 목록을 그대로 옮긴 것이다
(fixtures/pick_rank_scenes_1013.json). 기대 순서는 그 실기에 쓴 one_pick.candidates 의 결과(깊이 점 없이)다. 로그에 없는 gap_mm 은 50 mm 로 채웠다.
"""
import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from d2_motion.motion_math import load_recipe, recipe_parts
from d2_task.block_picker import BlockPicker
from d2_task.pick_rank import PickRanker, spread_parts

SRC = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load(next(SRC.glob('d2_*/d2_bringup/config/robot.yaml')).read_text(encoding='utf-8'))
SCENES = json.loads((Path(__file__).parent / 'fixtures' / 'pick_rank_scenes_1013.json').read_text(encoding='utf-8'))
RECIPES = SRC / 'recipe_manager' / 'recipes'
RANK = PickRanker(CFG)
T = CFG['block_actual_m'][2]
OWN = {'THICKNESS': CFG['block_actual_m'][2], 'WIDTH': CFG['block_actual_m'][1], 'LENGTH': CFG['block_actual_m'][0]}


def blk(x, y, up='THICKNESS', yaw=0.0, top=None, ov='none', tilted=False, gap=50.0, **kw):
    """정상 find_blocks 블록 하나(틈 gap mm 은 수평 두 축 모두). top 을 안 주면 작업대에 바로 놓인 높이."""
    hz = {'THICKNESS': ('LENGTH', 'WIDTH'), 'WIDTH': ('LENGTH', 'THICKNESS'), 'LENGTH': ('WIDTH', 'THICKNESS')}[up]
    b = {'x_m': x, 'y_m': y, 'top_z_m': CFG['table_z_m'] + OWN[up] if top is None else top, 'up': up, 'yaw_deg': yaw, 'overlap': ov,
         'tilted': tilted, 'confidence': 0.9, 'gap_mm': {a: gap for a in hz}, 'clear': {a: True for a in hz}}
    b.update(kw)
    return b


def rank(blocks, failed=(), pts=None):
    return RANK.rank(blocks, failed, pts)


def first(blocks):
    c, _ = rank(blocks)
    return c[0]


def tilted_flat(x, y, **kw):
    """다른 블록에 걸친 기울어진 눕힌 블록(윗면 가운데 · 손끝 높이를 잰 값이 붙은 꼴)."""
    return blk(x, y, top=0.035, tilted=True, center_top_z_m=0.025, tilt_tip_z_m=0.018, **kw)


# ---------------- 실기 장면 ----------------
@pytest.mark.parametrize('scene', SCENES, ids=[s['설명'][20:48] for s in SCENES])
def test_실기_장면_순서가_one_pick_과_같다(scene):
    c, cut = rank(scene['blocks'])
    assert [[i, g, d, round(s, 6)] for i, g, d, s, _ in c[:5]] == scene['expected_first5']
    assert len(c) == scene['expected_count']
    assert cut == {'깊이': 0, '블록': set()}


def test_BlockPicker_rank_는_PickRanker_와_같다():
    s = SCENES[0]['blocks']
    assert BlockPicker(CFG['find']['min_gap_mm'], CFG).rank(s) == rank(s)


def test_cfg_없이_만든_BlockPicker_rank_는_오류():
    with pytest.raises(ValueError):
        BlockPicker(CFG['find']['min_gap_mm']).rank([blk(0.45, 0.2)])


def test_목록이_아니면_오류():
    with pytest.raises(ValueError):
        RANK.rank({'blocks': []})


# ---------------- 규칙 ----------------
def test_혼자_선_블록보다_눕힌_블록_곧게_먼저():
    f = first([blk(0.45, 0.20, 'LENGTH'), blk(0.45, 0.35)])
    assert f[0] == 1 and f[4].startswith('2칸')


def test_안_덮은_기울어진_블록보다_눕힌_블록_곧게_먼저():
    assert first([tilted_flat(0.45, 0.20), blk(0.45, 0.35)])[0] == 1


def test_남을_덮은_기울어진_블록은_1칸_치우면_열리는_수_2_이상():
    B = [tilted_flat(0.45, 0.20), blk(0.47, 0.20, ov='under'), blk(0.45, 0.35)]
    f = first(B)
    assert f[0] == 0 and f[1] == 'TILT' and f[4].startswith('1칸')
    assert RANK.freed(B, 0) >= PickRanker.COVER_MIN_FREE


def test_기울여_집기_4칸이_옆으로_선_블록_5칸보다_먼저():
    f = first([blk(0.45, 0.20, 'WIDTH'), blk(0.30, 0.20, top=0.035, tilted=True, center_top_z_m=0.025, tilt_tip_z_m=0.018)])
    assert f[1] == 'TILT' and f[4].startswith('4칸')


def test_기울어진_옆으로_선_블록과_높이를_안_잰_기울어진_블록은_후보_아님():
    assert rank([blk(0.45, 0.2, 'WIDTH', tilted=True, center_top_z_m=0.02, tilt_tip_z_m=0.012)])[0] == []
    assert rank([blk(0.45, 0.2, top=0.035, tilted=True)])[0] == []


def test_넓은_폭_2점4mm_옮김은_2칸_좁은_폭_20도_틀기는_3칸이고_곧게_넓은_폭이_먼저():
    c, _ = rank([blk(0.45, 0.2)])
    tier = {(g, d, round(s * 1000, 1)): t[:2] for _, g, d, s, t in c}
    order = {(g, d, round(s * 1000, 1)): n for n, (_, g, d, s, _) in enumerate(c)}
    assert tier[('FLAT_LONG', 0, 2.4)] == '2칸' and tier[('FLAT_SHORT', 20, 0.0)] == '3칸'
    assert order[('FLAT_LONG', 0, 0.0)] < order[('FLAT_SHORT', 20, 0.0)]


def test_큰_옮김은_덮은_블록이라도_6칸():
    # 덮은 블록 0 의 양옆(y)에 높은 블록을 −x 쪽으로 치우쳐 두면 가운데(옮김 0)는 막히고 +x 끝 쪽으로 옮겨 잡기만 열린다
    gy = 0.0248 + 0.004
    B = [blk(0.45, 0.2, top=2 * T, ov='top'), blk(0.45, 0.2, ov='under'), blk(0.42, 0.2 + gy, top=0.04), blk(0.42, 0.2 - gy, top=0.04),
         blk(0.30, 0.35)]
    assert RANK.covers(B, 0)
    c, _ = rank(B)
    short0 = [(s, t) for i, g, d, s, t in c if i == 0 and g == 'FLAT_SHORT']
    assert short0 and all(abs(s) >= 0.005 for s, _ in short0)          # 좁은 폭은 옮겨 잡기만 남는다
    assert all(t.startswith('6칸') for _, t in short0)


def test_같은_칸_같은_층에서_치우면_열리는_블록_먼저():
    # 0 혼자 · 1 곧게 잡힘 · 2 는 1(한쪽)과 3 · 4(길이 끝 양쪽)에 막혀 곧게 안 됨 → 1 을 치우면 2 가 곧게 열린다
    B = [blk(0.30, 0.10), blk(0.45, 0.28), blk(0.45, 0.25), blk(0.535, 0.25), blk(0.365, 0.25)]
    assert RANK.freed(B, 0) == 0 and RANK.freed(B, 1) > 0
    assert first(B)[0] == 1


def test_층은_바닥_높이로_센다():
    assert RANK.layer_of(blk(0, 0, 'LENGTH')) == 1          # 작업대에 혼자 선 블록
    assert RANK.layer_of(blk(0, 0, 'WIDTH')) == 1
    assert RANK.layer_of(blk(0, 0, top=2 * T)) == 2         # 눕힌 블록 위 눕힌 블록


def test_덮인_블록_옆에_붙어만_있는_블록은_덮은_블록이_아니다():
    B = [blk(0.45, 0.2, 'WIDTH'), blk(0.45, 0.2 + 0.0196, ov='under')]      # 사각형이 1 mm 남짓 겹치지만 overlap=none
    assert not RANK.covers(B, 0)


def test_실패한_시도는_빠진다():
    B = [blk(0.45, 0.2)]
    a = first(B)
    c, _ = rank(B, failed=[(0.45, 0.2, a[1], a[2], a[3])])
    assert all(x[1:4] != a[1:4] for x in c)


def test_입력을_바꾸지_않고_같은_입력이면_같은_순서():
    B = copy.deepcopy(SCENES[1]['blocks'])
    B0 = copy.deepcopy(B)
    assert rank(B) == rank(B)
    assert B == B0


# ---------------- 깊이로 미리 거르기 ----------------
def test_깊이로_손가락_자리가_막힌_후보는_줄_세우기_전에_빠지고_수를_센다():
    B = [blk(0.45, 0.2)]
    c, _ = rank(B)
    a = c[0]
    fingers, tip = RANK.candidate_fingers(B[0], a[1], a[2], a[3])
    cx, cy = fingers[0][:2]
    pts = np.array([[cx + dx, cy + dy, tip + 0.01] for dx in np.linspace(-0.002, 0.002, 10) for dy in np.linspace(-0.002, 0.002, 10)])
    c2, cut = rank(B, pts=pts)
    assert all(x[1:4] != a[1:4] for x in c2)
    assert cut['깊이'] >= 1 and cut['블록'] == {1}


def test_바닥_점뿐이면_아무_후보도_안_빠진다():
    B = [blk(0.45, 0.2)]
    flat = np.array([[0.45 + dx, 0.2 + dy, 0.0] for dx in np.linspace(-0.06, 0.06, 40) for dy in np.linspace(-0.06, 0.06, 40)])
    c, _ = rank(B)
    c2, cut = rank(B, pts=flat)
    assert len(c2) == len(c) and cut['깊이'] == 0


def test_점이_모자란_손가락은_막힘으로_안_본다():
    fingers = RANK.finger_rects(0.45, 0.2, 0.0, 0.02)
    few = np.array([[fingers[0][0], fingers[0][1], 1.0]] * (PickRanker.FINGER_MIN_PTS - 1))
    assert RANK.fingers_blocked(fingers, 0.0, few) == []


# ---------------- 같은 층 1 mm 띄우기 ----------------
def moved(model):
    """레시피 하나의 띄우기 결과 {블록 이름 끝: (dx, dy) mm} — 옮긴 블록만."""
    parts = recipe_parts(CFG, load_recipe(str(RECIPES / f'{model}_recipe.json')))
    out = spread_parts(parts, CFG)
    return {p['part_id'].split('_V000_')[1]: (round((n[0] - p['center'][0]) * 1000, 1), round((n[1] - p['center'][1]) * 1000, 1))
            for p, n in zip(parts, out) if math.hypot(n[0] - p['center'][0], n[1] - p['center'][1]) > 1e-6}


def test_띄우기_레시피_1_2_좌판_10_11번():
    # 10/13 실기 로그 '같은 층 옆 블록 틈 1 mm 띄우기: _SEAT_001_02 (-0.0, +0.8) mm, _SEAT_001_03 (-0.0, +1.6) mm'
    for model in ('001_CHAIR_BENCH_V000', '002_CHAIR_BACK_V000'):
        assert moved(model) == {'SEAT_001_02': (0.0, 0.8), 'SEAT_001_03': (0.0, 1.6)}


def test_띄우기_레시피_4_바닥_위판_2_3번():
    # 10/13 실기 로그 '_BASE_001_02 (-0.0, +0.8) · _BASE_001_03 (-0.0, +1.6) · 0_TOP_001_02 (-0.0, +0.8) · 0_TOP_001_03 (-0.0, +1.6)'
    assert moved('004_DESK_PEDESTAL_V000') == {'BASE_001_02': (0.0, 0.8), 'BASE_001_03': (0.0, 1.6),
                                                'TOP_001_02': (0.0, 0.8), 'TOP_001_03': (0.0, 1.6)}


def test_띄우기는_입력을_바꾸지_않는다():
    parts = recipe_parts(CFG, load_recipe(str(RECIPES / '004_DESK_PEDESTAL_V000_recipe.json')))
    before = copy.deepcopy(parts)
    spread_parts(parts, CFG)
    assert [p['center'] for p in parts] == [p['center'] for p in before]


def test_더미_위_블록은_눕힘이고_확신도가_넉넉할_때만():
    """10/13 004: 더미 위에 비스듬히 걸친 눕힌 블록을 '옆으로 섬'(확신도 0.59)으로 보고 집어 받침 위 약 12 mm 에서 놓임 —
    더미 위(overlap top)는 눕힘 · 확신도 STACK_CONF_MIN 이상만 후보. 덮이지 않은 블록(none)은 확신도와 상관없이 그대로."""
    edge_on_top = blk(0.40, 0.20, up='WIDTH', ov='top', confidence=0.9)
    low_on_top = blk(0.40, 0.20, ov='top', confidence=PickRanker.STACK_CONF_MIN - 0.01)
    ok_on_top = blk(0.40, 0.20, ov='top', confidence=PickRanker.STACK_CONF_MIN)
    low_alone = blk(0.40, 0.20, confidence=0.3)
    assert rank([edge_on_top])[0] == [] and rank([low_on_top])[0] == []
    assert rank([ok_on_top])[0] and rank([low_alone])[0]

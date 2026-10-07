# -*- coding: utf-8 -*-
"""집을 블록 고르기 BlockPicker 시험 (W130) — 가짜 find_blocks 입력으로 규칙을 확인한다(실제 집기 연결 전)."""
import copy
from pathlib import Path

import pytest
import yaml

from d2_task.block_picker import BlockPicker, grasp_target

SRC = Path(__file__).resolve().parents[2]
CFG = yaml.safe_load(next(SRC.glob('d2_*/d2_bringup/config/robot.yaml')).read_text(encoding='utf-8'))
MIN_GAP = CFG['find']['min_gap_mm']


def blk(x=0.4, y=0.2, top=-0.003, up='THICKNESS', length=30.0, width=30.0, overlap='none', tilted=False, **over):
    """정상 find_blocks 블록 하나. length · width = 그 방향 틈(mm), clear 는 기준에 맞게 채운다."""
    b = {'x_m': x, 'y_m': y, 'top_z_m': top, 'yaw_deg': 10.0, 'up': up,
         'gap_mm': {'LENGTH': length, 'WIDTH': width}, 'clear': {'LENGTH': length >= MIN_GAP, 'WIDTH': width >= MIN_GAP},
         'overlap': overlap, 'tilted': tilted, 'confidence': 0.9}
    b.update(over)
    return b


def pick(blocks, up='THICKNESS', axis='LENGTH'):
    return BlockPicker(MIN_GAP).pick(blocks, up, axis)


def test_기준_틈은_robot_yaml에서_읽는다():
    assert MIN_GAP == 22.2


def test_잡기_이름에서_자세와_잡는_축():
    assert grasp_target('FLAT_LONG', CFG) == ('THICKNESS', 'LENGTH')
    assert grasp_target('FLAT_SHORT', CFG) == ('THICKNESS', 'WIDTH')
    assert grasp_target('EDGE_LONG', CFG) == ('WIDTH', 'LENGTH')
    assert grasp_target('STAND_LONG', CFG) == ('LENGTH', 'WIDTH')


def test_닫는_치수가_두께인_잡기도_기하로_정해진다():
    assert grasp_target('EDGE_SHORT', CFG) == ('WIDTH', 'THICKNESS')
    assert grasp_target('STAND_SHORT', CFG) == ('LENGTH', 'THICKNESS')


def test_여섯_잡기_전부_자세와_축이_정해진다():
    got = {g: grasp_target(g, CFG) for g in CFG['grasp_width_m']}
    assert got == {'FLAT_SHORT': ('THICKNESS', 'WIDTH'), 'FLAT_LONG': ('THICKNESS', 'LENGTH'),
                   'EDGE_SHORT': ('WIDTH', 'THICKNESS'), 'EDGE_LONG': ('WIDTH', 'LENGTH'),
                   'STAND_SHORT': ('LENGTH', 'THICKNESS'), 'STAND_LONG': ('LENGTH', 'WIDTH')}


def test_두께_틈이_응답에_없으면_NONE이고_있으면_고른다():
    up, axis = grasp_target('EDGE_SHORT', CFG)
    no_info = blk(up=up, length=80, width=80)                 # 두께 틈 칸이 없는 응답
    r = pick([no_info], up, axis)
    assert r['status'] == 'NONE' and r['counts']['no_gap_info'] == 1
    with_info = blk(up=up, length=80, width=80)
    with_info['gap_mm']['THICKNESS'], with_info['clear']['THICKNESS'] = 30.0, True
    r = pick([no_info, with_info], up, axis)
    assert (r['status'], r['index'], r['gap_mm']) == ('FOUND', 1, 30.0)
    narrow = copy.deepcopy(with_info)
    narrow['gap_mm']['THICKNESS'], narrow['clear']['THICKNESS'] = 10.0, False
    assert pick([narrow], up, axis)['counts']['no_clear'] == 1


def test_두께_틈_칸_형식이_틀리면_잘못된_값():
    up, axis = grasp_target('STAND_SHORT', CFG)
    b = blk(up=up)
    b['gap_mm']['THICKNESS'], b['clear']['THICKNESS'] = 'x', True
    assert pick([b], up, axis)['counts']['invalid'] == 1


def test_FLAT_LONG_이름에서_구한_자세와_축으로_FOUND와_거부():
    up, axis = grasp_target('FLAT_LONG', CFG)
    ok = blk(up=up, length=40)
    assert pick([ok], up, axis)['status'] == 'FOUND'
    assert pick([blk(up='WIDTH', length=40)], up, axis)['status'] == 'NONE'            # 다른 자세는 거부
    assert pick([blk(up=up, length=10)], up, axis)['status'] == 'NONE'                # 틈 부족은 거부
    up2, axis2 = grasp_target('EDGE_LONG', CFG)
    assert pick([blk(up='WIDTH', length=40)], up2, axis2)['status'] == 'FOUND'


def test_모르는_잡기():
    with pytest.raises(ValueError):
        grasp_target('FLAT_MIDDLE', CFG)


def test_틈이_가장_넓은_것을_고른다():
    r = pick([blk(length=25), blk(length=40), blk(length=30)])
    assert (r['status'], r['index'], r['gap_mm']) == ('FOUND', 1, 40)


def test_잡는_축의_틈만_본다():
    r = pick([blk(length=25, width=90), blk(length=30, width=1)], axis='LENGTH')
    assert r['index'] == 1
    assert pick([blk(length=25, width=90), blk(length=30, width=1)], axis='WIDTH')['index'] == 0


def test_기준_틈보다_좁으면_제외():
    assert pick([blk(length=22.1)])['reason'] == 'NO_MATCH'
    assert pick([blk(length=22.2)])['status'] == 'FOUND'          # 같으면 통과(≥)


def test_clear가_거짓이면_틈_숫자가_넓어도_제외():
    b = blk(length=50)
    b['clear']['LENGTH'] = False
    r = pick([b])
    assert r['status'] == 'NONE' and r['counts']['no_clear'] == 1


def test_목표_자세와_다른_블록은_제외():
    r = pick([blk(up='WIDTH', length=80), blk(up='LENGTH', length=80), blk(up='THICKNESS', length=25)])
    assert r['index'] == 2
    assert pick([blk(up='WIDTH')], up='THICKNESS')['counts']['other_up'] == 1


def test_덮인_것과_기울어진_것은_제외():
    r = pick([blk(overlap='under', length=90), blk(tilted=True, length=90)])
    assert r['status'] == 'NONE' and r['counts']['under'] == 1 and r['counts']['tilted'] == 1


def test_none이_있으면_틈이_더_넓은_top보다_none을_고른다():
    r = pick([blk(overlap='top', length=90), blk(overlap='none', length=25)])
    assert (r['index'], r['overlap']) == (1, 'none')


def test_none이_없으면_top에서_고른다():
    r = pick([blk(overlap='top', length=30), blk(overlap='top', length=50), blk(overlap='under', length=90)])
    assert (r['index'], r['overlap']) == (1, 'top')


def test_none_후보가_기준에_못_미치면_top으로_넘어간다():
    r = pick([blk(overlap='none', length=10), blk(overlap='top', length=30)])
    assert r['index'] == 1


def test_틈이_같으면_윗면이_높은_것():
    r = pick([blk(length=30, top=-0.01), blk(length=30, top=0.02), blk(length=30, top=0.0)])
    assert r['index'] == 1


def test_틈과_높이가_같으면_입력_순서가_앞선_것():
    r = pick([blk(length=30, top=0.0), blk(length=30, top=0.0), blk(length=30, top=0.0)])
    assert r['index'] == 0


def test_높이는_틈보다_뒤_기준():
    r = pick([blk(length=45, top=-0.02), blk(length=30, top=0.05)])
    assert r['index'] == 0


def test_후보가_없으면_다른_자세를_바로_집지_않는다():
    r = pick([blk(up='WIDTH', length=90), blk(up='LENGTH', length=90)], up='THICKNESS')
    assert r == {'status': 'NONE', 'reason': 'NO_MATCH', 'counts': dict(r['counts'])} and r['counts']['other_up'] == 2


def test_블록이_빈_목록이면_EMPTY():
    assert pick([]) == {'status': 'NONE', 'reason': 'EMPTY', 'counts': {}}


@pytest.mark.parametrize('blocks', [None, 'x', {}, 5, (blk(),)])
def test_블록이_목록이_아니면_부르는_쪽_실수로_ValueError(blocks):
    with pytest.raises(ValueError):
        pick(blocks)


BAD = [
    lambda b: b.pop('up'), lambda b: b.update(up='SIDE'), lambda b: b.update(overlap='half'), lambda b: b.update(overlap=None),
    lambda b: b.update(tilted='no'), lambda b: b.update(tilted=None), lambda b: b.pop('clear'), lambda b: b.update(clear={'LENGTH': 'yes', 'WIDTH': True}),
    lambda b: b.update(clear={'LENGTH': True}), lambda b: b.update(gap_mm={'LENGTH': 'x', 'WIDTH': 30}),
    lambda b: b.update(gap_mm={'LENGTH': float('nan'), 'WIDTH': 30}), lambda b: b.update(gap_mm={'LENGTH': float('inf'), 'WIDTH': 30}),
    lambda b: b.update(gap_mm=None), lambda b: b.update(x_m=float('nan')), lambda b: b.update(y_m=None), lambda b: b.update(top_z_m='1'),
    lambda b: b.update(yaw_deg=float('inf')), lambda b: b.update(x_m=True),
    lambda b: b.update(yaw_deg=90.0), lambda b: b.update(yaw_deg=-90.1), lambda b: b.update(yaw_deg=1e308 * 10), lambda b: b.update(x_m=1e999),
    lambda b: b.update(clear=[]), lambda b: b.update(gap_mm='30'), lambda b: b.update(clear={'LENGTH': True, 'WIDTH': True, 'THICKNESS': 'yes'}),
]


@pytest.mark.parametrize('mutate', BAD)
def test_잘못된_값의_블록은_건너뛰고_나머지로_고른다(mutate):
    bad = blk(length=90)
    mutate(bad)
    r = pick([bad, blk(length=30)])
    assert (r['status'], r['index']) == ('FOUND', 1)
    assert pick([bad])['counts']['invalid'] == 1


def test_yaw_경계는_음수_90_포함_90_제외():
    assert pick([blk(yaw_deg=-90.0)])['status'] == 'FOUND'
    assert pick([blk(yaw_deg=89.999)])['status'] == 'FOUND'
    assert pick([blk(yaw_deg=90.0)])['counts']['invalid'] == 1


def test_잘못된_값이_섞여도_남은_후보_선택은_안정적():
    good = [blk(length=30, top=0.0), blk(length=45, top=-0.01), blk(length=45, top=0.02)]
    bad = [blk(yaw_deg=float('nan')), blk(yaw_deg=200), '글자', None, blk(x_m=float('inf'))]
    blocks = [bad[0], good[0], bad[1], good[1], bad[2], good[2], bad[3], bad[4]]
    r = pick(blocks)
    assert (r['status'], r['index'], r['gap_mm']) == ('FOUND', 5, 45)
    assert pick(list(reversed(blocks)))['block'] == r['block']
    assert pick(blocks)['index'] == r['index']                      # 같은 입력은 같은 결과


def test_블록이_객체가_아니어도_건너뛴다():
    r = pick(['글자', None, 5, blk()])
    assert r['index'] == 3


def test_입력은_바뀌지_않고_결과는_복사본():
    blocks = [blk(length=30), blk(length=40)]
    before = copy.deepcopy(blocks)
    r = pick(blocks)
    r['block']['gap_mm']['LENGTH'] = -1
    assert blocks == before


@pytest.mark.parametrize('gap', [0, -1, None, 'x', float('nan'), True])
def test_기준_틈이_이상하면_설정_오류(gap):
    with pytest.raises(ValueError):
        BlockPicker(gap)


@pytest.mark.parametrize('up, axis', [('FLAT', 'LENGTH'), ('THICKNESS', 'DEPTH'), (None, 'LENGTH')])
def test_목표_자세나_축이_틀리면_오류(up, axis):
    with pytest.raises(ValueError):
        pick([blk()], up, axis)


def test_E49_틈_경계_예시_IRD():
    """IRD 응답 예: LENGTH 26.0 은 통과, WIDTH 8.5 는 clear=false."""
    b = blk(length=26.0, width=8.5)
    assert pick([b], axis='LENGTH')['status'] == 'FOUND'
    assert pick([b], axis='WIDTH')['status'] == 'NONE'

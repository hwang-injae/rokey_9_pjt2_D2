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


@pytest.mark.parametrize('grasp', ['EDGE_SHORT', 'STAND_SHORT'])
def test_닫는_치수가_두께면_틈_방향을_모르므로_추측하지_않는다(grasp):
    with pytest.raises(ValueError):
        grasp_target(grasp, CFG)


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


@pytest.mark.parametrize('blocks', [[], None, 'x', {}])
def test_블록이_없거나_목록이_아니면_EMPTY(blocks):
    assert pick(blocks) == {'status': 'NONE', 'reason': 'EMPTY', 'counts': {}}


BAD = [
    lambda b: b.pop('up'), lambda b: b.update(up='SIDE'), lambda b: b.update(overlap='half'), lambda b: b.update(overlap=None),
    lambda b: b.update(tilted='no'), lambda b: b.update(tilted=None), lambda b: b.pop('clear'), lambda b: b.update(clear={'LENGTH': 'yes', 'WIDTH': True}),
    lambda b: b.update(clear={'LENGTH': True}), lambda b: b.update(gap_mm={'LENGTH': 'x', 'WIDTH': 30}),
    lambda b: b.update(gap_mm={'LENGTH': float('nan'), 'WIDTH': 30}), lambda b: b.update(gap_mm={'LENGTH': float('inf'), 'WIDTH': 30}),
    lambda b: b.update(gap_mm=None), lambda b: b.update(x_m=float('nan')), lambda b: b.update(y_m=None), lambda b: b.update(top_z_m='1'),
    lambda b: b.update(yaw_deg=float('inf')), lambda b: b.update(x_m=True),
]


@pytest.mark.parametrize('mutate', BAD)
def test_잘못된_값의_블록은_건너뛰고_나머지로_고른다(mutate):
    bad = blk(length=90)
    mutate(bad)
    r = pick([bad, blk(length=30)])
    assert (r['status'], r['index']) == ('FOUND', 1)
    assert pick([bad])['counts']['invalid'] == 1


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


@pytest.mark.parametrize('up, axis', [('FLAT', 'LENGTH'), ('THICKNESS', 'THICKNESS'), (None, 'LENGTH')])
def test_목표_자세나_축이_틀리면_오류(up, axis):
    with pytest.raises(ValueError):
        pick([blk()], up, axis)


def test_E49_틈_경계_예시_IRD():
    """IRD 응답 예: LENGTH 26.0 은 통과, WIDTH 8.5 는 clear=false."""
    b = blk(length=26.0, width=8.5)
    assert pick([b], axis='LENGTH')['status'] == 'FOUND'
    assert pick([b], axis='WIDTH')['status'] == 'NONE'

# -*- coding: utf-8 -*-
"""변환기 ② 시험 (W117, E-69) — 기본 설계 4종(001 벤치 · 002 의자 · 003 책상 세운 다리 · 004 가운데 기둥)의 레시피 두 파일(recipe/2.0 구조 + placements/2.0 조립 방법)을
blocks/2.0 으로 바꿔 본다. 위치 · 방향 계산은 E-52 때와 같고, role · part 는 블록 이름에서, stage · grasp 는 조립 방법에서 나온다."""
import copy
import json
import re
from pathlib import Path

import pytest
import yaml

from d2_task.recipe_to_blocks import RecipeToBlocks, ori_extents
from d2_task.recipe_document import RecipeDocument, recipe_sha256

RECIPES = Path(__file__).parent / 'fixtures'             # 레시피 두 파일 4종(시험용 사본 — 한세교 실제 파일과 같은지는 test_recipe_document 가 본다)
BLOCK_MM = [75.0, 25.0, 15.0]   # robot.yaml block_size_m × 1000 (아래 시험이 같은지 확인한다)
IDS = {'001_CHAIR_BENCH_V000': ('bench', 'chair', 11), '002_CHAIR_BACK_V000': ('chair_back', 'chair', 16),
       '003_DESK_STAND_V000': ('desk_stand', 'desk', 9), '004_DESK_PEDESTAL_V000': ('desk_pedestal', 'desk', 11)}


def load(model_id):
    """레시피 두 파일을 (recipe 구조, placements 조립 방법) 복사본으로 읽는다. 형식 오류는 시험 실패로 알린다."""
    doc = RecipeDocument.load(RECIPES, model_id)
    return copy.deepcopy(doc.recipe), copy.deepcopy(doc.placements)


def convert(model_id):
    design_id, family, _ = IDS[model_id]
    return RecipeToBlocks(design_id, family, BLOCK_MM).convert(*load(model_id))


@pytest.mark.parametrize('model_id', IDS)
def test_4종_블록_수와_머리말(model_id):
    out = convert(model_id)
    design_id, family, n = IDS[model_id]
    assert (out['schema'], out['design_id'], out['family']) == ('blocks/2.0', design_id, family)
    assert [b['order'] for b in out['blocks']] == list(range(1, n + 1))
    assert all(b['ori'] in ori_extents(BLOCK_MM) and 'inferred' not in b for b in out['blocks'])      # inferred 는 스캔 설계만


@pytest.mark.parametrize('model_id', IDS)
def test_role_part_stage_grasp가_블록_이름과_조립_방법에서_나온다(model_id):
    """E-69: role · part 는 블록 이름(`<역할>[_<옵션>]_<부품 3자리>_<블록 2자리>), stage · grasp 는 조립 방법 줄. 번호(부품 번호)는 정수 part 로."""
    recipe, placements = load(model_id)
    steps = {s['sequence']: s for s in placements['steps']}
    for b in convert(model_id)['blocks']:
        st = steps[b['order']]
        m = re.fullmatch(r'([A-Z]+(?:_[A-Z]+)?)_([0-9]{3})_[0-9]{2}', st['block'])
        assert (b['role'], b['part'], b['stage'], b['grasp']) == (m.group(1), int(m.group(2)), st['stage'], st['grasp'])


def test_벤치_역할과_부품_묶음():
    b = {x['order']: x for x in convert('001_CHAIR_BENCH_V000')['blocks']}
    assert (b[1]['role'], b[1]['part'], b[1]['stage'], b[1]['grasp']) == ('LEG', 1, 1, 'FLAT_SHORT')
    assert (b[2]['role'], b[2]['part']) == ('LEG', 2)               # 오른쪽 다리는 다른 부품 묶음
    assert (b[3]['role'], b[3]['part'], b[3]['stage']) == ('LEG', 1, 2)
    assert b[9]['role'] == 'SEAT' and b[11]['role'] == 'SEAT'


def test_벤치_실제_레시피와_같다():
    b = {x['order']: x for x in convert('001_CHAIR_BENCH_V000')['blocks']}
    assert (b[1]['x'], b[1]['y'], b[1]['z'], b[1]['ori']) == (-25, 0, 0, 'y')
    assert (b[2]['x'], b[2]['y'], b[2]['z'], b[2]['ori']) == (25, 0, 0, 'y')
    assert (b[9]['x'], b[9]['y'], b[9]['z'], b[9]['ori']) == (0, -25, 60, 'x')
    assert [b[n]['z'] for n in range(1, 9)] == [0, 0, 15, 15, 30, 30, 45, 45]


def test_책상_세운_다리는_zx_이고_아랫면_높이():
    b = {x['order']: x for x in convert('003_DESK_STAND_V000')['blocks']}
    assert (b[1]['x'], b[1]['y'], b[1]['z'], b[1]['ori']) == (-30, -25, 0, 'zx')   # 중심 37.5 − 길이 75/2
    assert (b[5]['z'], b[5]['ori']) == (75, 'x')
    assert (b[7]['x'], b[7]['z'], b[7]['ori']) == (-25, 90, 'y')


def test_의자_등받이_위층():
    b = {x['order']: x for x in convert('002_CHAIR_BACK_V000')['blocks']}
    assert b[16]['z'] == 135 and b[16]['ori'] == 'x'


def test_steps_순서가_섞여도_sequence_순():
    r, pl = load('001_CHAIR_BENCH_V000')
    pl['steps'].reverse()
    out = RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r, pl)
    assert [b['order'] for b in out['blocks']] == list(range(1, 12))
    assert out == convert('001_CHAIR_BENCH_V000')


def test_원본_레시피를_바꾸지_않는다():
    r, pl = load('001_CHAIR_BENCH_V000')
    before = copy.deepcopy((r, pl))
    RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r, pl)
    assert (r, pl) == before


def test_출력은_JSON으로_직렬화된다():
    json.dumps(convert('003_DESK_STAND_V000'))


def test_design_id_family_는_명시_입력():
    for args in (('', 'chair'), ('bench', ''), (None, None)):
        with pytest.raises(ValueError):
            RecipeToBlocks(*args, BLOCK_MM)


def bad(mutate, reseal=True):
    """recipe(구조) · placements(조립 방법) 복사본을 mutate(recipe, placements) 로 망가뜨리면 변환이 ValueError 여야 한다.

    reseal=True 면 구조를 고친 뒤 짝 해시를 다시 맞춰, 해시 불일치가 아니라 고친 내용 때문에 거절되는지 본다."""
    r, pl = load('001_CHAIR_BENCH_V000')
    mutate(r, pl)
    if reseal:
        pl['recipe_sha256'] = recipe_sha256(r)
    with pytest.raises(ValueError):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r, pl)


@pytest.mark.parametrize('schema', ['cad_structure/1.0', 'cad_recipe/1.0', 'assembly.recipe/1.0', 'recipe/1.0', 'recipe/3.0', None])
def test_옛_형식과_다른_앞자리는_schema를_보고_거절(schema):
    """E-69: 칸 이름이 아니라 schema 로 확인한다. 옛 cad_* · assembly.recipe 는 변환하지 않고 거절, 받은 schema 가 이유에 적힌다."""
    r, pl = load('001_CHAIR_BENCH_V000')
    with pytest.raises(ValueError, match='schema'):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(dict(r, schema=schema), pl)
    with pytest.raises(ValueError, match='schema'):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(r, dict(pl, schema=schema))


def test_옛_두_파일을_서로_바꿔_넣으면_거절():
    r, pl = load('001_CHAIR_BENCH_V000')
    with pytest.raises(ValueError, match='schema'):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert(pl, r)


def test_짝_해시가_안_맞으면_거절():
    bad(lambda r, pl: pl.update(recipe_sha256='0' * 64), reseal=False)
    bad(lambda r, pl: r['blocks'][0].update(center_mm=[-25.0, 0.0, 8.5]), reseal=False)      # 구조만 고쳐 해시가 어긋난 조립 방법


def test_model_id가_다르면_거절():
    bad(lambda r, pl: pl.update(model_id='OTHER'))


def test_잘못된_schema():
    with pytest.raises(ValueError):
        RecipeToBlocks('bench', 'chair', BLOCK_MM).convert({}, {})


def test_없는_block():
    bad(lambda r, pl: pl['steps'][0].update(block='NOPE_001_01'))


def test_없는_part():
    bad(lambda r, pl: r['blocks'][0].update(part_id='PART_999'))


def test_sequence_중복():
    bad(lambda r, pl: pl['steps'][1].update(sequence=1))


def test_steps_없음():
    bad(lambda r, pl: pl.update(steps=[]))


def test_block_id가_모델ID와_블록_이름으로_만든_값과_다르면_거절():
    bad(lambda r, pl: pl['steps'][0].update(block_id='001_CHAIR_BENCH_V000_LEG_009_09'))


def test_45도_회전은_방향_복원_불가():
    s = 0.7071
    bad(lambda r, pl: r['blocks'][0].update(R=[[s, -s, 0], [s, s, 0], [0, 0, 1]]))


def test_크기가_75_25_15가_아니면_방향_복원_불가():
    bad(lambda r, pl: r['parts'][0].update(size_mm=[80.0, 25.0, 15.0]))


def test_거울_반사_회전은_거부():
    bad(lambda r, pl: r['blocks'][0].update(R=[[0, -1, 0], [1, 0, 0], [0, 0, -1]]))


def test_역할이_한_단어_옵션_0_1개가_아니면_거절():
    """작명 규칙 v3: 역할 한 단어 + 옵션 0~1개. 밑줄이 둘 이상인 역할 이름은 role 로 내보낼 수 없다."""
    def rename(r, pl):
        old = 'LEG_001_01'
        new = 'LEG_ARM_WHEEL_001_01'
        r['blocks'][0]['block'] = new
        pl['steps'][0].update(block=new, block_id=f'001_CHAIR_BENCH_V000_{new}')
        for st in pl['steps']:
            st['supports'] = [new if k == old else k for k in st['supports']]
    bad(rename)


def test_좌표는_레시피_원본_값_그대로():
    r, pl = load('003_DESK_STAND_V000')
    out = RecipeToBlocks('desk_stand', 'desk', BLOCK_MM).convert(r, pl)
    center = {b['block']: b['center_mm'] for b in r['blocks']}
    for step, b in zip(sorted(pl['steps'], key=lambda s: s['sequence']), out['blocks']):
        cx, cy, _ = center[step['block']]
        assert (b['x'], b['y']) == (cx, cy)


def test_블록_크기_두_곳은_허용_오차_없이_같다():
    """방향 계산(레시피 parts[].size_mm)과 방향 표(robot.yaml block_size_m × 1000)가 같아야 방향이 정해진다(한세교 W117 확인 요청)."""
    root = Path(__file__).resolve().parents[2]
    cfg = yaml.safe_load(next(root.glob('d2_robot/d2_bringup/config/robot.yaml')).read_text(encoding='utf-8'))
    assert [v * 1000.0 for v in cfg['block_size_m']] == BLOCK_MM
    for model_id in IDS:
        recipe, _ = load(model_id)
        assert all(p['size_mm'] == BLOCK_MM for p in recipe['parts']), model_id

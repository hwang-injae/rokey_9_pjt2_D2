# -*- coding: utf-8 -*-
"""기본 설계 블록 JSON 파일(<모델ID>_blocks.json) 시험 (W117) — 저장소의 파일이 지금 레시피에서 다시 만든 것과 같은지, 만들기 도구 동작."""
import json
import shutil
from pathlib import Path

import pytest
import yaml

from d2_task import base_blocks
from d2_task.design_checker import DesignChecker

RECIPES = Path(__file__).resolve().parents[2] / 'recipe_manager/recipes'
CFG = yaml.safe_load(base_blocks.DEFAULT_CONFIG.read_text(encoding='utf-8'))
IDS = ['001_CHAIR_BENCH', '002_CHAIR_BACK', '003_DESK_STAND', '004_DESK_PEDESTAL']
COUNTS = {'001_CHAIR_BENCH': 11, '002_CHAIR_BACK': 16, '003_DESK_STAND': 9, '004_DESK_PEDESTAL': 11}


def test_기본_설계_네_개를_찾는다():
    assert base_blocks.model_ids(RECIPES) == IDS


@pytest.mark.parametrize('model_id', IDS)
def test_저장소의_blocks_파일은_지금_레시피에서_다시_만든_것과_같다(model_id):
    """레시피가 바뀌었는데 파일을 다시 만들지 않았으면 여기서 실패한다 — `PYTHONPATH=src/d2_task python3 -m d2_task.base_blocks`."""
    saved = (RECIPES / f'{model_id}_blocks.json').read_text(encoding='utf-8')
    assert saved == base_blocks.render(base_blocks.build_blocks(RECIPES, model_id, CFG))


@pytest.mark.parametrize('model_id', IDS)
def test_파일_내용은_blocks_1_이고_검사를_통과한다(model_id):
    blocks = json.loads((RECIPES / f'{model_id}_blocks.json').read_text(encoding='utf-8'))
    assert (blocks['schema'], blocks['design_id']) == ('blocks/1', model_id)
    assert blocks['family'] == ('chair' if 'CHAIR' in model_id else 'desk')
    assert len(blocks['blocks']) == COUNTS[model_id] and [b['order'] for b in blocks['blocks']] == list(range(1, COUNTS[model_id] + 1))
    assert DesignChecker(CFG).check(blocks)['ok']


def test_main은_폴더에_파일을_쓴다(tmp_path):
    for path in RECIPES.glob('*_recipe.json'):
        shutil.copy(path, tmp_path / path.name)
    for path in RECIPES.glob('*_structure.json'):
        shutil.copy(path, tmp_path / path.name)
    assert base_blocks.main(['--recipes', str(tmp_path)]) == 0
    assert sorted(p.name for p in tmp_path.glob('*_blocks.json')) == [f'{m}_blocks.json' for m in IDS]
    for m in IDS:
        assert (tmp_path / f'{m}_blocks.json').read_text(encoding='utf-8') == (RECIPES / f'{m}_blocks.json').read_text(encoding='utf-8')


def test_하나라도_실패하면_아무것도_쓰지_않는다(tmp_path):
    for path in list(RECIPES.glob('*_recipe.json')) + list(RECIPES.glob('*_structure.json')):
        shutil.copy(path, tmp_path / path.name)
    bad = tmp_path / '004_DESK_PEDESTAL_recipe.json'
    data = json.loads(bad.read_text(encoding='utf-8'))
    data['schema'] = 'other/1'
    bad.write_text(json.dumps(data), encoding='utf-8')
    assert base_blocks.main(['--recipes', str(tmp_path)]) == 1
    assert list(tmp_path.glob('*_blocks.json')) == []

# -*- coding: utf-8 -*-
"""기본 설계 4종의 블록 JSON(blocks/1) 파일을 레시피 폴더에 만든다 (ROS 없이 동작, W117).

HMI 가 DB 기본 설계를 등록할 때(W111) 레시피 폴더 하나만 읽도록 `<모델ID>_blocks.json` 을 같은 폴더(구조 · 레시피 · 배치 옆)에 둔다.
변환기 ②(RecipeToBlocks)로 레시피(두 파일 또는 옛 한 파일)를 blocks/1 로 바꾸고 DesignChecker 로 검사해 통과한 것만 쓴다.
레시피가 바뀌면 다시 만든다:  PYTHONPATH=src/d2_task python3 -m d2_task.base_blocks
design_id = 모델 ID(IRD 2장), family = 모델 ID 가운데 이름(CHAIR → chair, DESK → desk). 파일은 mm · 설계 좌표계(blocks/1 그대로).
"""
import argparse
import json
import re
import sys
from pathlib import Path

import yaml

from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_blocks import RecipeToBlocks

SRC = Path(__file__).resolve().parents[2]
DEFAULT_RECIPES = SRC / 'recipe_manager' / 'recipes'
DEFAULT_CONFIG = SRC / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml'
MODEL_ID = re.compile(r'(\d{3})_(CHAIR|DESK)_[A-Z_]+')


def model_ids(folder):
    """레시피 폴더의 기본 설계 모델 ID 들(`<번호>_<CHAIR|DESK>_<이름>_recipe.json` 또는 옛 `.recipe.json`) — 번호순."""
    found = set()
    for path in Path(folder).iterdir():
        m = re.fullmatch(r'(.+?)(?:_recipe|\.recipe)\.json', path.name)
        if m and MODEL_ID.fullmatch(m.group(1)):
            found.add(m.group(1))
    return sorted(found)


def build_blocks(folder, model_id, cfg):
    """모델 하나를 blocks/1 로 바꾸고 DesignChecker 로 검사한다. 통과하면 dict, 아니면 ValueError(이유 포함)."""
    family = MODEL_ID.fullmatch(model_id).group(2).lower()
    document = RecipeDocument.load(folder, model_id)
    block_mm = [v * 1000.0 for v in cfg['block_size_m']]
    blocks = RecipeToBlocks(model_id, family, block_mm).convert(document.recipe, document.structure)
    result = DesignChecker(cfg).check(blocks)
    if not result['ok']:
        raise ValueError(f'{model_id}: 검사 실패 {result["errors"][:3]}')
    return blocks


def render(blocks):
    """파일에 쓸 글자(들여쓰기 1, 끝에 줄바꿈). 같은 입력이면 같은 글자라 시험이 파일과 비교할 수 있다."""
    return json.dumps(blocks, ensure_ascii=False, indent=1) + '\n'


def main(argv=None):
    """레시피 폴더의 기본 설계마다 `<모델ID>_blocks.json` 을 쓴다. 하나라도 변환 · 검사에 실패하면 아무것도 쓰지 않고 1 로 끝낸다."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--recipes', default=str(DEFAULT_RECIPES), help='레시피 폴더')
    ap.add_argument('--config', default=str(DEFAULT_CONFIG), help='robot.yaml')
    args = ap.parse_args(argv)
    cfg = yaml.safe_load(Path(args.config).read_text(encoding='utf-8'))
    ids = model_ids(args.recipes)
    try:
        outputs = {m: render(build_blocks(args.recipes, m, cfg)) for m in ids}
    except ValueError as e:
        print(f'만들지 않았다: {e}', file=sys.stderr)
        return 1
    for model_id, text in outputs.items():
        (Path(args.recipes) / f'{model_id}_blocks.json').write_text(text, encoding='utf-8')
        print(f'{model_id}_blocks.json')
    return 0


if __name__ == '__main__':
    sys.exit(main())

# -*- coding: utf-8 -*-
"""W122 생성 흐름 통합 시험(task 쪽, 로봇 · 웹 없이): 생성된 blocks/1 → 검사(check_design) + 변환기 ① → 레시피 두 파일 → 저장(design/1을 JSON으로 오가며)
→ 설계 선택 → 출발 → 진행표 → 끝까지 가상 로봇 조립.

웹 backend(AI 생성 · DesignStore)와 다리(d2_bridge)는 이 저장소에 아직 없다. 여기서는 그 둘이 하는 일(블록 JSON을 만들어 주고, 합격 응답의 structure · recipe를
design/1로 저장했다가 get_design으로 돌려주기)을 함수 몇 개로 흉내 낸다. 실제 연결 시험은 황인재님 쪽이 붙은 뒤에 한다(W122).
"""
import json
import sys
from pathlib import Path

import pytest

from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_blocks import RecipeToBlocks
from test_task_manager import CFG, SAFE_OK, FakeIO, drive, picks
from d2_task.task_manager import TaskManager

ROOT = Path(__file__).resolve().parents[3]
RECIPES = ROOT / 'src' / 'recipe_manager' / 'recipes'
sys.path.insert(0, str(ROOT / 'src' / 'recipe_manager'))
from recipe_manager.blocks_to_recipe import BlocksToRecipe  # noqa: E402     # numpy 필요(tests/requirements.txt) — 없으면 조용히 건너뛰지 않고 실패한다

BLOCK_MM = [v * 1000.0 for v in CFG['block_size_m']]
BASE = {'001_CHAIR_BENCH': 'chair', '002_CHAIR_BACK': 'chair', '003_DESK_STAND': 'desk', '004_DESK_PEDESTAL': 'desk'}


def generated_design(base_id, design_id):
    """AI 생성이 내는 blocks/1 을 흉내 낸다: 기본 설계를 블록 JSON으로 바꾸고 design_id 만 새 이름(소문자 · 점 포함, IRD 예시 chair_v1.1)으로."""
    doc = RecipeDocument.load(RECIPES, base_id)
    blocks = RecipeToBlocks(base_id, BASE[base_id], BLOCK_MM).convert(doc.recipe, doc.structure)
    return dict(blocks, design_id=design_id)


def check_and_store(blocks):
    """check_design 서비스(JsonQuery 글자)를 부르고, 합격 응답의 structure · recipe 를 design/1 로 JSON 글자로 저장했다가 읽어 온다(저장소 + MQTT 흉내)."""
    checker = DesignChecker(CFG)
    checker.blocks_to_recipe = BlocksToRecipe(checker.grasp_options, BLOCK_MM).convert      # task_node 에는 W110 이 d2_task 로 옮겨진 뒤 붙는다(E-58)
    ok, reason, text = checker.handle_json(json.dumps(blocks))
    assert (ok, reason) == (True, ''), text
    result = json.loads(text)
    assert result['ok'] and result['errors'] == [] and result['min_margin_mm'] is not None
    stored = json.dumps({'schema': 'design/1', 'design_id': blocks['design_id'], 'recipe': result['recipe'], 'structure': result['structure']})
    return json.loads(stored)


class StoreIO(FakeIO):
    """get_design 이 저장소에서 꺼낸 design/1 을 돌려주는 가짜 입출력 — 조회한 design_id 도 기록한다."""

    def __init__(self, design):
        super().__init__()
        self.design = design

    def get_design(self, design_id, should_abort):
        self.events.append(('call', 'load', design_id))
        return (True, '', self.design) if design_id == self.design['design_id'] else (False, '', None)


@pytest.mark.parametrize('base_id', BASE)
def test_생성_설계는_검사_저장_선택_출발_끝까지_조립된다(base_id):
    blocks = generated_design(base_id, 'chair_v1.1')
    design = check_and_store(blocks)
    io = StoreIO(design)
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    assert m.command('select_design', 'chair_v1.1') == (True, '') and m.state == 'READY'
    assert m.command('start') == (True, '') and m.state == 'CHECK'
    drive(m, 'DONE')
    n = len(blocks['blocks'])
    assert m.last_build['placed'] == n
    placed = [b for b, _ in picks(io)]
    assert len(placed) == n and len(set(placed)) == n
    assert all(b.startswith('CHAIR_V1.1_BLOCK_001_') for b in placed)       # E-60: 블록 이름의 설계 ID 부분은 대문자
    assert m.planner.design_id == 'chair_v1.1'                              # check_progress 에 실리는 design_id 는 저장된 이름 그대로
    assert not [s for s in io.states if s['state'] == 'ERROR']


def test_같은_설계를_두_번_저장해_읽어도_진행표가_같다():
    """저장 → 조회를 반복해도(캐시 · 새 판) 블록 이름 · 순서 · 받침이 바뀌지 않는다."""
    design = check_and_store(generated_design('001_CHAIR_BENCH', 'bench_v2'))
    from d2_task.task_planner import TaskPlanner
    a, b = (TaskPlanner(CFG, design['recipe'], design['structure']) for _ in range(2))
    assert [(x['block_id'], x['supports'], x['grasp']) for x in a.blocks] == [(x['block_id'], x['supports'], x['grasp']) for x in b.blocks]


def test_검사_불합격_설계는_저장할_레시피가_없다():
    """뜬 블록(받침 없음)은 check_design 이 CHECK_FAILED 로 막아 레시피가 안 나온다 — 로봇은 움직일 수 없다(SR-09)."""
    blocks = generated_design('001_CHAIR_BENCH', 'bad_v1')
    blocks['blocks'][-1]['z'] += 40.0
    checker = DesignChecker(CFG)
    checker.blocks_to_recipe = BlocksToRecipe(checker.grasp_options, BLOCK_MM).convert
    ok, reason, text = checker.handle_json(json.dumps(blocks))
    result = json.loads(text)
    assert (ok, reason) == (True, '') and not result['ok'] and 'recipe' not in result and result['errors']


def test_변환기가_안_붙은_동안은_합격이어도_ERROR():
    """지금 task_node 의 상태(W110 연결 전): 합격 설계도 레시피가 없으면 완성된 합격 응답이 아니라 ERROR — 웹이 저장하지 못한다."""
    ok, reason, _ = DesignChecker(CFG).handle_json(json.dumps(generated_design('001_CHAIR_BENCH', 'bench_v3')))
    assert (ok, reason) == (False, 'ERROR')

# -*- coding: utf-8 -*-
"""W122 생성 흐름 통합 시험(task 쪽, 로봇 · 웹 없이, E-69): 생성된 blocks/2.0 → 검사(check_design) + 변환기 ①(가짜) → recipe + placements 두 문서 →
저장(design/2.0 를 JSON 으로 오가며) → 설계 선택 → 출발 → 진행표 → 끝까지 가상 로봇 조립.

웹 backend(AI 생성 · DesignStore)와 다리(d2_bridge)는 이 저장소에 아직 없고, 변환기 ① 본체(W110 한세교, 10/10 오전)도 아직 없다.
여기서는 그 둘이 하는 일을 함수 몇 개로 흉내 낸다 — 블록 JSON 은 기본 설계 레시피를 변환기 ②로 바꿔 쓰고, 가짜 변환기 ①은 그 반대로
같은 기하의 recipe + placements 를 새 모델 ID 로 만들어 준다(이름 · 해시만 다시 계산). 변환기 ① 자체의 규칙은 한세교 쪽 시험이 본다.
"""
import copy
import json
from pathlib import Path

import pytest

from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument, recipe_sha256
from d2_task.recipe_to_blocks import RecipeToBlocks
from d2_task.task_manager import TaskManager
from d2_task.task_planner import TaskPlanner
from test_task_manager import CFG, SAFE_OK, FakeIO, drive, picks

FIXTURES = Path(__file__).parent / 'fixtures'
BLOCK_MM = [v * 1000.0 for v in CFG['block_size_m']]
BASE = {'001_CHAIR_BENCH': 'chair', '002_CHAIR_BACK': 'chair', '003_DESK_STAND': 'desk', '004_DESK_PEDESTAL': 'desk'}


def generated_design(base_id, design_id):
    """AI 생성이 내는 blocks/2.0 을 흉내 낸다: 기본 설계를 변환기 ②로 바꾸고 design_id 만 새 이름(소문자 · 점 포함, IRD 예시 chair_v1.1)으로."""
    doc = RecipeDocument.load(FIXTURES, base_id)
    blocks = RecipeToBlocks(base_id, BASE[base_id], BLOCK_MM).convert(doc.recipe, doc.placements)
    return dict(blocks, design_id=design_id)


def fake_converter(base_id):
    """변환기 ①(W110) 자리: blocks/2.0 요청을 받아 같은 기하의 {recipe, placements} 를 요청의 design_id 를 모델 ID 로 해서 돌려준다."""
    doc = RecipeDocument.load(FIXTURES, base_id)

    def convert(request):
        model_id = request['design_id']
        recipe = dict(copy.deepcopy(doc.recipe), model_id=model_id)
        placements = copy.deepcopy(doc.placements)
        placements['model_id'] = model_id
        placements['recipe_sha256'] = recipe_sha256(recipe)
        for st in placements['steps']:
            st['block_id'] = f'{model_id.upper()}_{st["block"]}'
        return {'recipe': recipe, 'placements': placements}
    return convert


def check_and_store(blocks, base_id):
    """check_design 서비스(JsonQuery 글자)를 부르고, 합격 응답의 recipe · placements 를 design/2.0 로 JSON 글자로 저장했다가 읽어 온다(저장소 + MQTT 흉내)."""
    checker = DesignChecker(CFG, blocks_to_recipe=fake_converter(base_id))
    ok, reason, text = checker.handle_json(json.dumps(blocks))
    assert (ok, reason) == (True, ''), text
    result = json.loads(text)
    assert result['ok'] and result['errors'] == [] and result['min_margin_mm'] is not None and result['schema'] == 'check_result/2.0'
    stored = json.dumps({'schema': 'design/2.0', 'design_id': blocks['design_id'], 'recipe': result['recipe'], 'placements': result['placements']})
    return json.loads(stored)


class StoreIO(FakeIO):
    """get_design 이 저장소에서 꺼낸 design/2.0 을 돌려주는 가짜 입출력 — 조회한 design_id 도 기록한다."""

    def __init__(self, design):
        super().__init__()
        self.design = design

    def get_design(self, design_id, should_abort):
        self.events.append(('call', 'load', design_id))
        return (True, '', self.design) if design_id == self.design['design_id'] else (False, '', None)


@pytest.mark.parametrize('base_id', BASE)
def test_생성_설계는_검사_저장_선택_출발_끝까지_조립된다(base_id):
    blocks = generated_design(base_id, 'chair_v1.1')
    design = check_and_store(blocks, base_id)
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
    assert all(b.startswith('CHAIR_V1.1_') for b in placed)                  # E-60: 블록 이름의 설계 ID 부분은 대문자(점 그대로)
    assert m.planner.design_id == 'chair_v1.1'                              # check_progress 에 실리는 design_id 는 저장된 이름 그대로
    assert not [s for s in io.states if s['state'] == 'ERROR']


def test_같은_설계를_두_번_저장해_읽어도_진행표가_같다():
    """저장 → 조회를 반복해도(캐시 · 새 판) 블록 이름 · 순서 · 받침이 바뀌지 않는다."""
    design = check_and_store(generated_design('001_CHAIR_BENCH', 'bench_v2'), '001_CHAIR_BENCH')
    a, b = (TaskPlanner(CFG, design['recipe'], design['placements']) for _ in range(2))
    assert [(x['block_id'], x['supports'], x['grasp']) for x in a.blocks] == [(x['block_id'], x['supports'], x['grasp']) for x in b.blocks]


def test_검사_불합격_설계는_저장할_레시피가_없다():
    """뜬 블록(받침 없음)은 check_design 이 CHECK_FAILED 로 막아 레시피가 안 나온다 — 로봇은 움직일 수 없다(SR-09)."""
    blocks = generated_design('001_CHAIR_BENCH', 'bad_v1')
    blocks['blocks'][-1]['z'] += 40.0
    checker = DesignChecker(CFG, blocks_to_recipe=fake_converter('001_CHAIR_BENCH'))
    ok, reason, text = checker.handle_json(json.dumps(blocks))
    result = json.loads(text)
    assert (ok, reason) == (True, '') and not result['ok'] and 'recipe' not in result and 'placements' not in result and result['errors']


def test_AI가_칸을_빠뜨리면_CHECK_FAILED로_돌려보내고_코드가_채우지_않는다():
    blocks = generated_design('001_CHAIR_BENCH', 'chair_v1.1')
    del blocks['blocks'][3]['role']
    ok, reason, text = DesignChecker(CFG, blocks_to_recipe=fake_converter('001_CHAIR_BENCH')).handle_json(json.dumps(blocks))
    result = json.loads(text)
    assert (ok, reason) == (True, '') and not result['ok'] and result['errors'][0]['reason'] == 'CHECK_FAILED' and 'role' in result['errors'][0]['detail']


def test_변환기가_안_붙은_동안은_합격이어도_ERROR():
    """지금 task_node 의 상태(W110 연결 전): 합격 설계도 레시피가 없으면 완성된 합격 응답이 아니라 ERROR — 웹이 저장하지 못한다."""
    ok, reason, _ = DesignChecker(CFG).handle_json(json.dumps(generated_design('001_CHAIR_BENCH', 'bench_v3')))
    assert (ok, reason) == (False, 'ERROR')

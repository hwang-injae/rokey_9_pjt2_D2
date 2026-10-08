# -*- coding: utf-8 -*-
"""W141 회귀 시험: 두 파일 참조 · 역할 이름 · 파일 전환 · 검사 응답 · 작업 시작. ROS 없이 실제 계산 클래스를 검증한다."""
import ast
import copy
import json
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument
from d2_task.recipe_to_blocks import RecipeToBlocks
from d2_task.run_logger import RunLogger
from d2_task.task_manager import TaskManager
from d2_task.task_planner import TaskPlanner
from test_task_manager import CFG, RECIPE, SAFE_OK, FakeIO, drive

FIXTURES = Path(__file__).parent / 'fixtures'
MODEL = '001_CHAIR_BENCH'


@pytest.fixture
def node():
    """ROS 없는 시험에서 실제 TaskNode 로컬 조회·진행 요청 본문만 실행한다. 통신·메시지 생성은 가짜다."""
    path = Path(__file__).parents[1] / 'd2_task/task_node.py'
    original = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'TaskNode')
    cls = ast.ClassDef(name='TaskNode', bases=[], keywords=[], decorator_list=[],
                       body=[n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in ('_local_design', 'check_progress')])
    scope = dict(RecipeDocument=RecipeDocument, CheckProgress=SimpleNamespace(Request=SimpleNamespace))
    code = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    exec(compile(code, str(path), 'exec'), scope)
    return scope['TaskNode']()


@pytest.fixture
def pair():
    """검증된 두 파일 원본을 매 시험마다 새로 읽는다. 손상 시험이 다른 시험에 영향을 주지 않는다."""
    d = RecipeDocument.load(FIXTURES, MODEL)
    return d.recipe, d.structure


def copy_pair(folder):
    """시험용 폴더에 두 원본 파일을 복사한다. 해시는 재직렬화하지 않아 원본 바이트 기준이다."""
    for suffix in ('_recipe.json', '_structure.json'):
        shutil.copyfile(FIXTURES / f'{MODEL}{suffix}', folder / f'{MODEL}{suffix}')


def test_두파일_블록좌표는_옛파일과_같다(pair):
    """형식 전환으로 실제 블록 방향·아랫면·놓는 순서가 바뀌지 않아야 한다."""
    convert = RecipeToBlocks(MODEL, 'chair', [75, 25, 15])
    assert convert.convert(*pair) == convert.convert(RECIPE)


def test_두파일_판단은_역할이름과_받침을_보존한다(pair):
    """같은 계산으로 실제 높이를 얻고 전체 역할 이름으로 진행표를 만든다."""
    p, old = TaskPlanner(CFG, *pair), TaskPlanner(CFG, RECIPE)
    assert p.blocks[0]['block_id'] == f'{MODEL}_LEG_001_01'
    assert p.blocks[2]['supports'] == [f'{MODEL}_LEG_001_01']
    for a, b in zip(p.blocks, old.blocks):
        assert (a['center'], a['quat'], a['grasp']) == (b['center'], b['quat'], b['grasp'])
    p.update_progress({k: {'state': 'absent'} for k in p.progress})
    assert p.next_block()['block_id'] == f'{MODEL}_LEG_001_01'


def test_순서를_바꿔도_블록이름이_바뀌지_않는다(pair):
    """서로 독립된 첫 두 다리의 순서만 바꾸면 ID·받침은 그대로이고 놓는 순서만 바뀐다."""
    r, s = pair
    r['steps'][0]['sequence'], r['steps'][1]['sequence'] = 2, 1
    p = TaskPlanner(CFG, r, s)
    assert p.blocks[0]['block_id'] == f'{MODEL}_LEG_002_01'
    assert p.blocks[2]['supports'] == [f'{MODEL}_LEG_001_01']


def test_입력문서는_변경하지_않는다(pair):
    """진행표 갱신과 반환 문서 수정이 원본 구조·레시피에 영향을 주지 않는다."""
    original = copy.deepcopy(pair)
    d = RecipeDocument(*pair)
    d.geometry['steps'][0]['supports'].append('other')
    d.design(MODEL)['structure']['blocks'].clear()
    assert pair == original


@pytest.mark.parametrize('mutate', [
    lambda r, s: s.update(schema='other/1'),
    lambda r, s: s.update(model_id='OTHER'),
    lambda r, s: r.update(structure_sha256='bad'),
    lambda r, s: s['blocks'][0].update(block='B001'),
    lambda r, s: s['blocks'][1].update(block=s['blocks'][0]['block']),
    lambda r, s: s['blocks'][0].update(part_id='NOPE'),
    lambda r, s: s['blocks'][0].update(center_mm=[float('inf'), 0, 0]),
    lambda r, s: s['blocks'][0].update(R=[[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
    lambda r, s: r['steps'][0].update(block='NOPE'),
    lambda r, s: r['steps'][1].update(sequence=1),
    lambda r, s: r['steps'][0].update(stage=True),
    lambda r, s: r['steps'][0].update(grasp='NOPE'),
    lambda r, s: r['steps'][0].update(supports=['NOPE']),
    lambda r, s: r['steps'][0].update(supports=[r['steps'][1]['block']]),
    lambda r, s: r['steps'][2].update(supports=[r['steps'][0]['block']] * 2),
    lambda r, s: r['steps'].pop(),
])
def test_두파일_손상은_조립전_거절한다(pair, mutate):
    """오류가 로봇 목표로 넘어가기 전에 파일·판단 공통 검증에서 거절된다."""
    mutate(*pair)
    with pytest.raises(ValueError):
        TaskPlanner(CFG, *pair)


def test_잡기이름과_축이_다르면_거절한다(pair):
    """유효한 이름이어도 방향에서 계산한 잡기와 다르면 시작하지 않는다."""
    pair[0]['steps'][0]['grasp'] = 'FLAT_LONG'
    with pytest.raises(ValueError, match='grasp'):
        TaskPlanner(CFG, *pair)


def test_새파일_우선_옛파일도_읽는다(tmp_path):
    """양쪽 파일이 있으면 새 두 파일을 쓰고, 새 파일이 없는 폴더는 옛 파일을 읽는다."""
    old = tmp_path / f'{MODEL}.recipe.json'
    old.write_text(json.dumps(RECIPE))
    assert RecipeDocument.load(tmp_path, MODEL).structure is None
    copy_pair(tmp_path)
    assert RecipeDocument.load(tmp_path, MODEL).structure['blocks'][0]['block'] == 'LEG_001_01'


def test_이름만_바뀐_한파일도_읽는다(tmp_path):
    """PR #46 단계의 _recipe.json + model/steps 도 지원한다."""
    (tmp_path / f'{MODEL}_recipe.json').write_text(json.dumps(RECIPE))
    assert RecipeDocument.load(tmp_path, MODEL).geometry == RECIPE


@pytest.mark.parametrize('literal', ['NaN', 'Infinity', '1e999'])
def test_깨진_새파일은_옛파일로_대체하지_않는다(tmp_path, literal):
    """새 출력 오류를 숨겨 다른 설계를 조립하지 않고 유한하지 않은 값도 거절한다."""
    (tmp_path / f'{MODEL}.recipe.json').write_text(json.dumps(RECIPE))
    (tmp_path / f'{MODEL}_recipe.json').write_text('{"x":' + literal + '}')
    with pytest.raises(ValueError):
        RecipeDocument.load(tmp_path, MODEL)


def test_구조파일이_없거나_해시가_다르면_거절한다(tmp_path):
    """누락 구조와 변경된 원본 파일을 조립 문서가 지정한 구조로 인정하지 않는다."""
    copy_pair(tmp_path)
    structure = tmp_path / f'{MODEL}_structure.json'
    structure.write_bytes(structure.read_bytes() + b'\n')
    with pytest.raises(ValueError, match='structure_sha256'):
        RecipeDocument.load(tmp_path, MODEL)
    structure.unlink()
    with pytest.raises(OSError):
        RecipeDocument.load(tmp_path, MODEL)


@pytest.mark.parametrize('design_id', ['../x', '/tmp/x', '.hidden', '', 'x\\y'])
def test_경로이름을_설계ID로_쓰지_않는다(tmp_path, design_id):
    """개발용 읽기 경로 밖의 파일을 선택할 수 없다."""
    with pytest.raises(ValueError):
        RecipeDocument.load(tmp_path, design_id)


def test_검사응답은_구조와_레시피_원본을_반환한다(pair):
    """변환기 ① 두 파일 계약을 받으면 HMI에 structure·recipe를 함께 내보낸다."""
    request = RecipeToBlocks(MODEL, 'chair', [75, 25, 15]).convert(*pair)
    checker = DesignChecker(CFG, blocks_to_recipe=lambda _: dict(recipe=pair[0], structure=pair[1]))
    ok, reason, text = checker.handle_json(json.dumps(request))
    result = json.loads(text)
    assert (ok, reason) == (True, '') and result['ok']
    assert (result['recipe'], result['structure']) == pair


@pytest.mark.parametrize('missing', ['structure', 'recipe'])
def test_변환기_두파일_누락은_ERROR(pair, missing):
    """부분 출력은 설계 합격이나 저장 가능한 결과로 전달하지 않는다."""
    output = dict(recipe=pair[0], structure=pair[1])
    del output[missing]
    request = RecipeToBlocks(MODEL, 'chair', [75, 25, 15]).convert(*pair)
    checker = DesignChecker(CFG, blocks_to_recipe=lambda _: output)
    assert checker.handle_json(json.dumps(request))[:2] == (False, 'ERROR')


def test_새_조립문서만_반환하면_ERROR(pair):
    """변환기 ①의 새 조립 문서는 구조 없이 합격 결과로 저장할 수 없다."""
    request = RecipeToBlocks(MODEL, 'chair', [75, 25, 15]).convert(*pair)
    checker = DesignChecker(CFG, blocks_to_recipe=lambda _: pair[0])
    assert checker.handle_json(json.dumps(request))[:2] == (False, 'ERROR')


def test_작업관리자는_원격_두파일로_조립한다(pair):
    """선택·출발 조회에서 받은 두 파일로 같은 run을 DONE까지 진행한다. 가짜 로봇만 사용한다."""
    io = FakeIO()
    io.get_design = lambda design_id, abort: (True, '', RecipeDocument(*pair).design(design_id))
    m = TaskManager(CFG, io, RunLogger(''))
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    assert m.command('select_design', MODEL) == (True, '')
    assert m.command('start') == (True, '')
    drive(m, 'DONE')
    assert m.last_build['placed'] == 11
    assert all(b.startswith(f'{MODEL}_LEG_') or b.startswith(f'{MODEL}_SEAT_') for b in io.world)


def test_노드_로컬조회는_두파일을_반환한다(node, tmp_path):
    """실제 TaskNode 로컬 메서드가 새 파일 이름과 원본 두 문서를 사용한다."""
    copy_pair(tmp_path)
    node.get_parameter = lambda name: SimpleNamespace(value=str(tmp_path))
    ok, reason, design = node._local_design(MODEL)
    assert ok and reason == '' and design['structure']['schema'] == 'cad_structure/1.0'
    (tmp_path / f'{MODEL}_structure.json').write_text('{}')
    assert node._local_design(MODEL) == (False, '', None)


def test_BACK_BEAM도_명시적_설계ID를_보낸다(node):
    """블록 이름에 _B가 있어도 잘라 내지 않고 manager의 설계 ID를 그대로 전달한다."""
    sent = []

    def capture(client, request, *args):
        """실제 메서드가 만든 요청만 기록하고 통신 전에 멈춘다."""
        sent.append(request)
        raise InterruptedError

    node.manager = SimpleNamespace(design_id='002_CHAIR_BACK', run_id='R1')
    node.check_cli, node.service_s, node._call = None, 3, capture
    blocks = ['002_CHAIR_BACK_BACK_001_01', '002_CHAIR_BACK_BEAM_001_01']
    with pytest.raises(InterruptedError):
        node.check_progress(blocks, lambda: False)
    assert sent[0].design_id == '002_CHAIR_BACK' and sent[0].block_ids == blocks

# -*- coding: utf-8 -*-
"""W141 회귀 시험(E-69): 두 파일 참조 · schema 거절 · 짝 해시 · 역할 이름 · 파일 읽기 · 검사 응답 · 작업 시작. ROS 없이 실제 계산 클래스를 검증한다.

레시피 두 파일 = 구조 `<모델ID>_recipe.json`(recipe/2.0) + 조립 방법 `<모델ID>_placements.csv`(placements/2.0). 옛 `_structure.json`(cad_structure/1.0) ·
`cad_recipe/1.0` · `assembly.recipe/1.0` 은 schema 를 보고 거절한다(변환하지 않음).
"""
import ast
import copy
import csv
import io
import json
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from d2_task.design_checker import DesignChecker
from d2_task.recipe_document import RecipeDocument, recipe_sha256
from d2_task.recipe_to_blocks import RecipeToBlocks
from d2_task.run_logger import RunLogger
from d2_task.task_manager import TaskManager
from d2_task.task_planner import TaskPlanner
from test_task_manager import CFG, SAFE_OK, FakeIO, drive

FIXTURES = Path(__file__).parent / 'fixtures'
MODEL = '001_CHAIR_BENCH_V000'


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
    """검증된 두 문서(recipe 구조, placements 조립 방법)를 매 시험마다 새로 읽는다. 손상 시험이 다른 시험에 영향을 주지 않는다."""
    d = RecipeDocument.load(FIXTURES, MODEL)
    return d.recipe, d.placements


def copy_pair(folder):
    """시험용 폴더에 두 원본 파일을 복사한다."""
    for suffix in ('_recipe.json', '_placements.csv'):
        shutil.copyfile(FIXTURES / f'{MODEL}{suffix}', folder / f'{MODEL}{suffix}')


def sealed(recipe, placements):
    """recipe 를 고친 뒤 짝 해시를 다시 맞춘다 — 해시 불일치가 아니라 고친 내용 때문에 거절되는지 보려고."""
    return recipe, dict(placements, recipe_sha256=recipe_sha256(recipe))


def test_두파일_판단은_역할이름과_받침을_보존한다(pair):
    """같은 계산으로 실제 높이를 얻고 전체 역할 이름으로 진행표를 만든다."""
    p = TaskPlanner(CFG, *pair)
    assert p.blocks[0]['block_id'] == f'{MODEL}_LEG_001_01'
    assert p.blocks[2]['supports'] == [f'{MODEL}_LEG_001_01']
    p.update_progress({k: {'state': 'absent'} for k in p.progress})
    assert p.next_block()['block_id'] == f'{MODEL}_LEG_001_01'


def test_시험용_파일은_레시피_도구의_실제_벤치_파일과_같다():
    """fixtures 가 레시피 도구 출력(src/recipe_manager/recipes)과 어긋나면(한쪽만 고침) 여기서 걸린다."""
    real = Path(__file__).resolve().parents[2] / 'recipe_manager/recipes'
    if json.loads((real / f'{MODEL}_recipe.json').read_text(encoding='utf-8')).get('schema') != 'recipe/2.0':
        pytest.skip('레시피 도구의 recipes/ 가 아직 E-69 형식이 아니다(한세교 PR 병합 전) — 병합되면 이 시험이 켜진다')
    for suffix in ('_recipe.json', '_placements.csv'):
        assert (FIXTURES / f'{MODEL}{suffix}').read_bytes() == (real / f'{MODEL}{suffix}').read_bytes()


def test_짝_해시는_객체_기준이라_키_순서나_들여쓰기와_상관없다(pair):
    """recipe_sha256 = 구조 객체를 키 정렬 · 공백 없는 JSON(UTF-8)으로 만든 sha256 (한세교 정함 — 파일 바이트 기준이 아니다)."""
    recipe, placements = pair
    shuffled = json.loads(json.dumps(recipe, indent=4, sort_keys=False))
    assert recipe_sha256(shuffled) == recipe_sha256(dict(reversed(list(recipe.items())))) == placements['recipe_sha256']
    assert len(placements['recipe_sha256']) == 64


def test_순서를_바꿔도_블록이름이_바뀌지_않는다(pair):
    """서로 독립된 첫 두 다리의 순서만 바꾸면 ID·받침은 그대로이고 놓는 순서만 바뀐다."""
    r, p = pair
    p['steps'][0]['sequence'], p['steps'][1]['sequence'] = 2, 1
    planner = TaskPlanner(CFG, r, p)
    assert planner.blocks[0]['block_id'] == f'{MODEL}_LEG_002_01'
    assert planner.blocks[2]['supports'] == [f'{MODEL}_LEG_001_01']


def test_입력문서는_변경하지_않는다(pair):
    """진행표 갱신과 반환 문서 수정이 원본 구조·조립 방법에 영향을 주지 않는다."""
    original = copy.deepcopy(pair)
    d = RecipeDocument(*pair)
    d.geometry['steps'][0]['supports'].append('other')
    d.design(MODEL)['recipe']['blocks'].clear()
    assert pair == original


@pytest.mark.parametrize('mutate, seal', [
    (lambda r, p: r.update(model_id='OTHER'), True),
    (lambda r, p: p.update(model_id='OTHER'), True),
    (lambda r, p: p.update(recipe_sha256='bad'), False),
    (lambda r, p: p.update(recipe_sha256='0' * 64), False),
    (lambda r, p: r['blocks'][0].update(center_mm=[-25.0, 0.0, 9.0]), False),            # 구조만 고쳐 해시가 어긋남
    (lambda r, p: r['blocks'][0].update(block='B001'), True),
    (lambda r, p: r['blocks'][1].update(block=r['blocks'][0]['block']), True),
    (lambda r, p: r['blocks'][0].update(part_id='NOPE'), True),
    (lambda r, p: r['blocks'][0].update(center_mm=[float('inf'), 0, 0]), False),            # 유한하지 않은 수는 해시를 만들 때부터 거절
    (lambda r, p: r['blocks'][0].update(R=[[1, 0, 0], [0, 1, 0], [0, 0, -1]]), True),
    (lambda r, p: p['steps'][0].update(block='NOPE'), True),
    (lambda r, p: p['steps'][0].update(block_id='001_CHAIR_BENCH_V000_LEG_009_09'), True),
    (lambda r, p: p['steps'][1].update(sequence=1), True),
    (lambda r, p: p['steps'][0].update(stage=True), True),
    (lambda r, p: p['steps'][0].update(grasp='NOPE'), True),
    (lambda r, p: p['steps'][0].update(supports=['NOPE']), True),
    (lambda r, p: p['steps'][0].update(supports=[p['steps'][1]['block']]), True),
    (lambda r, p: p['steps'][2].update(supports=[p['steps'][0]['block']] * 2), True),
    (lambda r, p: p['steps'].pop(), True),
])
def test_두파일_손상은_조립전_거절한다(pair, mutate, seal):
    """오류가 로봇 목표로 넘어가기 전에 파일·판단 공통 검증에서 거절된다. seal=True 는 해시를 맞춰 고친 내용 때문에 거절되는지 본다."""
    r, p = pair
    mutate(r, p)
    if seal:
        r, p = sealed(r, p)
    with pytest.raises(ValueError):
        TaskPlanner(CFG, r, p)


@pytest.mark.parametrize('schema', ['cad_structure/1.0', 'cad_recipe/1.0', 'assembly.recipe/1.0', 'recipe/1.0', 'recipe/3.0', 'placements/2.0', None])
def test_구조_schema가_recipe_2_0이_아니면_받은_schema를_적어_거절(pair, schema):
    r, p = pair
    if schema == 'placements/2.0':           # 두 문서를 바꿔 넣은 경우
        r, p = p, r
    else:
        r = dict(r, schema=schema)
    with pytest.raises(ValueError, match='schema'):
        RecipeDocument(r, p)


@pytest.mark.parametrize('schema', ['cad_structure/1.0', 'cad_recipe/1.0', 'placements/1.0', 'placements/3.0', 'recipe/2.0', None])
def test_조립_방법_schema가_placements_2_0이_아니면_거절(pair, schema):
    r, p = pair
    with pytest.raises(ValueError, match='schema'):
        RecipeDocument(r, dict(p, schema=schema))


def test_잡기이름과_축이_다르면_거절한다(pair):
    """유효한 이름이어도 방향에서 계산한 잡기와 다르면 시작하지 않는다."""
    r, p = pair
    p['steps'][0]['grasp'] = 'FLAT_LONG'
    with pytest.raises(ValueError, match='grasp'):
        TaskPlanner(CFG, r, p)


def test_옛_파일은_읽지_않는다(tmp_path):
    """E-69: 옛 `_structure.json` · cad_* · assembly.recipe 는 읽지 않는다. 새 두 파일이 모두 있어야 읽힌다."""
    old_structure = json.loads((FIXTURES / f'{MODEL}_recipe.json').read_text(encoding='utf-8'))
    old_structure['schema'] = 'cad_structure/1.0'
    (tmp_path / f'{MODEL}_structure.json').write_text(json.dumps(old_structure))
    with pytest.raises(OSError):
        RecipeDocument.load(tmp_path, MODEL)                              # 새 파일이 없으면 옛 파일로 대신하지 않는다
    (tmp_path / f'{MODEL}_recipe.json').write_text(json.dumps(old_structure))
    shutil.copyfile(FIXTURES / f'{MODEL}_placements.csv', tmp_path / f'{MODEL}_placements.csv')
    with pytest.raises(ValueError, match='cad_structure/1.0'):
        RecipeDocument.load(tmp_path, MODEL)
    copy_pair(tmp_path)
    assert RecipeDocument.load(tmp_path, MODEL).recipe['blocks'][0]['block'] == 'LEG_001_01'


def test_조립_방법_없이는_문서를_만들_수_없다(pair):
    with pytest.raises(TypeError):
        RecipeDocument(pair[0])
    with pytest.raises(ValueError):
        RecipeDocument(pair[0], None)
    with pytest.raises(TypeError):
        TaskPlanner(CFG, pair[0])


@pytest.mark.parametrize('literal', ['NaN', 'Infinity', '1e999'])
def test_깨진_파일은_유한하지_않은_값을_거절한다(tmp_path, literal):
    copy_pair(tmp_path)
    (tmp_path / f'{MODEL}_recipe.json').write_text('{"x":' + literal + '}')
    with pytest.raises(ValueError):
        RecipeDocument.load(tmp_path, MODEL)


def test_구조_파일이_바뀌어_짝_해시가_다르면_거절한다(tmp_path):
    """조립 방법이 쓰인 뒤 구조가 바뀌면(재생성 · 수작업) 읽기를 거부한다."""
    copy_pair(tmp_path)
    recipe = json.loads((tmp_path / f'{MODEL}_recipe.json').read_text(encoding='utf-8'))
    recipe['blocks'][0]['center_mm'][2] += 1.0
    (tmp_path / f'{MODEL}_recipe.json').write_text(json.dumps(recipe))
    with pytest.raises(ValueError, match='recipe_sha256'):
        RecipeDocument.load(tmp_path, MODEL)
    (tmp_path / f'{MODEL}_recipe.json').unlink()
    with pytest.raises(OSError):
        RecipeDocument.load(tmp_path, MODEL)


def test_CSV_머리줄로_읽고_칸_순서에_의존하지_않는다(tmp_path):
    copy_pair(tmp_path)
    rows = list(csv.DictReader(io.StringIO((tmp_path / f'{MODEL}_placements.csv').read_text(encoding='utf-8'))))
    names = list(reversed(list(rows[0])))                                  # 칸 순서를 뒤집어 다시 쓴다
    out = io.StringIO()
    w = csv.DictWriter(out, fieldnames=names, lineterminator='\n')
    w.writeheader()
    w.writerows(rows)
    (tmp_path / f'{MODEL}_placements.csv').write_text(out.getvalue(), encoding='utf-8')
    assert RecipeDocument.load(tmp_path, MODEL).placements == RecipeDocument.load(FIXTURES, MODEL).placements


@pytest.mark.parametrize('drop', ['block', 'block_id', 'sequence', 'stage', 'grasp', 'grasp_axis', 'supports', 'recipe_sha256', 'schema'])
def test_CSV_필수_칸이_빠지면_거절(tmp_path, drop):
    copy_pair(tmp_path)
    rows = list(csv.DictReader(io.StringIO((tmp_path / f'{MODEL}_placements.csv').read_text(encoding='utf-8'))))
    names = [k for k in rows[0] if k != drop]
    out = io.StringIO()
    w = csv.DictWriter(out, fieldnames=names, extrasaction='ignore', lineterminator='\n')
    w.writeheader()
    w.writerows(rows)
    (tmp_path / f'{MODEL}_placements.csv').write_text(out.getvalue(), encoding='utf-8')
    with pytest.raises(ValueError, match=drop):
        RecipeDocument.load(tmp_path, MODEL)


def test_CSV_줄마다_짝_해시가_다르면_거절(tmp_path):
    copy_pair(tmp_path)
    lines = (tmp_path / f'{MODEL}_placements.csv').read_text(encoding='utf-8').splitlines()
    lines[-1] = lines[-1][:-64] + '0' * 64
    (tmp_path / f'{MODEL}_placements.csv').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    with pytest.raises(ValueError, match='recipe_sha256'):
        RecipeDocument.load(tmp_path, MODEL)


@pytest.mark.parametrize('design_id', ['../x', '/tmp/x', '.hidden', '', 'x\\y'])
def test_경로이름을_설계ID로_쓰지_않는다(tmp_path, design_id):
    """개발용 읽기 경로 밖의 파일을 선택할 수 없다."""
    with pytest.raises(ValueError):
        RecipeDocument.load(tmp_path, design_id)


def test_검사응답은_recipe와_placements_원본을_반환한다(pair):
    """변환기 ① 두 문서 계약을 받으면 HMI 에 recipe · placements 를 함께 내보낸다(check_result/2.0)."""
    request = RecipeToBlocks(MODEL, 'chair', [75, 25, 15]).convert(*pair)
    checker = DesignChecker(CFG, blocks_to_recipe=lambda _: dict(recipe=pair[0], placements=pair[1]))
    ok, reason, text = checker.handle_json(json.dumps(request))
    result = json.loads(text)
    assert (ok, reason) == (True, '') and result['ok'] and result['schema'] == 'check_result/2.0'
    assert (result['recipe'], result['placements']) == pair


@pytest.mark.parametrize('missing', ['recipe', 'placements'])
def test_변환기_두문서_누락은_ERROR(pair, missing):
    """부분 출력은 설계 합격이나 저장 가능한 결과로 전달하지 않는다."""
    output = dict(recipe=pair[0], placements=pair[1])
    del output[missing]
    request = RecipeToBlocks(MODEL, 'chair', [75, 25, 15]).convert(*pair)
    checker = DesignChecker(CFG, blocks_to_recipe=lambda _: output)
    assert checker.handle_json(json.dumps(request))[:2] == (False, 'ERROR')


def test_옛_이름_structure로_돌려주면_ERROR(pair):
    """E-52 의 {structure, recipe} 모양은 더 받지 않는다."""
    request = RecipeToBlocks(MODEL, 'chair', [75, 25, 15]).convert(*pair)
    checker = DesignChecker(CFG, blocks_to_recipe=lambda _: dict(recipe=pair[0], structure=pair[1]))
    assert checker.handle_json(json.dumps(request))[:2] == (False, 'ERROR')


def test_작업관리자는_원격_두문서로_조립한다(pair):
    """선택·출발 조회에서 받은 design/2.0 의 두 문서로 같은 run을 DONE까지 진행한다. 가짜 로봇만 사용한다."""
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


def test_노드_로컬조회는_두문서를_반환한다(node, tmp_path):
    """실제 TaskNode 로컬 메서드가 새 파일 이름으로 design/2.0 로컬 모양(design_id · recipe · placements)을 만든다."""
    copy_pair(tmp_path)
    node.get_parameter = lambda name: SimpleNamespace(value=str(tmp_path))
    ok, reason, design = node._local_design(MODEL)
    assert ok and reason == '' and design['schema'] == 'design/2.0' and design['design_id'] == MODEL
    assert design['recipe']['schema'] == 'recipe/2.0' and design['placements']['schema'] == 'placements/2.0'
    (tmp_path / f'{MODEL}_recipe.json').write_text('{}')
    assert node._local_design(MODEL) == (False, '', None)


def test_BACK_BEAM도_명시적_설계ID를_보낸다(node):
    """블록 이름에 _B가 있어도 잘라 내지 않고 manager의 설계 ID를 그대로 전달한다."""
    sent = []

    def capture(client, request, *args):
        """실제 메서드가 만든 요청만 기록하고 통신 전에 멈춘다."""
        sent.append(request)
        raise InterruptedError

    node.manager = SimpleNamespace(design_id='002_CHAIR_BACK_V000', run_id='R1')
    node.check_cli, node.service_s, node._call = None, 3, capture
    blocks = ['002_CHAIR_BACK_V000_BACK_001_01', '002_CHAIR_BACK_V000_BEAM_001_01']
    with pytest.raises(InterruptedError):
        node.check_progress(blocks, lambda: False)
    assert sent[0].design_id == '002_CHAIR_BACK_V000' and sent[0].block_ids == blocks


@pytest.mark.parametrize('name', ['recipe_document', 'recipe_to_blocks'])
def test_웹이_import하는_두_파일은_ROS를_가져오지_않는다(name):
    """E-59 · E-62: 웹 backend(ROS 없음)가 d2_task.recipe_document · recipe_to_blocks 를 import 한다 → rclpy · ROS 메시지 import 가 들어오면 안 된다."""
    tree = ast.parse((Path(__file__).parents[1] / 'd2_task' / f'{name}.py').read_text(encoding='utf-8'))
    mods = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import): mods.update(a.name.split('.')[0] for a in n.names)
        elif isinstance(n, ast.ImportFrom) and n.module: mods.add(n.module.split('.')[0])
    ros = {'rclpy', 'std_msgs', 'geometry_msgs', 'd2_interfaces', 'd2_motion', 'ament_index_python', 'rosidl_runtime_py'}
    assert not (mods & ros), mods & ros


def test_CSV_schema_칸이_옛_형식이면_받은_schema를_적어_거절(tmp_path):
    """한세교 E-69 PR: CSV 에 schema 칸(모든 줄 placements/2.0)을 더했다 — 읽는 쪽이 앞자리 확인으로 거절할 수 있게."""
    copy_pair(tmp_path)
    text = (tmp_path / f'{MODEL}_placements.csv').read_text(encoding='utf-8')
    (tmp_path / f'{MODEL}_placements.csv').write_text(text.replace('placements/2.0', 'cad_recipe/1.0'), encoding='utf-8')
    with pytest.raises(ValueError, match='cad_recipe/1.0'):
        RecipeDocument.load(tmp_path, MODEL)


def test_CSV_schema_칸이_줄마다_다르면_거절(tmp_path):
    copy_pair(tmp_path)
    lines = (tmp_path / f'{MODEL}_placements.csv').read_text(encoding='utf-8').splitlines()
    lines[-1] = lines[-1].replace('placements/2.0', 'placements/3.0')
    (tmp_path / f'{MODEL}_placements.csv').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    with pytest.raises(ValueError, match='schema'):
        RecipeDocument.load(tmp_path, MODEL)


@pytest.mark.parametrize('model_id', ['001_CHAIR_BENCH_V000', '002_CHAIR_BACK_V000', '003_DESK_STAND_V000', '004_DESK_PEDESTAL_V000'])
def test_해시_계산이_레시피_도구와_글자까지_같다(model_id):
    """생성 쪽(RecipeBuilder.calculate_recipe_sha256, 한세교)과 읽는 쪽(recipe_sha256)이 같은 값을 내야 모든 파일이 해시 때문에 거절되지 않는다."""
    from d2_task.recipe_builder import RecipeBuilder
    doc = RecipeDocument.load(FIXTURES, model_id)
    assert recipe_sha256(doc.recipe) == RecipeBuilder().calculate_recipe_sha256(doc.recipe) == doc.placements['recipe_sha256']
    if model_id == '001_CHAIR_BENCH_V000':
        assert doc.placements['recipe_sha256'].startswith('9489c62c') and doc.placements['recipe_sha256'].endswith('3e8a')     # 한세교 시험에 고정된 벤치 값

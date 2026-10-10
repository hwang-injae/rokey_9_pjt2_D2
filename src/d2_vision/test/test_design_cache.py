# -*- coding: utf-8 -*-
"""W157 설계 읽기 캐시 시험 (pytest, ROS · 로봇 없음) — CheckProgress.run_id 가 바뀌면 설계를 다시 읽는다 (E-55 ① · E-60 ②).

세 겹으로 본다.
  ① RunDesignCache 규칙 자체(같은 판 · 새 판 · 빈 run_id · 다시 읽다 실패).
  ② wrist_block · mock_wrist_block **실제 메서드**(checker_for · tops_for · _remote_* · _local_* · on_check)를 소스에서 꺼내(ast)
     가짜 get_design 과 붙여 같은 규칙으로 도는지 — 두 노드가 같은 시험을 통과한다. rclpy 를 import 하지 않는다(CI 는 d2_vision 만 본다).
  ③ mock 이 get_design 의 AI 생성 · 스캔 설계(design/2.0)를 읽어 블록 이름 · 윗면 높이를 내는지 — d2_motion · d2_task 가 없으면(CI) 건너뛴다.
"""
import ast
import json
import logging
import math
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from d2_vision.block_checker import recipe_from_design, recipe_path, recipe_sha256
from d2_vision.design_cache import RunDesignCache
from test_block_checker import PLACEMENTS, RECIPE, ROOT, design

PKG = Path(__file__).resolve().parents[1] / 'd2_vision'
NAN = float('nan')


# ---------------- ① RunDesignCache ----------------
def loader(results):
    """호출할 때마다 results 의 다음 (값, 코드) 를 낸다(끝나면 마지막 것 반복). load.calls 가 부른 횟수."""
    calls = []

    def load():
        calls.append(1)
        return results[min(len(calls), len(results)) - 1]
    load.calls = calls
    return load


def test_같은_판은_한_번만_읽는다():
    cache, load = RunDesignCache(), loader([('v1', ''), ('v2', '')])
    assert cache.get('D1', 'R1', load) == ('v1', '')
    assert cache.get('D1', 'R1', load) == ('v1', '')
    assert len(load.calls) == 1


def test_판이_바뀌면_다시_읽고_그_판에서는_다시_안_읽는다():
    cache, load = RunDesignCache(), loader([('v1', ''), ('v2', ''), ('v3', '')])
    assert cache.get('D1', 'R1', load)[0] == 'v1'
    assert cache.get('D1', 'R2', load)[0] == 'v2'
    assert cache.get('D1', 'R2', load)[0] == 'v2'
    assert cache.get('D1', 'R1', load)[0] == 'v3'                  # 지난 판으로 돌아가도 새 판이다 — 기억은 가장 최근 판 하나
    assert len(load.calls) == 3


def test_열쇠마다_따로_기억한다():
    cache = RunDesignCache()
    assert cache.get('D1', 'R1', loader([('a', '')]))[0] == 'a'
    assert cache.get('D2', 'R1', loader([('b', '')]))[0] == 'b'
    assert cache.get('D1', 'R1', loader([('x', '')]))[0] == 'a'    # D2 를 읽었다고 D1 이 지워지지 않는다


def test_빈_run_id는_판_구분_없음이라_빈_값끼리는_캐시를_쓴다():
    """CheckProgress.srv: 빈 값이면 판 구분 없음. None 도 같다. 이름 있는 판과 오가면 다시 읽는다."""
    cache, load = RunDesignCache(), loader([('v1', ''), ('v2', ''), ('v3', '')])
    assert cache.get('D1', '', load)[0] == 'v1'
    assert cache.get('D1', None, load)[0] == 'v1'
    assert cache.get('D1', '', load)[0] == 'v1'
    assert cache.get('D1', 'R1', load)[0] == 'v2'
    assert cache.get('D1', '', load)[0] == 'v3'
    assert len(load.calls) == 3


def test_다시_읽다_실패하면_지난_판의_설계로_답하지_않는다():
    cache, load = RunDesignCache(), loader([('v1', ''), (None, 'TIMEOUT'), ('v3', '')])
    assert cache.get('D1', 'R1', load) == ('v1', '')
    assert cache.get('D1', 'R2', load) == (None, 'TIMEOUT')        # v1 을 대신 주지 않는다
    assert cache.get('D1', 'R1', load) == ('v3', '')               # 실패하면서 옛 값도 버려졌으니 R1 로 돌아가도 새로 읽는다


def test_실패는_캐시하지_않아_같은_판에서도_다시_시도한다():
    cache, load = RunDesignCache(), loader([(None, 'ERROR'), ('v2', '')])
    assert cache.get('D1', 'R1', load) == (None, 'ERROR')
    assert cache.get('D1', 'R1', load) == ('v2', '')
    assert len(load.calls) == 2


def test_실패_코드가_비면_ERROR():
    assert RunDesignCache().get('D1', 'R1', loader([(None, '')])) == (None, 'ERROR')


def test_빈_설계도_읽은_값이다():
    """블록이 하나도 없는 설계({})도 성공으로 읽은 것 — None(실패)과 구별한다."""
    cache, load = RunDesignCache(), loader([({}, ''), ({'x': 1}, '')])
    assert cache.get('D1', 'R1', load) == ({}, '')
    assert cache.get('D1', 'R1', load) == ({}, '')
    assert len(load.calls) == 1


# ---------------- ② 두 노드의 실제 메서드 ----------------
def load_methods(path, class_name, names, scope):
    """path 의 class_name 에서 names 메서드만 꺼내 scope(전역 이름 모음) 안에서 실행해 만든 빈 클래스를 돌려준다(ROS 설치 없이)."""
    tree = ast.parse(Path(path).read_text(encoding='utf-8'))
    original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    cls = ast.ClassDef(name=class_name, bases=[], keywords=[],
                       body=[n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in names], decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), scope)
    return scope[class_name]


def load_function(path, name, scope):
    """path 의 모듈 함수 name 하나만 꺼내 scope 안에서 실행해 돌려준다."""
    tree = ast.parse(Path(path).read_text(encoding='utf-8'))
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])), str(path), 'exec'), scope)
    return scope[name]


def versioned_design(design_id, n):
    """읽을 때마다 내용이 다른 design/2.0 — 구조의 parts 에 n 을 적는다(짝 해시도 맞춘다). 어느 판에서 읽은 것인지 알아보려고."""
    recipe = dict(RECIPE, model_id=design_id, parts=[{'tag': n}])
    return design(recipe, dict(PLACEMENTS, model_id=design_id, recipe_sha256=recipe_sha256(recipe)), design_id=design_id)


class FakeDesigns:
    """/d2/hmi/get_design 가짜. mode: ok · timeout(답 없음) · fail(success=false) · bad(design/2.0 아님) · down(서버 없음).
    ok 일 때 registry 에 있는 설계를 그대로, 없으면 읽을 때마다 번호가 오르는 versioned_design 을 답한다. requests 는 받은 design_id 목록."""

    def __init__(self, registry=None):
        self.mode, self.registry, self.requests, self.version = 'ok', registry or {}, [], 0

    def service_is_ready(self):
        return self.mode != 'down'

    def answer(self, req):
        """_call 이 부르는 답: None = 시간 초과, 그 밖은 JsonQuery 응답 모양(success · reason · response_json)."""
        design_id = json.loads(req.request_json)['design_id']
        self.requests.append(design_id)
        if self.mode == 'timeout':
            return None
        if self.mode == 'fail':
            return SimpleNamespace(success=False, reason='NOT_FOUND', response_json='')
        if self.mode == 'bad':
            return SimpleNamespace(success=True, reason='', response_json=json.dumps({'schema': 'design/1', 'design_id': design_id}))
        if design_id in self.registry:
            doc = self.registry[design_id]
        else:
            self.version += 1
            doc = versioned_design(design_id, self.version)
        return SimpleNamespace(success=True, reason='', response_json=json.dumps(doc))


class Harness:
    """wrist_block 또는 mock_wrist_block 의 실제 설계 읽기 메서드 + 가짜 get_design. ask() 한 번 = check_progress 요청 한 번.

    ask 결과: ok(설계를 읽어 판정 단계까지 갔나) · reason(실패 코드) · tag(읽은 설계의 번호 — versioned_design 의 parts) ·
    states(mock 만 — 블록마다 state)."""

    def __init__(self, kind, tmp_path, source='remote', recipe_dir='', registry=None, real_tops=None):
        self.kind, self.client = kind, FakeDesigns(registry)
        self.reads = []                                      # local 길로 읽은 파일 경로
        self.reached = []                                    # wrist: 설계를 읽은 뒤 TF 단계까지 간 요청 수
        params = {'design_source': source, 'recipe_dir': str(recipe_dir or tmp_path), 'fail': False}

        def load_recipe(p):
            self.reads.append(p)
            return {'structure': {'parts': [{'tag': len(self.reads)}]}}
        scope = dict(json=json, threading=threading, NAN=NAN, JsonQuery=SimpleNamespace(Request=SimpleNamespace),
                     recipe_from_design=recipe_from_design, recipe_path=recipe_path, RECIPE_SUFFIXES=('_recipe.json',),
                     load_recipe=load_recipe)
        if kind == 'wrist':
            cls = load_methods(PKG / 'wrist_block.py', 'WristBlock', {'checker_for', '_local_checker', '_remote_checker', 'on_check'}, scope)
        else:
            scope['recipe_tops'] = real_tops or (lambda cfg, recipe: {'A': recipe['structure']['parts'][0]['tag'],
                                                                    'B': recipe['structure']['parts'][0]['tag']})
            cls = load_methods(PKG / 'mock_wrist_block.py', 'MockWristBlock', {'tops_for', '_local_tops', '_remote_tops', 'on_check'}, scope)
        n = self.node = cls()
        n.designs, n.cli_design, n.service_s, n.cfg = RunDesignCache(), self.client, 0.05, {}
        n.get_parameter = lambda name: SimpleNamespace(value=params[name])
        n.get_logger = lambda: logging.getLogger('test_design_cache')
        n._call = lambda cli, req, wait_s: cli.answer(req)
        if kind == 'wrist':
            n._make_checker = lambda recipe: SimpleNamespace(tag=recipe['structure']['parts'][0]['tag'], blocks={'A': 1, 'B': 2})
            n.T_tcp2c = object()
            n.read_tcp = lambda: self.reached.append(1)      # 설계를 읽은 뒤 첫 단계 — None 을 돌려 TF 없음(TIMEOUT)으로 끝낸다
            n.now = lambda: 0.0
            n._fill = lambda res, ids, reason='': SimpleNamespace(reason=reason, ids=ids)
            self.last = None
            real = n.checker_for

            def spy(design_id, run_id):                      # 어떤 설계를 얻었는지 본다
                self.last = real(design_id, run_id)
                return self.last
            n.checker_for = spy
        else:
            n._ids = lambda name: set()

    def ask(self, design_id='D1', run_id='R1', ids=('A',)):
        """check_progress 요청 하나를 on_check 로 보낸다. 결과는 SimpleNamespace(ok, reason, tag, states)."""
        req = SimpleNamespace(design_id=design_id, run_id=run_id, block_ids=list(ids))
        if self.kind == 'wrist':
            before = len(self.reached)
            out = self.node.on_check(req, SimpleNamespace())
            checker, reason = self.last if design_id else (None, 'ERROR')
            ok = len(self.reached) > before
            return SimpleNamespace(ok=ok, reason=reason if not ok else '', tag=checker.tag if ok else None,
                                   states=None, filled=out.reason)
        res = SimpleNamespace()
        self.node.on_check(req, res)
        tag = res.top_z_m[0] if res.success and res.states[0] == 'present' else None
        return SimpleNamespace(ok=res.success, reason=res.reason, tag=tag, states=res.states, filled=res.reason)


KINDS = ['wrist', 'mock']


@pytest.fixture(params=KINDS)
def h(request, tmp_path):
    """같은 규칙 시험을 wrist_block · mock_wrist_block 에 한 번씩 돌린다."""
    return Harness(request.param, tmp_path)


def test_같은_run은_반복해도_블록_전부를_물어도_캐시를_유지한다(h):
    """옛 규칙('블록 전부를 묻는 요청 = 새 판')을 뺐다 — 같은 run_id 면 블록을 전부 물어도 get_design 을 다시 부르지 않는다."""
    assert h.ask(run_id='R1', ids=['A']).tag == 1
    assert h.ask(run_id='R1', ids=['A']).tag == 1
    assert h.ask(run_id='R1', ids=['A', 'B']).tag == 1       # 설계 블록 전부 = 시작 확인 모양
    assert h.ask(run_id='R1', ids=['B']).tag == 1
    assert h.client.requests == ['D1']


def test_새_run은_설계를_다시_읽는다(h):
    assert h.ask(run_id='R1').tag == 1
    assert h.ask(run_id='R2').tag == 2
    assert h.ask(run_id='R2', ids=['A', 'B']).tag == 2
    assert h.client.requests == ['D1', 'D1']


def test_설계마다_따로_읽는다(h):
    assert h.ask('D1', 'R1').tag == 1
    assert h.ask('D2', 'R1').tag == 2
    assert h.ask('D1', 'R1').tag == 1                        # D2 를 읽었다고 D1 을 다시 읽지 않는다
    assert h.client.requests == ['D1', 'D2']


def test_빈_run_id는_판_구분_없음(h):
    """srv 규칙: 빈 값 = 판 구분 없음. 빈 값끼리는 캐시, 이름 있는 판과 오가면 다시 읽는다."""
    assert [h.ask(run_id='').tag, h.ask(run_id='').tag] == [1, 1]
    assert h.ask(run_id='R1').tag == 2
    assert h.ask(run_id='').tag == 3
    assert h.client.requests == ['D1', 'D1', 'D1']


@pytest.mark.parametrize('mode, code', [('timeout', 'TIMEOUT'), ('fail', 'ERROR'), ('bad', 'ERROR'), ('down', 'ERROR')])
def test_다시_읽기가_실패하면_이전_설계로_답하지_않는다(h, mode, code):
    """새 판의 get_design 이 실패하면 지난 판의 설계를 쓰지 않고 실패로 답한다(wrist: ERROR · TIMEOUT, mock: 블록 모두 unknown).
    실패는 캐시하지 않아 같은 판의 다음 요청에 다시 시도하고, 성공하면 새 설계로 돌아온다."""
    assert h.ask(run_id='R1').tag == 1
    h.client.mode = mode
    out = h.ask(run_id='R2')
    assert not out.ok and out.reason == code and out.tag is None
    if h.kind == 'mock':
        assert out.states == ['unknown']
    assert not h.ask(run_id='R2').ok                         # 같은 판이어도 실패는 기억하지 않고 다시 시도
    h.client.mode = 'ok'
    back = h.ask(run_id='R2')
    assert back.ok and back.tag == 2                         # 이 읽기까지 get_design 성공은 R1 · R2 두 번뿐(실패 응답은 번호를 올리지 않는다)
    assert h.ask(run_id='R2').tag == 2                       # 그 뒤로는 캐시


def test_이전_판으로_돌아가도_실패한_사이_버려진_설계를_쓰지_않는다(h):
    assert h.ask(run_id='R1').tag == 1
    h.client.mode = 'timeout'
    assert not h.ask(run_id='R2').ok
    h.client.mode = 'ok'
    assert h.ask(run_id='R1').tag == 2                       # 옛 R1 설계(1)가 아니라 새로 읽은 것


def test_처음_읽기가_실패해도_다음_요청에_다시_받는다(h):
    h.client.mode = 'down'
    assert h.ask().reason == 'ERROR'
    h.client.mode = 'ok'
    assert h.ask().ok


def test_design_id가_비면_읽지_않고_ERROR(h):
    out = h.ask(design_id='')
    assert not out.ok and out.reason == 'ERROR' and h.client.requests == []


def test_local은_get_design을_안_부르고_판마다_파일을_읽는다(tmp_path):
    for kind in KINDS:
        (tmp_path / 'D1_recipe.json').write_text('{}', encoding='utf-8')
        hh = Harness(kind, tmp_path, source='local')
        assert [hh.ask(run_id='R1').tag, hh.ask(run_id='R1', ids=['A', 'B']).tag] == [1, 1]
        assert hh.ask(run_id='R2').tag == 2
        assert hh.client.requests == [] and len(hh.reads) == 2


def test_local에서_파일이_없으면_get_design으로_대신하지_않는다(tmp_path):
    for kind in KINDS:
        hh = Harness(kind, tmp_path, source='local')
        out = hh.ask('NOFILE')
        assert not out.ok and out.reason == 'ERROR' and hh.client.requests == []


def test_remote에서_실패해도_로컬_파일로_몰래_대신하지_않는다(tmp_path):
    (tmp_path / 'D1_recipe.json').write_text('{}', encoding='utf-8')
    for kind in KINDS:
        hh = Harness(kind, tmp_path, source='remote')
        hh.client.mode = 'down'
        out = hh.ask()
        assert not out.ok and hh.reads == []                 # recipe_dir 에 파일이 있어도 읽지 않는다


# ---------------- ③ mock 이 AI · 스캔 설계(get_design)의 이름 · 윗면 높이를 낸다 ----------------
def robot_cfg():
    """robot.yaml(저장소의 정본). 10/7 #51 이후 src/d2_robot/ 아래 — 옛 자리도 찾는다."""
    path = next((p for p in (ROOT / 'src/d2_robot/d2_bringup/config/robot.yaml', ROOT / 'src/d2_bringup/config/robot.yaml') if p.exists()), None)
    if path is None:
        pytest.skip('robot.yaml 이 없다')
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def generated_design(model_id, family, design_id, made_by):
    """기본 설계(model_id)의 블록 JSON 을 design_id 설계로 바꿔 변환기 ① 에 통과시킨 design/2.0 — AI 생성 · 스캔 저장 설계 모양(DB 에만 있음).
    블록 이름은 `<design_id 대문자>_<역할>_<부품>_<번호>` 가 된다."""
    import copy
    rd = pytest.importorskip('d2_task.recipe_document')
    rb = pytest.importorskip('d2_task.recipe_to_blocks')
    b2r = pytest.importorskip('d2_task.blocks_to_recipe')
    dc = pytest.importorskip('d2_task.design_checker')
    cfg = robot_cfg()
    block_mm = [v * 1000.0 for v in cfg['block_size_m']]
    base = rd.RecipeDocument.load(ROOT / 'src/recipe_manager/recipes', model_id)
    blocks = copy.deepcopy(rb.RecipeToBlocks(model_id, family, block_mm).convert(base.recipe, base.placements))
    blocks['design_id'] = design_id
    out = b2r.BlocksToRecipe(dc.DesignChecker(cfg).grasp_options, block_mm).convert(blocks)
    doc = rd.RecipeDocument(out['recipe'], out['placements']).design(design_id)
    return json.loads(json.dumps(dict(doc, made_by=made_by, family=family)))       # get_design 답은 DB 칸(made_by 등)이 더 붙는다 — 모르는 칸은 무시한다


def base_tops(model_id):
    """기본 설계 파일(로컬 길)의 {블록 이름: 윗면 높이 m} — AI · 스캔 설계와 높이를 맞춰 본다."""
    mm = pytest.importorskip('d2_motion.motion_math')
    cfg = robot_cfg()
    return {b['block_id']: b['center'][2] + mm.half_height(b['rot'], cfg['block_actual_m'])
            for b in mm.recipe_blocks(cfg, mm.load_recipe(str(ROOT / 'src/recipe_manager/recipes' / f'{model_id}_recipe.json')))}


@pytest.mark.parametrize('model_id, family, design_id, made_by', [
    ('001_CHAIR_BENCH', 'chair', 'ai_chair_v1_1', 'ai'),
    ('004_DESK_PEDESTAL', 'desk', 'scan_desk_0001', 'scan'),
])
def test_mock이_get_design의_AI_스캔_설계로_블록_이름과_윗면_높이를_낸다(tmp_path, model_id, family, design_id, made_by):
    """recipe_dir 에는 이 설계가 없다 — 오직 get_design(design/2.0) 으로만 알 수 있다. 블록 이름은 설계 이름 대문자 + 역할 · 부품 · 번호,
    높이는 같은 모양의 기본 설계와 같다(같은 블록을 같은 실측 두께로 쌓으니까). 다른 설계의 이름은 unknown."""
    mm = pytest.importorskip('d2_motion.motion_math')
    cfg = robot_cfg()
    doc = generated_design(model_id, family, design_id, made_by)
    real_tops = load_function(PKG / 'mock_wrist_block.py', 'recipe_tops', {'half_height': mm.half_height, 'recipe_blocks': mm.recipe_blocks})
    hh = Harness('mock', tmp_path, registry={design_id: doc}, real_tops=real_tops)
    hh.node.cfg = cfg
    expected = {name.replace(model_id, design_id.upper(), 1): top for name, top in base_tops(model_id).items()}
    assert all(name.startswith(design_id.upper() + '_') for name in expected)
    names = list(expected)
    res = SimpleNamespace()
    hh.node.on_check(SimpleNamespace(design_id=design_id, run_id='R1', block_ids=names + [f'{model_id}_LEG_001_01']), res)
    assert res.success and res.reason == ''
    assert res.states == ['present'] * len(names) + ['unknown']                       # 기본 설계 이름은 이 설계에 없다
    assert res.top_z_m[:-1] == pytest.approx([expected[n] for n in names], abs=1e-9)
    assert math.isnan(res.top_z_m[-1]) and all(math.isnan(v) for v in res.dx_m + res.dy_m)
    hh.node.on_check(SimpleNamespace(design_id=design_id, run_id='R1', block_ids=names), SimpleNamespace())
    assert hh.client.requests == [design_id]                                           # 같은 판 — 한 번만 받았다
    hh.node.on_check(SimpleNamespace(design_id=design_id, run_id='R2', block_ids=names), SimpleNamespace())
    assert hh.client.requests == [design_id, design_id]                                # 새 판 — 다시 받았다


# ---------------- ④ 동시 요청 — 노드 배선(ROS 없이 소스로 확인). 실제로 겹쳐 보내는 시험은 test_design_cache_ros.py ----------------
def group_kinds(path, class_name):
    """노드 __init__ 에서 서비스 · 클라이언트가 쓰는 콜백 그룹 종류를 소스로 읽는다. 반환: ({서비스 이름: (그룹 변수 또는 '<inline>', 종류)}, get_design 클라이언트의 (그룹 변수, 종류)).
    종류는 'exclusive' · 'reentrant' · None(그 밖 — 기본 그룹 등). 변수는 같은 __init__ 안의 `이름 = XxxCallbackGroup()` 대입으로 풀이한다."""
    tree = ast.parse(Path(path).read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    kinds = {'MutuallyExclusiveCallbackGroup': 'exclusive', 'ReentrantCallbackGroup': 'reentrant'}
    assigned = {t.id: kinds.get(getattr(n.value.func, 'id', ''))
                for n in ast.walk(init) if isinstance(n, ast.Assign) and isinstance(n.value, ast.Call)
                for t in n.targets if isinstance(t, ast.Name)}

    def kind_of(call):
        value = next((k.value for k in call.keywords if k.arg == 'callback_group'), None)
        if isinstance(value, ast.Name):
            return value.id, assigned.get(value.id)
        if isinstance(value, ast.Call):
            return '<inline>', kinds.get(getattr(value.func, 'id', ''))
        return None, None
    services, client = {}, None
    for n in ast.walk(init):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and len(n.args) >= 2 and isinstance(n.args[1], ast.Constant):
            if n.func.attr == 'create_service':
                services[n.args[1].value] = kind_of(n)
            elif n.func.attr == 'create_client' and n.args[1].value == '/d2/hmi/get_design':
                client = kind_of(n)
    return services, client


@pytest.mark.parametrize('file, cls', [('wrist_block.py', 'WristBlock'), ('mock_wrist_block.py', 'MockWristBlock')])
def test_check_progress는_차례로_처리하고_get_design_응답은_다른_그룹이다(file, cls):
    """RunDesignCache 는 스레드 안전하지 않다 — check_progress 가 겹쳐 들어오면 같은 판을 두 번 읽고 지난 판의 답이 새 판 캐시를 덮는다.
    그래서 check_progress 는 MutuallyExclusiveCallbackGroup(차례로), get_design 응답은 서비스와 다른 재진입 그룹이어야 한다(같은 그룹이면 응답이 못 돌아 멈춘다)."""
    services, client = group_kinds(PKG / file, cls)
    name, kind = services['/d2/vision/check_progress']
    assert kind == 'exclusive', f'check_progress 그룹이 차례로 처리하는 그룹이 아니다: {name}'
    assert client is not None and client[1] == 'reentrant' and client[0] != name, f'get_design 응답 그룹이 서비스와 같거나 재진입이 아니다: {client}'


def test_mock의_네_서비스는_한_직렬_그룹이다():
    """옛 단일 스레드 실행기와 같은 직렬 동작 — MockScan 의 run 별 촬영 · 번호 공유 상태를 서비스끼리 동시에 건드리지 않는다."""
    services, _ = group_kinds(PKG / 'mock_wrist_block.py', 'MockWristBlock')
    names = {'/d2/vision/check_progress', '/d2/vision/scan_capture', '/d2/vision/scan_infer', '/d2/vision/find_blocks'}
    assert set(services) == names
    assert len({v[0] for v in services.values()}) == 1 and all(v[1] == 'exclusive' for v in services.values())

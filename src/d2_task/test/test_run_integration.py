# -*- coding: utf-8 -*-
"""한 run 의 기록 일치 · 서로 다른 설계 두 run 의 저장 실패 보존 시험 (W122 task 쪽 준비, 로봇 · 웹 · ROS 없이).

설계 수신(design/2.0) → 가상 조립 → 진행표 → CSV → build/1 → 저장 요청(BuildSender)이 한 실행 흐름에서 같은 설계 · 같은 run 을 가리키는지 본다.
검사 묶음과 변환기 ①은 실제 DesignChecker · BlocksToRecipe 를 쓴다(task_node 와 같은 배선). 실제 TaskManager · TaskPlanner · RunLogger(파일) · BuildSender 를
쓰고, 로봇 · 원격 서비스 경계만 FakeIO · FakeFuture 로 대신한다. 내부 상태를 DONE · present 로 바꾸지 않는다(새 조립 전에 작업대를 비울 뿐 — test_build_saving.assemble 과 같음).

범위 밖: 웹 · MQTT · DB · 실제 ROS 서비스 · 손목 블록 인식의 설계 캐시. check_progress 요청에 실리는 design_id · run_id 는 task 쪽 값(manager)을 기록해 볼 뿐이다.
공급 방식은 robot.yaml 기본(공급 칸)이고, 흩뿌림 · 세우기 경로는 포함하지 않는다. 정지 · ERROR · 서로 겹치는 run 은 test_build_saving · test_run_logger 가 따로 본다.
"""
import copy
import csv
import json

import pytest

from d2_task.blocks_to_recipe import BlocksToRecipe
from d2_task.build_sender import BuildSender
from d2_task.design_checker import DesignChecker
from d2_task.run_logger import COLUMNS, RunLogger
from d2_task.task_manager import TaskManager
from test_build_saving import OK_JSON, RETRY_S, FakeFuture
from test_generation_flow import generated_design
from test_task_manager import CFG, SAFE_OK, FakeIO, drive, picks

A_ID, B_ID = 'chair_v1.1', 'desk_v2'     # 블록 수가 다른 두 설계(11개 · 9개) — 기록이 섞이면 개수 · 이름으로 드러난다


def store_with_real_converter(blocks):
    """blocks/2.0 을 실제 검사 묶음 + 변환기 ①에 통과시키고, 합격 응답의 recipe · placements 를 design/2.0 글자로 저장했다 읽어 온다.

    입력: blocks/2.0 dict. 출력: design/2.0 dict. 바깥 영향 없음(저장소 · MQTT 는 JSON 글자 왕복으로 흉내).
    실패: 합격하지 못하면 AssertionError(응답 글자 포함).
    """
    checker = DesignChecker(CFG)
    checker.blocks_to_recipe = BlocksToRecipe(checker.grasp_options, [v * 1000.0 for v in CFG['block_size_m']]).convert   # task_node 와 같은 배선
    ok, reason, text = checker.handle_json(json.dumps(blocks))
    assert (ok, reason) == (True, ''), text
    result = json.loads(text)
    assert result['ok'], result['errors']
    return json.loads(json.dumps({'schema': 'design/2.0', 'design_id': blocks['design_id'],
                                  'recipe': result['recipe'], 'placements': result['placements']}))


@pytest.fixture(scope='module')
def designs():
    """실제 변환기를 거친 design/2.0 두 개: chair_v1.1(001_CHAIR_BENCH 기하) · desk_v2(003_DESK_STAND 기하)."""
    return {A_ID: store_with_real_converter(generated_design('001_CHAIR_BENCH', A_ID)),
            B_ID: store_with_real_converter(generated_design('003_DESK_STAND', B_ID))}


class MultiIO(FakeIO):
    """get_design 이 저장소(설계 이름 → design/2.0)에서 꺼내 주는 가짜. check_progress 에 실리는 design_id · run_id 도 기록한다."""

    def __init__(self, designs):
        """설계 저장소를 받는다. check_requests 는 (design_id, run_id 또는 빈 글자, block_ids) 목록."""
        super().__init__()
        self.designs = designs
        self.check_requests = []

    def check_progress(self, block_ids, should_abort):
        """task_node.check_progress 가 요청 칸을 채우는 것과 같은 출처(manager.design_id · manager.run_id)를 기록하고 FakeIO 로 답한다."""
        self.check_requests.append((self.manager.design_id, self.manager.run_id or '', tuple(block_ids)))
        return super().check_progress(block_ids, should_abort)

    def get_design(self, design_id, should_abort):
        """저장소에 있는 설계의 복사본을 돌려준다. 없으면 (False, '', None)."""
        self.events.append(('call', 'load', design_id))
        doc = self.designs.get(design_id)
        return (True, '', copy.deepcopy(doc)) if doc else (False, '', None)


class Rig:
    """TaskManager(실제 RunLogger 파일) + BuildSender(가짜 서비스 future).

    save_build 요청 글자는 task_node._call_save 와 같은 방식(ensure_ascii=False, allow_nan=False)으로 만들어, 요약에 NaN 이 있으면 여기서 예외가 난다.
    BuildSender 의 시계는 now 를 직접 올린다.
    """

    def __init__(self, tmp_path, designs):
        """로그 폴더를 tmp_path 로 한 TaskManager 를 만들고 정지 · 그리퍼 신호를 받은 상태로 둔다."""
        self.dir = tmp_path
        self.io = MultiIO(designs)
        self.m = TaskManager(CFG, self.io, RunLogger(str(tmp_path)))
        self.io.manager = self.m
        self.m.on_safety(SAFE_OK)
        self.m.on_gripper({'grasped': False})
        self.sent, self.cancelled, self.now = [], [], 0.0
        self.sender = BuildSender(self.m, self._call, self.cancelled.append, lambda: True, RETRY_S, lambda: self.now)

    def _call(self, summary):
        """BuildSender 가 부르는 save_build 호출 자리: 보낼 글자까지 만들어 future 에 붙여 sent 에 쌓는다."""
        f = FakeFuture()
        f.summary = summary
        f.payload = json.dumps(summary, ensure_ascii=False, allow_nan=False)
        self.sent.append(f)
        return f

    def assemble(self, design_id):
        """작업대를 비우고(새 조립) 설계를 골라 출발해 DONE 까지 돈다. CSV 쓰기가 끝날 때까지 기다린다. 반환: run_id."""
        self.io.world = set()
        assert self.m.command('select_design', design_id) == (True, '')
        assert self.m.command('start') == (True, '')
        drive(self.m, 'DONE')
        assert self.m.logger.flush(5.0)
        return self.m.run_id

    def rows(self, run_id):
        """그 run 의 CSV 줄들(dict 목록)."""
        with open(self.dir / f'{run_id}.csv', encoding='utf-8', newline='') as f:
            return list(csv.DictReader(f))

    def pending_file(self, run_id):
        """그 run 의 미전송 요약 보관 파일 경로."""
        return self.dir / 'pending_builds' / f'{run_id}.json'

    def pump(self):
        """디스크 보관 · 삭제 쓰기를 마치고 BuildSender 를 한 번 돌린 뒤 다시 쓰기를 마친다."""
        assert self.m.logger.flush(5.0)
        self.sender.poll()
        assert self.m.logger.flush(5.0)


def expected_ids(design):
    """조립 방법의 sequence 순서대로 본 block_id 목록."""
    return [s['block_id'] for s in sorted(design['placements']['steps'], key=lambda s: s['sequence'])]


def request_ids(io, since=0):
    """pick_place 로 보낸 요청의 block_id 들(순서대로). since = 앞에서 이미 센 요청 수."""
    return [b for b, _ in picks(io)][since:]


def test_한_run_안에서_요청_진행표_CSV_build1이_같은_설계_같은_run을_가리킨다(tmp_path, designs):
    r = Rig(tmp_path, designs)
    a = r.assemble(A_ID)
    ids = expected_ids(designs[A_ID])
    n = len(ids)
    assert n == 11 and request_ids(r.io) == ids                                        # 요청: 조립 방법 순서 그대로
    mine = [p for p in r.io.progress if p['run_id'] == a]                              # 진행표: 이 run 의 메시지는 모두 이 설계 · 이 run
    assert mine and all(p['design_id'] == A_ID for p in mine)
    assert all({b['block_id'] for b in p['blocks']} == set(ids) for p in mine)
    assert {b['block_id']: b['state'] for b in mine[-1]['blocks']} == {i: 'present' for i in ids}
    assert {p['run_id'] for p in r.io.progress} <= {None, a}                           # 다른 run_id 가 섞이지 않는다
    rows = r.rows(a)                                                                   # CSV
    assert list(rows[0]) == list(COLUMNS)
    assert {x['run_id'] for x in rows} == {a} and {x['design_id'] for x in rows} == {A_ID}
    assert [x['block_id'] for x in rows if x['kind'] == 'placed'] == ids
    assert (rows[0]['kind'], rows[0]['value']) == ('start', '11') and (rows[-1]['kind'], rows[-1]['value']) == ('end', 'DONE')
    s = r.m.last_build                                                                 # build/1: 마지막 요약 · 대기 목록 · 디스크 보관 · 보낼 글자
    assert (s['schema'], s['run_id'], s['design_id'], s['result'], s['placed'], s['total']) == ('build/1', a, A_ID, 'DONE', n, n)
    assert [b['block_id'] for b in s['blocks']] == ids
    assert r.m.pending_builds == {a: s}
    assert json.loads(r.pending_file(a).read_text(encoding='utf-8')) == s
    r.pump()
    assert len(r.sent) == 1 and json.loads(r.sent[0].payload) == s
    assert r.io.check_requests and {(d, rid) for d, rid, _ in r.io.check_requests} == {(A_ID, a)}   # check_progress 에 실리는 값: 한 run 동안 같은 (설계, run), run_id 는 비지 않는다
    assert all(set(blocks) <= set(ids) for _, _, blocks in r.io.check_requests)
    assert (set(request_ids(r.io)) == {b['block_id'] for b in mine[-1]['blocks']}      # 네 곳이 같은 블록 집합
            == {x['block_id'] for x in rows if x['kind'] == 'placed'} == {b['block_id'] for b in s['blocks']})


def test_다른_설계의_다음_run은_이전_기록이_섞이지_않고_저장실패한_첫_결과가_보존된다(tmp_path, designs):
    r = Rig(tmp_path, designs)
    a = r.assemble(A_ID)
    ids_a, ids_b = expected_ids(designs[A_ID]), expected_ids(designs[B_ID])
    assert len(ids_a) == 11 and len(ids_b) == 9 and not set(ids_a) & set(ids_b)
    r.pump()                                                                           # A 를 한 번 보낸다
    r.sent[0].done(success=True, body='{"ok": false}')                                 # 서비스는 답했지만 저장 실패
    assert list(r.m.pending_builds) == [a]
    snap_a = copy.deepcopy(r.m.pending_builds[a])
    csv_a = r.rows(a)
    disk_a = r.pending_file(a).read_text(encoding='utf-8')
    picks_a, progress_a = len(picks(r.io)), len(r.io.progress)

    b = r.assemble(B_ID)                                                               # 다른 설계로 다음 run
    assert b != a
    assert request_ids(r.io, picks_a) == ids_b                                         # 요청 · 진행표: B 구간에는 B 이름만
    new = r.io.progress[progress_a:]
    mine_b = [p for p in new if p['run_id'] == b]
    assert mine_b and all(p['design_id'] == B_ID and {x['block_id'] for x in p['blocks']} == set(ids_b) for p in mine_b)
    assert {p['run_id'] for p in new} <= {None, b}                                     # A 의 run_id 는 B 진행표에 다시 나오지 않는다
    pairs = [(d, rid) for d, rid, _ in r.io.check_requests]                            # check_progress 에 실리는 (설계, run): A 쌍이 모두 먼저, 그 뒤 B 쌍, 섞인 쌍 없음
    assert set(pairs) == {(A_ID, a), (B_ID, b)} and pairs == sorted(pairs, key=lambda p: p[1] == b)
    assert all(set(blocks) <= set(ids_b) for _, rid, blocks in r.io.check_requests if rid == b)
    rows_b = r.rows(b)                                                                 # CSV: B 는 B 만, A 는 그대로
    assert {x['run_id'] for x in rows_b} == {b} and {x['design_id'] for x in rows_b} == {B_ID}
    assert [x['block_id'] for x in rows_b if x['kind'] == 'placed'] == ids_b
    assert r.rows(a) == csv_a
    sb = r.m.pending_builds[b]                                                         # build/1: 각자 자기 설계 · 개수, A 는 한 글자도 안 바뀐다
    assert (sb['run_id'], sb['design_id'], sb['placed'], sb['total'], sb['result']) == (b, B_ID, 9, 9, 'DONE')
    assert [x['block_id'] for x in sb['blocks']] == ids_b and r.m.last_build == sb
    assert set(r.m.pending_builds) == {a, b} and r.m.pending_builds[a] == snap_a
    assert (snap_a['design_id'], snap_a['placed']) == (A_ID, 11) and [x['block_id'] for x in snap_a['blocks']] == ids_a
    assert r.pending_file(a).read_text(encoding='utf-8') == disk_a
    assert json.loads(r.pending_file(b).read_text(encoding='utf-8')) == sb
    r.now = 0.5                                                                        # 방금 실패한 A 는 다시 보내기 간격 안이라 B 만 나간다
    r.pump()
    assert len(r.sent) == 2 and r.sent[1].summary['run_id'] == b and json.loads(r.sent[1].payload) == sb
    r.sent[1].done(success=True, body=OK_JSON)                                         # B 저장이 확인돼야 B 만 지워진다
    assert r.m.logger.flush(5.0)
    assert list(r.m.pending_builds) == [a] and r.m.pending_builds[a] == snap_a
    assert not r.pending_file(b).exists() and r.pending_file(a).read_text(encoding='utf-8') == disk_a
    assert r.m.last_build == sb                                                        # 표시용 최근 결과는 B
    r.now = RETRY_S + 1                                                                # 간격이 지나면 A 가 같은 내용으로 다시 나가고, 확인되면 A 도 지워진다
    r.pump()
    assert len(r.sent) == 3 and r.sent[2].summary == snap_a and json.loads(r.sent[2].payload) == snap_a
    r.sent[2].done(success=True, body=OK_JSON)
    assert r.m.logger.flush(5.0)
    assert r.m.pending_builds == {} and not r.pending_file(a).exists() and r.sender.waiting() == []

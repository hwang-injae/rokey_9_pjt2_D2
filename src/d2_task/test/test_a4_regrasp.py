"""W144 A4 자동 다시 집기(기본 켬 — 10/15 PL) — 가짜 측정 입력으로 흐름만 본다(실제 재픽 효과 · 비전 · 로봇은 확인 안 함). ROS 없음."""
import json
import time

from d2_task.scatter_pick import block_pose
from d2_task.task_manager import A4_MAX_TRIES, TaskManager, design_up, regrasp_decision
from test_a4_measure import MeasureIO, ok_body
from test_task_manager import CFG, IDS, SAFE_OK, drive


class RegraspIO(MeasureIO):
    """측정 · 다시 집기를 켠 가짜. offsets = 부를 때마다 꺼내 쓰는 (dx_mm, dy_mm) 또는 응답 dict(없으면 (0, 0)).
    moved_fail = 다시 집기 목표에서 '움직인 뒤 실패'를 흉내 낼 이유."""

    def __init__(self, offsets=(), regrasp=True, **kw):
        super().__init__('ok')
        self.offsets, self.regrasp, self.kw, self.goals, self.moved_fail = list(offsets), regrasp, kw, [], None

    def w144_regrasp_on(self):
        return self.regrasp

    def w144_measure(self, run_id, design_id, part_id, should_abort):
        self.measured.append((run_id, part_id))
        off = self.offsets.pop(0) if self.offsets else (0.0, 0.0)
        if isinstance(off, dict):
            return True, '', off
        part = next(p for p in self.manager.planner.parts if p['part_id'] == part_id)
        c = part['center']
        b = {'x_m': c[0] + off[0] / 1000, 'y_m': c[1] + off[1] / 1000, 'yaw_deg': 1.0, 'up': design_up(part['quat']), 'tilted': False,
             'top_z_m': c[2] + 0.0074}
        b.update(self.kw)
        return True, '', ok_body(part_id, [b], stamp=self.kw.get('stamp', time.time()))      # 요청 · 답 사이(같은 시계)

    def pick_place(self, goal, should_abort, on_feedback=None, on_result=None):
        self.goals.append(dict(goal))
        if goal.get('a4_regrasp') and self.moved_fail:
            on_feedback('pick')                               # 팔이 움직인 뒤
            return False, self.moved_fail
        return super().pick_place(goal, should_abort, on_feedback, on_result)


def start(io):
    m = TaskManager(CFG, io)
    io.manager = m
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    rows = []
    m.logger.log = lambda kind, part_id=None, value=None, module='task': rows.append((kind, part_id, value))
    assert m.command('select_design', 'bench') == (True, '') and m.command('start') == (True, '')
    return m, rows


def picks_of(io, pid):
    return [g for g in io.goals if g['part_id'] == pid]


def test_꺼져_있으면_벗어나도_다시_집지_않는다():
    io = RegraspIO([(5.0, 0.0)], regrasp=False)
    m, rows = start(io)
    drive(m, 'DONE')
    assert len(picks_of(io, IDS[0])) == 1 and not [r for r in rows if r[0] == 'a4_regrasp']


def test_3mm_넘으면_잰_자리에서_집어_설계_자리에_놓고_다시_잰다():
    io = RegraspIO([(5.0, -1.0), (0.5, 0.2)])
    m, rows = start(io)
    drive(m, 'DONE')
    g = picks_of(io, IDS[0])
    assert len(g) == 2 and g[1]['a4_regrasp'] == 1 and g[1]['supply_slot'] == ''
    assert list(g[1]['place_pose'][0]) == list(g[0]['place_pose'][0]) and list(g[1]['place_pose'][1]) == list(g[0]['place_pose'][1])   # 설계 자리 · 방향
    assert abs(g[1]['pick_pose'][0][0] - (g[0]['place_pose'][0][0] + 0.005)) < 1e-6      # 잰 중심(x +5 mm)에서 집음
    assert [p for _, p in io.measured][:2] == [IDS[0], IDS[0]]                          # 다시 놓은 뒤 다시 잼
    assert io.world == set(IDS)
    assert [v for k, p, v in rows if k == 'a4_regrasp'][0].startswith('1번째 다시 집기')


def test_두번_다시_놓아도_벗어나면_남은_오차를_적고_다음_블록으로():
    """10/14 19:47 사용자 확정: 2번 다시 놓은 뒤에도 문턱(A4_TOL_MM) 이상이면 사람 대기 없이 남은 dx · dy 와 소진을 적고 SELECT 로 이어 간다."""
    io = RegraspIO([(5.0, 0.0)] * 3)
    m, rows = start(io)
    drive(m, 'DONE')
    assert len(picks_of(io, IDS[0])) == 1 + A4_MAX_TRIES                     # 처음 + 2번, 더는 안 집음
    assert all(len(picks_of(io, p)) == 1 for p in IDS[1:])                    # 다음 블록들은 그대로 한 번씩
    assert 'WAIT_SUPPLY' not in [s['state'] for s in io.states]                 # 사람 대기 없음
    logs = [v for k, p, v in rows if k == 'a4_regrasp']
    assert logs[-1].startswith('보정 소진') and 'dx 5.0' in logs[-1]
    assert [f['part_id'] for e, f in io.w144_events if e == 'regrasp_exhausted'] == [IDS[0]]
    assert [p for _, p in io.measured].count(IDS[0]) == 1 + A4_MAX_TRIES        # 놓을 때마다 한 번씩 잼(3번)
    assert m.last_build['placed'] == len(IDS) and io.world == set(IDS)           # 놓은 수 · 완료 중복 없음


def _skipped_done(io, rows, m):
    """측정을 못 써도 사람 대기 없이 끝까지 — 다시 집기 0, 놓은 수 그대로, 까닭 기록(10/14 21:5x 사용자 결정)."""
    drive(m, 'DONE')
    assert 'WAIT_SUPPLY' not in [s['state'] for s in io.states]
    assert all(len(picks_of(io, p)) == 1 for p in IDS)                          # 다시 집지 않음
    assert m.last_build['placed'] == len(IDS) and io.world == set(IDS)
    return [v for k, p, v in rows if k == 'a4_regrasp']


def test_측정_거절이면_다시_집지_않고_까닭을_적고_다음_블록():
    io = RegraspIO([ok_body(IDS[0], [], 'NEIGHBOR_MERGED', '옆 블록과 붙음')])
    m, rows = start(io)
    logs = _skipped_done(io, rows, m)
    assert logs == ['측정 못 씀 — 다시 집기 안 함: 놓은 블록을 재지 못했어요(NEIGHBOR_MERGED: 옆 블록과 붙음)']
    assert [f['part_id'] for e, f in io.w144_events if e == 'regrasp_skipped'] == [IDS[0]]


def test_모든_블록이_늘_거절이어도_대기_반복_없이_끝까지():
    """10/14 진용 실기: 연짝 · 높은 층 블록이 늘 거절되면 [계속] 뒤에도 다시 거절돼 끝나지 않았다 — 이제는 블록마다 한 번 재고 넘어간다."""
    io = RegraspIO([ok_body(p, [], 'SIZE_MISMATCH', '가장자리가 안 보임') for p in IDS])
    m, rows = start(io)
    logs = _skipped_done(io, rows, m)
    assert len(logs) == len(IDS) and [p for _, p in io.measured] == IDS          # 블록마다 한 번만 잼


def test_측정_시각이_요청_앞이면_다시_집지_않고_다음_블록():
    io = RegraspIO([(5.0, 0.0)], stamp=1.0)
    m, rows = start(io)
    logs = _skipped_done(io, rows, m)
    assert 'measured_stamp' in logs[0]


def test_기울어짐_자세_다름이면_다시_집지_않고_다음_블록():
    for kw, word in (({'tilted': True}, '기울'), ({'up': 'LENGTH'}, '자세')):
        io = RegraspIO([(5.0, 0.0)], **kw)
        m, rows = start(io)
        logs = _skipped_done(io, rows, m)
        assert word in logs[0]


def test_다시_집기_길이_없으면_사람_확인_움직인_뒤_실패는_기존_정지처리():
    io = RegraspIO([(5.0, 0.0)])
    io.pick_script = [(True, ''), (False, 'PLAN_FAILED')]
    m, rows = start(io)
    drive(m, 'WAIT_SUPPLY')
    assert 'PLAN_FAILED' in io.states[-1]['message']
    io2 = RegraspIO([(5.0, 0.0)])
    io2.moved_fail = 'GRASP_FAILED'
    m2, _ = start(io2)
    drive(m2, 'ERROR')
    assert '다시 집기' in io2.states[-1]['message']


def test_판단_함수_목표_칸():
    part = {'part_id': 'P', 'center': (0.4, -0.05, 0.0074), 'quat': (0.0, 0.0, 0.0, 1.0), 'grasp': 'FLAT_SHORT'}
    row = {'status': 'MEASURED', 'x_m': 0.405, 'y_m': -0.05, 'dx_mm': 5.0, 'dy_mm': 0.0, 'yaw_deg': 2.0, 'up': 'THICKNESS',
           'tilted': False, 'top_z_m': 0.0150, 'measured_stamp': 100.0}
    sent = ((0.4011, -0.05, 0.0074), (0.0, 0.0, 0.0, 1.0))           # 놓을 때 보낸 목표(흩뿌림 1 mm 띄운 자리처럼 설계와 다를 수 있음)
    v, goal = regrasp_decision(row, 99.0, 101.0, part, sent, CFG, 0)
    assert v == 'REGRASP' and goal['pick_pose'] == block_pose(CFG, 'THICKNESS', 2.0, 0.405, -0.05, 0.0150)
    assert goal['place_pose'] == sent and goal['grasp'] == 'FLAT_SHORT' and goal['open_width_m'] == 0.0      # 설계 중심으로 덮어쓰지 않음
    assert regrasp_decision(dict(row, dx_mm=1.99), 99.0, 101.0, part, sent, CFG, 0) == ('OK', None)       # 2 mm 미만만 그대로
    assert regrasp_decision(dict(row, dx_mm=2.0), 99.0, 101.0, part, sent, CFG, 0)[0] == 'REGRASP'      # 2 mm 는 다시 집기(10/14 20:04)
    for bad in (None, dict(row, status='UNMEASURED', reason='FEW_POINTS', detail=''), dict(row, measured_stamp=None),
                dict(row, measured_stamp=98.0), dict(row, measured_stamp=102.0), dict(row, tilted=None), dict(row, up='WIDTH'),
                dict(row, yaw_deg=None), dict(row, top_z_m=None)):
        assert regrasp_decision(bad, 99.0, 101.0, part, sent, CFG, 0)[0] == 'SKIP'
    assert regrasp_decision(row, 99.0, 101.0, part, None, CFG, 0)[0] == 'SKIP'          # 원래 목표를 모름
    assert regrasp_decision(row, 99.0, 101.0, part, sent, CFG, A4_MAX_TRIES)[0] == 'EXHAUSTED'              # 소진 — 이어 감
    assert regrasp_decision(dict(row, dx_mm=1.0), 99.0, 101.0, part, sent, CFG, A4_MAX_TRIES)[0] == 'OK'
    assert regrasp_decision(dict(row, dx_mm=2.0), 99.0, 101.0, part, sent, CFG, A4_MAX_TRIES)[0] == 'EXHAUSTED'
    for bad in (dict(row, status='UNMEASURED', reason='FEW_POINTS', detail=''), dict(row, tilted=True), dict(row, measured_stamp=None)):
        assert regrasp_decision(bad, 99.0, 101.0, part, sent, CFG, A4_MAX_TRIES)[0] == 'SKIP'       # 측정 불가는 소진으로 치지 않음(따로 SKIP)
    assert json.dumps(goal['place_pose'])


def test_판단_뒤_적용_전에_정지가_끼면_다시_집지_않는다(monkeypatch):
    import d2_task.task_manager as tm
    from test_task_manager import SAFE_STOP
    io = RegraspIO([(5.0, 0.0)])
    m, rows = start(io)
    real = tm.regrasp_decision

    def decide_then_stop(*a):
        out = real(*a)
        m.on_safety(SAFE_STOP)                                    # 판단은 끝났고 적용 전에 정지
        return out
    monkeypatch.setattr(tm, 'regrasp_decision', decide_then_stop)
    drive(m, 'STOPPED')
    assert len(picks_of(io, IDS[0])) == 1 and not [r for r in rows if r[0] == 'a4_regrasp']


def test_받침_경유_모드_다시_놓는_자리는_처음_보낸_1mm_띄운_목표_놓은_수_중복_없음():
    from test_via_spare import SPARE_CFG, SpareIO, fast_wait, start as spare_start

    class A4Spare(SpareIO):
        def __init__(self):
            super().__init__()
            self.offsets, self.measured, self.w144_events = [(5.0, 0.0)], [], []

        def w144_regrasp_on(self):
            return True

        def w144_measure(self, run_id, design_id, part_id, should_abort):
            self.measured.append(part_id)
            sent = next(g for g in reversed(self.goals) if g['part_id'] == part_id)['place_pose'][0]   # 처음 놓은 목표(띄운 자리) 기준
            part = next(p for p in self.manager.planner.parts if p['part_id'] == part_id)
            off = self.offsets.pop(0) if self.offsets else (0.0, 0.0)
            b = {'x_m': sent[0] + off[0] / 1000, 'y_m': sent[1] + off[1] / 1000, 'yaw_deg': 0.0, 'up': design_up(part['quat']),
                 'tilted': False, 'top_z_m': sent[2] + 0.0074}
            return True, '', ok_body(part_id, [b], stamp=time.time())

        def w144_event(self, event, **fields):
            self.w144_events.append((event, fields))

    a4 = A4Spare()                                                # test_via_spare.make 와 같은 순서로 만든다
    m = TaskManager(SPARE_CFG, a4, open_width_m=0.0)
    a4.manager = m
    fast_wait(m)
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    spare_start(m)
    drive(m, 'DONE')
    first = [g for g in a4.goals if g['part_id'] == IDS[0]]
    from_spare = [g for g in first if g.get('from_spare')]
    regrasp = [g for g in first if g.get('a4_regrasp')]
    assert len(from_spare) == 1 and len(regrasp) == 1 and not regrasp[0].get('via_spare')
    assert regrasp[0]['place_pose'] == from_spare[0]['place_pose']              # 처음 보낸(1 mm 띄운) 목표 그대로
    assert m.last_build['placed'] == len(IDS)                                   # 다시 집어도 놓은 수는 블록 수 그대로
    assert a4.measured.count(IDS[0]) == 2 and a4.measured.count(IDS[1]) == 1    # 다시 놓은 뒤 다시 재고, 다음 블록으로


def test_OK_판정_뒤라도_정지가_끼면_다음_단계로_가지_않는다(monkeypatch):
    import d2_task.task_manager as tm
    from test_task_manager import SAFE_STOP
    io = RegraspIO([(0.5, 0.0)])                                  # 2 mm 안 — OK 판정
    m, rows = start(io)
    real = tm.regrasp_decision

    def ok_then_stop(*a):
        out = real(*a)
        assert out == ('OK', None)
        m.on_safety(SAFE_STOP)
        return out
    monkeypatch.setattr(tm, 'regrasp_decision', ok_then_stop)
    drive(m, 'STOPPED')
    assert len(io.goals) == 1 and [p for _, p in io.measured] == [IDS[0]]                       # 두 번째 블록으로 안 감


def test_measured_stamp_1ms_반올림은_받고_그보다_밖은_막는다():
    """#177 은 measured_stamp 를 round(…, 3) 으로 보낸다 — 요청 · 답 시각 경계에서 0.5 ms 반올림은 '이번 요청'으로 본다."""
    from d2_task.task_manager import STAMP_ROUND_S
    part = {'part_id': 'P', 'center': (0.4, -0.05, 0.0074), 'quat': (0.0, 0.0, 0.0, 1.0), 'grasp': 'FLAT_SHORT'}
    row = {'status': 'MEASURED', 'x_m': 0.405, 'y_m': -0.05, 'dx_mm': 5.0, 'dy_mm': 0.0, 'yaw_deg': 0.0, 'up': 'THICKNESS',
           'tilted': False, 'top_z_m': 0.0148}
    sent = (part['center'], part['quat'])
    t_req, t_resp = 100.0004, 100.0006                           # 가짜 비전처럼 요청 · 답이 1 ms 안
    for stamp in (round(t_req, 3), round(t_resp, 3)):            # 100.000 · 100.001 — 반올림으로 경계 밖에 찍힌 값
        assert regrasp_decision(dict(row, measured_stamp=stamp), t_req, t_resp, part, sent, CFG, 0)[0] == 'REGRASP'
    for stamp in (t_req - STAMP_ROUND_S - 1e-4, t_resp + STAMP_ROUND_S + 1e-4):
        assert regrasp_decision(dict(row, measured_stamp=stamp), t_req, t_resp, part, sent, CFG, 0)[0] == 'SKIP'


def _judge(dx_m, dy_m):
    """설계 중심 (0.4, −0.05) 에서 잰 중심이 (dx_m, dy_m) 만큼 비킨 응답 → placed_offset → regrasp_decision 결과."""
    from d2_task.task_manager import placed_offset
    part = {'part_id': 'P', 'center': (0.4, -0.05, 0.0074), 'quat': (0.0, 0.0, 0.0, 1.0), 'grasp': 'FLAT_SHORT'}
    body = ok_body('P', [{'x_m': 0.4 + dx_m, 'y_m': -0.05 + dy_m, 'yaw_deg': 0.0, 'up': 'THICKNESS', 'tilted': False, 'top_z_m': 0.0148}],
                   stamp=100.0)
    row = placed_offset(part['center'], 'P', True, '', body)
    return regrasp_decision(row, 99.0, 101.0, part, (part['center'], part['quat']), CFG, 0)[0], row


def test_경계_2mm_이상이면_다시_집기_미만이면_그대로_반올림_전_값으로():
    """10/14 20:04: |dx| ≥ 2 mm 또는 |dy| ≥ 2 mm 면 다시 집기. 두 축 모두 2 mm 미만일 때만 OK. 기록용 반올림(소수 2자리) 전 값으로 판단."""
    from d2_task.task_manager import A4_TOL_MM
    assert A4_TOL_MM == 2.0
    cases = {}
    for sign in (1, -1):
        for v, want in ((0.00199, 'OK'), (0.002, 'REGRASP'), (0.00201, 'REGRASP')):
            cases[(sign * v, 0.0)] = want                            # x 만
            cases[(0.0, sign * v)] = want                            # y 만
    cases.update({(0.00199, -0.00199): 'OK', (-0.00199, 0.002): 'REGRASP', (0.002, -0.002): 'REGRASP', (-0.00201, 0.00199): 'REGRASP',
                  (-0.00199, -0.00199): 'OK'})
    for (dx, dy), want in cases.items():
        assert _judge(dx, dy)[0] == want, (dx, dy)
    verdict, row = _judge(0.001996, 0.0)                         # 기록은 2.0 으로 보이지만 실제 1.996 mm
    assert row['dx_mm'] == 2.0 and verdict == 'OK'
    verdict, row = _judge(0.0, -0.001996)
    assert row['dy_mm'] == -2.0 and verdict == 'OK'


def test_다시_집기는_기본으로_켜고_측정은_따로_끌_수_있다():
    """10/15 PL: A4 다시 집기 늘 켬(시연 포함) — task_node 파라미터 기본값 w144_regrasp=True, 끌 때만 -p w144_regrasp:=false.
    w144_measure · w144_corner_judge 기본값은 그대로 꺼짐(다시 집기가 켜지면 측정도 같이 켜진다 — w144_regrasp_on)."""
    import ast
    from pathlib import Path
    tree = ast.parse((Path(__file__).parents[1] / 'd2_task/task_node.py').read_text(encoding='utf-8'))
    defaults = {c.args[0].value: c.args[1].value for c in ast.walk(tree)
                if isinstance(c, ast.Call) and getattr(c.func, 'attr', '') == 'declare_parameter'
                and len(c.args) == 2 and isinstance(c.args[0], ast.Constant) and isinstance(c.args[1], ast.Constant)}
    assert defaults['w144_regrasp'] is True
    assert defaults['w144_measure'] is False and defaults['w144_corner_judge'] is False

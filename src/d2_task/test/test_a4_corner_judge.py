"""W144 A4 단일 블록 모서리 판정(선택 · 기본 꺼짐 — task 파라미터 w144_corner_judge) — 모서리 높이(Z) → 중심 · 모서리 x · y
(10/15 사용자 합의, pytest · ROS · 로봇 없음). 꺼져 있으면 지금 판정(중심 dx · dy 만) 그대로.

Z: 모서리 4곳 중 가장 낮은 1곳을 빼고 남은 3곳의 평균과 하나라도 5 mm 이상 다르면 Z 문제 → 다시 집지 않고 사람 수정 대기([계속]이면 다시 잼).
   5 mm 는 '세 점 평균 대비 편차' 문턱이다 — 원시 높이 차(들린 양)로는 한 모서리 · 긴 끝 두 모서리 · 대각선 모두 7.5 mm 부터 Z 문제(2h/3).
   네 모서리가 같이 떠 있는 것(평균 높이)은 이 판정이 아니라 VERIFY 높이(±두께/2, 받침 경유 5 mm)가 본다.
   4곳을 다 못 얻으면 정상으로 보지 않는다(다시 집지 않고 까닭 · 다음 블록 — #182 규칙, 사람 대기 여부는 결정 대기).
X · Y: Z 통과 뒤 중심과 모서리 4곳(잰 중심 · 잰 yaw · 블록 치수로 만든 계산값)이 축마다 2 mm 이상이면 잰 자세에서 다시 집어 처음 목표에 놓는다.
   블록마다 2번까지, 그 뒤는 남은 오차를 적고 다음 블록. '기준 안'과 '보정 소진'은 기록 글로 가른다.
"""
import math

import pytest

from d2_task.scatter_pick import block_pose
from d2_task.task_manager import (A4_MAX_TRIES, A4_Z_TOL_MM, corner_xy_errors, corner_z_check, design_up,
                                    design_yaw_deg, regrasp_decision)
import time

from test_a4_measure import ok_body
from test_a4_regrasp import RegraspIO, picks_of, start
from test_task_manager import CFG, IDS, drive

A = CFG['block_actual_m'][0] * 500.0          # 윗면 긴 축 절반 mm(37.225)
B = CFG['block_actual_m'][1] * 500.0          # 눕힘 짧은 축 절반 mm(12.4)
FLAT = (0.0, 0.0, 0.0, 1.0)                   # 설계 yaw 0(긴 축 = base x)
ALONG_Y = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))   # 설계 yaw 90°


def z(*mm):
    """모서리 높이 mm 4개(기준 0.0148 m 위 차) → m 목록. None 은 못 잼."""
    return [None if v is None else 0.0148 + v / 1000.0 for v in mm]


# ---------- Z ----------

@pytest.mark.parametrize('mm, want', [
    ((0, 0, 0, 0), 'OK'),
    ((7.49, 0, 0, 0), 'OK'),                    # 한 모서리만 h 들림 → 평균 대비 2h/3 = 4.99 mm(원시 높이 차 7.49 mm 인데 통과)
    ((7.5, 0, 0, 0), 'Z'),                      # 2h/3 = 5.0 mm — 경계는 Z(이상)
    ((6, 6, 0, 0), 'OK'),                       # 긴 끝 두 모서리 6 mm 들림 → 4 mm
    ((7.5, 7.5, 0, 0), 'Z'),                    # 긴 끝 두 모서리 7.5 mm → 5 mm
    ((0, 0, 0, -30), 'OK'),                     # 가장 낮은 1곳(깊이 구멍 · 가장자리 걸침)은 뺀다
    ((9, 9, 9, 9), 'OK'),                       # 네 모서리가 같이 뜸 — 평균 높이는 이 판정이 아니라 VERIFY 높이가 본다
    ((-8, -8, 0, 0), 'Z'),                      # 두 모서리가 낮음 — 하나만 빠지고 남은 −8 이 평균과 5.3 mm
    ((8, 0, 0, -8), 'Z'),                       # 대각선 기울어짐 — −8 을 빼도 +8 이 남음
])
def test_Z_경계와_두_모서리_이상(mm, want):
    assert corner_z_check(z(*mm))[0] == want
    assert A4_Z_TOL_MM == 5.0


@pytest.mark.parametrize('corners', [None, [], z(0, 0, 0), z(0, 0, 0, None), z(None, None, 0, 0), [0.0148, True, 0.0148, 0.0148],
                                     [0.0148, float('nan'), 0.0148, 0.0148], 'abcd'])
def test_모서리_높이를_못_얻으면_정상이_아니다(corners):
    assert corner_z_check(corners)[0] == 'MISSING'


# ---------- X · Y ----------

def row(dx=0.0, dy=0.0, yaw=0.0, up='THICKNESS', corners=None):
    """placed_offset MEASURED 꼴(반올림 전 값 포함)."""
    return {'status': 'MEASURED', 'x_m': 0.4 + dx / 1000, 'y_m': -0.05 + dy / 1000, 'dx_mm': round(dx, 2), 'dy_mm': round(dy, 2),
            'dx_raw_mm': dx, 'dy_raw_mm': dy, 'yaw_deg': yaw, 'up': up, 'tilted': False, 'top_z_m': 0.0148,
            'measured_stamp': 100.0, 'corners_z_m': z(0, 0, 0, 0) if corners is None else corners}


def judge(r, quat=FLAT, tries=0, up='THICKNESS'):
    part = {'part_id': 'P', 'center': (0.4, -0.05, 0.0074), 'quat': quat, 'grasp': 'FLAT_SHORT'}
    return regrasp_decision(r, 99.0, 101.0, part, (part['center'], quat), CFG, tries, corners=True)[0]


def test_회전만_모서리_2mm_는_약_3_1도():
    worst = max(max(abs(e[1]), abs(e[2])) for e in corner_xy_errors(row(yaw=3.0), {'quat': FLAT}, CFG))
    assert abs(worst - (A * math.sin(math.radians(3.0)) + B * (1 - math.cos(math.radians(3.0))))) < 1e-9     # 1.965 mm
    assert judge(row(yaw=3.0)) == 'OK' and judge(row(yaw=3.1)) == 'REGRASP'     # 중심 0 · 회전만
    assert judge(row(yaw=-3.0)) == 'OK' and judge(row(yaw=-3.1)) == 'REGRASP'


def test_이동만_회전만_복합():
    assert judge(row(dx=2.0)) == 'REGRASP' and judge(row(dy=-1.99)) == 'OK'      # 이동만
    assert judge(row(yaw=4.0)) == 'REGRASP'                                      # 회전만(중심 OK)
    assert judge(row(dx=1.0, yaw=2.0)) == 'OK'                                   # 복합인데 모서리 x 1.41 · y 1.31 mm
    assert judge(row(dy=1.0, yaw=2.0)) == 'REGRASP'                              # 복합 — 모서리 y 1.0 + 1.31 = 2.31 mm


def test_yaw_는_180도_대칭으로_감아_같은_모서리끼리():
    assert judge(row(yaw=-89.0), quat=ALONG_Y) == 'OK'                            # 설계 90° · 잰 −89° = 1° 차
    assert judge(row(yaw=86.0), quat=ALONG_Y) == 'REGRASP'                        # 4° 차


def test_옆세움은_짧은_축이_두께_절반():
    errs = corner_xy_errors(row(yaw=3.0, up='WIDTH'), {'quat': FLAT}, CFG)
    t = CFG['block_actual_m'][2] * 500.0
    worst = max(max(abs(e[1]), abs(e[2])) for e in errs)
    assert abs(worst - (A * math.sin(math.radians(3.0)) + t * (1 - math.cos(math.radians(3.0))))) < 1e-9


def test_Z_가_먼저_Z_문제면_다시_집지_않고_사람_대기_못_얻으면_건너뜀():
    assert judge(row(dx=5.0, corners=z(8, 0, 0, 0))) == 'HOLD'                    # x · y 가 커도 Z 문제가 먼저
    assert judge(row(dx=5.0, corners=z(0, None, 0, 0))) == 'SKIP'                 # 못 얻음 — 정상 아님
    assert judge(row(dx=5.0), tries=A4_MAX_TRIES) == 'EXHAUSTED'
    assert judge(row(dx=5.0, corners=z(8, 0, 0, 0)), tries=A4_MAX_TRIES) == 'HOLD'   # 소진 뒤에도 Z 는 사람


# ---------- 꺼짐(기본)은 지금 판정 그대로 ----------

def test_꺼짐이면_모서리_칸이_있어도_보지_않고_중심만():
    part = {'part_id': 'P', 'center': (0.4, -0.05, 0.0074), 'quat': FLAT, 'grasp': 'FLAT_SHORT'}
    off = lambda r: regrasp_decision(r, 99.0, 101.0, part, (part['center'], FLAT), CFG, 0)[0]
    assert off(row(yaw=4.0)) == 'OK'                                            # 회전만 — 꺼짐이면 그대로
    assert off(row(corners=z(8, 0, 0, 0))) == 'OK'                              # Z 문제여도 꺼짐이면 사람 대기 없음
    assert off(row(dx=2.0, corners=z(None, None, None, None))) == 'REGRASP'     # 모서리 못 잼이어도 지금처럼 중심 재픽
    r = row(dx=2.0)
    del r['corners_z_m']
    assert off(r) == 'REGRASP' and judge(r) == 'SKIP'                           # 지금 비전 응답(칸 없음): 꺼짐 = 재픽 / 켜짐 = 못 얻음


# ---------- 흐름(가짜 로봇) — 지금 응답 · 새 응답 · 거절 응답 ----------

class CornerIO(RegraspIO):
    """RegraspIO + 모서리 판정 켜기(on) · 응답 꼴 고르기. offsets 항목 = 응답 dict(그대로) 또는
    {'dx', 'dy', 'dyaw'(설계 yaw 대비 °), 'dz'(모서리 높이 차 mm 4개 — None 이면 칸 없음 = 지금 비전 응답)}."""

    def __init__(self, offsets, on=True):
        super().__init__(offsets)
        self.on = on

    def w144_corner_on(self):
        return self.on

    def w144_measure(self, run_id, design_id, part_id, should_abort):
        self.measured.append((run_id, part_id))
        o = self.offsets.pop(0) if self.offsets else {}
        if 'ok' in o:
            return True, '', o
        part = next(p for p in self.manager.planner.parts if p['part_id'] == part_id)
        c, top = part['center'], part['center'][2] + 0.0074
        b = {'x_m': c[0] + o.get('dx', 0.0) / 1000, 'y_m': c[1] + o.get('dy', 0.0) / 1000,
             'yaw_deg': design_yaw_deg(part['quat']) + o.get('dyaw', 0.0), 'up': design_up(part['quat']), 'tilted': False,
             'top_z_m': top, 'confidence': 1.0, 'size_mm': [74.4, 24.8]}                     # 지금 비전(#177) 블록 칸 그대로
        if o.get('dz', (0.0,) * 4) is not None:                                              # 합의한 새 칸(제안 이름 corners_z_m)
            b['corners_z_m'] = [None if d is None else top + d / 1000 for d in o.get('dz', (0.0,) * 4)]
        return True, '', ok_body(part_id, [b], stamp=time.time())


def logs_of(rows):
    return [v for k, p, v in rows if k == 'a4_regrasp']


def test_지금_비전_응답_꺼짐이면_기존과_같이_중심_재픽_켜짐이면_재픽_안_하고_까닭():
    for on, picks, word in ((False, 2, '1번째 다시 집기'), (True, 1, '측정 못 씀')):
        io = CornerIO([{'dx': 5.0, 'dz': None}, {'dz': None}], on=on)
        m, rows = start(io)
        drive(m, 'DONE')
        assert len(picks_of(io, IDS[0])) == picks and logs_of(rows)[0].startswith(word)
        assert 'WAIT_SUPPLY' not in [s['state'] for s in io.states]
    assert '모서리 높이' in logs_of(rows)[0]
    measure = [v for k, p, v in rows if k == 'a4_measure'][0]
    assert 'corners_z_m' not in measure                                         # 칸이 없는 응답의 기록 꼴은 지금 그대로


def test_새_응답_꺼짐이면_회전_Z_를_보지_않음():
    io = CornerIO([{'dyaw': 4.0, 'dz': (8.0, 0.0, 0.0, 0.0)}], on=False)
    m, rows = start(io)
    drive(m, 'DONE')
    assert len(picks_of(io, IDS[0])) == 1 and not logs_of(rows)                 # 기존과 같이 아무 일 없음(OK 기록도 없음)


@pytest.mark.parametrize('on, code', [(False, c) for c in ('COVERED', 'TILTED', 'NEIGHBOR_MERGED', 'NO_PART', 'SIZE_MISMATCH', 'FEW_POINTS')]
                         + [(True, c) for c in ('COVERED', 'NEIGHBOR_MERGED', 'NO_PART', 'SIZE_MISMATCH', 'FEW_POINTS')])
def test_거절_응답은_기록만_자동_집기_사람_대기_없음_켜짐의_TILTED_만_예외(on, code):
    io = CornerIO([ok_body(IDS[0], [], code, '가짜 거절')], on=on)
    m, rows = start(io)
    drive(m, 'DONE')
    assert len(picks_of(io, IDS[0])) == 1 and 'WAIT_SUPPLY' not in [s['state'] for s in io.states]
    assert logs_of(rows)[0] == f'측정 못 씀 — 다시 집기 안 함: 놓은 블록을 재지 못했어요({code}: 가짜 거절)'
    assert not any(v.startswith('Z 문제') for v in logs_of(rows))               # Z 사람 대기와 따로


def test_Z_문제는_사람_수정_대기_계속이면_다시_재고_이어_감():
    io = CornerIO([{'dz': (7.5, 0.0, 0.0, 0.0)}, {}])                           # 원시 높이 차 7.5 mm(평균 대비 5.0) → 사람이 고친 뒤 평평
    m, rows = start(io)
    drive(m, 'WAIT_SUPPLY')
    assert len(picks_of(io, IDS[0])) == 1                                       # 자동으로 다시 집지 않음
    assert '기울' in io.states[-1]['message'] and logs_of(rows)[-1].startswith('Z 문제')
    assert [f['part_id'] for e, f in io.w144_events if e == 'regrasp_hold'] == [IDS[0]]
    assert m.command('start') == (True, '')                                     # 사람이 고치고 [계속]
    drive(m, 'DONE')
    assert [p for _, p in io.measured].count(IDS[0]) == 2 and len(picks_of(io, IDS[0])) == 1
    assert logs_of(rows)[1] == '기준 안: 다시 집기 0번 뒤 중심 · 모서리 2 mm 안'


def test_Z_경계_7_49mm_는_대기_없이_진행():
    io = CornerIO([{'dz': (7.49, 0.0, 0.0, 0.0)}])
    m, rows = start(io)
    drive(m, 'DONE')
    assert 'WAIT_SUPPLY' not in [s['state'] for s in io.states] and logs_of(rows)[0].startswith('기준 안')


def test_회전만_틀어지면_다시_집어_기준_안을_다시_집은_횟수와_함께_적는다():
    io = CornerIO([{'dyaw': 4.0}, {'dyaw': 0.5}])
    m, rows = start(io)
    drive(m, 'DONE')
    part = next(b for b in m.planner.parts if b['part_id'] == IDS[0])
    regrasp = [g for g in picks_of(io, IDS[0]) if g.get('a4_regrasp')]
    want = block_pose(CFG, design_up(part['quat']), design_yaw_deg(part['quat']) + 4.0, part['center'][0], part['center'][1],
                      part['center'][2] + 0.0074)
    assert len(regrasp) == 1 and regrasp[0]['pick_pose'][1] == pytest.approx(want[1])                     # 잰 자세(4° 틀어진 yaw)에서 집고
    assert regrasp[0]['pick_pose'][0] == pytest.approx(want[0], abs=1e-5)                                 # 잰 x · y 는 기록용 0.01 mm 반올림
    assert regrasp[0]['place_pose'] == picks_of(io, IDS[0])[0]['place_pose']                              # 처음 보낸 목표(위치 · 방향)에 놓음
    mine = [v for k, p, v in rows if k == 'a4_regrasp' and p == IDS[0]]
    assert mine[0].startswith('1번째 다시 집기') and '가장 큰 곳 c' in mine[0]              # 회전만이라 중심은 0 — 까닭은 모서리
    assert mine[-1] == '기준 안: 다시 집기 1번 뒤 중심 · 모서리 2 mm 안'


def test_두번_소진은_기준_안과_다른_기록으로_다음_블록():
    io = CornerIO([{'dyaw': 4.0}] * 3)
    m, rows = start(io)
    drive(m, 'DONE')
    assert len(picks_of(io, IDS[0])) == 1 + A4_MAX_TRIES
    mine = [v for k, p, v in rows if k == 'a4_regrasp' and p == IDS[0]]
    assert mine[-1].startswith('보정 소진') and not any(v.startswith('기준 안') for v in mine)
    assert m.last_build['placed'] == len(IDS) and 'WAIT_SUPPLY' not in [s['state'] for s in io.states]


def test_켜짐이면_TILTED_거절과_tilted_참은_자동_집기_없이_사람_대기_계속이면_다시_잰다():
    """안 B′(PL 확인 대기): 모서리 판정이 켜졌을 때만 TILTED 를 Z 문제처럼 사람 수정 대기. 꺼짐이면 위 시험대로 기록만(#182)."""
    io = CornerIO([ok_body(IDS[0], [], 'TILTED', '긴 축 양 끝 6 mm'), {}])
    m, rows = start(io)
    drive(m, 'WAIT_SUPPLY')
    assert len(picks_of(io, IDS[0])) == 1 and '기울' in io.states[-1]['message'] and 'TILTED' in io.states[-1]['message']
    assert logs_of(rows)[-1].startswith('Z 문제 — 사람 수정 대기')
    assert m.command('start') == (True, '')
    drive(m, 'DONE')
    assert logs_of(rows)[1] == '기준 안: 다시 집기 0번 뒤 중심 · 모서리 2 mm 안'
    assert judge(dict(row(dx=5.0), tilted=True)) == 'HOLD' and judge(dict(row(dx=5.0), tilted=None)) == 'SKIP'
    part = {'part_id': 'P', 'center': (0.4, -0.05, 0.0074), 'quat': FLAT, 'grasp': 'FLAT_SHORT'}
    assert regrasp_decision(dict(row(dx=5.0), tilted=True), 99.0, 101.0, part, (part['center'], FLAT), CFG, 0)[0] == 'SKIP'   # 꺼짐 = 지금대로


@pytest.mark.parametrize('dx, want', [(1.99, 'OK'), (2.0, 'REGRASP'), (-2.0, 'REGRASP')])
def test_XY_2mm_경계(dx, want):
    assert judge(row(dx=dx)) == want and judge(row(dy=dx)) == want


def test_다시_집기_중_정지_뒤_재개해도_처음_목표_횟수가_남고_중복_집기_없음():
    """다시 집기가 팔이 움직인 뒤 정지로 끝남 → STOPPED → 정지 풀림 · 그리퍼 빔 → 이어 감. 처음 놓기 목표(_a4_place)와 다시 집은 횟수(_a4_tries)는
    그대로 남고, 이미 놓인 블록을 한 번 더 놓지 않는다(놓은 수 = 블록 수). 정지 뒤 다시 잼 · 다시 집기를 이어 하지는 않는다(정지 = 사람 확인)."""
    from test_task_manager import SAFE_OK, SAFE_STOP

    class StopIO(CornerIO):
        def pick_place(self, goal, should_abort, on_feedback=None, on_result=None):
            if goal.get('a4_regrasp') and not getattr(self, 'stopped_once', False):
                self.stopped_once = True
                self.goals.append(dict(goal))
                on_feedback('lift')                                  # 팔이 움직인 뒤
                self.manager.on_safety(SAFE_STOP)
                return False, 'STOPPED'
            return super().pick_place(goal, should_abort, on_feedback, on_result)

    io = StopIO([{'dx': 5.0}, {}])
    m, rows = start(io)
    drive(m, 'STOPPED')
    first_target = m._a4_place[IDS[0]]
    assert m._a4_tries[IDS[0]] == 1 and len(picks_of(io, IDS[0])) == 2       # 처음 + 다시 집기(정지)
    m.on_safety(SAFE_OK)
    m.on_gripper({'grasped': False})
    drive(m, 'DONE')
    assert m._a4_place[IDS[0]] == first_target and m._a4_tries[IDS[0]] == 1   # 목표 · 횟수 그대로
    assert len(picks_of(io, IDS[0])) == 2 and all(len(picks_of(io, p)) == 1 for p in IDS[1:])
    assert m.last_build['placed'] == len(IDS)                                  # 놓은 수 중복 없음



def test_A4_사람_대기는_placed_check_공급_대기는_no_match_block_그대로():
    """10/15 PL: A4 모서리 판정의 사람 수정 대기(Z · TILTED)만 WAIT_SUPPLY 알림 placed_check, [계속]이면 VERIFY 에서 다시 잰다.
    받침 경유 · 공급의 사람 확인(_hold 기본)은 no_match_block 그대로."""
    for first in ({'dz': (7.5, 0.0, 0.0, 0.0)}, ok_body(IDS[0], [], 'TILTED', '긴 축 양 끝 6 mm')):
        io = CornerIO([first, {}])
        m, rows = start(io)
        drive(m, 'WAIT_SUPPLY')
        assert io.states[-1]['message_id'] == 'placed_check' and io.states[-1]['part_id'] == IDS[0]
        n = len(io.measured)
        assert m.command('start') == (True, '')
        drive(m, 'VERIFY')
        drive(m, 'SELECT')
        assert len(io.measured) == n + 1 and [p for _, p in io.measured][-1] == IDS[0]     # [계속] → 같은 블록 다시 잼
    from test_a4_regrasp import RegraspIO as R
    io = R([(5.0, 0.0)])
    io.pick_script = [(True, ''), (False, 'PLAN_FAILED')]                   # 다시 집기 길 없음 → 기존 사람 확인
    m, rows = start(io)
    drive(m, 'WAIT_SUPPLY')
    assert io.states[-1]['message_id'] == 'no_match_block'

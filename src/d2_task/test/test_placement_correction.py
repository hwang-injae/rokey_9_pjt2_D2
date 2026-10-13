"""W144 축별 경계·측정 불확실성·재배치 한도를 로봇 없이 검증한다."""

import pytest

from d2_task.placement_correction import PlacementCorrection


def measured(**changes):
    """유한한 실제 중심 측정 행을 만든다(m). 외부 영향 없음."""
    row = dict(state='present', dx_m=0.0, dy_m=0.0, dz_m=0.0)
    row.update(changes)
    return row


@pytest.mark.parametrize('axis', ['dx_m', 'dy_m', 'dz_m'])
@pytest.mark.parametrize('sign', [-1, 1])
def test_axis_boundary(axis, sign):
    """축별 ±3mm는 통과하고 바로 초과하면 재배치한다. 로봇 호출 없음."""
    policy = PlacementCorrection(0.003, 2)
    row = measured()
    row[axis] = sign * 0.003
    assert policy.judge('a', row) == 'PASS'
    row[axis] = sign * 0.003001
    assert policy.judge('a', row) == 'CORRECT'


@pytest.mark.parametrize('state', ['absent', 'occluded', 'unknown'])
def test_uncertain_presence(state):
    """존재를 확인하지 못한 블록은 재배치 후보로 쓰지 않는다. 외부 영향 없음."""
    row = measured()
    row['state'] = state
    assert PlacementCorrection(0.003, 2).judge('a', row) == 'UNMEASURED'


@pytest.mark.parametrize('value', [float('nan'), float('inf'), None, True, '0.004'])
def test_uncertain_coordinate(value):
    """불확실한 좌표는 다른 축이 초과해도 자동 재집기를 허용하지 않는다."""
    row = measured()
    row.update(dx_m=0.004, dy_m=value)
    assert PlacementCorrection(0.003, 2).judge('a', row) == 'UNMEASURED'


def test_budget_survives_success_and_is_per_target():
    """목표별 2회 이후 사람 호출, 성공·재관측으로 예산이 복원되지 않고 새 run만 초기화한다."""
    policy = PlacementCorrection(0.003, 2)
    row = measured()
    row['dx_m'] = 0.004
    for _ in range(2):
        assert policy.judge('a', row) == 'CORRECT'
        assert policy.reserve('a')
    assert policy.judge('a', row) == 'HUMAN'
    assert not policy.reserve('a')
    assert policy.judge('a', measured()) == 'PASS'
    assert policy.judge('a', row) == 'HUMAN'
    assert policy.judge('b', row) == 'CORRECT'
    policy.reset_run()
    assert policy.judge('a', row) == 'CORRECT'


@pytest.mark.parametrize('block_id', ['', '  ', None, [], True])
def test_invalid_target_rejected_before_judgment(block_id):
    """잘못된 목표는 판정과 예산 소비 양쪽에서 거절한다. 외부 영향 없음."""
    policy = PlacementCorrection(0.003, 2)
    with pytest.raises(ValueError):
        policy.judge(block_id, measured())
    with pytest.raises(ValueError):
        policy.reserve(block_id)


def observation(**changes):
    """내부 어댑터의 검증된 실측 입력을 만든다. 현물 정확도 검증을 대신하지 않는다."""
    row=dict(state='present',part_id='a',frame='base_link',identity_confirmed=True,
             accuracy_validated=True,center_m=[.004,0,0],uncertainty_m=[.0005]*3,
             quaternion_xyzw=[0,0,.6,.8],pose_confirmed=True,support_confirmed=True,
             scene_clear=True,regrasp_allowed=True,obstacles=[])
    row.update(changes)
    return row


@pytest.mark.parametrize('delta,u,expected',[(.002,.0005,'PASS'),(.003,.0005,'UNMEASURED'),
                                           (.0035,.0005,'UNMEASURED'),(.004,.0005,'CORRECT')])
def test_uncertainty_interval_at_three_mm(delta,u,expected):
    """센서 불확실성을 더한 범위가 3mm를 걸치면 자동 보정으로 보내지 않는다."""
    row=observation(center_m=[delta,0,0],uncertainty_m=[u]*3)
    assert PlacementCorrection(.003,2).assess('a',[0,0,0],row)['decision']==expected


@pytest.mark.parametrize('changes',[{'accuracy_validated':False},{'identity_confirmed':1},
                                   {'frame':'camera'},{'part_id':'b'},
                                   {'uncertainty_m':[None]*3},{'uncertainty_m':[-.001,0,0]}])
def test_unverified_exploration_never_requests_regrasp(changes):
    """탐색 관측·신원/좌표계/불확실성 오류를 자동 재집기로 보내지 않는다."""
    result=PlacementCorrection(.003,2).prepare_goal('a',([0,0,0],[0,0,0,1]),observation(**changes),'FLAT_SHORT',0)
    assert result['decision']=='UNMEASURED' and result['goal'] is None


def test_plan_uses_actual_pick_pose_original_target_and_obstacles():
    """실측 집기 자세를 쓰고 원래 목표를 유지하며 계획만으로 예산을 소비하지 않는다."""
    policy=PlacementCorrection(.003,2)
    row=observation(obstacles=[([.1,0,0],[0,0,0,1])])
    target=([0,0,0],[0,0,0,1])
    result=policy.prepare_goal('a',target,row,'FLAT_SHORT',0)
    assert result['decision']=='CORRECT'
    assert result['goal']['pick_pose']==([.004,0,0],[0,0,.6,.8])
    assert result['goal']['place_pose']==target
    assert result['goal']['obstacles']==row['obstacles']
    row['center_m'][0]=999
    assert result['goal']['pick_pose'][0][0]==.004
    assert policy.reserve('a') and policy.reserve('a') and not policy.reserve('a')
    assert policy.assess('a',[0,0,0],observation())['decision']=='HUMAN'


@pytest.mark.parametrize('changes,decision',[({'pose_confirmed':False},'UNMEASURED'),
                                            ({'quaternion_xyzw':[0,0,0,0]},'UNMEASURED'),
                                            ({'support_confirmed':False},'HUMAN'),
                                            ({'scene_clear':False},'HUMAN'),
                                            ({'regrasp_allowed':False},'HUMAN'),
                                            ({'obstacles':None},'UNMEASURED')])
def test_missing_pose_support_scene_blocks_goal(changes,decision):
    """방향/받침/주변 상태를 목표 자세로 대신 채워 재집기하지 않는다."""
    result=PlacementCorrection(.003,2).prepare_goal('a',([0,0,0],[0,0,0,1]),observation(**changes),'FLAT_SHORT',0)
    assert result['decision']==decision and result['goal'] is None


def test_bad_target_rotation_is_not_pass_even_with_zero_offset():
    """위치가 맞아도 잘못된 목표 자세는 PASS로 처리하지 않는다."""
    row=observation(center_m=[0,0,0])
    result=PlacementCorrection(.003,2).prepare_goal('a',([0,0,0],[0,0,0,0]),row,'FLAT_SHORT',0)
    assert result['decision']=='UNMEASURED' and result['reason']=='INVALID_TARGET_POSE'

"""RGB·깊이 기반 중심 측정의 이동·접합·모호성·불량 입력을 합성 원자료로 시험한다."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('w144_measurement',Path(__file__).with_name('measurement.py'))
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
PlacementMeasurement=module.PlacementMeasurement


def fixture():
    """관측 거리 0.5m의 정지 카메라와 눕힌 블록 치수 m를 만든다. ROS/로봇 영향 없음."""
    detector=PlacementMeasurement(.003,.005,.10,.002,np.deg2rad(3))
    transform=np.diag([1.,-1.,-1.,1.]);transform[2,3]=.5
    info=dict(height=300,width=400,k=[1000.,0.,200.,0.,1000.,150.,0.,0.,1.],d=[0.,0.,0.,0.,0.],distortion_model='plumb_bob')
    return detector,transform,info,np.array([.07445,.0248,.0148])


def render(detector,transform,info,size,centers):
    """알려진 위치의 목재 색과 이상 깊이를 만들되 정답 위치는 측정기에 주지 않는다."""
    h,w=info['height'],info['width'];v,u=np.indices((h,w))
    rgb=np.full((h,w,3),180,np.uint8);depth=np.full((h,w),.5)
    for center in centers:
        top=center[2]+size[2]/2;z=.5-top
        x=(u-200)*z/1000+transform[0,3];y=-(v-150)*z/1000+transform[1,3]
        mask=(abs(x-center[0])<=size[0]/2)&(abs(y-center[1])<=size[1]/2)
        depth[mask]=z;rgb[mask]=[80,140,190]
    return detector.frame(rgb,depth,info,transform)


@pytest.mark.parametrize('delta',[[.004,0,0],[0,-.006,0],[0,0,.004]])
def test_observation_does_not_snap_to_target(delta):
    """알려진 축별 이동을 관측에서 측정하며 중심을 검색 목표로 되돌리지 않는다."""
    detector,t,info,size=fixture();target=np.array([0.,0.,size[2]/2])
    actual=target+delta;frame=render(detector,t,info,size,[actual])
    result=detector.measure(frame,size,target)
    assert result['status']=='EXPLORATORY'
    assert np.max(abs(np.array(result['center_m'])-actual))<.0006
    assert result['accuracy_validated'] is False


def test_temporal_separation_of_touching_blocks():
    """붙은 같은 높이 블록을 전후 자료로 분리하고 정적 이웃을 새 블록으로 고르지 않는다."""
    detector,t,info,size=fixture();first=np.array([0.,-.0248,size[2]/2]);second=np.array([0.,0.,size[2]/2])
    before=render(detector,t,info,size,[first]);after=render(detector,t,info,size,[first,second])
    assert detector.measure(after,size,second)['status']=='UNKNOWN'
    result=detector.measure(after,size,second,before)
    assert result['status']=='EXPLORATORY'
    assert np.max(abs(np.array(result['center_m'])-second))<.0006


def test_unchanged_scene_is_not_new_placement():
    """동일 장면의 새 촬영을 새 블록으로 오인하지 않는다."""
    detector,t,info,size=fixture();target=np.array([0.,0.,size[2]/2])
    frame=render(detector,t,info,size,[target])
    assert detector.measure(frame,size,target,frame)['status']=='UNKNOWN'


def test_two_new_blocks_are_ambiguous():
    """검색 위치와 가까운 후보를 정답으로 고르지 않고 다중 후보를 거절한다."""
    detector,t,info,size=fixture();target=np.array([0.,0.,size[2]/2])
    before=render(detector,t,info,size,[])
    after=render(detector,t,info,size,[target+[0,.03,0],target-[0,.03,0]])
    result=detector.measure(after,size,target,before)
    assert result['status']=='UNKNOWN' and result['reason']=='AMBIGUOUS'


def test_depth_holes_and_partial_block_cannot_create_good_pose():
    """가려진 절반 블록이나 유효 깊이가 없는 장면에서 유한 중심을 만들지 않는다."""
    detector,t,info,size=fixture();target=np.array([0.,0.,size[2]/2])
    frame=render(detector,t,info,size,[target])
    rgb=frame['rgb'].copy();depth=frame['depth'].copy();rgb[:,:200]=180
    partial=detector.frame(rgb,depth,info,t)
    assert detector.measure(partial,size,target)['status']=='UNKNOWN'
    missing=detector.frame(frame['rgb'],np.full(depth.shape,np.nan),info,t)
    assert detector.measure(missing,size,target)['status']=='UNKNOWN'


def test_wrong_transform_and_mismatched_resolution_are_rejected():
    """좌표 변환 회전/화면 크기 오류를 정상 관측으로 처리하지 않는다."""
    detector,t,info,size=fixture();frame=render(detector,t,info,size,[])
    bad=t.copy();bad[0,0]=2
    with pytest.raises(ValueError):detector.frame(frame['rgb'],frame['depth'],info,bad)
    with pytest.raises(ValueError):detector.frame(frame['rgb'],frame['depth'][:100],info,t)


def test_camera_motion_is_registered_before_scene_difference():
    """카메라가 옮겨진 정적 장면을 새 블록으로 오인하지 않는다."""
    detector,t,info,size=fixture();target=np.array([0.,0.,size[2]/2])
    before=render(detector,t,info,size,[target])
    moved=t.copy();moved[0,3]=.004
    after=render(detector,moved,info,size,[target])
    assert detector.measure(after,size,target,before)['status']=='UNKNOWN'


def test_occluded_corner_is_not_completed_from_known_dimensions():
    """외곽 최대 폭이 맞아도 L자 가림을 알려진 치수로 채우지 않는다."""
    detector,t,info,size=fixture();target=np.array([0.,0.,size[2]/2])
    frame=render(detector,t,info,size,[target]);rgb=frame['rgb'].copy()
    rgb[150:,200:]=180
    masked=detector.frame(rgb,frame['depth'],info,t)
    result=detector.measure(masked,size,target)
    assert result['status']=='UNKNOWN'

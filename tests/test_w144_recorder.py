"""배치후observe추가만검증. 이동/촬영대역은실기검증아님."""
import sys,json
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).parents[1] / 'tools' / 'w144_capture'))
from placement_recorder import PlacementRecorder


def test_eleven_observe_captures(tmp_path):
    """11배치마다observe후촬영하며ZIP한개로저장한다."""
    events=[]
    def move():
        """도착대역."""
        events.append('move');return True
    def capture(p):
        """원자료대역."""
        events.append('capture');p.mkdir();(p/'raw.txt').write_text('fixture')
    r=PlacementRecorder(tmp_path/'run',move,lambda:events.append('stop'),capture)
    for i in range(11):r.after_place(str(i),[.4,0,.01],[0,0,0,1])
    archive=r.finish(True)
    assert events==['move','capture']*11 and archive.is_file()
    assert json.loads((r.folder/'run.json').read_text())['captures']==11


def test_failed_observe_stops_and_keeps_failure(tmp_path):
    """이동실패시촬영하지않고정지콜백뒤실패기록을보존한다."""
    events=[]
    r=PlacementRecorder(tmp_path/'run',lambda:False,lambda:events.append('stop'),lambda p:events.append('capture'))
    with pytest.raises(RuntimeError):r.after_place('a',[0,0,0],[0,0,0,1])
    assert events==['stop']
    assert r.finish(False).is_file() and not r.rows[0]['captured']



def test_capture_requests_exactly_one_pair(tmp_path,monkeypatch):
    """관측1회에RGB·깊이1쌍만요청하여20장반복촬영으로회귀하지않는다."""
    calls=[]
    monkeypatch.setattr('placement_recorder.subprocess.run',lambda args,**kwargs:calls.append(args))
    r=PlacementRecorder(tmp_path/'run',lambda:True,lambda:None)
    r._capture(tmp_path/'capture')
    assert len(calls)==1
    assert calls[0][calls[0].index('--frames')+1]=='1'


def test_attempt_links_grasp_release_and_retry_to_separate_images(tmp_path):
    """같은 부품의 재배치도 별도 시도로 보존해 파지와 사진을 혼동하지 않는다."""
    def capture(path):
        """촬영 대역은 한 쌍의 저장 폴더만 만든다."""
        path.mkdir()
    r=PlacementRecorder(tmp_path/'run',lambda:True,lambda:None,capture)
    target=[.4,0,.01];q=[0,0,0,1]
    source={'grasp':{'face':'long','tcp_center_m':[.1,.2,.3]}}
    first=r.begin_attempt('PART_A',1,target,q,evidence=source)
    source['grasp']['face']='changed'
    r.after_place('PART_A',target,q,action_id=first,evidence={'release':{'success':True}})
    second=r.begin_attempt('PART_A',1,target,q,retry_of=first,evidence={'grasp':{'face':'short'}})
    r.after_place('PART_A',target,q,action_id=second)
    archive=r.finish(True)
    traces=json.loads((r.folder/'attempts.json').read_text())
    assert traces[0]['evidence']['grasp']['face']=='long'
    assert traces[0]['result_evidence']['release']['success'] is True
    assert traces[1]['retry_of']==first
    assert [t['sequence'] for t in traces]==[1,1]
    assert [t['capture_folder'] for t in traces]==['01_observe','02_observe']
    assert [row['action_id'] for row in r.rows]==[first,second]
    assert archive.is_file()


def test_failed_pick_is_preserved_without_photo(tmp_path):
    """잡기 실패는 촬영 없이 시도 기록으로 남겨 다음 시도와 구분한다."""
    events=[]
    r=PlacementRecorder(tmp_path/'run',lambda:events.append('move'),lambda:None,
                        lambda path:events.append('capture'))
    action=r.begin_attempt('PART_A',1,[.4,0,.01],[0,0,0,1])
    r.failed_attempt(action,'gripper_failed',evidence={'width_m':.03})
    assert events==[] and r.rows==[]
    assert r.attempts[0]['state']=='place_failed'
    assert r.attempts[0]['result_evidence']=={'width_m':.03}
    with pytest.raises(ValueError):
        r.after_place('PART_A',[.4,0,.01],[0,0,0,1],action_id=action)


def test_trace_mismatch_rejected_before_observe(tmp_path):
    """잘못된 부품과 사진을 연결하려 하면 관측 이동 전에 거절한다."""
    events=[]
    r=PlacementRecorder(tmp_path/'run',lambda:events.append('move'),lambda:None)
    action=r.begin_attempt('PART_A',1,[.4,0,.01],[0,0,0,1])
    with pytest.raises(ValueError):
        r.after_place('PART_B',[.4,0,.01],[0,0,0,1],action_id=action)
    assert events==[] and r.rows==[] and r.attempts[0]['state']=='started'
    with pytest.raises(ValueError):
        r.begin_attempt('PART_A',1,[.4,0,.01],[0,0,0,1],retry_of=action)


def test_invalid_trace_json_cannot_begin_pick(tmp_path):
    """NaN 증거를 정상 기록으로 만들지 않고 집기 전에 거절한다."""
    r=PlacementRecorder(tmp_path/'run',lambda:True,lambda:None)
    with pytest.raises(ValueError):
        r.begin_attempt('PART_A',1,[.4,0,.01],[0,0,0,1],evidence={'z_m':float('nan')})
    assert r.attempts==[]

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

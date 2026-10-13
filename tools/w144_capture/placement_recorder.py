"""기존 객체인식 조립에 배치 성공 후 observe 촬영·ZIP만 추가한다. 집기/재시도 정책 변경 없음."""
import json
import math
import subprocess
import sys
import time
import zipfile
from pathlib import Path


class PlacementRecorder:
    """기존 실행기의 이동·정지 콜백을 사용해 목표와 원촬영 자료를 저장한다."""

    def __init__(self, folder, move_observe, stop_robot, capture=None):
        """폴더와 콜백 주입. move_observe는 도착 성공 bool, stop_robot는 기존 안전정지 처리다.

        이동 제한시간/Ctrl+C/정지 우선 처리는 기존 실행기 콜백이 보장해야 한다. capture 대역은 시험용이다.
        """
        self.folder=Path(folder)
        self.folder.mkdir(parents=True,exist_ok=False)
        self.move_observe=move_observe
        self.stop_robot=stop_robot
        self.capture=capture or self._capture
        self.rows=[]
        self.attempts=[]

    def begin_attempt(self, part_id, sequence, target_center_m, target_quaternion_xyzw,
                      evidence=None, retry_of=None):
        """집기 전 배치 시도와 실행기 증거를 저장하고 action_id를 반환한다.

        위치 단위 m, 방향 xyzw. evidence는 원 실행기의 파지/명령/보정 정보를 담은 JSON 객체다.
        누락 정보는 추측하지 않는다. 로봇 이동 없음; 형식/파일 실패는 집기 전에 예외를 낸다.
        """
        if not isinstance(part_id,str) or not part_id.strip():
            raise ValueError('part_id 필요')
        if isinstance(sequence,bool) or not isinstance(sequence,int) or sequence < 1:
            raise ValueError('조립 sequence는 양의 정수')
        center=self._pose_values(target_center_m,3)
        quaternion=self._pose_values(target_quaternion_xyzw,4)
        if not math.isclose(sum(x*x for x in quaternion),1.0,abs_tol=1e-3):
            raise ValueError('목표 쿼터니언은 단위 길이여야 한다')
        if evidence is not None and not isinstance(evidence,dict):
            raise ValueError('evidence는 JSON 객체')
        snapshot=json.loads(json.dumps(evidence or {},allow_nan=False))
        if retry_of is not None:
            parent=self._attempt(retry_of)
            if (parent['part_id']!=part_id or parent['sequence']!=sequence
                    or parent['target_center_m']!=center
                    or parent['target_quaternion_xyzw']!=quaternion):
                raise ValueError('재배치는 같은 부품/조립 순서/설계 목표를 사용해야 한다')
            if parent['state']=='started':
                raise ValueError('이전 시도가 끝나기 전에 재배치할 수 없다')
        action_id=f'action_{len(self.attempts)+1:04d}'
        self.attempts.append(dict(action_id=action_id,part_id=part_id,sequence=sequence,
            target_center_m=center,target_quaternion_xyzw=quaternion,retry_of=retry_of,
            started_unix_ns=time.time_ns(),state='started',evidence=snapshot))
        self._save()
        return action_id

    def failed_attempt(self, action_id, reason, evidence=None):
        """놓기 실패 시 촬영 없이 해당 시도를 보존한다. 정지/재시도 실행은 기존 실행기 책임이다.

        입력은 action_id/실패 이유/추가 JSON 증거, 반환 없음. 잘못된 ID/상태/JSON이면 예외.
        """
        row=self._attempt(action_id)
        if row['state']!='started':raise ValueError('이미 끝난 시도')
        if not isinstance(reason,str) or not reason.strip():raise ValueError('실패 이유 필요')
        if evidence is not None and not isinstance(evidence,dict):raise ValueError('evidence는 JSON 객체')
        snapshot=json.loads(json.dumps(evidence or {},allow_nan=False))
        row.update(state='place_failed',reason=reason,finished_unix_ns=time.time_ns(),
                   result_evidence=snapshot)
        self._save()

    def _attempt(self, action_id):
        """이번 실행의 시도를 찾는다. 없으면 ValueError, 외부 영향 없음."""
        for row in self.attempts:
            if row['action_id']==action_id:return row
        raise ValueError('이번 실행에 없는 action_id')

    @staticmethod
    def _pose_values(values, length):
        """위치/방향의 유한한 수를 복사한다. bool/NaN/크기 오류는 ValueError, 외부 영향 없음."""
        if not isinstance(values,(list,tuple)) or len(values)!=length:
            raise ValueError('위치/방향 크기 오류')
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in values):
            raise ValueError('위치/방향은 유한한 수여야 한다')
        return list(values)

    def _capture(self, destination):
        """capture.py로 정지 상태의 RGB·깊이·TF 한 쌍 저장. 수집 실패면 예외, 로봇 이동 없음."""
        subprocess.run([sys.executable,str(Path(__file__).with_name('capture.py')),
                        '--out',str(destination),'--label',destination.name,'--frames','1'],check=True,timeout=40)

    def after_place(self, block_id, target_center_m, target_quaternion_xyzw, *, action_id=None,
                    evidence=None):
        """배치 성공 뒤에만 호출: 목표 저장→observe 이동→촬영. 실패는 정지 콜백 후 다시 예외를 낸다.

        목표는 실제 블록의 설계 배치 중심(m), 쿼터니언xyzw이며 틀기/옮김 그리퍼 목표가 아니다.
        반환은 촬영폴더, 외부 영향은 이동콜백·촬영·파일쓰기. 기존 실행기는 실패 후 다음 집기를 금지한다.
        """
        trace=None
        snapshot=None
        if evidence is not None and not isinstance(evidence,dict):raise ValueError('evidence는 JSON 객체')
        snapshot=json.loads(json.dumps(evidence or {},allow_nan=False))
        if action_id is not None:
            trace=self._attempt(action_id)
            if (trace['state']!='started' or trace['part_id']!=block_id
                    or trace['target_center_m']!=list(target_center_m)
                    or trace['target_quaternion_xyzw']!=list(target_quaternion_xyzw)):
                raise ValueError('촬영을 연결할 배치 시도/목표 불일치')
            trace.update(state='placed',finished_unix_ns=time.time_ns(),result_evidence=snapshot)
        row=dict(sequence=len(self.rows)+1,block_id=block_id,target_center_m=list(target_center_m),
                 target_quaternion_xyzw=list(target_quaternion_xyzw),view='observe',captured=False)
        row.update(action_id=action_id,trace_available=trace is not None)
        self.rows.append(row)
        self._save()
        try:
            if not self.move_observe():raise RuntimeError('observe 도착 실패')
            destination=self.folder/f'{row["sequence"]:02d}_observe'
            self.capture(destination)
            row['captured']=True
            row['capture_folder']=destination.name
            if trace is not None:trace.update(state='observed',capture_folder=destination.name)
            self._save()
            return destination
        except BaseException as exc:
            row['error']=repr(exc)
            if trace is not None:trace.update(state='observation_failed',error=repr(exc))
            try:self.stop_robot()
            except BaseException as stop_error:row['stop_error']=repr(stop_error)
            self._save()
            raise

    def _save(self):
        """블록별 목표와 촬영 성공/실패 기록을 저장한다. 로봇 영향 없음."""
        (self.folder/'placements.json').write_text(json.dumps(self.rows,ensure_ascii=False,indent=2))
        path=self.folder/'attempts.json'
        temporary=path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.attempts,ensure_ascii=False,indent=2,allow_nan=False))
        temporary.replace(path)

    def finish(self, completed):
        """실행 종료 finally에서 성공/중단 여부 기록 후 ZIP 생성. 불완전 자료도 보존한다."""
        self._save()
        (self.folder/'run.json').write_text(json.dumps(dict(completed=bool(completed),placed=len(self.rows),
            captures=sum(r['captured'] for r in self.rows)),indent=2))
        archive=self.folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
            for file in self.folder.rglob('*'):
                if file.is_file():z.write(file,file.relative_to(self.folder))
        return archive

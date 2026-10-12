"""기존 객체인식 조립에 배치 성공 후 observe 촬영·ZIP만 추가한다. 집기/재시도 정책 변경 없음."""
import json
import subprocess
import sys
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

    def _capture(self, destination):
        """capture.py로 정지 상태의 RGB·깊이·TF20쌍 저장. 수집 실패면 예외, 로봇 이동 없음."""
        subprocess.run([sys.executable,str(Path(__file__).with_name('capture.py')),
                        '--out',str(destination),'--label',destination.name],check=True,timeout=40)

    def after_place(self, block_id, target_center_m, target_quaternion_xyzw):
        """배치 성공 뒤에만 호출: 목표 저장→observe 이동→촬영. 실패는 정지 콜백 후 다시 예외를 낸다.

        목표는 실제 블록의 설계 배치 중심(m), 쿼터니언xyzw이며 틀기/옮김 그리퍼 목표가 아니다.
        반환은 촬영폴더, 외부 영향은 이동콜백·촬영·파일쓰기. 기존 실행기는 실패 후 다음 집기를 금지한다.
        """
        row=dict(sequence=len(self.rows)+1,block_id=block_id,target_center_m=list(target_center_m),
                 target_quaternion_xyzw=list(target_quaternion_xyzw),view='observe',captured=False)
        self.rows.append(row)
        self._save()
        try:
            if not self.move_observe():raise RuntimeError('observe 도착 실패')
            destination=self.folder/f'{row["sequence"]:02d}_observe'
            self.capture(destination)
            row['captured']=True
            self._save()
            return destination
        except BaseException as exc:
            row['error']=repr(exc)
            try:self.stop_robot()
            except BaseException as stop_error:row['stop_error']=repr(stop_error)
            self._save()
            raise

    def _save(self):
        """블록별 목표와 촬영 성공/실패 기록을 저장한다. 로봇 영향 없음."""
        (self.folder/'placements.json').write_text(json.dumps(self.rows,ensure_ascii=False,indent=2))

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

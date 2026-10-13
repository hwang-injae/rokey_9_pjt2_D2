"""스캔 중 실시간 점군 미리보기 계산 LiveCloud (ROS 없음, 10/10 E-82 B · W168) — 노드 scan_live_preview 가 쓴다.

손목 카메라가 '지금 보는' 점군을 1 ~ 2 Hz 로 화면에 보여 준다(kind live — 화면이 덮어 그림, 보기 전용). 계산만 여기 두는 이유:
노드 없이 pytest 로 점 자리 · 크기 · 파일 돌려 쓰기 · 언제 낼지를 시험하려고(CLAUDE.md 2장 — 계산 파일은 같은 패키지 다른 파일).
순서: 깊이(mm) → base_link 점(block_checker.depth_to_base_points, 카메라 자세는 노드가 tcp_pose.camera_pose_mm 으로) →
조립 영역 둘레만(structure_scanner.crop_area_m — save_cloud 와 같은 자르기) → 복셀 5 mm 평균 · 500 KB 넘으면 복셀 키움
(structure_scanner.write_voxel_ply — save_cloud 와 같은 방식) → <scan_dir>/<run_id>/live_<k>.ply(LIVE_KEEP 개를 돌려 씀 · 임시 파일에 쓰고
이름 바꾸기 — 다리가 읽는 중에 덮이지 않게) → scan_preview/1.0 dict(IRD 6장 kind live).
단위: 깊이 mm, 점 · 자세 m(base_link), 영역 mm(robot.yaml 을 읽어).
"""
import time
from pathlib import Path

import numpy as np

from d2_vision.block_checker import depth_to_base_points
from d2_vision.structure_scanner import PREVIEW_SCHEMA, crop_area_m, write_voxel_ply

LIVE_VOXEL_M = 0.005          # IRD 6장: live 는 복셀 5 mm · ≤ 500 KB(capture 3 mm · 2 MB 보다 작게 — 1 ~ 2 Hz 로 자주 보냄)
LIVE_MAX_BYTES = 500_000
LIVE_KEEP = 5                 # live_1 ~ 5 를 돌려 쓴다 — 파일이 끝없이 쌓이지 않게, 다리가 앞 파일을 읽는 동안 다음 파일은 다른 이름
LIVE_STRIDE = 2               # 깊이 픽셀 간격(640 × 480 → 약 7만 점) — 5 mm 복셀이면 이 간격으로도 칸이 다 찬다
LIVE_RUN_ID = 'live'          # 작업 관리자 state 에 run_id 가 없을 때(스캔 아닌 때 강제로 켰을 때) 폴더 이름


def live_target(state, only_during_scan=True):
    """지금 실시간 점군을 낼지와 그 run_id. 입력: state = 마지막 /d2/task/state(state/1 dict, 아직 못 받았으면 None).
    only_during_scan(기본) 이면 state 가 'SCAN_' 으로 시작할 때만(SCAN_MOVE · SCAN_CAPTURE · SCAN_INFER · SCAN_REVIEW) — 평소엔 계산하지 않는다.
    출력: (낼지 bool, run_id 글자 — state 의 run_id, 없으면 LIVE_RUN_ID). 바깥 영향 없음."""
    st = (state or {}).get('state') or ''
    rid = (state or {}).get('run_id') or LIVE_RUN_ID
    if only_during_scan and not str(st).startswith('SCAN_'):
        return False, rid
    return True, str(rid)


class LiveCloud:
    """'지금 보이는' 점군 → 파일 · 알림. run 마다 순번(index, 1부터)을 센다. 바깥 영향: write 의 파일 쓰기만.

    입력(생성): cfg = robot.yaml dict(assembly_origin · assembly_area_half_m) · voxel_m · max_bytes · keep(돌려 쓸 파일 수) · stride."""

    def __init__(self, cfg, voxel_m=LIVE_VOXEL_M, max_bytes=LIVE_MAX_BYTES, keep=LIVE_KEEP, stride=LIVE_STRIDE):
        """robot.yaml 조립 원점 · 영역 반폭을 mm 로 읽어 둔다(save_cloud 와 같은 자르기). 키가 없으면 KeyError."""
        o = cfg['assembly_origin']
        self.origin_mm = np.array([o['x_m'], o['y_m'], o['z_m']], float) * 1000.0
        self.half_mm = float(cfg.get('assembly_area_half_m', 0.15)) * 1000.0
        self.voxel_m, self.max_bytes, self.keep, self.stride = float(voxel_m), int(max_bytes), max(1, int(keep)), int(stride)
        self.counts = {}                      # run_id → 지금까지 낸 수

    def points(self, depth_mm, intrinsics, T_base2cam_m):
        """깊이 한 장(mm, 0 = 없음) · 내부값 (fx, fy, ppx, ppy) 또는 dict · 카메라 자세 4×4(m) → 조립 영역 둘레의 base_link 점 (N, 3) m.
        깊이 0.1 ~ 1.0 m 밖은 버린다(block_checker 와 같음). 실패: 모양이 틀리면 ValueError."""
        depth = np.asarray(depth_mm, dtype=np.float64)
        T = np.asarray(T_base2cam_m, dtype=np.float64)
        if depth.ndim != 2 or T.shape != (4, 4):
            raise ValueError('깊이는 (H, W), 자세는 4×4 여야 한다')
        if isinstance(intrinsics, dict):
            intrinsics = tuple(float(intrinsics[k]) for k in ('fx', 'fy', 'ppx', 'ppy'))
        p = depth_to_base_points(np.nan_to_num(depth) / 1000.0, intrinsics, T, stride=self.stride)
        return crop_area_m(p, self.origin_mm, self.half_mm)

    def write(self, out_dir, run_id, pts):
        """점(m) → <out_dir>/live_<k>.ply(k = 순번을 keep 개로 돌림, 임시 파일 뒤 이름 바꾸기). 출력: (index, 경로, 점 수, 쓴 복셀 m).
        index = 이 run 에서 낸 순번(1부터 — 화면은 가장 큰 것으로 덮어 그림). 바깥 영향: 폴더 만들기 · 파일 쓰기. 실패: OSError."""
        index = self.counts.get(run_id, 0) + 1
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        path = out / f'live_{(index - 1) % self.keep + 1}.ply'
        n, voxel = write_voxel_ply(path, pts, self.voxel_m, self.max_bytes, 'd2 live', atomic=True)
        self.counts[run_id] = index
        return index, path, n, voxel

    @staticmethod
    def message(run_id, pose_id, index, path, points, stamp=None):
        """scan_preview/1.0 kind live 알림 dict(IRD 6장 — capture 와 같은 칸). pose_id = 지금 촬영 자세(모르면 '')."""
        return {'schema': PREVIEW_SCHEMA, 'stamp': round(time.time() if stamp is None else float(stamp), 3), 'run_id': str(run_id),
                'pose_id': str(pose_id or ''), 'index': int(index), 'kind': 'live', 'cloud_path': str(path), 'points': int(points)}

    def step(self, state, depth_mm, intrinsics, T_base2cam_m, scan_dir, only_during_scan=True, stamp=None):
        """한 번(노드 타이머 한 틱): 낼 때인지 → 깊이 · 자세가 있는지 → 점 → 파일 → 알림. 출력: (알림 dict 또는 None, 점 (N, 3) m 또는 None, 이유 글자).
        이유: '' = 냄 · 'not_scanning'(스캔 아님 — 계산 안 함) · 'no_depth' · 'no_intrinsics' · 'no_pose'(TF 못 구함 — 그 장은 건너뜀).
        바깥 영향: 낼 때 파일 하나 쓰기. 실패: 파일을 못 쓰면 OSError(노드가 경고만)."""
        go, rid = live_target(state, only_during_scan)
        if not go:
            return None, None, 'not_scanning'
        if depth_mm is None:
            return None, None, 'no_depth'
        if intrinsics is None:
            return None, None, 'no_intrinsics'
        if T_base2cam_m is None:
            return None, None, 'no_pose'
        pts = self.points(depth_mm, intrinsics, T_base2cam_m)
        index, path, n, _ = self.write(Path(scan_dir) / rid, rid, pts)
        return self.message(rid, '', index, path, n, stamp), pts, ''

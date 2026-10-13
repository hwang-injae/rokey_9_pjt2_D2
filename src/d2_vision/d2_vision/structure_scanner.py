"""스캔 추론기 ③ — 여러 자세의 깊이 영상 + YOLO 마스크를 블록 JSON(blocks/2)으로 바꾼다. ROS 없는 계산 (W115, SDD §6.9).

흐름:
  add_capture: 자세마다 깊이 한 장(+ YOLO 마스크) → 추론용 원본(깊이 uint16 mm · 내부 파라미터 · T) + 미리보기 · PLY 용 base_link 점군.
               깊이 여러 장 평균 · YOLO 는 노드(scan_capture) 일이다. 마스크는 attach_masks 로 뒤에 붙여도 된다.
  infer: layer_solver.LayerSolver(층 지도 + 정수계획, 확정 판 v12g — 10/12) 로 블록을 구하고
    ① 바닥 외곽(가장 아래층 블록 외곽)의 가운데 = (0, 0) 으로 옮김 → 좌표 소수 1자리 mm
    ② scan_num(블록 번호): 아래층부터, 같은 층은 위 블록을 많이 받치는 순 — 쌓는 순서가 아니라 번호(10/13 order → scan_num)
    ③ 신뢰도: 점 설명률(O 칸 중 블록 상자 ± 5 mm 안 비율) · 추정 블록 비율 · 촬영 자세 수 → 낮으면 ok False + 이유(SCAN_FAILED)
  save_cloud: 합친 점군(base_link, m)을 복셀 다운샘플해 binary PLY(≤ 2 MB, IRD E-67 cloud_path)로 저장.
  match_rate: 추론 vs 정답 일치율(V-48, E-45).

방향 코드(ori)는 d2_task.recipe_to_parts.ori_extents 와 같은 표다(정답 블록 JSON 이 그 변환기에서 나온다 — 시험이 같은지 확인한다).
d2_task 를 import 하지 않는다(CI 는 d2_vision 만 빌드 · 시험).
단위: 이 파일 안 계산은 mm(설계 좌표계, 블록 JSON 과 같음). 입력 자세 · 출력 PLY 는 m(base_link).
"""
import copy
import json
import math
import os
import time
from pathlib import Path

import numpy as np
from scipy import ndimage

from d2_vision.block_checker import depth_to_base_points
from d2_vision.layer_solver import MIN_POINTS, LayerSolver, ori_extents

SCHEMA = 'blocks/2'                    # 10/13: 블록 번호 칸 order → scan_num(쌓는 순서 order 와 헷갈리지 않게) — 앞자리 +1
FLYING_MM = 8.0           # 깊이 3×3 안에서 이보다 크게 튀는 픽셀(물체 모서리의 허공 점)은 미리보기 · PLY 점군에서 버린다
EXPLAIN_MIN = 0.8         # ③ 점 설명률 기준(실측 4장면 × 5 · 3자세 0.991 ~ 1.000 — 10/12)
INFERRED_MAX = 0.5        # ③ 추정 블록이 전체의 절반을 넘으면 다시 스캔
MIN_POSES = 2             # ③ 촬영 자세가 이보다 적으면 결과는 돌려주되 ok False(실측 s11: 자세 하나로는 일치율 0~0.2)
CLOUD_VOXEL_M = 0.003     # IRD E-67: base_link 다운샘플 복셀 3 mm
CLOUD_MAX_BYTES = 2_000_000
PREVIEW_SCHEMA = 'scan_preview/1.0'   # 스캔 중 점군 미리보기 알림(E-82, IRD 4.1 · 6장)


def _bottom_center(blocks, extent):
    """blocks/2 블록 목록의 바닥 외곽 가운데(x, y, mm) — 가장 낮은 층(최저 z + 7.5 mm 안) 블록들의 외곽 사각형 가운데."""
    zmin = min(b['z'] for b in blocks)
    xs, ys = [], []
    for b in blocks:
        if b['z'] <= zmin + 7.5:
            ex, ey, _ = extent[b['ori']]
            xs += [b['x'] - ex / 2, b['x'] + ex / 2]
            ys += [b['y'] - ey / 2, b['y'] + ey / 2]
    return (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2


def _block_list(blocks):
    """blocks/2 dict 또는 블록 목록 → 블록 목록."""
    return blocks['blocks'] if isinstance(blocks, dict) else list(blocks)


def match_rate(pred_blocks, truth_blocks, block_mm=(75.0, 25.0, 15.0), tol_mm=12.5):
    """추론 vs 정답 일치율(V-48, E-45). 바깥 영향 없음.

    입력: pred_blocks · truth_blocks = blocks/2 dict 또는 블록 목록(mm, 설계 좌표) · block_mm = 설계 블록 크기.
    맞음 = 같은 방향(ori) + 같은 격자 자리 — 두 설계를 각자 '바닥 외곽 가운데'로 옮긴 뒤 x · y 가 25 mm 격자 한 칸 안(|차| < 12.5 mm),
           z(아랫면)가 반 층 안(|차| < 7.5 mm). 한 블록은 한 번만 짝짓는다(가까운 쌍부터).
    출력: (rate, matched, detail) — rate = matched ÷ max(정답 수, 추론 수) · detail = {'pairs': [(정답 번호, 추론 번호, dx, dy, dz)],
          'missed': [맞는 짝이 없는 정답 블록], 'extra': [맞는 짝이 없는 추론 블록]}. 둘 다 비면 (0.0, 0, …).
    """
    pred, truth = _block_list(pred_blocks), _block_list(truth_blocks)
    detail = {'pairs': [], 'missed': list(truth), 'extra': list(pred)}
    if not pred or not truth:
        return 0.0, 0, detail
    ext = ori_extents(block_mm)
    pcx, pcy = _bottom_center(pred, ext)
    tcx, tcy = _bottom_center(truth, ext)
    pairs = []
    for i, t in enumerate(truth):
        for j, p in enumerate(pred):
            if p['ori'] != t['ori']:
                continue
            dx, dy, dz = (p['x'] - pcx) - (t['x'] - tcx), (p['y'] - pcy) - (t['y'] - tcy), p['z'] - t['z']
            if abs(dx) < tol_mm and abs(dy) < tol_mm and abs(dz) < 7.5:
                pairs.append((math.hypot(math.hypot(dx, dy), dz), i, j, dx, dy, dz))
    used_t, used_p = set(), set()
    for _, i, j, dx, dy, dz in sorted(pairs):
        if i in used_t or j in used_p:
            continue
        used_t.add(i)
        used_p.add(j)
        detail['pairs'].append((truth[i].get('scan_num', truth[i].get('order')), pred[j].get('scan_num', pred[j].get('order')), round(dx, 1), round(dy, 1), round(dz, 1)))
    detail['missed'] = [t for i, t in enumerate(truth) if i not in used_t]
    detail['extra'] = [p for j, p in enumerate(pred) if j not in used_p]
    matched = len(used_t)
    return matched / max(len(truth), len(pred)), matched, detail


class StructureScanner:
    """사람이 쌓은 구조물의 깊이 촬영(자세 2~5곳) + YOLO 마스크를 blocks/2 로 추론한다 (로봇 · ROS 없음, SDD §6.9).

    입력: cfg = robot.yaml 을 읽은 dict(assembly_origin · assembly_area_half_m · block_size_m · block_actual_m) ·
          design_id = 결과 설계 이름(노드가 'scan_<run_id>' 등으로 정함) · family = 기본 'unknown' · stride = 미리보기 · PLY 점군 픽셀 간격.
    바깥 영향: 없음(계산 · save_cloud 의 파일 쓰기만).
    한 번의 스캔(run)마다 새로 만들거나 reset() 한다.
    """

    def __init__(self, cfg, design_id='scan', family='unknown', stride=2):
        """robot.yaml 값(조립 원점 · 영역 반폭 · 블록 설계 · 실측 크기)을 mm 로 읽어 추론기를 만든다. 키가 없으면 KeyError."""
        o = cfg['assembly_origin']
        self.origin_mm = np.array([o['x_m'], o['y_m'], o['z_m']], float) * 1000.0
        self.yaw_deg = float(o.get('yaw_deg', 0.0))
        self.half_mm = float(cfg.get('assembly_area_half_m', 0.15)) * 1000.0
        self.block_mm = [v * 1000.0 for v in cfg['block_size_m']]
        actual_mm = [v * 1000.0 for v in cfg.get('block_actual_m', cfg['block_size_m'])]
        self.extent = ori_extents(self.block_mm)
        self.solver = LayerSolver(self.block_mm, actual_mm, self.half_mm)
        self.design_id, self.family, self.stride = design_id, family, int(stride)
        self.cloud_voxel_m = None
        self.reset()

    def reset(self):
        """모은 촬영을 모두 지운다(새 스캔)."""
        self.captures = []

    # ------------------------------------------------------------------ 촬영 모으기
    def add_capture(self, pose_id, depth_mm_u16, intrinsics, T_base2cam_m, masks=None):
        """자세 하나의 깊이 영상(+ YOLO 마스크)을 모아 둔다.

        입력: pose_id = 자세 이름(기록용) · depth_mm_u16 = (H, W) 깊이 mm(0 = 없음, float 평균이어도 됨) · intrinsics = {'fx','fy','ppx','ppy'} 또는
              (fx, fy, ppx, ppy) px · T_base2cam_m = 4×4 카메라 → base_link(m, TF base_link → rg2_tcp × 손목 보정 T_rg2tcp2camera — 10/10 W156) ·
              masks = (선택) 컬러(깊이와 정렬된) 영상의 YOLO 블록 마스크 목록 — (H, W) 크기만 받는다(attach_masks).
        저장: 추론용 depth_u16(반올림 uint16 — save_capture 의 _depth.png 와 같아 저장 파일로 다시 돌려도 같은 결과) · K · T · masks,
              미리보기 · PLY 용 points_m(base_link m, 물체 모서리의 허공 점(3×3 안 차 > 8 mm)과 0.1~1.0 m 밖은 버림) · cam_m.
        출력: 미리보기 점 수(int). 실패: 깊이 · 자세 모양이 틀리면 ValueError.
        """
        depth = np.asarray(depth_mm_u16, dtype=np.float64)
        T = np.asarray(T_base2cam_m, dtype=np.float64)
        if depth.ndim != 2 or T.shape != (4, 4):
            raise ValueError('깊이는 (H, W), 자세는 4×4 여야 한다')
        if isinstance(intrinsics, dict):
            fx, fy, ppx, ppy = (float(intrinsics[k]) for k in ('fx', 'fy', 'ppx', 'ppy'))
        else:
            fx, fy, ppx, ppy = (float(v) for v in intrinsics)
        d = depth / 1000.0
        valid = (d > 0.1) & (d < 1.0)
        dmax = ndimage.maximum_filter(np.where(valid, d, 0.0), size=3)
        dmin = ndimage.minimum_filter(np.where(valid, d, np.inf), size=3)
        valid &= (dmax - dmin) < FLYING_MM / 1000.0
        d = np.where(valid, d, 0.0)
        pts_c = depth_to_base_points(d, (fx, fy, ppx, ppy), np.eye(4), stride=self.stride)   # 카메라 좌표(block_checker 공용)
        R, t = T[:3, :3], T[:3, 3]
        self.captures.append({'pose_id': pose_id, 'points_m': pts_c @ R.T + t, 'cam_m': t.copy(),
                              'depth_u16': np.clip(np.round(np.nan_to_num(depth)), 0, 65535).astype(np.uint16),
                              'K': {'fx': fx, 'fy': fy, 'ppx': ppx, 'ppy': ppy}, 'T': T.copy(), 'masks': []})
        if masks:
            self.attach_masks(pose_id, masks)
        return int(len(pts_c))

    def capture_pose(self, pose_id, depth_mm_u16, intrinsics, T_base2cam_m, masks=None):
        """scan_capture 한 번: 같은 pose_id 를 이미 찍었으면 그 촬영을 빼고 새로 넣는다(같은 자세 다시 찍음 = 덮어쓰기). 입출력 · 실패는 add_capture 와 같다.
        captures 는 새 목록으로 바꿔 끼운다 — 그 전에 snapshot() 으로 떠 둔 목록은 그대로 남는다(미리보기 스레드가 안전하게 읽게)."""
        self.captures = [c for c in self.captures if c['pose_id'] != pose_id]
        return self.add_capture(pose_id, depth_mm_u16, intrinsics, T_base2cam_m, masks)

    def attach_masks(self, pose_id, masks):
        """pose_id 의 가장 최근 촬영에 YOLO 블록 마스크를 붙인다(이미 붙은 것은 바꿈).

        입력: masks = (H, W) 마스크 목록(bool 또는 0/1 — 깊이와 정렬된 컬러 영상 기준). 깊이와 크기가 다른 것은 버린다.
        출력: 붙인 수(int) — 그 자세 촬영이 없으면 0. 바깥 영향: 그 촬영 dict 의 'masks' 를 바꾼다.
        """
        cap = next((c for c in reversed(self.captures) if c['pose_id'] == pose_id), None)
        if cap is None:
            return 0
        hw = cap['depth_u16'].shape
        cap['masks'] = [m for m in (np.asarray(m).astype(bool) for m in masks or []) if m.shape == hw]
        return len(cap['masks'])

    def snapshot(self):
        """지금까지 모은 촬영의 얕은 복사본(같은 설정 · 촬영 목록만 따로). 점 배열은 add_capture 뒤 바뀌지 않아 함께 쓴다.
        save_preview 가 쓴다 — 노드는 촬영 잠금 안에서 떠서 다른 스레드로 넘긴다. 바깥 영향 없음."""
        snap = copy.copy(self)
        snap.captures = list(self.captures)
        return snap

    # ------------------------------------------------------------------ 추론
    def infer(self, bases=None):
        """모은 촬영 → blocks/2 추론(LayerSolver → 바닥 가운데 맞춤 → scan_num → 신뢰도). 바깥 영향 없음.

        입력: bases = (선택) 기본 설계 blocks/2 목록 — 주면 블록 수 · 외곽이 가장 가까운 설계의 family 를 쓴다.
        출력: {'ok', 'reason', 'blocks': blocks/2(mm, scan_num · x · y · z · ori · inferred), 'inferred_count', 'confidence': {...},
               'nearest_base'} — ok False 면 reason 에 SCAN_FAILED 로 넘길 사유(다시 스캔 안내). 실패 검사 순서:
               촬영 없음 → 작업면 위 점 < 200 → 블록 없음(정수계획 해 없음 포함) → 점 설명률 < 0.8 → 추정 > 절반 → 자세 < 2.
        confidence: explained · inferred_ratio · points · center_shift_mm · planes(자세마다 pose_id · align_mm · masks · faces) ·
                    solver · faces · candidates · milp_status · solve_ms · masks_missing(마스크가 없는 자세) · thresholds.
        """
        def fail(reason, conf=None, blocks=None):
            """ok False 응답(blocks 는 화면에 보여 줄 수 있게 그대로 담는다)."""
            body = {'schema': SCHEMA, 'design_id': self.design_id, 'family': self.family, 'blocks': blocks or []}
            return {'ok': False, 'reason': reason, 'blocks': body, 'inferred_count': sum(b['inferred'] for b in body['blocks']),
                    'confidence': conf or {}, 'nearest_base': None}

        if not self.captures:
            return fail('촬영이 없다')
        caps = self.captures
        raw, diag = self.solver.solve([(c['pose_id'], c['depth_u16'], c['K'], c['T']) for c in caps], self.origin_mm, self.yaw_deg,
                                      [c['masks'] for c in caps])
        center = _bottom_center(raw, self.extent) if raw else (0.0, 0.0)
        blocks = self._order([{'x': float(round(b['x'] - center[0], 1)), 'y': float(round(b['y'] - center[1], 1)),
                               'z': float(round(b['z'], 1)), 'ori': b['ori'], 'inferred': bool(b['inferred'])} for b in raw])
        n_inf = sum(1 for b in blocks if b['inferred'])
        planes = [{'pose_id': c['pose_id'], 'align_mm': [round(float(s[0]), 1), round(float(s[1]), 1)], 'masks': len(c['masks']),
                   'faces': int(f)} for c, s, f in zip(caps, diag['shifts'], diag['faces_per_view'])]
        conf = {'explained': round(diag['explained'], 3), 'inferred_ratio': round(n_inf / len(blocks), 3) if blocks else 1.0,
                'points': diag['points'], 'center_shift_mm': [round(float(x), 1) for x in center], 'planes': planes,
                'solver': 'layers_v12g', 'faces': diag['faces'], 'candidates': diag['candidates'], 'milp_status': diag['milp_status'],
                'solve_ms': diag['solve_ms'], 'masks_missing': [c['pose_id'] for c in caps if not c['masks']],
                'thresholds': {'explained_min': EXPLAIN_MIN, 'inferred_max': INFERRED_MAX}}
        if diag['points'] < MIN_POINTS:
            return fail(f'작업면 위 점이 {diag["points"]}개뿐이다(구조물을 못 찾음)', conf)
        nearest = self.nearest_base(blocks, bases) if bases and blocks else None
        family = nearest['family'] if nearest else self.family
        body = {'schema': SCHEMA, 'design_id': self.design_id, 'family': family, 'blocks': blocks}
        explained = diag['explained']
        if not blocks:
            return fail('블록을 하나도 못 맞췄다', conf)
        if explained < EXPLAIN_MIN:
            return fail(f'점 설명률 {explained:.2f} < {EXPLAIN_MIN}(블록으로 설명 못 하는 점이 많다 — 다시 스캔)', conf, blocks)
        if n_inf / len(blocks) > INFERRED_MAX:
            return fail(f'추정 블록 {n_inf}/{len(blocks)} 이 절반을 넘는다(가려진 곳이 많다 — 다시 스캔)', conf, blocks)
        if len(self.captures) < MIN_POSES:
            return fail(f'촬영 자세가 {len(self.captures)}곳뿐이다(SDD §6.9: 2~3곳 — 기울여 본 자세 하나로는 옆면 · 아래층이 틀린다)',
                        conf, blocks)
        return {'ok': True, 'reason': '', 'blocks': body, 'inferred_count': n_inf, 'confidence': conf,
                'nearest_base': nearest['design_id'] if nearest else None}

    # ------------------------------------------------------------------ 블록 사이 계산(mm)
    def _rect(self, b):
        """blocks/2 블록 → 상자 (x0, x1, y0, y1, z0, z1) mm."""
        ex, ey, ez = self.extent[b['ori']]
        return b['x'] - ex / 2, b['x'] + ex / 2, b['y'] - ey / 2, b['y'] + ey / 2, b['z'], b['z'] + ez

    def _order(self, blocks):
        """② scan_num: 아래층부터, 같은 높이는 위 블록을 많이 받치는 순(그다음 x · y). 출력: scan_num 을 단 blocks/2 블록 목록."""
        def carried(b):
            """b 윗면에 바로 얹힌 블록 수."""
            _, _, _, _, _, z1 = self._rect(b)
            x0, x1, y0, y1 = self._rect(b)[:4]
            k = 0
            for c in blocks:
                cx0, cx1, cy0, cy1, cz0, _ = self._rect(c)
                if abs(cz0 - z1) <= 3 and min(x1, cx1) - max(x0, cx0) > 1 and min(y1, cy1) - max(y0, cy0) > 1:
                    k += 1
            return k
        rows = sorted(blocks, key=lambda b: (b['z'], -carried(b), b['x'], b['y']))
        return [{'scan_num': i + 1, 'x': b['x'], 'y': b['y'], 'z': b['z'], 'ori': b['ori'], 'inferred': b['inferred']}
                for i, b in enumerate(rows)]

    def nearest_base(self, blocks, bases):
        """블록 수 · 바닥 외곽 크기가 가장 가까운 기본 설계(blocks/2 dict)를 고른다. bases 가 비면 None."""
        def sig(bl):
            """(블록 수, 외곽 x 폭, y 폭, 높이) mm."""
            rs = [self._rect(b) for b in bl]
            return len(bl), max(r[1] for r in rs) - min(r[0] for r in rs), max(r[3] for r in rs) - min(r[2] for r in rs), \
                max(r[5] for r in rs)
        n0, w0, d0, h0 = sig(blocks)
        best, key = None, None
        for base in bases:
            n1, w1, d1, h1 = sig(_block_list(base))
            k = (abs(n0 - n1), abs(w0 - w1) + abs(d0 - d1) + abs(h0 - h1))
            if key is None or k < key:
                best, key = base, k
        return best

    # ------------------------------------------------------------------ 점군 저장
    def save_cloud(self, path):
        """모은 점군을 base_link 좌표(m) binary PLY 로 저장한다(화면 점군 창 — IRD E-67 cloud_path).

        조립 영역(원점 ± half + 5 cm, 높이 −1 ~ +30 cm) 안 점만, 복셀 3 mm 평균으로 줄인다. 2 MB 를 넘으면 복셀을 1.25 배씩 키운다
        — 쓴 복셀 크기는 self.cloud_voxel_m 에 남는다.
        입력: path = 파일 경로. 출력: 쓴 점 수(int). 바깥 영향: 파일 쓰기. 실패: 촬영이 없으면 0 점 PLY.
        """
        if self.captures:
            p = np.concatenate([c['points_m'] for c in self.captures])
        else:
            p = np.zeros((0, 3))
        n, self.cloud_voxel_m = write_voxel_ply(path, crop_area_m(p, self.origin_mm, self.half_mm), CLOUD_VOXEL_M, CLOUD_MAX_BYTES,
                                                'd2 StructureScanner')
        return n


def crop_area_m(p, origin_mm, half_mm):
    """base_link 점(m) 중 조립 영역(원점 ± half + 5 cm, 높이 −1 ~ +30 cm) 안만 — 케이블 · 로봇 몸통 · 영역 밖 블록을 덜어 낸다.
    save_cloud(cloud.ply · preview) 와 실시간 미리보기(live_cloud)가 같이 쓴다. 입력 origin_mm · half_mm 는 mm. 출력: 걸러진 (N, 3) m."""
    o = np.asarray(origin_mm, dtype=np.float64) / 1000.0
    r = half_mm / 1000.0 + 0.05
    m = (np.abs(p[:, 0] - o[0]) < r) & (np.abs(p[:, 1] - o[1]) < r) & (p[:, 2] > o[2] - 0.01) & (p[:, 2] < o[2] + 0.30)
    return p[m]


def write_voxel_ply(path, p, voxel, max_bytes, comment, atomic=False):
    """점(m)을 복셀 voxel(m) 평균으로 줄여 base_link binary PLY 로 쓴다 — max_bytes 를 넘으면 복셀을 1.25 배씩 키운다.

    save_cloud(복셀 3 mm · ≤ 2 MB) · 실시간 미리보기(복셀 5 mm · ≤ 500 KB)가 같이 쓴다. atomic = 임시 파일(<path>.tmp)에 쓴 뒤 이름 바꾸기 —
    읽는 쪽(다리)이 쓰는 중인 파일을 보지 않게. 출력: (점 수, 쓴 복셀 m). 바깥 영향: 파일 쓰기. 실패: 쓸 수 없으면 OSError."""
    while True:
        if len(p):
            key = np.floor(p / voxel).astype(np.int64)
            _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
            inv = inv.ravel()
            pts = np.stack([np.bincount(inv, weights=p[:, k]) / cnt for k in range(3)], axis=1).astype('<f4')
        else:
            pts = np.zeros((0, 3), '<f4')
        header = ('ply\nformat binary_little_endian 1.0\ncomment {} base_link m voxel {:.4f}\n'
                  'element vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
                  ).format(comment, voxel, len(pts)).encode('ascii')
        if len(header) + pts.nbytes <= max_bytes or len(pts) == 0:
            break
        voxel *= 1.25
    target = Path(path)
    tmp = target.with_name(target.name + '.tmp') if atomic else target
    with open(tmp, 'wb') as f:
        f.write(header)
        f.write(pts.tobytes())
    if atomic:
        os.replace(tmp, target)
    return int(len(pts)), voxel


def read_ply_header(path):
    """PLY 머리말을 읽어 (형식, 점 수, 머리말 길이 bytes)를 돌려준다(시험 · 확인용). 형식이 틀리면 ValueError."""
    with open(path, 'rb') as f:
        data = f.read(4096)
    end = data.find(b'end_header\n')
    if not data.startswith(b'ply\n') or end < 0:
        raise ValueError('PLY 머리말이 아니다')
    fmt, count = None, None
    for line in data[:end].decode('ascii').splitlines():
        if line.startswith('format '):
            fmt = line.split()[1]
        if line.startswith('element vertex '):
            count = int(line.split()[2])
    return fmt, count, end + len(b'end_header\n')


def family_of(design_id):
    """기본 설계 이름 → 가구 family(IRD 2장 `chair` · `desk`). 이름에 CHAIR · DESK 가 없으면 'unknown'(예 001_CHAIR_BENCH_V000 → chair)."""
    name = str(design_id).upper()
    return 'chair' if 'CHAIR' in name else ('desk' if 'DESK' in name else 'unknown')


def json_ready(obj):
    """numpy 수 · 배열이 섞인 dict · list → 파이썬 기본형(JSON 으로 바로 보낼 수 있게). NaN · inf 는 그대로 둔다(보내는 쪽이 allow_nan=False 로 막는다)."""
    if isinstance(obj, dict):
        return {str(k): json_ready(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_ready(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return json_ready(obj.tolist())
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.floating):
        return float(obj)
    return obj


def save_preview(snap, out_dir, run_id, pose_id, index, stamp=None):
    """스캔 중 점군 미리보기(10/10 E-82): 지금까지 모은 점군을 <out_dir>/preview_<index>.ply 로 저장하고 알림 dict 를 돌려준다.

    입력: snap = StructureScanner.snapshot()(scan_capture 때 잠금 안에서 뜬 복사본) · out_dir = <scan_dir>/<run_id> ·
          run_id · pose_id = 이번 촬영 · index = 이번 run 에서 scan_capture 몇 번째(1부터) · stamp = 보낸 시각(s, 비우면 지금).
    출력: scan_preview/1.0 {"schema","stamp","run_id","pose_id","index","kind":"capture","cloud_path","points"}(IRD 6장) —
          PLY 는 cloud.ply 와 같은 save_cloud(base_link m, 복셀 3 mm · ≤ 2 MB), points = PLY 점 수. 화면은 index 가 가장 큰(최신) 파일로 바꿔 그린다 —
          누적이라 쌓으면 앞 자세 점이 겹친다(보기 전용).
    바깥 영향: 폴더 만들기 · 파일 하나 쓰기(snap.cloud_voxel_m 도 바뀌지만 복사본이라 원래 스캐너는 그대로).
    실패: 폴더 · 파일을 못 쓰면 OSError — 부르는 쪽이 경고만 남긴다(scan_capture 답 · scan_infer 와 무관)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f'preview_{int(index)}.ply'
    n = snap.save_cloud(str(path))
    return {'schema': PREVIEW_SCHEMA, 'stamp': round(time.time() if stamp is None else float(stamp), 3), 'run_id': str(run_id),
            'pose_id': str(pose_id), 'index': int(index), 'kind': 'capture', 'cloud_path': str(path), 'points': int(n)}


def save_capture(out_dir, pose_id, depth_mm, color, intrinsics, T_base2cam_m, meta=None):
    """scan_capture 한 자세를 자료로 남긴다(10/11 — W116 · W166 스캔이 저절로 구조물 사진 자료가 되게): capture_scene 과 같은 꼴
    <out_dir>/<pose_id>_color.png · _depth.png · _pose.json. 같은 자세를 다시 찍으면 덮어쓴다.

    입력: out_dir = <scan_dir>/<run_id> · depth_mm = (H, W) 깊이 mm(scan_frames 장 평균, 0 = 없음 — uint16 으로 반올림해 저장) ·
          color = BGR 또는 None(요청 뒤 컬러가 없으면 사진은 건너뜀) · intrinsics = (fx, fy, ppx, ppy) 또는 {'fx','fy','ppx','ppy'} px ·
          T_base2cam_m = 4×4(m) · meta = pose.json 에 더할 칸(run_id · index · tcp 등).
    pose.json 칸은 capture_scene 과 같은 이름(intrinsics_px · depth_unit 'mm' · color_size · T_base2cam_m(m) · tcp …)이라
    실측 시험(test_structure_scanner — add_capture(depth, intrinsics_px, T_base2cam_m))이 그대로 읽는다.
    출력: 쓴 파일 경로 목록. 바깥 영향: 폴더 만들기 · 파일 쓰기. 실패: cv2 가 없으면 ImportError, 폴더 · 파일을 못 쓰면 OSError — 노드는 경고만."""
    import cv2                                        # 노드(로봇 PC)에서만 부른다 — CI(cv2 없음)에서도 이 파일을 import 할 수 있게 여기서
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if isinstance(intrinsics, dict):
        intr = {k: float(intrinsics[k]) for k in ('fx', 'fy', 'ppx', 'ppy')}
    else:
        intr = dict(zip(('fx', 'fy', 'ppx', 'ppy'), (float(v) for v in intrinsics)))
    depth = np.asarray(depth_mm, dtype=np.float64)
    intr['width'], intr['height'] = int(depth.shape[1]), int(depth.shape[0])
    written = []
    path = out / f'{pose_id}_depth.png'
    if not cv2.imwrite(str(path), np.clip(np.round(np.nan_to_num(depth)), 0, 65535).astype(np.uint16)):
        raise OSError(f'깊이 PNG 를 못 씀: {path}')
    written.append(path)
    if color is not None:
        path = out / f'{pose_id}_color.png'
        if not cv2.imwrite(str(path), color):
            raise OSError(f'컬러 PNG 를 못 씀: {path}')
        written.append(path)
    pose = {'pose_id': str(pose_id), 'stamp': round(time.time(), 3), 'intrinsics_px': intr, 'depth_unit': 'mm',
            'color_size': None if color is None else [int(color.shape[1]), int(color.shape[0])],
            'T_base2cam_m': np.round(np.asarray(T_base2cam_m, dtype=np.float64), 6).tolist(), 'source': 'wrist_block scan_capture'}
    pose.update(meta or {})
    path = out / f'{pose_id}_pose.json'
    path.write_text(json.dumps(pose, ensure_ascii=False, indent=1), encoding='utf-8')
    written.append(path)
    return written


def scan_response(result, image_path, cloud_path):
    """infer() 결과 → scan_infer 응답 (success, reason, 응답 dict) — 노드가 JsonQuery 응답 세 칸에 그대로 넣는다(IRD 4.2).

    입력: result = StructureScanner.infer() 출력 · image_path = 컬러 사진 경로(없으면 '') · cloud_path = PLY 경로(저장 실패면 None).
    출력: ok → (True, '', {"ok":true,"blocks":blocks/2,"inferred_count","image_path"[,"cloud_path"][,"nearest_base"]}),
          신뢰도 미달 → (True, '', {"ok":false,"reason":"SCAN_FAILED","detail":이유}) — 답은 냈지만 결과가 나쁨(mock_scan 과 같은 두 층).
    cloud_path 가 None 이면 그 칸을 뺀다(PLY 저장 실패 — IRD 4.2 선택 칸, 10/8 PL: 없으면 화면 점군 창만 안 뜬다). 블록 좌표는 소수 1자리 mm 로 반올림.
    nearest_base(IRD 4.2 선택 칸, E-79)는 추론기가 기본 설계를 골랐을 때만 넣는다 — 못 골랐으면(None · 빈 글자) 뺀다. 바깥 영향 없음."""
    if not result.get('ok'):
        return True, '', {'ok': False, 'reason': 'SCAN_FAILED', 'detail': str(result.get('reason', ''))}
    blocks = json_ready(result['blocks'])
    for b in blocks['blocks']:
        for k in ('x', 'y', 'z'):
            b[k] = round(float(b[k]), 1)
        b['scan_num'], b['inferred'] = int(b['scan_num']), bool(b.get('inferred', False))
    body = {'ok': True, 'blocks': blocks, 'inferred_count': sum(1 for b in blocks['blocks'] if b['inferred']),
            'image_path': str(image_path or '')}
    if cloud_path is not None:
        body['cloud_path'] = str(cloud_path)
    nearest = result.get('nearest_base')
    if nearest:                                   # infer() 는 계산해 두는데 응답에서 빠져 화면 · AI 힌트까지 안 가던 것(10/13 W177)
        body['nearest_base'] = str(nearest)
    return True, '', body


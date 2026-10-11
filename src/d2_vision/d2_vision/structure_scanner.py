"""스캔 추론기 ③ — 여러 자세의 깊이 영상(점군)을 블록 JSON(blocks/1)으로 바꾼다. ROS 없는 계산 (W115, SDD §6.9).

흐름(SDD §6.9 ①~⑧):
  add_capture: 자세마다 깊이 한 장 → base_link 점군 + 카메라 위치를 모아 둠. 깊이 여러 장 평균은 노드(scan_capture) 일이다.
  infer:
    ① 점군 → 설계 좌표(mm, 조립 원점 기준) · 자세마다 작업면 평면을 다시 맞춰 기울기 · 높이 어긋남을 지움 → 작업면 위 점만
       (2 mm 와 '작업면 잡음 × 3 + 4 mm' 중 큰 값 위) · 자세 사이 수평 어긋남은 윗면 자국을 겹쳐 맞춤 · 가장 큰 덩어리만
    ② 높이 지도(5 mm 셀, 최고 z — block_checker.height_map 공용) · 2.5 mm 복셀 점유(점이 있음) · 빈 곳(카메라 빛이 지나감)
    ③ 층: 실측 두께(robot.yaml block_actual_m)로 z 를 설계 두께(15 mm)에 맞춰 늘림 → z 는 5 mm 격자(15 · 25 · 75 mm 블록 모두)
    ④ 바닥 외곽(가장 아래층 점의 가장 바깥)의 가운데 = (0, 0) → 2.5 mm 복셀 = 블록 모서리(12.5 mm 의 배수)가 격자 위
    ⑤ 블록 추출: 얇은 층마다 '보이는 면'으로 외곽(x 폭 · y 폭)을 구해 입체 S 를 만들고, S 를 75 × 25 × 15 상자(방향 6가지)로
       꽉 맞게 쪼갠다(채움 + 경계에 딱 맞음 점수, 같으면 눕힘 우선). 75 × 75 판(3개)은 받침 · 엇갈림(젠가식)으로 방향을 다시 고름
    ⑥ 가려진 블록: 받침이 없는 블록 바로 아래에 같은 자리 블록을 넣음(inferred: true), 바닥까지 반복(벽 구조)
    ⑦ order: 아래층부터, 같은 층은 위 블록을 많이 받치는 순
    ⑧ 신뢰도: 점 설명률(블록 표면 가까이 있는 점의 비율) · 추정 블록 비율 · 촬영 자세 수 → 낮으면 ok False + 이유(SCAN_FAILED)
  save_cloud: 합친 점군(base_link, m)을 복셀 다운샘플해 binary PLY(≤ 2 MB, IRD E-67 cloud_path)로 저장.
  match_rate: 추론 vs 정답 일치율(V-48, E-45).

방향 코드(ori)는 d2_task.recipe_to_blocks.ori_extents 와 같은 표다(정답 블록 JSON 이 그 변환기에서 나온다 — 시험이 같은지 확인한다).
d2_task 를 import 하지 않는다(CI 는 d2_vision 만 빌드 · 시험).
단위: 이 파일 안 계산은 mm(설계 좌표계, 블록 JSON 과 같음). 입력 자세 · 출력 PLY 는 m(base_link).
"""
import math

import numpy as np
from scipy import ndimage

from d2_vision.block_checker import depth_to_base_points, height_map

SCHEMA = 'blocks/1'
ORIS = ('x', 'y', 'xe', 'ye', 'zx', 'zy')
VOX = 2.5                 # mm — 블록 모서리(±37.5 · ±12.5 · ±7.5)가 복셀 경계에 오도록 2.5 mm
Z_STEP = 2                # 블록 아랫면 후보 = 2 복셀(5 mm)마다 — 15 · 25 · 75 mm 를 모두 담는 가장 큰 격자
TABLE_CLEAR_MM = 2.0      # SDD ① 작업면 위 2 mm 이상만 구조물로 본다
HEIGHT_MAX_MM = 200.0     # 구조물 높이 상한(촬영 자세 W134 '225 × 225 × 150 mm 다 보임' + 여유). 위의 점(케이블 등)은 버린다
PLANE_BAND_MM = 10.0      # 자세마다 작업면 평면을 맞출 때 쓰는 점: 원점 높이 ± 10 mm(손목 보정 기울기 수 mm 를 담는 폭)
FLYING_MM = 8.0           # 깊이 3×3 안에서 이보다 크게 튀는 픽셀(물체 모서리의 허공 점)은 버린다
CARVE_STOP_MM = 6.0       # 빈 곳 칠하기: 빛이 닿은 점 6 mm 앞에서 멈춘다(깊이 잡음 · 자세 사이 어긋남이 표면을 깎지 않게)
CARVE_EVERY = 1           # 빈 곳 칠하기에 쓰는 빛 = 점마다(stride 2 → 작업대에서 약 1.7 mm 간격, 2.5 mm 복셀이 빠짐없이 칠해짐)
FILL_MIN = 0.8            # 블록 바닥 모양의 80 % 이상이 앞면 칸이어야 후보(실물 틈 · 자세 어긋남 몇 mm 를 견딘다)
VERT_MIN = 0.7            # 블록 높이만큼 S 가 70 % 이상 있어야 후보(빈 곳 칠하기가 가장자리를 조금 깎아도 견딤)
FILL_FULL = 0.9           # 채움이 90 % 이상이면 다 찬 것으로 본다 — S 가장자리 한 칸(2.5 mm = 블록 폭의 10 %)이 빠져도 같은 점수
BOUNDARY_W = 0.5          # 점수 = 채움 + 0.5 × (4옆 중 앞면 경계에 붙은 비율 평균) + 방향 우선값
ORI_PRIOR = {'x': 0.03, 'y': 0.03, 'xe': 0.015, 'ye': 0.015, 'zx': 0.0, 'zy': 0.0}   # 같으면 눕힘 → 옆세움 → 세움(SDD ③ 15 mm 층 먼저). 한 옆 차이(0.125)보다 작게
SQUARE_TOL_MM = 6.0       # 75 × 75 판 찾기: 세 블록이 폭(25 mm) 간격에서 이만큼 어긋나도 한 판으로 본다(S 가 한쪽으로 5 mm 넓게 나올 때)
LATTICE_MM = 12.5         # 블록 옆면이 놓이기 쉬운 자리(바닥 외곽 가운데 기준 블록 폭의 반 — 25 mm 격자 칸 경계 · 가운데)
LATTICE_PRIOR = 0.004     # 옆면이 이 격자 위면 면마다 더함 — S 경계가 한 칸(2.5 mm) 흔들릴 때만 가름(한 옆 0.125 보다 훨씬 작게)
STACK_PRIOR = 0.2         # 바로 아래 블록과 같은 바닥 자리면 더함(벽 · 기둥은 같은 자리에 쌓인다) — 경계 한 옆(0.125)보다 크게
SLACK_MM = 10.0           # 놓은 블록 둘레 이만큼 안의 못 덮은 앞면 칸은 그 블록 몫으로 본다(실측 S 가 5~10 mm 넓게 나옴)
NARROW_COMP = 1.5         # 옆세움 · 세움은 남은 앞면 덩어리가 그 바닥 넓이의 1.5 배 이하일 때만
GAP_SNAP = 4              # 복셀(10 mm) — S 아랫부분이 이만큼 빠져도 받침 위에 놓는다(가려짐 · 빈 곳 칠하기가 아랫단을 깎음)
MAX_BLOCKS = 80
FREE_MAX_INFERRED = 0.2   # 추정 블록 상자 안이 '빈 곳으로 보인' 비율이 이보다 크면 그 자리에 넣지 않는다
EXPLAIN_MIN = 0.8         # ⑧ 점 설명률 기준(시험 · 실측으로 고른 값 — 보고서 참고)
INFERRED_MAX = 0.5        # ⑧ 추정 블록이 전체의 절반을 넘으면 다시 스캔
MIN_POSES = 2             # ⑧ 촬영 자세가 이보다 적으면 결과는 돌려주되 ok False(실측 s11: 자세 하나로는 일치율 0~0.2)
CLOUD_VOXEL_M = 0.003     # IRD E-67: base_link 다운샘플 복셀 3 mm
CLOUD_MAX_BYTES = 2_000_000


def ori_extents(block_mm):
    """블록 크기(길이 · 폭 · 두께, mm) → 방향 코드마다 (x · y · z 방향 길이). d2_task.recipe_to_blocks.ori_extents 와 같은 표."""
    L, W, T = block_mm
    return {'x': (L, W, T), 'y': (W, L, T), 'xe': (L, T, W), 'ye': (T, L, W), 'zx': (T, W, L), 'zy': (W, T, L)}


def _sat(vol):
    """3D 누적합(앞에 0 한 줄씩) — 상자 합을 8번 읽기로 구한다."""
    s = np.zeros(tuple(n + 1 for n in vol.shape), np.int32)
    s[1:, 1:, 1:] = vol.astype(np.int32).cumsum(0).cumsum(1).cumsum(2)
    return s


def _box(s, x0, y0, z0, x1, y1, z1):
    """누적합 s 에서 [x0, x1) × [y0, y1) × [z0, z1) 상자의 합(배열 입력 가능)."""
    return (s[x1, y1, z1] - s[x0, y1, z1] - s[x1, y0, z1] - s[x1, y1, z0]
            + s[x0, y0, z1] + s[x0, y1, z0] + s[x1, y0, z0] - s[x0, y0, z0])


def _bottom_center(blocks, extent):
    """blocks/1 블록 목록의 바닥 외곽 가운데(x, y, mm) — 가장 낮은 층(최저 z + 7.5 mm 안) 블록들의 외곽 사각형 가운데."""
    zmin = min(b['z'] for b in blocks)
    xs, ys = [], []
    for b in blocks:
        if b['z'] <= zmin + 7.5:
            ex, ey, _ = extent[b['ori']]
            xs += [b['x'] - ex / 2, b['x'] + ex / 2]
            ys += [b['y'] - ey / 2, b['y'] + ey / 2]
    return (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2


def _block_list(blocks):
    """blocks/1 dict 또는 블록 목록 → 블록 목록."""
    return blocks['blocks'] if isinstance(blocks, dict) else list(blocks)


def match_rate(pred_blocks, truth_blocks, block_mm=(75.0, 25.0, 15.0), tol_mm=12.5):
    """추론 vs 정답 일치율(V-48, E-45). 바깥 영향 없음.

    입력: pred_blocks · truth_blocks = blocks/1 dict 또는 블록 목록(mm, 설계 좌표) · block_mm = 설계 블록 크기.
    맞음 = 같은 방향(ori) + 같은 격자 자리 — 두 설계를 각자 '바닥 외곽 가운데'로 옮긴 뒤 x · y 가 25 mm 격자 한 칸 안(|차| < 12.5 mm),
           z(아랫면)가 반 층 안(|차| < 7.5 mm). 한 블록은 한 번만 짝짓는다(가까운 쌍부터).
    출력: (rate, matched, detail) — rate = matched ÷ max(정답 수, 추론 수) · detail = {'pairs': [(정답 order, 추론 order, dx, dy, dz)],
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
        detail['pairs'].append((truth[i].get('order'), pred[j].get('order'), round(dx, 1), round(dy, 1), round(dz, 1)))
    detail['missed'] = [t for i, t in enumerate(truth) if i not in used_t]
    detail['extra'] = [p for j, p in enumerate(pred) if j not in used_p]
    matched = len(used_t)
    return matched / max(len(truth), len(pred)), matched, detail


class StructureScanner:
    """사람이 쌓은 구조물의 깊이 촬영(자세 2~3곳)을 blocks/1 로 추론한다 (로봇 · ROS 없음, SDD §6.9).

    입력: cfg = robot.yaml 을 읽은 dict(assembly_origin · assembly_area_half_m · block_size_m · block_actual_m) ·
          design_id = 결과 설계 이름(노드가 'scan_<run_id>' 등으로 정함) · family = 기본 'unknown' · stride = 깊이 픽셀 간격.
    바깥 영향: 없음(계산 · save_cloud 의 파일 쓰기만).
    한 번의 스캔(run)마다 새로 만들거나 reset() 한다.
    """

    def __init__(self, cfg, design_id='scan', family='unknown', stride=2):
        """robot.yaml 값(조립 원점 · 영역 반폭 · 블록 설계 · 실측 크기)을 mm 로 읽어 둔다. 키가 없으면 KeyError."""
        o = cfg['assembly_origin']
        self.origin_mm = np.array([o['x_m'], o['y_m'], o['z_m']], float) * 1000.0
        self.yaw = math.radians(o.get('yaw_deg', 0.0))
        self.half_mm = float(cfg.get('assembly_area_half_m', 0.15)) * 1000.0
        self.block_mm = [v * 1000.0 for v in cfg['block_size_m']]
        actual = cfg.get('block_actual_m', cfg['block_size_m'])
        self.z_scale = self.block_mm[2] / (actual[2] * 1000.0)   # 실측 두께 14.8 → 설계 15 mm 층에 맞춤(SDD ③ 14.7 실측 규칙)
        self.extent = ori_extents(self.block_mm)
        self.design_id, self.family, self.stride = design_id, family, int(stride)
        self.cloud_voxel_m = None
        self.reset()

    def reset(self):
        """모은 촬영을 모두 지운다(새 스캔)."""
        self.captures = []

    # ------------------------------------------------------------------ 촬영 모으기
    def add_capture(self, pose_id, depth_mm_u16, intrinsics, T_base2cam_m):
        """자세 하나의 깊이 영상을 base_link 점군으로 바꿔 카메라 위치와 함께 모아 둔다.

        입력: pose_id = 자세 이름(기록용) · depth_mm_u16 = (H, W) 깊이 mm(0 = 없음) · intrinsics = {'fx','fy','ppx','ppy'} 또는
              (fx, fy, ppx, ppy) px · T_base2cam_m = 4×4 카메라 → base_link(m, TF base_link → rg2_tcp × 손목 보정 T_rg2tcp2camera — 10/10 W156).
        출력: 모은 점 수(int). 물체 모서리의 허공 점(깊이 3×3 안 차 > 8 mm)과 0.1~1.0 m 밖은 버린다.
        (base 변환을 따로 하는 이유: 카메라 위치가 빈 곳 칠하기에 필요해서 카메라 좌표 점 → T 로 직접 옮긴다.)
        실패: 깊이 · 자세 모양이 틀리면 ValueError.
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
        self.captures.append({'pose_id': pose_id, 'points_m': pts_c @ R.T + t, 'cam_m': t.copy()})
        return int(len(pts_c))

    # ------------------------------------------------------------------ 좌표 · 평면
    def _to_design(self, p_m):
        """base_link 점(m) → 설계 좌표(mm, 조립 원점 기준, yaw 를 되돌림). z 는 원점(작업면) 위 높이."""
        q = np.atleast_2d(p_m) * 1000.0 - self.origin_mm
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        return np.stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1], q[:, 2]], axis=1)

    def _fit_table(self, q):
        """작업면 평면 z = a·x + b·y + c 를 맞춘다(자세마다 — 손목 보정 기울기 · 높이 어긋남을 지운다, SDD §6.9 '주의' ②).

        입력: 설계 좌표 점 q (N, 3) mm. 원점 ± half · 높이 ± 10 mm 점으로 최소제곱, 3 mm 넘게 벗어난 점을 빼며 3번.
        출력: (a, b, c, rms_mm, 쓴 점 수). 점이 300개보다 적거나 기울기가 0.05(약 3°)를 넘으면 (0, 0, 0, nan, n) — 고치지 않는다.
        """
        h = self.half_mm
        m = (np.abs(q[:, 0]) < h) & (np.abs(q[:, 1]) < h) & (np.abs(q[:, 2]) < PLANE_BAND_MM)
        p = q[m]
        coef, rms = np.zeros(3), float('nan')
        for _ in range(3):
            if len(p) < 300:
                return 0.0, 0.0, 0.0, float('nan'), int(len(p))
            A = np.c_[p[:, 0], p[:, 1], np.ones(len(p))]
            coef = np.linalg.lstsq(A, p[:, 2], rcond=None)[0]
            r = p[:, 2] - A @ coef
            rms = float(np.sqrt(np.mean(r ** 2)))
            p = p[np.abs(r) < 3.0]
        if abs(coef[0]) > 0.05 or abs(coef[1]) > 0.05:
            return 0.0, 0.0, 0.0, float('nan'), int(len(p))
        return float(coef[0]), float(coef[1]), float(coef[2]), rms, int(len(p))

    def _level(self, p, a, b, c):
        """작업면 평면(a, b, c)을 빼 높이를 작업면 위로 바꾸고, 실측 두께 → 설계 두께 비율로 늘린다(SDD ③). 입력 · 출력 mm."""
        return np.c_[p[:, 0], p[:, 1], (p[:, 2] - (a * p[:, 0] + b * p[:, 1] + c)) * self.z_scale]

    # ------------------------------------------------------------------ 추론
    def infer(self, bases=None):
        """모은 촬영 → blocks/1 추론(SDD §6.9 ①~⑧). 바깥 영향 없음.

        입력: bases = (선택) 기본 설계 blocks/1 목록 — 주면 블록 수 · 외곽이 가장 가까운 설계의 family 를 쓴다.
        출력: {'ok', 'reason', 'blocks': blocks/1(mm, order · x · y · z · ori · inferred), 'inferred_count', 'confidence': {...},
               'nearest_base'} — ok False 면 reason 에 SCAN_FAILED 로 넘길 사유(다시 스캔 안내).
        """
        def fail(reason, conf=None, blocks=None):
            """ok False 응답(blocks 는 화면에 보여 줄 수 있게 그대로 담는다)."""
            body = {'schema': SCHEMA, 'design_id': self.design_id, 'family': self.family, 'blocks': blocks or []}
            return {'ok': False, 'reason': reason, 'blocks': body, 'inferred_count': sum(b['inferred'] for b in body['blocks']),
                    'confidence': conf or {}, 'nearest_base': None}

        if not self.captures:
            return fail('촬영이 없다')
        views, planes = [], []
        for cap in self.captures:                                                     # ① 설계 좌표 + 작업면 맞춤
            q = self._to_design(cap['points_m'])
            cam = self._to_design(cap['cam_m'][None, :])[0]
            a, b, c, rms, npl = self._fit_table(q)
            q = self._level(q, a, b, c)
            cam = self._level(cam[None, :], a, b, c)[0]
            # 작업면 위 기준 = 2 mm(SDD ①)와 '작업면 잡음 3배 + 4 mm' 중 큰 값 — 실측 작업면 잡음(rms 1.2~1.4 mm, 그림자 쪽은 더 큼)이
            # 3~8 mm 점을 만들어 블록 사이 빈 곳을 메운다. 그 아래 맨 아래층은 바로 위층 폭으로 채운다(_hull).
            clear = max(TABLE_CLEAR_MM, 3.0 * rms + 4.0) if rms == rms else TABLE_CLEAR_MM
            H = self.half_mm
            m = (q[:, 2] > clear) & (q[:, 2] < HEIGHT_MAX_MM) & (np.abs(q[:, 0]) < H) & (np.abs(q[:, 1]) < H)
            sq_v = q[m]
            main = sq_v[self._largest_component(sq_v)] if len(sq_v) else sq_v           # 자세 맞춤용(이 자세에서 가장 큰 덩어리)
            views.append({'q': q, 'cam': cam, 's': main, 'all': sq_v})
            planes.append({'pose_id': cap['pose_id'], 'tilt_deg': round(math.degrees(math.atan(math.hypot(a, b))), 2),
                           'offset_mm': round(c, 2), 'rms_mm': round(rms, 2) if rms == rms else None, 'points': npl,
                           'clear_mm': round(clear, 2), 'struct_points': len(main)})

        H = self.half_mm
        shifts = self._align_views(views)                                             # 자세 사이 수평 어긋남(손목 보정 x · y 오차) 맞춤
        for pl, sh in zip(planes, shifts):
            pl['align_mm'] = [round(float(sh[0]), 1), round(float(sh[1]), 1)]
        sq = np.concatenate([v['all'] for v in views])
        if len(sq) < 200:
            return fail(f'작업면 위 점이 {len(sq)}개뿐이다(구조물을 못 찾음)')
        sq = sq[self._largest_component(sq)]                                          # 모든 자세를 합친 뒤 덩어리 하나(케이블 · 다른 블록 · 잡음 빼기)
        # (자세 하나만 보면 가려진 틈 때문에 한 구조물이 두 덩어리로 나뉠 수 있어 합친 뒤 고른다)
        sq = sq[self._trim_low(sq)]
        bot = sq[sq[:, 2] < self.block_mm[2]]                                         # ④ 바닥 외곽 가운데 = (0, 0)
        if len(bot) < 50:
            bot = sq
        lo, hi = np.percentile(bot[:, :2], 0.5, axis=0), np.percentile(bot[:, :2], 99.5, axis=0)
        narrow = hi - lo < self.block_mm[2]                                         # 맨 아래층이 옆면 한 장으로만 보인 쪽(폭을 모름)은
        lo = np.where(narrow, np.percentile(sq[:, :2], 0.5, axis=0), lo)            # 구조물 전체 외곽으로(앞 · 옆 자세 중 하나가 빠진 경우)
        hi = np.where(narrow, np.percentile(sq[:, :2], 99.5, axis=0), hi)
        center = (lo + hi) / 2
        shift = np.array([center[0], center[1], 0.0])
        sq = sq - shift
        for v in views:
            v['q'] = v['q'] - shift
            v['cam'] = v['cam'] - shift
            v['s'] = v['s'] - shift
            v['all'] = v['all'] - shift

        n = int(round(2 * H / VOX))
        nz = int(round(HEIGHT_MAX_MM / VOX))
        occ = self._voxel_count(sq, n, nz) > 0                                        # ② 점유 복셀
        free_cnt = np.zeros((n, n, nz), np.int32)
        box = (np.r_[sq[:, :2].min(axis=0) - 20.0, 0.0], np.r_[sq[:, :2].max(axis=0) + 20.0, sq[:, 2].max() + 10.0])
        for v in views:
            free_cnt += self._carve(v, n, nz, box)
        # 빈 곳 = 빛이 지나갔고 점은 없는 곳. 표면 쪽 한 칸은 깎는다 — 블록 실측(74.45 mm) · 자세 어긋남으로 표면 복셀을 비우지 않게
        free = ndimage.binary_erosion((free_cnt >= 1) & ~occ, border_value=1)
        hm, _, _ = height_map(sq / 1000.0, (0.0, 0.0), H / 1000.0, cell_m=2 * VOX / 1000.0)   # SDD ② 5 mm 높이 지도(최고 z)
        top = np.repeat(np.repeat(hm * 1000.0, 2, axis=0), 2, axis=1)                  # 5 mm → 2.5 mm 복셀에 맞춤

        zcut = max(pl['clear_mm'] for pl in planes)
        S = self._hull(sq, free, top, n, nz, zcut)                                  # ⑤ 입체 S
        raw = self._tile(S)
        blocks = self._fix_square_layers([self._to_block(r, H) for r in raw])
        blocks, grid_err = self._snap(self._align_columns(blocks))                     # ④ 기둥 맞춤 · 격자 맞춤
        blocks = self._infer_hidden(blocks, free, n, nz, H)                           # ⑥
        blocks = self._order(blocks)                                                  # ⑦

        n_inf = sum(1 for b in blocks if b['inferred'])                               # ⑧ 신뢰도
        explained = self._explained(occ, blocks, n, nz, H)
        covered = self._covered(S, blocks, n, nz, H)
        conf = {'explained': round(explained, 3), 'hull_covered': round(covered, 3),
                'inferred_ratio': round(n_inf / len(blocks), 3) if blocks else 1.0,
                'grid_err_mm': round(grid_err, 2), 'points': int(len(sq)), 'center_shift_mm': [round(float(x), 1) for x in center],
                'planes': planes, 'thresholds': {'explained_min': EXPLAIN_MIN, 'inferred_max': INFERRED_MAX}}
        nearest = self.nearest_base(blocks, bases) if bases and blocks else None
        family = nearest['family'] if nearest else self.family
        body = {'schema': SCHEMA, 'design_id': self.design_id, 'family': family, 'blocks': blocks}
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

    def _align_views(self, views):
        """자세마다 구조물 윗면(가장 높은 층) 자국을 기준 자세(가장 위에서 내려다본 것)에 맞춰 x · y 로 옮긴다.

        손목 보정의 수평 오차(약 1 cm, SDD §6.9 '주의' ③)는 손목 방향을 따라 돌아서 자세마다 점군이 다른 쪽으로 밀린다.
        윗면은 세 자세 모두 보이므로 2.5 mm 격자 자국의 겹침(IoU)이 가장 큰 이동(± 20 mm)을 고른다. 겹침이 0.3 보다 작으면 옮기지 않는다.
        입력: views(각 'q' · 'cam' · 's'(가장 큰 덩어리) · 'all' — 이 함수가 제자리에서 고친다). 출력: 자세마다 쓴 이동 [(dx, dy)] mm.
        """
        shifts = [(0.0, 0.0)] * len(views)
        cand = [i for i, v in enumerate(views) if len(v['s']) > 50]
        if len(cand) < 2:
            return shifts

        def steep(v):
            """카메라가 구조물을 내려다보는 정도(수직에 가까울수록 1)."""
            c = v['s'].mean(axis=0)
            d = v['cam'] - c
            return d[2] / (np.linalg.norm(d) + 1e-9)
        ref = max(cand, key=lambda i: steep(views[i]))
        zt = np.percentile(views[ref]['s'][:, 2], 98)
        n = int(round(2 * self.half_mm / VOX))
        R = int(round(20.0 / VOX))

        def mask(p):
            """윗면 높이 ± 8 mm 점의 2.5 mm 격자 자국(작은 구멍은 메움)."""
            t = p[(p[:, 2] > zt - 8.0) & (p[:, 2] < zt + 8.0)]
            m = np.zeros((n, n), bool)
            ix = np.clip(np.floor((t[:, 0] + self.half_mm) / VOX).astype(int), 0, n - 1)
            iy = np.clip(np.floor((t[:, 1] + self.half_mm) / VOX).astype(int), 0, n - 1)
            m[ix, iy] = True
            return ndimage.binary_closing(m, iterations=2)
        mref = mask(views[ref]['s'])
        if mref.sum() < 20:
            return shifts
        for i in cand:
            if i == ref:
                continue
            mi = mask(views[i]['s'])
            if mi.sum() < 20:
                continue
            best, bi = 0.0, (0, 0)
            for dx in range(-R, R + 1):
                for dy in range(-R, R + 1):
                    sh = np.roll(np.roll(mi, dx, axis=0), dy, axis=1)
                    iou = (sh & mref).sum() / float((sh | mref).sum())
                    if iou > best + 1e-9 or (abs(iou - best) < 1e-9 and dx * dx + dy * dy < bi[0] ** 2 + bi[1] ** 2):
                        best, bi = iou, (dx, dy)
            if best < 0.3:
                continue
            d = np.array([bi[0] * VOX, bi[1] * VOX, 0.0])
            for k in ('q', 's', 'all', 'cam'):
                views[i][k] = views[i][k] + d
            shifts[i] = (d[0], d[1])
        return shifts

    # ------------------------------------------------------------------ ② 복셀 · 빈 곳
    def _vox_index(self, p, n, nz):
        """설계 좌표 점(mm, 가운데 맞춤 뒤) → 복셀 번호 (ix, iy, iz)와 격자 안 표시."""
        ix = np.floor((p[:, 0] + self.half_mm) / VOX).astype(np.int64)
        iy = np.floor((p[:, 1] + self.half_mm) / VOX).astype(np.int64)
        iz = np.floor(p[:, 2] / VOX).astype(np.int64)
        ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n) & (iz >= 0) & (iz < nz)
        return ix, iy, iz, ok

    def _voxel_count(self, p, n, nz):
        """점 수를 복셀마다 센다 → (n, n, nz) int."""
        ix, iy, iz, ok = self._vox_index(p, n, nz)
        flat = (ix[ok] * n + iy[ok]) * nz + iz[ok]
        return np.bincount(flat, minlength=n * n * nz).reshape(n, n, nz).astype(np.int32)

    def _trim_low(self, sq):
        """맨 아래층 높이(< 두께 − 3 mm)의 점은 그 위 점(맨 아래층 윗면 포함)의 바닥 자국 + 10 mm 안에 있을 때만 남긴다(bool).

        작업면에 닿은 블록 가장자리에서는 깊이가 블록과 작업면 사이로 번져(실측) 블록 밖에 낮은 점이 생긴다 — 바닥 외곽 가운데(④)를
        끌고 가지 않게 뺀다. 위 점이 없으면 모두 남긴다.
        """
        up = sq[:, 2] >= self.block_mm[2] - 3.0
        if not up.any():
            return np.ones(len(sq), bool)
        n = int(round(2 * self.half_mm / VOX))
        ix = np.clip(np.floor((sq[:, 0] + self.half_mm) / VOX).astype(int), 0, n - 1)
        iy = np.clip(np.floor((sq[:, 1] + self.half_mm) / VOX).astype(int), 0, n - 1)
        foot = np.zeros((n, n), bool)
        foot[ix[up], iy[up]] = True
        foot = ndimage.binary_dilation(foot, iterations=int(round(10.0 / VOX)))
        return up | foot[ix, iy]

    def _largest_component(self, sq):
        """구조물 점 중 가장 큰 덩어리(2.5 mm 복셀을 5 mm 이웃으로 이음)에 속한 점 표시(bool). 케이블 · 다른 블록 · 잡음을 뺀다."""
        n, nz = int(round(2 * self.half_mm / VOX)), int(round(HEIGHT_MAX_MM / VOX))
        cnt = self._voxel_count(sq, n, nz)
        lab, k = ndimage.label(ndimage.binary_dilation(cnt > 0, iterations=2))
        if k <= 1:
            return np.ones(len(sq), bool)
        best = int(np.argmax(ndimage.sum(cnt, lab, index=np.arange(1, k + 1)))) + 1
        ix, iy, iz, ok = self._vox_index(sq, n, nz)
        keep = np.zeros(len(sq), bool)
        keep[ok] = lab[ix[ok], iy[ok], iz[ok]] == best
        return keep

    def _carve(self, view, n, nz, box):
        """카메라 → 점 사이 빛이 지나간 복셀을 센다(빈 곳). 점 6 mm 앞에서 멈춘다. 출력 (n, n, nz) int.

        box = (lo, hi) 설계 좌표 mm — 구조물 둘레 20 mm 안만 칠한다(계산 시간: scan_infer 는 작업 관리자 서비스 시간 안에 끝나야 한다).
        """
        q, cam = view['q'][::CARVE_EVERY], view['cam']
        lo = np.maximum(np.asarray(box[0], float), [-self.half_mm, -self.half_mm, 0.0])
        hi = np.minimum(np.asarray(box[1], float), [self.half_mm, self.half_mm, nz * VOX])
        vec = q - cam
        L = np.linalg.norm(vec, axis=1)
        d = vec / np.maximum(L, 1e-9)[:, None]
        with np.errstate(divide='ignore', invalid='ignore'):                         # 상자에 들어가고 나가는 거리(slab 방법)
            t1, t2 = (lo - cam) / d, (hi - cam) / d
        tmin = np.nanmax(np.where(np.isnan(np.minimum(t1, t2)), -np.inf, np.minimum(t1, t2)), axis=1)
        tmax = np.nanmin(np.where(np.isnan(np.maximum(t1, t2)), np.inf, np.maximum(t1, t2)), axis=1)
        t0 = np.maximum(tmin, 0.0)
        t_end = np.minimum(tmax, L - CARVE_STOP_MM)
        good = t_end > t0
        q, d, t0, t_end = q[good], d[good], t0[good], t_end[good]
        out = np.zeros(n * n * nz, np.int64)
        if len(q) == 0:
            return out.reshape(n, n, nz).astype(np.int32)
        steps = int(np.ceil(np.max(t_end - t0) / VOX)) + 1
        ts = np.arange(steps) * VOX
        for s in range(0, len(q), 4000):
            tt = t0[s:s + 4000, None] + ts[None, :]
            m = tt < t_end[s:s + 4000, None]
            p = cam[None, None, :] + tt[..., None] * d[s:s + 4000, None, :]
            p = p[m]
            ix, iy, iz, ok = self._vox_index(p, n, nz)
            out += np.bincount((ix[ok] * n + iy[ok]) * nz + iz[ok], minlength=n * n * nz)
        return out.reshape(n, n, nz).astype(np.int32)

    # ------------------------------------------------------------------ ⑤ 입체 S
    def _hull(self, sq, free, top, n, nz, zcut):
        """얇은 층(2.5 mm)마다 점이 차지한 x 폭 · y 폭으로 입체 S(bool, (n, n, nz))를 만든다.

        - 블록 표면 점은 늘 블록 안쪽 자리에 있으므로, 층마다 점의 x 자리 모음 · y 자리 모음은 실제 폭보다 넓지 않다
          (-x 면은 y 폭을, -y 면은 x 폭을, 윗면은 둘 다 알려 준다 — 앞 · 옆 카메라가 보는 면).
        - 한쪽 폭이 10 mm 보다 좁게만 보이면(옆면 한 장만 보임 — 가려진 기둥 위쪽 등) 그쪽 폭을 모른다고 보고
          구조물 전체 외곽으로 둔다 — 빈 곳 칠하기가 범위를 줄인다.
        - 작업면 위 기준(zcut mm) 아래 층은 점을 버렸으므로 바로 위층 폭을 그대로 쓴다.
        - S = (x 폭 × y 폭) ∩ 빈 곳 아님 ∩ 높이 지도 윗면(+1 mm) 아래.
        """
        ix, iy, iz, ok = self._vox_index(sq, n, nz)
        ix, iy, iz = ix[ok], iy[ok], iz[ok]

        def spans(idx):
            """층(z) × 칸(x 또는 y)마다 점이 있는가 → (nz, n) bool."""
            g = np.zeros((nz, n), bool)
            g[iz, idx] = True
            g = ndimage.binary_dilation(g, structure=np.ones((3, 1), bool))        # 위아래 한 층씩 합침(점이 드문 얇은 층 메움)
            kc = int(np.ceil(zcut / VOX))
            g[:kc] |= g[kc]                                                        # 작업면 위 기준 아래는 점을 버렸으므로 바로 위층 폭을 씀
            return ndimage.binary_closing(g, structure=np.ones((1, 3), bool))      # 2 칸(5 mm) 구멍은 메움

        X, Y = spans(ix), spans(iy)
        bx = np.zeros(n, bool)
        by = np.zeros(n, bool)
        x0, x1 = np.percentile(ix, [0.5, 99.5]).astype(int)                      # 구조물 전체 외곽(튀는 점 0.5 % 빼고)
        y0, y1 = np.percentile(iy, [0.5, 99.5]).astype(int)
        bx[x0:x1 + 1] = True
        by[y0:y1 + 1] = True
        narrow = int(round(10.0 / VOX))
        S = np.zeros((n, n, nz), bool)
        notfree = ~free
        for k in range(nz):
            xk, yk = X[k], Y[k]
            xi, yi = np.flatnonzero(xk), np.flatnonzero(yk)
            if len(xi) == 0 or len(yi) == 0:
                continue
            if xi[-1] - xi[0] + 1 < narrow:
                xk = bx
            if yi[-1] - yi[0] + 1 < narrow:
                yk = by
            S[:, :, k] = np.outer(xk, yk) & notfree[:, :, k]
        zc = (np.arange(nz) + 0.5) * VOX
        seen = ndimage.binary_erosion(~np.isnan(top))                              # 높이 지도 가장자리 셀(반만 덮인 5 mm 셀)은 빼고
        t = np.where(seen, np.nan_to_num(top, nan=-np.inf), -np.inf)[:, :, None]
        skin = (zc[None, None, :] > t - self.block_mm[2]) & (zc[None, None, :] < t) & (t > self.block_mm[2] / 2)
        S |= skin & notfree                                                        # 보인 윗면 밑은 적어도 블록 두께(15 mm)만큼 차 있다
        S &= ~(zc[None, None, :] > np.nan_to_num(top, nan=np.inf)[:, :, None] + 1.0)
        return S

    # ------------------------------------------------------------------ ⑤ 쪼개기
    def _tile(self, S):
        """입체 S 를 아래에서 위로 블록 상자로 채운다(겹침 없음). 출력: [(ori, x0, y0, z0, dx, dy, dz, fill)] 복셀 단위.

        한 번에 한 높이씩: 칸마다 '아직 안 채운 가장 낮은 S' 높이를 구해, 가장 낮은 높이(+5 mm 안)의 칸 모음 F(앞면)에
        블록 바닥 모양(75 × 25 눕힘 · 75 × 15 옆세움 · 15 × 25 세움 — 모양이 서로 달라 방향이 정해진다)을 2D 로 맞춰 놓는다(_tile_layer).
        놓은 블록 둘레 10 mm 안의 못 덮은 F 칸은 그 블록 윗면까지 채운 것으로 본다(실물 틈 · 자세 어긋남으로 S 가 조금 넓은 몫).
        그 밖의 못 덮은 칸은 5 mm 올려 다시 본다.
        """
        n, _, nz = S.shape
        used = np.zeros_like(S)
        filled = np.zeros((n, n), np.int64)                 # 칸마다 채운 높이(복셀)
        shapes = []
        for o in ORIS:
            dx, dy, dz = (int(round(e / VOX)) for e in self.extent[o])
            if dz <= nz:
                shapes.append((o, dx, dy, dz))
        kk = np.arange(nz)[None, None, :]
        out = []
        tops = {0}
        slack = int(round(SLACK_MM / VOX))
        for _ in range(4 * nz):
            rest = S & ~used & (kk >= filled[:, :, None])
            has = rest.any(axis=2)
            if not has.any() or len(out) >= MAX_BLOCKS:
                break
            znext = np.where(has, rest.argmax(axis=2), nz)
            # 바로 아래(채운 높이)와 10 mm 안이면 S 아래쪽이 조금 빠진 것으로 보고 채운 높이(받침 위)에 놓는다
            base = np.where(znext - filled <= GAP_SNAP, filled, znext)
            zmin = int(base[has].min())
            F = has & (base <= zmin + Z_STEP)
            vals, cnt = np.unique(base[F], return_counts=True)
            v = int(vals[np.argmax(cnt)])                                        # 앞면 높이 = 가장 많은 칸의 높이
            near = [t for t in tops if v - GAP_SNAP <= t <= v + Z_STEP]
            # 블록은 바닥이나 다른 블록 윗면에 놓이므로 놓은 블록 윗면(위로 5 mm · 아래로 10 mm 안)이 있으면 그 높이, 없으면 5 mm 격자로
            z0 = min(near, key=lambda t: abs(t - v)) if near else (v + Z_STEP // 2) // Z_STEP * Z_STEP
            below = [(x0, y0, dx, dy) for (_, x0, y0, bz, dx, dy, dz, _) in out if bz + dz == z0]
            placed = self._tile_layer(F, S & ~used, used, z0, shapes, n, nz, below)
            reach = np.full((n, n), -1, np.int64)
            for (o, x0, y0, dx, dy, dz, fill) in placed:
                used[x0:x0 + dx, y0:y0 + dy, z0:z0 + dz] = True
                filled[x0:x0 + dx, y0:y0 + dy] = np.maximum(filled[x0:x0 + dx, y0:y0 + dy], z0 + dz)
                sl = (slice(max(x0 - slack, 0), x0 + dx + slack), slice(max(y0 - slack, 0), y0 + dy + slack))
                reach[sl] = np.maximum(reach[sl], z0 + dz)
                tops.add(z0 + dz)
                out.append((o, x0, y0, z0, dx, dy, dz, fill))
            left = F & (filled <= znext)
            absorb = left & (reach >= 0)
            filled[absorb] = np.maximum(filled[absorb], reach[absorb])
            other = left & ~absorb
            filled[other] = np.maximum(filled[other], znext[other] + Z_STEP)       # 못 덮은 칸은 5 mm 위에서 다시
        return out

    def _tile_layer(self, F, Sav, used, z0, shapes, n, nz, below):
        """앞면 칸 모음 F(2D)에 블록 바닥 모양을 욕심 방법으로 놓는다(바닥 높이 z0 복셀, 이미 놓은 블록 used 와 겹치지 않게).

        1차: 눕힘(x · y)만. 2차: 옆세움 · 세움 — 남은 F 덩어리가 그 블록 바닥의 1.5 배 이하일 때만(큰 판의 남은 띠를 좁은 블록으로
        메우지 않게). 점수 = F 채움(90 % 이상은 1, 같으면 더 찬 쪽) + 0.5 × 4옆이 F 경계에 붙은 비율 + 방향 우선값 + 격자 우선값
        + 바로 아래 블록과 같은 자리면 쌓기 우선값(벽 · 기둥). 출력: [(ori, x0, y0, dx, dy, dz, fill)].
        """
        top = min(nz, z0 + max(dz for _, _, _, dz in shapes))                   # 이 층에 놓을 블록이 닿는 높이까지만 누적합(계산 시간)
        satV = _sat(Sav[:, :, z0:top])
        satU = _sat(used[:, :, z0:top])
        avail = F.copy()
        placed = []
        H = self.half_mm
        below = set(below)
        for flat in (True, False):
            while True:
                sa = np.zeros((n + 3, n + 3), np.int32)                             # 2D 누적합(둘레 1 칸 덧댐)
                sa[1:, 1:] = np.pad(avail, 1).astype(np.int32).cumsum(0).cumsum(1)
                if not flat:
                    lab, k = ndimage.label(avail)
                    area = np.bincount(lab.ravel(), minlength=k + 1)
                best = None
                for o, dx, dy, dz in shapes:
                    if (o in ('x', 'y')) != flat or z0 + dz > nz:
                        continue
                    X, Y = np.meshgrid(np.arange(n - dx + 1), np.arange(n - dy + 1), indexing='ij')
                    X, Y = X.ravel(), Y.ravel()

                    def box2(x0, y0, x1, y1):
                        """2D 누적합으로 [x0, x1) × [y0, y1) 칸 수(둘레 1 칸 덧댄 좌표)."""
                        return sa[x1 + 1, y1 + 1] - sa[x0 + 1, y1 + 1] - sa[x1 + 1, y0 + 1] + sa[x0 + 1, y0 + 1]
                    fill = box2(X, Y, X + dx, Y + dy) / float(dx * dy)
                    k = fill >= FILL_MIN
                    if not flat:
                        k &= area[lab[X + dx // 2, Y + dy // 2]] <= NARROW_COMP * dx * dy
                    if not k.any():
                        continue
                    X, Y, fill = X[k], Y[k], fill[k]
                    zz0, zz1 = np.zeros_like(X), np.full_like(X, dz)
                    vert = _box(satV, X, Y, zz0, X + dx, Y + dy, zz1) / float(dx * dy * dz)
                    k = (vert >= VERT_MIN) & (_box(satU, X, Y, zz0, X + dx, Y + dy, zz1) == 0)
                    if not k.any():
                        continue
                    X, Y, fill = X[k], Y[k], fill[k]
                    faces = [1 - box2(X - 1, Y, X, Y + dy) / dy, 1 - box2(X + dx, Y, X + dx + 1, Y + dy) / dy,
                             1 - box2(X, Y - 1, X + dx, Y) / dx, 1 - box2(X, Y + dy, X + dx, Y + dy + 1) / dx]
                    grid = sum((np.abs(e * VOX - H) % LATTICE_MM < 0.1).astype(float) for e in (X, X + dx, Y, Y + dy))
                    stack = np.array([(int(x), int(y), dx, dy) in below for x, y in zip(X, Y)], float)
                    score = (np.minimum(fill / FILL_FULL, 1.0) + 0.05 * fill + BOUNDARY_W * np.mean(faces, axis=0) + ORI_PRIOR[o]
                             + LATTICE_PRIOR * grid + STACK_PRIOR * stack)
                    i = int(np.argmax(score))
                    if best is None or score[i] > best[0]:
                        best = (score[i], o, int(X[i]), int(Y[i]), dx, dy, dz, float(fill[i]))
                if best is None:
                    break
                _, o, x0, y0, dx, dy, dz, fill = best
                avail[x0:x0 + dx, y0:y0 + dy] = False
                placed.append((o, x0, y0, dx, dy, dz, fill))
        return placed

    def _to_block(self, r, H):
        """복셀 상자 → blocks/1 블록(mm, 가운데 x · y, 아랫면 z)."""
        o, x0, y0, z0, dx, dy, dz, _ = r
        return {'x': (x0 + dx / 2) * VOX - H, 'y': (y0 + dy / 2) * VOX - H, 'z': z0 * VOX, 'ori': o, 'inferred': False}

    # ------------------------------------------------------------------ 블록 사이 계산(mm)
    def _rect(self, b):
        """blocks/1 블록 → 상자 (x0, x1, y0, y1, z0, z1) mm."""
        ex, ey, ez = self.extent[b['ori']]
        return b['x'] - ex / 2, b['x'] + ex / 2, b['y'] - ey / 2, b['y'] + ey / 2, b['z'], b['z'] + ez

    def _overlap(self, a, b, tol=1.0):
        """두 블록이 tol(mm) 넘게 서로 파고드는가."""
        A, B = self._rect(a), self._rect(b)
        return all(min(A[2 * k + 1], B[2 * k + 1]) - max(A[2 * k], B[2 * k]) > tol for k in range(3))

    def _supported(self, b, blocks):
        """블록 b 가 바닥이나 아래 블록 윗면에 얹혀 있고, 가운데가 받침 면 외곽 안인가(대략 — 정확한 검사는 검사 묶음)."""
        if b['z'] <= 3.0:
            return True
        x0, x1, y0, y1, z0, _ = self._rect(b)
        xs, ys = [], []
        for c in blocks:
            if c is b:
                continue
            cx0, cx1, cy0, cy1, _, cz1 = self._rect(c)
            if abs(cz1 - z0) > 3.0:
                continue
            ox0, ox1, oy0, oy1 = max(x0, cx0), min(x1, cx1), max(y0, cy0), min(y1, cy1)
            if ox1 - ox0 > 1.0 and oy1 - oy0 > 1.0:
                xs += [ox0, ox1]
                ys += [oy0, oy1]
        return bool(xs) and min(xs) - 1 <= b['x'] <= max(xs) + 1 and min(ys) - 1 <= b['y'] <= max(ys) + 1

    def _cross_count(self, group, blocks):
        """group 블록들이 바로 위 · 아래의 다른 방향 눕힌 블록과 닿는 수(젠가식 엇갈림)."""
        k = 0
        for g in group:
            gx0, gx1, gy0, gy1, gz0, gz1 = self._rect(g)
            for c in blocks:
                if c in group or c['ori'] not in ('x', 'y') or c['ori'] == g['ori']:
                    continue
                cx0, cx1, cy0, cy1, cz0, cz1 = self._rect(c)
                if (abs(cz1 - gz0) <= 3 or abs(cz0 - gz1) <= 3) and min(gx1, cx1) - max(gx0, cx0) > 1 and min(gy1, cy1) - max(gy0, cy0) > 1:
                    k += 1
        return k

    def _fix_square_layers(self, blocks):
        """75 × 75 판(같은 높이 눕힌 블록 3개)은 x 3개 · y 3개 둘 다 같은 모양이라 점으로는 못 가른다 →
        받침받는 블록 수가 많은 쪽, 같으면 위 · 아래 블록과 엇갈리는 쪽(젠가식)을 고른다. 출력: 고친 블록 목록."""
        W = self.block_mm[1]
        blocks = list(blocks)
        done = set()
        changed = True
        while changed:
            changed = False
            for a in blocks:
                if a['ori'] not in ('x', 'y'):
                    continue
                wide = 'y' if a['ori'] == 'x' else 'x'           # 판에서 블록이 나란히 놓이는 쪽(블록 폭 방향)
                long_ = 'x' if wide == 'y' else 'y'
                key = (a['ori'], round(a['x']), round(a['y']), round(a['z']))
                if key in done:
                    continue
                done.add(key)

                tol = SQUARE_TOL_MM
                row = [b for b in blocks if b is not a and b['ori'] == a['ori'] and abs(b['z'] - a['z']) < 1
                       and abs(b[long_] - a[long_]) < tol]
                lo = next((b for b in row if abs(b[wide] - a[wide] + W) < tol), None)
                hi = next((b for b in row if abs(b[wide] - a[wide] - W) < tol), None)
                if lo is None or hi is None:
                    continue
                trio = [lo, a, hi]
                cx = float(np.mean([b['x'] for b in trio]))
                cy = float(np.mean([b['y'] for b in trio]))
                alt = []
                for d in (-W, 0.0, W):
                    nb = {'x': cx, 'y': cy, 'z': a['z'], 'ori': wide, 'inferred': False}
                    nb[long_] += d
                    alt.append(nb)
                rest = [b for b in blocks if all(b is not t for t in trio)]
                cur = (sum(self._supported(b, rest + trio) for b in trio), self._cross_count(trio, rest))
                new = (sum(self._supported(b, rest + alt) for b in alt), self._cross_count(alt, rest))
                if new > cur:
                    if any(self._overlap(t, o) for t in alt for o in rest):
                        continue
                    blocks = rest + alt
                    done.update((b['ori'], round(b['x']), round(b['y']), round(b['z'])) for b in alt)
                    changed = True
                    break
        return blocks

    # ------------------------------------------------------------------ ⑥ 가려진 블록
    def _free_frac(self, b, free, n, H):
        """블록 상자 안 복셀 중 '빈 곳으로 보인' 비율(카메라 빛이 지나간 곳)."""
        x0, x1, y0, y1, z0, z1 = self._rect(b)
        i0, i1 = int(round((x0 + H) / VOX)), int(round((x1 + H) / VOX))
        j0, j1 = int(round((y0 + H) / VOX)), int(round((y1 + H) / VOX))
        k0, k1 = int(round(z0 / VOX)), int(round(z1 / VOX))
        i0, j0, k0 = max(i0, 0), max(j0, 0), max(k0, 0)
        i1, j1, k1 = min(i1, n), min(j1, n), min(k1, free.shape[2])
        if i1 <= i0 or j1 <= j0 or k1 <= k0:
            return 1.0
        return float(free[i0:i1, j0:j1, k0:k1].mean())

    def _infer_hidden(self, blocks, free, n, nz, H):
        """받침이 없는 블록 바로 아래에 같은 자리 블록을 넣는다(inferred: true) — 바닥까지 반복(벽 구조, SDD ⑥).

        같은 방향이 겹치거나 빈 곳으로 보이면 같은 가운데의 다른 방향을 차례로 본다. 다 안 되면 넣지 않는다(검사 묶음이 잡는다).
        """
        blocks = list(blocks)
        queue = sorted(blocks, key=lambda b: -b['z'])
        while queue:
            b = queue.pop(0)
            if self._supported(b, blocks):
                continue
            for o in (b['ori'],) + tuple(x for x in ORIS if x != b['ori']):
                nb = {'x': b['x'], 'y': b['y'], 'z': b['z'] - self.extent[o][2], 'ori': o, 'inferred': True}
                if nb['z'] < -3.0:
                    continue
                nb['z'] = max(nb['z'], 0.0)
                if any(self._overlap(nb, c) for c in blocks) or self._free_frac(nb, free, n, H) > FREE_MAX_INFERRED:
                    continue
                blocks.append(nb)
                queue.append(nb)
                break
            if len(blocks) > MAX_BLOCKS:
                break
        return blocks

    # ------------------------------------------------------------------ ④ 격자 · ⑦ 순서
    def _align_columns(self, blocks):
        """같은 방향 블록이 바로 위아래로 바닥 넓이 30 % 넘게 겹쳐 쌓였으면(벽 · 기둥) 한 기둥으로 보고 x · y 를 기둥의 가운데값으로
        맞춘다 — 층마다 S 가 5~10 mm 넓게 나와 자리가 조금씩 흔들리는 것을 줄인다. 다른 블록과 겹치게 되면 그 기둥은 그대로 둔다."""
        n = len(blocks)
        parent = list(range(n))

        def root(i):
            """같은 기둥 묶음의 대표 번호(union-find)."""
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        for i, a in enumerate(blocks):
            ax0, ax1, ay0, ay1, az0, az1 = self._rect(a)
            for j, b in enumerate(blocks):
                if j <= i or a['ori'] != b['ori']:
                    continue
                bx0, bx1, by0, by1, bz0, bz1 = self._rect(b)
                if abs(az1 - bz0) > 3 and abs(bz1 - az0) > 3:
                    continue
                ov = max(0.0, min(ax1, bx1) - max(ax0, bx0)) * max(0.0, min(ay1, by1) - max(ay0, by0))
                if ov > 0.3 * (ax1 - ax0) * (ay1 - ay0):
                    parent[root(j)] = root(i)
        groups = {}
        for i in range(n):
            groups.setdefault(root(i), []).append(i)
        out = [dict(b) for b in blocks]
        for g in groups.values():
            if len(g) < 2:
                continue
            mx = float(np.median([blocks[i]['x'] for i in g]))
            my = float(np.median([blocks[i]['y'] for i in g]))
            trial = [dict(out[i], x=mx, y=my) for i in g]
            others = [out[k] for k in range(n) if k not in g]
            if not any(self._overlap(t, o) for t in trial for o in others):
                for i, t in zip(g, trial):
                    out[i] = t
        return out

    def _snap(self, blocks):
        """가운데 x · y 를 12.5 mm 격자(블록 폭 반)에서 2.5 mm 안이면 격자로, z 는 5 mm 격자로. 겹치면 되돌린다.
        출력: (블록 목록, 격자 어긋남 평균 mm)."""
        errs = []
        out = []
        for b in blocks:
            nb = dict(b)
            for k in ('x', 'y'):
                g = round(nb[k] / 12.5) * 12.5
                errs.append(abs(nb[k] - g))
                if abs(nb[k] - g) <= 2.5:
                    nb[k] = g
            nb['z'] = round(nb['z'] / 5.0) * 5.0
            out.append(nb if not any(self._overlap(nb, c) for c in out) else b)
        clean = [{'x': float(round(b['x'], 2)), 'y': float(round(b['y'], 2)), 'z': float(round(b['z'], 2)),
                  'ori': b['ori'], 'inferred': bool(b['inferred'])} for b in out]
        return clean, (float(np.mean(errs)) if errs else 0.0)

    def _order(self, blocks):
        """⑦ order: 아래층부터, 같은 높이는 위 블록을 많이 받치는 순(그다음 x · y). 출력: order 를 단 blocks/1 블록 목록."""
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
        return [{'order': i + 1, 'x': b['x'], 'y': b['y'], 'z': b['z'], 'ori': b['ori'], 'inferred': b['inferred']}
                for i, b in enumerate(rows)]

    # ------------------------------------------------------------------ ⑧ 신뢰도
    def _block_mask(self, blocks, n, nz, H, pad=0):
        """블록 상자들을 복셀 표시로(pad 복셀만큼 넓힘)."""
        m = np.zeros((n, n, nz), bool)
        for b in blocks:
            x0, x1, y0, y1, z0, z1 = self._rect(b)
            i0, i1 = int(round((x0 + H) / VOX)) - pad, int(round((x1 + H) / VOX)) + pad
            j0, j1 = int(round((y0 + H) / VOX)) - pad, int(round((y1 + H) / VOX)) + pad
            k0, k1 = int(round(z0 / VOX)) - pad, int(round(z1 / VOX)) + pad
            m[max(i0, 0):max(i1, 0), max(j0, 0):max(j1, 0), max(k0, 0):max(k1, 0)] = True
        return m

    def _explained(self, occ, blocks, n, nz, H):
        """점유 복셀 중 블록 표면에서 5 mm 안(블록 상자를 2 복셀 넓힌 안)에 있는 비율."""
        if not occ.any():
            return 0.0
        m = self._block_mask(blocks, n, nz, H, pad=2)
        return float((occ & m).sum() / occ.sum())

    def _covered(self, S, blocks, n, nz, H):
        """입체 S 중 블록이 차지한 비율."""
        if not S.any():
            return 0.0
        return float((S & self._block_mask(blocks, n, nz, H)).sum() / S.sum())

    def nearest_base(self, blocks, bases):
        """블록 수 · 바닥 외곽 크기가 가장 가까운 기본 설계(blocks/1 dict)를 고른다. bases 가 비면 None."""
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
        o = self.origin_mm / 1000.0
        r = self.half_mm / 1000.0 + 0.05
        m = (np.abs(p[:, 0] - o[0]) < r) & (np.abs(p[:, 1] - o[1]) < r) & (p[:, 2] > o[2] - 0.01) & (p[:, 2] < o[2] + 0.30)
        p = p[m]
        voxel = CLOUD_VOXEL_M
        while True:
            if len(p):
                key = np.floor(p / voxel).astype(np.int64)
                _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
                inv = inv.ravel()
                pts = np.stack([np.bincount(inv, weights=p[:, k]) / cnt for k in range(3)], axis=1).astype('<f4')
            else:
                pts = np.zeros((0, 3), '<f4')
            header = ('ply\nformat binary_little_endian 1.0\ncomment d2 StructureScanner base_link m voxel {:.4f}\n'
                      'element vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
                      ).format(voxel, len(pts)).encode('ascii')
            if len(header) + pts.nbytes <= CLOUD_MAX_BYTES or len(pts) == 0:
                break
            voxel *= 1.25
        with open(path, 'wb') as f:
            f.write(header)
            f.write(pts.tobytes())
        self.cloud_voxel_m = voxel
        return int(len(pts))


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


def scan_response(result, image_path, cloud_path):
    """infer() 결과 → scan_infer 응답 (success, reason, 응답 dict) — 노드가 JsonQuery 응답 세 칸에 그대로 넣는다(IRD 4.2).

    입력: result = StructureScanner.infer() 출력 · image_path = 컬러 사진 경로(없으면 '') · cloud_path = PLY 경로(저장 실패면 None).
    출력: ok → (True, '', {"ok":true,"blocks":blocks/1,"inferred_count","image_path"[,"cloud_path"]}),
          신뢰도 미달 → (True, '', {"ok":false,"reason":"SCAN_FAILED","detail":이유}) — 답은 냈지만 결과가 나쁨(mock_scan 과 같은 두 층).
    cloud_path 가 None 이면 그 칸을 뺀다(PLY 저장 실패 — IRD 4.2 선택 칸, 10/8 PL: 없으면 화면 점군 창만 안 뜬다). 블록 좌표는 소수 1자리 mm 로 반올림. 바깥 영향 없음."""
    if not result.get('ok'):
        return True, '', {'ok': False, 'reason': 'SCAN_FAILED', 'detail': str(result.get('reason', ''))}
    blocks = json_ready(result['blocks'])
    for b in blocks['blocks']:
        for k in ('x', 'y', 'z'):
            b[k] = round(float(b[k]), 1)
        b['order'], b['inferred'] = int(b['order']), bool(b.get('inferred', False))
    body = {'ok': True, 'blocks': blocks, 'inferred_count': sum(1 for b in blocks['blocks'] if b['inferred']),
            'image_path': str(image_path or '')}
    if cloud_path is not None:
        body['cloud_path'] = str(cloud_path)
    return True, '', body


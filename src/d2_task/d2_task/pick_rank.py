# -*- coding: utf-8 -*-
"""흩뿌린 블록 집기 순서 PickRanker · 같은 층 옆 블록 띄우기 spread_parts (ROS 없이 동작, 10/13 박진용 확정안).

시험 도구 one_pick.py 에서 10/13 실기로 확인한 고르기를 옮긴 것이다(레시피 4번 블록 11/11 · 집기 24/24 · 헛시도 0 · 사람 호출 0).
작업 판단(task)이 find_blocks 결과로 '어느 블록을 어떤 잡기로 먼저 해 볼지' 후보 목록을 만든다. 로봇 · 메시지에는 손대지 않는다(계산만).

고르는 규칙(위 칸 후보를 다 해 본 뒤 아래 칸):
  1 남을 덮은 블록 중 치우면 열리는 수 ≥ COVER_MIN_FREE (큰 옮김은 빼고 6칸으로)  2 눕힘 곧게(FLAT_LONG ±2.4 mm 포함)  3 눕힘 틀기
  4 기울여 집기  5 옆으로 선 · 선 블록  6 FLAT_SHORT 큰 옮김(10 · 20 · 27.2 mm)
  칸 안: 높은 층 → 치우면 열리는 수 → 곧게 · 틀기 · 기울여 집기 → 잡기 GRASP_ORDER → 작은 각 → 작은 옮김 → 손가락 쪽 틈 넓은 것.
깊이 점(pts)을 주면 줄 세우기 전에 모든 후보의 손가락 두 자리를 깊이로 보고 막힌 후보를 뺀다 — 위에서 본 사각형 검사만으로는
곧게 FLAT_LONG 37번 중 19번 · 틀기 25번 중 21번이 고른 뒤 깊이에서 걸려 헛시도가 쌓였다(10/12 ~ 13 실기).
"""
import math

import numpy as np

from d2_task.block_picker import BlockPicker, grasp_target

UPS = ('THICKNESS', 'WIDTH', 'LENGTH')
# 위로 선 축별 수평 두 축 (축 이름, yaw 에 더하는 각 °) — scatter_pick.block_pose 규약: 눕힘 · 옆으로 섬은 yaw = 길이 방향, 세움은 yaw = 폭 방향
HORIZ = {'THICKNESS': (('LENGTH', 0.0), ('WIDTH', 90.0)), 'WIDTH': (('LENGTH', 0.0), ('THICKNESS', 90.0)),
         'LENGTH': (('WIDTH', 0.0), ('THICKNESS', 90.0))}
POSE_KO = {'THICKNESS': '눕힘', 'WIDTH': '옆으로 섬', 'LENGTH': '세움'}
TIER_NAMES = {1: '덮은 블록 치우기', 2: '눕힘 곧게', 3: '눕힘 틀기', 4: '기울여 집기', 5: '옆으로 선·선', 6: '큰 옮김'}


def rect(cx, cy, yaw, half_a, half_b):
    """가운데 (cx, cy) m · 방향 yaw(rad, a 축) · 반길이 a · b 인 사각형의 네 꼭짓점 [(x, y)]."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * u - s * v, cy + s * u + c * v) for u, v in ((-half_a, -half_b), (half_a, -half_b), (half_a, half_b), (-half_a, half_b))]


def overlap(p, q):
    """볼록 다각형 두 개(꼭짓점 목록)가 겹치나 — 분리축 검사. 맞닿기만 해도 겹침으로 본다."""
    for poly in (p, q):
        for k in range(len(poly)):
            (x1, y1), (x2, y2) = poly[k], poly[(k + 1) % len(poly)]
            nx, ny = y1 - y2, x2 - x1
            a = [nx * x + ny * y for x, y in p]
            b = [nx * x + ny * y for x, y in q]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


class PickRanker:
    """find_blocks 블록 목록에서 집기 시도 후보를 해 볼 순서대로 만든다.

    입력: cfg = robot.yaml dict(block_actual_m(없으면 block_size_m) · table_z_m · grasp_depth_m · grasp_open_pick_m · grasp_width_m ·
    finger.thickness_m · finger.width_m). 입력 blocks 는 바꾸지 않는다. 같은 입력이면 늘 같은 순서.
    기울어진 블록은 부르는 쪽이 깊이로 윗면 평면을 재서 붙인 center_top_z_m(윗면 가운데 높이 m) · tilt_tip_z_m(손끝 높이 m)이 있을 때만 후보다.
    """
    YAW_OFFSETS = (0, 5, -5, 10, -10, 15, -15, 20, -20)     # 그리퍼 각도 틀기 (°) — 0 먼저
    GRASP_ORDER = ('FLAT_SHORT', 'FLAT_LONG', 'EDGE_SHORT', 'EDGE_LONG', 'STAND_SHORT', 'STAND_LONG')
    POS_ERR_M = 0.002          # 위치 오차 여유 e — 옆 블록 사각형을 이만큼 키운다(robot.yaml find.min_gap_mm = 2e + 18.2 의 e)
    GAIN_NEAR_M = 0.12         # 치운 블록에서 이 안의 블록만 '새로 잡히나' 따진다(블록 길이 + 손가락 자리)
    STACK_CONF_MIN = 0.7       # 더미 위(overlap top) 블록을 집는 확신도 최소 — 10/13 004 실기: 더미 위에 비스듬히 걸친 눕힌 블록을 '옆으로 섬'(확신도 0.59)으로
                               # 보고 집어 받침 위 약 12 mm 에서 놓임(로봇은 계획대로, 자세 판단이 틀림). 그 장면 다른 더미 위 블록은 0.74 ~ 0.83
    COVER_MIN_FREE = 2         # 남을 덮은 블록을 1칸으로 올리는 '치우면 열리는 수' 최소 — 곧게 잡히는 블록이 드러나거나 2개 이상 열림(10/13 박진용 · 안 B)
    SAME_M = 0.015             # 실패한 시도와 블록 위치가 이 안이면 같은 시도로 보고 다시 고르지 않는다
    STACK_TOL_M = 0.004        # 얹힌 블록의 잰 높이가 층 높이와 이 안이면 층 높이로 잡는다(비전 높이 판정 허용과 같은 크기)
    # 깊이 손가락 자리 검사: 손끝 + FINGER_CLEAR_M 보다 높은 점이 FINGER_BLOCK_FRAC 넘으면 막힘. 점이 FINGER_MIN_PTS 보다 적으면 모름(막힘 아님).
    # 비율 5 % 는 10/13 실기 값(20 % 일 때 거름 28번 중 다수가 20 ~ 21 % 로 기준에 붙어 그냥 내려가 옆 블록에 닿음)
    FINGER_CLEAR_M, FINGER_BLOCK_FRAC, FINGER_MIN_PTS = 0.003, 0.05, 30

    def __init__(self, cfg):
        """치수 · 손가락 · 여는 폭을 cfg 에서 읽는다. 키가 없으면 KeyError(설정 오류)."""
        self.cfg = cfg
        self.L, self.W, self.T = (float(v) for v in cfg.get('block_actual_m') or cfg['block_size_m'])
        self.size = {'LENGTH': self.L, 'WIDTH': self.W, 'THICKNESS': self.T}
        self.table_z = float(cfg['table_z_m'])
        self.depth = float(cfg['grasp_depth_m'])
        self.open_pick = {k: float(v) for k, v in cfg['grasp_open_pick_m'].items()}
        self.finger_t, self.finger_w = float(cfg['finger']['thickness_m']), float(cfg['finger']['width_m'])
        self.tilt_depth = self.T / 2           # 기울어진 블록은 윗면에서 두께 절반만 문다(밑에 깔린 블록을 안 찍게, 10/12 실기)
        s_long = self.L / 2 - self.finger_w / 2    # 손가락(폭 20 mm)이 긴 옆면 위에 다 올라가는 한계 = 27.2 mm
        s_end = self.W / 2 - self.finger_w / 2     # 끝면(24.8 mm)을 물 때는 2.4 mm 뿐
        self.slide = {'FLAT_SHORT': (0.0, 0.01, -0.01, 0.02, -0.02, s_long, -s_long), 'FLAT_LONG': (0.0, s_end, -s_end)}

    # ---------------- 기하 ----------------
    def block_rect(self, b, pad=0.0):
        """find_blocks 블록 → 위에서 본 사각형(수평 두 변, pad 만큼 키움). yaw = 윗면 긴 변 방향(세운 블록은 폭 변)."""
        a, c = {'THICKNESS': (self.L, self.W), 'WIDTH': (self.L, self.T), 'LENGTH': (self.W, self.T)}[b['up']]
        return rect(b['x_m'], b['y_m'], math.radians(b['yaw_deg']), a / 2 + pad, c / 2 + pad)

    def finger_rects(self, gx, gy, close_yaw, open_half):
        """잡는 점 (gx, gy) · 닫는 방향 close_yaw(rad) · 연 폭 절반 → 손가락 두 자리 [(가운데 x, y, yaw, 반두께, 반폭)]."""
        d = open_half + self.finger_t / 2
        return [(gx + sg * d * math.cos(close_yaw), gy + sg * d * math.sin(close_yaw), close_yaw, self.finger_t / 2, self.finger_w / 2)
                for sg in (1, -1)]

    @staticmethod
    def _poly(f):
        return rect(*f)

    def on_face(self, b, face_axis, face_half, fingers):
        """손가락 두 자리가 블록 b 의 면 범위 안인가 — 꼭짓점을 면 방향(face_axis, rad)에 투영해 가운데에서 face_half 안이면 True.
        옮김 ±27 mm 에 각도 틀기가 겹치면 손가락 모서리가 끝면 밖으로 나가 덜 문다(10/12 한석형 질문 ⑤, 여유값 없음)."""
        c, s = math.cos(face_axis), math.sin(face_axis)
        return all(abs((x - b['x_m']) * c + (y - b['y_m']) * s) <= face_half + 1e-9 for f in fingers for x, y in self._poly(f))

    def fingers_blocked(self, fingers, tip_z, pts):
        """손가락 자리 · 손끝 높이 tip_z (m) · 깊이 점 pts (N, 3) m → 막힌 손가락 [(번호, 높은 점 비율)]. 점이 모자란 손가락은 뺀다."""
        pts = np.asarray(pts, dtype=float)
        out = []
        for k, (cx, cy, yaw, ha, hb) in enumerate(fingers):
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
            u, v = dx * c + dy * s, -dx * s + dy * c
            inside = pts[(np.abs(u) <= ha) & (np.abs(v) <= hb)]
            if len(inside) < self.FINGER_MIN_PTS:
                continue
            frac = float(np.mean(inside[:, 2] > tip_z + self.FINGER_CLEAR_M))
            if frac > self.FINGER_BLOCK_FRAC:
                out.append((k + 1, frac))
        return out

    def top_of(self, b):
        """집을 블록 윗면 높이 (m) — 카메라 높이(1 ~ 2 mm 흔들림) 대신 층 높이로(10/11 실기: 그 높이로 물면 매달려 놓을 때 받침을 눌렀다).
        작업대에 바로 놓인 블록은 작업대 + 위로 선 치수, 얹힌 블록은 받침 0 ~ 3개 치수 합 중 잰 높이에 가장 가까운 층(STACK_TOL_M 안일 때, 아니면 잰 값).
        base_z_m(받침 윗면)이 있으면 받침 윗면 + 치수."""
        own = self.size[b['up']]
        if 'base_z_m' in b:
            return b['base_z_m'] + own
        if b.get('overlap') != 'top':
            return self.table_z + own
        dims = (0.0, self.T, self.W, self.L)
        levels = {a + c + d for a in dims for c in dims for d in dims}
        meas = b['top_z_m'] - self.table_z - own
        best = min(levels, key=lambda v: abs(v - meas))
        return self.table_z + own + best if abs(best - meas) <= self.STACK_TOL_M else b['top_z_m']

    def layer_of(self, b):
        """층(1부터) = 바닥 높이 ÷ 두께(반올림) + 1. 바닥 = 윗면 − 그 자세 높이, 기울어진 블록은 가운데 윗면(없으면 윗면) − 두께.
        윗면 높이로 세면 작업대에 혼자 선 블록이 5층이 돼 맨 앞에 왔다(10/13 실기: 같은 옆으로 선 블록을 블록 4개 동안 헛시도)."""
        z = (b.get('center_top_z_m', b['top_z_m']) - self.T) if b.get('tilted') else (b['top_z_m'] - self.size[b['up']])
        return max(1, round((z - self.table_z) / self.T) + 1)

    def covers(self, blocks, i):
        """i 번이 남을 덮은 블록인가 — 비전이 overlap=top 이라고 했거나 기울어진 블록 중, 위에서 본 사각형이 덮인(under) 블록과 겹치는 것.
        덮인 블록 옆에 붙어 선 블록은 사각형이 1 ~ 2 mm 겹쳐도 덮은 게 아니다."""
        b = blocks[i]
        if b.get('overlap') != 'top' and not b.get('tilted'):
            return False
        r = self.block_rect(b)
        return any(j != i and o.get('overlap') == 'under' and overlap(r, self.block_rect(o)) for j, o in enumerate(blocks))

    # ---------------- 후보 ----------------
    def grasp_options(self, blocks, i):
        """i 번 블록을 놓인 자세 그대로 위에서 잡을 수 있는 [(잡기, 각도 틀기 °, 손가락 옮김 m)] — 손가락이 옆 블록과 안 겹치는 것만.

        잡기는 그 자세의 SHORT · LONG. 손가락이 닿는 면이 손가락 폭보다 넓으면 면 밖으로 안 나가는지(on_face)도 본다. 옮기기는 눕힌 블록만.
        손끝(윗면 − grasp_depth_m)보다 윗면이 높은 블록만 막는다. 기울어진 옆 블록은 벽으로 안 친다 — 높이가 가장 높은 끝이라 낮은 끝 쪽까지
        막았다(10/12 가상). 그런 블록이 실제로 높은지는 깊이 검사가 본다."""
        b = blocks[i]
        up = b['up']
        tip_z = b['top_z_m'] - self.depth
        walls = [self.block_rect(o, self.POS_ERR_M) for j, o in enumerate(blocks) if j != i and o['top_z_m'] > tip_z and not o.get('tilted')]
        axes = dict(HORIZ[up])
        out = []
        for grasp in self.GRASP_ORDER:
            if grasp_target(grasp, self.cfg)[0] != up:
                continue
            close_name = grasp_target(grasp, self.cfg)[1]
            face_name = next(n for n in axes if n != close_name)          # 손가락이 닿는 면이 뻗은 수평 축 = 옮기는 축
            close_axis = math.radians(b['yaw_deg'] + axes[close_name])
            slide_axis = math.radians(b['yaw_deg'] + axes[face_name])
            face_half = self.size[face_name] / 2
            open_half = self.open_pick[grasp] / 2
            for dyaw in self.YAW_OFFSETS:
                for s in self.slide.get(grasp, (0.0,)):
                    gx, gy = b['x_m'] + s * math.cos(slide_axis), b['y_m'] + s * math.sin(slide_axis)
                    fingers = self.finger_rects(gx, gy, close_axis + math.radians(dyaw), open_half)
                    if self.size[face_name] >= self.finger_w and not self.on_face(b, slide_axis, face_half, fingers):
                        continue
                    polys = [self._poly(f) for f in fingers]
                    if not any(overlap(p, w) for p in polys for w in walls):
                        out.append((grasp, dyaw, s))
        return out

    def tilt_fingers_hit(self, blocks, i):
        """기울어진 블록 i 를 좁은 폭으로 물 손가락 두 자리가 손끝 높이(tilt_tip_z_m)보다 윗면이 높은 반듯한 옆 블록과 겹치나
        (10/13 실기: 깊이 검사만 하다가 옆으로 선 블록에 손가락이 걸려 SAFE_STOP). 기울어진 옆 블록은 벽으로 안 친다."""
        b = blocks[i]
        walls = [self.block_rect(o, self.POS_ERR_M) for j, o in enumerate(blocks)
                 if j != i and o['top_z_m'] > b['tilt_tip_z_m'] and not o.get('tilted')]
        fingers = self.finger_rects(b['x_m'], b['y_m'], math.radians(b['yaw_deg'] + 90), self.open_pick['FLAT_SHORT'] / 2)
        return any(overlap(self._poly(f), w) for f in fingers for w in walls)

    def _reach(self, blocks, k):
        """블록 k 를 지금 잡을 수 있나: (곧게 잡힘, 어떻게든 잡힘) — 위에서 본 손가락 자리 검사로만."""
        b = blocks[k]
        if b.get('overlap') == 'under' or b.get('tilted') or not BlockPicker.is_valid(b):
            return False, False
        opts = self.grasp_options(blocks, k)
        return any(d == 0 and sl == 0.0 for _, d, sl in opts), bool(opts)

    def freed(self, blocks, i):
        """i 번을 치우면 GAIN_NEAR_M 안 블록 중 새로 잡히면 +1, 곧게 잡히게 되면 +1 — 덮인 블록은 i 말고 덮은 블록이 없을 때만 드러난다고 본다.
        치운 뒤 장면은 깊이로 미리 볼 수 없어 위에서 본 사각형으로만 센다."""
        bi = blocks[i]
        ri = self.block_rect(bi)
        n = 0
        for k, b in enumerate(blocks):
            if k == i or math.hypot(b['x_m'] - bi['x_m'], b['y_m'] - bi['y_m']) > self.GAIN_NEAR_M:
                continue
            before = self._reach(blocks, k)
            b2 = b
            if b.get('overlap') == 'under' and overlap(ri, self.block_rect(b)):
                rk = self.block_rect(b)
                if not any(j not in (i, k) and o.get('overlap') != 'under' and overlap(rk, self.block_rect(o)) for j, o in enumerate(blocks)):
                    b2 = dict(b, overlap='none')
            after = self._reach([b2 if j == k else o for j, o in enumerate(blocks) if j != i], k - (1 if i < k else 0))
            n += (after[0] and not before[0]) + (after[1] and not before[1])
        return n

    def gap_of(self, b, grasp):
        """손가락 쪽 틈 (mm) — find_blocks gap_mm 의 닫는 축 값(block_finder: 그 축으로 닫힐 때 양쪽 틈 중 좁은 쪽). 기울여 집기는 좁은 폭."""
        return float((b.get('gap_mm') or {}).get(grasp_target('FLAT_SHORT' if grasp == 'TILT' else grasp, self.cfg)[1]) or 0.0)

    def candidate_fingers(self, b, grasp, dyaw, s):
        """후보 하나의 손가락 두 자리와 손끝 높이 (m) — 고른 뒤 집기와 같은 자리(옮김은 닫는 축이 아닌 수평 축으로)."""
        if grasp == 'TILT':
            return (self.finger_rects(b['x_m'], b['y_m'], math.radians(b['yaw_deg'] + 90), self.open_pick['FLAT_SHORT'] / 2),
                    b.get('tilt_tip_z_m', b['top_z_m'] - self.tilt_depth))
        if s == 0.0:
            gx, gy = b['x_m'], b['y_m']
        else:
            slide_axis = math.radians(b['yaw_deg'] if grasp == 'FLAT_SHORT' else b['yaw_deg'] + 90)
            gx, gy = b['x_m'] + s * math.cos(slide_axis), b['y_m'] + s * math.sin(slide_axis)
        close_axis = math.radians(b['yaw_deg'] + dyaw + dict(HORIZ[b['up']])[grasp_target(grasp, self.cfg)[1]])
        return self.finger_rects(gx, gy, close_axis, self.open_pick[grasp] / 2), self.top_of(b) - self.depth

    def tried(self, failed, b, grasp, dyaw, s):
        """같은 블록(위치 SAME_M 안) · 같은 잡기 · 각도 · 옮김으로 이미 실패했나. failed = [(x_m, y_m, 잡기, 각도 °, 옮김 m)]."""
        return any(math.hypot(b['x_m'] - f[0], b['y_m'] - f[1]) < self.SAME_M and tuple(f[2:5]) == (grasp, dyaw, s) for f in failed)

    @staticmethod
    def method_of(dyaw, s):
        """잡는 방법 이름 — 곧게 · 틀기 · 옮기기."""
        return '곧게' if dyaw == 0 and s == 0.0 else ('틀기' if s == 0.0 else '옮기기')

    def rank(self, blocks, failed=(), pts=None):
        """집기 시도 후보를 해 볼 순서대로 [(번호, 잡기, 각도 °, 옮김 m, 꼬리표)]와 거른 수 {'깊이': n, '블록': {번호(1부터)}}.

        거르기: 덮인 블록 · 값이 틀린 블록(BlockPicker.is_valid) · 더미 위(overlap top)인데 눕힘이 아니거나 확신도가 STACK_CONF_MIN 미만인 블록 ·
        실패한 시도(failed) · 기울어진 옆으로 선 · 선 블록 ·
        center_top_z_m 이 없거나 손가락 자리가 막힌 기울어진 블록 · pts 가 있으면 깊이로 손가락 자리가 막힌 후보.
        잡기 'TILT' = 기울어진 눕힌 블록을 공구를 기울여 좁은 폭으로 집기. blocks 가 목록이 아니면 ValueError(부르는 쪽 실수).
        """
        if not isinstance(blocks, list):
            raise ValueError(f'blocks 가 목록이 아니다: {type(blocks).__name__}')
        rows = []
        cut = {'깊이': 0, '블록': set()}
        for i, b in enumerate(blocks):
            if not BlockPicker.is_valid(b) or b.get('overlap') == 'under':
                continue
            if b.get('overlap') == 'top' and (b['up'] != 'THICKNESS' or float(b.get('confidence', 1.0)) < self.STACK_CONF_MIN):
                continue                  # 더미 위 블록은 눕힘 · 확신도 STACK_CONF_MIN 이상만 — 자세를 잘못 보면 손 안 위치가 틀려 놓을 때 높이 어긋남
            if b.get('tilted'):
                if not (b['up'] == 'THICKNESS' and 'center_top_z_m' in b and 'tilt_tip_z_m' in b and not self.tilt_fingers_hit(blocks, i)):
                    continue
                opts = [('TILT', 0, 0.0)]
            else:
                opts = self.grasp_options(blocks, i)
            if not opts:
                continue
            lay, fr, cov = self.layer_of(b), self.freed(blocks, i), self.covers(blocks, i)
            for grasp, dyaw, s in opts:
                if self.tried(failed, b, grasp, dyaw, s):
                    continue
                if pts is not None:
                    fingers, tip = self.candidate_fingers(b, grasp, dyaw, s)
                    if self.fingers_blocked(fingers, tip, pts):
                        cut['깊이'] += 1
                        cut['블록'].add(i + 1)
                        continue
                if grasp == 'FLAT_SHORT' and abs(s) >= 0.005:         # 큰 옮김 — p1 에 평균 24 · 최대 36.7 mm 비켜 놓였다(FLAT_LONG ±2.4 mm 는 3 ~ 4 mm)
                    tier = 6
                elif cov and fr >= self.COVER_MIN_FREE:
                    tier = 1
                elif grasp == 'TILT':
                    tier = 4
                elif b['up'] == 'THICKNESS':
                    tier = 2 if dyaw == 0 else 3
                else:
                    tier = 5
                method = 2 if grasp == 'TILT' else (0 if dyaw == 0 else 1)
                key = (tier, -lay, -fr, method, 0 if grasp == 'TILT' else self.GRASP_ORDER.index(grasp), abs(dyaw), abs(s), -self.gap_of(b, grasp))
                tag = (f'{tier}칸 {TIER_NAMES[tier]} · 층{lay} · 열림{fr} · {POSE_KO[b["up"]]}{" 기울어짐" if grasp == "TILT" else ""} · {grasp} · '
                       f'{"기울여 집기" if grasp == "TILT" else self.method_of(dyaw, s)} · 틈 {self.gap_of(b, grasp):.0f} mm')
                rows.append((key, i, grasp, dyaw, s, tag))
        rows.sort(key=lambda r: r[0])
        return [r[1:] for r in rows], cut


def _footprint(center, quat, size):
    """블록(중심 m · 쿼터니언 xyzw · 실제 치수 (L, W, T))을 위에서 본 사각형 네 꼭짓점 — 수평에 가까운 두 축."""
    x, y, z, w = quat
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    cols = sorted(range(3), key=lambda k: abs(R[2, k]))[:2]
    c = np.array(center[:2], dtype=float)
    u, v = R[:2, cols[0]] * size[cols[0]] / 2, R[:2, cols[1]] * size[cols[1]] / 2
    return [tuple(c + a * u + b * v) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def spread_parts(parts, cfg, gap_m=0.001):
    """같은 층(중심 높이 차 1 mm 안)에서 앞 순서 블록과 틈이 gap_m 보다 좁은 블록의 놓는 중심을 그 블록에서 멀어지는 쪽으로 모자란 만큼 옮긴다.

    설계는 블록 25 mm 간격인데 실제 폭이 24.8 mm 라 틈이 0.2 mm 뿐 — 놓을 때 옆 블록을 밀어 줄이 틀어졌다(10/13 실기 레시피 4 바닥, 박진용 1 mm).
    입력: parts = 레시피 순서대로 [{'part_id', 'center': (m), 'quat': (x, y, z, w)}](motion_math.recipe_parts 꼴), cfg = robot.yaml dict
    (block_actual_m · 없으면 block_size_m). 앞에서 옮긴 자리를 기준으로 차례로 쌓아 간다. 크게 겹치게 설계된 것(3 mm 넘게)은 건드리지 않는다.
    반환: 블록마다 놓을 중심 [x, y, z] m (입력은 바꾸지 않는다 — 확인 · 기록은 설계 중심 그대로 쓴다).
    """
    size = tuple(float(v) for v in cfg.get('block_actual_m') or cfg['block_size_m'])
    out = []
    for k, p in enumerate(parts):
        c = np.array(p['center'], dtype=float)
        for j in range(k):
            if abs(out[j][2] - c[2]) > 0.001:
                continue
            pa, pb = _footprint(out[j], parts[j]['quat'], size), _footprint(c, p['quat'], size)
            best = None
            for poly in (pa, pb):                      # 분리축: 두 사각형의 변 법선 네 개 중 가장 많이 떨어진 축
                for e in range(4):
                    d = np.array(poly[(e + 1) % 4]) - np.array(poly[e])
                    n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
                    if n @ (c[:2] - out[j][:2]) < 0:
                        n = -n
                    gap = min(n @ np.array(q) for q in pb) - max(n @ np.array(q) for q in pa)
                    if best is None or gap > best[0]:
                        best = (gap, n)
            gap, n = best
            if -0.003 < gap < gap_m:                   # 앞 블록을 옮겨 생긴 겹침(0.6 mm)까지 — 크게 겹치게 설계된 것은 건드리지 않는다
                c[:2] += n * (gap_m - gap)
        out.append(c)
    return [list(map(float, c)) for c in out]

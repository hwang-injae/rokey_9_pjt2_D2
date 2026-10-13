"""층 지도 + 정수계획 스캔 추론 계산 (W115 확정 판 v12g, 10/12 민범진) — ROS · cv2 없음(numpy · scipy 만, CI 에서도 돈다).

StructureScanner.infer 가 부른다. 입력 = 자세마다 (pose_id, 깊이 uint16 mm(컬러 정렬), 내부 파라미터, T_base2cam(m)) + 자세마다 YOLO 마스크.
출력 = 블록 목록(설계 좌표 mm = base mm − 조립 원점, z = 아랫면) — 가운데 맞춤은 부르는 쪽이 한다.

1) 자세마다 작업면 평면을 z = 0 으로 맞추고 높이 × 설계 두께 / 실측 두께. 윗면 자국 겹침(IoU ≥ 0.3)으로 자세 사이 x · y 를 맞춤.
2) 점을 면 방향으로 나눔: 윗면(수평) 점 → 그 윗면 아래 층 · 옆면(수직) 점 → 그 높이의 층(블록 안쪽으로 1.5 mm). 8 mm 아래 점은 버림.
3) 층마다 2.5 mm 칸 지도: O(점 있음) · F(카메라 빛이 그 층 높이로 지나감 = 빈 곳, O 둘레 1칸은 '모름') · 그늘(위 층에 O) · 모름.
4) 후보 = 층마다 눕힌 블록(x · y) + 세운 블록(zx · zy), 모서리는 12.5 mm 격자(위상 = YOLO 윗면 모서리)에만.
   점수 = O 칸 +1 − 안쪽 F 칸 × 2 + 그늘 칸 × 0.005 − 블록 값 6 ± YOLO 윗면 일치 · 어긋남 40.
5) 정수계획(scipy milp/HiGHS, 결정적): 점수 최대 · 겹침 없음 · 받침(눕힌 블록은 긴 축 두 반쪽 밑에 각각, 세운 블록은 밑에 하나).
   근거 없이 받침으로만 들어간 블록 = inferred(가려진 블록).
6) 윗면 근거 없는 75×75 판(같은 방향 3개)은 위(없으면 아래) 층과 엇갈리게 돌림(젠가 규칙).
연구 시제품 simple2.py(저장소 밖, W166 자료)의 v12g 설정(ALIGN 'old' · FACE_CLUSTER 끔 · FREE_GUARD 1)만 옮겼다.
cv2 함수는 scipy 로 바꿨다(erode · dilate = minimum · maximum_filter(mode='nearest'), 연결 성분 = ndimage.label(3×3),
minAreaRect = 볼록 껍질 회전 캘리퍼스) — 실측 4장면 × 5 · 3자세에서 시제품과 블록이 같음을 확인했다.
아래 조정값은 블록 75 × 25 × 15 mm 기준이다(블록 크기가 바뀌면 다시 맞춰야 한다).
"""
import math
import time

import numpy as np
from scipy import ndimage
from scipy.optimize import Bounds, LinearConstraint, milp
from scipy.sparse import coo_matrix
from scipy.spatial import ConvexHull, QhullError

V = 2.5                 # 칸 mm (블록 모서리 ±37.5 · ±12.5 가 칸 경계에 오게)
FLAT = ('x', 'y')
STAND = ('zx', 'zy')    # 옆세움(xe · ye)은 후보에 없다 — 기본 설계 4개에 없어서(시제품과 같음)
W_O, W_F, W_BLOCK, W_FACE = 1.0, 2.0, 6.0, 40.0
RING = 1                # 눕힌 블록 바닥 가장자리 몇 칸의 F 는 안 셈(실물이 설계보다 작음 + 자세 맞춤 오차)
W_SH = 0.005            # 위 층 점 아래 모르는 칸 — 블록 하나(300 칸) 합해도 블록 값보다 작게: 같은 점수일 때 '위 블록 바로 밑'을 고르는 정도
MIN_Z = 8.0             # 작업면 번짐(6~12 mm 가짜 점) — 이 아래 점은 버림
FREE_GUARD = 1          # O 칸 둘레 이만큼(칸)의 빈 칸 증거는 버림 — 자세 오차(1~3 mm)로 블록 가장자리가 깎여 세운 다리가 빠지던 것(003)을 막음
ALIGN_IOU = 0.3         # 자세 맞춤: 윗면 자국 IoU 가 이보다 작으면 옮기지 않음
ALIGN_R_MM = 20.0       # 자세 맞춤 찾는 범위 ± mm
LAT = 12.5              # 블록 모서리 격자 mm(75 · 25 의 공약수 — 15 는 한쪽 모서리만 격자)
SUPPORT_CELLS = 20      # 받침으로 칠 겹친 칸 수(5 × 4 칸 = 12.5 × 10 mm)
INFER_O = 15            # 근거(O 칸)가 이보다 적으면 inferred
MIN_POINTS = 200        # 작업면 위 점이 이보다 적으면 풀지 않는다(구조물을 못 찾음)
MILP_TIME_S = 2.0       # 정수계획 시간 제한(실측 ≤ 0.42 s)
EXPLAIN_PAD = 2         # 점 설명률: 블록 상자를 x · y 로 이만큼(칸 = 5 mm) 넓혀 셈


def ori_extents(block_mm):
    """블록 크기(길이 · 폭 · 두께, mm) → 방향 코드마다 (x · y · z 방향 길이). d2_task.recipe_to_parts.ori_extents 와 같은 표. 바깥 영향 없음."""
    L, W, T = block_mm
    return {'x': (L, W, T), 'y': (W, L, T), 'xe': (L, T, W), 'ye': (T, L, W), 'zx': (T, W, L), 'zy': (W, T, L)}


def _min_area_rect(pts):
    """2D 점들을 감싸는 가장 작은 회전 사각형(볼록 껍질 변마다 돌려 보는 회전 캘리퍼스 — cv2.minAreaRect 대신).

    입력: pts (N, 2) mm. 출력: (가운데 x, 가운데 y, 긴 변, 짧은 변, 긴 변 방향 deg ∈ [0, 180)) 또는 None(점이 한 줄 위 · 3개 미만).
    바깥 영향 없음. 입력은 cv2 와 같게 float32 로 한 번 자른 뒤 계산한다.
    """
    p = np.asarray(pts, np.float32).astype(np.float64)
    try:
        h = p[ConvexHull(p).vertices]
    except (QhullError, ValueError):  # 점이 모자라거나 한 줄 위 — 윗면으로 쓸 수 없다
        return None
    best = None
    for e in h - np.roll(h, 1, 0):
        n = math.hypot(e[0], e[1])
        if n < 1e-9:
            continue
        u = e / n
        a, b = h @ u, h @ np.array([-u[1], u[0]])
        w, d = a.max() - a.min(), b.max() - b.min()
        if best is None or w * d < best[0] - 1e-9:
            c = u * (a.max() + a.min()) / 2 + np.array([-u[1], u[0]]) * (b.max() + b.min()) / 2
            ang = math.degrees(math.atan2(u[1], u[0])) % 180.0
            best = (w * d, float(c[0]), float(c[1]), w, d, ang if w >= d else (ang + 90.0) % 180.0)
    if best is None:
        return None
    _, cx, cy, w, d, ang = best
    return cx, cy, float(max(w, d)), float(min(w, d)), ang


def _ov(d, i0, i1, j0, j1):
    """후보 d 바닥 사각형과 칸 범위 [i0, i1) × [j0, j1) 의 겹친 칸 수."""
    return max(0, min(i1, d['i0'] + d['a']) - max(i0, d['i0'])) * max(0, min(j1, d['j0'] + d['b']) - max(j0, d['j0']))


class _Grid:
    """구조물 둘레 칸 격자(설계 좌표 mm). lo = (x0, y0) mm, nx · ny 칸, nl 층, lh = 층 높이 mm. O · F = (nl, nx, ny) 점 수 · 빛 수."""

    def __init__(self, lo, nx, ny, nl, lh):
        """빈 격자를 만든다. 바깥 영향 없음."""
        self.lo, self.nx, self.ny, self.nl, self.lh = lo, nx, ny, nl, lh
        self.O = np.zeros((nl, nx, ny))
        self.F = np.zeros((nl, nx, ny))

    def cells(self, xy):
        """xy (N, 2) mm → (칸 번호 (N, 2), 격자 안 표시)."""
        i = np.floor((xy - self.lo) / V).astype(int)
        ok = (i[:, 0] >= 0) & (i[:, 1] >= 0) & (i[:, 0] < self.nx) & (i[:, 1] < self.ny)
        return i, ok

    def hits(self, v):
        """한 자세의 점을 O 에 더한다. 윗면 점(|nz| > 0.85, 층 경계 ± 4 mm) → 그 아래 층 · 옆면 점(|nz| < 0.35) → 그 높이 층, 블록 안쪽으로 1.5 mm."""
        lh = self.lh
        q, n = v['q'][v['ok']], v['n'][v['ok']]
        keep = q[:, 2] > MIN_Z
        q, n = q[keep], n[keep]
        top = np.abs(n[:, 2]) > 0.85
        kt = np.round(q[:, 2] / lh).astype(int) - 1
        top &= (np.abs(q[:, 2] - (kt + 1) * lh) < 4) & (kt >= 0)
        side = np.abs(n[:, 2]) < 0.35
        ks = np.floor(q[:, 2] / lh).astype(int)
        fr = q[:, 2] - ks * lh
        side &= (fr > 2) & (fr < lh - 2)          # 층 경계 2 mm 안 옆면 점은 어느 층인지 흔들려 뺀다
        nh = n[:, :2] / (np.linalg.norm(n[:, :2], axis=1, keepdims=True) + 1e-9)
        for sel, k, xy in ((top, kt, q[:, :2]), (side, ks, q[:, :2] - 1.5 * nh)):
            i, ok = self.cells(xy[sel])
            kk = k[sel]
            ok &= (kk < self.nl)
            np.add.at(self.O, (kk[ok], i[ok, 0], i[ok, 1]), 1)

    def carve(self, v):
        """빛이 층 가운데 띠(층 바닥 + 2 ~ 13 mm)를 지나간 칸에 F 를 더한다(빛 2 px 간격, 1.25 mm 걸음 — 칸당 약 2번이라 0.5 씩)."""
        lh = self.lh
        sl = (slice(None, None, 2), slice(None, None, 2))
        m = v['valid'][sl]
        q, Lc, L, cam = v['q'][sl][m], v['Lc'][sl][m], v['L'][sl][m], v['cam']
        dv = (q - cam) / L[:, None]
        lo = np.array([self.lo[0], self.lo[1], 0.0])
        hi = np.array([self.lo[0] + self.nx * V, self.lo[1] + self.ny * V, self.nl * lh])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (lo - cam) / dv, (hi - cam) / dv
        tmin = np.nanmax(np.where(np.isnan(np.minimum(t1, t2)), -np.inf, np.minimum(t1, t2)), 1)
        tmax = np.nanmin(np.where(np.isnan(np.maximum(t1, t2)), np.inf, np.maximum(t1, t2)), 1)
        t0 = np.maximum(tmin, 0)
        te = np.minimum(tmax, Lc)
        k = te > t0
        dv, t0, te = dv[k], t0[k], te[k]
        if len(t0) == 0:
            return
        ts = np.arange(int(np.ceil((te - t0).max() / 1.25)) + 1) * 1.25
        for a in range(0, len(t0), 6000):             # 6000 빛씩 — 메모리를 수백 MB 안으로
            tt = t0[a:a + 6000, None] + ts[None]
            mm = tt < te[a:a + 6000, None]
            p = cam + tt[mm][:, None] * np.repeat(dv[a:a + 6000], mm.sum(1), 0)
            kz = np.floor(p[:, 2] / lh).astype(int)
            fr = p[:, 2] - kz * lh
            sel = (fr > 2) & (fr < lh - 2) & (kz >= 0) & (kz < self.nl)
            p, kz = p[sel], kz[sel]
            i, ok = self.cells(p[:, :2])
            np.add.at(self.F, (kz[ok], i[ok, 0], i[ok, 1]), 0.5)

    def score(self):
        """칸 점수 S (nl, nx, ny): O(점 ≥ 2 · 점 ≥ 빛/3) = +1 · F(빛 ≥ 2 · 빛 ≥ 3 × 점, O 둘레 FREE_GUARD 칸 밖) = −2 ·
        위 층 어딘가에 O 가 있는 모르는 칸 = +W_SH · 나머지 0. 출력: (S, occ, shadow)."""
        occ = (self.O >= 2) & (self.O * 3 >= self.F)
        free = (self.F >= 2) & (self.F >= 3 * self.O)
        g = 2 * FREE_GUARD + 1
        free &= ~ndimage.maximum_filter(occ, size=(1, g, g), mode='nearest')
        shadow = np.zeros_like(occ)
        for k in range(self.nl - 2, -1, -1):
            shadow[k] = shadow[k + 1] | occ[k + 1]
        S = np.where(occ, W_O, np.where(free, -W_F, np.where(shadow, W_SH, 0.0)))
        return S, occ, shadow


class LayerSolver:
    """여러 자세의 깊이 + YOLO 마스크 → 블록 목록(층 지도 + 정수계획, 확정 판 v12g). ROS · 로봇 없음, 바깥 영향 없음(계산만).

    입력: block_mm = 설계 블록 (길이, 폭, 두께) mm · actual_mm = 실측 블록 크기 mm(두께로 높이를 늘림) · half_mm = 조립 원점 둘레 ± mm(robot.yaml
          assembly_area_half_m). 상태를 남기지 않아 한 객체를 여러 스캔에 써도 된다.
    """

    def __init__(self, block_mm, actual_mm, half_mm):
        """블록 크기 · 층 높이 · 높이 늘림 비율 · 관심 영역을 mm 로 둔다."""
        self.block_mm = tuple(float(v) for v in block_mm)
        self.ext = ori_extents(self.block_mm)
        self.lh = self.block_mm[2]                       # 층 높이 = 설계 두께(15 mm)
        self.zscale = self.lh / float(actual_mm[2])      # 실측 두께(14.8) → 설계 두께 층에 맞춤
        self.roi = float(half_mm)

    # ------------------------------------------------------------------ 전체
    def solve(self, views, origin_mm, yaw_deg, masks):
        """자세들 → (블록 목록, 진단 dict).

        입력: views = [(pose_id, depth (H, W) mm uint16(0 = 없음), {'fx','fy','ppx','ppy'} px, T_base2cam 4×4 m)] ·
              origin_mm = 조립 원점 (x, y, z) mm(base_link) · yaw_deg = 원점 yaw · masks = 자세마다 (H, W) bool 마스크 목록(없으면 []).
        출력: blocks = [{'x','y','z'(mm, 0.1 반올림 float),'ori','inferred'(bool)}] (z · x · y 순) ·
              diag = {'points','shifts','faces','faces_per_view','candidates','milp_status','milp_no_solution','solve_ms','explained','layers'}.
        실패: 작업면 위 점이 MIN_POINTS 보다 적거나 덩어리가 없으면 블록 [] (diag['points'] 로 이유를 안다). 정수계획 해가 없으면
              블록 [] + diag['milp_no_solution'] True. 예외를 내지 않는 것이 목표지만, 입력 모양이 틀리면 numpy 예외가 그대로 난다.
        """
        t0 = time.monotonic()
        diag = {'points': 0, 'shifts': [(0.0, 0.0)] * len(views), 'faces': 0, 'faces_per_view': [0] * len(views), 'candidates': 0,
                'milp_status': None, 'milp_no_solution': False, 'solve_ms': 0.0, 'explained': 0.0, 'layers': 0}
        origin_mm = np.asarray(origin_mm, float)
        vs = [self._prep(d, ii, T, origin_mm, yaw_deg) for _, d, ii, T in views]
        pts = [self._roi_points(v) for v in vs]
        diag['points'] = int(sum(len(p) for p in pts))
        if diag['points'] < MIN_POINTS:
            diag['solve_ms'] = round((time.monotonic() - t0) * 1000, 1)
            return [], diag
        diag['shifts'] = self._align(vs, pts)
        G = self._grid(vs, diag)
        if G is None:
            diag['solve_ms'] = round((time.monotonic() - t0) * 1000, 1)
            return [], diag
        S, occ, shadow = G.score()
        faces = []
        for i, (v, ms) in enumerate(zip(vs, masks)):
            f = self._top_faces(v, ms)
            diag['faces_per_view'][i] = len(f)
            faces += f
        diag['faces'] = len(faces)
        cs = self._candidates(G, S, faces, occ | shadow)
        diag['candidates'] = len(cs)
        chosen, status, no_x = self._milp(G, cs)
        diag['milp_status'], diag['milp_no_solution'] = status, no_x
        blocks = self._fix_slabs([self._to_block(G, d) for d in chosen], faces)
        blocks.sort(key=lambda b: (b['z'], b['x'], b['y']))
        blocks = [{'x': float(round(b['x'], 1)), 'y': float(round(b['y'], 1)), 'z': float(round(b['z'], 1)), 'ori': b['ori'],
                   'inferred': bool(b['inferred'])} for b in blocks]
        diag['explained'] = self._explained(G, occ, blocks)
        diag['layers'] = int(G.nl)
        diag['solve_ms'] = round((time.monotonic() - t0) * 1000, 1)
        return blocks, diag

    # ------------------------------------------------------------------ 1) 자세 준비
    def _prep(self, depth, ii, T, origin_mm, yaw_deg):
        """깊이 한 장 → 설계 좌표 점(H, W, 3) mm · 법선 · 쓸 점 표시 · 빛 길이 · 카메라 위치(작업면 맞춤 · 높이 늘림 뒤) dict."""
        R = self.roi
        d = np.asarray(depth).astype(np.float64)
        h, w = d.shape
        v, u = np.mgrid[0:h, 0:w]
        fx, fy, ppx, ppy = (float(ii[k]) for k in ('fx', 'fy', 'ppx', 'ppy'))
        pc = np.stack([(u - ppx) / fx * d, (v - ppy) / fy * d, d], -1).reshape(-1, 3) / 1000.0
        T = np.asarray(T, float)
        c, s = math.cos(math.radians(-yaw_deg)), math.sin(math.radians(-yaw_deg))
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        q = ((pc @ T[:3, :3].T + T[:3, 3]) * 1000.0 - origin_mm) @ Rz.T
        cam = Rz @ (T[:3, 3] * 1000.0 - origin_mm)
        valid = d.reshape(-1) > 0
        # 작업면 평면: 원점 둘레 |z| < 10 mm 점에 z = ax + by + c (3 mm 밖 빼며 3번) — 손목 보정 기울기 · 높이 어긋남을 자세마다 지운다
        m = valid & (np.abs(q[:, 0]) < R) & (np.abs(q[:, 1]) < R) & (np.abs(q[:, 2]) < 10)
        p = q[m][::4]
        coef = np.zeros(3)
        for _ in range(3):
            if len(p) < 3:                        # 작업면이 안 보임 — 맞추지 않고 그대로(점 수 검사에서 걸러진다)
                break
            A = np.c_[p[:, 0], p[:, 1], np.ones(len(p))]
            coef = np.linalg.lstsq(A, p[:, 2], rcond=None)[0]
            p = p[np.abs(p[:, 2] - A @ coef) < 3.0]
        a, b, c0 = coef
        q[:, 2] = (q[:, 2] - (a * q[:, 0] + b * q[:, 1] + c0)) * self.zscale
        cam = cam.copy()
        cam[2] = (cam[2] - (a * cam[0] + b * cam[1] + c0)) * self.zscale
        q = q.reshape(h, w, 3)
        valid = valid.reshape(h, w)
        # 깊이 튐(5×5 범위 > 8 mm) = 모서리 번짐 → 맞은 점으로 안 씀
        big = np.where(d > 0, d, 1e9).astype(np.float32)
        small = np.where(d > 0, d, -1e9).astype(np.float32)
        dmin = ndimage.minimum_filter(big, size=5, mode='nearest')
        flying = (ndimage.maximum_filter(small, size=5, mode='nearest') - dmin) > 8.0
        gx = np.zeros_like(q)
        gy = np.zeros_like(q)
        gx[:, 2:-2] = q[:, 4:] - q[:, :-4]
        gy[2:-2] = q[4:] - q[:-4]
        n = np.cross(gx, gy)
        n /= np.linalg.norm(n, axis=-1, keepdims=True) + 1e-9
        ray = q - cam
        L = np.linalg.norm(ray, axis=-1) + 1e-9
        n *= -np.sign((n * ray).sum(-1, keepdims=True) + 1e-12)       # 법선을 카메라 쪽으로
        cosg = np.abs((n * ray).sum(-1)) / L
        ok = valid & ~flying & (cosg > 0.25)                         # 빛과 거의 나란한 면(스치는 점)은 깊이가 흔들려 뺀다
        # 빈 곳 칠할 길이: 5×5 중 가장 가까운 깊이까지, 그 앞 5 mm 에서 멈춤(모서리 번짐이 블록을 깎지 않게)
        Lc = L * np.where(valid, np.minimum(dmin, d) / np.maximum(d, 1), 0) - 5.0
        return dict(q=q, n=n, ok=ok, valid=valid, L=L, Lc=Lc, cam=cam)

    def _roi_points(self, v):
        """쓸 점 중 원점 둘레 ± roi · 작업면 위 10 ~ 300 mm 점 (N, 3) mm."""
        R = self.roi
        p = v['q'][v['ok']]
        return p[(np.abs(p[:, 0]) < R) & (np.abs(p[:, 1]) < R) & (p[:, 2] > 10) & (p[:, 2] < 300)]

    def _align(self, views, pts):
        """윗면 자국(가장 높은 면 ± 8 mm)을 가장 내려다보는 자세에 x · y 로 맞춘다(± 20 mm, 1.25 mm). IoU < 0.3 이면 그대로.
        views 의 q · cam 을 제자리에서 옮긴다. 출력: 자세마다 이동 [(dx, dy)] mm."""
        R = self.roi
        cell = 1.25
        nb = int(2 * R / cell)

        def steep(i):
            """카메라가 구조물을 내려다보는 정도(수직이면 1)."""
            d = views[i]['cam'] - pts[i].mean(0)
            return d[2] / np.linalg.norm(d)
        have = [i for i in range(len(views)) if len(pts[i])]
        ref = max(have, key=steep)
        zt = np.percentile(pts[ref][:, 2], 98)

        def mask(p):
            """윗면 높이 점 → 1.25 mm 자국(닫힘 5×5 로 구멍 메움)."""
            t = p[np.abs(p[:, 2] - zt) < 8]
            m = np.zeros((nb, nb), bool)
            m[((t[:, 0] + R) / cell).astype(int).clip(0, nb - 1), ((t[:, 1] + R) / cell).astype(int).clip(0, nb - 1)] = True
            m = ndimage.maximum_filter(m, size=5, mode='nearest')
            return ndimage.minimum_filter(m, size=5, mode='nearest')
        mref = mask(pts[ref])
        shifts = []
        r = int(ALIGN_R_MM / cell)
        for i, v in enumerate(views):
            mi = mask(pts[i])
            if i == ref or mi.sum() < 50 or mref.sum() < 50:
                shifts.append((0.0, 0.0))
                continue
            best, bi = 0.0, (0, 0)
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    sh = np.roll(np.roll(mi, dx, 0), dy, 1)
                    iou = (sh & mref).sum() / float((sh | mref).sum())
                    if iou > best + 1e-9 or (abs(iou - best) < 1e-9 and dx * dx + dy * dy < bi[0] ** 2 + bi[1] ** 2):
                        best, bi = iou, (dx, dy)
            if best < ALIGN_IOU:
                shifts.append((0.0, 0.0))
                continue
            dd = np.array([bi[0] * cell, bi[1] * cell, 0.0])
            v['q'] = v['q'] + dd
            v['cam'] = v['cam'] + dd
            shifts.append((float(dd[0]), float(dd[1])))
        return shifts

    # ------------------------------------------------------------------ 2~3) 층 지도
    def _grid(self, views, diag):
        """모든 자세 점 → 큰 덩어리(가장 큰 것의 10 % 이상, 5 mm 칸 3점 이상) 둘레 + 30 mm 격자를 만들고 O · F 를 채운다.
        diag['points'] 를 맞춘 뒤 점 수로 고친다. 출력: _Grid 또는 None(덩어리 없음)."""
        R = self.roi
        pts = np.concatenate([self._roi_points(v) for v in views])
        diag['points'] = int(len(pts))
        cell = 5.0
        nb = int(2 * R / cell)
        ix = ((pts[:, 0] + R) / cell).astype(int).clip(0, nb - 1)
        iy = ((pts[:, 1] + R) / cell).astype(int).clip(0, nb - 1)
        cnt = np.zeros((nb, nb), np.int32)
        np.add.at(cnt, (ix, iy), 1)
        img = ndimage.maximum_filter(cnt >= 3, size=3, mode='nearest')
        lab, nlab = ndimage.label(img, structure=np.ones((3, 3), int))
        if nlab == 0:
            return None
        tot = {k: cnt[lab == k].sum() for k in range(1, nlab + 1)}
        best = max(tot, key=tot.get)
        keep = [k for k in tot if tot[k] >= 0.1 * tot[best]]      # 떨어진 기둥 2개 등도 함께(케이블 · 부스러기는 뺌)
        p = pts[np.isin(lab[ix, iy], keep)]
        lo = np.floor((np.percentile(p[:, :2], 0.5, 0) - 30) / V) * V
        hi = np.ceil((np.percentile(p[:, :2], 99.5, 0) + 30) / V) * V
        nx, ny = int(round((hi[0] - lo[0]) / V)), int(round((hi[1] - lo[1]) / V))
        nl = int(math.ceil((np.percentile(p[:, 2], 99.8) + 5) / self.lh))
        G = _Grid(lo, nx, ny, nl, self.lh)
        for v in views:
            G.hits(v)
            G.carve(v)
        return G

    # ------------------------------------------------------------------ YOLO 윗면
    def _top_faces(self, v, ms):
        """한 자세의 마스크 → 윗면 관측 dict(x, y, z, along('x'|'y'), L) 목록. 가장자리 2 px 깎음 · 마스크 점의 35 % 이상이 가장 높은 면 ± 4 mm 일 때만.
        축에서 15° 넘게 돈 면 · 긴 변 60 mm 미만(가려진 면) · 짧은 변 35 mm 초과는 버림."""
        out = []
        for m in ms:
            m = ndimage.minimum_filter(np.asarray(m, bool), size=5, mode='nearest')
            P = v['q'][m & v['ok']]
            if len(P) < 60:
                continue
            zt = np.percentile(P[:, 2], 95)
            top = P[P[:, 2] > zt - 4]
            if len(top) < 0.35 * len(P) or len(top) < 50:
                continue
            r = _min_area_rect(top[:, :2])
            if r is None:
                continue
            cx, cy, Lr, Sr, ang = r
            Lr, Sr = Lr + 3, Sr + 3                  # 마스크 가장자리 2 px 깎음 · 실물이 설계보다 작은 몫을 되돌림
            if min(ang % 90, 90 - ang % 90) > 15 or Lr < 60 or Sr > 35:
                continue
            out.append(dict(x=cx, y=cy, z=float(np.median(top[:, 2])), L=Lr, along='x' if (ang < 45 or ang > 135) else 'y'))
        return out

    # ------------------------------------------------------------------ 4) 후보
    def _lattice_phase(self, G, S, faces):
        """격자 위상(x, y) mm ∈ [0, 12.5): YOLO 윗면 모서리들(가운데 ± 반 길이)의 원형 평균, 2.5 mm 로 반올림.
        윗면이 없으면 가장 위 층 O 외곽의 모서리로."""
        hl, hw = self.block_mm[0] / 2, self.block_mm[1] / 2
        ph = []
        for ax in (0, 1):
            e = []
            for f in faces:
                half = hl if (f['along'] == 'x') == (ax == 0) else hw
                c = f['x'] if ax == 0 else f['y']
                e += [c - half, c + half]
            if not e:
                cnts = [(S[k] >= W_O).sum() for k in range(G.nl)]
                if max(cnts) == 0:
                    ph.append(0.0)
                    continue
                ks = [k for k in range(G.nl) if cnts[k] > 50]
                k = max(ks) if ks else int(np.argmax(cnts))
                idx = np.nonzero((S[k] >= W_O).any(1 - ax))[0]
                e = [G.lo[ax] + idx.min() * V, G.lo[ax] + (idx.max() + 1) * V]
            a = np.angle(np.mean(np.exp(2j * np.pi * np.array(e) / LAT))) / (2 * np.pi) * LAT
            ph.append(float(round((a % LAT) / V) * V % LAT))
        return ph

    @staticmethod
    def _on_lattice(G, ax, n, size, ph):
        """칸 시작 번호 0..n-1 중 블록 한쪽 모서리가 격자(ph + 12.5 m)에 오는 것(크기가 12.5 배수가 아니면 반대쪽 모서리도)."""
        x0 = G.lo[ax] + np.arange(n) * V
        ok = np.abs(((x0 - ph + LAT / 2) % LAT) - LAT / 2) < 0.1
        if size % LAT:
            ok |= np.abs(((x0 + size - ph + LAT / 2) % LAT) - LAT / 2) < 0.1
        return ok

    def _face_term(self, o, cx, cy, ztop, faces):
        """YOLO 윗면과 후보 윗면(높이 ztop mm)의 일치 +W_FACE · 어긋남 −W_FACE(같은 높이에서 30 % 넘게 겹치는데 방향 · 자리가 다름 —
        이음매가 다르다). 윗면 사각형은 관측 가운데 + 방향으로 놓은 길이 × 폭."""
        ex, ey, _ = self.ext[o]
        bl, bw = self.block_mm[0], self.block_mm[1]
        sc = 0.0
        for f in faces:
            if abs(f['z'] - ztop) >= 4:
                continue
            if o == f['along'] and abs(f['x'] - cx) < 5 and abs(f['y'] - cy) < 5:
                sc += W_FACE
                continue
            fx, fy = (bl / 2, bw / 2) if f['along'] == 'x' else (bw / 2, bl / 2)
            ix = min(cx + ex / 2, f['x'] + fx) - max(cx - ex / 2, f['x'] - fx)
            iy = min(cy + ey / 2, f['y'] + fy) - max(cy - ey / 2, f['y'] - fy)
            if ix > 0 and iy > 0 and ix * iy >= 0.3 * min(ex * ey, bl * bw):
                sc -= W_FACE
        return sc

    def _candidates(self, G, S, faces, hull):
        """후보 블록 dict(o, k(바닥 층), nl(층 수), i0, j0, a, b (칸), s(점수), no(O 칸 수), seen). 눕힌 블록 = 층마다, 세운 블록 = 기둥.
        모서리는 격자 위만. 점수 = 덮는 칸 점수 합 − 블록 값 + 윗면 일치 보너스. 근거 없는 후보도 둠(받침용) — 단
        F 칸이 15 % 넘거나, 구조물 안(O 이거나 위에 O) 칸이 절반 미만이면 뺌.
        세운 블록은 '보임'(층 셋 이상에 O 칸 4 개 이상)이거나, 보이는 세운 블록의 거울 자리(구조물 가운데 기준 x · y · 둘 다)일 때만 둠
        — 안 보이는 기둥 속은 눕힌 블록 쌓기가 흔하고(004), 가구 다리는 대칭으로 놓인다(003)."""
        cs = []
        Sc = np.pad(np.maximum(S, 0), ((0, 0), (1, 0), (1, 0))).cumsum(1).cumsum(2)    # O · 그늘 점수(빈 칸 벌점은 아래 안쪽만)
        Oc = np.pad((S >= W_O).astype(float), ((0, 0), (1, 0), (1, 0))).cumsum(1).cumsum(2)
        Fc = np.pad((S <= -W_F).astype(float), ((0, 0), (1, 0), (1, 0))).cumsum(1).cumsum(2)
        Hc = np.pad(hull.astype(float), ((0, 0), (1, 0), (1, 0))).cumsum(1).cumsum(2)

        def box(C, k, a, b):
            """k 층 칸 [i0, i0+a) × [j0, j0+b) 합 — 모든 i0, j0 한 번에."""
            return C[k, a:, b:] - C[k, :-a, b:] - C[k, a:, :-b] + C[k, :-a, :-b]
        ph = self._lattice_phase(G, S, faces)
        for o in FLAT + STAND:
            ex, ey, ez = self.ext[o]
            a, b, nlay = int(ex / V), int(ey / V), int(round(ez / self.lh))
            if G.nx - a + 1 <= 0 or G.ny - b + 1 <= 0:
                continue
            lat = self._on_lattice(G, 0, G.nx - a + 1, ex, ph[0])[:, None] & self._on_lattice(G, 1, G.ny - b + 1, ey, ph[1])[None, :]
            for k in range(G.nl - nlay + 1):
                s = sum(box(Sc, kk, a, b) for kk in range(k, k + nlay))
                no = sum(box(Oc, kk, a, b) for kk in range(k, k + nlay))
                # 빈 칸(F)은 바닥 사각형 가장자리 RING 칸을 뺀 안쪽만 센다 — 실물(74.45 × 24.8)이 설계보다 작고 자세 맞춤 · 보정 오차가 수 mm
                r_ = RING if o in FLAT else 0
                nf = sum(box(Fc, kk, a - 2 * r_, b - 2 * r_) for kk in range(k, k + nlay))[r_:r_ + s.shape[0], r_:r_ + s.shape[1]]
                nh = sum(box(Hc, kk, a, b) for kk in range(k, k + nlay))
                ok = lat & (nf <= 0.15 * a * b * nlay) & (nh >= 0.5 * a * b * nlay)
                seen = sum((box(Oc, kk, a, b) >= 4).astype(int) for kk in range(k, k + nlay)) >= 3
                for i0, j0 in zip(*np.nonzero(ok)):
                    sc = s[i0, j0] - W_F * nf[i0, j0] - W_BLOCK
                    cx = G.lo[0] + (i0 + a / 2) * V
                    cy = G.lo[1] + (j0 + b / 2) * V
                    sc += self._face_term(o, cx, cy, (k + nlay) * self.lh, faces)
                    cs.append(dict(o=o, k=k, nl=nlay, i0=int(i0), j0=int(j0), a=a, b=b, s=float(sc), no=int(no[i0, j0]),
                                   seen=bool(seen[i0, j0])))
        seen = [(d['o'], d['k'], d['i0'], d['j0'], d['no']) for d in cs if d['o'] in STAND and d['seen']]
        if not seen:                               # 보이는 기둥이 없으면 안 보이는 기둥(거울 자리)도 없다
            return [d for d in cs if d['o'] not in STAND or d['seen']]
        # 구조물 가운데(칸) = 1 층 이상 O 의 외곽 가운데(0 층은 작업면 번짐이 섞여 뺌). 보이는 기둥이 있으면 1 층 이상 O 가 있다
        ii, jj = np.nonzero((S[1:] >= W_O).any(0))
        ci, cj = (ii.min() + ii.max() + 1) / 2, (jj.min() + jj.max() + 1) / 2

        def mirrored(d):
            """거울 짝이 보이는 블록이면 그 근거(O 칸 수) 중 가장 큰 것, 아니면 0."""
            best = 0
            for o, k, i0, j0, no in seen:
                if o != d['o'] or k != d['k']:
                    continue
                mi, mj = 2 * ci - i0 - d['a'], 2 * cj - j0 - d['b']
                if any(abs(d['i0'] - xi) <= 1 and abs(d['j0'] - yj) <= 1 for xi, yj in ((mi, j0), (i0, mj), (mi, mj))):
                    best = max(best, no)
            return best
        out = []
        for d in cs:
            if d['o'] in STAND and not d['seen']:
                m = mirrored(d)
                if not m:
                    continue
                d['s'] += min(0.01 * m, 2.0)     # 같은 점수면 근거가 뚜렷한 기둥의 거울을 고르게(블록 값보다 작게 — 혼자서는 안 들어감)
            out.append(d)
        return out

    # ------------------------------------------------------------------ 5) 정수계획
    def _milp(self, G, cs):
        """점수 합 최대 · 겹침 없음(층 × 칸마다 ≤ 1) · 받침 — scipy milp(HiGHS).

        받침: 눕힌 블록은 긴 축 두 반쪽 각각 아래에 닿는 블록(20 칸 이상 겹침)이 있어야 한다(무게 중심이 받침 사이 — 젠가 규칙).
              세운 블록은 바닥 아래에 하나. 근거 없는 받침 블록도 위 블록 점수가 크면 들어간다(= 가려진 블록 추정).
        출력: (고른 후보 목록, milp 상태 번호 또는 None(후보 없음), 해 없음 bool). 해가 없으면(시간 초과 등) 아무것도 고르지 않는다."""
        n = len(cs)
        if not n:
            return [], None, False
        cells = {}
        for c, d in enumerate(cs):
            ii, jj = np.mgrid[d['i0']:d['i0'] + d['a'], d['j0']:d['j0'] + d['b']]
            for kk in range(d['k'], d['k'] + d['nl']):
                for cell in ((kk * G.nx + ii) * G.ny + jj).ravel():
                    cells.setdefault(int(cell), []).append(c)
        rows = {tuple(v) for v in cells.values() if len(v) > 1}     # 같은 후보 묶음 줄은 하나만
        R, C, Vv, lb, ub = [], [], [], [], []
        for r, v in enumerate(rows):
            R += [r] * len(v)
            C += list(v)
            Vv += [1] * len(v)
            lb.append(-np.inf)
            ub.append(1)
        top = {}
        for c, d in enumerate(cs):
            top.setdefault(d['k'] + d['nl'] - 1, []).append(c)
        r = len(rows)
        for c, d in enumerate(cs):
            if d['k'] == 0:
                continue
            i0, j0, a, b = d['i0'], d['j0'], d['a'], d['b']
            if d['o'] == 'x':
                zones = [(i0, i0 + a // 2, j0, j0 + b), (i0 + a - a // 2, i0 + a, j0, j0 + b)]
            elif d['o'] == 'y':
                zones = [(i0, i0 + a, j0, j0 + b // 2), (i0, i0 + a, j0 + b - b // 2, j0 + b)]
            else:
                zones = [(i0, i0 + a, j0, j0 + b)]
            for z in zones:
                sup = [e for e in top.get(d['k'] - 1, []) if _ov(cs[e], *z) >= min(SUPPORT_CELLS, (z[1] - z[0]) * (z[3] - z[2]))]
                R += [r] * (len(sup) + 1)
                C += sup + [c]
                Vv += [1] * len(sup) + [-1]
                lb.append(0)
                ub.append(np.inf)
                r += 1
        A = coo_matrix((Vv, (R, C)), shape=(r, n)).tocsr()
        w = np.array([d['s'] for d in cs])
        res = milp(-w, constraints=LinearConstraint(A, lb, ub), integrality=np.ones(n), bounds=Bounds(0, 1),
                   options=dict(time_limit=MILP_TIME_S, mip_rel_gap=1e-6))
        if res.x is None:
            return [], int(res.status), True
        return [cs[c] for c in np.nonzero(res.x > 0.5)[0]], int(res.status), False

    def _to_block(self, G, d):
        """후보 → 블록 dict(mm). 근거(O 칸)가 INFER_O 칸 미만이면 inferred(받침 규칙으로만 들어간 블록)."""
        ex, ey, _ = self.ext[d['o']]
        return dict(x=float(G.lo[0] + d['i0'] * V + ex / 2), y=float(G.lo[1] + d['j0'] * V + ey / 2), z=float(d['k'] * self.lh),
                    ori=d['o'], inferred=d['no'] < INFER_O)

    # ------------------------------------------------------------------ 6) 젠가 엇갈림
    def _fix_slabs(self, blocks, faces):
        """같은 층 · 같은 방향 3 개가 판(길이 × 길이)을 이루고, 그 판에 맞는 윗면 관측이 하나도 없으면 위(없으면 아래) 층 판과 엇갈리게 돌림.
        blocks 를 제자리에서 고쳐 돌려준다."""
        lh, bw = self.lh, self.block_mm[1]

        def faced(b):
            """b 윗면에 맞는 YOLO 윗면 관측이 있는가."""
            return any(abs(f['z'] - b['z'] - lh) < 4 and f['along'] == b['ori'] and abs(f['x'] - b['x']) < 5 and abs(f['y'] - b['y']) < 5
                       for f in faces)
        by = {}
        for b in blocks:
            if b['ori'] in FLAT:
                by.setdefault(round(b['z']), []).append(b)
        for z, bl in sorted(by.items(), key=lambda t: -t[0]):
            for o in FLAT:
                par = [b for b in bl if b['ori'] == o]
                if len(par) != 3 or any(faced(b) for b in par):
                    continue
                cx = float(np.mean([b['x'] for b in par]))
                cy = float(np.mean([b['y'] for b in par]))
                if not all(abs(b['x'] - cx) < 30 and abs(b['y'] - cy) < 30 for b in par):
                    continue
                nb_ori = [c['ori'] for zz in (z + lh, z - lh) for c in by.get(zz, []) if c['ori'] in FLAT
                          and abs(c['x'] - cx) < 40 and abs(c['y'] - cy) < 40]
                if not nb_ori:
                    continue
                want = 'y' if max(set(nb_ori), key=nb_ori.count) == 'x' else 'x'
                if want == o:
                    continue
                for dd, b in zip((-bw, 0, bw), par):
                    b['ori'] = want
                    b['x'], b['y'] = (cx + dd, cy) if want == 'y' else (cx, cy + dd)
        return blocks

    def _explained(self, G, occ, blocks):
        """점 설명률 = O 칸 중 블록 상자(x · y 로 EXPLAIN_PAD 칸 넓힘, 같은 층) 안에 드는 비율. O 칸이 없으면 0.0."""
        if not occ.any():
            return 0.0
        m = np.zeros_like(occ)
        for b in blocks:
            ex, ey, ez = self.ext[b['ori']]
            i0 = int(math.floor((b['x'] - ex / 2 - G.lo[0]) / V)) - EXPLAIN_PAD
            i1 = int(math.ceil((b['x'] + ex / 2 - G.lo[0]) / V)) + EXPLAIN_PAD
            j0 = int(math.floor((b['y'] - ey / 2 - G.lo[1]) / V)) - EXPLAIN_PAD
            j1 = int(math.ceil((b['y'] + ey / 2 - G.lo[1]) / V)) + EXPLAIN_PAD
            k0 = int(round(b['z'] / self.lh))
            k1 = k0 + int(round(ez / self.lh))
            m[max(k0, 0):max(k1, 0), max(i0, 0):max(i1, 0), max(j0, 0):max(j1, 0)] = True
        return float((occ & m).sum() / occ.sum())

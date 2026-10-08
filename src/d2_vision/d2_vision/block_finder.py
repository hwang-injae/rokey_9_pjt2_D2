"""흩어진 블록 찾기 BlockFinder — 마스크 → 블록마다 자세 · yaw · 중심 · 틈 · 겹침 · 기울기 (ROS 없는 계산, W086, SDD §6.11 ②).

노드(wrist_block 의 /d2/vision/find_blocks)가 공급 관측 자세(observe_supply)에서 찍은 컬러 · 깊이로 부른다. 계산만 한다 — 로봇 · 메시지 · 파일에 손대지 않는다.

두 단계로 나눈다(E-47: 본 방법 = YOLO seg, 예비 = 엣지 + 깊이):
  ① 마스크 만들기 — YOLO seg 결과(블록마다 보이는 윤곽 마스크)를 그대로 받거나, 모델이 없으면 masks_from_edges()(예비)로 만든다.
  ② 뒷단(공용) blocks_from_masks() — 마스크마다 깊이 → base 점 → 윗면 평면 → 자세 up · yaw · 중심 · 틈(gap_mm · clear) · overlap · tilted.
     YOLO 마스크든 예비 마스크든 같은 뒷단을 쓴다.

출력은 IRD `/d2/vision/find_blocks` 응답의 blocks 원소 그대로(dict, JSON 직렬화 가능, NaN 없음), 윗면 높이 순:
  {"x_m","y_m","top_z_m","yaw_deg","up","clear":{축:bool},"gap_mm":{축:mm},"overlap","tilted","confidence"}
  - x_m · y_m · top_z_m: base_link (m), 윗면 가운데. 기울어진 블록은 윗면 평면의 가운데 높이.
  - up: 작업면 위 높이를 먼저(15 · 25 · 75 + 포개짐 층, ±5 mm) 보고 그 후보 안에서만 윗면 모양으로 가린다. 어긋나면 confidence 절반(10/8 PL).
    다른 블록 위에 얹힌 블록은 '내 높이 − 아래 블록 높이'를 내 두께로 다시 본다. 영상 테두리 · 영역 경계에 잘린 윤곽은 under · confidence ≤ 0.2.
  - yaw_deg: 윗면의 긴 변이 base x 와 평행하면 0, 반시계 +, −90 ≤ yaw < 90 (motion_math.wrap_half 와 같은 경계).
    세움(up LENGTH)은 윗면 25 mm 변(WIDTH) 기준(10/8 E-62).
  - gap_mm · clear 의 키 = 블록의 수평인 두 축(눕힘 LENGTH · WIDTH, 옆세움 LENGTH · THICKNESS, 세움 WIDTH · THICKNESS — E-55 ③).
    gap_mm.<축> = 그 축 방향으로 손가락이 닫힐 때(블록 양끝 바깥으로 손가락이 내려갈 때) 양쪽 틈 중 좁은 쪽(mm), 최대 GAP_SEARCH_MM.
    clear.<축> = gap_mm ≥ find.min_gap_mm.
  - overlap: none · top(다른 블록 위에 올라앉음 · 걸침) · under(다른 블록에 덮임 — 보이는 면이 작거나 위에 걸친 블록이 맞닿음,
    또는 모양이 블록 하나와 안 맞는 덩어리 — 어느 쪽이든 집지 않고 틈 · 장애물로만 쓴다).
  - tilted: 윗면 평면의 양 끝 높이 차 ≥ TILT_MM(5 mm, E-43).
단위: 이 파일 안 좌표는 m(base_link), 치수 · 틈 · 높이 비교는 mm. 높이 지도는 block_checker.height_map(스캔 추론기와 같은 함수)을 쓴다.
cv2 는 예비 마스크(masks_from_edges)에서만 쓴다 — 뒷단은 numpy 만 써서 CI(cv2 없음)에서도 시험한다.
"""
import itertools
import math

import numpy as np

from d2_vision.block_checker import depth_to_base_points, height_map

UPS = ('THICKNESS', 'WIDTH', 'LENGTH')
# up → (윗면 긴 변 축, 윗면 짧은 변 축). gap_mm · clear 의 키가 이 두 축이다(IRD find_blocks, E-55 ③)
TOP_AXES = {'THICKNESS': ('LENGTH', 'WIDTH'), 'WIDTH': ('LENGTH', 'THICKNESS'), 'LENGTH': ('WIDTH', 'THICKNESS')}
DIM = {'LENGTH': 0, 'WIDTH': 1, 'THICKNESS': 2}       # size_m 안 순서 (robot.yaml block_actual_m = [길이, 폭, 두께])

# 흩뿌릴 영역 (m) — robot.yaml observe_supply_pose 주석의 값(x 183~586 · y 142~337 mm, W134 10/7). 아직 yaml 키가 없어 여기 둔다.
SUPPLY_AREA_M = ((0.183, 0.586), (0.142, 0.337))

Z_MIN_M, Z_MAX_M = 0.1, 1.0   # 깊이 유효 범위 — block_checker.depth_to_base_points 기본값과 같게(픽셀 ↔ 점 순서를 맞추려고)
TILT_MM = 5.0             # 양 끝 높이 차 이상이면 tilted (IRD · E-43)
TABLE_WINDOW_MM = 6.0     # 작업면 높이 다시 재기: robot.yaml table_z 근처 ±이 안의 점 중앙값 (실측 장면 −19.5 mm vs yaml −18 mm)
TOP_MIN_MM = 4.0          # 작업면 위 이보다 낮은 점은 윗면 후보에서 뺀다(깊이 잡음 ±2~3 mm)
MAX_SLOPE = 0.6           # 이웃 픽셀 사이 높이 기울기(dz/dxy)가 이보다 크면 옆면 · 가장자리 튐 — 윗면에서 뺀다(31°보다 가파름)
EDGE_PX = 3               # 평면 맞춤 때 마스크 가장자리를 이만큼 깎는다 — 경계 깊이 튐(SDD §6.11 ② "가장자리 몇 픽셀은 버림")
PLANE_RESID_MM = 4.0      # 평면에서 이보다 먼 점은 윗면이 아니다(옆면 · 다른 블록)
MIN_TOP_PTS = 30          # 윗면 점이 이보다 적으면 그 마스크는 블록으로 내지 않는다(깊이 구멍 · 너무 가려짐)
SHRINK_MM = 4.5           # 기울기 · 가장자리 거르기로 윗면 크기가 이만큼 작게 재진다 — 치수 비교 때 더해 준다
                          # (W114-v2 실측: 떨어진 눕힘 블록 윗면이 70 x 20 mm 로 재짐 → 74.5 x 24.8 과 맞춤. 합성 영상은 1~2 mm 라 조금 크게 나온다)
HEIGHT_TOL_MM = 5.0       # 자세 판정 ① 높이: 작업면 위 높이가 후보 높이(위 치수 + 받침 층) ± 이 안이면 후보 (10/8 PL — 높이 먼저)
SIGMA_H_MM = 4.0          # 높이 후보 안에서 같은 모양 점수일 때 높이가 더 가까운 쪽을 고르는 데만 쓴다
STACK_PENALTY = 2.0       # 자세 판정 ② 모양 점수에 받침 블록 하나마다 더하는 값 — 비슷하면 덜 쌓였다고 본다
SHAPE_MISMATCH = 4.0      # 고른 자세의 모양 점수가 이보다 크면(≈ 2σ) '높이와 모양이 어긋남' → confidence 절반(자세는 높이를 따른다)
FAT_PER_MM = (0.15, 0.10) # 높은 블록일수록 깊이가 윗면을 넓게 잰다(W114-v2 실측: 세운 블록 윗면 25x15 → 40x28 mm) —
                          # 기대보다 큰 쪽 허용 폭을 작업면 위 FAT_FROM_MM 넘는 높이 1 mm 마다 (긴 변, 짧은 변) 이만큼 넓힌다
FAT_FROM_MM = 30.0        # 이 높이까지는 넓히지 않는다 — 눕힘 · 옆세움 · 두 층(≤ 30 mm)은 실측에서 윗면이 오히려 작게 재짐
MAX_COST = 9.0            # 모양 점수가 이보다 크면(≈ 3σ) 블록 하나 모양이 아니다(붙은 두 블록 · 잘못 나뉜 마스크) → 집지 않게 under
BORDER_PX = 3             # 마스크가 영상 테두리 이 픽셀 안에 닿으면 잘린 윤곽 — 자세 · yaw 를 믿지 않는다(10/8 PL)
CLIP_RING_PX = 3          # 마스크를 이만큼 넓힌 테두리에 흩뿌릴 영역 밖 블록 높이 픽셀이 있으면 영역 경계에서 잘린 윤곽
CLIP_H_FRAC = 0.5         # 그 '블록 높이' = 내 높이의 이 비율 이상 — 내 가장자리 깊이 번짐(작업면 위 5~8 mm)은 잘림으로 세지 않는다
CLIP_OUT_FRAC = 0.02      # 마스크 픽셀 중 영역 밖 비율이 이보다 크면 잘린 윤곽(YOLO 마스크는 영역을 모른다)
CLIP_CONF = 0.2           # 잘린 윤곽의 confidence 상한 — under 로 내보내 집지 않게 한다
UNDER_AREA_RATIO = 0.55   # 보이는 면적 ÷ 기대 윗면 면적이 이보다 작으면 under(덮임)
CONTACT_PX = 5            # 두 마스크가 이 픽셀(약 4 mm) 안이면 맞닿음
UNDER_TOL_MM = 4.0        # 맞닿은 더 높은 블록의 밑면이 내 윗면 − 이 값보다 높으면 '나를 덮음'
HALO_PX = 6               # 틈 셀 때 내 마스크를 이만큼(약 5 mm) 넓혀 내 블록의 가장자리 깊이 번짐을 장애물로 세지 않는다
GAP_CELL_M = 0.002        # 틈 계산 높이 지도 셀 — 8 mm 틈을 ±1.4 mm 로 재려고 5 mm 대신 2 mm
GAP_SEARCH_MM = 60.0      # 이 거리 안에 장애물이 없으면 틈 = 이 값(충분히 넓음, 22.2 mm 기준의 2.7배)
STRIP_MARGIN_MM = 2.0     # 손가락 폭(finger.width_m) 양옆 여유 — 손가락이 지나갈 띠의 반폭 = 폭/2 + 이 값
OBST_MIN_MM = 5.0         # 작업면 위 이보다 낮은 것(테이프 · 잡음)은 장애물이 아니다
FINGER_Z_MARGIN_MM = 5.0  # 손가락 끝 높이(윗면 − grasp_depth)보다 이만큼 아래까지 장애물로 본다(세운 블록은 더 깊게 문다 — robot.yaml 공급 칸)


def finder_cfg(robot):
    """robot.yaml 을 읽은 dict → BlockFinder 설정 dict. 파일 읽기는 부르는 쪽(노드 · 시험)이 한다.

    입력: robot = robot.yaml dict (block_actual_m 또는 block_size_m · table_z_m · find.min_gap_mm · grasp_depth_m · finger.width_m).
    출력: {'size_m': (길이, 폭, 두께) m, 'table_z_m', 'min_gap_mm', 'grasp_depth_m', 'finger_width_m', 'area_m': ((x0,x1),(y0,y1)) m}.
    실패: 키가 없으면 KeyError (설정 오류 — 노드가 시작할 때 드러나게).
    """
    size = robot.get('block_actual_m') or robot['block_size_m']    # 실측 치수가 있으면 그것(쌓는 높이 · 장면 상자와 같은 값)
    return {
        'size_m': tuple(float(v) for v in size),
        'table_z_m': float(robot['table_z_m']),
        'min_gap_mm': float(robot['find']['min_gap_mm']),
        'grasp_depth_m': float(robot['grasp_depth_m']),
        'finger_width_m': float(robot['finger']['width_m']),
        'area_m': SUPPLY_AREA_M,
    }


def wrap_deg(a):
    """각도(도)를 −90 이상 90 미만으로. 블록은 180° 대칭 — motion_math.wrap_half 와 같은 식(d2_motion 을 import 하지 않으려고 여기 둔다)."""
    return (a + 90.0) % 180.0 - 90.0


# ---------------- 픽셀 도구 (numpy 만) ----------------
def _dilate(m, r):
    """bool 영상을 (2r+1) 정사각형으로 넓힌다. cv2 없이 이웃 밀기로."""
    out = m.copy()
    for _ in range(int(r)):
        o = out.copy()
        o[1:, :] |= out[:-1, :]
        o[:-1, :] |= out[1:, :]
        out = o.copy()
        out[:, 1:] |= o[:, :-1]
        out[:, :-1] |= o[:, 1:]
    return out


def _erode(m, r):
    """bool 영상을 (2r+1) 정사각형으로 깎는다."""
    return ~_dilate(~m, r)


def _bbox(m, pad, shape):
    """마스크를 감싸는 (v0, v1, u0, u1) 창, 사방 pad 픽셀 넓힘(영상 안으로 자름). 빈 마스크면 None."""
    vs, us = np.nonzero(m)
    if len(vs) == 0:
        return None
    return (max(vs.min() - pad, 0), min(vs.max() + pad + 1, shape[0]),
            max(us.min() - pad, 0), min(us.max() + pad + 1, shape[1]))


def base_image(depth_m, intr, T_base2cam):
    """깊이 영상 → 픽셀마다 base 점 (H, W, 3) m. 깊이 없는 픽셀은 NaN. block_checker.depth_to_base_points(stride 1)를 픽셀 자리로 되돌려 놓는다."""
    depth_m = np.asarray(depth_m, float)
    ok = (depth_m > Z_MIN_M) & (depth_m < Z_MAX_M)
    P = np.full(depth_m.shape + (3,), np.nan)
    P[ok] = depth_to_base_points(depth_m, intr, np.asarray(T_base2cam, float), stride=1, z_min_m=Z_MIN_M, z_max_m=Z_MAX_M)
    return P


def table_height(P, cfg):
    """작업면 높이(m)를 영상에서 다시 잰다: 흩뿌릴 영역 안, robot.yaml table_z_m ±TABLE_WINDOW_MM 안 점의 중앙값.
    그런 점이 영역 점의 20 % 미만이면(블록이 거의 덮음 · 깊이 없음) robot.yaml 값을 그대로 쓴다."""
    z0 = cfg['table_z_m']
    inside = _in_area(P, cfg) & np.isfinite(P[..., 2])
    z = P[..., 2][inside]
    near = z[np.abs(z - z0) < TABLE_WINDOW_MM / 1000.0]
    if len(z) == 0 or len(near) < 0.2 * len(z):
        return z0
    return float(np.median(near))


def _in_area(P, cfg):
    """픽셀의 base (x, y)가 흩뿌릴 영역 안인가 (H, W) bool. area_m 이 None 이면 전부 참."""
    area = cfg.get('area_m')
    if area is None:
        return np.ones(P.shape[:2], bool)
    (x0, x1), (y0, y1) = area
    with np.errstate(invalid='ignore'):
        return (P[..., 0] >= x0) & (P[..., 0] <= x1) & (P[..., 1] >= y0) & (P[..., 1] <= y1)


def slope_image(P):
    """픽셀마다 이웃(±1 픽셀) 사이 높이 기울기 |dz| / |dxy| 의 큰 값. 옆면 · 경계는 크고 윗면은 작다. 계산 못 하면 inf."""
    z, x, y = P[..., 2], P[..., 0], P[..., 1]
    out = np.full(z.shape, np.inf)
    with np.errstate(invalid='ignore', divide='ignore'):
        su = np.abs(z[:, 2:] - z[:, :-2]) / np.hypot(x[:, 2:] - x[:, :-2], y[:, 2:] - y[:, :-2])
        sv = np.abs(z[2:, :] - z[:-2, :]) / np.hypot(x[2:, :] - x[:-2, :], y[2:, :] - y[:-2, :])
        s = np.fmax(su[1:-1, :], sv[:, 1:-1])
    s[~np.isfinite(s)] = np.inf
    out[1:-1, 1:-1] = s
    return out


def _levels(size_mm):
    """블록이 놓일 수 있는 받침: [(높이 mm, 받침 블록 수)] — 작업면(0, 0) + 블록 치수 1~3개를 더한 값(포개짐). 같은 높이는 적은 수로."""
    out = {0.0: 0}
    for n in (1, 2, 3):
        for c in itertools.combinations_with_replacement(size_mm, n):
            out.setdefault(round(sum(c), 1), n)
    return sorted(out.items())


# ---------------- 뒷단 (공용) ----------------
def _measure(mask, P, slope, table_z, depth_m, intr, cfg, score):
    """마스크 하나 → 윗면 평면 · 자세 · 중심 · yaw 등(내부 dict). 윗면 점이 MIN_TOP_PTS 보다 적으면 None.

    순서: 가장자리 깎은 마스크 안 '완만한 점'으로 평면 z = a·x + b·y + c 맞춤(먼 점 빼고 한 번 더) → 평면에 붙은 점 = 윗면
    → 윗면 점 PCA 로 긴 변 방향 · 길이 · 폭 · 가운데 → 자세 판정(_classify)."""
    h_all = (P[..., 2] - table_z) * 1000.0
    with np.errstate(invalid='ignore'):
        usable = np.isfinite(h_all) & (h_all > TOP_MIN_MM) & (slope < MAX_SLOPE)
    sel = _erode(mask, EDGE_PX) & usable
    if sel.sum() < MIN_TOP_PTS:                  # 작은 마스크(세움 윗면 · 가려진 조각)는 깎지 않고 본다
        sel = mask & usable
    if sel.sum() < MIN_TOP_PTS:
        return None
    pts = P[sel]
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(len(pts))]
    coef = np.linalg.lstsq(A, pts[:, 2], rcond=None)[0]
    keep = np.abs(A @ coef - pts[:, 2]) < PLANE_RESID_MM / 1000.0
    if keep.sum() >= MIN_TOP_PTS:
        coef = np.linalg.lstsq(A[keep], pts[keep, 2], rcond=None)[0]
    # 윗면 = 가장자리를 깎지 않은 마스크 중 평면에 붙은 완만한 점
    cand = mask & usable
    with np.errstate(invalid='ignore'):
        top = cand & (np.abs(P[..., 0] * coef[0] + P[..., 1] * coef[1] + coef[2] - P[..., 2]) < PLANE_RESID_MM / 1000.0)
    if top.sum() < MIN_TOP_PTS:
        return None
    xy = P[top][:, :2]
    mean = xy.mean(axis=0)
    w, vec = np.linalg.eigh(np.cov((xy - mean).T))
    u = vec[:, np.argmax(w)]
    v = np.array([-u[1], u[0]])
    pu, pv = (xy - mean) @ u, (xy - mean) @ v
    lo_u, hi_u = np.percentile(pu, [1, 99])
    lo_v, hi_v = np.percentile(pv, [1, 99])
    center = mean + u * (lo_u + hi_u) / 2 + v * (lo_v + hi_v) / 2
    long_mm = (hi_u - lo_u) * 1000.0 + SHRINK_MM
    short_mm = (hi_v - lo_v) * 1000.0 + SHRINK_MM
    zp = xy @ coef[:2] + coef[2]
    z_lo, z_hi = np.percentile(zp, [2, 98])
    z_c = float(center @ coef[:2] + coef[2])
    tilt_mm = float((z_hi - z_lo) * 1000.0)
    tilted = tilt_mm >= TILT_MM
    h_c = (z_c - table_z) * 1000.0
    h_ref = (z_lo - table_z) * 1000.0 if tilted else h_c   # 기울어진 블록은 낮은 끝(바닥에 닿은 쪽)으로 받침을 본다
    up, cost, base_mm, mismatch = _classify(long_mm, short_mm, h_ref, cfg)
    # 보이는 면적: 마스크 픽셀 수 × 윗면 깊이에서 픽셀 하나의 넓이 — 깊이 구멍이 있어도 마스크로 센다
    fx, fy = intr[0], intr[1]
    d_med = float(np.median(depth_m[top]))
    vis_area = float(mask.sum()) * (d_med / fx) * (d_med / fy) * 1e6
    base_conf = score * min(1.0, top.sum() / max(sel.sum(), 1))
    return {
        'mask': mask, 'top': top, 'coef': coef, 'center': center, 'u': u, 'v': v,
        'z_c': z_c, 'h_c': h_c, 'h_ref': h_ref, 'tilted': tilted, 'tilt_mm': tilt_mm, 'up': up, 'base_mm': base_mm,
        'long_mm': long_mm, 'short_mm': short_mm, 'vis_area': vis_area, 'cost': cost, 'mismatch': mismatch,
        'base_conf': base_conf, 'under': False, 'top_of': False, 'on': [],
    }


def _shape_cost(up, long_mm, short_mm, h_mm, size_mm):
    """윗면 모양(긴 변 · 짧은 변 mm)이 자세 up 의 윗면(75×25 · 75×15 · 25×15)과 얼마나 다른가(제곱 합, 0 = 같음).
    가려진 블록은 보이는 면이 작으니 기대보다 작은 쪽은 너그럽게(σ 20 · 8 mm), 큰 쪽은 엄하게(σ 8 · 5 mm + 높이 × FAT_PER_MM) 본다."""
    e_long, e_short = (size_mm[DIM[a]] for a in TOP_AXES[up])
    hh = max(h_mm - FAT_FROM_MM, 0.0)
    c_l = ((long_mm - e_long) / (8.0 + FAT_PER_MM[0] * hh if long_mm > e_long else 20.0)) ** 2
    c_s = ((short_mm - e_short) / (5.0 + FAT_PER_MM[1] * hh if short_mm > e_short else 8.0)) ** 2
    return c_l + c_s


def _classify(long_mm, short_mm, h_ref_mm, cfg):
    """작업면 위 높이(mm)를 먼저, 윗면 모양은 보조로 → (up, 모양 점수, 받침 높이 mm, 어긋남 bool). (10/8 PL 규칙)

    ① 높이: 후보 = 위 치수(15 · 25 · 75) + 받침 층(0 · 블록 1~3개 합 — 15+15, 15+25 …). 높이가 ±HEIGHT_TOL_MM 안인 후보만 남긴다.
       맞는 후보가 없으면 가장 가까운 높이 하나를 쓰고 어긋남으로 표시한다.
    ② 모양: 남은 후보 중 모양 점수(+ 받침 블록 수 × STACK_PENALTY, 같으면 높이가 가까운 쪽)가 가장 작은 것.
    ③ 고른 자세의 모양 점수 > SHAPE_MISMATCH 면 어긋남(부르는 쪽이 confidence 를 낮춘다) — 자세는 높이를 따른 그대로."""
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    cands = [(abs(h_ref_mm - size_mm[DIM[up]] - L), up, L, n) for up in UPS for L, n in _levels(size_mm)]
    near = [c for c in cands if c[0] <= HEIGHT_TOL_MM]
    mismatch = not near
    if not near:
        near = [min(cands)]

    def key(c):
        return _shape_cost(c[1], long_mm, short_mm, h_ref_mm, size_mm) + STACK_PENALTY * c[3] + 0.1 * (c[0] / SIGMA_H_MM) ** 2

    dh, up, lvl, _ = min(near, key=key)
    shape = _shape_cost(up, long_mm, short_mm, h_ref_mm, size_mm)
    mismatch = mismatch or shape > SHAPE_MISMATCH
    return up, shape + (dh / SIGMA_H_MM) ** 2, lvl, mismatch


def _refine_top(blk, cfg):
    """다른 블록 위에 얹힌 블록(맞닿은 아래 블록이 있음, 기울지 않음)의 자세를 '내 높이 − 아래 블록 높이' = 내 두께로 다시 정한다(10/8 PL).
    두께가 15 · 25 · 75 중 ±HEIGHT_TOL_MM 안이고 모양도 맞는 자세가 있으면 그것(여럿이면 모양으로), 없으면 그대로 둔다. 입력 dict 를 고침."""
    if blk['tilted'] or not blk['on']:
        return
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    # 맞닿은 아래 블록이 여럿이면 높은 것부터 — 맞닿았다고 다 받침은 아니라서(보이지 않는 받침 위에 얹혀 낮은 블록과 옆으로 닿을 수 있음)
    # 두께가 맞고 모양도 맞는(SHAPE_MISMATCH 이하) 첫 받침만 쓴다. 없으면 높이 먼저 고른 자세 그대로.
    for a in sorted(blk['on'], key=lambda k: -k['h_c']):
        own = blk['h_c'] - a['h_c']
        ups = [(_shape_cost(up, blk['long_mm'], blk['short_mm'], blk['h_c'], size_mm), up) for up in UPS
               if abs(own - size_mm[DIM[up]]) <= HEIGHT_TOL_MM]
        ups = [c for c in ups if c[0] <= SHAPE_MISMATCH]
        if ups:
            shape, up = min(ups)
            blk.update(up=up, base_mm=a['h_c'], cost=shape + ((own - size_mm[DIM[up]]) / SIGMA_H_MM) ** 2, mismatch=False)
            return


def _clipped(mask, P, h_mm, h_c, cfg):
    """윤곽이 잘렸나(bool): 영상 테두리 BORDER_PX 안에 닿음 · 마스크 픽셀이 흩뿌릴 영역 밖(CLIP_OUT_FRAC 넘게) ·
    마스크 바로 바깥(CLIP_RING_PX)에 영역 밖이면서 내 높이 h_c(mm)의 CLIP_H_FRAC 이상인 픽셀이 있음(블록이 영역 밖으로 이어짐 —
    예비 마스크는 영역 안에서만 만들어져 경계에서 잘려 나온다)."""
    b = BORDER_PX
    if mask[:b].any() or mask[-b:].any() or mask[:, :b].any() or mask[:, -b:].any():
        return True
    if cfg.get('area_m') is None:
        return False
    win = _bbox(mask, CLIP_RING_PX + 1, mask.shape)
    v0, v1, u0, u1 = win
    m = mask[v0:v1, u0:u1]
    Pw = P[v0:v1, u0:u1]
    inside = _in_area(Pw, cfg)
    known = np.isfinite(Pw[..., 0])
    if (m & known & ~inside).sum() > CLIP_OUT_FRAC * max(m.sum(), 1):
        return True
    ring = _dilate(m, CLIP_RING_PX) & ~m
    with np.errstate(invalid='ignore'):
        tall = h_mm[v0:v1, u0:u1] > max(OBST_MIN_MM, CLIP_H_FRAC * h_c)
    return bool((ring & known & ~inside & tall).any())


def _mark_overlaps(blocks):
    """맞닿은 두 블록 중 위에 걸친 쪽을 top, 덮인 쪽을 under 로 표시한다(입력 dict 를 고침).

    A 와 B 가 영상에서 CONTACT_PX 안으로 맞닿고, 맞닿은 곳에서 B 윗면 − B 의 위 치수(= B 밑면)가 A 윗면 − UNDER_TOL_MM 보다 높으면
    B 가 A 위에 있다(포개짐 · 걸침). 옆에 붙은 더 높은 블록(옆세움 · 세움)은 밑면이 작업면이라 걸리지 않는다."""
    size_mm = None
    for a in blocks:
        a['_grow'] = _dilate(a['mask'], CONTACT_PX)
    for a, b in itertools.permutations(blocks, 2):
        contact = a['_grow'] & b['top']
        if contact.sum() < 10:
            continue
        size_mm = size_mm or [s * 1000.0 for s in a['cfg_size_m']]
        xyz = a['P'][contact]
        zb = xyz[:, 0] * b['coef'][0] + xyz[:, 1] * b['coef'][1] + b['coef'][2]
        b_bottom = (float(np.median(zb)) - a['table_z']) * 1000.0 - size_mm[DIM[b['up']]]
        if b_bottom >= a['h_c'] - UNDER_TOL_MM:
            a['under'] = True
            b['top_of'] = True
            b['on'].append(a)
    for a in blocks:
        a.pop('_grow', None)


def _gaps(blk, P, h_mm, valid, cfg):
    """블록의 수평 두 축마다 손가락이 내려갈 틈(mm, 양쪽 중 좁은 쪽). 반환 {축: mm}.

    장애물 = 내 마스크(HALO_PX 넓힘) 밖에서 작업면 위 높이 > 손가락 끝 높이(윗면 − grasp_depth − 여유, 최소 OBST_MIN_MM)인 점.
    그 점들로 block_checker.height_map(GAP_CELL_M 격자, 셀마다 최고 z)을 만들고, 축 방향 띠(손가락 폭 + 여유) 안에서
    블록 면(중심 ± 치수/2)부터 가장 가까운 장애물 셀까지 거리를 잰다. 깊이 없는 셀은 비었다고 본다."""
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    thr = max(blk['h_c'] - cfg['grasp_depth_m'] * 1000.0 - FINGER_Z_MARGIN_MM, OBST_MIN_MM)
    win = _bbox(blk['mask'], 160, blk['mask'].shape)      # 160 px ≈ 130 mm — 블록 반길이 37 + 찾는 거리 60 mm 보다 넉넉히
    v0, v1, u0, u1 = win
    own = _dilate(blk['mask'][v0:v1, u0:u1], HALO_PX)
    with np.errstate(invalid='ignore'):
        obst = valid[v0:v1, u0:u1] & ~own & (h_mm[v0:v1, u0:u1] > thr)
    half_win = (max(size_mm) / 2 + GAP_SEARCH_MM + 10.0) / 1000.0
    grid, x0, y0 = height_map(P[v0:v1, u0:u1][obst], tuple(blk['center']), half_win, GAP_CELL_M)
    n = grid.shape[0]
    cx = x0 + (np.arange(n) + 0.5) * GAP_CELL_M
    cy = y0 + (np.arange(n) + 0.5) * GAP_CELL_M
    X, Y = np.meshgrid(cx, cy, indexing='ij')
    occ = np.isfinite(grid)
    dx, dy = X[occ] - blk['center'][0], Y[occ] - blk['center'][1]
    strip = (cfg['finger_width_m'] * 1000.0 / 2 + STRIP_MARGIN_MM) / 1000.0
    out = {}
    long_axis, short_axis = TOP_AXES[blk['up']]
    for axis, d, p in ((long_axis, blk['u'], blk['v']), (short_axis, blk['v'], blk['u'])):
        a = dx * d[0] + dy * d[1]
        q = dx * p[0] + dy * p[1]
        half = size_mm[DIM[axis]] / 2000.0
        in_strip = np.abs(q) <= strip
        side = []
        for s in (1.0, -1.0):
            dist = (s * a[in_strip & (s * a > 0)] - half) * 1000.0
            side.append(float(np.clip(dist.min(), 0.0, GAP_SEARCH_MM)) if len(dist) else GAP_SEARCH_MM)
        out[axis] = round(min(side), 1)
    return out


def blocks_from_masks(masks, depth_m, intr, T_base2cam, cfg, scores=None, info=None):
    """블록 마스크 목록 → find_blocks 응답의 blocks 목록 (공용 뒷단 — YOLO seg 마스크든 예비 마스크든 같다).

    입력: masks = 블록마다 (H, W) bool 마스크(컬러 영상 기준, 깊이는 컬러에 맞춘 것) 목록 · depth_m = (H, W) m(0 = 없음) ·
          intr = (fx, fy, ppx, ppy) px · T_base2cam = 4x4 (m, base ← 카메라) · cfg = finder_cfg() ·
          scores = 마스크마다 신뢰도 0~1(YOLO conf, 없으면 1) · info = dict 를 주면 디버그 값(작업면 높이 · 블록마다 마스크 번호 ·
          빠진 마스크와 이유)을 채운다 — 응답에는 넣지 않는다.
    출력: list of dict(IRD 칸 그대로, 파이썬 기본형만 · NaN 없음), 윗면 높이(top_z_m) 높은 순.
    바깥 영향 없음. 윗면 점이 모자란 마스크(깊이 구멍 · 너무 작음)는 빼고 info['skipped'] 에 적는다. 예외를 내지 않는다(빈 목록 가능).
    """
    depth_m = np.asarray(depth_m, float)
    P = base_image(depth_m, intr, T_base2cam)
    table_z = table_height(P, cfg)
    slope = slope_image(P)
    h_mm = (P[..., 2] - table_z) * 1000.0
    valid = np.isfinite(h_mm)
    scores = [1.0] * len(masks) if scores is None else list(scores)
    found, skipped = [], []
    for i, m in enumerate(masks):
        m = np.asarray(m, bool)
        if m.shape != depth_m.shape or not m.any():
            skipped.append((i, '마스크 크기가 깊이와 다르거나 비었음'))
            continue
        blk = _measure(m, P, slope, table_z, depth_m, intr, cfg, float(scores[i]))
        if blk is None:
            skipped.append((i, f'윗면 점 {MIN_TOP_PTS}개 미만'))
            continue
        blk.update(index=i, P=P, table_z=table_z, cfg_size_m=cfg['size_m'])
        found.append(blk)
    _mark_overlaps(found)
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    out = []
    for blk in found:
        _refine_top(blk, cfg)
        e_long, e_short = (size_mm[DIM[a]] for a in TOP_AXES[blk['up']])
        blk['area_ratio'] = blk['vis_area'] / (e_long * e_short)
        blk['clipped'] = _clipped(blk['mask'], P, h_mm, blk['h_c'], cfg)
        conf = blk['base_conf'] * math.exp(-blk['cost'] / 8.0) * (0.5 if blk['mismatch'] else 1.0)
        if blk['clipped']:
            conf = min(conf, CLIP_CONF)
        blk['confidence'] = float(min(max(conf, 0.0), 1.0))
        gaps = _gaps(blk, P, h_mm, valid, cfg)
        # 덮였거나(맞닿은 위 블록 · 보이는 면이 작음) · 블록 하나 모양이 아니거나(MAX_COST) · 윤곽이 잘렸으면(자세 · yaw 를 못 믿음)
        # 집지 않고 장애물로만 쓰게 under
        under = blk['under'] or blk['area_ratio'] < UNDER_AREA_RATIO or blk['cost'] > MAX_COST or blk['clipped']
        top = blk['top_of'] or blk['base_mm'] > 0
        yaw = wrap_deg(math.degrees(math.atan2(blk['u'][1], blk['u'][0])))
        out.append((blk['index'], {
            'x_m': round(float(blk['center'][0]), 4),
            'y_m': round(float(blk['center'][1]), 4),
            'top_z_m': round(blk['z_c'], 4),
            'yaw_deg': round(yaw, 1) if round(yaw, 1) < 90.0 else -90.0,
            'up': blk['up'],
            'clear': {k: bool(g >= cfg['min_gap_mm']) for k, g in gaps.items()},
            'gap_mm': gaps,
            'overlap': 'under' if under else ('top' if top else 'none'),
            'tilted': bool(blk['tilted']),
            'confidence': round(blk['confidence'], 2),
        }))
    out.sort(key=lambda t: -t[1]['top_z_m'])
    if info is not None:
        info['table_z_m'] = table_z
        info['mask_index'] = [i for i, _ in out]
        info['skipped'] = skipped
        info['detail'] = {b['index']: {k: b[k] for k in ('long_mm', 'short_mm', 'h_c', 'tilt_mm', 'area_ratio', 'cost', 'base_mm',
                                                         'mismatch', 'clipped')}
                          for b in found}
    return [b for _, b in out]


# ---------------- 예비 마스크 (엣지 + 깊이, E-47) ----------------
FG_MM = 7.0               # 작업면 위 이보다 높으면 블록 후보 픽셀 (블록 ≥ 14.8 mm, 깊이 잡음 ±3 mm)
JUMP_SLOPE = 1.2          # 높이 기울기(dz/dxy)가 이보다 크면 층 경계 — 높이가 다른 블록 사이를 끊는다
MIN_AREA_MM2 = 200.0      # 이보다 작은 덩어리는 버린다(세움 윗면 25×15 = 370 mm²)
SINGLE_AREA_RATIO = 1.35  # 덩어리 넓이가 눕힘 윗면(75×25)의 이 배를 넘으면 여러 블록이 붙은 것 — 컬러 엣지로 더 가른다
HALF_HEIGHT = 0.5         # 마스크를 조각 높이의 이 비율보다 높은 픽셀로 줄인다(번진 깊이 경계의 가운데 = 실제 모서리)
CANNY_STEPS = ((40, 120), (15, 45))   # 붙은 덩어리를 끊을 컬러 엣지 문턱 — 먼저 강한 엣지만, 안 끊기면 약한 엣지까지


def masks_from_edges(color, depth_m, intr, T_base2cam, cfg, info=None):
    """예비 마스크(E-47): 컬러 엣지 + 깊이로 블록마다 마스크를 만든다. YOLO seg 모델이 없을 때 · 흔들릴 때 쓴다.

    입력: color (H, W, 3) BGR uint8 · depth_m (H, W) m(컬러에 맞춘 것) · intr · T_base2cam · cfg(finder_cfg). info = 디버그 dict(선택).
    출력: list of (H, W) bool 마스크. 바깥 영향 없음.
    방법: ① 흩뿌릴 영역 안에서 작업면 위 FG_MM 보다 높은 픽셀 ② 높이가 급히 바뀌는 곳(층 경계, JUMP_SLOPE)을 끊음
          ③ 덩어리가 눕힘 블록 하나보다 크면(붙은 블록) 컬러 Canny 엣지로 끊고, 그래도 크면 거리 변환 봉우리로 나눔
          ④ 끊느라 버린 경계 픽셀을 watershed 로 이웃 조각에 돌려준다.
    한계: 같은 높이로 딱 붙은 블록 · 나뭇결 엣지가 강한 블록은 잘못 나뉠 수 있다 — 서로 떨어진 눕힘 블록이 목표(본 방법은 YOLO).
    실패: cv2 가 없으면 ImportError.
    """
    import cv2
    P = base_image(depth_m, intr, T_base2cam)
    table_z = table_height(P, cfg)
    h = (P[..., 2] - table_z) * 1000.0
    with np.errstate(invalid='ignore'):
        fg = np.isfinite(h) & (h > FG_MM) & _in_area(P, cfg)
    k3 = np.ones((3, 3), np.uint8)
    fg = cv2.morphologyEx(fg.astype(np.uint8), cv2.MORPH_OPEN, k3).astype(bool)
    hs = np.nan_to_num(h, nan=0.0).astype(np.float32)
    hs = cv2.medianBlur(hs, 5)
    gx = cv2.Sobel(hs, cv2.CV_32F, 1, 0, ksize=3) / 8.0
    gy = cv2.Sobel(hs, cv2.CV_32F, 0, 1, ksize=3) / 8.0
    d = np.where(np.asarray(depth_m) > 0, np.asarray(depth_m, float), np.nan)
    mm_per_px = np.nan_to_num(d * 1000.0 / intr[0], nan=1.0)
    jump = np.hypot(gx, gy) / mm_per_px > JUMP_SLOPE
    core = fg & ~jump
    gray = cv2.GaussianBlur(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    edges = [cv2.dilate(cv2.Canny(gray, lo, hi), k3) > 0 for lo, hi in CANNY_STEPS]
    size_mm = [s * 1000.0 for s in cfg['size_m']]
    single_max = SINGLE_AREA_RATIO * size_mm[0] * size_mm[1]
    px_area = (np.nan_to_num(d, nan=0.5) / intr[0]) * (np.nan_to_num(d, nan=0.5) / intr[1]) * 1e6   # 픽셀 하나 넓이 mm²

    pieces = []
    n, lab = cv2.connectedComponents(core.astype(np.uint8), connectivity=4)
    for k in range(1, n):
        comp = lab == k
        area = float(px_area[comp].sum())
        if area < MIN_AREA_MM2:
            continue
        if area <= single_max:
            pieces.append(comp)
            continue
        pieces.extend(_split(comp, edges, px_area, single_max, cv2))
    markers = np.zeros(h.shape, np.int32)
    for i, p in enumerate(pieces, start=1):
        markers[p] = i
    markers[~cv2.dilate(fg.astype(np.uint8), k3, iterations=2).astype(bool)] = len(pieces) + 1   # 바깥(작업면)
    if pieces:
        cv2.watershed(np.ascontiguousarray(color), markers)
    masks = []
    for i, p in enumerate(pieces, start=1):
        m = (markers == i) & fg
        # 깊이가 번져 마스크가 블록보다 크다 — 그 조각 높이의 절반보다 높은 픽셀만 남겨 실제 모서리(높이 절반 선)에 맞춘다
        m &= np.nan_to_num(h, nan=0.0) > HALF_HEIGHT * float(np.median(h[p]))
        if px_area[m].sum() >= MIN_AREA_MM2:
            masks.append(m)
    if info is not None:
        info['table_z_m'] = table_z
        info['fg'] = fg
        info['core'] = core
    return masks


def _split(comp, edges, px_area, single_max, cv2):
    """붙은 블록 덩어리 하나를 나눈다: 컬러 엣지(강한 것 → 약한 것 순)로 끊어 보고, 그래도 큰 조각은 거리 변환 봉우리(블록 가운데)를 씨앗으로 나눈다.
    입력 edges = CANNY_STEPS 순서의 엣지 영상 목록. 출력: 조각 마스크 목록(MIN_AREA_MM2 보다 작은 조각은 버림)."""
    if len(edges) > 1:
        cut = comp & ~edges[0]
        n, lab = cv2.connectedComponents(cut.astype(np.uint8), connectivity=4)
        big = [lab == k for k in range(1, n) if px_area[lab == k].sum() >= MIN_AREA_MM2]
        if len(big) <= 1:                            # 강한 엣지로 안 끊김 — 약한 엣지로 다시
            return _split(comp, edges[1:], px_area, single_max, cv2)
        out = []
        for p in big:
            out.extend([p] if px_area[p].sum() <= single_max else _split(p, edges[1:], px_area, single_max, cv2))
        return out
    out = []
    cut = comp & ~edges[0]
    n, lab = cv2.connectedComponents(cut.astype(np.uint8), connectivity=4)
    for k in range(1, n):
        p = lab == k
        area = float(px_area[p].sum())
        if area < MIN_AREA_MM2:
            continue
        if area <= single_max:
            out.append(p)
            continue
        dist = cv2.distanceTransform(p.astype(np.uint8), cv2.DIST_L2, 5)
        seeds = dist > 0.6 * dist.max()
        m, sl = cv2.connectedComponents(seeds.astype(np.uint8))
        if m <= 2:                                   # 봉우리가 하나뿐 — 나누지 못함, 그대로 둔다(뒷단이 under · 비용으로 걸러 낸다)
            out.append(p)
            continue
        mk = np.zeros(p.shape, np.int32)
        mk[seeds] = sl[seeds]
        mk[~p] = m                                   # 덩어리 밖
        img = np.dstack([(dist / max(dist.max(), 1e-6) * 255).astype(np.uint8)] * 3)
        cv2.watershed(img, mk)
        out.extend((mk == j) & p for j in range(1, m) if px_area[(mk == j) & p].sum() >= MIN_AREA_MM2)
    return out


def masks_from_yolo(result, hw):
    """YOLO seg 결과 하나(ultralytics Results) → (블록 마스크 목록, 신뢰도 목록). 본 방법(E-38 · E-47)의 ① 단계.

    입력: result = model(...)[0] (masks.data (N, h, w) 0~1 · boxes.conf (N,)) · hw = 컬러 영상 (H, W).
    출력: ([(H, W) bool, …], [float, …]) — 검출이 없으면 ([], []).
    마스크 크기가 (H, W) 와 다르면 가장 가까운 픽셀로 늘린다(numpy 만 — CI 에 cv2 가 없다). 노드는 retina_masks=True 로 불러
    처음부터 원본 크기를 받는다(640×480 은 레터박스 여백이 없어 늘려도 같다). torch 텐서 · numpy 배열 둘 다 받는다. 바깥 영향 없음."""
    def arr(x):
        """torch 텐서 또는 배열 → numpy."""
        x = x.cpu().numpy() if hasattr(x, 'cpu') else x
        return np.asarray(x)
    if getattr(result, 'masks', None) is None or getattr(result, 'boxes', None) is None:
        return [], []
    data = arr(result.masks.data).astype(np.float32)
    conf = arr(result.boxes.conf).astype(float).ravel()
    H, W = hw
    if data.ndim != 3 or len(data) == 0:
        return [], []
    if data.shape[1:] != (H, W):
        rows = np.minimum((np.arange(H) + 0.5) * data.shape[1] / H, data.shape[1] - 1).astype(int)
        cols = np.minimum((np.arange(W) + 0.5) * data.shape[2] / W, data.shape[2] - 1).astype(int)
        data = data[:, rows][:, :, cols]
    masks = [m > 0.5 for m in data]
    return masks, [float(c) for c in conf[:len(masks)]] + [1.0] * max(0, len(masks) - len(conf))


OVERLAP_BGR = {'none': (0, 200, 0), 'top': (0, 200, 255), 'under': (0, 0, 255)}   # 그림 윤곽 색: 집을 수 있음 초록 · 위 노랑 · 덮임 빨강


def draw_found(color, blocks, masks, mask_index):
    """find_blocks 결과를 컬러 영상 위에 그린다(IRD `/d2/vision/wrist_image`, E-67 — 화면은 받은 그림을 그대로 보여 준다).

    입력: color (H, W, 3) BGR uint8 · blocks = find 결과(윗면 높이 순) · masks = BlockFinder.last_masks ·
          mask_index = last_info['mask_index'](blocks[k] 가 쓴 마스크 번호). 출력: 같은 크기 BGR 새 그림(자르거나 줄이지 않음).
    블록마다 윤곽(overlap 색) + '번호(1부터, 응답 순서) up 첫 글자 yaw°'. 바깥 영향 없음. 실패: cv2 가 없으면 ImportError."""
    import cv2
    img = np.ascontiguousarray(color).copy()
    for k, (b, mi) in enumerate(zip(blocks, mask_index)):
        m = np.asarray(masks[mi], np.uint8)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cnts, -1, OVERLAP_BGR.get(b['overlap'], (255, 255, 255)), 2)
        vs, us = np.nonzero(m)
        org = (int(us.mean()) - 20, int(vs.mean()) + 5) if len(us) else (5, 20 + 18 * k)
        org = (min(max(org[0], 2), img.shape[1] - 90), min(max(org[1], 14), img.shape[0] - 4))   # 영상 테두리 블록도 글자가 잘리지 않게
        txt = f'{k + 1} {b["up"][0]} {b["yaw_deg"]:+.0f}' + (' T' if b['tilted'] else '')
        cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


class BlockFinder:
    """find_blocks 계산 묶음: 설정을 들고 있다가 영상 한 장 → blocks 목록. 노드는 이것 하나만 만든다.

    입력: cfg = finder_cfg(robot.yaml dict). 바깥 영향 없음.
    find(color, depth_m, intr, T_base2cam, masks=None, scores=None) → blocks 목록(blocks_from_masks 와 같음).
      masks 가 None 이면 예비 마스크(masks_from_edges)를 만든다. YOLO seg 를 쓰면 그 마스크 목록 · conf 를 넘긴다.
    last_info 에 마지막 계산의 디버그 값(작업면 높이 · 마스크 번호 · 빠진 마스크)을 둔다.
    """

    def __init__(self, cfg):
        """설정 dict 를 받는다(finder_cfg 출력)."""
        self.cfg = cfg
        self.last_info = {}
        self.last_masks = []

    def find(self, color, depth_m, intr, T_base2cam, masks=None, scores=None):
        """영상 한 장에서 블록을 찾는다. 출력: find_blocks 응답 blocks 목록(윗면 높이 순). 실패 때 예외 대신 빈 목록 가능."""
        info = {}
        if masks is None:
            masks = masks_from_edges(color, depth_m, intr, T_base2cam, self.cfg, info=info)
            scores = None
        self.last_masks = list(masks)
        blocks = blocks_from_masks(masks, depth_m, intr, T_base2cam, self.cfg, scores=scores, info=info)
        self.last_info = info
        return blocks

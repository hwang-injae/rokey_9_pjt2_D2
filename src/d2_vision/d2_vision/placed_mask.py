"""조립 자리에 놓은 블록 하나를 YOLO 마스크(컬러) + 카메라 광선으로 재기 — find_blocks {"area": "assembly"} 의 placed_method:=mask (ROS 없는 계산).

W144(챌린지 A4)의 놓은 블록 다시 재기. placed_measure.measure_placed(깊이만)는 깊이 카메라가 윗면 가장자리를 3~5 mm 처지게 재서
크기가 작게(실기 15장: 67~72 × 20~24 mm, 실제 74.5 × 24.8) 나오고 가운데가 ±1.9 mm 흔들렸다.
여기서는 10/15 PL 결정대로 **기울기 · 덮임 · 자세 판단은 깊이(measure_placed)**, **크기 · 위치 · yaw 는 마스크**로 잰다:
  마스크 픽셀마다 카메라 광선을 쏴 '윗면 높이 평면'과 만나는 점(xy)을 모아 주축으로 잰다 — 가장자리 깊이가 처져도 윤곽은 컬러에서 온다.
  실기 사진 16장(V008 · V009 · V001 observe, 10/15 eval_mask.py): 잰 블록 8 → 15장 · 크기 74.3±0.3 × 25.4±0.6 mm · 가운데 흔들림 ±0.6~0.8 mm.
  dy −2.4±0.8 mm 치우침(다리 긴 축 방향)은 10/15 오후 세 자 비교(깊이 띠 · 마스크 · 컬러 밝기 경계, 다리 22장)로 가려졌다: 마스크 = 컬러이고
  **깊이 쪽이 −y 끝만 5 mm 잘려** 가운데가 치우친 것 + 다리가 실제로 긴 축 −2.4 mm 밀려 놓임(놓기 쪽 치우침 — 박진용). 컬러 ↔ 깊이 정렬
  어긋남이 아니므로 **마스크 옮김(px_shift)은 0 으로 쓴다**('−3 px 옮김' 안은 폐기 — 좌판에서 +3 mm 틀어짐). px_shift 인자 · 파라미터는
  정렬 보정이 필요해질 때를 위해 남겨 둔다(기본 0).
  응답 corners_z_m(10/15 한석형 #190)은 깊이 점으로 — 칸 위치는 마스크로 잰 가운데 · yaw, 높이 창은 placed_measure.corner_heights 와 같다.
단위: 입력 · 출력 좌표 m(base_link), 픽셀은 컬러(= 정렬된 깊이) 영상 기준. 바깥 영향 없음. numpy 만(CI 에 cv2 가 없다).
"""
import math

import numpy as np

from d2_vision.block_finder import TOP_AXES, _erode, wrap_deg
from d2_vision.placed_measure import SIZE_TOL_M, TOP_TOL_M, corner_heights, measure_placed, up_of
from d2_vision.spare_measure import UP_DIM

DEPTH_FINAL = ('NO_PART', 'COVERED', 'STANDING', 'FEW_POINTS', 'TILTED')   # 깊이가 정하는 거절 — 마스크로 다시 재지 않고 그대로 돌려준다(10/15 PL)
INNER_ERODE_PX = 6         # 윗면 높이는 마스크를 이만큼 깎은 안쪽 깊이 점으로 — 관측 높이 약 0.5 m 에서 1 px ≈ 0.8 mm 라 6 px ≈ 5 mm,
                           # 가장자리 처짐(3~5 mm 폭)을 비켜 간다(흉내 시험 값)
MIN_INNER_PTS = 30         # 안쪽 윗면 점이 이보다 적으면 윗면 높이를 믿지 않고 깊이 결과로 돌아간다(마스크가 작거나 깊이 구멍)
MIN_MASK_PX = 200          # 마스크 픽셀이 이보다 적으면 블록 하나 윤곽이 아니다 — 눕힘 윗면 75 × 25 mm 는 0.5 m 에서 약 2 800 px
MIN_FOOT_OVERLAP = 0.4     # 설계 자리 투영 픽셀 중 마스크가 덮는 비율이 이보다 적으면 이 블록이 아니다 — 폭 방향 15 mm 비켜 놓이면 40 %
                           # (깊이 재기도 설계 자리 ±REACH_M 15 mm 안만 본다). 붙은 이웃 마스크는 거의 0, 제자리 블록은 90 % 넘게 겹친다
MASK_EDGE_PCT = (0.5, 99.5)   # 양 끝 = 주축 투영의 이 백분위 — 마스크 테두리의 튀는 픽셀 몇 개(0.5 % ≈ 14 px)에 안 흔들리게.
                              # 깊이 쪽(EDGE_PCT 2 · 98)보다 좁게 자르는 까닭: 마스크는 가장자리 잡음이 거의 없어 많이 버리면 오히려 흔들린다
MASK_EDGE_KEEP = (MASK_EDGE_PCT[1] - MASK_EDGE_PCT[0]) / 100.0   # 백분위 폭 = 실제 폭 × 이 비율(고르게 퍼진 픽셀) — 크기는 이걸로 나눠 되돌린다


def _shift_mask(m, du, dv):
    """bool 마스크를 (du, dv) 정수 픽셀 옮긴다(u 오른쪽 · v 아래가 +). 영상 밖으로 나간 픽셀은 버리고 감싸 돌리지 않는다."""
    out = np.zeros_like(m)
    H, W = m.shape
    if abs(du) >= W or abs(dv) >= H:
        return out
    src = m[max(-dv, 0):H - max(dv, 0), max(-du, 0):W - max(du, 0)]
    out[max(dv, 0):max(dv, 0) + src.shape[0], max(du, 0):max(du, 0) + src.shape[1]] = src
    return out


def _footprint_mask(c, rot, size, up, intr, T_base2cam, H, W):
    """설계 자리 윗면(네 모서리)을 영상에 투영한 볼록 사각형의 bool 마스크 (H, W) — 마스크 고르기 기준. 모서리가 카메라 뒤면 빈 마스크."""
    names = ('LENGTH', 'WIDTH', 'THICKNESS')
    i, j = (names.index(k) for k in TOP_AXES[up])
    k = names.index(up)
    corners = []
    for si in (-1.0, 1.0):
        for sj in (-1.0, 1.0):
            loc = np.zeros(3)
            loc[k], loc[i], loc[j] = size[k] / 2.0, si * size[i] / 2.0, sj * size[j] / 2.0
            uv = _project(c + rot @ loc, intr, T_base2cam)
            if uv is None:
                return np.zeros((H, W), bool)
            corners.append(uv)
    corners = np.array(corners)[[0, 1, 3, 2]]                             # 둘레 차례(−− · −+ · ++ · +−)
    vv, uu = np.mgrid[0:H, 0:W]
    inside = np.ones((H, W), bool)
    ccw = _signed_area(corners) >= 0
    for (u1, v1), (u2, v2) in zip(corners, np.roll(corners, -1, axis=0)):
        cross = (u2 - u1) * (vv - v1) - (v2 - v1) * (uu - u1)              # 변마다 같은 쪽에 있으면 안
        inside &= (cross >= 0) if ccw else (cross <= 0)
    return inside


def _signed_area(poly):
    """다각형 꼭짓점 (N, 2)의 부호 있는 넓이 — 꼭짓점 차례 방향을 알려고."""
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _project(p, intr, T_base2cam):
    """base 점 p (3,) m → 영상 픽셀 (u, v) 실수. 카메라 뒤면 None. T_base2cam 은 카메라 → base(depth_to_base_points 와 같은 뜻)."""
    fx, fy, cx, cy = intr
    T = np.asarray(T_base2cam, float)
    pc = T[:3, :3].T @ (np.asarray(p, float) - T[:3, 3])
    if pc[2] <= 1e-6:
        return None
    return fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy


def _rays_to_plane(u, v, intr, T_base2cam, z):
    """픽셀 (u, v) 배열의 카메라 광선이 base 평면 z(m)와 만나는 점 xy (N, 2) m. 평면과 나란하거나 카메라 뒤인 광선은 뺀다."""
    fx, fy, cx, cy = intr
    T = np.asarray(T_base2cam, float)
    d = np.c_[(u - cx) / fx, (v - cy) / fy, np.ones(len(u))] @ T[:3, :3].T     # base 방향(길이는 상관없음)
    o = T[:3, 3]
    with np.errstate(divide='ignore', invalid='ignore'):
        s = (z - o[2]) / d[:, 2]
    ok = np.isfinite(s) & (s > 0)
    return o[:2] + d[ok, :2] * s[ok, None]


def measure_placed_mask_detail(pts_img, masks, part, block_actual_m, intr, T_base2cam, px_shift=(0, 0)):
    """measure_placed_mask 와 같은 일 + 어느 방법으로 쟀는지(info dict).

    입력: pts_img = base 점 (H, W, 3) m(깊이 없는 픽셀 NaN — block_finder.base_image 와 같은 꼴) · masks = YOLO 마스크 목록(bool (H, W),
          빈 마스크 · None 허용) · part = 설계 자리 {part_id, center (x, y, z) m, rot 3x3} · block_actual_m = [길이, 폭, 두께] m ·
          intr = (fx, fy, cx, cy) 픽셀 · T_base2cam = 카메라 → base 4x4 m · px_shift = 마스크를 옮길 (du, dv) 픽셀(실수 가능, 컬러 ↔ 깊이 정렬 보정).
    순서: ① measure_placed(깊이) — 거절이 DEPTH_FINAL(NO_PART · COVERED · STANDING · FEW_POINTS · TILTED)이면 그대로 돌려준다.
          SIZE_MISMATCH · NEIGHBOR_MERGED · 통과면 ② 로.
          ② 마스크를 px_shift(정수 픽셀로 반올림) 만큼 옮기고, 설계 자리 윗면을 영상에 투영한 사각형과 가장 많이 겹치는 마스크를 고른다
             (설계 자리의 MIN_FOOT_OVERLAP 미만이면 깊이 결과로). ③ 윗면 높이 = 그 마스크를 INNER_ERODE_PX 깎은 안쪽 깊이 점 중 설계 윗면 ± TOP_TOL_M
             인 점 z 의 가운데 값. ④ 마스크 픽셀마다 광선 → z = 윗면 높이 평면과 만나는 xy. ⑤ xy 주축(PCA)으로 yaw · 양 끝 백분위
             (MASK_EDGE_PCT)로 두 변 길이 · 가운데. ⑥ 크기가 블록보다 SIZE_TOL_M 넘게 크거나 작으면 마스크를 믿지 않고
             깊이 결과를 그대로 돌려준다(깊이가 통과한 블록을 거절로 바꾸지 않음).
    출력: (블록 dict 또는 None, 거절 (코드, 설명) 또는 None, info) — 앞 둘은 measure_placed 와 같은 꼴 · 같은 키(corners_z_m 포함 —
          칸 위치는 마스크로 잰 가운데 · yaw, 높이는 깊이 점).
          info = {'method': 'mask' | 'depth', 'reason': 깊이 결과를 쓴 까닭(마스크로 쟀으면 ''), 'depth_reject': 깊이 거절 코드 또는 None,
                  'depth_size_mm': 깊이로 잰 두 변(통과했을 때) 또는 None, 'mask_px'(옮겨서 영상 안에 남은 픽셀), 'inner_pts',
                  'center_px': 투영 픽셀 [u, v] 또는 None, 'shift_px': [du, dv](정수), 'mask_size_mm': 마스크로 잰 두 변(광선까지 갔을 때)}.
    실패 때: 마스크가 없거나 · 설계 가운데를 덮는 마스크가 없거나 · 투영이 영상 밖이거나 · 안쪽 윗면 점이 모자라면 깊이 결과를 그대로
             돌려주고 info['method'] = 'depth' · reason 에 까닭(마스크 크기가 블록과 안 맞을 때도 같다). part 칸이 없으면 KeyError(measure_placed 와 같음). 바깥 영향 없음."""
    pts_img = np.asarray(pts_img, float)
    du, dv = (int(round(float(s))) for s in px_shift)                  # 정수 픽셀 하나로 — 마스크 고르기 · 안쪽 높이 · 광선이 같은 옮김을 쓴다
    blk0, rej0 = measure_placed(pts_img.reshape(-1, 3), part, block_actual_m)
    info = {'method': 'depth', 'reason': '', 'depth_reject': rej0[0] if rej0 else None,
            'depth_size_mm': blk0['size_mm'] if blk0 else None, 'mask_px': 0, 'inner_pts': 0, 'center_px': None, 'shift_px': [du, dv]}

    def depth(reason):
        """깊이 결과를 그대로 돌려준다(까닭을 info 에 남김)."""
        info['reason'] = reason
        return blk0, rej0, info

    if rej0 is not None and rej0[0] in DEPTH_FINAL:
        return depth('깊이 거절 %s 은 깊이가 정한다' % rej0[0])
    if pts_img.ndim != 3 or pts_img.shape[2] != 3:
        return depth('점이 (H, W, 3) 영상 꼴이 아니다')
    H, W = pts_img.shape[:2]
    size = np.array([float(v) for v in block_actual_m])
    rot = np.asarray(part['rot'], float)
    up = up_of(rot)
    c = np.array([float(v) for v in part['center']])
    top_design = c[2] + float(np.abs(rot[2]) @ size) / 2.0
    a, b = (size[UP_DIM[k]] for k in TOP_AXES[up])                      # 윗면 두 변(긴 · 짧은)
    uv = _project((c[0], c[1], top_design), intr, T_base2cam)
    if uv is None:
        return depth('설계 가운데가 카메라 뒤')
    u0, v0 = int(round(uv[0])), int(round(uv[1]))
    info['center_px'] = [round(uv[0], 1), round(uv[1], 1)]
    if not (0 <= u0 < W and 0 <= v0 < H):
        return depth('설계 가운데 투영 (%d, %d) 이 영상 밖' % (u0, v0))
    foot = _footprint_mask(c, rot, size, up, intr, T_base2cam, H, W)   # 설계 자리 윗면을 영상에 투영한 사각형
    best, shifted, best_n = None, None, 0
    for m in masks or []:
        if m is None:
            continue
        m = np.asarray(m, bool)
        if m.shape != (H, W):
            continue
        sm = _shift_mask(m, du, dv)
        n = int(np.count_nonzero(sm & foot))
        if n > best_n:                                                    # 설계 자리와 가장 많이 겹치는 마스크 — 붙은 이웃 마스크가 먼저 와도 안 뽑힌다
            best, shifted, best_n = m, sm, n
    n_foot = int(np.count_nonzero(foot))
    if best is None or best_n < MIN_FOOT_OVERLAP * n_foot:
        return depth('설계 자리(%d px)의 %.0f %% 넘게 덮는 마스크가 없다(마스크 %d 개, 가장 많이 %d px)' % (n_foot, MIN_FOOT_OVERLAP * 100, len(masks or []), best_n))
    mask = best
    info['mask_px'] = int(np.count_nonzero(shifted))                     # 옮겨서 영상 안에 남은 픽셀 — 깎기 · 광선도 이 마스크
    if info['mask_px'] < MIN_MASK_PX:
        return depth('마스크 픽셀 %d < %d' % (info['mask_px'], MIN_MASK_PX))
    vs, us = np.nonzero(shifted)                                          # 깎기는 마스크를 감싼 창 안에서만(640×480 전체보다 빠름)
    pad = INNER_ERODE_PX + 1
    v_lo, v_hi, u_lo, u_hi = max(vs.min() - pad, 0), min(vs.max() + pad + 1, H), max(us.min() - pad, 0), min(us.max() + pad + 1, W)
    inner = _erode(shifted[v_lo:v_hi, u_lo:u_hi], INNER_ERODE_PX)
    z = pts_img[v_lo:v_hi, u_lo:u_hi, 2][inner]
    z = z[np.isfinite(z) & (np.abs(z - top_design) <= TOP_TOL_M)]      # 아래층 · 위 물체 점은 빼고 이 블록 윗면만
    info['inner_pts'] = int(len(z))
    if len(z) < MIN_INNER_PTS:
        return depth('마스크 안쪽 윗면 점 %d < %d' % (len(z), MIN_INNER_PTS))
    top_z = float(np.median(z))
    xy = _rays_to_plane(us.astype(float), vs.astype(float), intr, T_base2cam, top_z)   # 옮긴 마스크 픽셀 그대로(영상 밖으로 나간 픽셀은 없다)
    if len(xy) < MIN_MASK_PX:
        return depth('윗면 평면과 만나는 광선 %d < %d' % (len(xy), MIN_MASK_PX))
    mean = xy.mean(axis=0)
    w, vec = np.linalg.eigh(np.cov((xy - mean).T))
    ax_u = vec[:, int(np.argmax(w))]
    ax_v = np.array([-ax_u[1], ax_u[0]])
    lo_u, hi_u = np.percentile((xy - mean) @ ax_u, MASK_EDGE_PCT)
    lo_v, hi_v = np.percentile((xy - mean) @ ax_v, MASK_EDGE_PCT)
    span_u, span_v = (hi_u - lo_u) / MASK_EDGE_KEEP, (hi_v - lo_v) / MASK_EDGE_KEEP
    info['mask_size_mm'] = [round(span_u * 1000.0, 1), round(span_v * 1000.0, 1)]
    # 마스크 크기가 블록과 안 맞으면(이웃과 한 마스크로 붙음 · 일부만 덮음) 마스크를 믿지 않고 깊이 결과로 — 깊이가 통과한 블록을 거절로 바꾸지 않는다
    if span_u > a + SIZE_TOL_M or span_v > b + SIZE_TOL_M:
        return depth(f'마스크 윗면 {span_u * 1000:.0f} × {span_v * 1000:.0f} mm > 블록 {a * 1000:.0f} × {b * 1000:.0f} mm — 이웃과 한 마스크로 붙음')
    if span_u < a - SIZE_TOL_M or span_v < b - SIZE_TOL_M:
        return depth(f'마스크 윗면 {span_u * 1000:.0f} × {span_v * 1000:.0f} mm < 블록 {a * 1000:.0f} × {b * 1000:.0f} mm — 마스크가 블록 일부만 덮음')
    info['method'] = 'mask'
    ctr = mean + ax_u * (lo_u + hi_u) / 2 + ax_v * (lo_v + hi_v) / 2
    yaw = round(wrap_deg(math.degrees(math.atan2(ax_u[1], ax_u[0]))), 1)
    yaw = yaw if yaw < 90.0 else -90.0                                   # 반올림으로 90.0 이 되면 −90(measure_placed 와 같은 규칙)
    p_all = pts_img.reshape(-1, 3)
    p_all = p_all[np.isfinite(p_all).all(axis=1)]
    return {
        'part_id': part['part_id'],
        'x_m': round(float(ctr[0]), 4),
        'y_m': round(float(ctr[1]), 4),
        'top_z_m': round(top_z, 4),
        'yaw_deg': yaw,
        'up': up,
        'tilted': False,
        'confidence': 1.0,
        'size_mm': [round(span_u * 1000.0, 1), round(span_v * 1000.0, 1)],
        'corners_z_m': corner_heights(p_all, ctr, yaw, top_design, a, b),   # 높이는 깊이에서, 칸은 마스크로 잰 가운데 · yaw(한석형 #190)
    }, None, info


def measure_placed_mask(pts_img, masks, part, block_actual_m, intr, T_base2cam, px_shift=(0, 0)):
    """설계 자리에 놓인 블록 하나를 마스크 + 광선으로 잰다(기울기 · 덮임 · 자세는 깊이) — measure_placed 와 같은 꼴로 돌려준다.

    입력 · 순서 · 실패 때는 measure_placed_mask_detail 과 같다. 출력: (블록 dict, None) 또는 (None, (거절 코드, 설명)) —
    키 {part_id, x_m, y_m, top_z_m, yaw_deg, up, tilted, confidence, size_mm}. 어느 방법으로 쟀는지는 _detail 의 info 에만 있다. 바깥 영향 없음."""
    blk, rej, _ = measure_placed_mask_detail(pts_img, masks, part, block_actual_m, intr, T_base2cam, px_shift)
    return blk, rej

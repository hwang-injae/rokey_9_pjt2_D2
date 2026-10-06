"""블록 있음·없음·높이 판정 — ROS 없는 계산 (W041, SDD §6.3). 노드 wrist_block 이 쓴다.

흐름: 깊이 영상 → base 점군 → 높이 지도(5 mm 격자, 셀마다 최고 z) → 레시피 블록마다 설계 자리(ROI)의 높이 중앙값
      → 설계 윗면 높이와 비교 → state(present · absent · occluded · unknown) + top_z_m · dz_m. dx_m · dy_m 은 NaN(10/8 W100 전).

단위: 이 파일 안은 모두 m · rad (노드끼리 규칙). 레시피(mm)는 읽을 때 m 로 바꾼다.
좌표: base_link. 설계 좌표(cad) → base 는 assembly_origin(x, y, z, yaw) 하나로 바꾼다 (IRD 9장).
높이 지도는 스캔 추론기(StructureScanner, W115)도 같은 함수를 쓴다.
"""
import math

import numpy as np

NAN = float('nan')
CELL_M = 0.005          # 높이 지도 셀 (SDD §6.9 ②: 5 mm)
ROI_SHRINK_M = 0.004    # 블록 자리를 사방 4 mm 줄여서 본다 — 가장자리 깊이 튐 · 1~3 mm 어긋남을 피한다
MIN_CELLS = 6           # 이보다 적은 셀이면 unknown (가려짐 · 깊이 구멍)
TOL_FRACTION = 0.5      # 높이 허용 = 두께의 절반 (SDD §6.3 "기준 안: 두께 절반")


def depth_to_base_points(depth_m, intr, T_base2cam, stride=2, z_min_m=0.1, z_max_m=1.0):
    """깊이 영상 → base 점군. 입력: depth_m (H, W) m, intr (fx, fy, ppx, ppy), T_base2cam 4x4 (m).
    stride 픽셀마다 하나씩 뽑고, 깊이 0 과 범위 밖은 버린다. 출력: (N, 3) m."""
    fx, fy, ppx, ppy = intr
    d = depth_m[::stride, ::stride]
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    ok = (d > z_min_m) & (d < z_max_m)
    z = d[ok]
    pts_cam = np.stack([(u[ok] - ppx) * z / fx, (v[ok] - ppy) * z / fy, z, np.ones_like(z)], axis=1)
    return (pts_cam @ T_base2cam.T)[:, :3]


def height_map(points, origin_xy, half_m, cell_m=CELL_M):
    """점군 → 높이 지도. origin_xy 를 가운데로 ± half_m 영역을 cell_m 격자로 나눠 셀마다 **최고 z** 를 둔다.
    입력 points (N, 3) m. 출력: (grid (n, n) m — 점 없는 셀은 NaN, x0, y0) — x0·y0 는 grid[0][0] 셀의 왼쪽 아래 모서리.
    '최고 z' 인 이유: 블록 옆면 · 아래층이 섞여 보여도 윗면이 가장 높아서 윗면 높이가 남는다."""
    n = int(round(2 * half_m / cell_m))
    x0, y0 = origin_xy[0] - half_m, origin_xy[1] - half_m
    grid = np.full((n, n), -np.inf)        # NaN 으로 시작하면 max(NaN, z) 가 NaN 이라 -inf 로 시작한다
    if len(points) > 0:
        ix = np.floor((points[:, 0] - x0) / cell_m).astype(int)
        iy = np.floor((points[:, 1] - y0) / cell_m).astype(int)
        ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
        np.maximum.at(grid, (ix[ok], iy[ok]), points[ok, 2])
    grid[np.isinf(grid)] = NAN             # 점이 없는 셀
    return grid, x0, y0


def _cad_to_base(p_cad_m, origin):
    """설계 좌표 점(m) → base. origin = dict(x_m, y_m, z_m, yaw_deg) (robot.yaml assembly_origin)."""
    c, s = math.cos(math.radians(origin['yaw_deg'])), math.sin(math.radians(origin['yaw_deg']))
    return (origin['x_m'] + c * p_cad_m[0] - s * p_cad_m[1],
            origin['y_m'] + s * p_cad_m[0] + c * p_cad_m[1],
            origin['z_m'] + p_cad_m[2])


class BlockChecker:
    """레시피 블록마다 설계 자리의 높이를 보고 있음 · 없음을 판정한다 (로봇 · ROS 없음).

    입력: recipe = cad_recipe/1.0 dict(mm), origin = assembly_origin dict(m · deg), half_m = 조립 작업공간 반폭.
    check(points, block_ids) → 블록마다 dict(block_id, state, top_z_m, dz_m, dx_m, dy_m). dx · dy 는 늘 NaN(1차).
    바깥 영향 없음. 레시피에 없는 block_id 는 state unknown, 높이 NaN."""

    def __init__(self, recipe, origin, half_m, cell_m=CELL_M):
        self.origin, self.half_m, self.cell_m = origin, half_m, cell_m
        self.blocks = {}
        for b in recipe['blocks']:
            size = np.array(b['size_lwt_mm']) / 1000.0
            R = np.array(b['R_cad_from_block'])
            ext = np.abs(R) @ size / 2.0                        # 설계 x · y · z 축 방향 반폭 (m)
            cx, cy, cz = _cad_to_base(np.array(b['center_cad_mm']) / 1000.0, origin)
            self.blocks[b['block_id']] = {
                'center': (cx, cy), 'half_xy': (ext[0], ext[1]),
                'top_z': cz + ext[2], 'height': 2 * ext[2],
                'cad_c': np.array(b['center_cad_mm']) / 1000.0, 'cad_ext': ext, 'above_tops': [],
            }
        # 설계상 이 블록 위에 올라가는 블록들의 윗면 높이 — 가려진 블록을 '받침 있음'으로 추정할 때 쓴다
        for me in self.blocks.values():
            for other in self.blocks.values():
                if other is me:
                    continue
                overlap = np.all(np.abs(other['cad_c'][:2] - me['cad_c'][:2]) < me['cad_ext'][:2] + other['cad_ext'][:2] - 1e-6)
                if overlap and other['cad_c'][2] - other['cad_ext'][2] >= me['cad_c'][2] + me['cad_ext'][2] - 1e-6:
                    me['above_tops'].append(other['top_z'])

    def _roi_cells(self, grid, x0, y0, blk):
        """블록 자리(사방 ROI_SHRINK_M 줄임) 안에 들어가는 높이 지도 셀들의 z. 조립 원점 yaw 만큼 돌려서 본다."""
        n = grid.shape[0]
        xs = x0 + (np.arange(n) + 0.5) * self.cell_m
        X, Y = np.meshgrid(xs, y0 + (np.arange(n) + 0.5) * self.cell_m, indexing='ij')
        yaw = math.radians(self.origin['yaw_deg'])
        dx, dy = X - blk['center'][0], Y - blk['center'][1]
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy           # 셀 중심을 설계 축으로
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        hx, hy = blk['half_xy'][0] - ROI_SHRINK_M, blk['half_xy'][1] - ROI_SHRINK_M
        inside = (np.abs(lx) <= hx) & (np.abs(ly) <= hy)
        z = grid[inside]
        return z[~np.isnan(z)]

    def check(self, points, block_ids):
        """점군(base, m)으로 block_ids 를 판정한다. 출력은 요청 순서대로 같은 길이.
        판정: |측정 − 설계 윗면| ≤ 두께/2 → present · 더 낮음 → absent · 셀 부족 → unknown
              · 더 높음: 설계상 위에 올라가는 블록의 윗면과 맞으면 present(가려짐 — 위 블록이 그 높이에 있으면 받침도 있다,
                top_z · dz 는 못 재서 NaN) · 설계에 없는 높이면 occluded(설계 밖 물체 · 손)."""
        grid, x0, y0 = height_map(points, (self.origin['x_m'], self.origin['y_m']), self.half_m, self.cell_m)
        out = []
        for bid in block_ids:
            blk = self.blocks.get(bid)
            if blk is None:
                out.append(dict(block_id=bid, state='unknown', top_z_m=NAN, dz_m=NAN, dx_m=NAN, dy_m=NAN))
                continue
            z = self._roi_cells(grid, x0, y0, blk)
            if len(z) < MIN_CELLS:
                out.append(dict(block_id=bid, state='unknown', top_z_m=NAN, dz_m=NAN, dx_m=NAN, dy_m=NAN))
                continue
            top = float(np.median(z))
            dz = top - blk['top_z']
            tol = blk['height'] * TOL_FRACTION
            if abs(dz) <= tol:
                out.append(dict(block_id=bid, state='present', top_z_m=top, dz_m=dz, dx_m=NAN, dy_m=NAN))
            elif dz < 0:
                out.append(dict(block_id=bid, state='absent', top_z_m=top, dz_m=NAN, dx_m=NAN, dy_m=NAN))
            elif any(abs(top - t) <= tol for t in blk['above_tops']):
                out.append(dict(block_id=bid, state='present', top_z_m=NAN, dz_m=NAN, dx_m=NAN, dy_m=NAN))   # 가려짐
            else:
                out.append(dict(block_id=bid, state='occluded', top_z_m=top, dz_m=NAN, dx_m=NAN, dy_m=NAN))
        return out

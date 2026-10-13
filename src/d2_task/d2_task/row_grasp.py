# -*- coding: utf-8 -*-
"""연짝 잡기 규칙 — 같은 층에 긴 면을 맞대고 나란히 놓는 눕힌 블록(연짝)은 모두 FLAT_LONG (10/13 21시 PL 확정, ROS · d2_motion 없음).

연짝 3개가 FLAT_SHORT · FLAT_LONG · FLAT_LONG 으로 섞이면 첫 블록만 손목 방향 · 닫힘 축이 달라진다 → 셋 다 FLAT_LONG.
설계 자체를 고친다(작업 관리자는 실행 때 잡기를 바꾸지 않는다): HMI(web backend)가 AI · 스캔 설계를 저장할 때와 저장된 설계를
한 번에 고치는 스크립트에서 이 파일을 import 해 쓰고, 기본 설계 4개는 레시피 파일(CAD 속성 GRASP)로 맞춘다.
입력은 parts/4.0 의 parts 목록(설계 좌표 mm — x · y = 바닥 가운데, z = 아랫면, ori = 방향 코드, IRD 6장). 바깥 영향 없음.
"""

FLAT_ORIS = ('x', 'y')     # 눕힘(두께가 위) — x 는 긴 축이 설계 x, y 는 긴 축이 설계 y (IRD 2장 ori)
LAYER_TOL_MM = 1.0         # 같은 층 = 아랫면 높이 차가 이 안
SIDE_TOL_MM = 3.0          # 긴 면을 맞댐 = 폭 방향 중심 거리가 설계 폭 ± 이 값(틈 0 ~ 몇 mm)


def row_orders(parts, block_mm):
    """연짝에 든 눕힌 블록의 sequence 집합.

    연짝 = 두 블록이 모두 눕힘 · 같은 방향(ori) · 같은 층(아랫면 차 LAYER_TOL_MM 안)이고, 폭 방향 중심 거리가 설계 폭(±SIDE_TOL_MM),
    길이 방향 어긋남이 길이 절반보다 작음(긴 면이 서로 마주 봄). 입력: parts = parts/4.0 parts 목록, block_mm = [길이, 폭, 두께] mm
    (robot.yaml block_size_m × 1000). 출력: set(sequence). 칸이 빠진 블록은 KeyError(부르는 쪽 형식 검사가 먼저다).
    """
    length, width = block_mm[0], block_mm[1]
    flat = [p for p in parts if p['ori'] in FLAT_ORIS]
    rows = set()
    for a in flat:
        for b in flat:
            if a is b or a['ori'] != b['ori'] or abs(a['z'] - b['z']) > LAYER_TOL_MM:
                continue
            dx, dy = abs(b['x'] - a['x']), abs(b['y'] - a['y'])
            along_l, along_w = (dx, dy) if a['ori'] == 'x' else (dy, dx)
            if abs(along_w - width) <= SIDE_TOL_MM and along_l < length / 2:
                rows.add(a['sequence'])
                break
    return rows


def fix_row_grasps(parts, block_mm):
    """연짝 블록 중 grasp 가 FLAT_SHORT 인 것을 FLAT_LONG 으로 바꾼 새 parts 목록과 바꾼 sequence 목록.

    입력 목록은 바꾸지 않는다(블록마다 얕은 복사). 연짝이 아닌 블록 · 이미 FLAT_LONG 인 블록은 그대로.
    반환: (새 parts 목록, [바꾼 sequence …] — sequence 순).
    """
    rows = row_orders(parts, block_mm)
    out, changed = [], []
    for p in parts:
        q = dict(p)
        if p['sequence'] in rows and p.get('grasp') == 'FLAT_SHORT':
            q['grasp'] = 'FLAT_LONG'
            changed.append(p['sequence'])
        out.append(q)
    return out, sorted(changed)

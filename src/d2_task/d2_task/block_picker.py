# -*- coding: utf-8 -*-
"""집을 블록 고르기 BlockPicker — find_blocks 결과에서 다음 블록을 집을 후보 하나를 고른다 (ROS 없이 동작, W130, E-46 · E-49).

task(작업 판단)가 흩뿌린 공급 영역을 본 뒤 부른다. 점수식 없이 거르고 정렬한다. 로봇 · 메시지에는 손대지 않는다(계산만).
고르는 규칙(10/7 PL):
  ① 제외: 값이 잘못된 것 · overlap=under(다른 블록에 덮임) · tilted(걸쳐 기울어짐)
  ② 목표 자세(up)와 잡는 축에 맞고, 그 축의 clear=true 이며 gap_mm ≥ find.min_gap_mm 인 것만 남긴다
  ③ overlap=none 후보가 있으면 그중에서, 없으면 overlap=top 후보에서 gap_mm 이 가장 넓은 것
  ④ 틈까지 같으면 윗면(top_z_m)이 높은 것, 그래도 같으면 입력 순서가 앞선 것
후보가 없으면 다른 자세를 바로 집지 않고 '없음'을 돌려준다(세우기 연결 전에는 공급 안내로 처리).
find_blocks 응답에 없는 정보(예: 기울어진 블록이 걸친 아래 블록)는 만들어 쓰지 않는다 — 응답이 overlap=under 로 알려 줘야 제외된다.
"""
import copy
import math

UPS = ('THICKNESS', 'WIDTH', 'LENGTH')
AXES = ('LENGTH', 'WIDTH', 'THICKNESS')            # 잡는 축 = 손가락이 닫히는 블록 치수. IRD 4.2 예시는 LENGTH · WIDTH 만 보여 주므로 THICKNESS 틈은 응답에 있을 때만 쓴다
OVERLAPS = ('none', 'top', 'under')
# 잡기 종류 → 위를 향한 블록 면(IRD 2장: 눕힘 FLAT = 두께가 위, 옆세움 EDGE = 폭이 위, 세움 STAND = 길이가 위)
UP_OF_GRASP = {'FLAT': 'THICKNESS', 'EDGE': 'WIDTH', 'STAND': 'LENGTH'}


def grasp_target(grasp, cfg):
    """잡기 이름(예 FLAT_LONG) → (up, 잡는 축). 잡는 축 = 손가락이 닫히는 블록 치수(grasp_width_m)가 길이 · 폭 · 두께 중 어느 것인지.

    cfg = robot.yaml dict(block_size_m [길이, 폭, 두께] · grasp_width_m). 기하로 정해지는 값이다:
    FLAT_SHORT = (THICKNESS, WIDTH) · FLAT_LONG = (THICKNESS, LENGTH) · EDGE_SHORT = (WIDTH, THICKNESS) · EDGE_LONG = (WIDTH, LENGTH) ·
    STAND_SHORT = (LENGTH, THICKNESS) · STAND_LONG = (LENGTH, WIDTH). 잡는 축이 THICKNESS 인 잡기는 find_blocks 응답에
    그 방향 틈이 있을 때만 후보가 되고, 없으면 pick 이 NONE(NO_GAP_INFO)으로 처리한다 — 틈 값을 만들어 쓰지 않는다.
    """
    kind = grasp.split('_')[0]
    if kind not in UP_OF_GRASP or grasp not in cfg['grasp_width_m']:
        raise ValueError(f'모르는 잡기: {grasp!r}')
    length, width, thick = cfg['block_size_m']
    close = cfg['grasp_width_m'][grasp]
    for axis, size in (('LENGTH', length), ('WIDTH', width), ('THICKNESS', thick)):
        if abs(close - size) < 1e-6:
            return UP_OF_GRASP[kind], axis
    raise ValueError(f'{grasp}: 닫는 치수 {close} m 가 블록 길이 · 폭 · 두께 어느 것과도 다르다')


def _num(v):
    """bool 이 아닌 유한한 숫자인가."""
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        return False
    try:
        return math.isfinite(v)
    except OverflowError:                       # 10**1000 같은 큰 정수는 float 로 못 바꾼다 → 잘못된 값
        return False


class BlockPicker:
    """find_blocks 의 blocks 목록에서 집을 후보를 고른다.

    입력: min_gap_mm(robot.yaml find.min_gap_mm — 집을 틈 기준, mm). 입력 blocks 는 바꾸지 않는다.
    """

    def __init__(self, min_gap_mm):
        """기준 틈을 받는다. 유한한 양수가 아니면 ValueError(설정 오류)."""
        if not _num(min_gap_mm) or min_gap_mm <= 0:
            raise ValueError(f'find.min_gap_mm 이 양수가 아니다: {min_gap_mm!r}')
        self.min_gap_mm = min_gap_mm

    def pick(self, blocks, up, axis):
        """집을 블록을 고른다. up = 목표 자세(THICKNESS · WIDTH · LENGTH), axis = 잡는 축(LENGTH · WIDTH · THICKNESS, up 과 달라야 한다).
        잡는 축이 THICKNESS 이면 응답에 그 방향 clear · gap_mm 칸이 있는 블록만 후보가 된다(없으면 counts['no_gap_info']).

        반환: {'status': 'FOUND', 'index': 입력 번호, 'block': 그 dict(복사), 'overlap': 'none'|'top', 'gap_mm': 그 축 틈}
              또는 {'status': 'NONE', 'reason': 이유 글자, 'counts': {거른 이유: 개수}}.
        reason: EMPTY(블록 없음) · NO_MATCH(맞는 후보 없음). 잘못된 입력 블록은 건너뛰고 counts['invalid'] 에 센다.
        blocks 가 목록이 아니거나 up · axis 가 틀리거나 같으면 ValueError(부르는 쪽 실수). yaw_deg 는 −90 ≤ yaw < 90 밖이면 잘못된 값이다(IRD 4.2).
        """
        if up not in UPS or axis not in AXES:
            raise ValueError(f'up={up!r} axis={axis!r}')
        if axis == up:                           # 위를 향한 면과 손가락이 닫히는 면이 같을 수는 없다
            raise ValueError(f'잡는 축이 위를 향한 면과 같다: {axis!r}')
        if not isinstance(blocks, list):
            raise ValueError(f'blocks 가 목록이 아니다: {type(blocks).__name__}')     # 부르는 쪽 실수 — 빈 공급(EMPTY)과 섞지 않는다
        if not blocks:
            return {'status': 'NONE', 'reason': 'EMPTY', 'counts': {}}
        counts = {'invalid': 0, 'under': 0, 'tilted': 0, 'other_up': 0, 'no_gap_info': 0, 'no_clear': 0, 'narrow': 0}
        cands = []
        for i, b in enumerate(blocks):
            gap = self._gap(b, axis)
            if gap == 'no_info':
                counts['no_gap_info'] += 1
            elif gap is None:
                counts['invalid'] += 1
            elif b['overlap'] == 'under':
                counts['under'] += 1
            elif b['tilted']:
                counts['tilted'] += 1
            elif b['up'] != up:
                counts['other_up'] += 1
            elif b['clear'][axis] is not True:
                counts['no_clear'] += 1
            elif gap < self.min_gap_mm:
                counts['narrow'] += 1
            else:
                cands.append((i, b, gap))
        pool = [c for c in cands if c[1]['overlap'] == 'none'] or cands       # none 우선, 없으면 top
        if not pool:
            return {'status': 'NONE', 'reason': 'NO_MATCH', 'counts': counts}
        i, b, gap = max(pool, key=lambda c: (c[2], c[1]['top_z_m'], -c[0]))    # 틈 → 윗면 높이 → 입력 순서(앞이 우선)
        return {'status': 'FOUND', 'index': i, 'block': copy.deepcopy(b), 'overlap': b['overlap'], 'gap_mm': gap}

    @staticmethod
    def _gap(b, axis):
        """블록 하나의 형식이 맞으면 axis 방향 틈(mm), 잡는 축 THICKNESS 인데 응답에 그 방향 틈이 없으면 'no_info', 형식이 틀리면 None.

        위치 · yaw · 높이 · 틈은 유한한 숫자(yaw 는 −90 ≤ yaw < 90), 나머지 칸은 정해진 값이어야 한다.
        """
        try:
            if not all(_num(b[k]) for k in ('x_m', 'y_m', 'top_z_m', 'yaw_deg')) or not -90.0 <= b['yaw_deg'] < 90.0:
                return None
            if b['up'] not in UPS or b['overlap'] not in OVERLAPS or not isinstance(b['tilted'], bool):
                return None
            clear, gap = b['clear'], b['gap_mm']
            if not isinstance(clear, dict) or not isinstance(gap, dict):
                return None
            for a in AXES:
                if a in clear or a in gap:                 # 있는 칸은 형식이 맞아야 한다(LENGTH · WIDTH 는 꼭 있어야 한다)
                    if not isinstance(clear.get(a), bool) or not _num(gap.get(a)):
                        return None
            if not all(a in clear and a in gap for a in ('LENGTH', 'WIDTH')):
                return None
            if axis not in gap or axis not in clear:
                return 'no_info'
            return gap[axis]
        except (KeyError, TypeError, AttributeError):
            return None

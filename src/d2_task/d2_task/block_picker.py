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
AXES = ('LENGTH', 'WIDTH')                       # find_blocks 가 틈을 주는 방향(IRD 4.2). 두께(THICKNESS) 방향은 응답에 없다
OVERLAPS = ('none', 'top', 'under')
# 잡기 종류 → 위를 향한 블록 면(IRD 2장: 눕힘 FLAT = 두께가 위, 옆세움 EDGE = 폭이 위, 세움 STAND = 길이가 위)
UP_OF_GRASP = {'FLAT': 'THICKNESS', 'EDGE': 'WIDTH', 'STAND': 'LENGTH'}


def grasp_target(grasp, cfg):
    """잡기 이름(예 FLAT_LONG) → (up, 잡는 축). 잡는 축 = 손가락이 닫히는 블록 치수(grasp_width_m)가 길이 · 폭 중 어느 것인지.

    cfg = robot.yaml dict(block_size_m [길이, 폭, 두께] · grasp_width_m). 닫는 치수가 두께(15 mm)이면 find_blocks 가 그 방향 틈을
    주지 않으므로 ValueError — 알 수 없는 것을 추측하지 않는다(EDGE_SHORT · STAND_SHORT, 민범진 확인 대기).
    """
    kind = grasp.split('_')[0]
    if kind not in UP_OF_GRASP or grasp not in cfg['grasp_width_m']:
        raise ValueError(f'모르는 잡기: {grasp!r}')
    length, width, _thick = cfg['block_size_m']
    close = cfg['grasp_width_m'][grasp]
    for axis, size in (('LENGTH', length), ('WIDTH', width)):
        if abs(close - size) < 1e-6:
            return UP_OF_GRASP[kind], axis
    raise ValueError(f'{grasp}: 닫는 치수 {close} m 가 길이 · 폭이 아니라 find_blocks 틈 방향을 알 수 없다')


def _num(v):
    """bool 이 아닌 유한한 숫자인가."""
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


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
        """집을 블록을 고른다. up = 목표 자세(THICKNESS · WIDTH · LENGTH), axis = 잡는 축(LENGTH · WIDTH).

        반환: {'status': 'FOUND', 'index': 입력 번호, 'block': 그 dict(복사), 'overlap': 'none'|'top', 'gap_mm': 그 축 틈}
              또는 {'status': 'NONE', 'reason': 이유 글자, 'counts': {거른 이유: 개수}}.
        reason: EMPTY(블록 없음) · NO_MATCH(맞는 후보 없음). 잘못된 입력 블록은 건너뛰고 counts['invalid'] 에 센다.
        """
        if up not in UPS or axis not in AXES:
            raise ValueError(f'up={up!r} axis={axis!r}')
        if not isinstance(blocks, list) or not blocks:
            return {'status': 'NONE', 'reason': 'EMPTY', 'counts': {}}
        counts = {'invalid': 0, 'under': 0, 'tilted': 0, 'other_up': 0, 'no_clear': 0, 'narrow': 0}
        cands = []
        for i, b in enumerate(blocks):
            gap = self._gap(b, axis)
            if gap is None:
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
        """블록 하나의 형식이 맞으면 axis 방향 틈(mm), 아니면 None. 위치 · yaw · 높이 · 틈은 유한한 숫자, 나머지 칸은 정해진 값이어야 한다."""
        try:
            if not all(_num(b[k]) for k in ('x_m', 'y_m', 'top_z_m', 'yaw_deg')):
                return None
            if b['up'] not in UPS or b['overlap'] not in OVERLAPS or not isinstance(b['tilted'], bool):
                return None
            if not all(isinstance(b['clear'][a], bool) for a in AXES) or not all(_num(b['gap_mm'][a]) for a in AXES):
                return None
            return b['gap_mm'][axis]
        except (KeyError, TypeError):
            return None

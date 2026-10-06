from dataclasses import dataclass

import numpy as np

# 좌표 비교 허용오차(mm). CAD 좌표의 부동소수 오차를 흡수한다.
TOL = 1e-6


@dataclass(frozen=True)
class BoundingBox:
    """좌표축에 평행한 경계 상자. 부품이 차지하는 공간을 나타낸다.

    부품은 모두 축에 평행한 직육면체이므로, 최소·최대 꼭짓점 두 개로 정확히 표현된다.
    """

    lo: tuple[float, float, float]   # 최소 꼭짓점 (x, y, z), mm
    hi: tuple[float, float, float]   # 최대 꼭짓점 (x, y, z), mm

    def overlaps(self, other):
        """두 상자가 부피를 가진 공간을 함께 차지하는지 본다. 면끼리 맞닿기만 한 경우(쌓인 블록)는 겹침이 아니다.

        입력:
            other: 비교할 BoundingBox
        출력: 겹치면 True
        """
        # 겹치는 구간 = [두 lo 중 큰 값, 두 hi 중 작은 값]. 세 축 모두 길이가 양수여야 부피가 생긴다.
        overlap = np.minimum(self.hi, other.hi) - np.maximum(self.lo, other.lo)
        return bool((overlap > TOL).all())

    def rests_on(self, other):
        """이 상자가 other 위에 얹혀 있는지 본다.

        조건 1: 이 상자의 바닥면 높이 == other의 윗면 높이
        조건 2: 위에서 내려다볼 때(XY 평면) 두 상자가 면적을 가지고 겹친다

        입력:
            other: 아래에 있다고 볼 BoundingBox
        출력: 얹혀 있으면 True
        """
        # 높이 접촉과 XY 겹침을 함께 확인
        touching = abs(self.lo[2] - other.hi[2]) < TOL
        overlap_xy = np.minimum(self.hi[:2], other.hi[:2]) - np.maximum(self.lo[:2], other.lo[:2])
        return bool(touching and (overlap_xy > TOL).all())

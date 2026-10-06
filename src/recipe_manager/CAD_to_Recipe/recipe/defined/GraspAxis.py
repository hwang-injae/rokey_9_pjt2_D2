from enum import Enum


class GraspAxis(Enum):
    """그리퍼(RG2)가 닫히는 방향의 부품 로컬 축.

    값은 CADPartInstance.R의 열 번호다. R[:, 값]이 그 축의 모델 좌표 방향이 된다.
    예: WIDTH로 잡으면 그리퍼 손가락이 부품의 W축 양쪽 면을 집는다(벌림 폭 ≈ W 치수).
    사람은 파지 방법(GraspMethod, 예: FLAT_SHORT)을 입력하고, 이 축은 Recipe.build_from_plan가 계산한다.
    """

    LENGTH = 0      # L축 (가장 긴 축)
    WIDTH = 1       # W축
    THICKNESS = 2   # T축 (가장 짧은 축)

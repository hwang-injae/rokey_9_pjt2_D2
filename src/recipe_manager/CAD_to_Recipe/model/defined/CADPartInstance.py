from dataclasses import dataclass

import numpy as np

from model.defined.BoundingBox import BoundingBox
from model.defined.CADPartDefinition import CADPartDefinition


@dataclass(frozen=True)
class CADPartInstance:
    """모델 안에 놓이는 부품 하나. '어떤 규격(CADPartDefinition)을 어디에, 어떤 방향으로' 놓는지 담는다.

    CAD 용어로는 '부품 인스턴스'다. CAD 파일의 DXF INSERT 하나 또는 STEP 솔리드 하나가 이것 하나가 된다(1:1).
    """

    # CAD 원본 ID를 그대로 쓴다. 조립 순서가 바뀌어도 이 ID는 바뀌지 않는다.
    instance_id: str

    part: CADPartDefinition

    # 목표 위치: 부품 중심의 모델 좌표 (x, y, z), mm
    center_mm: tuple[float, float, float]

    # 목표 방향: 3×3 회전행렬 (모델 좌표 ← 부품 로컬 좌표).
    # 열 0/1/2 = 부품의 L/W/T 축이 모델 좌표에서 가리키는 방향.
    # 예: 단위행렬이면 L축이 모델 X, W축이 모델 Y, T축이 모델 Z(위)를 향한다.
    R: tuple[tuple[float, float, float], ...]

    def calculate_bounding_box(self):
        """이 부품이 목표 위치에서 차지하는 공간.

        출력: BoundingBox
        """
        # 회전된 직육면체를 모델 축에 투영한 반폭 = |R| @ (치수 / 2)
        # (축 정렬 회전만 허용하므로 |R|은 0과 1로 된 치환 행렬이고, 결과는 정확한 경계다.)
        half = np.abs(np.array(self.R)) @ (np.array(self.part.size_mm) / 2)
        center = np.array(self.center_mm)
        return BoundingBox(lo=tuple(center - half), hi=tuple(center + half))

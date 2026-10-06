from dataclasses import dataclass


@dataclass(frozen=True)
class CADPartDefinition:
    """부품 정의(규격). 같은 치수의 블록은 CADPartDefinition 하나를 공유한다.

    CAD 용어로는 '부품 정의'다. DXF의 BLOCK과 비슷하지만, BLOCK은 방향까지 담은 도형 묶음이고
    이 클래스는 방향 없이 치수로만 구분한다. 그래서 LV1 DXF의 BLOCK 11개가 정의 1개로 묶인다.
    예: LV1 벤치는 75×25×15 블록 CADPartDefinition 하나를 CADPartInstance 11개가 참조한다.
    """

    part_id: str   # 'PART_001', 'PART_002', ... (CAD에서 처음 나온 순서)

    # (L, W, T) 치수, mm. 항상 큰 순서다(L ≥ W ≥ T).
    # 부품 로컬 X축 = L, Y축 = W, Z축 = T 방향이다.
    size_mm: tuple[float, float, float]

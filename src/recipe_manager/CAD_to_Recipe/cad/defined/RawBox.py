from dataclasses import dataclass


@dataclass
class RawBox:
    """CAD 파일에서 읽은 부품 하나의 원시 데이터.

    아직 직육면체로 해석하기 전 상태다. 치수·중심·회전 계산은 Model.create_from_raw_boxes가 한다.
    """

    # CAD 안에서 이 부품을 가리키는 ID. 모델의 instance_id로 그대로 쓴다.
    #   STEP: 'STEP_SOLID_0001' (파일 안의 솔리드 순번)
    #   DXF : 'DXF_INSERT_<handle>' (INSERT 엔티티의 handle)
    source_id: str

    # 부품 표면의 꼭짓점 좌표 [(x, y, z), ...] (mm).
    # 면마다 꼭짓점을 따로 넣기 때문에 같은 좌표가 여러 번 나올 수 있다.
    vertices: list[tuple[float, float, float]]

    # DXF INSERT 속성값 {'SEQ': '1', 'STAGE': '1', 'GRASP': 'SIDE_25'}.
    # 형상 정보가 아니라 계획(Plan) 초안을 채우는 데만 쓴다. STEP은 항상 빈 dict.
    hints: dict[str, str]

from dataclasses import dataclass

import numpy as np

from model.defined.BoundingBox import TOL
from model.defined.CADPartInstance import CADPartInstance
from model.defined.CADPartDefinition import CADPartDefinition

MODEL_SCHEMA = 'cad_model/1.0'


@dataclass
class Model:
    """CAD에서 가져온 조립체 정의: 무엇(CADPartDefinition)을 어디에(CADPartInstance) 놓는가.

    좌표 규칙: mm, 오른손 좌표계, Z 위쪽, 책상면 Z=0. CAD 원점을 그대로 쓴다.
    조립 순서·잡기 방법은 여기에 없다. 그것은 레시피(Recipe)가 정한다.
    """

    model_id: str
    revision: str
    source_filename: str   # 원본 CAD 파일 이름
    source_sha256: str     # 원본 CAD 파일 해시. 계획이 같은 CAD로 작성됐는지 확인하는 데 쓴다.
    parts: list[CADPartDefinition]
    instances: list[CADPartInstance]

    @classmethod
    def create_from_raw_boxes(cls, raw_boxes, model_id, revision, source_filename, source_sha256):
        """CAD에서 읽은 RawBox 목록으로 모델을 만든다.

        1. 꼭짓점 → 직육면체 치수·중심·회전 계산
        2. 같은 치수끼리 CADPartDefinition 하나로 묶기
        3. 책상 아래로 내려간 부품, 서로 겹치는 부품 검사

        입력:
            raw_boxes: CadReader.read_cad가 돌려준 RawBox 목록
            model_id, revision: 모델 이름과 버전
            source_filename, source_sha256: 원본 CAD 파일 이름과 해시
        출력: Model
        실패: 상자가 아니거나, 축에 평행하지 않거나, 책상 아래이거나, 서로 겹치면 ValueError.
        """
        # 부품마다 직육면체 인식 → 같은 치수는 같은 CADPartDefinition
        parts_by_size = {}   # 반올림한 치수 → CADPartDefinition
        instances = []
        for raw in raw_boxes:
            # 1. 직육면체 인식: 면마다 중복된 꼭짓점을 하나로 합친다. 직육면체라면 정확히 8개가 남는다.
            points = np.asarray(raw.vertices, dtype=float)
            if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
                raise ValueError(f'{raw.source_id}: invalid CAD vertices')
            points = np.unique(np.round(points, 7), axis=0)
            lo, hi = points.min(axis=0), points.max(axis=0)
            dims = hi - lo
            if len(points) != 8 or (dims <= TOL).any():
                raise ValueError(f'{raw.source_id}: expected a box with eight distinct corners')
            # 축에 평행한 직육면체라면 모든 꼭짓점 좌표가 각 축의 최솟값 또는 최댓값이다.
            if not (np.isclose(points, lo, atol=TOL) | np.isclose(points, hi, atol=TOL)).all():
                raise ValueError(f'{raw.source_id}: only axis-aligned boxes are supported')

            # 부품 로컬 축은 치수 큰 순서로 정한다: L(가장 긴 축), W, T(가장 짧은 축).
            # order[k] = 부품의 k번째 축이 모델의 몇 번째 축(x=0, y=1, z=2)인가
            order = np.argsort(-dims, kind='stable')
            R = np.eye(3)[:, order]
            # 축 순서를 바꾸면 왼손 좌표계(det = -1)가 될 수 있다. W축 부호를 뒤집어 오른손 좌표계로 맞춘다.
            if np.linalg.det(R) < 0:
                R[:, 1] *= -1
            size = tuple(float(d) for d in dims[order])
            center = tuple(float(c) for c in (lo + hi) / 2)

            # 2. 같은 치수 → 같은 CADPartDefinition
            key = tuple(round(v, 6) for v in size)
            if key not in parts_by_size:
                parts_by_size[key] = CADPartDefinition(part_id=f'PART_{len(parts_by_size) + 1:03d}', size_mm=size)

            instances.append(CADPartInstance(
                instance_id=raw.source_id,
                part=parts_by_size[key],
                center_mm=center,
                R=tuple(tuple(float(v) for v in row) for row in R),
            ))


        # 3. 배치 검사 (순서와 무관한 최종 형상 검사): 책상 아래, 서로 겹침
        for n, instance in enumerate(instances):
            box = instance.calculate_bounding_box()
            if box.lo[2] < -TOL:
                raise ValueError(f'Part below table: {instance.instance_id}')
            for other in instances[:n]:
                if box.overlaps(other.calculate_bounding_box()):
                    raise ValueError(f'Parts overlap: {other.instance_id} / {instance.instance_id}')


        # 모델 생성
        return cls(model_id=model_id, revision=revision,
                   source_filename=source_filename, source_sha256=source_sha256,
                   parts=list(parts_by_size.values()), instances=instances)

    def convert_to_dict(self):
        """JSON으로 저장할 형태로 바꾼다.

        출력: dict
        """
        return {
            'schema': MODEL_SCHEMA,
            'model_id': self.model_id,
            'revision': self.revision,
            'frame': {'units': 'mm', 'handedness': 'right', 'z': 'up', 'table_z_mm': 0},
            'source_cad': {'filename': self.source_filename, 'sha256': self.source_sha256},
            'parts': [{'part_id': p.part_id, 'size_mm': list(p.size_mm)} for p in self.parts],
            'instances': [{'instance_id': i.instance_id,
                           'part_id': i.part.part_id,
                           'center_mm': list(i.center_mm),
                           'R': [list(row) for row in i.R]} for i in self.instances],
        }

from pathlib import Path

import numpy as np


class CadReader:
    """STEP · DXF 파일에서 블록마다 꼭짓점을 읽는다.

    CAD는 '어떤 블록이 어디에 놓이는지'만 담는다. 조립 순서는 읽지 않는다
    (DXF 속성에 순서 · 잡기가 적혀 있으면 hints로 넘겨 계획 양식을 미리 채우는 데만 쓴다).
    출력 블록 = {'id', 'vertices', 'hints'} — RecipeBuilder.make_model의 입력이다.
    """

    def read_cad(self, path):
        """CAD 파일을 읽어 부품마다 블록 하나를 만든다. 확장자로 형식을 고른다.

        입력:
            path: CAD 파일 경로 (.step · .stp · .dxf)
        출력: [{'id': str, 'vertices': [(x, y, z), ...] (mm), 'hints': {태그: 값}}]
        실패: 지원하지 않는 확장자면 ValueError.
        """
        suffix = Path(path).suffix.lower()
        if suffix in ('.step', '.stp'):
            return self.read_step(path)
        if suffix == '.dxf':
            return self.read_dxf(path)
        raise ValueError('Supported CAD inputs: .step, .stp, .dxf')

    def read_step(self, path):
        """STEP 파일을 읽는다. 파일 안의 독립 솔리드 하나가 부품 하나다(id = 'STEP_SOLID_<순번 4자리>').

        입력:
            path: STEP 파일 경로
        출력: 블록 목록 (hints는 늘 빈 dict)
        실패: 솔리드가 없거나 평면 6면 상자가 아니면 ValueError, cadquery가 없으면 ImportError.
        """
        # cadquery는 무거워 STEP을 읽을 때만 import한다. 길이 단위는 가져올 때 mm로 바뀐다(OCCT 동작).
        import cadquery as cq
        solids = cq.importers.importStep(str(path)).solids().vals()
        if not solids:
            raise ValueError('STEP contains no solids')


        # 솔리드마다 면 검사 후 꼭짓점 수집. 직육면체는 평면 6개다.
        boxes = []
        for n, solid in enumerate(solids, 1):
            box_id = f'STEP_SOLID_{n:04d}'
            faces = solid.Faces()
            if len(faces) != 6 or any(face.geomType() != 'PLANE' for face in faces):
                raise ValueError(f'{box_id}: only six-faced box solids are supported')
            boxes.append({'id': box_id, 'vertices': [vertex.toTuple() for vertex in solid.Vertices()], 'hints': {}})
        return boxes

    def read_dxf(self, path):
        """DXF 파일을 읽는다. 모델 공간의 INSERT 하나가 부품 하나다(id = 'DXF_INSERT_<handle>').

        INSERT가 가리키는 블록 정의 안에 3DFACE 6개(직육면체 6면)가 있어야 한다.

        입력:
            path: DXF 파일 경로
        출력: 블록 목록 (hints = INSERT 속성, 이 프로젝트 DXF는 SEQ · STAGE · GRASP)
        실패: DXF가 깨졌거나, 단위가 mm가 아니거나, INSERT · 3DFACE 조건이 맞지 않으면 ValueError. ezdxf가 없으면 ImportError.
        """
        # 단위 확인. INSUNITS = 4 가 mm — 없거나 다르면 좌표를 믿을 수 없어 거부한다.
        import ezdxf
        try:
            doc = ezdxf.readfile(path)
        except ezdxf.DXFError as exc:
            raise ValueError(f'Invalid DXF: {exc}') from exc
        if doc.units != 4:
            raise ValueError('DXF must declare millimetres (INSUNITS=4)')


        # INSERT마다 형상 검사 후 꼭짓점과 속성 수집
        boxes = []
        for entity in doc.modelspace():
            if entity.dxftype() != 'INSERT':
                raise ValueError(f'Each part must be an INSERT; found {entity.dxftype()}')
            box_id = 'DXF_INSERT_' + entity.dxf.handle

            # 스케일이 1이 아니거나 배열 INSERT(엔티티 하나로 여러 개)는 받지 않는다.
            scale = [entity.dxf.xscale, entity.dxf.yscale, entity.dxf.zscale]
            if not np.allclose(scale, 1) or entity.dxf.row_count != 1 or entity.dxf.column_count != 1:
                raise ValueError(f'{box_id}: scaled or array INSERT is not supported')

            # virtual_entities()는 INSERT의 위치 · 회전을 적용한 좌표를 돌려준다 — 꼭짓점이 이미 모델 좌표다.
            faces = list(entity.virtual_entities())
            if len(faces) != 6 or any(face.dxftype() != '3DFACE' for face in faces):
                raise ValueError(f'{box_id}: block must contain exactly six 3DFACE entities')
            boxes.append({'id': box_id,
                          'vertices': [tuple(face.dxf.get(f'vtx{k}')) for face in faces for k in range(4)],
                          'hints': {attrib.dxf.tag: attrib.dxf.text for attrib in entity.attribs}})


        if not boxes:
            raise ValueError('DXF modelspace contains no parts')
        return boxes

from pathlib import Path

import numpy as np

from cad.defined.RawBox import RawBox


class CadReader:
    """STEP·DXF 파일에서 부품별 꼭짓점을 읽는다.

    CAD는 '정의'만 담는다. 어떤 부품이 어디에 놓이는지만 읽고, 조립 순서는 읽지 않는다.
    (DXF 속성에 순서가 적혀 있으면 hints로 넘겨 계획 초안에만 쓴다.)
    """

    @staticmethod
    def read_cad(path):
        """CAD 파일을 읽어 부품마다 RawBox 하나를 만든다. 확장자로 형식을 고른다.

        입력:
            path: 읽을 CAD 파일 경로 (.step, .stp, .dxf)
        출력: RawBox 목록
        실패: 지원하지 않는 확장자면 ValueError.
        """
        # 확장자에 맞는 읽기 방식 선택
        suffix = Path(path).suffix.lower()
        if suffix in ('.step', '.stp'):
            return CadReader.read_step(path)
        if suffix == '.dxf':
            return CadReader.read_dxf(path)
        raise ValueError('Supported CAD inputs: .step, .stp, .dxf')

    @staticmethod
    def read_step(path):
        """STEP 파일을 읽는다. 파일 안의 독립 솔리드 하나가 부품 하나다.

        입력:
            path: STEP 파일 경로
        출력: RawBox 목록 (hints는 항상 빈 dict)
        실패: 솔리드가 없거나 평면 6면 상자가 아니면 ValueError, cadquery가 없으면 ImportError.
        """
        # 솔리드 읽기. cadquery는 무거운 라이브러리라 STEP을 읽을 때만 import한다.
        # STEP의 길이 단위는 가져올 때 mm로 변환된다(OCCT 동작).
        import cadquery as cq
        solids = cq.importers.importStep(str(path)).solids().vals()
        if not solids:
            raise ValueError('STEP contains no solids')


        # 솔리드마다 면 검사 후 꼭짓점 수집. 직육면체는 평면 6개로 이루어진다.
        boxes = []
        for n, solid in enumerate(solids, 1):
            source_id = f'STEP_SOLID_{n:04d}'
            faces = solid.Faces()
            if len(faces) != 6 or any(face.geomType() != 'PLANE' for face in faces):
                raise ValueError(f'{source_id}: only six-faced box solids are supported')
            vertices = [vertex.toTuple() for vertex in solid.Vertices()]
            boxes.append(RawBox(source_id, vertices, hints={}))
        return boxes

    @staticmethod
    def read_dxf(path):
        """DXF 파일을 읽는다. 모델 공간의 INSERT 하나가 부품 하나다.

        INSERT가 가리키는 블록 정의 안에 3DFACE 6개(직육면체의 6면)가 들어 있어야 한다.

        입력:
            path: DXF 파일 경로
        출력: RawBox 목록 (hints = INSERT 속성 SEQ/STAGE/GRASP)
        실패: DXF가 깨졌거나, 단위가 mm가 아니거나, INSERT·3DFACE 조건이 맞지 않으면 ValueError. ezdxf가 없으면 ImportError.
        """
        # 파일 열기와 단위 확인. INSUNITS=4 는 mm. 단위가 없거나 다르면 좌표를 믿을 수 없으므로 거부한다.
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
            source_id = 'DXF_INSERT_' + entity.dxf.handle

            # 스케일이 1이 아니거나 배열 INSERT(한 엔티티로 여러 개 배치)는 지원하지 않는다.
            scale = [entity.dxf.xscale, entity.dxf.yscale, entity.dxf.zscale]
            if not np.allclose(scale, 1) or entity.dxf.row_count != 1 or entity.dxf.column_count != 1:
                raise ValueError(f'{source_id}: scaled or array INSERT is not supported')

            # virtual_entities()는 블록 안의 엔티티를 INSERT의 위치·회전을 적용한 좌표로 돌려준다.
            # 즉 여기서 얻는 3DFACE 꼭짓점은 이미 모델 좌표다.
            faces = list(entity.virtual_entities())
            if len(faces) != 6 or any(face.dxftype() != '3DFACE' for face in faces):
                raise ValueError(f'{source_id}: block must contain exactly six 3DFACE entities')
            vertices = [tuple(face.dxf.get(f'vtx{k}')) for face in faces for k in range(4)]

            # INSERT에 붙은 속성(ATTRIB). 이 프로젝트의 DXF에는 SEQ/STAGE/GRASP가 들어 있다.
            hints = {attrib.dxf.tag: attrib.dxf.text for attrib in entity.attribs}
            boxes.append(RawBox(source_id, vertices, hints))


        # 부품이 하나도 없으면 거부
        if not boxes:
            raise ValueError('DXF modelspace contains no parts')
        return boxes

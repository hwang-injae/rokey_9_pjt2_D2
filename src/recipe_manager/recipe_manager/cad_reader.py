import numpy as np


class CadReader:
    """DXF 파일에서 블록마다 이름 · CAD 핸들 · 꼭짓점 · 속성을 읽는다.

    레시피의 원본은 DXF 하나다(E-52): INSERT 블록 이름 = 사람이 정한 블록 이름(`LEG_001_01`),
    INSERT 속성 SEQ · STAGE · GRASP = 놓는 순서 · 단계 · 잡기. 핸들은 CAD 대응 검증용으로만 넘긴다.
    STEP(`cads/*.step`)은 이름 · 속성이 없어 레시피 원본으로 쓰지 않는다(검사기 치수 확인용으로만 둔다).
    출력 블록 = {'block', 'handle', 'vertices', 'hints'} — RecipeBuilder.make_structure 의 입력이다.
    """

    def read_dxf(self, path):
        """DXF 파일을 읽는다. 모델 공간의 INSERT 하나가 블록 하나다.

        INSERT가 가리키는 블록 정의 안에 3DFACE 6개(직육면체 6면)가 있어야 한다.

        입력:
            path: DXF 파일 경로
        출력: [{'block': INSERT 블록 이름, 'handle': DXF 핸들, 'vertices': [(x, y, z), ...] (mm), 'hints': {SEQ · STAGE · GRASP}}]
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


        # INSERT마다 형상 검사 후 이름 · 핸들 · 꼭짓점 · 속성 수집
        boxes = []
        for entity in doc.modelspace():
            if entity.dxftype() != 'INSERT':
                raise ValueError(f'Each part must be an INSERT; found {entity.dxftype()}')
            name = entity.dxf.name

            # 스케일이 1이 아니거나 배열 INSERT(엔티티 하나로 여러 개)는 받지 않는다.
            scale = [entity.dxf.xscale, entity.dxf.yscale, entity.dxf.zscale]
            if not np.allclose(scale, 1) or entity.dxf.row_count != 1 or entity.dxf.column_count != 1:
                raise ValueError(f'{name}: scaled or array INSERT is not supported')

            # virtual_entities()는 INSERT의 위치 · 회전을 적용한 좌표를 돌려준다 — 꼭짓점이 이미 모델 좌표다.
            faces = list(entity.virtual_entities())
            if len(faces) != 6 or any(face.dxftype() != '3DFACE' for face in faces):
                raise ValueError(f'{name}: block must contain exactly six 3DFACE entities')
            boxes.append({'block': name, 'handle': entity.dxf.handle,
                          'vertices': [tuple(face.dxf.get(f'vtx{k}')) for face in faces for k in range(4)],
                          'hints': {attrib.dxf.tag: attrib.dxf.text for attrib in entity.attribs}})


        if not boxes:
            raise ValueError('DXF modelspace contains no parts')
        return boxes

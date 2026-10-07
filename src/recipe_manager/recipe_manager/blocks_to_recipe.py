import hashlib
import itertools

from recipe_manager.recipe_builder import RecipeBuilder


class BlocksToRecipe:
    """변환기 ① — 블록 JSON(blocks/1)을 레시피 두 파일(cad_structure/1.0 + cad_recipe/1.0)로 바꾼다 (W110 · W139, E-52).

    생성 · 스캔 설계처럼 CAD가 없는 설계에 쓴다. 검사 묶음(task DesignChecker)이 검사를 통과시킨 뒤 이 변환기를 부른다.
    - 블록 이름: 역할을 모르므로 `BLOCK_001_<블록 2자리>` 하나의 부품으로, 번호는 작명 규칙대로 아래층부터 · 앞 → 뒤 · 왼 → 오른.
    - 순서 = order, 단계 = 바닥 높이가 바뀔 때마다 1씩 오른다(같은 높이를 이어 놓으면 같은 단계).
    - 잡기 = 검사 묶음의 grasp_options(놓는 순간 두 손가락이 들어가는 후보) 중 **짧은 쪽 우선**, 짧은 쪽이 막히면 긴 쪽
      (10/7 한세교 — 긴 쪽 잡기에서 놓기 오차가 더 컸다. CAD 4종의 GRASP 속성도 같은 규칙).
    계산만 한다(파일 · 로봇 · 메시지 영향 없음). 받침 · 잡기 상태 · 겹침 검사는 RecipeBuilder 가 CAD 레시피와 같은 규칙으로 한다.
    """

    def __init__(self, grasp_options, block_mm):
        """잡기 후보를 주는 함수와 블록 크기를 받는다.

        입력:
            grasp_options: blocks/1 의 blocks 목록 → {order: [잡기 이름 …]} (task DesignChecker.grasp_options — 같은 손가락 규칙을 쓰려고 받는다)
            block_mm: 블록 크기 [길이, 폭, 두께] mm — robot.yaml block_size_m × 1000
        """
        self.grasp_options = grasp_options
        L, W, T = block_mm
        # 방향 코드 → (x · y · z 방향 길이) mm. IRD 2장 ori 표 · task ori_extents 와 같은 규칙.
        self.extent = {'x': (L, W, T), 'y': (W, L, T), 'xe': (L, T, W), 'ye': (T, L, W), 'zx': (T, W, L), 'zy': (W, T, L)}
        self.builder = RecipeBuilder()

    def convert(self, request):
        """blocks/1 → {'structure': 구조 dict, 'recipe': 조립 dict}. task DesignChecker 의 blocks_to_recipe 자리에 붙인다.

        입력:
            request: blocks/1 dict — design_id, blocks[{order, x, y, z(바닥 높이), ori}] (mm, 설계 좌표계)
        출력: {'structure': cad_structure/1.0, 'recipe': cad_recipe/1.0} — model_id = design_id, source_cad = None(CAD 없음)
        실패: design_id · order · ori 가 틀렸거나, 잡을 후보가 없거나, 겹침 · 뜬 블록이면 ValueError (어느 블록인지 적는다).
        """
        # 형식: design_id · order 1..N · ori 코드
        model_id = request.get('design_id') if isinstance(request, dict) else None
        items = sorted(request.get('blocks') or [], key=lambda b: b.get('order', 0)) if model_id else []
        if not model_id or not items:
            raise ValueError('blocks/1 needs design_id and blocks')
        if [b.get('order') for b in items] != list(range(1, len(items) + 1)):
            raise ValueError('order must run 1..N without gaps or duplicates')
        if len(items) > 99:
            raise ValueError('more than 99 blocks do not fit BLOCK_001_<2 digits>')
        for b in items:
            if b.get('ori') not in self.extent:
                raise ValueError(f'{b["order"]}: unknown ori {b.get("ori")!r}')


        # 블록 이름: 위치(바닥 높이 → y → x) 순서로 BLOCK_001_01 … — 작명 규칙의 블록 번호 순서와 같다
        by_position = sorted(items, key=lambda b: (round(b['z'], 3), round(b['y'], 3), round(b['x'], 3)))
        names = {b['order']: f'BLOCK_001_{n:02d}' for n, b in enumerate(by_position, 1)}


        # 블록마다 꼭짓점 8개 = 중심(x, y, 바닥 + 높이/2) ± 방향별 반길이
        boxes = []
        for b in items:
            dx, dy, dz = self.extent[b['ori']]
            center = (b['x'], b['y'], b['z'] + dz / 2)
            corners = [(center[0] + sx * dx / 2, center[1] + sy * dy / 2, center[2] + sz * dz / 2)
                       for sx, sy, sz in itertools.product((-1, 1), repeat=3)]
            boxes.append({'block': names[b['order']], 'handle': None, 'vertices': corners})
        structure = self.builder.make_structure(boxes, model_id, None, None)


        # 순서 · 단계 · 잡기(짧은 쪽 우선) → 조립 방법. 단계는 바닥 높이가 바뀔 때마다 1씩 오른다(규칙: 같거나 1씩 증가).
        options = self.grasp_options(items)
        hints, stage, last_z = {}, 0, None
        for b in items:
            if last_z is None or abs(b['z'] - last_z) > 1e-6:
                stage += 1
                last_z = b['z']
            choices = options.get(b['order']) or []
            if not choices:
                raise ValueError(f'{b["order"]}: no grasp — neither side has room for both fingers when it is placed')
            short = [g for g in choices if g.endswith('_SHORT')]
            hints[names[b['order']]] = {'SEQ': str(b['order']), 'STAGE': str(stage), 'GRASP': short[0] if short else choices[0]}
        structure_sha256 = hashlib.sha256(self.builder.make_json_text(structure).encode('utf-8')).hexdigest()
        return {'structure': structure, 'recipe': self.builder.make_recipe(structure, structure_sha256, hints)}

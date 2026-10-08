import hashlib
import json
import re

import numpy as np

# 형식 이름 = <이름>/<앞>.<뒤>. 레시피(구조)와 조립 방법은 짝이라 숫자를 같이 올린다 (10/8 한세교, E-52 의 cad_structure/1.0 · cad_recipe/1.0 다음).
RECIPE_SCHEMA = 'recipe/2.0'
PLACEMENTS_SCHEMA = 'placements/2.0'

# 좌표 비교 허용오차(mm). CAD 좌표의 부동소수 오차를 흡수한다.
TOL = 1e-6

# 블록 이름 = <역할>[_<옵션>]_<부품 3자리>_<블록 2자리> (예 LEG_001_01 · LEG_WHEEL_001_01).
# 역할 · 옵션은 영문 대문자 한 단어씩, 옵션은 0~1개 — 밑줄 든 역할(ARM_REST)이 역할 + 옵션으로 읽히는 것을 막는다(10/8 E-68 ⑥).
BLOCK_NAME = re.compile(r'([A-Z]+(?:_[A-Z]+)?)_([0-9]{3})_([0-9]{2})')

# 부품 로컬 축 이름 → 회전행렬 R의 열 번호. 부품 치수는 L ≥ W ≥ T 순서다.
AXES = {'LENGTH': 0, 'WIDTH': 1, 'THICKNESS': 2}

# 놓인 상태 → 그 상태에서 위를 향하는 부품 축.
#   FLAT(눕힘) 큰 면 L×W가 바닥 → T가 위 / EDGE(옆세움) L×T가 바닥 → W가 위 / STAND(세움) W×T가 바닥 → L이 위
VERTICAL_AXIS = {'FLAT': 2, 'EDGE': 1, 'STAND': 0}

# 파지 방법 6가지 <놓인 상태>_<LONG|SHORT> (IRD 2장, S-25). LONG = 수평 치수 중 긴 쪽을 손가락 사이에 끼운다.
# 그리퍼는 위에서 내려오므로 닫는 방향은 늘 수평이다. 치수는 이름에 넣지 않는다(닫는 폭은 블록 치수로 계산).
GRASPS = ('FLAT_SHORT', 'FLAT_LONG', 'EDGE_SHORT', 'EDGE_LONG', 'STAND_SHORT', 'STAND_LONG')

# 옛 DXF의 GRASP 속성(75×25×15 블록 기준 치수 이름) → 닫힘 축 이름. CAD 속성을 읽을 때만 쓴다.
LEGACY_GRASP = {'END_75': 'LENGTH', 'SIDE_25': 'WIDTH', 'THICKNESS_15': 'THICKNESS'}

# 조립 방법 CSV 칸 — (가) 조립 방법 · (나) 표시 칸(레시피 값을 읽기 좋게 옮김) · 짝 확인 해시 · 형식 이름. 모든 줄의 recipe_sha256 · schema 는 같다.
PLACEMENT_COLUMNS = ('block_id', 'block', 'sequence', 'stage', 'part_id', 'size_L_mm', 'size_W_mm', 'size_T_mm',
                     'center_x_mm', 'center_y_mm', 'center_z_mm', 'axis_L', 'axis_W', 'axis_T',
                     'grasp', 'grasp_axis', 'supports', 'cad_handle', 'recipe_sha256', 'schema')


class RecipeBuilder:
    """블록 목록 → 레시피(recipe/2.0, 구조) → 조립 방법(placements/2.0) · 배치표 줄. ROS · 파일 없이 dict만 다룬다.

    레시피 = 무엇을 어디에(블록 이름 · 부품 규격 · 중심 · 방향 · CAD 대응), 조립 방법 = 어떻게(순서 · 단계 · 잡기 · 받침).
    둘은 블록 이름으로 잇고, 조립 방법의 recipe_sha256 으로 '이 레시피에 대해 쓴 순서'임을 확인한다(구조만 다시 만들고 조립 방법을
    그대로 둔 경우를 읽는 쪽이 거부하게 — 10/8 PL).
    CAD 도구(recipe_manager/main.py)와 변환기 ①(blocks_to_recipe)이 같이 쓴다 — 계산 · 검사 규칙을 한 벌로 두려고.
    좌표 규칙: mm, 오른손 좌표계, Z 위쪽, 책상면 Z = 0. 로봇 · 현장 값(TCP · 조립 원점 · 그리퍼 값)은 다루지 않는다.
    """

    def make_recipe(self, boxes, model_id, source_filename, source_sha256):
        """블록 목록으로 레시피를 만든다: 꼭짓점 → 치수 · 중심 · 회전, 같은 치수는 같은 부품 규격, 이름 · 배치 검사.

        입력:
            boxes: [{'block': 'LEG_001_01', 'handle': '3E', 'vertices': [(x, y, z), ...] (mm)}] — CadReader.read_dxf 출력.
                   CAD가 없는 설계(변환기 ①)는 handle = None
            model_id: 모델 ID (예 001_CHAIR_BENCH)
            source_filename, source_sha256: 원본 CAD 파일 이름과 해시. CAD가 없으면 None
        출력: 레시피 dict (schema recipe/2.0 — source_cad · parts · blocks[block · part_id · center_mm · R · cad.handle]).
              CAD가 없으면 source_cad = None, 블록에 cad 칸 없음
        실패: 이름이 규칙과 다르거나 겹치거나 번호가 위치 순서와 다르면, 직육면체가 아니거나 축에 평행하지 않거나
              책상 아래이거나 서로 겹치면 ValueError.
        """
        # 블록마다 직육면체 인식 → 같은 치수는 같은 부품 규격
        parts = {}   # 반올림한 치수 → part dict
        blocks = []
        for box in boxes:
            name = box['block']
            if not BLOCK_NAME.fullmatch(name or ''):
                raise ValueError(f'{name!r}: block name must be <ROLE>[_<OPTION>]_<part 3 digits>_<block 2 digits> '
                                 f'(one word each, e.g. LEG_001_01 · LEG_WHEEL_001_01)')
            if any(b['block'] == name for b in blocks):
                raise ValueError(f'{name}: block name used twice')

            # 면마다 중복된 꼭짓점을 하나로 합친다. 직육면체라면 정확히 8개가 남는다.
            points = np.asarray(box['vertices'], dtype=float)
            if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
                raise ValueError(f'{name}: invalid CAD vertices')
            points = np.unique(np.round(points, 7), axis=0)
            lo, hi = points.min(axis=0), points.max(axis=0)
            dims = hi - lo
            if len(points) != 8 or (dims <= TOL).any():
                raise ValueError(f'{name}: expected a box with eight distinct corners')
            # 축에 평행한 직육면체라면 모든 꼭짓점 좌표가 각 축의 최솟값 또는 최댓값이다.
            if not (np.isclose(points, lo, atol=TOL) | np.isclose(points, hi, atol=TOL)).all():
                raise ValueError(f'{name}: only axis-aligned boxes are supported')

            # 부품 로컬 축은 치수 큰 순서(L, W, T). order[k] = 부품 k번째 축이 모델의 몇 번째 축(x 0 · y 1 · z 2)인가.
            # 축 순서를 바꾸면 왼손 좌표계(det = −1)가 될 수 있어 W축 부호를 뒤집어 오른손으로 맞춘다.
            order = np.argsort(-dims, kind='stable')
            R = np.eye(3)[:, order]
            if np.linalg.det(R) < 0:
                R[:, 1] *= -1
            size = [float(d) for d in dims[order]]

            # part_id 는 CAD에서 처음 나온 치수 순서. 젠가 블록만 쓰면 PART_001 하나다.
            key = tuple(round(v, 6) for v in size)
            if key not in parts:
                parts[key] = {'part_id': f'PART_{len(parts) + 1:03d}', 'size_mm': size}
            block = {'block': name,
                     'part_id': parts[key]['part_id'],
                     'center_mm': [float(c) for c in (lo + hi) / 2],
                     'R': [[float(v) for v in row] for row in R]}
            if box.get('handle') is not None:
                block['cad'] = {'handle': box['handle']}
            blocks.append(block)
        source_cad = {'filename': source_filename, 'sha256': source_sha256} if source_filename else None
        recipe = {'schema': RECIPE_SCHEMA, 'model_id': model_id,
                  'source_cad': source_cad,
                  'parts': list(parts.values()), 'blocks': blocks}


        # 순서와 무관한 최종 형상 검사: 책상 아래, 서로 겹침
        boxes_lo_hi = [self.calculate_bounding_box(recipe, block) for block in blocks]
        for n, (block, (lo, hi)) in enumerate(zip(blocks, boxes_lo_hi)):
            if lo[2] < -TOL:
                raise ValueError(f'Part below table: {block["block"]}')
            for other, (other_lo, other_hi) in zip(blocks[:n], boxes_lo_hi[:n]):
                # 겹치는 구간의 세 축 길이가 모두 양수면 부피를 함께 차지한다(면끼리 맞닿은 쌓기는 겹침 아님).
                if (np.minimum(hi, other_hi) - np.maximum(lo, other_lo) > TOL).all():
                    raise ValueError(f'Parts overlap: {other["block"]} / {block["block"]}')


        # 정한 번호가 작명 규칙의 위치 순서와 맞는지 확인
        self.check_block_numbering(recipe, boxes_lo_hi)
        return recipe

    def check_block_numbering(self, recipe, boxes_lo_hi):
        """블록 이름의 번호가 위치 순서와 맞는지 본다 — 부품 번호 = 같은 역할(+ 옵션) 부품을 앞(−y)→뒤, 왼(−x)→오른 순,
        블록 번호 = 그 부품 안에서 아래층부터, 같은 층이면 앞→뒤, 왼→오른. 번호는 1부터 빠짐없이.

        입력:
            recipe: 레시피 dict
            boxes_lo_hi: blocks 순서와 같은 (lo, hi) 목록 (mm)
        실패: 번호가 빠졌거나 위치 순서와 다르면 ValueError (어느 부품 · 블록인지 적는다).
        """
        # 부품(역할 + 부품 번호)별로 블록을 모은다. 위치 키는 (바닥 z, 중심 y, 중심 x) — 0.001 mm로 반올림해 같은 층을 묶는다.
        part_blocks = {}
        for block, (lo, hi) in zip(recipe['blocks'], boxes_lo_hi):
            role, part_no, block_no = BLOCK_NAME.fullmatch(block['block']).groups()
            center = (lo + hi) / 2
            key = (round(float(lo[2]), 3), round(float(center[1]), 3), round(float(center[0]), 3))
            part_blocks.setdefault((role, int(part_no)), []).append((int(block_no), key, block['block']))


        # 부품 안 블록 번호 = 1..n, 위치 순서와 같음
        first_block = {}   # (역할, 부품 번호) → 첫 블록 위치 (y, x)
        for part, items in part_blocks.items():
            items.sort()
            if [n for n, _, _ in items] != list(range(1, len(items) + 1)):
                raise ValueError(f'{part[0]}_{part[1]:03d}: block numbers must run 01..{len(items):02d}')
            if [k for _, k, _ in items] != sorted(k for _, k, _ in items):
                raise ValueError(f'{part[0]}_{part[1]:03d}: block numbers must go bottom → top, front → rear, left → right '
                                 f'({", ".join(name for _, _, name in items)})')
            first_block[part] = items[0][1][1:]


        # 역할 안 부품 번호 = 1..m, 첫 블록 위치(앞 → 뒤, 왼 → 오른) 순서와 같음
        roles = {}
        for (role, part_no), yx in first_block.items():
            roles.setdefault(role, []).append((part_no, yx))
        for role, items in roles.items():
            items.sort()
            if [n for n, _ in items] != list(range(1, len(items) + 1)):
                raise ValueError(f'{role}: part numbers must run 001..{len(items):03d}')
            if [yx for _, yx in items] != sorted(yx for _, yx in items):
                raise ValueError(f'{role}: part numbers must go front → rear, left → right')

    def make_placements(self, recipe, hints):
        """레시피와 블록마다의 순서 · 단계 · 잡기(CAD 속성 또는 AI가 쓴 값)로 검증된 조립 방법을 만든다.

        검사: ① 모든 블록에 SEQ · STAGE · GRASP ② sequence 1..N 연속, stage 1부터 0 또는 1씩 증가
        ③ 파지 방법의 상태가 레시피 배치와 같음 → 닫힘 축 계산 ④ 책상 위가 아닌 블록은 먼저 놓인 블록 위(공중에 뜨지 않음).

        입력:
            recipe: make_recipe 출력
            hints: {블록 이름: {'SEQ': '1', 'STAGE': '1', 'GRASP': 'FLAT_SHORT' 또는 옛 이름 SIDE_25 등}} — 정수는 글자 · 정수 둘 다
        출력: 조립 방법 dict (schema placements/2.0 — model_id · recipe_sha256 · steps[block · block_id · sequence · stage ·
              grasp · grasp_axis · supports], steps는 sequence 순서)
        실패: 속성이 빠졌거나 정수가 아니거나 검사 항목을 어기면 블록 이름을 담은 ValueError.
        """
        # ① 블록마다 속성 → 순서 · 단계 · 파지 방법. 빠진 것은 한 번에 모아 알린다.
        blocks = {block['block']: block for block in recipe['blocks']}
        missing = [f'{name}({", ".join(t for t in ("SEQ", "STAGE", "GRASP") if t not in hints.get(name, {}))})'
                   for name in blocks if not all(t in hints.get(name, {}) for t in ('SEQ', 'STAGE', 'GRASP'))]
        if missing:
            raise ValueError('CAD attributes missing: ' + ', '.join(missing))
        plan = []
        for name, block in blocks.items():
            hint = hints[name]
            try:
                sequence, stage = int(hint['SEQ']), int(hint['STAGE'])
            except ValueError:
                raise ValueError(f'{name}: CAD attributes SEQ · STAGE must be integers, got {hint["SEQ"]!r} · {hint["STAGE"]!r}') from None
            # 축 이름 · 옛 이름은 레시피 배치를 보고 파지 방법으로 바꾼다. 예: SIDE_25 → WIDTH → (눕힌 블록) FLAT_SHORT
            grasp = hint['GRASP']
            if grasp not in GRASPS:
                axis_name = LEGACY_GRASP.get(grasp, grasp)
                if axis_name not in AXES:
                    raise ValueError(f'{name}: unknown CAD GRASP attribute {grasp!r}')
                grasp = self.find_grasp_from_closing_axis(recipe, block, AXES[axis_name])
            plan.append((sequence, stage, grasp, block))


        # ② 순서와 단계
        if any(sequence < 1 or stage < 1 for sequence, stage, _, _ in plan):
            raise ValueError('SEQ and STAGE must be positive integers')
        plan.sort(key=lambda item: item[0])
        if [sequence for sequence, _, _, _ in plan] != list(range(1, len(plan) + 1)):
            raise ValueError('SEQ must run 1..N without gaps or duplicates')
        stages = [stage for _, stage, _, _ in plan]
        if stages[0] != 1 or any(after - before not in (0, 1) for before, after in zip(stages, stages[1:])):
            raise ValueError('STAGE must start at 1 and never decrease or skip along the sequence')


        # ③ · ④ 순서대로 하나씩 놓아 보며 파지 방법과 받침 확인
        placed = []   # (블록 이름, lo, hi) — 지금까지 놓인 블록
        steps = []
        for sequence, stage, grasp, block in plan:
            closing_axis = self.find_closing_axis(recipe, block, grasp)

            # 책상 위가 아니면, 이미 놓인 블록 중 바닥면이 그 윗면에 닿고 위에서 볼 때 면적이 겹치는 것이 받침이다.
            lo, hi = self.calculate_bounding_box(recipe, block)
            if lo[2] <= TOL:
                supports = []
            else:
                supports = [name for name, other_lo, other_hi in placed
                            if abs(lo[2] - other_hi[2]) < TOL
                            and (np.minimum(hi[:2], other_hi[:2]) - np.maximum(lo[:2], other_lo[:2]) > TOL).all()]
                if not supports:
                    raise ValueError(f'{block["block"]}: nothing placed beneath it yet; it would float at this point in the sequence')
            steps.append({'block': block['block'], 'block_id': f'{recipe["model_id"]}_{block["block"]}',
                          'sequence': sequence, 'stage': stage, 'grasp': grasp,
                          'grasp_axis': next(name for name, k in AXES.items() if k == closing_axis),
                          'supports': supports})
            placed.append((block['block'], lo, hi))
        return {'schema': PLACEMENTS_SCHEMA, 'model_id': recipe['model_id'],
                'recipe_sha256': self.calculate_recipe_sha256(recipe), 'steps': steps}

    def calculate_recipe_sha256(self, recipe):
        """짝 확인 해시 — 레시피 객체를 키 정렬 · 공백 없는 JSON(한글 그대로)의 UTF-8 바이트로 만든 sha256 (10/8 한세교, 바꾸지 않음).

        파일 바이트가 아니라 객체로 계산해 DB 저장 · MQTT 전달로 다시 써도(들여쓰기 · 키 순서가 바뀌어도) 값이 같다.

        입력:
            recipe: 레시피 dict
        출력: 16진 글자 64자
        """
        text = json.dumps(recipe, sort_keys=True, separators=(',', ':'), ensure_ascii=False)
        return hashlib.sha256(text.encode('utf-8')).hexdigest()

    def make_placement_rows(self, recipe, placements):
        """조립 방법 CSV 줄. sequence 순서로 조립 단계와 레시피의 목표 위치 · 방향을 한 줄씩 합친다(칸 = PLACEMENT_COLUMNS).

        입력:
            recipe: make_recipe 출력
            placements: make_placements 출력
        출력: 줄 목록. 한 줄 = PLACEMENT_COLUMNS 키의 dict (supports 는 목록 그대로 — 글자로 잇는 것은 파일 쓰는 쪽)
        """
        blocks = {block['block']: block for block in recipe['blocks']}
        sizes = {part['part_id']: part['size_mm'] for part in recipe['parts']}
        rows = []
        for step in placements['steps']:
            block = blocks[step['block']]
            size = sizes[block['part_id']]

            # 방향: 부품 L/W/T 축이 모델의 어느 축을 향하는지 '+X' · '-Y' 글자로. R의 k번째 열 = 부품 k번째 축 방향.
            directions = []
            for k in range(3):
                column = [block['R'][row][k] for row in range(3)]
                axis = max(range(3), key=lambda a: abs(column[a]))
                directions.append(('+' if column[axis] > 0 else '-') + 'XYZ'[axis])

            rows.append({'block_id': step['block_id'], 'block': step['block'],
                         'sequence': step['sequence'], 'stage': step['stage'], 'part_id': block['part_id'],
                         'size_L_mm': size[0], 'size_W_mm': size[1], 'size_T_mm': size[2],
                         'center_x_mm': block['center_mm'][0], 'center_y_mm': block['center_mm'][1],
                         'center_z_mm': block['center_mm'][2],
                         'axis_L': directions[0], 'axis_W': directions[1], 'axis_T': directions[2],
                         'grasp': step['grasp'], 'grasp_axis': step['grasp_axis'],
                         'supports': list(step['supports']), 'cad_handle': block.get('cad', {}).get('handle', ''),
                         'recipe_sha256': placements['recipe_sha256'], 'schema': placements['schema']})
        return rows

    def make_json_text(self, data):
        """JSON 파일에 쓸 글자. 한글은 그대로, 들여쓰기 2칸, 끝에 줄바꿈.

        입력:
            data: 저장할 dict
        출력: 문자열
        """
        return json.dumps(data, ensure_ascii=False, indent=2) + '\n'

    def calculate_bounding_box(self, recipe, block):
        """블록이 목표 위치에서 차지하는 축 정렬 상자.

        입력:
            recipe: 레시피 dict (부품 치수를 찾는 데 쓴다)
            block: 레시피의 blocks 항목
        출력: (lo, hi) — 최소 · 최대 꼭짓점 numpy 배열 (mm)
        """
        # 회전된 직육면체를 모델 축에 투영한 반폭 = |R| @ (치수 / 2). 축 정렬 회전만 받으므로 정확한 경계다.
        size = next(part['size_mm'] for part in recipe['parts'] if part['part_id'] == block['part_id'])
        half = np.abs(np.array(block['R'])) @ (np.array(size) / 2)
        center = np.array(block['center_mm'])
        return center - half, center + half

    def find_closing_axis(self, recipe, block, grasp):
        """파지 방법으로 블록을 잡을 때 그리퍼가 닫히는 부품 축을 구한다.

        입력:
            recipe: 레시피 dict
            block: 레시피의 blocks 항목
            grasp: 파지 방법 이름 (GRASPS 중 하나)
        출력: 부품 축 번호 (AXES 값 — 0 L · 1 W · 2 T)
        실패: 파지 방법의 상태가 레시피 배치와 다르면 ValueError (예: 세워 놓을 블록을 FLAT_*로 잡을 수 없다).
        """
        # 상태가 레시피 배치와 같은지 확인
        placed = self.read_placed_state(block)
        state, length = grasp.split('_')
        if state != placed:
            raise ValueError(f'{block["block"]}: grasp {grasp} needs the part {state}, but CAD places it {placed}')


        # 수평 축 중 긴 쪽 또는 짧은 쪽
        long_axis, short_axis = self.find_horizontal_axes(recipe, block)
        return long_axis if length == 'LONG' else short_axis

    def find_grasp_from_closing_axis(self, recipe, block, axis):
        """닫힘 축 → 파지 방법 이름. 옛 형식(축 이름 · SIDE_25 등)을 바꿀 때 쓴다.

        입력:
            recipe: 레시피 dict
            block: 레시피의 blocks 항목
            axis: 그리퍼가 닫히는 부품 축 번호 (수평이어야 한다)
        출력: 파지 방법 이름 (예: 'FLAT_SHORT')
        실패: 닫힘 축이 수직이면 ValueError.
        """
        long_axis, short_axis = self.find_horizontal_axes(recipe, block)
        if axis not in (long_axis, short_axis):
            name = next(n for n, k in AXES.items() if k == axis)
            raise ValueError(f'{block["block"]}: closing axis {name} is vertical; top-down grasp needs a horizontal axis')
        return f'{self.read_placed_state(block)}_{"LONG" if axis == long_axis else "SHORT"}'

    def find_horizontal_axes(self, recipe, block):
        """이 배치에서 수평인 부품 축 둘을 긴 쪽부터 돌려준다.

        치수가 같으면(정사각 단면) 길게 · 짧게가 같은 잡기라서 축 번호가 작은 쪽을 '긴 쪽'으로 고정한다.

        입력:
            recipe: 레시피 dict
            block: 레시피의 blocks 항목
        출력: [긴 쪽 축 번호, 짧은 쪽 축 번호]
        """
        up = VERTICAL_AXIS[self.read_placed_state(block)]
        size = next(part['size_mm'] for part in recipe['parts'] if part['part_id'] == block['part_id'])
        return sorted((k for k in AXES.values() if k != up), key=lambda k: (-size[k], k))

    def read_placed_state(self, block):
        """레시피 배치에서 블록이 놓인 상태를 읽는다.

        R의 k번째 열이 부품 k번째 축 방향이고 3번째 행이 높이(Z) 성분이다. Z 성분이 ±1인 축이 위를 향한다.

        입력:
            block: 레시피의 blocks 항목
        출력: 'FLAT' / 'EDGE' / 'STAND'
        실패: 위를 향하는 부품 축이 없으면(축 정렬이 아님) ValueError.
        """
        for state, axis in VERTICAL_AXIS.items():
            if abs(abs(block['R'][2][axis]) - 1.0) <= TOL:
                return state
        raise ValueError(f'{block["block"]}: no part axis points up; only axis-aligned placement is supported')

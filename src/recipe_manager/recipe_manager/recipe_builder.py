import numpy as np

MODEL_SCHEMA = 'cad_model/1.0'
PLAN_SCHEMA = 'cad_plan/1.0'
RECIPE_SCHEMA = 'assembly.recipe/1.0'   # cad_recipe/1.0 으로 바꿀 예정(E-44 ②) — 변환기 ②가 두 이름을 다 받게 된 뒤

# 좌표 비교 허용오차(mm). CAD 좌표의 부동소수 오차를 흡수한다.
TOL = 1e-6

# 부품 로컬 축 이름 → 회전행렬 R의 열 번호. 부품 치수는 L ≥ W ≥ T 순서다.
AXES = {'LENGTH': 0, 'WIDTH': 1, 'THICKNESS': 2}

# 놓인 상태 → 그 상태에서 위를 향하는 부품 축.
#   FLAT(눕힘) 큰 면 L×W가 바닥 → T가 위 / EDGE(옆세움) L×T가 바닥 → W가 위 / STAND(세움) W×T가 바닥 → L이 위
VERTICAL_AXIS = {'FLAT': 2, 'EDGE': 1, 'STAND': 0}

# 파지 방법 6가지 <놓인 상태>_<LONG|SHORT> (IRD 2장, S-25). LONG = 수평 치수 중 긴 쪽을 손가락 사이에 끼운다.
# 그리퍼는 위에서 내려오므로 닫는 방향은 늘 수평이다. 치수는 이름에 넣지 않는다(닫는 폭은 블록 치수로 계산).
GRASPS = ('FLAT_SHORT', 'FLAT_LONG', 'EDGE_SHORT', 'EDGE_LONG', 'STAND_SHORT', 'STAND_LONG')

# 옛 DXF의 GRASP 속성(75×25×15 블록 기준 치수 이름) → 닫힘 축 이름. CAD 힌트를 읽을 때만 쓴다.
LEGACY_GRASP = {'END_75': 'LENGTH', 'SIDE_25': 'WIDTH', 'THICKNESS_15': 'THICKNESS'}


class RecipeBuilder:
    """블록 목록 → 모델 → 계획 양식 → 검증된 레시피 · 배치표. ROS · 파일 없이 dict만 다룬다.

    모델 · 계획 · 레시피는 처음부터 저장할 JSON 모양 그대로의 dict다(변환 단계 없음).
    입력 블록은 CadReader가 CAD에서 읽거나, 다른 설계(블록 JSON 등)에서 만든다.
    좌표 규칙: mm, 오른손 좌표계, Z 위쪽, 책상면 Z = 0. 로봇 · 현장 값(TCP · 조립 원점 · 그리퍼 값)은 다루지 않는다.
    """

    def make_model(self, boxes, model_id, revision, source_filename, source_sha256):
        """블록 목록으로 모델을 만든다: 꼭짓점 → 치수 · 중심 · 회전, 같은 치수는 같은 부품(part), 배치 검사.

        입력:
            boxes: [{'id': 'DXF_INSERT_3E', 'vertices': [(x, y, z), ...] (mm), 'hints': {...}}]
            model_id, revision: 모델 이름과 버전
            source_filename, source_sha256: 원본 CAD 파일 이름과 해시
        출력: 모델 dict (schema cad_model/1.0 — parts · instances)
        실패: 직육면체가 아니거나, 축에 평행하지 않거나, 책상 아래이거나, 서로 겹치면 ValueError.
        """
        # 블록마다 직육면체 인식 → 같은 치수는 같은 부품
        parts = {}   # 반올림한 치수 → part dict
        instances = []
        for box in boxes:
            # 면마다 중복된 꼭짓점을 하나로 합친다. 직육면체라면 정확히 8개가 남는다.
            points = np.asarray(box['vertices'], dtype=float)
            if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
                raise ValueError(f'{box["id"]}: invalid CAD vertices')
            points = np.unique(np.round(points, 7), axis=0)
            lo, hi = points.min(axis=0), points.max(axis=0)
            dims = hi - lo
            if len(points) != 8 or (dims <= TOL).any():
                raise ValueError(f'{box["id"]}: expected a box with eight distinct corners')
            # 축에 평행한 직육면체라면 모든 꼭짓점 좌표가 각 축의 최솟값 또는 최댓값이다.
            if not (np.isclose(points, lo, atol=TOL) | np.isclose(points, hi, atol=TOL)).all():
                raise ValueError(f'{box["id"]}: only axis-aligned boxes are supported')

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
            instances.append({'instance_id': box['id'],
                              'part_id': parts[key]['part_id'],
                              'center_mm': [float(c) for c in (lo + hi) / 2],
                              'R': [[float(v) for v in row] for row in R]})
        model = {'schema': MODEL_SCHEMA, 'model_id': model_id, 'revision': revision,
                 'frame': {'units': 'mm', 'handedness': 'right', 'z': 'up', 'table_z_mm': 0},
                 'source_cad': {'filename': source_filename, 'sha256': source_sha256},
                 'parts': list(parts.values()), 'instances': instances}


        # 순서와 무관한 최종 형상 검사: 책상 아래, 서로 겹침
        boxes_lo_hi = [self.calculate_bounding_box(model, instance) for instance in instances]
        for n, (instance, (lo, hi)) in enumerate(zip(instances, boxes_lo_hi)):
            if lo[2] < -TOL:
                raise ValueError(f'Part below table: {instance["instance_id"]}')
            for other, (other_lo, other_hi) in zip(instances[:n], boxes_lo_hi[:n]):
                # 겹치는 구간의 세 축 길이가 모두 양수면 부피를 함께 차지한다(면끼리 맞닿은 쌓기는 겹침 아님).
                if (np.minimum(hi, other_hi) - np.maximum(lo, other_lo) > TOL).all():
                    raise ValueError(f'Parts overlap: {other["instance_id"]} / {instance["instance_id"]}')
        return model

    def make_plan_template(self, model, hints=None):
        """모델의 블록마다 단계 하나인 계획 양식. 힌트(DXF 속성 SEQ · STAGE · GRASP)가 있으면 미리 채운다.

        GRASP 힌트는 파지 방법 이름(FLAT_SHORT 등) · 닫힘 축 이름(WIDTH 등) · 옛 이름(SIDE_25 등) 중 하나다.

        입력:
            model: make_model 출력
            hints: {instance_id: {'SEQ': '1', 'STAGE': '1', 'GRASP': 'SIDE_25'}} 또는 None
        출력: 계획 dict (schema cad_plan/1.0). 채우지 못한 칸은 None — 사람이 채운다.
        실패: SEQ · STAGE가 정수가 아니거나 GRASP 이름을 모르면 ValueError.
        """
        # 블록마다 단계 하나, 힌트로 sequence · stage · grasp 채우기
        hints = hints or {}
        steps = []
        for instance in model['instances']:
            key = instance['instance_id']
            hint = hints.get(key, {})
            step = {'step_id': 'PLACE_' + key, 'instance_id': key, 'sequence': None, 'stage': None, 'grasp': None}

            # DXF 속성값은 문자열이라 정수로 바꾼다.
            for tag, name in (('SEQ', 'sequence'), ('STAGE', 'stage')):
                if tag in hint:
                    try:
                        step[name] = int(hint[tag])
                    except ValueError:
                        raise ValueError(f'{key}: CAD attribute {tag} must be an integer, got {hint[tag]!r}') from None

            # 축 이름 · 옛 이름은 CAD 배치를 보고 파지 방법으로 바꾼다. 예: SIDE_25 → WIDTH → (눕힌 블록) FLAT_SHORT
            if 'GRASP' in hint:
                value = hint['GRASP']
                if value in GRASPS:
                    step['grasp'] = value
                else:
                    axis_name = LEGACY_GRASP.get(value, value)
                    if axis_name not in AXES:
                        raise ValueError(f'{key}: unknown CAD GRASP attribute {value!r}')
                    step['grasp'] = self.find_grasp_from_closing_axis(model, instance, AXES[axis_name])
            steps.append(step)


        # 순서가 다 채워졌으면 순서대로 정렬해 사람이 읽기 쉽게 한다.
        if all(step['sequence'] is not None for step in steps):
            steps.sort(key=lambda step: step['sequence'])
        return {'schema': PLAN_SCHEMA, 'recipe_id': model['model_id'] + '_ASSEMBLY', 'revision': '1',
                'model_id': model['model_id'], 'model_revision': model['revision'],
                'source_cad_sha256': model['source_cad']['sha256'], 'steps': steps}

    def build_recipe(self, model, plan):
        """계획을 모델에 대조해 검증하고 레시피를 만든다.

        검사: ① 이 모델 · 이 CAD로 쓴 계획 ② 모든 블록을 정확히 한 번 ③ sequence 1..N 연속, stage 1부터 0 또는 1씩 증가
        ④ 파지 방법의 상태가 CAD 배치와 같음 → 닫힘 축 계산 ⑤ 책상 위가 아닌 블록은 먼저 놓인 블록 위(공중에 뜨지 않음).

        입력:
            model: make_model 출력
            plan: 사람이 쓴 계획 dict (plan.json 내용)
        출력: 레시피 dict (schema assembly.recipe/1.0 — model + steps, steps는 sequence 순서).
              단계마다 block_id = '<모델ID>_B<sequence 3자리>', 닫힘 축 grasp_axis, 받침 support_*_ids.
        실패: 형식이 틀리거나 검사 항목을 어기면 그 단계 · 블록 이름을 담은 ValueError.
        """
        # 형식: schema · steps 목록 · 파지 방법 이름
        if not isinstance(plan, dict) or plan.get('schema') != PLAN_SCHEMA:
            raise ValueError(f'Expected plan schema {PLAN_SCHEMA}')
        if not isinstance(plan.get('steps'), list) or not all(isinstance(s, dict) for s in plan['steps']):
            raise ValueError('Plan steps must be a list of objects')
        for step in plan['steps']:
            grasp = step.get('grasp')
            if grasp is not None and grasp not in GRASPS:
                raise ValueError(f'{step.get("step_id")}: grasp must be one of {list(GRASPS)}, got {grasp!r}')


        # ① 계획이 가리키는 모델 · CAD
        for name in ('recipe_id', 'revision'):
            value = plan.get(name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f'Plan {name} must be a nonempty string')
        if (plan.get('model_id'), plan.get('model_revision')) != (model['model_id'], model['revision']):
            raise ValueError('Plan must reference this model id and revision')
        if plan.get('source_cad_sha256') != model['source_cad']['sha256']:
            raise ValueError('Plan was written for a different CAD file (sha256 mismatch); run inspect again')


        # ② 모든 블록을 정확히 한 번
        instances = {instance['instance_id']: instance for instance in model['instances']}
        targets = [step.get('instance_id') for step in plan['steps']]
        if len(set(targets)) != len(targets) or set(targets) != set(instances):
            raise ValueError('Plan must place every model instance exactly once')
        step_ids = [step.get('step_id') for step in plan['steps']]
        if len(set(step_ids)) != len(step_ids) or not all(isinstance(k, str) and k for k in step_ids):
            raise ValueError('Plan step_id values must be unique nonempty strings')


        # ③ 순서와 단계. bool은 int의 하위 타입이라 type(...) is int로 비교한다.
        for step in plan['steps']:
            for name in ('sequence', 'stage'):
                value = step.get(name)
                if type(value) is not int or value < 1:
                    raise ValueError(f'{step["step_id"]}: {name} must be a positive integer, got {value!r}')
        ordered = sorted(plan['steps'], key=lambda step: step['sequence'])
        if [step['sequence'] for step in ordered] != list(range(1, len(ordered) + 1)):
            raise ValueError('sequence must run 1..N without gaps or duplicates')
        stages = [step['stage'] for step in ordered]
        if stages[0] != 1 or any(after - before not in (0, 1) for before, after in zip(stages, stages[1:])):
            raise ValueError('stage must start at 1 and never decrease or skip along the sequence')


        # ④ · ⑤ 순서대로 하나씩 놓아 보며 파지 방법과 받침 확인
        placed = []      # (instance_id, lo, hi) — 지금까지 놓인 블록
        block_of = {}    # instance_id → block_id
        steps = []
        for step in ordered:
            instance = instances[step['instance_id']]
            if step.get('grasp') is None:
                raise ValueError(f'{step["step_id"]}: grasp is not set')
            closing_axis = self.find_closing_axis(model, instance, step['grasp'])

            # 책상 위가 아니면, 이미 놓인 블록 중 바닥면이 그 윗면에 닿고 위에서 볼 때 면적이 겹치는 것이 받침이다.
            lo, hi = self.calculate_bounding_box(model, instance)
            if lo[2] <= TOL:
                supports = []
            else:
                supports = [k for k, other_lo, other_hi in placed
                            if abs(lo[2] - other_hi[2]) < TOL
                            and (np.minimum(hi[:2], other_hi[:2]) - np.maximum(lo[:2], other_lo[:2]) > TOL).all()]
                if not supports:
                    raise ValueError(f'{step["instance_id"]}: nothing placed beneath it yet; '
                                     'it would float at this point in the sequence')

            # 블록 이름 = 모델ID + 놓는 순서 — 이름만 보고 몇 번째로 놓는지 안다(W105).
            block_id = f'{model["model_id"]}_B{step["sequence"]:03d}'
            block_of[step['instance_id']] = block_id
            steps.append({'block_id': block_id, 'step_id': step['step_id'], 'instance_id': step['instance_id'],
                          'sequence': step['sequence'], 'stage': step['stage'], 'grasp': step['grasp'],
                          'grasp_axis': next(name for name, k in AXES.items() if k == closing_axis),
                          'support_instance_ids': supports,
                          'support_block_ids': [block_of[k] for k in supports]})
            placed.append((step['instance_id'], lo, hi))
        return {'schema': RECIPE_SCHEMA, 'recipe_id': plan['recipe_id'], 'revision': plan['revision'],
                'model': model, 'steps': steps}

    def make_placement_rows(self, recipe):
        """사람이 확인하는 배치표. sequence 순서로 레시피 단계와 CAD 목표 위치 · 방향을 한 줄씩 합친다.

        입력:
            recipe: build_recipe 출력
        출력: 줄 목록. 한 줄 = {'block_id', 'sequence', 'stage', 'instance_id', 'part_id', 'size_L_mm', 'size_W_mm',
              'size_T_mm', 'center_x_mm', 'center_y_mm', 'center_z_mm', 'axis_L', 'axis_W', 'axis_T',
              'grasp', 'grasp_axis', 'support_instance_ids', 'support_block_ids'}
        """
        model = recipe['model']
        instances = {instance['instance_id']: instance for instance in model['instances']}
        sizes = {part['part_id']: part['size_mm'] for part in model['parts']}
        rows = []
        for step in recipe['steps']:
            instance = instances[step['instance_id']]
            size = sizes[instance['part_id']]

            # 방향: 부품 L/W/T 축이 모델의 어느 축을 향하는지 '+X' · '-Y' 글자로. R의 k번째 열 = 부품 k번째 축 방향.
            directions = []
            for k in range(3):
                column = [instance['R'][row][k] for row in range(3)]
                axis = max(range(3), key=lambda a: abs(column[a]))
                directions.append(('+' if column[axis] > 0 else '-') + 'XYZ'[axis])

            rows.append({'block_id': step['block_id'], 'sequence': step['sequence'], 'stage': step['stage'],
                         'instance_id': step['instance_id'], 'part_id': instance['part_id'],
                         'size_L_mm': size[0], 'size_W_mm': size[1], 'size_T_mm': size[2],
                         'center_x_mm': instance['center_mm'][0], 'center_y_mm': instance['center_mm'][1],
                         'center_z_mm': instance['center_mm'][2],
                         'axis_L': directions[0], 'axis_W': directions[1], 'axis_T': directions[2],
                         'grasp': step['grasp'], 'grasp_axis': step['grasp_axis'],
                         'support_instance_ids': list(step['support_instance_ids']),
                         'support_block_ids': list(step['support_block_ids'])})
        return rows

    def calculate_bounding_box(self, model, instance):
        """블록이 목표 위치에서 차지하는 축 정렬 상자.

        입력:
            model: 모델 dict (부품 치수를 찾는 데 쓴다)
            instance: 모델의 instances 항목
        출력: (lo, hi) — 최소 · 최대 꼭짓점 numpy 배열 (mm)
        """
        # 회전된 직육면체를 모델 축에 투영한 반폭 = |R| @ (치수 / 2). 축 정렬 회전만 받으므로 정확한 경계다.
        size = next(part['size_mm'] for part in model['parts'] if part['part_id'] == instance['part_id'])
        half = np.abs(np.array(instance['R'])) @ (np.array(size) / 2)
        center = np.array(instance['center_mm'])
        return center - half, center + half

    def find_closing_axis(self, model, instance, grasp):
        """파지 방법으로 블록을 잡을 때 그리퍼가 닫히는 부품 축을 구한다.

        입력:
            model: 모델 dict
            instance: 모델의 instances 항목
            grasp: 파지 방법 이름 (GRASPS 중 하나)
        출력: 부품 축 번호 (AXES 값 — 0 L · 1 W · 2 T)
        실패: 파지 방법의 상태가 CAD 배치와 다르면 ValueError (예: 세워 놓을 블록을 FLAT_*로 잡을 수 없다).
        """
        # 상태가 CAD 배치와 같은지 확인
        placed = self.read_placed_state(instance)
        state, length = grasp.split('_')
        if state != placed:
            raise ValueError(f'{instance["instance_id"]}: grasp {grasp} needs the part {state}, '
                             f'but CAD places it {placed}')


        # 수평 축 중 긴 쪽 또는 짧은 쪽
        long_axis, short_axis = self.find_horizontal_axes(model, instance)
        return long_axis if length == 'LONG' else short_axis

    def find_grasp_from_closing_axis(self, model, instance, axis):
        """닫힘 축 → 파지 방법 이름. 옛 형식(축 이름 · SIDE_25 등)을 바꿀 때 쓴다.

        입력:
            model: 모델 dict
            instance: 모델의 instances 항목
            axis: 그리퍼가 닫히는 부품 축 번호 (수평이어야 한다)
        출력: 파지 방법 이름 (예: 'FLAT_SHORT')
        실패: 닫힘 축이 수직이면 ValueError.
        """
        long_axis, short_axis = self.find_horizontal_axes(model, instance)
        if axis not in (long_axis, short_axis):
            name = next(n for n, k in AXES.items() if k == axis)
            raise ValueError(f'{instance["instance_id"]}: closing axis {name} is vertical; '
                             'top-down grasp needs a horizontal axis')
        return f'{self.read_placed_state(instance)}_{"LONG" if axis == long_axis else "SHORT"}'

    def find_horizontal_axes(self, model, instance):
        """이 배치에서 수평인 부품 축 둘을 긴 쪽부터 돌려준다.

        치수가 같으면(정사각 단면) 길게 · 짧게가 같은 잡기라서 축 번호가 작은 쪽을 '긴 쪽'으로 고정한다.

        입력:
            model: 모델 dict
            instance: 모델의 instances 항목
        출력: [긴 쪽 축 번호, 짧은 쪽 축 번호]
        """
        up = VERTICAL_AXIS[self.read_placed_state(instance)]
        size = next(part['size_mm'] for part in model['parts'] if part['part_id'] == instance['part_id'])
        return sorted((k for k in AXES.values() if k != up), key=lambda k: (-size[k], k))

    def read_placed_state(self, instance):
        """CAD 배치에서 블록이 놓인 상태를 읽는다.

        R의 k번째 열이 부품 k번째 축 방향이고 3번째 행이 높이(Z) 성분이다. Z 성분이 ±1인 축이 위를 향한다.

        입력:
            instance: 모델의 instances 항목
        출력: 'FLAT' / 'EDGE' / 'STAND'
        실패: 위를 향하는 부품 축이 없으면(축 정렬이 아님) ValueError.
        """
        for state, axis in VERTICAL_AXIS.items():
            if abs(abs(instance['R'][2][axis]) - 1.0) <= TOL:
                return state
        raise ValueError(f'{instance["instance_id"]}: no part axis points up; only axis-aligned placement is supported')

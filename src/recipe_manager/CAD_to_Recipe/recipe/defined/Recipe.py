from dataclasses import dataclass

from model.defined.BoundingBox import TOL
from model.defined.Model import Model
from recipe.defined.Step import Step

RECIPE_SCHEMA = 'assembly.recipe/1.0'   # cad_recipe/1.0 으로 바꿀 예정 — 변환기 ②(recipe_to_blocks)가 이 이름을 검사해 W121에서 같이 바꾼다


@dataclass
class Recipe:
    """검증된 작업지시서: 모델의 모든 부품을 어떤 순서로, 어떻게 잡아 놓는가.

    로봇과 무관한 데이터다. 로봇 베이스 좌표·TCP·그리퍼 명령값(현장 설정)과 실행 결과(LOT)는 담지 않는다.
    """

    recipe_id: str
    revision: str
    model: Model    # 레시피가 참조하는 정확한 모델 버전을 함께 저장한다.
    steps: list[Step]   # sequence 순서

    @classmethod
    def build_from_plan(cls, model, plan):
        """계획을 모델에 대조해 검증하고 레시피를 만든다.

        검사 항목
        1. 계획이 이 모델·이 CAD 파일로 작성되었다
        2. 모든 부품을 정확히 한 번 배치한다
        3. sequence는 1..N 연속, stage는 1부터 0 또는 1씩 증가
        4. 파지 방법의 상태가 CAD 배치와 같다 → 닫힘 축 계산 (위에서 집으므로 닫힘 축은 늘 수평)
        5. 책상 위가 아닌 부품은 이미 놓인 부품 위에 놓인다 (공중에 뜨지 않는다)

        입력:
            model: Model
            plan: 사람이 작성한 Plan
        출력: Recipe
        실패: 검사 항목 중 하나라도 어기면 그 단계·부품 이름을 담은 ValueError.
        """
        # 1. 계획이 가리키는 모델·CAD 확인
        for name, value in (('recipe_id', plan.recipe_id), ('revision', plan.revision)):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f'Plan {name} must be a nonempty string')
        if (plan.model_id, plan.model_revision) != (model.model_id, model.revision):
            raise ValueError('Plan must reference this model id and revision')
        if plan.source_cad_sha256 != model.source_sha256:
            raise ValueError('Plan was written for a different CAD file (sha256 mismatch); run inspect again')


        # 2. 모든 부품을 정확히 한 번
        instances = {instance.instance_id: instance for instance in model.instances}
        targets = [step.instance_id for step in plan.steps]
        if len(set(targets)) != len(targets) or set(targets) != set(instances):
            raise ValueError('Plan must place every model instance exactly once')
        step_ids = [step.step_id for step in plan.steps]
        if len(set(step_ids)) != len(step_ids) or not all(isinstance(k, str) and k for k in step_ids):
            raise ValueError('Plan step_id values must be unique nonempty strings')


        # 3. 순서와 단계. bool은 int의 하위 타입이라 type(...) is int로 정확히 비교한다.
        for step in plan.steps:
            for name, value in (('sequence', step.sequence), ('stage', step.stage)):
                if type(value) is not int or value < 1:
                    raise ValueError(f'{step.step_id}: {name} must be a positive integer, got {value!r}')
        ordered = sorted(plan.steps, key=lambda step: step.sequence)
        if [step.sequence for step in ordered] != list(range(1, len(ordered) + 1)):
            raise ValueError('sequence must run 1..N without gaps or duplicates')
        stages = [step.stage for step in ordered]
        if stages[0] != 1 or any(after - before not in (0, 1) for before, after in zip(stages, stages[1:])):
            raise ValueError('stage must start at 1 and never decrease or skip along the sequence')


        # 4·5. 순서대로 하나씩 놓아 보며 파지 방법과 지지 확인
        placed = []   # 지금까지 놓인 CADPartInstance
        block_of = {}   # instance_id -> block_id
        steps = []
        for step in ordered:
            instance = instances[step.instance_id]

            # 상태가 CAD와 다르면 여기서 ValueError. 닫힘 축은 그 블록의 수평 치수로 정한다.
            if step.grasp is None:
                raise ValueError(f'{step.step_id}: grasp is not set')
            grasp_axis = step.grasp.find_closing_axis(instance)

            # 책상 위가 아니면 이미 놓인 부품 중 바로 아래에서 받쳐 주는 것을 찾는다.
            box = instance.calculate_bounding_box()
            if box.lo[2] <= TOL:
                supports = []
            else:
                supports = [other.instance_id for other in placed if box.rests_on(other.calculate_bounding_box())]
                if not supports:
                    raise ValueError(f'{step.instance_id}: nothing placed beneath it yet; '
                                     'it would float at this point in the sequence')

            # 블록 이름은 모델ID + 놓는 순서라, 이름만 보고 몇 번째로 놓는 블록인지 안다(W105).
            block_id = f'{model.model_id}_B{step.sequence:03d}'
            block_of[step.instance_id] = block_id
            steps.append(Step(step_id=step.step_id, instance_id=step.instance_id,
                              sequence=step.sequence, stage=step.stage,
                              grasp=step.grasp, grasp_axis=grasp_axis, support_instance_ids=supports,
                              block_id=block_id, support_block_ids=[block_of[k] for k in supports]))
            placed.append(instance)


        # 레시피 생성
        return cls(recipe_id=plan.recipe_id, revision=plan.revision, model=model, steps=steps)

    def make_placement_rows(self):
        """사람이 확인하는 배치표. sequence 순서대로 레시피 단계와 CAD 목표 위치·방향을 한 줄씩 합친다.

        출력: 줄 목록. 한 줄 = {'block_id', 'sequence','stage', 'instance_id', 'part_id', 'size_L_mm', 'size_W_mm', 'size_T_mm',
              'center_x_mm', 'center_y_mm', 'center_z_mm', 'axis_L', 'axis_W', 'axis_T',
              'grasp', 'grasp_axis', 'support_instance_ids', 'support_block_ids'}
        """
        instances = {instance.instance_id: instance for instance in self.model.instances}
        rows = []
        for step in self.steps:   # self.steps는 이미 sequence 순서다
            instance = instances[step.instance_id]

            # 방향: 부품의 L/W/T 축이 모델의 어느 축을 향하는지 '+X', '-Y' 같은 글자로 나타낸다.
            # R의 k번째 열이 부품 k번째 축의 방향이고, 축 정렬 회전이라 성분 하나만 ±1이다.
            directions = []
            for k in range(3):
                column = [instance.R[row][k] for row in range(3)]
                axis = max(range(3), key=lambda a: abs(column[a]))
                directions.append(('+' if column[axis] > 0 else '-') + 'XYZ'[axis])

            rows.append({
                'block_id': step.block_id,
                'sequence': step.sequence,
                'stage': step.stage,
                'instance_id': step.instance_id,
                'part_id': instance.part.part_id,
                'size_L_mm': instance.part.size_mm[0],
                'size_W_mm': instance.part.size_mm[1],
                'size_T_mm': instance.part.size_mm[2],
                'center_x_mm': instance.center_mm[0],
                'center_y_mm': instance.center_mm[1],
                'center_z_mm': instance.center_mm[2],
                'axis_L': directions[0],
                'axis_W': directions[1],
                'axis_T': directions[2],
                'grasp': step.grasp.name,
                'grasp_axis': step.grasp_axis.name,
                'support_instance_ids': list(step.support_instance_ids),
                'support_block_ids': list(step.support_block_ids),
            })
        return rows

    def convert_to_dict(self):
        """JSON으로 저장할 형태로 바꾼다. 참조하는 모델 전체를 함께 넣는다.

        출력: dict
        """
        return {
            'schema': RECIPE_SCHEMA,
            'recipe_id': self.recipe_id,
            'revision': self.revision,
            'model': self.model.convert_to_dict(),
            'steps': [step.convert_to_dict() for step in self.steps],
        }

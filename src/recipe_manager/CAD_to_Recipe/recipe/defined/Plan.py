from dataclasses import dataclass

from recipe.defined.GraspAxis import GraspAxis
from recipe.defined.GraspMethod import GraspMethod
from recipe.defined.Step import Step

PLAN_SCHEMA = 'assembly.plan/1.0'

# 이전 DXF의 GRASP 속성은 명목 치수가 들어간 이름을 쓴다(75×25×15 블록 기준).
# 그 이름이 뜻하는 닫힘 축으로 먼저 바꾸고, CAD 배치 방향을 보고 파지 방법으로 바꾼다.
# 예: SIDE_25 → WIDTH → (눕힌 블록이면) FLAT_SHORT, THICKNESS_15 → THICKNESS → EDGE_SHORT 또는 STAND_SHORT
# CAD 힌트를 읽을 때만 쓴다.
LEGACY_GRASP = {'END_75': 'LENGTH', 'SIDE_25': 'WIDTH', 'THICKNESS_15': 'THICKNESS'}


@dataclass
class Plan:
    """사람이 채우는 레시피 초안.

    inspect 명령이 양식을 만들고, 사람이 각 단계의 sequence·stage·grasp(파지 방법)를 채운다.
    build 명령이 이 계획을 검증해 Recipe를 만든다.
    """

    recipe_id: str
    revision: str
    model_id: str           # 이 계획이 대상으로 하는 모델
    model_revision: str
    source_cad_sha256: str  # 계획을 만들 때 읽은 CAD의 해시. CAD가 바뀌면 build가 거부한다.
    steps: list[Step]

    @classmethod
    def create_template(cls, model, hints=None):
        """모델의 CADPartInstance마다 Step 하나를 만든 양식. 힌트(DXF 속성)가 있으면 미리 채운다.

        GRASP 값은 파지 방법 이름(FLAT_SHORT 등), 닫힘 축 이름(WIDTH 등), 이전 이름(SIDE_25 등) 중 하나다.

        입력:
            model: Model
            hints: {instance_id: {'SEQ': '1', 'STAGE': '1', 'GRASP': 'SIDE_25'}, ...} 또는 None
        출력: Plan
        실패: SEQ·STAGE가 정수가 아니거나 GRASP 이름을 모르면 ValueError.
        """
        # 부품마다 단계 하나, 힌트로 sequence·stage·grasp 채우기
        hints = hints or {}
        steps = []
        for instance in model.instances:
            key = instance.instance_id
            hint = hints.get(key, {})
            step = Step(step_id='PLACE_' + key, instance_id=key)

            # DXF 속성값은 문자열이므로 정수로 바꾼다.
            for tag, attribute in (('SEQ', 'sequence'), ('STAGE', 'stage')):
                if tag in hint:
                    try:
                        setattr(step, attribute, int(hint[tag]))
                    except ValueError:
                        raise ValueError(f'{key}: CAD attribute {tag} must be an integer, '
                                         f'got {hint[tag]!r}') from None

            # 파지 방법 이름은 그대로, 축 이름·이전 이름은 CAD 배치 방향을 보고 파지 방법으로 바꾼다.
            if 'GRASP' in hint:
                value = hint['GRASP']
                if value in GraspMethod.__members__:
                    step.grasp = GraspMethod[value]
                else:
                    axis_name = LEGACY_GRASP.get(value, value)
                    if axis_name not in GraspAxis.__members__:
                        raise ValueError(f'{key}: unknown CAD GRASP attribute {value!r}')
                    step.grasp = GraspMethod.find_from_closing_axis(instance, GraspAxis[axis_name])

            steps.append(step)


        # 모든 순서가 채워졌으면 순서대로 정렬해 두어야 사람이 읽기 쉽다.
        if all(step.sequence is not None for step in steps):
            steps.sort(key=lambda step: step.sequence)


        # 계획 생성
        return cls(recipe_id=model.model_id + '_ASSEMBLY', revision='1',
                   model_id=model.model_id, model_revision=model.revision,
                   source_cad_sha256=model.source_sha256, steps=steps)

    @classmethod
    def create_from_dict(cls, data):
        """JSON에서 읽은 계획을 만든다. 값의 내용 검사는 Recipe.build_from_plan가 한다. 여기서는 형식만 맞춘다.

        입력:
            data: plan.json 내용 (dict)
        출력: Plan
        실패: schema가 다르거나 steps 형식이 틀리거나 grasp 이름을 모르면 ValueError.
        """
        # 스키마와 단계 목록 형식 확인
        if not isinstance(data, dict) or data.get('schema') != PLAN_SCHEMA:
            raise ValueError(f'Expected plan schema {PLAN_SCHEMA}')
        if not isinstance(data.get('steps'), list) or not all(isinstance(s, dict) for s in data['steps']):
            raise ValueError('Plan steps must be a list of objects')


        # 단계 읽기. grasp는 파지 방법 이름('FLAT_SHORT' 등) 또는 null
        steps = []
        for item in data['steps']:
            grasp = item.get('grasp')
            if grasp is not None and grasp not in GraspMethod.__members__:
                raise ValueError(f'{item.get("step_id")}: grasp must be one of '
                                 f'{list(GraspMethod.__members__)}, got {grasp!r}')
            steps.append(Step(step_id=item.get('step_id'),
                              instance_id=item.get('instance_id'),
                              sequence=item.get('sequence'),
                              stage=item.get('stage'),
                              grasp=GraspMethod[grasp] if grasp is not None else None))


        # 계획 생성
        return cls(recipe_id=data.get('recipe_id'), revision=data.get('revision'),
                   model_id=data.get('model_id'), model_revision=data.get('model_revision'),
                   source_cad_sha256=data.get('source_cad_sha256'), steps=steps)

    def convert_to_dict(self):
        """JSON으로 저장할 형태로 바꾼다.

        출력: dict
        """
        # 단계: 닫힘 축과 지지 부품은 사람이 쓰는 값이 아니라 Recipe.build_from_plan가 계산하므로 계획 파일에는 넣지 않는다.
        steps = []
        for step in self.steps:
            item = step.convert_to_dict()
            del item['grasp_axis']
            del item['support_instance_ids']
            steps.append(item)


        # 계획 전체
        return {
            'schema': PLAN_SCHEMA,
            'recipe_id': self.recipe_id,
            'revision': self.revision,
            'model_id': self.model_id,
            'model_revision': self.model_revision,
            'source_cad_sha256': self.source_cad_sha256,
            'steps': steps,
        }

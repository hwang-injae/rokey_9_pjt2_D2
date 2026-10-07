from dataclasses import dataclass, field

from recipe.defined.GraspAxis import GraspAxis
from recipe.defined.GraspMethod import GraspMethod


@dataclass
class Step:
    """작업지시서의 한 단계: 부품 하나를 집어서 목표 위치에 놓는다.

    계획(Plan)에서는 sequence·stage·grasp가 비어(None) 있을 수 있다.
    레시피(Recipe)가 되면 모두 채워지고 grasp_axis·support_instance_ids가 계산된다.
    """

    step_id: str       # 'PLACE_<instance_id>'
    instance_id: str   # 놓을 대상 (Model의 CADPartInstance)

    # 실행 순서 1..N. 빠짐·중복 없이 연속이어야 한다.
    sequence: int | None = None

    # 단계 묶음(예: 벤치의 '1층', '2층' …). 작업자 확인 지점으로 쓴다.
    # 1부터 시작하고, 순서를 따라 같거나 1씩 증가한다.
    stage: int | None = None

    # 파지 방법 (예: FLAT_SHORT = 눕혀서 짧게). 사람이 입력한다. 상태는 CAD 배치와 같아야 한다.
    grasp: GraspMethod | None = None

    # 그리퍼가 닫히는 부품 축. 사람이 쓰지 않는다. Recipe.build_from_plan가 grasp와 블록 치수로 계산한다.
    grasp_axis: GraspAxis | None = None

    # 이 단계 전에 이미 놓여 있어야 하는 지지 부품(바로 아래에 닿는 부품).
    # 사람이 쓰지 않는다. Recipe.build_from_plan가 형상과 순서로 계산한다. 책상 위에 놓으면 빈 목록.
    support_instance_ids: list[str] = field(default_factory=list)

    # 팀이 부르는 블록 이름 '<모델ID>_B<sequence 3자리>'(예: 001_CHAIR_BENCH_B001, 10/7 한세교 W105).
    # instance_id(DXF 핸들)는 CAD를 다시 저장하면 바뀌므로 노드·사람 사이에서는 이 이름을 쓴다.
    # 사람이 쓰지 않는다. Recipe.build_from_plan가 sequence로 붙인다.
    block_id: str | None = None
    support_block_ids: list[str] = field(default_factory=list)

    def convert_to_dict(self):
        """JSON으로 저장할 형태로 바꾼다. Enum은 이름 문자열로 쓴다.

        출력: dict. block_id가 없으면(계획 단계) block_id·support_block_ids 칸을 쓰지 않는다.
        """
        names = {} if self.block_id is None else {'block_id': self.block_id}
        support_names = {} if self.block_id is None else {'support_block_ids': list(self.support_block_ids)}
        return {
            **names,
            'step_id': self.step_id,
            'instance_id': self.instance_id,
            'sequence': self.sequence,
            'stage': self.stage,
            'grasp': self.grasp.name if self.grasp else None,
            'grasp_axis': self.grasp_axis.name if self.grasp_axis else None,
            'support_instance_ids': list(self.support_instance_ids),
            **support_names,
        }

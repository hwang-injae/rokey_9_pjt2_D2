from enum import Enum

from model.defined.BoundingBox import TOL
from recipe.defined.GraspAxis import GraspAxis

# 놓인 상태 → 그 상태에서 수직(위)을 향하는 부품 축. 부품 치수는 L ≥ W ≥ T 순서다.
#   FLAT  (눕힘)   큰 면 L×W가 바닥 → 두께 T가 위
#   EDGE  (옆세움) 중간 면 L×T가 바닥 → 폭 W가 위
#   STAND (세움)   작은 면 W×T가 바닥 → 길이 L이 위
VERTICAL_AXIS = {'FLAT': GraspAxis.THICKNESS, 'EDGE': GraspAxis.WIDTH, 'STAND': GraspAxis.LENGTH}


class GraspMethod(Enum):
    """파지 방법 6가지: <놓인 상태>_<LONG|SHORT> (docs/파지방법_6가지_v0.1.md).

    말 "〈상태〉 〈길게|짧게〉 잡아"와 1:1이다. 그리퍼는 위에서 내려오므로 닫는 방향은 항상 수평이다.
    - 상태: 블록의 어느 면이 바닥인가. CAD 배치 방향(R)으로 정해지므로 CAD와 맞아야 한다.
    - LONG: 수평 치수 둘 중 큰 쪽을 손가락 사이에 끼운다. SHORT: 작은 쪽을 끼운다.
    치수를 이름에 넣지 않는다. 닫는 폭은 그 블록의 치수로 계산한다.
    """

    FLAT_SHORT = 'FLAT_SHORT'     # 눕혀서 짧게   (젠가 25 mm, 1차 팀 이름 SIDE_25)
    FLAT_LONG = 'FLAT_LONG'       # 눕혀서 길게   (젠가 75 mm, 1차 팀 이름 END_75)
    EDGE_SHORT = 'EDGE_SHORT'     # 옆으로 세워서 짧게 (젠가 15 mm)
    EDGE_LONG = 'EDGE_LONG'       # 옆으로 세워서 길게 (젠가 75 mm)
    STAND_SHORT = 'STAND_SHORT'   # 위로 세워서 짧게   (젠가 15 mm)
    STAND_LONG = 'STAND_LONG'     # 위로 세워서 길게   (젠가 25 mm)

    def get_required_state(self):
        """이 파지 방법이 전제하는 놓인 상태.

        출력: 'FLAT' / 'EDGE' / 'STAND'
        """
        return self.value.split('_')[0]

    @property
    def is_long(self):
        """수평 치수 중 긴 쪽을 끼우는 방법인지.

        출력: LONG이면 True, SHORT이면 False
        """
        return self.value.endswith('_LONG')

    @staticmethod
    def read_placed_state(instance):
        """CAD 배치에서 블록이 놓인 상태를 읽는다.

        R의 k번째 열이 부품 k번째 축의 방향이고, 3번째 행이 높이(Z) 성분이다. Z 성분이 ±1인 축이 위를 향한다.

        입력:
            instance: CADPartInstance
        출력: 'FLAT' / 'EDGE' / 'STAND'
        실패: 위를 향하는 부품 축이 없으면(축 정렬이 아님) ValueError.
        """
        for state, axis in VERTICAL_AXIS.items():
            if abs(abs(instance.R[2][axis.value]) - 1.0) <= TOL:
                return state
        raise ValueError(f'{instance.instance_id}: no part axis points up; only axis-aligned placement is supported')

    @staticmethod
    def find_horizontal_axes(instance):
        """이 배치에서 수평인 부품 축 둘을 긴 쪽부터 돌려준다.

        치수가 같으면(정사각 단면) 길게·짧게가 같은 잡기라서, 축 번호가 작은 쪽을 '긴 쪽'으로 고정한다.

        입력:
            instance: CADPartInstance
        출력: [긴 쪽 GraspAxis, 짧은 쪽 GraspAxis]
        """
        # 위를 향하는 축을 뺀 나머지 두 축
        up = VERTICAL_AXIS[GraspMethod.read_placed_state(instance)]
        axes = [axis for axis in GraspAxis if axis != up]


        # 치수 큰 순서로 정렬 (같으면 축 번호 순)
        size = instance.part.size_mm
        return sorted(axes, key=lambda axis: (-size[axis.value], axis.value))

    def find_closing_axis(self, instance):
        """이 파지 방법으로 instance를 잡을 때 그리퍼가 닫히는 부품 축을 구한다.

        상태가 CAD 배치와 다르면 ValueError. 예: 세워 놓을 블록을 FLAT_*로 잡을 수 없다.

        입력:
            instance: CADPartInstance
        출력: GraspAxis
        실패: 상태가 CAD 배치와 다르면 ValueError.
        """
        # 상태가 CAD 배치와 같은지 확인
        placed = self.read_placed_state(instance)
        if self.get_required_state() != placed:
            raise ValueError(f'{instance.instance_id}: grasp {self.name} needs the part {self.get_required_state()}, '
                             f'but CAD places it {placed}')


        # 수평 축 중 긴 쪽 또는 짧은 쪽 선택
        long_axis, short_axis = self.find_horizontal_axes(instance)
        return long_axis if self.is_long else short_axis

    @classmethod
    def find_from_closing_axis(cls, instance, axis):
        """닫힘 축 → 파지 방법. 이전 형식(축 이름, SIDE_25 등)을 바꿀 때 쓴다.

        입력:
            instance: CADPartInstance
            axis: 그리퍼가 닫히는 GraspAxis (수평이어야 한다)
        출력: GraspMethod
        실패: 닫힘 축이 수직이면 ValueError.
        """
        # 닫힘 축이 수평인지 확인
        long_axis, short_axis = cls.find_horizontal_axes(instance)
        if axis not in (long_axis, short_axis):
            raise ValueError(f'{instance.instance_id}: closing axis {axis.name} is vertical; '
                             'top-down grasp needs a horizontal axis')


        # 상태 + 긴 쪽/짧은 쪽으로 이름 조립
        return cls[f'{cls.read_placed_state(instance)}_{"LONG" if axis == long_axis else "SHORT"}']

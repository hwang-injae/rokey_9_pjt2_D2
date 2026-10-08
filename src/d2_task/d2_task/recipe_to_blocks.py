# -*- coding: utf-8 -*-
"""변환기 ② — 레시피(structure + cad_recipe/1.0 두 파일)를 블록 JSON(blocks/1)으로 바꾼다 (ROS 없이 동작, W117).

기본 설계(벤치 · 의자 · 책상)를 DB와 AI 생성의 예시로 넘길 때, 그리고 검사 묶음이 같은 형식으로 시험할 때 쓴다.
검사 묶음 · HMI 가 부른다. 레시피를 읽기만 하고 파일 · 로봇 · 메시지에는 손대지 않는다.
"""

from d2_task.recipe_document import RecipeDocument

SCHEMA_OUT = 'blocks/1'                # IRD 3장 blocks/1 (안)



def ori_extents(block_mm):
    """블록 크기(길이 · 폭 · 두께, mm) → 방향 코드마다 (x · y · z 방향 길이). IRD 2장 `ori` 표 · jenga_check.py SIZE 와 같은 규칙.

    크기는 robot.yaml 의 block_size_m 한 곳에서 온다(검사 묶음과 같은 값을 쓰려고 이 함수를 같이 쓴다).
    """
    L, W, T = block_mm
    return {'x': (L, W, T), 'y': (W, L, T), 'xe': (L, T, W), 'ye': (T, L, W), 'zx': (T, W, L), 'zy': (W, T, L)}


class RecipeToBlocks:
    """레시피 하나를 blocks/1 하나로 바꾸는 변환기.

    입력: recipe(cad_recipe/1.0) + structure(cad_structure/1.0) dict(단위 mm). 출력: blocks/1 dict(mm, 설계 좌표계 — 바닥 외곽 가운데 = (0,0)).
    바깥 영향: 없음(계산만).
    실패: 형식이 다르거나 없는 instance · part 를 가리키거나 방향을 못 고르면 ValueError (어느 블록인지 메시지에 적는다).
    **블록 크기를 두 곳에서 가져온다:** ① 방향 길이 계산 = 레시피 parts[].size_mm, ② 방향 코드 표 = robot.yaml block_size_m × 1000(생성자의 block_mm).
    두 값은 **허용 오차 없이 같아야** 길이_i 가 표 6개 중 정확히 하나와 맞아 방향이 정해진다(지금 둘 다 설계 기준값 75 · 25 · 15 mm — 젠가 하나로 고정).
    """

    def __init__(self, design_id, family, block_mm):
        """design_id · family 는 레시피에 없으므로 부르는 쪽이 정해서 넣는다(규칙을 새로 만들지 않는다).

        block_mm: 블록 크기 [길이, 폭, 두께] mm — robot.yaml block_size_m × 1000. 방향 코드 표를 만드는 데 쓴다.
        레시피 parts[].size_mm 와 허용 오차 없이 같아야 한다(다르면 convert 가 방향을 못 골라 ValueError).
        """
        if not design_id or not family:
            raise ValueError('design_id 와 family 가 필요하다')
        self.design_id = design_id
        self.family = family
        self.extent = ori_extents(block_mm)

    def convert(self, recipe, structure):
        """recipe + structure(mm)를 blocks/1 로 바꾼다. 파일·로봇 영향 없이 잘못된 참조는 ValueError.

        order = sequence, x · y = 블록 중심, z = 아랫면 높이, ori = 회전 + 부품 크기로 복원.

        반환: {"schema","design_id","family","blocks":[{"order","x","y","z","ori","inferred"}]} (order 오름차순).
        """
        recipe = RecipeDocument(recipe, structure).geometry
        model = recipe.get('model')
        if not isinstance(model, dict) or model.get('frame', {}).get('units') != 'mm':
            raise ValueError('model.frame.units 가 mm 가 아니다')
        parts = {p['part_id']: p['size_mm'] for p in model.get('parts', [])}
        instances = {i['instance_id']: i for i in model.get('instances', [])}

        blocks, seen = [], set()
        for step in sorted(recipe.get('steps', []), key=lambda s: s['sequence']):
            seq, iid = step['sequence'], step['instance_id']
            if seq in seen:
                raise ValueError(f'sequence {seq} 가 두 번 나온다')
            seen.add(seq)
            if iid not in instances:
                raise ValueError(f'sequence {seq}: instance {iid} 가 model.instances 에 없다')
            inst = instances[iid]
            if inst['part_id'] not in parts:
                raise ValueError(f'sequence {seq}: part {inst["part_id"]} 가 model.parts 에 없다')
            x, y, cz = inst['center_mm']
            extent = self._world_extent(seq, inst['R'], parts[inst['part_id']])
            blocks.append({'order': seq, 'x': x, 'y': y, 'z': cz - extent[2] / 2,
                           'ori': self._ori(seq, extent), 'inferred': False})
        if not blocks:
            raise ValueError('레시피에 steps 가 없다')
        return {'schema': SCHEMA_OUT, 'design_id': self.design_id, 'family': self.family, 'blocks': blocks}

    @staticmethod
    def _world_extent(seq, R, size):
        """회전 R(열 = 블록 길이 · 폭 · 두께 축이 가리키는 설계 좌표)을 적용한 x · y · z 방향 길이(mm).

        R 이 축에 맞춘 90° 회전(각 행 · 열에 ±1 이 하나, 나머지 0, 행렬식 +1)이 아니면 ValueError.
        레시피 값이 정확히 0 · ±1 이라 오차 없이 == 로 비교한다(허용오차 없음).
        """
        for line in (R, list(zip(*R))):
            for v in line:
                if sorted(abs(e) for e in v) != [0, 0, 1]:
                    raise ValueError(f'sequence {seq}: 회전 R 이 축에 맞춘 90° 회전이 아니라 방향을 고를 수 없다 {R}')
        det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
               + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
        if det != 1:
            raise ValueError(f'sequence {seq}: 회전 R 의 행렬식이 {det} 이다(거울 반사는 쓸 수 없다) {R}')
        return [sum(abs(R[i][k]) * size[k] for k in range(3)) for i in range(3)]

    def _ori(self, seq, extent):
        """x · y · z 방향 길이가 방향 코드 표(self.extent)의 정확히 한 코드와 같으면 그 코드. 아니면 ValueError."""
        hits = [o for o, e in self.extent.items() if tuple(extent) == e]
        if len(hits) != 1:
            raise ValueError(f'sequence {seq}: 크기 {extent} 에 맞는 방향 코드가 {len(hits)}개다(x·y·xe·ye·zx·zy 중 하나여야 함)')
        return hits[0]

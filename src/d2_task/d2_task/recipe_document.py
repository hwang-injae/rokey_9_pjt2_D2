# -*- coding: utf-8 -*-
"""E-52 레시피 읽기: 옛 한 파일 · 새 structure/recipe 두 파일을 함께 받는다. ROS 없이 변환기와 작업 판단이 공유한다."""
import copy
import hashlib
import json
import math
import re
from pathlib import Path


class RecipeDocument:
    """원본 두 문서를 보존하고 기존 좌표 계산용 model/steps 표현을 만든다. 파일 읽기 외 로봇·메시지 영향 없음.

    새 문서는 모델 ID · 블록 참조 · 순서 · 받침 · 유한 좌표를 검증한다. 잘못된 입력은 ValueError.
    structure_sha256 은 파일 입력이면 원본 바이트로 확인한다. dict 입력은 원본 바이트가 없으므로 해시 모양만 확인한다.
    """

    def __init__(self, recipe, structure=None):
        """recipe 와 선택 structure 는 mm 단위 dict. geometry 는 기존 motion_math 가 읽는 표현(깊은 복사)이다."""
        if not isinstance(recipe, dict) or recipe.get('schema') not in ('assembly.recipe/1.0', 'cad_recipe/1.0'):
            raise ValueError('recipe schema 는 assembly.recipe/1.0 또는 cad_recipe/1.0 이어야 한다')
        self.recipe, self.structure = copy.deepcopy(recipe), copy.deepcopy(structure)
        if 'model' in recipe and structure is not None:
            raise ValueError('structure 는 model 을 포함하지 않는 새 recipe 와 함께 써야 한다')
        self.geometry = self._join() if 'model' not in recipe else copy.deepcopy(recipe)

    @classmethod
    def load(cls, folder, design_id):
        """새 _recipe.json 을 우선 읽고, 없을 때만 옛 .recipe.json 을 읽는다. 새 파일이 깨졌으면 옛 파일로 대체하지 않는다.

        새 두 파일은 원본 structure 바이트의 SHA256 이 조립 문서와 같아야 한다. 파일 없음은 OSError, 내용 오류는 ValueError.
        """
        if not isinstance(design_id, str) or not design_id or design_id.startswith('.') or any(c in design_id for c in '/\\'):
            raise ValueError('design_id 는 경로가 아닌 파일 이름이어야 한다')
        folder = Path(folder)
        recipe_path = folder / f'{design_id}_recipe.json'
        if not recipe_path.exists() and not recipe_path.is_symlink():
            recipe_path = folder / f'{design_id}.recipe.json'
        recipe = cls._json(recipe_path.read_bytes())
        if isinstance(recipe, dict) and 'model' in recipe:
            return cls(recipe)
        raw = (folder / f'{design_id}_structure.json').read_bytes()
        document = cls(recipe, cls._json(raw))
        if recipe['structure_sha256'] != hashlib.sha256(raw).hexdigest():
            raise ValueError('structure_sha256 이 구조 파일과 다르다')
        return document

    def design(self, design_id):
        """원본 recipe 와 structure 를 design/1 객체로 반환한다. 입력 문서를 변경하지 않는다."""
        result = {'schema': 'design/1', 'design_id': design_id, 'recipe': copy.deepcopy(self.recipe)}
        if self.structure is not None:
            result['structure'] = copy.deepcopy(self.structure)
        return result

    @staticmethod
    def _json(raw):
        """JSON 을 읽되 NaN·Infinity·지수 넘침은 ValueError 로 거절한다."""
        def constant(value):
            """JSON 에 없는 상수 이름을 거절한다."""
            raise ValueError(f'유한한 JSON 숫자가 아니다: {value}')

        def number(value):
            """실수 지수 넘침을 읽는 순간 거절한다."""
            out = float(value)
            if not math.isfinite(out):
                raise ValueError(f'유한한 JSON 숫자가 아니다: {value}')
            return out

        return json.loads(raw, parse_constant=constant, parse_float=number)

    @staticmethod
    def _vector(value, size, label):
        """유한한 숫자 목록을 검증한다. 단위는 호출 문맥의 mm 또는 회전 성분. bool 은 숫자가 아니다."""
        try:
            valid = isinstance(value, list) and len(value) == size and all(
                not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) for v in value)
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f'{label} 는 유한한 숫자 {size}개여야 한다')

    def _join(self):
        """블록 이름으로 두 문서를 잇고 model/steps 를 만든다. 위치 계산은 하지 않아 기존 motion_math 를 그대로 쓴다."""
        r, s = self.recipe, self.structure
        if r['schema'] != 'cad_recipe/1.0' or not isinstance(s, dict) or s.get('schema') != 'cad_structure/1.0':
            raise ValueError('새 recipe 에는 cad_structure/1.0 객체가 필요하다')
        model_id = r.get('model_id')
        if not isinstance(model_id, str) or not model_id or s.get('model_id') != model_id:
            raise ValueError('structure 와 recipe 의 model_id 가 다르거나 없다')
        if not isinstance(r.get('structure_sha256'), str) or not re.fullmatch(r'[0-9a-f]{64}', r['structure_sha256']):
            raise ValueError('structure_sha256 은 SHA256 16진수 64자여야 한다')
        parts, blocks, steps = s.get('parts'), s.get('blocks'), r.get('steps')
        if not all(isinstance(items, list) and items for items in (parts, blocks, steps)):
            raise ValueError('parts · blocks · steps 는 비어 있지 않은 목록이어야 한다')
        sizes = {}
        for p in parts:
            if not isinstance(p, dict) or not isinstance(p.get('part_id'), str) or p['part_id'] in sizes:
                raise ValueError('part_id 가 없거나 중복이다')
            self._vector(p.get('size_mm'), 3, 'size_mm')
            if any(v <= 0 for v in p['size_mm']):
                raise ValueError('size_mm 은 양수여야 한다')
            sizes[p['part_id']] = p['size_mm']
        instances = {}
        for b in blocks:
            if not isinstance(b, dict) or not isinstance(b.get('block'), str) or not re.fullmatch(
                    r'[A-Z]+(?:_[A-Z]+)*_[0-9]{3}_[0-9]{2}', b['block']) or b['block'] in instances:
                raise ValueError('block 이름이 역할_부품_블록 형식이 아니거나 중복이다')
            if not isinstance(b.get('part_id'), str) or b['part_id'] not in sizes:
                raise ValueError(f'{b["block"]}: 없는 part_id')
            self._vector(b.get('center_mm'), 3, 'center_mm')
            R = b.get('R')
            if not isinstance(R, list) or len(R) != 3:
                raise ValueError('R 은 3x3 회전이어야 한다')
            for row in R:
                self._vector(row, 3, 'R')
            if any(sorted(abs(v) for v in row) != [0, 0, 1] for row in R + [list(c) for c in zip(*R)]):
                raise ValueError('R 은 축에 맞춘 회전이어야 한다')
            det = sum(R[0][i] * (R[1][(i + 1) % 3] * R[2][(i + 2) % 3] -
                                R[1][(i + 2) % 3] * R[2][(i + 1) % 3]) for i in range(3))
            if det != 1:
                raise ValueError('R 은 거울 반사가 아닌 회전이어야 한다')
            instances[b['block']] = dict(instance_id=b['block'], part_id=b['part_id'], center_mm=b['center_mm'], R=R)
        sequences, rows = {}, []
        for st in steps:
            if not isinstance(st, dict) or not isinstance(st.get('block'), str) or st['block'] not in instances:
                raise ValueError('step 이 구조에 없는 block 을 가리킨다')
            seq = st.get('sequence')
            if isinstance(seq, bool) or not isinstance(seq, int) or seq <= 0 or seq in sequences.values() or st['block'] in sequences:
                raise ValueError('sequence · step block 이 중복이거나 잘못됐다')
            if st.get('grasp_axis') not in ('LENGTH', 'WIDTH', 'THICKNESS') or st.get('grasp') not in (
                    'FLAT_LONG', 'FLAT_SHORT', 'EDGE_LONG', 'EDGE_SHORT', 'STAND_LONG', 'STAND_SHORT'):
                raise ValueError('grasp · grasp_axis 가 잘못됐다')
            if isinstance(st.get('stage'), bool) or not isinstance(st.get('stage'), int) or st['stage'] <= 0:
                raise ValueError('stage 는 양의 정수여야 한다')
            supports = st.get('supports')
            if not isinstance(supports, list) or any(not isinstance(k, str) or k not in instances for k in supports):
                raise ValueError('supports 가 구조에 없는 block 을 가리킨다')
            if len(supports) != len(set(supports)):
                raise ValueError('supports 에 같은 block 이 중복이다')
            sequences[st['block']] = seq
            rows.append(dict(st, instance_id=st['block'], block_id=f'{model_id.upper()}_{st["block"]}',
                             support_instance_ids=supports))
        if set(sequences) != set(instances):
            raise ValueError('structure 의 모든 block 이 recipe 에 한 번씩 있어야 한다')
        for st in steps:
            if any(sequences[k] >= st['sequence'] for k in st['supports']):
                raise ValueError('받침은 해당 블록보다 먼저 놓아야 한다')
        return {'schema': 'cad_recipe/1.0', 'model': {'model_id': model_id, 'frame': {'units': 'mm'},
                'parts': copy.deepcopy(parts), 'instances': list(instances.values())}, 'steps': rows}

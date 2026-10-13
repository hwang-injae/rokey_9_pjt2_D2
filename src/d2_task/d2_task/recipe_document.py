# -*- coding: utf-8 -*-
"""E-69 레시피 읽기: 구조 `<모델ID>_recipe.json`(recipe/3.0) + 조립 방법 `<모델ID>_placements.csv`(placements/3.0)를 읽는다.

ROS 없이 변환기와 작업 판단이 공유한다. 웹 backend 도 import 한다(E-59) → rclpy · ROS 메시지 import 금지.
읽는 쪽은 칸 이름이 아니라 `schema` 로 내용을 확인한다: 이름이 다르거나 앞자리가 다르면 거절하고(변환하지 않음) 이유에 받은 schema 를 적는다(IRD 2장).
"""
import copy
import csv
import hashlib
import io
import json
import math
import re
from pathlib import Path

RECIPE_SCHEMA = 'recipe/3.0'          # 10/13 PLM: stocks 자재 참조와 template 트리 사용, 최상위 parts 없애고 template.composition[역할].parts[그룹] 로(같은 3.0 안에서 바뀐 구조 — PL 결정)
PLACEMENTS_SCHEMA = 'placements/3.0'
GRASPS = ('FLAT_LONG', 'FLAT_SHORT', 'EDGE_LONG', 'EDGE_SHORT', 'STAND_LONG', 'STAND_SHORT')


def same_major(got, expected):
    """형식 이름이 앞자리까지 같은지 본다(IRD 6장 형식 버전 규칙 — 뒷자리 +1 은 선택 칸을 더한 호환 변경이라 받는다).
    예: recipe/3.1 ↔ recipe/3.0 → True, recipe/2.0 ↔ recipe/3.0 → False. 웹 design_store.same_major 와 같은 식(점 없는 'state/2'도 앞자리로 본다)."""
    if not isinstance(got, str) or '/' not in got:
        return False
    name, ver = got.split('/', 1)
    exp_name, exp_ver = expected.split('/', 1)
    return name == exp_name and ver.split('.')[0] == exp_ver.split('.')[0]


def check_schema(obj, expected, label):
    """obj 가 객체이고 schema 가 expected 와 앞자리까지 같은지 본다(same_major). 아니면 받은 schema 를 적어 ValueError
    (옛 recipe/2.0 · cad_* · assembly.recipe 도 여기서 거절)."""
    got = obj.get('schema') if isinstance(obj, dict) else None
    if not same_major(got, expected):
        raise ValueError(f'{label} 의 schema 가 {expected} 와 앞자리가 다르다(받은 schema: {got!r}) — 옛 형식은 변환하지 않고 거절한다')


def recipe_sha256(recipe):
    """구조(recipe/3.0) 객체의 짝 확인 해시: 키 정렬 · 공백 없는 JSON(UTF-8)의 sha256 16진수 64자(한세교 정함 — 파일 바이트가 아니라 객체 기준)."""
    text = json.dumps(recipe, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


class RecipeDocument:
    """구조(recipe) 와 조립 방법(placements) 두 문서를 보존하고 기존 좌표 계산용 model/steps 표현을 만든다. 파일 읽기 외 로봇·메시지 영향 없음.

    두 문서는 schema · 모델 ID · 짝 해시(recipe_sha256) · 블록 참조 · 순서 · 받침 · 유한 좌표를 검증한다. 잘못된 입력은 ValueError.
    """

    def __init__(self, recipe, placements):
        """recipe(recipe/3.0)와 placements(placements/3.0)는 mm 단위 dict. geometry 는 기존 motion_math 가 읽는 표현(깊은 복사)이다."""
        check_schema(recipe, RECIPE_SCHEMA, 'recipe')
        check_schema(placements, PLACEMENTS_SCHEMA, 'placements')
        self.recipe, self.placements = copy.deepcopy(recipe), copy.deepcopy(placements)
        self.geometry = self._join()

    @classmethod
    def load(cls, folder, design_id):
        """folder 의 <design_id>_recipe.json 과 _placements.csv 를 읽는다. 짝 해시(recipe_sha256)가 구조와 맞아야 한다.

        파일 없음은 OSError, 내용 오류는 ValueError. 옛 `_structure.json` 은 읽지 않는다.
        """
        if not isinstance(design_id, str) or not design_id or design_id.startswith('.') or any(c in design_id for c in '/\\'):
            raise ValueError('design_id 는 경로가 아닌 파일 이름이어야 한다')
        folder = Path(folder)
        recipe = cls._json((folder / f'{design_id}_recipe.json').read_bytes())
        placements = cls.placements_from_csv((folder / f'{design_id}_placements.csv').read_text(encoding='utf-8'), design_id)
        return cls(recipe, placements)

    @staticmethod
    def placements_from_csv(text, model_id):
        """placements.csv 글자 → placements/3.0 객체(전달 JSON 모양). 칸 이름(머리줄)으로 읽어 열 순서에 의존하지 않는다.

        쓰는 칸: part_name · part_id · sequence · stage · grasp · grasp_axis · supports(';' 로 구분, 비면 없음) · recipe_sha256 · schema(두 칸 다 모든 줄 같은 값 —
        schema 는 placements/3.0 이어야 하고 아니면 받은 값을 적어 거절한다, 한세교 E-69 PR). 표시 칸(크기 · 중심 · 축)은 읽지 않는다.
        stock_id · cad_handle 칸은 placements/3.0 에서 빠졌다(있어도 읽지 않는다).
        칸이 빠졌거나 숫자가 아니면 ValueError.
        """
        rows = list(csv.DictReader(io.StringIO(text)))
        need = ('part_name', 'part_id', 'sequence', 'stage', 'grasp', 'grasp_axis', 'supports', 'recipe_sha256', 'schema')
        if not rows or any(k not in rows[0] for k in need):
            raise ValueError(f'placements.csv 에 {", ".join(need)} 칸이 모두 있고 한 줄 이상이어야 한다')
        shas, schemas = {r['recipe_sha256'] for r in rows}, {r['schema'] for r in rows}
        if len(shas) != 1 or len(schemas) != 1:
            raise ValueError('placements.csv 의 recipe_sha256 · schema 가 줄마다 다르다')
        steps = []
        for r in rows:
            try:
                seq, stage = int(r['sequence']), int(r['stage'])
            except (TypeError, ValueError) as e:
                raise ValueError(f'placements.csv 의 sequence · stage 가 정수가 아니다: {e}') from e
            steps.append({'part_name': r['part_name'], 'part_id': r['part_id'], 'sequence': seq, 'stage': stage, 'grasp': r['grasp'],
                          'grasp_axis': r['grasp_axis'], 'supports': [k for k in r['supports'].split(';') if k]})
        return {'schema': schemas.pop(), 'model_id': model_id, 'recipe_sha256': shas.pop(), 'steps': steps}

    def design(self, design_id):
        """원본 두 문서를 design/3.0 로컬 파일 모양으로 반환한다(design_id · recipe · placements 만 — IRD 6장 로컬 파일). 입력 문서를 변경하지 않는다."""
        return {'schema': 'design/3.0', 'design_id': design_id, 'recipe': copy.deepcopy(self.recipe),
                'placements': copy.deepcopy(self.placements)}

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

    def _read_composition(self, s):
        """구조(recipe/3.0)의 template.composition[역할].parts[그룹] 부품 항목을 읽어 (stocks, instances) 를 만든다.

        입력: s = recipe/3.0 dict. 역할 키 `LEG`, 그룹 키 `LEG_001`, 부품 항목 {part: '01', stock_id, center_mm, R, cad?}(mm).
        부품 이름 = `<그룹>_<part>` (예 LEG_001_01). 구성요소는 label(글자) · count(그룹 수와 같은 정수) · parts 가 있어야 한다.
        출력: stocks = [{stock_id, size_mm}] 와 instances = {부품 이름: {instance_id, stock_id, center_mm, R}} — 기존 motion_math 가 읽는
        내부 표현이다. stock_id 는 최상위 stocks의 자재를 참조한다. 중복 번호·없는 참조·유한하지 않거나 양수가 아닌 크기는 거절한다.
        `revision` · `name` · `description` · `tags` · `cad` 는 읽지 않는다(검사 21번 · 사람이 보는 칸).
        실패: 옛 구조(최상위 parts)이거나 칸이 빠지거나 값이 틀리면 ValueError.
        """
        if 'parts' in s:
            raise ValueError('옛 구조(최상위 parts)는 읽지 않는다 — template.composition[역할].parts 로 바꿔야 한다')
        template = s.get('template')
        composition = template.get('composition') if isinstance(template, dict) else None
        if not isinstance(composition, dict) or not composition:
            raise ValueError('template.composition 은 비어 있지 않은 객체여야 한다')
        stock_list = s.get('stocks')
        if not isinstance(stock_list, list) or not stock_list:
            raise ValueError('stocks 는 비어 있지 않은 자재 목록이어야 한다')
        stocks_by_id = {}
        for stock in stock_list:
            stock_id = stock.get('stock_id') if isinstance(stock, dict) else None
            if not isinstance(stock_id, str) or not stock_id or stock_id in stocks_by_id:
                raise ValueError('stock_id 는 비어 있지 않고 중복되지 않는 글자여야 한다')
            self._vector(stock.get('size_mm'), 3, f'{stock_id} size_mm')
            if any(v <= 0 for v in stock['size_mm']):
                raise ValueError(f'{stock_id}: size_mm 은 양수여야 한다')
            stocks_by_id[stock_id] = stock['size_mm']
        instances = {}                   # 부품 이름 → 검증된 자재 참조·위치·회전
        for role, comp in composition.items():
            if not isinstance(role, str) or not re.fullmatch(r'[A-Z]+(?:_[A-Z]+)?', role):
                raise ValueError(f'composition 키 {role!r} 가 역할[_옵션] 형식이 아니다')
            groups = comp.get('parts') if isinstance(comp, dict) else None
            if not isinstance(groups, dict) or not groups:
                raise ValueError(f'{role}: parts 는 비어 있지 않은 그룹 객체여야 한다')
            if not isinstance(comp.get('label'), str) or not comp['label']:
                raise ValueError(f'{role}: label 이 없다')
            if type(comp.get('count')) is not int or comp['count'] != len(groups):        # 2.0 · True 같은 비정수 타입은 거절
                raise ValueError(f'{role}: count({comp.get("count")!r}) 가 실제 그룹 수({len(groups)})와 다르다')
            for group, items in groups.items():
                if not isinstance(group, str) or not re.fullmatch(re.escape(role) + r'_[0-9]{3}', group):
                    raise ValueError(f'{role}: 그룹 이름 {group!r} 이 {role}_<3자리> 형식이 아니다')
                if not isinstance(items, list) or not items:
                    raise ValueError(f'{group}: 부품 항목은 비어 있지 않은 목록이어야 한다')
                for item in items:
                    if not isinstance(item, dict) or not isinstance(item.get('part'), str) or not re.fullmatch(r'[0-9]{2}', item['part']):
                        raise ValueError(f'{group}: part 가 2자리 글자가 아니다')
                    name = f'{group}_{item["part"]}'
                    if name in instances:
                        raise ValueError(f'{name}: 부품 이름이 중복이다')
                    stock_id = item.get('stock_id')
                    if not isinstance(stock_id, str) or stock_id not in stocks_by_id:
                        raise ValueError(f'{name}: stock_id 가 stocks 에 없다')
                    self._vector(item.get('center_mm'), 3, f'{name} center_mm')
                    R = item.get('R')
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
                    instances[name] = dict(instance_id=name, stock_id=stock_id, center_mm=item['center_mm'], R=R)
        stocks = copy.deepcopy(stock_list)
        return stocks, dict(sorted(instances.items()))        # 이름순 — DB(JSONB)가 키 순서를 바꿔도 같은 결과

    def _join(self):
        """블록 이름으로 두 문서를 잇고 model/steps 를 만든다. 위치 계산은 하지 않아 기존 motion_math 를 그대로 쓴다."""
        s, r = self.recipe, self.placements             # s = 구조(recipe/3.0), r = 조립 방법(placements/3.0)
        model_id = s.get('model_id')
        if not isinstance(model_id, str) or not model_id or r.get('model_id') != model_id:
            raise ValueError('recipe 와 placements 의 model_id 가 다르거나 없다')
        if r.get('recipe_sha256') != recipe_sha256(s):
            raise ValueError('placements 의 recipe_sha256 이 recipe(구조)와 맞지 않는다 — 다른 구조에 대해 쓴 조립 방법이다')
        steps = r.get('steps')
        if not isinstance(steps, list) or not steps:
            raise ValueError('steps 는 비어 있지 않은 목록이어야 한다')
        stocks, instances = self._read_composition(s)
        sequences, rows = {}, []
        for st in steps:
            if not isinstance(st, dict) or not isinstance(st.get('part_name'), str) or st['part_name'] not in instances:
                raise ValueError('step 이 recipe(구조)에 없는 block 을 가리킨다')
            seq = st.get('sequence')
            if isinstance(seq, bool) or not isinstance(seq, int) or seq <= 0 or seq in sequences.values() or st['part_name'] in sequences:
                raise ValueError('sequence · step block 이 중복이거나 잘못됐다')
            if st.get('grasp_axis') not in ('LENGTH', 'WIDTH', 'THICKNESS') or st.get('grasp') not in GRASPS:
                raise ValueError('grasp · grasp_axis 가 잘못됐다')
            if isinstance(st.get('stage'), bool) or not isinstance(st.get('stage'), int) or st['stage'] <= 0:
                raise ValueError('stage 는 양의 정수여야 한다')
            supports = st.get('supports')
            if not isinstance(supports, list) or any(not isinstance(k, str) or k not in instances for k in supports):
                raise ValueError('supports 가 구조에 없는 block 을 가리킨다')
            if len(supports) != len(set(supports)):
                raise ValueError('supports 에 같은 block 이 중복이다')
            sequences[st['part_name']] = seq
            part_id = f'{model_id.upper()}_{st["part_name"]}'
            if 'part_id' in st and st['part_id'] != part_id:
                raise ValueError(f'step 의 part_id({st["part_id"]!r})가 <모델ID 대문자>_<블록 이름>({part_id!r})과 다르다')
            rows.append(dict(st, instance_id=st['part_name'], part_id=part_id, support_instance_ids=supports))
        if set(sequences) != set(instances):
            raise ValueError('recipe(구조)의 모든 block 이 placements 에 한 번씩 있어야 한다')
        for st in steps:
            if any(sequences[k] >= st['sequence'] for k in st['supports']):
                raise ValueError('받침은 해당 블록보다 먼저 놓아야 한다')
        return {'schema': 'cad_recipe/1.0', 'model': {'model_id': model_id, 'frame': {'units': 'mm'},
                'stocks': copy.deepcopy(stocks), 'instances': list(instances.values())}, 'steps': rows}

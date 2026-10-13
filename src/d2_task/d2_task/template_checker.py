# -*- coding: utf-8 -*-
"""Template 검사(21번) — AI·스캔 설계가 그 Template(기본 설계 V000)의 모양 규칙 안에 있는지 본다 (ROS 없이 동작, W162).

DesignChecker 가 1~20번(형식·파고듦·받침·안정성·잡기·작업영역) 다음에 부른다. 규칙은 요청의 `base_recipe`(그 Template 의 V000 레시피 통째,
`recipe/4.0`)에서 `template.composition[역할]` 의 `count` · `revision` 만 읽는다(보여 줄 이름표 `label` 만 더 읽는다).
설계의 모양은 요청 `parts` 에서 직접 잰다 — 레시피를 믿지 않고 좌표를 잰다. 단위 mm, 설계 좌표계(앞 = -y, 뒤 = +y, 가구는 돌리지 않는다).
실패 코드: 요청이 틀림 = REQUEST_INVALID(웹 쪽 잘못, 다시 만들지 않음) · 역할·개수·놓는 방향·받침·상대 방향 = TEMPLATE_SHAPE ·
범위·같음·대칭·붙는 자리 = TEMPLATE_RANGE (설계 탓이라 AI 가 다시 만든다).
"""
import ast
import math
import re

from d2_task.recipe_document import same_major

BASE_SCHEMA = 'recipe/4.0'
TOL_MM = 0.000001            # 같다고 볼 오차 — 레시피 도구 · 변환기 TOL, 검사 묶음 PENETRATION_MM 과 같은 기준
MAX_EXPR_LEN = 200           # 범위 식 길이 상한(깊게 겹친 괄호로 파서가 느려지는 것을 막는다)
DESIGN_ID_RE = re.compile(r'(.+)_V\d{3}')
ROLE_KEY_RE = re.compile(r'[A-Z]+')
EQUAL_RE = re.compile(r'=\s*([A-Z]+)\.(width|depth|height)')
LAYS = {'FLAT_STACK': '눕혀 위로 쌓기', 'FLAT_SIDE_BY_SIDE': '눕혀 옆으로 나란히 놓기', 'STAND_STACK': '세워 위로 쌓기'}
RELATIONS = {'PERPENDICULAR': '직각', 'PARALLEL': '평행'}
ATS = ('X_ENDS', 'Y_ENDS', 'REAR_END', 'X_CENTER', 'TOP')
REVISION_KEYS = ('lay', 'long_edge', 'support', 'attach', 'height_mm', 'width_mm', 'depth_mm', 'same_size', 'mirror', 'material')
DIM_AXIS = {'width': 0, 'depth': 1, 'height': 2}                         # 치수 이름 → x · y · z
DIM_KEYS = {'width_mm': 'width', 'depth_mm': 'depth', 'height_mm': 'height'}
DIM_KO = {'width': '폭(좌우)', 'depth': '깊이(앞뒤)', 'height': '높이'}
AXIS_KO = ('좌우(x)', '앞뒤(y)', '높이(z)')


class RequestInvalid(ValueError):
    """check_design 요청이 틀렸다(base_recipe 없음·모양 틀림·design_id 안 맞음…). 설계 탓이 아니라 웹 쪽 잘못이라 재생성하지 않는다."""


def apply_operator(op, left, right):
    """식의 이항 연산 하나(+ - * /). 0 으로 나누면 잘못된 규칙이라 RequestInvalid."""
    if isinstance(op, ast.Add):
        return left + right
    if isinstance(op, ast.Sub):
        return left - right
    if isinstance(op, ast.Mult):
        return left * right
    if right == 0:
        raise RequestInvalid('범위 식이 0 으로 나눈다')
    return left / right


class Skip(Exception):
    """기준 값을 구할 수 없어(없는 역할·앞서 실패) 이 검사를 줄이기 위해 건너뛴다. 이미 다른 줄이 이유를 알렸다."""


def _n(value):
    """mm 값을 화면에 보일 글자로(소수 셋째 자리까지, 끝 0 제거)."""
    return f'{value:.3f}'.rstrip('0').rstrip('.')


class TemplateChecker:
    """base_recipe 에서 Template 규칙을 읽고(read_base), 설계의 parts 가 그 규칙에 맞는지 잰다(check).

    입력: extent = 방향 코드 → (x·y·z 길이 mm) (검사 묶음 DesignChecker.extent 와 같은 표 — 블록 크기는 robot.yaml 한 곳).
    바깥 영향: 없음(계산만, 입력을 바꾸지 않는다). 식은 ast 로 읽어 숫자 · + - * / · ROLE.width/depth/height 만 허용하고 eval 은 쓰지 않는다.
    실패: read_base 는 요청이 틀리면 RequestInvalid. check 는 설계가 틀리면 errors 를 돌려주고, 프로그램 예외는 그대로 올린다(서비스가 ERROR 로 알림).
    """

    def __init__(self, extent):
        """extent: {ori: (dx, dy, dz)} mm."""
        self.extent = extent

    # ---------------- 1. 요청 읽기 ----------------
    def read_base(self, request):
        """요청의 base_recipe 를 확인해 규칙 {역할: {label, count, lay, long_edge, support, attach, dims, same_size, mirror}} 를 돌려준다.

        먼저 본다: base_recipe 있음 · 객체 · schema 앞자리 recipe/4 · design_id 가 _V000 으로 끝남 · 요청 design_id(_V3자리 앞부분)와 model_id 앞부분이 같음 ·
        template.composition 있음. 그 뒤 revision 모양(키·값·식)도 틀리면 RequestInvalid(이유를 적음). 요청 안 `template` 칸은 읽지 않는다.
        """
        if not isinstance(request, dict):
            raise RequestInvalid('요청이 객체가 아니다')
        base = request.get('base_recipe')
        if base is None:
            raise RequestInvalid('base_recipe 가 없다 — 그 Template 의 기본 설계(V000) 레시피를 통째로 실어야 한다')
        if not isinstance(base, dict):
            raise RequestInvalid('base_recipe 가 객체가 아니다')
        if not same_major(base.get('schema'), BASE_SCHEMA):
            raise RequestInvalid(f'base_recipe 의 schema 가 {BASE_SCHEMA} 와 앞자리가 다르다(받은 schema: {base.get("schema")!r})')
        base_id = base.get('design_id')
        if not isinstance(base_id, str) or not base_id.endswith('_V000') or len(base_id) == len('_V000'):
            raise RequestInvalid(f'base_recipe 의 design_id {base_id!r} 가 기본 설계(…_V000)가 아니다')
        design_id = request.get('design_id')
        found = DESIGN_ID_RE.fullmatch(design_id) if isinstance(design_id, str) else None
        if not found:
            raise RequestInvalid(f'design_id {design_id!r} 가 <Template ID>_V<3자리> 모양이 아니다')
        if found.group(1) != base_id[:-len('_V000')]:
            raise RequestInvalid(f'design_id {design_id} 의 Template({found.group(1)})가 base_recipe 의 Template({base_id[:-len("_V000")]})와 다르다')
        template = base.get('template')
        composition = template.get('composition') if isinstance(template, dict) else None
        if not isinstance(composition, dict) or not composition:
            raise RequestInvalid('base_recipe 에 template.composition 이 없다')
        return self._read_composition(composition)

    def _read_composition(self, composition):
        """composition 의 역할마다 count · revision(· label)을 규칙으로 읽는다. 모양이 틀리면 RequestInvalid."""
        names = set(composition)
        if any(not isinstance(k, str) or not ROLE_KEY_RE.fullmatch(k) for k in names):
            raise RequestInvalid('composition 의 역할 이름이 영문 대문자 한 단어가 아니다')
        rules = {}
        for role, entry in composition.items():
            if not isinstance(entry, dict):
                raise RequestInvalid(f'{role} 구성요소가 객체가 아니다')
            count = entry.get('count')
            if not isinstance(count, int) or isinstance(count, bool) or count < 1:
                raise RequestInvalid(f'{role} count={count!r} 는 1 이상의 정수가 아니다')
            label = entry.get('label')
            rules[role] = dict(self._read_revision(role, entry.get('revision'), names), label=label if isinstance(label, str) and label else role, count=count)
        return rules

    def _read_revision(self, role, rev, names):
        """revision dict → 규칙 칸. 식은 이때 한 번에 읽어 안전한 것만 남긴다."""
        if not isinstance(rev, dict):
            raise RequestInvalid(f'{role} revision 이 객체가 아니다')
        # 모르는 선택 칸(예: note · 앞으로 더해질 메타데이터)은 읽지 않고 무시한다 — 형식 버전 규칙(앞자리 같으면 뒷자리는 선택 칸 추가의 호환 변경, IRD 6장).
        # 아는 칸(REVISION_KEYS)은 아래에서 그대로 엄격하게 확인한다. 선택 칸을 새 검사로 해석하지 않는다.

        def other(value, where):
            """revision 안에서 가리킨 다른 역할 이름을 확인한다. 입력: value = 역할 이름, where = 칸 이름(오류 문구용). 반환: value. 바깥 영향 없음. 실패: 이 Template 의 다른 역할이 아니면 RequestInvalid."""
            if not isinstance(value, str) or value not in names or value == role:
                raise RequestInvalid(f'{role} revision.{where} {value!r} 가 이 Template 의 다른 역할이 아니다')
            return value

        out = {'lay': None, 'long_edge': None, 'support': None, 'attach': None, 'dims': {}, 'same_size': False, 'mirror': None}
        if 'lay' in rev:
            if not isinstance(rev['lay'], str) or rev['lay'] not in LAYS:        # 글자인지 먼저 — 목록 · 객체는 해시가 안 돼 TypeError
                raise RequestInvalid(f'{role} revision.lay {rev["lay"]!r} 는 {", ".join(LAYS)} 가 아니다')
            out['lay'] = rev['lay']
        if 'long_edge' in rev:
            le = rev['long_edge']
            if not isinstance(le, dict) or not isinstance(le.get('relation'), str) or le['relation'] not in RELATIONS:
                raise RequestInvalid(f'{role} revision.long_edge 는 relation({"·".join(RELATIONS)}) · to 를 가진 객체여야 한다')
            out['long_edge'] = {'relation': le['relation'], 'to': other(le.get('to'), 'long_edge.to')}
        if 'support' in rev:
            sup = rev['support']
            if isinstance(sup,list) and sup:
                roles=['TABLE' if s == 'TABLE' else other(s,'support') for s in sup]
                out['support']={'roles':roles,'per_group':None}
            elif isinstance(sup,dict):
                count=sup.get('per_group')
                if isinstance(count,bool) or not isinstance(count,int) or count<1:
                    raise RequestInvalid(f'{role} revision.support.per_group 는 1 이상의 정수여야 한다')
                out['support']={'roles':[other(sup.get('to'),'support.to')],'per_group':count}
            else:
                raise RequestInvalid(f'{role} revision.support 는 비지 않은 목록 또는 to · per_group 객체여야 한다')
        if 'attach' in rev:
            at = rev['attach']
            if not isinstance(at, dict) or at.get('at') not in ATS or at.get('direction', 'INWARD') != 'INWARD':
                raise RequestInvalid(f'{role} revision.attach 는 to · at({"·".join(ATS)}) · direction(INWARD) · d_mm 을 가진 객체여야 한다')
            out['attach'] = {'to': other(at.get('to'), 'attach.to'), 'at': at['at'],
                             'd': self._read_range(role, 'attach.d_mm', at['d_mm'], names) if 'd_mm' in at else None}
        for key, dim in DIM_KEYS.items():
            if key in rev:
                out['dims'][dim] = self._read_dim(role, key, rev[key], names)
        if 'same_size' in rev:
            if not isinstance(rev['same_size'], bool):
                raise RequestInvalid(f'{role} revision.same_size 가 true · false 가 아니다')
            out['same_size'] = rev['same_size']
        if 'mirror' in rev:
            if rev['mirror'] not in ('X', 'XY'):
                raise RequestInvalid(f'{role} revision.mirror {rev["mirror"]!r} 는 X · XY 가 아니다')
            out['mirror'] = rev['mirror']
        return out

    def _read_dim(self, role, key, value, names):
        """height_mm 등 한 칸 → ('equal', 역할, 치수) 또는 ('range', 최소, 최대)."""
        if isinstance(value, str):
            found = EQUAL_RE.fullmatch(value.strip())
            if not found or found.group(1) not in names or found.group(1) == role:
                raise RequestInvalid(f'{role} revision.{key} {value!r} 는 "= 역할.치수" 모양이 아니다')
            return ('equal', found.group(1), found.group(2))
        return ('range',) + self._read_range(role, key, value, names)

    def _read_range(self, role, where, value, names):
        """[최소, 최대] → (최소, 최대). 각각 숫자(float) 또는 (글자, 안전한 식)."""
        if not isinstance(value, list) or len(value) != 2:
            raise RequestInvalid(f'{role} revision.{where} 는 [최소, 최대] 가 아니다')
        low, high = (self._read_bound(role, where, v, names) for v in value)
        if isinstance(low, float) and isinstance(high, float) and low > high:
            raise RequestInvalid(f'{role} revision.{where} 최소 {_n(low)} > 최대 {_n(high)}')
        return low, high

    def _read_bound(self, role, where, value, names):
        """범위 끝 하나 → float 또는 (원문, ast). 식은 + - * / 와 숫자, 이 Template 역할의 width/depth/height 만 허용한다."""
        if isinstance(value, bool) or not isinstance(value, (int, str, float)):
            raise RequestInvalid(f'{role} revision.{where} 범위 끝 {value!r} 가 숫자도 식도 아니다')
        if not isinstance(value, str):
            try:
                number = float(value)
            except OverflowError:                       # 아주 큰 정수
                number = math.inf
            if not math.isfinite(number):
                raise RequestInvalid(f'{role} revision.{where} 범위 끝이 유한한 수가 아니다')
            return number
        if len(value) > MAX_EXPR_LEN:
            raise RequestInvalid(f'{role} revision.{where} 식이 너무 길다')
        try:
            tree = ast.parse(value.strip(), mode='eval')
        except (SyntaxError, ValueError, RecursionError, MemoryError) as e:
            raise RequestInvalid(f'{role} revision.{where} 식 {value!r} 을 읽을 수 없다') from e
        label = f'{role} revision.{where} 식 {value!r}'
        self._check_node(tree.body, names - {role}, label)      # 자기 역할은 기준이 못 된다(== 형과 같은 규칙)
        self._fold(tree.body, label)
        return (value, tree.body)

    def _fold(self, node, where):
        """역할 치수가 없는 부분을 읽을 때 미리 계산해, 0 으로 나눔 · 유한하지 않은 값이면 잘못된 규칙(RequestInvalid)으로 거절한다.

        역할.치수가 들어 있는 부분은 값을 모르므로 None. 그 값이 0 이 되는 경우는 계산할 때 같은 RequestInvalid 로 알린다(_eval).
        """
        if isinstance(node, ast.Constant):
            return float(node.value)
        if isinstance(node, ast.Attribute):
            return None
        if isinstance(node, ast.UnaryOp):
            value = self._fold(node.operand, where)
            return None if value is None else (-value if isinstance(node.op, ast.USub) else value)
        left, right = self._fold(node.left, where), self._fold(node.right, where)
        if isinstance(node.op, ast.Div) and right == 0:
            raise RequestInvalid(f'{where} 이 0 으로 나눈다')
        if left is None or right is None:
            return None
        value = apply_operator(node.op, left, right)
        if not math.isfinite(value):
            raise RequestInvalid(f'{where} 의 값이 너무 크다')
        return value

    def _check_node(self, node, names, where):
        """식 노드가 허용된 것(숫자 · 단항 +- · 이항 + - * / · ROLE.치수)인지 본다. 아니면 RequestInvalid."""
        if isinstance(node, ast.Constant):
            ok = isinstance(node.value, (int, float)) and not isinstance(node.value, bool)
            if ok:
                try:
                    ok = math.isfinite(float(node.value))
                except OverflowError:
                    ok = False
            if not ok:
                raise RequestInvalid(f'{where} 에 허용되지 않는 값이 있다')
        elif isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
            self._check_node(node.operand, names, where)
        elif isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
            self._check_node(node.left, names, where)
            self._check_node(node.right, names, where)
        elif (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
              and node.value.id in names and node.attr in DIM_AXIS):
            return
        else:
            raise RequestInvalid(f'{where} 에 허용되지 않는 식이 있다(숫자 · + - * / · 역할.width/depth/height 만)')

    # ---------------- 2. 설계 재기 ----------------
    def check(self, rules, parts):
        """설계(parts: 형식이 맞는 parts/4.0 의 parts 목록 — sequence · x · y · z · ori · role · group)를 규칙에 맞춰 잰다.

        반환: errors 목록 [{part: None, reason: TEMPLATE_SHAPE | TEMPLATE_RANGE, detail, kind: 'template'}] — 모양(SHAPE)을 먼저, 같음 · 범위 · 대칭(RANGE)을 뒤에.
        한 줄에 한 가지 실패를 적고, 앞선 실패로 잴 수 없는 값은 같은 이유로 또 알리지 않는다.
        """
        return _Measure(self, rules, parts).run()

    def long_axis(self, ori):
        """방향 코드의 가장 긴 변이 놓인 축(0 x · 1 y · 2 z). 가장 긴 변이 둘 이상이면 None."""
        ext = self.extent[ori]
        longest = max(ext)
        axes = [i for i, e in enumerate(ext) if abs(e - longest) <= TOL_MM]
        return axes[0] if len(axes) == 1 else None


class _Group:
    """같은 전체 역할(옵션 포함, LEG_WHEEL) · 같은 group 번호 부품의 묶음. lo · hi = 상자 모서리(mm), boxes = 부품마다 (order, ori, lo, hi).

    묶음의 신원은 key = (전체 역할, 번호) — 변환기 ①이 묶는 기준과 같다. role = 옵션을 뗀 Template 역할(규칙을 찾고 개수를 세는 기준).
    """

    def __init__(self, full, number):
        """빈 묶음을 만든다. 입력: full = 전체 역할 글자(예 LEG_WHEEL), number = group 번호(정수). 바깥 영향 없음. 부품은 add 로 더하며, 만들 때는 실패하지 않는다."""
        self.full, self.number, self.boxes = full, number, []
        self.role = full.split('_')[0]
        self.key = (full, number)

    def add(self, order, ori, lo, hi):
        """부품 하나를 더하고 묶음 상자를 다시 잰다."""
        self.boxes.append((order, ori, lo, hi))
        self.lo = tuple(min(b[2][i] for b in self.boxes) for i in range(3))
        self.hi = tuple(max(b[3][i] for b in self.boxes) for i in range(3))

    def size(self, axis):
        """그 축(0 x · 1 y · 2 z) 길이."""
        return self.hi[axis] - self.lo[axis]


class _Measure:
    """check() 한 번의 계산 상태 — 묶음 만들기, 실패 모으기(중복 없이), 검사 순서. 밖에서 쓰지 않는다."""

    def __init__(self, checker, rules, parts):
        """부품을 (전체 역할, 번호) 묶음으로 모아 묶음 상자(mm)를 잰다.

        입력: checker = TemplateChecker(방향별 길이 표), rules = read_base 결과, parts = 형식이 맞는 parts/4.0 의 parts 목록(mm).
        바깥 영향 없음. 실패: parts 에 형식이 틀린 항목이 있으면 KeyError 등 프로그램 예외(검사 묶음이 형식을 먼저 확인한다).
        """
        self.checker, self.rules = checker, rules
        self.errors, self.seen = [], set()
        self.broken = set()           # 모양 실패가 이미 알려진 묶음 (역할, 번호) — 거기에 기대는 값은 다시 알리지 않는다
        self.unusable = set()         # 잴 수 없거나 같은 크기가 아니어서 기준으로 못 쓰는 역할
        self.parts = parts
        self.groups = {}              # (전체 역할, 번호) → _Group — 옵션이 다르면 번호가 같아도 다른 묶음
        self.by_role = {}             # Template 역할(옵션 뗀 것) → (전체 역할, 번호) 순 _Group 목록 (글자 · 정수 쌍이라 정렬이 늘 같다)
        for p in parts:
            dx, dy, dz = checker.extent[p['ori']]
            group = self.groups.setdefault((p['role'], p['group']), _Group(p['role'], p['group']))
            group.add(p['sequence'], p['ori'], (p['x'] - dx / 2, p['y'] - dy / 2, p['z']), (p['x'] + dx / 2, p['y'] + dy / 2, p['z'] + dz))
        for key in sorted(self.groups):
            self.by_role.setdefault(self.groups[key].role, []).append(self.groups[key])

    def add(self, reason, detail):
        """실패 한 줄을 더한다(같은 줄은 한 번만)."""
        if (reason, detail) not in self.seen:
            self.seen.add((reason, detail))
            self.errors.append({'part': None, 'reason': reason, 'detail': f'21번 {"형태" if reason == "TEMPLATE_SHAPE" else "범위"}: {detail}', 'kind': 'template'})

    def label(self, role):
        """화면 이름표(없으면 역할 이름)."""
        return self.rules[role]['label'] if role in self.rules else role

    def who(self, group):
        """실패 문구에 쓰는 이름 — 묶음이 둘 이상이면 번호도 붙인다."""
        label = self.label(group.role)
        return label if len(self.by_role[group.role]) == 1 else f'{label}({self.tag(group)})'

    def tag(self, group):
        """묶음 표시 '그룹 번호' — 같은 역할 안에 옵션이 섞여 있으면 전체 역할도 적는다('그룹 LEG_WHEEL 1')."""
        mixed = len({g.full for g in self.by_role[group.role]}) > 1
        return f'그룹 {group.full} {group.number}' if mixed else f'그룹 {group.number}'

    def run(self):
        """모양 → 같은 크기 → 치수 · 붙는 자리 · 대칭 순으로 모두 잰다."""
        self.shape_roles()
        for role in self.rules:
            for group in self.by_role.get(role, []):
                self.shape_group(self.rules[role], group)
        self.same_size()
        for role in self.rules:
            for group in self.by_role.get(role, []):
                self.dimensions(self.rules[role], group)
                self.attach(self.rules[role], group)
        for role in self.rules:
            self.mirror(self.rules[role], role)
        return self.errors

    # ---- 모양 (TEMPLATE_SHAPE) ----
    def shape_roles(self):
        """역할 목록 · 개수(묶음 수)가 규칙과 같은지."""
        extra = sorted(set(self.by_role) - set(self.rules))
        if extra:
            self.add('TEMPLATE_SHAPE', f'이 Template 에 없는 역할이 있다: {", ".join(extra)}')
        for role, rule in self.rules.items():
            have = len(self.by_role.get(role, []))
            if have == 0:
                self.add('TEMPLATE_SHAPE', f'{rule["label"]}({role}) 부품이 없다 — Template 은 {rule["count"]}개')
                self.unusable.add(role)
            else:
                if have != rule['count']:
                    self.add('TEMPLATE_SHAPE', f'{rule["label"]} 개수 {have}개 — Template 은 {rule["count"]}개')
                oris = sorted({b[1] for g in self.by_role[role] for b in g.boxes})
                if len(oris) > 1:      # 한 역할의 부품은 모두 같은 놓는 방향 — 묶음 하나만 돌려도 모양이 달라진다
                    self.add('TEMPLATE_SHAPE', f'{rule["label"]} 부품의 놓는 방향이 모두 같지 않다({", ".join(oris)})')
                    self.unusable.add(role)

    def shape_group(self, rule, group):
        """묶음 하나의 놓는 방향 · 쌓는 모양 · 받침 · 상대 방향."""
        who, lay = self.who(group), rule['lay']
        if lay:
            want = ('zx', 'zy') if lay == 'STAND_STACK' else ('x', 'y')
            bad = [b[0] for b in group.boxes if b[1] not in want]
            if bad:
                self.add('TEMPLATE_SHAPE', f'{who} 놓는 방향이 {LAYS[lay]} 가 아니다(부품 {", ".join(map(str, sorted(bad)))}번)')
            elif not self.stacked_right(lay, group):
                self.add('TEMPLATE_SHAPE', f'{who} 부품이 {LAYS[lay]} 모양이 아니다')
                bad = True
            if bad:
                self.broken.add(group.key)
                self.unusable.add(group.role)
        if rule['support']:
            self.support(rule, group)
        if rule['long_edge']:
            self.long_edge(rule, group)

    def stacked_right(self, lay, group):
        """STACK = 부품의 위·아래 면 자리가 같고 층이 이어짐, SIDE_BY_SIDE = 모두 같은 높이의 한 층."""
        boxes = sorted(group.boxes, key=lambda b: b[2][2])
        if lay == 'FLAT_SIDE_BY_SIDE':
            return all(abs(b[2][2] - boxes[0][2][2]) <= TOL_MM and abs(b[3][2] - boxes[0][3][2]) <= TOL_MM for b in boxes)
        same_footprint = all(abs(b[2][i] - boxes[0][2][i]) <= TOL_MM and abs(b[3][i] - boxes[0][3][i]) <= TOL_MM for b in boxes for i in (0, 1))
        return same_footprint and all(abs(b[2][2] - a[3][2]) <= TOL_MM for a, b in zip(boxes, boxes[1:]))

    def support(self, rule, group):
        """각 받쳐지는 그룹이 목록 역할 전부 또는 정확히 per_group개의 서로 다른 그룹 위인지 확인한다.

        TABLE은 그룹 아랫면 z=0. 실제 부품 접촉으로 세며 같은 받침의 여러 부품을 중복 세지 않는다.
        실패는 TEMPLATE_SHAPE(kind template), 로봇/메시지 영향 없음.
        """
        spec=rule['support']
        for role in spec['roles']:
            if role=='TABLE':
                if abs(group.lo[2])<=TOL_MM:continue
                detail='작업면 위에 얹혀야 한다'
            else:
                lower=self.by_role.get(role,[])
                if not lower:continue  # 역할 누락은 shape_roles에서 이미 실패로 보고한다.
                have=sum(self.rests_on(group,h) for h in lower)
                want=len(lower) if spec['per_group'] is None else spec['per_group']
                if have==want:continue
                qualifier='전부 ' if spec['per_group'] is None else '정확히 '
                detail=f'{self.label(role)} {qualifier}{want}개 그룹 위에 얹혀야 한다(실제 {have}개)'
            self.add('TEMPLATE_SHAPE',f'{self.who(group)} 받침이 맞지 않다 — {detail}')
            self.broken.add(group.key)

    def long_edge(self, rule, group):
        """긴 변 방향: 짝 묶음의 긴 변과 직각 · 평행."""
        le = rule['long_edge']
        other = self.partner(group, le['to'])
        if other is None:
            return
        mine, theirs = self.group_axis(group), self.group_axis(other)
        who = self.who(group)
        if mine is None or theirs is None:
            self.add('TEMPLATE_SHAPE', f'{who} 와 {self.who(other)} 의 긴 변 방향을 하나로 정할 수 없다(방향이 섞였거나 변 길이가 같다)')
            return
        right = (mine != theirs) if le['relation'] == 'PERPENDICULAR' else (mine == theirs)
        if not right:
            self.add('TEMPLATE_SHAPE', f'{who} 의 긴 변은 {self.who(other)} 의 긴 변과 {RELATIONS[le["relation"]]}이어야 한다 '
                                       f'(지금 {AXIS_KO[mine]} 방향 · {AXIS_KO[theirs]} 방향)')
            self.broken.add(group.key)

    def group_axis(self, group):
        """묶음 부품들의 긴 변 축. 부품마다 다르거나 정할 수 없으면 None."""
        axes = {self.checker.long_axis(b[1]) for b in group.boxes}
        return axes.pop() if len(axes) == 1 and None not in axes else None

    # ---- 같음 · 범위 · 대칭 (TEMPLATE_RANGE) ----
    def same_size(self):
        """same_size 인 역할은 묶음끼리 같은 크기여야 한다. 가장 먼저 보고, 다르면 그 역할을 기준으로 쓰는 식은 건너뛴다."""
        for role, rule in self.rules.items():
            groups = self.by_role.get(role, [])
            if not rule['same_size'] or len(groups) < 2:
                continue
            first = groups[0]
            for g in groups[1:]:
                if any(abs(g.size(i) - first.size(i)) > TOL_MM for i in range(3)):
                    self.add('TEMPLATE_RANGE', f'{rule["label"]} 크기가 서로 다르다 — {self.tag(first)}: {self.dims_text(first)}, {self.tag(g)}: {self.dims_text(g)}')
                    self.unusable.add(role)

    @staticmethod
    def dims_text(group):
        """묶음 크기 글자 '폭 × 깊이 × 높이 mm'."""
        return f'{_n(group.size(0))} × {_n(group.size(1))} × {_n(group.size(2))} mm'

    def dimensions(self, rule, group):
        """height/width/depth_mm: [최소, 최대] 안, 또는 "= 역할.치수"(같은 크기 + 양 끝 맞춤)."""
        for dim, spec in rule['dims'].items():
            axis, who = DIM_AXIS[dim], self.who(group)
            value = group.size(axis)
            if spec[0] == 'range':
                self.in_range(group, DIM_KO[dim], value, spec[1], spec[2])
                continue
            other = self.reference(group, spec[1])
            if other is None:
                continue
            oaxis = DIM_AXIS[spec[2]]
            ref = f'{self.who(other)} {DIM_KO[spec[2]]}'
            if abs(value - other.size(oaxis)) > TOL_MM:
                self.add('TEMPLATE_RANGE', f'{who} {DIM_KO[dim]} {_n(value)} mm — {ref} {_n(other.size(oaxis))} mm 와 같아야 한다')
            elif oaxis == axis and (abs(group.lo[axis] - other.lo[axis]) > TOL_MM or abs(group.hi[axis] - other.hi[axis]) > TOL_MM):
                self.add('TEMPLATE_RANGE', f'{who} {DIM_KO[dim]} 양 끝이 {self.who(other)} 와 맞지 않다 '
                                           f'(지금 {_n(group.lo[axis])} ~ {_n(group.hi[axis])}, 기준 {_n(other.lo[axis])} ~ {_n(other.hi[axis])})')

    def attach(self, rule, group):
        """붙는 자리: 기준 역할의 끝 · 가운데 · 윗면에서 안쪽으로 떨어진 거리 d 가 d_mm 범위 안."""
        at = rule['attach']
        if not at:
            return
        other = self.reference(group, at['to'], needs_usable=False)
        if other is None:
            return
        who, where = self.who(group), at['at']
        if where == 'TOP':
            if not self.rests_on(group, other):
                self.add('TEMPLATE_SHAPE', f'{who} 은 {self.who(other)} 윗면에 얹혀야 한다')
                self.broken.add(group.key)
            return
        if where == 'X_CENTER':
            d = abs((group.lo[0] + group.hi[0]) / 2 - (other.lo[0] + other.hi[0]) / 2)
            name = f'{self.who(other)} 좌우 가운데에서'
        elif where == 'REAR_END':
            d = other.hi[1] - group.hi[1]
            name = f'{self.who(other)} 뒤 끝에서'
        else:
            axis = 0 if where == 'X_ENDS' else 1
            d = min(group.lo[axis] - other.lo[axis], other.hi[axis] - group.hi[axis])
            name = f'{self.who(other)} {"좌우" if axis == 0 else "앞뒤"} 끝에서'
        if at['d']:
            self.in_range(group, f'자리(안쪽으로 {name} 떨어진 거리)', d, at['d'][0], at['d'][1])

    def mirror(self, rule, role):
        """mirror: 이 역할 전체의 가운데를 기준으로 가구 전체(같은 역할끼리)가 X(좌우) · XY(좌우 + 앞뒤) 대칭."""
        groups = self.by_role.get(role)
        if not rule['mirror'] or not groups:
            return
        lo = [min(g.lo[i] for g in groups) for i in range(3)]
        hi = [max(g.hi[i] for g in groups) for i in range(3)]
        boxes = [(g.role, b) for g in sum(self.by_role.values(), []) for b in g.boxes]
        for axis in (0, 1)[:len(rule['mirror'])]:
            center = (lo[axis] + hi[axis]) / 2
            for r, (order, _, blo, bhi) in boxes:
                mlo, mhi = 2 * center - bhi[axis], 2 * center - blo[axis]
                if not any(r2 == r and all(abs(c[2][i] - (mlo if i == axis else blo[i])) <= TOL_MM
                                           and abs(c[3][i] - (mhi if i == axis else bhi[i])) <= TOL_MM for i in range(3)) for r2, c in boxes):
                    self.add('TEMPLATE_RANGE', f'{self.label(role)} 가운데 기준 {AXIS_KO[axis][:2]} 대칭이 아니다 — {order}번 부품({self.label(r)})의 맞은편 자리에 같은 부품이 없다')
                    break

    # ---- 도우미 ----
    def in_range(self, group, what, value, low, high):
        """value 가 [low, high](숫자 또는 식) 안인지. 식을 못 풀면 건너뛰고(이유가 있으면 알림), 벗어나면 RANGE 실패.

        what = 잰 값의 이름(예 '높이'). 식이 계산되지 않으면(0 으로 나눔 등) 설계 탓이 아니라 규칙 탓이므로 RequestInvalid 가 그대로 올라간다.
        """
        try:
            lo, hi = self.evaluate(group, low), self.evaluate(group, high)
        except Skip:
            return
        if value < lo - TOL_MM or value > hi + TOL_MM:
            self.add('TEMPLATE_RANGE', f'{self.who(group)} {what} {_n(value)} mm — 범위 {_n(lo)} ~ {_n(hi)} mm')

    def evaluate(self, group, bound):
        """범위 끝 하나의 값. 숫자는 그대로, 식은 안전하게 계산한다(역할.치수는 이 묶음과 맞닿은 그 역할 묶음에서 잰다)."""
        if isinstance(bound, float):
            return bound
        return self._eval(bound[1], group)

    def _eval(self, node, group):
        """허용된 노드만 있는 식을 계산한다(read_base 가 모양을 이미 확인했다). 0 으로 나눔 · 유한하지 않은 값은 잘못된 규칙이므로 RequestInvalid."""
        if isinstance(node, ast.Constant):
            return float(node.value)
        if isinstance(node, ast.Attribute):
            other = self.reference(group, node.value.id)
            if other is None:
                raise Skip()
            return other.size(DIM_AXIS[node.attr])
        if isinstance(node, ast.UnaryOp):
            value = self._eval(node.operand, group)
            return -value if isinstance(node.op, ast.USub) else value
        value = apply_operator(node.op, self._eval(node.left, group), self._eval(node.right, group))
        if not math.isfinite(value):
            raise RequestInvalid('범위 식의 값이 너무 크다')
        return value

    def reference(self, group, role, needs_usable=True):
        """group 이 기준으로 삼을 role 의 묶음(맞닿은 것, 그 역할 묶음이 하나뿐이면 그것). 구할 수 없으면 None.

        없는 역할 · 잴 수 없는 역할이면 앞 줄이 이미 알렸으므로 조용히 None. 묶음이 여럿인데 맞닿은 것이 없으면 한 번 알린다.
        """
        if role not in self.by_role or (needs_usable and role in self.unusable):
            return None
        return self.partner(group, role)

    def partner(self, group, role):
        """role 묶음 중 group 과 맞닿은 첫 번째(번호 순). 맞닿은 것이 없고 묶음이 하나뿐이면 그것. 없으면 None(여럿이면 한 번 알림)."""
        candidates = self.by_role.get(role, [])
        if not candidates:
            return None
        touching = [h for h in candidates if self.touches(group, h)]
        if touching:
            return touching[0]
        if len(candidates) == 1:
            return candidates[0]
        key = group.key
        if key not in self.broken:
            self.broken.add(key)
            self.add('TEMPLATE_SHAPE', f'{self.who(group)} 에 맞닿은 {self.label(role)} 이 없다')
        return None

    @staticmethod
    def overlap(a, b, axis):
        """두 상자(묶음 또는 부품 (order, ori, lo, hi))가 그 축에서 겹친 길이(음수면 떨어진 거리)."""
        a_lo, a_hi = (a.lo, a.hi) if isinstance(a, _Group) else (a[2], a[3])
        b_lo, b_hi = (b.lo, b.hi) if isinstance(b, _Group) else (b[2], b[3])
        return min(a_hi[axis], b_hi[axis]) - max(a_lo[axis], b_lo[axis])

    def touches(self, a, b):
        """두 묶음이 맞닿으면 True — 묶음 상자가 아니라 부품끼리 본다(오목한 묶음의 빈 곳에 닿은 가짜 접촉을 막는다).

        부품 한 쌍이 한 축에서 면이 닿고(오차 안) 나머지 두 축은 겹치면 접촉. 묶음 치수(폭 · 깊이 · 높이)는 그대로 묶음 상자로 잰다.
        """
        return any(self.parts_touch(p, q) for p in a.boxes for q in b.boxes)

    def parts_touch(self, p, q):
        """부품 두 개가 한 축에서 면이 닿고 나머지 두 축은 겹치면 True."""
        for axis in range(3):
            others = [i for i in range(3) if i != axis]
            face = abs(p[3][axis] - q[2][axis]) <= TOL_MM or abs(q[3][axis] - p[2][axis]) <= TOL_MM
            if face and all(self.overlap(p, q, i) > TOL_MM for i in others):
                return True
        return False

    def rests_on(self, upper, lower):
        """upper 의 어떤 부품 아랫면이 lower 의 어떤 부품 윗면에 닿고 위에서 보아 겹치면 True (부품끼리 본다)."""
        return any(abs(p[2][2] - q[3][2]) <= TOL_MM and self.overlap(p, q, 0) > TOL_MM and self.overlap(p, q, 1) > TOL_MM
                   for p in upper.boxes for q in lower.boxes)

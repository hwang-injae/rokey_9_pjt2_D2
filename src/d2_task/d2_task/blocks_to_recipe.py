"""변환기 ① — AI가 쓴 설계도(blocks/2.0)를 레시피(recipe/2.0) + 조립 방법(placements/2.0)으로 바꾼다 (W110, E-69).

검사 묶음(DesignChecker)이 설계를 통과시킨 뒤 부른다. ROS · 파일 쓰기 · 로봇 · 메시지 영향 없음(계산만).
"""
import itertools
import json
import re
from pathlib import Path

from d2_task.recipe_builder import RecipeBuilder

SCHEMA_REQUEST = 'blocks/2.0'
REQUIRED_KEYS = ('order', 'x', 'y', 'z', 'ori', 'role', 'part', 'stage', 'grasp')
ROLES_PATH = Path(__file__).with_name('roles.json')

# 역할[_옵션] — 영문 대문자 한 단어씩, 옵션 0~1개 (작명 규칙 v3 2장)
ROLE_RE = re.compile(r'([A-Z]+)(?:_([A-Z]+))?')

# 같은 높이 · 같은 면으로 볼 허용오차(mm). blocks/2.0 좌표는 JSON 숫자 그대로라 부동소수 오차만 흡수한다.
TOL = 1e-6


class DesignRejected(ValueError):
    """AI가 쓴 값이 틀려 레시피를 만들 수 없음 — 검사 묶음이 CHECK_FAILED(재생성)로 돌려줄 실수. 프로그램 고장(ERROR)과 구분한다.

    errors: [{'block': order 또는 None, 'detail': 사람이 · AI가 읽을 이유}] — 한 번에 찾은 실수를 모두 담아 재생성 횟수를 줄인다.
    """

    def __init__(self, errors):
        """errors 목록을 받아 보존한다. 메시지는 detail 을 이은 글자."""
        super().__init__('; '.join(e['detail'] for e in errors))
        self.errors = errors


class BlocksToRecipe:
    """blocks/2.0 → {'recipe': recipe/2.0, 'placements': placements/2.0}.

    AI가 고른 값(역할 · 옵션 · 부품 묶음 · 순서 · 단계 · 잡기)은 그대로 옮기고 대신 채우지 않는다(E-69 ②).
    코드가 정하는 것은 블록 번호 · 중심 · 회전 R · 받침 · 닫힘 축 · recipe_sha256 뿐이다(RecipeBuilder — CAD 도구와 같은 규칙).
    검사 묶음이 보지 않는, 바꾸는 과정에서만 드러나는 AI 실수 세 가지를 검사한다:
      ① 역할 · 옵션이 목록(roles.json)에 있나 ② 같은 부품 블록이 면으로 맞닿아 한 덩어리인가 ③ AI가 고른 잡기로 손가락이 들어가나.
    """

    def __init__(self, grasp_options, block_mm, roles_path=ROLES_PATH):
        """잡기 후보 함수 · 블록 크기 · 역할 목록을 받는다.

        입력:
            grasp_options: blocks 목록 → {order: [잡기 이름 …]} — 검사 묶음 DesignChecker.grasp_options(손가락 규칙을 한 곳에 두려고 받는다)
            block_mm: 블록 설계 기준 크기 [길이, 폭, 두께] mm — robot.yaml block_size_m × 1000
            roles_path: 역할 · 옵션 목록 JSON 파일 (기본 = 이 파일 옆 roles.json)
        실패: 목록 파일이 없거나 깨졌으면 OSError · ValueError (설정 고장 — AI 실수가 아님).
        """
        self.grasp_options = grasp_options
        L, W, T = block_mm
        # 방향 코드 → (x · y · z 방향 길이) mm. IRD 2장 ori 표 · task ori_extents 와 같은 규칙.
        self.extent = {'x': (L, W, T), 'y': (W, L, T), 'xe': (L, T, W), 'ye': (T, L, W), 'zx': (T, W, L), 'zy': (W, T, L)}
        self.roles, self.options = self.load_role_list(roles_path)
        self.builder = RecipeBuilder()

    def load_role_list(self, path):
        """역할 · 옵션 목록 파일을 읽는다.

        입력:
            path: roles.json 경로
        출력: (역할 이름 집합, 옵션 이름 집합)
        실패: 파일이 없으면 OSError, roles · options 가 이름 → 뜻 객체가 아니면 ValueError.
        """
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        roles, options = data.get('roles'), data.get('options')
        if not isinstance(roles, dict) or not isinstance(options, dict) or not roles:
            raise ValueError(f'{path}: roles · options 는 이름 → 뜻 객체여야 한다')
        return set(roles), set(options)

    def convert(self, request):
        """blocks/2.0 하나를 레시피 + 조립 방법으로 바꾼다. 입력을 바꾸지 않는다.

        입력:
            request: blocks/2.0 dict — design_id, blocks[{order, x, y, z(아랫면), ori, role, part, stage, grasp}] (mm, 설계 좌표계)
        출력: {'recipe': recipe/2.0(model_id = design_id, source_cad = None), 'placements': placements/2.0}
        실패: AI가 쓴 값이 틀리면 DesignRejected(errors — block = order). grasp_options 자체의 예외는 그대로 올린다(프로그램 고장).
        """
        # 형식: 검사 묶음이 먼저 보지만, 따로 불려도 빈 칸을 대신 채우지 않게 다시 확인한다
        if not isinstance(request, dict) or request.get('schema') != SCHEMA_REQUEST or not request.get('design_id'):
            raise DesignRejected([{'block': None, 'detail': f'schema {SCHEMA_REQUEST} · design_id 가 있는 객체가 아니다'}])
        items = request.get('blocks')
        if not isinstance(items, list) or not items or not all(isinstance(b, dict) for b in items):
            raise DesignRejected([{'block': None, 'detail': 'blocks 가 비어 있거나 객체 목록이 아니다'}])
        missing = [{'block': b.get('order'), 'detail': f'{b.get("order")}번 블록에 {", ".join(k for k in REQUIRED_KEYS if k not in b)} 없음'}
                   for b in items if any(k not in b for k in REQUIRED_KEYS)]
        if missing:
            raise DesignRejected(missing)
        items = sorted(items, key=lambda b: b['order'])
        if [b['order'] for b in items] != list(range(1, len(items) + 1)):
            raise DesignRejected([{'block': None, 'detail': f'order 가 1부터 {len(items)}까지 하나씩 있지 않다'}])
        bad = [{'block': b['order'], 'detail': f'{b["order"]}번 ori · role · part · stage 가 형식에 맞지 않다 '
                                               f'(ori 6가지 · role 글자 · part · stage 1 이상 정수)'}
               for b in items if b['ori'] not in self.extent or not isinstance(b['role'], str)
               or any(not isinstance(b[k], int) or isinstance(b[k], bool) or b[k] < 1 for k in ('part', 'stage'))]
        if bad:
            raise DesignRejected(bad)


        # AI 실수 검사 ① 역할 목록 ② 부품 이어짐 ③ 고른 잡기 — 셋 다 모아 한 번에 돌려준다
        boxes = {b['order']: self.calculate_box(b) for b in items}
        errors = self.check_role_names(items) + self.check_parts_connected(items, boxes) + self.check_chosen_grasps(items)
        if errors:
            raise DesignRejected(errors)


        # 블록 이름 붙이기 → 꼭짓점 8개 → 레시피 · 조립 방법 (계산 · 받침 · 단계 규칙 검사는 CAD 도구와 같은 RecipeBuilder)
        names = self.make_block_names(items, boxes)
        vertices, hints = [], {}
        for b in items:
            lo, hi = boxes[b['order']]
            corners = [tuple((lo, hi)[s][k] for k, s in enumerate(pick)) for pick in itertools.product((0, 1), repeat=3)]
            vertices.append({'block': names[b['order']], 'handle': None, 'vertices': corners})
            hints[names[b['order']]] = {'SEQ': b['order'], 'STAGE': b['stage'], 'GRASP': b['grasp']}
        try:
            recipe = self.builder.make_recipe(vertices, request['design_id'], None, None)
            placements = self.builder.make_placements(recipe, hints)
        except ValueError as e:
            # 여기까지 오는 것은 단계 규칙 · 뜬 블록 · 겹침처럼 검사 묶음도 보는 AI 실수다 — 따로 불렸을 때도 CHECK_FAILED 로
            raise DesignRejected([{'block': None, 'detail': f'레시피 검사: {e}'}]) from e
        return {'recipe': recipe, 'placements': placements}

    def calculate_box(self, block):
        """블록 하나가 차지하는 축 정렬 상자.

        입력:
            block: blocks/2.0 블록 (x · y = 중심, z = 아랫면, mm)
        출력: (lo, hi) — 각각 (x, y, z) 튜플 mm
        """
        dx, dy, dz = self.extent[block['ori']]
        lo = (block['x'] - dx / 2, block['y'] - dy / 2, block['z'])
        hi = (block['x'] + dx / 2, block['y'] + dy / 2, block['z'] + dz)
        return lo, hi

    def check_role_names(self, items):
        """① 역할 · 옵션이 목록에 있는지 본다. 꼴(대문자 한 단어 + 옵션 0~1)도 다시 본다.

        입력:
            items: order 순 블록 목록
        출력: errors 목록(없으면 빈 목록). 같은 이름은 한 번만 적는다.
        """
        errors, seen = [], set()
        for b in items:
            role = b['role']
            if role in seen:
                continue
            seen.add(role)
            match = ROLE_RE.fullmatch(role) if isinstance(role, str) else None
            if not match:
                errors.append({'block': b['order'], 'detail': f'{b["order"]}번 role={role!r} 는 영문 대문자 역할 한 단어 + 옵션 0~1개(예 LEG · LEG_WHEEL)가 아니다'})
                continue
            name, option = match.groups()
            if name not in self.roles:
                errors.append({'block': b['order'], 'detail': f'{b["order"]}번 역할 {name} 는 역할 목록에 없다(있는 것: {", ".join(sorted(self.roles))}) — '
                                                         f'맞는 것을 쓰거나, 새 역할이면 뜻과 함께 목록에 더해야 한다'})
            if option is not None and option not in self.options:
                errors.append({'block': b['order'], 'detail': f'{b["order"]}번 옵션 {option} 는 옵션 목록에 없다(있는 것: {", ".join(sorted(self.options))})'})
        return errors

    def check_parts_connected(self, items, boxes):
        """② 같은 부품(역할 + part 값)의 블록이 면으로 맞닿아 한 덩어리인지 본다.

        맞닿음 = 한 축에서 면이 붙고(간격 0) 나머지 두 축에서 넓이가 0보다 크게 겹침 — 모서리 · 꼭짓점만 닿는 것은 아님(10/8 한세교:
        젠가는 면으로 닿아야 버틴다). 겹치는 넓이가 버틸 만큼인지는 검사 묶음의 안정성 검사가 본다.

        입력:
            items: order 순 블록 목록
            boxes: {order: (lo, hi)}
        출력: errors 목록 — 떨어진 덩어리마다 블록 번호를 적는다
        """
        errors = []
        for (role, part), members in self.group_parts(items).items():
            # 첫 블록부터 맞닿은 블록을 따라가며 닿는 블록을 모은다
            reached, frontier = {members[0]}, [members[0]]
            while frontier:
                current = frontier.pop()
                for other in members:
                    if other not in reached and self.is_face_contact(boxes[current], boxes[other]):
                        reached.add(other)
                        frontier.append(other)
            apart = [o for o in members if o not in reached]
            if apart:
                errors.append({'block': apart[0], 'detail': f'{role} part {part} 의 블록 {", ".join(map(str, sorted(reached)))}번과 '
                                                            f'{", ".join(map(str, apart))}번이 면으로 맞닿아 있지 않다 — 떨어진 블록은 다른 part 값으로'})
        return errors

    def is_face_contact(self, a, b):
        """두 상자가 면으로 맞닿았는지 — 한 축은 간격 0, 나머지 두 축은 겹친 길이 > 0.

        입력:
            a, b: (lo, hi) 상자 mm
        출력: True / False
        """
        overlap = [min(a[1][k], b[1][k]) - max(a[0][k], b[0][k]) for k in range(3)]
        return any(abs(overlap[k]) <= TOL and all(overlap[j] > TOL for j in range(3) if j != k) for k in range(3))

    def check_chosen_grasps(self, items):
        """③ AI가 고른 잡기가, 놓는 순간 손가락이 들어가는 후보(grasp_options)에 있는지 본다. 놓인 자세와 다른 잡기도 여기서 걸린다.

        입력:
            items: order 순 블록 목록
        출력: errors 목록 — 가능한 잡기를 이유에 적어 AI가 바로 고치게 한다
        """
        options = self.grasp_options(items)
        errors = []
        for b in items:
            allowed = options.get(b['order']) or []
            if b['grasp'] not in allowed:
                hint = f'가능: {", ".join(allowed)}' if allowed else '이 순서로는 잡을 수 있는 방향이 없다 — 순서를 바꿀 것'
                errors.append({'block': b['order'], 'detail': f'{b["order"]}번 grasp {b["grasp"]} 로는 놓는 순간 손가락이 들어가지 않는다({hint})'})
        return errors

    def group_parts(self, items):
        """블록을 부품(역할[_옵션], part 값)별로 모은다.

        입력:
            items: order 순 블록 목록
        출력: {(역할, part 값): [order …]} — order 순
        """
        groups = {}
        for b in items:
            groups.setdefault((b['role'], b['part']), []).append(b['order'])
        return groups

    def make_block_names(self, items, boxes):
        """블록 이름을 매긴다 — 작명 규칙 v3: 역할[_옵션]마다 부품 번호(첫 블록 위치 앞 −y → 뒤, 왼 −x → 오른),
        부품 안 블록 번호(아래층부터, 같은 층은 앞 → 뒤, 왼 → 오른). AI의 part 값은 묶음 표시일 뿐 번호로 쓰지 않는다.

        RecipeBuilder.check_block_numbering 과 같은 위치 키(0.001 mm 반올림)를 써서 그 검사를 그대로 통과한다.

        입력:
            items: order 순 블록 목록
            boxes: {order: (lo, hi)}
        출력: {order: 블록 이름}
        """
        # 부품 안 블록 순서: (바닥 z, 중심 y, 중심 x)
        def position(order):
            lo, hi = boxes[order]
            return (round(lo[2], 3), round((lo[1] + hi[1]) / 2, 3), round((lo[0] + hi[0]) / 2, 3))

        parts_by_role = {}
        for (role, _), members in self.group_parts(items).items():
            ordered = sorted(members, key=position)
            parts_by_role.setdefault(role, []).append(ordered)


        # 역할마다 부품 순서: 첫 블록의 (y, x), 같으면 첫 블록 높이 → 번호 1부터
        names = {}
        for role, parts in parts_by_role.items():
            parts.sort(key=lambda ordered: (position(ordered[0])[1:], position(ordered[0])[0]))
            for part_no, ordered in enumerate(parts, 1):
                for block_no, order in enumerate(ordered, 1):
                    names[order] = f'{role}_{part_no:03d}_{block_no:02d}'
        return names

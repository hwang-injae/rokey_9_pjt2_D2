# -*- coding: utf-8 -*-
"""검사 묶음 DesignChecker — 블록 JSON(blocks/1)이 쌓을 만한 설계인지 본다 (ROS 없이 동작, W109, SDD 6.7).

`/d2/task/check_design`(JsonQuery) 서비스 뒤에서 task 노드가 부른다. ROS 없이 시험할 수 있게 노드와 나눴다(팀 규칙 ②).
10/3 jenga_check.py(안정성 계산)를 가져와 내민 구조 버그를 고치고, 받침 · 손가락 틈 · 작업영역을 더했다.
단위: 블록 JSON · 이 파일 안은 mm(설계 좌표계 — 바닥 외곽 가운데 = (0,0), z = 블록 아랫면 높이, 작업면 z = 0).
"""
import json
import math

from d2_task.recipe_to_blocks import ori_extents
from d2_task.recipe_document import RecipeDocument

SCHEMA_REQUEST = 'blocks/1'       # check_design 요청 · 변환기 ①이 받는 형식 (IRD 6장)
SCHEMA_RECIPE = 'cad_recipe/1.0'  # 변환기 ①이 돌려줘야 하는 레시피 형식 이름 (E-44 — 레시피 안쪽 전체 검증은 W110 계약)
PENETRATION_MM = 0.1     # 세 방향 모두 이 값보다 깊게 겹치면 파고듦. 면이 닿기만 하면 허용 (SDD 6.7 검사 2)
FLAT, EDGE, STAND = 'FLAT', 'EDGE', 'STAND'
# 방향 코드 → 위를 향한 면에 따른 잡기 종류 (IRD 2장: 눕힘 = FLAT, 옆세움 = EDGE, 세움 = STAND)
STATE_OF_ORI = {'x': FLAT, 'y': FLAT, 'xe': EDGE, 'ye': EDGE, 'zx': STAND, 'zy': STAND}


class DesignChecker:
    """블록 JSON 하나를 형식 → 파고듦 → 받침 → 안정성 → 잡기 틈 → 작업영역 순으로 검사해 check_result/1 을 만든다.

    입력: robot.yaml 을 읽은 dict(cfg — block_size_m · finger · grasp_depth_m · assembly_origin ·
    assembly_area_half_m · table · check.margin_mm · check.max_blocks), blocks/1 dict. 변환기 ①(blocks_to_recipe)은 선택으로 받는다.
    바깥 영향: 없음(계산만). 로봇 · 메시지를 건드리지 않는다.
    실패: 설계가 나쁘면 예외 없이 ok=false + errors 로 돌려준다(HMI 가 detail 을 LLM 에 되돌려 줌). cfg 에 키가 없으면 만들 때 ValueError.
    """

    def __init__(self, cfg, blocks_to_recipe=None):
        """cfg 를 mm 로 바꾼다. 변환기 ①은 옛 recipe dict 또는 {structure, recipe} 를 반환한다. 미연결이면 레시피를 안 낸다."""
        try:
            self.block_mm = [v * 1000.0 for v in cfg['block_size_m']]
            self.extent = ori_extents(self.block_mm)
            self.finger_t = cfg['finger']['thickness_m'] * 1000.0
            self.finger_w = cfg['finger']['width_m'] * 1000.0
            self.grasp_depth = cfg['grasp_depth_m'] * 1000.0
            self.origin = cfg['assembly_origin']
            self.area_half = cfg['assembly_area_half_m']
            self.table = cfg['table']
            self.margin_mm = cfg['check']['margin_mm']
            self.max_blocks = cfg['check']['max_blocks']
        except KeyError as e:
            raise ValueError(f'robot.yaml 에 {e.args[0]} 가 없다') from e
        self.blocks_to_recipe = blocks_to_recipe

    # ---------------- 바깥에서 부르는 곳 ----------------
    def check(self, request):
        """blocks/1 요청 → check_result/1. ok · min_margin_mm(쌓는 도중 최소 여유, 못 구하면 None) · errors[block · reason · detail] · recipe(ok 일 때).

        검사는 순서대로 하되, 형식이 틀리면 바로 돌려주고(뒤 검사가 읽을 수 없음), 받침이 없는 블록이 있으면 안정성은 건너뛴다.
        reason: OUT_OF_SCOPE(블록 수 초과) · CHECK_FAILED(나머지) · ERROR(변환기 ① 실패).
        """
        errors = []
        blocks = self._parse(request, errors)
        if errors:
            return self._result(False, None, errors)
        boxes = [self._box(b) for b in blocks]
        errors += self._penetration(boxes)
        below = self._supports(boxes, errors)
        margin = None
        if below is not None:
            margin = self._stability(boxes, below, errors)
        errors += self._grasp_errors(boxes)
        errors += self._workspace(boxes)
        if errors:
            return self._result(False, margin, errors)
        result = self._result(True, margin, [])
        if self.blocks_to_recipe is not None:
            try:
                converted = self.blocks_to_recipe(request)
                if isinstance(converted, dict) and 'recipe' in converted:
                    document = RecipeDocument(converted['recipe'], converted.get('structure'))
                    if document.structure is None:
                        raise ValueError('두 파일 변환 결과에 structure 가 없다')
                    result.update(structure=document.structure, recipe=document.recipe)
                else:
                    if not isinstance(converted, dict) or converted.get('schema') != SCHEMA_RECIPE:
                        raise ValueError(f'변환기 ① 결과가 {SCHEMA_RECIPE} 객체가 아니다')
                    if 'model_id' in converted and 'model' not in converted:
                        raise ValueError('새 recipe 에는 structure 가 필요하다')
                    # 출력 전환 전의 한 파일 계약도 유지한다. 새 두 파일은 위에서 참조 관계까지 검증한다.
                    result['recipe'] = converted
            except Exception as e:   # 변환기는 다른 파트 코드라 어떤 예외든 ERROR 로 알린다
                return self._result(False, margin, [{'block': None, 'reason': 'ERROR', 'detail': f'변환기 ① 실패: {e}'}])
        return result

    def handle_json(self, request_json):
        """`check_design`(JsonQuery) 한 번을 처리한다. 반환: (success, reason, response_json 글자).

        요청은 blocks/1 객체 글자 그대로(schema 가 'blocks/1' 이어야 한다). 서비스가 처리했나(success)와 설계가 합격인가(응답 안 ok)를 나눈다:
        설계 불합격은 (True, '', ok:false + errors) — 좌표가 유한한 수가 아닌 것(NaN · Infinity · 1e999 처럼 읽으면 무한대가 되는 수)은 불합격이 아니라 요청 오류다.
        JSON 이 깨졌거나 schema 가 다르거나 안쪽 예외거나 변환기 ①이 실패 · 잘못된 결과를 내면 (False, 'ERROR', 이유).
        변환기 ①이 안 붙은 동안은 검사에 통과해도 레시피가 없어 완성된 합격 응답이 아니므로 (False, 'ERROR') 로 답한다.
        바깥 영향: 없음(계산만). 시간 제한 5초는 부르는 쪽이 건다 — 이 함수는 계산을 중간에 끊지 못한다.
        """
        try:
            request = json.loads(request_json, parse_constant=self._reject_constant, parse_float=self._finite_float)
            if not isinstance(request, dict) or request.get('schema') != SCHEMA_REQUEST:
                raise ValueError(f'요청이 schema {SCHEMA_REQUEST} 객체가 아니다')
            result = self.check(request)
            if result['ok'] and self.blocks_to_recipe is None:
                result = self._result(False, result['min_margin_mm'], [{'block': None, 'reason': 'ERROR',
                                      'detail': '변환기 ① 미연결 — 검사는 통과했지만 레시피를 만들 수 없다'}])
            failed = any(e['reason'] == 'ERROR' for e in result['errors'])   # ERROR 는 변환기 ① 쪽 실패뿐 — 설계 탓이 아니다
            return (not failed), ('ERROR' if failed else ''), json.dumps(result, ensure_ascii=False, allow_nan=False)
        except Exception as e:   # 어떤 예외든 서비스 실패(ERROR)로 알린다 — 검사 때문에 task 노드가 죽으면 안 된다
            err = self._result(False, None, [{'block': None, 'reason': 'ERROR', 'detail': f'검사 서비스 처리 실패: {e}'}])
            return False, 'ERROR', json.dumps(err, ensure_ascii=False, allow_nan=False)

    @staticmethod
    def _reject_constant(name):
        """json.loads 가 NaN · Infinity · -Infinity 를 만나면 불러서 요청 오류로 만든다(표준 JSON 이 아니다)."""
        raise ValueError(f'JSON 에 유한하지 않은 수({name})가 있다')

    @staticmethod
    def _finite_float(text):
        """json.loads 가 소수 · 지수 글자(예: 1e999)를 읽을 때 불러서 float 로 바꾼다. 읽은 값이 Infinity 면 요청 오류."""
        value = float(text)
        if not math.isfinite(value):
            raise ValueError(f'JSON 에 유한하지 않은 수({text})가 있다')
        return value

    def grasp_options(self, blocks):
        """놓는 시점(앞 블록들만 놓인 상태)에 손가락이 들어가는 잡기 후보. {order: [잡기 이름…]}.

        blocks: 형식이 맞는 blocks/1 의 blocks 목록(order 순). 변환기 ① · 집을 블록 고르기(W130)가 같은 규칙을 쓰려고 연다.
        """
        boxes = [self._box(b) for b in sorted(blocks, key=lambda b: b['order'])]
        return {bx['order']: self._grasps(bx, boxes[:i]) for i, bx in enumerate(boxes)}

    # ---------------- 1. 형식 ----------------
    def _parse(self, request, errors):
        """형식을 확인해 order 순 블록 목록을 돌려준다. 틀린 곳은 errors 에 쌓는다."""
        def bad(detail, block=None, reason='CHECK_FAILED'):
            errors.append({'block': block, 'reason': reason, 'detail': f'형식: {detail}'})

        items = request.get('blocks') if isinstance(request, dict) else None
        if not isinstance(items, list) or not items:
            bad('blocks 가 비어 있거나 목록이 아니다')
            return []
        if len(items) > self.max_blocks:
            bad(f'블록 {len(items)}개 > 상한 {self.max_blocks}개', reason='OUT_OF_SCOPE')
            return []
        for k, it in enumerate(items, 1):
            if not isinstance(it, dict):
                bad(f'{k}번째 항목이 객체가 아니다')
                continue
            miss = [key for key in ('order', 'x', 'y', 'z', 'ori') if key not in it]
            if miss:
                bad(f'{k}번째 항목에 {", ".join(miss)} 없음')
                continue
            for key in ('x', 'y', 'z'):
                if not self._num(it[key]):
                    bad(f'{k}번째 항목의 {key} 가 숫자가 아니다')
            if it['ori'] not in self.extent:
                bad(f'{k}번째 항목 ori={it["ori"]!r} 는 x, y, xe, ye, zx, zy 가 아니다')
        if errors:
            return []
        orders = [it['order'] for it in items]
        if any(not isinstance(o, int) or isinstance(o, bool) for o in orders) or sorted(orders) != list(range(1, len(items) + 1)):
            bad(f'order 가 1부터 {len(items)}까지 하나씩 있지 않다')
            return []
        return sorted(items, key=lambda it: it['order'])

    @staticmethod
    def _num(v):
        """bool 은 숫자로 안 친다(JSON true 가 1 로 읽히는 것을 막는다)."""
        return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)

    def _box(self, b):
        """블록 하나 → 상자 정보. x0 x1 y0 y1 z0 z1 은 mm, i 는 0부터 센 순서."""
        dx, dy, dz = self.extent[b['ori']]
        return {'order': b['order'], 'ori': b['ori'], 'x': b['x'], 'y': b['y'], 'dx': dx, 'dy': dy, 'dz': dz,
                'x0': b['x'] - dx / 2, 'x1': b['x'] + dx / 2, 'y0': b['y'] - dy / 2, 'y1': b['y'] + dy / 2,
                'z0': b['z'], 'z1': b['z'] + dz}

    @staticmethod
    def _gap(a0, a1, b0, b1):
        """두 구간이 겹친 길이(음수면 떨어진 거리)."""
        return min(a1, b1) - max(a0, b0)

    # ---------------- 2. 파고듦 ----------------
    def _penetration(self, boxes):
        """세 방향 모두 PENETRATION_MM 보다 깊게 겹친 쌍을 errors 로."""
        errs = []
        for i, a in enumerate(boxes):
            for b in boxes[i + 1:]:
                if (self._gap(a['x0'], a['x1'], b['x0'], b['x1']) > PENETRATION_MM
                        and self._gap(a['y0'], a['y1'], b['y0'], b['y1']) > PENETRATION_MM
                        and self._gap(a['z0'], a['z1'], b['z0'], b['z1']) > PENETRATION_MM):
                    errs.append({'block': b['order'], 'reason': 'CHECK_FAILED',
                                 'detail': f'{a["order"]}번 ↔ {b["order"]}번 겹침'})
        return errs

    # ---------------- 3. 받침 ----------------
    def _supports(self, boxes, errors):
        """블록마다 받침 목록을 만든다. below[i] = [(받침 index 또는 None(작업면), 닿은 면 (x0,x1,y0,y1)), …].

        받침 = 아랫면 높이가 작업면(z=0) 또는 앞 순서 블록의 윗면과 정확히 같고 면이 겹치는 것. 하나도 없으면 공중(errors 에 쌓고 None 반환).
        높이는 '정확히 같음'을 == 로 본다 — 생성 · 변환 값이 JSON 숫자 그대로라 오차가 생기지 않는다.
        """
        below, floating = [], False
        for i, a in enumerate(boxes):
            sup = []
            if a['z0'] == 0:
                sup.append((None, (a['x0'], a['x1'], a['y0'], a['y1'])))
            for j in range(i):
                b = boxes[j]
                if b['z1'] == a['z0']:
                    x0, x1 = max(a['x0'], b['x0']), min(a['x1'], b['x1'])
                    y0, y1 = max(a['y0'], b['y0']), min(a['y1'], b['y1'])
                    if x1 > x0 and y1 > y0:
                        sup.append((j, (x0, x1, y0, y1)))
            if a['z0'] < 0:
                errors.append({'block': a['order'], 'reason': 'CHECK_FAILED', 'detail': f'{a["order"]}번 작업면 아래(z<0)'})
                floating = True
            elif not sup:
                later = [b['order'] for b in boxes[i + 1:] if b['z1'] == a['z0'] and
                         self._gap(a['x0'], a['x1'], b['x0'], b['x1']) > 0 and self._gap(a['y0'], a['y1'], b['y0'], b['y1']) > 0]
                hint = f' (받침 {later[0]}번이 더 늦게 놓임)' if later else ''
                errors.append({'block': a['order'], 'reason': 'CHECK_FAILED', 'detail': f'{a["order"]}번 공중{hint}'})
                floating = True
            below.append(sup)
        return None if floating else below

    # ---------------- 4. 안정성 ----------------
    def _stability(self, boxes, below, errors):
        """order 순서로 하나씩 놓을 때마다, 블록 b 와 그 위에 얹힌 덩어리 U(b) 의 무게중심이 U 가 바깥(받침 · 작업면)에 닿은 면들의
        볼록 껍질 안쪽으로 margin_mm 이상 들어와 있는지 본다. 가장 작은 여유(mm)를 돌려주고, 모자란 블록은 errors 에 쌓는다.

        10/3 jenga_check 는 위 블록의 하중을 받침 접촉면 중심에 걸어서, 한쪽으로 내민 구조를 실제보다 안전하게 판정했다
        (20 mm 씩 내민 계단 4개가 통과). 여기서는 U(b) 전체의 무게중심을 쓴다 — 블록은 모두 같은 재질이라 질량 = 부피.
        """
        n = len(boxes)
        above = [[] for _ in range(n)]
        for i, sup in enumerate(below):
            for j, _ in sup:
                if j is not None:
                    above[j].append(i)
        vol = [b['dx'] * b['dy'] * b['dz'] for b in boxes]
        worst = {}
        overall = float('inf')
        for k in range(n):                       # k 번째까지 놓은 상태
            for b in range(k + 1):
                part = self._part_above(b, k, above)
                mass = sum(vol[m] for m in part)
                com = (sum(vol[m] * boxes[m]['x'] for m in part) / mass, sum(vol[m] * boxes[m]['y'] for m in part) / mass)
                pts = []
                for m in part:
                    for j, (x0, x1, y0, y1) in below[m]:
                        if j is None or j not in part:
                            pts += [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                mg = self._margin_in_hull(com, self._hull(pts))
                overall = min(overall, mg)
                if mg < self.margin_mm and (b not in worst or mg < worst[b][0]):
                    worst[b] = (mg, k)
        for b, (mg, k) in sorted(worst.items()):
            errors.append({'block': boxes[b]['order'], 'reason': 'CHECK_FAILED',
                           'detail': f'{boxes[b]["order"]}번 위 덩어리: {boxes[k]["order"]}번까지 놓았을 때 여유 {mg:.1f} mm < {self.margin_mm}'})
        return overall

    @staticmethod
    def _part_above(b, k, above):
        """블록 b 와, b 위에 (k 번째까지 놓인 것 중) 차례로 얹힌 블록 전부의 index 집합."""
        seen, todo = {b}, [b]
        while todo:
            for i in above[todo.pop()]:
                if i <= k and i not in seen:
                    seen.add(i)
                    todo.append(i)
        return seen

    @staticmethod
    def _hull(pts):
        """점들의 볼록 껍질(반시계). jenga_check.hull 과 같다."""
        pts = sorted(set(pts))
        if len(pts) < 3:
            return pts

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        lo, up = [], []
        for p in pts:
            while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
                lo.pop()
            lo.append(p)
        for p in reversed(pts):
            while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
                up.pop()
            up.append(p)
        return lo[:-1] + up[:-1]

    @staticmethod
    def _margin_in_hull(p, poly):
        """점이 볼록 다각형 안이면 가장 가까운 변까지 거리(+), 밖이면 음수. jenga_check.margin_in_hull 과 같다."""
        best = float('inf')
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            ex, ey = b[0] - a[0], b[1] - a[1]
            best = min(best, (ex * (p[1] - a[1]) - ey * (p[0] - a[0])) / math.hypot(ex, ey))
        return best

    # ---------------- 5. 잡기 틈 · 막힌 칸 ----------------
    def _grasps(self, a, earlier):
        """블록 a 를 놓는 시점(earlier 만 놓인 상태)에 두 손가락이 다 들어가는 잡기 이름 목록.

        잡기 후보는 위를 향한 면이 정하는 종류(FLAT · EDGE · STAND)에 닫는 방향 둘(수평 x · y)이다. 손가락은 닫는 방향 양쪽 면 바깥에
        두께 finger.thickness · 폭 finger.width(블록 가운데 기준)로 서고, 끝이 블록 윗면 - grasp_depth 에 닿으므로 그 높이 위쪽 기둥이 비어야 한다.
        """
        state = STATE_OF_ORI[a['ori']]
        out = []
        for axis in ('x', 'y'):
            close, other = (a['dx'], a['dy']) if axis == 'x' else (a['dy'], a['dx'])
            name = f'{state}_{"LONG" if close > other else "SHORT"}'
            if self._fingers_free(a, axis, earlier):
                out.append(name)
        return out

    def _fingers_free(self, a, axis, earlier):
        """닫는 방향 axis 로 잡을 때 두 손가락 자리에 앞 블록이 없으면 True."""
        tip = a['z1'] - self.grasp_depth
        for side in (-1, 1):
            if axis == 'x':
                edge = a['x'] + side * a['dx'] / 2
                slab = (min(edge, edge + side * self.finger_t), max(edge, edge + side * self.finger_t),
                        a['y'] - self.finger_w / 2, a['y'] + self.finger_w / 2)
            else:
                edge = a['y'] + side * a['dy'] / 2
                slab = (a['x'] - self.finger_w / 2, a['x'] + self.finger_w / 2,
                        min(edge, edge + side * self.finger_t), max(edge, edge + side * self.finger_t))
            for b in earlier:
                if (self._gap(slab[0], slab[1], b['x0'], b['x1']) > 0 and self._gap(slab[2], slab[3], b['y0'], b['y1']) > 0
                        and b['z1'] > tip):
                    return False
        return True

    def _grasp_errors(self, boxes):
        """잡을 수 있는 후보가 하나도 없는 블록(옛 '양쪽 막힌 칸')을 errors 로."""
        errs = []
        for i, a in enumerate(boxes):
            if not self._grasps(a, boxes[:i]):
                errs.append({'block': a['order'], 'reason': 'CHECK_FAILED',
                             'detail': f'{a["order"]}번 잡을 면 없음: 놓는 시점에 두 손가락이 다 들어가는 방향이 없다'})
        return errs

    # ---------------- 6. 작업영역 ----------------
    def _workspace(self, boxes):
        """설계 바깥 상자를 조립 원점(위치 + yaw)에 놓은 모서리 전부가 조립 작업공간(원점 ± assembly_area_half_m)과 table 범위 안인지 본다."""
        o = self.origin
        yaw = math.radians(o['yaw_deg'])
        c, s = math.cos(yaw), math.sin(yaw)
        half = self.area_half
        errs = []
        for a in boxes:
            for x, y in ((a['x0'], a['y0']), (a['x1'], a['y0']), (a['x1'], a['y1']), (a['x0'], a['y1'])):
                bx = o['x_m'] + (c * x - s * y) / 1000.0
                by = o['y_m'] + (s * x + c * y) / 1000.0
                if abs(bx - o['x_m']) > half or abs(by - o['y_m']) > half:
                    errs.append({'block': a['order'], 'reason': 'CHECK_FAILED', 'detail': f'{a["order"]}번 작업영역 밖(조립 작업공간 ± {half} m)'})
                    break
                if not (self.table['x_m'][0] <= bx <= self.table['x_m'][1] and self.table['y_m'][0] <= by <= self.table['y_m'][1]):
                    errs.append({'block': a['order'], 'reason': 'CHECK_FAILED', 'detail': f'{a["order"]}번 작업대 밖'})
                    break
        return errs

    @staticmethod
    def _result(ok, margin, errors):
        """check_result/1 dict."""
        return {'schema': 'check_result/1', 'ok': ok, 'min_margin_mm': margin, 'errors': errors}

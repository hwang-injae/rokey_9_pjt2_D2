# -*- coding: utf-8 -*-
"""로봇 조립 가능 검사(W201) — 완성 배치를 로봇(옆에서 집는 두 손가락 그리퍼)이 블록 하나씩 더해 끝까지 지을 순서가 있는지 찾는다(ROS 없이 동작).

사람은 지어도 로봇은 못 짓는 배치(같은 층 2 × 2 — 어떤 순서로 놓아도 마지막 블록은 끝 · 옆이 다 막혀 손가락이 못 들어감)를
위치 · 방향만으로 가려 낸다. 스캔 설계 · 생성 설계 모두에 쓴다(10/14 PL ③). 검사 프로그램(DesignChecker)이 부른다(연결 = PM).
규칙(10/14 PL ⑥ · E-121):
  R0 층 규칙     밑면 높이가 같은 블록(한 층)은 방향 코드(ori — 자세 · 방향)가 모두 같다. 구조 전체에 한 번, 순서 찾기 전
  R1 받침 순서   바로 아래 닿는 블록이 모두 먼저 놓여 있다. 작업면 위는 늘 된다
  R2 닿는 면     아래 블록과 닿는 면은 각각 짧은 변이 MIN_BEARING_MM 이상
  R3 안정성      검사 프로그램 4번 그대로 — 블록 b 와 그 위에 얹힌 덩어리의 무게중심이 바깥 접촉면 볼록 껍질 안쪽 check.margin_mm 이상
  R5 손가락 자리  검사 프로그램 5번 그대로(_grasps · _fingers_free — robot.yaml finger · grasp_depth_m)
R3 · R5 는 판정이 두 곳에서 갈리지 않게 검사 프로그램의 계산을 그대로 불러 쓴다(E-121 ④) — 그래서 DesignChecker 객체를 받는다
(이 파일이 design_checker 를 import 하면 검사 프로그램이 이 파일을 import 할 때 서로 물린다).
단위 mm, 설계 좌표계(작업면 z = 0). 기울기는 보지 않고, 그리퍼는 손가락 두 개만 본다(손바닥 · 손목 · 팔 충돌은 안 봄).
"""
import time
from functools import lru_cache

MIN_BEARING_MM = 15.0      # R2 닿는 면 최소 폭(10/14 PL — 25 mm 면 기본 설계 003 세운 다리 받침 면 25 × 15 가 떨어진다)
GRID_XY_MM = 2.5           # 스캔 격자 맞춤(판정에만): 블록 중심 x · y 는 젠가 반 치수(7.5 · 12.5 · 37.5)의 공약수 2.5 mm 칸에 놓인다
GRID_Z_MM = 5.0            # 밑면 높이는 블록 높이(15 · 25 · 75)의 합이라 5 mm 칸
MAX_STATES = 200000        # 한 층 순서 찾기에서 따지는 '놓인 묶음' 수 상한. 넘으면 '다 못 따짐' 실패(아래 시간 상한과 같은 처리)
TIME_LIMIT_S = 1.5         # 한 층 순서 찾기 시간 상한(time.monotonic) — 웹이 check_design 답을 기다리는 5초(robot.yaml mqtt.req_timeout_s) 안에
                           # 검사 전체가 끝나게. 묶음 상한만으로는 최악 약 32초(10/15 한석형 리뷰). 넘으면 '다 못 따짐' R3 실패(10/15 PL '가')
EPS = 1e-6                 # 같은 높이 · 닿음으로 볼 오차(검사 프로그램 PENETRATION_MM 과 같은 기준)
POSE_KO = {'x': '눕힘 · 긴 변 좌우', 'y': '눕힘 · 긴 변 앞뒤', 'xe': '옆세움 · 긴 변 좌우', 'ye': '옆세움 · 긴 변 앞뒤',
           'zx': '세움 · 넓은 면 앞뒤', 'zy': '세움 · 넓은 면 좌우'}


class AssemblyFeasibility:
    """parts(sequence · x · y · z · ori) → 로봇이 R0 ~ R5 를 지키며 다 놓는 순서가 있나. AI 순서(sequence)를 먼저 보고, 안 되면 다른 순서를 찾는다.

    입력: checker = DesignChecker 객체(_box · _grasps · _fingers_free · _hull · _margin_in_hull · margin_mm 을 그대로 쓴다).
    바깥 영향: 없음(계산만, 입력을 바꾸지 않는다).
    실패: 설계가 안 되면 예외 없이 ok False + failures. parts 모양이 틀리면(키 없음) KeyError — 검사 프로그램 형식 검사(_parse) 뒤에 부른다.
    전제: 아래층(밑면이 낮은 층)을 다 놓은 뒤 위층을 놓는다 — 다른 순서를 찾을 때만. AI 순서는 그대로 따라 본다.
    """

    def __init__(self, checker):
        """checker: DesignChecker 객체 — 블록 크기 · 손가락 · margin_mm 은 모두 그 객체(robot.yaml)의 값이다."""
        self.checker = checker

    # ---------------- 바깥에서 부르는 곳 ----------------
    def check_parts(self, parts, snap_to_grid=False):
        """parts → {'ok', 'given_order_ok', 'order', 'grasps', 'failures'}.

        입력: parts = parts/4.0 의 parts 목록(검사 프로그램 _parse 뒤 모양, mm). snap_to_grid = 스캔 설계면 True — x · y 는 GRID_XY_MM,
              z 는 GRID_Z_MM 칸에 맞춰 판정만 한다(저장 좌표는 그대로, E-121 ⑤).
        출력: ok = 되는 순서가 있나 · given_order_ok = AI 가 쓴 sequence 순서 그대로 되나 · order = 되는 순서(sequence 번호 목록, 없으면 None —
              AI 순서가 되면 그 순서) · grasps = {sequence: 그 순서로 놓는 순간 되는 잡기 이름 목록} · failures = [{part, layer, rule, detail}]
              (ok 이면 빈 목록. part = 막힌 블록의 sequence, R0 줄은 층 전체라 None. layer = 층 번호(1부터). detail = '문제 → 확인 → 방법' 꼴, E-114).
        """
        # 상자 · 층 · 닿음을 미리 잰다 — 순서와 상관없다
        boxes = [self.checker._box(self.snap_part(p) if snap_to_grid else p) for p in parts]
        self.prepare_contacts(boxes)


        # R0 층 규칙 — 섞인 층이 있으면 순서를 찾지 않는다
        failures = self.find_layer_mix()
        if failures:
            return self.make_result(False, False, None, {}, failures)


        # AI 순서(sequence 오름차순) 그대로 — 되면 그 순서를 돌려준다(코드가 AI 칸을 고치지 않는다, E-121 ⑨)
        given = sorted(range(len(boxes)), key=lambda i: boxes[i]['sequence'])
        grasps, placed = {}, frozenset()
        for i in given:
            ok, why, names = self.try_add_part(i, placed)
            if not ok:
                break
            grasps[boxes[i]['sequence']] = names
            placed = placed | {i}
        if len(placed) == len(boxes):
            return self.make_result(True, True, [boxes[i]['sequence'] for i in given], grasps, [])


        # 다른 순서 — 낮은 층부터 층마다 찾는다. 막힌 층에서 멈추고 그 층의 막힌 블록을 알린다
        order, grasps, placed = [], {}, frozenset()
        for layer in range(1, len(self.layer_z) + 1):
            found, placed, failures = self.find_layer_order(layer, placed)
            if found is None:
                return self.make_result(False, False, None, {}, failures)
            for i, names in found:
                order.append(boxes[i]['sequence'])
                grasps[boxes[i]['sequence']] = names
        return self.make_result(True, False, order, grasps, [])

    # ---------------- 준비 ----------------
    @staticmethod
    def snap_part(part):
        """스캔 좌표를 판정용 격자에 맞춘 사본(x · y → GRID_XY_MM, z → GRID_Z_MM). 원본은 바꾸지 않는다."""
        out = dict(part)
        out['x'] = round(part['x'] / GRID_XY_MM) * GRID_XY_MM
        out['y'] = round(part['y'] / GRID_XY_MM) * GRID_XY_MM
        out['z'] = round(part['z'] / GRID_Z_MM) * GRID_Z_MM
        return out

    def prepare_contacts(self, boxes):
        """층(밑면 높이) · 받침 · 위 블록을 잰다. below[i] = [(받침 index 또는 None(작업면), 닿은 면 (x0, x1, y0, y1))] — 검사 프로그램 _supports 와 같은 꼴."""
        self.boxes = boxes
        n = len(boxes)
        self.layer_z = sorted({round(b['z0'], 6) for b in boxes})
        self.layer_of = [self.layer_z.index(round(b['z0'], 6)) + 1 for b in boxes]
        self.below = []
        for a in boxes:
            sup = [(None, (a['x0'], a['x1'], a['y0'], a['y1']))] if abs(a['z0']) < EPS else []
            for j, b in enumerate(boxes):
                if b is not a and abs(b['z1'] - a['z0']) < EPS:
                    x0, x1 = max(a['x0'], b['x0']), min(a['x1'], b['x1'])
                    y0, y1 = max(a['y0'], b['y0']), min(a['y1'], b['y1'])
                    if x1 - x0 > EPS and y1 - y0 > EPS:
                        sup.append((j, (x0, x1, y0, y1)))
            self.below.append(sup)
        self.above = [[] for _ in range(n)]
        for i, sup in enumerate(self.below):
            for j, _ in sup:
                if j is not None:
                    self.above[j].append(i)

    # ---------------- R0 층 규칙 ----------------
    def find_layer_mix(self):
        """한 층에 방향 코드가 섞인 층마다 실패 한 줄(part None)."""
        failures = []
        for layer in range(1, len(self.layer_z) + 1):
            kinds = {}
            for i, b in enumerate(self.boxes):
                if self.layer_of[i] == layer:
                    kinds.setdefault(b['ori'], []).append(b['sequence'])
            if len(kinds) > 1:
                mix = ' / '.join(f'{POSE_KO.get(o, o)} {len(s)}개({", ".join(map(str, s[:4]))}{"…" if len(s) > 4 else ""}번)' for o, s in kinds.items())
                failures.append({'part': None, 'layer': layer, 'rule': 'R0',
                                 'detail': f'{layer}층에 자세 · 방향이 다른 블록이 섞여 있어요({mix}) → 그 층 블록이 모두 같은 자세 · 같은 방향인지 → '
                                           f'3D 미리보기에서 {layer}층만 보고, 섞였으면 한 가지로 맞추거나 다시 스캔하세요'})
        return failures

    # ---------------- 한 층 순서 찾기 ----------------
    def find_layer_order(self, layer, placed):
        """층 하나를 다 놓는 순서. placed = 아래층까지 놓인 블록(index frozenset).

        ① 순서와 상관없는 것(받침 없음 · R2 · 블록 자기 R3 — 받침이 모두 아래층이라 같은 층 순서와 상관없음)을 먼저 거르고
        ② R5 거꾸로 빼기(층을 다 놓은 상태에서 '마지막에 놓아도 손가락이 들어가는' 블록을 하나씩 뺀다 — 빼면 손가락 자리는 넓어지기만 해
           아무거나 먼저 빼도 답이 같다)로 어떤 순서로도 안 되는 층을 바로 가린 뒤
        ③ ② 의 순서부터 전부 따진다(아래 덩어리 R3 는 같은 층 순서에 따라 달라서). 같은 '놓인 묶음'은 한 번만 본다.
        반환: (되는 순서 [(index, 잡기 이름 목록)] 또는 None, 끝난 placed, failures).
        """
        members = sorted((i for i in range(len(self.boxes)) if self.layer_of[i] == layer), key=lambda i: self.boxes[i]['sequence'])

        # ① 순서와 상관없는 규칙
        failures = []
        for i in members:
            why = self.find_unsupported(i) or self.find_narrow_contact(i) or self.find_unstable(i, placed | {i}, (i,))
            if why:
                failures.append(self.make_failure(i, *why))
        if failures:
            return None, placed, failures


        # ② R5 거꾸로 빼기 — 못 빼는 블록이 남으면 어떤 순서로도 손가락이 안 들어간다
        rest, peeled = list(members), []
        while rest:
            last = next((i for i in reversed(rest) if self.find_grasps(i, placed | (set(rest) - {i}))), None)
            if last is None:
                return None, placed, [self.make_failure(i, *self.find_blocked_fingers(i, placed | (set(rest) - {i}))) for i in rest]
            rest.remove(last)
            peeled.append(last)
        candidates = peeled[::-1]


        # ③ 층 안 전부 따지기(R1 ~ R5) — 가장 많이 놓은 상태를 기억해 두었다가 못 찾으면 거기서 막힌 블록을 알린다
        deepest = {'placed': placed, 'states': 0, 'timeout': False}
        started = time.monotonic()

        @lru_cache(maxsize=None)
        def place_rest(cur):
            deepest['states'] += 1
            if deepest['states'] > MAX_STATES:
                return None
            if time.monotonic() - started > TIME_LIMIT_S:
                deepest['timeout'] = True
                return None
            if len(cur) > len(deepest['placed']):
                deepest['placed'] = cur
            left = [i for i in candidates if i not in cur]
            if not left:
                return ()
            for i in left:
                ok, _, names = self.try_add_part(i, cur)
                if ok:
                    tail = place_rest(cur | {i})
                    if tail is not None:
                        return ((i, names),) + tail
            return None

        found = place_rest(placed)
        if found is not None:
            return list(found), placed | set(members), []
        if deepest['states'] > MAX_STATES or deepest['timeout']:
            return None, placed, [{'part': None, 'layer': layer, 'rule': 'R3',
                                   'detail': f'{layer}층 블록이 너무 많거나 얽혀 놓는 순서를 다 따져 보지 못했어요 → 그 층 블록 수 · 받침 위치 → '
                                             f'층을 나눠 쌓거나 블록 수를 줄여 다시 검사하세요'}]
        stuck = deepest['placed']
        return None, stuck, [self.make_failure(i, *self.try_add_part(i, stuck)[1]) for i in candidates if i not in stuck]

    # ---------------- 블록 하나 더하기 (R1 ~ R5) ----------------
    def try_add_part(self, i, placed):
        """블록 i 를 지금(placed 가 놓인 상태) 더할 수 있나. 반환: (된다, 안 되면 (규칙, 글) · 되면 None, 되면 잡기 이름 목록)."""
        why = self.find_unsupported(i)
        if why:
            return False, why, []
        missing = [j for j, _ in self.below[i] if j is not None and j not in placed]
        if missing:
            return False, ('R1', f'받침 {", ".join(str(self.boxes[j]["sequence"]) for j in missing)}번이 아직 안 놓였어요 → 아래 블록부터 놓는 순서인지 → '
                                 f'받침 블록의 sequence 를 더 앞으로'), []
        why = self.find_narrow_contact(i)
        if why:
            return False, why, []
        names = self.find_grasps(i, placed)
        if not names:
            return False, self.find_blocked_fingers(i, placed), []
        new_placed = placed | {i}
        why = self.find_unstable(i, new_placed, tuple(sorted(self.collect_supporters(i, new_placed) | {i})))
        if why:
            return False, why, []
        return True, None, names

    def find_unsupported(self, i):
        """받침도 작업면도 없는 블록(공중)이면 ('R1', 글), 아니면 None."""
        if self.below[i]:
            return None
        return ('R1', '아래에 받치는 블록도 작업면도 없어요(공중) → 밑면 높이가 아래 블록 윗면과 같은지 → 3D 미리보기에서 그 블록 높이를 확인하세요')

    def find_narrow_contact(self, i):
        """아래 블록과 닿은 면의 짧은 변이 MIN_BEARING_MM 보다 좁으면 ('R2', 글), 아니면 None. 작업면은 보지 않는다."""
        for j, (x0, x1, y0, y1) in self.below[i]:
            short = min(x1 - x0, y1 - y0)
            if j is not None and short < MIN_BEARING_MM - EPS:
                return ('R2', f'아래 {self.boxes[j]["sequence"]}번에 {short:g} mm만 걸쳐 있어요({MIN_BEARING_MM:g} mm 미만) → 아래 블록과 겹친 폭 → '
                              f'3D 미리보기에서 그 블록을 아래 블록 쪽으로 옮기거나 다시 스캔하세요')
        return None

    def find_grasps(self, i, placed):
        """검사 프로그램 _grasps 그대로 — 블록 i 를 놓는 순간 두 손가락이 다 들어가는 잡기 이름 목록."""
        return self.checker._grasps(self.boxes[i], [self.boxes[j] for j in placed])

    def find_blocked_fingers(self, i, placed):
        """손가락 자리가 없는 이유 ('R5', 글) — 닫는 방향(x · y)마다 자리를 막은 블록 번호."""
        a = self.boxes[i]
        parts = []
        for axis, ko in (('x', '좌우'), ('y', '앞뒤')):
            hit = [self.boxes[j]['sequence'] for j in sorted(placed) if not self.checker._fingers_free(a, axis, [self.boxes[j]])]
            parts.append(f'{ko}로 잡기 ← {", ".join(map(str, sorted(hit)))}번' if hit else f'{ko}로 잡기 ← ?')
        return ('R5', f'놓는 순간 손가락 자리가 없어요({" / ".join(parts)}) → 끝 · 옆에 붙은 블록 → '
                      f'같은 층에서 2 × 2 처럼 사방이 막힌 배치인지 보고, 이웃 블록을 띄우거나 하나 빼세요')

    # ---------------- R3 안정성 (검사 프로그램 4번 계산) ----------------
    def find_unstable(self, i, placed, roots):
        """roots 의 블록마다 '그 블록 + 위에 얹힌 덩어리' 무게중심 여유가 margin_mm 보다 작으면 ('R3', 글), 다 되면 None."""
        for r in roots:
            margin = self.calculate_group_margin(self.collect_group(r, placed), placed)
            if margin < self.checker.margin_mm:
                who = '이 블록이' if r == i else f'아래 {self.boxes[r]["sequence"]}번 위 덩어리가'
                return ('R3', f'놓으면 {who} 기울어요(여유 {margin:.1f} mm < {self.checker.margin_mm} mm) → 무게중심이 받침 안쪽 {self.checker.margin_mm} mm 안인지 → '
                              f'위 블록을 받침 가운데 쪽으로 옮기거나 받침을 더하세요')
        return None

    def calculate_group_margin(self, group, placed):
        """덩어리 group 의 무게중심 → 덩어리가 바깥(작업면 · 덩어리 밖 놓인 받침)에 닿은 면들의 볼록 껍질 가장자리까지(mm, 밖이면 음수).
        검사 프로그램 _stability 와 같은 계산(같은 재질이라 질량 = 부피, _hull · _margin_in_hull)."""
        boxes = self.boxes
        vol = {m: boxes[m]['dx'] * boxes[m]['dy'] * boxes[m]['dz'] for m in group}
        mass = sum(vol.values())
        com = (sum(vol[m] * boxes[m]['x'] for m in group) / mass, sum(vol[m] * boxes[m]['y'] for m in group) / mass)
        pts = []
        for m in group:
            for j, (x0, x1, y0, y1) in self.below[m]:
                if j is None or (j in placed and j not in group):
                    pts += [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        return self.checker._margin_in_hull(com, self.checker._hull(pts))

    def collect_group(self, b, placed):
        """블록 b 와 그 위에 (놓인 것 중) 차례로 얹힌 블록 전부의 index 집합."""
        seen, todo = {b}, [b]
        while todo:
            for k in self.above[todo.pop()]:
                if k in placed and k not in seen:
                    seen.add(k)
                    todo.append(k)
        return seen

    def collect_supporters(self, i, placed):
        """블록 i 를 아래로 받치는(받침의 받침까지) 놓인 블록 index 집합 — i 를 얹으면 무게가 더해지는 덩어리들의 뿌리."""
        out, todo = set(), [i]
        while todo:
            for j, _ in self.below[todo.pop()]:
                if j is not None and j in placed and j not in out:
                    out.add(j)
                    todo.append(j)
        return out

    # ---------------- 결과 ----------------
    def make_failure(self, i, rule, detail):
        """블록 i 의 실패 한 줄 — 글 앞에 '몇 층 · 몇 번'을 붙인다."""
        b = self.boxes[i]
        return {'part': b['sequence'], 'layer': self.layer_of[i], 'rule': rule, 'detail': f'{self.layer_of[i]}층 {b["sequence"]}번: {detail}'}

    @staticmethod
    def make_result(ok, given_order_ok, order, grasps, failures):
        """출력 dict 하나로 모은다."""
        return {'ok': ok, 'given_order_ok': given_order_ok, 'order': order, 'grasps': grasps, 'failures': failures}

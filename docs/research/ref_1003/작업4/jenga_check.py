"""젠가 설계 검사기 (작업 4 V-09용). 혼자 실행되고 표준 라이브러리만 쓴다. Python 3.8 이상.

L/W/T, SIZE, Block, overlap, hull, margin_in_hull, check 는 jenga_designs.py 와 같은 계산이다(그대로 옮김).
더한 것:
  load_design(path)  설계 JSON(템플릿 원본 lvN_*.json 이나 GPT 출력)을 Block 목록으로 바꾼다.
                     템플릿 원본의 orientation 은 한글이고 '세움'이 zx·zy 둘 다라서 size_mm 로 방향 코드를 고른다.
  format_errors()    형식 검사
  penetrations()     블록끼리 파고듦 검사. x·y·z 세 방향 모두 0.1 mm 넘게 겹칠 때만 파고듦이다(면끼리 닿은 것은 괜찮다)
  step_margins()     order 순서대로 한 개씩 놓을 때마다 check 한다. 그중 가장 작은 값이 '쌓는 중 최소 여유'
  build_template()   V-09 대안(템플릿을 고르고 숫자만 정하기)용 설계 6종 생성기
  draw_png()         그림 저장. 이 기능만 matplotlib 이 필요하다

사용:
  python3 jenga_check.py lv1_bench.json lv2_chair.json     # 검사 (파일 여러 개 가능)
  python3 jenga_check.py --convert lv1_bench.json          # GPT 출력 형식 {"blocks": [...]} 으로 바꿔 출력
  python3 jenga_check.py --png lv1.png lv1_bench.json      # 그림 저장 (matplotlib 필요)
  python3 jenga_check.py --template bookshelf sections=4   # 템플릿으로 만들어 검사
통과: 형식 OK + 파고듦 없음 + 쌓는 중 최소 여유 >= 7 mm. 공중에 뜬 블록이 있으면 여유를 -1000000000 으로 보고 실패.
"""
import json
import math
import sys

# ---------- jenga_designs.py 에서 그대로 옮긴 부분 ----------
L, W, T = 75.0, 25.0, 15.0
SIZE = {"x": (L, W, T), "y": (W, L, T),            # 눕힘(길이 방향 x / y)
        "xe": (L, T, W), "ye": (T, L, W),          # 옆으로 세움
        "zx": (T, W, L), "zy": (W, T, L)}          # 똑바로 세움
ORI_KO = {"x": "눕힘(좌우)", "y": "눕힘(앞뒤)", "xe": "옆세움(좌우)", "ye": "옆세움(앞뒤)",
          "zx": "세움", "zy": "세움"}


class Block:
    def __init__(s, x, y, z, ori, tag=""):
        s.x, s.y, s.z, s.ori, s.tag = x, y, z, ori, tag
        s.dx, s.dy, s.dz = SIZE[ori]

    @property
    def box(s):
        return (s.x - s.dx / 2, s.x + s.dx / 2, s.y - s.dy / 2, s.y + s.dy / 2, s.z, s.z + s.dz)

    def com(s):
        return (s.x, s.y, s.z + s.dz / 2)


def overlap(a, b):
    ax0, ax1, ay0, ay1, _, _ = a.box
    bx0, bx1, by0, by1, _, _ = b.box
    x0, x1, y0, y1 = max(ax0, bx0), min(ax1, bx1), max(ay0, by0), min(ay1, by1)
    if x1 - x0 > 1e-6 and y1 - y0 > 1e-6:
        return (x0, x1, y0, y1)
    return None


def hull(pts):
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


def margin_in_hull(p, poly):
    """점이 볼록 다각형 안이면 가장 가까운 변까지 거리(+), 밖이면 음수."""
    if len(poly) < 3:
        return -1.0
    best = float("inf")
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        ln = math.hypot(ex, ey)
        d = (ex * (p[1] - a[1]) - ey * (p[0] - a[0])) / ln   # 반시계 다각형: 안쪽이면 +
        best = min(best, d)
    return best


def check(blocks):
    """위에서부터 하중(무게·작용점)을 아래로 넘기며 블록마다 '받친 면 안에 무게중심이 있는가'를 본다.
    여러 받침에 걸친 블록은 접촉 면적 비율로 나누고, 접촉면 중심에 하중을 건다(근사)."""
    n = len(blocks)
    mass = [b.dx * b.dy * b.dz for b in blocks]
    load = [[m, m * blocks[i].x, m * blocks[i].y] for i, m in enumerate(mass)]
    order = sorted(range(n), key=lambda i: -blocks[i].z)
    worst = (float("inf"), None)
    for i in order:
        bi = blocks[i]
        sup = []
        if bi.z < 1e-6:
            x0, x1, y0, y1, _, _ = bi.box
            sup.append((None, (x0, x1, y0, y1)))
        else:
            for j in range(n):
                if j != i and abs(blocks[j].z + blocks[j].dz - bi.z) < 1e-6:
                    ov = overlap(bi, blocks[j])
                    if ov:
                        sup.append((j, ov))
        if not sup:
            return -1e9, i
        pts = []
        for _, (x0, x1, y0, y1) in sup:
            pts += [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        poly = hull(pts)
        m, mx, my = load[i]
        com = (mx / m, my / m)
        mg = margin_in_hull(com, poly)
        if mg < worst[0]:
            worst = (mg, i)
        tot_area = sum((x1 - x0) * (y1 - y0) for _, (x0, x1, y0, y1) in sup)
        for j, (x0, x1, y0, y1) in sup:
            if j is None:
                continue
            f = (x1 - x0) * (y1 - y0) / tot_area
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            load[j][0] += f * m
            load[j][1] += f * m * cx
            load[j][2] += f * m * cy
    return worst


# ---------- 여기부터 더한 부분 ----------
PASS_MM = 7.0           # 통과 기준: 쌓는 중 최소 여유 >= 7 mm (SRD FR-P02)
PENETRATION_MM = 0.1    # x·y·z 세 방향 모두 이 값보다 깊게 겹치면 파고듦
SET_BLOCKS = 54         # 젠가 1세트. 넘으면 경고만 한다(통과 판정에는 쓰지 않음)
KEYS = ("order", "x", "y", "z", "ori")


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _plain(v):
    """37.0 -> 37, 37.5 -> 37.5 (출력을 짧게)."""
    v = float(v)
    return int(v) if v.is_integer() else v


def code_from_size(size_mm):
    """size_mm [좌우, 앞뒤, 높이] 로 방향 코드를 고른다."""
    size = tuple(float(v) for v in size_mm)
    for code, s in SIZE.items():
        if s == size:
            return code
    raise ValueError("size_mm %s 에 맞는 방향 코드가 없다" % (list(size),))


def parse_text(text):
    """JSON 글자를 읽는다. 앞뒤 ``` 코드 블록 표시만 떼고, 그 밖에는 고치지 않는다."""
    s = text.strip()
    if s.startswith("```"):
        s = s.split("\n", 1)[1] if "\n" in s else ""
        if s.rstrip().endswith("```"):
            s = s.rstrip()[:-3]
    return json.loads(s)


def items_from_data(data):
    """읽은 JSON 을 [{"order","x","y","z","ori"}, ...] 로 바꾼다.
    받는 형식: 템플릿 원본({"sequence": [...]}), {"blocks": [...]}, 또는 목록 [...]."""
    if isinstance(data, dict) and isinstance(data.get("sequence"), list):
        out = []
        for it in data["sequence"]:
            out.append({"order": it["order"], "x": _plain(it["center_xy_mm"][0]),
                        "y": _plain(it["center_xy_mm"][1]), "z": _plain(it["bottom_z_mm"]),
                        "ori": code_from_size(it["size_mm"])})
        return out
    if isinstance(data, dict) and "blocks" in data:
        return data["blocks"]
    return data


def read_items(path):
    with open(path, encoding="utf-8") as f:
        return items_from_data(parse_text(f.read()))


def format_errors(items):
    if not isinstance(items, list) or not items:
        return ["블록 목록이 없다 (목록 [...] 이나 {\"blocks\": [...]} 이어야 한다)"]
    errs = []
    for k, it in enumerate(items, 1):
        if not isinstance(it, dict):
            errs.append("%d번째 항목이 객체가 아니다" % k)
            continue
        miss = [key for key in KEYS if key not in it]
        if miss:
            errs.append("%d번째 항목에 %s 없음" % (k, ", ".join(miss)))
        for key in ("x", "y", "z"):
            if key in it and not _num(it[key]):
                errs.append("%d번째 항목의 %s 가 숫자가 아니다" % (k, key))
        if "ori" in it and it["ori"] not in SIZE:
            errs.append("%d번째 항목 ori=%r 는 x, y, xe, ye, zx, zy 가 아니다" % (k, it["ori"]))
        if _num(it.get("z")) and it["z"] < 0:
            errs.append("%d번째 항목의 z 가 음수다" % k)
    orders = [it.get("order") for it in items if isinstance(it, dict)]
    if any(not isinstance(o, int) or isinstance(o, bool) for o in orders) or \
            sorted(orders) != list(range(1, len(items) + 1)):
        errs.append("order 가 1부터 %d까지 하나씩 있지 않다" % len(items))
    return errs


def blocks_from_items(items):
    """형식이 맞는 목록을 order 순서의 Block 목록으로."""
    items = sorted(items, key=lambda it: it["order"])
    return [Block(float(it["x"]), float(it["y"]), float(it["z"]), it["ori"], it.get("part", ""))
            for it in items]


def load_design(path):
    """설계 JSON 파일 -> order 순서의 Block 목록. 형식이 틀리면 ValueError."""
    items = read_items(path)
    errs = format_errors(items)
    if errs:
        raise ValueError("; ".join(errs))
    return blocks_from_items(items)


def penetrations(blocks, tol=PENETRATION_MM):
    """파고든 블록 쌍 [(order a, order b, 겹친 부피 mm3), ...]. blocks 는 order 순서."""
    bad = []
    for i in range(len(blocks)):
        a = blocks[i].box
        for j in range(i + 1, len(blocks)):
            b = blocks[j].box
            dx = min(a[1], b[1]) - max(a[0], b[0])
            dy = min(a[3], b[3]) - max(a[2], b[2])
            dz = min(a[5], b[5]) - max(a[4], b[4])
            if dx > tol and dy > tol and dz > tol:
                bad.append((i + 1, j + 1, round(dx * dy * dz)))
    return bad


def step_margins(blocks):
    """order 순서로 1개, 2개, ... n개를 놓았을 때마다 check. [(놓은 개수, 여유 mm, 가장 위험한 블록 order), ...]"""
    out = []
    for k in range(1, len(blocks) + 1):
        mg, idx = check(blocks[:k])
        out.append((k, mg, None if idx is None else idx + 1))
    return out


def evaluate_items(items):
    """형식·파고듦·쌓는 중 여유를 모두 본다. 결과 dict 를 돌려준다."""
    errs = format_errors(items)
    res = {"format_ok": not errs, "errors": errs, "pass": False}
    if errs:
        return res
    blocks = blocks_from_items(items)
    steps = step_margins(blocks)
    k, mg, at = min(steps, key=lambda s: s[1])
    res.update({
        "blocks": len(blocks),
        "height": max(b.z + b.dz for b in blocks),
        "levels": len(set(round(b.z, 3) for b in blocks)),
        "penetrations": penetrations(blocks),
        "final_margin": check(blocks)[0],
        "min_margin": mg, "min_step": k, "min_block": at,
        "floating": mg <= -1e8,
        "too_many": len(blocks) > SET_BLOCKS,
    })
    res["pass"] = (not res["penetrations"]) and mg >= PASS_MM
    return res


def summary(res):
    """검사 결과 한 줄."""
    if not res["format_ok"]:
        return "형식 실패: " + "; ".join(res["errors"]) + " | 실패"
    pen = res["penetrations"]
    pen_s = "파고듦 없음" if not pen else "파고듦 %d쌍 (order %s)" % (
        len(pen), ", ".join("%d-%d" % (a, b) for a, b, _ in pen[:5]))
    if res["floating"]:
        mg_s = "쌓는 중: 공중에 뜬 블록 있음 (%d개째 놓을 때, order %s)" % (res["min_step"], res["min_block"])
    else:
        mg_s = "완성 여유 %.1f mm, 쌓는 중 최소 여유 %.1f mm (%d개째 놓을 때, order %s)" % (
            res["final_margin"], res["min_margin"], res["min_step"], res["min_block"])
    warn = " | 경고: 블록 %d개 > 1세트 %d개" % (res["blocks"], SET_BLOCKS) if res["too_many"] else ""
    return "형식 OK | 블록 %d개, 높이 %.0f mm, 높이 칸 %d개 | %s | %s%s | %s" % (
        res["blocks"], res["height"], res["levels"], pen_s, mg_s, warn, "통과" if res["pass"] else "실패")


def to_codes(items):
    """GPT 출력 형식 글자. 블록 하나에 한 줄."""
    rows = ['  {"order": %d, "x": %s, "y": %s, "z": %s, "ori": "%s"}' % (
        it["order"], json.dumps(_plain(it["x"])), json.dumps(_plain(it["y"])), json.dumps(_plain(it["z"])), it["ori"])
        for it in sorted(items, key=lambda it: it["order"])]
    return '{"blocks": [\n' + ",\n".join(rows) + "\n]}"


# ---------- V-09 대안: 템플릿 6종 생성기 (jenga_designs.py 설계 함수, 기본값이면 lv1~lv6 과 같다) ----------
def bench(layers=4):
    b = []
    for k in range(layers):
        for sx in (-1, 1):
            b.append(Block(sx * 25, 0, k * T, "y", "다리 벽"))
    for yy in (-25, 0, 25):
        b.append(Block(0, yy, layers * T, "x", "좌판"))
    return b


def chair(legs=4, back=5):
    """jenga_designs.chair() 에 다리 층수(legs)와 등받이 층수(back)를 숫자로 뺀 것. 기본값이면 Lv2 와 같다."""
    b = bench(legs)
    z0 = (legs + 1) * T
    for k in range(back):
        b.append(Block(0, 25, z0 + k * T, "x", "등받이"))
    return b


def bookshelf(sections=3, wall_layers=3):
    b, z = [], 0.0
    for s in range(sections):
        for k in range(wall_layers):
            for sx in (-1, 1):
                b.append(Block(sx * 25, 0, z, "y", "옆판"))
            z += T
        for yy in (-25, 0, 25):
            b.append(Block(0, yy, z, "x", "선반"))
        z += T
    return b


def table_standing():
    b = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            b.append(Block(sx * 30, sy * 25, 0, "zx", "세운 다리"))
    for sy in (-1, 1):
        b.append(Block(0, sy * 25, L, "x", "보"))
    for xx in (-25, 0, 25):
        b.append(Block(xx, 0, L + T, "y", "상판"))
    return b


def chair_standing():
    b = table_standing()
    z = L + 2 * T
    for xx in (-25, 0, 25):
        b.append(Block(xx, 30, z, "zy", "세운 등받이"))
    b.append(Block(0, 30, z + L, "x", "등받이 덮개"))
    return b


def corbel_arch(open_w=150.0, pillar=3, step=10.0, n=5):
    """양쪽 기둥을 pillar층 곧게 쌓고, 그 위로 한 층마다 step씩 안쪽으로 내밀어 n층, 맨 위 쐐기 블록으로 만나는 아치."""
    b = []
    k = 0
    for j in range(pillar + n):
        shift = max(0, j - pillar + 1) * step
        inner = open_w / 2 - shift
        cx = inner + L / 2
        for sx in (-1, 1):
            b.append(Block(sx * cx, 0, j * T, "x", "기둥" if j < pillar else "내민 층"))
    gap = open_w - 2 * n * step
    b.append(Block(0, 0, (pillar + n) * T, "x", "쐐기(맨 위)"))
    return b, gap


# 이름: (함수, 기본 숫자, 정수여야 하는 숫자)
TEMPLATES = {
    "bench": (bench, {"layers": 4}, ("layers",)),
    "chair": (chair, {"legs": 4, "back": 5}, ("legs", "back")),
    "bookshelf": (bookshelf, {"sections": 3, "wall_layers": 3}, ("sections", "wall_layers")),
    "table_standing": (table_standing, {}, ()),
    "chair_standing": (chair_standing, {}, ()),
    "corbel_arch": (corbel_arch, {"open_w": 150, "pillar": 3, "step": 10, "n": 5}, ("pillar", "n")),
}
COPY_GAP_MM = 50.0   # count 가 2 이상이면 같은 가구를 좌우(x)로 이 간격을 두고 나란히 놓는다


def build_template(name, params=None, count=1):
    """템플릿 이름과 숫자로 설계를 만든다 -> GPT 출력과 같은 형식의 목록. 잘못된 값이면 ValueError."""
    if name not in TEMPLATES:
        raise ValueError("템플릿 이름 %r 은 %s 중 하나가 아니다" % (name, ", ".join(TEMPLATES)))
    fn, defaults, int_keys = TEMPLATES[name]
    params = dict(params or {})
    unknown = [k for k in params if k not in defaults]
    if unknown:
        raise ValueError("%s 에 없는 숫자: %s (쓸 수 있는 것: %s)" % (name, ", ".join(unknown), ", ".join(defaults) or "없음"))
    for k, v in params.items():
        if not _num(v) or v <= 0 or (k in int_keys and float(v) != int(v)):
            raise ValueError("%s=%r 는 %s" % (k, v, "1 이상의 정수여야 한다" if k in int_keys else "0보다 큰 숫자여야 한다"))
        if k in int_keys:
            params[k] = int(v)
    if not isinstance(count, int) or isinstance(count, bool) or not 1 <= count <= 4:
        raise ValueError("count=%r 는 1~4 의 정수여야 한다" % (count,))
    out = fn(**params)
    one = out[0] if isinstance(out, tuple) else out
    if len(one) * count > 300:
        raise ValueError("블록이 너무 많다 (%d개)" % (len(one) * count))
    one = sorted(one, key=lambda b: (b.z, b.x, b.y))
    x0 = min(b.box[0] for b in one)
    x1 = max(b.box[1] for b in one)
    pitch = (x1 - x0) + COPY_GAP_MM
    items = []
    for c in range(count):
        shift = (c - (count - 1) / 2) * pitch
        for b in one:
            items.append({"order": len(items) + 1, "x": _plain(b.x + shift), "y": _plain(b.y),
                          "z": _plain(b.z), "ori": b.ori, "part": b.tag})
    return items


# ---------- 그림 (matplotlib 필요) ----------
PALETTE = ["#d8b98a", "#c9a46e", "#e2c79c", "#b88d55", "#d1ae7a", "#e8d2ad"]


def draw_png(items, path, title=""):
    """설계 그림을 PNG 로 저장한다. 제목은 영문·숫자만 (한글 글꼴이 없어도 되게)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    blocks = blocks_from_items(items)
    fig = plt.figure(figsize=(5, 5), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    for b in blocks:
        x0, x1, y0, y1, z0, z1 = b.box
        v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
             (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        idx = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
        ax.add_collection3d(Poly3DCollection([[v[k] for k in f] for f in idx],
                                             facecolors=PALETTE[int(round(b.z / T)) % len(PALETTE)],
                                             edgecolors="#5a4630", linewidths=0.5))
    xs = [v for b in blocks for v in b.box[0:2]]
    ys = [v for b in blocks for v in b.box[2:4]]
    zs = [v for b in blocks for v in b.box[4:6]]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    r = max(max(xs) - min(xs), max(ys) - min(ys)) / 2 + 8
    ax.set_xlim(cx - r, cx + r)
    ax.set_ylim(cy - r, cy + r)
    ax.set_zlim(0, max(zs) + 8)
    ax.set_box_aspect((2 * r, 2 * r, max(zs) + 8))
    ax.view_init(elev=22, azim=-58)
    ax.locator_params(nbins=4)
    ax.tick_params(labelsize=7)
    ax.set_xlabel("x (mm)", fontsize=8)
    ax.set_ylabel("y (mm)", fontsize=8)
    if title:
        ax.set_title(title, fontsize=9)
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def _main(argv):
    if len(argv) >= 2 and argv[0] == "--convert":
        for p in argv[1:]:
            print(to_codes(read_items(p)))
        return 0
    if len(argv) >= 2 and argv[0] == "--template":
        params = {}
        for kv in argv[2:]:
            k, v = kv.split("=", 1)
            params[k] = int(v) if v.lstrip("-").isdigit() else float(v)
        count = int(params.pop("count", 1))
        items = build_template(argv[1], params, count)
        print(" ".join(argv[1:]) + ": " + summary(evaluate_items(items)))
        return 0
    if len(argv) == 3 and argv[0] == "--png":
        draw_png(read_items(argv[2]), argv[1], argv[2])
        print("그림 저장:", argv[1])
        return 0
    if not argv:
        print(__doc__)
        return 1
    bad = 0
    for p in argv:
        try:
            res = evaluate_items(read_items(p))
        except (ValueError, KeyError, TypeError, IndexError) as e:   # JSON 이 아니거나 템플릿 원본 형식이 깨진 경우
            res = {"format_ok": False, "errors": ["읽기 실패: %s" % e], "pass": False}
        print("%s: %s" % (p, summary(res)))
        bad += 0 if res["pass"] else 1
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))

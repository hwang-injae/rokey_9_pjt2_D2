"""웹캠 화면 -> 책상 좌표(mm) 맞추기. V-06 팔 추적 결과를 로봇 좌표(base_link)로 바꿀 때 쓴다.
책상은 평면이다. 그래서 테이프 십자 4개 이상의 (화면 픽셀, 로봇 좌표 mm)로 변환 행렬 H 하나를 구한다.
H 는 화면 픽셀 (u, v) -> base_link 의 (x, y) mm 로 바꾼다. 다른 스크립트: from webcam_calib import load_h, px_to_mm

필요: pip install "mediapipe==1.0.1"   (OpenCV, numpy 가 같이 깔린다). selftest 는 numpy 만 있어도 된다.
실행: python3 webcam_calib.py calib --cam 1          # 테이프 4점을 차례로 클릭 -> webcam_H.json
      python3 webcam_calib.py calib --image 책상.jpg  # 웹캠 대신 사진으로 해 보기
      python3 webcam_calib.py check --cam 1 --truth "450,0;350,-150;650,150;550,-200"   # 다른 4점으로 오차 보기
      python3 webcam_calib.py selftest                # 카메라 없이 계산만 시험. PASS 가 나와야 한다
화면: calib = 위에 다음에 클릭할 점 번호와 mm 좌표가 나온다. 왼쪽 클릭 = 찍기, u = 하나 취소, r = 처음부터, q = 끝.
      다 찍으면 H 를 저장하고 100 mm 격자, 책상 테두리(굵게), 로봇 바닥 중심(base), 조립 영역(450, 0)을 겹쳐 보여 준다.
      check = 클릭한 곳의 계산 좌표(mm)를 보여 주고, 결과를 webcam_check.csv 에 쌓는다. q = 끝.
      화면 글자는 영어다 (OpenCV 글꼴은 한글을 못 쓴다). 한글 안내는 터미널에 나온다.

테이프 점 재는 법 (단위 mm, 로봇 좌표 base_link):
  1. 로봇을 멈춘다 (정지 상태 확인). 사람이 책상에 테이프를 붙이고 재는 동안 로봇이 움직이면 안 된다.
  2. x = 로봇 바닥 중심에서 로봇 앞쪽(책상 긴 변 방향)으로 잰 거리. 먼 끝이 x = +850, 뒤 끝이 x = -350.
     y = 로봇 왼쪽이 +, 오른쪽이 -. 카메라는 오른쪽(-y) 바깥에 있다.
  3. 바닥 중심에서 재기 어려우면 책상 모서리에서 잰다.
     먼저 로봇 바닥 중심이 먼 끝에서 850 mm, 카메라 쪽 긴 변에서 325 mm 인지 줄자로 확인한다. 그다음
     x = 850 - (먼 끝에서 잰 거리),   y = (카메라 쪽 긴 변에서 잰 거리) - 325
  4. calib 4점 기본값 = (250,-250) (750,-250) (750,250) (250,250). 다른 점을 쓰면 --pts 로 준다.
     check 4점은 이와 다른 곳에 붙인다 (예: (450,0) (350,-150) (650,150) (550,-200)).
     테이프에 번호를 써 두고, 화면이 말하는 번호 순서대로 클릭한다.
주의: H 는 점이 책상 '위에 붙어 있다'고 보고 계산한다.
  책상에서 떠 있는 손은 카메라에서 먼 쪽(+y)으로 밀려 보인다. 카메라 (450, -1100) mm, 책상 위 1400 mm 일 때
  (450, 0) 위 100 mm 의 점 -> +y 로 약 85 mm, 200 mm -> 약 183 mm 밀린다 (selftest 출력 값).
  카메라를 건드리면 calib 를 다시 한다. 해상도(--width --height)도 calib 때와 같아야 한다.
"""
import argparse
import csv
import datetime
import json
import math
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:   # selftest 와 px_to_mm 등은 numpy 만으로 된다
    cv2 = None

DEFAULT_PTS = "250,-250;750,-250;750,250;250,250"
DEFAULT_TRUTH = "450,0;350,-150;650,150;550,-200"
DESK = (-350, 850, -325, 325)   # 책상 x 시작, x 끝, y 시작, y 끝 (mm, base_link)
WORK = (450, 0)                 # 조립 영역 (mm)


# ---------- 계산 (다른 스크립트도 쓴다) ----------

def fit_h(img_pts, mm_pts):
    """픽셀 점 -> mm 점 변환 행렬 H (3x3). 점 4개 이상. 못 구하면 None."""
    src = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    dst = np.asarray(mm_pts, dtype=np.float64).reshape(-1, 2)
    if len(src) < 4 or len(src) != len(dst):
        return None
    if cv2 is not None:
        try:
            H, _ = cv2.findHomography(src, dst, 0)
        except cv2.error:
            H = None
    else:
        H = fit_h_numpy(src, dst)
    if H is None or H.shape != (3, 3) or not np.all(np.isfinite(H)) or np.linalg.cond(H) > 1e12:
        return None   # 점이 한 줄에 있거나 같은 곳을 두 번 찍었다
    return H / H[2, 2] if abs(H[2, 2]) > 1e-12 else H


def fit_h_numpy(src, dst):
    """cv2 가 없을 때 쓰는 같은 계산 (정규화한 DLT, 최소제곱)."""
    def norm(p):
        c = p.mean(axis=0)
        s = math.sqrt(2) / max(np.sqrt(((p - c) ** 2).sum(axis=1)).mean(), 1e-12)
        return np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1.0]])
    t1, t2 = norm(src), norm(dst)
    a = np.c_[src, np.ones(len(src))] @ t1.T
    b = np.c_[dst, np.ones(len(dst))] @ t2.T
    rows = []
    for (x, y, _), (u, v, _) in zip(a, b):
        rows.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        rows.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, vt = np.linalg.svd(np.array(rows))
    return np.linalg.inv(t2) @ vt[-1].reshape(3, 3) @ t1


def px_to_mm(H, u, v):
    """화면 픽셀 (u, v) -> 책상 좌표 (x, y) mm."""
    x, y, w = np.asarray(H, dtype=np.float64) @ (float(u), float(v), 1.0)
    return float(x / w), float(y / w)


def mm_to_px(H, x, y):
    """책상 좌표 (x, y) mm -> 화면 픽셀 (u, v)."""
    u, v, w = np.linalg.inv(np.asarray(H, dtype=np.float64)) @ (float(x), float(y), 1.0)
    return float(u / w), float(v / w)


def load_calib(path):
    """calib 가 저장한 json 을 읽는다. H 는 numpy 3x3 으로 바꿔 준다."""
    if not os.path.exists(path):
        raise SystemExit("%s 가 없다. 먼저 python3 webcam_calib.py calib 로 만든다 (다른 곳에 있으면 경로를 준다)" % path)
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        d["H"] = np.array(d["H"], dtype=np.float64).reshape(3, 3)
    except (ValueError, KeyError, TypeError):
        raise SystemExit("%s 를 읽을 수 없다. calib 를 다시 해서 새로 만든다" % path)
    return d


def load_h(path):
    """H (numpy 3x3) 만 돌려준다."""
    return load_calib(path)["H"]


def h_warnings(H, img_pts):
    """클릭 순서가 틀렸을 때 생기는 문제를 찾는다. 문제마다 한 문장."""
    w = [float(np.dot(H[2], (u, v, 1.0))) for u, v in img_pts]
    if min(w) * max(w) <= 0:
        return ["점 순서가 꼬였다. 테이프 번호와 클릭 순서를 맞춰서 r 로 다시 찍는다."]
    cu, cv_ = np.mean(np.asarray(img_pts, dtype=np.float64), axis=0)
    x0, y0 = px_to_mm(H, cu, cv_)
    x1, y1 = px_to_mm(H, cu + 1, cv_)
    x2, y2 = px_to_mm(H, cu, cv_ + 1)
    if (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0) > 0:   # 위에서 본 책상은 이 값이 늘 음수다
        return ["좌우가 뒤집혔다. 클릭 순서가 거울처럼 바뀌었거나 카메라 영상이 좌우 반전이다. 확인하고 r 로 다시 찍는다."]
    return []


def parse_pts(s, what):
    """'x,y;x,y;...' -> [(x, y), ...] (mm)."""
    try:
        pts = [tuple(float(v) for v in p.split(",")) for p in s.replace(" ", "").strip(";").split(";")]
    except ValueError:
        pts = []
    if not pts or any(len(p) != 2 for p in pts):
        raise SystemExit('%s 형식이 틀렸다. 예: "250,-250;750,-250;750,250;250,250" (점은 ; 로, x 와 y 는 , 로 나눈다. mm)'
                         % what)
    return pts


# ---------- 화면 ----------

def need_cv2():
    if cv2 is None:
        raise SystemExit('OpenCV(cv2)가 없다. pip install "mediapipe==1.0.1" 로 깐 환경에서 실행한다 (OpenCV 가 같이 깔린다)')


def text(img, s, org, color=(255, 255, 255), scale=0.6):
    """검은 바탕 위 글자 (밝은 책상 위에서도 보이게)."""
    (tw, th), base = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
    cv2.rectangle(img, (org[0] - 3, org[1] - th - 4), (org[0] + tw + 3, org[1] + base), (0, 0, 0), -1)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def draw_grid(img, H):
    """책상 위 100 mm 격자, 책상 테두리, base (0,0) 와 x/y 화살표, 조립 영역 (450,0) 을 그린다."""
    hi = np.linalg.inv(H)
    ref = (hi @ (WORK[0], WORK[1], 1.0))[2]

    def px(x, y):
        u, v, w = hi @ (float(x), float(y), 1.0)
        if w * ref <= 0:   # 카메라 뒤쪽 점은 그리지 않는다
            return None
        return int(np.clip(u / w, -1e5, 1e5)), int(np.clip(v / w, -1e5, 1e5))

    def line(p, q, color, thick):
        a, b = px(*p), px(*q)
        if a and b:
            cv2.line(img, a, b, color, thick, cv2.LINE_AA)

    x0, x1, y0, y1 = DESK
    for x in range(int(math.ceil(x0 / 100.0)) * 100, x1 + 1, 100):
        line((x, y0), (x, y1), (170, 170, 170), 1)
    for y in range(int(math.ceil(y0 / 100.0)) * 100, y1 + 1, 100):
        line((x0, y), (x1, y), (170, 170, 170), 1)
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    for i in range(4):
        line(corners[i], corners[(i + 1) % 4], (0, 255, 255), 3)
    o, ax, ay = px(0, 0), px(100, 0), px(0, 100)
    if o and ax and ay:
        cv2.arrowedLine(img, o, ax, (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.25)
        cv2.arrowedLine(img, o, ay, (0, 200, 0), 2, cv2.LINE_AA, tipLength=0.25)
        text(img, "x", (ax[0] + 4, ax[1]), (0, 0, 255))
        text(img, "y", (ay[0] + 4, ay[1]), (0, 200, 0))
    for (x, y), label, color in (((0, 0), "base (0,0)", (0, 0, 255)), (WORK, "work (450,0)", (255, 0, 255))):
        p = px(x, y)
        if p:
            cv2.circle(img, p, 7, color, -1)
            text(img, label, (p[0] + 10, p[1] + 30), color)


def open_source(a, width, height):
    """카메라 또는 사진. (프레임 읽기 함수, 닫기 함수, json 에 적을 이름) 을 돌려준다."""
    if a.image:
        img = cv2.imread(a.image)
        if img is None:
            raise SystemExit("사진을 열 수 없다: %s (경로와 파일 이름을 본다)" % a.image)
        return (lambda: img.copy()), (lambda: None), a.image
    cap = cv2.VideoCapture(a.cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened() or not cap.read()[0]:
        cap.release()
        raise SystemExit("카메라 %d번을 열 수 없다. --cam 1 처럼 번호를 바꿔 본다 (번호 확인: ls /dev/video*)" % a.cam)

    def grab():
        ok, f = cap.read()
        if not ok:
            raise SystemExit("카메라 영상이 끊겼다. USB 를 다시 꽂고 다시 실행한다")
        return f
    return grab, cap.release, a.cam


def make_window(name, st):
    """창을 만들고 마우스를 연결한다. 클릭은 st['new'] 에, 마우스 위치는 st['mouse'] 에 원래 픽셀로 쌓인다."""
    def on_mouse(event, x, y, flags, param):
        p = (int(round(x / st["scale"])), int(round(y / st["scale"])))
        st["mouse"] = p
        if event == cv2.EVENT_LBUTTONDOWN:
            st["new"].append(p)
    cv2.namedWindow(name)
    cv2.setMouseCallback(name, on_mouse)


def show(name, img, st):
    """큰 화면은 줄여서 보여 준다 (좌표는 원래 픽셀 기준). 누른 키를 돌려준다."""
    st["scale"] = min(1.0, 1400.0 / img.shape[1], 900.0 / img.shape[0])
    view = cv2.resize(img, None, fx=st["scale"], fy=st["scale"]) if st["scale"] < 1.0 else img
    cv2.imshow(name, view)
    return cv2.waitKey(20) & 0xFF


def fmt(p):
    return "(%g, %g)" % (p[0], p[1])


# ---------- calib ----------

def save_calib(path, H, img_pts, mm_pts, cam, size):
    d = {"H": H.tolist(), "img_pts": [list(p) for p in img_pts], "mm_pts": [list(p) for p in mm_pts],
         "cam": cam, "width": size[0], "height": size[1],
         "saved_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=1)


def cmd_calib(a):
    need_cv2()
    mm_pts = parse_pts(a.pts, "--pts")
    n = len(mm_pts)
    if n < 4:
        raise SystemExit("--pts 는 4점 이상이어야 한다 (지금 %d점)" % n)
    grab, close, cam = open_source(a, a.width, a.height)
    win = "webcam_calib (q: quit)"
    st = {"new": [], "mouse": None, "scale": 1.0}
    make_window(win, st)
    clicks, H, warn, saved, size = [], None, [], False, None

    def say_next():
        k = len(clicks)
        print("%d/%d: %s mm 테이프 십자를 클릭한다." % (k + 1, n, fmt(mm_pts[k])))

    print("테이프 %d점을 차례로 클릭한다. u = 하나 취소, r = 처음부터, q = 끝." % n)
    say_next()
    while True:
        frame = grab()
        size = (frame.shape[1], frame.shape[0])
        while st["new"]:
            p = st["new"].pop(0)
            if H is not None:
                print("이미 다 찍었다. 다시 하려면 r 을 누른다.")
                continue
            clicks.append(p)
            print("  %d번 점: 픽셀 (%d, %d)" % (len(clicks), p[0], p[1]))
            if len(clicks) < n:
                say_next()
                continue
            H = fit_h(clicks, mm_pts)
            if H is None:
                print("H 를 못 구했다. 점이 한 줄에 있거나 같은 곳을 두 번 찍었다. 처음부터 다시 찍는다.")
                clicks = []
                say_next()
                continue
            errs = [math.hypot(*np.subtract(px_to_mm(H, *c), m)) for c, m in zip(clicks, mm_pts)]
            print("H 를 구했다. 찍은 점을 다시 계산한 오차(mm): %s | 최대 %.1f mm"
                  % (", ".join("%d번 %.1f" % (i + 1, e) for i, e in enumerate(errs)), max(errs)))
            if n == 4:
                print("  (점이 4개면 이 오차는 늘 0 이다. 진짜 정확도는 check 로 잰다.)")
            warn = h_warnings(H, clicks)
            for s in warn:
                print("[주의] " + s)
            save_calib(a.out, H, clicks, mm_pts, cam, size)
            saved = True
            print("저장했다: %s (해상도 %dx%d)" % (os.path.abspath(a.out), size[0], size[1]))
            print("화면의 base 표시가 실제 로봇 바닥 중심 위에, 노란 테두리가 책상 가장자리에 오는지 본다. r = 다시, q = 끝.")
        if H is not None:
            draw_grid(frame, H)
        for i, c in enumerate(clicks):
            cv2.circle(frame, c, 6, (0, 0, 255), -1)
            text(frame, str(i + 1), (c[0] + 8, c[1] - 8), (0, 0, 255))
        if H is None:
            text(frame, "%d/%d: click tape cross at %s mm" % (len(clicks) + 1, n, fmt(mm_pts[len(clicks)])),
                 (10, 30), (0, 255, 255), 0.8)
            text(frame, "u: undo  r: restart  q: quit", (10, 60))
        else:
            text(frame, "saved %s   r: redo  q: quit" % os.path.basename(a.out), (10, 30), (0, 255, 0), 0.8)
            if warn:
                text(frame, "WARNING: click order? see terminal", (10, 60), (0, 0, 255), 0.8)
        k = show(win, frame, st)
        if k == ord("q"):
            break
        if k == ord("u") and H is None and clicks:
            clicks.pop()
            print("마지막 점을 지웠다.")
            say_next()
        if k == ord("r"):
            clicks, H, warn = [], None, []
            print("처음부터 다시 찍는다.")
            say_next()
    close()
    cv2.destroyAllWindows()
    if H is not None:
        print("끝. H 파일: %s" % os.path.abspath(a.out))
    elif saved:
        print("끝. 새로 다 찍지 않아서 파일은 그 전에 저장한 것 그대로다: %s" % os.path.abspath(a.out))
    else:
        print("끝. H 를 만들지 않아서 저장하지 않았다.")
    return 0


# ---------- check ----------

def cmd_check(a):
    need_cv2()
    cal = load_calib(a.h)
    H = cal["H"]
    truth = parse_pts(a.truth, "--truth") if a.truth else []
    width = a.width or int(cal.get("width") or 1280)
    height = a.height or int(cal.get("height") or 720)
    grab, close, _ = open_source(a, width, height)
    win = "webcam_calib check (q: quit)"
    st = {"new": [], "mouse": None, "scale": 1.0}
    make_window(win, st)
    log = os.path.join(os.getcwd(), "webcam_check.csv")
    new = not os.path.exists(log)
    f = open(log, "a", newline="", encoding="utf-8")
    wr = csv.writer(f)
    if new:
        wr.writerow(["시각", "점번호", "클릭px_x", "클릭px_y", "계산x_mm", "계산y_mm", "실제x_mm", "실제y_mm", "오차mm"])
    pts, errs, size_ok = [], [], None
    print("H 파일: %s (calib %s, %sx%s)" % (os.path.abspath(a.h), cal.get("saved_at", "?"), cal.get("width"),
                                           cal.get("height")))
    if truth:
        print("실제 좌표를 아는 점 %d개를 차례로 클릭한다. 첫 점: %s mm" % (len(truth), fmt(truth[0])))
    else:
        print("아무 곳이나 클릭하면 책상 좌표(mm)를 보여 준다. 오차를 재려면 --truth 를 준다.")
    while True:
        frame = grab()
        if size_ok is None:
            size_ok = (frame.shape[1], frame.shape[0]) == (cal.get("width"), cal.get("height"))
            if not size_ok:
                print("[주의] 화면 크기 %dx%d 가 calib 때(%sx%s)와 다르다. 좌표가 틀린다. --width --height 를 calib 때와 같게 준다."
                      % (frame.shape[1], frame.shape[0], cal.get("width"), cal.get("height")))
        while st["new"]:
            u, v = st["new"].pop(0)
            x, y = px_to_mm(H, u, v)
            k = len(pts) + 1
            row = [datetime.datetime.now().strftime("%H:%M:%S"), k, u, v, round(x, 1), round(y, 1), "", "", ""]
            msg = "점 %d: 픽셀 (%d, %d) -> (%.1f, %.1f) mm" % (k, u, v, x, y)
            if k <= len(truth):
                tx, ty = truth[k - 1]
                d = math.hypot(x - tx, y - ty)
                errs.append(d)
                row[6:] = [tx, ty, round(d, 1)]
                msg += " | 실제 %s | 오차 dx %+.1f, dy %+.1f, 거리 %.1f mm" % (fmt((tx, ty)), x - tx, y - ty, d)
            print(msg)
            wr.writerow(row)
            f.flush()
            pts.append((u, v, x, y))
            if truth and k == len(truth):
                print("[결과] %d점 오차: 최대 %.1f mm, 평균 %.1f mm (기록: %s)"
                      % (len(errs), max(errs), sum(errs) / len(errs), log))
            elif k < len(truth):
                print("  다음: %d/%d %s mm" % (k + 1, len(truth), fmt(truth[k])))
        draw_grid(frame, H)
        for u, v, x, y in pts:
            cv2.circle(frame, (u, v), 6, (0, 0, 255), -1)
            text(frame, "%.0f,%.0f" % (x, y), (u + 8, v - 8), (0, 0, 255))
        if len(pts) < len(truth):
            text(frame, "%d/%d: click tape cross at %s mm" % (len(pts) + 1, len(truth), fmt(truth[len(pts)])),
                 (10, 30), (0, 255, 255), 0.8)
        elif errs:
            text(frame, "max %.1f mm  mean %.1f mm" % (max(errs), sum(errs) / len(errs)), (10, 30), (0, 255, 0), 0.8)
        if st["mouse"]:
            mx, my = px_to_mm(H, *st["mouse"])
            text(frame, "mouse %.0f, %.0f mm   q: quit" % (mx, my), (10, frame.shape[0] - 15))
        if show(win, frame, st) == ord("q"):
            break
    close()
    cv2.destroyAllWindows()
    f.close()
    print("끝. 기록: %s" % log)
    return 0


# ---------- selftest ----------

def synth_camera(cam, look, focal=900.0, size=(1280, 720)):
    """가상 핀홀 카메라. 점 (x, y, z) mm -> 픽셀 (u, v) 함수를 돌려준다."""
    c = np.asarray(cam, dtype=np.float64)
    fwd = np.asarray(look, dtype=np.float64) - c
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, (0.0, 0.0, 1.0))
    right /= np.linalg.norm(right)
    rot = np.array([right, np.cross(fwd, right), fwd])   # 카메라 x = 오른쪽, y = 아래, z = 앞

    def project(x, y, z=0.0):
        X, Y, Z = rot @ (np.array([x, y, z], dtype=np.float64) - c)
        return focal * X / Z + size[0] / 2.0, focal * Y / Z + size[1] / 2.0
    return project


def cmd_selftest(a):
    cam, look, size = (450.0, -1100.0, 1400.0), (450.0, 0.0, 0.0), (1280, 720)
    proj = synth_camera(cam, look, 900.0, size)
    mm = parse_pts(DEFAULT_PTS, "pts")
    px = [proj(x, y) for x, y in mm]
    ok = True
    down = math.degrees(math.atan2(cam[2], look[1] - cam[1]))
    print("가상 카메라: 위치 (450, -1100, 1400) mm, (450, 0, 0) 을 봄 (아래로 %.0f도), f = 900 px, %dx%d" % (down, *size))
    print("기본 4점의 픽셀: " + ", ".join("(%.0f, %.0f)" % p for p in px))
    if not all(0 <= u < size[0] and 0 <= v < size[1] for u, v in px):
        print("[FAIL] 기본 4점이 화면 밖에 있다")
        ok = False
    H = fit_h(px, mm)
    if H is None:
        print("[FAIL] H 를 못 구했다")
        return 1
    print("H 계산: %s" % ("cv2.findHomography" if cv2 is not None else "numpy DLT (cv2 가 없어서 대신 씀)"))
    worst = 0.0
    for x, y in parse_pts(DEFAULT_TRUTH, "truth"):
        u, v = proj(x, y)
        ex, ey = px_to_mm(H, u, v)
        bu, bv = mm_to_px(H, ex, ey)
        worst = max(worst, math.hypot(ex - x, ey - y), math.hypot(bu - u, bv - v))
    print("확인 4점 왕복 오차: 최대 %.4f mm (기준 0.5 미만) -> %s" % (worst, "OK" if worst < 0.5 else "FAIL"))
    ok &= worst < 0.5
    if cv2 is not None:
        h2 = fit_h_numpy(np.array(px), np.array(mm, dtype=np.float64))
        h2 /= h2[2, 2]
        diff = max(math.hypot(*np.subtract(px_to_mm(H, u, v), px_to_mm(h2, u, v))) for u, v in px)
        print("numpy 대체 계산과 차이: 최대 %.4f mm -> %s" % (diff, "OK" if diff < 0.5 else "FAIL"))
        ok &= diff < 0.5
    order_ok = not h_warnings(H, px)
    for order in ((0, 2, 1, 3), (0, 3, 2, 1)):   # 2, 3번을 바꿔 찍음(꼬임), 거꾸로 돌아가며 찍음(거울)
        wrong = [px[i] for i in order]
        hw = fit_h(wrong, mm)
        order_ok = order_ok and hw is not None and bool(h_warnings(hw, wrong))
    print("클릭 순서 실수 경고 (꼬임, 거울): %s" % ("OK" if order_ok else "FAIL"))
    ok &= order_ok
    u, v = proj(*WORK)
    sx = math.hypot(*np.subtract(px_to_mm(H, u + 1, v), px_to_mm(H, u, v)))
    sy = math.hypot(*np.subtract(px_to_mm(H, u, v + 1), px_to_mm(H, u, v)))
    print("조립 영역 (450, 0) 에서 화면 1 px = 가로 %.2f mm, 세로 %.2f mm (클릭 1 px 실수의 크기)" % (sx, sy))
    print("[높이 영향] 책상 위로 떠 있는 점을 책상 H 로 바꾸면 카메라에서 먼 쪽(+y)으로 밀려 보인다:")
    dist, camh = look[1] - cam[1], cam[2]
    for hgt in (100.0, 200.0):
        ex, ey = px_to_mm(H, *proj(WORK[0], WORK[1], hgt))
        expect = hgt * dist / (camh - hgt)   # 닮은 삼각형으로 구한 값
        good = abs(ex - WORK[0]) < 1.0 and abs(ey - expect) < 1.0 and ey > 0
        print("  (450, 0) 위 %3.0f mm -> (%.1f, %.1f) mm 로 보임: +y 로 %.1f mm 밀림 = 높이의 %.2f배 (식 %.1f mm) %s"
              % (hgt, ex, ey, ey, ey / hgt, expect, "OK" if good else "FAIL"))
        ok &= good
    print("  높이가 작을 때는 약 %.2f배 (= 카메라까지 수평 거리 %.0f / 카메라 높이 %.0f). 높을수록 더 커진다."
          % (dist / camh, dist, camh))
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="웹캠 픽셀 -> 책상 좌표(mm) 맞추기")
    sub = ap.add_subparsers(dest="cmd")
    c = sub.add_parser("calib", help="테이프 점을 클릭해서 H 를 만든다")
    c.add_argument("--pts", default=DEFAULT_PTS,
                   help='클릭할 테이프 점의 로봇 좌표 (mm, "x,y;x,y;..." 4점 이상, 기본 "%(default)s")')
    c.add_argument("--cam", type=int, default=0, help="웹캠 번호 (기본 0)")
    c.add_argument("--width", type=int, default=1280, help="화면 가로 픽셀 (기본 1280)")
    c.add_argument("--height", type=int, default=720, help="화면 세로 픽셀 (기본 720)")
    c.add_argument("--image", default="", help="웹캠 대신 사진 파일")
    c.add_argument("--out", default="webcam_H.json", help="저장할 파일 (기본 webcam_H.json)")
    k = sub.add_parser("check", help="다른 테이프 점을 클릭해서 오차를 잰다")
    k.add_argument("--truth", default="",
                   help='실제 좌표 (mm, "x,y;..."). 주면 k번째 클릭과 비교한다. 예: "%s"' % DEFAULT_TRUTH)
    k.add_argument("--h", default="webcam_H.json", help="calib 가 만든 파일 (기본 webcam_H.json)")
    k.add_argument("--cam", type=int, default=0, help="웹캠 번호 (기본 0)")
    k.add_argument("--width", type=int, default=0, help="화면 가로 픽셀 (기본: calib 때 값)")
    k.add_argument("--height", type=int, default=0, help="화면 세로 픽셀 (기본: calib 때 값)")
    k.add_argument("--image", default="", help="웹캠 대신 사진 파일")
    sub.add_parser("selftest", help="카메라 없이 계산만 시험한다")
    a = ap.parse_args()
    if a.cmd == "calib":
        return cmd_calib(a)
    if a.cmd == "check":
        return cmd_check(a)
    if a.cmd == "selftest":
        return cmd_selftest(a)
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())

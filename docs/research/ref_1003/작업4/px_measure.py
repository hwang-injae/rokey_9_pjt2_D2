"""V-17 사진에서 거리 재기. 사진을 클릭하면 점을 찍고, 두 번 찍을 때마다 두 점 사이 거리를 출력한다.

실행: python3 px_measure.py 사진.jpg                    # 픽셀로
      python3 px_measure.py 사진.jpg --mm-per-px 0.25   # mm 도 같이
키: r = 점 지우기, q = 끝. 큰 사진은 화면에 맞게 줄여 보여 주지만, 출력 좌표와 거리는 원래 사진 기준이다.
쓰는 법(V-17): 첫 사진에서 블록 긴 변 양 끝을 찍어 '75 / 픽셀 거리' 로 mm/픽셀을 구한다.
              사진마다 움직이지 않은 기준 모서리 → 새로 놓은 블록의 같은 모서리 순서로 찍고 dx(좌우)를 비교한다.
"""
import argparse
import math

import cv2


class Measure:
    def __init__(self, scale, mm_per_px):
        self.scale, self.mm, self.pts = scale, mm_per_px, []

    def click(self, x, y):
        """화면 좌표 -> 원래 사진 좌표로 바꿔 점을 더한다. 두 번째 점마다 거리 글자를 돌려준다."""
        p = (x / self.scale, y / self.scale)
        self.pts.append(p)
        msg = "점 %d: (%.1f, %.1f)" % (len(self.pts), p[0], p[1])
        if len(self.pts) % 2 == 0:
            a, b = self.pts[-2], self.pts[-1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            d = math.hypot(dx, dy)
            msg += " | dx %.1f px, dy %.1f px, 거리 %.1f px" % (dx, dy, d)
            if self.mm:
                msg += " = dx %.2f mm, dy %.2f mm, 거리 %.2f mm" % (dx * self.mm, dy * self.mm, d * self.mm)
        return msg


def main():
    ap = argparse.ArgumentParser(description="V-17 사진 거리 재기")
    ap.add_argument("image")
    ap.add_argument("--mm-per-px", type=float, default=0.0)
    a = ap.parse_args()
    img = cv2.imread(a.image)
    if img is None:
        raise SystemExit("사진을 열 수 없다: %s" % a.image)
    scale = min(1.0, 1400.0 / img.shape[1], 900.0 / img.shape[0])
    view = cv2.resize(img, None, fx=scale, fy=scale) if scale < 1.0 else img.copy()
    m = Measure(scale, a.mm_per_px)
    shown = view.copy()

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(m.click(x, y))
            cv2.circle(shown, (x, y), 4, (0, 0, 255), -1)
            if len(m.pts) % 2 == 0:
                a_, b_ = m.pts[-2], m.pts[-1]
                cv2.line(shown, (int(a_[0] * scale), int(a_[1] * scale)), (int(b_[0] * scale), int(b_[1] * scale)),
                         (0, 255, 255), 1)

    win = "px_measure (r: reset, q: quit)"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    while True:
        cv2.imshow(win, shown)
        k = cv2.waitKey(20) & 0xFF
        if k == ord("q"):
            break
        if k == ord("r"):
            m.pts.clear()
            shown[:] = view
            print("점을 지웠다")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

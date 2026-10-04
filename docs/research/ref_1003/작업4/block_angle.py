"""V-05 블록 찾기와 방향 재기. 위에서 찍은 사진 한 장에서 블록마다 중심과 긴 변 각도를 구한다 (OpenCV minAreaRect).

준비: 무광 검은 종이 위에 블록(밝은 나무색)을 서로 닿지 않게 놓는다. 카메라는 위에서 수직으로 내려다보게 고정한다.
      블록 10개는 2줄(줄마다 5개)로 놓으면 번호가 '위 줄 왼쪽부터' 붙는다.
실행: python3 block_angle.py 사진.jpg --rows 2
      python3 block_angle.py --cam 0 --rows 2          # 웹캠으로 한 장 찍어서 (cam_shot.jpg 로 저장)
      python3 block_angle.py 사진.jpg --rows 2 --true 0,15,30,45,60,90,105,120,135,170
결과: 블록마다 번호·중심·긴 변 각도를 출력하고, 번호와 각도를 그린 <사진>_result.jpg 를 저장한다.
각도: 사진 오른쪽이 0°, 반시계 방향이 + (0~180°). 위에서 내려다본 각도기와 같은 방향이다.
--true: 각도기로 놓은 각도(번호 순서). 1번 블록을 0°로 놓고, 나머지는 1번과의 각도 차이로 비교한다.
"""
import argparse
import math
import os

import cv2


def find_blocks(img, min_area_ratio=0.002):
    gray = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)   # 밝은 블록 / 어두운 바닥
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = min_area_ratio * img.shape[0] * img.shape[1]
    out = []
    for c in contours:
        if cv2.contourArea(c) < min_area:
            continue
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        e1, e2 = box[1] - box[0], box[2] - box[1]
        e = e1 if math.hypot(*e1) >= math.hypot(*e2) else e2      # 긴 변
        ang = math.degrees(math.atan2(-e[1], e[0])) % 180.0        # 사진 y축은 아래쪽이라 부호를 바꾼다
        long_px, short_px = max(rect[1]), min(rect[1])
        out.append({"cx": rect[0][0], "cy": rect[0][1], "angle": ang, "long": long_px, "short": short_px,
                    "box": box.astype(int)})
    return out


def number_blocks(blocks, rows):
    """위 줄부터, 줄 안에서는 왼쪽부터 번호를 붙인다."""
    blocks = sorted(blocks, key=lambda b: b["cy"])
    per = math.ceil(len(blocks) / rows) if blocks else 0
    out = []
    for r in range(rows):
        out += sorted(blocks[r * per:(r + 1) * per], key=lambda b: b["cx"])
    return out


def main():
    ap = argparse.ArgumentParser(description="V-05 블록 방향")
    ap.add_argument("image", nargs="?", default="")
    ap.add_argument("--cam", type=int, default=-1, help="사진 대신 웹캠 번호로 한 장 찍기")
    ap.add_argument("--rows", type=int, default=1, help="블록을 놓은 줄 수 (번호 붙이기용)")
    ap.add_argument("--true", default="", help="각도기 각도, 쉼표로 (번호 순서)")
    a = ap.parse_args()
    if a.cam >= 0:
        cap = cv2.VideoCapture(a.cam)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        for _ in range(10):          # 노출이 자리 잡게 몇 장 버린다
            ok, img = cap.read()
        cap.release()
        if not ok:
            raise SystemExit("웹캠에서 사진을 못 찍었다")
        a.image = "cam_shot.jpg"
        cv2.imwrite(a.image, img)
    else:
        img = cv2.imread(a.image)
        if img is None:
            raise SystemExit("사진을 열 수 없다: %s" % a.image)
    blocks = number_blocks(find_blocks(img), max(1, a.rows))
    print("찾은 블록: %d개 (사진 %dx%d)" % (len(blocks), img.shape[1], img.shape[0]))
    true = [float(v) for v in a.true.split(",")] if a.true else []
    errs = []
    for k, b in enumerate(blocks, 1):
        line = "%2d번: 중심 (%4.0f, %4.0f) px, 긴 변 %4.0f px, 짧은 변 %4.0f px, 각도 %6.1f°" % (
            k, b["cx"], b["cy"], b["long"], b["short"], b["angle"])
        if true and k <= len(true):
            meas = (b["angle"] - blocks[0]["angle"]) % 180.0     # 1번 블록과의 차이
            err = (meas - (true[k - 1] - true[0]) + 90.0) % 180.0 - 90.0
            errs.append(abs(err))
            line += " | 1번과 차이 %6.1f°, 각도기 %6.1f°, 오차 %+5.1f°" % (meas, true[k - 1] - true[0], err)
        print(line)
        cv2.drawContours(img, [b["box"]], 0, (0, 0, 255), 2)
        cv2.putText(img, "%d: %.0f" % (k, b["angle"]), (int(b["cx"]) - 30, int(b["cy"]) + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    if errs:
        print("최대 각도 오차: %.1f° (기준 ±5°)" % max(errs))
    if true and len(true) != len(blocks):
        print("주의: 각도기 값 %d개, 찾은 블록 %d개 — 개수가 다르다" % (len(true), len(blocks)))
    root, _ = os.path.splitext(a.image)
    cv2.imwrite(root + "_result.jpg", img)
    print("결과 사진:", root + "_result.jpg")


if __name__ == "__main__":
    main()

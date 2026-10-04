"""캘리브레이션 품질 점검 (로봇 안 움직임).

체커보드는 책상에 고정돼 있으므로, 사진마다 계산한 '베이스 기준 체커보드 위치'는
이상적으로 전부 같아야 한다. 값이 흩어질수록 T_gripper2camera.npy 가 부정확하다.

사용 (Calibration_Tutorial 폴더에서, handeye_calibration.py 실행 후):
    python3 check_calib.py            # 체커보드 내부 코너 8x6 (handeye 기본값)
    python3 check_calib.py 7 10       # 다른 크기로 시험

10/3 작업 2 V-15용으로 덧붙인 것: 맨 끝에 코너 5곳(네 귀퉁이 + 가운데)의 로봇 좌표를 낸다.
그리퍼를 끝까지 닫은 손끝으로 그 코너를 짚어 읽은 posx와 x·y를 비교한다.
"""
import sys
import json

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

cols, rows = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) == 3 else (8, 6)
SQUARE = 25.0  # mm, handeye_calibration.py 의 square_size 와 같아야 함
board = (cols, rows)

data = json.load(open("data/calibrate_data.json"))
poses, files = data["poses"], data["file_name"]
T_g2c = np.load("T_gripper2camera.npy")

objp = np.zeros((cols * rows, 3), np.float32)
objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * SQUARE


def pose_mat(x, y, z, rx, ry, rz):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


# 1) 코너 검출 + 카메라 내부 파라미터
imgpts, objpts, ok_idx, shape = [], [], [], None
for i, f in enumerate(files):
    img = cv2.imread("data/" + f)
    if img is None:
        print("읽기 실패:", f)
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    shape = gray.shape[::-1]
    found, c = cv2.findChessboardCorners(gray, board, None)
    if not found:
        continue
    c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    imgpts.append(c)
    objpts.append(objp)
    ok_idx.append(i)

print(f"코너 검출: {len(ok_idx)}/{len(files)}장 (board={board})")
if len(ok_idx) < 5:
    sys.exit("코너를 찾은 사진이 너무 적음 → checkerboard_size(내부 코너 수) 확인")

rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpts, imgpts, shape, None, None)
print(f"이미지 크기 {shape}, 재투영 오차 RMS {rms:.3f}px (1px 이하 권장)")

# 2) 사진마다 베이스 → 체커보드 원점(첫 내부 코너) 위치
pts, T_list = [], []
for k, i in enumerate(ok_idx):
    R, _ = cv2.Rodrigues(rvecs[k])
    T_c2t = np.eye(4)
    T_c2t[:3, :3] = R
    T_c2t[:3, 3] = tvecs[k].ravel()
    T_b2t = pose_mat(*poses[i][:6]) @ T_g2c @ T_c2t
    pts.append(T_b2t[:3, 3])
    T_list.append(T_b2t)
    print(f"{files[i]:>45s}  체커보드 원점(base) = {np.round(T_b2t[:3, 3], 1)}")

pts = np.array(pts)
print("평균 (mm):    ", np.round(pts.mean(0), 1))
print("표준편차 (mm):", np.round(pts.std(0), 1), " ← 축마다 3mm 이하 양호 / 10mm 이상이면 데이터·설정 불량")
print("→ TCP 손끝을 체커보드 첫 내부 코너에 직접 대고 읽은 posx 와 위 '평균'을 비교하세요."
      " z 가 N mm 낮게 나오면 verify 도 N mm 더 내려갑니다.")

# 3) (V-15) 코너 5곳의 로봇 좌표: 사진마다 구해 평균. 손끝으로 짚은 posx와 비교한다
corners = [(0, 0), (cols - 1, 0), (0, rows - 1), (cols - 1, rows - 1), (cols // 2, rows // 2)]
print("\n[V-15] 코너 5곳 (열, 행) — 카메라로 구한 로봇 좌표 평균 (mm)")
print("  출력된 x·y에 가장 가까운 체커보드 코너(검은 칸 꼭짓점이 만나는 점)를 손끝으로 짚는다.")
for n, (ci, ri) in enumerate(corners, start=1):
    p_board = np.array([ci * SQUARE, ri * SQUARE, 0.0, 1.0])
    pc = np.array([(T @ p_board)[:3] for T in T_list])
    m, sd = pc.mean(0), pc.std(0)
    print(f"  코너 {n} ({ci},{ri}): x {m[0]:7.1f}  y {m[1]:7.1f}  z {m[2]:7.1f}   사진 간 흩어짐 {np.round(sd, 1)}")

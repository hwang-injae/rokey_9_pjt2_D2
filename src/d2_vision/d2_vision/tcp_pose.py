# -*- coding: utf-8 -*-
"""손목 카메라 자세 = MoveIt TF base_link → rg2_tcp × 손목 보정값(rg2_tcp 틀) — wrist_block · capture_scene · check_wrist_calib 가 같이 쓴다.

10/9 E-70 ③(PL): 손목 보정 · 촬영 · 인식 때 로봇 자세를 펜던트 posx 가 아니라 MoveIt TF(frame_id → tcp_link, robot.yaml)로 읽는다.
보정값도 같은 틀이다 — config/T_rg2tcp2camera.npy(카메라 → rg2_tcp 4x4, mm, 10/10 4각도로 다시 계산). 옛 T_gripper2camera.npy(posx 틀)는 지웠다.
두 틀은 10/10 E-81 뒤로 같은 점(손가락 가운데)이고 방향만 손목 Z 로 −90° 다르다(R_rg2tcp = R_posx · Rz(−90°), motion_math 머리말)
→ T_rg2tcp←cam = Rz(+90°) · T_posx←cam(POSX_TO_RG2TCP).
카메라 자세는 camera_pose_mm 한 곳에서 낸다 — TF × 보정값 + 기울기 밀림 고침(10/10 E-83, 보정 json 의 tilt_shift).
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

CALIB_NAME = 'T_rg2tcp2camera.npy'
GRAVITY = np.array([0.0, 0.0, -1.0])     # base_link 에서 중력 방향(단위)
# rg2_tcp 틀에서 본 posx 틀(같은 원점, z 축으로 +90°) — posx 틀 보정값을 rg2_tcp 틀로 옮길 때 왼쪽에 곱한다(거꾸로는 역행렬)
POSX_TO_RG2TCP = np.array([[0.0, -1.0, 0.0, 0.0],
                           [1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0]])


def load_calib(path):
    """보정 파일(.npy, 카메라 → rg2_tcp 4x4, mm)을 읽는다. 못 읽거나 4x4 가 아니면 예외(OSError · ValueError)."""
    T = np.load(path)
    if T.shape != (4, 4):
        raise ValueError(f'{path}: 4x4 가 아니다 {T.shape}')
    return T


def default_calib_path():
    """설치된 d2_vision share/config 의 보정 파일, 없으면 소스 트리(config/) — 소스에서 바로 돌리는 check_wrist_calib 용."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory('d2_vision')) / 'config' / CALIB_NAME
        if p.exists():
            return p
    except Exception:                      # 패키지가 설치 안 된 PC — 소스 트리로
        pass
    return Path(__file__).resolve().parents[1] / 'config' / CALIB_NAME


def load_tilt_shift(calib_path):
    """보정 파일 옆 json(같은 이름 .json)의 tilt_shift.mm_per_g(mm) — 없거나 못 읽으면 0.0(밀림 고침 없음, 옛 판과 같음)."""
    try:
        j = json.loads(Path(calib_path).with_suffix('.json').read_text(encoding='utf-8'))
        return float(j.get('tilt_shift', {}).get('mm_per_g', 0.0))
    except (OSError, ValueError, TypeError, AttributeError):
        return 0.0


def camera_pose_mm(T_b2t, T_tcp2c, tilt_shift_mm=0.0):
    """카메라 자세(base_link 기준 4x4 mm) = TF(base ← rg2_tcp) × 보정값(rg2_tcp ← 카메라) + 기울기 밀림 고침.

    10/10 E-83: 카메라를 기울이면 잰 점이 카메라에서 먼 쪽 · 위로 밀린다(방향은 맞음 — 작업대 평면은 수평으로 나옴, 30°에서 약 10 mm).
    15자세 실측으로 '중력 중 광축에 수직인 성분 × tilt_shift_mm' 만큼 카메라 위치를 옮기면 맞는다(남는 오차 평균 2 mm — 보정 json tilt_shift).
    내려다보는 자세(광축 ∥ 중력)는 성분이 0 이라 고침도 0 — 손목 보정(수직 4각도)은 그대로다.
    입력: T_b2t · T_tcp2c 4x4(mm), tilt_shift_mm(load_tilt_shift). 출력: 새 4x4(입력은 안 바꿈).
    """
    T = T_b2t @ T_tcp2c
    if tilt_shift_mm:
        z = T[:3, 2]
        T[:3, 3] += tilt_shift_mm * (GRAVITY - z * (GRAVITY @ z))
    return T


def transform_to_matrix_mm(t):
    """geometry_msgs/TransformStamped(m) → 4x4 (mm). 번역은 1000 배, 회전은 그대로."""
    q = t.transform.rotation
    p = t.transform.translation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x * 1000.0, p.y * 1000.0, p.z * 1000.0]
    return T


class TcpPose:
    """노드 하나에 붙는 TF 읽기 — frame_id(base_link) 기준 tcp_link(rg2_tcp) 자세를 4x4 mm 로.

    /tf 구독은 리스너의 재진입 콜백 그룹으로 이 노드의 실행기가 돌린다 — spin_thread 는 쓰지 않는다(노드를 다른 실행기로 옮겨 버림).
    그래서 기다리며 부르는 쪽은 다른 스레드여야 한다: wrist_block = 멀티스레드 실행기 콜백, capture_scene = 실행기를 뒤 스레드로 돌린 앞 스레드,
    check_wrist_calib = spin_once 로 /tf 를 받은 뒤 기다림 없이(timeout_s=0) 묻는다.
    바깥 영향: /tf · /tf_static 구독뿐. 실패 때(브링업 없음 · 시간 초과) None.
    """

    def __init__(self, node, frame_id='base_link', tcp_link='rg2_tcp'):
        """node 에 tf2 버퍼 · 리스너를 만든다. frame_id · tcp_link 는 robot.yaml 값."""
        from tf2_ros import Buffer, TransformListener
        self.frame_id, self.tcp_link = frame_id, tcp_link
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)

    def matrix_mm(self, timeout_s=3.0):
        """지금 base_link → rg2_tcp 4x4 (mm). timeout_s 안에 못 받으면 None(브링업 · robot_state_publisher 없음)."""
        from rclpy.duration import Duration
        from rclpy.time import Time
        try:
            t = self.buffer.lookup_transform(self.frame_id, self.tcp_link, Time(), timeout=Duration(seconds=timeout_s))
        except Exception:                  # LookupException · ExtrapolationException · 시간 초과 — 모두 '못 받음'
            return None
        return transform_to_matrix_mm(t)

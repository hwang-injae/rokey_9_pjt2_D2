"""tcp_pose(10/10 W156) — posx 틀 보정값을 rg2_tcp 틀로 옮기는 행렬과 TF → 4x4 변환. ROS 없이 돈다."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from d2_vision.tcp_pose import CALIB_NAME, POSX_TO_RG2TCP, camera_pose_mm, load_calib, load_tilt_shift, transform_to_matrix_mm

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def _pose(rng):
    """무작위 자세 4x4 (mm)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.random(random_state=rng).as_matrix()
    T[:3, 3] = rng.uniform(-500, 500, 3)
    return T


def test_posx_길과_tf_길의_카메라_자세가_같다():
    """R_rg2tcp = R_posx · Rz(−90°)(같은 원점, motion_math 머리말)일 때 T_b←posx @ T_posx←cam == T_b←rg2 @ (POSX_TO_RG2TCP @ T_posx←cam)."""
    rng = np.random.default_rng(0)
    rz_m90 = np.eye(4)
    rz_m90[:3, :3] = Rotation.from_euler('z', -90, degrees=True).as_matrix()
    T_posx2c = _pose(rng)
    for _ in range(50):
        T_b2posx = _pose(rng)
        T_b2rg2 = T_b2posx @ rz_m90
        assert np.allclose(T_b2posx @ T_posx2c, T_b2rg2 @ (POSX_TO_RG2TCP @ T_posx2c), atol=1e-9)


def test_축_약속_x_rg2_는_마이너스_y_posx():
    """x_rg2 = −y_posx, y_rg2 = +x_posx (motion_math 머리말)."""
    rz_m90 = Rotation.from_euler('z', -90, degrees=True).as_matrix()
    assert np.allclose(rz_m90[:, 0], [0, -1, 0]) and np.allclose(rz_m90[:, 1], [1, 0, 0])
    assert np.allclose(POSX_TO_RG2TCP[:3, :3] @ rz_m90, np.eye(3))


def test_config_보정값은_rg2tcp_틀_하나뿐이고_강체_변환():
    """10/10 W156 다시 계산 뒤: config 에 T_rg2tcp2camera 하나(옛 posx 틀 T_gripper2camera 는 지움) · 회전은 직교 · det 1 ·
    카메라는 rg2_tcp 위 약 184 mm · 옆 약 80 mm(손목 옆에 붙은 자리) — 틀을 잘못 넣으면(posx 틀 그대로) x · y 가 뒤바뀐다."""
    T = load_calib(CONFIG / CALIB_NAME)
    assert not (CONFIG / 'T_gripper2camera.npy').exists()
    R = T[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-6) and abs(np.linalg.det(R) - 1.0) < 1e-6
    assert np.allclose(T[3], [0, 0, 0, 1])
    assert -190 < T[2, 3] < -175 and -80 < T[0, 3] < -65 and 30 < T[1, 3] < 40


def test_transform_to_matrix_mm():
    """TransformStamped(m, 쿼터니언) → 4x4 mm."""
    q = Rotation.from_euler('z', 30, degrees=True).as_quat()
    t = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.4261, y=-0.0725, z=0.0138),
        rotation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3])))
    T = transform_to_matrix_mm(t)
    assert np.allclose(T[:3, 3], [426.1, -72.5, 13.8])
    assert np.allclose(T[:3, :3], Rotation.from_euler('z', 30, degrees=True).as_matrix())


def test_camera_pose_내려다보면_기울기_밀림_고침_없음():
    """광축 ∥ 중력(내려다봄)이면 tilt_shift 가 있어도 TF × 보정값 그대로(10/10 E-83 — 수직 4각도 보정은 그대로)."""
    T_b2t = np.eye(4)
    T_b2t[:3, :3] = Rotation.from_euler('x', 180, degrees=True).as_matrix()   # 공구 z 가 아래
    T_b2t[:3, 3] = [400.0, -70.0, 300.0]
    T_tcp2c = np.eye(4)
    T_tcp2c[:3, 3] = [-73.0, 35.0, -184.0]
    assert np.allclose(camera_pose_mm(T_b2t, T_tcp2c, 21.42), T_b2t @ T_tcp2c)


def test_camera_pose_30도_기울이면_시선에_수직인_아래로_옮김():
    """광축이 수직에서 30° 기울면 카메라 위치 += k × (중력 중 광축에 수직인 성분) — k 21.42 mm 면 (−9.28, 0, −5.36) mm."""
    T_b2t = np.eye(4)
    T_b2t[:3, :3] = Rotation.from_euler('y', 180 - 30, degrees=True).as_matrix()   # 공구 z = (sin30, 0, −cos30)
    T_tcp2c = np.eye(4)
    base = T_b2t @ T_tcp2c
    got = camera_pose_mm(T_b2t, T_tcp2c, 21.42)
    assert np.allclose(got[:3, :3], base[:3, :3])
    assert np.allclose(got[:3, 3] - base[:3, 3], [-21.42 * np.cos(np.radians(30)) * 0.5, 0.0, -21.42 * 0.25], atol=1e-6)


def test_load_tilt_shift_json_없으면_0(tmp_path):
    """보정 json 이 없거나 칸이 없으면 0(옛 판과 같음), 있으면 그 값 — config 의 값은 10/10 15자세 맞춤 21.42 mm."""
    assert load_tilt_shift(tmp_path / 'none.npy') == 0.0
    (tmp_path / 'a.json').write_text('{"file": "a.npy"}', encoding='utf-8')
    assert load_tilt_shift(tmp_path / 'a.npy') == 0.0
    assert abs(load_tilt_shift(CONFIG / CALIB_NAME) - 21.42) < 1e-9

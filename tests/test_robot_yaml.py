"""robot.yaml(설정 정본, IRD 9장) 의 필수 키·형식·값 사이 관계를 지킨다.

노드가 켜진 뒤에야 알게 되는 '키 없음'·'잡기 폭 순서 뒤집힘' 같은 실수를 PR 때 잡는다. 숫자는 파일에서 읽고, 관계만 본다.
"""
import pytest

from conftest import ROBOT_YAML

GRASPS = ('FLAT_SHORT', 'FLAT_LONG', 'EDGE_SHORT', 'EDGE_LONG', 'STAND_SHORT', 'STAND_LONG')   # IRD 2장 잡기 6가지
BLOCK_AXES = ('LENGTH', 'WIDTH', 'THICKNESS')
BLOCK_SIZE_M = (0.075, 0.025, 0.015)                                                            # 젠가 블록 설계 치수


def num(v):
    """숫자(int·float, bool 제외)인지. 입력: 아무 값. 출력: bool."""
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def test_top_level_keys(robot_cfg):
    """frame_id·table_z_m·assembly_area_half_m·grasp_depth_m·speed_scale 이 있고 형식이 맞다 (speed_scale 은 0 초과 1 이하)."""
    assert isinstance(robot_cfg['frame_id'], str) and robot_cfg['frame_id']
    assert num(robot_cfg['table_z_m'])
    assert num(robot_cfg['assembly_area_half_m']) and robot_cfg['assembly_area_half_m'] > 0
    assert num(robot_cfg['grasp_depth_m']) and robot_cfg['grasp_depth_m'] > 0
    assert num(robot_cfg['speed_scale']) and 0 < robot_cfg['speed_scale'] <= 1


def test_table(robot_cfg):
    """table.x_m·y_m 은 [작은 값, 큰 값] 둘씩, base_clear_m 은 양수."""
    t = robot_cfg['table']
    for k in ('x_m', 'y_m'):
        assert len(t[k]) == 2 and all(num(v) for v in t[k]) and t[k][0] < t[k][1], k
    assert num(t['base_clear_m']) and t['base_clear_m'] > 0


def test_assembly_origin(robot_cfg):
    """assembly_origin 에 x_m·y_m·z_m·yaw_deg 숫자가 있다."""
    o = robot_cfg['assembly_origin']
    assert all(num(o[k]) for k in ('x_m', 'y_m', 'z_m', 'yaw_deg'))


def test_block_sizes(robot_cfg):
    """block_size_m 은 [0.075, 0.025, 0.015](내림차순), block_actual_m 은 각각 설계 치수 이하이고 차이가 1 mm 안."""
    size, actual = robot_cfg['block_size_m'], robot_cfg['block_actual_m']
    assert len(size) == 3 and len(actual) == 3
    assert all(abs(a - b) < 1e-9 for a, b in zip(size, BLOCK_SIZE_M)), size
    assert size[0] > size[1] > size[2]
    for a, s in zip(actual, size):
        assert num(a) and a <= s + 1e-9 and s - a <= 0.001 + 1e-9, (a, s)


def test_supply_slots(robot_cfg):
    """공급 칸은 정확히 6개. 칸마다 x_m·y_m·surface_z_m·yaw_deg·block_up·grasp 가 있고 6칸의 잡기가 서로 다르다."""
    slots = robot_cfg['supply_slots']
    assert len(slots) == 6
    for i, st in enumerate(slots, 1):
        assert all(num(st[k]) for k in ('x_m', 'y_m', 'surface_z_m', 'yaw_deg')), f'칸 {i}'
        assert st['block_up'] in BLOCK_AXES, f'칸 {i}'
        assert st['grasp'] in GRASPS, f'칸 {i}'
    assert sorted(st['grasp'] for st in slots) == sorted(GRASPS), '6칸의 잡기는 서로 달라야 한다'


def test_home_pose(robot_cfg):
    """home_pose.joints_deg 는 관절 6개."""
    j = robot_cfg['home_pose']['joints_deg']
    assert len(j) == 6 and all(num(v) for v in j)


def test_finger_and_gripper(robot_cfg):
    """finger.thickness_m·width_m, gripper 의 힘·오프셋·허용·시간·손가락 끝 보정점이 숫자로 있다."""
    f, g = robot_cfg['finger'], robot_cfg['gripper']
    assert num(f['thickness_m']) and f['thickness_m'] > 0
    assert num(f['width_m']) and f['width_m'] > 0
    for k in ('force_n', 'feedback_offset_m', 'command_offset_m', 'check_tolerance_m', 'settle_timeout_s'):
        assert num(g[k]), k
    fh = g['finger_height']
    assert num(fh['touch_tcp_z_m']) and num(fh['touch_display_width_m'])


@pytest.mark.parametrize('key', ('grasp_width_m', 'grasp_open_pick_m', 'grasp_open_place_m', 'grasp_close_m'))
def test_grasp_tables_have_all_six(robot_cfg, key):
    """잡기 폭 표 4개 각각에 잡기 6가지 키가 모두 숫자로 있다."""
    table = robot_cfg[key]
    assert set(table) == set(GRASPS), f'{key}: {sorted(set(GRASPS) ^ set(table))}'
    assert all(num(v) and v > 0 for v in table.values())


@pytest.mark.parametrize('grasp', GRASPS)
def test_grasp_width_order(robot_cfg, grasp):
    """잡기마다 닫기 명령 < 블록 폭 < 놓을 때 열림 <= 집을 때 열림 (좁게 열면 블록에 걸리고, 닫기가 더 크면 힘이 안 걸린다)."""
    c = robot_cfg
    close, width = c['grasp_close_m'][grasp], c['grasp_width_m'][grasp]
    open_place, open_pick = c['grasp_open_place_m'][grasp], c['grasp_open_pick_m'][grasp]
    assert close < width < open_place <= open_pick, (grasp, close, width, open_place, open_pick)


def test_motion_and_stop(robot_cfg):
    """motion.approach_m·transit_clearance_m·pick_up_m·place_up_m 과 stop.first_wait_s 가 숫자로 있다."""
    m = robot_cfg['motion']
    for k in ('approach_m', 'transit_clearance_m', 'pick_up_m', 'place_up_m'):
        assert num(m[k]) and m[k] >= 0, k
    assert num(robot_cfg['stop']['first_wait_s']) and robot_cfg['stop']['first_wait_s'] > 0


def test_no_personal_paths():
    """robot.yaml 에 개인 절대 경로(/home/)가 없다 (팀 규칙 — 경로는 패키지 share 로)."""
    assert '/home/' not in ROBOT_YAML.read_text(encoding='utf-8')

"""robot.yaml(설정 정본, IRD 9장) 의 필수 키·형식·값 사이 관계를 지킨다.

노드가 켜진 뒤에야 알게 되는 '키 없음'·'잡기 폭 순서 뒤집힘' 같은 실수를 PR 때 잡는다. 숫자는 파일에서 읽고, 관계만 본다.
"""
import pytest
import yaml

from conftest import ROBOT_YAML

GRASPS = ('FLAT_SHORT', 'FLAT_LONG', 'EDGE_SHORT', 'EDGE_LONG', 'STAND_SHORT', 'STAND_LONG')   # IRD 2장 잡기 6가지
BLOCK_AXES = ('LENGTH', 'WIDTH', 'THICKNESS')
BLOCK_SIZE_M = (0.075, 0.025, 0.015)                                                            # 젠가 블록 설계 치수


def duplicate_keys(text):
    """YAML 글에서 같은 덩어리 안에 두 번 나온 키를 찾는다. 입력: YAML 글자. 출력: ['find (57행)', …] — 없으면 빈 목록.

    PyYAML(yaml.safe_load)은 같은 키가 두 번 나오면 오류 없이 뒤의 값으로 덮어써, 앞 덩어리의 키가 소리 없이 사라진다
    (10/7 PR #53 — 두 사람이 따로 `find:`를 더할 수 있었다). 실패 때: YAML 문법이 틀리면 예외.
    """
    found = []

    class Loader(yaml.SafeLoader):
        """키가 겹치는지 보는 읽개(SafeLoader 와 같고 덩어리 만들기만 바꿈)."""

    def mapping(loader, node, deep=False):
        seen = set()
        for key_node, _ in node.value:
            key = loader.construct_object(key_node, deep=True)
            if key in seen:
                found.append(f'{key} ({key_node.start_mark.line + 1}행)')
            seen.add(key)
        return loader.construct_mapping(node, deep)

    Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    yaml.load(text, Loader=Loader)
    return found


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
    """공급 칸은 정확히 6개. 칸마다 x_m·y_m·yaw_deg·block_up·grasp 가 있고 6칸의 잡기가 서로 다르다."""
    slots = robot_cfg['supply_slots']
    assert len(slots) == 6
    for i, st in enumerate(slots, 1):
        assert all(num(st[k]) for k in ('x_m', 'y_m', 'yaw_deg')), f'칸 {i}'
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


def test_timeouts_inner_shorter_than_outer(robot_cfg):
    """기다리는 시간은 안쪽일수록 짧다: timeout.service_s < timeout.command_s < mqtt.req_timeout_s (두 PC 에 걸친 약속).

    웹(req_timeout_s)이 '시간 초과'라고 보여 준 명령이 로봇에서 뒤늦게 실행되면, 사람이 멈춘 줄 알고 작업 영역에 들어갈 수 있다.
    작업 관리자는 command_s 안에 확정 못 하면 실행하지 않으므로 command_s 가 웹보다 짧아야 하고(10/7 PL E-55),
    설계 꺼내기(get_design, service_s)는 그 확정 안에서 끝나야 한다. 숫자를 바꿔도 이 순서만 지키면 통과한다.
    """
    t, m = robot_cfg['timeout'], robot_cfg['mqtt']
    for v in (t['service_s'], t['command_s'], m['req_timeout_s']):
        assert num(v) and v > 0
    assert t['service_s'] < t['command_s'] < m['req_timeout_s'], (
        f"timeout.service_s {t['service_s']} < timeout.command_s {t['command_s']} < mqtt.req_timeout_s {m['req_timeout_s']} "
        '순서가 깨졌다 — 웹이 시간 초과로 보여 준 명령이 로봇에서 뒤늦게 실행될 수 있다(IRD 9 · 10장)')


def test_alive_signal_survives_one_late_signal(robot_cfg):
    """연결 신호 끊김 기준(mqtt.lost_after_s)은 보내는 주기(mqtt.alive_s)의 2배보다 길다 — 신호 하나가 늦은 것만으로
    끊김(조립 WAIT_HMI · 화면 배너)이 되지 않게(IRD 10.3). 숫자를 바꿔도 이 관계만 지키면 통과한다."""
    m = robot_cfg['mqtt']
    assert num(m['alive_s']) and m['alive_s'] > 0 and num(m['lost_after_s'])
    assert m['lost_after_s'] > 2 * m['alive_s'], (
        f"mqtt.lost_after_s {m['lost_after_s']} 가 alive_s {m['alive_s']} 의 2배 이하 — 신호 하나만 늦어도 끊김으로 본다")


def test_no_personal_paths():
    """robot.yaml 에 개인 절대 경로(/home/)가 없다 (팀 규칙 — 경로는 패키지 share 로)."""
    assert '/home/' not in ROBOT_YAML.read_text(encoding='utf-8')


def test_no_duplicate_keys():
    """robot.yaml 의 같은 덩어리 안에 같은 키가 두 번 없다 — 있으면 앞의 값이 조용히 사라진다. 키를 더할 땐 있는 덩어리(`find:` 등) 아래에 넣는다."""
    assert duplicate_keys(ROBOT_YAML.read_text(encoding='utf-8')) == []


def test_duplicate_key_detector_works():
    """검사 자체가 맨 위(`find` 두 번)와 안쪽(`check.b` 두 번) 겹침을 모두 잡는다."""
    text = 'find:\n  a: 1\ncheck:\n  b: 2\n  b: 3\nfind:\n  c: 4\n'
    assert sorted(duplicate_keys(text)) == ['b (5행)', 'find (6행)']

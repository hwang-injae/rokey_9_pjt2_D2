"""d2_motion.motion_math(ROS 없이 도는 집기·놓기 계산) 의 회전·잡기 이름·공급 칸·TCP·레시피 계산을 고정한다.

실기로 확인한 식(10/4~10/6)이 리팩터링으로 바뀌지 않게 한다. 좌표·칸 자세는 지어내지 않고 robot.yaml 에서 읽는다.
"""
import hashlib
import json
import math

import pytest

from d2_motion import motion_math as mm

RPYS = [(0, 0, 0), (math.pi, 0, 0), (0, math.pi, 0), (0, 0, math.pi),        # quat_from_axes 의 네 갈래를 모두 지나게
        (0.3, -0.7, 1.9), (-2.5, 1.2, -0.4), (math.pi / 2, 0, 0), (0, -math.pi / 2, 0)]
IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
SIZE_M = [0.075, 0.025, 0.015]


def close(a, b, tol=1e-9):
    """두 벡터(또는 숫자)가 tol 안에서 같은지. 입력: 같은 길이의 시퀀스 또는 숫자. 출력: bool."""
    if isinstance(a, (int, float)):
        return abs(a - b) <= tol
    return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b))


@pytest.mark.parametrize('rpy', RPYS)
def test_quaternion_round_trip(rpy):
    """rpy_matrix -> quat_from_axes -> matrix_from_quat 가 원래 행렬로 돌아오고, 쿼터니언 노름 1 · w >= 0."""
    R = mm.rpy_matrix(*rpy)
    q = mm.quat_from_axes(*(mm.column(R, k) for k in range(3)))
    assert close(math.sqrt(sum(v * v for v in q)), 1.0)
    assert q[3] >= 0
    R2 = mm.matrix_from_quat(q)
    assert all(close(R[i], R2[i]) for i in range(3)), (rpy, R, R2)


@pytest.mark.parametrize('psi', [0.0, 0.3, -1.2, math.pi / 2, 2.9])
def test_tcp_quat_down(psi):
    """tcp_quat_down(psi): 접근 축 z 가 (0,0,-1), 닫힘 축 y 가 (cos psi, sin psi, 0)."""
    x, y, z = mm.quat_axes(mm.tcp_quat_down(psi))
    assert close(z, [0.0, 0.0, -1.0])
    assert close(y, [math.cos(psi), math.sin(psi), 0.0])


@pytest.mark.parametrize('a', [i * 0.37 - 7.0 for i in range(40)])
def test_wrap_half(a):
    """wrap_half 는 [-pi/2, pi/2) 안이고 원래 각과 pi 의 배수만큼만 다르다 (닫힘 방향은 180도 대칭)."""
    w = mm.wrap_half(a)
    assert -math.pi / 2 <= w < math.pi / 2
    k = (a - w) / math.pi
    assert close(k, round(k), 1e-9)


def test_grasp_name_flat():
    """눕힘(항등 회전, THICKNESS 가 위): WIDTH 를 끼우면 FLAT_SHORT, LENGTH 면 FLAT_LONG, THICKNESS 는 ValueError."""
    assert mm.up_axis(IDENTITY) == 'THICKNESS'
    assert mm.grasp_name(IDENTITY, 'WIDTH') == 'FLAT_SHORT'
    assert mm.grasp_name(IDENTITY, 'LENGTH') == 'FLAT_LONG'
    with pytest.raises(ValueError):
        mm.grasp_name(IDENTITY, 'THICKNESS')


def test_grasp_name_edge():
    """옆세움(x 축 90도, WIDTH 가 위): THICKNESS -> EDGE_SHORT, LENGTH -> EDGE_LONG."""
    R = mm.rpy_matrix(math.pi / 2, 0, 0)
    assert mm.up_axis(R) == 'WIDTH'
    assert mm.grasp_name(R, 'THICKNESS') == 'EDGE_SHORT'
    assert mm.grasp_name(R, 'LENGTH') == 'EDGE_LONG'


def test_grasp_name_stand():
    """세움(y 축 -90도, LENGTH 가 위): THICKNESS -> STAND_SHORT, WIDTH -> STAND_LONG."""
    R = mm.rpy_matrix(0, -math.pi / 2, 0)
    assert mm.up_axis(R) == 'LENGTH'
    assert mm.grasp_name(R, 'THICKNESS') == 'STAND_SHORT'
    assert mm.grasp_name(R, 'WIDTH') == 'STAND_LONG'


def test_half_height_flat():
    """눕힌 블록의 반높이 = 두께 / 2 = 0.0075 m."""
    assert close(mm.half_height(IDENTITY, SIZE_M), 0.0075)


@pytest.mark.parametrize('slot', [1, 2, 3, 4, 5, 6])
def test_slot_pose_matches_yaml(robot_cfg, slot):
    """실제 robot.yaml 공급 칸: 계산한 위 축 == block_up, 그 칸의 grasp 가 그 자세에서 가능한 잡기, 중심 z = 바닥 + 반높이."""
    st = robot_cfg['supply_slots'][slot - 1]
    center, rot = mm.slot_block_pose(robot_cfg, slot)
    assert mm.up_axis(rot) == st['block_up']
    assert mm.grasp_name(rot, mm.GRASP_AXIS[st['grasp']]) == st['grasp']
    assert close(center[:2], (st['x_m'], st['y_m']))
    half = mm.half_height(rot, robot_cfg['block_actual_m'])
    assert half > 0 and close(center[2], robot_cfg['table_z_m'] + half)


def test_slot_out_of_range(robot_cfg):
    """없는 칸 번호(0, 7)는 ValueError."""
    for slot in (0, len(robot_cfg['supply_slots']) + 1):
        with pytest.raises(ValueError):
            mm.slot_block_pose(robot_cfg, slot)


def test_rg2_tip_height_monotonic():
    """RG2 손가락 끝 높이는 열림 폭 0.01~0.10 m 에서 단조 감소한다 — 손가락이 호를 그려 넓게 열수록 끝이 올라간다(바닥에서 멀어진다).
    75 mm 블록을 문 폭과 다 닫은 폭의 끝 높이 차는 코드 주석의 16.8 mm."""
    widths = [0.01 + 0.005 * i for i in range(19)]                       # 0.010 ~ 0.100
    heights = [mm.rg2_tip_height_m(w) for w in widths]
    assert all(h1 > h2 for h1, h2 in zip(heights, heights[1:])), heights
    drop_mm = (mm.rg2_tip_height_m(0.009) - mm.rg2_tip_height_m(0.075 + 0.0094)) * 1000
    assert abs(drop_mm - 16.8) < 0.1, drop_mm


def _place_center(cfg, rot):
    """조립 원점 위에 rot 자세로 놓인 블록의 중심 (m) — 원점 z + 실측 블록 반높이."""
    o = cfg['assembly_origin']
    return (o['x_m'], o['y_m'], o['z_m'] + mm.half_height(rot, cfg['block_actual_m']))


def test_pick_place_tcp_slot1(robot_cfg):
    """칸 1(FLAT_LONG) 블록을 같은 자세로 조립 원점에 놓는 TCP: 예외 없이 계산되고, 집기·놓기 쿼터니언 z 축이 아래, 집기 x·y = 칸 좌표."""
    st = robot_cfg['supply_slots'][0]
    center, rot = mm.slot_block_pose(robot_cfg, 1)
    pick_xyz, pick_q, place_xyz, place_q, held = mm.pick_place_tcp(
        robot_cfg, center, rot, _place_center(robot_cfg, rot), rot, 'FLAT_LONG', slot=1)
    assert close(mm.quat_axes(pick_q)[2], [0.0, 0.0, -1.0])
    assert close(mm.quat_axes(place_q)[2], [0.0, 0.0, -1.0])
    assert close(pick_xyz[:2], (st['x_m'], st['y_m']))
    assert close(place_xyz[:2], (robot_cfg['assembly_origin']['x_m'], robot_cfg['assembly_origin']['y_m']))
    assert len(held['size_m']) == 3 and len(held['offset_m']) == 3


def test_pick_place_tcp_rejects_other_grasp_in_slot(robot_cfg):
    """공급 칸의 잡기는 하나로 고정 — 칸 1(FLAT_LONG)에 FLAT_SHORT 를 주면 ValueError (10/6 실기)."""
    center, rot = mm.slot_block_pose(robot_cfg, 1)
    with pytest.raises(ValueError):
        mm.pick_place_tcp(robot_cfg, center, rot, _place_center(robot_cfg, rot), rot, 'FLAT_SHORT', slot=1)


def test_pick_place_tcp_rejects_different_up_axis(robot_cfg):
    """공급 자세(칸 3, WIDTH 위)와 놓을 자세(눕힘, THICKNESS 위)의 위 축이 다르면 ValueError — 위에서 집어 그대로 놓으므로."""
    center, rot = mm.slot_block_pose(robot_cfg, 3)
    assert mm.up_axis(rot) != mm.up_axis(IDENTITY)
    with pytest.raises(ValueError):
        mm.pick_place_tcp(robot_cfg, center, rot, _place_center(robot_cfg, IDENTITY), IDENTITY, 'EDGE_LONG', slot=3)


def _recipe(size_mm=(75, 25, 15), model_id='001_CHAIR_BENCH'):
    """E-52 두 파일(조립 · 구조)의 작은 레시피 — 눕힌 블록 두 개를 쌓는다 (SEAT_001_02 가 SEAT_001_01 위).
    반환: (조립, 구조). recipe_blocks 에는 load_recipe 처럼 구조를 'structure' 칸에 붙여 넣는다. steps 는 일부러 순서를 뒤집어 둔다."""
    structure = {'schema': 'cad_structure/1.0', 'model_id': model_id,
                 'parts': [{'part_id': 'PART_001', 'size_mm': list(size_mm)}],
                 'blocks': [{'block': 'SEAT_001_01', 'part_id': 'PART_001', 'center_mm': [0, 0, 7.5], 'R': IDENTITY},
                            {'block': 'SEAT_001_02', 'part_id': 'PART_001', 'center_mm': [0, 0, 22.5], 'R': IDENTITY}]}
    recipe = {'schema': 'cad_recipe/1.0', 'model_id': model_id,
              'steps': [{'block': 'SEAT_001_02', 'sequence': 2, 'stage': 2, 'grasp': 'FLAT_SHORT', 'grasp_axis': 'WIDTH',
                         'supports': ['SEAT_001_01']},
                        {'block': 'SEAT_001_01', 'sequence': 1, 'stage': 1, 'grasp': 'FLAT_SHORT', 'grasp_axis': 'WIDTH',
                         'supports': []}]}
    return recipe, structure


def _joined(size_mm=(75, 25, 15)):
    """_recipe() 를 load_recipe 가 돌려주는 모양(조립 + 'structure' 칸)으로."""
    recipe, structure = _recipe(size_mm)
    return dict(recipe, structure=structure)


def test_recipe_blocks_stacks_on_actual_thickness(robot_cfg):
    """레시피 2블록: sequence 순서, block_id = '<model_id>_<블록 이름>', 잡기 둘 다 FLAT_SHORT(끼우는 축 y = WIDTH),
    위 블록 중심 z = 원점 z + 실측 두께 x 1.5. 설계값(15 mm 층)이 아니라 실측 두께로 쌓아 올린다 — 10/6 실기 교훈(10층에서 +1.8 mm)."""
    blocks = mm.recipe_blocks(robot_cfg, _joined())
    assert [b['block_id'] for b in blocks] == ['001_CHAIR_BENCH_SEAT_001_01', '001_CHAIR_BENCH_SEAT_001_02']
    assert all(b['grasp'] == 'FLAT_SHORT' for b in blocks)
    o, t = robot_cfg['assembly_origin'], robot_cfg['block_actual_m'][2]
    assert close(blocks[0]['center'][2], o['z_m'] + t * 0.5)
    assert close(blocks[1]['center'][2], o['z_m'] + t * 1.5)
    assert close(blocks[0]['center'][:2], (o['x_m'], o['y_m']))


def test_recipe_blocks_rejects_wrong_size(robot_cfg):
    """블록 크기가 robot.yaml block_size_m 와 다르면([70, 25, 15]) ValueError."""
    with pytest.raises(ValueError):
        mm.recipe_blocks(robot_cfg, _joined((70, 25, 15)))


def test_recipe_blocks_rejects_old_blocks_format(robot_cfg):
    """옛 blocks[] 형식(구조 칸 · model 칸 없음)은 W139 뒤 읽지 않는다(W138) — KeyError."""
    with pytest.raises(KeyError):
        mm.recipe_blocks(robot_cfg, {'schema': 'm0609.jenga.cad_recipe/1.0', 'blocks': []})


def test_load_recipe_reads_structure_next_to_it(tmp_path, robot_cfg):
    """load_recipe: 조립 파일 옆 <model_id>_structure.json 을 붙이고, 없으면 FileNotFoundError.
    recipe_files 는 조립 파일(_recipe.json)만 — 구조 파일 · 옛 이름 .recipe.json 은 목록에 안 넣는다."""
    recipe, structure = _recipe()
    (tmp_path / '001_CHAIR_BENCH_recipe.json').write_text(json.dumps(recipe))
    (tmp_path / '003_DESK_STAND.recipe.json').write_text(json.dumps(recipe))
    with pytest.raises(FileNotFoundError):
        mm.load_recipe(str(tmp_path / '001_CHAIR_BENCH_recipe.json'))
    (tmp_path / '001_CHAIR_BENCH_structure.json').write_text(json.dumps(structure))
    files = mm.recipe_files(str(tmp_path))
    assert [mm.recipe_name(f) for f in files] == ['001_CHAIR_BENCH']
    loaded = mm.load_recipe(files[0])
    assert loaded['structure'] == structure
    assert len(mm.recipe_blocks(robot_cfg, loaded)) == 2
    assert mm.recipe_model_id(loaded) == '001_CHAIR_BENCH'


def test_load_recipe_checks_structure_sha256(tmp_path):
    """structure_sha256 = 구조 파일 바이트 그대로의 sha256 (한세교 10/7). 맞으면 읽고, 구조 파일이 바뀌면 ValueError."""
    recipe, structure = _recipe()
    sp = tmp_path / '001_CHAIR_BENCH_structure.json'
    sp.write_text(json.dumps(structure))
    rp = tmp_path / '001_CHAIR_BENCH_recipe.json'
    rp.write_text(json.dumps(dict(recipe, structure_sha256=hashlib.sha256(sp.read_bytes()).hexdigest())))
    assert mm.load_recipe(str(rp))['structure'] == structure
    sp.write_text(json.dumps(structure, indent=1))
    with pytest.raises(ValueError):
        mm.load_recipe(str(rp))


@pytest.mark.parametrize('ratio, factor', [(0.0, 1.0), (1.0, 1.0), (0.5, 0.5)])
def test_move_speed_scale_ok(robot_cfg, ratio, factor):
    """move_to speed_ratio: 0 = 평소, 0 < 값 ≤ 1 = speed_scale 에 곱함 (W121 C-9)."""
    assert close(mm.move_speed_scale(robot_cfg, ratio), robot_cfg['speed_scale'] * factor)


@pytest.mark.parametrize('ratio', [-0.1, 1.5, float('nan')])
def test_move_speed_scale_reject(robot_cfg, ratio):
    """평소보다 빠르거나 음수 · NaN 이면 None (move_to 가 PLAN_FAILED 로 거절)."""
    assert mm.move_speed_scale(robot_cfg, ratio) is None


@pytest.mark.parametrize('grasp', ['FLAT_SHORT', 'FLAT_LONG', 'EDGE_SHORT', 'EDGE_LONG', 'STAND_SHORT', 'STAND_LONG'])
def test_pick_open_width(robot_cfg, grasp):
    """PickPlace open_width_m: 0 = robot.yaml 기본, 블록 폭보다 크면 그 값, 블록 폭 이하 · NaN 이면 None (W121 C-10)."""
    width = robot_cfg['grasp_width_m'][grasp]
    assert mm.pick_open_width(robot_cfg, grasp, 0.0) == robot_cfg['grasp_open_pick_m'][grasp]
    assert close(mm.pick_open_width(robot_cfg, grasp, width + 0.004), width + 0.004)
    assert mm.pick_open_width(robot_cfg, grasp, width) is None
    assert mm.pick_open_width(robot_cfg, grasp, width - 0.005) is None
    assert mm.pick_open_width(robot_cfg, grasp, float('nan')) is None

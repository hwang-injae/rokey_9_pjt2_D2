"""d2_interfaces 의 .srv·.action 파일 형식과 약속한 칸(IRD 5장) 을 지킨다.

구분선 수, 칸 줄 '타입 이름' 형식, 받는 파트가 기대는 칸 이름, CMakeLists 등록을 본다. 파일이 정본이라
문서와 다르면 파일이 맞지만, 칸 이름을 바꾸면 받는 파트가 깨지므로 여기서 걸린다.
"""
import re

import pytest

from conftest import ROOT

IFACE = ROOT / 'src' / 'd2_interfaces'
SRV_FILES = sorted((IFACE / 'srv').glob('*.srv'))
ACTION_FILES = sorted((IFACE / 'action').glob('*.action'))
PRIMS = 'bool int8 uint8 int16 uint16 int32 uint32 int64 uint64 float32 float64 string'.split()
# 타입: ROS 기본형 또는 pkg/Msg · pkg/msg/Msg, 뒤에 [] · [N] 배열 가능
TYPE_RE = re.compile(r'^(?:' + '|'.join(PRIMS) + r'|[a-z][a-z0-9_]*/(?:msg/)?[A-Z][A-Za-z0-9]*)(?:\[\d*\])?$')
NAME_RE = re.compile(r'^[a-z][a-z0-9_]*$')


def sections(path):
    """.srv/.action 을 '---' 로 나눈다. 입력: Path. 출력: [[(타입, 이름) 또는 원래 줄], ...] — 주석(#)·빈 줄은 뺀다.
    칸 줄이 두 단어가 아니면 그 줄 문자열을 그대로 넣어 test_field_lines 가 잡게 한다. 못 읽으면 예외."""
    out = [[]]
    for raw in path.read_text(encoding='utf-8').splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        if line == '---':
            out.append([])
            continue
        parts = line.split()
        out[-1].append(tuple(parts) if len(parts) == 2 else line)
    return out


def fields(section):
    """칸 목록 -> {이름: 타입}. 입력: sections() 의 한 구간. 출력: dict (두 단어가 아닌 줄은 뺀다)."""
    return {name: typ for item in section if isinstance(item, tuple) for typ, name in [item]}


def test_interface_files_exist():
    """srv·action 파일이 하나 이상 있다 (glob 이 비면 아래 시험이 조용히 통과해 버린다)."""
    assert SRV_FILES and ACTION_FILES


@pytest.mark.parametrize('path', SRV_FILES, ids=lambda p: p.name)
def test_srv_has_one_separator(path):
    """.srv 는 요청 --- 응답, 구분선이 정확히 1개."""
    assert len(sections(path)) == 2, path.name


@pytest.mark.parametrize('path', ACTION_FILES, ids=lambda p: p.name)
def test_action_has_two_separators(path):
    """.action 은 목표 --- 결과 --- 피드백, 구분선이 정확히 2개."""
    assert len(sections(path)) == 3, path.name


@pytest.mark.parametrize('path', SRV_FILES + ACTION_FILES, ids=lambda p: p.name)
def test_field_lines(path):
    """칸 줄은 '타입 이름' 두 단어. 타입은 ROS 기본형(+배열) 또는 pkg/Msg, 이름은 소문자_숫자."""
    bad = []
    for sec in sections(path):
        for item in sec:
            if not isinstance(item, tuple):
                bad.append(f'두 단어가 아님: {item!r}')
            elif not TYPE_RE.match(item[0]):
                bad.append(f'타입 모양이 아님: {item[0]!r}')
            elif not NAME_RE.match(item[1]):
                bad.append(f'이름 모양이 아님: {item[1]!r}')
    assert not bad, f'{path.name}:\n  ' + '\n  '.join(bad)


def test_hmi_command_fields():
    """HmiCommand.srv — 요청 cmd·design_id·mode, 응답 success·reason (화면 → 작업 관리자)."""
    req, res = (fields(s) for s in sections(IFACE / 'srv' / 'HmiCommand.srv'))
    assert {'cmd', 'design_id', 'mode'} <= set(req)
    assert {'success', 'reason'} <= set(res)


def test_stop_request_fields():
    """StopRequest.srv — 요청 source·reason (화면·키 → 정지 노드)."""
    req, _ = (fields(s) for s in sections(IFACE / 'srv' / 'StopRequest.srv'))
    assert {'source', 'reason'} <= set(req)


def test_move_to_fields():
    """MoveTo.srv — 요청 target (observe | home | 촬영 · 공급 관측 자세) · speed_ratio (W121, 0 = 평소 속도)."""
    req, _ = (fields(s) for s in sections(IFACE / 'srv' / 'MoveTo.srv'))
    assert 'target' in req
    assert req.get('speed_ratio') == 'float64'


def test_check_progress_request_has_design_id_and_run_id():
    """CheckProgress.srv 요청 — design_id(E-52) 다음에 run_id(E-60), 그 뒤 block_ids. 순서가 바뀌면 받는 쪽(손목 블록 인식)과 어긋난다."""
    req, _ = (fields(s) for s in sections(IFACE / 'srv' / 'CheckProgress.srv'))
    assert list(req) == ['design_id', 'run_id', 'block_ids']
    assert req['run_id'] == 'string'


def test_check_progress_fields():
    """CheckProgress.srv — 응답 states·dx_m·dy_m·dz_m·top_z_m (모두 배열, 같은 번호끼리 한 블록)."""
    _, res = (fields(s) for s in sections(IFACE / 'srv' / 'CheckProgress.srv'))
    for k in ('states', 'dx_m', 'dy_m', 'dz_m', 'top_z_m'):
        assert k in res and res[k].endswith('[]'), k


def test_gripper_command_fields():
    """GripperCommand.srv — 응답 grasped (그리퍼 노드가 잡힘까지 답한다, S-01)."""
    _, res = (fields(s) for s in sections(IFACE / 'srv' / 'GripperCommand.srv'))
    assert res.get('grasped') == 'bool'


def test_pick_place_action_fields():
    """PickPlace.action — 목표 block_id·supply_slot·pick_pose·place_pose·grasp, 피드백 step."""
    goal, _, feedback = (fields(s) for s in sections(IFACE / 'action' / 'PickPlace.action'))
    assert {'block_id', 'supply_slot', 'pick_pose', 'place_pose', 'grasp'} <= set(goal)
    assert goal['pick_pose'] == goal['place_pose'] == 'geometry_msgs/Pose'
    assert goal.get('open_width_m') == 'float64'          # W121, 0 = robot.yaml grasp_open_pick_m
    assert goal.get('obstacles') == 'geometry_msgs/Pose[]'   # E-53 B안, 흩뿌림 공중 이동 계획용(빈 목록 = 다른 블록 없음)
    assert 'step' in feedback


def test_json_query_fields():
    """JsonQuery.srv(W121) — 요청 request_json, 응답 success·reason·response_json."""
    req, res = (fields(s) for s in sections(IFACE / 'srv' / 'JsonQuery.srv'))
    assert 'request_json' in req
    assert {'success', 'reason', 'response_json'} <= set(res)


def test_cmakelists_registers_every_interface():
    """srv/·action/ 의 모든 파일이 CMakeLists.txt 의 rosidl_generate_interfaces 에 적혀 있다 (빠지면 빌드돼도 메시지가 없다)."""
    cmake = (IFACE / 'CMakeLists.txt').read_text(encoding='utf-8')
    missing = [f'{p.parent.name}/{p.name}' for p in SRV_FILES + ACTION_FILES if f'"{p.parent.name}/{p.name}"' not in cmake]
    assert not missing, 'CMakeLists.txt 에 없는 인터페이스: ' + ', '.join(missing)

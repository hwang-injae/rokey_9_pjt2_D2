"""pr_check.py 의 정규식이 지키는 규칙을 고정한다 — 키 모양·문서 이름·비밀 파일·로봇 이동 코드·정지 설정 단어.

정규식을 고치다가 잡아야 할 것을 놓치거나(예: proj- 키), 잡지 말아야 할 것을 잡는(예: JointTrajectory) 일을 막는다.
키 모양 문자열은 파일에 쓰지 않고 실행 때 이어 붙여 만든다 — PR 검사 4번이 파일 안의 키 모양을 막는다(팀 규칙 4).
"""
import pytest

# 키 모양은 실행 때 이어 붙인다 (파일에 그대로 쓰면 PR 검사에 걸린다)
KEY_LIKE = ['sk-' + 'proj-' + 'A' * 30, 'sk-' + 'B' * 25, 'sk-' + 'svcacct-' + 'c9' * 12]
NOT_KEY = ['sk-short', 'skeleton', 'sk-' + 'proj-', 'sk-' + 'A' * 19]     # 19자는 20자 미만이라 키로 안 본다

DOC_OK = ['01_요구사항_BR-SR_v3_100616.md', 'TS-01_정지가_안_들음_R-01_v1_100519.md', 'docs/x_v4-1_100316.md']
DOC_BAD = ['회의록.md', 'foo_v1.md', 'foo_v1_1006.md', 'foo_v1_100616.txt']

SECRET_OK = ['.env', 'config/.env.local', 'certs/robot.pem', 'a/credentials.json', 'secrets.yaml', 'k/server.key']
SECRET_NOT = ['docs/env/README.md', 'src/d2_bringup/config/robot.yaml', 'docs/environment.md']

MOTION_OK = ['movej(', 'amovel(', 'MoveGroup', 'FollowJointTrajectory', 'movejx(', 'from dsr_msgs2.srv import MoveJoint']
MOTION_NOT = ['JointTrajectory', 'trajectory_msgs', 'move_to(', 'MoveGroupInterface']


@pytest.mark.parametrize('text', KEY_LIKE)
def test_key_re_matches_key_like(pr_check_mod, text):
    """KEY_RE 는 OpenAI 키 모양(sk-, sk-proj-, sk-svcacct- + 20자 이상)을 잡는다."""
    assert pr_check_mod.KEY_RE.search(text)


@pytest.mark.parametrize('text', NOT_KEY)
def test_key_re_ignores_short_or_words(pr_check_mod, text):
    """KEY_RE 는 짧은 글자·보통 단어(skeleton)는 잡지 않는다."""
    assert not pr_check_mod.KEY_RE.search(text)


@pytest.mark.parametrize('name', DOC_OK)
def test_doc_re_accepts_rule_names(pr_check_mod, name):
    """DOC_RE 는 이름_v<버전>[-<부>]_<MMDDHH>.md 를 받는다 (팀 규칙 9)."""
    assert pr_check_mod.DOC_RE.search(name)


@pytest.mark.parametrize('name', DOC_BAD)
def test_doc_re_rejects_other_names(pr_check_mod, name):
    """DOC_RE 는 버전·월일시가 없거나 자릿수가 다른 이름을 막는다."""
    assert not pr_check_mod.DOC_RE.search(name)


@pytest.mark.parametrize('path', SECRET_OK)
def test_secret_files_matches(pr_check_mod, path):
    """SECRET_FILES 는 .env(.*)·.pem·.key·credentials*·secrets.yaml 을 잡는다."""
    assert pr_check_mod.SECRET_FILES.search(path)


@pytest.mark.parametrize('path', SECRET_NOT)
def test_secret_files_ignores_normal(pr_check_mod, path):
    """SECRET_FILES 는 docs/env/ 폴더나 robot.yaml 같은 보통 파일은 잡지 않는다."""
    assert not pr_check_mod.SECRET_FILES.search(path)


def test_env_example_is_allowed(pr_check_mod):
    """`.env.example` 은 정규식에는 걸리지만 pr_check 가 따로 허용한다 — check_secrets 와 같은 식으로 확인."""
    f = '.env.example'
    assert pr_check_mod.SECRET_FILES.search(f)
    assert f.endswith('.env.example')       # check_secrets 의 `not f.endswith('.env.example')` 가 걸러 낸다


@pytest.mark.parametrize('text', MOTION_OK)
def test_motion_re_matches_moving_code(pr_check_mod, text):
    """MOTION_RE 는 팔을 움직이는 표시(movej·amovel·MoveGroup·FollowJointTrajectory)를 잡는다."""
    assert pr_check_mod.MOTION_RE.search(text)


@pytest.mark.parametrize('text', MOTION_NOT)
def test_motion_re_ignores_message_only(pr_check_mod, text):
    """MOTION_RE 는 메시지 모양만 쓰는 JointTrajectory·trajectory_msgs 는 잡지 않는다."""
    assert not pr_check_mod.MOTION_RE.search(text)


def test_signal_re(pr_check_mod):
    """SIGNAL_RE: SignalHandlerOptions.NO 또는 init_ros( 가 있어야 Ctrl+C 정지 설정으로 본다."""
    assert pr_check_mod.SIGNAL_RE.search('rclpy.init(signal_handler_options=SignalHandlerOptions.NO)')
    assert pr_check_mod.SIGNAL_RE.search('node = init_ros(args)')
    assert not pr_check_mod.SIGNAL_RE.search('rclpy.init(signal_handler_options=SignalHandlerOptions.ALL)')
    assert not pr_check_mod.SIGNAL_RE.search('rclpy.init()')


def test_stop_re(pr_check_mod):
    """STOP_RE: SafeStop·move_stop·MoveStop·hold_here 가 서기 정지 호출이다."""
    assert pr_check_mod.STOP_RE.search('self.safe = SafeStop(node)')
    assert pr_check_mod.STOP_RE.search('self.move_stop(2)')
    assert not pr_check_mod.STOP_RE.search('SafeStopper()')
    assert not pr_check_mod.STOP_RE.search('stop = True')

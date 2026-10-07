"""pytest 공통 설정 — 저장소 루트 경로, d2_motion 을 ROS 없이 import, pr_check.py 읽기, robot.yaml, 저장소 파일 목록.

실행은 늘 저장소 루트에서 `python3 -m pytest tests -q`. 저장소 루트에 pytest.ini·pyproject.toml 은 두지 않는다 —
colcon test 가 src/ 패키지 시험을 돌 때 그것을 rootdir 로 잡아 버린다.
"""
import importlib.util
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]                        # 저장소 루트
ROBOT_YAML = ROOT / 'src' / 'd2_robot' / 'd2_bringup' / 'config' / 'robot.yaml'
PR_CHECK_PY = ROOT / '.github' / 'scripts' / 'pr_check.py'
SKIP_DIRS = {'build', 'install', 'log', 'node_modules', '.git'}   # git 이 없을 때 rglob 에서 빼는 폴더

# d2_motion.motion_math 는 `import math` 뿐이라 ROS 없이 돈다 (d2_motion/__init__.py 는 비어 있다)
_D2_MOTION = str(ROOT / 'src' / 'd2_robot' / 'd2_motion')
if _D2_MOTION not in sys.path:
    sys.path.insert(0, _D2_MOTION)


@pytest.fixture(scope='session')
def pr_check_mod():
    """.github/scripts/pr_check.py 를 모듈로 읽는다 — 폴더 이름에 점이 있어 import 문으로는 못 읽는다.

    입력: 없음. 출력: 모듈(KEY_RE·DOC_RE·SECRET_FILES·MOTION_RE·SIGNAL_RE·STOP_RE 가 있다).
    실패 때: 파일이 없거나 문법 오류면 예외 그대로 → 이 fixture 를 쓰는 시험이 모두 실패한다.
    """
    spec = importlib.util.spec_from_file_location('pr_check', PR_CHECK_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='session')
def robot_cfg():
    """src/d2_robot/d2_bringup/config/robot.yaml 을 dict 로 읽는다 — 시험의 숫자·좌표는 지어내지 않고 여기서 읽는다.

    입력: 없음. 출력: dict. 실패 때: 파일이 없거나 YAML 문법이 틀리면 예외로 실패.
    """
    return yaml.safe_load(ROBOT_YAML.read_text(encoding='utf-8'))


def _repo_files(suffixes=None):
    """저장소 파일 목록 (절대 경로 Path, 정렬). suffixes(튜플) 를 주면 그 이름으로 끝나는 것만.

    git 이 있으면 추적 파일 + 추적 안 된(무시되지 않은) 파일 — 로컬에서 이름을 바꾸는 중인 문서도 보려고.
    디스크에 없는 것(지웠거나 이름을 바꾼 옛 이름)은 뺀다. CI 의 깨끗한 checkout 에서는 추적 파일과 같다.
    실패 때: git 이 없거나 오류면 예외 없이 rglob(SKIP_DIRS 제외)으로 넘어간다.
    """
    try:
        base = ['git', '-C', str(ROOT), '-c', 'core.quotepath=off', 'ls-files']
        tracked = subprocess.run(base, capture_output=True, text=True, check=True).stdout
        others = subprocess.run(base + ['--others', '--exclude-standard'], capture_output=True, text=True, check=True).stdout
        paths = {ROOT / n for n in (tracked + others).splitlines() if n}
    except (OSError, subprocess.CalledProcessError):
        paths = {p for p in ROOT.rglob('*') if p.is_file() and not (set(p.relative_to(ROOT).parts[:-1]) & SKIP_DIRS)}
    if suffixes:
        paths = {p for p in paths if p.name.endswith(tuple(suffixes))}
    return sorted(p for p in paths if p.is_file())


@pytest.fixture(scope='session')
def repo_files():
    """_repo_files 를 돌려주는 fixture — 시험에서 `repo_files(('.md',))` 처럼 부른다."""
    return _repo_files

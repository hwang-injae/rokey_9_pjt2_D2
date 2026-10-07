# -*- coding: utf-8 -*-
"""colcon 빌드 없이 `python3 -m pytest src/d2_task/test` 로 돌리도록 소스 폴더를 import 경로에 넣는다 (ROS 없는 시험용)."""
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
for pkg in ('d2_task', 'd2_motion', 'd2_robot/d2_motion'):
    sys.path.insert(0, str(SRC / pkg))

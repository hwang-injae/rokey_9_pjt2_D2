"""스캔 '추천 설계'(10/13 플랜 B 1단계 — web/backend/design_store.py DesignStore.match_scan)가 비전 일치율(V-48)과 같게 고르는지 지킨다.

OpenAI · MQTT 없이 돈다(저장소 + 기본 설계 4개). 지키는 것: 기본 설계를 그대로 스캔하면 그 설계가 1등(16/16) · 블록 하나가 다르면 그만큼 낮음 ·
자리가 통째로 옮겨져도(바닥 외곽 가운데 기준) 같음 · 비전 structure_scanner.match_rate 와 같은 맞은 수 · 빈 스캔은 빈 목록.
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / 'web' / 'backend', ROOT / 'src' / 'd2_task', ROOT / 'src' / 'd2_vision'):
    sys.path.insert(0, str(p))

from design_store import DesignStore  # noqa: E402

RECIPES = ROOT / 'src' / 'd2_robot' / 'd2_bringup' / 'recipes'
BACK = '002_CHAIR_BACK_V000'


@pytest.fixture
def store(tmp_path):
    """임시 폴더 저장소 + 기본 설계 4개."""
    s = DesignStore(tmp_path, RECIPES, [75, 25, 15])
    s.register_bases()
    return s


def scan_of(store, design_id, dx=0.0, dy=0.0):
    """저장된 설계를 스캔한 것처럼 — 위치 · 방향만(scan_num 은 거꾸로), 자리를 (dx, dy) mm 만큼 통째로 옮길 수 있음."""
    parts = store.get_design(design_id)['parts']['parts']
    return [{'scan_num': len(parts) - i, 'x': p['x'] + dx, 'y': p['y'] + dy, 'z': p['z'], 'ori': p['ori'], 'inferred': False}
            for i, p in enumerate(parts)]


def test_same_design_is_first_with_full_match(store):
    """기본 설계를 그대로(통째로 옮겨) 스캔하면 그 설계가 1등, 다 맞음 — 다른 설계는 낮다. 최대 3개."""
    got = store.match_scan(scan_of(store, BACK, dx=40.0, dy=-30.0))
    assert len(got) == 3
    assert got[0]['design_id'] == BACK and got[0]['matched'] == got[0]['scan_count'] == got[0]['part_count'] == 16
    assert got[0]['rate'] == 1.0 and all(m['rate'] < 1.0 for m in got[1:])


def test_one_block_off_lowers_rate(store):
    """블록 하나 방향이 다르면 15/16 — 그래도 그 설계가 1등(사람이 화면에서 보고 수락)."""
    scan = scan_of(store, BACK)
    top = max(range(len(scan)), key=lambda i: scan[i]['z'])           # 맨 위 블록(바닥 외곽 가운데는 그대로)
    scan[top]['ori'] = 'y' if scan[top]['ori'] != 'y' else 'x'
    got = store.match_scan(scan, top=1)
    assert got[0]['design_id'] == BACK and got[0]['matched'] == 15 and got[0]['rate'] == round(15 / 16, 3)


def test_same_count_as_vision_match_rate(store):
    """맞은 수가 비전 structure_scanner.match_rate(V-48)와 같다 — 같은 식을 두 곳에 둔 것이 어긋나지 않게."""
    ss = pytest.importorskip('d2_vision.structure_scanner')
    scan = scan_of(store, '001_CHAIR_BENCH_V000', dx=7.0)
    for m in store.match_scan(scan, top=4):
        parts = store.get_design(m['design_id'])['parts']['parts']
        rate, matched, _ = ss.match_rate(scan, parts, (75.0, 25.0, 15.0))
        assert (m['matched'], m['rate']) == (matched, round(rate, 3)), m['design_id']


def test_empty_scan_is_empty(store):
    """스캔 블록이 없으면 추천도 없다."""
    assert store.match_scan([]) == []

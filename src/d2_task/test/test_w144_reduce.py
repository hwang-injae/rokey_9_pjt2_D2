# -*- coding: utf-8 -*-
"""W144 bag 줄이기 FramePicker 시험 — 색 초당 2장 고르기 · 깊이 · camera_info 같은 시각 짝(먼저 와도 · 나중에 와도) (ROS · bag 없이)."""
from d2_task.w144_reduce import PAIR_S, PERIOD_S, FramePicker, stamp_of


def test_색은_PERIOD_마다_한_장():
    p = FramePicker()
    kept = [k / 30.0 for k in range(90) if p.feed_color(k / 30.0)]       # 30 fps 3 초
    assert len(kept) == 6 and all(b - a >= PERIOD_S - 1e-9 for a, b in zip(kept, kept[1:]))


def test_짝은_먼저_와도_나중에_와도_색마다_하나():
    p = FramePicker()
    written = []
    for k in range(60):                                   # 30 fps 2 초 — 깊이는 짝수 칸에서 색보다 먼저, 홀수 칸에서 나중에 온다
        t = k / 30.0
        d = ('/d', t + 0.005)
        if k % 2 == 0:
            written += p.feed_pair('/d', d[1], d)
        if p.feed_color(t):
            written += [it for _, it in p.pending_for(t)]
        if k % 2 == 1:
            written += p.feed_pair('/d', d[1], d)
    assert len(written) == len(p.kept) == 4
    assert all(any(abs(c - st) <= PAIR_S for c in p.kept) for _, st in written)


def test_짝_시각이_멀면_안_씀():
    p = FramePicker()
    p.feed_color(0.0)
    assert p.feed_pair('/d', 0.0 + PAIR_S * 2, 'x') == []


def test_CDR_머리_뒤_header_시각():
    data = b'\x00\x01\x00\x00' + (5).to_bytes(4, 'little', signed=True) + (250_000_000).to_bytes(4, 'little') + b'rest'
    assert abs(stamp_of(data) - 5.25) < 1e-9

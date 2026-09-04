"""平板×多点締結(実車031 / 1285-20)の不変条件。2026-09-05。

掃引モデルに収まる実車部品は19件中4件しかなく、うち2件がこの平板だった
(残り2件が007と011)。OCCTを呼ばずに済む範囲だけ見る。
"""
from __future__ import annotations

import math
import random

from synthetic_generator.families import Knobs, flat_plate_part


def _draw(count: int = 12):
    rng = random.Random(20260905)
    knobs = Knobs()
    out = []
    while len(out) < count:
        got = flat_plate_part(rng, knobs)
        if got is not None:
            out.append(got)
    return out


def test_plate_is_flat_with_one_shared_normal():
    for spec, bead, flange, rib in _draw():
        assert (bead, flange, rib) == (None, None, None), "平板は特徴を持たない"
        assert spec.target_folds == 0
        assert spec.plate_margin_mm is not None, "平板であることの印"
        normals = [p.normal_xyz for p in spec.annotated_points]
        for n in normals[1:]:
            dot = sum(a * b for a, b in zip(normals[0], n))
            assert abs(dot) > 1.0 - 1e-9, "全ての締結点が同じ法線を向く"


def test_plate_has_four_to_six_joints_spread_in_the_plane():
    for spec, _bead, _flange, _rib in _draw():
        points = [p.position_xyz for p in spec.annotated_points]
        assert 4 <= len(points) <= 6, "実車031/20はどちらも4点。多点締結の族"
        gap = min(math.dist(a, b) for i, a in enumerate(points) for b in points[i + 1:])
        assert gap >= 20.0 - 1e-6, "点間の最小距離(実車の最小は16mm)"
        # 一直線に並ばない — 並ぶと外形が張れない
        best = max(
            abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            for i, a in enumerate(points) for j, b in enumerate(points[i + 1:], i + 1)
            for c in points[j + 1:]) / 2.0
        assert best > 0.0


def test_plate_outline_keeps_the_bearing_margin():
    """外形の余白は必要平面R以上。隅Rはそれとは別に小さく持つ(実車4.5〜8mm)。"""
    for spec, _bead, _flange, _rib in _draw():
        assert spec.plate_margin_mm >= spec.min_bearing_radius_mm - 1e-9
        assert 5.0 <= spec.plate_corner_radius_mm <= 12.0
        assert spec.plate_corner_radius_mm < spec.plate_margin_mm

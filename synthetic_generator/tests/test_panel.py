"""大型パネル族(実車002-002/017/049/062 ほか)の不変条件。

OCCT を呼ばない範囲だけ見る。形状の成立は tools/run_occt_batch.py で通す。
"""
from __future__ import annotations

import collections
import math
import random

from synthetic_generator.families import (
    PANEL_BEAD_RIDGE_MM, PANEL_HALF_WIDTH_MM, PANEL_MAX_POINTS, Knobs, panel_part,
)


def _draw(count: int = 40):
    rng = random.Random(20260908)
    out = []
    while len(out) < count:
        got = panel_part(rng, Knobs())
        if got is not None:
            out.append(got[0])
    return out


def test_band_width_stays_inside_the_ruling():
    """裁定(2026-09-08): 帯幅は実車 p75 に合わせて 幅 ~120mm(半幅 60)まで。"""
    for spec in _draw():
        pn = spec.panel
        assert spec.half_width_mm == pn["half_width_mm"]
        assert PANEL_HALF_WIDTH_MM[0] - 1e-9 <= pn["half_width_mm"] <= PANEL_HALF_WIDTH_MM[1] + 1e-9


def test_beads_fit_side_by_side_and_leave_the_centre_flat():
    """アンカー2点は中心線(y=0)に載るので、中心は必ず平地で残す。"""
    for spec in _draw():
        pn = spec.panel
        b = spec.min_bearing_radius_mm
        spans = sorted(tuple(s) for s in pn["bead_spans"])
        assert 1 <= len(spans) <= 4
        for lo, hi in spans:
            assert hi > lo
            assert -pn["half_width_mm"] < lo and hi < pn["half_width_mm"]
            assert not (lo <= 0.0 <= hi), "中心にビードは置かない"
        for (a0, a1), (b0, b1) in zip(spans, spans[1:]):
            assert b0 > a1, "ビードどうしが重なっている"
        # 中心の平地はアンカーの座面が入る
        inner = min((lo for lo, hi in spans if lo > 0.0), default=pn["half_width_mm"])
        assert inner >= b + 2.0 - 1e-9


def test_bead_walls_survive_their_own_fillets():
    """浅くて稜線Rが大きいと壁が反転して掃引が裏返る(2026-09-08 実測で3割が落ちた)。"""
    for spec in _draw():
        for _y, bd in spec.panel["beads"]:
            theta = math.radians(bd["wall_angle_deg"])
            sb = bd["ridge_radius_mm"] * math.tan(theta / 2.0)
            assert bd["depth_mm"] - 2.0 * sb * math.sin(theta) > 0.5
            assert bd["top_width_mm"] / 2.0 > sb
            assert PANEL_BEAD_RIDGE_MM[0] <= bd["ridge_radius_mm"] <= PANEL_BEAD_RIDGE_MM[1]


def test_lands_can_host_a_bearing_seat():
    for spec in _draw():
        b = spec.min_bearing_radius_mm
        for lo, hi in spec.panel["lands"]:
            assert hi - lo >= 2.0 * b - 1e-9


def test_points_are_counted_and_have_radii():
    for spec in _draw():
        pn = spec.panel
        n = len(spec.annotated_points)
        assert 2 <= n <= PANEL_MAX_POINTS
        assert pn["factors"]["points"] == n
        assert len(pn["point_radii"]) == n
        assert all(r > 0.0 for r in pn["point_radii"])


def test_walls_carry_no_fastening_point():
    for spec in _draw():
        for arm in spec.panel["arms"]:
            if arm["role"] == "wall":
                assert not arm["points"]


def test_factors_span_their_ranges():
    seen = collections.defaultdict(set)
    for spec in _draw(60):
        for k, v in spec.panel["factors"].items():
            if k != "half_width_mm":
                seen[k].add(v)
    for k in ("folds", "beads", "walls", "arms", "points"):
        assert len(seen[k]) >= 3, (k, sorted(seen[k]))

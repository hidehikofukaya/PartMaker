"""スポット溶接列(AMS 依頼 9 §3.1/3.2 の着手依頼、2026-09-24)の不変条件。OCCT は呼ばない。"""
from __future__ import annotations

import random

from synthetic_generator.annotate import build_joints
from synthetic_generator.families import (
    BOX_WELD_POINTS, PANEL_WELD_POINTS, WELD_EDGE_MM, Knobs, _weld_row, box_bracket_part,
    panel_part,
)
from synthetic_generator.variants import part_rng, reseed_panel


def test_weld_row_keeps_edge_distance_and_pitch():
    for width, d, p in ((300.0, 10.0, 45.0), (61.0, 7.0, 70.0), (18.0, 14.0, 30.0)):
        row = _weld_row(width, d, p)
        if width < 2 * d:
            assert row == []
            continue
        assert row and min(row) >= d - 1e-9 and max(row) <= width - d + 1e-9
        gaps = [b - a for a, b in zip(row, row[1:])]
        assert all(0.66 * p <= g <= 1.34 * p for g in gaps), gaps


def _kinds_match(blk):
    kinds, radii = blk["point_kinds"], blk["point_radii"]
    assert len(kinds) == len(radii)
    assert all((k == "spot_weld") == (r is None) for k, r in zip(kinds, radii))
    return kinds.count("spot_weld")


def test_panel_weld_rows_sit_on_walls_at_edge_distance():
    rng = random.Random(924)
    for _ in range(6):
        spec, *_ = panel_part(rng, Knobs(weld_rows=True))
        pn = spec.panel
        assert PANEL_WELD_POINTS[0] <= len(spec.annotated_points) <= PANEL_WELD_POINTS[1]
        assert _kinds_match(pn) > 0
        d = pn["weld"]["edge_mm"]
        assert WELD_EDGE_MM[0] <= d <= WELD_EDGE_MM[1]
        for a in pn["arms"]:
            for t, _s, kind in a["points"]:
                if kind == "spot_weld":
                    assert a["role"] == "wall" and abs(a["length_mm"] - t - d) < 1e-9


def test_box_weld_rows_count_and_kinds():
    for group in ("bend", "draw"):
        rng = random.Random(924)
        spec, *_ = box_bracket_part(rng, Knobs(weld_rows=True, box_group=group))
        assert BOX_WELD_POINTS[0] <= len(spec.annotated_points) <= BOX_WELD_POINTS[1]
        assert _kinds_match(spec.box) > 0


def test_weld_points_become_weld_joints():
    spec, *_ = panel_part(random.Random(924), Knobs(weld_rows=True))
    kinds = spec.panel["point_kinds"]
    joints = build_joints("T", spec.annotated_points, 8.0, kinds=kinds, thickness_mm=1.2)
    for j, k in zip(joints, kinds):
        d = j.to_dict()
        if k == "spot_weld":
            assert d["type"] == "weld" and "contact_xyz" in d["per_part"][0]
            assert "hole_diameter_mm" not in d["per_part"][0]
        else:
            assert d["type"] == "mounting_hole"


def test_reseed_keeps_weld_walls():
    spec, *_ = panel_part(random.Random(924), Knobs(weld_rows=True))
    new, *_ = reseed_panel(spec, part_rng("T", salt="reseed1"))
    for a, b in zip(spec.panel["arms"], new.panel["arms"]):
        if any(q[2] == "spot_weld" for q in a["points"]):
            assert a["length_mm"] == b["length_mm"]

"""タブ付きブラケット(実車1285-18)の不変条件。2026-09-06。

OCCT を呼ばない範囲だけ見る。形状の成立は tools/run_occt_batch.py で通す。
"""
from __future__ import annotations

import math
import random

from synthetic_generator.families import (
    TAB_BEARING_MM, TAB_HINGE_DEG, TAB_MIN_R_MM, TAB_WELD_COUNT, Knobs, tab_bracket_part,
)
from synthetic_generator.occt_build import _fillet_corner_2d, branch_frames


def _draw(count: int = 24):
    rng = random.Random(20260906)
    out = []
    while len(out) < count:
        got = tab_bracket_part(rng, Knobs())
        if got is not None:
            out.append(got)
    return out


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def test_upstand_goes_up_and_flange_goes_down_with_three_normals():
    for spec, bead, flange, rib in _draw():
        assert (bead, flange, rib) == (None, None, None)
        br = spec.branch
        up, fl = br["arms"]
        assert up["side"] == 1 and fl["side"] == -1, "立ち上がりは表、フランジは裏"
        assert up["outline"]["kind"] == "tabs" and len(up["outline"]["tabs"]) == TAB_WELD_COUNT
        assert fl["outline"]["kind"] == "trapezoid"
        assert TAB_HINGE_DEG[0] <= br["hinge_deg"] <= TAB_HINGE_DEG[1]
        assert min(up["radius_mm"], fl["radius_mm"], br["corner_radius"]) >= TAB_MIN_R_MM
        # タブ半径 = 必要半径(溶接相当)
        b = spec.min_bearing_radius_mm
        assert TAB_BEARING_MM[0] <= b <= TAB_BEARING_MM[1]
        assert all(abs(r - b) < 1e-9 for _s, r in up["outline"]["tabs"])
        normals = {tuple(round(c, 2) for c in p.normal_xyz) for p in spec.annotated_points}
        assert len(normals) == 3, "台 / 立ち上がり / フランジの3方向"
        assert len(spec.annotated_points) in (TAB_WELD_COUNT + 2, TAB_WELD_COUNT + 3)


def test_welds_sit_on_tab_centres_and_bolt_inside_flange():
    for spec, *_ in _draw():
        br = spec.branch
        lay = branch_frames(br["hub_xy"], br["arms"], origin=tuple(br["origin"]),
                            hub_u=tuple(br["hub_u"]), hub_v=tuple(br["hub_v"]),
                            corner_radius=br["corner_radius"], fillet_radius=br["fillet_radius"])
        fr = lay["arms"][0]
        b = spec.min_bearing_radius_mm
        welds = spec.annotated_points[:TAB_WELD_COUNT]
        for (s_c, _r), p in zip(br["arms"][0]["outline"]["tabs"], welds):
            expect = tuple(fr["a"][i] + br["arms"][0]["length_mm"] * fr["tip"][i] + s_c * fr["axis"][i]
                           for i in range(3))
            assert math.dist(p.position_xyz, expect) < 1e-6
            assert _dot(p.normal_xyz, fr["normal"]) > 0.9999
        fr = lay["arms"][2]
        bolt = spec.annotated_points[TAB_WELD_COUNT]
        t = _dot(tuple(bolt.position_xyz[i] - fr["a"][i] for i in range(3)), fr["tip"])
        assert b - 1e-6 <= t <= br["arms"][1]["length_mm"] - b + 1e-6, "ボルトの必要平面が高さに収まる"


def test_fillet_corner_is_tangent_to_both_edges():
    start, mid, end = _fillet_corner_2d((0.0, 10.0), (0.0, 0.0), (10.0, 0.0), 3.0)
    assert math.dist(start, (0.0, 3.0)) < 1e-9 and math.dist(end, (3.0, 0.0)) < 1e-9
    assert abs(math.hypot(mid[0] - 3.0, mid[1] - 3.0) - 3.0) < 1e-9, "円弧の中点は中心から R"

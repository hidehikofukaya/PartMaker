"""多面ブラケット族(実車002-024)の不変条件。

OCCT を呼ばない範囲だけ見る。形状の成立は tools/run_occt_batch.py で通す。
"""
from __future__ import annotations

import collections
import math
import random

from synthetic_generator.families import (
    BOX_CORNER_R_MM, BOX_CORNER_R_OVER_T, BOX_DRAW_RATIO, BOX_GROUPS, BOX_MAX_POINTS,
    BOX_WALL_FOLD_DEG, BOX_WALL_HEIGHT_MM, BOX_WELD_BEARING_MM, Knobs, box_bracket_part,
)
from synthetic_generator.occt_build import box_frames


def _draw(count: int = 40, group: str | None = None):
    rng = random.Random(20260908)
    out = []
    while len(out) < count:
        got = box_bracket_part(rng, Knobs(box_group=group))
        if got is not None:
            out.append(got[0])
    return out


def _ccw_convex(xy) -> bool:
    n = len(xy)
    return all((xy[(i + 1) % n][0] - xy[i][0]) * (xy[(i + 2) % n][1] - xy[i][1])
               - (xy[(i + 1) % n][1] - xy[i][1]) * (xy[(i + 2) % n][0] - xy[i][0]) > 0.0
               for i in range(n))


def test_hub_is_a_ccw_convex_polygon():
    for spec in _draw():
        hub = [tuple(q) for q in spec.box["hub_xy"]]
        assert 3 <= len(hub) <= 6
        assert _ccw_convex(hub), hub


def test_adjacent_walls_always_share_a_sewn_corner():
    """隣り合う辺の両方に壁があるなら角は継ぎ目にする(開けると根本フィレットが衝突する)。"""
    for spec in _draw():
        bx = spec.box
        n = len(bx["hub_xy"])
        walls = {int(k) for k in bx["walls"]}
        closed = set(bx["closed"])
        for k in range(n):
            both = (k - 1) % n in walls and k % n in walls
            assert both == (k in closed), (k, sorted(walls), sorted(closed))


def test_arms_and_walls_never_share_an_edge():
    for spec in _draw():
        bx = spec.box
        walls = {int(k) for k in bx["walls"]}
        assert not walls & {a["edge"] for a in bx["arms"]}


def test_flanges_sit_on_a_wall_with_a_non_degenerate_root():
    for spec in _draw():
        bx = spec.box
        for f in bx["flanges"]:
            assert str(f["wall"]) in bx["walls"] or f["wall"] in bx["walls"]
            assert f["to_mm"] - f["from_mm"] > 0.0
            assert f["side"] in (1, -1)


def test_wall_values_stay_in_range_and_clear_the_corner_fillet():
    for spec in _draw():
        bx = spec.box
        for w in bx["walls"].values():
            assert BOX_WALL_FOLD_DEG[0] <= w["fold_deg"] <= BOX_WALL_FOLD_DEG[1]
            assert w["height_mm"] <= BOX_WALL_HEIGHT_MM[1]
            tangent = bx["corner_r_mm"] * math.tan(math.radians(w["fold_deg"]) / 2.0)
            assert math.isclose(w["tangent_mm"], tangent, rel_tol=1e-9)
            # タブが載る余地(フィレットの接線点から溶接の座面 + 逃げ)
            assert w["height_mm"] > tangent
            assert w["fillet_mm"] == bx["corner_r_mm"], "角のRは1部品で同一(ゲートAの都合)"


def test_points_carry_a_radius_and_an_owner():
    for spec in _draw():
        bx = spec.box
        n_points = len(spec.annotated_points)
        assert 2 <= n_points <= BOX_MAX_POINTS
        assert len(bx["point_radii"]) == n_points
        assert len(bx["point_owners"]) == n_points
        b = spec.min_bearing_radius_mm
        weld = bx["weld_bearing_mm"]
        assert BOX_WELD_BEARING_MM[0] <= weld <= BOX_WELD_BEARING_MM[1]
        assert all(r in (b, weld) for r in bx["point_radii"])
        owners = set(bx["point_owners"])
        assert owners <= ({"hub"} | {f"wall_{k}" for k in bx["walls"]}
                          | {f"arm_{a['edge']}" for a in bx["arms"]}
                          | {f"flange_{i}" for i in range(len(bx["flanges"]))})


def test_tab_points_use_the_weld_bearing_radius():
    """壁の上端のタブ = 溶接。タブの本数と『壁が持ち主の溶接点』の数は一致する。"""
    for spec in _draw():
        bx = spec.box
        weld = bx["weld_bearing_mm"]
        for k, w in bx["walls"].items():
            tabs = len(w["tabs"])
            welds = sum(1 for owner, r in zip(bx["point_owners"], bx["point_radii"])
                        if owner == f"wall_{k}" and r == weld)
            assert welds == tabs


def test_factors_span_their_ranges():
    seen = collections.defaultdict(set)
    for spec in _draw(60):
        f = spec.box["factors"]
        assert f["points"] == len(spec.annotated_points)
        assert f["walls"] == len(spec.box["walls"])
        assert f["arms"] == len(spec.box["arms"])
        assert f["flanges"] == len(spec.box["flanges"])
        for k, v in f.items():
            seen[k].add(v)
    for k in ("sides", "walls", "closed", "tabs", "flanges", "arms", "points"):
        assert len(seen[k]) >= 3, (k, sorted(seen[k]))


def test_group_targeting_gives_exactly_that_structure():
    """plate = 壁なし+腕 / bend = 壁はあるが角の連結なし / draw = 角を連結(絞り)。"""
    for group in BOX_GROUPS:
        for spec in _draw(15, group=group):
            bx = spec.box
            assert bx["group"] == group
            if group == "plate":
                assert not bx["walls"] and bx["arms"]
            elif group == "bend":
                assert bx["walls"] and not bx["closed"]
            else:
                assert bx["closed"]


def test_corner_radius_respects_the_real_floor():
    """角のR / 板厚 の下限は実車の最小 2.4(2026-09-08 実測、002+1285 の109箇所)。"""
    for spec in _draw(40):
        bx = spec.box
        assert bx["corner_r_mm"] >= BOX_CORNER_R_OVER_T * spec.thickness_mm - 1e-9
        assert BOX_CORNER_R_MM[0] - 1e-9 <= bx["corner_r_mm"] <= BOX_CORNER_R_MM[1] + 1e-9
        assert all(w["fillet_mm"] == bx["corner_r_mm"] for w in bx["walls"].values())


def test_draw_depth_stays_inside_the_real_envelope():
    """継ぎ目の長さ lam = min(h_i / dt_i) が draw_ratio * R を超えない
    (実車の深さ/R は 最大 11.06。角の無い壁は絞りではないので対象外)。"""
    cap = BOX_DRAW_RATIO[-1][0][1]
    for spec in _draw(40, group="draw"):
        bx = spec.box
        lay = box_frames([tuple(q) for q in bx["hub_xy"]],
                         {int(k): w for k, w in bx["walls"].items()}, set(bx["closed"]),
                         origin=tuple(bx["origin"]), hub_u=tuple(bx["hub_u"]),
                         hub_v=tuple(bx["hub_v"]))
        for k, seam in lay["seams"].items():
            kx, ky = seam["edges"]
            lam = min(lay["walls"][kx]["ends"]["right"]["t"] / lay["walls"][kx]["ends"]["right"]["dt"],
                      lay["walls"][ky]["ends"]["left"]["t"] / lay["walls"][ky]["ends"]["left"]["dt"])
            assert lam <= cap * bx["corner_r_mm"] + 1e-6, (lam, bx["corner_r_mm"])

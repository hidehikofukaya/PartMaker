"""絞りトレイ(実車002-057 の簡略版、展開図第2版)の不変条件。2026-09-06。

OCCT を呼ばない範囲だけ見る。形状の成立(フィレットの収束)は tools/run_occt_batch.py で通す。
"""
from __future__ import annotations

import random

from synthetic_generator.families import (
    TAB_SEAT_MARGIN,
    DRAWN_BEARING_MM, DRAWN_HUB_POINTS, DRAWN_WALL_FOLD_DEG, Knobs, drawn_tray_part,
)
from synthetic_generator.occt_build import drawn_notch_mm, drawn_tray_frames, tab_footprint_mm


def _draw(count: int = 20):
    rng = random.Random(20260906)
    out = []
    while len(out) < count:
        got = drawn_tray_part(rng, Knobs())
        if got is not None:
            out.append(got)
    return out


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def test_three_drawn_walls_two_seams_one_bent_tab():
    tab_counts = set()
    for spec, bead, flange, rib in _draw():
        assert (bead, flange, rib) == (None, None, None)
        dr = spec.drawn
        assert set(dr["walls"]) == {"A", "B", "C"}
        assert [a["edge"] for a in dr["arms"]] == [2], "曲げタブは後辺(辺2)だけ"
        assert set(dr["seams"]) == {"AB", "CB"}
        b = spec.min_bearing_radius_mm
        assert DRAWN_BEARING_MM[0] <= b <= DRAWN_BEARING_MM[1]
        radii = {round(w["fillet_mm"], 6) for w in dr["walls"].values()} | {round(dr["seam_fillet_mm"], 6)}
        assert len(radii) == 1, "3本のフィレットは同じ半径(縁が円弧で収まる条件)"
        for w in dr["walls"].values():
            assert DRAWN_WALL_FOLD_DEG[0] <= w["fold_deg"] <= DRAWN_WALL_FOLD_DEG[1]
            # 裁定 2026-09-09: タブ円は必要半径より一回り大きい
            assert all(abs(r - b * TAB_SEAT_MARGIN) < 1e-9 for _s, r in w["tabs"])
            tab_counts.add(len(w["tabs"]))
    assert len(tab_counts) >= 2, "タブの個数に乱数性がある"


def test_seam_ends_sit_on_the_lower_wall_top_and_tabs_avoid_the_notch():
    for spec, *_ in _draw():
        dr = spec.drawn
        lay = drawn_tray_frames(dr["hub_xy"], dr["walls"], origin=tuple(dr["origin"]),
                                hub_u=tuple(dr["hub_u"]), hub_v=tuple(dr["hub_v"]))
        sides = {"AB": (("A", "left"), ("B", "right")), "CB": (("C", "right"), ("B", "left"))}
        for name in lay["seams"]:
            ts = [lay["walls"][k]["ends"][side]["t"] for k, side in sides[name]]
            heights = [lay["walls"][k]["height"] for k, _side in sides[name]]
            # 継ぎ目の端は低い方の壁の上端に乗る(もう一方は細る)
            assert min(abs(t - h) for t, h in zip(ts, heights)) < 1e-6
        for key, fr in lay["walls"].items():
            w = dr["walls"][key]
            for side in ("left", "right"):
                end = fr["ends"][side]
                if end is None:
                    continue
                notch = drawn_notch_mm(w["fillet_mm"], end["turn"])
                q_s = end["s"] + (notch * end["dt"] if side == "left" else -notch * end["dt"])
                for s_c, r in w["tabs"]:
                    f = tab_footprint_mm(r, w["tab_root_r_mm"])   # 根元Rを含む占有幅
                    if side == "left":
                        assert s_c - f >= q_s - 1e-6, "タブ(根元R込み)はノッチより内側"
                    else:
                        assert s_c + f <= q_s + 1e-6


def test_points_on_walls_tab_and_hub():
    for spec, *_ in _draw():
        pts = spec.annotated_points
        dr = spec.drawn
        lay = drawn_tray_frames(dr["hub_xy"], dr["walls"], origin=tuple(dr["origin"]),
                                hub_u=tuple(dr["hub_u"]), hub_v=tuple(dr["hub_v"]))
        hub = [p for p in pts if p.normal_xyz == lay["normal"]]
        assert len(hub) == DRAWN_HUB_POINTS
        n_tabs = sum(len(w["tabs"]) for w in dr["walls"].values())
        for key, fr in lay["walls"].items():
            on_wall = [p for p in pts if p.normal_xyz == fr["normal"]]
            assert len(on_wall) == len(dr["walls"][key]["tabs"])
            for p in on_wall:
                t = _dot(tuple(p.position_xyz[i] - fr["a"][i] for i in range(3)), fr["tip"])
                assert abs(t - fr["height"]) < 1e-6, "溶接はタブの円の中心(壁の上端の高さ)"
        assert len(pts) - n_tabs - DRAWN_HUB_POINTS in (1, 2), "曲げタブの溶接は 1〜2"
        normals = {tuple(round(c, 2) for c in p.normal_xyz) for p in pts}
        assert len(normals) == 5, "壁3 + タブ + ハブ"

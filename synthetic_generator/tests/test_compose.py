"""合成族(因子が独立に変わる部品、AutoMetalSheet 依頼 2026-09-06、第2期)の不変条件。

OCCT を呼ばない範囲だけ見る。形状の成立は tools/run_occt_batch.py で通す。
"""
from __future__ import annotations

import collections
import random

from synthetic_generator.families import (
    COMPOSE_ARMS, COMPOSE_FOLDS, COMPOSE_POINTS, Knobs, compose_part,
)


def _draw(count: int = 40):
    rng = random.Random(20260906)
    out = []
    while len(out) < count:
        got = compose_part(rng, Knobs())
        if got is not None:
            out.append(got)
    return out


def test_factors_are_recorded_and_realised_within_ranges():
    seen = collections.defaultdict(set)
    for spec, bead, flange, rib in _draw():
        b = spec.min_bearing_radius_mm
        assert flange is None, "第2期: 壁はフランジではなく短い壁腕"
        cp = spec.compose
        f = cp["factors"]
        assert set(cp["factors_sampled"]) <= set(f)
        assert f["folds"] in COMPOSE_FOLDS and f["arms"] in COMPOSE_ARMS
        assert f["points"] == len(spec.annotated_points) and 2 <= f["points"] <= max(COMPOSE_POINTS)
        assert len(cp["point_radii"]) == len(spec.annotated_points)
        assert f["section"] == ("bead" if bead else "rib" if rib else "none")
        walls = [a for a in cp["arms"] if a["role"] == "wall"]
        arms = [a for a in cp["arms"] if a["role"] == "arm"]
        assert f["walls"] == {0: "none", 1: "one", 2: "both"}[len(walls)]
        assert f["arms"] == len(arms) and f["notches"] == len(cp["notches"])
        assert f["tabs"] == sum(1 for a in arms if a["outline"]["kind"] == "tabs")
        assert all(not w["points"] for w in walls), "壁腕に締結点は無い"
        if bead is not None:
            assert cp["bead_span"] is not None
        for n in cp["notches"]:
            assert n["reason"] in ("waist", "bend_relief"), "切欠きは理由のあるものだけ"
        for k, v in f.items():
            seen[k].add(v)
    # 因子ごとに複数の値が出る(族名で構造が決まらない)
    for k in ("folds", "section", "walls", "arms", "base", "points"):
        assert len(seen[k]) >= 2, k


def test_tab_circles_are_larger_than_the_seat_and_radii_are_uniform():
    """裁定 2026-09-09: 1部品1半径。タブ円だけは座面より大きくする。"""
    for spec, bead, flange, rib in _draw():
        cp = spec.compose
        b = spec.min_bearing_radius_mm
        radii = list(cp["point_radii"])
        assert all(r == b for r in radii), "1部品1半径"
        k = 2
        for a in cp["arms"]:
            k += len(a["points"])
            if a["outline"]["kind"] == "tabs":
                for _s, r in a["outline"]["tabs"]:
                    # 裁定 2026-09-09: タブ円は座面半径より大きい
                    assert r > b
        assert cp["weld_bearing_mm"] == b

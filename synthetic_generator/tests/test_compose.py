"""合成族(因子が独立に変わる部品、AutoMetalSheet 依頼 2026-09-06)の不変条件。

OCCT を呼ばない範囲だけ見る。形状の成立は tools/run_occt_batch.py で通す。
"""
from __future__ import annotations

import collections
import random

from synthetic_generator.families import (
    COMPOSE_ARMS, COMPOSE_FOLDS, COMPOSE_POINTS, COMPOSE_WELD_BEARING_MM, Knobs, compose_part,
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
        cp = spec.compose
        f = cp["factors"]
        assert set(cp["factors_sampled"]) <= set(f)
        assert f["folds"] in COMPOSE_FOLDS and f["arms"] in COMPOSE_ARMS
        assert f["points"] == len(spec.annotated_points) and 2 <= f["points"] <= max(COMPOSE_POINTS)
        assert f["section"] == ("bead" if bead else "rib" if rib else "none")
        assert f["walls"] == ("both" if flange and flange.both_sides else "one" if flange else "none")
        assert f["arms"] == len(cp["arms"]) and f["notches"] == len(cp["notches"])
        assert f["tabs"] == sum(1 for a in cp["arms"] if a["outline"]["kind"] == "tabs")
        for k, v in f.items():
            seen[k].add(v)
    # 因子ごとに複数の値が出る(族名で構造が決まらない)
    for k in ("folds", "section", "walls", "arms", "base", "points"):
        assert len(seen[k]) >= 2, k


def test_arms_avoid_flange_side_and_tabs_use_weld_bearing():
    for spec, bead, flange, rib in _draw():
        cp = spec.compose
        blocked = set() if flange is None else ({-1, 1} if flange.both_sides else {flange.side})
        for a in cp["arms"]:
            assert a["side"] not in blocked, "フランジのある側に腕は置かない"
            assert a["t1_mm"] > a["t0_mm"]
            if a["outline"]["kind"] == "tabs":
                for _s, r in a["outline"]["tabs"]:
                    assert COMPOSE_WELD_BEARING_MM[0] <= r <= COMPOSE_WELD_BEARING_MM[1]
                    assert abs(r - cp["weld_bearing_mm"]) < 1e-9
        if rib is not None:
            assert flange is None, "リブとフランジは併存しない"

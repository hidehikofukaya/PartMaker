"""分岐ファミリ(実車026)の不変条件。2026-09-05。

OCCTを呼ばずに済む範囲だけ見る。形状の成立は tools/run_occt_batch.py で通す。
"""
from __future__ import annotations

import math
import random

from synthetic_generator.families import (
    BRANCH_GUSSET_FOLD_DEG, BRANCH_GUSSET_PROB, Knobs, branch_part,
)


def _draw(count: int = 24):
    rng = random.Random(20260905)
    knobs = Knobs()
    out = []
    while len(out) < count:
        got = branch_part(rng, knobs)
        if got is not None:
            out.append(got)
    return out


def test_branch_has_three_or_four_arms_on_a_convex_hub():
    for spec, bead, flange, rib in _draw():
        assert (bead, flange, rib) == (None, None, None), "分岐族は断面特徴を持たない"
        br = spec.branch
        assert br is not None
        assert 3 <= len(br["arms"]) <= 4
        assert len({a["edge"] for a in br["arms"]}) == len(br["arms"]), "1辺に腕は1本"
        # 凸四角形(反時計回り): 隣り合う辺の外積が全て正
        xy = br["hub_xy"]
        for i in range(4):
            a, b, c = xy[i - 1], xy[i], xy[(i + 1) % 4]
            turn = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            assert turn > 0.0, "ハブは凸(展開図が自己交差しない根拠)"


def test_gusset_corners_have_both_arms_at_ninety_degrees():
    """ガセットは隣り合う90度腕2本の間にだけ入る(側端が平行でないと壁が垂直にならない)。"""
    seen = 0
    for spec, _bead, _flange, _rib in _draw(40):
        br = spec.branch
        for vtx in br["gussets"]:
            seen += 1
            edges = {a["edge"]: a for a in br["arms"]}
            assert (vtx - 1) % 4 in edges and vtx in edges, "ガセットの両隣に腕がある"
            assert edges[(vtx - 1) % 4]["fold_deg"] == BRANCH_GUSSET_FOLD_DEG
            assert edges[vtx]["fold_deg"] == BRANCH_GUSSET_FOLD_DEG
    if BRANCH_GUSSET_PROB > 0.0:
        assert seen > 0, "40件引いてガセットが1つも出ないのは異常"
    else:
        assert seen == 0, "ガセットは無効のはず"


def test_branch_joints_sit_on_arms_and_hub_with_distinct_normals():
    for spec, _bead, _flange, _rib in _draw():
        pts = spec.annotated_points
        assert len(pts) >= 3
        normals = {tuple(round(c, 3) for c in p.normal_xyz) for p in pts}
        assert len(normals) >= 3, "腕ごとに法線が違う = 3つ以上の面を締結している"
        # 対の最小距離(必要平面が重ならない配置の目安)
        gap = min(math.dist(a.position_xyz, b.position_xyz)
                  for i, a in enumerate(pts) for b in pts[i + 1:])
        assert gap >= 15.0

"""チャンネル + 座面ファミリ(実車144)の不変条件。2026-09-05。

OCCTを呼ばずに済む範囲だけ見る。形状の成立は tools/run_occt_batch.py で通す。
"""
from __future__ import annotations

import math
import random

from synthetic_generator.families import (
    CHANNEL_MIN_R_MM, CHANNEL_THICKNESS_MM, CHANNEL_WALL_FOLD_DEG, Knobs, channel_seat_part,
)
from synthetic_generator.occt_build import channel_seat_frames


def _draw(count: int = 24):
    rng = random.Random(20260905)
    knobs = Knobs()
    out = []
    while len(out) < count:
        got = channel_seat_part(rng, knobs)
        if got is not None:
            out.append(got)
    return out


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def test_walls_always_open_toward_the_seats():
    """座面側が長辺の台形: 壁の折れ角は最大88度、最小Rは4、板厚0.6。"""
    for spec, bead, flange, rib in _draw():
        assert (bead, flange, rib) == (None, None, None)
        ch = spec.channel
        assert CHANNEL_WALL_FOLD_DEG[0] <= ch["wall_fold_deg"] <= 88.0
        assert ch["wall_depth1_mm"] > ch["wall_depth0_mm"], "壁の下辺は斜め"
        assert min(ch["wall_radius_mm"], ch["seat_radius_mm"]) >= CHANNEL_MIN_R_MM
        assert spec.thickness_mm == CHANNEL_THICKNESS_MM


def test_seats_share_one_normal_and_web_has_exactly_one_point():
    for spec, _bead, _flange, _rib in _draw():
        pts = spec.annotated_points
        assert len(pts) in (3, 5), "座面各1〜2点 + ウェブ1点"
        geom = {k: v for k, v in spec.channel.items()
                if k not in ("seat_depth_mm", "seat_corner_mm", "diag_deg", "seat_width_mm")}
        lay = channel_seat_frames(**geom)
        a = lay["walls"]["A"]["seat_normal"]
        b = lay["walls"]["B"]["seat_normal"]
        assert _dot(a, b) > 0.9999, "座面2枚の法線は一致"
        seat_pts = [p for p in pts if _dot(p.normal_xyz, a) > 0.9999]
        web_pts = [p for p in pts if _dot(p.normal_xyz, lay["normal"]) > 0.9999]
        assert len(web_pts) == 1 and len(seat_pts) == len(pts) - 1
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, _dot(a, lay["normal"])))))
        assert 10.0 < tilt < 70.0, "ウェブの締結点は座面に対して一定の角度差"
        gap = min(math.dist(p.position_xyz, q.position_xyz)
                  for i, p in enumerate(pts) for q in pts[i + 1:])
        assert gap >= 20.0 - 1e-6, "点間は他族と同じ最小20mm(実車の対は31mm)"

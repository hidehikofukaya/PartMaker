"""3締結点ファミリ(実車007/011の再現、2026-09-04)の骨格を守る。

OCCTを呼ばずに済む範囲だけ見る — 「法線が一致する2点が同一パネル、3点目が曲げの
向こう側、余白は従来ルール」という設計上の不変条件。実際の形状生成は
`tools/run_occt_batch.py tools/recipes/occt13.json` で通す。
"""
from __future__ import annotations

import math
import random

from synthetic_generator.families import Knobs, three_point_part
from synthetic_generator.general_geometry import plan_for


def _angle_deg(a, b) -> float:
    dot = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    return math.degrees(math.acos(dot))


def _draw(count: int = 12):
    rng = random.Random(4649)
    knobs = Knobs()
    out = []
    while len(out) < count:
        got = three_point_part(rng, knobs)
        if got is not None:
            out.append(got)
    return out


def test_three_points_with_one_matching_normal_pair():
    for spec, bead, flange, rib in _draw():
        assert bead is None and rib is None and flange is not None, "特徴はフランジのみ"
        assert len(spec.extra_points) == 1, "締結点は3つ"
        mate = spec.extra_points[0]
        # 3点目は point1 か point2 のどちらか一方とだけ法線が一致する
        to_1 = _angle_deg(mate.normal_xyz, spec.point1.normal_xyz)
        to_2 = _angle_deg(mate.normal_xyz, spec.point2.normal_xyz)
        assert min(to_1, to_2) < 0.01, "対の法線は一致する"
        assert max(to_1, to_2) > 40.0, "もう1点の法線は明確に違う(折れ角ぶん)"


def test_single_fold_between_the_pair_and_the_lone_point():
    for spec, _bead, _flange, _rib in _draw():
        plan = plan_for(spec)
        assert spec.target_folds == 1
        assert len(plan.panel_frames) == 2, "2点群の間の曲げは1本"
        fold = _angle_deg(spec.point1.normal_xyz, spec.point2.normal_xyz)
        assert 40.0 < fold < 95.0, f"折れ角が実車帯から外れた: {fold:.1f}deg"


def test_mate_keeps_the_existing_margin_rules():
    """帯端・曲げの接線・パネル端から min_bearing_radius_mm 以上(従来ルール踏襲)。"""
    for spec, _bead, _flange, _rib in _draw():
        plan = plan_for(spec)
        mate = spec.extra_points[0]
        bearing = spec.min_bearing_radius_mm
        hosts = [f for f, anchor in zip(plan.panel_frames, (spec.point1, spec.point2))
                 if _angle_deg(mate.normal_xyz, anchor.normal_xyz) < 0.01]
        assert hosts, "対の相方は既存の締結点と同じパネルに載る"
        frame = hosts[0]
        index = plan.panel_frames.index(frame)
        near_cut, far_cut = plan.fold_tangents[index]
        delta = [mate.position_xyz[i] - frame.origin[i] for i in range(3)]
        run = sum(delta[i] * frame.u[i] for i in range(3))
        across = sum(delta[i] * frame.v[i] for i in range(3))
        assert frame.near_run_mm + near_cut + bearing - 1e-9 <= run
        assert run <= frame.far_run_mm - far_cut - bearing + 1e-9
        assert abs(across) <= spec.half_width_mm - bearing + 1e-9
        # パネル上に載っている(法線方向の外れが無い)
        normal_off = sum(delta[i] * (frame.u[(i + 1) % 3] * frame.v[(i + 2) % 3]
                                     - frame.u[(i + 2) % 3] * frame.v[(i + 1) % 3])
                         for i in range(3))
        assert abs(normal_off) < 1e-6, "相方はパネル平面上にある"

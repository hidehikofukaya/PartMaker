import math
import random

from synthetic_generator.classify import (
    MIN_BASE_BEND_RADIUS_MM,
    MIN_NEUTRAL_PLANE_RADIUS_MM,
    classify,
    fold_tangent_length_mm,
)
from synthetic_generator.templates.parallel_same_offset import (
    BEND_RADIUS_CAP_MM,
    FLANGE_WIDTH_SAFETY_MARGIN_MM,
    sample,
)


def _lateral_distance(spec) -> float:
    """point1->point2の締結軸(point1の法線)方向を除いた接平面内距離(=frame.run_length_mm相当)。"""
    n = spec.point1.normal_xyz
    n_len = math.sqrt(sum(c * c for c in n))
    n = tuple(c / n_len for c in n)
    delta = tuple(b - a for a, b in zip(spec.point1.position_xyz, spec.point2.position_xyz))
    offset = sum(d * c for d, c in zip(delta, n))
    tangential_sq = sum(d * d for d in delta) - offset * offset
    return math.sqrt(max(0.0, tangential_sq))


def _offset_mm(spec) -> float:
    """point1->point2の締結軸(point1の法線)方向成分(=frame.offset_mm相当)。"""
    n = spec.point1.normal_xyz
    n_len = math.sqrt(sum(c * c for c in n))
    n = tuple(c / n_len for c in n)
    delta = tuple(b - a for a, b in zip(spec.point1.position_xyz, spec.point2.position_xyz))
    return sum(d * c for d, c in zip(delta, n))


def test_sample_always_classifies_as_parallel_same_offset() -> None:
    rng = random.Random(42)
    ramp_extents = []
    bend_radii = []
    for _ in range(300):
        spec = sample(rng)
        assert classify(spec.point1, spec.point2) == "parallel_same_offset"
        assert 1.0 <= spec.thickness_mm <= 2.5
        assert 6.0 <= spec.hole_diameter_mm <= 14.0
        assert 15.0 <= spec.min_bearing_radius_mm <= 30.0
        # panel_width should track 2x the bearing radius (rough, low-waste sizing), not be
        # an unrelated arbitrary size -- this is the "avoid creating excess material from the
        # start" requirement ahead of the exact Phase 1.5 trim. A fixed safety margin
        # (2026-08-06, side-flange width clearance) is added on top of the ratio-based term.
        margin = 2.0 * FLANGE_WIDTH_SAFETY_MARGIN_MM
        assert 2.0 * spec.min_bearing_radius_mm + margin <= spec.panel_width_mm <= 2.5 * spec.min_bearing_radius_mm + margin

        # jog_ramp_extent_mm must never eat into either fastening point's minimum bearing
        # radius -- this has to hold by construction, not by luck.
        lateral = _lateral_distance(spec)
        assert 0.0 <= spec.jog_ramp_extent_mm <= max(0.0, lateral - 2.0 * spec.min_bearing_radius_mm) + 1e-6
        ramp_extents.append(spec.jog_ramp_extent_mm)

        # bend_radius_mmはメイン曲げの下限R10(ユーザー確定、2026-08-24)を常に満たす。
        # 幾何的に本来の実行可能上限がR10未満の場合はあえてR10(=実行可能上限を超える値)を
        # 返し、gsd_build.py側の事前チェックで明示的にInfeasibleとして弾かれる設計。
        # そうでない場合は締結点のmin_bearing_radius_mmを侵さない範囲に収まっているはず。
        assert MIN_BASE_BEND_RADIUS_MM <= spec.bend_radius_mm <= BEND_RADIUS_CAP_MM
        x_start = (lateral - spec.jog_ramp_extent_mm) / 2.0
        offset = _offset_mm(spec)
        tangent_length = fold_tangent_length_mm(offset, spec.jog_ramp_extent_mm, spec.bend_radius_mm)
        clear = x_start - tangent_length
        if clear < spec.min_bearing_radius_mm - 1e-6:
            # 実行可能上限が4mm未満だったケース: あえて最小値ちょうどを返しているはず
            assert abs(spec.bend_radius_mm - MIN_BASE_BEND_RADIUS_MM) < 1e-9
        bend_radii.append(spec.bend_radius_mm)

    # 300件も引けば、鋭い直角に近いケースも緩やかなランプに近いケースも両方出るはず
    assert min(ramp_extents) < 1.0
    assert max(ramp_extents) > 10.0
    # bend_radius_mmもランダム性が保たれているはず(下限付近から上限付近まで幅がある)
    assert min(bend_radii) < MIN_BASE_BEND_RADIUS_MM + 1.0
    assert max(bend_radii) > MIN_BASE_BEND_RADIUS_MM + 5.0


def test_sample_is_deterministic_given_seeded_rng() -> None:
    spec_a = sample(random.Random(7))
    spec_b = sample(random.Random(7))
    assert spec_a == spec_b

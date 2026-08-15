import math
import random

from synthetic_generator.classify import (
    MIN_NEUTRAL_PLANE_RADIUS_MM,
    classify,
    end_panel_corners,
    ramp_fold_angles_rad,
    tangent_length_for_bend_angle_rad,
    two_point_frame,
)
from synthetic_generator.templates.general_two_point import BEND_RADIUS_CAP_MM, sample


def _feasible_bend_radius_bound(spec) -> float:
    """gsd_build.build_general_two_pointが実際に見る事前チェックの実行可能上限を、
    サンプリング側とは独立に再計算する(テスト用の照合式)。"""
    frame = two_point_frame(spec.point1, spec.point2)
    flat1_corners = end_panel_corners(
        spec.point1.position_xyz, frame.u1, frame.w, -spec.min_bearing_radius_mm, spec.fold1_run_mm, spec.half_width_mm
    )
    flat2_corners = end_panel_corners(
        spec.point2.position_xyz, frame.u2, frame.w, -spec.fold2_run_mm, spec.min_bearing_radius_mm, spec.half_width_mm
    )
    mid_near = tuple((flat1_corners[2][i] + flat1_corners[3][i]) / 2 for i in range(3))
    mid_far = tuple((flat2_corners[0][i] + flat2_corners[1][i]) / 2 for i in range(3))
    fold1_angle, fold2_angle = ramp_fold_angles_rad(mid_near, mid_far, frame.u1, frame.u2)
    ramp_length = math.sqrt(sum((mid_far[i] - mid_near[i]) ** 2 for i in range(3)))

    unit_t1 = tangent_length_for_bend_angle_rad(fold1_angle, 1.0)
    unit_t2 = tangent_length_for_bend_angle_rad(fold2_angle, 1.0)
    bound1 = BEND_RADIUS_CAP_MM if unit_t1 < 1e-9 else (spec.fold1_run_mm - spec.min_bearing_radius_mm) / unit_t1
    bound2 = BEND_RADIUS_CAP_MM if unit_t2 < 1e-9 else (spec.fold2_run_mm - spec.min_bearing_radius_mm) / unit_t2
    bound_ramp = BEND_RADIUS_CAP_MM if (unit_t1 + unit_t2) < 1e-9 else ramp_length / (unit_t1 + unit_t2)
    return min(BEND_RADIUS_CAP_MM, bound1, bound2, bound_ramp, 0.9 * spec.half_width_mm)


def test_sample_covers_multiple_configuration_classes() -> None:
    """parallel_same_offset.pyと違い、このサンプラーは特定の1クラスに限定しない —
    300件も引けば複数のconfiguration classが出現するはず。"""
    rng = random.Random(42)
    classes_seen = set()
    for _ in range(300):
        spec = sample(rng)
        classes_seen.add(classify(spec.point1, spec.point2))
    assert len(classes_seen) >= 3


def test_sample_respects_manufacturing_ranges() -> None:
    rng = random.Random(1)
    for _ in range(300):
        spec = sample(rng)
        assert 1.0 <= spec.thickness_mm <= 2.5
        assert 6.0 <= spec.hole_diameter_mm <= 14.0
        assert 15.0 <= spec.min_bearing_radius_mm <= 30.0
        assert spec.half_width_mm >= spec.min_bearing_radius_mm
        assert spec.fold1_run_mm > spec.min_bearing_radius_mm
        assert spec.fold2_run_mm > spec.min_bearing_radius_mm
        assert MIN_NEUTRAL_PLANE_RADIUS_MM <= spec.bend_radius_mm <= BEND_RADIUS_CAP_MM


def test_sample_bend_radius_is_feasible_by_construction_or_forced_to_minimum() -> None:
    """bend_radius_mmは、独立に再計算した実行可能上限の範囲内にあるか、実行可能上限が
    4mm未満のときはあえて最小値ちょうどになっている(parallel_same_offset.pyと同じ設計)。"""
    rng = random.Random(7)
    saw_forced_minimum = False
    saw_comfortable_margin = False
    for _ in range(300):
        spec = sample(rng)
        bound = _feasible_bend_radius_bound(spec)
        if bound < MIN_NEUTRAL_PLANE_RADIUS_MM:
            assert abs(spec.bend_radius_mm - MIN_NEUTRAL_PLANE_RADIUS_MM) < 1e-9
            saw_forced_minimum = True
        else:
            assert spec.bend_radius_mm <= bound + 1e-6
            if spec.bend_radius_mm > MIN_NEUTRAL_PLANE_RADIUS_MM + 1.0:
                saw_comfortable_margin = True

    # 300件も引けば両方のケースが出現するはず(歩留まりの実態を反映)
    assert saw_forced_minimum
    assert saw_comfortable_margin


def test_sample_is_deterministic_given_seeded_rng() -> None:
    spec_a = sample(random.Random(7))
    spec_b = sample(random.Random(7))
    assert spec_a == spec_b

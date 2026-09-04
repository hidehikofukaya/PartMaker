import math
import random

from synthetic_generator.classify import MIN_BASE_BEND_RADIUS_MM, classify
from synthetic_generator.templates.general_two_point import (
    BEND_RADIUS_CAP_MM,
    FOLD_SLACK_RANGE_MM,
    FOLD_TILT_PERTURBATION_RANGE_DEG,
    sample,
)


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
        # ユーザー確定(2026-08-24): 締結点1つの最小必要平面は直径25mm、板幅は25〜50mm
        assert 12.5 <= spec.min_bearing_radius_mm <= 25.0
        assert spec.half_width_mm >= spec.min_bearing_radius_mm
        assert spec.half_width_mm <= 25.0
        assert MIN_BASE_BEND_RADIUS_MM <= spec.bend_radius_mm <= BEND_RADIUS_CAP_MM
        assert spec.bend_radius_mm <= 0.9 * spec.half_width_mm + 1e-9
        assert FOLD_SLACK_RANGE_MM[0] <= spec.fold1_slack_mm <= FOLD_SLACK_RANGE_MM[1]
        assert FOLD_SLACK_RANGE_MM[0] <= spec.fold2_slack_mm <= FOLD_SLACK_RANGE_MM[1]
        lo, hi = (math.radians(v) for v in FOLD_TILT_PERTURBATION_RANGE_DEG)
        assert lo <= spec.fold1_tilt_perturbation_rad <= hi


def test_sample_bend_radius_is_feasible_by_construction() -> None:
    """bend_radius_mmは、half_widthから決まる実行可能上限(0.9倍)の範囲内にある。

    SS8.3の設計変更(R先出し)により、以前はランプ幾何(fold1_run_mm/fold2_run_mm由来の
    接線長)にも依存していたため、その上限がメイン曲げの下限(R10)未満になり「あえて
    R10を強制する」ケースがあった。その依存はgsd_build.build_general_two_point側の
    権威あるチェック(free_fold_seed/solve_free_fold)に移り、サンプラーが把握できる
    独立な上限はhalf_widthの0.9倍だけになった。half_width_mm>=12.5(bearing radius下限)
    なので0.9*half_width_mm>=11.25はMIN_BASE_BEND_RADIUS_MM(10)を必ず上回り、
    「強制的に最小値」の分岐は新設計では構造的に発生しない。
    """
    rng = random.Random(7)
    saw_comfortable_margin = False
    for _ in range(300):
        spec = sample(rng)
        bound = min(BEND_RADIUS_CAP_MM, 0.9 * spec.half_width_mm)
        assert bound >= MIN_BASE_BEND_RADIUS_MM
        assert MIN_BASE_BEND_RADIUS_MM <= spec.bend_radius_mm <= bound + 1e-6
        if spec.bend_radius_mm > MIN_BASE_BEND_RADIUS_MM + 1.0:
            saw_comfortable_margin = True

    assert saw_comfortable_margin


def test_sample_is_deterministic_given_seeded_rng() -> None:
    spec_a = sample(random.Random(7))
    spec_b = sample(random.Random(7))
    assert spec_a == spec_b

import random

import pytest

from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM
from synthetic_generator.reinforcement import (
    CORNER_RADIUS_MIN_MM,
    FLANGE_ANGLE_DEFAULT_DEG,
    FLANGE_ANGLE_OBLIQUE_RANGE_DEG,
    FLANGE_BEND_RADIUS_RATIO_CAP,
    sample_reinforcement,
)


def test_sample_reinforcement_respects_manufacturing_constraints() -> None:
    rng = random.Random(0)
    angles_seen = set()
    flange_bend_radii = []
    for _ in range(200):
        params = sample_reinforcement(thickness_mm=1.6, rng=rng)
        assert params.corner_radius_mm >= CORNER_RADIUS_MIN_MM
        assert params.flange_angle_deg == FLANGE_ANGLE_DEFAULT_DEG or (
            FLANGE_ANGLE_OBLIQUE_RANGE_DEG[0] <= params.flange_angle_deg <= FLANGE_ANGLE_OBLIQUE_RANGE_DEG[1]
        )
        assert params.flange_height_mm > 0
        assert 0.0 <= params.reinforcement_direction_deg <= 180.0
        # フランジ根本Rは中立面R最小4mm(ユーザー製造制約、2026-08-06)を満たし、
        # かつflange_height_mmを超えない(超えると隣接パネル長を超えて実機で破綻する
        # ことがSS6.9で判明したため)。thickness_mm=1.6ではflange_heightの下限
        # (height_ratio下限3.0)でも0.9倍が4mmを上回るため、この組み合わせでは
        # 常に「真に実行可能」なはず(強制的な最小値マーカーにはならない)。
        assert MIN_NEUTRAL_PLANE_RADIUS_MM <= params.flange_bend_radius_mm <= 0.9 * params.flange_height_mm + 1e-9
        assert params.flange_bend_radius_mm <= FLANGE_BEND_RADIUS_RATIO_CAP * 1.6 + 1e-9
        angles_seen.add(params.flange_angle_deg == FLANGE_ANGLE_DEFAULT_DEG)
        flange_bend_radii.append(params.flange_bend_radius_mm)

    # 200件も引けば90度・斜めフランジの両方が出現するはず(既定確率85%)
    assert angles_seen == {True, False}
    # flange_bend_radius_mmもランダム性が保たれているはず(4mm付近から上限付近まで)
    assert min(flange_bend_radii) < MIN_NEUTRAL_PLANE_RADIUS_MM + 0.3
    assert max(flange_bend_radii) > MIN_NEUTRAL_PLANE_RADIUS_MM + 0.3


def test_sample_reinforcement_flange_bend_radius_forced_to_minimum_when_infeasible() -> None:
    """flange_height_mmが小さすぎてR4以上のフィレットが成立しない場合、
    flange_bend_radius_mmはあえて最小値ちょうどを返す(gsd_build.py側の事前チェックで
    明示的にInfeasibleとして弾かれるようにするため)。"""
    rng = random.Random(0)
    # thickness=1.0, height_ratio=2.0 -> flange_height=2.0mm, 0.9倍=1.8mm < 4mm
    params = sample_reinforcement(thickness_mm=1.0, rng=rng, flange_height_ratio_range=(2.0, 2.0))
    assert params.flange_height_mm == pytest.approx(2.0)
    assert params.flange_bend_radius_mm == pytest.approx(MIN_NEUTRAL_PLANE_RADIUS_MM)


def test_sample_reinforcement_scales_with_thickness() -> None:
    rng = random.Random(1)
    thin = sample_reinforcement(thickness_mm=1.0, rng=rng, flange_height_ratio_range=(5.0, 5.0))
    thick = sample_reinforcement(thickness_mm=2.0, rng=rng, flange_height_ratio_range=(5.0, 5.0))
    assert thick.flange_height_mm == pytest.approx(2 * thin.flange_height_mm)


def test_sample_reinforcement_rejects_nonpositive_thickness() -> None:
    with pytest.raises(ValueError):
        sample_reinforcement(thickness_mm=0.0, rng=random.Random(0))


def test_sample_reinforcement_rejects_corner_radius_max_below_min() -> None:
    with pytest.raises(ValueError):
        sample_reinforcement(thickness_mm=1.5, rng=random.Random(0), corner_radius_max_mm=1.0)

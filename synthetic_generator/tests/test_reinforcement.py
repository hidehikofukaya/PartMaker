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
        # フランジ根本Rは中立面R最小(ユーザー製造制約)を満たし、かつflange_height_mmを
        # 超えない(超えると隣接パネル長を超えて実機で破綻することがSS6.9で判明)。
        # 2026-08-24にR最小が4→5mmへ上がった結果、thickness=1.6mmでは
        # FLANGE_BEND_RADIUS_RATIO_CAP*1.6=4.8mm < 5mm となり、真の実行可能上限が
        # 常にR最小を下回る。この場合サンプラは「あえてR最小ちょうど」を返して
        # 下流の事前チェックに弾かせる設計なので、そのマーカーを許容する。
        assert params.flange_bend_radius_mm >= MIN_NEUTRAL_PLANE_RADIUS_MM
        forced = params.flange_bend_radius_mm == MIN_NEUTRAL_PLANE_RADIUS_MM
        assert forced or params.flange_bend_radius_mm <= 0.9 * params.flange_height_mm + 1e-9
        assert forced or params.flange_bend_radius_mm <= FLANGE_BEND_RADIUS_RATIO_CAP * 1.6 + 1e-9
        angles_seen.add(params.flange_angle_deg == FLANGE_ANGLE_DEFAULT_DEG)
        flange_bend_radii.append(params.flange_bend_radius_mm)

    # 200件も引けば90度・斜めフランジの両方が出現するはず(既定確率85%)
    assert angles_seen == {True, False}
    # 【既知の設計衝突・2026-08-24】中立面R最小がR5に上がった結果、板厚1.6mmでは
    # フランジ根本Rの真の実行可能上限(FLANGE_BEND_RADIUS_RATIO_CAP*1.6 = 4.8mm)が
    # R5を下回るため、flange_bend_radius_mmは常に強制最小値5.0になりランダム性が無い。
    # = この板厚ではフランジが常にInfeasibleとして弾かれる、という状態。
    # フランジは既定で無効(include_flanges=False)なので現状の生成はブロックしないが、
    # フランジを再度有効にする際は「根本Rは小さく保つ(1〜3×板厚)」という以前の
    # ユーザー確定方針とR5最小が両立しないため、どちらを採るかの判断が要る。
    assert set(flange_bend_radii) == {MIN_NEUTRAL_PLANE_RADIUS_MM}


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

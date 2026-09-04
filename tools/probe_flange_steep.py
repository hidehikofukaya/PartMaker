"""急な折れ角(実車の90度)でフランジが成立するかを測る(2026-09-04)。

families.flange_part は最大折れ角20度以下しかフランジを許さない(CATIA時代の
「急な曲げではフランジ不成立」という申し送り)。しかし実車007/011のフランジは
90度の曲げを跨いでいる。OCCT版の実際の拘束は occt_build._check_flange_radii の
凹側クリアランス R - h >= 2mm だけで、折れ角そのものは効かないはず。
その仮説を、折れ角の帯ごとの生成成功率で確かめる。

使い方: python tools/probe_flange_steep.py [1帯あたりの試行数]
"""
from __future__ import annotations

import collections
import dataclasses
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.families import (  # noqa: E402
    Knobs, _draw_spec, max_fold_angle_deg, plan_flange_on_surface, plan_for, sample_flange,
)
from synthetic_generator.occt_build import OcctPartBuilder  # noqa: E402

BANDS = [(0, 20), (20, 45), (45, 70), (70, 100), (100, 135)]


def main() -> None:
    per_band = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    out = pathlib.Path(__file__).resolve().parent / "probe_output" / "flange_steep"
    out.mkdir(parents=True, exist_ok=True)
    builder = OcctPartBuilder()
    rng = random.Random(20260904)
    knobs = Knobs()

    made: collections.Counter = collections.Counter()
    drawn: collections.Counter = collections.Counter()
    reasons: dict = collections.defaultdict(collections.Counter)

    def band_of(angle):
        for lo, hi in BANDS:
            if lo <= angle < hi:
                return (lo, hi)
        return None

    # 折れ角の帯が埋まるまで引き続ける
    for _ in range(per_band * len(BANDS) * 60):
        if all(drawn[b] >= per_band for b in BANDS):
            break
        try:
            spec = _draw_spec(rng, knobs)
            plan = plan_for(spec)
        except ValueError:
            continue
        band = band_of(max_fold_angle_deg(plan.panel_frames))
        if band is None or drawn[band] >= per_band:
            continue
        drawn[band] += 1

        flange = sample_flange(rng, plan.panel_frames, plan.fold_tilts, spec.half_width_mm,
                               spec.bend_radius_mm, spec.point1.normal_xyz)
        if flange is None:
            reasons[band]["sample_flange が None"] += 1
            continue
        extension = (flange.extension_mm if flange.side < 0 else 0.0,
                     flange.extension_mm if flange.side > 0 else 0.0)
        try:
            wide = plan_for(spec, side_extension_mm=extension)
            plan_flange_on_surface(wide.panel_frames, flange, half_width_mm=spec.half_width_mm,
                                   fold_tangents=wide.fold_tangents)
        except ValueError as exc:
            reasons[band][f"配置: {str(exc)[:44]}"] += 1
            continue
        try:
            part = builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm, bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm, fold2_slack_mm=spec.fold2_slack_mm,
                target_folds=spec.target_folds,
                out_dir=str(out), part_name=f"steep_{sum(drawn.values()):04d}", flange=flange)
        except Exception as exc:
            reasons[band][f"生成: {type(exc).__name__}: {str(exc)[:36]}"] += 1
            continue
        made[band] += 1

    print(f"{'折れ角の帯':<14} {'試行':>5} {'成功':>5} {'成功率':>7}   主な失敗")
    print("-" * 96)
    for band in BANDS:
        n, ok = drawn[band], made[band]
        top = "; ".join(f"{k}({v})" for k, v in reasons[band].most_common(2))
        print(f"{band[0]:>3}-{band[1]:<3}deg     {n:>5} {ok:>5} {ok / max(1, n):>7.0%}   {top}")


if __name__ == "__main__":
    main()

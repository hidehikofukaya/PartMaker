"""「両側フランジ + 中央ビード」の断面が成立するかを測る(2026-09-04、実車014)。

実車014は 007/011 と違い、フランジが**両側**にあり、さらに帯の**中央にビード**が通って
1点と2点の間の剛性を確保している(tools/probe_section_profile.py で実測)。合成側は
現在 1部品1特徴で、フランジも片側だけなので、どちらも作れない。

合成側の断面は (帯幅方向, 基準面からの高さ) の要素列でしかないので、両者の合成は
「ビード断面の両端の平地を、フランジの根本R+壁で挟む」だけで書ける。それが本当に
掃引・縫合・全ゲートを通るかを、この探りで確かめる(族には未接続)。

使い方: python tools/probe_flange_bead_section.py [試行数]
"""
from __future__ import annotations

import collections
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

import synthetic_generator.occt_build as ob  # noqa: E402
from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.families import Knobs, _draw_spec  # noqa: E402
from synthetic_generator.flange import sample_flange  # noqa: E402
from synthetic_generator.general_geometry import (  # noqa: E402
    check_bead_feasible_occt, plan_for)

SQRT_HALF = math.sqrt(0.5)


def flange_bead_section(flange, section, half_width_mm):
    """ビード断面の両端の平地に、左右それぞれ根本R+壁を継ぐ。

    要素列は左から右へ連続していないと `_edges_of` が繋がらないので、左の壁は
    上端から根本へ、右の壁は根本から上端へ向ける。
    """
    r, h, e = flange.root_radius_mm, flange.height_mm, flange.direction
    if h <= r + 1.0:
        raise ValueError("flange height leaves no straight wall above the root radius")
    edge = half_width_mm
    # 根本Rの中心は (±edge, e*r)。円弧の中点は45度方向。
    left_mid = (-(edge + r * SQRT_HALF), e * r - e * r * SQRT_HALF)
    right_mid = (edge + r * SQRT_HALF, e * r - e * r * SQRT_HALF)
    return [
        ("line", (-(edge + r), e * h), (-(edge + r), e * r), "flange_wall_l"),
        ("arc", (-(edge + r), e * r), left_mid, (-edge, 0.0), "flange_root_l"),
        *section,
        ("arc", (edge, 0.0), right_mid, (edge + r, e * r), "flange_root_r"),
        ("line", (edge + r, e * r), (edge + r, e * h), "flange_wall_r"),
    ]


def check_both_sides(builder, path, flange, bend_radius_mm: float) -> None:
    """凹側クリアランス。**左右の壁は同じ向き(flange.direction)に立つ**ので、
    曲げ軸(wに平行)からの半径は ez だけで決まり、帯幅方向の位置 side には依らない。
    したがって片側フランジと同じ判定で足りる — 既存の `_check_flange_radii` を使う。
    """
    builder._check_flange_radii(path, flange, bend_radius_mm)


def wrap_flat(flat_section, flange, half_width_mm):
    """平地断面にも同じ壁を継ぐ(ランアウトの loft は要素が1対1で対応する必要がある
    ので、ビード断面と平地断面で壁の要素数を揃える)。"""
    r, h, e = flange.root_radius_mm, flange.height_mm, flange.direction
    edge = half_width_mm
    left_mid = (-(edge + r * SQRT_HALF), e * r - e * r * SQRT_HALF)
    right_mid = (edge + r * SQRT_HALF, e * r - e * r * SQRT_HALF)
    return [
        ("line", (-(edge + r), e * h), (-(edge + r), e * r), "flange_wall_l"),
        ("arc", (-(edge + r), e * r), left_mid, (-edge, 0.0), "flange_root_l"),
        *flat_section,
        ("arc", (edge, 0.0), right_mid, (edge + r, e * r), "flange_root_r"),
        ("line", (edge + r, e * r), (edge + r, e * h), "flange_wall_r"),
    ]


def main() -> None:
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    out = pathlib.Path(__file__).resolve().parent / "probe_output" / "flange_bead"
    out.mkdir(parents=True, exist_ok=True)
    builder = ob.OcctPartBuilder()
    rng = random.Random(20260904)
    # 実車014の帯: 折れは計90度、フランジは低め、帯幅50mm前後
    # 実車014: 単独点まで57mm、帯幅50mm、折れ計90度。脚を伸ばさないとビードが載らない。
    knobs = Knobs(bearing_radius_mm=(12.5, 16.0), half_width_ratio=(1.6, 2.2),
                  max_half_width_mm=30.0, bend_radius_mm=(12.0, 18.0),
                  turn_range_deg=(45.0, 90.0), leg_slack_mm=(25.0, 70.0))

    made = drawn = 0
    fails: collections.Counter = collections.Counter()
    raw_bead, raw_flat = ob._bead_section, ob._flat_section
    for i in range(trials):
        try:
            spec = _draw_spec(rng, knobs, folds=1)
            plan = plan_for(spec)
        except ValueError:
            continue
        flange = sample_flange(rng, plan.panel_frames, plan.fold_tilts, spec.half_width_mm,
                               spec.bend_radius_mm, spec.point1.normal_xyz)
        if flange is None:
            fails["sample_flange が None"] += 1
            continue
        # ビード族と同じ事前判定: 載らない置き方は引き直す(ランアウトが座面の間に
        # 収まるか)。断面の合成とは無関係な既存の問題なので、ここで潰しておく。
        bead = None
        for _ in range(12):
            candidate = sample_bead(rng, spec.half_width_mm)
            try:
                check_bead_feasible_occt(plan, candidate, spec.bend_radius_mm)
            except ValueError:
                continue
            bead = candidate
            break
        if bead is None:
            fails["ビードが載る置き方を引けない"] += 1
            continue
        drawn += 1

        def bead_patch(b, lift, hw, *, role="bead", _f=flange):
            section, breaks = raw_bead(b, lift, hw, role=role)
            return flange_bead_section(_f, section, hw), breaks

        def flat_patch(breaks, _f=flange):
            return wrap_flat(raw_flat(breaks), _f, spec.half_width_mm)

        try:
            path, _s, _w, _n = ob._build_path(plan, spec.bend_radius_mm)
            check_both_sides(builder, path, flange, spec.bend_radius_mm)
            ob._bead_section, ob._flat_section = bead_patch, flat_patch
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm, bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm, fold2_slack_mm=spec.fold2_slack_mm,
                target_folds=spec.target_folds, out_dir=str(out),
                part_name=f"fb_{i:03d}", bead=bead)
            made += 1
        except Exception as exc:
            fails[f"{type(exc).__name__}: {str(exc)[:56]}"] += 1
        finally:
            ob._bead_section, ob._flat_section = raw_bead, raw_flat

    print(f"引けた {drawn}/{trials}   ビルド成功 {made}/{max(1, drawn)} = {made/max(1,drawn):.0%}")
    for k, v in fails.most_common(6):
        print(f"   {v:3d}  {k}")


if __name__ == "__main__":
    main()

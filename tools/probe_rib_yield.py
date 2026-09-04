"""リブ生成の失敗を包括的に層別する(2026-09-04)。

「どの段で落ちるか」だけでなく、**落ちた形状の寸法**を記録して、成功する寸法帯と
失敗する寸法帯を切り分ける。目標は成立率50%以上。

使い方: python tools/probe_rib_yield.py [試行回数]
"""
from __future__ import annotations

import collections
import math
import pathlib
import random
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.families import FAMILIES, Knobs, fold_angle_deg  # noqa: E402
from synthetic_generator.general_geometry import plan_for  # noqa: E402
from synthetic_generator.occt_build import OcctPartBuilder  # noqa: E402
from synthetic_generator.rib import triangle_inradius_mm  # noqa: E402

STAGES = (
    ("filleting", "フィレット失敗"),
    ("not valid", "無効シェル"),
    ("deviates", "ゲートA(プリミティブずれ)"),
    ("junk edge", "ゴミエッジ"),
    ("closed loops", "外形が崩壊"),
    ("fold back", "面が折り返る"),
    ("off the surface", "締結点が外れる"),
    ("came back empty", "空のSTEP"),
)


def stage_of(message: str) -> str:
    for key, label in STAGES:
        if key in message:
            return label
    return message[:40]


def main() -> None:
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 300
    out = pathlib.Path(__file__).resolve().parent / "probe_output" / "rib_yield"
    out.mkdir(parents=True, exist_ok=True)
    builder = OcctPartBuilder()
    knobs = Knobs(distance_mm=(50.0, 120.0), fold_weights=((1, 0.7), (2, 0.3)),
                  bearing_radius_mm=(15.0, 22.0))
    rng = random.Random(20260914)

    stages: collections.Counter = collections.Counter()
    good: list = []
    bad: dict = collections.defaultdict(list)
    start = time.time()
    drawn = 0
    for _ in range(trials):
        result = FAMILIES["rib"](rng, knobs)
        if result is None:
            stages["候補が引けない"] += 1
            continue
        spec, _bead, _flange, rib = result
        drawn += 1
        plan = plan_for(spec)
        angle = fold_angle_deg(plan.panel_frames, rib.fold_index)
        inradius = triangle_inradius_mm(rib.half_width_mm, rib.leg1_mm, rib.leg2_mm,
                                        math.radians(angle))
        record = {
            "c": rib.half_width_mm, "leg1": rib.leg1_mm, "leg2": rib.leg2_mm,
            "angle": angle, "inradius": inradius, "half_width": spec.half_width_mm,
            "ratio": rib.half_width_mm / spec.half_width_mm,
            "leg_ratio": min(rib.leg1_mm, rib.leg2_mm) / rib.half_width_mm,
        }
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                target_folds=spec.target_folds,
                out_dir=str(out), part_name="probe", rib=rib)
        except ValueError as exc:
            label = stage_of(str(exc))
            stages[label] += 1
            bad[label].append(record)
            continue
        stages["成功"] += 1
        good.append(record)

    seconds = time.time() - start
    total = sum(stages.values())
    print(f"{trials}試行 / {seconds:.0f}秒  候補 {drawn}件、成立 {len(good)}件 "
          f"(候補あたり {len(good) / max(1, drawn):.1%})\n")
    print("段別:")
    for label, n in stages.most_common():
        print(f"  {n:5d}  ({n / total:5.1%})  {label}")

    def band(records, key, edges):
        counts = [0] * (len(edges) + 1)
        for r in records:
            value = r[key]
            index = sum(1 for e in edges if value >= e)
            counts[index] += 1
        return counts

    print("\n寸法帯ごとの成立率(成功 / 成功+失敗):")
    for key, edges, label in (
        ("inradius", [6.5, 7.5, 9.0, 11.0], "リブ三角形の内接円半径[mm]"),
        ("angle", [60.0, 75.0, 95.0, 115.0], "折れ角[度]"),
        ("ratio", [0.5, 0.6, 0.7], "リブ半幅 / 板半幅"),
        ("leg_ratio", [1.3, 1.6, 2.0], "脚 / リブ半幅"),
    ):
        every = [r for records in bad.values() for r in records] + good
        hits, alls = band(good, key, edges), band(every, key, edges)
        names = [f"<{edges[0]}"] + [f"{a}-{b}" for a, b in zip(edges, edges[1:])] + [f">={edges[-1]}"]
        print(f"  {label}")
        for name, hit, all_ in zip(names, hits, alls):
            rate = f"{hit / all_:5.1%}" if all_ else "   - "
            print(f"    {name:>10}: {rate}  ({hit}/{all_})")


if __name__ == "__main__":
    main()

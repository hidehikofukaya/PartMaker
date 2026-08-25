"""壁のスイープ失敗は「長さの取り方」で直るのかを実測する(2026-08-25、ユーザー質問)。

失敗の実体は「掃引はできるが頂部エッジが期待位置から3.3〜10.0mmずれる」。
判定は`GetMinimumDistance(バンド, 期待頂部点)`なので、**バンドを伸ばせば距離は
単調に縮むだけ**である。したがって:
  - 壁の向きが正しく、長さが足りないだけなら、伸ばせば距離は0に落ちる
  - 向き自体が違うなら、いくら伸ばしても0にはならない(平行にすれ違う)
1.0/1.5/2/3/5倍で振って、どちらなのかを決める。
"""
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

import synthetic_generator.gsd_build as gsd_build  # noqa: E402
from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

ATTEMPTS = int(sys.argv[1]) if len(sys.argv) > 1 else 40
RATIOS = (1.0, 1.5, 2.0, 3.0, 5.0)


def main() -> None:
    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    for i in range(catia.Documents.Count, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass

    cur: dict = {}
    original_plan = gsd_build.plan_bead_on_surface

    def capture_plan(panel_frames, bead, **kwargs):
        plan = original_plan(panel_frames, bead, **kwargs)
        cur["n_bead_panels"] = len(plan.top_panel_index)
        return plan

    gsd_build.plan_bead_on_surface = capture_plan

    def sweep_scan(doc, part, hsf, spa, body, curve_ref, surface_ref, top_refs, bead, label):
        theta = bead.wall_angle_deg
        best_per_ratio = []
        for ratio in RATIOS:
            best = None
            for angle in (theta, 180.0 - theta, -theta, -(180.0 - theta)):
                sweep = hsf.AddNewSweepLine(curve_ref)
                sweep.Mode = 4
                sweep.FirstGuideSurf = surface_ref
                sweep.SetAngle(1, angle)
                sweep.SetLength(1, bead.wall_slant_mm * ratio)
                body.AppendHybridShape(sweep)
                try:
                    part.Update()
                    m = spa.GetMeasurable(part.CreateReferenceFromObject(sweep))
                    d = max(m.GetMinimumDistance(r) for r in top_refs)
                    best = d if best is None else min(best, d)
                except Exception:
                    pass
                builder._delete_feature(doc, part, sweep)
            best_per_ratio.append(f"{best:5.2f}" if best is not None else "    X")
        print(f"[{cur['attempt']:03d}] panels={cur.get('n_bead_panels', '?')} "
              f"slant={bead.wall_slant_mm:5.1f}  " +
              "  ".join(f"x{r:g}:{v}" for r, v in zip(RATIOS, best_per_ratio)), flush=True)
        raise ValueError("scan stop")

    builder._bead_wall = sweep_scan

    rng = random.Random(20260825)
    for attempt in range(1, ATTEMPTS + 1):
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        cur.clear()
        cur["attempt"] = attempt
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                out_dir=str(pathlib.Path(__file__).resolve().parent / "probe_output"),
                part_name=f"len_{attempt:03d}", bead=bead,
            )
        except Exception as exc:
            if "scan stop" not in str(exc):
                pass


if __name__ == "__main__":
    main()

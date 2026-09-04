"""Mode=4スイープが落ちる理由を切り分ける(2026-08-25)。

実測されている症状は2つある:
  (a) ±(180-θ)だけ通り、±θ(内向き)が落ちる  → 角度側の問題
  (b) 4角度すべて落ちる                       → 投影した閉曲線側の問題
どちらなのかは、①曲線そのものの健全性(CurveParが引けるか)②長さ1mmでも落ちるか
③角度を振ったときにどこから落ちるか、の3点で分かる。`_bead_wall`を診断版に
差し替えて、失敗する試行を一通り観測する。
"""
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

ATTEMPTS = 20


def main() -> None:
    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    for i in range(catia.Documents.Count, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass

    state = {"spec": {}, "rows": [], "plan": None}

    # 本番の外形(基準面をSplitしてから境界オフセット)と、トリム無しの境界オフセットを
    # 同条件で比べるため、planを横取りしておく。
    import synthetic_generator.gsd_build as gsd_build
    original_plan = gsd_build.plan_bead_on_surface

    def capture_plan(*args, **kwargs):
        plan = original_plan(*args, **kwargs)
        state["plan"] = plan
        return plan

    gsd_build.plan_bead_on_surface = capture_plan

    def diagnostic_wall(doc, part, hsf, spa, body, curve_ref, surface_ref, top_refs, bead, label):
        theta = bead.wall_angle_deg

        def try_sweep(angle, length):
            sweep = hsf.AddNewSweepLine(curve_ref)
            sweep.Mode = 4
            sweep.FirstGuideSurf = surface_ref
            sweep.SetAngle(1, angle)
            sweep.SetLength(1, length)
            body.AppendHybridShape(sweep)
            try:
                part.Update()
                d = max(spa.GetMeasurable(part.CreateReferenceFromObject(sweep))
                        .GetMinimumDistance(r) for r in top_refs)
                result = f"{d:5.2f}"
            except Exception:
                result = "    X"
            builder._delete_feature(doc, part, sweep)
            return result

        # 外形曲線そのものの健全性: 長さと、期待される頂部エッジまでの距離。
        # 壁が直線ルーリングなら根元->頂部の距離は斜辺長slantに一致するはず。
        cm = spa.GetMeasurable(curve_ref)
        outline_mm = cm.Length
        to_top = max(cm.GetMinimumDistance(r) for r in top_refs)
        curve_ok = try_sweep(5.0, 1.0) != "    X"

        inward = try_sweep(theta, bead.wall_slant_mm)
        outward = try_sweep(180.0 - theta, bead.wall_slant_mm)
        state["rows"].append(dict(state["spec"], curve_ok=curve_ok, inward=inward,
                                  outward=outward, depth=bead.depth_mm, theta=theta,
                                  topw=bead.top_width_mm, hf=bead.half_footprint_mm,
                                  slant=bead.wall_slant_mm, outline=outline_mm, to_top=to_top))
        # A/B: トリム無しの境界をそのまま同じ量だけオフセットした曲線でも掃引してみる。
        untrimmed = "n/a"
        try:
            boundary = hsf.AddNewBoundaryOfSurface(surface_ref)
            body.AppendHybridShape(boundary)
            part.Update()
            alt = hsf.AddNewCurvePar(part.CreateReferenceFromObject(boundary), surface_ref,
                                     state["plan"].outline_offset_mm, False, True)
            body.AppendHybridShape(alt)
            part.Update()
            alt_ref = part.CreateReferenceFromObject(alt)
            state["rows"][-1]["bnd0"] = spa.GetMeasurable(
                part.CreateReferenceFromObject(boundary)).Length
            state["rows"][-1]["alt"] = spa.GetMeasurable(alt_ref).Length
            results = []
            for angle in (theta, 180.0 - theta, -theta, -(180.0 - theta)):
                sweep = hsf.AddNewSweepLine(alt_ref)
                sweep.Mode = 4
                sweep.FirstGuideSurf = surface_ref
                sweep.SetAngle(1, angle)
                sweep.SetLength(1, bead.wall_slant_mm)
                body.AppendHybridShape(sweep)
                try:
                    part.Update()
                    results.append("O")
                except Exception:
                    results.append("X")
                builder._delete_feature(doc, part, sweep)
            untrimmed = "".join(results)
        except Exception as exc:
            untrimmed = "ERR"
        state["rows"][-1]["untrimmed"] = untrimmed
        raise ValueError("diagnostic stop")

    builder._bead_wall = diagnostic_wall

    rng = random.Random(20260825)
    for attempt in range(1, ATTEMPTS + 1):
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        state["spec"] = dict(attempt=attempt, bendr=spec.bend_radius_mm,
                             hw=spec.half_width_mm)
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
                part_name=f"iso_{attempt:03d}",
                bead=bead,
            )
        except Exception as exc:
            if "diagnostic stop" not in str(exc):
                continue
    print("attempt bendR   hw  depth theta  topW    hf slant outline to_top curve inward outward", flush=True)
    for r in state["rows"]:
        print(f"{r['attempt']:5d} {r['bendr']:6.1f} {r['hw']:4.1f} {r['depth']:5.1f} "
              f"{r['theta']:5.1f} {r['topw']:5.1f} {r['hf']:5.1f} {r['slant']:5.1f} "
              f"{r['outline']:7.1f} {r['to_top']:6.2f} "
              f"{'OK ' if r['curve_ok'] else 'BAD'}  {r['inward']}  {r['outward']}  "
              f"untrimmed={r.get('untrimmed', '?')} "
              f"bnd0={r.get('bnd0', 0):.1f} alt={r.get('alt', 0):.1f}", flush=True)


if __name__ == "__main__":
    main()

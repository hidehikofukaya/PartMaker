"""四隅Rの各段階を別々のCATPartに保存し、Rが正しく付いているかを実測する
(2026-08-25、ユーザー依頼: 四隅にRを着ける直前まで生成し、正しくつける検討を)。

出力(tools/probe_output/corner_stages/):
  stage1_sharp.CATPart   四隅R適用**直前** = 尖り外形(単段オフセット)の壁バンド
  stage2_rounded.CATPart 四隅R適用後の壁バンド(根本・頂稜線フィレットはまだ)
  stage3_full.CATPart    全工程(本番経路そのまま)

実測(stage2で):
  root_sharp  尖り角位置(start_run, ±hf)からバンドまでの距離。
              根本が丸まっていれば cR*(1-cos45) ≈ 0.29*cR だけ離れるはず。0なら尖ったまま。
  root_arc    根本コーナー円弧の中点(45度位置)からバンドまでの距離。丸まっていれば ~0。
  top_pinch   頂部のコーナー半径は cR - wall_run に縮む。これが小さいと頂部四隅は
              実質尖って見える(ユーザー指摘の疑い筋)。値を印字する。
"""
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import plan_bead_on_surface, sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import (  # noqa: E402
    resolve_bead_slacks,
    sample as sample_spec,
)

OUT = pathlib.Path(__file__).resolve().parent / "probe_output" / "corner_stages"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    for i in range(catia.Documents.Count, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass

    state: dict = {}

    def staged(doc, part, hsf, spa, body, surface, *, bead, panel_frames, half_width_mm,
               min_bearing_radius_mm, fold_tangents, fold_tilts):
        mode = state["mode"]
        surface_ref = part.CreateReferenceFromObject(surface)
        plan = plan_bead_on_surface(
            panel_frames, bead, inset_mm=2.0 * min_bearing_radius_mm,
            guide_margin_mm=builder.BEAD_GUIDE_MARGIN_MM, half_width_mm=half_width_mm,
            fold_tangents=fold_tangents, fold_tilts=fold_tilts)
        from synthetic_generator.bead import _cross, _normalize
        normals = [_normalize(_cross(f.u, f.v)) for f in panel_frames]

        # ① 頂面オフセット(壁の向き判定プローブに必要)
        wall_samples = [(b, normals[i]) for b, i in zip(plan.wall_top_bases, plan.wall_top_panel_index)]
        _top, top_ref, signs = builder._bead_top_offset(
            doc, part, hsf, spa, body, surface_ref, bead, wall_samples)
        wall_tops = [tuple(b[i] + signs[k] * bead.depth_mm * n[i] for i in range(3))
                     for k, (b, n) in enumerate(wall_samples)]

        # ②トリム(本番と同一)
        trimmed_ref = surface_ref
        keep_ref = builder._point_refs(part, hsf, body, [plan.trim_keep_probe])[0]
        remove_refs = builder._point_refs(part, hsf, body, plan.trim_remove_probes)
        for s_index, section in enumerate(plan.trim_sections):
            corner_refs = builder._point_refs(part, hsf, body, list(section))
            plane = hsf.AddNewPlane3Points(*corner_refs)
            body.AppendHybridShape(plane)
            part.Update()
            plane_ref = part.CreateReferenceFromObject(plane)
            kept = None
            for orientation in (1, -1):
                split = hsf.AddNewHybridSplit(trimmed_ref, plane_ref, orientation)
                body.AppendHybridShape(split)
                try:
                    part.Update()
                except Exception:
                    builder._delete_feature(doc, part, split)
                    continue
                split_ref = part.CreateReferenceFromObject(split)
                m = spa.GetMeasurable(split_ref)
                if (m.GetMinimumDistance(keep_ref) < 0.15
                        and m.GetMinimumDistance(remove_refs[s_index]) > 1.0):
                    kept = split_ref
                    break
                builder._delete_feature(doc, part, split)
            if kept is None:
                raise ValueError("trim failed in probe")
            trimmed_ref = kept
        boundary = hsf.AddNewBoundaryOfSurface(trimmed_ref)
        body.AppendHybridShape(boundary)
        part.Update()
        boundary_ref = part.CreateReferenceFromObject(boundary)

        # ③外形: sharp=単段 / rounded=2段(本番と同一)
        def parallel(source_ref, offset_mm, probe_refs, tol, corner_type=0):
            for reverse in (False, True):
                cp = hsf.AddNewCurvePar(source_ref, surface_ref, offset_mm, reverse, True)
                if corner_type:
                    cp.CurveParType = corner_type  # 1 = Round(GUIの「コーナータイプ」)
                body.AppendHybridShape(cp)
                try:
                    part.Update()
                except Exception:
                    builder._delete_feature(doc, part, cp)
                    continue
                ref = part.CreateReferenceFromObject(cp)
                m = spa.GetMeasurable(ref)
                if max(m.GetMinimumDistance(r) for r in probe_refs) < tol:
                    return ref
                builder._delete_feature(doc, part, cp)
            raise ValueError("parallel curve failed in probe")

        outline_probe_refs = builder._point_refs(part, hsf, body, plan.outline_probes)
        if mode == "sharp":
            outline_ref = parallel(boundary_ref, plan.outline_offset_mm, outline_probe_refs, 0.15)
        else:
            inner_probe = builder._point_refs(part, hsf, body, [plan.inner_loop_probe])[0]
            inner_ref = parallel(
                boundary_ref, plan.outline_offset_mm + bead.corner_radius_mm, [inner_probe], 1.0)
            outline_ref = parallel(inner_ref, bead.corner_radius_mm, outline_probe_refs, 0.15,
                                   corner_type=1)

        # ④壁バンド(本番の角度選定そのまま)
        wall_top_refs = builder._point_refs(part, hsf, body, wall_tops)
        _band, band_ref = builder._bead_wall(
            doc, part, hsf, spa, body, outline_ref, surface_ref, wall_top_refs, bead, "band")

        # 実測(四隅Rの有無)
        hf = bead.half_footprint_mm
        cr = bead.corner_radius_mm
        first = panel_frames[0]
        inset = 2.0 * min_bearing_radius_mm
        start_run = first.near_run_mm + inset

        def at(frame, run, width):
            return tuple(frame.origin[i] + run * frame.u[i] + width * frame.v[i] for i in range(3))

        sharp_pt = at(first, start_run, hf)
        c45 = 1.0 - math.cos(math.radians(45))
        arc_pt = at(first, start_run + cr * c45, hf - cr * c45)
        refs = builder._point_refs(part, hsf, body, [sharp_pt, arc_pt])
        m = spa.GetMeasurable(band_ref)
        d_sharp = m.GetMinimumDistance(refs[0])
        d_arc = m.GetMinimumDistance(refs[1])
        print(f"  [{mode:>7}] 尖り角位置->バンド {d_sharp:5.2f}mm (丸なら~{0.29 * cr:.2f}) / "
              f"円弧中点->バンド {d_arc:5.2f}mm (丸なら~0) / "
              f"root cR={cr:.1f} top cR={cr - bead.wall_run_mm:.1f}", flush=True)

        target = OUT / f"stage_{mode}.CATPart"
        doc.SaveAs(str(target))
        print(f"  保存: {target}", flush=True)
        raise ValueError("stage stop")

    # --- 対象の(spec, bead)を1組選ぶ: 3パネルにビード区間があり、頂部cRも十分残るもの ---
    rng = random.Random(424242)
    chosen = None
    while chosen is None:
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        resolved = resolve_bead_slacks(rng, spec, bead)
        if resolved is None:
            continue
        spec, bead = resolved
        if bead.corner_radius_mm - bead.wall_run_mm < 4.0:
            continue  # 頂部の四隅Rが薄い組はデモに不向き
        chosen = (spec, bead)
    spec, bead = chosen
    print(f"spec確定: cR={bead.corner_radius_mm:.1f} wall_run={bead.wall_run_mm:.1f} "
          f"hf={bead.half_footprint_mm:.1f} depth={bead.depth_mm:.1f} "
          f"角度={bead.wall_angle_deg:.1f}", flush=True)

    def build(mode, name):
        state["mode"] = mode
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm, bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm, fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                out_dir=str(OUT), part_name=name, bead=bead)
            return True
        except Exception as exc:
            if "stage stop" not in str(exc):
                print(f"  [{mode}] 失敗: {str(exc)[:90]}", flush=True)
                return False
            return True

    original = builder._add_bead_to_surface
    builder._add_bead_to_surface = staged
    print("stage1: 四隅R適用直前(尖り外形の壁バンド)", flush=True)
    build("sharp", "s1")
    print("stage2: 四隅R適用後の壁バンド", flush=True)
    build("rounded", "s2")
    builder._add_bead_to_surface = original
    print("stage3: 全工程(本番経路)", flush=True)
    if build("full", "stage3_full"):
        print(f"  保存: {OUT / 'stage3_full_mid.CATPart'}", flush=True)


if __name__ == "__main__":
    main()

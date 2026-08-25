"""角R付き閉曲線を「中心線の測地オフセット+解析キャップ/円弧のJoin」で作る案の実機検証
(2026-08-25、ユーザー指摘: 壁同士の縦エッジにコーナーRが無い)。

境界オフセット方式(SS10)は頑健だが、内側オフセットは凸角を尖ったまま残すので、
掃引した壁バンドの四隅が尖る。エッジ(BRep)参照フィレットは自動化不可(SS4)なので、
**外形曲線側にコーナーRを織り込む**しかない。構成:

  - 長辺2本: 中心線(解析点→スプライン→投影、SS8.9で実証)を±half_footprintへ
    測地オフセット(AddNewCurvePar、24/24実証)。中心線の走行範囲を
    [start_run+cR, end_run-cR]に絞れば、端点がそのままコーナー接点になる(トリム不要)。
  - キャップ2本: 端パネルの平坦区間内の直線(解析座標が厳密に面上)。
  - 円弧4本: 同じく平坦区間内の解析5点スプライン。
  - 全8要素をJoin → Mode=4スイープ。

検証点: (a) CurvePar端点が解析コーナー接点に一致するか (b) Joinが閉曲線として通るか
(c) そのJoinをMode=4スイープが受けるか (d) 掃引が全ドラフト角で成立するか。
"""
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import (  # noqa: E402
    resolve_bead_slacks,
    sample as sample_spec,
)

ATTEMPTS = 12
OUT = pathlib.Path(__file__).resolve().parent / "probe_output" / "corner_outline"


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

    state = {"n": 0}

    def experiment(*args, **kwargs):
        try:
            return _experiment(*args, **kwargs)
        except ValueError:
            raise
        except Exception:
            import traceback
            traceback.print_exc()
            raise ValueError("probe stop")

    def _experiment(doc, part, hsf, spa, body, surface, *, bead, panel_frames, half_width_mm,
                    min_bearing_radius_mm, fold_tangents, fold_tilts):
        n = state["n"]
        surface_ref = part.CreateReferenceFromObject(surface)
        hf = bead.half_footprint_mm
        inset = 2.0 * min_bearing_radius_mm
        first, last = panel_frames[0], panel_frames[-1]
        start_run = first.near_run_mm + inset
        end_run = last.far_run_mm - inset

        def at(frame, run, width):
            return tuple(frame.origin[i] + run * frame.u[i] + width * frame.v[i] for i in range(3))

        # コーナーR: 内側ループのキャップ幅 2*(hf-cR) が残ること、端パネルの平坦区間に
        # トリム位置が収まることから上限が決まる
        flat_high0 = first.far_run_mm - fold_tangents[0][1] - 1.0
        flat_low0 = first.near_run_mm + 1.0
        corner_r = min(0.8 * hf, 8.0, hf - 1.5)
        d2 = half_width_mm - hf + corner_r
        trim_start = start_run + corner_r - d2
        trim_end = end_run - corner_r + d2
        ok_start = flat_low0 <= trim_start <= flat_high0
        flat_low_last = last.near_run_mm + fold_tangents[-1][0] + 1.0
        flat_high_last = last.far_run_mm - 1.0
        ok_end = flat_low_last <= trim_end <= flat_high_last
        if not (ok_start and ok_end):
            print(f"[{n:02d}] トリム位置が平坦区間外 (cR={corner_r:.1f}, d2={d2:.1f})", flush=True)
            raise ValueError("probe stop")

        # --- ①トリム(既存経路と同じ: 平面Split x2、keep/removeプローブ) ---
        keep_ref = builder._point_refs(part, hsf, body, [
            at(panel_frames[len(panel_frames) // 2],
               (panel_frames[len(panel_frames) // 2].near_run_mm
                + panel_frames[len(panel_frames) // 2].far_run_mm) / 2.0, 0.0)])[0]
        trimmed_ref = surface_ref
        for frame, run_mm in ((first, trim_start), (last, trim_end)):
            normal = [
                frame.u[1] * frame.v[2] - frame.u[2] * frame.v[1],
                frame.u[2] * frame.v[0] - frame.u[0] * frame.v[2],
                frame.u[0] * frame.v[1] - frame.u[1] * frame.v[0],
            ]
            base_pt = at(frame, run_mm, 0.0)
            plane_refs = builder._point_refs(part, hsf, body, [
                base_pt, at(frame, run_mm, half_width_mm),
                tuple(base_pt[i] + 10.0 * normal[i] for i in range(3))])
            plane = hsf.AddNewPlane3Points(*plane_refs)
            body.AppendHybridShape(plane)
            part.Update()
            plane_ref = part.CreateReferenceFromObject(plane)
            kept = None
            for orientation in (1, -1):
                piece = hsf.AddNewHybridSplit(trimmed_ref, plane_ref, orientation)
                body.AppendHybridShape(piece)
                try:
                    part.Update()
                except Exception:
                    builder._delete_feature(doc, part, piece)
                    continue
                piece_ref = part.CreateReferenceFromObject(piece)
                if spa.GetMeasurable(piece_ref).GetMinimumDistance(keep_ref) < 0.15:
                    kept = piece_ref
                    break
                builder._delete_feature(doc, part, piece)
            if kept is None:
                print(f"[{n:02d}] トリム失敗", flush=True)
                raise ValueError("probe stop")
            trimmed_ref = kept

        # --- ②境界 → 内側d2オフセット(尖った内側ループ) ---
        boundary = hsf.AddNewBoundaryOfSurface(trimmed_ref)
        body.AppendHybridShape(boundary)
        part.Update()
        boundary_ref = part.CreateReferenceFromObject(boundary)
        inner_probe = builder._point_refs(part, hsf, body, [
            at(first, start_run + corner_r + 2.0, hf - corner_r)])[0]
        inner_ref = None
        for reverse in (False, True):
            cp = hsf.AddNewCurvePar(boundary_ref, surface_ref, d2, reverse, True)
            body.AppendHybridShape(cp)
            try:
                part.Update()
            except Exception:
                builder._delete_feature(doc, part, cp)
                continue
            ref = part.CreateReferenceFromObject(cp)
            if spa.GetMeasurable(ref).GetMinimumDistance(inner_probe) < 1.0:
                inner_ref = ref
                break
            builder._delete_feature(doc, part, cp)
        if inner_ref is None:
            print(f"[{n:02d}] 内側ループNG (d2={d2:.1f})", flush=True)
            raise ValueError("probe stop")

        # --- ③外向きcRオフセット → 四隅R付き外形 ---
        outline_probes = builder._point_refs(part, hsf, body, [
            at(first, start_run + corner_r + 2.0, -hf),
            at(first, start_run + corner_r + 2.0, hf),
            at(first, start_run, 0.0),
            at(last, end_run, 0.0),
        ])
        outline_ref = None
        for reverse in (False, True):
            cp = hsf.AddNewCurvePar(inner_ref, surface_ref, corner_r, reverse, True)
            body.AppendHybridShape(cp)
            try:
                part.Update()
            except Exception:
                builder._delete_feature(doc, part, cp)
                continue
            ref = part.CreateReferenceFromObject(cp)
            m = spa.GetMeasurable(ref)
            worst = max(m.GetMinimumDistance(pr) for pr in outline_probes)
            if worst < 0.15:
                outline_ref = ref
                break
            builder._delete_feature(doc, part, cp)
        if outline_ref is None:
            print(f"[{n:02d}] 外向きオフセットNG (cR={corner_r:.1f})", flush=True)
            raise ValueError("probe stop")

        # --- ④スイープ ---
        def try_sweep(profile_ref, angle, length):
            sweep = hsf.AddNewSweepLine(profile_ref)
            sweep.Mode = 4
            sweep.FirstGuideSurf = surface_ref
            sweep.SetAngle(1, angle)
            sweep.SetLength(1, length)
            body.AppendHybridShape(sweep)
            try:
                part.Update()
                ok = True
            except Exception:
                ok = False
            builder._delete_feature(doc, part, sweep)
            return ok

        theta = bead.wall_angle_deg
        row = [f"{a:+.0f}:{'O' if try_sweep(outline_ref, a, bead.wall_slant_mm) else 'X'}"
               for a in (theta, 180.0 - theta, -theta, -(180.0 - theta))]
        print(f"[{n:02d}] cR={corner_r:.1f} hf={hf:.1f} d2={d2:.1f}  " + " ".join(row), flush=True)
        doc.SaveAs(str(OUT / f"co_{n:03d}.CATPart"))
        raise ValueError("probe stop")

    builder._add_bead_to_surface = experiment

    rng = random.Random(20260825)
    done = 0
    attempt = 0
    while done < ATTEMPTS and attempt < 120:
        attempt += 1
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        resolved = resolve_bead_slacks(rng, spec, bead)
        if resolved is None:
            continue
        spec, bead = resolved
        state["n"] = attempt
        done += 1
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                out_dir=str(OUT), part_name=f"co_{attempt:03d}", bead=bead,
            )
        except Exception as exc:
            if "probe stop" not in str(exc):
                print(f"[{attempt:02d}] 事前脱落: {str(exc)[:70]}", flush=True)


if __name__ == "__main__":
    main()

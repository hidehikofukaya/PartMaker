"""折れ目を跨ぐフランジ構成のスパイク(2026-08-25、ユーザー仕様①〜⑤の実機検証)。

ユーザー指定の工程:
  ①基準面を作成し、フランジ作成可能か判断
  ②長手方向エッジが面内の弧なら、外側の弧のエッジへ
  ③エッジからコーナーR+1〜2mm分の面を拡張(外挿コマンド)
  ④拡張面の外周エッジからドラフト方向90度でスイープ
  ⑤拡張面とスイープを結合してエッジフィレット→完成

本スパイクでの写像(未実証APIを避け、ビードで実証済みのプリミティブに寄せる):
  ③ 外挿(AddNewExtrapol)は未実証でBRep境界参照が要る疑いがある。代わりに
     「基準面を最初から拡張分だけ広く作る」案を採る予定だが、このスパイクでは
     既存の面の端をそのまま「拡張後の外周」とみなす(形は同じ、名目幅が縮むだけ)。
  ④ 側端曲線 = 中心線(投影)の測地オフセット(CurvePar、実証済み)。
     そこからMode=4・角度±90度のスイープ(ビード壁と同一プリミティブ)。
  ⑤ エッジフィレット(BRep参照、自動化不可)ではなく、拡張面×壁の
     AddNewFilletBiTangent(トリム込み、ビード足元Rと同一プリミティブ)。

検証点:
  (a) 側端近傍の測地オフセット曲線が折れ目を跨いで引けるか
  (b) 90度スイープが折れ目を跨いで成立するか(凸側/凹側の両方を観測)
  (c) BiTangent根本Rが折れ目を跨ぐ長い根本に沿って成立するか
"""
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.general_geometry import plan_general_two_point  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

OUT = pathlib.Path(__file__).resolve().parent / "probe_output" / "flange_spike"
FLANGE_HEIGHT_MM = 8.0   # 凹側の折れ目を跨ぐ場合 h < 基準面R(>=10) が必要なので控えめに
FLANGE_ROOT_R_MM = 5.0
MAX_FOLD_DEG = 50.0      # 「曲げが少ない」基準面だけを対象にする(スパイクでは50度)
ATTEMPTS = 6


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
            raise ValueError("spike stop")

    def _experiment(doc, part, hsf, spa, body, surface, *, bead, panel_frames, half_width_mm,
                    min_bearing_radius_mm, fold_tangents, fold_tilts):
        n = state["n"]
        surface_ref = part.CreateReferenceFromObject(surface)
        first, last = panel_frames[0], panel_frames[-1]

        def at(frame, run, width):
            return tuple(frame.origin[i] + run * frame.u[i] + width * frame.v[i] for i in range(3))

        # --- 中心線(端接線の補助点つき。SS13の学びを反映) ---
        centre_pts = []
        for idx, frame in enumerate(panel_frames):
            near_cut, far_cut = fold_tangents[idx]
            lo = frame.near_run_mm + near_cut + 1.0
            hi = frame.far_run_mm - far_cut - 1.0
            if hi - lo < 1.0:
                continue
            for k in range(5):
                centre_pts.append(at(frame, lo + (hi - lo) * k / 4.0, 0.0))
        pts = [centre_pts[0]]
        for q in centre_pts[1:]:
            if math.dist(pts[-1], q) > 0.01:
                pts.append(q)
        # 端接線の補助点(SS13): 隣接点間の内分点として挿入する。端の外側に置くと
        # パネル先頭点と逆行して自己交差スプラインになる(このスパイクで実測)。
        def lerp(a, b, t):
            return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))
        pts = [pts[0], lerp(pts[0], pts[1], 0.15)] + pts[1:-1]             + [lerp(pts[-2], pts[-1], 0.85), pts[-1]]
        refs = builder._point_refs(part, hsf, body, pts)
        spline = hsf.AddNewSpline()
        spline.SetSplineType(0)
        spline.SetClosing(0)
        for r in refs:
            spline.AddPointWithConstraintExplicit(r, None, -1.0, 1, None, 0.0)
        body.AppendHybridShape(spline)
        part.Update()
        centre = hsf.AddNewProject(part.CreateReferenceFromObject(spline), surface_ref)
        centre.Normal = True
        body.AppendHybridShape(centre)
        part.Update()
        centre_ref = part.CreateReferenceFromObject(centre)

        # --- (a) 側端曲線: 端の0.2mm内側へ測地オフセット。両側を作り、side=+1を選ぶ ---
        edge_offset = half_width_mm - 0.2
        side_probe = builder._point_refs(
            part, hsf, body,
            [at(panel_frames[len(panel_frames) // 2],
                (panel_frames[len(panel_frames) // 2].near_run_mm
                 + panel_frames[len(panel_frames) // 2].far_run_mm) / 2.0, edge_offset)])[0]
        root_ref = None
        for reverse in (False, True):
            cp = hsf.AddNewCurvePar(centre_ref, surface_ref, edge_offset, reverse, True)
            body.AppendHybridShape(cp)
            try:
                part.Update()
            except Exception:
                builder._delete_feature(doc, part, cp)
                continue
            ref = part.CreateReferenceFromObject(cp)
            if spa.GetMeasurable(ref).GetMinimumDistance(side_probe) < 0.5:
                root_ref = ref
                break
            builder._delete_feature(doc, part, cp)
        if root_ref is None:
            print(f"[{n:02d}] 側端曲線NG", flush=True)
            raise ValueError("spike stop")

        # --- (b) 90度スイープ(±90の両方を観測し、成立した方を採る) ---
        from synthetic_generator.bead import _cross, _normalize
        mid = panel_frames[len(panel_frames) // 2]
        mid_run = (mid.near_run_mm + mid.far_run_mm) / 2.0
        normal = _normalize(_cross(mid.u, mid.v))
        base_pt = at(mid, mid_run, edge_offset)
        top_candidates = builder._point_refs(part, hsf, body, [
            tuple(base_pt[i] + FLANGE_HEIGHT_MM * normal[i] for i in range(3)),
            tuple(base_pt[i] - FLANGE_HEIGHT_MM * normal[i] for i in range(3)),
        ])
        results = []
        chosen = None
        for angle in (90.0, -90.0):
            sweep = hsf.AddNewSweepLine(root_ref)
            sweep.Mode = 4
            sweep.FirstGuideSurf = surface_ref
            sweep.SetAngle(1, angle)
            sweep.SetLength(1, FLANGE_HEIGHT_MM)
            body.AppendHybridShape(sweep)
            try:
                part.Update()
            except Exception:
                results.append(f"{angle:+.0f}:X")
                builder._delete_feature(doc, part, sweep)
                continue
            ref = part.CreateReferenceFromObject(sweep)
            m = spa.GetMeasurable(ref)
            d = min(m.GetMinimumDistance(top_candidates[0]), m.GetMinimumDistance(top_candidates[1]))
            results.append(f"{angle:+.0f}:{d:.2f}mm")
            if chosen is None and d < 0.15:
                chosen = (sweep, ref, angle)
            else:
                builder._delete_feature(doc, part, sweep)
        if chosen is None:
            print(f"[{n:02d}] 90度スイープNG [{' '.join(results)}]", flush=True)
            raise ValueError("spike stop")
        wall, wall_ref, used_angle = chosen

        # --- (c) 根本R: 基準面 x 壁のBiTangent(keep=中心線側+壁の上端) ---
        keep_pts = []
        for idx, frame in enumerate(panel_frames):
            near_cut, far_cut = fold_tangents[idx]
            lo = frame.near_run_mm + near_cut + 1.0
            hi = frame.far_run_mm - far_cut - 1.0
            if hi - lo < 2.0:
                continue
            keep_pts.append(at(frame, (lo + hi) / 2.0, 0.0))
        keep_refs = builder._point_refs(part, hsf, body, keep_pts)
        # 壁上端のkeep(採用された向きの実測符号で)
        top_keep = builder._point_refs(part, hsf, body, [
            tuple(base_pt[i] + math.copysign(1.0, 0)  # placeholder, replaced below
                  for i in range(3))])
        # 上の行は使わない(符号決定が面倒なので、両候補のうち近い方を使う)
        m = spa.GetMeasurable(wall_ref)
        d0 = m.GetMinimumDistance(top_candidates[0])
        top_keep_ref = top_candidates[0] if d0 < 0.15 else top_candidates[1]
        try:
            fillet, fillet_ref = builder._bead_bitangent(
                doc, part, hsf, spa, body, wall_ref, surface_ref, FLANGE_ROOT_R_MM,
                keep_refs + [top_keep_ref], [], "flange root")
            ok = "OK"
        except ValueError as exc:
            ok = f"NG ({str(exc)[:60]})"
        print(f"[{n:02d}] 側端O 90度スイープ[{' '.join(results)}] 根本R: {ok}", flush=True)
        doc.SaveAs(str(OUT / f"flange_{n:03d}.CATPart"))
        raise ValueError("spike stop")

    builder._add_bead_to_surface = experiment

    rng = random.Random(515151)
    done = 0
    attempt = 0
    from synthetic_generator.bead import BeadParams
    dummy = BeadParams(depth_mm=5.0, top_width_mm=12.0, wall_angle_deg=50.0,
                       ridge_radius_mm=5.0, corner_radius_mm=9.0)
    while done < ATTEMPTS and attempt < 400:
        attempt += 1
        spec = sample_spec(rng)
        # 「曲げが少ない」基準面だけを対象にする(①の使い分け)
        try:
            plan = plan_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm, bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm, fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad)
        except ValueError:
            continue
        folds = []
        for i in range(len(plan.panel_frames) - 1):
            a, b = plan.panel_frames[i], plan.panel_frames[i + 1]
            folds.append(math.degrees(math.acos(max(-1.0, min(1.0,
                sum(a.u[k] * b.u[k] for k in range(3)))))))
        if max(folds) > MAX_FOLD_DEG:
            continue
        state["n"] = attempt
        done += 1
        print(f"[{attempt:02d}] 折れ角 {'/'.join(f'{f:.0f}' for f in folds)}度 "
              f"R={spec.bend_radius_mm:.1f}", flush=True)
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm, bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm, fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                out_dir=str(OUT), part_name=f"fl_{attempt:03d}", bead=dummy)
        except Exception as exc:
            if "spike stop" not in str(exc):
                print(f"    事前脱落: {str(exc)[:80]}", flush=True)


if __name__ == "__main__":
    main()

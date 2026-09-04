"""AddNewCurvePar(平行曲線)が失敗する条件を切り分ける(2026-08-24)。

SS8.9続き。中心線側は解決した:
  - 解析点群 -> スプライン: OK (Length 167.17mm)
  - スプライン -> AddNewProject(Normal=True)で基準面へ投影: OK (Length 167.66mm)
ところが`AddNewCurvePar`が、投影済み曲線でも交線でも失敗する。
旧w平行版では同じAPIが動いていたはずなので、何が条件なのかをここで特定する。

振る条件:
  - 入力曲線: 単一平面の交線(定義上厳密に面上) / 投影済みスプライン
  - オフセット量: 2 / 5 / 10 mm
  - iInvertDirection: False / True
  - 最終引数(Euclidean/測地): True / False
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.classify import (  # noqa: E402
    FasteningPoint,
    free_fold_seed,
    sheared_panel_corners,
    solve_free_fold,
    tangent_length_for_bend_angle_rad,
)
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"

POINT1 = FasteningPoint((0.0, 0.0, 0.0), (0.7091245539987692, -0.4192780250104878, 0.566875916457342))
POINT2 = FasteningPoint(
    (105.16387734930477, 9.362453294474978, -80.55987327589155),
    (0.8798637674773494, -0.4223685455820081, 0.21781772743168493),
)
BEND_RADIUS_MM = 11.081175192980025
HALF_WIDTH_MM = 14.51036472028607
MIN_BEARING_RADIUS_MM = 13.142192161995899


def main() -> None:
    builder = SyntheticPartBuilder()
    count = builder.catia.Documents.Count
    for i in range(count, 0, -1):
        builder.catia.Documents.Item(i).Close()
    print(f"closed {count} document(s)", flush=True)

    seed = free_fold_seed(
        POINT1, POINT2, bend_radius_mm=BEND_RADIUS_MM,
        min_bearing_radius_mm=MIN_BEARING_RADIUS_MM,
        fold1_slack_mm=46.17202428508006, fold2_slack_mm=1.5474142248551326,
    )
    chain = solve_free_fold(
        seed, POINT1, POINT2, target_a1_rad=seed.a1_rad + math.radians(0.4155136573387424)
    )

    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "CURVEPAR_ISOLATE"

    margin = MIN_BEARING_RADIUS_MM
    corner_sets = [
        sheared_panel_corners(chain.panel1.origin, chain.panel1.u, chain.panel1.v,
                              -margin, chain.L1_mm, HALF_WIDTH_MM,
                              near_tilt_rad=0.0, far_tilt_rad=chain.a1_rad),
        sheared_panel_corners(chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v,
                              0.0, chain.L2_mm, HALF_WIDTH_MM,
                              near_tilt_rad=chain.a1_rad, far_tilt_rad=chain.a2_rad),
        sheared_panel_corners(chain.panel3.origin, chain.panel3.u, chain.panel3.v,
                              0.0, chain.L3_mm + margin, HALF_WIDTH_MM,
                              near_tilt_rad=chain.a2_rad, far_tilt_rad=0.0),
    ]
    faces = [builder.rect_fill(hsf, part, body, c) for c in corner_sets]
    whole = builder.join(hsf, part, body, faces)
    part.Update()
    mid = corner_sets[1]
    for fold_mid in (
        tuple((mid[0][i] + mid[1][i]) / 2 for i in range(3)),
        tuple((mid[2][i] + mid[3][i]) / 2 for i in range(3)),
    ):
        edge_ref = builder.find_edge_near(doc, part, spa, hsf, body, whole, fold_mid)
        whole = builder.edge_fillet_group(part, [edge_ref], BEND_RADIUS_MM)
        body.AppendHybridShape(whole)
        part.Update()
    surface_ref = part.CreateReferenceFromObject(whole)
    print("base surface + fillets OK", flush=True)

    candidates = {}

    # (a) 単一平面の交線(panel_midのv法線平面)。定義上、厳密に面上にある。
    origin, u, v = chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v
    o_pt = builder.point(hsf, body, *origin)
    v_pt = builder.point(hsf, body, *tuple(origin[i] + v[i] for i in range(3)))
    part.Update()
    v_axis = builder.line_pt_pt(hsf, part, body, o_pt, v_pt)
    part.Update()
    plane = hsf.AddNewPlaneNormal(
        part.CreateReferenceFromObject(v_axis), part.CreateReferenceFromObject(o_pt)
    )
    body.AppendHybridShape(plane)
    part.Update()
    inter = hsf.AddNewIntersection(part.CreateReferenceFromObject(plane), surface_ref)
    inter.Name = "single_plane_intersection"
    body.AppendHybridShape(inter)
    part.Update()
    candidates["intersection"] = inter
    print("single-plane intersection OK", flush=True)

    # (b) 解析点群 -> スプライン -> 投影
    tan1 = tangent_length_for_bend_angle_rad(chain.fold1_angle_rad, BEND_RADIUS_MM)
    tan2 = tangent_length_for_bend_angle_rad(chain.fold2_angle_rad, BEND_RADIUS_MM)
    cut1 = tan1 / math.cos(chain.a1_rad) + 1.0
    cut2 = tan2 / math.cos(chain.a2_rad) + 1.0
    spans = [
        (chain.panel1.origin, chain.panel1.u, -margin, chain.L1_mm - cut1),
        (chain.panel_mid.origin, chain.panel_mid.u, cut1, chain.L2_mm - cut2),
        (chain.panel3.origin, chain.panel3.u, cut2, chain.L3_mm + margin),
    ]
    pts = []
    for org, uu, lo, hi in spans:
        for k in range(5):
            run = lo + (hi - lo) * k / 4
            pts.append(tuple(org[i] + run * uu[i] for i in range(3)))
    objs = [builder.point(hsf, body, *p) for p in pts]
    part.Update()
    spline = hsf.AddNewSpline()
    spline.SetSplineType(0)
    spline.SetClosing(0)
    for pt in objs:
        spline.AddPointWithConstraintExplicit(
            part.CreateReferenceFromObject(pt), None, -1.0, 1, None, 0.0
        )
    spline.Name = "analytic_spline"
    body.AppendHybridShape(spline)
    part.Update()
    proj = hsf.AddNewProject(part.CreateReferenceFromObject(spline), surface_ref)
    proj.Normal = True
    proj.Name = "analytic_projected"
    body.AppendHybridShape(proj)
    part.Update()
    candidates["projected"] = proj
    print("analytic spline + projection OK", flush=True)

    # --- CurveParの条件を総当たり ---
    print("\n=== AddNewCurvePar sweep ===", flush=True)
    for name, curve in candidates.items():
        curve_ref = part.CreateReferenceFromObject(curve)
        for offset in (2.0, 5.0, 10.0):
            for invert in (False, True):
                for euclid in (True, False):
                    try:
                        cp = hsf.AddNewCurvePar(curve_ref, surface_ref, offset, invert, euclid)
                        body.AppendHybridShape(cp)
                        part.Update()
                        try:
                            length = spa.GetMeasurable(part.CreateReferenceFromObject(cp)).Length
                            extra = f" Length={length:.1f}mm"
                        except Exception:
                            extra = " (Length unmeasurable)"
                        print(f"  {name} off={offset:4.1f} invert={invert!s:5} euclid={euclid!s:5}: OK{extra}", flush=True)
                        builder._delete_feature(doc, part, cp)
                    except Exception as exc:
                        msg = str(exc)[:45]
                        print(f"  {name} off={offset:4.1f} invert={invert!s:5} euclid={euclid!s:5}: FAILED {msg}", flush=True)
                        try:
                            builder._delete_feature(doc, part, cp)
                        except Exception:
                            pass

    OUTPUT_DIR.mkdir(exist_ok=True)
    doc.SaveAs(str(OUTPUT_DIR / "curvepar_isolate.CATPart"))
    print("saved", flush=True)


if __name__ == "__main__":
    main()

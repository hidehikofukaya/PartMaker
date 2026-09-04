"""ビード中心線を「解析的に求めた点群 -> スプライン -> 基準面へ投影」で作る案の実機検証。

SS8.9で判明したこと(2026-08-24):
  - パネルごとのv法線平面と基準面のIntersectionは、各々**単一枝**で正常(Length測定可)。
  - しかし各平面は無限に広がるため、1本の交線が**部品全長(約168mm)を貫いてしまう**。
    つまりほぼ重なった3本ができ、そのJoinが失敗していた(枝の問題ではなく重複の問題)。
  - 4点(p1/fold1/fold2/p2)への最小二乗平面で代用する案は、ずれが横方向余裕内に
    収まるのが800サンプル中55%に留まり不十分。

本案: 中心線が乗るべき位置は解析的に厳密に分かっている(各パネルのv=0のu軸)。
曲げフィレット領域を避けて平坦区間だけから点群を取り、スプラインで結び、
`AddNewProject`で基準面へ落とす。フィレット領域はスプラインが滑らかに補間したものを
投影が実際の曲面へ吸着させる。これなら交線を一切使わずJoinも不要で、
`AddNewCurvePar`に渡せる単一の曲線が得られる(はず)。

平坦区間の境界: 折れ目の接線長T=R*tan(phi/2)は**折れ目に垂直に**測るが、中心線は
折れ目を角度a(傾き)で斜めに横切るため、u方向の走行距離では T/cos(a) 消費する。
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
FOLD1_SLACK_MM = 46.17202428508006
FOLD2_SLACK_MM = 1.5474142248551326
FOLD1_TILT_PERTURBATION_DEG = 0.4155136573387424

FLAT_CLEARANCE_MM = 1.0   # フィレット接点からさらに離す余裕
POINTS_PER_PANEL = 5


def main() -> None:
    builder = SyntheticPartBuilder()
    count = builder.catia.Documents.Count
    for i in range(count, 0, -1):
        builder.catia.Documents.Item(i).Close()
    print(f"closed {count} pre-existing document(s)", flush=True)

    seed = free_fold_seed(
        POINT1, POINT2, bend_radius_mm=BEND_RADIUS_MM,
        min_bearing_radius_mm=MIN_BEARING_RADIUS_MM,
        fold1_slack_mm=FOLD1_SLACK_MM, fold2_slack_mm=FOLD2_SLACK_MM,
    )
    chain = solve_free_fold(
        seed, POINT1, POINT2,
        target_a1_rad=seed.a1_rad + math.radians(FOLD1_TILT_PERTURBATION_DEG),
    )
    print(
        f"chain: fold1={math.degrees(chain.fold1_angle_rad):.1f}deg "
        f"fold2={math.degrees(chain.fold2_angle_rad):.1f}deg "
        f"a1={math.degrees(chain.a1_rad):.1f}deg a2={math.degrees(chain.a2_rad):.1f}deg",
        flush=True,
    )

    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "SPLINE_PROBE"

    margin = MIN_BEARING_RADIUS_MM
    corner_sets = [
        sheared_panel_corners(
            chain.panel1.origin, chain.panel1.u, chain.panel1.v,
            -margin, chain.L1_mm, HALF_WIDTH_MM, near_tilt_rad=0.0, far_tilt_rad=chain.a1_rad,
        ),
        sheared_panel_corners(
            chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v,
            0.0, chain.L2_mm, HALF_WIDTH_MM, near_tilt_rad=chain.a1_rad, far_tilt_rad=chain.a2_rad,
        ),
        sheared_panel_corners(
            chain.panel3.origin, chain.panel3.u, chain.panel3.v,
            0.0, chain.L3_mm + margin, HALF_WIDTH_MM, near_tilt_rad=chain.a2_rad, far_tilt_rad=0.0,
        ),
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
    print("base surface + fillets OK", flush=True)
    surface_ref = part.CreateReferenceFromObject(whole)

    # --- 各パネルの平坦区間だけから中心線の点をサンプリングする ---
    tan1 = tangent_length_for_bend_angle_rad(chain.fold1_angle_rad, BEND_RADIUS_MM)
    tan2 = tangent_length_for_bend_angle_rad(chain.fold2_angle_rad, BEND_RADIUS_MM)
    # 折れ目を角度aで斜めに横切るので、u方向には T/cos(a) 消費する
    cut1 = tan1 / math.cos(chain.a1_rad) + FLAT_CLEARANCE_MM
    cut2 = tan2 / math.cos(chain.a2_rad) + FLAT_CLEARANCE_MM
    print(f"fillet consumes along u: fold1={cut1:.2f}mm fold2={cut2:.2f}mm", flush=True)

    spans = [
        (chain.panel1.origin, chain.panel1.u, -margin, chain.L1_mm - cut1),
        (chain.panel_mid.origin, chain.panel_mid.u, cut1, chain.L2_mm - cut2),
        (chain.panel3.origin, chain.panel3.u, cut2, chain.L3_mm + margin),
    ]
    xyz_points = []
    for origin, u, lo, hi in spans:
        if hi - lo < 2.0:
            print(f"  WARNING: flat span too short ({hi - lo:.2f}mm)", flush=True)
            continue
        for k in range(POINTS_PER_PANEL):
            run = lo + (hi - lo) * k / (POINTS_PER_PANEL - 1)
            xyz_points.append(tuple(origin[i] + run * u[i] for i in range(3)))
    print(f"sampled {len(xyz_points)} centreline points on flat regions", flush=True)

    point_objs = [builder.point(hsf, body, *p) for p in xyz_points]
    part.Update()

    # --- スプライン ---
    try:
        spline = hsf.AddNewSpline()
        spline.SetSplineType(0)
        spline.SetClosing(0)
        for pt in point_objs:
            spline.AddPointWithConstraintExplicit(
                part.CreateReferenceFromObject(pt), None, -1.0, 1, None, 0.0
            )
        spline.Name = "centreline_spline"
        body.AppendHybridShape(spline)
        part.Update()
        print("spline OK", flush=True)
        try:
            length = spa.GetMeasurable(part.CreateReferenceFromObject(spline)).Length
            print(f"  spline Length = {length:.2f}mm", flush=True)
        except Exception as exc:
            print(f"  spline Length failed: {str(exc)[:60]}", flush=True)
    except Exception as exc:
        print(f"spline FAILED: {str(exc)[:120]}", flush=True)
        doc.SaveAs(str(OUTPUT_DIR / "centreline_spline_probe.CATPart"))
        return

    # --- 基準面へ投影 ---
    projected = None
    try:
        proj = hsf.AddNewProject(part.CreateReferenceFromObject(spline), surface_ref)
        proj.Normal = True   # 面法線方向へ投影(HybridShapeProjectのプロパティ。SetNormalModeは存在しない)
        proj.Name = "centreline_projected"
        body.AppendHybridShape(proj)
        part.Update()
        print("AddNewProject OK", flush=True)
        try:
            length = spa.GetMeasurable(part.CreateReferenceFromObject(proj)).Length
            print(f"  projected Length = {length:.2f}mm", flush=True)
        except Exception as exc:
            print(f"  projected Length failed: {str(exc)[:60]}", flush=True)
        projected = proj
    except Exception as exc:
        print(f"AddNewProject FAILED: {str(exc)[:120]}", flush=True)

    # --- 本命: AddNewCurvePar が通るか(壁の根元曲線) ---
    for label, curve in (("spline(未投影)", spline), ("projected", projected)):
        if curve is None:
            continue
        curve_ref = part.CreateReferenceFromObject(curve)
        for reverse in (False, True):
            try:
                root = hsf.AddNewCurvePar(curve_ref, surface_ref, 10.0, reverse, True)
                root.Name = f"root_{label}_{'R' if reverse else 'L'}"
                body.AppendHybridShape(root)
                part.Update()
                print(f"  AddNewCurvePar[{label}] reverse={reverse}: OK", flush=True)
            except Exception as exc:
                print(f"  AddNewCurvePar[{label}] reverse={reverse}: FAILED {str(exc)[:70]}", flush=True)

    OUTPUT_DIR.mkdir(exist_ok=True)
    doc.SaveAs(str(OUTPUT_DIR / "centreline_spline_probe.CATPart"))
    viewer = builder.catia.ActiveWindow.ActiveViewer
    viewer.Update()
    viewer.Reframe()
    viewer.CaptureToFile(2, str(OUTPUT_DIR / "centreline_spline_probe.png"))
    print("saved + screenshot", flush=True)


if __name__ == "__main__":
    main()

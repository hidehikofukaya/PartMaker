"""ビード手順⑤(4枚の壁を4隅R付きの連続バンドにする)の閉じ方を実機で比較する(2026-08-24)。

現状(SS8.9): 対角方式
    f1 = BiTangent(left, start)   … OK
    f2 = BiTangent(right, end)    … OK
    band = BiTangent(f1, f2)      … 失敗
失敗の内訳は「keep 3/6外れ 最悪21.24mm」= ちょうど片側の壁3個分。つまり最終段が
2つのL字リボンを統合せず、片側を削り落としている。

ビードのフットプリントは4壁の閉ループなので、pairwise BiTangentをどう並べても
**最終段だけは必ず2隅を同時に作る**ことになる(4面・4隅、1回目1隅・2回目1隅・3回目2隅)。
その2隅同時生成がこの形状で成立するのかを、以下の戦略で切り分ける。

  A: 対角(現行)              f1=(L,S) f2=(R,E) -> (f1,f2)
  B: 逐次                    ((L,S),R) -> (…,E)
  C: 対角 + 最終段だけ小R
  D: 対角 + 最終段のtrimMode/removeFaceを変える

各戦略で、6個の壁根元プローブ全てが結果面上に乗るかどうかで合否を判定する。
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import BeadPanelFrame, BeadParams, plan_bead_on_surface  # noqa: E402
from synthetic_generator.classify import (  # noqa: E402
    FasteningPoint,
    free_fold_seed,
    sheared_panel_corners,
    solve_free_fold,
    tangent_length_for_bend_angle_rad,
)
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"

POINT1 = FasteningPoint((0.0, 0.0, 0.0), (0.7800312387928039, 0.21211178193677718, 0.5886933484174666))
POINT2 = FasteningPoint(
    (24.30359962098499, -68.62918910755124, 144.9674165750089),
    (0.19870462012118822, -0.3579774571251853, -0.9123423776919934),
)
BEND_RADIUS_MM = 11.56030141400419
HALF_WIDTH_MM = 25.0
MIN_BEARING_RADIUS_MM = 23.473464455183723
FOLD1_SLACK_MM = 49.315062550877016
FOLD2_SLACK_MM = 43.108138747799245
FOLD1_TILT_PERTURBATION_RAD = -0.021562977200378728
BEAD = BeadParams(
    depth_mm=9.109614587119138,
    top_width_mm=13.733648251153701,
    wall_angle_deg=58.42740771706355,
    ridge_radius_mm=8.5101904054768,
    corner_radius_mm=9.05354681994448,
)


def main() -> None:
    builder = SyntheticPartBuilder()
    count = builder.catia.Documents.Count
    for i in range(count, 0, -1):
        builder.catia.Documents.Item(i).Close()
    print(f"closed {count} document(s)", flush=True)

    seed = free_fold_seed(
        POINT1, POINT2, bend_radius_mm=BEND_RADIUS_MM,
        min_bearing_radius_mm=MIN_BEARING_RADIUS_MM,
        fold1_slack_mm=FOLD1_SLACK_MM, fold2_slack_mm=FOLD2_SLACK_MM,
    )
    chain = solve_free_fold(seed, POINT1, POINT2, target_a1_rad=seed.a1_rad + FOLD1_TILT_PERTURBATION_RAD)

    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "BAND_STRATEGIES"

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

    tangent1 = tangent_length_for_bend_angle_rad(chain.fold1_angle_rad, BEND_RADIUS_MM)
    tangent2 = tangent_length_for_bend_angle_rad(chain.fold2_angle_rad, BEND_RADIUS_MM)
    run_cut1 = tangent1 / math.cos(chain.a1_rad)
    run_cut2 = tangent2 / math.cos(chain.a2_rad)
    panel_frames = [
        BeadPanelFrame(chain.panel1.origin, chain.panel1.u, chain.panel1.v, -margin, chain.L1_mm),
        BeadPanelFrame(chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v, 0.0, chain.L2_mm),
        BeadPanelFrame(chain.panel3.origin, chain.panel3.u, chain.panel3.v, 0.0, chain.L3_mm + margin),
    ]
    plan = plan_bead_on_surface(
        panel_frames, BEAD, inset_mm=2.0 * margin,
        guide_margin_mm=builder.BEAD_GUIDE_MARGIN_MM, half_width_mm=HALF_WIDTH_MM,
        fold_tangents=[(0.0, run_cut1), (run_cut1, run_cut2), (run_cut2, 0.0)],
    )

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    def normalize(v):
        n = math.sqrt(sum(c * c for c in v))
        return tuple(c / n for c in v)

    normals = [normalize(cross(f.u, f.v)) for f in panel_frames]
    wall_samples = [(b, normals[i]) for b, i in zip(plan.wall_top_bases, plan.wall_top_panel_index)]
    top_samples = [(b, normals[i]) for b, i in zip(plan.top_keep_bases, plan.top_panel_index)]
    remove_samples = [(b, normals[i]) for b, i in zip(plan.top_remove_bases, plan.top_remove_panel_index)]
    _top, top_ref, signs = builder._bead_top_offset(
        doc, part, hsf, spa, body, surface_ref, BEAD, wall_samples + top_samples + remove_samples
    )
    wall_tops = [
        tuple(base[i] + signs[k] * BEAD.depth_mm * normal[i] for i in range(3))
        for k, (base, normal) in enumerate(wall_samples)
    ]
    print("top offset OK", flush=True)

    # 中心線 -> 根元曲線 -> 幅ガイド -> 壁4枚
    point_objs = builder._point_refs(part, hsf, body, plan.centreline_points)
    spline = hsf.AddNewSpline()
    spline.SetSplineType(0)
    spline.SetClosing(0)
    for pr in point_objs:
        spline.AddPointWithConstraintExplicit(pr, None, -1.0, 1, None, 0.0)
    body.AppendHybridShape(spline)
    part.Update()
    centreline = hsf.AddNewProject(part.CreateReferenceFromObject(spline), surface_ref)
    centreline.Normal = True
    body.AppendHybridShape(centreline)
    part.Update()
    centreline_ref = part.CreateReferenceFromObject(centreline)

    roots = []
    for reverse in (False, True):
        r = hsf.AddNewCurvePar(centreline_ref, surface_ref, BEAD.half_footprint_mm, reverse, True)
        body.AppendHybridShape(r)
        part.Update()
        roots.append(part.CreateReferenceFromObject(r))

    def guide_ref(ends):
        a, b = builder._point_refs(part, hsf, body, list(ends))
        line = hsf.AddNewLinePtPt(a, b)
        body.AppendHybridShape(line)
        part.Update()
        return part.CreateReferenceFromObject(line)

    start_ref = guide_ref(plan.start_guide)
    end_ref = guide_ref(plan.end_guide)
    wall_top_refs = builder._point_refs(part, hsf, body, wall_tops)
    walls = [
        builder._bead_wall(doc, part, hsf, spa, body, curve, surface_ref, top, BEAD, label)[1]
        for curve, top, label in zip(
            (roots[0], start_ref, roots[1], end_ref), wall_top_refs,
            ("left", "start", "right", "end"),
        )
    ]
    print("4 walls OK", flush=True)

    root_refs = builder._point_refs(part, hsf, body, plan.wall_root_probes)
    left_refs = builder._point_refs(part, hsf, body, plan.wall_root_probes_left)
    right_refs = builder._point_refs(part, hsf, body, plan.wall_root_probes_right)

    def try_bitangent(r1, r2, radius, keep, label, trim=(1, 1), verbose=False):
        """成功したら(feature, ref)、失敗ならNone。全プローブが乗ることを要求する。

        verbose=Trueなら全プローブの実測距離を出す(判定閾値の問題なのか、本当に形状が
        違うのかを切り分けるため)。
        """
        best = None
        for o1 in (1, -1):
            for o2 in (1, -1):
                f = hsf.AddNewFilletBiTangent(r1, r2, radius, o1, o2, trim[0], trim[1])
                body.AppendHybridShape(f)
                try:
                    part.Update()
                except Exception:
                    if verbose:
                        print(f"    {label} o=({o1:+d},{o2:+d}) trim={trim}: update失敗", flush=True)
                    builder._delete_feature(doc, part, f)
                    continue
                ref = part.CreateReferenceFromObject(f)
                m = spa.GetMeasurable(ref)
                ds = [m.GetMinimumDistance(p) for p in keep]
                bad = sum(1 for d in ds if d >= builder.BEAD_PROBE_TOLERANCE_MM)
                if verbose:
                    shown = " ".join(f"{d:7.3f}" for d in ds)
                    print(f"    {label} o=({o1:+d},{o2:+d}) trim={trim}: 距離[{shown}] 外れ{bad}/{len(ds)}", flush=True)
                if bad == 0:
                    print(f"    {label} o=({o1:+d},{o2:+d}) trim={trim}: OK", flush=True)
                    return f, ref
                if best is None or bad < best:
                    best = bad
                builder._delete_feature(doc, part, f)
        print(f"    {label} trim={trim}: 全滅 (最良でも {best}/{len(keep)} 外れ)", flush=True)
        return None

    print("\n=== 戦略A: 対角(現行) ===", flush=True)
    a1 = try_bitangent(walls[0], walls[1], BEAD.corner_radius_mm, left_refs, "f1=(L,S)")
    a2 = try_bitangent(walls[2], walls[3], BEAD.corner_radius_mm, right_refs, "f2=(R,E)")
    if a1 and a2:
        # relimitation modeは0〜3で挙動が変わらないことを確認済み(効果なし)。
        # 設計文書(bead_construction_flowchart.md SS6.3)では2隅同時のリボン生成が
        # 実証されているので原理的には可能。実証時との差はコーナーRの比率:
        #   実証時: corner_R=6mm, フットプリント全幅41.7mm (比 0.29)
        #   今回:   corner_R=9.05mm, 全幅24.9mm            (比 0.73)
        # コーナーRがフットプリントに対して過大なのが原因と見て、半径を振って安全域を探す。
        full_width = 2.0 * BEAD.half_footprint_mm
        print(f"  --- corner radius sweep (フットプリント全幅={full_width:.1f}mm) ---", flush=True)
        print("  プローブ順: 左壁3点(panel1,mid,panel3) 右壁3点(panel1,mid,panel3)", flush=True)
        for radius in (9.05, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0):
            ratio = radius / full_width
            ok = try_bitangent(a1[1], a2[1], radius, root_refs,
                               f"band R={radius:.1f} (比{ratio:.2f})", verbose=True)
            if ok:
                print(f"  => R={radius:.1f}mm (フットプリント全幅比 {ratio:.2f}) で成功", flush=True)
                break

    OUTPUT_DIR.mkdir(exist_ok=True)
    doc.SaveAs(str(OUTPUT_DIR / "bead_band_strategies.CATPart"))
    print("\nsaved", flush=True)


if __name__ == "__main__":
    main()

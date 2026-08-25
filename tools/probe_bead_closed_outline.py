"""ビードの壁を「丸めコーナー入り閉曲線の1回スイープ」で作る案の実機検証(2026-08-24)。

背景(SS8.9続き): 4枚の壁をBiTangent3回で連続バンドにする現行方式は、最終段
(2つのL字リボンの統合)が成立しない。向き4通り・trimMode・relimitationMode(0〜3)・
コーナーR(9.1〜3.0mm、フットプリント全幅比0.36〜0.12)のいずれを振っても、
必ず片側のリボンが丸ごとトリムで消える(左3点が面上なら右3点が21mm外れ、逆も同様)。
設計文書SS6.3では2隅同時のリボン生成が実証済みと記録されているが、今の
(自由折れ目チェーン+投影スプライン中心線)の形状では再現しない。

本案はユーザーの元仕様(SS1.1 手順③「全ての壁をトリムで1つの連続した壁に仕上げる」)に
素直に従う: **丸めコーナーを最初から織り込んだ閉じた根元曲線**を作り、1回スイープする。
コーナーRは曲線に含まれているので、後からフィレットで隅を作る必要が無い(=BiTangentで
ループを閉じる必要が無い)。

中心線で確立した「解析点群 -> スプライン -> AddNewProject」がそのまま使える。
SS3.2に「丸めコーナー入り閉曲線のスイープが失敗する(未解決)」という記録があるが、
それは旧い曲線構築での結果なので、今の道具立てで改めて確かめる価値がある。
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
DEDUPE_TOLERANCE_MM = 0.01

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

FLAT_CLEARANCE_MM = 1.0
ARC_POINTS = 5          # コーナー円弧1本あたりの点数
EDGE_POINTS_PER_PANEL = 4


def build_outline(panel_frames, tilts, bead, start_run, end_run, fold_cuts):
    """ビードのフットプリント外形(丸めコーナー入り閉曲線)の3D標本点を、走行順に返す。

    (run, width) のローカル座標で丸め長方形を描き、各パネルのフレームで3Dへ写す。
    長辺(width=±half_footprint)は3パネルをまたぐので、曲げフィレット領域を避けて
    平坦区間だけから採る(フィレット上はスプライン補間+投影に任せる。中心線と同じ考え方)。
    端のキャップとコーナー円弧は端パネルの平坦部に収まるので素直に置ける。
    """
    hf = bead.half_footprint_mm
    cr = bead.corner_radius_mm
    first, last = panel_frames[0], panel_frames[-1]

    def at(frame, run, width):
        return tuple(frame.origin[i] + run * frame.u[i] + width * frame.v[i] for i in range(3))

    def long_edge_points(width, reverse):
        """width一定の長辺を、全パネルの平坦区間から採る。"""
        pts = []
        for index, frame in enumerate(panel_frames):
            near_tilt, far_tilt = tilts[index]
            near_cut, far_cut = fold_cuts[index]
            # 折れ目は傾いているので、この幅座標での境界runはシアーぶんずれる
            lo = frame.near_run_mm + width * math.tan(near_tilt) + near_cut + FLAT_CLEARANCE_MM
            hi = frame.far_run_mm + width * math.tan(far_tilt) - far_cut - FLAT_CLEARANCE_MM
            if index == 0:
                lo = max(lo, start_run + cr)
            if index == len(panel_frames) - 1:
                hi = min(hi, end_run - cr)
            if hi - lo < 1.0:
                continue
            for k in range(EDGE_POINTS_PER_PANEL):
                run = lo + (hi - lo) * k / (EDGE_POINTS_PER_PANEL - 1)
                pts.append(at(frame, run, width))
        return list(reversed(pts)) if reverse else pts

    def arc_points(frame, centre_run, centre_width, theta0, theta1):
        pts = []
        for k in range(ARC_POINTS):
            theta = math.radians(theta0 + (theta1 - theta0) * k / (ARC_POINTS - 1))
            run = centre_run + cr * math.cos(theta)
            width = centre_width + cr * math.sin(theta)
            pts.append(at(frame, run, width))
        return pts

    outline = []
    outline += long_edge_points(-hf, reverse=False)                       # 長辺(手前側)
    outline += arc_points(last, end_run - cr, -hf + cr, -90.0, 0.0)       # 終端コーナー1
    outline += [at(last, end_run, w) for w in (-hf + cr, 0.0, hf - cr)]   # 終端キャップ
    outline += arc_points(last, end_run - cr, hf - cr, 0.0, 90.0)         # 終端コーナー2
    outline += long_edge_points(hf, reverse=True)                         # 長辺(奥側、逆走)
    outline += arc_points(first, start_run + cr, hf - cr, 90.0, 180.0)    # 始端コーナー1
    outline += [at(first, start_run, w) for w in (hf - cr, 0.0, -hf + cr)]  # 始端キャップ
    outline += arc_points(first, start_run + cr, -hf + cr, 180.0, 270.0)  # 始端コーナー2

    # 円弧の終点とキャップ/長辺の始点は同じ位置なので、そのまま繋ぐと連続重複点になる。
    # 閉曲線なので末尾が先頭と一致するのも同様。重複点が残るとAddNewSplineが退化して
    # Updateに失敗するため、ここで間引く(2026-08-25)。
    deduped = []
    for pt in outline:
        if deduped and math.dist(deduped[-1], pt) < DEDUPE_TOLERANCE_MM:
            continue
        deduped.append(pt)
    while len(deduped) > 2 and math.dist(deduped[0], deduped[-1]) < DEDUPE_TOLERANCE_MM:
        deduped.pop()
    return deduped


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
    body.Name = "CLOSED_OUTLINE"

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
    cut1 = tangent1 / math.cos(chain.a1_rad)
    cut2 = tangent2 / math.cos(chain.a2_rad)
    panel_frames = [
        BeadPanelFrame(chain.panel1.origin, chain.panel1.u, chain.panel1.v, -margin, chain.L1_mm),
        BeadPanelFrame(chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v, 0.0, chain.L2_mm),
        BeadPanelFrame(chain.panel3.origin, chain.panel3.u, chain.panel3.v, 0.0, chain.L3_mm + margin),
    ]
    tilts = [(0.0, chain.a1_rad), (chain.a1_rad, chain.a2_rad), (chain.a2_rad, 0.0)]
    fold_cuts = [(0.0, cut1), (cut1, cut2), (cut2, 0.0)]

    inset = 2.0 * margin
    start_run = panel_frames[0].near_run_mm + inset
    end_run = panel_frames[-1].far_run_mm - inset
    outline = build_outline(panel_frames, tilts, BEAD, start_run, end_run, fold_cuts)
    print(f"outline: {len(outline)} points", flush=True)

    # 閉じたスプライン -> 投影
    refs = builder._point_refs(part, hsf, body, outline)
    spline = hsf.AddNewSpline()
    spline.SetSplineType(0)
    spline.SetClosing(1)   # 閉曲線
    for r in refs:
        spline.AddPointWithConstraintExplicit(r, None, -1.0, 1, None, 0.0)
    spline.Name = "bead_outline_spline"
    body.AppendHybridShape(spline)
    try:
        part.Update()
        print("closed spline OK", flush=True)
    except Exception as exc:
        print(f"closed spline FAILED: {str(exc)[:90]}", flush=True)
        doc.SaveAs(str(OUTPUT_DIR / "bead_closed_outline.CATPart"))
        return
    try:
        length = spa.GetMeasurable(part.CreateReferenceFromObject(spline)).Length
        print(f"  spline Length = {length:.2f}mm", flush=True)
    except Exception as exc:
        print(f"  spline Length failed: {str(exc)[:50]}", flush=True)

    projected = hsf.AddNewProject(part.CreateReferenceFromObject(spline), surface_ref)
    projected.Normal = True
    projected.Name = "bead_outline_projected"
    body.AppendHybridShape(projected)
    try:
        part.Update()
        print("projection OK", flush=True)
        try:
            length = spa.GetMeasurable(part.CreateReferenceFromObject(projected)).Length
            print(f"  projected Length = {length:.2f}mm", flush=True)
        except Exception as exc:
            print(f"  projected Length failed: {str(exc)[:50]}", flush=True)
    except Exception as exc:
        print(f"projection FAILED: {str(exc)[:90]}", flush=True)
        doc.SaveAs(str(OUTPUT_DIR / "bead_closed_outline.CATPart"))
        return

    # 1回のMode=4スイープで壁バンドを作る(これが本命)
    outline_ref = part.CreateReferenceFromObject(projected)
    theta = BEAD.wall_angle_deg
    print("\n=== 閉曲線の単発スイープ ===", flush=True)
    for angle_deg in (-theta, -(180.0 - theta), 180.0 - theta, theta):
        sweep = hsf.AddNewSweepLine(outline_ref)
        sweep.Mode = 4
        sweep.FirstGuideSurf = surface_ref
        sweep.SetAngle(1, angle_deg)
        sweep.SetLength(1, BEAD.wall_slant_mm * 1.6)
        sweep.SetLength(2, 3.0)
        sweep.Name = f"band_sweep_{angle_deg:.0f}"
        body.AppendHybridShape(sweep)
        try:
            part.Update()
        except Exception as exc:
            print(f"  angle={angle_deg:+.0f}deg: FAILED ({str(exc)[:45]})", flush=True)
            builder._delete_feature(doc, part, sweep)
            continue
        try:
            # CATIAの面積の既定単位は m^2。mm^2 に直すには 1e6 を掛ける(既知の落とし穴)。
            area = spa.GetMeasurable(part.CreateReferenceFromObject(sweep)).Area * 1e6
            expected = length * (BEAD.wall_slant_mm * 1.6 + 3.0)
            print(f"  angle={angle_deg:+.0f}deg: OK  面積={area:.1f}mm^2 "
                  f"(理論値≒{expected:.1f}mm^2, 比={area/expected:.3f})", flush=True)
        except Exception:
            print(f"  angle={angle_deg:+.0f}deg: OK (面積測定不可)", flush=True)

    OUTPUT_DIR.mkdir(exist_ok=True)
    doc.SaveAs(str(OUTPUT_DIR / "bead_closed_outline.CATPart"))
    viewer = builder.catia.ActiveWindow.ActiveViewer
    viewer.Update()
    viewer.Reframe()
    viewer.CaptureToFile(2, str(OUTPUT_DIR / "bead_closed_outline.png"))
    print("saved + screenshot", flush=True)


if __name__ == "__main__":
    main()

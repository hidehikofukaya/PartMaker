"""新方式ビード構築の一気通貫スパイク(roadmap SS6.25の後継、フィレット済み基準面への後付け)。

方針(ユーザー承認済み、2026-08-22):
  1. 主形状は既存の実績あるパイプライン(シャープjoin -> edge_fillet)でそのまま作る(変更しない)。
  2. 完成済みの丸め済み主面に対し、ビードを"後付け"する:
     - 幅方向中心を通る平面と主面の交線(centerline) -- ヒント①(AddNewIntersection)
     - centerlineをGeodesic平行曲線で±方向にオフセットし、outer_left/root_left/
       root_right/outer_rightの4本を作る -- ヒント②(AddNewCurvePar)
     - root_left/root_rightをそれぞれガイドに、主面を参照サーフェスにしたSweepLine
       (Mode=4、実機検証済み)で壁を立ち上げる -- ヒント③
     - 壁の遠端エッジ(top_left/top_right)を取り、天面パッチをFillで張る
     - outer<->root間の平地パッチもFillで作る(主面を直接使わず、曲線境界から再構築する
       ことで、フィレット済み主面の実際の曲率に厳密に一致させる -- これが「浮き」対策の核心)
     - 全ピースをシャープにjoinし、ビード自身の折れ目(根元4本+天面2本の角)にedge_filletを当てる

この段階では端の逃げ(端で平地に戻すrun-out)は含めない(旧Step1-2相当のスコープ、
まず貫通ビードの基本パイプラインを通すことを優先)。
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"


def build_filleted_base(builder, hsf, part, body, spa, doc, *, half_width=60.0, run=80.0, bend_deg=70.0, radius=30.0):
    half_angle = math.radians(bend_deg) / 2.0
    u1 = (math.cos(half_angle), 0.0, -math.sin(half_angle))
    u2 = (math.cos(half_angle), 0.0, math.sin(half_angle))
    w = (0.0, 1.0, 0.0)

    def corners(u, run0, run1, hw):
        def pt(r, ww):
            return (r * u[0] + ww * w[0], r * u[1] + ww * w[1], r * u[2] + ww * w[2])
        return [pt(run0, -hw), pt(run0, hw), pt(run1, hw), pt(run1, -hw)]

    c1 = corners(u1, run, 0.0, half_width)
    c2 = corners(u2, 0.0, run, half_width)
    face1 = builder.rect_fill(hsf, part, body, c1)
    face2 = builder.rect_fill(hsf, part, body, c2)
    sharp = builder.join(hsf, part, body, [face1, face2])
    part.Update()

    edge_ref = builder.find_edge_near(doc, part, spa, hsf, body, sharp, (0.0, 0.0, 0.0))
    filleted = builder.edge_fillet(part, edge_ref, radius)
    body.AppendHybridShape(filleted)
    part.Update()
    return filleted, w, c1, c2


def far_edges(doc, part, spa, hsf, body, sweep_face, main_surface_ref, depth_mm, near_ref=None):
    """スイープ面のエッジのうち、主面からの距離がdepth_mmにほぼ等しいものを全て
    「遠端(壁の天面側)エッジ」として返す(ガイド曲線が主面の複数パッチ(平地/アーク/平地)を
    横切ると、遠端も複数セグメントに分かれる -- 実機確認済み)。根元エッジ(距離0)や
    プロファイル辺(距離0〜depth未満)は除外する。near_refを渡すと、そこからの距離が
    近い順(=ガイドに沿ったnear->far順)にソートして返す -- AddNewFillのAddBoundは
    境界を連結順に渡す必要がある(既存rect_fillの使い方と同じ流儀)。
    """
    sel = doc.Selection
    sel.Clear()
    sel.Add(sweep_face)
    sel.Search("Topology.CGMEdge,sel")
    base_meas = spa.GetMeasurable(main_surface_ref)
    found = []
    for i in range(1, sel.Count2 + 1):
        item = sel.Item2(i)
        eref = item.Reference
        dist = base_meas.GetMinimumDistance(eref)
        if abs(dist - depth_mm) < 0.05 * depth_mm + 0.01:
            found.append(eref)
    sel.Clear()
    if not found:
        raise RuntimeError("no far edge found")
    if near_ref is not None:
        found.sort(key=lambda eref: spa.GetMeasurable(eref).GetMinimumDistance(near_ref))
    return found


def cap_line(builder, hsf, part, body, corners_row, width_dir, width_a, width_b):
    """corners_row(近端2点 or 遠端2点)から幅width_a/width_bの2点を作り、直線で結ぶ。
    主面上(リフト無し)の境界専用 -- リフトされた遠端キャップにはvertex_nearを使う。"""
    left, right = corners_row
    center = tuple((left[i] + right[i]) / 2 for i in range(3))
    pa = builder.point(hsf, body, *(tuple(center[i] + width_a * width_dir[i] for i in range(3))))
    pb = builder.point(hsf, body, *(tuple(center[i] + width_b * width_dir[i] for i in range(3))))
    return builder.line_pt_pt(hsf, part, body, pa, pb)


def vertex_near(doc, spa, hsf, body, builder, part, sweep_face, main_surface_ref, approx_point, depth_mm):
    """sweep_face上の頂点のうち、主面からの距離がdepth_mmに一致(=遠端側)し、かつ
    approx_pointに最も近いものをReferenceで返す(遠端はリフトされているため解析的な
    厳密一致は期待できず、近似点で当たりを付けて実際の頂点を特定する)。"""
    sel = doc.Selection
    sel.Clear()
    sel.Add(sweep_face)
    sel.Search("Topology.CGMVertex,sel")
    probe = builder.point(hsf, body, *approx_point)
    part.Update()
    probe_ref = part.CreateReferenceFromObject(probe)
    base_meas = spa.GetMeasurable(main_surface_ref)
    best_ref, best_dist = None, None
    for i in range(1, sel.Count2 + 1):
        vref = sel.Item2(i).Reference
        vmeas = spa.GetMeasurable(vref)
        if abs(base_meas.GetMinimumDistance(vref) - depth_mm) > 0.05 * depth_mm + 0.01:
            continue
        d = vmeas.GetMinimumDistance(probe_ref)
        if best_dist is None or d < best_dist:
            best_ref, best_dist = vref, d
    sel.Clear()
    if best_ref is None:
        raise RuntimeError("vertex not found")
    return best_ref


def screenshot(catia, path):
    OUTPUT_DIR.mkdir(exist_ok=True)
    viewer = catia.ActiveWindow.ActiveViewer
    viewer.Update()
    viewer.Reframe()
    viewer.CaptureToFile(2, str(path))
    print(f"  screenshot: {path}", flush=True)


def main() -> None:
    builder = SyntheticPartBuilder()
    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "SPIKE_BEAD_ON_FILLETED"

    half_width = 60.0
    filleted, w, c1, c2 = build_filleted_base(builder, hsf, part, body, spa, doc, half_width=half_width)
    surface_ref = part.CreateReferenceFromObject(filleted)
    print("1) filleted main surface OK", flush=True)

    # ビード断面パラメータ(bead.py BEAD_*_RANGE_MMの中央値あたり)
    depth_mm, top_width_mm, wall_angle_deg = 7.0, 30.0, 50.0
    wall_run_mm = depth_mm / math.tan(math.radians(wall_angle_deg))
    wall_slant_mm = depth_mm / math.sin(math.radians(wall_angle_deg))
    half_top = top_width_mm / 2.0
    half_footprint = half_top + wall_run_mm
    print(f"   bead: depth={depth_mm} top_width={top_width_mm} wall_angle={wall_angle_deg} "
          f"half_footprint={half_footprint:.2f} wall_slant={wall_slant_mm:.2f}", flush=True)

    origin_pt = builder.point(hsf, body, 0.0, 0.0, 0.0)
    axis_pt = builder.point(hsf, body, *w)
    axis_line = builder.line_pt_pt(hsf, part, body, origin_pt, axis_pt)
    plane = hsf.AddNewPlaneNormal(
        part.CreateReferenceFromObject(axis_line), part.CreateReferenceFromObject(origin_pt)
    )
    body.AppendHybridShape(plane)
    part.Update()
    centerline = hsf.AddNewIntersection(part.CreateReferenceFromObject(plane), surface_ref)
    body.AppendHybridShape(centerline)
    part.Update()
    centerline_ref = part.CreateReferenceFromObject(centerline)
    print("2) centerline (Intersection) OK", flush=True)

    def curve_par(distance, invert):
        c = hsf.AddNewCurvePar(centerline_ref, surface_ref, distance, invert, True)
        body.AppendHybridShape(c)
        part.Update()
        return c, part.CreateReferenceFromObject(c)

    outer_left, outer_left_ref = curve_par(half_width, False)
    root_left, root_left_ref = curve_par(half_footprint, False)
    root_right, root_right_ref = curve_par(half_footprint, True)
    outer_right, outer_right_ref = curve_par(half_width, True)
    print("3) 4x CurvePar (outer_left/root_left/root_right/outer_right) OK", flush=True)

    # 平地パッチ(outer<->root)。端は既知の解析直線(c1[0]/c1[1]がrun=80端、c2[2]/c2[3]がrun=80端)。
    # centerlineの近端はc1のrun=80側(fold内部)、遠端はc2のrun=80側。
    near_row = (c1[0], c1[1])  # leg1のrun=80端 (near側の"外側の端")
    far_row = (c2[2], c2[3])   # leg2のrun=80端 (far側の"外側の端")

    near_cap_left = cap_line(builder, hsf, part, body, near_row, w, -half_width, -half_footprint)
    far_cap_left = cap_line(builder, hsf, part, body, far_row, w, -half_width, -half_footprint)
    plain_left = builder.fill(hsf, part, body, [outer_left, near_cap_left, root_left, far_cap_left])

    near_cap_right = cap_line(builder, hsf, part, body, near_row, w, half_footprint, half_width)
    far_cap_right = cap_line(builder, hsf, part, body, far_row, w, half_footprint, half_width)
    plain_right = builder.fill(hsf, part, body, [outer_right, near_cap_right, root_right, far_cap_right])
    part.Update()
    print("4) plain_left / plain_right Fill OK", flush=True)

    # 壁: Mode=4スイープ。符号(内向き=中心に向かう向き)は実機で確認する。
    centerline_length = spa.GetMeasurable(centerline_ref).Length

    def build_wall(guide_ref, invert_angle_sign):
        sweep = hsf.AddNewSweepLine(guide_ref)
        sweep.Mode = 4
        sweep.FirstGuideSurf = surface_ref
        angle = -wall_angle_deg if invert_angle_sign else wall_angle_deg
        sweep.SetAngle(1, angle)
        sweep.SetLength(1, wall_slant_mm)
        body.AppendHybridShape(sweep)
        part.Update()
        return sweep

    wall_left = build_wall(root_left_ref, invert_angle_sign=False)
    print("5) wall_left Sweep(Mode=4) OK", flush=True)
    wall_right = build_wall(root_right_ref, invert_angle_sign=True)
    print("6) wall_right Sweep(Mode=4) OK", flush=True)

    # 天面キャップの端点は遠端エッジ自体がリフトされているため解析的に一致しない。
    # 近似点(半幅top_widthの位置、リフトは無視)で当たりを付けて実際の頂点を特定する。
    approx_near_left = tuple(near_row[0][i] + half_top * w[i] for i in range(3))
    approx_near_right = tuple(near_row[0][i] + (-half_top) * w[i] for i in range(3))
    approx_far_left = tuple(far_row[0][i] + half_top * w[i] for i in range(3))
    approx_far_right = tuple(far_row[0][i] + (-half_top) * w[i] for i in range(3))

    v_near_left = vertex_near(doc, spa, hsf, body, builder, part, wall_left, surface_ref, approx_near_left, depth_mm)
    v_near_right = vertex_near(doc, spa, hsf, body, builder, part, wall_right, surface_ref, approx_near_right, depth_mm)
    v_far_left = vertex_near(doc, spa, hsf, body, builder, part, wall_left, surface_ref, approx_far_left, depth_mm)
    v_far_right = vertex_near(doc, spa, hsf, body, builder, part, wall_right, surface_ref, approx_far_right, depth_mm)
    print("7b) top-cap vertices located OK", flush=True)

    # near側頂点からの距離でソートし、near->farの連結順にする(AddNewFillのAddBoundは
    # 既存rect_fillと同じく境界を連結順に渡す必要があると判明したため)。
    top_left_refs = far_edges(doc, part, spa, hsf, body, wall_left, surface_ref, depth_mm, near_ref=v_near_left)
    top_right_refs = far_edges(doc, part, spa, hsf, body, wall_right, surface_ref, depth_mm, near_ref=v_near_right)
    print(f"7c) far edges found: left={len(top_left_refs)} segments, right={len(top_right_refs)} segments "
          f"(depth_mm={depth_mm:.1f})", flush=True)

    top_near_cap = hsf.AddNewLinePtPt(v_near_left, v_near_right)
    body.AppendHybridShape(top_near_cap)
    top_far_cap = hsf.AddNewLinePtPt(v_far_right, v_far_left)
    body.AppendHybridShape(top_far_cap)
    part.Update()
    print("7d) top-cap lines OK", flush=True)

    # 連結順(v_near_left -> near_cap -> v_near_right -> [right: near->far] -> v_far_right ->
    # far_cap -> v_far_left -> [left: far->near(逆順)] -> v_near_left)でループを閉じる。
    top_patch = hsf.AddNewFill()
    top_patch.AddBound(part.CreateReferenceFromObject(top_near_cap))
    for ref in top_right_refs:
        top_patch.AddBound(ref)
    top_patch.AddBound(part.CreateReferenceFromObject(top_far_cap))
    for ref in reversed(top_left_refs):
        top_patch.AddBound(ref)
    body.AppendHybridShape(top_patch)
    part.Update()
    print("8) top patch Fill OK", flush=True)

    whole = builder.join(hsf, part, body, [plain_left, wall_left, top_patch, wall_right, plain_right])
    part.Update()
    print("9) FULL JOIN OK", flush=True)

    screenshot(builder.catia, OUTPUT_DIR / "10_bead_on_filleted_surface_raw.png")

    area = spa.GetMeasurable(part.CreateReferenceFromObject(whole)).Area * 1e6
    print(f"total area = {area:.1f}mm^2", flush=True)

    # 構築用の点・線・平面・元の主面(filleted)・センターラインを非表示にして、
    # 最終形状(whole)だけのクリーンな画を撮る。
    sel = doc.Selection
    sel.Clear()
    for shape in (origin_pt, axis_pt, axis_line, plane, centerline, filleted,
                  outer_left, root_left, root_right, outer_right,
                  near_cap_left, far_cap_left, near_cap_right, far_cap_right,
                  top_near_cap, top_far_cap):
        sel.Add(shape)
    sel.VisProperties.SetShow(1)
    sel.Clear()
    screenshot(builder.catia, OUTPUT_DIR / "11_bead_on_filleted_surface_clean.png")

    doc.Close()


if __name__ == "__main__":
    main()

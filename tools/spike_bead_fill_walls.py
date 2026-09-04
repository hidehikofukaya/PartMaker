"""ビードの壁をFillで再構築する方式のスパイク(ユーザー指示2026-08-23)。

背景: AddNewSweepLine(Mode=4)で作った面は、境界曲線の計算(局所法線に正しく追従する
ドラフト)としては正しく機能するが、その面自体を複数回連鎖してAddNewHybridTrimに
渡すと毎回Updateが失敗することを実機で確定した(平坦な基準面上の単純なケースでも
再現、AddNewExtract/AddNewHealing/ゼロオフセット/同一文書内でのCopy・PasteSpecialに
よる依存解除、いずれも効果なし)。一方、幾何的に同一の傾斜面でもrect_fill(Fill由来)
なら連鎖トリムが問題なく成功することも確認済み。

方式: Sweepは境界曲線の抽出だけに使い、実際にトリムに使う壁面はその境界エッジから
Fillで再構築する。曲げをまたぐ区間は(平地・アーク・平地)の3セグメントに自然分解
されるので、それぞれ独立したFillパッチにしてJoinで1枚にまとめる。
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

    face1 = builder.rect_fill(hsf, part, body, corners(u1, run, 0.0, half_width))
    face2 = builder.rect_fill(hsf, part, body, corners(u2, 0.0, run, half_width))
    sharp = builder.join(hsf, part, body, [face1, face2])
    part.Update()
    edge_ref = builder.find_edge_near(doc, part, spa, hsf, body, sharp, (0.0, 0.0, 0.0))
    filleted = builder.edge_fillet(part, edge_ref, radius)
    body.AppendHybridShape(filleted)
    part.Update()
    return filleted, w, u1, u2, [face1, face2, sharp]


def decompose_wall_to_fill(builder, hsf, part, body, spa, doc, sweep_wall, surface_ref, depth_mm,
                            length_mm, near_point):
    """Sweep壁を(根元セグメント, 遠端セグメント, つなぎのプロファイル辺2本)の組に分解し、
    各組をFillパッチにして返す。曲げをまたぐ壁は複数パッチ(通常3枚)になる。

    分類はまず辺の長さでプロファイル辺(長さ=length_mm、壁の立ち上がり方向の辺)を
    分離してから、残りを基準面からの距離(根元=0、遠端=depth_mm)で分ける
    (プロファイル辺は根元側の端点で最小距離が0になるため、長さでの分離が先に必要)。
    根元・遠端・プロファイルの各セグメントは、near_point(壁ガイドの近い側の端の
    近似点)からの距離でソートし、同じ並び順で対応付ける。
    """
    base_meas = spa.GetMeasurable(surface_ref)

    sel = doc.Selection
    sel.Clear()
    sel.Add(sweep_wall)
    sel.Search("Topology.CGMEdge,sel")
    root_edges, far_edges_, profile_edges = [], [], []
    for i in range(1, sel.Count2 + 1):
        eref = sel.Item2(i).Reference
        L = spa.GetMeasurable(eref).Length
        if abs(L - length_mm) < 0.05 * length_mm + 0.01:
            profile_edges.append(eref)
            continue
        d = base_meas.GetMinimumDistance(eref)
        if d < 0.05:
            root_edges.append(eref)
        elif abs(d - depth_mm) < 0.05 * depth_mm + 0.01:
            far_edges_.append(eref)
    sel.Clear()

    near_pt_obj = builder.point(hsf, body, *near_point)
    part.Update()
    near_ref = part.CreateReferenceFromObject(near_pt_obj)

    def sort_by_near(edges):
        return sorted(edges, key=lambda e: spa.GetMeasurable(e).GetMinimumDistance(near_ref))

    root_sorted = sort_by_near(root_edges)
    far_sorted = sort_by_near(far_edges_)
    profile_sorted = sort_by_near(profile_edges)

    if len(root_sorted) != len(far_sorted) or len(profile_sorted) != len(root_sorted) + 1:
        raise RuntimeError(
            f"unexpected segment counts: root={len(root_sorted)} far={len(far_sorted)} "
            f"profile={len(profile_sorted)}"
        )

    def extracted(eref):
        e = hsf.AddNewExtract(eref)
        body.AppendHybridShape(e)
        part.Update()
        return part.CreateReferenceFromObject(e)

    patches = []
    for i in range(len(root_sorted)):
        fill = hsf.AddNewFill()
        fill.AddBound(extracted(root_sorted[i]))
        fill.AddBound(extracted(profile_sorted[i]))
        fill.AddBound(extracted(far_sorted[i]))
        fill.AddBound(extracted(profile_sorted[i + 1]))
        body.AppendHybridShape(fill)
        part.Update()
        patches.append(fill)
    return patches


def build_wall_face(builder, hsf, part, body, spa, doc, guide_ref, surface_ref, angle_deg, length_mm,
                     depth_mm, near_point):
    sweep = hsf.AddNewSweepLine(guide_ref)
    sweep.Mode = 4
    sweep.FirstGuideSurf = surface_ref
    sweep.SetAngle(1, angle_deg)
    sweep.SetLength(1, length_mm)
    body.AppendHybridShape(sweep)
    part.Update()
    patches = decompose_wall_to_fill(
        builder, hsf, part, body, spa, doc, sweep, surface_ref, depth_mm, length_mm, near_point
    )
    print(f"    [debug] decompose returned {len(patches)} patches", flush=True)
    if not patches:
        raise RuntimeError("wall decomposition produced no patches")
    for i, p in enumerate(patches):
        a = spa.GetMeasurable(part.CreateReferenceFromObject(p)).Area * 1e6
        print(f"    [debug] patch{i} area={a:.2f}", flush=True)
    if len(patches) == 1:
        face = patches[0]
    else:
        face = builder.join(hsf, part, body, patches)
    part.Update()
    print("    [debug] face built, checking area now", flush=True)
    a = spa.GetMeasurable(part.CreateReferenceFromObject(face)).Area * 1e6
    print(f"    [debug] face area={a:.2f}", flush=True)
    return face, part.CreateReferenceFromObject(face)


def main() -> None:
    builder = SyntheticPartBuilder()
    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "SPIKE_FILL_WALLS"

    half_width, run, bend_deg = 60.0, 80.0, 70.0
    filleted, w, u1, u2, hide_base = build_filleted_base(
        builder, hsf, part, body, spa, doc, half_width=half_width, run=run, bend_deg=bend_deg
    )
    surface_ref = part.CreateReferenceFromObject(filleted)

    depth_mm, wall_angle_deg, top_width_mm = 7.0, 50.0, 30.0
    wall_slant_mm = depth_mm / math.sin(math.radians(wall_angle_deg))
    wall_run_mm = depth_mm / math.tan(math.radians(wall_angle_deg))
    half_footprint = top_width_mm / 2.0 + wall_run_mm
    run_out_pos = run - 15.0

    origin_pt = builder.point(hsf, body, 0.0, 0.0, 0.0)
    axis_pt = builder.point(hsf, body, *w)
    axis_line = builder.line_pt_pt(hsf, part, body, origin_pt, axis_pt)
    plane = hsf.AddNewPlaneNormal(part.CreateReferenceFromObject(axis_line), part.CreateReferenceFromObject(origin_pt))
    body.AppendHybridShape(plane)
    part.Update()
    centerline = hsf.AddNewIntersection(part.CreateReferenceFromObject(plane), surface_ref)
    body.AppendHybridShape(centerline)
    part.Update()
    centerline_ref = part.CreateReferenceFromObject(centerline)

    root_left = hsf.AddNewCurvePar(centerline_ref, surface_ref, half_footprint, False, True)
    body.AppendHybridShape(root_left)
    part.Update()
    near_point_left = tuple(run * u1[i] - half_footprint * w[i] for i in range(3))
    wall_left, wall_left_ref = build_wall_face(
        builder, hsf, part, body, spa, doc, part.CreateReferenceFromObject(root_left),
        surface_ref, wall_angle_deg, wall_slant_mm, depth_mm, near_point_left
    )
    print(f"wall_left rebuilt as Fill+Join OK, area={spa.GetMeasurable(wall_left_ref).Area * 1e6:.2f}", flush=True)

    root_right = hsf.AddNewCurvePar(centerline_ref, surface_ref, half_footprint, True, True)
    body.AppendHybridShape(root_right)
    part.Update()
    near_point_right = tuple(run * u1[i] + half_footprint * w[i] for i in range(3))
    wall_right, wall_right_ref = build_wall_face(
        builder, hsf, part, body, spa, doc, part.CreateReferenceFromObject(root_right),
        surface_ref, wall_angle_deg, wall_slant_mm, depth_mm, near_point_right
    )
    print(f"wall_right rebuilt as Fill+Join OK, area={spa.GetMeasurable(wall_right_ref).Area * 1e6:.2f}", flush=True)

    def width_guide(u, run_pos):
        left_pt = tuple(run_pos * u[i] - half_footprint * w[i] for i in range(3))
        right_pt = tuple(run_pos * u[i] + half_footprint * w[i] for i in range(3))
        p_l = builder.point(hsf, body, *left_pt)
        p_r = builder.point(hsf, body, *right_pt)
        guide = builder.line_pt_pt(hsf, part, body, p_r, p_l)
        part.Update()
        return part.CreateReferenceFromObject(guide), left_pt

    guide_near, near_point_n = width_guide(u1, run_out_pos)
    wall_near, wall_near_ref = build_wall_face(
        builder, hsf, part, body, spa, doc, guide_near,
        surface_ref, wall_angle_deg, wall_slant_mm, depth_mm, near_point_n
    )
    print(f"wall_near rebuilt OK, area={spa.GetMeasurable(wall_near_ref).Area * 1e6:.2f}", flush=True)

    guide_far, near_point_f = width_guide(u2, run_out_pos)
    wall_far, wall_far_ref = build_wall_face(
        builder, hsf, part, body, spa, doc, guide_far,
        surface_ref, wall_angle_deg, wall_slant_mm, depth_mm, near_point_f
    )
    print(f"wall_far rebuilt OK, area={spa.GetMeasurable(wall_far_ref).Area * 1e6:.2f}", flush=True)

    print("dist(left,near)=", spa.GetMeasurable(wall_left_ref).GetMinimumDistance(wall_near_ref), flush=True)
    print("dist(left,far)=", spa.GetMeasurable(wall_left_ref).GetMinimumDistance(wall_far_ref), flush=True)
    print("dist(right,near)=", spa.GetMeasurable(wall_right_ref).GetMinimumDistance(wall_near_ref), flush=True)
    print("dist(right,far)=", spa.GetMeasurable(wall_right_ref).GetMinimumDistance(wall_far_ref), flush=True)

    import itertools
    count = 0
    for o1, o2, o3, o4, o5, o6 in itertools.product([1, -1], repeat=6):
        try:
            s = hsf.AddNewHybridTrim(wall_left_ref, o1, wall_near_ref, o2)
            body.AppendHybridShape(s)
            part.Update()
            s_ref = part.CreateReferenceFromObject(s)
            s = hsf.AddNewHybridTrim(s_ref, o3, wall_right_ref, o4)
            body.AppendHybridShape(s)
            part.Update()
            s_ref = part.CreateReferenceFromObject(s)
            s = hsf.AddNewHybridTrim(s_ref, o5, wall_far_ref, o6)
            body.AppendHybridShape(s)
            part.Update()
            area = spa.GetMeasurable(part.CreateReferenceFromObject(s)).Area * 1e6
            print(f"o=({o1},{o2},{o3},{o4},{o5},{o6}): OK area={area:.2f}", flush=True)
            count += 1
        except Exception:
            pass
    print("total working combos (Fill-rebuilt walls, curved base):", count, flush=True)

    doc.Close()


if __name__ == "__main__":
    main()

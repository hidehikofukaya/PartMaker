"""ビードを打点手前で終わらせる「閉じた筒」版のスパイク(ユーザー指摘2026-08-22への対応)。

追加要素: 走行方向の壁(wall_left/wall_right)に加えて、両端(打点側)に幅方向の壁
(wall_near/wall_far)を立て、4枚の壁+天面オフセットを全てトリムで閉じたループに連結し、
その稜線全周をエッジフィレットする。

幅方向の壁のガイド線は、p_right->p_left の順で作ると内向き(折れ目/ビード中心側)に
ドラフトすることを実機で確認済み(reversed_guide=Trueが正)。角度の符号は効かない
(直線ガイドの場合、W方向はガイドの構築順だけで決まる)。
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


def find_ridge_edges(doc, spa, surface_ref, target_obj, depth_mm, near_ref_meas, want_near, expected_lengths):
    sel = doc.Selection
    sel.Clear()
    sel.Add(target_obj)
    sel.Search("Topology.CGMEdge,sel")
    base_meas = spa.GetMeasurable(surface_ref)
    out = []
    for i in range(1, sel.Count2 + 1):
        eref = sel.Item2(i).Reference
        d_base = base_meas.GetMinimumDistance(eref)
        if abs(d_base - depth_mm) > 0.05 * depth_mm + 0.01:
            continue
        length = spa.GetMeasurable(eref).Length
        if expected_lengths is not None and not any(abs(length - el) < 0.5 for el in expected_lengths):
            continue
        d_ref = near_ref_meas.GetMinimumDistance(eref)
        is_near = d_ref < 1.0
        if is_near == want_near:
            out.append(eref)
    sel.Clear()
    return out


def screenshot(catia, path, hide_shapes, doc):
    OUTPUT_DIR.mkdir(exist_ok=True)
    sel = doc.Selection
    sel.Clear()
    for s in hide_shapes:
        sel.Add(s)
    sel.VisProperties.SetShow(1)
    sel.Clear()
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
    body.Name = "SPIKE_CLOSED_TUBE"

    half_width, run, bend_deg = 60.0, 80.0, 70.0
    filleted, w, u1, u2, hide_base = build_filleted_base(
        builder, hsf, part, body, spa, doc, half_width=half_width, run=run, bend_deg=bend_deg
    )
    surface_ref = part.CreateReferenceFromObject(filleted)

    depth_mm, wall_angle_deg, top_width_mm = 7.0, 50.0, 30.0
    wall_slant_mm = depth_mm / math.sin(math.radians(wall_angle_deg))
    wall_run_mm = depth_mm / math.tan(math.radians(wall_angle_deg))
    half_footprint = top_width_mm / 2.0 + wall_run_mm
    run_out_margin = 15.0  # 打点(パネル外端)からの余白
    run_out_pos = run - run_out_margin  # 折れ目からの距離

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

    def build_run_wall(offset_dist, invert):
        curve = hsf.AddNewCurvePar(centerline_ref, surface_ref, offset_dist, invert, True)
        body.AppendHybridShape(curve)
        part.Update()
        curve_ref = part.CreateReferenceFromObject(curve)
        sweep = hsf.AddNewSweepLine(curve_ref)
        sweep.Mode = 4
        sweep.FirstGuideSurf = surface_ref
        sweep.SetAngle(1, wall_angle_deg)
        sweep.SetLength(1, wall_slant_mm)
        body.AppendHybridShape(sweep)
        part.Update()
        return sweep, part.CreateReferenceFromObject(sweep)

    wall_left, wall_left_ref = build_run_wall(half_footprint, False)
    wall_right, wall_right_ref = build_run_wall(half_footprint, True)
    print("run-direction walls (left/right) OK", flush=True)

    def build_width_wall(u, run_pos, extra_construction):
        left_pt = tuple(run_pos * u[i] - half_footprint * w[i] for i in range(3))
        right_pt = tuple(run_pos * u[i] + half_footprint * w[i] for i in range(3))
        p_l = builder.point(hsf, body, *left_pt)
        p_r = builder.point(hsf, body, *right_pt)
        # reversed(p_r -> p_l)にすると内向き(折れ目側)にドラフトすることを実機で確認済み。
        guide = builder.line_pt_pt(hsf, part, body, p_r, p_l)
        part.Update()
        guide_ref = part.CreateReferenceFromObject(guide)
        sweep = hsf.AddNewSweepLine(guide_ref)
        sweep.Mode = 4
        sweep.FirstGuideSurf = surface_ref
        sweep.SetAngle(1, wall_angle_deg)
        sweep.SetLength(1, wall_slant_mm)
        body.AppendHybridShape(sweep)
        part.Update()
        extra_construction.extend([p_l, p_r, guide])
        return sweep, part.CreateReferenceFromObject(sweep)

    extra = []
    wall_near, wall_near_ref = build_width_wall(u1, run_out_pos, extra)
    wall_far, wall_far_ref = build_width_wall(u2, run_out_pos, extra)
    print("width-direction walls (near/far, run-out) OK", flush=True)

    offset = hsf.AddNewOffset(surface_ref, depth_mm, False, 0.001)
    body.AppendHybridShape(offset)
    part.Update()
    offset_ref = part.CreateReferenceFromObject(offset)

    # 4壁+天面を順にトリムして閉じたループにする。向きパラメータは実機で当たりを付ける。
    step = hsf.AddNewHybridTrim(wall_left_ref, 1, offset_ref, -1)
    body.AppendHybridShape(step)
    part.Update()
    step_ref = part.CreateReferenceFromObject(step)
    print("trim 1 (left+top) OK, area=", spa.GetMeasurable(step_ref).Area * 1e6, flush=True)

    step = hsf.AddNewHybridTrim(step_ref, 1, wall_right_ref, -1)
    body.AppendHybridShape(step)
    part.Update()
    step_ref = part.CreateReferenceFromObject(step)
    print("trim 2 (+right) OK, area=", spa.GetMeasurable(step_ref).Area * 1e6, flush=True)

    for o1c, o2c in ((-1, 1), (-1, -1)):
        step3 = hsf.AddNewHybridTrim(step_ref, o1c, wall_near_ref, o2c)
        body.AppendHybridShape(step3)
        part.Update()
        step3_ref = part.CreateReferenceFromObject(step3)
        area3 = spa.GetMeasurable(step3_ref).Area * 1e6
        print(f"trim 3 (+near) o1c={o1c} o2c={o2c}: area={area3:.1f}", flush=True)
        for o1d in (1, -1):
            for o2d in (1, -1):
                try:
                    step4 = hsf.AddNewHybridTrim(step3_ref, o1d, wall_far_ref, o2d)
                    body.AppendHybridShape(step4)
                    part.Update()
                    area4 = spa.GetMeasurable(part.CreateReferenceFromObject(step4)).Area * 1e6
                    print(f"  trim 4 (+far) o1d={o1d} o2d={o2d}: OK area={area4:.1f}", flush=True)
                except Exception as exc:  # noqa: BLE001
                    print(f"  trim 4 (+far) o1d={o1d} o2d={o2d}: FAIL {str(exc)[:80]}", flush=True)

    doc.Close()


if __name__ == "__main__":
    main()

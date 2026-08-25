"""4枚の個別オーバーサイズ壁スイープを、BiTangentシェイプフィレット3回で
1本の連続壁バンド(4隅全てR付き)に統合できるかを検証する。
  f1 = BiT(左壁A, 近ガイド壁B)   -> 角AB
  f2 = BiT(右壁C, 遠ガイド壁D)   -> 角CD
  f3 = BiT(f1, f2)               -> 角BC + 角DA を1フィーチャーで(2交差同時)
方向は総当たり+プローブ点(残るべき側の点)で選ぶ。BRep参照は一切使わない。
"""
import sys
import math
sys.path.insert(0, r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_generator\src")
from synthetic_generator.gsd_build import SyntheticPartBuilder

LOG_PATH = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\test_bitangent_band.log"
log = open(LOG_PATH, "w", encoding="utf-8", buffering=1)


def p(*args):
    msg = " ".join(str(a) for a in args)
    log.write(msg + "\n")
    log.flush()


def cleanup_failed(doc, part, feature_obj):
    try:
        sel2 = doc.Selection
        sel2.Clear()
        sel2.Add(feature_obj)
        sel2.Delete()
        part.Update()
    except Exception as exc:
        p(f"    [cleanup warning] {str(exc)[:100]}")


builder = SyntheticPartBuilder()
doc = builder.new_part_document()
part = doc.Part
hsf = part.HybridShapeFactory
spa = doc.GetWorkbench("SPAWorkbench")
body = part.HybridBodies.Add()
body.Name = "BITANGENT_BAND"

half_width, run, bend_deg, bend_radius = 60.0, 80.0, 70.0, 30.0
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
edge_ref0 = builder.find_edge_near(doc, part, spa, hsf, body, sharp, (0.0, 0.0, 0.0))
base_filleted = builder.edge_fillet(part, edge_ref0, bend_radius)
body.AppendHybridShape(base_filleted)
part.Update()
surface_ref = part.CreateReferenceFromObject(base_filleted)
p("step1: reference OK")

depth_mm, wall_angle_deg, top_width_mm = 7.0, 50.0, 30.0
wall_slant_mm = depth_mm / math.sin(math.radians(wall_angle_deg))
wall_run_mm = depth_mm / math.tan(math.radians(wall_angle_deg))
half_footprint = top_width_mm / 2.0 + wall_run_mm
run_out_pos = run - 15.0
MARGIN = 20.0
CORNER_R = 2.0

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
root_left_ref = part.CreateReferenceFromObject(root_left)

root_right = hsf.AddNewCurvePar(centerline_ref, surface_ref, half_footprint, True, True)
body.AppendHybridShape(root_right)
part.Update()
root_right_ref = part.CreateReferenceFromObject(root_right)


def width_guide(u, run_pos, margin):
    left_pt = tuple(run_pos * u[i] - (half_footprint + margin) * w[i] for i in range(3))
    right_pt = tuple(run_pos * u[i] + (half_footprint + margin) * w[i] for i in range(3))
    p_l = builder.point(hsf, body, *left_pt)
    p_r = builder.point(hsf, body, *right_pt)
    guide = builder.line_pt_pt(hsf, part, body, p_r, p_l)
    part.Update()
    return guide, part.CreateReferenceFromObject(guide)


guide_near_over, guide_near_over_ref = width_guide(u1, run_out_pos, MARGIN)
guide_far_over, guide_far_over_ref = width_guide(u2, run_out_pos, MARGIN)


def make_wall(curve_ref, label):
    sw = hsf.AddNewSweepLine(curve_ref)
    sw.Mode = 4
    sw.FirstGuideSurf = surface_ref
    sw.SetAngle(1, wall_angle_deg)
    sw.SetLength(1, wall_slant_mm)
    body.AppendHybridShape(sw)
    part.Update()
    area = spa.GetMeasurable(part.CreateReferenceFromObject(sw)).Area * 1e6
    p(f"wall {label} OK, area={area:.2f}mm^2")
    return sw, part.CreateReferenceFromObject(sw)


wallA, wallA_ref = make_wall(root_left_ref, "A(left)")
wallB, wallB_ref = make_wall(guide_near_over_ref, "B(near)")
wallC, wallC_ref = make_wall(root_right_ref, "C(right)")
wallD, wallD_ref = make_wall(guide_far_over_ref, "D(far)")

# ---- プローブ点: 各壁の「残るべき中央部」の根元付近の点 ----
def probe(coord):
    pt = builder.point(hsf, body, *coord)
    part.Update()
    return part.CreateReferenceFromObject(pt)

# 壁の根元は基準面上の曲線。ベンドフィレットの接線点はrun≈R*tan(35°)≈21mmなので
# プローブは確実に平面部となるrun=50に置く(run=30はまだ円筒面上で座標が1.69mm浮く)。
PROBE_RUN = 50.0
probe_A = probe(tuple(PROBE_RUN * u1[i] - half_footprint * w[i] for i in range(3)))
probe_B = probe(tuple(run_out_pos * u1[i] for i in range(3)))  # guide_near中央
probe_C = probe(tuple(PROBE_RUN * u1[i] + half_footprint * w[i] for i in range(3)))
probe_D = probe(tuple(run_out_pos * u2[i] for i in range(3)))  # guide_far中央


def bitangent(ref1, ref2, probes, label):
    """全方向総当たり。全probeが距離<0.1で残る最初の成功を返す。"""
    results = []
    for o1 in (1, -1):
        for o2 in (1, -1):
            fb = hsf.AddNewFilletBiTangent(ref1, ref2, CORNER_R, o1, o2, 1, 1)
            body.AppendHybridShape(fb)
            try:
                part.Update()
            except Exception as exc:
                p(f"  {label} o1={o1} o2={o2}: FAILED {str(exc)[:80]}")
                cleanup_failed(doc, part, fb)
                continue
            fref = part.CreateReferenceFromObject(fb)
            meas = spa.GetMeasurable(fref)
            area = meas.Area * 1e6
            dists = [meas.GetMinimumDistance(pr) for pr in probes]
            ok = all(d < 0.1 for d in dists)
            p(f"  {label} o1={o1} o2={o2}: built, area={area:.2f}mm^2, probe_dists={[f'{d:.2f}' for d in dists]}, keep={'YES' if ok else 'no'}")
            if ok:
                return fb, fref, area
            results.append((fb, area, dists))
            cleanup_failed(doc, part, fb)
    raise RuntimeError(f"{label}: no orientation kept all probes")


p("--- f1 = BiT(A,B) ---")
f1, f1_ref, area1 = bitangent(wallA_ref, wallB_ref, [probe_A, probe_B], "f1")
p("--- f2 = BiT(C,D) ---")
f2, f2_ref, area2 = bitangent(wallC_ref, wallD_ref, [probe_C, probe_D], "f2")
p("--- f3 = BiT(f1,f2): two corners at once ---")
f3, f3_ref, area3 = bitangent(f1_ref, f2_ref, [probe_A, probe_B, probe_C, probe_D], "f3")

expected_band = 287.31 * wall_slant_mm
p(f"final band area={area3:.2f}mm^2 (rough expectation ~{expected_band:.0f}mm^2 minus corner cuts)")

viewer0 = doc.Application.ActiveWindow.ActiveViewer
viewer0.Reframe()
shot_path = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\bitangent_band_success.png"
viewer0.CaptureToFile(2, shot_path)
p("screenshot saved:", shot_path)
out_path = r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\bitangent_band_success.CATPart"
doc.SaveAs(out_path)
p("saved to:", out_path)

doc.Close()
p("DONE")

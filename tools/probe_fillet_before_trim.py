import sys
import math
sys.path.insert(0, r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_generator\src")
from synthetic_generator.gsd_build import SyntheticPartBuilder

LOG_PATH = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\test_fillet_before_trim.log"
log = open(LOG_PATH, "w", encoding="utf-8", buffering=1)


def p(*args):
    msg = " ".join(str(a) for a in args)
    log.write(msg + "\n")
    log.flush()


def corner_point(hsf, part, body, a_ref, b_ref):
    pt = hsf.AddNewIntersection(a_ref, b_ref)
    body.AppendHybridShape(pt)
    part.Update()
    return part.CreateReferenceFromObject(pt)


def trim_to_middle(hsf, part, body, spa, curve_ref, cut_a, cut_b, expected_length):
    candidates = [(-1, 1), (-1, 2), (1, 1), (1, 2), (-1, -1), (-1, -2), (1, -1), (1, -2)]
    best = None
    for o1, o2 in candidates:
        try:
            split = hsf.AddNewHybridSplit(curve_ref, cut_a, o1)
            split.AddCuttingElem(cut_b, o2)
            body.AppendHybridShape(split)
            part.Update()
            L = spa.GetMeasurable(part.CreateReferenceFromObject(split)).Length
            if abs(L - expected_length) < 0.5:
                part.InWorkObject = split
                part.Update()
                return split
            if best is None or abs(L - expected_length) < abs(best[1] - expected_length):
                best = (split, L)
        except Exception:
            continue
    if best is None:
        raise RuntimeError("trim_to_middle: no valid split combination found")
    return best[0]


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
sf = part.ShapeFactory
body = part.HybridBodies.Add()
body.Name = "FILLET_BEFORE_TRIM"

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


def width_guide(u, run_pos):
    left_pt = tuple(run_pos * u[i] - half_footprint * w[i] for i in range(3))
    right_pt = tuple(run_pos * u[i] + half_footprint * w[i] for i in range(3))
    p_l = builder.point(hsf, body, *left_pt)
    p_r = builder.point(hsf, body, *right_pt)
    guide = builder.line_pt_pt(hsf, part, body, p_r, p_l)
    part.Update()
    return guide, part.CreateReferenceFromObject(guide)


guide_near, guide_near_ref = width_guide(u1, run_out_pos)
guide_far, guide_far_ref = width_guide(u2, run_out_pos)

c_ln = corner_point(hsf, part, body, root_left_ref, guide_near_ref)
c_lf = corner_point(hsf, part, body, root_left_ref, guide_far_ref)
c_rn = corner_point(hsf, part, body, root_right_ref, guide_near_ref)
c_rf = corner_point(hsf, part, body, root_right_ref, guide_far_ref)

centerline_length = spa.GetMeasurable(centerline_ref).Length
run_out_margin = run - run_out_pos
expected_run_mid = centerline_length - 2.0 * run_out_margin
left_mid = trim_to_middle(hsf, part, body, spa, root_left_ref, c_ln, c_lf, expected_run_mid)
right_mid = trim_to_middle(hsf, part, body, spa, root_right_ref, c_rn, c_rf, expected_run_mid)

loop = builder.join(hsf, part, body, [left_mid, guide_near, right_mid, guide_far])
part.Update()
loop_ref = part.CreateReferenceFromObject(loop)
p("step2: loop OK, length=", spa.GetMeasurable(loop_ref).Length)

sweep = hsf.AddNewSweepLine(loop_ref)
sweep.Mode = 4
sweep.FirstGuideSurf = surface_ref
sweep.SetAngle(1, wall_angle_deg)
sweep.SetLength(1, wall_slant_mm)
body.AppendHybridShape(sweep)
part.Update()
sweep_ref = part.CreateReferenceFromObject(sweep)
p("wall sweep OK, area=", spa.GetMeasurable(sweep_ref).Area * 1e6)

# ---- NEW: find seam edges on the RAW SWEEP (not the trim result) ----
sel = doc.Selection
sel.Clear()
sel.Add(sweep)
sel.Search("Topology.CGMEdge,sel")
p(f"raw sweep has {sel.Count2} edges, wall_slant_mm={wall_slant_mm:.3f}")
corner_probes = [c_ln, c_lf, c_rn, c_rf]
seam_edges = []
for i in range(1, sel.Count2 + 1):
    item = sel.Item2(i)
    meas_i = spa.GetMeasurable(item.Reference)
    L = meas_i.Length
    dmins = [meas_i.GetMinimumDistance(cp) for cp in corner_probes]
    dmin = min(dmins)
    p(f"  edge[{i}] length={L:.3f} dist_to_corner={dmin:.3f}")
    seam_edges.append(item.Reference)
p(f"trying ALL {len(seam_edges)} edges on raw sweep (no filtering)")

# ---- fillet with EXACT confirmed macro properties: prop=1(tangency), relim=2(connect), trim=0(trim), r=2mm ----
RADIUS = 2.0
success_count = 0
fillet_feature = None
for i, edge_ref in enumerate(seam_edges):
    try:
        fx = sf.AddNewSurfaceEdgeFilletWithConstantRadius(None, 1, RADIUS)
        fx.AddObjectToFillet(edge_ref)
        fx.FilletBoundaryRelimitation = 2
        fx.FilletTrimSupport = 0
        body.AppendHybridShape(fx)
        part.Update()
        p(f"  seam[{i}]: FILLET OK !!! (radius={RADIUS})")
        success_count += 1
        fillet_feature = fx
    except Exception as exc:
        p(f"  seam[{i}]: FAILED {str(exc)[:150]}")
        cleanup_failed(doc, part, fx)

p(f"RESULT: {success_count}/{len(seam_edges)} seam edges filleted successfully on raw sweep")

if success_count > 0:
    # screenshot for visual confirmation
    viewer0 = doc.Application.ActiveWindow.ActiveViewer
    viewer0.Reframe()
    shot_path = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\fillet_before_trim_wall.png"
    viewer0.CaptureToFile(2, shot_path)
    p("screenshot saved:", shot_path)

    out_path = r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\fillet_before_trim_wall.CATPart"
    doc.SaveAs(out_path)
    p("saved to:", out_path)

doc.Close()
p("DONE")

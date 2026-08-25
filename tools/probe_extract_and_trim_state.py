"""切り分け実験:
  A: BRepName再構築参照を AddNewExtract に渡す
     -> 成功すれば「参照解決はOK、フィレット演算だけが失敗」と確定
     -> 失敗すれば「BRepName参照のCOM解決自体が壊れている」と確定
  B: 手動成功時のツリー状態を完全再現: トリム(ref_wall)を作った後に
     スイープのBRepNameエッジへフィレット(手動テストしたCATPartにはトリムが存在した)
"""
import sys
import math
sys.path.insert(0, r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_generator\src")
from synthetic_generator.gsd_build import SyntheticPartBuilder

LOG_PATH = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\test_extract_and_trim_state.log"
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
body.Name = "EXTRACT_TRIM_STATE"

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
p("step2: loop OK")

sweep = hsf.AddNewSweepLine(loop_ref)
sweep.Mode = 4
sweep.FirstGuideSurf = surface_ref
sweep.SetAngle(1, wall_angle_deg)
sweep.SetLength(1, wall_slant_mm)
body.AppendHybridShape(sweep)
part.Update()
sweep_ref = part.CreateReferenceFromObject(sweep)
p("wall sweep OK")

MODS = "WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR29"


def collect_seams():
    sel_x = doc.Selection
    sel_x.Clear()
    sel_x.Add(sweep)
    sel_x.Search("Topology.CGMEdge,sel")
    seams = []
    for i in range(1, sel_x.Count2 + 1):
        ref = sel_x.Item2(i).Reference
        try:
            dn = ref.DisplayName
        except Exception:
            continue
        if not dn.startswith("Selection_REdge:"):
            continue
        L = spa.GetMeasurable(ref).Length
        if abs(L - 10.863) > 0.5:
            continue
        name = dn[len("Selection_"):]
        marker = "Cf14:())"
        pos = name.rfind(marker)
        seams.append((i, name[:pos + len(marker)] + ";" + MODS + ")"))
    sel_x.Clear()
    return seams


seams = collect_seams()
p(f"found {len(seams)} seam edges")
part.InWorkObject = sweep
part.Update()

# ==== A: AddNewExtract with the rebuilt BRepName reference ====
i, brep = seams[0]
p(f"--- A: AddNewExtract on rebuilt BRepName edge[{i}] ---")
try:
    ext_ref = part.CreateReferenceFromBRepName(brep, sweep)
    ext = hsf.AddNewExtract(ext_ref)
    body.AppendHybridShape(ext)
    part.Update()
    L_ext = spa.GetMeasurable(part.CreateReferenceFromObject(ext)).Length
    p(f"  A: Extract OK !!! length={L_ext:.3f} (expected ~10.863)")
except Exception as exc:
    p(f"  A: Extract FAILED {str(exc)[:120]}")

# ==== B: build the trim (ref_wall) FIRST, then fillet the sweep edge ====
p("--- B: create trim(surface+sweep) then fillet sweep BRepName edge ---")
ref_wall = hsf.AddNewHybridTrim(surface_ref, 1, sweep_ref, 1)
body.AppendHybridShape(ref_wall)
part.Update()
p("  trim OK; tree now matches the manually-tested document")
part.InWorkObject = ref_wall
part.Update()

ok = None
for i, brep in seams:
    fx = sf.AddNewSurfaceEdgeFilletWithConstantRadius(None, 1, 2.0)
    try:
        fx.AddObjectToFillet(part.CreateReferenceFromBRepName(brep, sweep))
        fx.FilletBoundaryRelimitation = 2
        fx.FilletTrimSupport = 0
        body.AppendHybridShape(fx)
        part.Update()
        p(f"  B edge[{i}]: FILLET OK !!!")
        ok = fx
        break
    except Exception as exc:
        p(f"  B edge[{i}]: FAILED {str(exc)[:120]}")
        cleanup_failed(doc, part, fx)

if ok:
    viewer0 = doc.Application.ActiveWindow.ActiveViewer
    viewer0.Reframe()
    shot_path = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\trim_state_fillet_success.png"
    viewer0.CaptureToFile(2, shot_path)
    p("screenshot saved:", shot_path)
    out_path = r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\trim_state_fillet_success.CATPart"
    doc.SaveAs(out_path)
    p("saved to:", out_path)

doc.Close()
p("RESULT:", "SUCCESS" if ok else "B failed (see A result for reference-resolution verdict)")
p("DONE")

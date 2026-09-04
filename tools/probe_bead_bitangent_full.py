"""厳密判定版 v2: パネル2側の「ビードが立ち上がる向き」を推測せず実測で決める。

v1の誤り: パネル2の法線を +n2 と決め打ちしていたが、2枚のパネルは谷折りなので
連続な面法線は パネル1=+n1 / パネル2=-n2 になる(くさび内側)。
この符号ミスが wall D の期待頂部位置と top_keep2 の両方に波及し、
topR の keep 判定が keep_max=7.00(=depth ちょうど)で全滅していた。

対策: 先に頂面オフセットを作り、そのオフセット面に対して ±n2 の両候補を距離測定して
実際の向き n2_eff を確定してから、壁とプローブを組み立てる。
"""
import sys
import math
sys.path.insert(0, r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_generator\src")
from synthetic_generator.gsd_build import SyntheticPartBuilder

SCR = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad"
log = open(SCR + r"\test_bead_strict2.log", "w", encoding="utf-8", buffering=1)


def p(*args):
    log.write(" ".join(str(a) for a in args) + "\n")
    log.flush()


def cleanup_failed(doc, part, feature_obj):
    try:
        s = doc.Selection
        s.Clear()
        s.Add(feature_obj)
        s.Delete()
        part.Update()
    except Exception as exc:
        p(f"    [cleanup warning] {str(exc)[:100]}")


builder = SyntheticPartBuilder()
doc = builder.new_part_document()
part = doc.Part
hsf = part.HybridShapeFactory
spa = doc.GetWorkbench("SPAWorkbench")
body = part.HybridBodies.Add()
body.Name = "BEAD_STRICT2"

half_width, run, bend_deg, bend_radius = 60.0, 80.0, 70.0, 30.0
ha = math.radians(bend_deg) / 2.0
u1 = (math.cos(ha), 0.0, -math.sin(ha))
u2 = (math.cos(ha), 0.0, math.sin(ha))
w = (0.0, 1.0, 0.0)
n1 = (math.sin(ha), 0.0, math.cos(ha))
n2_raw = (-math.sin(ha), 0.0, math.cos(ha))


def lin(*terms):
    return tuple(sum(s * v[i] for s, v in terms) for i in range(3))


def corners(u, r0, r1, hw):
    def pt(r, ww):
        return lin((r, u), (ww, w))
    return [pt(r0, -hw), pt(r0, hw), pt(r1, hw), pt(r1, -hw)]


face1 = builder.rect_fill(hsf, part, body, corners(u1, run, 0.0, half_width))
face2 = builder.rect_fill(hsf, part, body, corners(u2, 0.0, run, half_width))
sharp = builder.join(hsf, part, body, [face1, face2])
part.Update()
edge_ref0 = builder.find_edge_near(doc, part, spa, hsf, body, sharp, (0.0, 0.0, 0.0))
base_filleted = builder.edge_fillet(part, edge_ref0, bend_radius)
body.AppendHybridShape(base_filleted)
part.Update()
surface_ref = part.CreateReferenceFromObject(base_filleted)
base_area = spa.GetMeasurable(surface_ref).Area * 1e6
p(f"step1: reference OK, base area={base_area:.2f}mm^2")

depth_mm, wall_angle_deg, top_width_mm = 7.0, 50.0, 30.0
wall_slant_mm = depth_mm / math.sin(math.radians(wall_angle_deg))
wall_run_mm = depth_mm / math.tan(math.radians(wall_angle_deg))
half_footprint = top_width_mm / 2.0 + wall_run_mm
top_half = top_width_mm / 2.0
run_out_pos = run - 15.0
MARGIN, CORNER_R, RIDGE_R = 20.0, 6.0, 2.0
WALL_OVER, WALL_UNDER, PROBE_RUN = 1.6, 3.0, 50.0


def probe(coord):
    pt = builder.point(hsf, body, *coord)
    part.Update()
    return part.CreateReferenceFromObject(pt)


# ---- 頂面オフセットを先に作り、向きを実測 ----
top_probe_p1 = probe(lin((PROBE_RUN, u1), (depth_mm, n1)))
top = None
for orient in (0, 1):
    cand = hsf.AddNewOffset(surface_ref, depth_mm, orient, 0.01)
    body.AppendHybridShape(cand)
    try:
        part.Update()
    except Exception as exc:
        p(f"offset orient={orient}: FAILED {str(exc)[:60]}")
        cleanup_failed(doc, part, cand)
        continue
    d = spa.GetMeasurable(part.CreateReferenceFromObject(cand)).GetMinimumDistance(top_probe_p1)
    p(f"offset orient={orient}: dist to (+n1) probe = {d:.3f}")
    if d < 0.1:
        top = cand
        break
    cleanup_failed(doc, part, cand)
if top is None:
    p("top offset FAILED")
    doc.Close()
    raise SystemExit(1)
top_ref = part.CreateReferenceFromObject(top)
top_meas = spa.GetMeasurable(top_ref)
p("top offset OK (bead rises toward +n1 on panel1)")

# パネル2側の立ち上がり向きをオフセット面に問い合わせて確定
n2_eff = None
for sign, vec in ((+1, n2_raw), (-1, tuple(-c for c in n2_raw))):
    cand_pt = probe(lin((PROBE_RUN, u2), (depth_mm, vec)))
    d = top_meas.GetMinimumDistance(cand_pt)
    p(f"  panel2 normal candidate sign={sign:+d}: dist={d:.3f}")
    if d < 0.1:
        n2_eff = vec
if n2_eff is None:
    p("could not determine panel2 rise direction")
    doc.Close()
    raise SystemExit(1)
p(f"n2_eff = {tuple(round(c,4) for c in n2_eff)}  (n2_raw = {tuple(round(c,4) for c in n2_raw)})")

# ---- ビード輪郭曲線 ----
origin_pt = builder.point(hsf, body, 0.0, 0.0, 0.0)
axis_pt = builder.point(hsf, body, *w)
axis_line = builder.line_pt_pt(hsf, part, body, origin_pt, axis_pt)
plane = hsf.AddNewPlaneNormal(part.CreateReferenceFromObject(axis_line),
                              part.CreateReferenceFromObject(origin_pt))
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
    p_l = builder.point(hsf, body, *lin((run_pos, u), (-(half_footprint + margin), w)))
    p_r = builder.point(hsf, body, *lin((run_pos, u), (half_footprint + margin, w)))
    g = builder.line_pt_pt(hsf, part, body, p_r, p_l)
    part.Update()
    return g, part.CreateReferenceFromObject(g)


guide_near, guide_near_ref = width_guide(u1, run_out_pos, MARGIN)
guide_far, guide_far_ref = width_guide(u2, run_out_pos, MARGIN)

wall_specs = [
    ("A(left)", root_left_ref, lin((PROBE_RUN, u1), (-top_half, w), (depth_mm, n1))),
    ("B(near)", guide_near_ref, lin((run_out_pos - wall_run_mm, u1), (depth_mm, n1))),
    ("C(right)", root_right_ref, lin((PROBE_RUN, u1), (top_half, w), (depth_mm, n1))),
    ("D(far)", guide_far_ref, lin((run_out_pos - wall_run_mm, u2), (depth_mm, n2_eff))),
]
ANGLE_CANDIDATES = [-wall_angle_deg, -(180.0 - wall_angle_deg),
                    180.0 - wall_angle_deg, wall_angle_deg]


def make_wall(curve_ref, expected_top, label):
    tp = probe(expected_top)
    chosen = None
    for ang in ANGLE_CANDIDATES:
        sw = hsf.AddNewSweepLine(curve_ref)
        sw.Mode = 4
        sw.FirstGuideSurf = surface_ref
        sw.SetAngle(1, ang)
        sw.SetLength(1, wall_slant_mm)
        body.AppendHybridShape(sw)
        try:
            part.Update()
        except Exception:
            cleanup_failed(doc, part, sw)
            continue
        d = spa.GetMeasurable(part.CreateReferenceFromObject(sw)).GetMinimumDistance(tp)
        cleanup_failed(doc, part, sw)
        if d < 0.1:
            chosen = ang
            break
    if chosen is None:
        raise RuntimeError(f"{label}: no draft angle matched")
    sw = hsf.AddNewSweepLine(curve_ref)
    sw.Mode = 4
    sw.FirstGuideSurf = surface_ref
    sw.SetAngle(1, chosen)
    sw.SetLength(1, wall_slant_mm * WALL_OVER)
    sw.SetLength(2, WALL_UNDER)
    body.AppendHybridShape(sw)
    part.Update()
    a = spa.GetMeasurable(part.CreateReferenceFromObject(sw)).Area * 1e6
    p(f"wall {label}: angle={chosen:+.1f} OK, area={a:.2f}mm^2")
    return sw, part.CreateReferenceFromObject(sw)


walls = {}
for label, cref, etop in wall_specs:
    walls[label] = make_wall(cref, etop, label)
wallA_ref = walls["A(left)"][1]
wallB_ref = walls["B(near)"][1]
wallC_ref = walls["C(right)"][1]
wallD_ref = walls["D(far)"][1]

# ---- プローブ群 ----
probe_A = probe(lin((PROBE_RUN, u1), (-half_footprint, w)))
probe_B = probe(lin((run_out_pos, u1)))
probe_C = probe(lin((PROBE_RUN, u1), (half_footprint, w)))
probe_D = probe(lin((run_out_pos, u2)))

base_keep = []
for u in (u1, u2):
    for r in (PROBE_RUN, run):
        for ww in (-half_width, -half_width * 0.7, half_width * 0.7, half_width):
            base_keep.append(probe(lin((r, u), (ww, w))))
p(f"base keep probes: {len(base_keep)}")

top_keep1 = probe(lin((PROBE_RUN, u1), (depth_mm, n1)))
top_keep2 = probe(lin((PROBE_RUN, u2), (depth_mm, n2_eff)))
top_remove = probe(lin((PROBE_RUN, u1), (half_width * 0.8, w), (depth_mm, n1)))
base_remove1 = probe(lin((PROBE_RUN, u1)))
base_remove2 = probe(lin((PROBE_RUN, u2)))


def bitangent(ref1, ref2, radius, keep, remove, label, trim=1, relim=1,
              tol=0.15, remove_min=1.0):
    for o1 in (1, -1):
        for o2 in (1, -1):
            fb = hsf.AddNewFilletBiTangent(ref1, ref2, radius, o1, o2, trim, relim)
            body.AppendHybridShape(fb)
            try:
                part.Update()
            except Exception as exc:
                p(f"  {label} o1={o1} o2={o2}: BUILD FAILED {str(exc)[:55]}")
                cleanup_failed(doc, part, fb)
                continue
            fref = part.CreateReferenceFromObject(fb)
            meas = spa.GetMeasurable(fref)
            area = meas.Area * 1e6
            dk = [meas.GetMinimumDistance(x) for x in keep]
            dr = [meas.GetMinimumDistance(x) for x in remove]
            keep_ok = all(d < tol for d in dk)
            rem_ok = all(d > remove_min for d in dr)
            p(f"  {label} o1={o1} o2={o2}: area={area:9.2f} "
              f"keep_max={max(dk):6.2f}({'OK' if keep_ok else 'NG'}) "
              f"rem_min={(min(dr) if dr else float('inf')):6.2f}({'OK' if rem_ok else 'NG'})")
            if keep_ok and rem_ok:
                p(f"  {label}: ACCEPTED o1={o1} o2={o2}")
                return fb, fref, area
            cleanup_failed(doc, part, fb)
    raise RuntimeError(f"{label}: no orientation satisfied keep+remove")


p("--- band (corner R) ---")
f1, f1_ref, _ = bitangent(wallA_ref, wallB_ref, CORNER_R, [probe_A, probe_B], [], "f1(A,B)")
f2, f2_ref, _ = bitangent(wallC_ref, wallD_ref, CORNER_R, [probe_C, probe_D], [], "f2(C,D)")
f3, f3_ref, band_area = bitangent(f1_ref, f2_ref, CORNER_R,
                                  [probe_A, probe_B, probe_C, probe_D], [], "f3(f1,f2)")
p(f"BAND OK, area={band_area:.2f}mm^2")

p("--- topR = BiT(band, top) ---")
hat = hat_ref = None
for relim in (1, 0, 2):
    for trim in (1, 2):
        try:
            hat, hat_ref, a = bitangent(
                f3_ref, top_ref, RIDGE_R,
                keep=[probe_A, probe_B, probe_C, probe_D, top_keep1, top_keep2],
                remove=[top_remove],
                label=f"topR(r={relim},t={trim})", trim=trim, relim=relim)
            p(f"TOP RIDGE OK, area={a:.2f}")
            break
        except RuntimeError:
            pass
    if hat is not None:
        break
if hat is None:
    p("topR FAILED")
    doc.SaveAs(r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\bead_strict2_bandtop.CATPart")
    doc.Close()
    raise SystemExit(1)

p("--- footR = BiT(hat, base) ---")
bead = bead_ref = None
for relim in (1, 0, 2):
    for trim in (1, 2):
        try:
            bead, bead_ref, a = bitangent(
                hat_ref, surface_ref, RIDGE_R,
                keep=base_keep + [top_keep1, top_keep2],
                remove=[base_remove1, base_remove2],
                label=f"footR(r={relim},t={trim})", trim=trim, relim=relim)
            p(f"FOOT RIDGE OK, area={a:.2f}")
            break
        except RuntimeError:
            pass
    if bead is not None:
        break
if bead is None:
    p("footR FAILED")
    doc.SaveAs(r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\bead_strict2_hat.CATPart")
    doc.Close()
    raise SystemExit(1)

final_area = spa.GetMeasurable(bead_ref).Area * 1e6
p(f"FULL BEAD OK, final area={final_area:.2f}mm^2 (base alone was {base_area:.2f})")

sel = doc.Selection
sel.Clear()
for i in range(1, body.HybridShapes.Count + 1):
    sel.Add(body.HybridShapes.Item(i))
sel.VisProperties.SetShow(1)
sel.Clear()
sel.Add(bead)
sel.VisProperties.SetShow(0)
sel.Clear()
part.Update()
v = doc.Application.ActiveWindow.ActiveViewer
v.Reframe()
v.CaptureToFile(2, SCR + r"\bead_strict2.png")
p("screenshot saved")
doc.SaveAs(r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\bead_strict2.CATPart")
p("saved CATPart")
doc.Close()
p("DONE")

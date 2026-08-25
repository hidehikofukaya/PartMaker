"""ドラフト方向診断: 4枚の壁それぞれについて、SetAngleの候補値を変えながら
「頂部エッジが理論位置(内側に倒れた位置)に来ているか」を距離計測で判定する。

ビードの壁は根元(footprint)から立ち上がり、頂面(top_width)に向かって
**内側に**倒れるのが正。外側に倒れていれば ~2*wall_run のズレが出る。

判定: 内側期待点との距離(d_in)と外側期待点との距離(d_out)を両方測る。
  d_in≈0 かつ d_out大 -> 正しい
  d_out≈0 かつ d_in大 -> 逆向き(要修正)
"""
import sys
import math
sys.path.insert(0, r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_generator\src")
from synthetic_generator.gsd_build import SyntheticPartBuilder

LOG_PATH = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\test_draft_direction.log"
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
body.Name = "DRAFT_DIAGNOSIS"

half_width, run, bend_deg, bend_radius = 60.0, 80.0, 70.0, 30.0
half_angle = math.radians(bend_deg) / 2.0
u1 = (math.cos(half_angle), 0.0, -math.sin(half_angle))
u2 = (math.cos(half_angle), 0.0, math.sin(half_angle))
w = (0.0, 1.0, 0.0)
n1 = (math.sin(half_angle), 0.0, math.cos(half_angle))   # u1 x w
n2 = (-math.sin(half_angle), 0.0, math.cos(half_angle))  # u2 x w


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
p("reference OK")

depth_mm, wall_angle_deg, top_width_mm = 7.0, 50.0, 30.0
wall_slant_mm = depth_mm / math.sin(math.radians(wall_angle_deg))
wall_run_mm = depth_mm / math.tan(math.radians(wall_angle_deg))
half_footprint = top_width_mm / 2.0 + wall_run_mm
run_out_pos = run - 15.0
MARGIN = 20.0
PROBE_RUN = 50.0
p(f"depth={depth_mm} wall_angle={wall_angle_deg} slant={wall_slant_mm:.3f} run={wall_run_mm:.3f}")
p(f"half_footprint={half_footprint:.3f} top_half={top_width_mm/2:.3f}")

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


guide_near, guide_near_ref = width_guide(u1, run_out_pos, MARGIN)
guide_far, guide_far_ref = width_guide(u2, run_out_pos, MARGIN)


def probe(coord):
    pt = builder.point(hsf, body, *coord)
    part.Update()
    return part.CreateReferenceFromObject(pt)


def lin(*terms):
    """terms = [(scalar, vec), ...] -> summed 3-tuple"""
    return tuple(sum(s * v[i] for s, v in terms) for i in range(3))


# 各壁の「頂部エッジ中央」の理論位置(内側に倒れた場合 / 外側に倒れた場合)
# A(left): 根元 w=-half_footprint, 内側の頂部 w=-top_width/2, 外側なら w=-(half_footprint+wall_run)
top_half = top_width_mm / 2.0
cases = {
    "A(left)": dict(
        curve_ref=root_left_ref,
        inside=lin((PROBE_RUN, u1), (-top_half, w), (depth_mm, n1)),
        outside=lin((PROBE_RUN, u1), (-(half_footprint + wall_run_mm), w), (depth_mm, n1)),
    ),
    "C(right)": dict(
        curve_ref=root_right_ref,
        inside=lin((PROBE_RUN, u1), (top_half, w), (depth_mm, n1)),
        outside=lin((PROBE_RUN, u1), (half_footprint + wall_run_mm, w), (depth_mm, n1)),
    ),
    # B(near): 根元 run=run_out_pos(u1側), 内側の頂部は曲げ側 = run小さい方
    "B(near)": dict(
        curve_ref=guide_near_ref,
        inside=lin((run_out_pos - wall_run_mm, u1), (depth_mm, n1)),
        outside=lin((run_out_pos + wall_run_mm, u1), (depth_mm, n1)),
    ),
    "D(far)": dict(
        curve_ref=guide_far_ref,
        inside=lin((run_out_pos - wall_run_mm, u2), (depth_mm, n2)),
        outside=lin((run_out_pos + wall_run_mm, u2), (depth_mm, n2)),
    ),
}

for label, c in cases.items():
    c["probe_in"] = probe(c["inside"])
    c["probe_out"] = probe(c["outside"])

ANGLE_CANDIDATES = [wall_angle_deg, -wall_angle_deg,
                    180.0 - wall_angle_deg, -(180.0 - wall_angle_deg),
                    90.0 - wall_angle_deg, -(90.0 - wall_angle_deg)]

p("")
p("=== draft angle diagnosis (d_in should be ~0 for correct draft) ===")
verdict = {}
for label, c in cases.items():
    p(f"--- wall {label} ---")
    best = None
    for ang in ANGLE_CANDIDATES:
        sw = hsf.AddNewSweepLine(c["curve_ref"])
        sw.Mode = 4
        sw.FirstGuideSurf = surface_ref
        sw.SetAngle(1, ang)
        sw.SetLength(1, wall_slant_mm)
        body.AppendHybridShape(sw)
        try:
            part.Update()
        except Exception as exc:
            p(f"  angle={ang:+7.2f}: BUILD FAILED {str(exc)[:60]}")
            cleanup_failed(doc, part, sw)
            continue
        meas = spa.GetMeasurable(part.CreateReferenceFromObject(sw))
        area = meas.Area * 1e6
        d_in = meas.GetMinimumDistance(c["probe_in"])
        d_out = meas.GetMinimumDistance(c["probe_out"])
        tag = "INWARD-OK" if d_in < 0.1 else ("outward" if d_out < 0.1 else "neither")
        p(f"  angle={ang:+7.2f}: area={area:8.2f} d_in={d_in:6.3f} d_out={d_out:6.3f}  -> {tag}")
        if d_in < 0.1 and best is None:
            best = ang
        cleanup_failed(doc, part, sw)
    verdict[label] = best
    p(f"  => correct angle for {label}: {best}")

p("")
p("=== SUMMARY ===")
for label, ang in verdict.items():
    p(f"  {label}: {ang}")

doc.Close()
p("DONE")

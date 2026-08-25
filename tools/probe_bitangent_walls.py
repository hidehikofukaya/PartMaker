"""エッジ参照フリーの縦シームR: 壁を個別スイープ(オーバーサイズ)で2枚作り、
AddNewFilletBiTangent(サーフェス対の双接フィレット)で接続できるかを検証する。
成功すればBRep参照が一切不要になり、外部COMフィレット問題を根本回避できる。
"""
import sys
import math
sys.path.insert(0, r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_generator\src")
from synthetic_generator.gsd_build import SyntheticPartBuilder

LOG_PATH = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\test_bitangent_walls.log"
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
body.Name = "BITANGENT_WALLS"

# ---- introspect AddNewFilletBiTangent signature first ----
try:
    ti = hsf._oleobj_.GetTypeInfo()
    attr = ti.GetTypeAttr()
    for fi in range(attr[6]):
        try:
            fd = ti.GetFuncDesc(fi)
            names = ti.GetNames(fd[0])
            if names and "Fillet" in names[0]:
                p(f"HSF method: {names}")
        except Exception:
            pass
except Exception as exc:
    p(f"introspection failed: {str(exc)[:100]}")

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

# 左側の根元曲線(全長のまま=オーバーサイズ)
root_left = hsf.AddNewCurvePar(centerline_ref, surface_ref, half_footprint, False, True)
body.AppendHybridShape(root_left)
part.Update()
root_left_ref = part.CreateReferenceFromObject(root_left)


def width_guide(u, run_pos, margin):
    left_pt = tuple(run_pos * u[i] - (half_footprint + margin) * w[i] for i in range(3))
    right_pt = tuple(run_pos * u[i] + (half_footprint + margin) * w[i] for i in range(3))
    p_l = builder.point(hsf, body, *left_pt)
    p_r = builder.point(hsf, body, *right_pt)
    guide = builder.line_pt_pt(hsf, part, body, p_r, p_l)
    part.Update()
    return guide, part.CreateReferenceFromObject(guide)


# 近側の幅ガイド(マージン付き=オーバーサイズ)
guide_near_over, guide_near_over_ref = width_guide(u1, run_out_pos, MARGIN)

# ---- 壁A: root_left(開曲線)のMode=4スイープ ----
p("--- wall A: open-curve sweep of root_left ---")
sweepA = hsf.AddNewSweepLine(root_left_ref)
sweepA.Mode = 4
sweepA.FirstGuideSurf = surface_ref
sweepA.SetAngle(1, wall_angle_deg)
sweepA.SetLength(1, wall_slant_mm)
body.AppendHybridShape(sweepA)
try:
    part.Update()
    areaA = spa.GetMeasurable(part.CreateReferenceFromObject(sweepA)).Area * 1e6
    p(f"wall A OK, area={areaA:.2f}mm^2")
except Exception as exc:
    p(f"wall A FAILED: {str(exc)[:150]}")
    doc.Close()
    raise SystemExit(1)
sweepA_ref = part.CreateReferenceFromObject(sweepA)

# ---- 壁B: guide_near_over(開曲線)のMode=4スイープ ----
p("--- wall B: open-curve sweep of guide_near_over ---")
sweepB = hsf.AddNewSweepLine(guide_near_over_ref)
sweepB.Mode = 4
sweepB.FirstGuideSurf = surface_ref
sweepB.SetAngle(1, wall_angle_deg)
sweepB.SetLength(1, wall_slant_mm)
body.AppendHybridShape(sweepB)
try:
    part.Update()
    areaB = spa.GetMeasurable(part.CreateReferenceFromObject(sweepB)).Area * 1e6
    p(f"wall B OK, area={areaB:.2f}mm^2")
except Exception as exc:
    p(f"wall B FAILED: {str(exc)[:150]}")
    doc.Close()
    raise SystemExit(1)
sweepB_ref = part.CreateReferenceFromObject(sweepB)

# ---- AddNewFilletBiTangent(壁A, 壁B) 全方向探索 ----
CORNER_R = 2.0
p("--- BiTangent fillet wall A x wall B, r=2.0, all orientations ---")
ok = None
for o1 in (1, -1):
    for o2 in (1, -1):
        try:
            fb = hsf.AddNewFilletBiTangent(sweepA_ref, sweepB_ref, CORNER_R, o1, o2, 1, 1)
            body.AppendHybridShape(fb)
            part.Update()
            area = spa.GetMeasurable(part.CreateReferenceFromObject(fb)).Area * 1e6
            p(f"  o1={o1} o2={o2}: OK !!! area={area:.2f}mm^2")
            ok = fb
            break
        except Exception as exc:
            p(f"  o1={o1} o2={o2}: FAILED {str(exc)[:100]}")
            try:
                cleanup_failed(doc, part, fb)
            except Exception:
                pass
    if ok:
        break

if ok:
    viewer0 = doc.Application.ActiveWindow.ActiveViewer
    viewer0.Reframe()
    shot_path = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\bitangent_walls_success.png"
    viewer0.CaptureToFile(2, shot_path)
    p("screenshot saved:", shot_path)
    out_path = r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\bitangent_walls_success.CATPart"
    doc.SaveAs(out_path)
    p("saved to:", out_path)

doc.Close()
p("RESULT:", "SUCCESS" if ok else "ALL FAILED")
p("DONE")

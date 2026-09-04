"""bead_strict2.CATPart を開き、
 (a) パネル2正面からのキャプチャ
 (b) ランアウト端より外側の基準面が残っているか等の実測
を行う。"""
import math
import win32com.client

SCR = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad"
LOG = open(SCR + r"\verify_bead2.log", "w", encoding="utf-8", buffering=1)


def p(*a):
    LOG.write(" ".join(str(x) for x in a) + "\n")
    LOG.flush()


PART = r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\bead_strict2.CATPart"
ha = math.radians(70.0) / 2.0
u1 = (math.cos(ha), 0.0, -math.sin(ha))
u2 = (math.cos(ha), 0.0, math.sin(ha))
w = (0.0, 1.0, 0.0)
n1 = (math.sin(ha), 0.0, math.cos(ha))
n2e = (math.sin(ha), 0.0, -math.cos(ha))   # n2_eff (実測で確定済み)
depth = 7.0
run, half_width = 80.0, 60.0
run_out_pos = 65.0


def lin(*terms):
    return tuple(sum(s * v[i] for s, v in terms) for i in range(3))


app = win32com.client.Dispatch("CATIA.Application")
app.DisplayFileAlerts = False
doc = app.Documents.Open(PART)
part = doc.Part
spa = doc.GetWorkbench("SPAWorkbench")
hsf = part.HybridShapeFactory
body = part.HybridBodies.Item("BEAD_STRICT2")
n = body.HybridShapes.Count
bead = body.HybridShapes.Item(n)
bead_ref = part.CreateReferenceFromObject(bead)
meas = spa.GetMeasurable(bead_ref)
p(f"final feature: {bead.Name!r}, area={meas.Area*1e6:.2f}mm^2")

work = part.HybridBodies.Add()
work.Name = "VERIFY_PROBES"


def d_to(coord, label, expect):
    pt = hsf.AddNewPointCoord(*coord)
    work.AppendHybridShape(pt)
    part.Update()
    d = meas.GetMinimumDistance(part.CreateReferenceFromObject(pt))
    ok = "OK " if ((expect == "keep" and d < 0.15) or (expect == "gone" and d > 1.0)) else "NG!"
    p(f"  [{ok}] {label:44s} dist={d:7.3f}  (expect {expect})")
    return d


p("--- ランアウト端より外側の基準面(中心線上)は残るべき ---")
for r in (70.0, 75.0, 80.0):
    d_to(lin((r, u1)), f"panel1 run={r} w=0 (beyond run-out)", "keep")
for r in (70.0, 75.0, 80.0):
    d_to(lin((r, u2)), f"panel2 run={r} w=0 (beyond run-out)", "keep")

p("--- ビード直下の基準面は消えるべき ---")
for r in (30.0, 50.0):
    d_to(lin((r, u1)), f"panel1 run={r} w=0 (under bead)", "gone")
    d_to(lin((r, u2)), f"panel2 run={r} w=0 (under bead)", "gone")

p("--- ビード頂面(中心線上、深さdepth)は残るべき ---")
for r in (30.0, 50.0, 60.0):
    d_to(lin((r, u1), (depth, n1)), f"panel1 run={r} bead top", "keep")
    d_to(lin((r, u2), (depth, n2e)), f"panel2 run={r} bead top", "keep")

p("--- 基準面の四隅・辺は残るべき ---")
for u, nm in ((u1, "panel1"), (u2, "panel2")):
    for ww in (-half_width, 0.0, half_width):
        d_to(lin((run, u), (ww, w)), f"{nm} far edge w={ww}", "keep")

# パネル2正面ビュー
sel = doc.Selection
sel.Clear()
for i in range(1, n + 1):
    sel.Add(body.HybridShapes.Item(i))
sel.Add(work)
sel.VisProperties.SetShow(1)
sel.Clear()
sel.Add(bead)
sel.VisProperties.SetShow(0)
sel.Clear()
part.Update()

viewer = app.ActiveWindow.ActiveViewer
vp = viewer.Viewpoint3D
for name, sight, up in [
    ("panel2_face", tuple(-c for c in n2e), u2),
    ("panel1_face", tuple(-c for c in n1), u1),
]:
    vp.PutSightDirection(tuple(float(c) for c in sight))
    vp.PutUpDirection(tuple(float(c) for c in up))
    viewer.Update()
    viewer.Reframe()
    viewer.CaptureToFile(2, SCR + f"\\verify_{name}.png")
    p(f"captured {name}")

doc.Close(0) if False else doc.Close()
p("DONE")

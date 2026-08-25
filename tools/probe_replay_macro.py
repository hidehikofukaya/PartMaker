"""ユーザーの成功マクロを、記録対象だった実ドキュメント(repro_case_seam_fillet.CATPart)
上でCOMからそのまま再生する。BRepName文字列はマクロ原文のものをそのまま使用。
成功 -> COMでもフィレットは可能。今までの失敗は再構築ドキュメントとの差が原因。
失敗 -> 「同じ文書・同じ文字列・同じ手順」でもGUI内マクロとCOM外部実行に挙動差がある
        ことが確定(最終確認としてCATIA内でのマクロ再生をユーザーに依頼する)。
"""
import win32com.client

LOG_PATH = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\test_replay_macro.log"
log = open(LOG_PATH, "w", encoding="utf-8", buffering=1)


def p(*args):
    msg = " ".join(str(a) for a in args)
    log.write(msg + "\n")
    log.flush()


BREP = ("REdge:(Edge:(Face:(Brp:(GSMSweep.1;(Brp:(GSMLine.11);Brp:(GSMFill.2)));None:();Cf14:());"
        "Face:(Brp:(GSMSweep.1;(Brp:(GSMCurvePar.1;(Brp:(GSMIntersect.1;(Brp:(GSMPlane.1);Brp:(GSMFill.2)));"
        "Brp:(GSMFill.2)));Brp:(GSMFill.2)));None:();Cf14:());None:(Limits1:();Limits2:());Cf14:());"
        "WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR29)")

PART_PATH = r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\repro_case_seam_fillet.CATPart"

app = win32com.client.Dispatch("CATIA.Application")
app.DisplayFileAlerts = False
doc = app.Documents.Open(PART_PATH)
part = doc.Part
sf = part.ShapeFactory
p("document opened")

body = part.HybridBodies.Item("REPRO_CASE_FOR_MANUAL_TEST")
sweep = None
for i in range(1, body.HybridShapes.Count + 1):
    shp = body.HybridShapes.Item(i)
    if "GSMSweep" in str(shp.Name) or "スイープ" in str(shp.Name) or "ｽｲｰﾌﾟ" in str(shp.Name):
        sweep = shp
        p(f"sweep feature found: {shp.Name!r}")
        break
if sweep is None:
    # fallback: dump names
    names = [str(body.HybridShapes.Item(i).Name) for i in range(1, body.HybridShapes.Count + 1)]
    p("sweep not found; shapes:", " | ".join(names[-15:]))
    raise SystemExit(1)

# --- exact macro replay ---
p("--- replaying macro exactly ---")
f = sf.AddNewSurfaceEdgeFilletWithConstantRadius(None, 1, 5.0)
try:
    f.FilletBoundaryRelimitation = 2
    f.EdgePropagation = 1
    f.FilletBoundaryRelimitation = 2
    f.FilletTrimSupport = 0
    ref = part.CreateReferenceFromBRepName(BREP, sweep)
    p("CreateReferenceFromBRepName OK")
    f.AddObjectToFillet(ref)
    p("AddObjectToFillet OK")
    # macro then set radius 4 -> 3 -> 2 via parameter; use Radius property directly
    f.Radius.Value = 2.0
    part.Update()
    p("REPLAY SUCCESS !!! fillet built at r=2.0")
    out = r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\replay_macro_success.CATPart"
    doc.SaveAs(out)
    p("saved:", out)
except Exception as exc:
    p(f"REPLAY FAILED: {str(exc)[:200]}")

doc.Close()
p("DONE")

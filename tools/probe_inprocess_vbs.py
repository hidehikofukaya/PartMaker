"""SystemService.Evaluate で、フィレット作成VBScriptをCATIAプロセス内部で実行する。
外部COMで失敗した「同一ドキュメント・同一BRepName・同一手順」がin-processなら通るかを検証。
"""
import win32com.client

LOG_PATH = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\test_inprocess_vbs.log"
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
p("document opened (active)")

# find actual sweep feature name from python side first
part = doc.Part
body = part.HybridBodies.Item("REPRO_CASE_FOR_MANUAL_TEST")
sweep_name = None
for i in range(1, body.HybridShapes.Count + 1):
    nm = str(body.HybridShapes.Item(i).Name)
    if "GSMSweep" in nm or "スイープ" in nm or "ｽｲｰﾌﾟ" in nm:
        sweep_name = nm
        break
p(f"sweep feature name: {sweep_name!r}")

vbs = f'''
Function DoFillet()
  On Error Resume Next
  Set partDocument1 = CATIA.ActiveDocument
  Set part1 = partDocument1.Part
  Set shapeFactory1 = part1.ShapeFactory
  Set f = shapeFactory1.AddNewSurfaceEdgeFilletWithConstantRadius(Nothing, 1, 2.000000)
  f.FilletBoundaryRelimitation = 2
  f.EdgePropagation = 1
  f.FilletTrimSupport = 0
  Set hb = part1.HybridBodies.Item("REPRO_CASE_FOR_MANUAL_TEST")
  Set sw = hb.HybridShapes.Item("{sweep_name}")
  Set ref1 = part1.CreateReferenceFromBRepName("{BREP}", sw)
  f.AddObjectToFillet ref1
  part1.Update
  If Err.Number <> 0 Then
    DoFillet = "FAILED: " & Err.Number & " " & Err.Description
  Else
    DoFillet = "OK"
  End If
End Function
'''

sysvc = app.SystemService
result = None
for lang in (1, 0, 2):
    try:
        result = sysvc.Evaluate(vbs, lang, "DoFillet", [])
        p(f"Evaluate(lang={lang}) returned: {result!r}")
        break
    except Exception as exc:
        p(f"Evaluate(lang={lang}) raised: {str(exc)[:150]}")

if result == "OK":
    out = r"C:\Users\hide2\IdeaBox\PartMaker\tools\probe_output\inprocess_vbs_success.CATPart"
    doc.SaveAs(out)
    p("IN-PROCESS FILLET SUCCESS !!! saved:", out)
else:
    p("in-process attempt did not succeed")

doc.Close()
p("DONE")

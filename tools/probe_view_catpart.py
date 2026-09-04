"""生成済みCATPartを開き、最終フィーチャーだけを表示して複数方向から撮る。"""
import math
import pathlib
import sys

import win32com.client

OUT = pathlib.Path(__file__).resolve().parent / "probe_output"


def norm(v):
    m = math.sqrt(sum(c * c for c in v))
    return tuple(c / m for c in v)


def main(path: str, tag: str) -> None:
    app = win32com.client.Dispatch("CATIA.Application")
    app.DisplayFileAlerts = False
    for i in range(app.Documents.Count, 0, -1):
        try: app.Documents.Item(i).Close()
        except Exception: pass
    doc = app.Documents.Open(path)
    part = doc.Part
    body = part.HybridBodies.Item(1)
    n = body.HybridShapes.Count
    sel = doc.Selection
    sel.Clear()
    for i in range(1, n + 1):
        sel.Add(body.HybridShapes.Item(i))
    sel.VisProperties.SetShow(1)
    sel.Clear()
    sel.Add(body.HybridShapes.Item(n))
    sel.VisProperties.SetShow(0)
    sel.Clear()
    part.Update()
    viewer = app.ActiveWindow.ActiveViewer
    vp = viewer.Viewpoint3D
    for name, sight, up in (
        ("iso", norm((-1.0, -1.0, -0.8)), (0.0, 0.0, 1.0)),
        ("iso2", norm((1.0, -0.8, -1.0)), (0.0, 0.0, 1.0)),
    ):
        vp.PutSightDirection(tuple(float(c) for c in sight))
        vp.PutUpDirection(tuple(float(c) for c in up))
        viewer.Update()
        viewer.Reframe()
        p = OUT / f"view_{tag}_{name}.png"
        viewer.CaptureToFile(2, str(p))
        print(f"  {p}", flush=True)
    doc.Close()


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

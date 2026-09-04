"""1つのCATPartを複数の視線方向から撮る(部品の向きが任意なので、固定視線だと
真横から見て何も分からない絵になることがある)。"""
import math
import pathlib
import sys

import win32com.client

HIDE_PREFIXES = ("点", "直線", "Point", "Line", "ﾎﾟｲﾝﾄ")
VIEWS = {"a": (-1.0, -1.0, -0.8), "b": (0.0, 0.0, -1.0), "c": (-1.0, 0.0, 0.0),
         "d": (0.0, -1.0, 0.0)}


def norm(v):
    m = math.sqrt(sum(c * c for c in v))
    return tuple(c / m for c in v)


def main() -> None:
    path = pathlib.Path(sys.argv[1])
    out_dir = path.parent / "png"
    out_dir.mkdir(exist_ok=True)
    app = win32com.client.GetActiveObject("DELMIA.Application")
    app.DisplayFileAlerts = False
    doc = app.Documents.Open(str(path))
    part = doc.Part
    sel = doc.Selection
    for b in range(1, part.HybridBodies.Count + 1):
        body = part.HybridBodies.Item(b)
        sel.Clear()
        hidden = 0
        for i in range(1, body.HybridShapes.Count + 1):
            shape = body.HybridShapes.Item(i)
            if str(shape.Name).startswith(HIDE_PREFIXES):
                sel.Add(shape)
                hidden += 1
        if hidden:
            sel.VisProperties.SetShow(1)
        sel.Clear()
    viewer = app.ActiveWindow.ActiveViewer
    vp = viewer.Viewpoint3D
    for tag, direction in VIEWS.items():
        vp.PutSightDirection(norm(direction))
        vp.PutUpDirection((0.0, 0.0, 1.0) if abs(direction[2]) < 0.9 else (0.0, 1.0, 0.0))
        viewer.Update()
        viewer.Reframe()
        target = out_dir / f"{path.stem}_{tag}.png"
        viewer.CaptureToFile(3, str(target))
        print(target, flush=True)
    doc.Close()


if __name__ == "__main__":
    main()

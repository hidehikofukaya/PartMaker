"""バッチ出力のCATPartを開き、構築ジオメトリ(点・直線)を隠して撮る。

失敗形状は構築途中で保存されているので「最後のフィーチャーだけ表示」は使えない
(プローブ点が最後に来ていることがある)。サーフェス類は全て表示し、点/直線だけ隠す
(SS6.15で確立したパターン)。
"""
import math
import pathlib
import sys

import win32com.client

HIDE_PREFIXES = ("点", "直線", "Point", "Line", "ﾎﾟｲﾝﾄ")


def norm(v):
    m = math.sqrt(sum(c * c for c in v))
    return tuple(c / m for c in v)


def render(app, path: pathlib.Path, out_dir: pathlib.Path) -> None:
    doc = app.Documents.Open(str(path))
    try:
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
        part.Update()
        viewer = app.ActiveWindow.ActiveViewer
        vp = viewer.Viewpoint3D
        vp.PutSightDirection(tuple(float(c) for c in norm((-1.0, -1.0, -0.8))))
        vp.PutUpDirection((0.0, 0.0, 1.0))
        viewer.Update()
        viewer.Reframe()
        target = out_dir / (path.stem + ".png")
        viewer.CaptureToFile(2, str(target))
        print(f"  {target.name}", flush=True)
    finally:
        doc.Close()


def main() -> None:
    target = pathlib.Path(sys.argv[1])
    root = target if target.is_dir() else target.parent
    out_dir = root / "png"
    out_dir.mkdir(exist_ok=True)
    app = win32com.client.GetActiveObject("DELMIA.Application")
    app.DisplayFileAlerts = False
    for i in range(app.Documents.Count, 0, -1):
        try: app.Documents.Item(i).Close()
        except Exception: pass
    paths = sorted(root.glob("*.CATPart")) if target.is_dir() else [target]
    for path in paths:
        try:
            render(app, path, out_dir)
        except Exception as exc:
            print(f"  {path.name}: FAILED {str(exc)[:80]}", flush=True)


if __name__ == "__main__":
    main()

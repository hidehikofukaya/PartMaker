"""複数のCATPartを1枚のモンタージュ画像にまとめる(品質の一括目視用、2026-08-25)。

個別スクリーンショットを何十枚も見るのは非効率なので、各部品を等角1視点で撮り、
3Dビューポート部分だけを切り出して格子に並べる。異常(刃・途切れ・両側フランジ・
スリバー)は縮小画像でも十分に判別できる。

使い方: python tools/render_montage.py <CATPartのディレクトリ> <出力png> [最大件数] [開始位置]
"""
import math
import pathlib
import sys

import win32com.client
from PIL import Image, ImageDraw

HIDE_PREFIXES = ("点", "直線", "Point", "Line", "ﾎﾟｲﾝﾄ")
# DELMIAウィンドウ内の3Dビューポート領域(ツリー・ツールバーを除いた範囲)
VIEWPORT = (505, 25, 1870, 975)
CELL = (340, 236)   # 1コマのサイズ(縮小後)
COLUMNS = 5
SIGHT = (-1.0, -1.0, -0.8)


def norm(v):
    m = math.sqrt(sum(c * c for c in v))
    return tuple(c / m for c in v)


def main() -> None:
    src_dir = pathlib.Path(sys.argv[1])
    out_path = pathlib.Path(sys.argv[2])
    limit = int(sys.argv[3]) if len(sys.argv) > 3 else 25
    offset = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    parts = sorted(src_dir.glob("*.CATPart"))[offset:offset + limit]
    if not parts:
        raise SystemExit(f"{src_dir} にCATPartが無い")
    shots_dir = out_path.parent / "_shots"
    shots_dir.mkdir(parents=True, exist_ok=True)

    app = win32com.client.GetActiveObject("DELMIA.Application")
    app.DisplayFileAlerts = False
    tiles = []
    for path in parts:
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
            viewer = app.ActiveWindow.ActiveViewer
            vp = viewer.Viewpoint3D
            vp.PutSightDirection(norm(SIGHT))
            vp.PutUpDirection((0.0, 0.0, 1.0))
            viewer.Update()
            viewer.Reframe()
            shot = shots_dir / (path.stem + ".png")
            viewer.CaptureToFile(3, str(shot))
            tiles.append((path.stem, shot))
            print(f"  撮影 {path.stem}", flush=True)
        finally:
            doc.Close()

    cols = min(COLUMNS, len(tiles))
    rows = math.ceil(len(tiles) / cols)
    sheet = Image.new("RGB", (cols * CELL[0], rows * CELL[1]), (30, 30, 30))
    draw = ImageDraw.Draw(sheet)
    for index, (name, shot) in enumerate(tiles):
        img = Image.open(shot).crop(VIEWPORT).resize(CELL, Image.LANCZOS)
        x = (index % cols) * CELL[0]
        y = (index // cols) * CELL[1]
        sheet.paste(img, (x, y))
        draw.rectangle([x, y, x + CELL[0] - 1, y + CELL[1] - 1], outline=(90, 90, 90))
        draw.text((x + 6, y + 4), name[-4:], fill=(255, 255, 120))
    sheet.save(out_path)
    print(f"\nモンタージュ {len(tiles)}件 -> {out_path}", flush=True)


if __name__ == "__main__":
    main()

"""Mode=4がFirstGuideSurfを消費するのは確認済み(probe_bead_sweep_verify.py)。

ここでの疑問: Mode=4は本当に基準面の"局所法線"に沿って追従してドラフトしているのか、
それとも根元曲線の始点1箇所だけで決めた固定方向にオフセットしているだけなのか
(後者なら、折れ目をまたいだ瞬間に壁が基準面から外れて浮く — まさに解決したい問題そのもの)。

長さを大きく誇張(200mm)してスイープすれば、固定方向オフセットなら折れ目の先(leg2)で
壁が基準面から明後日の方向にはみ出すはずで、局所法線追従なら折れ目をまたいでも
壁がbase surfaceに寄り添い続けるはず。この違いは目視で一目瞭然になるスケール。

構築用の点・線・平面・中間サーフェス(face1/face2/sharp接合)を非表示にしてから、
基準面+根元曲線+スイープ結果だけが見えるスクリーンショットを撮る。
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"


def build_filleted_base(builder, hsf, part, body, spa, doc, *, half_width=60.0, run=80.0, bend_deg=70.0, radius=30.0):
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

    edge_ref = builder.find_edge_near(doc, part, spa, hsf, body, sharp, (0.0, 0.0, 0.0))
    filleted = builder.edge_fillet(part, edge_ref, radius)
    body.AppendHybridShape(filleted)
    part.Update()
    return filleted, w, face1, face2, sharp


def hide(doc, *hybrid_shapes):
    sel = doc.Selection
    sel.Clear()
    for shape in hybrid_shapes:
        sel.Add(shape)
    sel.VisProperties.SetShow(1)
    sel.Clear()


def screenshot(catia, path):
    OUTPUT_DIR.mkdir(exist_ok=True)
    viewer = catia.ActiveWindow.ActiveViewer
    viewer.Update()  # Part.Update()直後にReframeすると再描画前の古いバウンディングで
    viewer.Reframe()  # フィットすることがあるため、明示的に再描画してからReframeする
    viewer.CaptureToFile(2, str(path))
    print(f"  screenshot: {path}", flush=True)


def main() -> None:
    builder = SyntheticPartBuilder()
    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "PROBE_MODE4_CLOSEUP"

    filleted, w, face1, face2, sharp = build_filleted_base(builder, hsf, part, body, spa, doc)
    surface_ref = part.CreateReferenceFromObject(filleted)

    origin_pt = builder.point(hsf, body, 0.0, 0.0, 0.0)
    axis_pt = builder.point(hsf, body, *w)
    axis_line = builder.line_pt_pt(hsf, part, body, origin_pt, axis_pt)
    plane = hsf.AddNewPlaneNormal(
        part.CreateReferenceFromObject(axis_line), part.CreateReferenceFromObject(origin_pt)
    )
    body.AppendHybridShape(plane)
    part.Update()

    centerline = hsf.AddNewIntersection(part.CreateReferenceFromObject(plane), surface_ref)
    body.AppendHybridShape(centerline)
    part.Update()

    root = hsf.AddNewCurvePar(part.CreateReferenceFromObject(centerline), surface_ref, 15.0, False, True)
    body.AppendHybridShape(root)
    part.Update()

    guide_ref = part.CreateReferenceFromObject(root)
    sweep = hsf.AddNewSweepLine(guide_ref)
    sweep.Mode = 4
    sweep.FirstGuideSurf = surface_ref
    sweep.SetAngle(1, 15.0)
    sweep.SetLength(1, 200.0)  # 誇張した長さ(実運用のビード深さ4-10mmよりずっと大きい、視覚検証専用)
    body.AppendHybridShape(sweep)
    part.Update()
    print("mode=4, length=200mm: Update OK", flush=True)

    # 中間の構築物(点・線・平面・シャープ結合前のface1/face2/sharp)を非表示にして
    # 基準面+根元曲線+スイープだけにする。
    hide(doc, origin_pt, axis_pt, axis_line, plane, face1, face2, sharp)
    screenshot(builder.catia, OUTPUT_DIR / "05_mode4_closeup_wide.png")

    # 誇張したスイープ(200mm)は基準面(~130mm)よりずっと大きいはずなのに前回のスクリーンショット
    # では見えなかった(Reframe()がUpdate直後の再描画前バウンディングでフィットした疑い)。
    # 基準面も一時的に隠してスイープだけにした画も撮る(確実にフィットさせるため)。
    hide(doc, filleted)
    screenshot(builder.catia, OUTPUT_DIR / "05b_mode4_sweep_only.png")
    sel = doc.Selection
    sel.Clear()
    sel.Add(filleted)
    sel.VisProperties.SetShow(0)  # 明示的に表示へ戻す(SetShow(1)の再呼び出しがトグルか直接指定か不明なため)
    sel.Clear()

    # 比較用: Mode=0(FirstGuideSurfを無視することが判明済み)を同じ長さで並べて作る
    root2_ref = part.CreateReferenceFromObject(root)
    sweep0 = hsf.AddNewSweepLine(root2_ref)
    sweep0.Mode = 0
    sweep0.FirstGuideSurf = surface_ref
    sweep0.SetAngle(1, 15.0)
    sweep0.SetLength(1, 200.0)
    body.AppendHybridShape(sweep0)
    part.Update()
    print("mode=0, length=200mm: Update OK (comparison)", flush=True)
    hide(doc, filleted)
    screenshot(builder.catia, OUTPUT_DIR / "06b_mode0_sweep_only_comparison.png")

    doc.Close()


if __name__ == "__main__":
    main()

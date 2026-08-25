"""AddNewSweepLineの「参照サーフェス+ドラフト角」サブモードを実機グリッドサーチで特定する。

公式ドキュメント調査(このセッションの前段)では`Mode`プロパティがサブモード選択の鍵だと
判明したが、列挙値の名前・数値はCATIAのCOM型情報にもドキュメントにも出てこなかった
(CATGSMIDLItf型ライブラリを`win32com.client.gencache.EnsureModule`でフル生成して確認済み、
名前付き定数は一切無い)。そのため実機で候補値を総当たりする。

`FirstGuideSurf`プロパティ(get/setとも`Reference`型)に基準サーフェスを渡せることは
pywin32のgen_py型情報から確定済み。`SetAngle(ii, iElem)`のiElemはVT_R8(生のdouble)、
`SetLength(ii, iElem)`も同様に生のdoubleと確定済み(いずれもオブジェクト参照ではない)。

各Modeの値について:
  1. 新規SweepLineをガイド曲線(ヒント①②で検証済みのbead根元曲線)から作る
  2. Mode = 候補値
  3. FirstGuideSurf = 基準サーフェス(既にRがついた基準面)
  4. SetAngle(1, ドラフト角)
  5. SetLength(1, 長さ)
  6. body.AppendHybridShape + Update
成功したものはスクリーンショットを撮る。全て失敗したら、次の一手(ロフト代替案)に
切り替える判断材料として、各失敗の例外メッセージを記録する。
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"


def build_filleted_base(builder, hsf, part, body, spa, doc, *, half_width=60.0, run=80.0, bend_deg=70.0, radius=30.0):
    """probe_bead_surface_curve.pyと同じ「既にRがついた基準面」を再現する。"""
    half_angle = math.radians(bend_deg) / 2.0
    n_unused = None  # noqa: F841 - 対称性の説明用、実コードでは未使用
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
    return filleted, w


def build_root_curve(builder, hsf, part, body, surface_obj, width_dir, offset_mm=15.0):
    """probe_bead_surface_curve.pyと同じ手順でビード根元曲線(基準面上、offset_mm片側)を作る。"""
    origin_pt = builder.point(hsf, body, 0.0, 0.0, 0.0)
    axis_pt = builder.point(hsf, body, *width_dir)
    axis_line = builder.line_pt_pt(hsf, part, body, origin_pt, axis_pt)
    plane = hsf.AddNewPlaneNormal(
        part.CreateReferenceFromObject(axis_line), part.CreateReferenceFromObject(origin_pt)
    )
    body.AppendHybridShape(plane)
    part.Update()

    surface_ref = part.CreateReferenceFromObject(surface_obj)
    centerline = hsf.AddNewIntersection(part.CreateReferenceFromObject(plane), surface_ref)
    body.AppendHybridShape(centerline)
    part.Update()

    root = hsf.AddNewCurvePar(part.CreateReferenceFromObject(centerline), surface_ref, offset_mm, False, True)
    body.AppendHybridShape(root)
    part.Update()
    return root


def screenshot(catia, path):
    OUTPUT_DIR.mkdir(exist_ok=True)
    viewer = catia.ActiveWindow.ActiveViewer
    viewer.Reframe()
    viewer.CaptureToFile(2, str(path))
    print(f"  screenshot: {path}", flush=True)


def try_mode(builder, mode_value, angle_deg, length_mm) -> bool:
    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = f"PROBE_SWEEP_mode{mode_value}"

    try:
        filleted, w = build_filleted_base(builder, hsf, part, body, spa, doc)
        root = build_root_curve(builder, hsf, part, body, filleted, w)
        surface_ref = part.CreateReferenceFromObject(filleted)
        guide_ref = part.CreateReferenceFromObject(root)

        sweep = hsf.AddNewSweepLine(guide_ref)
        sweep.Mode = mode_value
        sweep.FirstGuideSurf = surface_ref
        sweep.SetAngle(1, angle_deg)
        sweep.SetLength(1, length_mm)
        body.AppendHybridShape(sweep)
        part.Update()

        print(f"mode={mode_value}: OK", flush=True)
        screenshot(builder.catia, OUTPUT_DIR / f"04_sweep_mode{mode_value}_ok.png")
        doc.Close()
        return True
    except Exception as exc:  # noqa: BLE001 - グリッドサーチなので拾って次へ進む
        msg = str(exc)[:200].replace("\n", " ")
        print(f"mode={mode_value}: FAIL {msg}", flush=True)
        doc.Close()
        return False


def main() -> None:
    builder = SyntheticPartBuilder()
    any_ok = False
    for mode_value in range(0, 6):
        if try_mode(builder, mode_value, angle_deg=15.0, length_mm=10.0):
            any_ok = True
    print()
    print("ANY MODE SUCCEEDED" if any_ok else "ALL MODES FAILED")


if __name__ == "__main__":
    main()

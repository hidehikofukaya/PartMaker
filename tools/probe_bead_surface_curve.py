"""ヒント①②(交差曲線・平行曲線)が「フィレット済みの基準面」上で実際に機能するかの実機スパイク。

手順:
  1. 既存の実績ある方式(シャープ結合->AddNewSurfaceEdgeFilletWithConstantRadius)で、
     2枚のパネルが1本の折れ目で繋がった「既にRがついた基準面」を作る(ビードのテストベッド)。
  2. 幅方向中心(width=0)を通る平面と基準面の交線を AddNewIntersection で引く
     (ヒント①: 平面との交差)。折れ目をまたいでカーブが連続しているかを確認する。
  3. そのセンターライン曲線を AddNewCurvePar で基準面に沿って(Geodesic=True)左右に
     オフェットし、ビードの根元曲線(root curve)候補を得る(ヒント②: 平行曲線)。
  4. 各曲線の長さを GetMeasurable で実測し、幾何が壊れていないか(極端な値になっていないか)
     を数値でも確認する。
  5. 構築用の点・線・平面を非表示にしてスクリーンショットを撮り、目視レビュー用に保存する。

ユーザーはスマホから作業中でCATIA画面を直接見られないため、スクリーンショットは必須。
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"


def build_filleted_base(builder, hsf, part, body, spa, doc, *, half_width=60.0, run=80.0, bend_deg=70.0, radius=30.0):
    """2枚のパネルを1本の折れ目でシャープ結合し、エッジフィレットで丸めた「既にRがついた基準面」を作る。

    既存のbuild_general_two_point/_assemble_general_two_pointと同じ
    「シャープjoin -> 実エッジにedge_fillet」パターンをこのスパイク専用に最小構成で再現する。
    """
    import math

    half_angle = math.radians(bend_deg) / 2.0
    # panel1: run方向-run〜0、normalは(sin(half_angle), 0, cos(half_angle))寄りに傾ける代わりに、
    # 単純に「XZ平面内で折れ目がY軸(幅方向)に平行、原点で角度bend_degに折れる」形状にする。
    n1 = (math.sin(half_angle), 0.0, math.cos(half_angle))
    n2 = (-math.sin(half_angle), 0.0, math.cos(half_angle))
    u1 = (math.cos(half_angle), 0.0, -math.sin(half_angle))  # panel1の走行方向(折れ目に向かう)
    u2 = (math.cos(half_angle), 0.0, math.sin(half_angle))  # panel2の走行方向(折れ目から離れる)
    w = (0.0, 1.0, 0.0)  # 幅方向(共通)

    def corners(origin, u, run0, run1, hw):
        def pt(r, ww):
            return tuple(origin[i] + r * u[i] + ww * w[i] for i in range(3))
        return [pt(run0, -hw), pt(run0, hw), pt(run1, hw), pt(run1, -hw)]

    panel1_corners = corners((0.0, 0.0, 0.0), u1, run, 0.0, half_width)
    panel2_corners = corners((0.0, 0.0, 0.0), u2, 0.0, run, half_width)

    face1 = builder.rect_fill(hsf, part, body, panel1_corners)
    face2 = builder.rect_fill(hsf, part, body, panel2_corners)
    sharp = builder.join(hsf, part, body, [face1, face2])
    part.Update()

    fold_mid = (0.0, 0.0, 0.0)
    edge_ref = builder.find_edge_near(doc, part, spa, hsf, body, sharp, fold_mid)
    filleted = builder.edge_fillet(part, edge_ref, radius)
    body.AppendHybridShape(filleted)
    part.Update()
    print(f"base surface OK: bend={bend_deg}deg, R={radius}, half_width={half_width}, run={run}", flush=True)
    return filleted, u1, u2, w, half_width, run


def screenshot(catia, doc, path):
    OUTPUT_DIR.mkdir(exist_ok=True)
    viewer = catia.ActiveWindow.ActiveViewer
    viewer.Reframe()
    viewer.CaptureToFile(2, str(path))  # format 2 = PNG
    print(f"screenshot saved: {path}", flush=True)


def hide(doc, *hybrid_shapes):
    sel = doc.Selection
    sel.Clear()
    for shape in hybrid_shapes:
        sel.Add(shape)
    sel.VisProperties.SetShow(1)  # 1 = 非表示
    sel.Clear()


def main() -> None:
    builder = SyntheticPartBuilder()
    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "PROBE_CURVE_ON_SURFACE"

    filleted, u1, u2, w, half_width, run = build_filleted_base(builder, hsf, part, body, spa, doc)
    surface_ref = part.CreateReferenceFromObject(filleted)

    screenshot(builder.catia, doc, OUTPUT_DIR / "01_base_filleted_surface.png")

    # --- ヒント①: 幅方向中心(width=0)を通る平面と基準面の交線 ---
    origin_pt = builder.point(hsf, body, 0.0, 0.0, 0.0)
    axis_pt = builder.point(hsf, body, w[0], w[1], w[2])  # width_dir方向に1mm進んだ点
    axis_line = builder.line_pt_pt(hsf, part, body, origin_pt, axis_pt)
    center_plane = hsf.AddNewPlaneNormal(
        part.CreateReferenceFromObject(axis_line), part.CreateReferenceFromObject(origin_pt)
    )
    body.AppendHybridShape(center_plane)
    part.Update()
    print("center plane OK", flush=True)

    centerline = hsf.AddNewIntersection(part.CreateReferenceFromObject(center_plane), surface_ref)
    body.AppendHybridShape(centerline)
    try:
        part.Update()
        # GetMeasurable().Lengthはドキュメントの線形単位(mm)で直接返る(既知80mm辺で較正済み、
        # Areaのm^2とは異なりmm->m変換は不要)。
        length = spa.GetMeasurable(part.CreateReferenceFromObject(centerline)).Length
        print(f"centerline intersection OK: length={length:.2f}mm (expect somewhat less than {run * 2:.1f}mm, the flat run sum, since the fillet arc is shorter than the cut corner)", flush=True)
    except Exception as exc:  # noqa: BLE001 - スパイクなので拾って報告する
        print(f"centerline intersection FAILED: {exc}", flush=True)
        screenshot(builder.catia, doc, OUTPUT_DIR / "02_intersection_failed.png")
        doc.Close()
        return

    hide(doc, origin_pt, axis_pt, axis_line, center_plane)
    screenshot(builder.catia, doc, OUTPUT_DIR / "02_centerline_on_filleted_surface.png")

    # --- ヒント②: センターラインを基準面に沿って(Geodesic)左右にオフセット ---
    centerline_ref = part.CreateReferenceFromObject(centerline)
    offset_mm = 15.0
    left = hsf.AddNewCurvePar(centerline_ref, surface_ref, offset_mm, False, True)
    body.AppendHybridShape(left)
    try:
        part.Update()
        left_len = spa.GetMeasurable(part.CreateReferenceFromObject(left)).Length
        print(f"left parallel curve OK: length={left_len:.2f}mm", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"left parallel curve FAILED: {exc}", flush=True)
        screenshot(builder.catia, doc, OUTPUT_DIR / "03_curvepar_left_failed.png")
        doc.Close()
        return

    right = hsf.AddNewCurvePar(centerline_ref, surface_ref, offset_mm, True, True)
    body.AppendHybridShape(right)
    try:
        part.Update()
        right_len = spa.GetMeasurable(part.CreateReferenceFromObject(right)).Length
        print(f"right parallel curve OK: length={right_len:.2f}mm", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"right parallel curve FAILED: {exc}", flush=True)
        screenshot(builder.catia, doc, OUTPUT_DIR / "03_curvepar_right_failed.png")
        doc.Close()
        return

    screenshot(builder.catia, doc, OUTPUT_DIR / "03_root_curves_on_filleted_surface.png")
    print("ALL OK", flush=True)
    doc.Close()


if __name__ == "__main__":
    main()

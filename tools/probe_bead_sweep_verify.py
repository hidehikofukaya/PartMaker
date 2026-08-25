"""probe_bead_sweep_draft.pyで「Update成功」した4つのMode値(0,1,2,4)のうち、実際に
FirstGuideSurf(基準サーフェス)を消費しているのはどれかを数値で切り分ける。

前回のスクリーンショットは基準パネル(~130mm)に対してスイープ幅(10mm)が小さすぎて
目視では判別できなかった(このプロジェクトの既存の教訓 — 小スケール形状はスクリーンショット
より実測値で検証する、roadmap SS6.17参照)。

方法: 同じMode・同じ長さ・同じ角度で、(a) FirstGuideSurfを設定した場合 と
(b) FirstGuideSurfを設定しない場合 の面積(GetMeasurable.Area)を比較する。
面積が一致するなら、そのModeはFirstGuideSurfを無視している(=期待した「基準面+ドラフト角」
動作ではない)。異なるなら、そのModeは基準サーフェスを実際に使っている可能性が高い候補。

さらに、本命候補が見つかったら根元エッジが基準面上の曲線と厳密に一致するかを
GetMinimumDistanceで実測し、「浮き」が無いことを直接確認する。
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
    return filleted, w


def build_root_curve(builder, hsf, part, body, surface_obj, width_dir, offset_mm=15.0):
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


def measure_area_mm2(spa, surface_ref) -> float:
    # roadmap既出の教訓: GetMeasurable().AreaはM^2で返る(Lengthはmmで直接返るのと違う)。
    # 今回のセッションで一度Lengthを誤って*1000したので、ここは明示的に区別してコメントする。
    return spa.GetMeasurable(surface_ref).Area * 1e6


def try_variant(builder, mode_value, *, use_reference_surface: bool, angle_deg=15.0, length_mm=10.0):
    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = f"PROBE_VERIFY_mode{mode_value}_{'ref' if use_reference_surface else 'noref'}"

    try:
        filleted, w = build_filleted_base(builder, hsf, part, body, spa, doc)
        root = build_root_curve(builder, hsf, part, body, filleted, w)
        surface_ref = part.CreateReferenceFromObject(filleted)
        guide_ref = part.CreateReferenceFromObject(root)

        sweep = hsf.AddNewSweepLine(guide_ref)
        sweep.Mode = mode_value
        if use_reference_surface:
            sweep.FirstGuideSurf = surface_ref
        sweep.SetAngle(1, angle_deg)
        sweep.SetLength(1, length_mm)
        body.AppendHybridShape(sweep)
        part.Update()

        sweep_ref = part.CreateReferenceFromObject(sweep)
        area = measure_area_mm2(spa, sweep_ref)
        doc.Close()
        return area
    except Exception as exc:  # noqa: BLE001
        doc.Close()
        return None, str(exc)[:150].replace("\n", " ")


def main() -> None:
    builder = SyntheticPartBuilder()
    working_modes = [0, 1, 2, 4]

    print("--- 面積比較: FirstGuideSurfあり vs なし ---", flush=True)
    for mode_value in working_modes:
        area_with = try_variant(builder, mode_value, use_reference_surface=True)
        area_without = try_variant(builder, mode_value, use_reference_surface=False)

        def fmt(a):
            if isinstance(a, tuple):
                return f"FAIL({a[1]})"
            return f"{a:.4f}mm^2"

        same = (
            isinstance(area_with, float)
            and isinstance(area_without, float)
            and abs(area_with - area_without) < 1e-3
        )
        verdict = "IGNORES FirstGuideSurf (same result)" if same else "differs -- candidate"
        print(f"mode={mode_value}: with_ref={fmt(area_with)} without_ref={fmt(area_without)} -> {verdict}", flush=True)


if __name__ == "__main__":
    main()

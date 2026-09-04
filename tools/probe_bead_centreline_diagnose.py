"""Join失敗の原因診断: 基準面だけ(ビード無し)を構築し、3セグメントの中心線を
個別に調べる(2026-08-24)。fold1=37deg/fold2=80deg、a1=30deg/a2=32degという
tools/probe_free_fold_centreline.pyより急な折れ目角度のケースでJOIN後のUpdate()が
失敗した(build_general_two_point経由の実測)。
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.classify import (  # noqa: E402
    FasteningPoint,
    free_fold_seed,
    solve_free_fold,
)
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"

POINT1 = FasteningPoint((0.0, 0.0, 0.0), (0.7800312387928039, 0.21211178193677718, 0.5886933484174666))
POINT2 = FasteningPoint(
    (24.30359962098499, -68.62918910755124, 144.9674165750089),
    (0.19870462012118822, -0.3579774571251853, -0.9123423776919934),
)
BEND_RADIUS_MM = 11.56030141400419
HALF_WIDTH_MM = 25.0
MIN_BEARING_RADIUS_MM = 23.473464455183723
FOLD1_SLACK_MM = 49.315062550877016
FOLD2_SLACK_MM = 43.108138747799245
FOLD1_TILT_PERTURBATION_RAD = -0.021562977200378728


def screenshot(catia, path):
    OUTPUT_DIR.mkdir(exist_ok=True)
    viewer = catia.ActiveWindow.ActiveViewer
    viewer.Update()
    viewer.Reframe()
    viewer.CaptureToFile(2, str(path))
    print(f"  screenshot: {path}", flush=True)


def main() -> None:
    seed = free_fold_seed(
        POINT1, POINT2, bend_radius_mm=BEND_RADIUS_MM, min_bearing_radius_mm=MIN_BEARING_RADIUS_MM,
        fold1_slack_mm=FOLD1_SLACK_MM, fold2_slack_mm=FOLD2_SLACK_MM,
    )
    chain = solve_free_fold(seed, POINT1, POINT2, target_a1_rad=seed.a1_rad + FOLD1_TILT_PERTURBATION_RAD)

    builder = SyntheticPartBuilder()
    print("building base surface (no bead) via build_general_two_point ...", flush=True)
    generated = builder.build_general_two_point(
        POINT1, POINT2,
        min_bearing_radius_mm=MIN_BEARING_RADIUS_MM,
        half_width_mm=HALF_WIDTH_MM,
        bend_radius_mm=BEND_RADIUS_MM,
        fold1_slack_mm=FOLD1_SLACK_MM,
        fold2_slack_mm=FOLD2_SLACK_MM,
        fold1_tilt_perturbation_rad=FOLD1_TILT_PERTURBATION_RAD,
        out_dir=str(OUTPUT_DIR),
        part_name="bead_diag_base",
    )
    print(f"base surface OK: {generated.catpart_path}", flush=True)

    doc = builder.catia.Documents.Open(generated.catpart_path)
    part = doc.Part
    hsf = part.HybridShapeFactory
    body = part.HybridBodies.Item(1)
    surface = None
    for i in range(1, body.HybridShapes.Count + 1):
        surface = body.HybridShapes.Item(i)
    surface_ref = part.CreateReferenceFromObject(surface)
    print(f"using surface: {surface.Name}", flush=True)

    panel_frames = [
        (chain.panel1.origin, chain.panel1.u, chain.panel1.v, "panel1"),
        (chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v, "panel_mid"),
        (chain.panel3.origin, chain.panel3.u, chain.panel3.v, "panel3"),
    ]

    segments = []
    for origin, u, v, label in panel_frames:
        origin_pt = builder.point(hsf, body, *origin)
        v_tip = builder.point(hsf, body, *tuple(origin[i] + v[i] for i in range(3)))
        part.Update()
        v_axis = builder.line_pt_pt(hsf, part, body, origin_pt, v_tip)
        part.Update()
        plane = hsf.AddNewPlaneNormal(part.CreateReferenceFromObject(v_axis), part.CreateReferenceFromObject(origin_pt))
        body.AppendHybridShape(plane)
        part.Update()
        segment = hsf.AddNewIntersection(part.CreateReferenceFromObject(plane), surface_ref)
        segment.Name = f"seg_{label}"
        body.AppendHybridShape(segment)
        try:
            part.Update()
            print(f"  {label}: intersection built OK", flush=True)
        except Exception as exc:
            print(f"  {label}: intersection Update FAILED: {exc}", flush=True)
        segments.append(segment)

    screenshot(builder.catia, OUTPUT_DIR / "bead_diag_segments.png")

    # Join を2個ずつ試して、どのペアで壊れるか切り分ける
    for i in range(len(segments) - 1):
        try:
            joined = builder.join(hsf, part, body, [segments[i], segments[i + 1]])
            joined.Name = f"join_{i}_{i+1}"
            part.Update()
            print(f"  join(seg{i},seg{i+1}) OK", flush=True)
        except Exception as exc:
            print(f"  join(seg{i},seg{i+1}) FAILED: {exc}", flush=True)

    doc.Save()
    print("done", flush=True)


if __name__ == "__main__":
    main()

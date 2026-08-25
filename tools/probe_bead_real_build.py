"""SS8.6のビード中心線(複数平面交線+Join)を、build_general_two_pointの本番経路
(bead付き)で実際にCATIA構築できるかを検証する(2026-08-24)。

tools/probe_free_fold_centreline.pyで中心線構築とAddNewCurvePar自体は検証済み。
ここでは_add_bead_to_surfaceの③〜⑦(根元曲線・壁4枚・BiTangent×3・頂稜線R・足元R)
まで含めた完全なビード構築が、自由折れ目チェーンの傾いたパネル上で通るかを見る。
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import BeadParams  # noqa: E402
from synthetic_generator.classify import FasteningPoint  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"

POINT1 = FasteningPoint(
    position_xyz=(0.0, 0.0, 0.0),
    normal_xyz=(0.7800312387928039, 0.21211178193677718, 0.5886933484174666),
)
POINT2 = FasteningPoint(
    position_xyz=(24.30359962098499, -68.62918910755124, 144.9674165750089),
    normal_xyz=(0.19870462012118822, -0.3579774571251853, -0.9123423776919934),
)
BEND_RADIUS_MM = 11.56030141400419
HALF_WIDTH_MM = 25.0
MIN_BEARING_RADIUS_MM = 23.473464455183723
FOLD1_SLACK_MM = 49.315062550877016
FOLD2_SLACK_MM = 43.108138747799245
FOLD1_TILT_PERTURBATION_RAD = -0.021562977200378728
BEAD = BeadParams(
    depth_mm=9.109614587119138,
    top_width_mm=13.733648251153701,
    wall_angle_deg=58.42740771706355,
    ridge_radius_mm=8.5101904054768,
    corner_radius_mm=9.05354681994448,
)


def screenshot(catia, path):
    OUTPUT_DIR.mkdir(exist_ok=True)
    viewer = catia.ActiveWindow.ActiveViewer
    viewer.Update()
    viewer.Reframe()
    viewer.CaptureToFile(2, str(path))
    print(f"  screenshot: {path}", flush=True)


def main() -> None:
    builder = SyntheticPartBuilder()
    print("building via build_general_two_point (with bead) ...", flush=True)
    generated = builder.build_general_two_point(
        POINT1,
        POINT2,
        min_bearing_radius_mm=MIN_BEARING_RADIUS_MM,
        half_width_mm=HALF_WIDTH_MM,
        bend_radius_mm=BEND_RADIUS_MM,
        fold1_slack_mm=FOLD1_SLACK_MM,
        fold2_slack_mm=FOLD2_SLACK_MM,
        fold1_tilt_perturbation_rad=FOLD1_TILT_PERTURBATION_RAD,
        out_dir=str(OUTPUT_DIR),
        part_name="free_fold_chain_bead_01",
        bead=BEAD,
    )
    print(f"OK: stp={generated.stp_path}", flush=True)
    print(f"    catpart={generated.catpart_path}", flush=True)

    catia = builder.catia
    catia.Documents.Open(generated.catpart_path)
    screenshot(catia, OUTPUT_DIR / "free_fold_chain_bead_01_iso.png")


if __name__ == "__main__":
    main()

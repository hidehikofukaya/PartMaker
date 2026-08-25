"""SS8の自由折れ目チェーン(classify.free_fold_seed/solve_free_fold)を、実際に
CATIAでbuild_general_two_pointに通して構築できるかを検証する(2026-08-24)。

傾いた折れ目(a1!=0)により各パネルが矩形ではなく台形(sheared_panel_corners)になる —
これはrect_fillに新規に要求される形になるため、実機で通るかどうかがこの検証の主眼。
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.classify import FasteningPoint  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"

# tools/probe_output配下のスクリプト実行で選んだケース(index=9, sample(random.Random(2026))の
# 825件の自由折れ目チェーン成功例のうち、曲げ角<100度・L2>30mmの条件で最初に見つかったもの)。
POINT1 = FasteningPoint(
    position_xyz=(0.0, 0.0, 0.0),
    normal_xyz=(0.7091245539987692, -0.4192780250104878, 0.566875916457342),
)
POINT2 = FasteningPoint(
    position_xyz=(105.16387734930477, 9.362453294474978, -80.55987327589155),
    normal_xyz=(0.8798637674773494, -0.4223685455820081, 0.21781772743168493),
)
BEND_RADIUS_MM = 11.081175192980025
HALF_WIDTH_MM = 14.51036472028607
MIN_BEARING_RADIUS_MM = 13.142192161995899
FOLD1_SLACK_MM = 46.17202428508006
FOLD2_SLACK_MM = 1.5474142248551326
FOLD1_TILT_PERTURBATION_RAD = 0.4155136573387424


def screenshot(catia, path):
    OUTPUT_DIR.mkdir(exist_ok=True)
    viewer = catia.ActiveWindow.ActiveViewer
    viewer.Update()
    viewer.Reframe()
    viewer.CaptureToFile(2, str(path))
    print(f"  screenshot: {path}", flush=True)


def main() -> None:
    builder = SyntheticPartBuilder()
    print("building via build_general_two_point ...", flush=True)
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
        part_name="free_fold_chain_real_01",
    )
    print(f"OK: stp={generated.stp_path}", flush=True)
    print(f"    catpart={generated.catpart_path}", flush=True)

    catia = builder.catia
    catia.Documents.Open(generated.catpart_path)
    screenshot(catia, OUTPUT_DIR / "free_fold_chain_real_01_iso.png")


if __name__ == "__main__":
    main()

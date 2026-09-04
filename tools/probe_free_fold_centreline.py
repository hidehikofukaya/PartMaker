"""自由折れ目チェーンの基準面上に、ビード中心線をどう置くか(SS8.6)の実機検証。

旧w平行版は「幅方向に垂直な単一平面と基準面の交線」で、曲げフィレット領域も含めて
確実に面へ沿わせていた。新しいチェーンでは3パネルの走行方向・法線がそれぞれ異なり
(w1,w_mid,w2が別々の平面をなす)、単一平面では成立しない(実測: p1,fold1,fold2の
作る平面からp2までの距離0.75mm、厳密な共面ではない)。

ここでは「パネルごとに自分のv(=n×u)を法線とする平面と基準面の交線を取り、3本を
Joinする」方式が実機で通るか(単一の連続曲線になるか、AddNewCurvePar等の下流操作の
入力として使えるか)を、既存の実ビルド済みパーツ(free_fold_chain_real_01_mid.CATPart)
に対して検証する。
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
PART_PATH = OUTPUT_DIR / "free_fold_chain_real_01_mid.CATPart"

POINT1 = FasteningPoint((0.0, 0.0, 0.0), (0.7091245539987692, -0.4192780250104878, 0.566875916457342))
POINT2 = FasteningPoint(
    (105.16387734930477, 9.362453294474978, -80.55987327589155),
    (0.8798637674773494, -0.4223685455820081, 0.21781772743168493),
)


def main() -> None:
    seed = free_fold_seed(
        POINT1, POINT2, bend_radius_mm=11.081175192980025, min_bearing_radius_mm=13.142192161995899,
        fold1_slack_mm=46.17202428508006, fold2_slack_mm=1.5474142248551326,
    )
    chain = solve_free_fold(seed, POINT1, POINT2, target_a1_rad=seed.a1_rad + 0.4155136573387424)

    builder = SyntheticPartBuilder()
    doc = builder.catia.Documents.Open(str(PART_PATH))
    part = doc.Part
    hsf = part.HybridShapeFactory
    body = part.HybridBodies.Item(1)
    # 既存の基準面サーフェス(唯一のハイブリッドシェイプ)を参照する。
    surface = None
    for i in range(1, body.HybridShapes.Count + 1):
        surface = body.HybridShapes.Item(i)
    surface_ref = part.CreateReferenceFromObject(surface)
    print(f"using surface: {surface.Name}", flush=True)

    def make_plane_intersection(origin, u, v, label):
        p0 = builder.point(hsf, body, *origin)
        p1 = builder.point(hsf, body, *tuple(origin[i] + u[i] for i in range(3)))
        axis_line = builder.line_pt_pt(hsf, part, body, p0, p1)
        part.Update()
        # 平面法線=v(そのパネルの幅方向)、原点を通る。
        n_tip = builder.point(hsf, body, *tuple(origin[i] + v[i] for i in range(3)))
        axis2 = builder.line_pt_pt(hsf, part, body, p0, n_tip)
        part.Update()
        plane = hsf.AddNewPlaneNormal(part.CreateReferenceFromObject(axis2), part.CreateReferenceFromObject(p0))
        body.AppendHybridShape(plane)
        part.Update()
        curve = hsf.AddNewIntersection(part.CreateReferenceFromObject(plane), surface_ref)
        curve.Name = label
        body.AppendHybridShape(curve)
        part.Update()
        return curve

    seg1 = make_plane_intersection(chain.panel1.origin, chain.panel1.u, chain.panel1.v, "seg1_panel1")
    seg2 = make_plane_intersection(chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v, "seg2_mid")
    seg3 = make_plane_intersection(chain.panel3.origin, chain.panel3.u, chain.panel3.v, "seg3_panel3")
    print("built 3 plane-intersection curves OK", flush=True)


    # Joinを試す(surfaceのjoinヘルパーはcurveでも動くはず、同じAddNewJoin API)。
    try:
        joined = builder.join(hsf, part, body, [seg1, seg2, seg3])
        joined.Name = "centreline_joined"
        part.Update()
        print("JOIN OK", flush=True)
    except Exception as exc:
        print(f"JOIN FAILED: {exc}", flush=True)
        doc.Save()
        return

    # 本番コードが実際に次に行う操作: 中心線から半フットプリント分オフセットした
    # 「根元曲線」(壁の下端)をAddNewCurveParで作る(2つのreverse方向、gsd_build.py
    # _add_bead_to_surface手順③と同じ)。joinしたintersectionカーブが本当に
    # AddNewCurveParの入力として使えるかがここでの本質的な検証。
    joined_ref = part.CreateReferenceFromObject(joined)
    half_footprint_mm = 10.0
    for reverse in (False, True):
        try:
            root = hsf.AddNewCurvePar(joined_ref, surface_ref, half_footprint_mm, reverse, True)
            root.Name = f"root_{'right' if reverse else 'left'}"
            body.AppendHybridShape(root)
            part.Update()
            print(f"AddNewCurvePar(reverse={reverse}) OK", flush=True)
        except Exception as exc:
            print(f"AddNewCurvePar(reverse={reverse}) FAILED: {exc}", flush=True)

    doc.Save()
    viewer = builder.catia.ActiveWindow.ActiveViewer
    viewer.Update()
    viewer.Reframe()
    viewer.CaptureToFile(2, str(OUTPUT_DIR / "free_fold_centreline_probe.png"))
    print("screenshot saved", flush=True)


if __name__ == "__main__":
    main()

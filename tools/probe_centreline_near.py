"""中心線Join失敗の根本原因を切り分け、AddNewNearによる修正を検証する(2026-08-24)。

仮説(SS8.9): 各パネルのv法線平面は**無限に広がる**ため、基準面と交差させると
意図したパネル上の1本だけでなく、他のパネルやフィレット領域にも枝ができ、
1つのIntersectionオブジェクトが複数の非連結成分を含んでしまう。
そのためJoinのUpdate()が失敗し、GetMeasurable().Lengthも失敗する。

CATIA標準の対処は`AddNewNear(多枝要素, 参照要素)`で、参照点に最も近い枝だけを
取り出すもの。コードベース全体でこのAPIは未使用だったので、ここで検証する。

検証手順(必ず全ドキュメントを閉じたクリーンな状態から。SS8.9の教訓):
  1. 基準面を構築(3枚の傾いたパネル + 曲げフィレット2箇所)
  2. パネルごとにv法線平面 x 基準面のIntersectionを作る
  3. 各Intersectionが多枝かどうかをGetMeasurable().Lengthの成否で判定
  4. AddNewNearで単一枝を取り出し、再度Lengthを測る
  5. Near結果3本をJoinし、さらにAddNewCurveParまで通るか確認
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.classify import (  # noqa: E402
    FasteningPoint,
    free_fold_seed,
    sheared_panel_corners,
    solve_free_fold,
)
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"

POINT1 = FasteningPoint((0.0, 0.0, 0.0), (0.7091245539987692, -0.4192780250104878, 0.566875916457342))
POINT2 = FasteningPoint(
    (105.16387734930477, 9.362453294474978, -80.55987327589155),
    (0.8798637674773494, -0.4223685455820081, 0.21781772743168493),
)
BEND_RADIUS_MM = 11.081175192980025
HALF_WIDTH_MM = 14.51036472028607
MIN_BEARING_RADIUS_MM = 13.142192161995899
FOLD1_SLACK_MM = 46.17202428508006
FOLD2_SLACK_MM = 1.5474142248551326
FOLD1_TILT_PERTURBATION_DEG = 0.4155136573387424


def try_length(spa, part, obj, label):
    """GetMeasurable().Length が通るか = 単一の素直な曲線として扱えるか。"""
    try:
        length = spa.GetMeasurable(part.CreateReferenceFromObject(obj)).Length
        print(f"    {label}: Length = {length:.2f}mm  (単一枝として扱える)", flush=True)
        return length
    except Exception as exc:
        print(f"    {label}: Length FAILED -> 多枝の疑い ({str(exc)[:60]})", flush=True)
        return None


def main() -> None:
    builder = SyntheticPartBuilder()

    # SS8.9の教訓: 残留状態による偽陽性を避けるため、必ず全ドキュメントを閉じてから始める。
    count = builder.catia.Documents.Count
    for i in range(count, 0, -1):
        builder.catia.Documents.Item(i).Close()
    print(f"closed {count} pre-existing document(s)", flush=True)

    seed = free_fold_seed(
        POINT1, POINT2, bend_radius_mm=BEND_RADIUS_MM,
        min_bearing_radius_mm=MIN_BEARING_RADIUS_MM,
        fold1_slack_mm=FOLD1_SLACK_MM, fold2_slack_mm=FOLD2_SLACK_MM,
    )
    chain = solve_free_fold(
        seed, POINT1, POINT2,
        target_a1_rad=seed.a1_rad + math.radians(FOLD1_TILT_PERTURBATION_DEG),
    )
    print(
        f"chain: fold1={math.degrees(chain.fold1_angle_rad):.1f}deg "
        f"fold2={math.degrees(chain.fold2_angle_rad):.1f}deg "
        f"a1={math.degrees(chain.a1_rad):.1f}deg a2={math.degrees(chain.a2_rad):.1f}deg",
        flush=True,
    )

    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "NEAR_PROBE"

    margin = MIN_BEARING_RADIUS_MM
    corner_sets = [
        sheared_panel_corners(
            chain.panel1.origin, chain.panel1.u, chain.panel1.v,
            -margin, chain.L1_mm, HALF_WIDTH_MM, near_tilt_rad=0.0, far_tilt_rad=chain.a1_rad,
        ),
        sheared_panel_corners(
            chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v,
            0.0, chain.L2_mm, HALF_WIDTH_MM, near_tilt_rad=chain.a1_rad, far_tilt_rad=chain.a2_rad,
        ),
        sheared_panel_corners(
            chain.panel3.origin, chain.panel3.u, chain.panel3.v,
            0.0, chain.L3_mm + margin, HALF_WIDTH_MM, near_tilt_rad=chain.a2_rad, far_tilt_rad=0.0,
        ),
    ]
    faces = [builder.rect_fill(hsf, part, body, c) for c in corner_sets]
    whole = builder.join(hsf, part, body, faces)
    part.Update()
    print("base surface joined OK", flush=True)

    mid = corner_sets[1]
    for fold_mid in (
        tuple((mid[0][i] + mid[1][i]) / 2 for i in range(3)),
        tuple((mid[2][i] + mid[3][i]) / 2 for i in range(3)),
    ):
        edge_ref = builder.find_edge_near(doc, part, spa, hsf, body, whole, fold_mid)
        whole = builder.edge_fillet_group(part, [edge_ref], BEND_RADIUS_MM)
        body.AppendHybridShape(whole)
        part.Update()
    print("bend fillets OK", flush=True)
    surface_ref = part.CreateReferenceFromObject(whole)

    # 各パネルの「平坦区間の中点」を Near の参照点にする(フィレット領域を避ける)。
    frames = [
        (chain.panel1.origin, chain.panel1.u, chain.panel1.v, -margin, chain.L1_mm, "panel1"),
        (chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v, 0.0, chain.L2_mm, "panel_mid"),
        (chain.panel3.origin, chain.panel3.u, chain.panel3.v, 0.0, chain.L3_mm + margin, "panel3"),
    ]

    raw_segments = []
    near_segments = []
    for origin, u, v, near_run, far_run, label in frames:
        origin_pt = builder.point(hsf, body, *origin)
        v_tip = builder.point(hsf, body, *tuple(origin[i] + v[i] for i in range(3)))
        part.Update()
        v_axis = builder.line_pt_pt(hsf, part, body, origin_pt, v_tip)
        part.Update()
        plane = hsf.AddNewPlaneNormal(
            part.CreateReferenceFromObject(v_axis), part.CreateReferenceFromObject(origin_pt)
        )
        body.AppendHybridShape(plane)
        part.Update()
        segment = hsf.AddNewIntersection(part.CreateReferenceFromObject(plane), surface_ref)
        segment.Name = f"raw_{label}"
        body.AppendHybridShape(segment)
        part.Update()
        print(f"  {label}: intersection built OK", flush=True)
        try_length(spa, part, segment, f"raw_{label}")
        raw_segments.append(segment)

        # AddNewNear: 参照点に最も近い枝だけを取り出す
        mid_run = (near_run + far_run) / 2.0
        ref_xyz = tuple(origin[i] + mid_run * u[i] for i in range(3))
        ref_pt = builder.point(hsf, body, *ref_xyz)
        part.Update()
        try:
            near = hsf.AddNewNear(
                part.CreateReferenceFromObject(segment), part.CreateReferenceFromObject(ref_pt)
            )
            near.Name = f"near_{label}"
            body.AppendHybridShape(near)
            part.Update()
            print(f"  {label}: AddNewNear OK", flush=True)
            try_length(spa, part, near, f"near_{label}")
            near_segments.append(near)
        except Exception as exc:
            print(f"  {label}: AddNewNear FAILED: {str(exc)[:80]}", flush=True)

    if len(near_segments) == len(frames):
        try:
            joined = builder.join(hsf, part, body, near_segments)
            joined.Name = "centreline_near_joined"
            part.Update()
            print("JOIN of Near-extracted segments OK", flush=True)
            try_length(spa, part, joined, "joined")
            joined_ref = part.CreateReferenceFromObject(joined)
            for reverse in (False, True):
                root = hsf.AddNewCurvePar(joined_ref, surface_ref, 10.0, reverse, True)
                root.Name = f"root_{'right' if reverse else 'left'}"
                body.AppendHybridShape(root)
                part.Update()
                print(f"  AddNewCurvePar(reverse={reverse}) OK", flush=True)
        except Exception as exc:
            print(f"JOIN/CurvePar of Near segments FAILED: {str(exc)[:100]}", flush=True)

    OUTPUT_DIR.mkdir(exist_ok=True)
    doc.SaveAs(str(OUTPUT_DIR / "centreline_near_probe.CATPart"))
    viewer = builder.catia.ActiveWindow.ActiveViewer
    viewer.Update()
    viewer.Reframe()
    viewer.CaptureToFile(2, str(OUTPUT_DIR / "centreline_near_probe.png"))
    print("saved + screenshot", flush=True)


if __name__ == "__main__":
    main()

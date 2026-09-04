"""ビード(曲げをまたぐ)のCATIA実機スパイク。roadmap SS6.25 Step 2。

確かめること:
  A. `Fill`が捻れた4辺輪郭(ランプの壁セル)を受けられるか、15セルが単一シェルにjoinされるか
  B. ビード縦4本 x 3パネル と、ビードで5分割されたメイン折れ目のフィレットが通るか
     (縦横のフィレット列が交差する — SS6.15の一段上のリスク)

使い方: python tools/probe_bead.py [--lateral 35] [--no-fillet]
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import BeadParams, bead_cells  # noqa: E402
from synthetic_generator.classify import (  # noqa: E402
    FasteningPoint,
    end_panel_corners,
    two_point_frame,
)
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUT_DIR = pathlib.Path(r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_parts\_bead_probe")


def midpoint(a, b):
    return tuple((a[i] + b[i]) / 2 for i in range(3))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lateral", type=float, default=35.0, help="横方向オフセット(壁の捻れを含めるか)")
    ap.add_argument("--no-fillet", action="store_true", help="StageAだけ(joinとexportのみ)")
    ap.add_argument("--order", choices=["fold-first", "bead-first"], default="fold-first")
    ap.add_argument("--fold-radius", type=float, default=8.0, help="メイン折れ目のR")
    ap.add_argument("--bead-edge-order", default="0123", help="ビード縦エッジを当てる順(ストリップ番号の並び)")
    ap.add_argument("--propagation", type=int, default=0, help="edge_filletの伝播モード(0=なし,1=接線伝播)")
    ap.add_argument("--name", default="BEAD_PROBE")
    args = ap.parse_args()

    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(120.0, args.lateral, 60.0), normal_xyz=(1.0, 0.0, 1.0))
    frame = two_point_frame(p1, p2)
    flat1 = end_panel_corners(p1.position_xyz, frame.u1, frame.w, -20.0, 45.0, 40.0)
    flat2 = end_panel_corners(p2.position_xyz, frame.u2, frame.w, -45.0, 20.0, 40.0)
    ramp = [flat1[3], flat1[2], flat2[1], flat2[0]]
    panels = [flat1, ramp, flat2]

    bead = BeadParams(depth_mm=6.0, top_width_mm=30.0, wall_angle_deg=45.0, bend_radius_mm=4.0)
    cells = bead_cells(panels, bead)
    print(f"cells={len(cells)} lateral={args.lateral}mm")

    builder = SyntheticPartBuilder()
    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = f"{args.name}_synthetic"

    # --- Stage A: Fill x 15 -> join -> Update
    faces = []
    for i, cell in enumerate(cells):
        faces.append(builder.rect_fill(hsf, part, body, cell))
        print(f"  fill {i:02d} ok", flush=True)
    whole = builder.join(hsf, part, body, faces)
    part.Update()
    print("Stage A ok: 15 cells joined")

    if not args.no_fillet:
        # --- Stage B: ビード縦4本(パネルごと) -> メイン折れ目(ストリップごとに5分割)
        strip_order = [int(c) for c in args.bead_edge_order]
        bead_edges = [
            (f"bead p{panel_index} edge{strip}", midpoint(*cells[panel_index * 5 + strip][1:3]))
            for panel_index in range(len(panels))
            for strip in strip_order  # セルkとセルk+1の間の縦エッジ
        ]
        fold_edges = [
            (f"fold{fold_index} strip{strip}", midpoint(*cells[fold_index * 5 + strip][2:4]))
            for fold_index in range(len(panels) - 1)
            for strip in range(5)
        ]
        # SS6.15の教訓: 大きい(構造の)フィレットを先、小さいフィレットを後に当てる
        targets = fold_edges + bead_edges if args.order == "fold-first" else bead_edges + fold_edges

        for label, target in targets:
            radius = bead.bend_radius_mm if label.startswith("bead") else args.fold_radius
            try:
                edge_ref = builder.find_edge_near(doc, part, spa, hsf, body, whole, target)
                filleted = builder.edge_fillet(part, edge_ref, radius, propagation_mode=args.propagation)
                body.AppendHybridShape(filleted)
                part.Update()
                whole = filleted
                print(f"  fillet {label} (R{radius}): OK", flush=True)
            except Exception as exc:  # noqa: BLE001 - スパイクなので全部拾って続ける
                print(f"  fillet {label} (R{radius}): FAIL {str(exc)[:100]}", flush=True)
                print("  -> 1件目の失敗で以降は連鎖するため打ち切る", flush=True)
                break

    part.InWorkObject = whole
    part.Update()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    generated = builder.export_stp(doc, whole, str(OUT_DIR), args.name)
    print("exported", generated.stp_path)


if __name__ == "__main__":
    main()

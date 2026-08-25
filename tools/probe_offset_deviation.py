"""頂面オフセットが解析的な `base + depth*n` からどれだけずれるかをパネル・走行位置別に測る。

背景(2026-08-25): 単発スイープ方式に切り替えたところ、向き判定が「終端パネルの壁頂部で
0.19〜0.35mm ずれる」として落ちるようになった。許容差0.15mmをわずかに超えるだけなので、
単に緩めるのではなく、ずれが局所的(フィレット/境界の近く)か系統的かを先に確かめる。
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import BeadPanelFrame  # noqa: E402
from synthetic_generator.classify import (  # noqa: E402
    FasteningPoint, free_fold_seed, sheared_panel_corners, solve_free_fold,
    tangent_length_for_bend_angle_rad,
)
from synthetic_generator.gsd_build import SyntheticPartBuilder, _cross, _normalize  # noqa: E402

OUT = pathlib.Path(__file__).resolve().parent / "probe_output"
P1 = FasteningPoint((0.0, 0.0, 0.0), (0.7091245539987692, -0.4192780250104878, 0.566875916457342))
P2 = FasteningPoint((105.16387734930477, 9.362453294474978, -80.55987327589155),
                    (0.8798637674773494, -0.4223685455820081, 0.21781772743168493))
R, HW, MARGIN = 11.081175192980025, 14.51036472028607, 13.142192161995899
S1, S2, TILT = 46.17202428508006, 1.5474142248551326, 0.4155136573387424
DEPTH = 6.0


def main() -> None:
    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    for i in range(catia.Documents.Count, 0, -1):
        try: catia.Documents.Item(i).Close()
        except Exception: pass

    seed = free_fold_seed(P1, P2, min_bearing_radius_mm=MARGIN, bend_radius_mm=R,
                          fold1_slack_mm=S1, fold2_slack_mm=S2)
    chain = solve_free_fold(seed, P1, P2, target_a1_rad=seed.a1_rad + TILT)
    assert chain is not None, "chain did not converge"

    frames = [
        BeadPanelFrame(chain.panel1.origin, chain.panel1.u, chain.panel1.v, -MARGIN, chain.L1_mm),
        BeadPanelFrame(chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v, 0.0, chain.L2_mm),
        BeadPanelFrame(chain.panel3.origin, chain.panel3.u, chain.panel3.v, 0.0, chain.L3_mm + MARGIN),
    ]
    corners = [
        sheared_panel_corners(chain.panel1.origin, chain.panel1.u, chain.panel1.v,
                              -MARGIN, chain.L1_mm, HW,
                              near_tilt_rad=0.0, far_tilt_rad=chain.a1_rad),
        sheared_panel_corners(chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v,
                              0.0, chain.L2_mm, HW,
                              near_tilt_rad=chain.a1_rad, far_tilt_rad=chain.a2_rad),
        sheared_panel_corners(chain.panel3.origin, chain.panel3.u, chain.panel3.v,
                              0.0, chain.L3_mm + MARGIN, HW,
                              near_tilt_rad=chain.a2_rad, far_tilt_rad=0.0),
    ]

    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "OFFSET_DEV"

    faces = [builder.rect_fill(hsf, part, body, c) for c in corners]
    whole = builder.join(hsf, part, body, faces)
    part.Update()
    for cs in (corners[1], corners[2]):
        mid = tuple((cs[0][i] + cs[1][i]) / 2 for i in range(3))
        edge = builder.find_edge_near(doc, part, spa, hsf, body, whole, mid)
        whole = builder.edge_fillet_group(part, [edge], R)
        body.AppendHybridShape(whole)
        part.Update()
    surface_ref = part.CreateReferenceFromObject(whole)
    print("base surface OK", flush=True)

    t1 = tangent_length_for_bend_angle_rad(chain.fold1_angle_rad, R) / math.cos(chain.a1_rad)
    t2 = tangent_length_for_bend_angle_rad(chain.fold2_angle_rad, R) / math.cos(chain.a2_rad)
    cuts = [(0.0, t1), (t1, t2), (t2, 0.0)]

    for orientation in (0, 1):
        offset = hsf.AddNewOffset(surface_ref, DEPTH, orientation, 0.01)
        body.AppendHybridShape(offset)
        try:
            part.Update()
        except Exception as exc:
            print(f"orientation={orientation}: build FAILED {str(exc)[:60]}", flush=True)
            builder._delete_feature(doc, part, offset)
            continue
        meas = spa.GetMeasurable(part.CreateReferenceFromObject(offset))
        print(f"\n=== orientation={orientation} ===", flush=True)
        for pi, frame in enumerate(frames):
            n = _normalize(_cross(frame.u, frame.v))
            near_cut, far_cut = cuts[pi]
            lo, hi = frame.near_run_mm + near_cut + 1.0, frame.far_run_mm - far_cut - 1.0
            if hi - lo < 2.0:
                print(f"  panel{pi}: 平坦区間なし", flush=True)
                continue
            row = []
            for k in range(5):
                run = lo + (hi - lo) * k / 4
                base = tuple(frame.origin[i] + run * frame.u[i] for i in range(3))
                for sign in (1.0, -1.0):
                    pt = tuple(base[i] + sign * DEPTH * n[i] for i in range(3))
                    obj = builder.point(hsf, body, *pt)
                    part.Update()
                    d = meas.GetMinimumDistance(part.CreateReferenceFromObject(obj))
                    if sign > 0: plus = d
                    else: minus = d
                row.append(f"run={run:6.1f} +n={plus:5.2f} -n={minus:5.2f}")
            print(f"  panel{pi} (run {lo:.1f}..{hi:.1f}):", flush=True)
            for r in row: print("     ", r, flush=True)
        builder._delete_feature(doc, part, offset)

    doc.Close()


if __name__ == "__main__":
    main()

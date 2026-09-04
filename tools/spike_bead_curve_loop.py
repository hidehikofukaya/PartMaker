"""ビードの壁を「曲線を閉ループにしてから1回だけスイープする」方式で構築するスパイク
(ユーザー提案2026-08-23)。

これまでの「壁面(Sweep結果)を複数回連鎖トリムする」方式は、平坦な単純ケースでも
0/64で全滅することが実機で確定した。曲線同士の交差トリム(AddNewHybridTrim)も
同様に全滅したが、「交点(AddNewIntersection)で切ってJoinする」方式
(AddNewHybridSplit、点でのカット)は安定して動作することを確認した。

方式:
  ① オフセットでビード天面/底面を作る
  ② 壁の起点となる4本の曲線(root_left/root_right/near/far)を作る
  ③ 隣接曲線同士の交点を求め、各曲線をその交点でSplitして中央区間だけ残し、
     4本をJoinして1本の閉曲線にする
  ④ 閉曲線を1回だけスイープ(Mode=4)して壁面全体を一気に作る
  ⑤ スイープ結果の稜線(閉曲線のはず)をエッジフィレット
  ⑥ 天面・壁面・基準面をトリムで統合
  ⑦ 根元のエッジをフィレット
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
    return filleted, w, u1, u2, [face1, face2, sharp]


def corner_point(hsf, part, body, a_ref, b_ref):
    pt = hsf.AddNewIntersection(a_ref, b_ref)
    body.AppendHybridShape(pt)
    part.Update()
    return part.CreateReferenceFromObject(pt)


def trim_to_middle(hsf, part, body, spa, curve_ref, cut_a, cut_b, expected_length):
    """curve_refを2つの交点cut_a/cut_bで切り、中央区間(両交点の間)だけを残す。

    2回連続でAddNewHybridSplitを呼ぶ(1回目の結果を2回目の入力にする)と、
    ドキュメントの履歴が積み重なるにつれて2回目のSplitが不安定になり、実機で
    失敗することが判明した(ユーザー指摘2026-08-23)。代わりに、1回のSplit
    フィーチャーに`AddCuttingElem`で両方の交点を渡し、1回のUpdateで中央区間を
    直接得る方式に変更(o1=-1, o2=1が中央区間を正しく返すことを実機確認済み)。
    向きは複数候補を試し、長さがexpected_lengthに最も近いものを採用する。
    """
    # 失敗した試行がツリーに残ると後続のカーブの処理を不安定にすることが実機で判明した
    # (ユーザー指摘2026-08-23、rightをleftより先に処理すると単独では成功する) -- 既知の
    # 成功パターン(o1=-1,o2=1)を最初に1回だけ試し、無駄な失敗試行を残さないようにする。
    candidates = [(-1, 1), (-1, 2), (1, 1), (1, 2), (-1, -1), (-1, -2), (1, -1), (1, -2)]
    best = None
    for o1, o2 in candidates:
        try:
            split = hsf.AddNewHybridSplit(curve_ref, cut_a, o1)
            split.AddCuttingElem(cut_b, o2)
            body.AppendHybridShape(split)
            part.Update()
            L = spa.GetMeasurable(part.CreateReferenceFromObject(split)).Length
            if abs(L - expected_length) < 0.5:
                print(f"    [debug] trim_to_middle: first-try combo o1={o1} o2={o2} matched, length={L:.2f}", flush=True)
                part.InWorkObject = split
                part.Update()
                return split
            if best is None or abs(L - expected_length) < abs(best[1] - expected_length):
                best = (split, L)
        except Exception:
            continue
    if best is None:
        raise RuntimeError("trim_to_middle: no valid split combination found")
    print(f"    [debug] trim_to_middle picked length={best[1]:.2f} (expected~{expected_length:.2f})", flush=True)
    return best[0]


def main() -> None:
    builder = SyntheticPartBuilder()
    doc = builder.new_part_document()
    part = doc.Part
    hsf = part.HybridShapeFactory
    spa = doc.GetWorkbench("SPAWorkbench")
    body = part.HybridBodies.Add()
    body.Name = "SPIKE_CURVE_LOOP"

    half_width, run, bend_deg = 60.0, 80.0, 70.0
    filleted, w, u1, u2, hide_base = build_filleted_base(
        builder, hsf, part, body, spa, doc, half_width=half_width, run=run, bend_deg=bend_deg
    )
    surface_ref = part.CreateReferenceFromObject(filleted)

    depth_mm, wall_angle_deg, top_width_mm = 7.0, 50.0, 30.0
    wall_slant_mm = depth_mm / math.sin(math.radians(wall_angle_deg))
    wall_run_mm = depth_mm / math.tan(math.radians(wall_angle_deg))
    half_footprint = top_width_mm / 2.0 + wall_run_mm
    run_out_pos = run - 15.0

    origin_pt = builder.point(hsf, body, 0.0, 0.0, 0.0)
    axis_pt = builder.point(hsf, body, *w)
    axis_line = builder.line_pt_pt(hsf, part, body, origin_pt, axis_pt)
    plane = hsf.AddNewPlaneNormal(part.CreateReferenceFromObject(axis_line), part.CreateReferenceFromObject(origin_pt))
    body.AppendHybridShape(plane)
    part.Update()
    centerline = hsf.AddNewIntersection(part.CreateReferenceFromObject(plane), surface_ref)
    body.AppendHybridShape(centerline)
    part.Update()
    centerline_ref = part.CreateReferenceFromObject(centerline)

    root_left = hsf.AddNewCurvePar(centerline_ref, surface_ref, half_footprint, False, True)
    body.AppendHybridShape(root_left)
    part.Update()
    root_left_ref = part.CreateReferenceFromObject(root_left)

    root_right = hsf.AddNewCurvePar(centerline_ref, surface_ref, half_footprint, True, True)
    body.AppendHybridShape(root_right)
    part.Update()
    root_right_ref = part.CreateReferenceFromObject(root_right)

    def width_guide(u, run_pos):
        left_pt = tuple(run_pos * u[i] - half_footprint * w[i] for i in range(3))
        right_pt = tuple(run_pos * u[i] + half_footprint * w[i] for i in range(3))
        p_l = builder.point(hsf, body, *left_pt)
        p_r = builder.point(hsf, body, *right_pt)
        guide = builder.line_pt_pt(hsf, part, body, p_r, p_l)
        part.Update()
        return guide, part.CreateReferenceFromObject(guide)

    guide_near, guide_near_ref = width_guide(u1, run_out_pos)
    guide_far, guide_far_ref = width_guide(u2, run_out_pos)
    print("4 guide curves built OK", flush=True)

    c_ln = corner_point(hsf, part, body, root_left_ref, guide_near_ref)
    c_lf = corner_point(hsf, part, body, root_left_ref, guide_far_ref)
    c_rn = corner_point(hsf, part, body, root_right_ref, guide_near_ref)
    c_rf = corner_point(hsf, part, body, root_right_ref, guide_far_ref)
    print("4 corner points found OK", flush=True)

    centerline_length = spa.GetMeasurable(centerline_ref).Length
    run_out_margin = run - run_out_pos
    expected_run_mid = centerline_length - 2.0 * run_out_margin  # 中央区間の長さの目安
    left_mid = trim_to_middle(hsf, part, body, spa, root_left_ref, c_ln, c_lf, expected_run_mid)
    right_mid = trim_to_middle(hsf, part, body, spa, root_right_ref, c_rn, c_rf, expected_run_mid)
    # near/farガイドはwidth_guideの時点で既に半幅ちょうど(-half_footprint〜+half_footprint)
    # で作っているため、root_left/root_rightと違ってトリム不要(既に正しい範囲)。
    print("2 trimmed middle segments (left/right) OK; near/far need no trim", flush=True)
    for name, seg in [("left", left_mid), ("right", right_mid)]:
        L = spa.GetMeasurable(part.CreateReferenceFromObject(seg)).Length
        print(f"  {name}_mid length={L:.2f}", flush=True)

    loop = builder.join(hsf, part, body, [left_mid, guide_near, right_mid, guide_far])
    part.Update()
    loop_ref = part.CreateReferenceFromObject(loop)
    loop_len = spa.GetMeasurable(loop_ref).Length
    print(f"closed loop curve OK, total length={loop_len:.2f}", flush=True)

    sweep = hsf.AddNewSweepLine(loop_ref)
    sweep.Mode = 4
    sweep.FirstGuideSurf = surface_ref
    sweep.SetAngle(1, wall_angle_deg)
    sweep.SetLength(1, wall_slant_mm)
    body.AppendHybridShape(sweep)
    part.Update()
    sweep_ref = part.CreateReferenceFromObject(sweep)
    area = spa.GetMeasurable(sweep_ref).Area * 1e6
    print(f"single sweep of closed loop OK! area={area:.2f}", flush=True)

    doc.Close()


if __name__ == "__main__":
    main()

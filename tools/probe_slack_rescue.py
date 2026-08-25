"""「全パネルにビード区間」で棄却された試行は、同じ締結点のままslackを選び直せば
救えるのかを純Pythonで実測する(2026-08-25、ユーザー指示: 棄却以外の道の検討)。

棄却の実体は「フィレットの食い込みがパネルの平坦区間を食い尽くす」ことだが、
パネル長L1/L2/L3は中間折れ位置=slackで動かせる。つまりこの棄却は締結点の性質ではなく
**折れ目の置き方の性質**である可能性が高い。CATIA非依存の全チェック
(seed→solve→傾き→接線長→干渉→シアー→plan_bead_on_surface)を再現し、
棄却された試行ごとにslack候補を探索して回収率を測る。
"""
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import BeadPanelFrame, plan_bead_on_surface, sample_bead  # noqa: E402
from synthetic_generator.classify import (  # noqa: E402
    MAX_FOLD_ANGLE_DEG,
    MAX_FOLD_TILT_DEG,
    MIN_PANEL_CLEARANCE_MM,
    MIN_SHEARED_PANEL_SPAN_MM,
    MIN_SHEARED_PANEL_SPAN_RATIO,
    free_fold_seed,
    panel_quad_clearance_mm,
    sheared_panel_corners,
    solve_free_fold,
    tangent_length_for_bend_angle_rad,
)
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

GUIDE_MARGIN = 20.0  # gsd_build.BEAD_GUIDE_MARGIN_MM
RESCUE_CANDIDATES = 30


def python_side(spec, bead, slack1, slack2):
    """gsd_buildのCATIA非依存チェックを全て再現する。戻り値は 'OK' か失敗の種別。"""
    p1, p2 = spec.point1, spec.point2
    margin = spec.min_bearing_radius_mm
    R = spec.bend_radius_mm
    hw = spec.half_width_mm
    seed = free_fold_seed(p1, p2, bend_radius_mm=R, min_bearing_radius_mm=margin,
                          fold1_slack_mm=slack1, fold2_slack_mm=slack2,
                          max_fold_deg=MAX_FOLD_ANGLE_DEG)
    if seed is None:
        return "seed"
    cap = math.radians(MAX_FOLD_TILT_DEG)
    target = max(-cap, min(cap, seed.a1_rad + spec.fold1_tilt_perturbation_rad))
    chain = solve_free_fold(seed, p1, p2, target_a1_rad=target)
    if chain is None:
        return "solve"
    f1, f2 = chain.fold1_angle_rad, chain.fold2_angle_rad
    if max(f1, f2) > math.radians(MAX_FOLD_ANGLE_DEG):
        return "fold_angle"
    if max(abs(chain.a1_rad), abs(chain.a2_rad)) > cap:
        return "tilt"
    t1 = tangent_length_for_bend_angle_rad(f1, R)
    t2 = tangent_length_for_bend_angle_rad(f2, R)
    if chain.L1_mm - t1 < margin or chain.L3_mm - t2 < margin or chain.L2_mm < t1 + t2:
        return "tangent"
    c1 = sheared_panel_corners(chain.panel1.origin, chain.panel1.u, chain.panel1.v,
                               -margin, chain.L1_mm, hw, near_tilt_rad=0.0, far_tilt_rad=chain.a1_rad)
    c3 = sheared_panel_corners(chain.panel3.origin, chain.panel3.u, chain.panel3.v,
                               0.0, chain.L3_mm + margin, hw, near_tilt_rad=chain.a2_rad, far_tilt_rad=0.0)
    if panel_quad_clearance_mm(c1, c3) < MIN_PANEL_CLEARANCE_MM:
        return "clearance"
    for near_run, far_run, near_t, far_t in ((-margin, chain.L1_mm, 0.0, chain.a1_rad),
                                             (0.0, chain.L2_mm, chain.a1_rad, chain.a2_rad),
                                             (0.0, chain.L3_mm + margin, chain.a2_rad, 0.0)):
        nominal = far_run - near_run
        span = nominal - hw * abs(math.tan(far_t) - math.tan(near_t))
        if span < MIN_SHEARED_PANEL_SPAN_MM or span < MIN_SHEARED_PANEL_SPAN_RATIO * nominal:
            return "shear"
    frames = [
        BeadPanelFrame(chain.panel1.origin, chain.panel1.u, chain.panel1.v, -margin, chain.L1_mm),
        BeadPanelFrame(chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v, 0.0, chain.L2_mm),
        BeadPanelFrame(chain.panel3.origin, chain.panel3.u, chain.panel3.v, 0.0, chain.L3_mm + margin),
    ]
    run_cut1 = t1 / math.cos(chain.a1_rad)
    run_cut2 = t2 / math.cos(chain.a2_rad)
    try:
        plan_bead_on_surface(
            frames, bead, inset_mm=2.0 * margin, guide_margin_mm=GUIDE_MARGIN,
            half_width_mm=hw,
            fold_tangents=[(0.0, run_cut1), (run_cut1, run_cut2), (run_cut2, 0.0)],
            fold_tilts=[(0.0, chain.a1_rad), (chain.a1_rad, chain.a2_rad), (chain.a2_rad, 0.0)],
        )
    except ValueError as exc:
        text = str(exc)
        if "no flat bearing stretch" in text:
            return "no_flat"
        return "plan_other"
    return "OK"


def main() -> None:
    rng = random.Random(20260825)
    from collections import Counter
    outcomes = Counter()
    no_flat_cases = []
    for attempt in range(1, 301):
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        result = python_side(spec, bead, spec.fold1_slack_mm, spec.fold2_slack_mm)
        outcomes[result] += 1
        if result == "no_flat":
            no_flat_cases.append((attempt, spec, bead))
    print("300試行の純Python再現:", dict(outcomes))
    print(f"\n『全パネルにビード区間』で棄却: {len(no_flat_cases)} 件 -> slack再探索で救えるか")

    rescue_rng = random.Random(777)
    rescued = 0
    tries_used = []
    for attempt, spec, bead in no_flat_cases:
        found = None
        for k in range(1, RESCUE_CANDIDATES + 1):
            s1 = rescue_rng.uniform(0.0, 60.0)
            s2 = rescue_rng.uniform(0.0, 60.0)
            if python_side(spec, bead, s1, s2) == "OK":
                found = k
                break
        if found:
            rescued += 1
            tries_used.append(found)
    print(f"回収: {rescued}/{len(no_flat_cases)} "
          f"(候補{RESCUE_CANDIDATES}個まで、要した候補数の中央値 "
          f"{sorted(tries_used)[len(tries_used)//2] if tries_used else '-'})")


if __name__ == "__main__":
    main()

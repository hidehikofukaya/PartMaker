"""一般2点部品の純Python側ジオメトリ計画(CATIA非依存)。

`gsd_build.build_general_two_point`のCATIA非依存部分(seed→solve→傾き→接線長→
干渉→シアーの全チェックとパネル構成の決定)をここへ抽出した(2026-08-25)。
目的は2つ:

1. **単一の真実**: これまで同じチェック群がgsd_build内と診断プローブに重複しており、
   乖離のリスクがあった。builderは本モジュールを呼ぶだけになる。
2. **事前解決**: バッチ側がCATIAに触る前に「この締結点で通るslack/ビードがあるか」を
   探索できる(`templates.general_two_point.resolve_bead_slacks`)。棄却は締結点の
   性質(傾き)にだけ適用し、折れ目の置き方の性質(平坦区間・逃げ位置)は棄却ではなく
   slackの選び直しで解決する — 実測でno_flat棄却の64%・plan棄却の50%が回収できる
   (docs/catia_bead_fillet_investigation_log.md SS12)。

失敗は全てValueError(明示的なInfeasible)。メッセージはgsd_build時代と同一に保つ。
"""

from __future__ import annotations

import dataclasses
import math

from synthetic_generator.bead import (
    BEAD_GUIDE_MARGIN_MM,
    BeadPanelFrame,
    BeadParams,
    plan_bead_on_surface,
)
from synthetic_generator.classify import (
    MAX_FOLD_ANGLE_DEG,
    MAX_FOLD_TILT_DEG,
    MIN_NEUTRAL_PLANE_RADIUS_MM,
    MIN_PANEL_CLEARANCE_MM,
    MIN_SHEARED_PANEL_SPAN_MM,
    MIN_SHEARED_PANEL_SPAN_RATIO,
    FasteningPoint,
    Vec3,
    end_panel_corners,
    free_fold_seed,
    panel_quad_clearance_mm,
    sheared_panel_corners,
    single_fold_layout,
    solve_free_fold,
    tangent_length_for_bend_angle_rad,
    two_point_frame,
)


@dataclasses.dataclass(frozen=True)
class GeneralTwoPointPlan:
    """CATIAビルダーがそのまま組めるパネル構成(全てのチェックを通過済み)。"""

    panel_corner_sets: list[list[Vec3]]
    fillet_groups: list[tuple[list[Vec3], float]]
    geometry_label: str
    panel_frames: list[BeadPanelFrame]
    half_width_mm: float
    min_bearing_radius_mm: float
    fold_tangents: list[tuple[float, float]]
    fold_tilts: list[tuple[float, float]]


def plan_general_two_point(
    point1: FasteningPoint,
    point2: FasteningPoint,
    *,
    min_bearing_radius_mm: float,
    half_width_mm: float,
    bend_radius_mm: float,
    fold1_slack_mm: float,
    fold2_slack_mm: float,
    fold1_tilt_perturbation_rad: float,
) -> GeneralTwoPointPlan:
    """単曲げ or 自由折れ目チェーンのパネル構成を決め、全ての幾何チェックを行う。

    通らない場合はValueError(Infeasible)。gsd_buildから抽出したロジックそのもので、
    チェックの内容・順序・メッセージは抽出前と同一。
    """
    if bend_radius_mm < MIN_NEUTRAL_PLANE_RADIUS_MM:
        raise ValueError(
            f"bend_radius_mm={bend_radius_mm:.1f} is below the mandatory minimum "
            f"{MIN_NEUTRAL_PLANE_RADIUS_MM:.1f}mm. Infeasible; not attempting construction."
        )
    if bend_radius_mm > 0.9 * half_width_mm:
        raise ValueError(
            f"bend_radius_mm={bend_radius_mm:.1f} exceeds 0.9x half_width_mm={half_width_mm:.1f}. "
            "Infeasible; not attempting construction."
        )

    margin = min_bearing_radius_mm
    seed = free_fold_seed(
        point1,
        point2,
        bend_radius_mm=bend_radius_mm,
        min_bearing_radius_mm=margin,
        fold1_slack_mm=fold1_slack_mm,
        fold2_slack_mm=fold2_slack_mm,
        max_fold_deg=MAX_FOLD_ANGLE_DEG,
    )
    if seed is None:
        raise ValueError(
            "free_fold_seed found no feasible w-parallel construction for this fastening-point "
            "pair (parallel normals with no lateral offset, fold angle beyond the manufacturing "
            "limit, or the fold fillets would overlap on the ramp). "
            "Infeasible; not attempting construction."
        )

    frame = two_point_frame(point1, point2)
    layout = single_fold_layout(
        point1,
        point2,
        frame,
        bend_radius_mm=bend_radius_mm,
        min_bearing_radius_mm=margin,
        half_width_mm=half_width_mm,
        ramp_fold_angles=(seed.fold1_angle_rad, seed.fold2_angle_rad),
    )
    if layout is not None:
        # 単曲げ(roadmap SS6.24): 2平面の交線で1回だけ曲げる。2枚のパネルが折れ目を
        # 共有するので断面が2セグメントになり、flat同士の干渉は原理的に起きない。
        # SS8の自由折れ目チェーンとは無関係(w平行のまま、旧来ロジックを再利用)。
        panel_corner_sets = [
            end_panel_corners(
                layout.origin1, frame.u1, frame.w, -margin, layout.d1_mm, layout.half_width_mm
            ),
            end_panel_corners(
                layout.origin2, frame.u2, frame.w, -layout.d2_mm, margin, layout.half_width_mm
            ),
        ]
        first = panel_corner_sets[0]
        fold_mids = [tuple((first[2][i] + first[3][i]) / 2 for i in range(3))]
        panel_frames = [
            BeadPanelFrame(layout.origin1, frame.u1, frame.w, -margin, layout.d1_mm),
            BeadPanelFrame(layout.origin2, frame.u2, frame.w, -layout.d2_mm, margin),
        ]
        return GeneralTwoPointPlan(
            panel_corner_sets=panel_corner_sets,
            fillet_groups=[(fold_mids, bend_radius_mm)],
            geometry_label=f"single fold={math.degrees(layout.bend_angle_rad):.1f}deg, R={bend_radius_mm:.1f}",
            panel_frames=panel_frames,
            half_width_mm=layout.half_width_mm,
            min_bearing_radius_mm=margin,
            fold_tangents=[
                (0.0, tangent_length_for_bend_angle_rad(layout.bend_angle_rad, bend_radius_mm)),
                (tangent_length_for_bend_angle_rad(layout.bend_angle_rad, bend_radius_mm), 0.0),
            ],
            fold_tilts=[(0.0, 0.0), (0.0, 0.0)],  # 単曲げの折れ目は傾かない
        )

    # 傾きの上限内を狙って解く(単に棄却するより歩留まりが落ちない)。
    # a2は閉合条件で決まるので上限を保証できず、解けた後に改めて検査する。
    tilt_cap = math.radians(MAX_FOLD_TILT_DEG)
    target_a1 = seed.a1_rad + fold1_tilt_perturbation_rad
    target_a1 = max(-tilt_cap, min(tilt_cap, target_a1))
    chain = solve_free_fold(seed, point1, point2, target_a1_rad=target_a1)
    if chain is None:
        raise ValueError(
            f"solve_free_fold did not converge for fold1 tilt perturbation "
            f"{math.degrees(fold1_tilt_perturbation_rad):.1f}deg from the seed's "
            f"{math.degrees(seed.a1_rad):.1f}deg. Infeasible; not attempting construction."
        )

    fold1_angle, fold2_angle = chain.fold1_angle_rad, chain.fold2_angle_rad
    if max(fold1_angle, fold2_angle) > math.radians(MAX_FOLD_ANGLE_DEG):
        raise ValueError(
            f"solve_free_fold converged to fold angles ({math.degrees(fold1_angle):.1f}deg, "
            f"{math.degrees(fold2_angle):.1f}deg) beyond the manufacturing limit "
            f"{MAX_FOLD_ANGLE_DEG:.0f}deg (the homotopy solve is free to move the dihedral "
            "angles away from the seed's). Infeasible; not attempting construction."
        )
    # 折れ目の傾き上限(ユーザー承認、2026-08-25)。a1は上限内を狙って解いているが、
    # a2は閉合条件で決まるため保証できない。ここで両方を改めて検査する。
    # 傾きが大きいとパネルが激しくシアーした平行四辺形になり、ねじれたリボンや
    # 先端が尖った形状という板金として成立しないものになる。
    tilt1_deg = abs(math.degrees(chain.a1_rad))
    tilt2_deg = abs(math.degrees(chain.a2_rad))
    if max(tilt1_deg, tilt2_deg) > MAX_FOLD_TILT_DEG:
        raise ValueError(
            f"solve_free_fold converged to fold tilts ({tilt1_deg:.1f}deg, "
            f"{tilt2_deg:.1f}deg) beyond the limit {MAX_FOLD_TILT_DEG:.0f}deg -- the panels "
            "would be heavily sheared parallelograms. Infeasible; not attempting construction."
        )

    tangent1 = tangent_length_for_bend_angle_rad(fold1_angle, bend_radius_mm)
    tangent2 = tangent_length_for_bend_angle_rad(fold2_angle, bend_radius_mm)
    if chain.L1_mm - tangent1 < margin:
        raise ValueError(
            f"fold1 (angle={math.degrees(fold1_angle):.1f}deg) eats {tangent1:.1f}mm into panel1, "
            f"leaving less than min_bearing_radius_mm={margin:.1f}mm before point1. "
            "Infeasible; not attempting construction."
        )
    if chain.L3_mm - tangent2 < margin:
        raise ValueError(
            f"fold2 (angle={math.degrees(fold2_angle):.1f}deg) eats {tangent2:.1f}mm into panel3, "
            f"leaving less than min_bearing_radius_mm={margin:.1f}mm before point2. "
            "Infeasible; not attempting construction."
        )
    if chain.L2_mm < tangent1 + tangent2:
        raise ValueError(
            f"fold1/fold2 tangent lengths ({tangent1:.1f}mm + {tangent2:.1f}mm) exceed the "
            f"middle panel's own run length ({chain.L2_mm:.1f}mm) -- the two fillets would "
            "overlap. Infeasible; not attempting construction."
        )

    panel1_corners = sheared_panel_corners(
        chain.panel1.origin, chain.panel1.u, chain.panel1.v,
        -margin, chain.L1_mm, half_width_mm, near_tilt_rad=0.0, far_tilt_rad=chain.a1_rad,
    )
    panel_mid_corners = sheared_panel_corners(
        chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v,
        0.0, chain.L2_mm, half_width_mm, near_tilt_rad=chain.a1_rad, far_tilt_rad=chain.a2_rad,
    )
    panel3_corners = sheared_panel_corners(
        chain.panel3.origin, chain.panel3.u, chain.panel3.v,
        0.0, chain.L3_mm + margin, half_width_mm, near_tilt_rad=chain.a2_rad, far_tilt_rad=0.0,
    )

    # panel1とpanel3は隣接しない(panel_midを挟む)ため、折れ角が大きいと3Dで
    # 面同士が交差/近接しうる(旧w平行版のflat_panels_clearance_mmと同じ懸念、
    # SS6.24/SS8.8)。実際に構築するパネルの4隅同士で3D線分距離を測る。
    clearance = panel_quad_clearance_mm(panel1_corners, panel3_corners)
    if clearance < MIN_PANEL_CLEARANCE_MM:
        raise ValueError(
            f"panel1 and panel3 come within {clearance:.1f}mm of each other "
            f"(minimum {MIN_PANEL_CLEARANCE_MM:.1f}mm) -- the folded panels would interfere. "
            "Infeasible; not attempting construction."
        )

    # 折れ目が傾いていると各パネルは台形になる。幅方向の端では走行長が
    # `half_width * |tan(far_tilt) - tan(near_tilt)|` だけ削られるので、
    # 傾きが強く走行長が短いパネルは潰れて(あるいは自己交差して)Fill/Joinが失敗する。
    # これはCATIA側の脆さではなく純粋に幾何の話なので、実機に触る前に弾く
    # (「joinの失敗は本物のバグの兆候」という既存方針を保つため。2026-08-25に
    # 120試行中1件の生failureとして実測)。
    for label, near_run, far_run, near_tilt, far_tilt in (
        ("panel1", -margin, chain.L1_mm, 0.0, chain.a1_rad),
        ("panel_mid", 0.0, chain.L2_mm, chain.a1_rad, chain.a2_rad),
        ("panel3", 0.0, chain.L3_mm + margin, chain.a2_rad, 0.0),
    ):
        nominal = far_run - near_run
        shear = half_width_mm * abs(math.tan(far_tilt) - math.tan(near_tilt))
        span = nominal - shear
        if span < MIN_SHEARED_PANEL_SPAN_MM or span < MIN_SHEARED_PANEL_SPAN_RATIO * nominal:
            raise ValueError(
                f"{label} is too sheared: run span {nominal:.1f}mm minus shear "
                f"{shear:.1f}mm leaves {span:.1f}mm at the width edges "
                f"(need >= {MIN_SHEARED_PANEL_SPAN_MM:.1f}mm and "
                f">= {MIN_SHEARED_PANEL_SPAN_RATIO:.0%} of the span). "
                "Infeasible; not attempting construction."
            )

    fold1_mid = tuple((panel_mid_corners[0][i] + panel_mid_corners[1][i]) / 2 for i in range(3))
    fold2_mid = tuple((panel_mid_corners[2][i] + panel_mid_corners[3][i]) / 2 for i in range(3))
    panel_frames = [
        BeadPanelFrame(chain.panel1.origin, chain.panel1.u, chain.panel1.v, -margin, chain.L1_mm),
        BeadPanelFrame(chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v, 0.0, chain.L2_mm),
        BeadPanelFrame(
            chain.panel3.origin, chain.panel3.u, chain.panel3.v, 0.0, chain.L3_mm + margin
        ),
    ]
    # 曲げフィレットが**走行方向に**食う長さ。接線長T自体は折れ目に垂直に測った量
    # だが、ビードの中心線は折れ目を傾きaで斜めに横切るので、u方向には T/cos(a)
    # 消費する(SS8.9)。プローブ点と中心線標本点をこの外側にしか置かないために要る。
    run_cut1 = tangent1 / math.cos(chain.a1_rad)
    run_cut2 = tangent2 / math.cos(chain.a2_rad)
    return GeneralTwoPointPlan(
        panel_corner_sets=[panel1_corners, panel_mid_corners, panel3_corners],
        fillet_groups=[([fold1_mid], bend_radius_mm), ([fold2_mid], bend_radius_mm)],
        geometry_label=(
            f"fold1={math.degrees(fold1_angle):.1f}deg, fold2={math.degrees(fold2_angle):.1f}deg, "
            f"a1={math.degrees(chain.a1_rad):.1f}deg, a2={math.degrees(chain.a2_rad):.1f}deg, "
            f"R={bend_radius_mm:.1f}"
        ),
        panel_frames=panel_frames,
        half_width_mm=half_width_mm,
        min_bearing_radius_mm=margin,
        fold_tangents=[(0.0, run_cut1), (run_cut1, run_cut2), (run_cut2, 0.0)],
        fold_tilts=[(0.0, chain.a1_rad), (chain.a1_rad, chain.a2_rad), (chain.a2_rad, 0.0)],
    )



def check_bead_feasible(plan: GeneralTwoPointPlan, bead: BeadParams) -> None:
    """このパネル構成にビードが載るか(載らなければValueError)。

    `gsd_build._add_bead_to_surface`が実機構築時に呼ぶ`plan_bead_on_surface`と
    同じ引数で呼ぶ — 事前判定と実機側の判定が乖離しないことをここで保証する。
    """
    plan_bead_on_surface(
        plan.panel_frames,
        bead,
        inset_mm=2.0 * plan.min_bearing_radius_mm,
        guide_margin_mm=BEAD_GUIDE_MARGIN_MM,
        half_width_mm=plan.half_width_mm,
        fold_tangents=plan.fold_tangents,
        fold_tilts=plan.fold_tilts,
    )

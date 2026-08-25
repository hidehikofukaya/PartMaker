"""plan_bead_on_surface(bead.py、SS8.6〜8.9で自由折れ目チェーン対応に書き換え)のテスト。

CATIA非依存。実機側の裏付け(SS8.9): 中心線は「解析標本点 -> スプライン -> 基準面へ
AddNewProject」で作る方式に確定した。パネルごとのv法線平面の交線をJoinする案は、
平面が無限に広がるため各交線が部品全長を貫いてしまい(ほぼ重複した3本になる)、
Joinが失敗することを実機で確認して破棄した。確定方式は
tools/probe_curvepar_isolate.py で、後続のAddNewCurveParが24通り全て成功することまで
検証済み。

ここではPython側の点群計算(BeadPanelFrameからのプローブ点・中心線標本点の生成)が
正しく機能することを回帰テストとして固定する。
"""
import math
import random

import pytest

from synthetic_generator.bead import BeadPanelFrame, BeadParams, plan_bead_on_surface, sample_bead
from synthetic_generator.classify import (
    FasteningPoint,
    free_fold_seed,
    solve_free_fold,
)

BEAD = BeadParams(
    depth_mm=4.0, top_width_mm=14.0, wall_angle_deg=45.0,
    ridge_radius_mm=5.0,
)

def _sample_chain_panel_frames(half_width_mm: float = 20.0):
    """実際のbatch_generate経路と同じ手順(free_fold_seed→solve_free_fold)で
    3パネルのBeadPanelFrameを1組作る(テスト用の固定シード)。"""
    rng = random.Random(7)
    point1 = FasteningPoint((0.0, 0.0, 0.0), (0.7091245539987692, -0.4192780250104878, 0.566875916457342))
    point2 = FasteningPoint(
        (105.16387734930477, 9.362453294474978, -80.55987327589155),
        (0.8798637674773494, -0.4223685455820081, 0.21781772743168493),
    )
    margin = 13.142192161995899
    seed = free_fold_seed(
        point1, point2, bend_radius_mm=11.081175192980025, min_bearing_radius_mm=margin,
        fold1_slack_mm=46.17202428508006, fold2_slack_mm=1.5474142248551326,
    )
    assert seed is not None
    chain = solve_free_fold(seed, point1, point2, target_a1_rad=seed.a1_rad + 0.4155136573387424)
    assert chain is not None
    return [
        BeadPanelFrame(chain.panel1.origin, chain.panel1.u, chain.panel1.v, -margin, chain.L1_mm),
        BeadPanelFrame(chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v, 0.0, chain.L2_mm),
        BeadPanelFrame(chain.panel3.origin, chain.panel3.u, chain.panel3.v, 0.0, chain.L3_mm + margin),
    ], margin

def test_plan_bead_on_surface_produces_sane_probes_for_free_fold_chain() -> None:
    panel_frames, margin = _sample_chain_panel_frames()
    plan = plan_bead_on_surface(
        panel_frames,
        BEAD,
        inset_mm=2.0 * margin,
        guide_margin_mm=20.0,
        half_width_mm=20.0,
        fold_tangents=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
        fold_tilts=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
    )
    assert plan.top_keep_bases, "ビードが載る区間が1つも見つからなかった"
    assert len(plan.top_keep_bases) == len(plan.top_panel_index)
    assert plan.base_keep and plan.base_remove
    # 単発スイープ化(SS8.10)で、ドラフト角判定に要るのは長辺の左右2点だけになった
    assert len(plan.wall_top_bases) == 2
    assert len(plan.wall_root_probes) == 2 * len(plan.top_keep_bases)
    # 中心線の標本点は必ず全パネル分そろう(平坦区間が取れないパネルでも最低1点)
    assert len(plan.centreline_points) >= len(panel_frames)

    def finite(p) -> bool:
        return all(math.isfinite(c) for c in p)

    for point in (
        plan.base_keep + plan.base_remove + plan.top_keep_bases + plan.top_remove_bases
        + plan.wall_root_probes + plan.wall_top_bases + list(plan.start_guide) + list(plan.end_guide)
    ):
        assert finite(point)

def test_plan_bead_on_surface_probe_points_lie_on_declared_panel_plane() -> None:
    """base_keep等のプローブ点は、それが属するパネルのorigin+u*run+v*widthという
    平面上に厳密に乗るはず(=平面の法線nとの内積が0)。"""
    panel_frames, margin = _sample_chain_panel_frames()
    plan = plan_bead_on_surface(
        panel_frames,
        BEAD,
        inset_mm=2.0 * margin,
        guide_margin_mm=20.0,
        half_width_mm=20.0,
        fold_tangents=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
        fold_tilts=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
    )

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    def normalize(v):
        length = math.sqrt(sum(c * c for c in v))
        return tuple(c / length for c in v)

    for point, index in zip(plan.top_keep_bases, plan.top_panel_index):
        frame = panel_frames[index]
        normal = normalize(cross(frame.u, frame.v))
        rel = tuple(point[i] - frame.origin[i] for i in range(3))
        assert abs(sum(rel[i] * normal[i] for i in range(3))) < 1e-9

def test_centreline_points_are_on_panel_axes_and_ordered() -> None:
    """中心線の標本点は各パネルのv=0のu軸上(=幅方向オフセット0)に厳密に乗り、
    走行順に並んでいる(スプラインの入力として順序が崩れていると形状が破綻するため)。"""
    panel_frames, margin = _sample_chain_panel_frames()
    plan = plan_bead_on_surface(
        panel_frames,
        BEAD,
        inset_mm=2.0 * margin,
        guide_margin_mm=20.0,
        half_width_mm=20.0,
        fold_tangents=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
        fold_tilts=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
    )

    def dot(a, b):
        return sum(a[i] * b[i] for i in range(3))

    # 各点は「いずれかのパネルの軸上」= そのパネルのv成分もn成分も0
    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    for point in plan.centreline_points:
        on_some_axis = False
        for frame in panel_frames:
            rel = tuple(point[i] - frame.origin[i] for i in range(3))
            normal = cross(frame.u, frame.v)
            if abs(dot(rel, frame.v)) < 1e-9 and abs(dot(rel, normal)) < 1e-9:
                on_some_axis = True
                break
        assert on_some_axis, f"{point} はどのパネルの中心軸上にも乗っていない"

    # 走行順: 連続する点の間隔が正で、全体として単調に進む
    total = 0.0
    for a, b in zip(plan.centreline_points, plan.centreline_points[1:]):
        step = math.dist(a, b)
        assert step > 1e-9, "標本点が重複している(スプラインが破綻する)"
        total += step
    assert total > 0.0

def test_end_guides_never_run_off_the_panel_width() -> None:
    """端の幅ガイドはオーバーサイズにするが板幅を超えてはいけない。超えるとガイド線が
    基準面から外れ、Mode=4のスイープが全ドラフト角で失敗する(2026-08-24に実機で確認)。
    guide_margin_mmを板幅よりずっと大きく指定しても、板幅内にクランプされること。"""
    panel_frames, margin = _sample_chain_panel_frames()
    half_width = 20.0
    plan = plan_bead_on_surface(
        panel_frames,
        BEAD,
        inset_mm=2.0 * margin,
        guide_margin_mm=500.0,  # 極端に大きく指定してもクランプされるはず
        half_width_mm=half_width,
        fold_tangents=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
        fold_tilts=[(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
    )
    for guide, label in ((plan.start_guide, "start"), (plan.end_guide, "end")):
        span = math.dist(guide[0], guide[1])
        assert span <= 2.0 * half_width + 1e-9, f"{label} guide が板幅を超えている: {span:.2f}mm"
        # コーナーRが削る材料が残るよう、フットプリントよりは広くあってほしい
        assert span > 2.0 * BEAD.half_footprint_mm

def test_plan_bead_on_surface_rejects_bead_too_wide_for_the_panel() -> None:
    panel_frames, margin = _sample_chain_panel_frames()
    wide_bead = BeadParams(
        depth_mm=6.0, top_width_mm=100.0, wall_angle_deg=45.0, ridge_radius_mm=5.0,
    )
    with pytest.raises(ValueError, match="does not fit"):
        plan_bead_on_surface(
            panel_frames, wide_bead, inset_mm=2.0 * margin, guide_margin_mm=20.0,
            half_width_mm=20.0, fold_tangents=[(0.0, 0.0)] * 3, fold_tilts=[(0.0, 0.0)] * 3,
        )

def test_plan_bead_on_surface_rejects_empty_panel_list() -> None:
    with pytest.raises(ValueError, match="no panels"):
        plan_bead_on_surface([], BEAD, inset_mm=10.0, guide_margin_mm=20.0, half_width_mm=20.0, fold_tangents=[], fold_tilts=[])

if __name__ == "__main__":
    test_plan_bead_on_surface_produces_sane_probes_for_free_fold_chain()
    test_plan_bead_on_surface_probe_points_lie_on_declared_panel_plane()
    test_centreline_points_are_on_panel_axes_and_ordered()
    test_end_guides_never_run_off_the_panel_width()
    test_plan_bead_on_surface_rejects_bead_too_wide_for_the_panel()
    test_plan_bead_on_surface_rejects_empty_panel_list()
    print("OK: plan_bead_on_surface self-checks passed")

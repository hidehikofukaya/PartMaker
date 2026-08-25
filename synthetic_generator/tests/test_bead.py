"""bead.py のセル分割幾何(roadmap SS6.25)を検証する。CATIA非依存の純粋関数のみ。"""

import math
import random

import pytest

from synthetic_generator.bead import (
    BEAD_DEPTH_RANGE_MM,
    BEAD_TOP_WIDTH_RANGE_MM,
    BEAD_WALL_ANGLE_RANGE_DEG,
    BeadParams,
    sample_bead,
)
from synthetic_generator.classify import (
    MIN_NEUTRAL_PLANE_RADIUS_MM,
    FasteningPoint,
    end_panel_corners,
    two_point_frame,
)

BEAD = BeadParams(
    depth_mm=6.0, top_width_mm=30.0, wall_angle_deg=45.0,
    ridge_radius_mm=5.0,
)


def _planarity_error(cell) -> float:
    """4隅の4点目が、最初の3点が張る平面からどれだけ外れているか[mm]。"""
    a, b, c, d = cell
    u = tuple(b[i] - a[i] for i in range(3))
    v = tuple(c[i] - a[i] for i in range(3))
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    length = math.sqrt(sum(x * x for x in n))
    assert length > 1e-9, "degenerate cell"
    return abs(sum((d[i] - a[i]) * n[i] for i in range(3))) / length

def test_sample_bead_stays_within_the_agreed_ranges_and_is_deterministic() -> None:
    for seed in range(200):
        half_width = 12.5 + (seed % 26) * 0.5  # 板半幅12.5〜25mm(ユーザー指定の板幅25〜50mm)
        bead = sample_bead(random.Random(seed), half_width)
        assert BEAD_DEPTH_RANGE_MM[0] <= bead.depth_mm <= BEAD_DEPTH_RANGE_MM[1]
        assert BEAD_TOP_WIDTH_RANGE_MM[0] <= bead.top_width_mm <= BEAD_TOP_WIDTH_RANGE_MM[1]
        assert BEAD_WALL_ANGLE_RANGE_DEG[0] <= bead.wall_angle_deg <= BEAD_WALL_ANGLE_RANGE_DEG[1]
        # ユーザー製造制約: 全ての半径が中立面R最小以上
        assert bead.ridge_radius_mm >= MIN_NEUTRAL_PLANE_RADIUS_MM
        # 足元・頂稜線の2つのフィレットが壁の上で重ならないこと
        # (R最小を強制した結果として成立しない組み合わせは bead_fits が弾く)
        assert bead.ridges_fit_on_wall or bead.ridge_radius_mm == MIN_NEUTRAL_PLANE_RADIUS_MM
    assert sample_bead(random.Random(7), 20.0) == sample_bead(random.Random(7), 20.0)

def test_sample_bead_keeps_the_footprint_inside_the_panel_when_feasible() -> None:
    """板幅から逆算した頂部幅を使うので、成立した組み合わせはフットプリント+余白が
    半幅に収まっているはず(収まらない=最小値を強制した組み合わせのみ)。"""
    forced_minimum = 0
    for seed in range(300):
        half_width = 12.5 + (seed % 26) * 0.5
        bead = sample_bead(random.Random(seed), half_width)
        if bead.top_width_mm == BEAD_TOP_WIDTH_RANGE_MM[0]:
            forced_minimum += 1  # 幅が足りず最小値を強制されたケース(下流で弾かれる)
            continue
        assert bead.half_footprint_mm + bead.side_margin_mm <= half_width + 1e-9
    assert forced_minimum < 300, "全ケースが強制最小値なら、レンジ設定が破綻している"


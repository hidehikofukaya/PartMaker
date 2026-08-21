"""bead.py のセル分割幾何(roadmap SS6.25)を検証する。CATIA非依存の純粋関数のみ。"""

import math
import random

import pytest

from synthetic_generator.bead import (
    BEAD_DEPTH_RANGE_MM,
    BEAD_TOP_WIDTH_RANGE_MM,
    BEAD_WALL_ANGLE_RANGE_DEG,
    BeadParams,
    _miter_offset,
    _panel_axes,
    bead_cells,
    bead_fits,
    sample_bead,
)
from synthetic_generator.classify import (
    MIN_NEUTRAL_PLANE_RADIUS_MM,
    FasteningPoint,
    end_panel_corners,
    two_point_frame,
)

BEAD = BeadParams(depth_mm=6.0, top_width_mm=30.0, wall_angle_deg=45.0, bend_radius_mm=4.0)


def _two_fold_panels(lateral_mm: float = 0.0) -> list[list[tuple[float, float, float]]]:
    """build_general_two_pointの2曲げ経路と同じ手順でflat1/ランプ/flat2を作る。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(120.0, lateral_mm, 60.0), normal_xyz=(1.0, 0.0, 1.0))
    frame = two_point_frame(p1, p2)
    flat1 = end_panel_corners(p1.position_xyz, frame.u1, frame.w, -20.0, 45.0, 40.0)
    flat2 = end_panel_corners(p2.position_xyz, frame.u2, frame.w, -45.0, 20.0, 40.0)
    ramp = [flat1[3], flat1[2], flat2[1], flat2[0]]
    return [flat1, ramp, flat2]


def _planarity_error(cell) -> float:
    """4隅の4点目が、最初の3点が張る平面からどれだけ外れているか[mm]。"""
    a, b, c, d = cell
    u = tuple(b[i] - a[i] for i in range(3))
    v = tuple(c[i] - a[i] for i in range(3))
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    length = math.sqrt(sum(x * x for x in n))
    assert length > 1e-9, "degenerate cell"
    return abs(sum((d[i] - a[i]) * n[i] for i in range(3))) / length


def test_every_bead_cell_is_planar_without_lateral_offset() -> None:
    """横方向オフセットが無ければ全セルが平面であること(rect_fillで作れる前提)。"""
    cells = bead_cells(_two_fold_panels(0.0), BEAD)
    assert len(cells) == 15  # 3パネル x 5ストリップ
    assert max(_planarity_error(cell) for cell in cells) < 1e-9


def test_only_the_ramp_walls_twist_when_there_is_a_lateral_offset() -> None:
    """横方向オフセットがあるとランプの壁セルだけが捻れる(双曲放物面)。近似誤差では
    なく正しい幾何なので、捻れる場所が壁2枚に限られることだけを固定する。"""
    cells = bead_cells(_two_fold_panels(35.0), BEAD)
    twisted = {i for i, cell in enumerate(cells) if _planarity_error(cell) > 1e-9}
    assert twisted == {6, 8}  # ランプ(パネル1)の立上り・立下りのみ
    assert max(_planarity_error(cells[i]) for i in twisted) < 2.0


def test_bead_is_continuous_across_the_folds() -> None:
    """あるパネルのセルkの折れ目側の辺と、次のパネルのセルkの折れ目側の辺が厳密に
    一致するはず(一致しなければ曲げ部でビードが途切れる)。"""
    cells = bead_cells(_two_fold_panels(35.0), BEAD)
    for panel_index in range(2):
        for strip in range(5):
            upstream = cells[panel_index * 5 + strip]
            downstream = cells[(panel_index + 1) * 5 + strip]
            # 規約[near_left, near_right, far_right, far_left]の far 側 == 次の near 側
            assert upstream[3] == pytest.approx(downstream[0], abs=1e-9)
            assert upstream[2] == pytest.approx(downstream[1], abs=1e-9)


def test_outer_cells_keep_the_original_panel_outline() -> None:
    """ビードを入れてもパネルの外形は変わらないこと(両端のセルが元の4隅を保持する)。"""
    panels = _two_fold_panels()
    cells = bead_cells(panels, BEAD)
    for panel_index, corners in enumerate(panels):
        low_side = cells[panel_index * 5]
        high_side = cells[panel_index * 5 + 4]
        assert low_side[0] == corners[0] and low_side[3] == corners[3]
        assert high_side[1] == corners[1] and high_side[2] == corners[2]


def test_top_strip_sits_exactly_one_depth_above_a_flat_panel() -> None:
    """平坦パネル上では、頂部ストリップは法線方向にちょうどdepthだけ浮くはず。"""
    flat = end_panel_corners((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), -30.0, 30.0, 40.0)
    _run, _width, normal = _panel_axes(flat)
    top = bead_cells([flat], BEAD)[2]
    for point in top:
        assert sum(point[i] * normal[i] for i in range(3)) == pytest.approx(BEAD.depth_mm)


def test_wall_strip_stands_at_the_requested_angle() -> None:
    """立上りストリップが、一般面に対してwall_angle_degで立っているはず。"""
    flat = end_panel_corners((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), -30.0, 30.0, 40.0)
    wall = bead_cells([flat], BEAD)[1]
    rise = wall[1][2] - wall[0][2]  # 法線=+Zなので高さ方向の差
    run = abs(wall[1][1] - wall[0][1])
    assert math.degrees(math.atan2(rise, run)) == pytest.approx(BEAD.wall_angle_deg)


def test_miter_offset_degenerates_to_a_plain_offset_when_the_panels_are_coplanar() -> None:
    """折れなし(na=nb)ならマイター量は素直に depth*n になるはず。"""
    normal = (0.0, 0.0, 1.0)
    assert _miter_offset(normal, normal, 6.0) == pytest.approx((0.0, 0.0, 6.0))
    # 直角に折れる場合は長さ depth/cos(45度) = depth*sqrt(2)
    shift = _miter_offset((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 6.0)
    assert math.sqrt(sum(c * c for c in shift)) == pytest.approx(6.0 * math.sqrt(2))


def test_bead_fits_rejects_panels_narrower_than_the_footprint() -> None:
    wide = _two_fold_panels()
    assert bead_fits(wide, BEAD)
    narrow = [
        end_panel_corners((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), -20.0, 20.0, 18.0)
    ]
    assert not bead_fits(narrow, BEAD)  # half_width=18mm < 頂部15mm+壁6mm+余白5mm


def test_sample_bead_stays_within_the_agreed_ranges_and_is_deterministic() -> None:
    for seed in range(200):
        bead = sample_bead(random.Random(seed))
        assert BEAD_DEPTH_RANGE_MM[0] <= bead.depth_mm <= BEAD_DEPTH_RANGE_MM[1]
        assert BEAD_TOP_WIDTH_RANGE_MM[0] <= bead.top_width_mm <= BEAD_TOP_WIDTH_RANGE_MM[1]
        assert BEAD_WALL_ANGLE_RANGE_DEG[0] <= bead.wall_angle_deg <= BEAD_WALL_ANGLE_RANGE_DEG[1]
        assert bead.bend_radius_mm >= MIN_NEUTRAL_PLANE_RADIUS_MM
        # 壁の上下2つのフィレットが壁の上で重ならないこと(R強制の4mmを引いた場合を除く)
        tangent = 2 * bead.bend_radius_mm * math.tan(math.radians(bead.wall_angle_deg) / 2)
        assert tangent <= bead.wall_slant_mm or bead.bend_radius_mm == MIN_NEUTRAL_PLANE_RADIUS_MM
    assert sample_bead(random.Random(7)) == sample_bead(random.Random(7))


def test_bead_fits_rejects_a_collapsed_top_strip() -> None:
    """隣接パネルの折れ角が180度に近いとマイター量が発散し(式の分母 1+na.nb -> 0)、
    頂部ストリップの走行長が潰れる/反転する。幅は足りていてもビード不可と判定するはず
    (実測でCATIA側のjoinがここで失敗していた)。"""
    flat = end_panel_corners((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), -20.0, 60.0, 40.0)
    # ほぼ来た方向へ折り返す(折れ角約170度)ランプ
    direction = (math.cos(math.radians(170.0)), 0.0, math.sin(math.radians(170.0)))
    length = 50.0

    def folded(width: float) -> tuple[float, float, float]:
        return (60.0 + length * direction[0], width, length * direction[2])

    panels = [flat, [flat[3], flat[2], folded(40.0), folded(-40.0)]]

    assert all(math.dist(c[0], c[1]) / 2 >= BEAD.half_footprint_mm for c in panels)  # 幅は足りている
    assert not bead_fits(panels, BEAD)

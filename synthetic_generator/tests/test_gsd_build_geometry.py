"""gsd_build.py のうちCATIAに依存しない純粋な幾何計算(_jog_frame/_panel_corners)を検証する。

COM呼び出し自体(SyntheticPartBuilderのメソッド)はCATIA無しでは検証できない。
"""

import math

import pytest

from synthetic_generator.classify import FasteningPoint
from synthetic_generator.gsd_build import _jog_frame, _panel_corners, _tangent_arc_boundary


def test_jog_frame_axis_and_width_dir_orthonormal() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(50.0, 0.0, 20.0), normal_xyz=(0.0, 0.0, 1.0))

    frame = _jog_frame(p1, p2)

    assert frame.axis_dir == pytest.approx((1.0, 0.0, 0.0))
    assert frame.width_dir == pytest.approx((0.0, 1.0, 0.0))
    assert frame.run_length_mm == pytest.approx(50.0)
    assert frame.offset_mm == pytest.approx(20.0)


def test_jog_frame_handles_diagonal_lateral_offset() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(30.0, 40.0, 15.0), normal_xyz=(0.0, 0.0, 1.0))

    frame = _jog_frame(p1, p2)

    assert frame.run_length_mm == pytest.approx(50.0)  # 3-4-5
    assert frame.offset_mm == pytest.approx(15.0)
    # axis_dir must be unit length and orthogonal to the panel normal
    assert math.hypot(*frame.axis_dir[:2]) == pytest.approx(1.0)
    assert frame.axis_dir[2] == pytest.approx(0.0, abs=1e-9)


def test_jog_frame_rejects_coincident_lateral_position() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(0.0, 0.0, 20.0), normal_xyz=(0.0, 0.0, 1.0))  # pure offset, no lateral run

    with pytest.raises(ValueError):
        _jog_frame(p1, p2)


def test_panel_corners_flat_rectangle_shape() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(50.0, 0.0, 20.0), normal_xyz=(0.0, 0.0, 1.0))
    frame = _jog_frame(p1, p2)

    corners = _panel_corners(p1.position_xyz, frame, run0=0.0, run1=25.0, height0=0.0, height1=0.0, half_width=10.0)

    assert corners == [
        pytest.approx((0.0, -10.0, 0.0)),
        pytest.approx((0.0, 10.0, 0.0)),
        pytest.approx((25.0, 10.0, 0.0)),
        pytest.approx((25.0, -10.0, 0.0)),
    ]


def test_panel_corners_ramp_interpolates_height() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(50.0, 0.0, 20.0), normal_xyz=(0.0, 0.0, 1.0))
    frame = _jog_frame(p1, p2)

    corners = _panel_corners(p1.position_xyz, frame, run0=20.0, run1=30.0, height0=0.0, height1=20.0, half_width=10.0)

    assert corners == [
        pytest.approx((20.0, -10.0, 0.0)),
        pytest.approx((20.0, 10.0, 0.0)),
        pytest.approx((30.0, 10.0, 20.0)),
        pytest.approx((30.0, -10.0, 20.0)),
    ]


def test_tangent_arc_boundary_produces_true_tangent_points() -> None:
    """corner->tangent直線は、origin->tangent半径ベクトルと直交する(=真の接線)はず。"""
    origin = (0.0, 0.0, 0.0)
    axis_dir = (1.0, 0.0, 0.0)
    width_dir = (0.0, 1.0, 0.0)
    radius = 15.0

    boundary = _tangent_arc_boundary(origin, axis_dir, width_dir, far_distance=30.0, half_width=20.0, radius=radius)

    for corner, tangent in [(boundary.corner_a, boundary.tangent_a), (boundary.corner_b, boundary.tangent_b)]:
        assert math.dist(origin, tangent) == pytest.approx(radius)
        radial = tuple(tangent[i] - origin[i] for i in range(3))
        tangent_line = tuple(corner[i] - tangent[i] for i in range(3))
        assert sum(radial[i] * tangent_line[i] for i in range(3)) == pytest.approx(0.0, abs=1e-9)


def test_tangent_arc_boundary_symmetric_for_symmetric_corners() -> None:
    origin = (0.0, 0.0, 0.0)
    axis_dir = (1.0, 0.0, 0.0)
    width_dir = (0.0, 1.0, 0.0)

    boundary = _tangent_arc_boundary(origin, axis_dir, width_dir, far_distance=30.0, half_width=20.0, radius=15.0)

    # tangent_a/tangent_bはwidth(=y)符号だけが反転した鏡像のはず
    assert boundary.tangent_a[0] == pytest.approx(boundary.tangent_b[0])
    assert boundary.tangent_a[1] == pytest.approx(-boundary.tangent_b[1])


def test_tangent_arc_boundary_sweep_matches_known_value() -> None:
    """far_distance=30, half_width=20, radius=15の解析解(手計算検証済み)と一致するはず。"""
    origin = (0.0, 0.0, 0.0)
    axis_dir = (1.0, 0.0, 0.0)
    width_dir = (0.0, 1.0, 0.0)

    boundary = _tangent_arc_boundary(origin, axis_dir, width_dir, far_distance=30.0, half_width=20.0, radius=15.0)

    distance = math.hypot(30.0, 20.0)
    k = math.degrees(math.atan2(20.0, 30.0) + math.acos(15.0 / distance))
    expected_sweep = (360.0 - 2.0 * k) % 360.0
    assert boundary.sweep_deg == pytest.approx(expected_sweep)
    # 遠端(corner側)を通らない「奥側」の弧を通るよう、掃引角は半円(180度)より大きいはず
    # (=近い側のショートカットではなく円の裏側を回る)
    assert boundary.sweep_deg > 90.0


def test_tangent_arc_boundary_rejects_when_corner_inside_circle() -> None:
    origin = (0.0, 0.0, 0.0)
    axis_dir = (1.0, 0.0, 0.0)
    width_dir = (0.0, 1.0, 0.0)

    with pytest.raises(ValueError):
        _tangent_arc_boundary(origin, axis_dir, width_dir, far_distance=5.0, half_width=5.0, radius=15.0)


def test_panel_corners_degenerates_to_vertical_wall_when_run0_equals_run1() -> None:
    """ramp_extent=0(run0==run1)のとき、旧来の垂直リザー(幅ゼロの壁)に一致するはず。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(50.0, 0.0, 20.0), normal_xyz=(0.0, 0.0, 1.0))
    frame = _jog_frame(p1, p2)

    corners = _panel_corners(p1.position_xyz, frame, run0=25.0, run1=25.0, height0=0.0, height1=20.0, half_width=10.0)

    assert corners == [
        pytest.approx((25.0, -10.0, 0.0)),
        pytest.approx((25.0, 10.0, 0.0)),
        pytest.approx((25.0, 10.0, 20.0)),
        pytest.approx((25.0, -10.0, 20.0)),
    ]



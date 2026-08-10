"""gsd_build.py のうちCATIAに依存しない純粋な幾何計算(_jog_frame/_panel_corners)を検証する。

COM呼び出し自体(SyntheticPartBuilderのメソッド)はCATIA無しでは検証できない。
"""

import math

import pytest

from synthetic_generator.classify import FasteningPoint
from synthetic_generator.gsd_build import (
    _end_panel_corners,
    _jog_frame,
    _panel_corners,
    _tangent_arc_boundary,
    _two_point_frame,
)


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


def _norm(v: tuple[float, float, float]) -> float:
    return math.sqrt(sum(c * c for c in v))


def _dot3(a, b) -> float:
    return sum(x * y for x, y in zip(a, b))


def test_two_point_frame_reduces_to_jog_frame_when_normals_match() -> None:
    """n1=n2(現行parallel_sameクラス)のとき、u1=u2=_jog_frameのaxis_dir、
    w=width_dirに厳密に一致するはず(SS6.20の一般化がregression-safeであることの確認)。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(30.0, 40.0, 15.0), normal_xyz=(0.0, 0.0, 1.0))

    old = _jog_frame(p1, p2)
    new = _two_point_frame(p1, p2)

    assert new.w == pytest.approx(old.width_dir)
    assert new.u1 == pytest.approx(old.axis_dir)
    assert new.u2 == pytest.approx(old.axis_dir)


def test_two_point_frame_reduces_to_jog_frame_for_tilted_parallel_normals() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(-0.65, -0.72, -0.26))
    p2 = FasteningPoint(position_xyz=(-94.0, 40.0, 36.0), normal_xyz=(-0.65, -0.72, -0.26))

    old = _jog_frame(p1, p2)
    new = _two_point_frame(p1, p2)

    assert new.w == pytest.approx(old.width_dir)
    assert new.u1 == pytest.approx(old.axis_dir)
    assert new.u2 == pytest.approx(old.axis_dir)


def test_two_point_frame_orthogonal_normals_preserves_angle_between_axes() -> None:
    """n1⊥n2の一般ケース: wは両方の法線に直交し、u1/u2は単位直交で、u1・u2のなす角が
    n1・n2のなす角と一致するはず(共に同じwまわりの90度回転で得られるため)。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(50.0, 10.0, 30.0), normal_xyz=(1.0, 0.0, 0.0))

    frame = _two_point_frame(p1, p2)

    assert _norm(frame.w) == pytest.approx(1.0)
    assert _dot3(frame.w, frame.n1) == pytest.approx(0.0, abs=1e-9)
    assert _dot3(frame.w, frame.n2) == pytest.approx(0.0, abs=1e-9)
    assert _norm(frame.u1) == pytest.approx(1.0)
    assert _norm(frame.u2) == pytest.approx(1.0)
    assert _dot3(frame.u1, frame.n1) == pytest.approx(0.0, abs=1e-9)
    assert _dot3(frame.u1, frame.w) == pytest.approx(0.0, abs=1e-9)
    assert _dot3(frame.u2, frame.n2) == pytest.approx(0.0, abs=1e-9)
    assert _dot3(frame.u2, frame.w) == pytest.approx(0.0, abs=1e-9)

    angle_normals = math.acos(max(-1.0, min(1.0, _dot3(frame.n1, frame.n2))))
    angle_axes = math.acos(max(-1.0, min(1.0, _dot3(frame.u1, frame.u2))))
    assert angle_axes == pytest.approx(angle_normals)


def test_two_point_frame_handles_antiparallel_normals() -> None:
    """反平行(parallel_oppositeクラス)もn1×n2=0で退化フォールバック分岐に入り、
    エラーにならず横方向変位から求まるはず。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(40.0, 0.0, 10.0), normal_xyz=(0.0, 0.0, -1.0))

    frame = _two_point_frame(p1, p2)

    assert _dot3(frame.w, frame.n1) == pytest.approx(0.0, abs=1e-9)
    assert _dot3(frame.w, frame.n2) == pytest.approx(0.0, abs=1e-9)
    assert _norm(frame.u1) == pytest.approx(1.0)
    assert _norm(frame.u2) == pytest.approx(1.0)


def test_two_point_frame_rejects_coincident_lateral_position_when_normals_parallel() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(0.0, 0.0, 20.0), normal_xyz=(0.0, 0.0, 1.0))  # 横方向変位ゼロ

    with pytest.raises(ValueError):
        _two_point_frame(p1, p2)


def test_end_panel_corners_flat_rectangle_shape() -> None:
    origin = (5.0, 0.0, 0.0)
    u = (1.0, 0.0, 0.0)
    w = (0.0, 1.0, 0.0)

    corners = _end_panel_corners(origin, u, w, near=-10.0, far=20.0, half_width=8.0)

    assert corners == [
        pytest.approx((-5.0, -8.0, 0.0)),
        pytest.approx((-5.0, 8.0, 0.0)),
        pytest.approx((25.0, 8.0, 0.0)),
        pytest.approx((25.0, -8.0, 0.0)),
    ]


def test_general_ramp_corners_are_planar_for_orthogonal_normals() -> None:
    """一般ケース(n1⊥n2)でも、flat1のランプ側辺(index[2]/[3])とflat2のランプ側辺
    (index[0]/[1])を直接繋いだ4点(ramp_corners)は、平行な2直線(共にw方向)を繋ぐため
    幾何的に必ず同一平面上にあるはず(scalar triple productがほぼ0)。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(60.0, 25.0, 40.0), normal_xyz=(1.0, 0.0, 0.0))
    frame = _two_point_frame(p1, p2)
    half_width = 15.0

    flat1_corners = _end_panel_corners(p1.position_xyz, frame.u1, frame.w, near=-20.0, far=18.0, half_width=half_width)
    flat2_corners = _end_panel_corners(p2.position_xyz, frame.u2, frame.w, near=-22.0, far=20.0, half_width=half_width)
    ramp_corners = [flat1_corners[3], flat1_corners[2], flat2_corners[1], flat2_corners[0]]

    a, b, c, d = ramp_corners
    v1 = tuple(b[i] - a[i] for i in range(3))
    v2 = tuple(c[i] - a[i] for i in range(3))
    v3 = tuple(d[i] - a[i] for i in range(3))
    # scalar triple product v1 . (v2 x v3) == 0 <=> 4点が同一平面上
    cross23 = (
        v2[1] * v3[2] - v2[2] * v3[1],
        v2[2] * v3[0] - v2[0] * v3[2],
        v2[0] * v3[1] - v2[1] * v3[0],
    )
    triple_product = _dot3(v1, cross23)
    assert triple_product == pytest.approx(0.0, abs=1e-6)

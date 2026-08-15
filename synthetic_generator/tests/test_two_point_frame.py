"""classify.py の任意法線・任意位置の一般化(two_point_frame/end_panel_corners/
ramp_fold_angles_rad、roadmap SS6.20)を検証する。CATIA非依存の純粋関数のみ。
"""

import math

import pytest

from synthetic_generator.classify import (
    FasteningPoint,
    end_panel_corners,
    fold_tangent_length_mm,
    ramp_fold_angles_rad,
    tangent_length_for_bend_angle_rad,
    two_point_frame,
)
from synthetic_generator.gsd_build import _jog_frame


def _norm(v: tuple[float, float, float]) -> float:
    return math.sqrt(sum(c * c for c in v))


def _dot3(a, b) -> float:
    return sum(x * y for x, y in zip(a, b))


def test_two_point_frame_reduces_to_jog_frame_when_normals_match() -> None:
    """n1=n2(parallel_sameクラス)のとき、u1=u2=_jog_frameのaxis_dir、
    w=width_dirに厳密に一致するはず(SS6.20の一般化がregression-safeであることの確認)。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(30.0, 40.0, 15.0), normal_xyz=(0.0, 0.0, 1.0))

    old = _jog_frame(p1, p2)
    new = two_point_frame(p1, p2)

    assert new.w == pytest.approx(old.width_dir)
    assert new.u1 == pytest.approx(old.axis_dir)
    assert new.u2 == pytest.approx(old.axis_dir)


def test_two_point_frame_reduces_to_jog_frame_for_tilted_parallel_normals() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(-0.65, -0.72, -0.26))
    p2 = FasteningPoint(position_xyz=(-94.0, 40.0, 36.0), normal_xyz=(-0.65, -0.72, -0.26))

    old = _jog_frame(p1, p2)
    new = two_point_frame(p1, p2)

    assert new.w == pytest.approx(old.width_dir)
    assert new.u1 == pytest.approx(old.axis_dir)
    assert new.u2 == pytest.approx(old.axis_dir)


def test_two_point_frame_orthogonal_normals_preserves_angle_between_axes() -> None:
    """n1⊥n2の一般ケース: wは両方の法線に直交し、u1/u2は単位直交で、u1・u2のなす角が
    n1・n2のなす角と一致するはず(共に同じwまわりの90度回転で得られるため)。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(50.0, 10.0, 30.0), normal_xyz=(1.0, 0.0, 0.0))

    frame = two_point_frame(p1, p2)

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

    frame = two_point_frame(p1, p2)

    assert _dot3(frame.w, frame.n1) == pytest.approx(0.0, abs=1e-9)
    assert _dot3(frame.w, frame.n2) == pytest.approx(0.0, abs=1e-9)
    assert _norm(frame.u1) == pytest.approx(1.0)
    assert _norm(frame.u2) == pytest.approx(1.0)


def test_two_point_frame_rejects_coincident_lateral_position_when_normals_parallel() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(0.0, 0.0, 20.0), normal_xyz=(0.0, 0.0, 1.0))  # 横方向変位ゼロ

    with pytest.raises(ValueError):
        two_point_frame(p1, p2)


def test_end_panel_corners_flat_rectangle_shape() -> None:
    origin = (5.0, 0.0, 0.0)
    u = (1.0, 0.0, 0.0)
    w = (0.0, 1.0, 0.0)

    corners = end_panel_corners(origin, u, w, near=-10.0, far=20.0, half_width=8.0)

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
    frame = two_point_frame(p1, p2)
    half_width = 15.0

    flat1_corners = end_panel_corners(p1.position_xyz, frame.u1, frame.w, near=-20.0, far=18.0, half_width=half_width)
    flat2_corners = end_panel_corners(p2.position_xyz, frame.u2, frame.w, near=-22.0, far=20.0, half_width=half_width)
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


def test_ramp_fold_angles_match_parallel_case_alpha_formula() -> None:
    """n1=n2の対称ジグザグでは、fold1・fold2の折れ角が等しく、既存のfold_tangent_length_mmが
    内部で使うalpha=atan2(offset,ramp_extent)に一致するはず(SS6.20の一般化が既存の解析式を
    特殊ケースとして含んでいることの確認)。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(50.0, 0.0, 20.0), normal_xyz=(0.0, 0.0, 1.0))
    frame = two_point_frame(p1, p2)
    old_frame = _jog_frame(p1, p2)

    x_start = 15.0
    ramp_extent = 8.0
    x_end2 = old_frame.run_length_mm - (x_start + ramp_extent)

    mid_near = tuple(p1.position_xyz[i] + x_start * frame.u1[i] for i in range(3))
    mid_far = tuple(p2.position_xyz[i] - x_end2 * frame.u2[i] for i in range(3))

    fold1_angle, fold2_angle = ramp_fold_angles_rad(mid_near, mid_far, frame.u1, frame.u2)

    expected_alpha = math.atan2(old_frame.offset_mm, ramp_extent)
    assert fold1_angle == pytest.approx(expected_alpha)
    assert fold2_angle == pytest.approx(expected_alpha)

    # tangent_length_for_bend_angle_rad経由でも既存のfold_tangent_length_mmと一致するはず
    radius = 10.0
    assert tangent_length_for_bend_angle_rad(fold1_angle, radius) == pytest.approx(
        fold_tangent_length_mm(old_frame.offset_mm, ramp_extent, radius)
    )


def test_ramp_fold_angles_sane_for_orthogonal_normals() -> None:
    """直交ケースでも、両方の折れ角が0〜180度の範囲に収まり、退化(NaN等)しないはず。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(80.0, 20.0, 50.0), normal_xyz=(1.0, 0.0, 0.0))
    frame = two_point_frame(p1, p2)

    mid_near = tuple(p1.position_xyz[i] + 25.0 * frame.u1[i] for i in range(3))
    mid_far = tuple(p2.position_xyz[i] - 25.0 * frame.u2[i] for i in range(3))

    fold1_angle, fold2_angle = ramp_fold_angles_rad(mid_near, mid_far, frame.u1, frame.u2)

    assert 0.0 <= fold1_angle <= math.pi
    assert 0.0 <= fold2_angle <= math.pi

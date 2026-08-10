import math

import pytest

from synthetic_generator.classify import (
    COPLANAR_OFFSET_TOLERANCE_MM,
    FasteningPoint,
    classify,
    fold_tangent_length_mm,
    is_strength_part,
    tangent_length_for_bend_angle_rad,
)


def test_coplanar_flat_when_same_normal_and_zero_offset() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(100.0, 30.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    assert classify(p1, p2) == "coplanar_flat"
    assert is_strength_part(classify(p1, p2)) is False


def test_parallel_same_offset_when_offset_exceeds_tolerance() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(
        position_xyz=(100.0, 30.0, COPLANAR_OFFSET_TOLERANCE_MM + 5.0), normal_xyz=(0.0, 0.0, 1.0)
    )
    assert classify(p1, p2) == "parallel_same_offset"
    assert is_strength_part(classify(p1, p2)) is True


def test_parallel_opposite_when_normals_antiparallel() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(0.0, 0.0, -20.0), normal_xyz=(0.0, 0.0, -1.0))
    assert classify(p1, p2) == "parallel_opposite"
    assert is_strength_part(classify(p1, p2)) is True


def test_orthogonal_when_normals_perpendicular() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(20.0, 0.0, 0.0), normal_xyz=(1.0, 0.0, 0.0))
    assert classify(p1, p2) == "orthogonal"
    assert is_strength_part(classify(p1, p2)) is True


def test_oblique_when_normals_at_intermediate_angle() -> None:
    angle = math.radians(45.0)
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(
        position_xyz=(30.0, 0.0, 30.0),
        normal_xyz=(math.sin(angle), 0.0, math.cos(angle)),
    )
    assert classify(p1, p2) == "oblique"
    assert is_strength_part(classify(p1, p2)) is True


def test_classify_boundary_just_inside_coplanar_tolerance() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(
        position_xyz=(0.0, 0.0, COPLANAR_OFFSET_TOLERANCE_MM - 0.01), normal_xyz=(0.0, 0.0, 1.0)
    )
    assert classify(p1, p2) == "coplanar_flat"


def test_classify_rejects_zero_length_normal() -> None:
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 0.0))
    p2 = FasteningPoint(position_xyz=(1.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        classify(p1, p2)


def test_fold_tangent_length_matches_radius_for_vertical_riser() -> None:
    """ramp_extent=0(垂直リザー、折れ角90度)のとき、接線長はRそのものに一致するはず。"""
    assert fold_tangent_length_mm(offset_mm=20.0, ramp_extent_mm=0.0, radius_mm=10.0) == pytest.approx(10.0)


def test_fold_tangent_length_shrinks_for_shallow_ramp() -> None:
    """ランプが緩やか(ramp_extentが大きい=折れ角が小さい)なほど、同じRでも接線長は
    短くなるはず(ユーザー知見: 平行に近い角度ほど剛性・応力の観点で望ましい、の裏付け)。
    """
    steep = fold_tangent_length_mm(offset_mm=20.0, ramp_extent_mm=5.0, radius_mm=10.0)
    shallow = fold_tangent_length_mm(offset_mm=20.0, ramp_extent_mm=100.0, radius_mm=10.0)
    assert shallow < steep


def test_fold_tangent_length_zero_when_radius_zero() -> None:
    assert fold_tangent_length_mm(offset_mm=20.0, ramp_extent_mm=10.0, radius_mm=0.0) == pytest.approx(0.0)


def test_tangent_length_for_bend_angle_matches_fold_tangent_length_at_90_degrees() -> None:
    """flat_flangeのflange_angle_deg=90度(垂直なフランジ)は、ジョグのalpha=90度
    (垂直リザー)と同じ意味を持つはずなので、同じ接線長になるはず。"""
    via_bend_angle = tangent_length_for_bend_angle_rad(math.radians(90.0), radius_mm=8.0)
    via_fold = fold_tangent_length_mm(offset_mm=20.0, ramp_extent_mm=0.0, radius_mm=8.0)
    assert via_bend_angle == pytest.approx(via_fold)


def test_tangent_length_for_bend_angle_zero_when_angle_zero() -> None:
    """折れ角0度(折れなし、平坦延長)なら接線長も0のはず。"""
    assert tangent_length_for_bend_angle_rad(0.0, radius_mm=10.0) == pytest.approx(0.0)

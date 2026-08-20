"""classify.py の任意法線・任意位置の一般化(two_point_frame/end_panel_corners/
ramp_fold_angles_rad、roadmap SS6.20)を検証する。CATIA非依存の純粋関数のみ。
"""

import math

import pytest

from synthetic_generator.classify import (
    SINGLE_FOLD_MAX_LATERAL_RATIO,
    FasteningPoint,
    flat_panels_clearance_mm,
    single_fold_layout,
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


# ---------------------------------------------------------------- 単曲げ経路と面干渉チェック(SS6.24)

_WASTEFUL_RAMP_ANGLES = (math.radians(120.0), math.radians(120.0))  # excess = 240-90 = 150度


def _orthogonal_pair(lateral_mm: float = 40.0):
    """n1=+Z / n2=+X の直交ペア。w=(0,1,0)方向にlateral_mmだけずらす。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(90.0, lateral_mm, 70.0), normal_xyz=(1.0, 0.0, 0.0))
    return p1, p2


def test_single_fold_panels_share_an_identical_fold_edge() -> None:
    """単曲げの核心: 幅中心を揃えれば、flat1の折れ目側の辺とflat2の折れ目側の辺が
    (幅方向オフセットがあっても)厳密に一致するはず。一致しなければjoinで単一シェルに
    ならない。"""
    p1, p2 = _orthogonal_pair(lateral_mm=40.0)
    frame = two_point_frame(p1, p2)
    layout = single_fold_layout(
        p1,
        p2,
        frame,
        bend_radius_mm=8.0,
        min_bearing_radius_mm=20.0,
        half_width_mm=25.0,
        ramp_fold_angles=_WASTEFUL_RAMP_ANGLES,
    )
    assert layout is not None

    panel1 = end_panel_corners(layout.origin1, frame.u1, frame.w, -20.0, layout.d1_mm, layout.half_width_mm)
    panel2 = end_panel_corners(layout.origin2, frame.u2, frame.w, -layout.d2_mm, 20.0, layout.half_width_mm)

    # end_panel_cornersの順序は[near,-hw],[near,hw],[far,hw],[far,-hw]
    for a, b in ((panel1[2], panel2[1]), (panel1[3], panel2[0])):
        assert a == pytest.approx(b, abs=1e-9)

    # 折れ角はu1とu2のなす角(直交法線なので90度)
    assert layout.bend_angle_rad == pytest.approx(math.pi / 2)


def test_single_fold_keeps_bearing_clearance_around_each_point() -> None:
    """幅を広げても、各締結点の幅方向の余白がbearing radiusを下回らないはず
    (広げた半幅 - 幅中心のずれ = 元のhalf_width)。"""
    half_width, bearing, lateral = 25.0, 20.0, 60.0
    p1, p2 = _orthogonal_pair(lateral_mm=lateral)
    frame = two_point_frame(p1, p2)
    layout = single_fold_layout(
        p1,
        p2,
        frame,
        bend_radius_mm=8.0,
        min_bearing_radius_mm=bearing,
        half_width_mm=half_width,
        ramp_fold_angles=_WASTEFUL_RAMP_ANGLES,
    )
    assert layout is not None
    assert layout.half_width_mm == pytest.approx(half_width + lateral / 2)

    for point, origin in ((p1, layout.origin1), (p2, layout.origin2)):
        offset_from_center = abs(_dot3(tuple(point.position_xyz[i] - origin[i] for i in range(3)), frame.w))
        assert layout.half_width_mm - offset_from_center == pytest.approx(half_width)
        assert layout.half_width_mm - offset_from_center >= bearing


def test_single_fold_declined_for_parallel_normals_and_for_modest_excess() -> None:
    """法線が平行(交線が無い=ジョグが必然)なら単曲げ不可。無駄な曲げ量が閾値未満の
    「妥当な2曲げ部品」も、多様性を失わないためあえて置き換えない(ユーザー方針)。"""
    parallel1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    parallel2 = FasteningPoint(position_xyz=(90.0, 10.0, 30.0), normal_xyz=(0.0, 0.0, 1.0))
    kwargs = dict(bend_radius_mm=8.0, min_bearing_radius_mm=20.0, half_width_mm=25.0)
    assert (
        single_fold_layout(
            parallel1,
            parallel2,
            two_point_frame(parallel1, parallel2),
            ramp_fold_angles=_WASTEFUL_RAMP_ANGLES,
            **kwargs,
        )
        is None
    )

    p1, p2 = _orthogonal_pair()
    frame = two_point_frame(p1, p2)
    modest = (math.radians(50.0), math.radians(50.0))  # excess = 100-90 = 10度 < 30度
    assert single_fold_layout(p1, p2, frame, ramp_fold_angles=modest, **kwargs) is None


def test_single_fold_declined_when_lateral_offset_too_large() -> None:
    """幅方向オフセットが大きすぎるケースは、パネルを広げすぎるので2曲げに任せる。"""
    half_width = 25.0
    p1, p2 = _orthogonal_pair(lateral_mm=SINGLE_FOLD_MAX_LATERAL_RATIO * half_width + 1.0)
    frame = two_point_frame(p1, p2)
    assert (
        single_fold_layout(
            p1,
            p2,
            frame,
            bend_radius_mm=8.0,
            min_bearing_radius_mm=20.0,
            half_width_mm=half_width,
            ramp_fold_angles=_WASTEFUL_RAMP_ANGLES,
        )
        is None
    )


def test_flat_panels_clearance_detects_crossing_and_ignores_lateral_bypass() -> None:
    """flat1(z=0のx軸上)を断面上で横切るようにflat2を配置すると距離0(=干渉)。同じ配置でも
    幅方向にすれ違っていれば3Dでは干渉しないので無限大を返すはず。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    # flat2を(40,0,-30)->(80,0,30)の線分として置く(x=60でz=0のflat1を貫く)。
    # flat2は締結点2から見て[-fold2_run, +margin]の範囲なので、+margin側の端が(80,0,30)。
    u2 = (0.5547001962252291, 0.0, 0.8320502943378437)  # normalize((40,0,60))
    margin, half_width = 20.0, 25.0
    far_end = (80.0, 0.0, 30.0)
    p2 = FasteningPoint(
        position_xyz=tuple(far_end[i] - margin * u2[i] for i in range(3)),
        normal_xyz=(-u2[2], 0.0, u2[0]),  # u2に垂直(かつw=±Yに垂直)な板厚方向
    )
    runs = dict(
        min_bearing_radius_mm=margin,
        fold1_run_mm=120.0,
        fold2_run_mm=math.dist(far_end, (40.0, 0.0, -30.0)) - margin,
    )

    frame = two_point_frame(p1, p2)
    assert flat_panels_clearance_mm(p1, p2, frame, half_width_mm=half_width, **runs) == 0.0

    # 同一の断面配置でも、幅方向(w=±Y)に2*half_width以上離れていれば干渉しない
    bypass = FasteningPoint(
        position_xyz=(p2.position_xyz[0], 60.0, p2.position_xyz[2]), normal_xyz=p2.normal_xyz
    )
    assert (
        flat_panels_clearance_mm(
            p1, bypass, two_point_frame(p1, bypass), half_width_mm=half_width, **runs
        )
        == math.inf
    )


def test_flat_panels_clearance_positive_for_a_plain_jog() -> None:
    """素直なジョグ(法線が同じで段差だけ)は干渉しないので、正の距離が返るはず。"""
    p1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    p2 = FasteningPoint(position_xyz=(120.0, 0.0, 40.0), normal_xyz=(0.0, 0.0, 1.0))
    frame = two_point_frame(p1, p2)
    clearance = flat_panels_clearance_mm(
        p1, p2, frame, min_bearing_radius_mm=20.0, fold1_run_mm=40.0, fold2_run_mm=40.0, half_width_mm=25.0
    )
    assert clearance > 0.0

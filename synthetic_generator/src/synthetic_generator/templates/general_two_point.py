"""任意の法線・任意の位置の締結点2点ペアをサンプリングする(roadmap SS6.20〜6.21)。

`parallel_same_offset.py`(法線がほぼ一致する`parallel_same_offset`クラス専用)を
一般化したもの: 法線n1・n2は完全にランダムな相対角度(0〜180度)を取り、締結点2の
位置も締結点1から見て完全にランダムな3D方向・距離を取る。`classify()`による分類は
5クラスいずれになってもよい(`coplanar_flat`のみ、締結点が同一平面上で強度部品扱いに
ならないクラスだが、メイン形状生成自体はこのモジュールの対象外にしない — 剛性方向の
補強が要らないだけで、面を繋ぐこと自体は引き続き必要なため)。

フランジ・Phase 1.5トリムは未対応(`include_flanges`を持たない`build_general_two_point`
専用、SS6.20参照)。半径R>=4mmの必須制約・締結点周りのbearing radius保護・fold1/fold2の
フィレット同士のランプ上での重なり回避は、`parallel_same_offset.py`と同じ
「実行可能範囲を計算してからその中でサンプリングする」方針を、独立な2つの折れ角
(fold1_run_mm/fold2_run_mmが対称でなくなったため)に一般化して適用する。
"""

from __future__ import annotations

import dataclasses
import math
import random

from synthetic_generator.classify import (
    MIN_NEUTRAL_PLANE_RADIUS_MM,
    FasteningPoint,
    Vec3,
    end_panel_corners,
    ramp_fold_angles_rad,
    tangent_length_for_bend_angle_rad,
    two_point_frame,
)

MIN_BEARING_RADIUS_RANGE_MM = (15.0, 30.0)  # parallel_same_offset.pyと同じ想定レンジ
HALF_WIDTH_MARGIN_RATIO_RANGE = (1.1, 1.5)  # bearing_radiusに対する幅方向の余裕
BEND_RADIUS_CAP_MM = 50.0  # ユーザー知見: 小部品でもR40〜50まであり得る、の上限(parallel_same_offset.pyと同じ)

# 締結点からランプ側の折れ目までの距離(fold1_run_mm/fold2_run_mm)の初期サンプリング幅。
# 下限はbearing_radius+FOLD_RUN_SLACK_MIN_MM(フィレットの後退量を吸収する最低限の余地)、
# 上限はbearing_radius+FOLD_RUN_SLACK_MAX_MMとする(bend_radius_mmは後段でこの幾何から
# 逆算した実行可能上限内でサンプリングするため、ここでの下限は「必ず成立する」ことより
# 「妥当な範囲」であることが目的)。
FOLD_RUN_SLACK_MIN_MM = 10.0
FOLD_RUN_SLACK_MAX_MM = 80.0

# 締結点間の距離(|p2-p1|)のサンプリング範囲。任意の3D方向。
OFFSET_DISTANCE_RANGE_MM = (60.0, 200.0)


@dataclasses.dataclass(frozen=True)
class GeneralTwoJointSpec:
    point1: FasteningPoint
    point2: FasteningPoint
    thickness_mm: float
    hole_diameter_mm: float
    min_bearing_radius_mm: float
    half_width_mm: float
    fold1_run_mm: float  # 締結点1からランプ側の折れ目(fold1)までの距離
    fold2_run_mm: float  # 締結点2からランプ側の折れ目(fold2)までの距離
    bend_radius_mm: float  # fold1・fold2共通の単一半径


def _random_unit_vector(rng: random.Random) -> Vec3:
    z = rng.uniform(-1.0, 1.0)
    theta = rng.uniform(0.0, 2 * math.pi)
    r = math.sqrt(max(0.0, 1.0 - z * z))
    return (r * math.cos(theta), r * math.sin(theta), z)


def _rotate_about_axis(v: Vec3, axis: Vec3, angle_rad: float) -> Vec3:
    """ロドリゲスの回転公式でvをaxis(単位ベクトル)まわりにangle_rad回転する。"""
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    dot = sum(v[i] * axis[i] for i in range(3))
    cross = (
        axis[1] * v[2] - axis[2] * v[1],
        axis[2] * v[0] - axis[0] * v[2],
        axis[0] * v[1] - axis[1] * v[0],
    )
    return tuple(v[i] * cos_a + cross[i] * sin_a + axis[i] * dot * (1 - cos_a) for i in range(3))


def sample(
    rng: random.Random,
    *,
    thickness_range_mm: tuple[float, float] = (1.0, 2.5),
    hole_diameter_range_mm: tuple[float, float] = (6.0, 14.0),
) -> GeneralTwoJointSpec:
    """任意の法線・任意の位置の締結点ペアを1組サンプリングする。"""
    n1 = _random_unit_vector(rng)
    rotation_axis = _random_unit_vector(rng)
    rotation_angle = rng.uniform(0.0, math.pi)  # 0〜180度、全configuration classをカバー
    n2 = _rotate_about_axis(n1, rotation_axis, rotation_angle)

    p1: Vec3 = (0.0, 0.0, 0.0)
    offset_distance = rng.uniform(*OFFSET_DISTANCE_RANGE_MM)
    offset_dir = _random_unit_vector(rng)
    p2: Vec3 = tuple(p1[i] + offset_distance * offset_dir[i] for i in range(3))

    point1 = FasteningPoint(position_xyz=p1, normal_xyz=n1)
    point2 = FasteningPoint(position_xyz=p2, normal_xyz=n2)

    # two_point_frameが失敗するのは法線が完全に平行かつ横方向変位もゼロという
    # 測度ゼロの縮退ケースのみ(連続一様分布からの乱数サンプリングでは実質発生しない)。
    frame = two_point_frame(point1, point2)

    thickness = rng.uniform(*thickness_range_mm)
    hole_diameter = rng.uniform(*hole_diameter_range_mm)
    bearing_radius = rng.uniform(*MIN_BEARING_RADIUS_RANGE_MM)
    half_width = bearing_radius * rng.uniform(*HALF_WIDTH_MARGIN_RATIO_RANGE)

    fold_run_min = bearing_radius + FOLD_RUN_SLACK_MIN_MM
    fold_run_max = bearing_radius + FOLD_RUN_SLACK_MAX_MM
    fold1_run = rng.uniform(fold_run_min, fold_run_max)
    fold2_run = rng.uniform(fold_run_min, fold_run_max)

    # fold1_run/fold2_runが決まって初めてランプの実際の幾何(fold1・fold2の折れ角)が
    # 定まる(parallel_same_offsetのoffsetのような単純な閉じた式は一般ケースには無いため、
    # 実際にflat1/flat2のランプ側辺を計算してramp_fold_angles_radに渡す)。
    flat1_corners = end_panel_corners(p1, frame.u1, frame.w, -bearing_radius, fold1_run, half_width)
    flat2_corners = end_panel_corners(p2, frame.u2, frame.w, -fold2_run, bearing_radius, half_width)
    mid_near = tuple((flat1_corners[2][i] + flat1_corners[3][i]) / 2 for i in range(3))
    mid_far = tuple((flat2_corners[0][i] + flat2_corners[1][i]) / 2 for i in range(3))
    fold1_angle, fold2_angle = ramp_fold_angles_rad(mid_near, mid_far, frame.u1, frame.u2)
    ramp_length = math.sqrt(sum((mid_far[i] - mid_near[i]) ** 2 for i in range(3)))

    # bend_radius_mmは、両方のfoldのフィレット後退(tangent length)が
    # (a) それぞれの締結点のbearing_radius分の余白を侵さない、
    # (b) 2つの後退量の合計がランプ自体の長さを超えない(SS6.16のジョグ重なり教訓の一般化)、
    # (c) パネル幅の0.9倍を超えない、
    # を全て満たす実行可能上限の範囲内でサンプリングする(parallel_same_offset.pyと同じ
    # 「実行可能範囲を計算してからサンプリングする」方針)。
    unit_tangent1 = tangent_length_for_bend_angle_rad(fold1_angle, 1.0)
    unit_tangent2 = tangent_length_for_bend_angle_rad(fold2_angle, 1.0)

    max_from_fold1 = (
        BEND_RADIUS_CAP_MM
        if unit_tangent1 < 1e-9
        else max(0.0, fold1_run - bearing_radius) / unit_tangent1
    )
    max_from_fold2 = (
        BEND_RADIUS_CAP_MM
        if unit_tangent2 < 1e-9
        else max(0.0, fold2_run - bearing_radius) / unit_tangent2
    )
    tangent_sum_per_radius = unit_tangent1 + unit_tangent2
    max_from_ramp_overlap = (
        BEND_RADIUS_CAP_MM if tangent_sum_per_radius < 1e-9 else ramp_length / tangent_sum_per_radius
    )

    max_bend_radius = min(BEND_RADIUS_CAP_MM, max_from_fold1, max_from_fold2, max_from_ramp_overlap, 0.9 * half_width)
    # ユーザー製造制約: 中立面Rは全て最小R4を守る。max_bend_radiusが4mm未満(=この組み合わせ
    # ではR4以上のフィレットが物理的に成立しない)場合はあえて4mmを引いておき、
    # build_general_two_point側の事前チェックで明示的にInfeasibleとして弾かれるようにする
    # (parallel_same_offset.pyと同じ「失敗したらアボートでいい」設計)。
    bend_radius = rng.uniform(MIN_NEUTRAL_PLANE_RADIUS_MM, max(MIN_NEUTRAL_PLANE_RADIUS_MM, max_bend_radius))

    return GeneralTwoJointSpec(
        point1=point1,
        point2=point2,
        thickness_mm=thickness,
        hole_diameter_mm=hole_diameter,
        min_bearing_radius_mm=bearing_radius,
        half_width_mm=half_width,
        fold1_run_mm=fold1_run,
        fold2_run_mm=fold2_run,
        bend_radius_mm=bend_radius,
    )

"""2締結点のconfiguration classを法線関係+同一平面判定で分類する。

閾値は docs/synthetic_two_joint_generation_roadmap.md SS3.1 で実データ
(3アセンブリ、31,846組の締結点ペア)から検証済み: 内積の自然なギャップが
+-0.9 / +-0.15 付近にあり、parallel_same 内の同一平面クラスタは
オフセット 1.0mm 未満に集中する。
"""

from __future__ import annotations

import dataclasses
import math
from typing import Literal

ConfigClass = Literal[
    "coplanar_flat",
    "parallel_same_offset",
    "parallel_opposite",
    "orthogonal",
    "oblique",
]

Vec3 = tuple[float, float, float]

DOT_PARALLEL_THRESHOLD = 0.9
DOT_ORTHOGONAL_THRESHOLD = 0.15
COPLANAR_OFFSET_TOLERANCE_MM = 1.0

# coplanar_flat のみが非強度部品(docs SS3: 「強度部品でないのは締結点が同一平面の場合だけ」)
NON_STRENGTH_CLASSES: frozenset[ConfigClass] = frozenset({"coplanar_flat"})

# ユーザー製造制約(2026-08-06): 中立面Rの最小値。ジョグの折れ目・フランジ根本の折れ目を
# 含む全てのフィレット半径に適用する。templates/parallel_same_offset.pyとreinforcement.py
# の両方が参照するため、どちらにも依存しないこのモジュールに置く。
MIN_NEUTRAL_PLANE_RADIUS_MM = 4.0


def _normalize(v: Vec3) -> Vec3:
    length = math.sqrt(sum(c * c for c in v))
    if length < 1e-9:
        raise ValueError(f"cannot normalize near-zero vector: {v}")
    return (v[0] / length, v[1] / length, v[2] / length)


def _dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


@dataclasses.dataclass(frozen=True)
class FasteningPoint:
    """締結点1つ分。normal_xyz は joints.json の axis.direction_xyz(板厚方向)に対応。"""

    position_xyz: Vec3
    normal_xyz: Vec3


def classify(point1: FasteningPoint, point2: FasteningPoint) -> ConfigClass:
    n1 = _normalize(point1.normal_xyz)
    n2 = _normalize(point2.normal_xyz)
    d = _dot(n1, n2)

    if d > DOT_PARALLEL_THRESHOLD:
        offset_mm = abs(_dot(_sub(point2.position_xyz, point1.position_xyz), n1))
        if offset_mm < COPLANAR_OFFSET_TOLERANCE_MM:
            return "coplanar_flat"
        return "parallel_same_offset"
    if d < -DOT_PARALLEL_THRESHOLD:
        return "parallel_opposite"
    if -DOT_ORTHOGONAL_THRESHOLD < d < DOT_ORTHOGONAL_THRESHOLD:
        return "orthogonal"
    return "oblique"


def is_strength_part(config_class: ConfigClass) -> bool:
    return config_class not in NON_STRENGTH_CLASSES


def tangent_length_for_bend_angle_rad(bend_angle_rad: float, radius_mm: float) -> float:
    """ある折れ目(外向きの曲がり角 bend_angle_rad)にradius_mmのエッジフィレットを付けたとき、
    理論上のシャープな角から実際にフィレットが始まる点までの距離(接線長)。

    bend_angle_rad=90度(垂直な折れ)のとき接線長はradius_mmそのものに一致する。
    ジョグの折れ(fold_tangent_length_mm)にもフランジ根本の折れ(flange_angle_degが
    そのままbend_angleに対応する — flat_flangeのdirection = sin(angle)*normal +
    cos(angle)*outward_tangentという定義上、angle=0は折れなしの平坦延長、angle=90度は
    垂直な折れで、ジョグのalpha = atan2(offset, ramp_extent)と同じ意味を持つ)にも
    同じ式を使う。
    """
    return radius_mm * math.tan(bend_angle_rad / 2.0)


def fold_tangent_length_mm(offset_mm: float, ramp_extent_mm: float, radius_mm: float) -> float:
    """ジョグの折れ目(flat-ランプ間)にradius_mmのエッジフィレットを付けたときの接線長。
    ランプの傾き角(水平からの角度)を alpha = atan2(offset_mm, ramp_extent_mm) として
    tangent_length_for_bend_angle_rad(alpha, radius_mm) を返す。

    templates/parallel_same_offset.py(締結点の必要最小半径を侵さない範囲でbend_radius_mm
    を実行可能サンプリングするため)とgsd_build.py(実際の構築前の最終安全確認)の両方から
    同じ式を使う必要があるため、どちらにも依存しないこのモジュールに置いている。
    """
    alpha = math.atan2(offset_mm, ramp_extent_mm)
    return tangent_length_for_bend_angle_rad(alpha, radius_mm)


# ---------------------------------------------------------------- 任意法線・任意位置への一般化(2026-08-10)
#
# roadmap SS6.20参照。元gsd_build.pyにあったが、templates/general_two_point.py(サンプリング)
# とgsd_build.py(実際の構築)の両方から同じ式を使う必要があるため、循環importを避けて
# ここに置く(fold_tangent_length_mm等と同じ理由)。_jog_frameは締結点1の法線nを全体の
# 基準にしており、n1≈n2(parallel_sameクラス)でしか正しくない。一般化のコアは
# 「全ての折れ目を単一の共通方向wに平行にする」制約を保ったまま、flat1(法線n1)側と
# flat2(法線n2)側でそれぞれ別のローカル走行方向(u1/u2)を持たせること — n1=n2のとき
# _jog_frameと数値的に一致することをテストで確認済み(u1=u2=axis_dir、w=width_dirに
# 厳密に退化する)。


@dataclasses.dataclass(frozen=True)
class TwoPointFrame:
    w: Vec3  # 全ての折れ目に共通な方向(幅方向)。n1・n2両方に直交する
    u1: Vec3  # 締結点1でのローカル走行方向(締結点2の方へ向く単位ベクトル)
    u2: Vec3  # 締結点2でのローカル走行方向(u1と同じ「順方向」の意味、締結点1から見て奥へ向く)
    n1: Vec3
    n2: Vec3


def two_point_frame(point1: FasteningPoint, point2: FasteningPoint) -> TwoPointFrame:
    """任意の法線・任意の位置の2締結点から、共通幅方向wと各点のローカル走行方向u1/u2を求める。

    wは「flat1(法線n1)にもflat2(法線n2)にも直線の折れ目を許す」ために両方の法線に
    直交する必要がある。n1とn2が平行でなければ`w = normalize(n1 x n2)`の1つ(符号除く)に
    一意に決まる。n1 ∥ n2(parallel_sameクラス。反平行のparallel_oppositeクラスも
    n1×n2=0で同じ分岐に入る)のときは、_jog_frameと同じフォールバック(締結点間の接平面内
    変位方向を使う)で決める。両方退化する場合(法線一致かつ横方向変位ゼロ)は
    _jog_frameと同じくInfeasible(ValueError)。

    u1/u2の符号は、両方とも「締結点1→2への変位ベクトルと正の内積を持つ」側を選ぶことで、
    (符号が反転しうる外積由来のベクトルに対して)一貫した「順方向」を与える — n1=n2の
    退化ケースでは、この符号解決を経てu1=u2=_jog_frameのaxis_dirに厳密に一致する
    (テストで確認済み)。
    """
    n1 = _normalize(point1.normal_xyz)
    n2 = _normalize(point2.normal_xyz)
    delta = _sub(point2.position_xyz, point1.position_xyz)

    cross = _cross(n1, n2)
    cross_len = math.sqrt(sum(c * c for c in cross))
    if cross_len > 1e-9:
        w = tuple(c / cross_len for c in cross)
    else:
        offset = _dot(delta, n1)
        tangential = tuple(delta[i] - offset * n1[i] for i in range(3))
        tangential_len = math.sqrt(sum(c * c for c in tangential))
        if tangential_len < 1e-6:
            raise ValueError("point1/point2 must be laterally separated when normals are parallel")
        axis_fallback = tuple(c / tangential_len for c in tangential)
        w = _normalize(_cross(n1, axis_fallback))

    def _resolve_sign(u_raw: Vec3) -> Vec3:
        return u_raw if _dot(u_raw, delta) >= 0 else tuple(-c for c in u_raw)

    u1 = _resolve_sign(_cross(w, n1))
    u2 = _resolve_sign(_cross(w, n2))

    return TwoPointFrame(w=w, u1=u1, u2=u2, n1=n1, n2=n2)


def end_panel_corners(origin: Vec3, u: Vec3, w: Vec3, near: float, far: float, half_width: float) -> list[Vec3]:
    """origin中心のローカル(u,w)フレームで、走行方向near〜far・幅方向±half_widthの
    平坦矩形の4隅を返す(_panel_cornersのheight0=height1=0特殊形に相当、n方向成分は
    常に0=originが乗る平面上)。順序は[near,-hw],[near,hw],[far,hw],[far,-hw]
    (_panel_cornersと同じ規約)。
    """

    def pt(run: float, width: float) -> Vec3:
        return tuple(origin[i] + run * u[i] + width * w[i] for i in range(3))

    return [pt(near, -half_width), pt(near, half_width), pt(far, half_width), pt(far, -half_width)]


def ramp_fold_angles_rad(mid_near: Vec3, mid_far: Vec3, u1: Vec3, u2: Vec3) -> tuple[float, float]:
    """ランプ中心線(mid_near->mid_far)とflat1/flat2それぞれのローカル走行方向(u1/u2)との
    なす角[ラジアン]を返す(fold1の折れ角, fold2の折れ角)。

    tangent_length_for_bend_angle_rad(このモジュール)にそのまま渡せる「外向きの曲がり角」の
    定義に合わせている: 折れなし(flat1の延長線上にランプがある)なら0、垂直な折れなら
    90度。n1=n2(平行ケース)のときu1=u2なので両方の角度が等しくなり、既存の
    fold_tangent_length_mm(offset,ramp_extent,R)が返すalpha=atan2(offset,ramp_extent)と
    厳密に一致する(テストで確認済み) — 対称なジグザグという特殊ケースを含む一般化になっている。
    """

    def _angle(a: Vec3, b: Vec3) -> float:
        d = max(-1.0, min(1.0, _dot(a, b)))
        return math.acos(d)

    ramp_dir = _normalize(_sub(mid_far, mid_near))
    return _angle(u1, ramp_dir), _angle(u2, ramp_dir)

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


# ---------------------------------------------------------------- 単曲げ経路と面干渉チェック(2026-08-21)
#
# roadmap SS6.24参照。build_general_two_pointは元々「flat1+ランプ+flat2」の3ピース固定で、
# 折れ目が必ず2本できる(法線が平行なジョグ由来の構成)。法線が交わる一般ケースでは
# 2平面の交線で1回曲げれば足りるため、「無駄な曲げ量」が大きいケースだけ単曲げに置き換える。

# 無駄な曲げ量 = (fold1の折れ角 + fold2の折れ角) - 正味の折れ角θ。0なら2つの折れが同方向に
# θを分担しているだけ(妥当な2曲げ部品)、大きいほど行き過ぎて戻るS字=明らかに余計な曲げ。
# 30度は3000サンプルの実測(単曲げ可能ケースのexcess中央値43度)から、単曲げ可能ケースの
# 37%だけを置き換える保守的な水準として選んだ較正値(ユーザー判断、2026-08-21)。
SINGLE_FOLD_MIN_EXCESS_RAD = math.radians(30.0)

# 単曲げでは2枚のパネルが1本の折れ目を共有するため、幅方向オフセットをランプの台形で
# 吸収できない(2曲げ経路との構造的な差)。両パネルの幅を half_width + |dw|/2 に広げて
# 吸収するので、広げすぎないよう |dw| に上限を置く(3.0倍 = 幅の膨張は2.5倍以内)。
SINGLE_FOLD_MAX_LATERAL_RATIO = 3.0

# 断面上でflat1とflat2がこれ以下まで近づいたら干渉扱い(実測: 完全交差6.2%に加えて
# 5mm未満のニアミスが1.6%)。板厚(最大2.5mm)の両側分を見込んだ値。
MIN_PANEL_CLEARANCE_MM = 5.0


@dataclasses.dataclass(frozen=True)
class SingleFoldLayout:
    """2平面の交線で1回だけ曲げる構成。two_point_frameのu1/u2/wと組で使う。"""

    origin1: Vec3  # flat1のローカル原点(締結点1を共通の幅中心cmまでw方向にずらした点)
    origin2: Vec3  # flat2のローカル原点(同上)
    d1_mm: float  # 締結点1から折れ目までの距離(u1方向)
    d2_mm: float  # 折れ目から締結点2までの距離(u2方向)
    bend_angle_rad: float  # 折れ目での外向きの曲がり角(=u1とu2のなす角)
    half_width_mm: float  # 幅方向オフセットを吸収するため広げた後の半幅


def single_fold_layout(
    point1: FasteningPoint,
    point2: FasteningPoint,
    frame: TwoPointFrame,
    *,
    bend_radius_mm: float,
    min_bearing_radius_mm: float,
    half_width_mm: float,
    ramp_fold_angles: tuple[float, float],
) -> SingleFoldLayout | None:
    """単曲げで作るべきケースならそのレイアウトを、そうでなければNoneを返す。

    Noneを返す条件: 法線が平行で交線が存在しない / 交線が締結点の後方にある /
    折れ目のフィレットがbearing radiusを侵す / 幅方向オフセットが大きすぎる /
    2曲げでの無駄な曲げ量がSINGLE_FOLD_MIN_EXCESS_RADに満たない(=妥当な2曲げ部品)。

    交線Lは必ずwに平行なので、両パネルをw方向に共通の幅中心cmまでずらせば、flat1の
    折れ目側の辺とflat2の折れ目側の辺は厳密に一致する(テストで確認済み)。
    """
    n1, n2, u1, u2, w = frame.n1, frame.n2, frame.u1, frame.u2, frame.w
    if abs(abs(_dot(n1, n2)) - 1.0) < 1e-9:
        return None  # 平行な2平面は交わらない(ジョグが必然)

    bend_angle = math.acos(max(-1.0, min(1.0, _dot(u1, u2))))
    excess = ramp_fold_angles[0] + ramp_fold_angles[1] - bend_angle
    if excess < SINGLE_FOLD_MIN_EXCESS_RAD:
        return None  # 2つの折れが素直にθを分担している = 妥当な2曲げ部品なので触らない

    delta = _sub(point2.position_xyz, point1.position_xyz)
    lateral = _dot(delta, w)
    if abs(lateral) > SINGLE_FOLD_MAX_LATERAL_RATIO * half_width_mm:
        return None

    # 交線: 締結点1からu1方向にd1進んだ点がflat2の平面にも乗る条件を解く
    # (flat1の平面上を動く限りn1成分は0のままなので、1次元の線形方程式になる)
    denominator = _dot(u1, n2)
    if abs(denominator) < 1e-9:
        return None
    d1 = _dot(delta, n2) / denominator
    fold_point = tuple(point1.position_xyz[i] + d1 * u1[i] for i in range(3))
    d2 = _dot(_sub(point2.position_xyz, fold_point), u2)

    tangent = tangent_length_for_bend_angle_rad(bend_angle, bend_radius_mm)
    if d1 - tangent < min_bearing_radius_mm or d2 - tangent < min_bearing_radius_mm:
        return None

    widened = half_width_mm + abs(lateral) / 2.0
    shift1 = lateral / 2.0  # 締結点1から共通幅中心cmまでのw方向のずれ
    shift2 = -lateral / 2.0
    return SingleFoldLayout(
        origin1=tuple(point1.position_xyz[i] + shift1 * w[i] for i in range(3)),
        origin2=tuple(point2.position_xyz[i] + shift2 * w[i] for i in range(3)),
        d1_mm=d1,
        d2_mm=d2,
        bend_angle_rad=bend_angle,
        half_width_mm=widened,
    )


def _segment_distance_2d(
    a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]
) -> float:
    """2D線分ab-cd間の最小距離(交差していれば0)。"""

    def cross(o, p, q) -> float:
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])

    d1, d2 = cross(c, d, a), cross(c, d, b)
    d3, d4 = cross(a, b, c), cross(a, b, d)
    if (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0):
        return 0.0

    def point_to_segment(p, s, e) -> float:
        se = (e[0] - s[0], e[1] - s[1])
        length_sq = se[0] * se[0] + se[1] * se[1]
        t = 0.0 if length_sq < 1e-12 else ((p[0] - s[0]) * se[0] + (p[1] - s[1]) * se[1]) / length_sq
        t = max(0.0, min(1.0, t))
        return math.hypot(p[0] - s[0] - t * se[0], p[1] - s[1] - t * se[1])

    return min(
        point_to_segment(a, c, d),
        point_to_segment(b, c, d),
        point_to_segment(c, a, b),
        point_to_segment(d, a, b),
    )


def flat_panels_clearance_mm(
    point1: FasteningPoint,
    point2: FasteningPoint,
    frame: TwoPointFrame,
    *,
    min_bearing_radius_mm: float,
    fold1_run_mm: float,
    fold2_run_mm: float,
    half_width_mm: float,
) -> float:
    """2曲げ構成でflat1とflat2が干渉していないかを、wに垂直な断面上の距離で測る。

    折れ目は全てwに平行なので、断面(u1,n1平面)ではflat1-ランプ-flat2が3本の線分になる。
    隣接する2組は折れ目を共有しているだけだが、非隣接のflat1-flat2は交差しうる —
    既存の事前チェック(R下限・接線長のbearing侵食・ランプ上のフィレット重なり)は
    どれもこれを見ておらず、実測で6.2%が実際に面同士で干渉していた(roadmap SS6.24)。

    幅方向にすれ違っている(|dw| >= 2*half_width)場合は断面で交差していても3Dでは
    干渉しないため、無限大を返す(実測でこの過剰判定が全交差517件中333件を占めた)。
    """
    lateral = abs(_dot(_sub(point2.position_xyz, point1.position_xyz), frame.w))
    if lateral >= 2 * half_width_mm:
        return math.inf

    def project(p: Vec3) -> tuple[float, float]:
        rel = _sub(p, point1.position_xyz)
        return (_dot(rel, frame.u1), _dot(rel, frame.n1))

    def along(origin: Vec3, u: Vec3, run: float) -> Vec3:
        return tuple(origin[i] + run * u[i] for i in range(3))

    p1, p2 = point1.position_xyz, point2.position_xyz
    flat1 = (project(along(p1, frame.u1, -min_bearing_radius_mm)), project(along(p1, frame.u1, fold1_run_mm)))
    flat2 = (project(along(p2, frame.u2, -fold2_run_mm)), project(along(p2, frame.u2, min_bearing_radius_mm)))
    return _segment_distance_2d(*flat1, *flat2)

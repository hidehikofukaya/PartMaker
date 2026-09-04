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

# ユーザー製造制約: 中立面Rの最小値。ジョグの折れ目・フランジ根本の折れ目・ビードの
# 稜線/コーナーを含む**全ての**フィレット半径に適用する。templates/*.pyとreinforcement.py、
# bead.pyが参照するため、どれにも依存しないこのモジュールに置く。
# 2026-08-06にR4として導入、2026-08-24にユーザー指示でR5へ引き上げ。
MIN_NEUTRAL_PLANE_RADIUS_MM = 5.0

# ユーザー確定(2026-08-24): 基準面(メイン形状の曲げ)のRは慣例的に10mm以上を狙う。
# 設計基準ではないが実務上の慣例。ビードの稜線/コーナー等はこれではなく
# MIN_NEUTRAL_PLANE_RADIUS_MM(全フィレット共通の最小R)に従う。
MIN_BASE_BEND_RADIUS_MM = 10.0


def _normalize(v: Vec3) -> Vec3:
    length = math.sqrt(sum(c * c for c in v))
    if length < 1e-9:
        raise ValueError(f"cannot normalize near-zero vector: {v}")
    return (v[0] / length, v[1] / length, v[2] / length)


def _dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: Vec3, k: float) -> Vec3:
    return (v[0] * k, v[1] * k, v[2] * k)


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


def end_panel_corners(origin: Vec3, u: Vec3, w: Vec3, near: float, far: float, half_width: float,
                      half_width_neg: float | None = None) -> list[Vec3]:
    """origin中心のローカル(u,w)フレームで、走行方向near〜far・幅方向±half_widthの
    平坦矩形の4隅を返す(_panel_cornersのheight0=height1=0特殊形に相当、n方向成分は
    常に0=originが乗る平面上)。順序は[near,-hw],[near,hw],[far,hw],[far,-hw]
    (_panel_cornersと同じ規約)。
    """

    def pt(run: float, width: float) -> Vec3:
        return tuple(origin[i] + run * u[i] + width * w[i] for i in range(3))

    neg = half_width_neg if half_width_neg is not None else half_width
    return [pt(near, -neg), pt(near, half_width), pt(far, half_width), pt(far, -neg)]


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

# 傾いた折れ目で台形になったパネルが、幅方向の端で潰れない(自己交差しない)ための
# 最小走行長。幅端では走行長が half_width*|tan(far_tilt)-tan(near_tilt)| だけ削られる。
MIN_SHEARED_PANEL_SPAN_MM = 2.0

# 同上を比率でも要求する。絶対値だけだと「幅端で3mm・反対の端で40mm」のような
# 極端なスリバーが通ってしまい、基準面が細長く歪んで互いに交差する。その上には
# ビードを正しく置けず、ドラフト角の判定が全滅する(2026-08-25、失敗形状の目視で確認。
# 実測でシアー比率の最小値は-49.4=公称走行長の50倍のシアー)。
MIN_SHEARED_PANEL_SPAN_RATIO = 0.5


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
    min_excess_rad: float | None = None,
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
    # `min_excess_rad`は「2曲げの救済としてのみ単曲げを使う」ゲート。単曲げを
    # **設計の選択肢として狙う**場合(target_folds=1、2026-09-04)は0以下を渡して外す。
    threshold = SINGLE_FOLD_MIN_EXCESS_RAD if min_excess_rad is None else min_excess_rad
    if excess < threshold:
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


# ---------------------------------------------------------------- 自由折れ目チェーン(2026-08-24)
#
# docs/catia_bead_fillet_investigation_log.md SS8参照。two_point_frameのw平行制約
# (全ての折れ目がn1×n2に平行)を緩め、中間折れ目1本を「任意方向に傾けられる」ようにした
# 一般化。p1-panel1-fold1-panel_mid-fold2-panel3-p2の3パネル構成で、各折れ目の傾き
# (a1/a2、パネル固有の幅方向vに対する折れ目軸の角度)にランダム性を持たせることで
# 締結点ペアごとに複数の形状(3パラメータ族: a1, a2, および展開全長に対応する自由度)が
# 作れる(ユーザー指示、2026-08-24)。
#
# SS8.7で証明済み: 中心線が全折れ目に垂直(gamma=0)な構成は、締結点の幅方向ズレ
# dw=dot(p2-p1, n1xn2)が非ゼロである限り存在しない(折れ目の枚数・方向によらず)。
# したがって帯が折れ目を斜めに横切ること自体は避けられない設計上の性質であり、
# 折れ目方向を自由にする利点はgammaの削減ではなく形状多様性の拡大にある。

# ユーザー確定(2026-08-24): 1つの折れ目の曲げ角上限は135度(SS8.4実測: 135度が
# 頭打ち点、150度にしても総合成立率が改善しない)。free_fold_seedの既定値と、
# gsd_build.pyがsolve_free_fold後の実際の曲げ角(ホモトピーで動きうる)を
# 事後チェックする際の両方で使う共通定数。
MAX_FOLD_ANGLE_DEG = 135.0

# ユーザー承認(2026-08-25): 折れ目の傾き(a1/a2)の上限。傾きは締結点の幅方向ズレを
# 吸収するために導入したが、無制限だとパネルが激しくシアーした平行四辺形になり、
# ねじれたリボンや先端が尖った形状という板金として成立しないものが量産される
# (実測: 傾きは中央値29.4度・p90で59.2度、tan(59度)=1.66なので半幅25mmのパネルは
# 走行方向に±41mmずれる)。実測のトレードオフ:
#   上限15/20/25/30/40/60度 -> 通過率14.6/21.4/27.9/34.4/45.8/55.9%、
#   シアー比>=0.8(ほぼ長方形)の割合 100/100/97/89/71/58%
# 品質と歩留まりの釣り合いで30度を採る。
MAX_FOLD_TILT_DEG = 5.0


def rotate_about_axis(v: Vec3, axis: Vec3, angle_rad: float) -> Vec3:
    """ロドリゲスの回転公式でv(任意ベクトル)をaxis(単位ベクトル)まわりにangle_rad回転する。

    templates/general_two_point.pyの乱数生成用と用途は異なるが式は同じなので、
    自由折れ目チェーンの構築(法線・走行方向を折れ目軸まわりに回す)にはこちらを使う。
    """
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    dot = _dot(v, axis)
    cross = _cross(axis, v)
    return tuple(v[i] * cos_a + cross[i] * sin_a + axis[i] * dot * (1 - cos_a) for i in range(3))


def _angle_between(a: Vec3, b: Vec3) -> float:
    return math.acos(max(-1.0, min(1.0, _dot(a, b))))


def _solve_linear(matrix: list[list[float]], rhs: list[float]) -> list[float] | None:
    """n元連立1次方程式を部分ピボット付きガウス消去法で解く(自由折れ目チェーンの
    ニュートン法で使う数値ヤコビアンの求解専用。5x5程度の小規模行列のみを想定)。
    特異に近い場合はNoneを返す(呼び出し側でダンピング等の対処に回す)。
    """
    n = len(rhs)
    aug = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot_row = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot_row][col]) < 1e-14:
            return None
        aug[col], aug[pivot_row] = aug[pivot_row], aug[col]
        pivot = aug[col][col]
        aug[col] = [x / pivot for x in aug[col]]
        for r in range(n):
            if r == col:
                continue
            factor = aug[r][col]
            if factor != 0.0:
                aug[r] = [aug[r][k] - factor * aug[col][k] for k in range(n + 1)]
    return [aug[i][n] for i in range(n)]


@dataclasses.dataclass(frozen=True)
class ChainPanelFrame:
    """自由折れ目チェーンの1パネル分のローカルフレーム。originはこのパネルの走行座標
    run=0の点(パネル1ならp1、パネル中間・パネル3なら手前側の折れ目点)。
    """

    origin: Vec3
    u: Vec3  # 走行方向(単位ベクトル)
    n: Vec3  # 面法線(単位ベクトル)

    @property
    def v(self) -> Vec3:
        """幅方向(単位ベクトル) = n x u。(u,v,n)はこの順で右手系の正規直交基底になる
        (u x v = n)。折れ目軸g(a) = cos(a)*v + sin(a)*u の基準。"""
        return _cross(self.n, self.u)


@dataclasses.dataclass(frozen=True)
class FreeFoldChain:
    """p1-panel1-fold1-panel_mid-fold2-panel3-p2 の3パネル構成。

    fold1/fold2の位置はpanel_mid.origin/panel3.originそのもの(=中心線が折れ目を
    横切る点)。中心線(ビード中心線にもなる、SS8.6)は p1 -> panel_mid.origin ->
    panel3.origin -> p2 の3線分ポリラインで、chain構築時の閉合条件により
    panel3.origin + L3_mm*panel3.u は厳密に point2.position_xyz に一致する
    (`solve_free_fold`のニュートン法収束条件そのもの)。
    """

    panel1: ChainPanelFrame
    panel_mid: ChainPanelFrame
    panel3: ChainPanelFrame
    L1_mm: float  # panel1: p1からfold1までの走行距離
    L2_mm: float  # panel_mid: fold1からfold2までの走行距離
    L3_mm: float  # panel3: fold2からp2までの走行距離
    a1_rad: float  # fold1の傾き(panel1.vに対するfold1の折れ目軸の角度。0ならv平行=旧w平行版)
    a2_rad: float  # fold2の傾き(panel_mid.vに対する角度)

    @property
    def fold1_angle_rad(self) -> float:
        """fold1の二面角(=外向きの曲がり角、tangent_length_for_bend_angle_radにそのまま渡せる)。
        回転軸(折れ目軸)は常にpanel1.n・panel_mid.nの両方に直交するため、この角度は
        a1_radの値によらずpanel1.nとpanel_mid.nのなす角に厳密に一致する(SS8.8で証明・検証済み:
        折れ目軸まわりの回転はどんなa1でも法線をその回転角だけ運ぶ)。"""
        return _angle_between(self.panel1.n, self.panel_mid.n)

    @property
    def fold2_angle_rad(self) -> float:
        return _angle_between(self.panel_mid.n, self.panel3.n)

    @property
    def end_point(self) -> Vec3:
        return tuple(self.panel3.origin[i] + self.L3_mm * self.panel3.u[i] for i in range(3))


def sheared_panel_corners(
    origin: Vec3,
    u: Vec3,
    v: Vec3,
    near_run_mm: float,
    far_run_mm: float,
    half_width_mm: float,
    *,
    near_tilt_rad: float = 0.0,
    far_tilt_rad: float = 0.0,
    half_width_neg_mm: float | None = None,
) -> list[Vec3]:
    """自由折れ目パネルの4隅を返す(end_panel_cornersの一般化)。

    境界(near/far)がそれぞれ折れ目軸の傾き(near_tilt_rad/far_tilt_rad)を持つ場合、
    幅方向座標widthでの境界上の点は run = boundary_run + width*tan(tilt) になる
    (SS8.8で導出・検証済み: 隣接パネルの折れ目軸gが自パネルの幅方向vとなす角は
    両パネルで厳密に同じ値になるため、隣接パネル同士は同じtilt値で計算すれば
    境界の4隅の座標が寸分違わず一致する)。tilt=0ならend_panel_cornersと同じ矩形になる
    (境界の自由端— 締結点まわりのbearing margin — は常にtilt=0)。

    順序は[near,-hw],[near,hw],[far,hw],[far,-hw](end_panel_cornersと同じ規約)。
    `half_width_neg_mm`を渡すとv負側だけ幅を変えられる(フランジ側の拡張、SS14。
    省略時は従来どおり左右対称)。
    """
    neg = half_width_neg_mm if half_width_neg_mm is not None else half_width_mm

    def pt(run: float, width: float, tilt: float) -> Vec3:
        shifted_run = run + width * math.tan(tilt)
        return tuple(origin[i] + shifted_run * u[i] + width * v[i] for i in range(3))

    return [
        pt(near_run_mm, -neg, near_tilt_rad),
        pt(near_run_mm, half_width_mm, near_tilt_rad),
        pt(far_run_mm, half_width_mm, far_tilt_rad),
        pt(far_run_mm, -neg, far_tilt_rad),
    ]


def _advance_chain_link(
    origin: Vec3, u: Vec3, n: Vec3, run_mm: float, tilt_rad: float, fold_rotation_rad: float
) -> tuple[Vec3, Vec3, Vec3]:
    """originからu方向にrun_mm進んだ点を折れ目点とし、そこで折れ目軸
    g=cos(tilt_rad)*v+sin(tilt_rad)*u まわりにfold_rotation_rad回転して次パネルの
    (走行方向, 法線)を返す。戻り値は(fold_point, next_u, next_n)。"""
    v = _cross(n, u)
    g = tuple(math.cos(tilt_rad) * v[i] + math.sin(tilt_rad) * u[i] for i in range(3))
    fold_point = tuple(origin[i] + run_mm * u[i] for i in range(3))
    next_u = rotate_about_axis(u, g, fold_rotation_rad)
    next_n = rotate_about_axis(n, g, fold_rotation_rad)
    return fold_point, next_u, next_n


def _chain_from_params(
    p1: Vec3,
    n1: Vec3,
    e_a: Vec3,
    e_b: Vec3,
    psi_rad: float,
    L1_mm: float,
    a1_rad: float,
    phi1_rad: float,
    L2_mm: float,
    a2_rad: float,
    phi2_rad: float,
    L3_mm: float,
) -> tuple[FreeFoldChain, Vec3, Vec3]:
    """8パラメータ(psi,L1,a1,phi1,L2,a2,phi2,L3)からFreeFoldChainを組み立てる。
    e_a/e_bはp1での法線n1に直交する正規直交基底(u1の向きpsiの基準)。
    戻り値は(chain, 実際の終点座標, 終端法線) — 終点がpoint2と一致するかは
    呼び出し側(free_fold_seed/solve_free_fold)の閉合条件チェックに委ねる。
    """
    u1 = tuple(math.cos(psi_rad) * e_a[i] + math.sin(psi_rad) * e_b[i] for i in range(3))
    fold1_pt, u_mid, n_mid = _advance_chain_link(p1, u1, n1, L1_mm, a1_rad, phi1_rad)
    fold2_pt, u3, n3 = _advance_chain_link(fold1_pt, u_mid, n_mid, L2_mm, a2_rad, phi2_rad)
    end_pt = tuple(fold2_pt[i] + L3_mm * u3[i] for i in range(3))
    chain = FreeFoldChain(
        panel1=ChainPanelFrame(origin=p1, u=u1, n=n1),
        panel_mid=ChainPanelFrame(origin=fold1_pt, u=u_mid, n=n_mid),
        panel3=ChainPanelFrame(origin=fold2_pt, u=u3, n=n3),
        L1_mm=L1_mm,
        L2_mm=L2_mm,
        L3_mm=L3_mm,
        a1_rad=a1_rad,
        a2_rad=a2_rad,
    )
    return chain, end_pt, n3


def _in_plane_basis(n: Vec3) -> tuple[Vec3, Vec3]:
    """n(単位法線)に直交する任意の正規直交基底を1組返す。"""
    reference = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    e_a = _normalize(_cross(n, reference))
    e_b = _cross(n, e_a)
    return e_a, e_b


def _signed_angle_about(v_from: Vec3, v_to: Vec3, axis: Vec3) -> float:
    return math.atan2(_dot(_cross(v_from, v_to), axis), _dot(v_from, v_to))


def free_fold_seed(
    point1: FasteningPoint,
    point2: FasteningPoint,
    *,
    bend_radius_mm: float,
    min_bearing_radius_mm: float,
    fold1_slack_mm: float,
    fold2_slack_mm: float,
    max_fold_deg: float = MAX_FOLD_ANGLE_DEG,
) -> FreeFoldChain | None:
    """折れ目を全てw=n1xn2に平行に保つ版の閉形式解(SS8.2/8.3/8.8)。

    two_point_frameを再利用してu1/u2/wを求め、中間折れ目位置
    a = min_bearing_radius_mm + T1 + fold1_slack_mm(bはfold2側も同様)を、
    T(接線長)とa/bの相互依存を不動点反復で解く(T=R*tan(theta/2)のthetaはa/bが
    決まらないと定まらないため、SS8.3参照)。w直交断面へ射影してからプロファイルを
    作る(p1・p2のw座標が違うため、3DのままB-Aを取るとランプ方向にw成分が乗る
    バグを2026-08-24に実測で発見、SS8.4参照)。

    【重要】ここでa1=a2=0(fold軸=v)と単純に置くのは誤り — 締結点の幅方向ズレ
    dw=dot(p2-p1,w)が非ゼロの限り、a1=a2=0の帯は中心線のw座標が全長にわたって
    p1のw座標に固定されてしまい、原理的にp2に届かない(SS8.8で判明)。正しくは、
    全長S(w直交断面上)とdwからgamma=atan2(dw,S)を求め、その分だけ各パネルの
    走行方向(u1,ランプ方向,u2)を最初からw方向にシアーしてから(u1s等)、
    その結果としてのfold軸の傾き(a1,a2、一般に非ゼロ)を逆算する。この構成が
    自由折れ目族の厳密な一員であること(=`solve_free_fold`のホモトピー継続の
    初期値としてそのまま使えること)は実測で変換残差9.2e-14まで検証済み(SS8.8)。

    n1・n2が平行に近い(two_point_frameのフォールバック分岐)場合や、折れ角が
    実現不能に大きい場合はNoneを返す。
    """
    n1 = _normalize(point1.normal_xyz)
    frame = two_point_frame(point1, point2)
    u1, u2, w = frame.u1, frame.u2, frame.w
    p1, p2 = point1.position_xyz, point2.position_xyz

    def project_out_w(p: Vec3) -> Vec3:
        return _sub(p, _scale(w, _dot(p, w)))

    q1, q2 = project_out_w(p1), project_out_w(p2)

    tangent1 = tangent2 = bend_radius_mm
    for _ in range(20):
        a = min_bearing_radius_mm + tangent1 + fold1_slack_mm
        b = min_bearing_radius_mm + tangent2 + fold2_slack_mm
        point_a = _add(q1, _scale(u1, a))
        point_b = _sub(q2, _scale(u2, b))
        ramp = _sub(point_b, point_a)
        ramp_len = math.sqrt(_dot(ramp, ramp))
        if ramp_len < 1e-6:
            return None
        ramp_dir = _scale(ramp, 1.0 / ramp_len)
        fold1_angle = _angle_between(u1, ramp_dir)
        fold2_angle = _angle_between(ramp_dir, u2)
        new_tangent1 = tangent_length_for_bend_angle_rad(fold1_angle, bend_radius_mm)
        new_tangent2 = tangent_length_for_bend_angle_rad(fold2_angle, bend_radius_mm)
        if abs(new_tangent1 - tangent1) < 1e-7 and abs(new_tangent2 - tangent2) < 1e-7:
            tangent1, tangent2 = new_tangent1, new_tangent2
            break
        tangent1, tangent2 = new_tangent1, new_tangent2
    if max(fold1_angle, fold2_angle) > math.radians(max_fold_deg):
        return None
    if ramp_len < tangent1 + tangent2:
        return None

    total_length = a + ramp_len + b  # w直交断面上のプロファイル全長S
    dw = _dot(_sub(p2, p1), w)
    gamma = math.atan2(dw, total_length)
    cos_g, sin_g = math.cos(gamma), math.sin(gamma)

    # 走行方向をgammaだけwへシアーする(展開面上で一定方向の帯が3Dでも一定幅を保つ、
    # SS8.1の等長写像の核心)。u1s等はwと合成してもn1に直交し続ける(u1,w共にn1に
    # 直交するため、その線形結合も直交する)。
    u1_sheared = _normalize(_add(_scale(u1, cos_g), _scale(w, sin_g)))
    ramp_sheared = _normalize(_add(_scale(ramp_dir, cos_g), _scale(w, sin_g)))
    u2_sheared = _normalize(_add(_scale(u2, cos_g), _scale(w, sin_g)))

    # 折れ目の二面角は元のw平行版のまま(シアーしても法線の回転角自体は変わらない
    # — 回転軸はあくまでw)。
    phi1_w = _signed_angle_about(u1, ramp_dir, w)
    phi2_w = _signed_angle_about(ramp_dir, u2, w)
    n_mid = rotate_about_axis(n1, w, phi1_w)
    n3 = rotate_about_axis(n_mid, w, phi2_w)

    def to_alpha_phi(u_in: Vec3, n_in: Vec3, u_out: Vec3, n_out: Vec3, phi_w: float):
        """折れ目軸gをwのどちらの向きに取るかで(alpha,phi)の符号が変わる
        (gと-gは同一直線)。両方試し、n・uの写り方が厳密に一致する側のうち
        |alpha|が小さい方を採る(SS8.8)。"""
        best = None
        for sign in (1.0, -1.0):
            g = _scale(w, sign)
            phi = sign * phi_w
            if math.sqrt(sum((c - d) ** 2 for c, d in zip(rotate_about_axis(n_in, g, phi), n_out))) > 1e-7:
                continue
            if math.sqrt(sum((c - d) ** 2 for c, d in zip(rotate_about_axis(u_in, g, phi), u_out))) > 1e-7:
                continue
            v_in = _cross(n_in, u_in)
            alpha = math.atan2(_dot(g, u_in), _dot(g, v_in))
            if best is None or abs(alpha) < abs(best[0]):
                best = (alpha, phi)
        return best

    seed_a1 = to_alpha_phi(u1_sheared, n1, ramp_sheared, n_mid, phi1_w)
    seed_a2 = to_alpha_phi(ramp_sheared, n_mid, u2_sheared, n3, phi2_w)
    if seed_a1 is None or seed_a2 is None:
        return None
    a1_rad, phi1_rad = seed_a1
    a2_rad, phi2_rad = seed_a2

    e_a, e_b = _in_plane_basis(n1)
    psi = math.atan2(_dot(u1_sheared, e_b), _dot(u1_sheared, e_a))
    L1 = a / cos_g
    L2 = ramp_len / cos_g
    L3 = b / cos_g
    return _chain_from_params(p1, n1, e_a, e_b, psi, L1, a1_rad, phi1_rad, L2, a2_rad, phi2_rad, L3)[0]


def solve_free_fold(
    seed: FreeFoldChain,
    point1: FasteningPoint,
    point2: FasteningPoint,
    *,
    target_a1_rad: float,
    homotopy_steps: int = 3,
    newton_iters: int = 40,
    max_step_mm: float = 30.0,
) -> FreeFoldChain | None:
    """seedから、fold1の傾き(a1)をtarget_a1_radまで連続的に動かして閉合を保ったまま
    解く(ホモトピー継続、SS8.8)。

    未知数(psi, phi1, L2, a2, phi2)の5変数を、拘束(終点==point2の3本 + 終端法線==
    ±point2法線の2本)の5本でニュートン法により解く。fold2の傾き(a2)はseedの値を
    初期値にしつつ、5変数の1つとして自由に動く(a2自体を独立にホモトピー目標として
    固定すると拘束過多になり閉合できないケースが増える — ユーザー①の「ランダム性」は
    a1側の摂動で担保する設計、SS8.8の実測: 素朴にpsi/a1/a2を独立乱数で振ると
    成立率5.0%・帯全長 中央値486mmで実用外、seedを初期値にしたホモトピー継続で
    のみ収束率67.5〜80.0%まで安定する)。L1・L3はseedの値のまま固定
    (=中間折れ位置は最初にbearing半径+接線長+ランダムslackで決めたものを保つ、
    ユーザー①の設計方針)。
    """
    n1 = seed.panel1.n
    e_a, e_b = _in_plane_basis(n1)
    p1, p2 = point1.position_xyz, point2.position_xyz
    n2 = _normalize(point2.normal_xyz)

    psi0 = math.atan2(_dot(seed.panel1.u, e_b), _dot(seed.panel1.u, e_a))
    phi1_0 = (
        seed.fold1_angle_rad
        if _dot(_cross(seed.panel1.n, seed.panel_mid.n), seed.panel1.v) >= 0
        else -seed.fold1_angle_rad
    )
    phi2_0 = (
        seed.fold2_angle_rad
        if _dot(_cross(seed.panel_mid.n, seed.panel3.n), seed.panel_mid.v) >= 0
        else -seed.fold2_angle_rad
    )
    x = [psi0, phi1_0, seed.L2_mm, seed.a2_rad, phi2_0]

    b1, b2 = _in_plane_basis(n2)

    def residual(vec: list[float], a1: float) -> list[float]:
        psi, phi1, L2, a2, phi2 = vec
        _, end_pt, n3 = _chain_from_params(
            p1, n1, e_a, e_b, psi, seed.L1_mm, a1, phi1, L2, a2, phi2, seed.L3_mm
        )
        diff = _sub(end_pt, p2)
        return [diff[0], diff[1], diff[2], _dot(n3, b1), _dot(n3, b2)]

    for step in range(1, homotopy_steps + 1):
        a1 = seed.a1_rad + (target_a1_rad - seed.a1_rad) * step / homotopy_steps
        converged = False
        for _ in range(newton_iters):
            r = residual(x, a1)
            err = math.sqrt(sum(c * c for c in r))
            if err < 1e-7:
                converged = True
                break
            jac = [[0.0] * 5 for _ in range(5)]
            for k in range(5):
                h = 1e-7 * max(1.0, abs(x[k]))
                xp = x[:]
                xp[k] += h
                rp = residual(xp, a1)
                for row in range(5):
                    jac[row][k] = (rp[row] - r[row]) / h
            step_vec = _solve_linear(jac, [-c for c in r])
            if step_vec is None:
                return None
            norm = math.sqrt(sum(c * c for c in step_vec))
            if norm > max_step_mm:
                step_vec = [c * max_step_mm / norm for c in step_vec]
            x = [x[i] + step_vec[i] for i in range(5)]
        if not converged:
            return None

    psi, phi1, L2, a2_raw, phi2 = x
    # 折れ目軸gと-gは同一直線なので、a2は(-90,90]度に畳む(ソルバが2piの整数倍だけ
    # 流すことがある。実測 修正前max 3171度、SS8.8)。
    a2 = math.atan2(math.sin(a2_raw), math.cos(a2_raw))
    if a2 > math.pi / 2:
        a2 -= math.pi
    elif a2 <= -math.pi / 2:
        a2 += math.pi
    if L2 <= 0.0:
        return None
    chain, end_pt, n3 = _chain_from_params(
        p1, n1, e_a, e_b, psi, seed.L1_mm, target_a1_rad, phi1, L2, a2, phi2, seed.L3_mm
    )
    if math.sqrt(sum((end_pt[i] - p2[i]) ** 2 for i in range(3))) > 1e-4:
        return None
    return chain


def _segment_distance_3d(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> float:
    """3D線分ab-cd間の最小距離(交差を含む)。標準的な最近接点法(クランプ付き)。"""
    u = _sub(b, a)
    v = _sub(d, c)
    w0 = _sub(a, c)
    aa, bb, cc = _dot(u, u), _dot(u, v), _dot(v, v)
    dd, ee = _dot(u, w0), _dot(v, w0)
    denom = aa * cc - bb * bb
    s = 0.0 if denom < 1e-12 else max(0.0, min(1.0, (bb * ee - cc * dd) / denom))
    t = (bb * s + ee) / cc if cc > 1e-12 else 0.0
    if t < 0.0:
        t = 0.0
        s = 0.0 if aa < 1e-12 else max(0.0, min(1.0, -dd / aa))
    elif t > 1.0:
        t = 1.0
        s = 0.0 if aa < 1e-12 else max(0.0, min(1.0, (bb - dd) / aa))
    closest_on_ab = tuple(a[i] + s * u[i] for i in range(3))
    closest_on_cd = tuple(c[i] + t * v[i] for i in range(3))
    return math.sqrt(sum((closest_on_ab[i] - closest_on_cd[i]) ** 2 for i in range(3)))


def panel_quad_clearance_mm(corners_a: list[Vec3], corners_b: list[Vec3]) -> float:
    """2つの平面四角形(sheared_panel_cornersが返す4隅、閉ループ順)の最小距離を、
    全辺同士の3D線分距離の最小値として求める。panel1とpanel3のような非隣接パネル同士の
    干渉チェックに使う(w平行版の`flat_panels_clearance_mm`の一般化、SS8.8)。
    """
    na, nb = len(corners_a), len(corners_b)
    best = math.inf
    for i in range(na):
        a1, a2 = corners_a[i], corners_a[(i + 1) % na]
        for j in range(nb):
            b1, b2 = corners_b[j], corners_b[(j + 1) % nb]
            best = min(best, _segment_distance_3d(a1, a2, b1, b2))
    return best

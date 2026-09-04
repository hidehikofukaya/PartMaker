"""parallel_same_offset クラスの締結点ペアをサンプリングする(Phase 1 PoC対象)。

法線がほぼ同方向(数度以内のジッタ)で、締結軸方向に MIN_OFFSET_MM 以上
MAX_OFFSET_MM 以下だけ離れた2点を生成する。分類の自己検証として、
生成後に classify() で parallel_same_offset になることを確認する。

panel_width_mm は締結点間の走行距離(lateral)ではなく、締結点周りの構造上
必要な最小半径(min_bearing_radius_mm)から算出する — 余肉削減(docs/
synthetic_two_joint_generation_roadmap.md Phase 1.5参照)の前段として、
最初から余肉が出にくいラフなサイジングにするため。正確な長円(スタジアム)形状
へのトリムはPhase 1.5でCATIA側に実装する。
"""

from __future__ import annotations

import dataclasses
import math
import random

from synthetic_generator.classify import (
    MIN_BASE_BEND_RADIUS_MM,
    MIN_NEUTRAL_PLANE_RADIUS_MM,
    FasteningPoint,
    Vec3,
    classify,
    fold_tangent_length_mm,
)

MIN_OFFSET_MM = 5.0
MAX_OFFSET_MM = 60.0
MIN_LATERAL_SPAN_MM = 20.0
MAX_LATERAL_SPAN_MM = 150.0
MAX_NORMAL_JITTER_DEG = 3.0  # cos(3 deg) ~= 0.9986 > 0.9 のしきい値を余裕を持って満たす

# 締結点周りに構造・強度上必要な最小半径(ユーザー例: R25)。この円の直径+わずかな
# マージンでパネル幅を決める(ざっくり余肉が出にくいサイジング)。厳密な長円形状への
# トリムはPhase 1.5(余肉削減)で行う — ここではラフな矩形サイジングのみ。
MIN_BEARING_RADIUS_RANGE_MM = (15.0, 30.0)
PANEL_WIDTH_MARGIN_RATIO_RANGE = (1.05, 1.25)  # 2*半径に対する余裕(5〜25%)
BEND_RADIUS_CAP_MM = 50.0  # ユーザー知見: 小部品でもR40〜50まであり得る、の上限

# フランジ配置の方針転換(2026-08-06、gsd_build.py参照)により、フランジ根本の折れが
# 幅方向+half_width側から中心(締結点)に向かって侵入するようになった。旧配置(フランジは
# 走行方向端)向けのpanel_widthサイジング(2*bearing_radius*1.05〜1.25倍のみ)では、
# その侵入量(flange_bend_radius_mm x flange_angle_degから決まるtangent_length)を
# 全く考慮しておらず、2000サンプルで77%が幅方向の事前チェック(gsd_build.py)で
# Infeasible判定される歩留まり崩壊が実測された。reinforcement.pyのサンプリング範囲
# (板厚1.0〜2.5mm・FLANGE_BEND_RADIUS_RATIO_CAP=3倍・斜めフランジ最大120度)から
# 逆算した理論上限を安全マージンとして両側(=panel_widthに2倍)に上乗せする
# (5000サンプルで検証、幅方向Infeasibleを0%に抑制)。
FLANGE_WIDTH_SAFETY_MARGIN_MM = 12.0


@dataclasses.dataclass(frozen=True)
class TwoJointSpec:
    point1: FasteningPoint
    point2: FasteningPoint
    thickness_mm: float
    hole_diameter_mm: float
    min_bearing_radius_mm: float  # 締結点周りの構造上必要な最小半径(余肉削減の基準円、ユーザー例: R25)
    panel_width_mm: float  # ベースパネル(ジョグ)の締結点走行方向に直交する幅。フランジ幅そのものではない。

    # ジョグの折れを何mm分の水平距離にかけて緩やかに遷移させるか。0に近いほど従来通りの
    # 鋭い直角ジョグ、大きいほど面同士の角度が平行に近い緩やかな遷移になる(ユーザー指摘:
    # メイン形状部分は面同士の角度をできるだけ平行に近づけ、折れ目のRを大きくする方が
    # 剛性・応力の観点で望ましい)。締結点周りのmin_bearing_radius_mmを両端で必ず確保した
    # 残りの範囲でしかサンプリングしないため、構造的に常に成立する。
    jog_ramp_extent_mm: float

    # ジョグ折れ目のエッジフィレット半径。フィレットは理論上のシャープな角より手前
    # (fold_tangent_length_mm分)から丸め始まるため、この後退が締結点のmin_bearing_radius_mm
    # を侵さない範囲(0〜実行可能上限)でのみサンプリングする — 構造的に常に成立する
    # (2026-08-06、歩留まり改善: 以前はreinforcement.py側で幾何と無関係に独立サンプリング
    # していたため、大きな値を引くと締結点に食い込み高確率で失敗していた)。
    bend_radius_mm: float


def _random_unit_vector(rng: random.Random) -> Vec3:
    z = rng.uniform(-1.0, 1.0)
    theta = rng.uniform(0.0, 2 * math.pi)
    r = math.sqrt(max(0.0, 1.0 - z * z))
    return (r * math.cos(theta), r * math.sin(theta), z)


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(v: Vec3) -> Vec3:
    length = math.sqrt(sum(c * c for c in v))
    return (v[0] / length, v[1] / length, v[2] / length)


def _tangent_basis(n: Vec3) -> tuple[Vec3, Vec3]:
    arbitrary = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = _normalize(_cross(n, arbitrary))
    v = _cross(n, u)
    return u, v


def _jitter_direction(n: Vec3, max_deg: float, rng: random.Random) -> Vec3:
    """n を最大 max_deg 度だけランダムな向きに傾ける(実データの微小なばらつきを模す)。"""
    if max_deg <= 0:
        return n
    u, v = _tangent_basis(n)
    angle = math.radians(rng.uniform(0.0, max_deg))
    phi = rng.uniform(0.0, 2 * math.pi)
    s = math.sin(angle)
    tilted = tuple(
        n[i] * math.cos(angle) + u[i] * math.cos(phi) * s + v[i] * math.sin(phi) * s for i in range(3)
    )
    return _normalize(tilted)


def sample(
    rng: random.Random,
    *,
    thickness_range_mm: tuple[float, float] = (1.0, 2.5),
    hole_diameter_range_mm: tuple[float, float] = (6.0, 14.0),
) -> TwoJointSpec:
    """parallel_same_offset クラスに分類される締結点ペアを1組サンプリングする。"""
    n1 = _random_unit_vector(rng)
    n2 = _jitter_direction(n1, MAX_NORMAL_JITTER_DEG, rng)

    u, v = _tangent_basis(n1)

    # bearing_radiusを先にサンプリングし、lateral(締結点間距離)の下限をそれに連動させる。
    # 順序を逆にする(lateralを先に決め打ちの範囲から独立に引く)と、bearing_radiusが
    # 大きい・lateralが小さい組み合わせでは、ジョグを完全に鋭い直角(ramp_extent=0)に
    # してもx_start(締結点からジョグ角までの距離)がbearing_radius未満になり、
    # bend_radius=0(フィレットなし)でさえ成立しない、という致命的な組み合わせが
    # 発生しうる(2026-08-06、歩留まり改善で発覚)。
    bearing_radius = rng.uniform(*MIN_BEARING_RADIUS_RANGE_MM)
    lateral_min = max(MIN_LATERAL_SPAN_MM, 2.0 * bearing_radius)
    lateral_max = max(MAX_LATERAL_SPAN_MM, lateral_min)
    lateral = rng.uniform(lateral_min, lateral_max)
    lateral_angle = rng.uniform(0.0, 2 * math.pi)
    offset = rng.uniform(MIN_OFFSET_MM, MAX_OFFSET_MM)

    p1: Vec3 = (0.0, 0.0, 0.0)
    p2: Vec3 = tuple(
        p1[i]
        + lateral * math.cos(lateral_angle) * u[i]
        + lateral * math.sin(lateral_angle) * v[i]
        + offset * n1[i]
        for i in range(3)
    )

    point1 = FasteningPoint(position_xyz=p1, normal_xyz=n1)
    point2 = FasteningPoint(position_xyz=p2, normal_xyz=n2)

    actual_class = classify(point1, point2)
    if actual_class != "parallel_same_offset":
        raise RuntimeError(
            f"sampled pair classified as {actual_class!r}, expected 'parallel_same_offset' "
            "(sampling ranges may need retuning)"
        )

    thickness = rng.uniform(*thickness_range_mm)
    hole_diameter = rng.uniform(*hole_diameter_range_mm)
    panel_width = 2.0 * bearing_radius * rng.uniform(*PANEL_WIDTH_MARGIN_RATIO_RANGE) + 2.0 * FLANGE_WIDTH_SAFETY_MARGIN_MM

    # 締結点周りにbearing_radius分の余地を両端で必ず残した残り区間内でのみランプ幅を
    # サンプリングする(=構造的に常に成立する)。lateralはpoint1->point2の接平面内
    # 走行距離で、frame.run_length_mmと厳密に一致する(同じ接平面基底で計算しているため)。
    max_ramp_extent = max(0.0, lateral - 2.0 * bearing_radius)
    jog_ramp_extent = rng.uniform(0.0, max_ramp_extent)

    # bend_radius_mmは、折れ目フィレットの後退(fold_tangent_length_mm)が締結点の
    # bearing_radius分の余白を侵さない実行可能上限の範囲内でサンプリングする(構造的に
    # 常に成立)。x_startはgsd_build._panel_cornersと同じ定義(ジョグ折れ目までの距離)。
    x_start = (lateral - jog_ramp_extent) / 2.0
    available_clear = max(0.0, x_start - bearing_radius)
    unit_tangent_length = fold_tangent_length_mm(offset, jog_ramp_extent, 1.0)
    if unit_tangent_length < 1e-9:
        max_bend_radius = BEND_RADIUS_CAP_MM
    else:
        max_bend_radius = min(BEND_RADIUS_CAP_MM, available_clear / unit_tangent_length)
    # 幅方向の実行可能性も考慮する: フィレット半径がパネル幅の半分(half_width)に近い、
    # または超えると、実機でCATIAのUpdateが失敗するケースを確認した(2026-08-06)。
    # 0.9倍を安全マージンとしてさらにクランプする。
    half_width = panel_width / 2.0
    max_bend_radius = min(max_bend_radius, 0.9 * half_width)
    # ユーザー製造制約(2026-08-06): 中立面Rは全て最小R4を守る。max_bend_radiusが
    # 4mm未満(=この組み合わせではR4以上のフィレットが物理的に成立しない)の場合は
    # あえて4mmを引いておき、gsd_build.py側の事前チェック(締結点必要最小半径/幅方向)で
    # 明示的にInfeasibleとして弾かれるようにする(「失敗したらアボートでいい」方針のまま)。
    # メイン曲げは慣例のR10以上を狙う(成立しない組み合わせはあえてR10を返し、
    # builder側の事前チェックで明示的にInfeasibleとして弾かれるようにする)。
    bend_radius = rng.uniform(MIN_BASE_BEND_RADIUS_MM, max(MIN_BASE_BEND_RADIUS_MM, max_bend_radius))

    return TwoJointSpec(
        point1=point1,
        point2=point2,
        thickness_mm=thickness,
        hole_diameter_mm=hole_diameter,
        min_bearing_radius_mm=bearing_radius,
        panel_width_mm=panel_width,
        jog_ramp_extent_mm=jog_ramp_extent,
        bend_radius_mm=bend_radius,
    )

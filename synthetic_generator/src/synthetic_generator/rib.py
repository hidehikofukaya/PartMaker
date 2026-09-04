"""曲げの内角を橋渡しするリブ(2026-09-04、D3。ユーザー指摘で全面改訂)。

ユーザー指定の形状: **正四面体を内角側に押し付けたような形**。

    曲げ線に沿って見た図              展開平面図(フットプリント)
      パネル1                          ┌─────────┐
         │╲   A                        │     A     │
         │ ╲ ╱╲    ← 稜A–Bが           │    ╱ ╲    │
         │  ╲╱  ╲     内角を横切る   V1 ├──●───●──┤ V2  ← 曲げ線
         │  ╱╲   ╲                     │    ╲ ╱    │
         └─╱──╲───B── パネル2          │     B     │
                                       └─────────┘

四面体 V1-V2-A-B のうち、面 V1V2A はパネル1に、面 V1V2B はパネル2に乗っている。
中立面に現れるのは残り2面 **V1AB と V2AB** で、これが内角を横切って張られる。
稜 AB は曲げ線に**直交**する — ここが要点で、稜が曲げ線に平行だと曲げの開きに効かない。

生成条件(ユーザー指定): **折れ角が45度以上** かつ **締結点が近すぎてビードが置けない**
(`general_geometry.bead_room_mm < 0`)。どちらか欠ければリブは作らない。

工程(他の特徴と違う点): 基準面を**シャープな折れ**のまま作ってリブを置き、そのあと
**リブの幅の外側だけ**を中立面R最小(5mm)でフィレットする。リブ幅の中は曲げが
リブそのものに置き換わるのでフィレットしない。
"""

from __future__ import annotations

import dataclasses
import math
import random

from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM

# リブを作る折れ角の下限(ユーザー決定 2026-09-04)。
RIB_MIN_FOLD_ANGLE_DEG = 45.0

# 「締結点が近すぎてビードが置けない」の判定値[mm]。`bead_room_mm`(ビード本体に
# 使える中心線の長さ)がこれ未満ならリブ。**構築可否では判定しない** —
# ビードの置き場所を端パネル固定から探索に変えたら、技術的には長さ10mmのビードでも
# 置けてしまい、リブが1件も出なくなった(2026-09-04実測)。狙いは「短くて
# まともなビードにならない部品」なので、本体長で切る。
# 実測: この値で全体の3.0%、その帯の締結点間距離は中央値65mm(40〜131mm)。
RIB_MAX_BEAD_ROOM_MM = 40.0
# リブ部品の基準面フィレット半径。ユーザー決定により中立面R最小に固定する。
RIB_BEND_RADIUS_MM = MIN_NEUTRAL_PLANE_RADIUS_MM

# 半幅に対するリブ半幅 c の比。稜線を最小Rでフィレットするぶんくぼみが浅くなるので、
# ユーザー了承(2026-09-04)のもと数mm大きめに振る。
RIB_HALF_WIDTH_RATIO_RANGE = (0.45, 0.75)
# 折れ線から A / B までの距離を c の何倍にするか。sqrt(3)≒1.73 で正三角形になる。
RIB_LEG_RATIO_RANGE = (1.2, 2.2)
# フィレットがシャープへ落ちる遷移帯の幅[mm]。
RIB_TAPER_MM = 1.5
# リブのフットプリントより外側に残すシャープな折れの幅[mm]。
#
# **これが無いと稜線フィレットが通らない。**遷移ロフト(円弧->頂点)の退化頂点が
# リブの角 V1/V2 と同じ点に来るため、フィレットの walking がそこで破綻する
# (2026-09-04実測: StripeStatus=2 WalkingFailure。最小構成の平板2枚+四面体では
#  同じ四面体がR2〜R5すべて通るので、原因は四面体ではなく周囲のトポロジー)。
RIB_SHARP_MARGIN_MM = 4.0
# 稜線フィレット(中立面R最小=5mm)を入れても形が残る最小サイズ。
# 小さいリブほどフィレットの条件が悪く、失敗するか OCCT が長時間止まる
# (2026-09-04実測: シャープ帯2.5mm・小リブで222試行中111がフィレット失敗)。
# ユーザー了承のもと数mm大きめに寄せる。
MIN_RIB_HALF_WIDTH_MM = 10.0
MIN_RIB_LEG_MM = 12.0
# リブ三角形 V1-A-B の内接円半径が稜線フィレット半径より小さいと、3辺のフィレットが
# 重なって OCCT が発散する(実測: c=10・脚12・折れ角90度で内接円4.6mm < R5 となり、
# `BRepFilletAPI_MakeFillet` が数分止まってバッチが停止した。2026-09-04)。
# 余裕を持って 1.3倍を要求する。
RIB_INRADIUS_MARGIN = 1.3


@dataclasses.dataclass(frozen=True)
class RibParams:
    half_width_mm: float    # c: 曲げ線上での半幅(V1V2 の半分)
    leg1_mm: float          # a: 折れ線からパネル1側の頂点Aまでの距離
    leg2_mm: float          # b: 折れ線からパネル2側の頂点Bまでの距離
    taper_mm: float         # フィレットがシャープへ落ちる遷移帯の幅
    sharp_margin_mm: float  # フットプリントより外側に残すシャープな折れの幅
    fold_index: int         # どの曲げに載せるか(0始まり)

    @property
    def reach_mm(self) -> float:
        """走行方向に必要な余地(折れ線から遠いほう)。"""
        return max(self.leg1_mm, self.leg2_mm)


def triangle_inradius_mm(half_width: float, leg1: float, leg2: float,
                         fold_angle_rad: float) -> float:
    """リブ三角形 V1-A-B の内接円半径。稜線フィレットが成立する条件の指標。"""
    side_a = math.hypot(leg1, half_width)
    side_b = math.hypot(leg2, half_width)
    ridge = math.sqrt(max(0.0, leg1 * leg1 + leg2 * leg2
                          + 2.0 * leg1 * leg2 * math.cos(fold_angle_rad)))
    semi = 0.5 * (side_a + side_b + ridge)
    area_sq = semi * (semi - side_a) * (semi - side_b) * (semi - ridge)
    if area_sq <= 0.0 or semi <= 0.0:
        return 0.0
    return math.sqrt(area_sq) / semi


def sample_rib(
    rng: random.Random,
    *,
    half_width_mm: float,
    fold_index: int,
    leg_room_mm: tuple[float, float],
    fold_angle_rad: float,
) -> RibParams | None:
    """この曲げに載るリブを1つ引く(載らなければNone)。

    `leg_room_mm` は頂点 A / B を置ける走行方向の余地。端パネル側は締結点の座面円を
    侵さない範囲、中間パネル側は隣の曲げのタンジェント線まで。
    """
    c_hi = min(RIB_HALF_WIDTH_RATIO_RANGE[1] * half_width_mm,
               half_width_mm - RIB_TAPER_MM - RIB_SHARP_MARGIN_MM - 1.0,
               min(leg_room_mm) / RIB_LEG_RATIO_RANGE[0])
    c_lo = max(MIN_RIB_HALF_WIDTH_MM, RIB_HALF_WIDTH_RATIO_RANGE[0] * half_width_mm)
    c_lo = min(c_lo, c_hi)          # 幅が足りない板ではとにかく最大まで使う
    if c_hi < c_lo:
        return None
    c = rng.uniform(0.5 * (c_lo + c_hi), c_hi)
    legs = []
    for room in leg_room_mm:
        low = max(MIN_RIB_LEG_MM, RIB_LEG_RATIO_RANGE[0] * c)
        high = min(RIB_LEG_RATIO_RANGE[1] * c, room)
        if high < low:
            return None
        legs.append(rng.uniform(0.5 * (low + high), high))
    # 稜線フィレットが成立する大きさか(内接円 >= 1.3 x フィレット半径)。
    inradius = triangle_inradius_mm(c, legs[0], legs[1], fold_angle_rad)
    if inradius < RIB_INRADIUS_MARGIN * MIN_NEUTRAL_PLANE_RADIUS_MM:
        return None
    return RibParams(
        half_width_mm=c,
        leg1_mm=legs[0],
        leg2_mm=legs[1],
        taper_mm=RIB_TAPER_MM,
        sharp_margin_mm=RIB_SHARP_MARGIN_MM,
        fold_index=fold_index,
    )


def leg_room_mm(plan, fold_index: int) -> tuple[float, float]:
    """曲げ `fold_index` の前後で、リブの頂点を置ける走行方向の余地[mm]。

    端パネル側は締結点の座面円(半径=座面半径)を侵さない範囲まで、
    中間パネル側は隣の曲げのタンジェント線まで。
    ここを「平坦区間の長さ − 2×座面半径」で近似すると、単曲げでパネル原点が締結点に
    ある構成で余地を過小評価して、リブが作れるのに作らなくなる(2026-09-04実測)。
    """
    frames, tangents = plan.panel_frames, plan.fold_tangents
    margin = plan.min_bearing_radius_mm
    before, after = frames[fold_index], frames[fold_index + 1]
    block_before = (before.near_run_mm + 2.0 * margin if fold_index == 0
                    else before.near_run_mm + tangents[fold_index][0])
    room_before = before.far_run_mm - block_before
    last = fold_index + 1 == len(frames) - 1
    block_after = (after.far_run_mm - 2.0 * margin if last
                   else after.far_run_mm - tangents[fold_index + 1][1])
    room_after = block_after - after.near_run_mm
    return room_before, room_after

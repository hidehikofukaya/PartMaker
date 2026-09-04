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
# リブ部品の基準面フィレット半径。ユーザー決定により中立面R最小に固定する。
RIB_BEND_RADIUS_MM = MIN_NEUTRAL_PLANE_RADIUS_MM

# 半幅に対するリブ半幅 c の比。0.5なら板幅のちょうど半分をリブが占める。
RIB_HALF_WIDTH_RATIO_RANGE = (0.30, 0.60)
# 折れ線から A / B までの距離を c の何倍にするか。sqrt(3)≒1.73 で正三角形になる。
RIB_LEG_RATIO_RANGE = (1.0, 2.0)
# フィレットがシャープへ落ちる遷移帯の幅[mm](リブ幅のすぐ外側)。
RIB_TAPER_MM = 1.5


@dataclasses.dataclass(frozen=True)
class RibParams:
    half_width_mm: float    # c: 曲げ線上での半幅(V1V2 の半分)
    leg1_mm: float          # a: 折れ線からパネル1側の頂点Aまでの距離
    leg2_mm: float          # b: 折れ線からパネル2側の頂点Bまでの距離
    taper_mm: float         # フィレットがシャープへ落ちる遷移帯の幅
    fold_index: int         # どの曲げに載せるか(0始まり)

    @property
    def reach_mm(self) -> float:
        """走行方向に必要な余地(折れ線から遠いほう)。"""
        return max(self.leg1_mm, self.leg2_mm)


def sample_rib(
    rng: random.Random,
    *,
    half_width_mm: float,
    fold_count: int,
    flat_runs_mm: tuple[float, float],
) -> RibParams | None:
    """この基準面に載るリブを1つ引く(載らなければNone)。

    `flat_runs_mm` はリブを載せる曲げの前後にある直線区間の長さ。頂点A/Bがそこに
    収まらなければならない。
    """
    if fold_count < 1:
        return None
    c_max = RIB_HALF_WIDTH_RATIO_RANGE[1] * half_width_mm
    if c_max < 3.0 or half_width_mm - c_max < RIB_TAPER_MM + 2.0:
        return None
    c = rng.uniform(RIB_HALF_WIDTH_RATIO_RANGE[0] * half_width_mm,
                    min(c_max, half_width_mm - RIB_TAPER_MM - 2.0))
    run1, run2 = flat_runs_mm
    legs = []
    for run in (run1, run2):
        low = 1.0 * c
        high = min(RIB_LEG_RATIO_RANGE[1] * c, run - 1.0)
        if high < low:
            return None
        legs.append(rng.uniform(low, high))
    return RibParams(
        half_width_mm=c,
        leg1_mm=legs[0],
        leg2_mm=legs[1],
        taper_mm=RIB_TAPER_MM,
        fold_index=rng.randrange(fold_count),
    )

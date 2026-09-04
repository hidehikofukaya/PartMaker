"""曲げの内角を橋渡しするリブ(2026-09-04、D3。3度目の改訂)。

## 形状

**曲げのコーナーを、幅方向の中央だけ大きな半径で丸めたもの。**
曲げ線に直交する各断面で、パネル1 → 円弧 → パネル2 と繋ぎ、円弧の半径を

    rho(y) = R_bend + (rho_max - R_bend) * (1 - |y| / c)      (|y| <= c)
    rho(y) = R_bend                                            (|y| >  c)

とする。y はリブ半幅 c を持つ幅方向座標。中央 (y=0) で最も大きく膨らみ、
リブの端 (|y|=c) で基準面の曲げRに一致するので、**基準面のフィレットと連続**する。

* 平面視のフットプリントは**菱形**(接線長 T(y)=rho(y)*tan(φ/2) が y に線形なので、
  境界が直線になる)
* パネルとの境目は**接線連続**(角が立たない)
* 円弧なので**稜線も丸い**
* エッジは全て直線か円弧

## なぜフィレット演算をやめたか

前版はシャープな四面体を作って `BRepFilletAPI_MakeFillet` で丸めていたが、
**頂点ブレンドの境界が解析曲線にならない**。実測(2026-09-04、33部品):
ずれは中央値 0.625mm / 最大 1.095mm で、正しい許容(0.25×板厚 = 0.25〜0.625mm)でも
33件中2件しか通らなかった。フィレット自体の失敗・無効シェルも合わせると成立率2.4%。
**丸みは演算ではなく構築で作る**ことにした。

## 生成条件(ユーザー指定 2026-09-04)

**折れ角45度以上** かつ **締結点が近すぎてビードがまともに載らない**
(`general_geometry.bead_room_mm < RIB_MAX_BEAD_ROOM_MM`)。
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

# 「締結点が近すぎてビードが置けない」の判定値[mm](`bead_room_mm` がこれ未満)。
RIB_MAX_BEAD_ROOM_MM = 40.0

# 半幅に対するリブ半幅 c の比。2026-09-04にユーザー指摘で20%拡大した
# (控えめすぎたため)。座面への干渉は `leg_room_mm` が端で 2×座面半径 を
# 差し引いて守るので、その範囲内なら広げてよい。
RIB_HALF_WIDTH_RATIO_RANGE = (0.42, 0.84)
# 中央の膨らみ半径 rho_max を、基準面の曲げRの何倍にするか(同じく20%拡大)。
RIB_BULGE_RATIO_RANGE = (3.0, 7.2)
# 断面を刻む本数(片側)。多いほど滑らかだが面数が増える。
RIB_STATIONS_PER_SIDE = 4
MIN_RIB_HALF_WIDTH_MM = 6.0


@dataclasses.dataclass(frozen=True)
class RibParams:
    half_width_mm: float    # c: 曲げ線方向のリブの半幅
    bulge_radius_mm: float  # rho_max: 中央での丸めの半径
    fold_index: int         # どの曲げに載せるか(0始まり)
    stations: int = RIB_STATIONS_PER_SIDE

    def radius_at(self, y: float, bend_radius_mm: float) -> float:
        """幅座標 y での丸め半径。|y|>=c では基準面の曲げRに一致する。"""
        t = min(1.0, abs(y) / self.half_width_mm)
        return bend_radius_mm + (self.bulge_radius_mm - bend_radius_mm) * (1.0 - t)

    def reach_mm(self, fold_angle_rad: float) -> float:
        """走行方向に必要な余地(中央での接線長)。"""
        return self.bulge_radius_mm * math.tan(0.5 * fold_angle_rad)


def sample_rib(
    rng: random.Random,
    *,
    half_width_mm: float,
    fold_index: int,
    leg_room_mm: tuple[float, float],
    fold_angle_rad: float,
    bend_radius_mm: float = RIB_BEND_RADIUS_MM,
) -> RibParams | None:
    """この曲げに載るリブを1つ引く(載らなければNone)。

    `leg_room_mm` は接線点を置ける走行方向の余地(端パネル側は締結点の座面円まで、
    中間パネル側は隣の曲げのタンジェント線まで)。
    """
    # 外側に残す帯が薄いと縫合できず外形が複数ループになる(実測: 120件中23件)。
    c_hi = min(RIB_HALF_WIDTH_RATIO_RANGE[1] * half_width_mm, half_width_mm - 3.0)
    c_lo = min(max(MIN_RIB_HALF_WIDTH_MM, RIB_HALF_WIDTH_RATIO_RANGE[0] * half_width_mm), c_hi)
    if c_hi < MIN_RIB_HALF_WIDTH_MM:
        return None
    c = rng.uniform(c_lo, c_hi)

    # 中央の膨らみ半径は、接線長が両パネルの余地に収まる範囲で引く。
    half_tan = math.tan(0.5 * fold_angle_rad)
    if half_tan <= 1e-6:
        return None
    rho_hi = min(RIB_BULGE_RATIO_RANGE[1] * bend_radius_mm, min(leg_room_mm) / half_tan)
    rho_lo = RIB_BULGE_RATIO_RANGE[0] * bend_radius_mm
    if rho_hi < rho_lo:
        return None
    return RibParams(half_width_mm=c, bulge_radius_mm=rng.uniform(rho_lo, rho_hi),
                     fold_index=fold_index)


def leg_room_mm(plan, fold_index: int) -> tuple[float, float]:
    """曲げ `fold_index` の前後で、接線点を置ける走行方向の余地[mm]。

    端パネル側は締結点の座面円(半径=座面半径)を侵さない範囲まで、
    中間パネル側は隣の曲げのタンジェント線まで。
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

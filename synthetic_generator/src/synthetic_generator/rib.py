"""曲げをまたぐリブ(補強くぼみ)の断面パラメータ(2026-09-04、多様性拡張D3)。

ビード(`bead.py`)・フランジ(`flange.py`)と並ぶ3つ目の補強プリミティブ。
ユーザー指定(2026-09-04): **曲げの内角方向にひし形状のくぼみをつけて剛性を高める**。

なぜ要るか(実測):

* ビードは中心線上に「2×座面半径 + ランアウト + 本体」を要求するので、締結点が近い
  部品には**原理的に載らない**(締結点間 40〜80mm で成立率 0.0%)。
* フランジは最大折れ角20度以下が条件なので、距離に関係なく13〜20%で頭打ち。
* 結果、現行の120〜220mm帯ですら **34%** が無補強、60〜100mm帯では **84%** が無補強。
* 一方リブを置く余地は全帯で **99.4〜100%** ある(曲げに隣接する直線区間の中央値
  26〜35mm、凹側半径の余裕も全件クリア)。

構築(`occt_build`):

    平地 ─(先端=点)─ ノーズ ─[曲げ弧だけが稜]─ ノーズ ─(先端=点)─ 平地

断面はビードと同じ9要素(平地/足R/壁/稜線R/頂部/稜線R/壁/足R/平地)。違いは
(a) **曲げ弧のぶんだけが full section**で前後はすぐ点へすぼまる、
(b) 立ち上げ向きが**凹側に固定**、(c) 先端が1点に収束する。
平面視は菱形、立体では「八面体の半分が内角側へ出っ張る」形になる
(2026-09-04のユーザー指摘で、平坦な稜線が長い形から作り直した)。

曲げ弧の上だけを full section にするのは形の狙いだけでなく構築上の要請でもある:
そこは回転掃引(`MakeRevol`)で厳密な円錐/トーラスになるが、断面を s に沿って
連続的に縮めると曲げ上でも直線織り面になり、部品の**側端が円弧ではなく弦の折れ線**に
なってしまう。曲げ弧=稜、直線区間=テーパ、とすると両方が厳密なまま両立する。
"""

from __future__ import annotations

import dataclasses
import math
import random

from synthetic_generator.bead import (
    BEAD_RIDGE_WALL_CONSUMPTION_MAX,
    BEAD_SIDE_CLEARANCE_MM,
    BeadPanelFrame,
)
from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM

# 深さ。曲げRの凹側に載るので R - 深さ >= 中立面R最小(5mm) を満たす必要がある。
# 曲げRは10〜22.5mmなので、上限6mmなら大半の曲げで成立する。
RIB_DEPTH_RANGE_MM = (4.0, 8.0)
RIB_TOP_WIDTH_RANGE_MM = (6.0, 20.0)
RIB_WALL_ANGLE_RANGE_DEG = (45.0, 70.0)
# 先端(点)までの長さ。**幅**に対する倍率で決める(2026-09-04にユーザー指摘で変更)。
# 深さ基準にすると細長い稜線になってしまい、狙いの「八面体の半分」に見えない。
# 1.0前後だと縦横がほぼ等しい菱形になる。
RIB_NOSE_WIDTH_RATIO_RANGE = (0.8, 1.6)
MIN_RIB_NOSE_MM = 4.0


@dataclasses.dataclass(frozen=True)
class RibParams:
    depth_mm: float
    top_width_mm: float
    wall_angle_deg: float
    ridge_radius_mm: float
    nose_ratio: float       # 先端までの長さ / フットプリント半幅
    fold_index: int         # どの曲げに載せるか(0始まり)

    @property
    def wall_run_mm(self) -> float:
        return self.depth_mm / math.tan(math.radians(self.wall_angle_deg))

    @property
    def ridge_setback_mm(self) -> float:
        return self.ridge_radius_mm * math.tan(math.radians(self.wall_angle_deg) / 2.0)

    @property
    def half_footprint_mm(self) -> float:
        return self.top_width_mm / 2.0 + self.wall_run_mm

    @property
    def nose_length_mm(self) -> float:
        """曲げ弧の端から先端(点)までの長さ。"""
        return max(MIN_RIB_NOSE_MM, self.nose_ratio * self.half_footprint_mm)


def sample_rib(
    rng: random.Random,
    *,
    half_width_mm: float,
    bend_radius_mm: float,
    fold_count: int,
) -> RibParams | None:
    """この基準面に載るリブを1つサンプリングする(載らなければNone)。

    ビードと同じく「実行可能な上限を計算してからその中で引く」方針。
    依存の向き: 深さ(曲げRの凹側で決まる) -> 稜線R(壁の斜辺で決まる) -> 頂部幅(板幅で決まる)。
    """
    if fold_count < 1:
        return None  # 曲げが無ければリブは置けない(曲げの内角を補強する特徴なので)

    # 1-2. (壁角度, 深さ)を実行可能領域の上で**同時に一様**に引く(sample_beadと同じ
    #      リジェクションサンプリング。逐次に引くと「急な壁 ⇒ 必ず深い」という板金設計に
    #      存在しない相関がデータに入る)。実行可能条件は2つ:
    #        (a) 凹側に載るので R - depth >= 中立面R最小
    #        (b) 稜線Rが中立面R最小を下回れない:
    #            2*R0*tan(θ/2) <= K*slant = K*depth/sin(θ)
    max_depth = min(RIB_DEPTH_RANGE_MM[1], bend_radius_mm - MIN_NEUTRAL_PLANE_RADIUS_MM)
    if max_depth < RIB_DEPTH_RANGE_MM[0]:
        return None
    for _ in range(200):
        wall_angle = rng.uniform(*RIB_WALL_ANGLE_RANGE_DEG)
        depth = rng.uniform(RIB_DEPTH_RANGE_MM[0], max_depth)
        theta = math.radians(wall_angle)
        min_depth = (2.0 * MIN_NEUTRAL_PLANE_RADIUS_MM * math.tan(theta / 2.0) * math.sin(theta)
                     / BEAD_RIDGE_WALL_CONSUMPTION_MAX)
        if depth >= min_depth:
            break
    else:
        return None
    slant = depth / math.sin(theta)
    max_ridge = BEAD_RIDGE_WALL_CONSUMPTION_MAX * slant / (2.0 * math.tan(theta / 2.0))
    ridge_radius = rng.uniform(MIN_NEUTRAL_PLANE_RADIUS_MM, max_ridge)

    # 3. 頂部幅: 足Rの後退量 + 余白を引いた残り幅に収まること。
    #    かつ頂部が稜線フィレットで食い尽くされないこと(top/2 > setback)。
    setback = ridge_radius * math.tan(theta / 2.0)
    wall_run = depth / math.tan(theta)
    max_top_half = half_width_mm - setback - BEAD_SIDE_CLEARANCE_MM - wall_run
    min_top_half = max(RIB_TOP_WIDTH_RANGE_MM[0] / 2.0, setback + 0.5)
    if max_top_half < min_top_half:
        return None
    top_width = 2.0 * rng.uniform(min_top_half, min(max_top_half, RIB_TOP_WIDTH_RANGE_MM[1] / 2.0))

    return RibParams(
        depth_mm=depth,
        top_width_mm=top_width,
        wall_angle_deg=wall_angle,
        ridge_radius_mm=ridge_radius,
        nose_ratio=rng.uniform(*RIB_NOSE_WIDTH_RATIO_RANGE),
        fold_index=rng.randrange(fold_count),
    )


def rib_fits(rib: RibParams, panel_frames: list[BeadPanelFrame], *, half_width_mm: float) -> bool:
    """板幅にフットプリントが収まるか(純Pythonの事前判定)。

    走行方向の余地(曲げの前後にノーズが取れるか)は経路が要るので `occt_build` 側で見る。
    """
    return rib.half_footprint_mm + rib.ridge_setback_mm + BEAD_SIDE_CLEARANCE_MM <= half_width_mm

"""折れ目を跨ぐ側端フランジの計画(CATIA非依存)。2026-08-25、SS14。

ビード(bead.py)と対をなす補強。使い分けはユーザー指定(2026-08-25):
**最大折れ角20度以下ならフランジ、それ以外(急でフランジ不成立)はビード**。

構成(ユーザー指定の①〜⑤を実証済みプリミティブへ写像したもの。spike_flange_over_bends.pyで
6/6成立を確認):
  ③ 基準面をフランジ側だけ (根本R + 2mm) 広く作る(plan_general_two_pointの
     side_extension_mm。外挿コマンドの代替 — 面生成を自前で握っているため)。
  ④ 側端曲線 = 中心線(投影)の測地オフセット(CurvePar)。そこからMode=4・±90度の
     ドラフトスイープで壁を立てる。
  ⑤ 拡張面 x 壁の AddNewFilletBiTangent(トリム込み)で根本R。エッジフィレット
     (BRep参照、自動化不可)は使わない。

側の選択(②): 長手方向エッジが面内の弧を描く場合は**外側(凸側=エッジが長い側)**。
内側だと拡張・オフセットが自己交差しうるため。実質直線なら乱数で選ぶ(多様性)。

方向と高さの制約: フランジが**凹側**の折れ目を跨ぐ場合、基準面から高さhの平行面は
半径 R_bend - h に縮み、h >= R_bend で自己交差する。凸側交差は無制約。方向は
凹側交差が無い側を優先し、やむを得ず凹側を跨ぐ場合は h <= R_bend - 2mm を課す。
"""

from __future__ import annotations

import dataclasses
import math
import random

from synthetic_generator.bead import BeadPanelFrame, _cross, _normalize
from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM, Vec3

# 使い分けのしきい値(ユーザー指定、2026-08-25)。最大折れ角がこれ以下ならフランジ。
FLANGE_MAX_FOLD_ANGLE_DEG = 20.0

FLANGE_HEIGHT_RANGE_MM = (10.0, 20.0)
# 根本R。下限は中立面R最小。慣例の1〜3t(t=1〜2.5mm)とはR5最小則が薄板で衝突するが、
# ユーザー承認(2026-08-25)によりR5〜8で進める。
FLANGE_ROOT_RADIUS_RANGE_MM = (MIN_NEUTRAL_PLANE_RADIUS_MM, 8.0)
FLANGE_EXTENSION_MARGIN_MM = 2.0     # 拡張 = 根本R + これ(ユーザー指定の+1〜2mm)
FLANGE_CONCAVE_CLEARANCE_MM = 2.0    # 凹側交差の h <= R_bend - これ
FLANGE_EDGE_INSET_MM = 0.2           # 壁の根本曲線を端からわずかに内側へ(境界上の退化回避)
# 面内の弧の判定: 両側端の長さ差がこれ未満なら「実質直線」として側を乱数で選ぶ
FLANGE_STRAIGHT_EDGE_TOLERANCE_MM = 0.5
CENTRELINE_POINTS_PER_PANEL = 5
FLAT_CLEARANCE_MM = 1.0


@dataclasses.dataclass(frozen=True)
class FlangeParams:
    height_mm: float        # 壁の全高(根本Rを含む)
    root_radius_mm: float   # 根本R(基準面 x 壁のBiTangent)
    side: int               # +1 = v正側 / -1 = v負側
    direction: int          # +1 = パネル法線(+u x v)側へ立てる / -1 = 反対側

    @property
    def extension_mm(self) -> float:
        """フランジ側に基準面を広げる量(根本Rが名目幅を食わないための余白込み)。"""
        return self.root_radius_mm + FLANGE_EXTENSION_MARGIN_MM


@dataclasses.dataclass(frozen=True)
class FlangeSurfacePlan:
    """CATIA側(gsd_build._add_flange_to_surface)へ渡す配置情報。全て解析座標。"""

    edge_offset_mm: float          # 中心線から壁の根本曲線までの測地オフセット量
    centreline_points: list[Vec3]  # 中心線の標本点(端接線の補助点込み、走行順)
    side_probe: Vec3               # CurveParの向き選定用(根本曲線上の点)
    wall_top_probe: Vec3           # 壁の上端の期待位置(±90度スイープの向き選定用)
    root_keep_points: list[Vec3]   # 根本R後も残るべき基準面上の点(各パネル中央)
    wall_keep_point: Vec3          # 根本R後も残るべき壁上の点(上端)


def max_fold_angle_deg(panel_frames: list[BeadPanelFrame]) -> float:
    """パネル列の最大折れ角[deg]。使い分け判定(<=20度ならフランジ)に使う。"""
    worst = 0.0
    for a, b in zip(panel_frames, panel_frames[1:]):
        cos = max(-1.0, min(1.0, sum(a.u[i] * b.u[i] for i in range(3))))
        worst = max(worst, math.degrees(math.acos(cos)))
    return worst


def _edge_length(panel_frames, fold_tilts, half_width_mm: float, side: float) -> float:
    """側端ポリラインの長さ(面内の弧の凸側=長い側の判定用)。"""
    total = 0.0
    for frame, (near_tilt, far_tilt) in zip(panel_frames, fold_tilts):
        near = frame.near_run_mm + side * half_width_mm * math.tan(near_tilt)
        far = frame.far_run_mm + side * half_width_mm * math.tan(far_tilt)
        total += abs(far - near)
    return total


def choose_flange_side(
    rng: random.Random,
    panel_frames: list[BeadPanelFrame],
    fold_tilts: list[tuple[float, float]],
    half_width_mm: float,
) -> int:
    """②: 長手方向エッジが面内の弧なら外側(凸=長い側)、実質直線なら乱数。"""
    plus = _edge_length(panel_frames, fold_tilts, half_width_mm, 1.0)
    minus = _edge_length(panel_frames, fold_tilts, half_width_mm, -1.0)
    if abs(plus - minus) < FLANGE_STRAIGHT_EDGE_TOLERANCE_MM:
        return 1 if rng.random() < 0.5 else -1
    return 1 if plus > minus else -1


def fold_concavity_signs(panel_frames: list[BeadPanelFrame]) -> list[int]:
    """各折れ目の凹側の符号(+1=法線側が凹 / -1=反法線側が凹)。

    パネルi+1の走行方向がパネルiの法線側へ傾いていれば(u_{i+1}・n_i > 0)、
    法線側で面が閉じる=法線側が凹。
    """
    signs = []
    for a, b in zip(panel_frames, panel_frames[1:]):
        normal = _normalize(_cross(a.u, a.v))
        dot = sum(b.u[i] * normal[i] for i in range(3))
        signs.append(1 if dot > 0 else -1)
    return signs


def max_flange_height_mm(
    panel_frames: list[BeadPanelFrame], direction: int, bend_radius_mm: float
) -> float:
    """方向directionのフランジ高さの上限。凹側の折れ目を跨ぐなら R_bend - 2mm。"""
    if any(s == direction for s in fold_concavity_signs(panel_frames)):
        return bend_radius_mm - FLANGE_CONCAVE_CLEARANCE_MM
    return float("inf")


def sample_flange(
    rng: random.Random,
    panel_frames: list[BeadPanelFrame],
    fold_tilts: list[tuple[float, float]],
    half_width_mm: float,
    bend_radius_mm: float,
) -> FlangeParams | None:
    """このパネル列に載るフランジを1つサンプリングする。成立しなければNone。

    方向は凹側交差の無い側を優先(高さの自由度が最大)。両方向に凹側交差がある
    (S字)場合は h <= R_bend - 2 を満たす範囲でサンプリングし、下限10mmを
    割るなら不成立(ビードへフォールバック)。
    """
    side = choose_flange_side(rng, panel_frames, fold_tilts, half_width_mm)
    directions = [1, -1] if rng.random() < 0.5 else [-1, 1]
    directions.sort(key=lambda d: 0 if math.isinf(
        max_flange_height_mm(panel_frames, d, bend_radius_mm)) else 1)
    for direction in directions:
        h_max = min(FLANGE_HEIGHT_RANGE_MM[1],
                    max_flange_height_mm(panel_frames, direction, bend_radius_mm))
        if h_max < FLANGE_HEIGHT_RANGE_MM[0]:
            continue
        height = rng.uniform(FLANGE_HEIGHT_RANGE_MM[0], h_max)
        root_radius = rng.uniform(*FLANGE_ROOT_RADIUS_RANGE_MM)
        if height < root_radius + 2.0:  # 根本Rの上に真っ直ぐな壁が残ること
            root_radius = max(MIN_NEUTRAL_PLANE_RADIUS_MM, height - 2.0)
        return FlangeParams(height_mm=height, root_radius_mm=root_radius,
                            side=side, direction=direction)
    return None


def plan_flange_on_surface(
    panel_frames: list[BeadPanelFrame],
    flange: FlangeParams,
    *,
    half_width_mm: float,
    fold_tangents: list[tuple[float, float]],
) -> FlangeSurfacePlan:
    """フランジの配置情報(中心線標本点・プローブ点)を解析的に求める。

    中心線はビードと同じ「平坦区間の標本点 -> スプライン -> 投影」方式。端接線の
    補助点は**隣接点間の内分点**として挿入する(端の外側に置くとパネル先頭点と
    逆行して自己交差スプラインになる — spike_flange_over_bendsで実測、SS13/SS14)。
    """

    def at(frame: BeadPanelFrame, run: float, width: float) -> Vec3:
        return tuple(frame.origin[i] + run * frame.u[i] + width * frame.v[i] for i in range(3))

    centre: list[Vec3] = []
    flats: list[tuple[int, float, float]] = []
    for idx, frame in enumerate(panel_frames):
        near_cut, far_cut = fold_tangents[idx]
        lo = frame.near_run_mm + near_cut + FLAT_CLEARANCE_MM
        hi = frame.far_run_mm - far_cut - FLAT_CLEARANCE_MM
        if hi - lo < 1.0:
            continue
        flats.append((idx, lo, hi))
        for k in range(CENTRELINE_POINTS_PER_PANEL):
            centre.append(at(frame, lo + (hi - lo) * k / (CENTRELINE_POINTS_PER_PANEL - 1), 0.0))
    if len(flats) < len(panel_frames):
        missing = [i for i in range(len(panel_frames)) if i not in [f[0] for f in flats]]
        raise ValueError(
            f"flange centreline has no flat stretch on panel(s) {missing} -- the bend fillets "
            "consume those panels. Infeasible; not attempting construction."
        )
    pts = [centre[0]]
    for q in centre[1:]:
        if math.dist(pts[-1], q) > 0.01:
            pts.append(q)
    if len(pts) < 3:
        raise ValueError("flange centreline collapsed to fewer than 3 sample points. Infeasible.")

    def lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
        return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))

    pts = [pts[0], lerp(pts[0], pts[1], 0.15)] + pts[1:-1] + [lerp(pts[-2], pts[-1], 0.85), pts[-1]]

    # 壁の根本 = 拡張後の外周のわずか内側
    edge_offset = half_width_mm + flange.extension_mm - FLANGE_EDGE_INSET_MM

    mid_idx, mid_lo, mid_hi = flats[len(flats) // 2]
    mid_frame = panel_frames[mid_idx]
    mid_run = (mid_lo + mid_hi) / 2.0
    normal = _normalize(_cross(mid_frame.u, mid_frame.v))
    base_edge = at(mid_frame, mid_run, flange.side * edge_offset)
    wall_top = tuple(
        base_edge[i] + flange.direction * flange.height_mm * normal[i] for i in range(3)
    )
    root_keep = [at(panel_frames[i], (lo + hi) / 2.0, 0.0) for i, lo, hi in flats]

    return FlangeSurfacePlan(
        edge_offset_mm=edge_offset,
        centreline_points=pts,
        side_probe=base_edge,
        wall_top_probe=wall_top,
        root_keep_points=root_keep,
        wall_keep_point=wall_top,
    )

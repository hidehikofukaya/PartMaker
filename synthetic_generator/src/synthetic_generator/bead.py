"""ビード補強(線状プレス補強)の断面パラメータとセル分割(roadmap SS4.4 / SS6.25)。

フランジ(境界エッジを曲げる)とは別種のプリミティブで、一般面の内部に入れる線状の
段差である。中立面表現では「面の局所的なオフセット」として現れるため、既存のジョグ
構築と同じ原理 — 平面セルをシャープに結合してからエッジフィレット — で作れる。

構成(ユーザー確定、2026-08-21): 走行方向に走る1本のビードを幅方向中央に置き、断面は
台形(平地・立上り・頂部・立下り・平地の5ストリップ)。曲げをまたいで走らせるのが目的
なので、各パネル(flat1/ランプ/flat2)を同じ5ストリップに分割し、頂部だけを深さぶん
オフセットする。

**曲げ部での頂部の折れ目位置**: 隣接する2パネルの法線をna/nbとすると、両方の面を
深さdだけオフセットした平面同士の交線は、元の折れ目から `d * (na+nb)/(1+na.nb)`
(マイターオフセット)だけずれる。この式は na=nb(折れなし)で d*na に、直角で
d*(na+nb)(長さ d*sqrt2 = d/cos45度)に退化する、通常のオフセットポリラインの
マイター式そのもの。

**セルの平面性**: 平地・頂部の各セルは常に平面(元パネル/オフセット面のどちらか一方に
4隅とも乗るため)。立上り・立下りのセルも、パネルの走行方向が幅方向成分を持たない限り
平面になる。

例外はランプ(横方向オフセットを吸収するため台形になっているパネル)の壁セルで、ここは
**幾何的に捻れる**(双曲放物面になる)。ランプの走行方向は横方向オフセットを吸収する
ぶん幅方向成分を持つ一方、頂部のマイターオフセットは各パネルの法線の和で決まるため
幅方向成分を持たない — この差だけ、壁の上辺と下辺の向きがずれる。実物のビードが
曲げをまたぐ場合も壁は同様に捻れるので、これは近似誤差ではなく正しい幾何である
(実測で1〜2mm程度)。CATIAの`Fill`が非平面の4辺輪郭を受けられるかは実機で検証する。
"""

from __future__ import annotations

import dataclasses
import math
import random

from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM, Vec3

# ユーザー確定(2026-08-21)のサンプリングレンジ。中立面表現なので深さ=中立面のオフセット量、
# 壁角度は一般面からの立ち上がり角(ジョグのランプ角と同じ意味)。
BEAD_DEPTH_RANGE_MM = (4.0, 10.0)
BEAD_TOP_WIDTH_RANGE_MM = (20.0, 40.0)
BEAD_WALL_ANGLE_RANGE_DEG = (40.0, 60.0)

# ビードのフットプリント外側から、パネルの外形(半幅)までに残すべき平地の余白。
# 縦フィレットの後退量を吸収するためのもので、実行可能性は builder 側で改めて検査する。
BEAD_SIDE_MARGIN_MM = 5.0


@dataclasses.dataclass(frozen=True)
class BeadParams:
    depth_mm: float
    top_width_mm: float
    wall_angle_deg: float
    bend_radius_mm: float  # 縦4本の折れ目(壁の上下)に共通の半径

    @property
    def wall_run_mm(self) -> float:
        """壁の幅方向への投影長(平地から頂部までに幅方向で消費する距離)。"""
        return self.depth_mm / math.tan(math.radians(self.wall_angle_deg))

    @property
    def half_footprint_mm(self) -> float:
        """ビードが幅方向に占める範囲の半分(頂部の半幅 + 壁の投影長)。"""
        return self.top_width_mm / 2.0 + self.wall_run_mm

    @property
    def wall_slant_mm(self) -> float:
        """壁そのものの長さ(斜辺)。縦フィレットの後退量がこれを超えると成立しない。"""
        return self.depth_mm / math.sin(math.radians(self.wall_angle_deg))


def sample_bead(rng: random.Random) -> BeadParams:
    """ビード断面を1つサンプリングする。

    bend_radius_mmは中立面R最小4mmを下限とし、壁の斜辺長の半分(壁の上下2つの
    フィレットが壁の上で重ならない条件)を上限にする。上限が4mm未満になる組み合わせは
    あえて4mmを返し、builder側の事前チェックで明示的にInfeasibleとして弾かれるように
    する(templates/parallel_same_offset.pyと同じ設計)。
    """
    depth = rng.uniform(*BEAD_DEPTH_RANGE_MM)
    top_width = rng.uniform(*BEAD_TOP_WIDTH_RANGE_MM)
    wall_angle = rng.uniform(*BEAD_WALL_ANGLE_RANGE_DEG)

    half_angle_tangent = math.tan(math.radians(wall_angle) / 2.0)
    slant = depth / math.sin(math.radians(wall_angle))
    max_radius = slant / (2.0 * half_angle_tangent) if half_angle_tangent > 1e-9 else slant
    bend_radius = rng.uniform(
        MIN_NEUTRAL_PLANE_RADIUS_MM, max(MIN_NEUTRAL_PLANE_RADIUS_MM, max_radius)
    )
    return BeadParams(
        depth_mm=depth,
        top_width_mm=top_width,
        wall_angle_deg=wall_angle,
        bend_radius_mm=bend_radius,
    )


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: Vec3, k: float) -> Vec3:
    return (v[0] * k, v[1] * k, v[2] * k)


def _dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(v: Vec3) -> Vec3:
    length = math.sqrt(_dot(v, v))
    if length < 1e-9:
        raise ValueError(f"cannot normalize near-zero vector: {v}")
    return _scale(v, 1.0 / length)


def _panel_axes(corners: list[Vec3]) -> tuple[Vec3, Vec3, Vec3]:
    """パネル4隅([near,-hw],[near,+hw],[far,+hw],[far,-hw])から(走行方向u, 幅方向w, 法線n)。

    法線は全パネルで同じ式 cross(u, w) を使う。u自体が締結点フレームで符号解決済みなので、
    これだけで隣接パネル間の向きが揃う(個別の符号合わせは不要)。
    """
    width_dir = _normalize(_sub(corners[1], corners[0]))
    near_center = _scale(_add(corners[0], corners[1]), 0.5)
    far_center = _scale(_add(corners[2], corners[3]), 0.5)
    run_dir = _normalize(_sub(far_center, near_center))
    return run_dir, width_dir, _normalize(_cross(run_dir, width_dir))


def _miter_offset(normal_a: Vec3, normal_b: Vec3, depth_mm: float) -> Vec3:
    """法線na/nbの2面を深さd(それぞれの法線方向)にオフセットしたとき、交線が動く量。"""
    denominator = 1.0 + _dot(normal_a, normal_b)
    if denominator < 1e-9:
        raise ValueError("adjacent panels fold back on themselves; bead offset is undefined")
    return _scale(_add(normal_a, normal_b), depth_mm / denominator)


def _top_level_shifts(
    axes: list[tuple[Vec3, Vec3, Vec3]], bead: BeadParams
) -> tuple[list[Vec3], list[Vec3]]:
    """各パネル境界で頂部が動く量(near側, far側)。外側の端は自分の法線方向、
    内側(折れ目)は隣接パネルとのマイターオフセット。"""
    normals = [axis[2] for axis in axes]
    near_offsets = [_scale(normals[0], bead.depth_mm)]
    far_offsets = []
    for i in range(len(axes) - 1):
        shift = _miter_offset(normals[i], normals[i + 1], bead.depth_mm)
        far_offsets.append(shift)
        near_offsets.append(shift)
    far_offsets.append(_scale(normals[-1], bead.depth_mm))
    return near_offsets, far_offsets


def bead_cells(panel_corner_sets: list[list[Vec3]], bead: BeadParams) -> list[list[Vec3]]:
    """メイン形状のパネル群を、ビード断面の5ストリップに分割したセル群へ置き換える。

    入力の各パネルは`classify.end_panel_corners`の規約([near,-hw],[near,+hw],[far,+hw],
    [far,-hw])に従う4隅。出力も同じ規約の4隅を持つセルのリストで、そのまま
    `rect_fill`+`join`に流せる(パネルあたり5セル)。

    ビードは各パネルの幅中心線に沿って置く。隣接パネルは折れ目の辺を厳密に共有して
    いる(ramp_cornersがflat1/flat2の辺をそのまま使う構築なので)ため、幅中心・半幅とも
    境界で一致し、ビードは折れ目をまたいで自動的に連続する。横方向オフセットで
    ランプが台形になっていても、幅中心線で切った分割線は互いに平行なのでセルは平面のまま。
    """
    if not panel_corner_sets:
        raise ValueError("panel_corner_sets must not be empty")

    axes = [_panel_axes(corners) for corners in panel_corner_sets]
    near_offsets, far_offsets = _top_level_shifts(axes, bead)

    half_top = bead.top_width_mm / 2.0
    half_foot = bead.half_footprint_mm

    cells: list[list[Vec3]] = []
    for corners, (_, width_dir, _normal), near_shift, far_shift in zip(
        panel_corner_sets, axes, near_offsets, far_offsets
    ):
        near_center = _scale(_add(corners[0], corners[1]), 0.5)
        far_center = _scale(_add(corners[2], corners[3]), 0.5)

        def near(width: float, lifted: bool = False, _c=near_center, _s=near_shift) -> Vec3:
            point = _add(_c, _scale(width_dir, width))
            return _add(point, _s) if lifted else point

        def far(width: float, lifted: bool = False, _c=far_center, _s=far_shift) -> Vec3:
            point = _add(_c, _scale(width_dir, width))
            return _add(point, _s) if lifted else point

        cells.extend(
            [
                # 平地(低い側)
                [corners[0], near(-half_foot), far(-half_foot), corners[3]],
                # 立上り
                [near(-half_foot), near(-half_top, True), far(-half_top, True), far(-half_foot)],
                # 頂部
                [
                    near(-half_top, True),
                    near(half_top, True),
                    far(half_top, True),
                    far(-half_top, True),
                ],
                # 立下り
                [near(half_top, True), near(half_foot), far(half_foot), far(half_top, True)],
                # 平地(高い側)
                [near(half_foot), corners[1], corners[2], far(half_foot)],
            ]
        )
    return cells


def bead_fits(panel_corner_sets: list[list[Vec3]], bead: BeadParams) -> bool:
    """このパネル群にビードを入れられるか(幅方向・走行方向の両方を検査する)。

    1. **幅方向**: 横方向オフセットがある一般ケースでは、パネルごとに幅の中心が異なる
       (ランプが台形になる)。ビードは各パネルの幅中心に置くので、パネル境界(near/far)
       ごとの実際の半幅がフットプリント+余白以上あればよい。
    2. **走行方向**: 頂部の折れ目はマイターオフセットぶん走行方向にずれる。折れ角が
       180度に近いとマイター量が発散する(式の分母 1+na.nb が0に近づく)ため、頂部
       ストリップの走行長が0以下に潰れる、あるいは反転することがある — 実測で、幅の
       条件を満たすサンプルの11%がこれに該当し、CATIA側のjoinがそこで失敗していた
       (roadmap SS6.25)。頂部の両端には折れ目のフィレットが載るので、走行長は最低でも
       R2本ぶんを要求する。
    """
    required = bead.half_footprint_mm + BEAD_SIDE_MARGIN_MM
    for corners in panel_corner_sets:
        near_half = math.dist(corners[0], corners[1]) / 2.0
        far_half = math.dist(corners[2], corners[3]) / 2.0
        if min(near_half, far_half) < required:
            return False

    axes = [_panel_axes(corners) for corners in panel_corner_sets]
    try:
        near_offsets, far_offsets = _top_level_shifts(axes, bead)
    except ValueError:
        return False  # 折れ角が180度ちょうどに近く、オフセット面の交線が定義できない

    for corners, (run_dir, _width_dir, _normal), near_shift, far_shift in zip(
        panel_corner_sets, axes, near_offsets, far_offsets
    ):
        near_center = _scale(_add(corners[0], corners[1]), 0.5)
        far_center = _scale(_add(corners[2], corners[3]), 0.5)
        base_run = math.dist(near_center, far_center)
        top_run = base_run + _dot(_sub(far_shift, near_shift), run_dir)
        if top_run < 2.0 * bead.bend_radius_mm:
            return False
    return True

"""OCCT(pythonocc-core)で中立面STEPを直接生成するバックエンド(2026-09-04、刷新S1/S2)。

`gsd_build.SyntheticPartBuilder`(CATIA/DELMIA COM)と同じインターフェースを持ち、
ライセンス・GUI無しで動く。設計の根拠は docs/HANDOVER_partmaker_renewal.md §3。

## 構築原理: 「1本の断面を中心線に沿って掃引する」

自由折れ目チェーンの折れ目軸は**全て共通方向 w に平行**である
(`classify.free_fold_seed` のw平行構成。傾き摂動を0にすると厳密に平行になる —
実測で軸間角の最大 0.07度、摂動ありでは最大 7.3度)。したがって基準面は

    「w に直交する平面内の2D中心線(直線-円弧-直線-円弧-直線)を、
      断面プロファイルとともに掃引したもの」

として厳密に書ける。掃引は2種類しかない:

* 直線区間 -> `BRepPrimAPI_MakePrism`(平行移動)
* 曲げ区間 -> `BRepPrimAPI_MakeRevol`(軸 = (曲げ中心, w) まわりの回転)

**w は大域固定方向なので、掃引フレームが捻れることが原理的に起こり得ない。**
Gemini版が `BRepOffsetAPI_MakePipeShell` の多断面補間で螺旋リボンになった問題
(docs/REVIEW_bead_sweep_twist.md)は、この構成では発生しない。

面の種類も構成的に決まる:

| 断面要素 | 直線区間 | 曲げ区間 |
|---|---|---|
| 直線(平地・壁・頂部・フランジ壁) | 平面 | 円錐 / 円筒 / 平面(円環) |
| 円弧(稜線R・足R・フランジ根本R) | 円筒(斜め押し出し) | トーラス |

エッジは全て直線か円弧になる(引継ぎ書 §3.4 ゲートA)。断面の平面は w を含むため、
横ズレ(シアー角γ<=5度)がある部品では直線区間の円弧要素だけが厳密な円筒ではなく
**斜め押し出し面**になる。エッジは厳密な円のままなので下流の抽出には影響しない。

## ビード / フランジ

ビードは断面を差し替えることで載る(壁+稜線R+足Rを閉じた9要素のG1プロファイル)。
両端はランアウト(短い直線区間での `BRepOffsetAPI_ThruSections(ruled=True)`)で
平地断面へ戻す。B-splineが出るのはこのランアウト2箇所だけ。
フランジは断面の端に「根本R + 壁」を足すだけなので全長で断面が変わらず、
ランアウトも不要(=B-spline 0)。

余肉カット(隅のR)は最後に角柱ツールでブーリアン減算する。締結点の穴は開けない
(ユーザー指定、2026-08-30 / 2026-09-04)。
"""

from __future__ import annotations

import dataclasses
import math
import os

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepLProp import BRepLProp_SLProps
from OCC.Core.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_Transform,
)
from OCC.Core.BRepFill import brepfill
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeRevol
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.GC import GC_MakeArcOfCircle, GC_MakeCircle
from OCC.Core.Interface import Interface_Static
from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TDataStd import TDataStd_Name
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.GCPnts import GCPnts_AbscissaPoint
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_VERTEX
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCC.Core.TopoDS import topods
from OCC.Core.XCAFApp import XCAFApp_Application
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool
from OCC.Core.GProp import GProp_GProps
from OCC.Core.gp import gp_Ax1, gp_Dir, gp_Pln, gp_Pnt, gp_Trsf, gp_Vec

from synthetic_generator.bead import BeadParams
from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM, FasteningPoint, Vec3
from synthetic_generator.flange import FLANGE_CONCAVE_CLEARANCE_MM, FlangeParams
from synthetic_generator.general_geometry import (
    BEAD_MIN_RUNOUT_MM,
    bead_placement,
    plan_general_two_point,
)
from synthetic_generator.rib import RIB_BEND_RADIUS_MM, RibParams

SEW_TOLERANCE_MM = 0.01
# ランアウト長(平地->ビード断面の遷移)。実物のビードの走り終いは深さの2〜3倍程度。
RUNOUT_DEPTH_RATIO = 2.0
MIN_RUNOUT_MM = 3.0
# 隅の余肉カットのツール角柱の張り出し(基準面の両側へこれだけ伸ばす)。カット域の
# 基準面は厳密に平面なので薄くてよい。厚くするとU字部品で反対側のパネルまで削る。
CUT_TOOL_HALF_DEPTH_MM = 5.0
# これ未満のエッジが出た部品は捨てる(引継ぎ書 §4.3 のゴミ幾何)。
MIN_EDGE_LENGTH_MM = 0.05
# 継ぎ目にしない角で、隣り合う壁の根本を頂点からどれだけ離すか(ベンドリリーフ)。
# これより近いと 2 本の根本フィレットが頂点で衝突して OCCT が収束しない(2026-09-08 実測)。
BOX_CORNER_RELIEF_MM = 6.0
# マイター点が余白の何倍まで伸びるのを許すか。超える隅は余白の半径で丸める
# (CADのストローク結合と同じ考え方。実車20の凸包には約45度の隅がある)。
FLAT_PLATE_MITER_LIMIT = 1.8
# 隅Rどうしの間に残す直線の最小長。0.05mmまで許すと縫合後に極小の
# ゴミエッジが出る(実測 0.0134mm)。
FLAT_PLATE_MIN_STRAIGHT_MM = 1.0
# 分岐部品で、折ったあとの腕どうしに要る最小すきま。
ARM_CLEARANCE_MM = 1.0
# 腕の根元(曲げ線の両端)の逃がしノッチ半径。応力集中を防ぐための最低R。
# 腕の先の隅を落とす量。矩形をある程度保つため最小Rに留める。
ARM_TIP_RELIEF_MM = MIN_NEUTRAL_PLANE_RADIUS_MM
# ガセット(腕どうしを繋ぐ面取り壁)の曲げRと最小高さ。
GUSSET_BEND_R_MM = MIN_NEUTRAL_PLANE_RADIUS_MM
GUSSET_MIN_HEIGHT_MM = 10.0
# タブの自由端と相手の腕との隙間(重ね接合。溶接前提)。
GUSSET_LAP_GAP_MM = 1.0
# エッジが1本の直線/円弧から外れてよい上限[mm](引継ぎ書 §3.4 ゲートA、0.25t の
# 最も厳しい側 t=1.0mm に合わせる)。リブの稜線フィレットは頂点ブレンドの境界が
# 解析曲線にならないことがあり、実測で最大0.797mm外れた(2026-09-04)。
MAX_PRIMITIVE_DEVIATION_MM = 0.25
# 隣接する2面の法線がなす角の上限[度]。板金の中立面は自分の上に折り返らないので、
# これを超えるエッジがあれば掃引の破綻(ねじれ・面の裏返り)を意味する。
# 実測(健全な300部品): 最大 112度。崩壊部品では 180度近くになる。
MAX_DIHEDRAL_TURN_DEG = 150.0
# 締結点が面から外れてよい上限[mm]。部品が自分の座面に届かなくなる崩壊を拾う
# (2026-09-04実測: 300部品中1件のリブ部品で23.9mm外れていた)。
# 帯幅の絞り(実車014型)。絞りは**逃げより手前**、ビードが通っている途中で起こす —
# 実車014も絞りは中間パネル(ビードあり)で、孤立点のパネルは絞りきった一定幅。
TAPER_LENGTH_SHARE = 0.80    # 最後の直線区間のうち絞りに使う割合
TAPER_STEPS = 8              # smoothstep の刻み数(1段だと稜線が両端で折れる)
TAPER_CLEAR_BEARINGS = 3.0   # 端から何 bearing ぶんを絞りきった平地にするか


def _taper_width(s: float, t0: float, t1: float, wide: float, narrow: float) -> float:
    """絞り区間の位置 s における半幅。smoothstep なので両端で傾きが 0 になり、
    フランジの稜線が一定幅の区間となめらかに繋がる。"""
    x = min(1.0, max(0.0, (s - t0) / max(1e-9, t1 - t0)))
    return wide + (narrow - wide) * x * x * (3.0 - 2.0 * x)
MAX_FASTENING_OFFSET_MM = 0.1


@dataclasses.dataclass(frozen=True)
class GeneratedPart:
    stp_path: str
    catpart_path: str
    face_labels: tuple = ()   # ({name, role, geo, area_mm2, radius_mm, centroid}, ...)


# ---------------------------------------------------------------- ベクトル(純Python)


def _normalize(v: Vec3) -> Vec3:
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if length < 1e-12:
        raise ValueError("cannot normalise a zero-length vector")
    return (v[0] / length, v[1] / length, v[2] / length)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _add(*vs: Vec3) -> Vec3:
    return (sum(v[0] for v in vs), sum(v[1] for v in vs), sum(v[2] for v in vs))


def _scale(v: Vec3, k: float) -> Vec3:
    return (v[0] * k, v[1] * k, v[2] * k)


def _rotate(v: Vec3, axis: Vec3, angle: float) -> Vec3:
    """ロドリゲスの回転公式(axisは単位ベクトル)。"""
    c, s = math.cos(angle), math.sin(angle)
    k = _cross(axis, v)
    d = _dot(axis, v)
    return tuple(v[i] * c + k[i] * s + axis[i] * d * (1.0 - c) for i in range(3))


def _sub_axis(fold) -> Vec3:
    return fold["axis"]


def _rotate_about(point: Vec3, centre: Vec3, axis: Vec3, angle: float) -> Vec3:
    return _add(centre, _rotate(_add(point, _scale(centre, -1.0)), axis, angle))


def _signed_angle_about(v_from: Vec3, v_to: Vec3, axis: Vec3) -> float:
    """axisに直交する成分どうしのなす符号付き角。"""
    a = _add(v_from, _scale(axis, -_dot(v_from, axis)))
    b = _add(v_to, _scale(axis, -_dot(v_to, axis)))
    return math.atan2(_dot(_cross(a, b), axis), _dot(a, b))


# ---------------------------------------------------------------- 断面プロファイル(2D)
#
# 断面は (y, z) の2D座標で持つ。y = 幅方向(掃引フレームのey = ±w)、z = 面法線方向。
# 要素は ("line", p0, p1, role) か ("arc", p0, pmid, p1, role)。
# roleはSTEPの面名になる。

Elem = tuple


def _flat_section(y_breaks: list[float]) -> list[Elem]:
    """z=0の平地断面。`y_breaks`でビード断面と要素数・対応順を揃える(ランアウトの
    ruled loftは断面どうしの要素が1対1に対応している必要があるため)。"""
    return [
        ("line", (y_breaks[i], 0.0), (y_breaks[i + 1], 0.0), f"base_{i}")
        for i in range(len(y_breaks) - 1)
    ]


def _bead_section(bead, lift: int, half_width_mm: float, *, role: str = "bead") -> tuple[list[Elem], list[float]]:
    """ビード断面(9要素)と、対応する平地断面のyブレークポイント(10点)を返す。

    幾何(右半分、liftを掛ける前):
      平地 z=0 (|y| >= yf+sb) / 足R(半径R、中心 (yf+sb, R)) / 壁(角度θ) /
      頂稜線R(半径R、中心 (yt-sb, d-R)) / 頂部 z=d (|y| <= yt-sb)
    ここで yt=頂部半幅, wr=d/tanθ, yf=yt+wr(フットプリント半幅),
    sb=R*tan(θ/2)(稜線フィレットの後退量)。
    """
    theta = math.radians(bead.wall_angle_deg)
    depth = bead.depth_mm
    radius = bead.ridge_radius_mm
    yt = bead.top_width_mm / 2.0
    yf = yt + depth / math.tan(theta)
    sb = radius * math.tan(theta / 2.0)

    if yt - sb <= 0.1:
        raise ValueError(
            f"bead top ridge fillets (setback {sb:.2f}mm each) consume the whole top width "
            f"({bead.top_width_mm:.1f}mm). Infeasible; not attempting construction."
        )
    if yf + sb > half_width_mm - 0.5:
        raise ValueError(
            f"bead footprint ({yf + sb:.1f}mm half) does not fit in the half width "
            f"({half_width_mm:.1f}mm). Infeasible; not attempting construction."
        )

    cos_t, sin_t = math.cos(theta), math.sin(theta)
    y_breaks = [
        -half_width_mm,
        -(yf + sb),
        -(yf - sb * cos_t),
        -(yt + sb * cos_t),
        -(yt - sb),
        yt - sb,
        yt + sb * cos_t,
        yf - sb * cos_t,
        yf + sb,
        half_width_mm,
    ]

    def p(y: float, z: float) -> tuple[float, float]:
        return (y, lift * z)

    # 足Rの中心 (yf+sb, R)、頂稜線Rの中心 (yt-sb, d-R)。円弧の中点は半角方向。
    foot_c = (yf + sb, radius)
    top_c = (yt - sb, depth - radius)
    foot_mid = (foot_c[0] - radius * math.sin(theta / 2.0), foot_c[1] - radius * math.cos(theta / 2.0))
    top_mid = (top_c[0] + radius * math.sin(theta / 2.0), top_c[1] + radius * math.cos(theta / 2.0))

    section = [
        ("line", p(-half_width_mm, 0.0), p(-(yf + sb), 0.0), "base_l"),
        ("arc", p(-(yf + sb), 0.0), p(-foot_mid[0], foot_mid[1]),
         p(-(yf - sb * cos_t), sb * sin_t), f"{role}_foot_l"),
        ("line", p(-(yf - sb * cos_t), sb * sin_t),
         p(-(yt + sb * cos_t), depth - sb * sin_t), f"{role}_wall_l"),
        ("arc", p(-(yt + sb * cos_t), depth - sb * sin_t), p(-top_mid[0], top_mid[1]),
         p(-(yt - sb), depth), f"{role}_ridge_l"),
        ("line", p(-(yt - sb), depth), p(yt - sb, depth), f"{role}_top"),
        ("arc", p(yt - sb, depth), p(top_mid[0], top_mid[1]),
         p(yt + sb * cos_t, depth - sb * sin_t), f"{role}_ridge_r"),
        ("line", p(yt + sb * cos_t, depth - sb * sin_t),
         p(yf - sb * cos_t, sb * sin_t), f"{role}_wall_r"),
        ("arc", p(yf - sb * cos_t, sb * sin_t), p(foot_mid[0], foot_mid[1]),
         p(yf + sb, 0.0), f"{role}_foot_r"),
        ("line", p(yf + sb, 0.0), p(half_width_mm, 0.0), "base_r"),
    ]
    return section, y_breaks


def _bead_bump(bead, lift: int, y_c: float, role: str) -> tuple[list[Elem], float, float]:
    """1本のビードの「足R - 壁 - 稜線R - 頂部 - 稜線R - 壁 - 足R」7要素を、中心 y_c に置く。
    `_bead_section` の中央部分と同じ幾何。戻り値は (要素列, 左端 y, 右端 y)。"""
    theta = math.radians(bead.wall_angle_deg)
    depth, radius = bead.depth_mm, bead.ridge_radius_mm
    yt = bead.top_width_mm / 2.0
    yf = yt + depth / math.tan(theta)
    sb = radius * math.tan(theta / 2.0)
    if yt - sb <= 0.1:
        raise ValueError("bead top ridge fillets consume the whole top width. Infeasible.")
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    # 壁は z = sb*sin(θ) から depth - sb*sin(θ) まで。浅くて稜線Rが大きいと**反転**し、
    # 掃引すると法線が裏返って隣の面と 180 度になる(2026-09-08 実測: 3割が落ちた)。
    if depth - 2.0 * sb * sin_t <= 0.2:
        raise ValueError(
            f"the bead wall vanishes between the ridge and foot fillets "
            f"(depth {depth:.1f}mm, setback {sb:.1f}mm each). Infeasible.")
    foot_c = (yf + sb, radius)
    top_c = (yt - sb, depth - radius)
    foot_mid = (foot_c[0] - radius * math.sin(theta / 2.0),
                foot_c[1] - radius * math.cos(theta / 2.0))
    top_mid = (top_c[0] + radius * math.sin(theta / 2.0),
               top_c[1] + radius * math.cos(theta / 2.0))

    def p(y: float, z: float) -> tuple[float, float]:
        return (y_c + y, lift * z)

    section = [
        ("arc", p(-(yf + sb), 0.0), p(-foot_mid[0], foot_mid[1]),
         p(-(yf - sb * cos_t), sb * sin_t), f"{role}_foot_l"),
        ("line", p(-(yf - sb * cos_t), sb * sin_t),
         p(-(yt + sb * cos_t), depth - sb * sin_t), f"{role}_wall_l"),
        ("arc", p(-(yt + sb * cos_t), depth - sb * sin_t), p(-top_mid[0], top_mid[1]),
         p(-(yt - sb), depth), f"{role}_ridge_l"),
        ("line", p(-(yt - sb), depth), p(yt - sb, depth), f"{role}_top"),
        ("arc", p(yt - sb, depth), p(top_mid[0], top_mid[1]),
         p(yt + sb * cos_t, depth - sb * sin_t), f"{role}_ridge_r"),
        ("line", p(yt + sb * cos_t, depth - sb * sin_t),
         p(yf - sb * cos_t, sb * sin_t), f"{role}_wall_r"),
        ("arc", p(yf - sb * cos_t, sb * sin_t), p(foot_mid[0], foot_mid[1]),
         p(yf + sb, 0.0), f"{role}_foot_r"),
    ]
    return section, y_c - (yf + sb), y_c + (yf + sb)


def multi_bead_section(beads, lift: int, half_width_mm: float,
                       ext_neg: float = 0.0, ext_pos: float = 0.0,
                       min_land_mm: float = 2.0):
    """幅方向に複数のビードを並べた断面(全長を走る)。beads = [(中心 y, BeadParams), ...]。

    大型パネル族(2026-09-08)。実車の大きなパネルは断面に 2〜4 本のビードを持つ
    (002-033 は断面R 59 か所)。既存の `_bead_section` は 1 本だけだった。
    戻り値は (断面, 平地のブレークポイント, 各ビードの [左端, 右端])。
    """
    lo, hi = -half_width_mm - ext_neg, half_width_mm + ext_pos
    items = sorted(beads, key=lambda b: b[0])
    section: list = []
    breaks = [lo]
    spans: list = []
    cursor = lo
    for i, (y_c, bead) in enumerate(items):
        bump, left, right = _bead_bump(bead, lift, y_c, f"bead{i}")
        if left - cursor < min_land_mm:
            raise ValueError(f"bead {i} leaves no land on its left ({left - cursor:.1f}mm). "
                             "Infeasible.")
        section.append(("line", (cursor, 0.0), (left, 0.0), f"base_{i}"))
        section += bump
        breaks += [left, right]
        spans.append([left, right])
        cursor = right
    if hi - cursor < min_land_mm:
        raise ValueError("the last bead leaves no land on its right. Infeasible.")
    section.append(("line", (cursor, 0.0), (hi, 0.0), "base_r"))
    breaks.append(hi)
    return section, breaks, spans


def _flange_walls(flange: FlangeParams, half_width_mm: float):
    """基準面の端に継ぐ「根本R + 壁」を (左側の要素列, 右側の要素列) で返す。

    左の壁は上端から根本へ、右の壁は根本から上端へ向ける — 要素列は左から右へ
    連続していないと `_edges_of` が繋がらないため。
    """
    r, h, e = flange.root_radius_mm, flange.height_mm, flange.direction
    if h <= r + 1.0:
        raise ValueError(
            f"flange height ({h:.1f}mm) leaves no straight wall above the root radius "
            f"({r:.1f}mm). Infeasible; not attempting construction."
        )
    hw = half_width_mm
    q = math.sqrt(0.5)                       # 45度方向(円弧の中点)
    left_mid = (-(hw + r * q), e * r * (1.0 - q))
    right_mid = (hw + r * q, e * r * (1.0 - q))
    left = [("line", (-(hw + r), e * h), (-(hw + r), e * r), "flange_wall_l"),
            ("arc", (-(hw + r), e * r), left_mid, (-hw, 0.0), "flange_root_l")]
    right = [("arc", (hw, 0.0), right_mid, (hw + r, e * r), "flange_root_r"),
             ("line", (hw + r, e * r), (hw + r, e * h), "flange_wall_r")]
    return left, right


def _with_flange(section: list[Elem], flange: FlangeParams, half_width_mm: float) -> list[Elem]:
    """基準面の断面(平地でもビードでも)の両端に壁を継ぐ。実車014型で使う。"""
    left, right = _flange_walls(flange, half_width_mm)
    return [*left, *section, *right]


def _flange_section(flange: FlangeParams, half_width_mm: float) -> list[Elem]:
    """基準面 + 根本R(90度) + フランジ壁。根本Rは名目幅の**外側**に置くので、
    基準面の半幅は名目のまま保たれる(CATIA版のside_extensionは不要)。"""
    r = flange.root_radius_mm
    h = flange.height_mm
    if h <= r + 1.0:
        raise ValueError(
            f"flange height ({h:.1f}mm) leaves no straight wall above the root radius "
            f"({r:.1f}mm). Infeasible; not attempting construction."
        )
    if flange.both_sides:    # 実車014型: 左右どちらにも壁を立てる
        return _with_flange(
            [("line", (-half_width_mm, 0.0), (half_width_mm, 0.0), "base")],
            flange, half_width_mm)
    s = flange.side          # +1 = ey正側
    e = flange.direction     # +1 = 面法線側へ立てる
    w_edge = s * half_width_mm
    root_end = (s * (half_width_mm + r), e * r)
    # 中心 (w_edge, e*r) の90度円弧。中点は45度方向。
    mid = (w_edge + s * r * math.sin(math.pi / 4.0), e * r - e * r * math.cos(math.pi / 4.0))
    base = ("line", (-s * half_width_mm, 0.0), (w_edge, 0.0), "base")
    arc = ("arc", (w_edge, 0.0), mid, root_end, "flange_root")
    wall = ("line", root_end, (root_end[0], e * h), "flange_wall")
    return [base, arc, wall]


# ---------------------------------------------------------------- 3Dへの写像


class _Frame:
    """掃引の現在位置。origin = 中心線上の点、ey = 幅方向(±w)、ez = 面法線。"""

    __slots__ = ("origin", "ey", "ez")

    def __init__(self, origin: Vec3, ey: Vec3, ez: Vec3) -> None:
        self.origin, self.ey, self.ez = origin, ey, ez

    def point(self, yz: tuple[float, float]) -> Vec3:
        return _add(self.origin, _scale(self.ey, yz[0]), _scale(self.ez, yz[1]))

    def translated(self, vec: Vec3) -> "_Frame":
        return _Frame(_add(self.origin, vec), self.ey, self.ez)

    def rotated(self, centre: Vec3, axis: Vec3, angle: float) -> "_Frame":
        moved = _add(centre, _rotate(_add(self.origin, _scale(centre, -1.0)), axis, angle))
        return _Frame(moved, _rotate(self.ey, axis, angle), _rotate(self.ez, axis, angle))


def _normalize2(a):
    n = math.hypot(a[0], a[1])
    return (a[0] / n, a[1] / n) if n > 1e-12 else a


def _outward_normal_2d(a, b):
    """反時計回りの多角形で、辺 a->b の外向き法線。"""
    return _normalize2((b[1] - a[1], -(b[0] - a[0])))


def _convex_hull_2d(points):
    """反時計回りの凸包(monotone chain)。同一点はまとめる。"""
    pts = sorted(set((round(x, 6), round(y, 6)) for x, y in points))
    if len(pts) < 3:
        return pts

    def turn(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def build(seq):
        out: list = []
        for p in seq:
            while len(out) >= 2 and turn(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out

    return build(pts)[:-1] + build(list(reversed(pts)))[:-1]


def branch_frames(hub_xy, arms, *, origin: Vec3, hub_u: Vec3, hub_v: Vec3,
                  corner_radius, fillet_radius=None):
    """分岐部品の幾何を、OCCTを呼ばずに出す(族の締結点配置とビルダーが共有する)。

    戻り値: {"outline": 断面要素, "roots": {辺: (始点2D, 終点2D)},
             "normal": ハブ法線, "arms": {辺: {"a","b","tip","axis","width","normal"}}}
    腕の a,b は曲げ後の根本エッジ、tip は腕の伸びる向き、axis は折り軸(根本エッジの向き)、
    normal は腕面の法線(ハブ法線と連続する向き)。
    """
    normal = _normalize(_cross(hub_u, hub_v))

    def to_space(xy):
        return _add(origin, _add(_scale(hub_u, xy[0]), _scale(hub_v, xy[1])))

    outline, roots = _hub_outline(hub_xy, {a["edge"] for a in arms}, corner_radius,
                                  fillet_radius)
    frames = {}
    for arm in arms:
        index = arm["edge"]
        a, b = to_space(roots[index][0]), to_space(roots[index][1])
        axis = _normalize(_add(b, _scale(a, -1.0)))
        radius, angle = arm["radius_mm"], math.radians(arm["fold_deg"])
        # side = -1: 裏側(-法線)へ折る(既定、分岐族)。+1: 表側へ折る(実車1285-18の立ち上がり)。
        side = arm.get("side", -1)
        centre = _add(a, _scale(normal, side * radius))
        outward = _normalize(_cross(axis, normal))
        tip = _normalize(_add(_scale(outward, math.cos(angle)),
                              _scale(normal, side * math.sin(angle))))
        revolve_axis = axis if side < 0 else _scale(axis, -1.0)
        frames[index] = {
            "a": _rotate_about(a, centre, revolve_axis, angle),
            "b": _rotate_about(b, centre, revolve_axis, angle),
            "tip": tip, "axis": axis, "width": math.dist(a, b),
            "normal": _normalize(_cross(tip, axis)),
            "root_a": a, "root_b": b, "centre": centre, "angle": angle,
            "revolve_axis": revolve_axis, "side": side,
        }
    return {"outline": outline, "roots": roots, "normal": normal, "arms": frames,
            "to_space": to_space}


def _rotate_vec(v: Vec3, axis: Vec3, angle: float) -> Vec3:
    return _rotate_about(v, (0.0, 0.0, 0.0), axis, angle)


def step_frame(fr: dict, outline: dict, hub_normal: Vec3) -> dict:
    """段差腕(実車057の前側): 根本で rise 度上がった斜面を run だけ進み、根本と平行な軸で
    (rise - ledge_tilt) 度戻して、ハブとほぼ平行な棚を作る。棚の座標系を返す。

    戻り値: {"a2","b2"(斜面の先), "a3","b3"(棚の根本), "tip2"(棚の伸びる向き),
             "centre2","axis2","angle2"(2本目の曲げ: 回転軸の向きと正の角), "normal2"}
    """
    axis, tip, a, b = fr["axis"], fr["tip"], fr["a"], fr["b"]
    run, radius = outline["run_mm"], outline["ledge_radius_mm"]
    angle = math.radians(fr_deg(fr) - outline["ledge_tilt_deg"])
    a2 = _add(a, _scale(tip, run))
    b2 = _add(b, _scale(tip, run))
    outward = _normalize(_cross(axis, hub_normal))
    n_s = fr["normal"]
    best = None
    for sign in (1.0, -1.0):
        tip2 = _rotate_vec(tip, axis, sign * angle)
        if _dot(tip2, outward) <= _dot(tip, outward):
            continue                       # 戻す向きでない
        for m in (n_s, _scale(n_s, -1.0)):
            centre = _add(a2, _scale(m, radius))
            a3 = _rotate_about(a2, centre, axis, sign * angle)
            step = _add(a3, _scale(a2, -1.0))
            if _dot(step, tip) > 0.0 and _dot(step, tip2) > 0.0:
                best = (sign, centre, a3, tip2)
    if best is None:
        raise ValueError("no consistent second bend for the step arm. Infeasible.")
    sign, centre, a3, tip2 = best
    b3 = _rotate_about(b2, centre, axis, sign * angle)
    return {"a2": a2, "b2": b2, "a3": a3, "b3": b3, "tip2": tip2, "centre2": centre,
            "axis2": axis if sign > 0 else _scale(axis, -1.0), "angle2": angle,
            "normal2": _normalize(_cross(tip2, axis))}


def fr_deg(fr: dict) -> float:
    return math.degrees(fr["angle"])


# ---------------------------------------------------------------- 絞りの角(実車057の簡略版)

def tab_footprint_mm(radius_mm: float, root_r_mm: float) -> float:
    """半円タブ(半径 r)の根元に凹R(ρ)を付けたときの、タブが上端の辺で占める片側の幅。
    根元の円は上端の直線と外接し、タブの円とも外接するので、中心の s オフセットは
    sqrt((r+ρ)^2 - ρ^2) = sqrt(r^2 + 2 r ρ)。"""
    return math.sqrt(radius_mm * radius_mm + 2.0 * radius_mm * root_r_mm)


def tab_top_section(height_mm: float, tabs, s_hi: float, s_lo: float, root_r_mm: float):
    """壁の上端(t = height)を s_hi から s_lo へ向かって辿る断面要素列。途中に半円の溶接タブ
    (中心 (height, s_c)、半径 r)を、根元の凹R(root_r)つきで入れる。

    タブの根元を直角に落とすと応力集中で裂けるので、実務どおり最小Rの逃げを付ける
    (ユーザー指示 2026-09-06)。凹Rの中心は上端の外側 (height + ρ, s_c ± sqrt(r^2+2rρ))。
    """
    order = sorted(tabs, key=lambda t: -t[0])
    for s_c, r in order:
        f = tab_footprint_mm(r, root_r_mm)
        if s_c - f < s_lo + 0.5 or s_c + f > s_hi - 0.5:
            raise ValueError("a weld tab (with its root radius) runs off the wall. Infeasible.")
    for (s1, r1), (s2, r2) in zip(order, order[1:]):
        if s1 - tab_footprint_mm(r1, root_r_mm) < s2 + tab_footprint_mm(r2, root_r_mm) + 1.0:
            raise ValueError("weld tabs overlap. Infeasible.")
    h, rho = height_mm, root_r_mm
    section = []
    cursor = s_hi
    for s_c, r in order:
        f = tab_footprint_mm(r, rho)
        # 根元Rの中心(上端の外側)と、タブの円との接点
        k = r / (r + rho)
        t_touch = h + rho * k              # 接点の t (中心同士を結ぶ線上、タブ中心から r)
        s_touch = f * k
        # 右側の根元R: 直線の接点 (h, s_c+f) -> タブ円の接点 (t_touch, s_c + s_touch)
        c_r = (h + rho, s_c + f)
        m_r = _arc_mid_2d(c_r, rho, (h, s_c + f), (t_touch, s_c + s_touch))
        c_l = (h + rho, s_c - f)
        m_l = _arc_mid_2d(c_l, rho, (t_touch, s_c - s_touch), (h, s_c - f))
        section.append(("line", (h, cursor), (h, s_c + f)))
        section.append(("arc", (h, s_c + f), m_r, (t_touch, s_c + s_touch)))
        section.append(("arc", (t_touch, s_c + s_touch), (h + r, s_c), (t_touch, s_c - s_touch)))
        section.append(("arc", (t_touch, s_c - s_touch), m_l, (h, s_c - f)))
        cursor = s_c - f
    section.append(("line", (h, cursor), (h, s_lo)))
    return [e for e in section if e[0] != "line" or math.dist(e[1], e[2]) >= MIN_EDGE_LENGTH_MM]


def _split_top_line(section, height_mm: float, s_values):
    """壁の上端(t = height)の直線要素を、指定の s で分割する。深さ2のフランジの根本の
    頂点を輪郭に入れるため(頂点が無いと縫合したときに根本エッジが取り出せない)。"""
    want = sorted(s_values)
    if not want:
        return section
    out = []
    for elem in section:
        if elem[0] != "line" or abs(elem[1][0] - height_mm) > 1e-6 \
                or abs(elem[2][0] - height_mm) > 1e-6:
            out.append(elem)
            continue
        s_a, s_b = elem[1][1], elem[2][1]
        lo, hi = min(s_a, s_b), max(s_a, s_b)
        inside = [s for s in want if lo + MIN_EDGE_LENGTH_MM < s < hi - MIN_EDGE_LENGTH_MM]
        if not inside:
            out.append(elem)
            continue
        chain = [s_a] + (inside if s_a < s_b else list(reversed(inside))) + [s_b]
        for p, q in zip(chain, chain[1:]):
            out.append(("line", (height_mm, p), (height_mm, q)))
    return out


def _arc_mid_2d(centre, radius, p, q):
    """中心 centre、半径 radius の円上で p と q の間(短い方)の中点。"""
    u = ((p[0] + q[0]) / 2.0 - centre[0], (p[1] + q[1]) / 2.0 - centre[1])
    k = math.hypot(*u)
    if k < 1e-9:
        raise ValueError("degenerate arc. Infeasible.")
    return (centre[0] + u[0] / k * radius, centre[1] + u[1] / k * radius)


def drawn_notch_mm(fillet_mm: float, turn_rad: float) -> float:
    """継ぎ目の端のノッチ長。フィレットの接線長 R tan(θ/2) の 2 倍 + 4。
    接線長 + 2 だとフィレットの帯がノッチの角に触れて OCCT が収束しない(2026-09-06 実測:
    失敗12件のうち 2 倍にすると 12 件とも収束)。"""
    return 2.0 * fillet_mm * math.tan(turn_rad / 2.0) + 4.0


def drawn_tray_frames(hub_xy, walls: dict, *, origin: Vec3, hub_u: Vec3, hub_v: Vec3):
    """ハブ(凸四角形 v0..v3)の辺0(v0->v1)に壁B、辺1(v1->v2)に壁A、辺3(v3->v0)に壁C を立てる。
    A–B は頂点 v1 から、C–B は頂点 v0 から出る**平面の交線**(継ぎ目)で互いにトリムする。
    壁は裏側(-法線)へ折る。walls = {"A"|"B"|"C": {"fold_deg", "height_mm"}}(C は任意)。

    戻り値: {"normal", "to_space", "vertices", "walls": {key: {a, b, tip, axis, normal, width,
             height, ends: {"left"|"right": None | {"t","s","dt","ds","turn","P"}}}},
             "seams": {"AB": {...}, "CB": {...}}}
    継ぎ目の端 P は「2枚の壁の上端のうち低い方」に置き、高い方は上端を P へ向けて細らせる。
    """
    normal = _normalize(_cross(hub_u, hub_v))

    def to_space(xy):
        return _add(origin, _add(_scale(hub_u, xy[0]), _scale(hub_v, xy[1])))

    v = [to_space(q) for q in hub_xy]
    edges = {"B": (v[0], v[1]), "A": (v[1], v[2]), "C": (v[3], v[0])}
    frames = {}
    for key, spec in walls.items():
        a, b = edges[key]
        axis = _normalize(_add(b, _scale(a, -1.0)))
        angle = math.radians(spec["fold_deg"])
        outward = _normalize(_cross(axis, normal))
        tip = _normalize(_add(_scale(outward, math.cos(angle)), _scale(normal, -math.sin(angle))))
        frames[key] = {"a": a, "b": b, "axis": axis, "tip": tip, "angle": angle,
                       "normal": _normalize(_cross(tip, axis)), "width": math.dist(a, b),
                       "height": spec["height_mm"], "ends": {"left": None, "right": None}}
    seams = {}
    # (継ぎ目名, 壁X, Xのどちら側の端か, 壁Y, Yのどちら側の端か, 頂点)
    for name, kx, side_x, ky, side_y, vertex in (("AB", "A", "left", "B", "right", v[1]),
                                                  ("CB", "C", "right", "B", "left", v[0])):
        if kx not in frames or ky not in frames:
            continue
        fx, fy = frames[kx], frames[ky]
        d = _cross(fx["normal"], fy["normal"])
        if math.sqrt(_dot(d, d)) < 1e-6:
            raise ValueError("two walls are coplanar. Infeasible.")
        d = _normalize(d)
        if _dot(d, _add(fx["tip"], fy["tip"])) < 0.0:
            d = _scale(d, -1.0)
        turn = math.acos(max(-1.0, min(1.0, _dot(fx["normal"], fy["normal"]))))
        info = {}
        for key, fr, side in ((kx, fx, side_x), (ky, fy, side_y)):
            dt, ds = _dot(d, fr["tip"]), _dot(d, fr["axis"])
            if dt < 0.2:
                raise ValueError("the seam runs almost along the hub. Infeasible.")
            info[key] = (dt, ds, side)
        lam = min(fr["height"] / info[key][0] for key, fr in ((kx, fx), (ky, fy)))
        P = _add(vertex, _scale(d, lam))
        for key, fr in ((kx, fx), (ky, fy)):
            dt, ds, side = info[key]
            s0 = 0.0 if side == "left" else fr["width"]
            fr["ends"][side] = {"t": lam * dt, "s": s0 + lam * ds, "dt": dt, "ds": ds,
                                "turn": turn, "P": P, "tapers": lam * dt < fr["height"] - 1e-6}
        seams[name] = {"dir": d, "end": P, "turn_deg": math.degrees(turn), "vertex": vertex}
    return {"normal": normal, "to_space": to_space, "vertices": v, "walls": frames, "seams": seams}

def _group_touching(segments):
    """端点を共有する線分どうしをまとめる(union-find)。継ぎ目のある角は
    「壁の根本 + 継ぎ目」が頂点でつながって 1 グループになる。"""
    def key(p):
        return tuple(round(c, 3) for c in p)

    parent: dict = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x, y):
        parent[find(x)] = find(y)

    for a, b in segments:
        union(key(a), key(b))
    groups: dict = {}
    for seg in segments:
        groups.setdefault(find(key(seg[0])), []).append(seg)
    return list(groups.values())


def box_frames(hub_xy, walls: dict, closed, *, origin: Vec3, hub_u: Vec3, hub_v: Vec3):
    """ハブ(反時計回りの凸多角形 v0..v(N-1))の辺 i (v_i -> v_i+1) に壁を立て、頂点 k で
    隣り合う壁(辺 k-1 の右端と辺 k の左端)を**平面の交線**(継ぎ目)で互いにトリムする。

    `drawn_tray_frames`(ハブ四角 + 壁3 + 継ぎ目2 の固定形)の一般化。壁は裏側(-法線)へ折る。
    walls = {辺番号: {"fold_deg", "height_mm"}}、closed = 継ぎ目にする頂点番号の集合。
    継ぎ目の端 P は「2枚の壁の上端のうち低い方」に置き、高い方は上端を P へ向けて細らせる
    (`_drawn_wall_face` がそのまま使える)。

    戻り値: {"normal", "to_space", "vertices",
             "walls": {i: {a, b, axis, tip, angle, normal, width, height, ends}},
             "seams": {k: {dir, end, turn_deg, vertex, edges: (k-1, k)}}}
    """
    normal = _normalize(_cross(hub_u, hub_v))
    n = len(hub_xy)

    def to_space(xy):
        return _add(origin, _add(_scale(hub_u, xy[0]), _scale(hub_v, xy[1])))

    v = [to_space(q) for q in hub_xy]
    frames = {}
    for i, spec in walls.items():
        a0, b0 = v[i], v[(i + 1) % n]
        axis = _normalize(_add(b0, _scale(a0, -1.0)))
        edge_len = math.dist(a0, b0)
        # 根本は辺の一部でよい(角を継ぎ目にせず開ける場合は、隣の壁との間に逃げを取る)
        s0 = float(spec.get("root_from_mm", 0.0))
        s1 = float(spec.get("root_to_mm", edge_len))
        if not (0.0 <= s0 < s1 <= edge_len + 1e-9) or s1 - s0 < MIN_EDGE_LENGTH_MM:
            raise ValueError("a wall root does not fit on its hub edge. Infeasible.")
        a = _add(a0, _scale(axis, s0))
        b = _add(a0, _scale(axis, s1))
        angle = math.radians(spec["fold_deg"])
        outward = _normalize(_cross(axis, normal))
        tip = _normalize(_add(_scale(outward, math.cos(angle)), _scale(normal, -math.sin(angle))))
        frames[i] = {"a": a, "b": b, "axis": axis, "tip": tip, "angle": angle,
                     "normal": _normalize(_cross(tip, axis)), "width": s1 - s0,
                     "height": spec["height_mm"], "ends": {"left": None, "right": None},
                     "root_mm": (s0, s1), "edge_len": edge_len}
    seams = {}
    for k in sorted(closed):
        kx, ky = (k - 1) % n, k % n
        if kx not in frames or ky not in frames:
            raise ValueError("a closed corner needs walls on both of its edges. Infeasible.")
        fx, fy = frames[kx], frames[ky]
        # 継ぎ目にする角は、両方の壁の根本が頂点まで届いていること
        if abs(fx["root_mm"][1] - fx["edge_len"]) > 1e-6 or fy["root_mm"][0] > 1e-6:
            raise ValueError("a sewn corner needs both wall roots to reach the vertex. Infeasible.")
        d = _cross(fx["normal"], fy["normal"])
        if math.sqrt(_dot(d, d)) < 1e-6:
            raise ValueError("two walls at a closed corner are coplanar. Infeasible.")
        d = _normalize(d)
        if _dot(d, _add(fx["tip"], fy["tip"])) < 0.0:
            d = _scale(d, -1.0)
        turn = math.acos(max(-1.0, min(1.0, _dot(fx["normal"], fy["normal"]))))
        info = {}
        for key, fr, side in ((kx, fx, "right"), (ky, fy, "left")):
            dt, ds = _dot(d, fr["tip"]), _dot(d, fr["axis"])
            if dt < 0.2:
                raise ValueError("the seam runs almost along the hub. Infeasible.")
            info[key] = (dt, ds, side)
        lam = min(fr["height"] / info[key][0] for key, fr in ((kx, fx), (ky, fy)))
        P = _add(v[k], _scale(d, lam))
        for key, fr in ((kx, fx), (ky, fy)):
            dt, ds, side = info[key]
            if fr["ends"][side] is not None:
                raise ValueError("a wall end is claimed by two seams. Infeasible.")
            s0 = 0.0 if side == "left" else fr["width"]
            fr["ends"][side] = {"t": lam * dt, "s": s0 + lam * ds, "dt": dt, "ds": ds,
                                "turn": turn, "P": P, "tapers": lam * dt < fr["height"] - 1e-6}
        seams[k] = {"dir": d, "end": P, "turn_deg": math.degrees(turn), "vertex": v[k],
                    "edges": (kx, ky)}
    return {"normal": normal, "to_space": to_space, "vertices": v,
            "walls": frames, "seams": seams}


def channel_seat_frames(*, origin: Vec3, hub_u: Vec3, hub_v: Vec3, length_mm: float,
                        width_mm: float, wall_fold_deg: float, wall_radius_mm: float,
                        wall_depth0_mm: float, wall_depth1_mm: float, diag_start_mm: float,
                        diag_end_mm: float, seat_fold_deg: float | None, seat_radius_mm: float):
    """実車144型の幾何(OCCTを呼ばない。族の締結点配置とビルダーが共有する)。

    ウェブ(台の平面)= 長さ length(u方向) x 幅 width(v方向) の矩形。両長辺に壁を
    折り角 wall_fold で折り出す(裏側 = -法線側)。壁の下辺は u=diag_start..diag_end の
    範囲で深さ depth0 -> depth1 の**斜辺**。その斜辺から座面を seat_fold で外側へ折る。
    両壁は v=width/2 の面で鏡像。座面2枚の法線が**一致**するのは法線が鏡映面に乗るとき
    だけなので、seat_fold_deg=None なら「座面法線の v 成分 = 0」になる γ を解いて使う
    (α=88, β=43.5 の実車では γ≈88.5。実車の 86.1 は座面同士が 5 度ずれる)。
    """
    normal = _normalize(_cross(hub_u, hub_v))

    def at(x, y):
        return _add(origin, _add(_scale(hub_u, x), _scale(hub_v, y)))

    walls = {}
    for key, a_xy, b_xy in (("A", (0.0, 0.0), (length_mm, 0.0)),
                            ("B", (length_mm, width_mm), (0.0, width_mm))):
        a, b = at(*a_xy), at(*b_xy)
        axis = _normalize(_add(b, _scale(a, -1.0)))
        alpha = math.radians(wall_fold_deg)
        centre = _add(a, _scale(normal, -wall_radius_mm))
        outward = _normalize(_cross(axis, normal))
        tip = _normalize(_add(_scale(outward, math.cos(alpha)), _scale(normal, -math.sin(alpha))))
        a2 = _rotate_about(a, centre, axis, alpha)
        b2 = _rotate_about(b, centre, axis, alpha)
        wall_normal = _normalize(_cross(tip, axis))
        # 斜辺の端点をウェブの u 座標で指定する(両壁で同じ x に来るよう鏡像にする)。
        # 壁A の axis は +u、壁B は -u なので、B は a2 から測る距離を反転する。
        if key == "A":
            s_lo, s_hi, d_lo, d_hi = diag_start_mm, diag_end_mm, wall_depth0_mm, wall_depth1_mm
        else:
            s_lo, s_hi = length_mm - diag_end_mm, length_mm - diag_start_mm
            d_lo, d_hi = wall_depth1_mm, wall_depth0_mm
        q_a = _add(a2, _add(_scale(axis, s_lo), _scale(tip, d_lo)))
        q_b = _add(a2, _add(_scale(axis, s_hi), _scale(tip, d_hi)))
        d = _normalize(_add(q_b, _scale(q_a, -1.0)))
        # 座面は壁の**外側**(壁の法線側)へ折る。e0 = 壁面内で斜辺に直交しウェブから離れる向き。
        e0 = _normalize(_add(tip, _scale(d, -_dot(tip, d))))
        sign = 1.0 if _dot(_cross(d, e0), wall_normal) > 0.0 else -1.0
        if seat_fold_deg is None:
            # 座面法線 n(θ) = n0 cosθ + (d x n0) sinθ (n0 ⊥ d)。v成分 = 0 を解く。
            n0_v = _dot(wall_normal, hub_v)
            k_v = _dot(_cross(d, wall_normal), hub_v)
            theta = math.atan2(-n0_v, k_v)
            if theta * sign < 0.0:
                theta += math.pi if sign > 0 else -math.pi
            gamma = abs(theta)
        else:
            gamma = math.radians(seat_fold_deg)
            theta = sign * gamma
        seat_centre = _add(q_a, _scale(wall_normal, seat_radius_mm))
        qa2 = _rotate_about(q_a, seat_centre, d, theta)
        qb2 = _rotate_about(q_b, seat_centre, d, theta)
        e = _normalize(_add(_scale(e0, math.cos(gamma)), _scale(wall_normal, math.sin(gamma))))
        walls[key] = {
            "root_a": a, "root_b": b, "axis": axis, "centre": centre, "angle": alpha,
            "a2": a2, "b2": b2, "tip": tip, "normal": wall_normal,
            "q_a": q_a, "q_b": q_b, "diag": d, "diag_len": math.dist(q_a, q_b),
            "seat_centre": seat_centre, "seat_theta": theta,
            "seat_a": qa2, "seat_b": qb2, "seat_out": e,
            "seat_fold_deg": math.degrees(gamma),
            # γ=0 で壁の法線に連続する向き(分岐族の腕と同じ約束)
            "seat_normal": _normalize(_cross(d, e)) if sign > 0 else _normalize(_cross(e, d)),
        }
    return {"normal": normal, "at": at, "walls": walls}


def _fillet_corner_2d(prev_v, v, next_v, radius):
    """2D の凸角 v を半径 radius で丸める(両辺に接する円弧)。(start, mid, end) を返す。"""
    d_in = _normalize2((v[0] - prev_v[0], v[1] - prev_v[1]))
    d_out = _normalize2((next_v[0] - v[0], next_v[1] - v[1]))
    cos_turn = max(-1.0, min(1.0, d_in[0] * d_out[0] + d_in[1] * d_out[1]))
    interior = math.pi - math.acos(cos_turn)
    if interior < math.radians(20.0) or interior > math.radians(178.0):
        raise ValueError("corner too sharp or too flat to fillet. Infeasible.")
    t = radius / math.tan(interior / 2.0)
    inward = _normalize2((d_out[0] - d_in[0], d_out[1] - d_in[1]))
    reach = radius / math.sin(interior / 2.0)
    centre = (v[0] + inward[0] * reach, v[1] + inward[1] * reach)
    return ((v[0] - t * d_in[0], v[1] - t * d_in[1]),
            (centre[0] - inward[0] * radius, centre[1] - inward[1] * radius),
            (v[0] + t * d_out[0], v[1] + t * d_out[1]))


def _hub_outline(xy, arm_edges, corner_radius, fillet_radius=None):
    """分岐部品のハブ輪郭と、辺ごとの腕の根本エッジを返す。

    `fillet_radius` = {頂点番号: R}。**両隣に腕が無い**頂点だけを凸に丸める
    (実車1285-18 の台の角 R8 など)。腕の付く辺は根本エッジが短くなるので対象外。

    **隣り合う2辺の両方に腕が付く角にだけ**、1つのコーナーを入れる。コーナーは
    両方の曲げ線に接するところから始まり、ハブの内側へ**くぼむ**円弧
    (接円の長い方の弧)。曲げ線がここで途切れると応力が集中して裂けるので、
    実務でも角をえぐって逃がす。隣に腕が無い角はコーナーを設けず、そのまま
    ハブのエッジに繋がる。

    戻り値は (断面要素の列, {辺の番号: (根本エッジの始点, 終点)})。
    """
    count = len(xy)
    corner: dict = {}
    for i in range(count):
        before, after = (i - 1) % count, i        # 頂点 i に入る辺 / 出る辺
        if before not in arm_edges or after not in arm_edges:
            continue
        relief_mm = corner_radius[i] if isinstance(corner_radius, dict) else corner_radius
        v, prev_v, next_v = xy[i], xy[i - 1], xy[(i + 1) % count]
        d_in = _normalize2((v[0] - prev_v[0], v[1] - prev_v[1]))
        d_out = _normalize2((next_v[0] - v[0], next_v[1] - v[1]))
        cos_turn = max(-1.0, min(1.0, d_in[0] * d_out[0] + d_in[1] * d_out[1]))
        interior = math.pi - math.acos(cos_turn)
        if interior < math.radians(20.0):
            raise ValueError("the hub polygon has a spike. Infeasible.")
        inward = _normalize2((d_out[0] - d_in[0], d_out[1] - d_in[1]))
        # コーナーは**ハブ頂点そのものを中心**とする半径 relief_mm の円弧。こうすると
        # 両端の接線が曲げ線に直交する = **腕の側端エッジと滑らかに繋がる**。
        # 曲げ線(ハブの辺)に接する円弧にすると、腕の側端が根元で90度に折れてしまう。
        corner[i] = {
            "start": (v[0] - relief_mm * d_in[0], v[1] - relief_mm * d_in[1]),
            "end": (v[0] + relief_mm * d_out[0], v[1] + relief_mm * d_out[1]),
            "mid": (v[0] + relief_mm * inward[0], v[1] + relief_mm * inward[1]),
        }

    for i, radius in (fillet_radius or {}).items():
        before, after = (i - 1) % count, i
        if before in arm_edges or after in arm_edges or i in corner:
            continue
        start, mid, end = _fillet_corner_2d(xy[i - 1], xy[i], xy[(i + 1) % count], radius)
        corner[i] = {"start": start, "mid": mid, "end": end}

    section, roots = [], {}
    for i in range(count):
        begin = corner[i]["end"] if i in corner else xy[i]
        finish = corner[(i + 1) % count]["start"] if (i + 1) % count in corner             else xy[(i + 1) % count]
        if math.dist(begin, finish) < MIN_EDGE_LENGTH_MM:
            raise ValueError(
                f"hub edge {i} is consumed by its corner reliefs. Infeasible.")
        section.append(("line", begin, finish, f"hub_edge_{i}"))
        roots[i] = (begin, finish)
        nxt = (i + 1) % count
        if nxt in corner:
            c = corner[nxt]
            section.append(("arc", c["start"], c["mid"], c["end"], f"hub_corner_{nxt}"))
    return section, roots


def _edges_of(section: list[Elem], frame: _Frame):
    """断面要素列を3Dエッジ列 [(edge, role), ...] にする。"""
    out = []
    for elem in section:
        if elem[0] == "line":
            _, a, b, role = elem
            edge = BRepBuilderAPI_MakeEdge(gp_Pnt(*frame.point(a)), gp_Pnt(*frame.point(b))).Edge()
        else:
            _, a, m, b, role = elem
            arc = GC_MakeArcOfCircle(
                gp_Pnt(*frame.point(a)), gp_Pnt(*frame.point(m)), gp_Pnt(*frame.point(b))
            ).Value()
            edge = BRepBuilderAPI_MakeEdge(arc).Edge()
        out.append((edge, role))
    return out


def _wire_of(section: list[Elem], frame: _Frame):
    maker = BRepBuilderAPI_MakeWire()
    for edge, _role in _edges_of(section, frame):
        maker.Add(edge)
    return maker.Wire()


# ---------------------------------------------------------------- 中心線(経路)


@dataclasses.dataclass
class _Straight:
    panel: int
    vector: Vec3      # この区間の変位そのもの(u方向の走行 + 横ズレ補正のw成分)
    length: float     # 経路パラメータ(u方向の走行長)

    @property
    def direction(self) -> Vec3:
        return _normalize(self.vector)

    def scaled(self, length: float) -> "_Straight":
        k = length / self.length
        return _Straight(self.panel, _scale(self.vector, k), length)


@dataclasses.dataclass
class _Bend:
    fold: int
    angle: float      # w まわりの符号付き回転角
    radius: float
    normal: Vec3      # 曲げ手前のパネル法線(曲げ中心の計算に使う)


def _widen_section(section, y_breaks, ext_neg: float, ext_pos: float):
    """断面の両端の平地要素を外へ広げる(非対称余白)。y_breaks も揃える。"""
    section = list(section)
    first, last = section[0], section[-1]
    section[0] = (first[0], (first[1][0] - ext_neg, first[1][1]), first[2], first[3])
    section[-1] = (last[0], last[1], (last[2][0] + ext_pos, last[2][1]), last[3])
    y_breaks = list(y_breaks)
    y_breaks[0] -= ext_neg
    y_breaks[-1] += ext_pos
    return section, y_breaks


def _check_bearing_margin(shape, points, radii, tolerance_mm: float = 0.3) -> None:
    """締結点から外形(自由エッジ)までの距離が座面半径以上か(ML 側の「座面比」)。
    腕や切欠きが座面に食い込む配置を弾く(2026-09-06 夜、ML 返答: 第1期は 600 部品中 17 で
    0.95 未満。原因は台形の腕の斜辺と、隣の腕の側辺)。"""
    amap = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, amap)
    free = [topods.Edge(amap.FindKey(i)) for i in range(1, amap.Size() + 1)
            if amap.FindFromIndex(i).Size() == 1]
    for index, (point, radius) in enumerate(zip(points, radii)):
        vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(*point.position_xyz)).Vertex()
        worst = float("inf")
        for edge in free:
            dist = BRepExtrema_DistShapeShape(vertex, edge)
            dist.Perform()
            worst = min(worst, dist.Value())
        if worst < radius - tolerance_mm:
            raise ValueError(
                f"fastening point {index + 1} is {worst:.1f}mm from the outline "
                f"(bearing radius {radius:.1f}mm). Infeasible; resample."
            )


def sweep_steps(plan, bend_radius_mm: float):
    """掃引の直線区間ごとの実フレーム(合成族の腕・切欠き・追加点が使う。OCCT を呼ばない)。

    戻り値: [{"panel", "origin"(区間の始点=中心線上), "ey"(幅方向 +v 側が正), "ez"(法線),
             "direction"(単位), "vector", "length"}, ...] と、共通折り軸 w。
    区間の始点は曲げの接線点(端パネルでは帯の端)。パネルの側辺は origin + t*direction
    + (±half_width)*ey、t ∈ [0, length]。
    """
    path, start, w, normal0 = _build_path(plan, bend_radius_mm)
    ey = w if _dot(w, plan.panel_frames[0].v) > 0 else _scale(w, -1.0)
    frame = _Frame(start, ey, normal0)
    steps = []
    for step in path:
        if isinstance(step, _Straight):
            steps.append({"panel": step.panel, "origin": frame.origin, "ey": frame.ey,
                          "ez": frame.ez, "direction": step.direction, "vector": step.vector,
                          "length": step.length})
            frame = frame.translated(step.vector)
        else:
            sign = 1.0 if step.angle > 0 else -1.0
            centre = _add(frame.origin, _scale(step.normal, -sign * step.radius))
            frame = frame.rotated(centre, w, step.angle)
    return steps, w


def compose_arm_frame(step: dict, side: int, t0: float, t1: float, half_width_mm: float,
                      fold_deg: float, radius_mm: float) -> dict:
    """掃引パネルの側辺 (t0..t1、側 side) を根本にした腕のフレーム(分岐族の腕と同じ約束:
    裏側 = -法線側へ折る)。戻り値は branch_frames の腕と同じキー。"""
    n = step["ez"]
    y = side * half_width_mm
    p0 = _add(step["origin"], _add(_scale(step["direction"], t0), _scale(step["ey"], y)))
    p1 = _add(step["origin"], _add(_scale(step["direction"], t1), _scale(step["ey"], y)))
    # outward = cross(axis, n) が帯の外(side*ey)を向くように根本の向きを決める
    a, b = (p1, p0) if side > 0 else (p0, p1)
    axis = _normalize(_add(b, _scale(a, -1.0)))
    outward = _normalize(_cross(axis, n))
    angle = math.radians(fold_deg)
    centre = _add(a, _scale(n, -radius_mm))
    tip = _normalize(_add(_scale(outward, math.cos(angle)), _scale(n, -math.sin(angle))))
    return {"a": _rotate_about(a, centre, axis, angle), "b": _rotate_about(b, centre, axis, angle),
            "tip": tip, "axis": axis, "width": math.dist(a, b),
            "normal": _normalize(_cross(tip, axis)), "root_a": a, "root_b": b,
            "centre": centre, "angle": angle, "revolve_axis": axis, "side": -1}


def _build_path(plan, bend_radius_mm: float) -> tuple[list, Vec3, Vec3, Vec3]:
    """パネル列から (経路, 開始点, 共通折れ目軸w, 開始パネル法線) を作る。

    横ズレ(シアー)は直線区間の方向にw成分として乗せる。曲げ区間はwを保存する
    (w軸まわりの回転なので構造的にそうなる)ため、その分だけ直線区間で補正して
    終端が締結点2の幅方向位置に来るようにする。
    """
    frames = plan.panel_frames
    tangents = plan.fold_tangents
    tilts = plan.fold_tilts
    normals = [_normalize(_cross(f.u, f.v)) for f in frames]

    # 折れ目軸(解析式)。全ての折れ目で共通のはず。曲げ0本(平板)なら幅方向をそのまま使う。
    axes = []
    for k in range(len(frames) - 1):
        a = tilts[k][1]
        axes.append(_normalize(_add(_scale(frames[k].v, math.cos(a)), _scale(frames[k].u, math.sin(a)))))
    w = axes[0] if axes else frames[0].v
    if _dot(w, frames[0].v) < 0.0:
        w = _scale(w, -1.0)
    for axis in axes[1:]:
        if abs(abs(_dot(axis, w)) - 1.0) > 1e-4:
            raise ValueError(
                "fold axes are not parallel (angle "
                f"{math.degrees(math.acos(min(1.0, abs(_dot(axis, w))))):.2f}deg); the OCCT "
                "backend requires the w-parallel fold family (set the tilt perturbation to 0). "
                "Infeasible; not attempting construction."
            )

    start = _add(frames[0].origin, _scale(frames[0].u, frames[0].near_run_mm + tangents[0][0]))
    end = _add(frames[-1].origin, _scale(frames[-1].u, frames[-1].far_run_mm))

    path: list = []
    for k, frame in enumerate(frames):
        near_cut, far_cut = tangents[k]
        length = (frame.far_run_mm - far_cut) - (frame.near_run_mm + near_cut)
        if length <= 1.0:
            raise ValueError(
                f"panel {k} has only {length:.1f}mm of flat run between its bend fillets. "
                "Infeasible; not attempting construction."
            )
        path.append(_Straight(panel=k, vector=_scale(frame.u, length), length=length))
        if k < len(frames) - 1:
            phi = _signed_angle_about(frame.u, frames[k + 1].u, w)
            # ほぼ0度の折れ目は、幅方向に細長いスリバー面(実測 0.7〜1.6mm^2)と
            # 0.04mmのゴミエッジを生む。曲げの円弧長で1mmを下限にする。
            if abs(phi) * bend_radius_mm < 1.0:
                raise ValueError(
                    f"fold {k} is only {math.degrees(abs(phi)):.2f}deg "
                    f"({abs(phi) * bend_radius_mm:.2f}mm of arc at R={bend_radius_mm:.1f}) -- "
                    "it would leave sliver faces. Infeasible; not attempting construction."
                )
            path.append(_Bend(fold=k, angle=phi, radius=bend_radius_mm, normal=normals[k]))

    # 幅方向(w)のつじつま合わせ。曲げ区間はwを動かさないので、その不足分を
    # 直線区間の長さ比で配る。
    straights = [s for s in path if isinstance(s, _Straight)]
    natural = sum(_dot(s.vector, w) for s in straights)
    deficit = _dot(end, w) - _dot(start, w) - natural
    total_straight = sum(s.length for s in straights)
    rate = deficit / total_straight if total_straight > 1e-9 else 0.0
    for s in straights:
        # **正規化しない** — u方向成分を厳密に保つ(正規化すると1区間あたり
        # length*rate^2/2 だけ縮み、終端が締結点2から0.08mmずれる。2026-09-04実測)。
        s.vector = _add(s.vector, _scale(w, rate * s.length))

    return path, start, w, normals[0]


# ---------------------------------------------------------------- STEP出力


GEO_NAMES = {0: "planar", 1: "cylindrical", 2: "conical", 3: "spherical", 4: "toroidal",
             5: "bezier", 6: "bspline", 7: "revolution", 8: "extrusion", 9: "offset"}


def describe_faces(faces: dict) -> list[dict]:
    """面ラベルの一覧(引継ぎ書 §4.2 の `features.faces[]`)。名前は構築時に確定して
    いるので推定は要らない。STEPにはXCAFで同じ名前が入る。"""
    out = []
    props = GProp_GProps()
    for face, name in faces.items():
        surface = BRepAdaptor_Surface(face)
        brepgprop.SurfaceProperties(face, props)
        centre = props.CentreOfMass()
        entry = {
            "name": name,
            "role": name.split("_", 2)[-1] if name.count("_") >= 2 else name,
            "feature": name.split("_")[0],
            "geo": GEO_NAMES.get(surface.GetType(), str(surface.GetType())),
            "area_mm2": round(props.Mass(), 4),
            "centroid": [round(centre.X(), 4), round(centre.Y(), 4), round(centre.Z(), 4)],
        }
        if surface.GetType() == 1:
            entry["radius_mm"] = round(surface.Cylinder().Radius(), 4)
        elif surface.GetType() == 2:
            entry["radius_mm"] = round(surface.Cone().RefRadius(), 4)
        elif surface.GetType() == 4:
            entry["radius_mm"] = round(surface.Torus().MinorRadius(), 4)
        out.append(entry)
    return sorted(out, key=lambda item: item["name"])


def face_centroid(face) -> Vec3:
    props = GProp_GProps()
    brepgprop.SurfaceProperties(face, props)
    centre = props.CentreOfMass()
    return (centre.X(), centre.Y(), centre.Z())


def primitive_deviation(edge, samples: int = 24) -> float:
    """エッジを1本の直線/円弧に当てたときの最大ずれ[mm](引継ぎ書 §3.4 ゲートA)。

    OCCTの曲線**型**では判定しない — 直線織り面の稜線は幾何的に厳密な直線なのに
    B-splineとして表現されるため。下流の抽出器はプリミティブ当てはめで判定する。
    """
    curve = BRepAdaptor_Curve(edge)
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    points = [curve.Value(u0 + (u1 - u0) * k / samples) for k in range(samples + 1)]
    first, last = points[0], points[-1]
    vector = (last.X() - first.X(), last.Y() - first.Y(), last.Z() - first.Z())
    squared = sum(c * c for c in vector)
    line = 1e9
    if squared > 1e-18:
        line = 0.0
        for point in points[1:-1]:
            offset = (point.X() - first.X(), point.Y() - first.Y(), point.Z() - first.Z())
            t = sum(a * b for a, b in zip(vector, offset)) / squared
            projected = tuple(first.Coord()[i] + t * vector[i] for i in range(3))
            line = max(line, math.dist((point.X(), point.Y(), point.Z()), projected))
    arc = 1e9
    try:
        circle = GC_MakeCircle(first, points[len(points) // 2], last).Value()
        centre, radius = circle.Location(), circle.Radius()
        normal = circle.Axis().Direction()
        arc = 0.0
        for point in points:
            arc = max(arc, abs(centre.Distance(point) - radius))
            delta = (point.X() - centre.X(), point.Y() - centre.Y(), point.Z() - centre.Z())
            arc = max(arc, abs(delta[0] * normal.X() + delta[1] * normal.Y()
                              + delta[2] * normal.Z()))
    except Exception:
        pass
    return min(line, arc)


def boundary_loop_count(shape) -> int:
    """外形(自由エッジ)が作る閉ループの本数。閉じていなければ -1。

    開いたシェル1枚の板金部品の外形は**必ず1本の閉ループ**になる(穴は開けない
    方針なので内側ループも無い)。縫合が失敗して面がばらけると、面ごとに開いた
    辺が残ってループ数が跳ね上がる — 崩壊の最も確実な兆候。
    実測: 健全な部品は 1、崩壊した SYN_general_two_point_0048 は 自由エッジ60本。
    """
    edge_faces = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces)
    adjacency: dict = {}
    edges = []
    for i in range(1, edge_faces.Size() + 1):
        edge = topods.Edge(edge_faces.FindKey(i))
        if BRep_Tool.Degenerated(edge) or edge_faces.FindFromIndex(i).Size() != 1:
            continue
        ends = []
        explorer = TopExp_Explorer(edge, TopAbs_VERTEX)
        while explorer.More():
            ends.append(topods.Vertex(explorer.Current()))
            explorer.Next()
        if len(ends) != 2:
            return -1
        index = len(edges)
        edges.append(ends)
        for vertex in ends:
            key = _vertex_key(vertex)
            adjacency.setdefault(key, []).append(index)
    if not edges:
        return 0
    if any(len(v) != 2 for v in adjacency.values()):
        return -1          # 端が開いている / 3本以上が集まる = 閉じていない
    seen: set = set()
    loops = 0
    for start in range(len(edges)):
        if start in seen:
            continue
        loops += 1
        stack = [start]
        while stack:
            current = stack.pop()
            if current in seen:
                continue
            seen.add(current)
            for vertex in edges[current]:
                stack.extend(adjacency[_vertex_key(vertex)])
    return loops


def _vertex_key(vertex, grid: float = 1e-4):
    point = BRep_Tool.Pnt(vertex)
    return (round(point.X() / grid), round(point.Y() / grid), round(point.Z() / grid))


def worst_dihedral_deg(shape) -> float:
    """隣接する2面の法線がなす角の最大値[度]。

    中立面は自分の上に折り返らないので、180度に近い値は掃引の破綻(ねじれ・
    面の裏返り)を意味する。ユーザー提案の「勾配が異常な形状は怪しい」の実装。
    """
    edge_faces = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces)
    worst = 0.0
    for i in range(1, edge_faces.Size() + 1):
        edge = topods.Edge(edge_faces.FindKey(i))
        if BRep_Tool.Degenerated(edge) or edge_faces.FindFromIndex(i).Size() != 2:
            continue
        normals = []
        for shape_face in edge_faces.FindFromIndex(i):
            face = topods.Face(shape_face)
            curve, first, last = BRep_Tool.CurveOnSurface(edge, face)
            if curve is None:
                normals = []
                break
            uv = curve.Value(0.5 * (first + last))
            props = BRepLProp_SLProps(BRepAdaptor_Surface(face), uv.X(), uv.Y(), 1, 1e-6)
            if not props.IsNormalDefined():
                normals = []
                break
            direction = props.Normal()
            if face.Orientation() == TopAbs_REVERSED:
                direction.Reverse()
            normals.append((direction.X(), direction.Y(), direction.Z()))
        if len(normals) == 2:
            cosine = max(-1.0, min(1.0, _dot(normals[0], normals[1])))
            worst = max(worst, math.degrees(math.acos(cosine)))
    return worst


def _read_step(path: str):
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != 1:
        raise ValueError(f"cannot read back {path}")
    reader.TransferRoots()
    shape = reader.OneShape()
    # 稀に何も転送されない(空のSTEP)。バッチを落とさず、その部品だけ捨てる。
    if shape is None or shape.IsNull():
        raise ValueError(f"{path} came back empty. Infeasible; resample.")
    return shape


def check_shape(shape, fastening_points=(), expected_loops: int = 1) -> None:
    """出来上がった形の合格判定(通らなければValueErrorで棄却)。

    * 0.05mm未満のゴミエッジが無い(引継ぎ書 §4.3)
    * 全エッジが1本の直線/円弧に載る(ゲートA)
    * シェルが有効
    * **外形が閉じた1本のループ**(崩壊検知。縫合漏れを全部拾う)
    * **隣接面が折り返らない**(崩壊検知。ねじれ・面の裏返りを拾う)
    * **締結点が面の上にある**(崩壊検知。部品が自分の締結点に届かなくなる型)
    """
    _reject_junk_edges(shape)
    if not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError("the shell is not valid. Infeasible; resample.")
    loops = boundary_loop_count(shape)
    if loops != expected_loops:
        # 期待値は通常1。ガセット付きの分岐部品は角に窓が開くので 1 + ガセット数。
        raise ValueError(
            f"the outline is {loops} closed loops, not {expected_loops} -- faces did not "
            "sew (shape collapsed). Infeasible; resample."
        )
    turn = worst_dihedral_deg(shape)
    if turn > MAX_DIHEDRAL_TURN_DEG:
        raise ValueError(
            f"two adjacent faces turn {turn:.0f}deg (limit {MAX_DIHEDRAL_TURN_DEG:.0f}) "
            "-- the surface folds back on itself. Infeasible; resample."
        )
    for index, point in enumerate(fastening_points):
        vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(*point)).Vertex()
        distance = BRepExtrema_DistShapeShape(shape, vertex)
        distance.Perform()
        if distance.Value() > MAX_FASTENING_OFFSET_MM:
            raise ValueError(
                f"fastening point {index + 1} is {distance.Value():.2f}mm off the surface "
                "-- the part does not reach its own bearing area. Infeasible; resample."
            )


def _reject_junk_edges(shape, minimum_mm: float = MIN_EDGE_LENGTH_MM) -> None:
    """0.05mm未満のエッジが1本でもあれば部品を捨てる(サンプラーが引き直す)。

    引継ぎ書 §4.3「ゴミ幾何を出さない」。CATIA版は607本出していた。構成的にはほぼ
    出ないが、余肉カットのブーリアンが既存エッジをかすめる縮退配置で稀に1本出る
    (2026-09-04実測: 300部品に1本)。棄却は0.1秒なので、混ぜるより捨てるほうが安い。
    頂点(リブの先端)を閉じる退化エッジは長さ0が正しい表現なので除外する。
    """
    explorer = TopExp_Explorer(shape, TopAbs_EDGE)
    seen = set()
    while explorer.More():
        edge = topods.Edge(explorer.Current())
        if edge not in seen and not BRep_Tool.Degenerated(edge):
            seen.add(edge)
            length = GCPnts_AbscissaPoint.Length(BRepAdaptor_Curve(edge))
            if length < minimum_mm:
                raise ValueError(
                    f"the build produced a {length:.4f}mm junk edge "
                    f"(minimum {minimum_mm:.2f}mm). Infeasible; resample."
                )
            deviation = primitive_deviation(edge)
            if deviation > MAX_PRIMITIVE_DEVIATION_MM:
                raise ValueError(
                    f"an edge deviates {deviation:.3f}mm from a single line/arc "
                    f"(gate A allows {MAX_PRIMITIVE_DEVIATION_MM:.2f}mm). Resample."
                )
        explorer.Next()


def _export_step(shape, names: dict, step_path: str) -> None:
    app = XCAFApp_Application.GetApplication()
    doc = TDocStd_Document("MDTV-XCAF")
    app.NewDocument("MDTV-XCAF", doc)
    tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    root = tool.AddShape(shape, False)
    TDataStd_Name.Set(root, "sheet_metal_part")
    for face, name in names.items():
        label = tool.AddSubShape(root, face)
        if not label.IsNull():
            TDataStd_Name.Set(label, name)
    # **順序が重要**: `write.stepcaf.subshapes.name` はSTEPCAFの静的パラメータなので、
    # ライターを1つ構築するまで存在しない。先に設定すると SetIVal が False を返して
    # 面名が1つも書かれない(2026-09-04に実測)。
    writer = STEPCAFControl_Writer()
    Interface_Static.SetCVal("write.step.schema", "AP214IS")
    Interface_Static.SetCVal("write.step.unit", "MM")
    Interface_Static.SetIVal("write.stepcaf.subshapes.name", 1)
    writer.SetNameMode(True)
    writer.Transfer(doc)
    if writer.Write(step_path) != 1:
        raise RuntimeError(f"STEP write failed for {step_path}")


# ---------------------------------------------------------------- ビルダー


class OcctPartBuilder:
    """`gsd_build.SyntheticPartBuilder` と同じ `build_general_two_point` を持つ
    OCCTバックエンド。CATIA/DELMIAは不要。"""

    def build_general_two_point(
        self,
        point1: FasteningPoint,
        point2: FasteningPoint,
        *,
        min_bearing_radius_mm: float,
        half_width_mm: float,
        bend_radius_mm: float,
        fold1_slack_mm: float,
        fold2_slack_mm: float,
        fold1_tilt_perturbation_rad: float = 0.0,
        target_folds: int | None = None,
        extra_points: tuple = (),
        # 面上検査に掛ける締結点。省略時は point1/point2 + extra_points。011型・014型は
        # 掃引アンカー(point1/point2)が締結点ではないので、実在の点だけを渡す。
        check_points: tuple | None = None,
        taper_half_width_mm: float | None = None,
        out_dir: str,
        part_name: str,
        bead: BeadParams | None = None,
        flange: FlangeParams | None = None,
        rib: RibParams | None = None,
        # 合成族(2026-09-06): 側辺の腕、側辺の切欠き、非対称余白
        arms=(),
        notches=(),
        side_extension_mm: tuple[float, float] = (0.0, 0.0),
        # ビードを全長ではなく区間 [s0, s3](中心線の弧長)に置く(合成族 第2期)。None なら全長。
        bead_span: tuple[float, float] | None = None,
        # 締結点ごとの座面半径。指定すると「点から外形までの距離 >= 半径」を最終形状で検査する。
        check_radii: tuple = (),
        # 大型パネル族(2026-09-08): 幅方向に並べたビード [(中心 y, BeadParams), ...]。
        # 全長を走る。単一の `bead` とは併用しない。
        beads=(),
    ) -> GeneratedPart:
        # 1部品1特徴が原則。例外はビード + フランジだけ(実車014型で「両側フランジ +
        # 中央ビード」が1点と2点の間の剛性を担っている。ユーザー決定 2026-09-04)。
        if rib is not None and (bead is not None or flange is not None):
            raise ValueError("a rib cannot be combined with a bead or a flange")
        if beads and (bead is not None or rib is not None):
            raise ValueError("multi-bead panels take neither a single bead nor a rib")
        if abs(fold1_tilt_perturbation_rad) > 1e-9:
            # 折れ目軸が共通でなくなる(実測: 摂動ありで軸間角 最大7.3度)ため、
            # 断面掃引の前提が崩れる。サンプラー側で0に固定してある。
            raise ValueError(
                "the OCCT backend requires fold1_tilt_perturbation_rad == 0 "
                "(the fold axes must stay parallel). Infeasible."
            )

        plan = plan_general_two_point(
            point1,
            point2,
            min_bearing_radius_mm=min_bearing_radius_mm,
            half_width_mm=half_width_mm,
            bend_radius_mm=bend_radius_mm,
            fold1_slack_mm=fold1_slack_mm,
            fold2_slack_mm=fold2_slack_mm,
            fold1_tilt_perturbation_rad=0.0,
            side_extension_mm=side_extension_mm,
            target_folds=target_folds,
        )
        ext_neg, ext_pos = side_extension_mm

        path, start, w, normal0 = _build_path(plan, bend_radius_mm)
        ey = w if _dot(w, plan.panel_frames[0].v) > 0 else _scale(w, -1.0)
        frame = _Frame(start, ey, normal0)

        faces: dict = {}          # face -> name
        if bead is not None and taper_half_width_mm is not None:
            # 実車014型。ビードは対の側の板端から通し、孤立点の手前でだけ平地に
            # 戻す。帯幅も孤立点の必要平面幅まで絞る。
            self._check_flange_radii(path, flange, bend_radius_mm) if flange else None
            self._sweep_tapered_bead(
                path, frame, w, bead, flange,
                half_width_mm=half_width_mm,
                narrow_half_width_mm=taper_half_width_mm,
                min_bearing_radius_mm=min_bearing_radius_mm,
                # フランジとビードは同じ側へ出す(2026-09-04のユーザー指摘)。
                # フランジの向きは「裏側へ折る」規則で決まっているのでそれに従う。
                lift=self._bead_lift(path, bead, bend_radius_mm,
                                     forced=flange.direction if flange else None),
                faces=faces)
        elif bead is not None:
            lift = self._bead_lift(path, bead, bend_radius_mm)
            section, y_breaks = _bead_section(bead, lift, half_width_mm)
            if flange is None and (ext_neg > 0.0 or ext_pos > 0.0):
                # 非対称余白: 断面の両端の平地要素だけを広げる(ビードの位置は帯の中心のまま)
                section, y_breaks = _widen_section(section, y_breaks, ext_neg, ext_pos)
            flat = _flat_section(y_breaks)
            if flange is not None:
                # ランアウトの loft は断面どうしの要素が1対1で対応する必要があるので、
                # ビード側と平地側の**両方**に同じ壁を継ぐ。
                self._check_flange_radii(path, flange, bend_radius_mm)
                section = _with_flange(section, flange, half_width_mm)
                flat = _with_flange(flat, flange, half_width_mm)
            self._sweep_with_bead(path, frame, w, section, flat,
                                  bead, min_bearing_radius_mm, faces, placed=bead_span)
        elif beads:
            # 大型パネル: 幅方向に複数のビード。全長を走るので走り出し/走り終わりは無い。
            lift = self._bead_lift(path, beads[0][1], bend_radius_mm,
                                   forced=flange.direction if flange else None)
            section, _breaks, _spans = multi_bead_section(
                beads, lift, half_width_mm, ext_neg, ext_pos)
            if flange is not None:
                self._check_flange_radii(path, flange, bend_radius_mm)
                section = _with_flange(section, flange, half_width_mm)
            self._sweep_uniform(path, frame, w, section, faces)
        elif rib is not None:
            self._build_rib_part(plan, w, ey, rib, half_width_mm, bend_radius_mm, faces)
        else:
            section = (_flange_section(flange, half_width_mm) if flange is not None
                       else _flat_section([-half_width_mm - ext_neg, half_width_mm + ext_pos]))
            if flange is not None:
                self._check_flange_radii(path, flange, bend_radius_mm)
            self._sweep_uniform(path, frame, w, section, faces)

        shape, faces = self._sew(faces)
        if arms:
            steps, _w = sweep_steps(plan, bend_radius_mm)
            shape, faces = self._attach_side_arms(shape, faces, steps, arms,
                                                  half_width_mm, side_extension_mm)
        shape, faces = self._apply_corner_relief(
            shape, faces, path, start, ey, normal0, w,
            half_width_mm=half_width_mm,
            radius_mm=min_bearing_radius_mm,
            # 両側フランジは隅を落とす余地が無い(side=0で左右とも除外)。
            exclude_side=(None if flange is None else
                          (0 if flange.both_sides else flange.side)),
            side_extension_mm=side_extension_mm,
        )
        if notches:
            steps, _w = sweep_steps(plan, bend_radius_mm)
            shape, faces = self._cut_side_notches(shape, faces, steps, notches,
                                                  half_width_mm, side_extension_mm)
        shape, faces = self._unify(shape, faces)
        if check_radii:
            wanted = check_points if check_points is not None else (point1, point2, *extra_points)
            _check_bearing_margin(shape, wanted, check_radii)
        os.makedirs(out_dir, exist_ok=True)
        stp_path = os.path.abspath(os.path.join(out_dir, part_name + "_mid.stp"))
        _export_step(shape, faces, stp_path)
        # **検査は書き出したSTEPに対して行う。**下流が読むのはこのファイルであり、
        # STEPの往復で曲線が再近似される(2026-09-04実測: メモリ上0.25mm以内だった
        # フィレット稜線が、読み戻すと0.839mmずれていた)。
        try:
            wanted = check_points if check_points is not None else (
                point1, point2, *extra_points)
            check_shape(_read_step(stp_path), tuple(p.position_xyz for p in wanted))
        except ValueError:
            os.remove(stp_path)
            raise
        return GeneratedPart(stp_path=stp_path, catpart_path="",
                             face_labels=tuple(describe_faces(faces)))

    def _attach_side_arms(self, shape, faces, steps, arms, half_width_mm: float,
                          side_extension_mm):
        """掃引パネルの側辺の一部を根本にして腕を縫合する(合成族)。縫合はエッジの一部への
        接続を扱える(2026-09-06 実証: 帯の長い側辺の一部に腕を縫っても外形ループ 1)。

        arms[i] = {"panel", "side", "t0_mm", "t1_mm", "fold_deg", "radius_mm", "length_mm",
                   "outline"(rect/trapezoid/tabs), "relief_mm"}
        """
        new_faces = dict(faces)
        groups: dict = {}
        by_panel: dict = {}
        roots: dict = {}
        for i, arm in enumerate(arms):
            step = steps[arm["panel"]]
            width = half_width_mm + (side_extension_mm[1] if arm["side"] > 0 else side_extension_mm[0])
            fr = compose_arm_frame(step, arm["side"], arm["t0_mm"], arm["t1_mm"], width,
                                   arm["fold_deg"], arm["radius_mm"])
            root = BRepBuilderAPI_MakeEdge(gp_Pnt(*fr["root_a"]), gp_Pnt(*fr["root_b"])).Edge()
            bend = topods.Face(BRepPrimAPI_MakeRevol(
                root, gp_Ax1(gp_Pnt(*fr["centre"]), gp_Dir(*fr["axis"])), fr["angle"]).Shape())
            outline = arm.get("outline") or {"kind": "rect"}
            relief = arm.get("relief_mm", ARM_TIP_RELIEF_MM)
            if outline["kind"] == "tabs":
                face = self._tab_arm_face(fr["a"], fr["b"], fr["tip"], fr["axis"],
                                          outline["height_mm"], outline["tabs"],
                                          outline.get("root_r_mm", MIN_NEUTRAL_PLANE_RADIUS_MM))
            elif outline["kind"] == "trapezoid":
                face = self._trapezoid_arm_face(fr["a"], fr["b"], fr["tip"], fr["axis"],
                                                arm["length_mm"], outline["shrink_a_mm"],
                                                outline["shrink_b_mm"], relief)
            else:
                face = self._arm_face(fr["a"], fr["b"], fr["tip"], fr["axis"], arm["length_mm"], relief)
            new_faces[bend] = f"bend_arm_{i}"
            new_faces[face] = f"arm_{i}"
            groups[i] = [bend, face]
            by_panel[i] = arm["panel"]
            roots[i] = root
        # 腕どうし
        self._check_arm_clearance(groups)
        # 腕と帯の面。根本エッジに触れている面(根本のパネルとその隣の細片)は除く —
        # 名前ではなく幾何で除く(ビード/リブの掃引はパネルを走行方向に細かく割るため)。
        for i, group in groups.items():
            root = roots[i]
            for face, name in faces.items():
                touch = BRepExtrema_DistShapeShape(root, face)
                touch.Perform()
                if touch.Value() < 0.05:
                    continue
                for own in group:
                    dist = BRepExtrema_DistShapeShape(own, face)
                    dist.Perform()
                    if dist.Value() < ARM_CLEARANCE_MM:
                        raise ValueError(
                            f"arm {i} comes within {dist.Value():.2f}mm of {name}. Infeasible.")
        return self._sew(new_faces)

    def _cut_side_notches(self, shape, faces, steps, notches, half_width_mm: float,
                          side_extension_mm):
        """側辺の切欠き(合成族、原則 D「凸包でない基板」)。角の逃げと同じブーリアン切削。

        notches[j] = {"panel", "side", "t_mm"(中心), "kind": "arc"|"rect", "radius_mm"(arc),
                      "depth_mm", "length_mm"(rect), "corner_r_mm"(rect の内側の隅)}
        """
        for notch in notches:
            step = steps[notch["panel"]]
            side = notch["side"]
            width = half_width_mm + (side_extension_mm[1] if side > 0 else side_extension_mm[0])
            frame = _Frame(step["origin"], step["ey"], step["ez"])

            def at(t, y):
                return _add(frame.origin, _add(_scale(step["direction"], t), _scale(frame.ey, y)))
            t_c, y_edge = notch["t_mm"], side * width
            if notch["kind"] == "arc":
                r = notch["radius_mm"]
                p0, p1 = at(t_c - r, y_edge), at(t_c + r, y_edge)
                mid_in = at(t_c, y_edge - side * r)
                mid_out = at(t_c, y_edge + side * r)
                arc_in = GC_MakeArcOfCircle(gp_Pnt(*p0), gp_Pnt(*mid_in), gp_Pnt(*p1)).Value()
                arc_out = GC_MakeArcOfCircle(gp_Pnt(*p1), gp_Pnt(*mid_out), gp_Pnt(*p0)).Value()
                wire = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(arc_in).Edge(),
                                               BRepBuilderAPI_MakeEdge(arc_out).Edge()).Wire()
            else:
                d, L, rc = notch["depth_mm"], notch["length_mm"], notch["corner_r_mm"]
                # (t, y) の2D で内側の2隅を丸めた矩形。外側は辺の外へ 5mm はみ出させる。
                y_in, y_out = y_edge - side * d, y_edge + side * 5.0
                c0, c1 = (t_c - L / 2.0, y_in), (t_c + L / 2.0, y_in)
                o0, o1 = (t_c - L / 2.0, y_out), (t_c + L / 2.0, y_out)
                f0 = _fillet_corner_2d(o0, c0, c1, rc)
                f1 = _fillet_corner_2d(c0, c1, o1, rc)
                pts2 = [("line", o0, f0[0]), ("arc", f0[0], f0[1], f0[2]), ("line", f0[2], f1[0]),
                        ("arc", f1[0], f1[1], f1[2]), ("line", f1[2], o1), ("line", o1, o0)]
                maker = BRepBuilderAPI_MakeWire()
                for e in pts2:
                    if e[0] == "line":
                        maker.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*at(*e[1])), gp_Pnt(*at(*e[2]))).Edge())
                    else:
                        arc = GC_MakeArcOfCircle(gp_Pnt(*at(*e[1])), gp_Pnt(*at(*e[2])),
                                                 gp_Pnt(*at(*e[3]))).Value()
                        maker.Add(BRepBuilderAPI_MakeEdge(arc).Edge())
                wire = maker.Wire()
            face = BRepBuilderAPI_MakeFace(wire, True).Face()
            shift = gp_Trsf()
            shift.SetTranslation(gp_Vec(*_scale(frame.ez, -CUT_TOOL_HALF_DEPTH_MM)))
            face = topods.Face(BRepBuilderAPI_Transform(face, shift, True).Shape())
            tool = BRepPrimAPI_MakePrism(face, gp_Vec(*_scale(frame.ez, 2.0 * CUT_TOOL_HALF_DEPTH_MM))).Shape()
            cut = BRepAlgoAPI_Cut(shape, tool)
            cut.Build()
            if not cut.IsDone():
                raise ValueError("side notch boolean failed. Infeasible.")
            faces = {
                topods.Face(m): name
                for f, name in faces.items()
                for m in (list(cut.Modified(f)) or [f])
                if not cut.IsDeleted(f)
            }
            shape = cut.Shape()
        return shape, faces

    # -------------------------------------------------------- 掃引

    def _sweep_segment(self, step, frame: _Frame, w: Vec3, section, faces, tag: str) -> _Frame:
        """1区間を掃引して面を作り、掃引後のフレームを返す。"""
        if isinstance(step, _Straight):
            vec = step.vector
            gvec = gp_Vec(*vec)
            for edge, role in _edges_of(section, frame):
                face = BRepPrimAPI_MakePrism(edge, gvec).Shape()
                faces[topods.Face(face)] = f"{tag}_{role}"
            return frame.translated(vec)
        sign = 1.0 if step.angle > 0 else -1.0
        centre = _add(frame.origin, _scale(step.normal, -sign * step.radius))
        axis = gp_Ax1(gp_Pnt(*centre), gp_Dir(*w))
        for edge, role in _edges_of(section, frame):
            face = BRepPrimAPI_MakeRevol(edge, axis, step.angle).Shape()
            faces[topods.Face(face)] = f"{tag}_{role}"
        return frame.rotated(centre, w, step.angle)

    def _sweep_uniform(self, path, frame: _Frame, w: Vec3, section, faces) -> None:
        for step in path:
            tag = f"panel_{step.panel}" if isinstance(step, _Straight) else f"bend_{step.fold}"
            frame = self._sweep_segment(step, frame, w, section, faces, tag)

    def _sweep_with_bead(self, path, frame: _Frame, w: Vec3, bead_section, flat_section,
                         bead: BeadParams, min_bearing_radius_mm: float, faces,
                         placed=None) -> None:
        """平地 -> ランアウト -> ビード -> ランアウト -> 平地 の順に掃引する。
        `placed=(s0, s3)` を渡すとビードをその区間に置く(合成族の区間ビード)。"""
        # 区間を s で並べ、ランアウトが収まる直線区間を探す(`general_geometry` と
        # 同じ規則。開始位置を端パネルに固定しない — 固定すると折れ目が締結点の
        # 近くにある構成が全部ビード不可になる)。
        spans, total, cursor = [], 0.0, 0.0
        for step in path:
            span = step.length if isinstance(step, _Straight) else abs(step.angle) * step.radius
            spans.append((cursor, cursor + span, isinstance(step, _Straight)))
            cursor += span
        total = cursor
        inset = 2.0 * min_bearing_radius_mm
        # ランアウトは深さの2倍で**固定**する。空きに合わせて縮めると走り終いの壁が
        # ほぼ垂直になり、実物のビードに見えない(2026-09-04のユーザー指摘)。
        runout = max(BEAD_MIN_RUNOUT_MM, RUNOUT_DEPTH_RATIO * bead.depth_mm)
        if placed is None:
            placed = bead_placement(spans, total, inset, runout)
        else:
            # 区間ビード: ランアウトは直線区間の中に完全に収まっていること
            s0, s3 = placed
            straights = [(a, b) for a, b, st in spans if st]
            if not any(a - 1e-6 <= s0 and s0 + runout <= b + 1e-6 for a, b in straights) or \
                    not any(a - 1e-6 <= s3 - runout and s3 <= b + 1e-6 for a, b in straights):
                raise ValueError("a bead run-out would sit on a bend. Infeasible.")
            if s0 < inset - 1e-6 or s3 > total - inset + 1e-6:
                raise ValueError("the bead span runs into a bearing area. Infeasible.")
        if placed is None:
            raise ValueError(
                f"a bead with {runout:.1f}mm run-outs does not fit between the bearing areas "
                f"(inset {inset:.1f}mm each end of {total:.1f}mm). Infeasible."
            )
        s0, s3 = placed
        s1, s2 = s0 + runout, s3 - runout
        if s2 - s1 < 5.0:
            raise ValueError(
                f"bead run ({s2 - s1:.1f}mm of full section) is too short between the "
                f"{runout:.1f}mm run-outs. Infeasible; not attempting construction."
            )

        cursor = 0.0
        for step in path:
            tag = f"panel_{step.panel}" if isinstance(step, _Straight) else f"bend_{step.fold}"
            if isinstance(step, _Bend):
                span = abs(step.angle) * step.radius
                # **曲げがビード区間の外にあることがある**(開始位置を探索式にしたため)。
                # 無条件にビード断面で掃引すると前後の平地断面と噛み合わず、縫合が
                # 全面的に失敗して部品が崩壊する(2026-09-04に実測: 自由エッジ88本)。
                # ランアウトは必ず直線区間の中にあるので、曲げは区間の内か外の
                # どちらかに完全に入る。
                inside = s1 - 1e-6 <= cursor + span / 2.0 <= s2 + 1e-6
                frame = self._sweep_segment(
                    step, frame, w, bead_section if inside else flat_section,
                    faces, f"bend_{step.fold}")
                cursor += span
                continue
            # 直線区間は断面が変わる位置で分割する。
            cuts = [c for c in (s0, s1, s2, s3) if cursor + 1e-9 < c < cursor + step.length - 1e-9]
            marks = [cursor] + cuts + [cursor + step.length]
            for i in range(len(marks) - 1):
                a, b = marks[i], marks[i + 1]
                piece = step.scaled(b - a)
                mid = 0.5 * (a + b)
                if s0 - 1e-6 <= mid <= s1 + 1e-6 or s2 - 1e-6 <= mid <= s3 + 1e-6:
                    frame = self._loft_runout(piece, frame, bead_section, flat_section,
                                              rising=(mid < 0.5 * (s1 + s2)), faces=faces,
                                              tag=f"runout_{0 if mid < 0.5 * (s1 + s2) else 1}")
                else:
                    sec = bead_section if s1 - 1e-6 <= mid <= s2 + 1e-6 else flat_section
                    frame = self._sweep_segment(piece, frame, w, sec, faces, tag)
            cursor += step.length

    def build_branch_part(self, hub_xy, arms, *, origin: Vec3, hub_u: Vec3, hub_v: Vec3,
                          corner_radius, out_dir: str, part_name: str,
                          check_points=(), gussets=(), fillet_radius=None) -> GeneratedPart:
        """分岐部品(実車026)。平面のハブから、辺ごとに別の軸で腕を折り出す。

        既存の掃引は「全ての曲げ軸が1方向に平行」しか作れないので分岐は作れない。
        ここではハブを平面1枚として作り、**各腕をその根本エッジを軸に回転掃引**する。
        腕ごとに軸が違ってよいのは、軸が「その腕の根本エッジ」だから。

        一枚板から作れるための条件:
        * 展開可能 — 平面と円筒(軸は隣接平面の交線に平行)しか作らないので構築上保証。
        * 展開図が自己交差しない — **凸ハブなら構築上保証**。
        * ハブ頂点のコーナー — 隣り合う腕の間の角を、頂点中心の円弧でくぼませる
          (両端の接線が腕の側端エッジに一致する)。
        * 腕どうしが3Dで干渉しない — `_check_arm_clearance`。

        `gussets` = ガセットで塞ぐ角(頂点番号)の列。その角では両方の腕が90度で、
        腕の側端どうしを垂直な壁(面取り)で繋ぐ。ハブ側は逃がしを残すので小さな
        **窓**が開き、境界ループが1つ増える(実車026はここを絞りで埋めているが、
        絞りは展開不能なので我々のモデルでは作らない)。
        """
        layout = branch_frames(hub_xy, arms, origin=origin, hub_u=hub_u, hub_v=hub_v,
                               corner_radius=corner_radius, fillet_radius=fillet_radius)
        to_space, normal = layout["to_space"], layout["normal"]

        wire = BRepBuilderAPI_MakeWire()
        for elem in layout["outline"]:
            if elem[0] == "line":
                wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*to_space(elem[1])),
                                                 gp_Pnt(*to_space(elem[2]))).Edge())
            else:
                arc = GC_MakeArcOfCircle(gp_Pnt(*to_space(elem[1])), gp_Pnt(*to_space(elem[2])),
                                         gp_Pnt(*to_space(elem[3]))).Value()
                wire.Add(BRepBuilderAPI_MakeEdge(arc).Edge())
        hub = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(*origin), gp_Dir(*normal)), wire.Wire())
        if not hub.IsDone():
            raise ValueError("the hub outline does not bound a planar face. Infeasible.")

        faces: dict = {hub.Face(): "hub"}
        groups: dict = {}
        by_edge = {arm["edge"]: arm for arm in arms}
        for index, fr in layout["arms"].items():
            arm = by_edge[index]
            root = BRepBuilderAPI_MakeEdge(gp_Pnt(*fr["root_a"]), gp_Pnt(*fr["root_b"])).Edge()
            bend = BRepPrimAPI_MakeRevol(
                root, gp_Ax1(gp_Pnt(*fr["centre"]), gp_Dir(*fr["revolve_axis"])),
                fr["angle"]).Shape()
            bend_face = topods.Face(bend)
            faces[bend_face] = f"bend_{index}"
            outline = arm.get("outline") or {"kind": "rect"}
            if outline["kind"] == "step":
                step_faces = self._step_arm_faces(fr, outline, normal)
                for face, suffix in step_faces:
                    faces[face] = f"arm_{index}{suffix}"
                groups[index] = [bend_face] + [f for f, _ in step_faces]
                continue
            if outline["kind"] == "tabs":
                arm_face = self._tab_arm_face(fr["a"], fr["b"], fr["tip"], fr["axis"],
                                              outline["height_mm"], outline["tabs"],
                                              outline.get("root_r_mm", MIN_NEUTRAL_PLANE_RADIUS_MM))
            elif outline["kind"] == "trapezoid":
                arm_face = self._trapezoid_arm_face(
                    fr["a"], fr["b"], fr["tip"], fr["axis"], arm["length_mm"],
                    outline["shrink_a_mm"], outline["shrink_b_mm"],
                    arm.get("relief_mm", ARM_TIP_RELIEF_MM))
            else:
                arm_face = self._arm_face(fr["a"], fr["b"], fr["tip"], fr["axis"],
                                          arm["length_mm"],
                                          arm.get("relief_mm", ARM_TIP_RELIEF_MM))
            faces[arm_face] = f"arm_{index}"
            groups[index] = [bend_face, arm_face]

        for corner in gussets:
            for face, name in self._gusset_faces(layout, by_edge, corner, len(hub_xy)):
                faces[face] = name

        shape, named = self._sew(faces)
        self._check_arm_clearance(groups)
        os.makedirs(out_dir, exist_ok=True)
        stp_path = os.path.abspath(os.path.join(out_dir, part_name + "_mid.stp"))
        _export_step(shape, named, stp_path)
        try:
            check_shape(_read_step(stp_path), tuple(p.position_xyz for p in check_points))
        except ValueError:
            os.remove(stp_path)
            raise
        return GeneratedPart(stp_path=stp_path, catpart_path="",
                             face_labels=tuple(describe_faces(named)))

    def build_drawn_tray(self, hub_xy, walls: dict, arms, *, origin: Vec3, hub_u: Vec3,
                         hub_v: Vec3, seam_fillet_mm: float, corner_radius: float,
                         out_dir: str, part_name: str, check_points=()) -> GeneratedPart:
        """実車057の簡略版: ハブ + 絞りの角(壁A/B) + 曲げタブ(arms、辺2・3)。

        1. ハブ・壁A・壁B を鋭いエッジで作って縫合(3面が頂点 v1 で出会う)。
        2. ハブ–A、ハブ–B、継ぎ目 A–B の3本に OCCT のフィレット(頂点はブレンド面 = 絞り)。
        3. タブ(腕)は辺2・3の根本エッジから回転掃引で曲げ、最後に全部縫合。

        walls[key] = {"fold_deg", "height_mm", "fillet_mm", "tabs": [(s, r)...],
                      "taper_from_mm": 継ぎ目側の細りを始める根本座標(任意)}
        """
        lay = drawn_tray_frames(hub_xy, walls, origin=origin, hub_u=hub_u, hub_v=hub_v)
        normal, to_space, v = lay["normal"], lay["to_space"], lay["vertices"]
        by_edge = {arm["edge"]: arm for arm in arms}
        if set(by_edge) - {2, 3} or (3 in by_edge and "C" in walls):
            raise ValueError("drawn tray arms must sit on free hub edges (2, and 3 without wall C).")

        # --- ハブ: 辺2・3 には腕の根本の頂点を入れておく(フィレット後もそのまま残る)
        roots = {}
        pts = []
        square = walls["A"].get("corner_square_mm", 0.0)
        for i in range(4):
            a_xy, b_xy = hub_xy[i], hub_xy[(i + 1) % 4]
            pts.append(a_xy)
            if i == 2 and square > 0.0:
                # v2 から辺1 の内向き法線方向へ square だけ進んだ点を挟む(凸は保たれる)
                e1 = _normalize2((hub_xy[2][0] - hub_xy[1][0], hub_xy[2][1] - hub_xy[1][1]))
                inward = (-e1[1], e1[0])
                pts.append((a_xy[0] + inward[0] * square, a_xy[1] + inward[1] * square))
            if i in by_edge:
                d = _normalize2((b_xy[0] - a_xy[0], b_xy[1] - a_xy[1]))
                s0, s1 = by_edge[i]["root_from_mm"], by_edge[i]["root_to_mm"]
                r0 = (a_xy[0] + d[0] * s0, a_xy[1] + d[1] * s0)
                r1 = (a_xy[0] + d[0] * s1, a_xy[1] + d[1] * s1)
                roots[i] = (to_space(r0), to_space(r1))
                pts.extend([r0, r1])
        wire = BRepBuilderAPI_MakeWire()
        for i in range(len(pts)):
            p0, p1 = pts[i], pts[(i + 1) % len(pts)]
            if math.dist(p0, p1) < MIN_EDGE_LENGTH_MM:
                raise ValueError("hub outline has a degenerate edge. Infeasible.")
            wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*to_space(p0)), gp_Pnt(*to_space(p1))).Edge())
        hub = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(*origin), gp_Dir(*normal)), wire.Wire())
        if not hub.IsDone():
            raise ValueError("the hub outline does not bound a planar face. Infeasible.")

        # --- 壁A/B(鋭い): 根本、遠い端、上端(タブ + 継ぎ目へ向けた細り)、継ぎ目
        wall_faces = {}
        for key, fr in lay["walls"].items():
            wall_faces[key] = self._drawn_wall_face(fr, walls[key], key)
        sew = BRepBuilderAPI_Sewing(SEW_TOLERANCE_MM)
        sew.Add(hub.Face())
        for f in wall_faces.values():
            sew.Add(f)
        sew.Perform()
        shell = sew.SewedShape()
        if sew.NbMultipleEdges() > 0:
            raise ValueError("the drawn corner does not sew cleanly. Infeasible.")

        # --- フィレット: 2面に共有されるエッジ = ハブ–A、ハブ–B、継ぎ目
        amap = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp.MapShapesAndAncestors(shell, TopAbs_EDGE, TopAbs_FACE, amap)
        fillet = BRepFilletAPI_MakeFillet(shell)
        expected = 2 * len(lay["seams"]) + len(lay["walls"]) - len(lay["seams"])
        added = 0
        for i in range(1, amap.Size() + 1):
            if amap.FindFromIndex(i).Size() != 2:
                continue
            edge = topods.Edge(amap.FindKey(i))
            curve = BRepAdaptor_Curve(edge)
            p0, p1 = curve.Value(curve.FirstParameter()), curve.Value(curve.LastParameter())
            ends = ((p0.X(), p0.Y(), p0.Z()), (p1.X(), p1.Y(), p1.Z()))

            def near(q):
                return any(math.dist(e, q) < 1e-3 for e in ends)
            radius = None
            for name, seam in lay["seams"].items():
                if near(seam["end"]) and near(seam["vertex"]):
                    radius = seam_fillet_mm
            for key, (a, b) in (("B", (v[0], v[1])), ("A", (v[1], v[2])), ("C", (v[3], v[0]))):
                if key in walls and near(a) and near(b):
                    radius = walls[key]["fillet_mm"]
            if radius is None:
                continue
            fillet.Add(radius, edge)
            added += 1
        if added != expected:
            raise ValueError(f"expected {expected} shared edges at the drawn corners, found {added}. Infeasible.")
        try:
            fillet.Build()
        except Exception as exc:        # OCCT が例外で落ちる組み合わせもある
            raise ValueError(f"the corner fillet failed: {exc}. Infeasible.")
        if not fillet.IsDone():
            raise ValueError("the corner fillet did not converge. Infeasible.")
        blended = fillet.Shape()

        # --- 面に名前を付ける(フィレット後の面は種類で判別)
        faces: dict = {}
        explorer = TopExp_Explorer(blended, TopAbs_FACE)
        counts: dict = {}
        while explorer.More():
            face = topods.Face(explorer.Current())
            kind = BRepAdaptor_Surface(face).GetType()
            props = GProp_GProps()
            brepgprop.SurfaceProperties(face, props)
            c = props.CentreOfMass()
            centre = (c.X(), c.Y(), c.Z())
            if kind == 0:   # 平面: ハブ / 壁A / 壁B を法線で判別
                n = BRepAdaptor_Surface(face).Plane().Axis().Direction()
                nv = (n.X(), n.Y(), n.Z())
                best = max([("hub", normal)] + [(f"wall_{k}", fr["normal"])
                                                 for k, fr in lay["walls"].items()],
                           key=lambda kv: abs(_dot(kv[1], nv)))
                name = best[0]
            elif kind == 1:
                name = "draw_fillet"
            else:
                name = "draw_corner"
            counts[name] = counts.get(name, 0) + 1
            faces[face] = name if counts[name] == 1 else f"{name}_{counts[name]}"
            explorer.Next()

        # --- タブ(腕)
        groups: dict = {}
        for index, arm in by_edge.items():
            a, b = roots[index]
            axis = _normalize(_add(b, _scale(a, -1.0)))
            angle = math.radians(arm["fold_deg"])
            centre = _add(a, _scale(normal, -arm["radius_mm"]))
            outward = _normalize(_cross(axis, normal))
            tip = _normalize(_add(_scale(outward, math.cos(angle)), _scale(normal, -math.sin(angle))))
            root = BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge()
            bend = topods.Face(BRepPrimAPI_MakeRevol(
                root, gp_Ax1(gp_Pnt(*centre), gp_Dir(*axis)), angle).Shape())
            a2, b2 = _rotate_about(a, centre, axis, angle), _rotate_about(b, centre, axis, angle)
            outline = arm.get("outline") or {"kind": "rect"}
            if outline["kind"] == "trapezoid":
                arm_face = self._trapezoid_arm_face(a2, b2, tip, axis, arm["length_mm"],
                                                    outline["shrink_a_mm"], outline["shrink_b_mm"],
                                                    arm.get("relief_mm", ARM_TIP_RELIEF_MM))
            else:
                arm_face = self._arm_face(a2, b2, tip, axis, arm["length_mm"],
                                          arm.get("relief_mm", ARM_TIP_RELIEF_MM))
            faces[bend] = f"bend_{index}"
            faces[arm_face] = f"arm_{index}"
            groups[index] = [bend, arm_face]
        groups[99] = list(wall_faces.values())     # 絞り壁(整数キー: 干渉判定はキーを比較する)

        shape, named = self._sew(faces)
        self._check_arm_clearance(groups)     # 腕どうし + 腕と絞り壁(フィレット前の壁で保守的に)
        os.makedirs(out_dir, exist_ok=True)
        stp_path = os.path.abspath(os.path.join(out_dir, part_name + "_mid.stp"))
        _export_step(shape, named, stp_path)
        try:
            check_shape(_read_step(stp_path), tuple(p.position_xyz for p in check_points))
        except ValueError:
            os.remove(stp_path)
            raise
        return GeneratedPart(stp_path=stp_path, catpart_path="",
                             face_labels=tuple(describe_faces(named)))

    @classmethod
    def _drawn_wall_face(cls, fr: dict, spec: dict, key: str):
        """絞り壁の輪郭(壁座標 t=高さ方向, s=根本方向、根本は s=0..w)。

        両端はそれぞれ「自由端」(根本に直交する縦の辺)か「継ぎ目」(継ぎ目の端 P ->
        継ぎ目に直交するノッチ -> 細り -> 上端)。上端(高さ h)に半円の溶接タブ。
        ノッチ長はフィレットの接線長 + 2(フィレットの終端が直交平面で切れて円弧になる)。
        """
        w, h = fr["width"], fr["height"]
        tabs = sorted(spec.get("tabs", ()), key=lambda t: -t[0])
        fillet = spec.get("fillet_mm", 0.0)
        taper = spec.get("taper_from_mm", {})

        def seam_piece(end, side):
            """継ぎ目側の (P, Q, 細りの始点 s) を返す。"""
            t_p, s_p, dt, ds = end["t"], end["s"], end["dt"], end["ds"]
            notch = spec.get("notch_mm", drawn_notch_mm(fillet, end["turn"]))
            perp = (-ds, dt) if side == "left" else (ds, -dt)     # 材料側へ向く直交方向
            q = (t_p + notch * perp[0], s_p + notch * perp[1])
            default = s_p + (0.35 * w if side == "left" else -0.35 * w)
            s_taper = taper.get(side, default) if end["tapers"] else q[1]
            return (t_p, s_p), q, s_taper

        left, right = fr["ends"]["left"], fr["ends"]["right"]
        section = [("line", (0.0, 0.0), (0.0, w))]
        # 右端(s=w 側)
        if right is None:
            section.append(("line", (0.0, w), (h, w)))
            top_hi = w
        else:
            P, Q, s_taper = seam_piece(right, "right")
            section.append(("line", (0.0, w), P))
            section.append(("line", P, Q))
            section.append(("line", Q, (h, s_taper)))
            top_hi = s_taper
        # 左端の細り始点
        if left is None:
            top_lo = 0.0
        else:
            P_l, Q_l, s_taper_l = seam_piece(left, "left")
            top_lo = s_taper_l
        # 上端(s 減少方向)にタブ(根元に凹R)。深さ2のフランジの根本はここで頂点にする。
        root_r = spec.get("tab_root_r_mm", MIN_NEUTRAL_PLANE_RADIUS_MM)
        splits = tuple(spec.get("top_splits", ()))
        for s in splits:
            if not (top_lo + 1.0 < s < top_hi - 1.0):
                raise ValueError("a depth-2 flange root runs off the wall top. Infeasible.")
            for s_c, r in tabs:
                if abs(s - s_c) < tab_footprint_mm(r, root_r) + 1.0:
                    raise ValueError("a depth-2 flange root hits a weld tab. Infeasible.")
        top = tab_top_section(h, tabs, top_hi, top_lo, root_r)
        section += _split_top_line(top, h, splits)
        # 左端
        if left is None:
            section.append(("line", (h, 0.0), (0.0, 0.0)))
        else:
            section.append(("line", (h, top_lo), Q_l))
            section.append(("line", Q_l, P_l))
            section.append(("line", P_l, (0.0, 0.0)))
        return cls._outline_face(fr["a"], fr["tip"], fr["axis"], section)

    def build_box_bracket(self, hub_xy, walls: dict, closed, arms, *, origin: Vec3,
                          hub_u: Vec3, hub_v: Vec3, corner_r_mm: float, out_dir: str,
                          part_name: str, flanges=(), rib=None, deep=(), check_points=(),
                          check_radii=()) -> GeneratedPart:
        """実車002-024 の族: 凸多角形のハブ + 任意の辺に立てた壁 + 角の連結(絞り)。

        `build_drawn_tray`(ハブ四角 + 壁3 + 継ぎ目2)の一般化。手順は同じで、
        **鋭いエッジで縫ってからフィレット**する(腕を生やしてから連結する手法は不可)。

        1. ハブ(多角形。壁の無い辺には腕の根本の頂点を入れる)と壁を鋭いエッジで縫合。
        2. 壁の根本 N 本 + 継ぎ目 M 本を**すべて同じ半径**でフィレット(半径が違うと
           頂点のブレンド面の縁が自由曲線になってゲートAで落ちる。2026-09-06 実測)。
        3. 腕(タブ)は壁の無い辺の根本エッジから回転掃引で曲げ、最後に全部縫合。

        walls[edge] = {"fold_deg", "height_mm", "tabs": [(s, r)...], "taper_from_mm"}
        closed = 継ぎ目にする頂点番号の集合。arms[i] = {"edge", "root_from_mm", ...}
        flanges[i] = {"wall", "from_mm", "to_mm", "side"(±1), "fold_deg", "radius_mm",
                      "length_mm", "outline"} — 壁の上端から折る**深さ2**のパネル。
        """
        lay = box_frames(hub_xy, walls, closed, origin=origin, hub_u=hub_u, hub_v=hub_v)
        normal, to_space, v = lay["normal"], lay["to_space"], lay["vertices"]
        n = len(hub_xy)
        by_edge = {arm["edge"]: arm for arm in arms}
        if set(by_edge) & set(walls):
            raise ValueError("an arm and a wall cannot share a hub edge. Infeasible.")
        self._check_open_corners(lay, n)

        # --- ハブの輪郭(腕の根本の頂点を挟む)
        roots = {}
        pts = []
        for i in range(n):
            a_xy, b_xy = hub_xy[i], hub_xy[(i + 1) % n]
            pts.append(a_xy)
            d = _normalize2((b_xy[0] - a_xy[0], b_xy[1] - a_xy[1]))
            cuts = []
            if i in by_edge:
                s0, s1 = by_edge[i]["root_from_mm"], by_edge[i]["root_to_mm"]
                r0 = (a_xy[0] + d[0] * s0, a_xy[1] + d[1] * s0)
                r1 = (a_xy[0] + d[0] * s1, a_xy[1] + d[1] * s1)
                roots[i] = (to_space(r0), to_space(r1))
                cuts += [r0, r1]
            if i in lay["walls"]:
                s0, s1 = lay["walls"][i]["root_mm"]
                if s0 > MIN_EDGE_LENGTH_MM:
                    cuts.append((a_xy[0] + d[0] * s0, a_xy[1] + d[1] * s0))
                if lay["walls"][i]["edge_len"] - s1 > MIN_EDGE_LENGTH_MM:
                    cuts.append((a_xy[0] + d[0] * s1, a_xy[1] + d[1] * s1))
            pts.extend(sorted(cuts, key=lambda q: (q[0] - a_xy[0]) * d[0] + (q[1] - a_xy[1]) * d[1]))
        if rib is None:
            base_faces = {self._planar_face(pts, to_space, origin, normal): "hub"}
            rib_edges: list = []
        else:
            base_faces, rib_edges = self._hub_with_rib(pts, rib, to_space, origin, normal)

        # --- 壁(鋭い)。フィレット半径は全部そろえる。深さ2のフランジの根本は頂点にしておく。
        splits: dict = {}
        for fl in flanges:
            if fl["wall"] not in lay["walls"]:
                raise ValueError("a depth-2 flange needs a wall to sit on. Infeasible.")
            if fl["to_mm"] - fl["from_mm"] < MIN_EDGE_LENGTH_MM:
                raise ValueError("a depth-2 flange root is degenerate. Infeasible.")
            splits.setdefault(fl["wall"], []).extend((fl["from_mm"], fl["to_mm"]))
        wall_faces = {}
        for edge, fr in lay["walls"].items():
            spec = dict(walls[edge], fillet_mm=corner_r_mm,
                        top_splits=sorted(splits.get(edge, ())))
            wall_faces[edge] = self._drawn_wall_face(fr, spec, str(edge))
        sew = BRepBuilderAPI_Sewing(SEW_TOLERANCE_MM)
        for f in base_faces:
            sew.Add(f)
        for f in wall_faces.values():
            sew.Add(f)
        sew.Perform()
        shell = sew.SewedShape()
        if sew.NbMultipleEdges() > 0:
            raise ValueError("the box corners do not sew cleanly. Infeasible.")

        # --- フィレット: 壁の根本 + 継ぎ目。端点を共有するものだけを**一括で**、
        # 離れているものは**別々に**掛ける。同じハブ面の上で隣り合う 2 本の根本を
        # 一括で掛けると、たとえ 70mm 離れていても OCCT が収束しない(2026-09-08 実測)。
        targets = [(fr["a"], fr["b"]) for fr in lay["walls"].values()]
        targets += [(sm["vertex"], sm["end"]) for sm in lay["seams"].values()]
        targets += rib_edges
        blended = shell
        for group in _group_touching(targets):
            blended = self._fillet_roots(blended, group, corner_r_mm)

        faces = self._name_box_faces(blended, lay, normal, origin, base_faces)

        # --- 腕(タブ)
        groups: dict = {}
        group_roots: dict = {}
        for index, arm in by_edge.items():
            a, b = roots[index]
            axis = _normalize(_add(b, _scale(a, -1.0)))
            angle = math.radians(arm["fold_deg"])
            centre = _add(a, _scale(normal, -arm["radius_mm"]))
            outward = _normalize(_cross(axis, normal))
            tip = _normalize(_add(_scale(outward, math.cos(angle)),
                                  _scale(normal, -math.sin(angle))))
            root = BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge()
            bend = topods.Face(BRepPrimAPI_MakeRevol(
                root, gp_Ax1(gp_Pnt(*centre), gp_Dir(*axis)), angle).Shape())
            a2, b2 = _rotate_about(a, centre, axis, angle), _rotate_about(b, centre, axis, angle)
            outline = arm.get("outline") or {"kind": "rect"}
            if outline["kind"] == "tabs":
                arm_face = self._tab_arm_face(a2, b2, tip, axis, arm["length_mm"],
                                              outline["tabs"], outline.get("root_r_mm",
                                              MIN_NEUTRAL_PLANE_RADIUS_MM))
            elif outline["kind"] == "trapezoid":
                arm_face = self._trapezoid_arm_face(a2, b2, tip, axis, arm["length_mm"],
                                                    outline["shrink_a_mm"],
                                                    outline["shrink_b_mm"],
                                                    arm.get("relief_mm", ARM_TIP_RELIEF_MM))
            else:
                arm_face = self._arm_face(a2, b2, tip, axis, arm["length_mm"],
                                          arm.get("relief_mm", ARM_TIP_RELIEF_MM))
            faces[bend] = f"bend_arm_{index}"
            faces[arm_face] = f"arm_{index}"
            groups[index] = [bend, arm_face]

        # --- 深さ2のフランジ(壁の上端から折る)
        frames3: dict = {}
        for j, fl in enumerate(flanges):
            fr = lay["walls"][fl["wall"]]
            h, side = fr["height"], fl["side"]
            p0 = _add(fr["a"], _scale(fr["tip"], h), _scale(fr["axis"], fl["from_mm"]))
            p1 = _add(fr["a"], _scale(fr["tip"], h), _scale(fr["axis"], fl["to_mm"]))
            n_eff = _scale(fr["normal"], side)
            angle = math.radians(fl["fold_deg"])
            centre = _add(p0, _scale(n_eff, -fl["radius_mm"]))
            rot = _scale(fr["axis"], side)
            tip_f = _normalize(_add(_scale(fr["tip"], math.cos(angle)),
                                    _scale(n_eff, -math.sin(angle))))
            root = BRepBuilderAPI_MakeEdge(gp_Pnt(*p0), gp_Pnt(*p1)).Edge()
            bend = topods.Face(BRepPrimAPI_MakeRevol(
                root, gp_Ax1(gp_Pnt(*centre), gp_Dir(*rot)), angle).Shape())
            a2 = _rotate_about(p0, centre, rot, angle)
            b2 = _rotate_about(p1, centre, rot, angle)
            outline = fl.get("outline") or {"kind": "rect"}
            splits3 = [x for d in deep if d["flange"] == j
                       for x in (d["from_mm"], d["to_mm"])]
            if outline["kind"] == "tabs":
                face = self._tab_arm_face(a2, b2, tip_f, fr["axis"], fl["length_mm"],
                                          outline["tabs"], outline.get(
                                              "root_r_mm", MIN_NEUTRAL_PLANE_RADIUS_MM))
            elif outline["kind"] == "trapezoid":
                face = self._trapezoid_arm_face(a2, b2, tip_f, fr["axis"], fl["length_mm"],
                                                outline["shrink_a_mm"], outline["shrink_b_mm"],
                                                fl.get("relief_mm", ARM_TIP_RELIEF_MM))
            else:
                face = self._arm_face(a2, b2, tip_f, fr["axis"], fl["length_mm"],
                                      fl.get("relief_mm", ARM_TIP_RELIEF_MM),
                                      tip_splits=splits3)
            if splits3 and outline["kind"] != "rect":
                raise ValueError("a depth-3 panel needs a rectangular flange. Infeasible.")
            faces[bend] = f"bend_flange_{j}"
            faces[face] = f"flange_{j}"
            groups[200 + j] = [bend, face]
            group_roots[200 + j] = root
            frames3[j] = (a2, tip_f, fr["axis"], _normalize(_cross(tip_f, fr["axis"])),
                          fl["length_mm"])

        # --- 深さ3のパネル(深さ2フランジの先端から折る)
        for m, dp in enumerate(deep):
            if dp["flange"] not in frames3:
                raise ValueError("a depth-3 panel needs its flange. Infeasible.")
            a2, tip_f, ax, n_f, length = frames3[dp["flange"]]
            side3 = dp["side"]
            n_eff = _scale(n_f, side3)
            rot = _scale(ax, -side3)
            angle = math.radians(dp["fold_deg"])
            p0 = _add(a2, _scale(tip_f, length), _scale(ax, dp["to_mm"]))
            p1 = _add(a2, _scale(tip_f, length), _scale(ax, dp["from_mm"]))
            centre = _add(p0, _scale(n_eff, dp["radius_mm"]))
            root3 = BRepBuilderAPI_MakeEdge(gp_Pnt(*p0), gp_Pnt(*p1)).Edge()
            bend3 = topods.Face(BRepPrimAPI_MakeRevol(
                root3, gp_Ax1(gp_Pnt(*centre), gp_Dir(*rot)), angle).Shape())
            tip3 = _normalize(_add(_scale(tip_f, math.cos(angle)),
                                   _scale(n_eff, math.sin(angle))))
            a3 = _rotate_about(p0, centre, rot, angle)
            b3 = _rotate_about(p1, centre, rot, angle)
            face3 = self._arm_face(a3, b3, tip3, _scale(ax, -1.0), dp["length_mm"],
                                   dp.get("relief_mm", ARM_TIP_RELIEF_MM))
            faces[bend3] = f"bend_deep_{m}"
            faces[face3] = f"deep_{m}"
            groups[300 + m] = [bend3, face3]
            group_roots[300 + m] = root3

        groups[99] = list(wall_faces.values())     # 壁(フィレット前の面で保守的に)

        shape, named = self._sew(faces)
        self._check_arm_clearance(groups, group_roots)
        os.makedirs(out_dir, exist_ok=True)
        stp_path = os.path.abspath(os.path.join(out_dir, part_name + "_mid.stp"))
        _export_step(shape, named, stp_path)
        try:
            read = _read_step(stp_path)
            check_shape(read, tuple(p.position_xyz for p in check_points))
            if check_radii:
                _check_bearing_margin(read, list(check_points), list(check_radii))
        except ValueError:
            os.remove(stp_path)
            raise
        return GeneratedPart(stp_path=stp_path, catpart_path="",
                             face_labels=tuple(describe_faces(named)))

    @staticmethod
    def _planar_face(pts, to_space, origin: Vec3, normal: Vec3):
        """2D の点列(ハブ座標)から平面の面を作る。"""
        wire = BRepBuilderAPI_MakeWire()
        for i in range(len(pts)):
            p0, p1 = pts[i], pts[(i + 1) % len(pts)]
            if math.dist(p0, p1) < MIN_EDGE_LENGTH_MM:
                raise ValueError("hub outline has a degenerate edge. Infeasible.")
            wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*to_space(p0)), gp_Pnt(*to_space(p1))).Edge())
        face = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(*origin), gp_Dir(*normal)), wire.Wire())
        if not face.IsDone():
            raise ValueError("the outline does not bound a planar face. Infeasible.")
        return face.Face()

    @classmethod
    def _hub_with_rib(cls, pts, rib: dict, to_space, origin: Vec3, normal: Vec3):
        """ハブを x = t0..t1 の帯で切り、その帯を台形のリブ(フランク・天面・フランク)にする。

        リブの折り線はハブ座標の x = 一定で、y 方向に走る。帯の y 側の境界は
        **折り線に直交**しているので、根本フィレットの切り口が円弧になる(45度の面取りだと
        楕円になってゲートAを破る。2026-09-08 実測)。したがってこの構成は
        「ハブの上下辺(辺0と辺2)が x 軸に平行で、そこに壁も腕も無い」四角形だけで使う。

        戻り値: ({面: 名前}, [フィレットする折り線の端点の対])
        """
        t0, t1 = rib["t0_mm"], rib["t1_mm"]
        h, alpha = rib["height_mm"], math.radians(rib["fold_deg"])
        run = h / math.tan(alpha)
        if t1 - t0 < 2.0 * run + 4.0:
            raise ValueError("the rib band is too narrow for its height. Infeasible.")
        ys = sorted({q[1] for q in pts})
        y0, y1 = ys[0], ys[-1]
        left = [q for q in pts if q[0] < t0 - 1e-9]
        right = [q for q in pts if q[0] > t1 + 1e-9]
        if not left or not right:
            raise ValueError("the rib does not fit inside the hub. Infeasible.")
        if any(t0 - 1e-9 <= q[0] <= t1 + 1e-9 for q in pts):
            raise ValueError("the rib band hits a hub outline vertex. Infeasible.")
        # 帯の左右の切り口は (t, y0) - (t, y1)。辺0(y=y0)と辺2(y=y1)を横切る前提。
        cut0 = [(t0, y0), (t0, y1)]
        cut1 = [(t1, y0), (t1, y1)]
        # 左右のピースは、元の輪郭のうち帯の外側 + 切り口
        order = sorted(range(len(pts)), key=lambda i: pts[i][0])
        left_poly = [q for q in pts if q[0] < t0]
        right_poly = [q for q in pts if q[0] > t1]
        left_poly = cls._order_ccw(left_poly + cut0)
        right_poly = cls._order_ccw(right_poly + cut1)

        def at(x, y, z):
            return _add(to_space((x, y)), _scale(normal, z))

        faces = {cls._planar_face(left_poly, to_space, origin, normal): "hub",
                 cls._planar_face(right_poly, to_space, origin, normal): "hub_2"}
        quads = (("rib_flank", [at(t0, y0, 0.0), at(t0, y1, 0.0),
                                at(t0 + run, y1, h), at(t0 + run, y0, h)]),
                 ("rib_top", [at(t0 + run, y0, h), at(t0 + run, y1, h),
                              at(t1 - run, y1, h), at(t1 - run, y0, h)]),
                 ("rib_flank_2", [at(t1 - run, y0, h), at(t1 - run, y1, h),
                                  at(t1, y1, 0.0), at(t1, y0, 0.0)]))
        for name, quad in quads:
            own = _normalize(_cross(_add(quad[1], _scale(quad[0], -1.0)),
                                    _add(quad[3], _scale(quad[0], -1.0))))
            faces[cls._quad_face(quad, own)] = name
        edges = [(at(t0, y0, 0.0), at(t0, y1, 0.0)),
                 (at(t0 + run, y0, h), at(t0 + run, y1, h)),
                 (at(t1 - run, y0, h), at(t1 - run, y1, h)),
                 (at(t1, y0, 0.0), at(t1, y1, 0.0))]
        return faces, edges

    @staticmethod
    def _order_ccw(pts):
        """凸な点集合を反時計回りに並べる(重複は落とす)。"""
        uniq = []
        for q in pts:
            if not any(math.dist(q, o) < 1e-6 for o in uniq):
                uniq.append(q)
        cx = sum(q[0] for q in uniq) / len(uniq)
        cy = sum(q[1] for q in uniq) / len(uniq)
        return sorted(uniq, key=lambda q: math.atan2(q[1] - cy, q[0] - cx))

    @staticmethod
    def _fillet_roots(shape, segments, radius: float):
        """端点で指定した共有エッジ群に、同じ半径のフィレットを一括で掛ける。"""
        amap = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, amap)
        fillet = BRepFilletAPI_MakeFillet(shape)
        added = 0
        for i in range(1, amap.Size() + 1):
            if amap.FindFromIndex(i).Size() != 2:
                continue
            edge = topods.Edge(amap.FindKey(i))
            curve = BRepAdaptor_Curve(edge)
            p0, p1 = curve.Value(curve.FirstParameter()), curve.Value(curve.LastParameter())
            ends = ((p0.X(), p0.Y(), p0.Z()), (p1.X(), p1.Y(), p1.Z()))
            if not any(all(any(math.dist(e, q) < 1e-3 for e in ends) for q in seg)
                       for seg in segments):
                continue
            fillet.Add(radius, edge)
            added += 1
        if added != len(segments):
            raise ValueError(f"expected {len(segments)} shared edges at the box corners, "
                             f"found {added}. Infeasible.")
        try:
            fillet.Build()
        except Exception as exc:
            raise ValueError(f"the corner fillet failed: {exc}. Infeasible.")
        if not fillet.IsDone():
            raise ValueError("the corner fillet did not converge. Infeasible.")
        return fillet.Shape()

    @staticmethod
    def _check_open_corners(lay: dict, n: int) -> None:
        """隣り合う辺の両方に壁があるなら、その角は必ず継ぎ目にする。

        開けたままだと 2 本の根本フィレットが頂点で衝突して OCCT が収束しない
        (2026-09-08 実測: 折れ角 75 度の隣接 2 壁を開けた場合に不収束)。角を開けたければ
        ハブの輪郭に切欠きを入れて根本を離す必要がある — 第1期ではその形は作らない。
        """
        for k in range(n):
            kx, ky = (k - 1) % n, k % n
            if k in lay["seams"] or kx not in lay["walls"] or ky not in lay["walls"]:
                continue
            fx, fy = lay["walls"][kx], lay["walls"][ky]
            gap_x = fx["edge_len"] - fx["root_mm"][1]
            gap_y = fy["root_mm"][0]
            need = BOX_CORNER_RELIEF_MM
            if gap_x < need or gap_y < need:
                raise ValueError(f"open corner {k}: the two wall roots are only "
                                 f"{gap_x:.1f}/{gap_y:.1f}mm from the vertex "
                                 f"(need {need}mm of relief). Infeasible.")

    @staticmethod
    def _name_box_faces(blended, lay: dict, normal: Vec3, origin: Vec3, base_faces=None) -> dict:
        """フィレット後の面に名前を付ける。平面はハブ/壁を**法線と面までの距離**で選ぶ
        (向かい合う壁は法線が逆平行で、|内積| だけだと区別が付かない)。リブの面は
        フィレット前の面の重心と法線で対応付ける。"""
        refs = [("hub", normal, origin)] + [(f"wall_{k}", fr["normal"], fr["a"])
                                            for k, fr in lay["walls"].items()]
        for face, name in (base_faces or {}).items():
            if name.startswith("rib"):
                nd = BRepAdaptor_Surface(face).Plane().Axis().Direction()
                refs.append((name, (nd.X(), nd.Y(), nd.Z()), face_centroid(face)))
        faces: dict = {}
        counts: dict = {}
        explorer = TopExp_Explorer(blended, TopAbs_FACE)
        while explorer.More():
            face = topods.Face(explorer.Current())
            kind = BRepAdaptor_Surface(face).GetType()
            if kind == 0:
                nd = BRepAdaptor_Surface(face).Plane().Axis().Direction()
                nv = (nd.X(), nd.Y(), nd.Z())
                centre = face_centroid(face)
                near = [r for r in refs if abs(_dot(r[1], nv)) > 0.9] or refs
                name = min(near, key=lambda r: abs(_dot(_add(centre, _scale(r[2], -1.0)), r[1])))[0]
            elif kind == 1:
                name = "draw_fillet"
            else:
                name = "draw_corner"
            counts[name] = counts.get(name, 0) + 1
            faces[face] = name if counts[name] == 1 else f"{name}_{counts[name]}"
            explorer.Next()
        return faces

    def build_channel_seat(self, *, out_dir: str, part_name: str, seat_depth_mm: float,
                           seat_corner_mm: float, check_points=(), **geom) -> GeneratedPart:
        """実車144型: チャンネル(ウェブ + 斜めに切った壁2枚) + 壁から外へ折った座面2枚。

        ウェブをハブ、壁を対辺の腕、座面を壁の斜辺から折る腕、の**2段**折り。
        トポロジは固定(ハブ + 壁2 + 座面2)なので専用ビルダー。平面と円筒しか作らないので
        展開可能で、既存の全ゲートがそのまま効く。
        """
        lay = channel_seat_frames(**geom)
        normal, at = lay["normal"], lay["at"]
        L, W = geom["length_mm"], geom["width_mm"]

        wire = BRepBuilderAPI_MakeWire()
        corners = [at(0.0, 0.0), at(L, 0.0), at(L, W), at(0.0, W)]
        for i in range(4):
            wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*corners[i]),
                                             gp_Pnt(*corners[(i + 1) % 4])).Edge())
        hub = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(*geom["origin"]), gp_Dir(*normal)), wire.Wire())
        if not hub.IsDone():
            raise ValueError("web face failed. Infeasible.")
        faces: dict = {hub.Face(): "web"}
        groups: dict = {}

        for key, w in lay["walls"].items():
            root = BRepBuilderAPI_MakeEdge(gp_Pnt(*w["root_a"]), gp_Pnt(*w["root_b"])).Edge()
            bend = BRepPrimAPI_MakeRevol(
                root, gp_Ax1(gp_Pnt(*w["centre"]), gp_Dir(*w["axis"])), w["angle"]).Shape()
            faces[topods.Face(bend)] = f"wall_bend_{key}"
            # 壁 = 四角形 a2 -> b2 -> q_b -> q_a (下辺 q_a-q_b が座面の根本エッジ)
            wall = self._quad_face([w["a2"], w["b2"], w["q_b"], w["q_a"]], w["normal"])
            faces[wall] = f"wall_{key}"
            diag = BRepBuilderAPI_MakeEdge(gp_Pnt(*w["q_a"]), gp_Pnt(*w["q_b"])).Edge()
            theta = w["seat_theta"]
            axis_dir = w["diag"] if theta > 0 else _scale(w["diag"], -1.0)
            seat_bend = BRepPrimAPI_MakeRevol(
                diag, gp_Ax1(gp_Pnt(*w["seat_centre"]), gp_Dir(*axis_dir)), abs(theta)).Shape()
            faces[topods.Face(seat_bend)] = f"seat_bend_{key}"
            seat = self._arm_face(w["seat_a"], w["seat_b"], w["seat_out"], w["diag"],
                                  seat_depth_mm, seat_corner_mm)
            faces[seat] = f"seat_{key}"
            groups[key] = [topods.Face(bend), wall, topods.Face(seat_bend), seat]

        shape, named = self._sew(faces)
        self._check_arm_clearance(groups)
        os.makedirs(out_dir, exist_ok=True)
        stp_path = os.path.abspath(os.path.join(out_dir, part_name + "_mid.stp"))
        _export_step(shape, named, stp_path)
        try:
            check_shape(_read_step(stp_path), tuple(p.position_xyz for p in check_points))
        except ValueError:
            os.remove(stp_path)
            raise
        return GeneratedPart(stp_path=stp_path, catpart_path="",
                             face_labels=tuple(describe_faces(named)))

    @staticmethod
    def _quad_face(pts, normal: Vec3):
        wire = BRepBuilderAPI_MakeWire()
        for i in range(4):
            if math.dist(pts[i], pts[(i + 1) % 4]) < MIN_EDGE_LENGTH_MM:
                raise ValueError("wall quad has a degenerate edge. Infeasible.")
            wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*pts[i]), gp_Pnt(*pts[(i + 1) % 4])).Edge())
        face = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(*pts[0]), gp_Dir(*normal)), wire.Wire())
        if not face.IsDone():
            raise ValueError("wall quad is not planar. Infeasible.")
        return face.Face()

    @staticmethod
    def _outline_face(a, tip: Vec3, axis: Vec3, section):
        """腕の平面内の (t, s) 座標で書いた断面要素列 [("line",p,q) | ("arc",p,m,q)] を面にする。"""
        def at(ts):
            return _add(a, _add(_scale(tip, ts[0]), _scale(axis, ts[1])))
        wire = BRepBuilderAPI_MakeWire()
        for elem in section:
            if elem[0] == "line":
                if math.dist(elem[1], elem[2]) < MIN_EDGE_LENGTH_MM:
                    continue
                wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*at(elem[1])), gp_Pnt(*at(elem[2]))).Edge())
            else:
                arc = GC_MakeArcOfCircle(gp_Pnt(*at(elem[1])), gp_Pnt(*at(elem[2])),
                                         gp_Pnt(*at(elem[3]))).Value()
                wire.Add(BRepBuilderAPI_MakeEdge(arc).Edge())
        if not wire.IsDone():
            raise ValueError("the arm outline does not close into a wire. Infeasible.")
        plane = gp_Pln(gp_Pnt(*a), gp_Dir(*_cross(tip, axis)))
        face = BRepBuilderAPI_MakeFace(plane, wire.Wire())
        if not face.IsDone():
            raise ValueError("the arm outline does not bound a planar face. Infeasible.")
        return face.Face()

    @classmethod
    def _step_arm_faces(cls, fr: dict, outline: dict, hub_normal: Vec3):
        """段差腕: 斜面(平面) + 2本目の曲げ(円筒) + 棚(平面、舌つき)。[(face, 名前の接尾辞)]。"""
        sf = step_frame(fr, outline, hub_normal)
        width = fr["width"]
        run = outline["run_mm"]
        slope = cls._outline_face(fr["a"], fr["tip"], fr["axis"], [
            ("line", (0.0, 0.0), (0.0, width)), ("line", (0.0, width), (run, width)),
            ("line", (run, width), (run, 0.0)), ("line", (run, 0.0), (0.0, 0.0))])
        edge = BRepBuilderAPI_MakeEdge(gp_Pnt(*sf["a2"]), gp_Pnt(*sf["b2"])).Edge()
        bend2 = topods.Face(BRepPrimAPI_MakeRevol(
            edge, gp_Ax1(gp_Pnt(*sf["centre2"]), gp_Dir(*sf["axis2"])), sf["angle2"]).Shape())
        depth, relief = outline["ledge_mm"], outline.get("relief_mm", ARM_TIP_RELIEF_MM)
        tongue = outline.get("tongue")
        pa, pb = (0.0, 0.0), (0.0, width)
        if tongue:
            # 棚の先端辺の s=[s0,s1] から長さ l の舌を出す。舌の先の2隅を丸める。
            s_c, w_t, l_t = tongue["s_mm"], tongue["width_mm"], tongue["length_mm"]
            s0, s1 = s_c - w_t / 2.0, s_c + w_t / 2.0
            if s0 < relief + 0.5 or s1 > width - relief - 0.5:
                raise ValueError("the tongue runs off the ledge. Infeasible.")
            r_t = min(relief, w_t / 2.0 - 0.5)
            c_far_b, c_far_a = (depth + l_t, s1), (depth + l_t, s0)
            fb = _fillet_corner_2d((depth, s1), c_far_b, c_far_a, r_t)
            fa = _fillet_corner_2d(c_far_b, c_far_a, (depth, s0), r_t)
            cb = _fillet_corner_2d(pb, (depth, width), (depth, s1), relief)
            ca = _fillet_corner_2d((depth, s0), (depth, 0.0), pa, relief)
            section = [("line", pa, pb), ("line", pb, cb[0]), ("arc", cb[0], cb[1], cb[2]),
                       ("line", cb[2], (depth, s1)), ("line", (depth, s1), fb[0]),
                       ("arc", fb[0], fb[1], fb[2]), ("line", fb[2], fa[0]),
                       ("arc", fa[0], fa[1], fa[2]), ("line", fa[2], (depth, s0)),
                       ("line", (depth, s0), ca[0]), ("arc", ca[0], ca[1], ca[2]),
                       ("line", ca[2], pa)]
        else:
            cb = _fillet_corner_2d(pb, (depth, width), (depth, 0.0), relief)
            ca = _fillet_corner_2d((depth, width), (depth, 0.0), pa, relief)
            section = [("line", pa, pb), ("line", pb, cb[0]), ("arc", cb[0], cb[1], cb[2]),
                       ("line", cb[2], ca[0]), ("arc", ca[0], ca[1], ca[2]), ("line", ca[2], pa)]
        ledge = cls._outline_face(sf["a3"], sf["tip2"], fr["axis"], section)
        return [(slope, ""), (bend2, "_step"), (ledge, "_ledge")]

    @classmethod
    def _tab_arm_face(cls, a, b, tip: Vec3, axis: Vec3, height_mm: float, tabs,
                      root_r_mm: float = MIN_NEUTRAL_PLANE_RADIUS_MM):
        """立ち上がり + 半円の溶接タブ(実車1285-18)。tabs = [(根本エッジ上の位置 s, 半径 r), ...]。
        タブは高さ height の上端に載る半円で、円の中心が溶接点。根元に凹R(root_r)。"""
        width = math.dist(a, b)
        section = [("line", (0.0, 0.0), (0.0, width)), ("line", (0.0, width), (height_mm, width))]
        section += tab_top_section(height_mm, tabs, width, 0.0, root_r_mm)
        section.append(("line", (height_mm, 0.0), (0.0, 0.0)))
        return cls._outline_face(a, tip, axis, section)

    @classmethod
    def _trapezoid_arm_face(cls, a, b, tip: Vec3, axis: Vec3, length_mm: float,
                            shrink_a_mm: float, shrink_b_mm: float, relief_mm: float):
        """先端の辺が根本より狭い台形の腕(shrink=0 で矩形)。先端の2隅は relief で丸める。"""
        width = math.dist(a, b)
        tip_w = width - shrink_a_mm - shrink_b_mm
        if tip_w < 2.0 * relief_mm + 1.0 or shrink_a_mm < 0.0 or shrink_b_mm < 0.0:
            raise ValueError("the trapezoid tip is too narrow for its reliefs. Infeasible.")
        # (t, s) 座標。根本 a=(0,0), b=(0,width)。先端は t=length。
        pa, pb = (0.0, 0.0), (0.0, width)
        cb, ca = (length_mm, width - shrink_b_mm), (length_mm, shrink_a_mm)
        fb = _fillet_corner_2d(pb, cb, ca, relief_mm)
        fa = _fillet_corner_2d(cb, ca, pa, relief_mm)
        section = [("line", pa, pb), ("line", pb, fb[0]), ("arc", fb[0], fb[1], fb[2]),
                   ("line", fb[2], fa[0]), ("arc", fa[0], fa[1], fa[2]), ("line", fa[2], pa)]
        return cls._outline_face(a, tip, axis, section)

    @staticmethod
    def _gusset_faces(layout, by_edge, corner: int, count: int):
        """ガセット = 隣り合う2本の90度腕の側端を繋ぐ垂直な壁(面取り)。

        両腕が90度なら腕の側端はどちらもハブ法線に平行なので、腕・曲げ・ガセットは
        すべてハブ面に垂直な柱面になる。つまりハブ面上の2D問題: 腕Aの壁の線 → 曲げ
        (円弧) → 面取りの直線 → 曲げ → 腕Bの壁の線 という折れ線を、腕の高さぶん
        法線方向へ押し出せばよい。腕の壁は branch_frames が corner_radius で接線長ぶん
        詰めてあるので、腕の壁の端 = 曲げの始点になる。
        """
        before, after = (corner - 1) % count, corner
        fa, fb = layout["arms"][before], layout["arms"][after]
        normal = layout["normal"]
        for fr in (fa, fb):
            if abs(_dot(fr["tip"], normal) + 1.0) > 0.02:
                raise ValueError("gusset needs both arms folded 90deg. Skipped.")
        pa0, pb0 = fa["b"], fb["a"]                                   # 曲げ出口、角側の端
        da = _normalize(_add(fa["b"], _scale(fa["a"], -1.0)))         # 腕Aの壁の向き(角へ)
        db = _normalize(_add(fb["b"], _scale(fb["a"], -1.0)))         # 腕Bの壁の向き(角から)
        # 腕は -法線側へ折れる(裏側)。壁は「深い方の曲げ出口」から「浅い方の腕先」まで。
        base = min(_dot(pa0, normal), _dot(pb0, normal))
        top = max(_dot(_add(fa["b"], _scale(fa["tip"], by_edge[before]["length_mm"])), normal),
                  _dot(_add(fb["a"], _scale(fb["tip"], by_edge[after]["length_mm"])), normal))
        if base - top < GUSSET_MIN_HEIGHT_MM:
            raise ValueError("gusset has no height between the two arms. Skipped.")

        def lift(p, h):
            return _add(p, _scale(normal, h - _dot(p, normal)))

        pa, pb = lift(pa0, base), lift(pb0, base)
        if math.dist(pa, pb) < MIN_EDGE_LENGTH_MM:
            raise ValueError("gusset chord collapsed. Skipped.")
        # 曲げは腕Aの壁の端 pa から**ちょうど**始まる(そこが縫合の共有エッジ、
        # 許容0.01mm)。折れ線の角 P = pa + t*da、弦は P から pb へ向き、
        # t = R*tan(turn/2) は弦の向きに依存する — 不動点反復で自己整合させる
        # (1回で打ち切ると 0.18mm ずれて腕と縫えない、2026-09-05実測)。
        t = 0.0
        for _ in range(12):
            corner_pt = _add(pa, _scale(da, t))
            chord = _normalize(_add(pb, _scale(corner_pt, -1.0)))
            turn = math.acos(max(-1.0, min(1.0, _dot(da, chord))))
            t = GUSSET_BEND_R_MM * math.tan(turn / 2.0)
        corner_pt = _add(pa, _scale(da, t))
        chord = _normalize(_add(pb, _scale(corner_pt, -1.0)))

        def fillet(p, d_in, d_out):
            """点 p で向き d_in -> d_out に曲がる壁を半径 GUSSET_BEND_R_MM で丸める。
            戻り値 (始点, 中点, 終点)。p が折れ線の角、始点/終点が接点。"""
            cos_turn = max(-1.0, min(1.0, _dot(d_in, d_out)))
            turn = math.acos(cos_turn)
            if turn < math.radians(3.0):
                return None
            t = GUSSET_BEND_R_MM * math.tan(turn / 2.0)
            s0 = _add(p, _scale(d_in, -t))
            s1 = _add(p, _scale(d_out, t))
            bis = _normalize(_add(_scale(d_in, -1.0), d_out))
            centre = _add(p, _scale(bis, GUSSET_BEND_R_MM / math.cos(turn / 2.0)))
            mid = _add(centre, _scale(_normalize(_add(p, _scale(centre, -1.0))), GUSSET_BEND_R_MM))
            return (s0, mid, s1)

        # 腕Aから折り出し、腕Bには**重ねる**(曲げで繋がない)。両側を曲げで留めると
        # 面取りの両端に「ハブ・腕・ガセット」の3枚が集まり、扇形角の和が
        # 90+90+ハブ角 < 360 になって展開できない(実車026はここを絞りで埋めている)。
        # 重ねならタブの端が自由エッジになり、角の窓は外形線に繋がって境界ループは
        # 1本のまま。接合は溶接前提(実物の箱の角と同じ作り)。
        arc_a = fillet(corner_pt, da, chord)
        height = _scale(normal, top - base)
        out = []
        if arc_a is not None:
            arc = GC_MakeArcOfCircle(gp_Pnt(*arc_a[0]), gp_Pnt(*arc_a[1]),
                                     gp_Pnt(*arc_a[2])).Value()
            edge = BRepBuilderAPI_MakeEdge(arc).Edge()
            out.append((topods.Face(BRepPrimAPI_MakePrism(edge, gp_Vec(*height)).Shape()),
                        f"gusset_bend_{corner}"))
        start_pt = arc_a[2] if arc_a else pa
        end_pt = _add(pb, _scale(chord, -GUSSET_LAP_GAP_MM))
        if math.dist(start_pt, end_pt) < MIN_EDGE_LENGTH_MM:
            raise ValueError("gusset tab is shorter than its bend. Skipped.")
        edge = BRepBuilderAPI_MakeEdge(gp_Pnt(*start_pt), gp_Pnt(*end_pt)).Edge()
        out.append((topods.Face(BRepPrimAPI_MakePrism(edge, gp_Vec(*height)).Shape()),
                    f"gusset_{corner}"))
        return out

    @staticmethod
    def _arm_face(a, b, tip: Vec3, axis: Vec3, length_mm: float, relief_mm: float,
                  tip_splits=()):
        """腕のパネル。根本エッジ a-b はそのまま使い(縫合のため)、先端の2隅を
        relief_mm で落とす — 実務の余肉カット。締結点の必要平面は族が長さと幅で担保する。
        tip_splits は先端の辺に入れる頂点(深さ3のパネルの根本)。"""
        width = math.dist(a, b)
        if relief_mm > 0.5 * min(width, length_mm) - 0.5:
            raise ValueError("the arm relief eats the whole tip. Infeasible.")

        def at(t, s):
            return _add(a, _add(_scale(tip, t), _scale(axis, s)))

        wire = BRepBuilderAPI_MakeWire()
        wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge())
        pts = [(0.0, width), (length_mm - relief_mm, width)]
        wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*at(*pts[0])), gp_Pnt(*at(*pts[1]))).Edge())
        for t0, s0, tm, sm, t1, s1 in (
            (length_mm - relief_mm, width, length_mm - relief_mm * 0.293,
             width - relief_mm * 0.293, length_mm, width - relief_mm),
        ):
            arc = GC_MakeArcOfCircle(gp_Pnt(*at(t0, s0)), gp_Pnt(*at(tm, sm)),
                                     gp_Pnt(*at(t1, s1))).Value()
            wire.Add(BRepBuilderAPI_MakeEdge(arc).Edge())
        chain = [width - relief_mm]
        for sv in sorted(tip_splits, reverse=True):
            if not (relief_mm + 1.0 < sv < width - relief_mm - 1.0):
                raise ValueError("a depth-3 panel root runs off the flange tip. Infeasible.")
            chain.append(sv)
        chain.append(relief_mm)
        for s_a, s_b in zip(chain, chain[1:]):
            wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*at(length_mm, s_a)),
                                             gp_Pnt(*at(length_mm, s_b))).Edge())
        arc = GC_MakeArcOfCircle(
            gp_Pnt(*at(length_mm, relief_mm)),
            gp_Pnt(*at(length_mm - relief_mm * 0.293, relief_mm * 0.293)),
            gp_Pnt(*at(length_mm - relief_mm, 0.0))).Value()
        wire.Add(BRepBuilderAPI_MakeEdge(arc).Edge())
        wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*at(length_mm - relief_mm, 0.0)),
                                         gp_Pnt(*a)).Edge())
        if not wire.IsDone():
            raise ValueError("the arm outline does not close into a wire. Infeasible.")
        plane = gp_Pln(gp_Pnt(*a), gp_Dir(*_cross(tip, axis)))
        face = BRepBuilderAPI_MakeFace(plane, wire.Wire())
        if not face.IsDone():
            raise ValueError("the arm outline does not bound a planar face. Infeasible.")
        return face.Face()

    @staticmethod
    def _check_arm_clearance(groups, roots=None) -> None:
        """折ったあとの腕どうしの当たり。展開図が重ならなくても3Dでぶつかりうる。
        既存の `check_shape` は縫合済みシェルの妥当性しか見ないので面の貫通を拾えない。

        roots[key] = その群の根本エッジ。相手の群のうち根本エッジに触れている面(= 親)は
        除く(名前ではなく幾何で親を判定する。2026-09-06 の誤検出の教訓)。"""
        roots = roots or {}
        keys = sorted(groups)

        def parent(edge, face) -> bool:
            probe = BRepExtrema_DistShapeShape(edge, face)
            probe.Perform()
            return probe.IsDone() and probe.Value() < 0.05

        for i, ka in enumerate(keys):
            for kb in keys[i + 1:]:
                for fa in groups[ka]:
                    for fb in groups[kb]:
                        # 相手の群に自分の親(根本エッジに接する面)が居るなら、その組は見ない
                        if ka in roots and parent(roots[ka], fb):
                            continue
                        if kb in roots and parent(roots[kb], fa):
                            continue
                        probe = BRepExtrema_DistShapeShape(fa, fb)
                        probe.Perform()
                        if probe.IsDone() and probe.Value() < ARM_CLEARANCE_MM:
                            raise ValueError(
                                f"arms {ka} and {kb} come within {probe.Value():.2f}mm "
                                f"(minimum {ARM_CLEARANCE_MM}mm). Infeasible.")

    def build_flat_plate(self, points, *, margin_mm: float, corner_radius_mm: float,
                         out_dir: str, part_name: str) -> GeneratedPart:
        """平板 x 多点締結(実車031 / 1285-20)。

        外形は**締結点の凸包の各辺を margin だけ外へ平行移動して交わらせ、隅を
        corner_radius で丸めたもの**。実車の平板も「締結点の配置に沿った多角形 +
        小さな隅R」で、材料が締結点を余白つきで包む形をしている(実測: 031は
        余白18〜22mmに対し隅R4.5〜8、20は隅R5)。余白と隅Rは別物なので分けて持つ。
        辺は直線・隅は円弧だけなのでゲートAは自明に通る。
        """
        normal = _normalize(points[0].normal_xyz)
        seed = (0.0, 0.0, 1.0) if abs(normal[2]) < 0.9 else (1.0, 0.0, 0.0)
        u = _normalize(_cross(normal, seed))
        v = _cross(normal, u)
        origin = points[0].position_xyz

        def to_plane(p):
            d = _add(p, _scale(origin, -1.0))
            return (_dot(d, u), _dot(d, v))

        def to_space(xy):
            return _add(origin, _add(_scale(u, xy[0]), _scale(v, xy[1])))

        hull = _convex_hull_2d([to_plane(p.position_xyz) for p in points])
        if len(hull) < 3:
            raise ValueError("the fastening points are collinear; no plate outline. Infeasible.")

        # 各辺を外へ margin 平行移動し、隣どうしの交点(マイター点)を出す。
        count = len(hull)
        lines = []
        for i in range(count):
            a, b = hull[i], hull[(i + 1) % count]
            n = _outward_normal_2d(a, b)
            lines.append(((a[0] + margin_mm * n[0], a[1] + margin_mm * n[1]),
                          _normalize2((b[0] - a[0], b[1] - a[1])), n))
        miters = []
        for i in range(count):
            p0, d0, n0 = lines[i]
            p1, d1, n1 = lines[(i + 1) % count]
            cross = d0[0] * d1[1] - d0[1] * d1[0]
            if abs(cross) < 1e-9:
                raise ValueError("the plate outline has a degenerate corner. Infeasible.")
            t = ((p1[0] - p0[0]) * d1[1] - (p1[1] - p0[1]) * d1[0]) / cross
            miters.append(((p0[0] + t * d0[0], p0[1] + t * d0[1]), n0, n1))

        section: list = []
        first_start = None
        for i in range(count):
            miter, n0, n1 = miters[i]
            b = hull[(i + 1) % count]
            prev_miter = miters[i - 1][0]
            d = _normalize2((miter[0] - prev_miter[0], miter[1] - prev_miter[1]))
            nxt = miters[(i + 1) % count][0]
            d_next = _normalize2((nxt[0] - miter[0], nxt[1] - miter[1]))
            cos_turn = max(-1.0, min(1.0, d[0] * d_next[0] + d[1] * d_next[1]))
            interior = math.pi - math.acos(cos_turn)
            bisector = _normalize2((n0[0] + n1[0], n0[1] + n1[1]))
            reach = margin_mm / max(1e-9, math.sin(interior / 2.0))
            if reach <= FLAT_PLATE_MITER_LIMIT * margin_mm:
                # 通常の隅: マイター点を小さなRで丸める(実車の隅R4.5〜8mm)。
                back = corner_radius_mm / math.tan(interior / 2.0)
                arc_start = (miter[0] - back * d[0], miter[1] - back * d[1])
                arc_end = (miter[0] + back * d_next[0], miter[1] + back * d_next[1])
                centre = (miter[0] - (corner_radius_mm / math.sin(interior / 2.0)) * bisector[0],
                          miter[1] - (corner_radius_mm / math.sin(interior / 2.0)) * bisector[1])
                arc_mid = (centre[0] + corner_radius_mm * bisector[0],
                           centre[1] + corner_radius_mm * bisector[1])
            else:
                # 鋭い隅はマイターが棘になるので、余白の半径でそのまま回り込ませる
                # (実車031の外形にもR36.5/R49の大きな円弧がある)。
                arc_start = (b[0] + margin_mm * n0[0], b[1] + margin_mm * n0[1])
                arc_end = (b[0] + margin_mm * n1[0], b[1] + margin_mm * n1[1])
                arc_mid = (b[0] + margin_mm * bisector[0], b[1] + margin_mm * bisector[1])
            if first_start is None:
                first_start = arc_start
            else:
                if math.dist(section[-1][3], arc_start) < FLAT_PLATE_MIN_STRAIGHT_MM:
                    raise ValueError("the corner fillets eat a whole edge. Infeasible.")
                section.append(("line", section[-1][3], arc_start, f"edge_{i}"))
            section.append(("arc", arc_start, arc_mid, arc_end, f"corner_{i}"))
        if math.dist(section[-1][3], first_start) < FLAT_PLATE_MIN_STRAIGHT_MM:
            raise ValueError("the corner fillets eat a whole edge. Infeasible.")
        section.append(("line", section[-1][3], first_start, "edge_0"))

        edges = []
        for elem in section:
            if elem[0] == "line":
                _, a2, b2, _role = elem
                edges.append(BRepBuilderAPI_MakeEdge(
                    gp_Pnt(*to_space(a2)), gp_Pnt(*to_space(b2))).Edge())
            else:
                _, a2, m2, b2, _role = elem
                arc = GC_MakeArcOfCircle(gp_Pnt(*to_space(a2)), gp_Pnt(*to_space(m2)),
                                         gp_Pnt(*to_space(b2))).Value()
                edges.append(BRepBuilderAPI_MakeEdge(arc).Edge())
        wire = BRepBuilderAPI_MakeWire()
        for edge in edges:
            wire.Add(edge)
        if not wire.IsDone():
            raise ValueError("the plate outline does not close into a wire. Infeasible.")
        face = BRepBuilderAPI_MakeFace(
            gp_Pln(gp_Pnt(*origin), gp_Dir(*normal)), wire.Wire())
        if not face.IsDone():
            raise ValueError("the plate outline does not bound a planar face. Infeasible.")

        shape, faces = self._sew({face.Face(): "plate"})
        os.makedirs(out_dir, exist_ok=True)
        stp_path = os.path.abspath(os.path.join(out_dir, part_name + "_mid.stp"))
        _export_step(shape, faces, stp_path)
        try:
            check_shape(_read_step(stp_path), tuple(p.position_xyz for p in points))
        except ValueError:
            os.remove(stp_path)
            raise
        return GeneratedPart(stp_path=stp_path, catpart_path="",
                             face_labels=tuple(describe_faces(faces)))

    def _sweep_tapered_bead(self, path, frame: _Frame, w: Vec3, bead: BeadParams,
                            flange, *, half_width_mm: float, narrow_half_width_mm: float,
                            min_bearing_radius_mm: float, lift: int, faces) -> None:
        """実車014型の掃引。掃引は必ず「対の側 -> 孤立点」の向きで来る。

        区間の並び(実車014の実測どおり):

          0 ─ ビード(広い) ─ 曲げ ─ 絞り(ビードは通ったまま) ─ ビード(狭い)
            ─ 逃げ(ビード->平地) ─ 平地(狭い、孤立点の座面) ─ total

        * ビードは対の側の板端から通す。対の2点は帯の中心線をまたいでいるので
          ビードはその間を通り、座面を避ける必要がない(実車014実測: 対のパネルは
          板端までビード深さ3.5mmが一定)。逃げるのは孤立点の側だけ。
        * 帯幅は**ビードが通っている途中で**絞る。実車014も絞りは中間パネル
          (ビードあり)で起き、孤立点のパネルは絞りきった一定幅。孤立点の座面に
          絞りが食い込まないためにも、絞りは逃げより手前に置く必要がある。
        * 絞りは smoothstep で刻む。1段のルールド面で落とすとフランジの稜線が
          両端で折れる(2026-09-04のユーザー指摘)。
        """
        wide_bead, y_breaks = _bead_section(bead, lift, half_width_mm)
        footprint = -y_breaks[1]                       # ビードが幅方向に占める半分
        if narrow_half_width_mm < footprint + 1.0:
            raise ValueError(
                f"the narrow end ({narrow_half_width_mm:.1f}mm half width) cannot hold the "
                f"bead footprint ({footprint:.1f}mm). Infeasible; not attempting construction."
            )

        def wrap(section, hw):
            return _with_flange(section, flange, hw) if flange is not None else section

        def bead_at(hw):
            return wrap(_bead_section(bead, lift, hw)[0], hw)

        narrow_breaks = [-narrow_half_width_mm, *y_breaks[1:-1], narrow_half_width_mm]
        sec_bead_wide = wrap(wide_bead, half_width_mm)
        sec_bead_narrow = bead_at(narrow_half_width_mm)
        sec_flat_narrow = wrap(_flat_section(narrow_breaks), narrow_half_width_mm)

        spans, cursor = [], 0.0
        for step in path:
            span = step.length if isinstance(step, _Straight) else abs(step.angle) * step.radius
            spans.append((cursor, cursor + span, isinstance(step, _Straight)))
            cursor += span
        total = cursor
        runout = max(BEAD_MIN_RUNOUT_MM, RUNOUT_DEPTH_RATIO * bead.depth_mm)
        # 孤立点は端から最大 bearing だけ手前に置かれ、その周り bearing が平地で要る。
        # よって端から 3*bearing は絞りきった平地にしておく。
        s3 = total - TAPER_CLEAR_BEARINGS * min_bearing_radius_mm
        s2 = s3 - runout
        # 絞りは最後の直線区間のうち、逃げより手前に取る。
        last = next((sp for sp in reversed(spans) if sp[2]), None)
        if last is None or s2 <= last[0]:
            raise ValueError(
                "the bead run-out does not fit in the last straight stretch. Infeasible."
            )
        available = s2 - last[0]
        t1 = s2
        t0 = t1 - TAPER_LENGTH_SHARE * available
        if t0 <= last[0] + 1e-6:
            raise ValueError("no room to taper the band before the run-out. Infeasible.")

        cursor = 0.0
        for step in path:
            if isinstance(step, _Bend):
                span = abs(step.angle) * step.radius
                if cursor + span > t0 + 1e-6:
                    raise ValueError("a bend falls inside the taper or run-out stretch")
                frame = self._sweep_segment(step, frame, w, sec_bead_wide, faces,
                                            f"bend_{step.fold}")
                cursor += span
                continue
            tag = f"panel_{step.panel}"
            # 絞りは smoothstep の刻みで割る。
            marks = [cursor]
            for c in (*(t0 + (t1 - t0) * k / TAPER_STEPS for k in range(TAPER_STEPS + 1)),
                      s2, s3):
                if cursor + 1e-9 < c < cursor + step.length - 1e-9 and c > marks[-1] + 1e-9:
                    marks.append(c)
            marks.append(cursor + step.length)
            for a, b in zip(marks, marks[1:]):
                piece = step.scaled(b - a)
                mid = 0.5 * (a + b)
                if mid <= t0:
                    frame = self._sweep_segment(piece, frame, w, sec_bead_wide, faces, tag)
                elif mid <= t1:
                    frame = self._loft_runout(
                        piece, frame, bead_at(_taper_width(a, t0, t1, half_width_mm,
                                                           narrow_half_width_mm)),
                        bead_at(_taper_width(b, t0, t1, half_width_mm,
                                             narrow_half_width_mm)),
                        rising=False, faces=faces, tag="taper")
                elif mid <= s2:
                    frame = self._sweep_segment(piece, frame, w, sec_bead_narrow, faces, tag)
                elif mid <= s3:
                    frame = self._loft_runout(piece, frame, sec_bead_narrow, sec_flat_narrow,
                                              rising=False, faces=faces, tag="runout_1")
                else:
                    frame = self._sweep_segment(piece, frame, w, sec_flat_narrow, faces, tag)
            cursor += step.length

    # -------------------------------------------------------- リブ(可変半径のコーナーブレンド)

    def _build_rib_part(self, plan, w: Vec3, ey: Vec3, rib: RibParams,
                        half_width_mm: float, bend_radius_mm: float, faces) -> None:
        """リブ部品は断面掃引では作れないので、面を明示的に張る。

        リブは「曲げのコーナーを幅方向の中央だけ大きな半径で丸めたもの」。
        曲げ線に直交する各断面で パネル1 -> 円弧(半径 rho(y)) -> パネル2 と繋ぐ。
        rho は中央で大きく、リブの端 |y|=c で基準面の曲げRに一致するので、
        **基準面のフィレットと連続**する(遷移帯も頂点も要らない)。

        接線長 T(y) = rho(y)*tan(φ/2) は y に線形なので、パネルの境界は直線になり、
        平面視のフットプリントは菱形になる。パネルとの境目は接線連続。

        丸みをフィレット演算で付けようとすると頂点ブレンドの境界が解析曲線にならず、
        ゲートA(1本の直線/円弧)を通らない(2026-09-04実測: ずれ中央値0.625mm、
        33件中2件しか通過しない)。**丸みは構築で作る。**
        """
        frames, tangents = plan.panel_frames, plan.fold_tangents
        width = half_width_mm
        target = min(rib.fold_index, len(frames) - 2)

        def pt(index: int, run: float, y: float) -> Vec3:
            frame = frames[index]
            return _add(frame.origin, _scale(frame.u, run), _scale(ey, y))

        # 折れ角と接線長は自分で測った符号付き角から出す(plan側の値とは0.2度ずれる)。
        folds = []
        for k in range(len(frames) - 1):
            normal = _normalize(_cross(frames[k].u, frames[k].v))
            phi = _signed_angle_about(frames[k].u, frames[k + 1].u, w)
            folds.append({
                "phi": phi, "run": frames[k].far_run_mm, "normal": normal,
                "half_tan": abs(math.tan(phi / 2.0)),
                "next_base": frames[k + 1].near_run_mm, "axis": w,
            })

        stations = self._rib_stations(rib, width, folds[target], bend_radius_mm)

        # --- パネル(平面多角形) ---
        for k, frame in enumerate(frames):
            near = self._panel_edge(k, frames, folds, target, width, pt, stations, near=True)
            far = self._panel_edge(k, frames, folds, target, width, pt, stations, near=False)
            points = near + list(reversed(far))
            maker = BRepBuilderAPI_MakeWire()
            for i in range(len(points)):
                a, b = points[i], points[(i + 1) % len(points)]
                if math.dist(a, b) < 1e-9:
                    continue
                maker.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge())
            faces[topods.Face(BRepBuilderAPI_MakeFace(maker.Wire(), True).Face())] = f"panel_{k}"

        # --- 曲げ ---
        for k, fold in enumerate(folds):
            cut = bend_radius_mm * fold["half_tan"]
            sign = 1.0 if fold["phi"] > 0 else -1.0
            centre = _add(pt(k, fold["run"] - cut, 0.0),
                          _scale(fold["normal"], -sign * bend_radius_mm))
            axis = gp_Ax1(gp_Pnt(*centre), gp_Dir(*w))
            spans = ([(-width, -rib.half_width_mm), (rib.half_width_mm, width)]
                     if k == target else [(-width, width)])
            for index, (y0, y1) in enumerate(spans):
                edge = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(*pt(k, fold["run"] - cut, y0)),
                    gp_Pnt(*pt(k, fold["run"] - cut, y1))).Edge()
                patch = BRepPrimAPI_MakeRevol(edge, axis, fold["phi"]).Shape()
                faces[topods.Face(patch)] = f"bend_{k}_{index}"
            if k != target:
                continue

            # --- リブ本体: 隣り合う断面の円弧どうしを直線織り面でつなぐ ---
            arcs = [self._rib_arc(pt, k, fold, y, radius) for y, radius in stations]
            for index, (first, second) in enumerate(zip(arcs, arcs[1:])):
                try:
                    face = brepfill.Face(first, second)
                except RuntimeError as exc:
                    raise ValueError(f"rib blend patch {index} failed ({exc})")
                faces[topods.Face(face)] = f"rib_{index}"

    @staticmethod
    def _rib_stations(rib: RibParams, width: float, fold, bend_radius_mm: float):
        """リブ断面を刻む (y, 丸め半径) の列。両端は基準面の曲げRに一致する。"""
        count = max(2, rib.stations)
        values = []
        for i in range(-count, count + 1):
            y = rib.half_width_mm * i / count
            values.append((y, rib.radius_at(y, bend_radius_mm)))
        return values

    @staticmethod
    def _rib_arc(pt, k: int, fold, y: float, radius: float):
        """幅座標 y での、両パネルに接する円弧(パネルk -> パネルk+1)。"""
        cut = radius * fold["half_tan"]
        sign = 1.0 if fold["phi"] > 0 else -1.0
        start = pt(k, fold["run"] - cut, y)
        centre = _add(pt(k, fold["run"] - cut, 0.0), _scale(fold["normal"], -sign * radius))
        centre = _add(centre, _scale(pt(k, 0.0, y), 1.0), _scale(pt(k, 0.0, 0.0), -1.0))
        end = _rotate_about(start, centre, _sub_axis(fold), fold["phi"])
        mid = _rotate_about(start, centre, _sub_axis(fold), fold["phi"] / 2.0)
        arc = GC_MakeArcOfCircle(gp_Pnt(*start), gp_Pnt(*mid), gp_Pnt(*end)).Value()
        return BRepBuilderAPI_MakeEdge(arc).Edge()

    @staticmethod
    def _panel_edge(k, frames, folds, target: int, width: float, pt, stations,
                    *, near: bool) -> list:
        """パネル k の手前側/奥側の境界を y 昇順の点列で返す。

        リブが載る曲げに面する側は、接線長 T(y)=rho(y)*tan(φ/2) をなぞる折れ線になる
        (rho が y に線形なので実際には直線2本 + 外側の直線)。
        """
        if near:
            if k == 0:
                run = frames[0].near_run_mm
                return [pt(0, run, -width), pt(0, run, width)]
            fold = folds[k - 1]
            base = frames[k].near_run_mm
            if k - 1 != target:
                cut = RIB_BEND_RADIUS_MM * fold["half_tan"]
                return [pt(k, base + cut, -width), pt(k, base + cut, width)]
            edge = [pt(k, base + radius * fold["half_tan"], y) for y, radius in stations]
            outer = RIB_BEND_RADIUS_MM * fold["half_tan"]
            return ([pt(k, base + outer, -width)] + edge + [pt(k, base + outer, width)])
        if k == len(frames) - 1:
            run = frames[k].far_run_mm
            return [pt(k, run, -width), pt(k, run, width)]
        fold = folds[k]
        if k != target:
            cut = RIB_BEND_RADIUS_MM * fold["half_tan"]
            return [pt(k, fold["run"] - cut, -width), pt(k, fold["run"] - cut, width)]
        edge = [pt(k, fold["run"] - radius * fold["half_tan"], y) for y, radius in stations]
        outer = RIB_BEND_RADIUS_MM * fold["half_tan"]
        return ([pt(k, fold["run"] - outer, -width)] + edge
                + [pt(k, fold["run"] - outer, width)])

    def _loft_runout(self, step: _Straight, frame: _Frame, bead_section, flat_section,
                     *, rising: bool, faces, tag: str) -> _Frame:
        """ランアウト。`BRepOffsetAPI_ThruSections` は断面エッジをB-splineに作り直して
        しまう(実測: ビード部品あたり20本の非解析エッジ)ので、要素ペアごとに
        `brepfill.Face` で直線織り面を張る — 入力エッジがそのまま境界になる。"""
        nxt = frame.translated(step.vector)
        start_sec, end_sec = (flat_section, bead_section) if rising else (bead_section, flat_section)
        a_edges = _edges_of(start_sec, frame)
        b_edges = _edges_of(end_sec, nxt)
        if len(a_edges) != len(b_edges):
            raise ValueError(f"{tag}: run-out sections have different element counts")
        for index, ((ea, role), (eb, _)) in enumerate(zip(a_edges, b_edges)):
            try:
                face = brepfill.Face(ea, eb)
            except RuntimeError as exc:      # 退化した対(長さ0の要素など)
                raise ValueError(f"{tag}: run-out ruled face {index} failed ({exc})")
            faces[topods.Face(face)] = f"{tag}_{role}"
        return nxt

    # -------------------------------------------------------- 実行可能性(曲げ上のR)

    @staticmethod
    def _bead_lift(path, bead: BeadParams, bend_radius_mm: float,
                   forced: int | None = None) -> int:
        """ビードの立ち上げ向き(+1 = パネル法線側)。曲げ上ではビード頂部の半径が
        R + sign(折れ角)*lift*深さ になるので、最小半径が最大になる向きを選ぶ。
        それでも中立面R最小を割るならInfeasible。

        `forced` を渡すとその向きだけを見る。フランジと同居する部品では、両方が
        同じ側へ出ていないと実物に見えない(2026-09-04のユーザー指摘)。フランジの
        向きは「裏側へ折る」規則で決まっているので、ビードがそれに合わせる。
        """
        bends = [s for s in path if isinstance(s, _Bend)]
        best, best_radius = 1, -1e9
        for lift in ((forced,) if forced is not None else (1, -1)):
            worst = min(
                (bend_radius_mm + (1.0 if b.angle > 0 else -1.0) * lift * bead.depth_mm
                 for b in bends),
                default=bend_radius_mm,
            )
            if worst > best_radius:
                best, best_radius = lift, worst
        if best_radius < MIN_NEUTRAL_PLANE_RADIUS_MM:
            raise ValueError(
                f"a bead of depth {bead.depth_mm:.1f}mm crossing a bend of R="
                f"{bend_radius_mm:.1f}mm leaves a top radius of {best_radius:.1f}mm, below the "
                f"{MIN_NEUTRAL_PLANE_RADIUS_MM:.1f}mm neutral-plane minimum. "
                "Infeasible; not attempting construction."
            )
        return best

    @staticmethod
    def _check_flange_radii(path, flange: FlangeParams, bend_radius_mm: float) -> None:
        for step in path:
            if not isinstance(step, _Bend):
                continue
            # 曲げ上のフランジ壁は「軸に直交する平面内の円環」になるので、壁自体は
            # 曲がらない(曲率は周方向のみ)。したがってR5最小則ではなく、flange.py
            # と同じ凹側クリアランス(2mm、ユーザー承認済み)で判定する。
            radius = bend_radius_mm + (1.0 if step.angle > 0 else -1.0) * flange.direction * flange.height_mm
            if radius < FLANGE_CONCAVE_CLEARANCE_MM:
                raise ValueError(
                    f"the flange (h={flange.height_mm:.1f}mm) crosses a bend of R="
                    f"{bend_radius_mm:.1f}mm on the concave side, leaving {radius:.1f}mm at the "
                    "wall top. Infeasible; not attempting construction."
                )

    # -------------------------------------------------------- 縫合と余肉カット

    @staticmethod
    def _sew(faces: dict):
        sewing = BRepBuilderAPI_Sewing(SEW_TOLERANCE_MM)
        for face in faces:
            sewing.Add(face)
        sewing.Perform()
        shape = sewing.SewedShape()
        renamed = {}
        for face, name in faces.items():
            sewn = sewing.Modified(face) if sewing.IsModified(face) else face
            renamed[topods.Face(sewn)] = name
        return shape, renamed

    @staticmethod
    def _fillet_rib_edges(shape, faces):
        """リブの稜線(V1A/V2A/V1B/V2B と稜AB)を中立面R最小で丸める。

        ユーザー指定(2026-09-04)。基準面の曲げは自前で厳密に張っているが、リブの
        稜線は3辺が1点に集まる頂点ブレンドを伴うので、ここだけは OCCT の
        `BRepFilletAPI_MakeFillet` に任せる。落ちたらその部品は捨てて引き直す
        (混ぜるより安い)。
        """
        rib_faces = {face for face, name in faces.items() if name.startswith("rib_")}
        if not rib_faces:
            return shape, faces
        edge_faces = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces)
        builder = BRepFilletAPI_MakeFillet(shape)
        added = 0
        for i in range(1, edge_faces.Size() + 1):
            edge = topods.Edge(edge_faces.FindKey(i))
            if BRep_Tool.Degenerated(edge):
                continue
            if any(topods.Face(f) in rib_faces for f in edge_faces.FindFromIndex(i)):
                builder.Add(MIN_NEUTRAL_PLANE_RADIUS_MM, edge)
                added += 1
        if added == 0:
            return shape, faces
        try:
            builder.Build()
            done = builder.IsDone()
        except RuntimeError as exc:      # OCCTは内部エラーを例外で投げることがある
            raise ValueError(f"filleting the rib ridges raised ({str(exc)[:60]}). Resample.")
        if not done:
            faulty = builder.NbFaultyContours()
            raise ValueError(
                f"filleting the rib ridges failed ({faulty} faulty contours). Resample."
            )
        result = builder.Shape()
        renamed = {}
        for face, name in faces.items():
            for target in (list(builder.Modified(face)) or [face]):
                renamed.setdefault(topods.Face(target), name)
        sources = [(face_centroid(f), n) for f, n in faces.items()]
        final = {}
        explorer = TopExp_Explorer(result, TopAbs_FACE)
        while explorer.More():
            face = topods.Face(explorer.Current())
            name = renamed.get(face)
            if name is None:      # フィレットが生んだ面(稜線ブレンド)
                centre = face_centroid(face)
                name = "rib_fillet_" + min(sources, key=lambda i: math.dist(i[0], centre))[1]
            final[face] = name
            explorer.Next()
        return result, final

    @staticmethod
    def _unify(shape, faces):
        """同一曲面上の隣接面と一直線のエッジを併合する。

        断面を9要素に割っている都合で、平地区間は9枚の同一平面が並び、端の外形も
        9本の一直線エッジに割れる。放置すると抽出側に「平面|平面の継ぎ目」
        (引継ぎ書 §4.3 の`seam`)と余計な外形頂点を撒くことになるので、最後に潰す。
        面名は履歴で引き継ぐ(併合された面は最初に割り当てられた名前を採る)。"""
        unifier = ShapeUpgrade_UnifySameDomain(shape, True, True, False)
        unifier.Build()
        result = unifier.Shape()
        history = unifier.History()
        renamed: dict = {}
        for face, name in faces.items():
            modified = list(history.Modified(face)) if history is not None else []
            for target in (modified or [face]):
                key = topods.Face(target)
                renamed.setdefault(key, name)
        # 履歴に出てこない面は重心が最も近い元の面から名前を借りる(全ての面に
        # 名前が付くことを保証する — 下流の面ラベル教師が欠けないように)。
        sources = [(face_centroid(f), n) for f, n in faces.items()]
        final: dict = {}
        explorer = TopExp_Explorer(result, TopAbs_FACE)
        while explorer.More():
            face = topods.Face(explorer.Current())
            name = renamed.get(face)
            if name is None:
                centre = face_centroid(face)
                name = min(sources, key=lambda item: math.dist(item[0], centre))[1]
            final[face] = name
            explorer.Next()
        return result, final

    def _apply_corner_relief(self, shape, faces, path, start: Vec3, ey: Vec3, ez: Vec3, w: Vec3,
                             *, half_width_mm: float, radius_mm: float, exclude_side,
                             side_extension_mm: tuple[float, float] = (0.0, 0.0)):
        """両端の隅を R=締結点の必要最小半径 で丸めて落とす(実務の余肉カット)。
        フランジ部品はフランジ側の2隅を残す。"""
        excluded = () if exclude_side is None else (
            (-1, 1) if exclude_side == 0 else (exclude_side,))
        first, last = path[0], path[-1]
        # 端辺に 2*(hw - R) の残りが出る。half_width は bearing半径の1.0〜1.3倍なので
        # ほぼ0になり得て、0.04mmのゴミエッジが生まれる(2026-09-04実測)。少し弱める。
        radius_mm = min(radius_mm, half_width_mm - 0.5)
        if radius_mm < 1.0:
            raise ValueError(
                f"corner relief radius collapses at half width {half_width_mm:.1f}mm. Infeasible."
            )
        if radius_mm > min(first.length, last.length) - 1.0:
            raise ValueError(
                f"corner relief (R={radius_mm:.1f}mm) reaches past the flat bearing stretch "
                f"into a bend fillet. Infeasible; not attempting construction."
            )
        # 終端のフレームを求める(掃引と同じ手順を空回しする)。
        frame = _Frame(start, ey, ez)
        for step in path:
            if isinstance(step, _Straight):
                frame = frame.translated(step.vector)
            else:
                sign = 1.0 if step.angle > 0 else -1.0
                centre = _add(frame.origin, _scale(step.normal, -sign * step.radius))
                frame = frame.rotated(centre, w, step.angle)
        tools = []
        for frm, inward, label in (
            (_Frame(start, ey, ez), first.direction, "p1"),
            (frame, _scale(last.direction, -1.0), "p2"),
        ):
            for side in (-1, 1):
                if side in excluded:
                    continue
                width = half_width_mm + (side_extension_mm[1] if side > 0 else side_extension_mm[0])
                tools.append(self._relief_tool(frm, inward, side, width, radius_mm))
        for tool in tools:
            cut = BRepAlgoAPI_Cut(shape, tool)
            cut.Build()
            if not cut.IsDone():
                raise ValueError("corner relief boolean failed")
            faces = {
                topods.Face(m): name
                for face, name in faces.items()
                for m in (list(cut.Modified(face)) or [face])
                if not cut.IsDeleted(face)
            }
            shape = cut.Shape()
        return shape, faces

    @staticmethod
    def _relief_tool(frame: _Frame, inward: Vec3, side: int, half_width_mm: float, radius_mm: float):
        """1隅ぶんの切り取り角柱。隅の点と、側辺・端辺への接円弧で囲む。"""
        corner = frame.point((side * half_width_mm, 0.0))
        d_end = _scale(frame.ey, -side)                     # 端辺に沿って内側へ
        d_side = _normalize(inward)                         # 側辺に沿って内側へ
        cos_psi = max(-1.0, min(1.0, _dot(d_end, d_side)))
        psi = math.acos(cos_psi)
        if psi < 1e-3 or psi > math.pi - 1e-3:
            raise ValueError("degenerate corner for the relief cut")
        t = radius_mm / math.tan(psi / 2.0)
        tp_end = _add(corner, _scale(d_end, t))
        tp_side = _add(corner, _scale(d_side, t))
        bisector = _normalize(_add(d_end, d_side))
        centre = _add(corner, _scale(bisector, radius_mm / math.sin(psi / 2.0)))
        mid = _add(centre, _scale(_normalize(_add(corner, _scale(centre, -1.0))), radius_mm))
        arc = GC_MakeArcOfCircle(gp_Pnt(*tp_side), gp_Pnt(*mid), gp_Pnt(*tp_end)).Value()
        wire = BRepBuilderAPI_MakeWire(
            BRepBuilderAPI_MakeEdge(arc).Edge(),
            BRepBuilderAPI_MakeEdge(gp_Pnt(*tp_end), gp_Pnt(*corner)).Edge(),
            BRepBuilderAPI_MakeEdge(gp_Pnt(*corner), gp_Pnt(*tp_side)).Edge(),
        ).Wire()
        face = BRepBuilderAPI_MakeFace(wire, True).Face()
        shift = gp_Trsf()
        shift.SetTranslation(gp_Vec(*_scale(frame.ez, -CUT_TOOL_HALF_DEPTH_MM)))
        face = topods.Face(BRepBuilderAPI_Transform(face, shift, True).Shape())
        return BRepPrimAPI_MakePrism(
            face, gp_Vec(*_scale(frame.ez, 2.0 * CUT_TOOL_HALF_DEPTH_MM))
        ).Shape()

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
from OCC.Core.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

from synthetic_generator.bead import BeadParams
from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM, FasteningPoint, Vec3
from synthetic_generator.flange import FLANGE_CONCAVE_CLEARANCE_MM, FlangeParams
from synthetic_generator.general_geometry import (
    BEAD_MIN_RUNOUT_MM,
    bead_placement,
    plan_general_two_point,
)
from synthetic_generator.rib import RibParams

SEW_TOLERANCE_MM = 0.01
# ランアウト長(平地->ビード断面の遷移)。実物のビードの走り終いは深さの2〜3倍程度。
RUNOUT_DEPTH_RATIO = 2.0
MIN_RUNOUT_MM = 3.0
# 隅の余肉カットのツール角柱の張り出し(基準面の両側へこれだけ伸ばす)。カット域の
# 基準面は厳密に平面なので薄くてよい。厚くするとU字部品で反対側のパネルまで削る。
CUT_TOOL_HALF_DEPTH_MM = 5.0
# これ未満のエッジが出た部品は捨てる(引継ぎ書 §4.3 のゴミ幾何)。
MIN_EDGE_LENGTH_MM = 0.05
# エッジが1本の直線/円弧から外れてよい上限[mm](引継ぎ書 §3.4 ゲートA、0.25t の
# 最も厳しい側 t=1.0mm に合わせる)。リブの稜線フィレットは頂点ブレンドの境界が
# 解析曲線にならないことがあり、実測で最大0.797mm外れた(2026-09-04)。
MAX_PRIMITIVE_DEVIATION_MM = 0.25
# 隣接する2面の法線がなす角の上限[度]。板金の中立面は自分の上に折り返らないので、
# これを超えるエッジがあれば掃引の破綻(ねじれ・面の裏返り)を意味する。
# 実測(健全な300部品): 最大 112度。崩壊部品では 180度近くになる。
MAX_DIHEDRAL_TURN_DEG = 150.0


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
    return reader.OneShape()


def check_shape(shape) -> None:
    """出来上がった形の合格判定(通らなければValueErrorで棄却)。

    * 0.05mm未満のゴミエッジが無い(引継ぎ書 §4.3)
    * 全エッジが1本の直線/円弧に載る(ゲートA)
    * シェルが有効
    * **外形が閉じた1本のループ**(崩壊検知。縫合漏れを全部拾う)
    * **隣接面が折り返らない**(崩壊検知。ねじれ・面の裏返りを拾う)
    """
    _reject_junk_edges(shape)
    if not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError("the shell is not valid. Infeasible; resample.")
    loops = boundary_loop_count(shape)
    if loops != 1:
        raise ValueError(
            f"the outline is {loops} closed loops, not 1 -- faces did not sew "
            "(shape collapsed). Infeasible; resample."
        )
    turn = worst_dihedral_deg(shape)
    if turn > MAX_DIHEDRAL_TURN_DEG:
        raise ValueError(
            f"two adjacent faces turn {turn:.0f}deg (limit {MAX_DIHEDRAL_TURN_DEG:.0f}) "
            "-- the surface folds back on itself. Infeasible; resample."
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
        out_dir: str,
        part_name: str,
        bead: BeadParams | None = None,
        flange: FlangeParams | None = None,
        rib: RibParams | None = None,
    ) -> GeneratedPart:
        if sum(x is not None for x in (bead, flange, rib)) > 1:
            raise ValueError("a part takes at most one reinforcement (bead, flange or rib)")
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
            target_folds=target_folds,
        )

        path, start, w, normal0 = _build_path(plan, bend_radius_mm)
        ey = w if _dot(w, plan.panel_frames[0].v) > 0 else _scale(w, -1.0)
        frame = _Frame(start, ey, normal0)

        faces: dict = {}          # face -> name
        if bead is not None:
            lift = self._bead_lift(path, bead, bend_radius_mm)
            section, y_breaks = _bead_section(bead, lift, half_width_mm)
            self._sweep_with_bead(path, frame, w, section, _flat_section(y_breaks),
                                  bead, min_bearing_radius_mm, faces)
        elif rib is not None:
            self._build_rib_part(plan, w, ey, rib, half_width_mm, bend_radius_mm, faces)
        else:
            section = (_flange_section(flange, half_width_mm) if flange is not None
                       else _flat_section([-half_width_mm, half_width_mm]))
            if flange is not None:
                self._check_flange_radii(path, flange, bend_radius_mm)
            self._sweep_uniform(path, frame, w, section, faces)

        shape, faces = self._sew(faces)
        if rib is not None:
            shape, faces = self._fillet_rib_edges(shape, faces)
        shape, faces = self._apply_corner_relief(
            shape, faces, path, start, ey, normal0, w,
            half_width_mm=half_width_mm,
            radius_mm=min_bearing_radius_mm,
            exclude_side=(flange.side if flange is not None else None),
        )
        shape, faces = self._unify(shape, faces)
        os.makedirs(out_dir, exist_ok=True)
        stp_path = os.path.abspath(os.path.join(out_dir, part_name + "_mid.stp"))
        _export_step(shape, faces, stp_path)
        # **検査は書き出したSTEPに対して行う。**下流が読むのはこのファイルであり、
        # STEPの往復で曲線が再近似される(2026-09-04実測: メモリ上0.25mm以内だった
        # フィレット稜線が、読み戻すと0.839mmずれていた)。
        try:
            check_shape(_read_step(stp_path))
        except ValueError:
            os.remove(stp_path)
            raise
        return GeneratedPart(stp_path=stp_path, catpart_path="",
                             face_labels=tuple(describe_faces(faces)))

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
                         bead: BeadParams, min_bearing_radius_mm: float, faces) -> None:
        """平地 -> ランアウト -> ビード -> ランアウト -> 平地 の順に掃引する。"""
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
        placed = bead_placement(spans, total, inset)
        if placed is None:
            raise ValueError(
                "no place on the centreline for a bead (bearing areas and run-outs do not "
                "fit). Infeasible; not attempting construction."
            )
        s0, s3 = placed
        head = next(s1 for a, s1, straight in spans if straight and a <= s0 < s1)
        tail = next(a for a, s1, straight in reversed(spans) if straight and a < s3 <= s1)
        runout = min(RUNOUT_DEPTH_RATIO * bead.depth_mm, head - s0 - 0.5, s3 - tail - 0.5)
        if runout < BEAD_MIN_RUNOUT_MM:
            raise ValueError(
                f"only {runout:.1f}mm is left for the bead run-out "
                f"(need >= {BEAD_MIN_RUNOUT_MM:.1f}mm). Infeasible."
            )
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

    # -------------------------------------------------------- リブ(内角を橋渡しする三角形2枚)

    def _build_rib_part(self, plan, w: Vec3, ey: Vec3, rib: RibParams,
                        half_width_mm: float, bend_radius_mm: float, faces) -> None:
        """リブ部品は断面掃引では作れないので、面を明示的に張る。

        パネルの境界が曲げ線のところで **V 字**(V1 -> A -> V2)になり、幅方向に
        一定でないため。パネルは平面なので多角形1枚で張れる。曲げは:

          |y| >= c+t : 通常の円筒フィレット(厳密)
          |y| in [c, c+t] : 円弧から鋭角へ落ちる遷移(頂点V へのロフト)
          |y| <  c   : フィレットせず、リブの三角形 V1AB / V2AB に置き換わる

        (ユーザー指定 2026-09-04: 基準面をシャープなまま作ってリブを置き、そのあと
         リブ幅の外側だけを最小Rでフィレットする、という工程の帰結)
        """
        frames, tangents = plan.panel_frames, plan.fold_tangents
        width = half_width_mm
        # sharp = リブのフットプリント外側に残すシャープな折れ、taper = その外の遷移帯。
        c, sharp, taper = rib.half_width_mm, rib.sharp_margin_mm, rib.taper_mm
        outer = c + sharp                      # ここまでシャープ / ここから遷移
        target = min(rib.fold_index, len(frames) - 2)

        def pt(index: int, run: float, y: float) -> Vec3:
            frame = frames[index]
            return _add(frame.origin, _scale(frame.u, run), _scale(ey, y))

        # 接線長は**自分で測った折れ角から**出す。`plan.fold_tangents` は法線どうしの
        # 角度で計算されており、ここで使う「wまわりの符号付き角」と0.2度ほどずれる。
        # R=5mmで94度回すと0.8mmの食い違いになり、遷移ロフトが接続しなくなる
        # (2026-09-04に実測)。自前で閉じたほうが構成的に正しい。
        folds = []
        cuts = []
        for k in range(len(frames) - 1):
            normal = _normalize(_cross(frames[k].u, frames[k].v))
            phi = _signed_angle_about(frames[k].u, frames[k + 1].u, w)
            cut = bend_radius_mm * abs(math.tan(phi / 2.0))
            fold_run = frames[k].far_run_mm
            centre = _add(pt(k, fold_run - cut, 0.0),
                          _scale(normal, -(1.0 if phi > 0 else -1.0) * bend_radius_mm))
            folds.append({"phi": phi, "run": fold_run, "cut": cut, "centre": centre})
            cuts.append(cut)

        # --- パネル(平面多角形) ---
        for k, frame in enumerate(frames):
            near = self._panel_edge(k, frames, cuts, rib, target, width, pt, near=True)
            far = self._panel_edge(k, frames, cuts, rib, target, width, pt, near=False)
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
            axis = gp_Ax1(gp_Pnt(*fold["centre"]), gp_Dir(*w))
            spans = ([(-width, -(outer + taper)), (outer + taper, width)] if k == target
                     else [(-width, width)])
            for index, (y0, y1) in enumerate(spans):
                edge = BRepBuilderAPI_MakeEdge(
                    gp_Pnt(*pt(k, fold["run"] - fold["cut"], y0)),
                    gp_Pnt(*pt(k, fold["run"] - fold["cut"], y1))).Edge()
                patch = BRepPrimAPI_MakeRevol(edge, axis, fold["phi"]).Shape()
                faces[topods.Face(patch)] = f"bend_{k}_{index}"
            if k != target:
                continue

            # 遷移: y=±(c+taper) の円弧から、y=±c の鋭角の点 V へ落とす。
            for side in (-1, 1):
                y_arc = side * (outer + taper)
                start_pt = pt(k, fold["run"] - fold["cut"], y_arc)
                mid_pt = _rotate_about(start_pt, fold["centre"], w, fold["phi"] / 2.0)
                end_pt = _rotate_about(start_pt, fold["centre"], w, fold["phi"])
                arc = GC_MakeArcOfCircle(gp_Pnt(*start_pt), gp_Pnt(*mid_pt), gp_Pnt(*end_pt)).Value()
                loft = BRepOffsetAPI_ThruSections(False, True)
                loft.AddVertex(BRepBuilderAPI_MakeVertex(
                    gp_Pnt(*pt(k, fold["run"], side * outer))).Vertex())
                loft.AddWire(BRepBuilderAPI_MakeWire(
                    BRepBuilderAPI_MakeEdge(arc).Edge()).Wire())
                loft.Build()
                if not loft.IsDone():
                    raise ValueError("the rib fillet taper failed")
                explorer = TopExp_Explorer(loft.Shape(), TopAbs_FACE)
                while explorer.More():
                    faces[topods.Face(explorer.Current())] = \
                        f"rib_taper_{'p' if side > 0 else 'm'}"
                    explorer.Next()

            # リブ本体: 四面体 V1-V2-A-B のうち中立面に出る2面。
            v1 = pt(k, fold["run"], -c)
            v2 = pt(k, fold["run"], c)
            apex_a = pt(k, fold["run"] - rib.leg1_mm, 0.0)
            # **注意**: 折れ目の位置はパネル k+1 の run=0 とは限らない。自由折れ目
            # チェーンでは origin が折れ目そのものだが、単曲げ(single_fold_layout)では
            # origin が締結点にあり折れ目は near_run_mm にある(ここを取り違えて
            # パネルが巨大なスリバーになった。2026-09-04に実測)。
            base_next = frames[k + 1].near_run_mm
            apex_b = pt(k + 1, base_next + rib.leg2_mm, 0.0)
            for name, corner in (("rib_l", v1), ("rib_r", v2)):
                wire = BRepBuilderAPI_MakeWire(
                    BRepBuilderAPI_MakeEdge(gp_Pnt(*corner), gp_Pnt(*apex_a)).Edge(),
                    BRepBuilderAPI_MakeEdge(gp_Pnt(*apex_a), gp_Pnt(*apex_b)).Edge(),
                    BRepBuilderAPI_MakeEdge(gp_Pnt(*apex_b), gp_Pnt(*corner)).Edge()).Wire()
                faces[topods.Face(BRepBuilderAPI_MakeFace(wire, True).Face())] = name

    @staticmethod
    def _panel_edge(k, frames, cuts, rib: RibParams, target: int,
                    width: float, pt, *, near: bool) -> list:
        """パネル k の手前側/奥側の境界を y 昇順の点列で返す。

        リブが載る曲げに面する側だけ V 字(タンジェント線 -> V1 -> 頂点 -> V2 -> タンジェント線)。
        """
        c, taper = rib.half_width_mm, rib.taper_mm
        outer = c + rib.sharp_margin_mm
        if near:
            if k == 0:
                run = frames[0].near_run_mm
                return [pt(0, run, -width), pt(0, run, width)]
            # 折れ目はパネルkの near_run_mm の位置(単曲げでは0ではない)。
            base = frames[k].near_run_mm
            cut = base + cuts[k - 1]
            if k - 1 != target:
                return [pt(k, cut, -width), pt(k, cut, width)]
            return [pt(k, cut, -width), pt(k, cut, -(outer + taper)), pt(k, base, -outer),
                    pt(k, base, -c), pt(k, base + rib.leg2_mm, 0.0), pt(k, base, c),
                    pt(k, base, outer), pt(k, cut, outer + taper), pt(k, cut, width)]
        if k == len(frames) - 1:
            run = frames[k].far_run_mm
            return [pt(k, run, -width), pt(k, run, width)]
        fold_run, cut = frames[k].far_run_mm, cuts[k]
        if k != target:
            return [pt(k, fold_run - cut, -width), pt(k, fold_run - cut, width)]
        return [pt(k, fold_run - cut, -width), pt(k, fold_run - cut, -(outer + taper)),
                pt(k, fold_run, -outer), pt(k, fold_run, -c),
                pt(k, fold_run - rib.leg1_mm, 0.0), pt(k, fold_run, c),
                pt(k, fold_run, outer), pt(k, fold_run - cut, outer + taper),
                pt(k, fold_run - cut, width)]

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
    def _bead_lift(path, bead: BeadParams, bend_radius_mm: float) -> int:
        """ビードの立ち上げ向き(+1 = パネル法線側)。曲げ上ではビード頂部の半径が
        R + sign(折れ角)*lift*深さ になるので、最小半径が最大になる向きを選ぶ。
        それでも中立面R最小を割るならInfeasible。"""
        bends = [s for s in path if isinstance(s, _Bend)]
        best, best_radius = 1, -1e9
        for lift in (1, -1):
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
                             *, half_width_mm: float, radius_mm: float, exclude_side):
        """両端の隅を R=締結点の必要最小半径 で丸めて落とす(実務の余肉カット)。
        フランジ部品はフランジ側の2隅を残す。"""
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
                if exclude_side is not None and side == exclude_side:
                    continue
                tools.append(self._relief_tool(frm, inward, side, half_width_mm, radius_mm))
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

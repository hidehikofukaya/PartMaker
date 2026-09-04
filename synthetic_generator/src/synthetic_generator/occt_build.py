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

from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_Transform,
)
from OCC.Core.BRepFill import brepfill
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeRevol
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Core.Interface import Interface_Static
from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
from OCC.Core.TDataStd import TDataStd_Name
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SHELL
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods
from OCC.Core.XCAFApp import XCAFApp_Application
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool
from OCC.Core.GProp import GProp_GProps
from OCC.Core.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

from synthetic_generator.bead import BeadParams
from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM, FasteningPoint, Vec3
from synthetic_generator.flange import FLANGE_CONCAVE_CLEARANCE_MM, FlangeParams
from synthetic_generator.general_geometry import plan_general_two_point

SEW_TOLERANCE_MM = 0.01
# ランアウト長(平地->ビード断面の遷移)。実物のビードの走り終いは深さの2〜3倍程度。
RUNOUT_DEPTH_RATIO = 2.0
MIN_RUNOUT_MM = 3.0
# 隅の余肉カットのツール角柱の張り出し(基準面の両側へこれだけ伸ばす)。カット域の
# 基準面は厳密に平面なので薄くてよい。厚くするとU字部品で反対側のパネルまで削る。
CUT_TOOL_HALF_DEPTH_MM = 5.0


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


def _bead_section(bead: BeadParams, lift: int, half_width_mm: float) -> tuple[list[Elem], list[float]]:
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
         p(-(yf - sb * cos_t), sb * sin_t), "bead_foot_l"),
        ("line", p(-(yf - sb * cos_t), sb * sin_t),
         p(-(yt + sb * cos_t), depth - sb * sin_t), "bead_wall_l"),
        ("arc", p(-(yt + sb * cos_t), depth - sb * sin_t), p(-top_mid[0], top_mid[1]),
         p(-(yt - sb), depth), "bead_ridge_l"),
        ("line", p(-(yt - sb), depth), p(yt - sb, depth), "bead_top"),
        ("arc", p(yt - sb, depth), p(top_mid[0], top_mid[1]),
         p(yt + sb * cos_t, depth - sb * sin_t), "bead_ridge_r"),
        ("line", p(yt + sb * cos_t, depth - sb * sin_t),
         p(yf - sb * cos_t, sb * sin_t), "bead_wall_r"),
        ("arc", p(yf - sb * cos_t, sb * sin_t), p(foot_mid[0], foot_mid[1]),
         p(yf + sb, 0.0), "bead_foot_r"),
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

    # 折れ目軸(解析式)。全ての折れ目で共通のはず。
    axes = []
    for k in range(len(frames) - 1):
        a = tilts[k][1]
        axes.append(_normalize(_add(_scale(frames[k].v, math.cos(a)), _scale(frames[k].u, math.sin(a)))))
    w = axes[0]
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
        out_dir: str,
        part_name: str,
        bead: BeadParams | None = None,
        flange: FlangeParams | None = None,
    ) -> GeneratedPart:
        if bead is not None and flange is not None:
            raise ValueError("a part takes either a bead or a flange, not both")
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
        else:
            section = (_flange_section(flange, half_width_mm) if flange is not None
                       else _flat_section([-half_width_mm, half_width_mm]))
            if flange is not None:
                self._check_flange_radii(path, flange, bend_radius_mm)
            self._sweep_uniform(path, frame, w, section, faces)

        shape, faces = self._sew(faces)
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
        total = sum(s.length if isinstance(s, _Straight) else abs(s.angle) * s.radius for s in path)
        inset = 2.0 * min_bearing_radius_mm
        first, last = path[0], path[-1]
        if not isinstance(first, _Straight) or not isinstance(last, _Straight):
            raise ValueError("path must start and end with a straight segment")
        # ランアウトは平坦区間の中に収める必要がある(曲げの上で断面を変えると
        # 織り面が曲げ円筒に接せず折れる)。空きが足りなければ短くする。
        room = min(first.length - inset, last.length - inset) - 1.0
        runout = min(RUNOUT_DEPTH_RATIO * bead.depth_mm, room)
        if runout < MIN_RUNOUT_MM:
            raise ValueError(
                f"only {room:.1f}mm of flat run is left for the bead run-out "
                f"(need >= {MIN_RUNOUT_MM:.1f}mm). Infeasible; not attempting construction."
            )
        s0, s1 = inset, inset + runout
        s3, s2 = total - inset, total - inset - runout
        if s2 - s1 < 5.0:
            raise ValueError(
                f"bead run ({s2 - s1:.1f}mm of full section) is too short between the "
                f"{runout:.1f}mm run-outs. Infeasible; not attempting construction."
            )

        if s1 > first.length - 1.0:
            raise ValueError(
                f"the start run-out ends at {s1:.1f}mm but panel 0's flat run is only "
                f"{first.length:.1f}mm. Infeasible; not attempting construction."
            )
        if s2 < total - last.length + 1.0:
            raise ValueError(
                f"the end run-out starts at {s2:.1f}mm, inside a bend fillet "
                f"(last flat run starts at {total - last.length:.1f}mm). "
                "Infeasible; not attempting construction."
            )

        cursor = 0.0
        for step in path:
            tag = f"panel_{step.panel}" if isinstance(step, _Straight) else f"bend_{step.fold}"
            if isinstance(step, _Bend):
                frame = self._sweep_segment(step, frame, w, bead_section, faces, tag)
                cursor += abs(step.angle) * step.radius
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

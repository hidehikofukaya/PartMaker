"""GSD(サーフェス)で中立面を直接構築する低レベルCATIA V5 Automationラッパー。

実機(CATIA V5-6, D:\\Dassault Systemes\\B32)で動作確認済み。`build_parallel_same_offset`
はジョグ(平面->傾斜ランプ->平面。ランプの傾き・折れ目Rは可変)+両端フランジの
STP/CATPart出力まで通しで成功している。
`export_stp`は中立面のみのクリーンな結果をCATPartとして保存(SaveAs)した上でSTP出力する。
作業履歴が残った方のドキュメント(`doc`)はCATPart保存せず閉じている。

現在の構築方式(2026-08-06、SS6.7参照): flat1/ランプ/flat2をシャープな角のまま先に
完全結合し、結合済みシェルの実エッジに`part.ShapeFactory.AddNewSurfaceEdgeFilletWithConstantRadius`
を順番に適用してRを付ける。旧方式(AddNewFilletBiTangentで未結合サーフェスを先に
橋渡ししてからJoin)はRが大きいとブリッジ面が実際のパネル残余部分と接触せず「浮いた」
状態になり、Update()は成功するのに形状が繋がっていない不具合があった(ユーザー報告)。
新方式はエッジフィレットが既に位相的に正しいシェルの実エッジを丸めるだけなので、
浮いた面が原理的に発生しない(25部品バッチで成功した全件が単一OPEN_SHELLと確認済み)。

実機検証で判明した非自明な点:

  - AddNewXxxが作る形状はHybridBody.AppendHybridShape()で明示的にツリーへ登録しないと、
    part.Update()が成功してもExportDataが空の(サーフェスエンティティなし)STPを吐く。
    例外は出ない。既存catia_midsurface.pyのパターンをそのまま踏襲する必要がある。
  - AddNewSweepLineは、SetFirstLengthDefinitionType/SetLength/SetAngle、および
    SetFirstLengthLaw/SetAngularLaw(lawType 0/1/2/3全て試行)のいずれの組み合わせでも
    Update失敗した(configなしのデフォルトでも失敗)。フランジは平坦(平面)なので、
    そもそもSweepLineの角度法則機構は不要 — フランジ4隅の座標を解析的に計算して
    rect_fill(既に検証済みの点/線/Fillパターン)で作る方が単純かつ確実だった。
    非90度の斜めフランジも、法線と接線ベクトルを角度でブレンドする式(flat_flange参照)
    でrect_fillのまま対応できたため、SweepLineは結局不要になった。曲面プロファイルの
    スイープが本当に必要になる場合(ビード等)のみ再調査対象として残す。
  - (履歴/現在は不使用) AddNewFilletBiTangentで未結合の2サーフェスを先に橋渡ししていた
    旧方式では、iOrientation1/iOrientation2が隣接する2つの曲げで符号反転する
    (1曲げ目-1,-1、2曲げ目1,1でのみUpdate成功)、iSupportsTrimModeは1が必須、
    7要素の単純foldはUpdate失敗し隣接ペアを先にグループ化するJoin木構造が必要、
    という知見があった。エッジフィレット方式への切り替えでこれらの制約(と「Rが浮く」
    不具合そのもの)は解消されたため現在のコードは使っていないが、GSDのJoin/Fillet系
    APIの一般的な癖として記録しておく。
  - AddNewSurfaceEdgeFilletWithConstantRadius(part.ShapeFactory、MecMod/PartInterfaces系
    でGSDのHybridShapeFactoryとは別系統)の対象エッジは、Selection.Search("Topology.CGMEdge,sel")
    で列挙したSelection.Item2(i)の`.Value`を`part.CreateReferenceFromObject`に渡すと
    エラーになる。`.Reference`プロパティが直接有効なReferenceを返すのでそちらを使う。
"""

from __future__ import annotations

import dataclasses
import math
import os

import pythoncom
from win32com.client import Dispatch

from synthetic_generator.bead import BeadParams, bead_cells, bead_fits
from synthetic_generator.classify import (
    MIN_NEUTRAL_PLANE_RADIUS_MM,
    MIN_PANEL_CLEARANCE_MM,
    FasteningPoint,
    Vec3,
    end_panel_corners,
    flat_panels_clearance_mm,
    fold_tangent_length_mm,
    ramp_fold_angles_rad,
    single_fold_layout,
    tangent_length_for_bend_angle_rad,
    two_point_frame,
)
from synthetic_generator.reinforcement import ReinforcementParams
from synthetic_generator.templates.parallel_same_offset import TwoJointSpec


def r8_array(n: int):
    """CATIA の byref double 配列引数用 VARIANT(既存 catia_midsurface.py の定石を再利用)。"""
    from win32com.client import VARIANT

    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8 | pythoncom.VT_BYREF, [0.0] * n)


@dataclasses.dataclass(frozen=True)
class GeneratedPart:
    stp_path: str
    catpart_path: str


# ---------------------------------------------------------------- ジョグ幾何(純粋関数、CATIA非依存)


@dataclasses.dataclass(frozen=True)
class _JogFrame:
    normal: Vec3
    axis_dir: Vec3  # point1->point2 の接平面内成分の単位ベクトル(ジョグの走行方向)
    width_dir: Vec3  # normal x axis_dir
    run_length_mm: float
    offset_mm: float  # ジョグの高さ(締結軸方向のオフセット)


def _normalize(v: Vec3) -> Vec3:
    length = math.sqrt(sum(c * c for c in v))
    return (v[0] / length, v[1] / length, v[2] / length)


def _dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _jog_frame(point1: FasteningPoint, point2: FasteningPoint) -> _JogFrame:
    n = _normalize(point1.normal_xyz)
    delta = _sub(point2.position_xyz, point1.position_xyz)
    offset = _dot(delta, n)
    tangential = tuple(delta[i] - offset * n[i] for i in range(3))
    run_length = math.sqrt(sum(c * c for c in tangential))
    if run_length < 1e-6:
        raise ValueError("point1/point2 must be laterally separated to build a jog panel")
    axis_dir = tuple(c / run_length for c in tangential)
    width_dir = _normalize(_cross(n, axis_dir))
    return _JogFrame(normal=n, axis_dir=axis_dir, width_dir=width_dir, run_length_mm=run_length, offset_mm=offset)


def _panel_corners(
    origin: Vec3, frame: _JogFrame, run0: float, run1: float, height0: float, height1: float, half_width: float
) -> list[Vec3]:
    """run0側とrun1側で異なる高さ(height0/height1)を許すことで、平坦パネル
    (height0==height1)と傾斜ランプ(height0!=height1)の両方をこの1関数で表す。
    run0==run1かつheight0!=height1のときは旧来の垂直リザー(幅ゼロの壁)に一致する。
    """

    def pt(run: float, width: float, height: float) -> Vec3:
        return tuple(
            origin[i] + run * frame.axis_dir[i] + width * frame.width_dir[i] + height * frame.normal[i]
            for i in range(3)
        )

    return [pt(run0, -half_width, height0), pt(run0, half_width, height0), pt(run1, half_width, height1), pt(run1, -half_width, height1)]


# ---------------------------------------------------------------- Phase 1.5 余肉削減(円弧トリム、純粋関数)


@dataclasses.dataclass(frozen=True)
class _TangentArcBoundary:
    """締結点周りの必要最小半径Rの円と、遠端の直線境界(2点)の凸包境界。

    境界は4本の閉ループ: line(corner_a, corner_b) -> line(corner_b, tangent_b)
    -> arc(tangent_b -> tangent_a, sweep_deg分CCW) -> line(tangent_a, corner_a)。
    """

    corner_a: Vec3
    corner_b: Vec3
    tangent_a: Vec3
    tangent_b: Vec3
    sweep_deg: float


def _tangent_arc_boundary(
    origin: Vec3, axis_dir: Vec3, width_dir: Vec3, far_distance: float, half_width: float, radius: float
) -> _TangentArcBoundary:
    """締結点(origin)を中心とした半径radiusの円と、originからaxis_dir方向に
    far_distance、幅方向に±half_widthの2点(corner_a/corner_b)との凸包境界を計算する。

    円から各コーナーへの外接線の接点(tangent_a/tangent_b)を解析的に求める
    (標準的な「点から円への接線」公式: 接点への角度は、円中心からコーナーへの角度から
    acos(radius/distance)だけずれる)。2点はorigin基準で対称(±half_width)なので
    tangent_a/tangent_bも対称になる。

    sweep_degは、tangent_bを角度0とした場合にtangent_aへ到達するまでのCCW(+normal側から
    見て反時計回り)掃引角度[度]。これが遠端(corner_a/corner_b)側を通らない「奥側」
    (円のorigin<-axis_dir側)を通る円弧になることは、tangent_a/tangent_bが対称に
    origin中心±K度(K=atan2(half_width,far_distance)+acos(radius/distance))の位置にある
    ことから、sweep=(360-2K) mod 360として導出される(実機の目視・測定検証で確認済み、
    docs/synthetic_two_joint_generation_roadmap.md 参照)。
    """
    distance = math.hypot(far_distance, half_width)
    if distance <= radius:
        raise ValueError(
            f"far_distance={far_distance:.2f}, half_width={half_width:.2f} give a corner distance "
            f"{distance:.2f} that does not exceed radius={radius:.2f} -- no tangent line exists. "
            "Infeasible; not attempting construction."
        )
    phi = math.acos(radius / distance)
    angle_to_corner_b = math.atan2(half_width, far_distance)
    theta_b = angle_to_corner_b + phi
    theta_a = -theta_b

    def pt(u: float, v: float) -> Vec3:
        return tuple(origin[i] + u * axis_dir[i] + v * width_dir[i] for i in range(3))

    corner_a = pt(far_distance, -half_width)
    corner_b = pt(far_distance, half_width)
    tangent_a = pt(radius * math.cos(theta_a), radius * math.sin(theta_a))
    tangent_b = pt(radius * math.cos(theta_b), radius * math.sin(theta_b))
    sweep_deg = math.degrees(theta_a - theta_b) % 360.0

    return _TangentArcBoundary(corner_a=corner_a, corner_b=corner_b, tangent_a=tangent_a, tangent_b=tangent_b, sweep_deg=sweep_deg)


class SyntheticPartBuilder:
    """新規CATPartドキュメントに対してGSDサーフェスを構築する薄いラッパー。

    既存の中立面抽出(catia_midsurface.py)と異なり、ソリッドを経由しない
    (docs/synthetic_two_joint_generation_roadmap.md SS5参照)。
    """

    def __init__(self) -> None:
        self.catia = Dispatch("CATIA.Application")
        self.catia.DisplayFileAlerts = False

    def new_part_document(self):
        return self.catia.Documents.Add("Part")

    # ------------------------------------------------------------ 検証済みプリミティブ

    def point(self, hsf, body, x_mm: float, y_mm: float, z_mm: float):
        """AddNewPointCoord: 座標指定の点を作成する。作成直後にbodyへ登録しないと
        ドキュメントツリーに反映されず、後続のExportDataが空になる(既存
        catia_midsurface.pyのAppendHybridShape+Updateパターンを踏襲)。"""
        shape = hsf.AddNewPointCoord(x_mm, y_mm, z_mm)
        body.AppendHybridShape(shape)
        return shape

    def line_pt_pt(self, hsf, part, body, point1, point2):
        """AddNewLinePtPt: 2点間の直線を作成する。"""
        ref1 = part.CreateReferenceFromObject(point1)
        ref2 = part.CreateReferenceFromObject(point2)
        shape = hsf.AddNewLinePtPt(ref1, ref2)
        body.AppendHybridShape(shape)
        return shape

    def fill(self, hsf, part, body, boundary_curves: list):
        """AddNewFill + AddBound: 閉じた境界曲線群からパッチを生成する。

        境界曲線群は1つの閉ループを構成している必要がある(呼び出し側の責務)。
        """
        fill_obj = hsf.AddNewFill()
        for curve in boundary_curves:
            ref = part.CreateReferenceFromObject(curve)
            fill_obj.AddBound(ref)
        body.AppendHybridShape(fill_obj)
        return fill_obj

    def rect_fill(self, hsf, part, body, corners: list[Vec3]):
        """4隅の座標(閉ループ順)から矩形パッチを1枚作る(point*4 -> line*4 -> fill)。"""
        points = [self.point(hsf, body, *c) for c in corners]
        n = len(points)
        lines = [self.line_pt_pt(hsf, part, body, points[i], points[(i + 1) % n]) for i in range(n)]
        return self.fill(hsf, part, body, lines)

    # ------------------------------------------------------------ Phase 1.5 余肉削減(円弧トリム、2026-08-07追加)
    #
    # AddNewCircleCtrPtWithAngles(iCenter, iCrossingPoint, iSupport, iGeodesic, iStartAngle,
    # iEndAngle)を実機グリッドサーチで検証した結果: (1) iStartAngle/iEndAngleは度単位
    # (ラジアンではない、AddNewCircleCenterAxisWithAngles等の他API疑いとは異なる)、
    # (2) iCrossingPointが実際の角度0の基準方向になる(iSupportの絶対座標系に依存しない
    # ため、任意平面の向きでもロバスト — CATIAの平面ローカル座標系の規約を推測する必要が
    # ない)、(3) 角度は+normal側から見て反時計回り(CCW)が正、を実測(GetMeasurable().Length
    # と既知点までのGetMinimumDistanceで確認)。AddNewCircleCenterAxisWithAngles(サポート面
    # 不要、軸線のみ)も試したが角度パラメータが無視されて常に全周円になったため不採用。

    def trimmed_end_panel(
        self,
        hsf,
        part,
        body,
        origin: Vec3,
        normal: Vec3,
        axis_dir: Vec3,
        width_dir: Vec3,
        far_distance: float,
        half_width: float,
        radius: float,
    ):
        """締結点(origin)周りの必要最小半径radiusの円と、遠端(originからaxis_dir方向に
        far_distance、幅方向±half_widthの2点)との凸包(接線2本+円弧+直線)でパネルを作る
        (docs/synthetic_two_joint_generation_roadmap.md SS6.3の「余肉削減」原則の実装)。
        """
        b = _tangent_arc_boundary(origin, axis_dir, width_dir, far_distance, half_width, radius)

        center = self.point(hsf, body, *origin)
        axis_pt = self.point(hsf, body, *(tuple(origin[i] + normal[i] for i in range(3))))
        axis_line = self.line_pt_pt(hsf, part, body, center, axis_pt)
        plane = hsf.AddNewPlaneNormal(
            part.CreateReferenceFromObject(axis_line), part.CreateReferenceFromObject(center)
        )
        body.AppendHybridShape(plane)

        center_ref = part.CreateReferenceFromObject(center)
        plane_ref = part.CreateReferenceFromObject(plane)
        crossing = self.point(hsf, body, *b.tangent_b)
        crossing_ref = part.CreateReferenceFromObject(crossing)
        arc = hsf.AddNewCircleCtrPtWithAngles(center_ref, crossing_ref, plane_ref, False, 0.0, b.sweep_deg)
        body.AppendHybridShape(arc)

        pa = self.point(hsf, body, *b.corner_a)
        pb = self.point(hsf, body, *b.corner_b)
        pta = self.point(hsf, body, *b.tangent_a)
        line_ab = self.line_pt_pt(hsf, part, body, pa, pb)
        line_b_tb = self.line_pt_pt(hsf, part, body, pb, crossing)
        line_ta_a = self.line_pt_pt(hsf, part, body, pta, pa)
        return self.fill(hsf, part, body, [line_ab, line_b_tb, arc, line_ta_a])

    def join(self, hsf, part, body, elements: list):
        """AddNewJoin: 複数サーフェス/曲線を1つに結合する(2引数APIをペアワイズに畳み込む)。"""
        if len(elements) < 2:
            raise ValueError("join requires at least 2 elements")
        refs = [part.CreateReferenceFromObject(e) for e in elements]
        result = hsf.AddNewJoin(refs[0], refs[1])
        body.AppendHybridShape(result)
        for ref in refs[2:]:
            result_ref = part.CreateReferenceFromObject(result)
            result = hsf.AddNewJoin(result_ref, ref)
            body.AppendHybridShape(result)
        return result

    def _discard_failed_attempt(self, out_dir: str, basename: str, *docs) -> None:
        """アボートした試行の後片付け: 渡されたドキュメントを保存せず閉じ、万一
        部分的に書き出されていた出力ファイルがあれば削除する。

        ユーザー指摘(2026-08-06): アボートした部品がそのままCATPartとして残ると、
        学習材料を選別する際の邪魔になる。呼び出し側は例外を再送出すること
        (アボート自体は握りつぶさない — 「失敗したらアボートでいい」の方針のまま)。
        """
        for doc in docs:
            try:
                doc.Close()
            except Exception:
                pass  # 既に閉じている/壊れている場合もあるためベストエフォート
        for suffix in ("_mid.CATPart", "_mid.stp"):
            path = os.path.join(out_dir, basename + suffix)
            if os.path.exists(path):
                os.remove(path)

    def _paste_result(self, doc, surface, max_attempts: int = 3):
        """selをコピーし、新規PartへAs Result(CATPrtResult)コピーする。ペースト結果が
        空でないことを検証し、空ならリトライする。

        実機で発見(2026-08-06): SelectionのCopy/PasteSpecialはWindowsクリップボード
        経由のため、連続バッチ実行時に稀に空の結果が貼り付けられることがあった
        (30部品バッチで4件再現、単体で再実行すると正常だったため幾何固有のバグではなく
        クリップボードのタイミング起因と判断)。part.Update()は成功しExportDataも例外を
        出さないため、事後のファイル内容確認以外では検知できない — ここでFace検索で
        非空を確認してから先に進む。
        """
        for attempt in range(max_attempts):
            sel = doc.Selection
            sel.Clear()
            sel.Add(surface)
            sel.Copy()

            new_doc = self.catia.Documents.Add("Part")
            s2 = new_doc.Selection
            s2.Clear()
            s2.Add(new_doc.Part)
            s2.PasteSpecial("CATPrtResult")
            new_doc.Part.Update()

            s2.Clear()
            s2.Add(new_doc.Part)
            s2.Search("Topology.CGMFace,sel")
            is_empty = s2.Count2 == 0
            s2.Clear()
            if not is_empty:
                return new_doc
            new_doc.Close()

        raise RuntimeError(f"PasteSpecial produced empty geometry after {max_attempts} attempts")

    def export_stp(self, doc, surface, out_dir: str, basename: str) -> GeneratedPart:
        """中立面のみを新規CATPartへAs Resultコピーし、CATPartとして保存した上でstp出力する
        (catia_midsurface.pyと同じAs Resultコピーの定石+SaveAsを追加)。

        `doc`(点/線/フィレット/Joinの作業履歴が残ったドキュメント)はCATPartとしては
        保存せず閉じる — 保存するのは中立面のみのクリーンな結果の方。作業履歴側も
        必要になったら別途保存するよう拡張できる。

        途中で失敗した場合はnew_doc/docの両方を保存せず閉じ、部分的に書き出された
        ファイルがあれば削除してから例外を再送出する。
        """
        os.makedirs(out_dir, exist_ok=True)
        catpart_path = os.path.abspath(os.path.join(out_dir, basename + "_mid.CATPart"))
        stp_path = os.path.abspath(os.path.join(out_dir, basename + "_mid.stp"))

        try:
            new_doc = self._paste_result(doc, surface)
        except Exception:
            self._discard_failed_attempt(out_dir, basename, doc)
            raise

        try:
            new_doc.SaveAs(catpart_path)
            new_doc.ExportData(stp_path, "stp")
        except Exception:
            self._discard_failed_attempt(out_dir, basename, new_doc, doc)
            raise

        new_doc.Close()
        doc.Close()
        return GeneratedPart(stp_path=stp_path, catpart_path=catpart_path)

    # ------------------------------------------------------------ フランジ(角度可変、rect_fillベース)

    def flat_flange(
        self,
        hsf,
        part,
        body,
        guide_point_a: Vec3,
        guide_point_b: Vec3,
        normal: Vec3,
        outward_tangent: Vec3,
        height_mm: float,
        angle_deg: float = 90.0,
    ):
        """境界直線エッジ(guide_point_a -> guide_point_b)からフランジ面を立ち上げる。

        angle_degはベースパネル面を基準にした角度(90度=法線方向へ垂直に立てる、
        既定かつ主要ケース)。90度以外は、normalとoutward_tangent(ベースパネル面内で
        フランジが立つ側へ向く単位ベクトル)を角度に応じてブレンドする:
          direction = sin(angle)*normal + cos(angle)*outward_tangent
        90度ではsin=1,cos=0となり、旧来のnormal方向のみの計算と完全に一致する。

        SweepLineが実機で全てUpdate失敗したため、既に検証済みのrect_fillで代用している
        (モジュールdocstring参照)。斜めフランジ時に実務でよく作られる「戻りフランジ」
        (先端に一般面と平行なもう一つのフランジを追加する構造)はまだ未実装 — 現状は
        角度がついた単純な片フランジのみを生成する(次の増分)。
        """
        angle_rad = math.radians(angle_deg)
        s, c = math.sin(angle_rad), math.cos(angle_rad)
        direction = tuple(s * normal[i] + c * outward_tangent[i] for i in range(3))
        corners = [
            guide_point_a,
            guide_point_b,
            tuple(guide_point_b[i] + height_mm * direction[i] for i in range(3)),
            tuple(guide_point_a[i] + height_mm * direction[i] for i in range(3)),
        ]
        return self.rect_fill(hsf, part, body, corners)

    # ------------------------------------------------------------ エッジフィレット(2026-08-06追加)
    #
    # AddNewFilletBiTangent(2枚の未結合サーフェスをRで橋渡しする方式)は、Rを大きくすると
    # ブリッジ面が実際の(トリムされた)パネル残余部分と接触せず「浮いた」結果になり、
    # Update()自体は成功するのに形状として繋がっていないケースがあった(ユーザー指摘)。
    #
    # 代わりに: 1) シャープな角のまま先に完全結合(平面同士の結合はR不要で常に確実)、
    # 2) その結合済みシェルの実際のエッジに対しAddNewSurfaceEdgeFilletWithConstantRadius
    # (part.ShapeFactory、MecMod/PartInterfacesの機能。GSDのHybridShapeFactoryとは別系統)
    # を順番に適用する方式に変更した。これは常にトポロジ的に正しい(既に繋がっている
    # シェルの実エッジを丸めるだけなので、浮いた面が生じない)。目的のエッジは、解析的に
    # 計算済みの折れ目中点への最短距離でSelection.Search結果から特定する
    # (Selection.Item2(i).Referenceが有効なReferenceを返す — part.CreateReferenceFromObject
    # をエッジオブジェクトに直接使うとエラーになったため、この方法を使うこと)。

    def find_edge_near(self, doc, part, spa, hsf, body, surface_obj, target_point: Vec3):
        """surface_objのエッジのうち、target_point(3D座標)に最も近いものへのReferenceを返す。

        Selection.Item2(i).Value を part.CreateReferenceFromObject に渡すとエラーになった
        (エッジオブジェクトは直接変換できない)。Selection.Item2(i).Reference が有効な
        Referenceを直接返すので、それを使う。
        """
        sel = doc.Selection
        sel.Clear()
        sel.Add(surface_obj)
        sel.Search("Topology.CGMEdge,sel")

        probe = self.point(hsf, body, *target_point)
        part.Update()
        probe_ref = part.CreateReferenceFromObject(probe)

        best_ref, best_dist = None, None
        for i in range(1, sel.Count2 + 1):
            item = sel.Item2(i)
            edge_meas = spa.GetMeasurable(item.Reference)
            d = edge_meas.GetMinimumDistance(probe_ref)
            if best_dist is None or d < best_dist:
                best_ref, best_dist = item.Reference, d
        return best_ref

    def edge_fillet(self, part, edge_ref, radius_mm: float, *, propagation_mode: int = 0):
        """AddNewSurfaceEdgeFilletWithConstantRadius(iEdgeToFillet, iPropagMode, iRadius)。
        iPropagMode=0(隣接エッジへの伝播なし、この1エッジのみ)で動作確認済み。"""
        return part.ShapeFactory.AddNewSurfaceEdgeFilletWithConstantRadius(edge_ref, propagation_mode, radius_mm)

    def edge_fillet_group(self, part, edge_refs: list, radius_mm: float):
        """複数エッジを1つのEdgeFilletフィーチャーにまとめて当てる(AddObjectToFillet)。

        2026-08-21(roadmap SS6.25)の実機検証: ビードが曲げをまたぐと、ビードの縦フィレット列と
        メイン折れ目のフィレット列が交差する。1エッジずつ順に当てると、**先にどちらを当てても
        後から当てた側が交差点で必ずUpdate失敗する**(順序・半径・横方向オフセットを変えても
        再現、SS6.15の「角の共有」の一段上の問題)。全エッジを1フィーチャーにまとめると、
        CATIA側が交差部のコーナーブレンドを一括で解くため成功する。

        エッジ1本だけを渡した場合は`edge_fillet`と完全に同じ結果になる。
        """
        if not edge_refs:
            raise ValueError("edge_fillet_group requires at least 1 edge")
        fillet = self.edge_fillet(part, edge_refs[0], radius_mm)
        for edge_ref in edge_refs[1:]:
            fillet.AddObjectToFillet(edge_ref)
        return fillet

    # ------------------------------------------------------------ Phase 1 オーケストレーション

    def build_parallel_same_offset(
        self,
        spec: TwoJointSpec,
        reinforcement: ReinforcementParams,
        out_dir: str,
        part_name: str,
        *,
        include_flanges: bool = False,
    ) -> GeneratedPart:
        """ベースパネル(平面->傾斜ランプ->平面のジョグ)+ 両端フランジ(任意)を構築する。

        方針転換(2026-08-07、ユーザー指示): フランジ生成はいったん後回しにし、まず
        締結点同士を面(中立面)でつなぐメイン形状の生成そのものに注力する
        (学習データとして、フランジ付き部品より前にこちらを優先する)。そのため
        `include_flanges`の既定値をFalseにした。Trueにすれば、下記の§6.11〜6.15で
        確定したフランジ配置・コーナーリリーフ・幅マージンのロジックがそのまま動く
        (実装は削除しておらず、フラグで無効化しているだけ)。

        ユーザー知見(2026-08-04): 締結点をつなぐメイン形状(ジョグ)は、面同士の角度を
        できるだけ平行に近づけ、折れ目のRをできるだけ大きく取るほうが剛性・応力の観点で
        望ましい(小部品でもR40〜50まであり得る)。ただし締結点周りの必要最小半径
        (min_bearing_radius_mm)は侵せない。今回は自動生成のため特定の意図(最適化)は
        持たせず、spec.jog_ramp_extent_mmとspec.bend_radius_mm(いずれも締結点周りの
        必要最小半径を侵さない実行可能範囲内でのみサンプリング済み、
        templates/parallel_same_offset.py参照。2026-08-06に歩留まり改善のためbend_radius_mm
        もこちら側でジョグ幾何と同時にサンプリングするよう変更)でランプ形状と折れ目Rを
        決める。ramp_extent=0のときは旧来の垂直リザー(直角2曲げ)に自然に退化する。

        フランジ配置の方針転換(2026-08-06、ユーザー確定・SS6.11 Step3を置き換え):
        剛性を高めたい方向(θ)への厳密な一致は、離散的な辺の選択肢しかない以上不可能
        (連続的なθに対して選べる辺は有限個)。代わりに梁のフランジと同じ考え方を採用する
        — 締結点を結ぶ線(走行方向/frame.axis_dir)に平行なフランジは、どちらの締結点に
        荷重がかかっても、その正確な方向によらず横方向の曲げに広く効く。よって:
          - フランジはflat1・flat2それぞれの幅方向+half_width側の側辺(片側のみ、
            対称両側は採用しない・シンプルさ優先)に、走行方向に平行に立てる。
          - reinforcement_direction_deg(θ)はこのクラスでは今後使用しない(サンプリングは
            reinforcement.py側に残っているが未使用の付随値)。
          - flange_angle_degは可変(既定90度、稀に60〜120度の斜めフランジ。§4.2/reinforcement.py参照)。
            斜めフランジ時の戻りフランジ(先端に一般面と平行なもう一つのフランジ)は未実装。
          - フランジ輪郭のコーナーR(AddNewCorner、外R5mm)は未実装。次の増分として残している。

        コーナーリリーフ(2026-08-06、実機検証で確定): フランジ根本の折れ(flange_fold1/2)は、
        この配置ではメイン形状の折れ(fold1/fold2)と同じ角(頂点)を共有する(旧・走行方向端
        配置では離れた辺にあり無関係だった)。両方に独立してフィレットを当てると、フィレット
        領域が競合し、CATIA側のUpdateが失敗するか、あるいは(フィレット順序を逆にした場合)
        エラーなく幾何が縮退する(丸みがつかず鋭い折れ目のまま描画される)ことを、複数パターン
        のスクリーンショット目視検証で確認した。フィレット順序の反転は縮退が再発するため不採用。
        代わりに実務のシートメタルで一般的な「コーナーリリーフ」(2つの曲げ加工が交差する角では、
        一方の曲げをその角の手前で止めて干渉を避ける)にならい、フランジの走行方向の辺を、
        メイン折れのフィレット後退量(tangent_length、下記の締結点クリアランス計算で使うのと
        同じ値)ぶんだけ角の手前で止める(flange1_relieved/flange2_relieved)。中立面R最小4mm
        (このクラスの必須制約)を含む複数のR・ランプ角の組み合わせで、フィレット順序を変えずに
        Update成功・目視ともクリーンであることを確認済み。

        構築順序(SS6.11 Step1/Step2、ユーザー確定・コーナーリリーフ導入後も変更なし): flat1・
        ランプ・flat2・フランジ1・フランジ2をシャープな角のまま先に1回のJoinで完全結合し、
        メイン形状の折れ(fold1/fold2)を先にフィレットしてから、フランジ根本の折れを最後に
        フィレットする。フランジ根本のR(reinforcement.flange_bend_radius_mm)はメイン形状の
        bend_radius_mmより小さめ(板厚の0〜3倍程度、reinforcement.py参照)で、実務のビーム
        フランジとしてのシャープさを保つ。

        締結点の必要最小半径(min_bearing_radius_mm)の確保(2026-08-06、必須要件):
        走行方向については、flat1/flat2はrun=0/run_length(締結点そのものの位置)からではなく、
        run=-flange_margin / run_length+flange_marginから始めることで、締結点を中心とした
        平坦な最小半径分の余白を確保する(flange_margin=min_bearing_radius_mm。フランジが
        幅方向側辺に移ったため、以前のような走行方向フィレット後退量の上乗せは不要)。
        幅方向については、フランジ根本の折れが+half_width側から中心(締結点)に向かって
        後退してくる(tangent_length_for_bend_angle_rad(flange_angle_deg, flange_bend_radius_mm))ため、
        half_width - この後退量がmin_bearing_radius_mmを下回らないことを別途チェックする。
        ジョグ側についても、折れにフィレット半径Rを付けると
        理論上のシャープな角より手前から丸め始まる(classify.fold_tangent_length_mm参照)ため、
        丸め始点から締結点までの残り平坦長がmin_bearing_radius_mmを下回る場合は構築前に
        例外を送出する(CATIA呼び出し前のPython側の事前チェック)。spec.bend_radius_mmは
        サンプリング時点で既にこの制約を満たすよう実行可能範囲から選ばれているため、
        この例外は通常発生しないはず — 万一の回帰に備えた安全網として残している。
        """
        frame = _jog_frame(spec.point1, spec.point2)
        origin = spec.point1.position_xyz
        # Phase 1.5訂正(2026-08-07、ユーザー指摘): トリム経路(include_flanges=False)では
        # half_widthをspec.panel_width_mm/2(フランジ用に余裕を持たせた広いサンプリング幅)
        # ではなく、min_bearing_radius_mmそのものにする。_tangent_arc_boundaryは
        # half_width==radiusのとき厳密に退化し、接線が幅方向±radiusの平行線に、円弧が
        # ちょうど半円(180度)になる(解析的に検証済み) — これが真の「長円(スタジアム)」
        # 形状で、以前の(panel_width_mm由来の広いhalf_widthを使った)構築は接点が
        # 遠端寄りに偏り、三角形+先端だけ丸めたような形になっていた。ランプも同じ
        # half_widthを使うことで、flat1・ランプ・flat2の境界幅が揃う(段差なく結合できる)。
        half_width = spec.min_bearing_radius_mm if not include_flanges else spec.panel_width_mm / 2.0
        x_start = (frame.run_length_mm - spec.jog_ramp_extent_mm) / 2.0
        x_end = x_start + spec.jog_ramp_extent_mm

        tangent_length = fold_tangent_length_mm(frame.offset_mm, spec.jog_ramp_extent_mm, spec.bend_radius_mm)
        clear_length = x_start - tangent_length
        if clear_length < spec.min_bearing_radius_mm:
            raise ValueError(
                f"bend_radius_mm={spec.bend_radius_mm:.1f} at this jog geometry "
                f"(ramp_extent={spec.jog_ramp_extent_mm:.1f}, offset={frame.offset_mm:.1f}) "
                f"eats {tangent_length:.1f}mm into the flat, leaving only {clear_length:.1f}mm "
                f"before the fastening point -- less than min_bearing_radius_mm="
                f"{spec.min_bearing_radius_mm:.1f}mm. Infeasible; not attempting construction."
            )

        # 2026-08-07発覚: fold1(flat1-ランプ)とfold2(ランプ-flat2)は対称なジョグのため
        # 同じtangent_length分だけランプ側に後退する。ランプ区間(jog_ramp_extent_mm)が
        # 短く、bend_radius_mmが大きいと、両端からの後退量の合計(2*tangent_length)が
        # ランプ自体の長さを超え、fold1・fold2のフィレット領域がランプ上で重なって
        # CATIA側のUpdateが失敗する(フランジ根本とメイン折れが角を共有する場合と同種の
        # 「フィレット領域の競合」)。これはフランジの有無に関わらず起こりうるメインジョグ
        # 自体の事前チェック漏れだったため、ここで独立してInfeasibleとして弾く。
        if 2.0 * tangent_length > spec.jog_ramp_extent_mm:
            raise ValueError(
                f"bend_radius_mm={spec.bend_radius_mm:.1f} at this jog geometry "
                f"(ramp_extent={spec.jog_ramp_extent_mm:.1f}, offset={frame.offset_mm:.1f}) "
                f"has fold1/fold2 tangent lengths ({tangent_length:.1f}mm each) that together "
                f"({2.0 * tangent_length:.1f}mm) exceed the ramp's own extent -- the two fillets "
                "would overlap on the ramp. Infeasible; not attempting construction."
            )

        # ユーザー製造制約(2026-08-06): 中立面Rは全て最小R4を守るため、templates.py/
        # reinforcement.py側のサンプリングは、幅方向・フランジ高さ方向の実行可能上限が
        # 4mm未満のときはあえて4mm(=真の実行可能上限を超える値)を返すようにしてある。
        # そのケースをここで明示的にInfeasibleとして弾く(以前はサンプリング側の
        # クランプだけで保証していたが、下限4mmの導入でクランプが効かなくなるケースが
        # 生まれたため、事前チェックとして独立させた)。
        if spec.bend_radius_mm > 0.9 * half_width:
            raise ValueError(
                f"bend_radius_mm={spec.bend_radius_mm:.1f} exceeds 0.9x half_width={half_width:.1f} "
                "(would not satisfy the mandatory R4 minimum without exceeding panel width). "
                "Infeasible; not attempting construction."
            )
        if include_flanges and reinforcement.flange_bend_radius_mm > 0.9 * reinforcement.flange_height_mm:
            raise ValueError(
                f"flange_bend_radius_mm={reinforcement.flange_bend_radius_mm:.1f} exceeds "
                f"0.9x flange_height_mm={reinforcement.flange_height_mm:.1f} (would not satisfy "
                "the mandatory R4 minimum without exceeding the flange's own height). "
                "Infeasible; not attempting construction."
            )

        if include_flanges:
            # SS6.11 Step3訂正(2026-08-06、ユーザー方針転換): フランジは締結点を結ぶ線に
            # 平行な幅方向側辺(片側のみ)に立てる。荷重の正確な方向(θ)が不明でも、スパンに
            # 平行なフランジは横方向の曲げに広く効く(梁のフランジと同じ原理)。
            # reinforcement_direction_degはこの用途にはもう使わない。
            #
            # フランジ根本の折れは幅方向のedgeにあるため、その後退量は「走行方向の余白」
            # (flange_margin)ではなく「幅方向の余白」(half_width)を侵す。締結点は幅方向の
            # 中心(width=0)にあるので、half_width - flange_tangent_lengthが
            # min_bearing_radius_mmを下回らないことを確認する。
            flange_tangent_length = tangent_length_for_bend_angle_rad(
                math.radians(reinforcement.flange_angle_deg), reinforcement.flange_bend_radius_mm
            )
            flange_clear_width = half_width - flange_tangent_length
            if flange_clear_width < spec.min_bearing_radius_mm:
                raise ValueError(
                    f"flange_bend_radius_mm={reinforcement.flange_bend_radius_mm:.1f} eats "
                    f"{flange_tangent_length:.1f}mm into the panel width, leaving only "
                    f"{flange_clear_width:.1f}mm before the fastening point (centered at width=0) -- "
                    f"less than min_bearing_radius_mm={spec.min_bearing_radius_mm:.1f}mm. "
                    "Infeasible; not attempting construction."
                )

        ramp_corners = _panel_corners(origin, frame, x_start, x_end, 0.0, frame.offset_mm, half_width)

        # Phase 1.5 余肉削減(2026-08-07、include_flanges=Falseのとき): flat1/flat2は
        # それぞれ締結点1個だけを持つ面なので、docs/synthetic_two_joint_generation_roadmap.md
        # SS6.3の原則により、必要最小材料は「締結点周りの半径min_bearing_radius_mmの円」と
        # 「ランプ側の遠端(ramp_cornersの該当辺)」の凸包(接線2本+円弧+直線)になる。
        # 円自体がrun方向に-radius(=-min_bearing_radius_mm)まで自然に広がるため、以前の
        # flange_margin(=min_bearing_radius_mmだけ締結点の外側に矩形を延長)は不要になった
        # (円がその役割を厳密な形状で代替する)。
        #
        # フランジを有効化した場合(include_flanges=True)は、フランジのガイド辺が円弧
        # 境界にどう接続すべきか未検討のため、Phase 1.5トリムはまだ適用しない
        # (フランジ復活時に対応する別課題として残す) — 旧来の矩形+flange_marginのまま。
        flange_margin = spec.min_bearing_radius_mm
        flat1_corners = flat2_corners = None
        flange1_relieved = flange2_relieved = None
        if include_flanges:
            flat1_corners = _panel_corners(origin, frame, -flange_margin, x_start, 0.0, 0.0, half_width)
            flat2_corners = _panel_corners(
                origin, frame, x_end, frame.run_length_mm + flange_margin,
                frame.offset_mm, frame.offset_mm, half_width
            )
            # コーナーリリーフ(2026-08-06、実機検証で確定): フランジ根本の折れは、メイン形状の
            # 折れ(fold1/fold2)と同じ角(頂点)を共有する。両方に別々のフィレットを当てると、
            # フィレット領域が競合し、CATIA側のUpdateが失敗するか(発生順序次第で)エラーなく
            # 幾何が縮退する(丸みがつかず鋭い折れ目のまま描画される)ケースがあることを、
            # 複数パターンのスクリーンショット目視検証で確認した。フィレット順序の反転では
            # 縮退が再発したため不採用。実務のシートメタルで一般的な「コーナーリリーフ」
            # (2つの曲げ加工が交差する角では、一方の曲げをその角の手前で止めて干渉を避ける)
            # にならい、フランジの走行方向の辺を、メイン折れのフィレット後退量
            # (tangent_length、上でjog側の事前チェックに使ったのと同じ値)ぶんだけ手前で
            # 止める。構築順序(メイン折れ→フランジ根本、SS6.11確定順序)は変更しない。
            flange1_relieved = _panel_corners(
                origin, frame, -flange_margin, x_start - tangent_length, 0.0, 0.0, half_width
            )[2]
            flange2_relieved = _panel_corners(
                origin, frame, x_end + tangent_length, frame.run_length_mm + flange_margin,
                frame.offset_mm, frame.offset_mm, half_width
            )[1]

        doc = self.new_part_document()
        try:
            part = doc.Part
            hsf = part.HybridShapeFactory
            spa = doc.GetWorkbench("SPAWorkbench")
            # AddNewXxxが作る形状はbodyへAppendHybridShapeしないとドキュメントツリーに
            # 反映されず、Updateしても幾何が確定しない(既存catia_midsurface.pyと同じ定石)。
            body = part.HybridBodies.Add()
            body.Name = f"{part_name}_synthetic"

            riser = self.rect_fill(hsf, part, body, ramp_corners)
            if include_flanges:
                flat1 = self.rect_fill(hsf, part, body, flat1_corners)
                flat2 = self.rect_fill(hsf, part, body, flat2_corners)
            else:
                # Phase 1.5 余肉削減: 締結点周りの半径min_bearing_radius_mmの円とランプ側
                # 遠端の凸包(trimmed_end_panel参照)。flat2はflat1の鏡像(点2から見て
                # ランプは-axis_dir方向)なので、axis_dir/width_dirを両方符号反転して渡す
                # (両方反転すればCATIAの円弧掃引角の向き規約(+normal側から見てCCW)を
                # 保ったまま鏡像を表現できる — 実機検証で確認済み)。
                flat1 = self.trimmed_end_panel(
                    hsf, part, body, origin, frame.normal, frame.axis_dir, frame.width_dir,
                    x_start, half_width, spec.min_bearing_radius_mm,
                )
                flat2 = self.trimmed_end_panel(
                    hsf, part, body, spec.point2.position_xyz, frame.normal,
                    tuple(-c for c in frame.axis_dir), tuple(-c for c in frame.width_dir),
                    frame.run_length_mm - x_end, half_width, spec.min_bearing_radius_mm,
                )
            pieces = [flat1, riser, flat2]

            if include_flanges:
                # 方針転換(2026-08-06): フランジは締結点を結ぶ線(=走行方向/frame.axis_dir)に
                # 平行な、幅方向+half_width側の側辺(片側のみ)に立てる。両flatパネルとも同じ
                # +width側の辺を使うので外向き接線はどちらもframe.width_dirで共通。
                outward_side = frame.width_dir
                flange1 = self.flat_flange(
                    hsf, part, body, flat1_corners[1], flange1_relieved, frame.normal, outward_side,
                    reinforcement.flange_height_mm, reinforcement.flange_angle_deg,
                )
                flange2 = self.flat_flange(
                    hsf, part, body, flange2_relieved, flat2_corners[2], frame.normal, outward_side,
                    reinforcement.flange_height_mm, reinforcement.flange_angle_deg,
                )
                pieces += [flange1, flange2]

            # SS6.11 Step1: シャープな角のまま全要素(flat1/ランプ/flat2 + 任意でフランジ1/2)を
            # 先に完全結合してから、実エッジにフィレットを順番に当てる(モジュールdocstring
            # 参照 — Rが大きいときAddNewFilletBiTangentのブリッジ面が「浮く」問題をこの方式で
            # 解消した)。フランジ根本の折れはまだシャープなまま(SS6.11 Step2でフィレットを追加する)。
            sharp_whole = self.join(hsf, part, body, pieces)
            part.Update()

            # fold1(flat1-ランプ境界)はramp_corners[0]/[1](run=x_start側の辺)と同一。
            # flat1_corners[2]/[3]も同じ辺だったが、flat1_corners はinclude_flanges=Falseの
            # ときNone(Phase 1.5トリムではflat1_cornersという矩形自体が存在しない)ため、
            # 常に存在するramp_corners側から計算する。
            fold1_mid = tuple((ramp_corners[0][i] + ramp_corners[1][i]) / 2 for i in range(3))
            fold2_mid = tuple((ramp_corners[2][i] + ramp_corners[3][i]) / 2 for i in range(3))

            edge1_ref = self.find_edge_near(doc, part, spa, hsf, body, sharp_whole, fold1_mid)
            filleted_once = self.edge_fillet(part, edge1_ref, spec.bend_radius_mm)
            body.AppendHybridShape(filleted_once)
            part.Update()

            edge2_ref = self.find_edge_near(doc, part, spa, hsf, body, filleted_once, fold2_mid)
            whole = self.edge_fillet(part, edge2_ref, spec.bend_radius_mm)
            body.AppendHybridShape(whole)
            part.Update()

            if include_flanges:
                # SS6.11 Step2: メイン形状のフィレットが終わってから、フランジ根本の折れを
                # 最後にフィレットする(ユーザー確定の構築順序)。中立面R最小4mmの制約導入
                # (2026-08-06)によりflange_bend_radius_mmは常に4mm以上(既に事前チェック済み)
                # なので、以前あった「半径がほぼ0ならスキップ」の分岐は不要になった。
                flange_fold1_mid = tuple((flat1_corners[1][i] + flange1_relieved[i]) / 2 for i in range(3))
                flange_fold2_mid = tuple((flange2_relieved[i] + flat2_corners[2][i]) / 2 for i in range(3))

                flange_edge1_ref = self.find_edge_near(doc, part, spa, hsf, body, whole, flange_fold1_mid)
                whole = self.edge_fillet(part, flange_edge1_ref, reinforcement.flange_bend_radius_mm)
                body.AppendHybridShape(whole)
                part.Update()

                flange_edge2_ref = self.find_edge_near(doc, part, spa, hsf, body, whole, flange_fold2_mid)
                whole = self.edge_fillet(part, flange_edge2_ref, reinforcement.flange_bend_radius_mm)
                body.AppendHybridShape(whole)
                part.Update()

            part.InWorkObject = whole
            part.Update()
        except Exception:
            self._discard_failed_attempt(out_dir, part_name, doc)
            raise

        return self.export_stp(doc, whole, out_dir, part_name)

    # ------------------------------------------------------------ 任意法線・任意位置(2026-08-10、roadmap SS6.20)

    def build_general_two_point(
        self,
        point1: FasteningPoint,
        point2: FasteningPoint,
        *,
        min_bearing_radius_mm: float,
        half_width_mm: float,
        fold1_run_mm: float,
        fold2_run_mm: float,
        bend_radius_mm: float,
        out_dir: str,
        part_name: str,
        bead: BeadParams | None = None,
    ) -> GeneratedPart:
        """任意の法線・任意の位置の締結点2点を、単曲げまたは2曲げの面でつなぐ。

        2026-08-21(roadmap SS6.24)より2経路ある: 2平面の交線で1回曲げれば足りるうえ、
        3ピース構成での「無駄な曲げ量」が大きいケース(`classify.single_fold_layout`が
        判定)は**パネル2枚+折れ目1本**で作る。それ以外は従来どおりflat1・ランプ・flat2の
        3ピース(折れ目2本)で作る。後者にはflat同士の面干渉チェックが入る。

        `classify.two_point_frame`(共通幅方向w、締結点ごとのローカル走行方向u1/u2)を使い、
        `build_parallel_same_offset`のn1=n2専用ロジックを一般化したもの。フランジ・
        Phase 1.5トリム(円弧)は未対応 — まずメイン形状生成の成功を優先する方針
        (ユーザー確定、2026-08-10)。トリムは`trimmed_end_panel`の円弧掃引角が
        axis_dir/width_dirの符号に依存するため、flat1・flat2で法線が異なる一般ケースでの
        符号整合を別途検討してから再適用する。

        `fold1_run_mm`/`fold2_run_mm`はそれぞれ締結点1/2からランプ側の折れ目までの距離
        (旧来の対称な`jog_ramp_extent_mm`分割と異なり、独立な自由パラメータ)。
        `bend_radius_mm`は両方の折れに共通の単一半径(旧来と同じ設計)。
        """
        if bend_radius_mm < MIN_NEUTRAL_PLANE_RADIUS_MM:
            raise ValueError(
                f"bend_radius_mm={bend_radius_mm:.1f} is below the mandatory minimum "
                f"{MIN_NEUTRAL_PLANE_RADIUS_MM:.1f}mm. Infeasible; not attempting construction."
            )
        if bend_radius_mm > 0.9 * half_width_mm:
            raise ValueError(
                f"bend_radius_mm={bend_radius_mm:.1f} exceeds 0.9x half_width_mm={half_width_mm:.1f}. "
                "Infeasible; not attempting construction."
            )

        frame = two_point_frame(point1, point2)
        margin = min_bearing_radius_mm

        flat1_corners = end_panel_corners(
            point1.position_xyz, frame.u1, frame.w, -margin, fold1_run_mm, half_width_mm
        )
        flat2_corners = end_panel_corners(
            point2.position_xyz, frame.u2, frame.w, -fold2_run_mm, margin, half_width_mm
        )
        ramp_corners = [flat1_corners[3], flat1_corners[2], flat2_corners[1], flat2_corners[0]]

        mid_near = tuple((flat1_corners[2][i] + flat1_corners[3][i]) / 2 for i in range(3))
        mid_far = tuple((flat2_corners[0][i] + flat2_corners[1][i]) / 2 for i in range(3))
        fold1_angle, fold2_angle = ramp_fold_angles_rad(mid_near, mid_far, frame.u1, frame.u2)

        layout = single_fold_layout(
            point1,
            point2,
            frame,
            bend_radius_mm=bend_radius_mm,
            min_bearing_radius_mm=margin,
            half_width_mm=half_width_mm,
            ramp_fold_angles=(fold1_angle, fold2_angle),
        )
        if layout is not None:
            # 単曲げ(roadmap SS6.24): 2平面の交線で1回だけ曲げる。2枚のパネルが折れ目を
            # 共有するので断面が2セグメントになり、flat同士の干渉は原理的に起きない
            # (下の2曲げ経路のクリアランスチェックはこちらには不要)。fold1_run_mm/
            # fold2_run_mm は使わない — 折れ目の位置は交線として幾何的に決まるため。
            panel_corner_sets = [
                end_panel_corners(
                    layout.origin1, frame.u1, frame.w, -margin, layout.d1_mm, layout.half_width_mm
                ),
                end_panel_corners(
                    layout.origin2, frame.u2, frame.w, -layout.d2_mm, margin, layout.half_width_mm
                ),
            ]
            first = panel_corner_sets[0]
            fold_mids = [tuple((first[2][i] + first[3][i]) / 2 for i in range(3))]
            panel_corner_sets, fillet_groups = self._apply_bead(
                panel_corner_sets, [fold_mids], bend_radius_mm, bead
            )
            return self._assemble_general_two_point(
                panel_corner_sets,
                fillet_groups,
                f"single fold={math.degrees(layout.bend_angle_rad):.1f}deg, R={bend_radius_mm:.1f}",
                out_dir,
                part_name,
            )

        tangent1 = tangent_length_for_bend_angle_rad(fold1_angle, bend_radius_mm)
        tangent2 = tangent_length_for_bend_angle_rad(fold2_angle, bend_radius_mm)

        if fold1_run_mm - tangent1 < min_bearing_radius_mm:
            raise ValueError(
                f"fold1 (angle={math.degrees(fold1_angle):.1f}deg, bend_radius_mm={bend_radius_mm:.1f}) "
                f"eats {tangent1:.1f}mm into flat1, leaving less than min_bearing_radius_mm="
                f"{min_bearing_radius_mm:.1f}mm before point1. Infeasible; not attempting construction."
            )
        if fold2_run_mm - tangent2 < min_bearing_radius_mm:
            raise ValueError(
                f"fold2 (angle={math.degrees(fold2_angle):.1f}deg, bend_radius_mm={bend_radius_mm:.1f}) "
                f"eats {tangent2:.1f}mm into flat2, leaving less than min_bearing_radius_mm="
                f"{min_bearing_radius_mm:.1f}mm before point2. Infeasible; not attempting construction."
            )
        ramp_length = math.sqrt(sum((mid_far[i] - mid_near[i]) ** 2 for i in range(3)))
        if tangent1 + tangent2 > ramp_length:
            raise ValueError(
                f"fold1/fold2 tangent lengths ({tangent1:.1f}mm + {tangent2:.1f}mm) exceed the "
                f"ramp's own length ({ramp_length:.1f}mm) -- the two fillets would overlap on the "
                "ramp. Infeasible; not attempting construction."
            )
        # 2026-08-21(roadmap SS6.24): 断面上でflat1とflat2が交差/近接するケース(実測6.2%が
        # 実際に3Dで面同士が干渉、さらに1.6%が5mm未満のニアミス)。上の3つの事前チェックは
        # どれも隣接する折れ目まわりしか見ておらず、非隣接のflat同士を見ていなかった。
        clearance = flat_panels_clearance_mm(
            point1,
            point2,
            frame,
            min_bearing_radius_mm=margin,
            fold1_run_mm=fold1_run_mm,
            fold2_run_mm=fold2_run_mm,
            half_width_mm=half_width_mm,
        )
        if clearance < MIN_PANEL_CLEARANCE_MM:
            raise ValueError(
                f"flat1 and flat2 come within {clearance:.1f}mm of each other in the section "
                f"(minimum {MIN_PANEL_CLEARANCE_MM:.1f}mm) -- the folded panels would interfere. "
                "Infeasible; not attempting construction."
            )

        fold1_mid = tuple((ramp_corners[0][i] + ramp_corners[1][i]) / 2 for i in range(3))
        fold2_mid = tuple((ramp_corners[2][i] + ramp_corners[3][i]) / 2 for i in range(3))
        panel_corner_sets, fillet_groups = self._apply_bead(
            [flat1_corners, ramp_corners, flat2_corners],
            [[fold1_mid], [fold2_mid]],
            bend_radius_mm,
            bead,
        )
        return self._assemble_general_two_point(
            panel_corner_sets,
            fillet_groups,
            f"fold1={math.degrees(fold1_angle):.1f}deg, fold2={math.degrees(fold2_angle):.1f}deg, "
            f"R={bend_radius_mm:.1f}",
            out_dir,
            part_name,
        )

    @staticmethod
    def _apply_bead(
        panel_corner_sets: list[list[Vec3]],
        fold_groups: list[list[Vec3]],
        bend_radius_mm: float,
        bead: BeadParams | None,
    ) -> tuple[list[list[Vec3]], list[tuple[list[Vec3], float]]]:
        """ビードがあればパネルを5ストリップのセル群に置き換え、フィレット群を組み直す。

        ビード無しの場合は、パネルもフィレット群もそのまま(折れ目1本=1フィーチャーと
        いう従来の挙動を厳密に保つ)。ビードを指定されたのにパネル幅が足りない場合は
        Infeasible(ValueError)にする — 黙ってビード無しで作ると、バッチの記録上は
        ビード付きなのに実物には無い、という食い違いが生じるため。

        ビード有りの場合、各折れ目は5ストリップに分断されるので、その5本を1つの
        フィレットフィーチャーにまとめる。ビードの縦4本 x パネル数も、まとめて
        もう1つのフィーチャーにする(交差部をCATIAに一括で解かせるため、SS6.25)。
        """
        if bead is None:
            return panel_corner_sets, [(mids, bend_radius_mm) for mids in fold_groups]
        if not bead_fits(panel_corner_sets, bead):
            raise ValueError(
                f"bead footprint (half={bead.half_footprint_mm:.1f}mm + margin) does not fit "
                "within the panel width. Infeasible; not attempting construction."
            )

        cells = bead_cells(panel_corner_sets, bead)
        panel_count = len(panel_corner_sets)

        def midpoint(a: Vec3, b: Vec3) -> Vec3:
            return tuple((a[i] + b[i]) / 2 for i in range(3))

        fold_mids = [
            midpoint(cells[fold_index * 5 + strip][2], cells[fold_index * 5 + strip][3])
            for fold_index in range(panel_count - 1)
            for strip in range(5)
        ]
        bead_mids = [
            midpoint(cells[panel_index * 5 + strip][1], cells[panel_index * 5 + strip][2])
            for panel_index in range(panel_count)
            for strip in range(4)
        ]
        return cells, [(fold_mids, bend_radius_mm), (bead_mids, bead.bend_radius_mm)]

    def _assemble_general_two_point(
        self,
        panel_corner_sets: list[list[Vec3]],
        fillet_groups: list[tuple[list[Vec3], float]],
        geometry_label: str,
        out_dir: str,
        part_name: str,
    ) -> GeneratedPart:
        """平坦パネル群をjoinし、フィレットを当ててエクスポートする。

        単曲げ(パネル2枚・折れ目1本)・2曲げ(flat1+ランプ+flat2・折れ目2本)・ビード付き
        (パネルを5ストリップに分割、SS6.25)で共通。`fillet_groups`は(折れ目中点のリスト,
        半径)の組で、1組が1つのEdgeFilletフィーチャーになる — ビードのように複数の
        フィレット列が交差する場合、まとめないとCATIAが交差点を解けない(SS6.25)。
        """
        doc = self.new_part_document()
        try:
            part = doc.Part
            hsf = part.HybridShapeFactory
            spa = doc.GetWorkbench("SPAWorkbench")
            body = part.HybridBodies.Add()
            body.Name = f"{part_name}_synthetic"

            faces = [self.rect_fill(hsf, part, body, corners) for corners in panel_corner_sets]
            whole = self.join(hsf, part, body, faces)
            part.Update()

            # 2026-08-10発覚(roadmap SS6.20): 任意法線ケースでは、fold1/fold2の折れ角の
            # 数値自体は事前チェック(タンジェント長・R>=4mm等)を満たしていても、2箇所目の
            # エッジフィレットのUpdateがCATIA側の数値的な脆さで失敗することがある。実機で
            # 72通り以上の(fold1角度, fold2角度, 幅方向オフセット, 3D回転)の組み合わせを
            # 体系的に検証したが、系統的な角度閾値やオフセット閾値としては再現できず、
            # 特定の3D配置に固有の現象と判断した(小さな摂動を加えても安定して再現する
            # 一方、幾何パラメータを変えた再構成では再現しない)。Python側の事前チェックで
            # 確実に予測するのは非現実的なため、ここでCATIA側の失敗を捕まえてInfeasible
            # (ValueError)に変換し、batch_generate.pyの既存skip-and-retry方針(SS6.18)に
            # 委ねる。他のステップ(join等)で例外が出た場合は本来のバグの可能性が高いため、
            # ここでは変換せずそのまま送出する。
            try:
                for fold_mids, radius_mm in fillet_groups:
                    edge_refs = [
                        self.find_edge_near(doc, part, spa, hsf, body, whole, fold_mid)
                        for fold_mid in fold_mids
                    ]
                    whole = self.edge_fillet_group(part, edge_refs, radius_mm)
                    body.AppendHybridShape(whole)
                    part.Update()
            except Exception as fillet_exc:
                raise ValueError(
                    f"CATIA edge fillet failed for this specific fold geometry "
                    f"({geometry_label}) -- a known numerically fragile case "
                    f"(roadmap SS6.20), not predictable from a Python-side pre-check. "
                    f"Infeasible; not attempting further construction. Original error: {fillet_exc}"
                ) from fillet_exc

            part.InWorkObject = whole
            part.Update()
        except Exception:
            self._discard_failed_attempt(out_dir, part_name, doc)
            raise

        return self.export_stp(doc, whole, out_dir, part_name)

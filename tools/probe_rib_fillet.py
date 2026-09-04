"""リブの稜線フィレットが通る条件を総当たりで探る(2026-09-04)。

`BRepFilletAPI_MakeFillet` が `IsDone()==False` を返すだけでは原因が分からないので、
診断API(`NbFaultyContours` / `StripeStatus` / `HasResult`)を読み、
形状の与え方(開いたシェル / 閉じたソリッド / 部分シェル)、順番、半径、
フィレット曲面の種類を振って通る組み合わせを探す。

使い方: python tools/probe_rib_fillet.py [件数]
"""
from __future__ import annotations

import collections
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from OCC.Core.BRep import BRep_Tool  # noqa: E402
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing  # noqa: E402
from OCC.Core.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCC.Core.BRepFilletAPI import BRepFilletAPI_MakeFillet  # noqa: E402
from OCC.Core.BRepOffset import BRepOffset_Skin  # noqa: E402
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakeOffsetShape  # noqa: E402
from OCC.Core.ChFi3d import ChFi3d_Polynomial, ChFi3d_QuasiAngular, ChFi3d_Rational  # noqa: E402
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SHELL  # noqa: E402
from OCC.Core.TopExp import TopExp_Explorer, topexp  # noqa: E402
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape  # noqa: E402
from OCC.Core.TopoDS import topods  # noqa: E402

import synthetic_generator.occt_build as ob  # noqa: E402
from synthetic_generator.families import FAMILIES, Knobs  # noqa: E402

SHAPES = {"rational": ChFi3d_Rational, "quasi": ChFi3d_QuasiAngular, "poly": ChFi3d_Polynomial}


def capture(count: int):
    """素の(フィレット前の)リブ部品のシェルと面名を集める。"""
    out = pathlib.Path(__file__).resolve().parent / "probe_output" / "rib_raw"
    out.mkdir(parents=True, exist_ok=True)
    original = ob.OcctPartBuilder._sew
    grabbed = []
    rng = random.Random(11)
    builder = ob.OcctPartBuilder()
    knobs = Knobs(distance_mm=(50.0, 120.0), bearing_radius_mm=(12.5, 18.0))
    while len(grabbed) < count:
        drawn = FAMILIES["rib"](rng, knobs)
        if drawn is None:
            continue
        spec, _bead, _flange, rib = drawn
        holder: dict = {}

        def hook(faces, _h=holder):
            shape, named = original(faces)
            _h["shape"], _h["faces"] = shape, named
            return shape, named

        ob.OcctPartBuilder._sew = staticmethod(hook)
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                target_folds=spec.target_folds,
                out_dir=str(out), part_name="probe", rib=rib)
            grabbed.append((holder["shape"], holder["faces"], rib))
        except Exception:
            pass
        finally:
            ob.OcctPartBuilder._sew = original
    return grabbed


def rib_edges(shape, faces, which: str = "all"):
    """リブに接するエッジ。which='ridge' は稜AB、'base' は底辺4本。"""
    rib_faces = {f for f, n in faces.items() if n.startswith("rib_")}
    mapping = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, mapping)
    ridge, base = [], []
    for i in range(1, mapping.Size() + 1):
        edge = topods.Edge(mapping.FindKey(i))
        if BRep_Tool.Degenerated(edge):
            continue
        touching = [topods.Face(f) for f in mapping.FindFromIndex(i)]
        hits = sum(f in rib_faces for f in touching)
        if hits == 2:
            ridge.append(edge)
        elif hits == 1:
            base.append(edge)
    return {"all": ridge + base, "ridge": ridge, "base": base}[which]


def try_fillet(shape, edges, radius: float, *, fillet_shape=None, one_by_one=False):
    """フィレットを試し、(成功か, 診断文字列) を返す。"""
    try:
        current = shape
        groups = [[e] for e in edges] if one_by_one else [edges]
        note = ""
        for group in groups:
            maker = BRepFilletAPI_MakeFillet(current)
            if fillet_shape is not None:
                maker.SetFilletShape(fillet_shape)
            for edge in group:
                maker.Add(radius, edge)
            maker.Build()
            if not maker.IsDone():
                note = (f"faulty contours {maker.NbFaultyContours()}, "
                        f"faulty vertices {maker.NbFaultyVertices()}, "
                        f"has result {maker.HasResult()}, "
                        f"stripe {[maker.StripeStatus(i) for i in range(1, maker.NbContours() + 1)]}")
                return False, note
            current = maker.Shape()
        return bool(BRepCheck_Analyzer(current).IsValid()), "valid" if True else note
    except Exception as exc:  # OCCTが例外を投げることもある
        return False, f"{type(exc).__name__}: {str(exc)[:60]}"


def as_solid(shape):
    """開いたシェルを厚み付けして閉じたソリッドにする(フィレットはソリッドに強い)。"""
    offset = BRepOffsetAPI_MakeOffsetShape()
    offset.PerformByJoin(shape, 1.0, 1.0e-3, BRepOffset_Skin, False, False, 0, False)
    if not offset.IsDone():
        return None
    return offset.Shape()


def main() -> None:
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    samples = capture(count)
    print(f"素のリブシェル {len(samples)} 件\n")

    trials = [
        ("開シェル / 全辺 / R5",            lambda s, f: try_fillet(s, rib_edges(s, f), 5.0)),
        ("開シェル / 稜のみ / R5",          lambda s, f: try_fillet(s, rib_edges(s, f, "ridge"), 5.0)),
        ("開シェル / 底辺のみ / R5",        lambda s, f: try_fillet(s, rib_edges(s, f, "base"), 5.0)),
        ("開シェル / 稜->底辺 / R5",        lambda s, f: try_fillet(
            s, rib_edges(s, f, "ridge") + rib_edges(s, f, "base"), 5.0, one_by_one=True)),
        ("開シェル / 底辺->稜 / R5",        lambda s, f: try_fillet(
            s, rib_edges(s, f, "base") + rib_edges(s, f, "ridge"), 5.0, one_by_one=True)),
        ("開シェル / 全辺 / R1",            lambda s, f: try_fillet(s, rib_edges(s, f), 1.0)),
        ("開シェル / 全辺 / R5 / rational", lambda s, f: try_fillet(
            s, rib_edges(s, f), 5.0, fillet_shape=SHAPES["rational"])),
        ("開シェル / 全辺 / R5 / quasi",    lambda s, f: try_fillet(
            s, rib_edges(s, f), 5.0, fillet_shape=SHAPES["quasi"])),
    ]

    for label, run in trials:
        ok = 0
        notes: collections.Counter = collections.Counter()
        for shape, faces, _rib in samples:
            good, note = run(shape, faces)
            ok += good
            if not good:
                notes[note[:70]] += 1
        print(f"  {label:32s}: {ok}/{len(samples)}")
        for note, n in notes.most_common(2):
            print(f"        {n}件  {note}")

    # ソリッド化してから
    print()
    ok = solids = 0
    notes = collections.Counter()
    for shape, faces, _rib in samples:
        solid = as_solid(shape)
        if solid is None:
            notes["厚み付け失敗"] += 1
            continue
        solids += 1
        good, note = try_fillet(solid, rib_edges(solid, {}, "all") or [], 5.0)
        ok += good
    print(f"  厚み付けソリッド化: {solids}/{len(samples)} 成功")


if __name__ == "__main__":
    main()

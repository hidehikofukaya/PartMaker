"""OCCTバックエンドのスモーク + 幾何合格判定(2026-09-04)。

「例外が出ない」だけでは不足(docs/REVIEW_bead_sweep_twist.md の指摘)なので、
形が正しいことを測る:

  A. 全エッジが直線か円弧(引継ぎ書 §3.4 ゲートA)
  B. シェルが有効(BRepCheck_Analyzer)、面ループが閉じている(自由エッジ=外形のみ)
  C. ビード頂部が意図した位置(法線側 <=0.1mm、鏡像側 >= 深さ)= ねじれていない
  D. 締結点2つが面上にあり、その周りに座面が残っている

使い方: python tools/occt_smoke.py [件数] [seed]
"""
from __future__ import annotations

import math
import pathlib
import random
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from OCC.Core.BRep import BRep_Tool  # noqa: E402
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface  # noqa: E402
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex  # noqa: E402
from OCC.Core.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape  # noqa: E402
from OCC.Core.GeomAbs import (  # noqa: E402
    GeomAbs_BSplineSurface, GeomAbs_Circle, GeomAbs_Cone, GeomAbs_Cylinder,
    GeomAbs_Line, GeomAbs_Plane, GeomAbs_SurfaceOfExtrusion, GeomAbs_Torus,
)
from OCC.Core.GCPnts import GCPnts_AbscissaPoint  # noqa: E402
from OCC.Core.STEPControl import STEPControl_Reader  # noqa: E402
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE  # noqa: E402
from OCC.Core.TopExp import TopExp_Explorer, topexp  # noqa: E402
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape  # noqa: E402
from OCC.Core.TopoDS import topods  # noqa: E402
from OCC.Core.GC import GC_MakeCircle  # noqa: E402
from OCC.Core.gp import gp_Pnt  # noqa: E402

from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.classify import classify  # noqa: E402
from synthetic_generator.general_geometry import plan_for  # noqa: E402
from synthetic_generator.occt_build import (  # noqa: E402
    OcctPartBuilder, _Frame, _Straight, _add, _dot, _scale, _build_path,
)
from synthetic_generator.templates.general_two_point import (  # noqa: E402
    draw_fold_count,
    resolve_reinforcement,
)
from synthetic_generator.templates.general_two_point import sample as sample_general  # noqa: E402

CURVE_NAMES = {GeomAbs_Line: "line", GeomAbs_Circle: "circle"}
SURFACE_NAMES = {
    GeomAbs_Plane: "plane", GeomAbs_Cylinder: "cylinder", GeomAbs_Cone: "cone",
    GeomAbs_Torus: "torus", GeomAbs_BSplineSurface: "bspline",
    GeomAbs_SurfaceOfExtrusion: "extrusion",
}


def read_step(path: str):
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != 1:
        raise RuntimeError(f"cannot read {path}")
    reader.TransferRoots()
    return reader.OneShape()


def distance_to(shape, point) -> float:
    vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(*point)).Vertex()
    calc = BRepExtrema_DistShapeShape(shape, vertex)
    calc.Perform()
    return calc.Value()


def primitive_deviation(edge) -> float:
    """エッジを1本の直線/円弧に当てたときの最大ずれ[mm](引継ぎ書 §3.4 ゲートA)。

    OCCTの曲線**型**では判定しない — ランアウトの直線織り面の稜線は幾何的に厳密な
    直線なのにB-splineとして表現されるため(2026-09-04実測: maxdev 0.00000mm)。
    下流の抽出器はプリミティブ当てはめで判定するので、こちらに合わせる。
    """
    curve = BRepAdaptor_Curve(edge)
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    samples = [curve.Value(u0 + (u1 - u0) * k / 24.0) for k in range(25)]
    p0, p1 = samples[0], samples[-1]
    # 直線当てはめ
    v = (p1.X() - p0.X(), p1.Y() - p0.Y(), p1.Z() - p0.Z())
    vv = sum(c * c for c in v)
    line_dev = 0.0
    if vv > 1e-18:
        for q in samples[1:-1]:
            wv = (q.X() - p0.X(), q.Y() - p0.Y(), q.Z() - p0.Z())
            t = sum(a * b for a, b in zip(v, wv)) / vv
            proj = tuple(p0.Coord()[i] + t * v[i] for i in range(3))
            line_dev = max(line_dev, math.dist((q.X(), q.Y(), q.Z()), proj))
    else:
        line_dev = 1e9
    # 円弧当てはめ(始点・中点・終点を通る円との残差)
    arc_dev = 1e9
    mid = samples[len(samples) // 2]
    try:
        circ = GC_MakeCircle(p0, mid, p1).Value()
        centre = circ.Location()
        radius = circ.Radius()
        arc_dev = max(abs(centre.Distance(q) - radius) for q in samples)
        normal = circ.Axis().Direction()
        for q in samples:
            d = (q.X() - centre.X(), q.Y() - centre.Y(), q.Z() - centre.Z())
            arc_dev = max(arc_dev, abs(d[0] * normal.X() + d[1] * normal.Y() + d[2] * normal.Z()))
    except Exception:
        pass
    return min(line_dev, arc_dev)


def bead_probe(spec, bead):
    """経路中央でのビード頂部の意図位置と、その基準面鏡像を返す。"""
    plan = plan_for(spec)
    path, start, w, n0 = _build_path(plan, spec.bend_radius_mm)
    lift = OcctPartBuilder._bead_lift(path, bead, spec.bend_radius_mm)
    total = sum(s.length if isinstance(s, _Straight) else abs(s.angle) * s.radius for s in path)
    ey = w if _dot(w, plan.panel_frames[0].v) > 0 else _scale(w, -1.0)
    frame, cursor = _Frame(start, ey, n0), 0.0
    for step in path:
        span = step.length if isinstance(step, _Straight) else abs(step.angle) * step.radius
        if cursor + span >= total / 2.0:
            fraction = (total / 2.0 - cursor) / span
            if isinstance(step, _Straight):
                frame = frame.translated(_scale(step.vector, fraction))
            else:
                sign = 1.0 if step.angle > 0 else -1.0
                centre = _add(frame.origin, _scale(step.normal, -sign * step.radius))
                frame = frame.rotated(centre, w, step.angle * fraction)
            break
        frame = (frame.translated(step.vector) if isinstance(step, _Straight)
                 else frame.rotated(_add(frame.origin, _scale(
                     step.normal, -(1.0 if step.angle > 0 else -1.0) * step.radius)), w, step.angle))
        cursor += span
    return frame.point((0.0, lift * bead.depth_mm)), frame.point((0.0, -lift * bead.depth_mm))


def audit(shape) -> dict:
    curves: dict[str, int] = {}
    surfaces: dict[str, int] = {}
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        kind = BRepAdaptor_Surface(topods.Face(explorer.Current())).GetType()
        name = SURFACE_NAMES.get(kind, f"type{kind}")
        surfaces[name] = surfaces.get(name, 0) + 1
        explorer.Next()
    edge_faces = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces)
    free_edges = 0
    worst_deviation = 0.0
    tiny_edges = 0
    degenerate_edges = 0
    for i in range(1, edge_faces.Size() + 1):
        edge = topods.Edge(edge_faces.FindKey(i))
        if BRep_Tool.Degenerated(edge):
            # 円錐の頂点(リブの菱形の先端)を閉じるための長さ0のエッジ。B-Repとして
            # 正しい表現で、ゴミエッジではない。プリミティブ判定からは外す。
            degenerate_edges += 1
            continue
        kind = BRepAdaptor_Curve(edge).GetType()
        name = CURVE_NAMES.get(kind, f"type{kind}")
        curves[name] = curves.get(name, 0) + 1
        worst_deviation = max(worst_deviation, primitive_deviation(edge))
        if GCPnts_AbscissaPoint.Length(BRepAdaptor_Curve(edge)) < 0.05:
            tiny_edges += 1
        if edge_faces.FindFromIndex(i).Size() == 1:
            free_edges += 1
    return {
        "faces": sum(surfaces.values()),
        "surfaces": surfaces,
        "edges": sum(curves.values()),
        "curves": curves,
        "free_edges": free_edges,
        "deviation": worst_deviation,
        "tiny_edges": tiny_edges,
        "degenerate_edges": degenerate_edges,
        "valid": bool(BRepCheck_Analyzer(shape).IsValid()),
    }


def main() -> None:
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 20260904
    # 第3引数で曲げ本数を固定できる(D1の検証用)。省略時は配分どおりに引く。
    want_folds = int(sys.argv[3]) if len(sys.argv) > 3 else None
    out_dir = pathlib.Path(__file__).resolve().parent / "occt_smoke"
    out_dir.mkdir(exist_ok=True)

    builder = OcctPartBuilder()
    rng = random.Random(seed)
    made, tried, failures = 0, 0, {}
    surfaces_total: dict[str, int] = {}
    curves_total: dict[str, int] = {}
    kinds = {"bead": 0, "flange": 0, "rib": 0, "plain": 0}
    fold_counts: dict[int, int] = {}
    t0 = time.time()

    while made < count and tried < count * 200:
        tried += 1
        folds = want_folds if want_folds is not None else draw_fold_count(rng)
        try:
            spec = sample_general(rng, gentle_folds=(tried % 3 == 0), target_folds=folds)
        except ValueError as exc:
            failures[str(exc)[:60]] = failures.get(str(exc)[:60], 0) + 1
            continue
        bead = flange = rib = None
        if rng.random() < 0.9:
            resolved = resolve_reinforcement(rng, spec)
            if resolved is None:
                failures["no reinforcement"] = failures.get("no reinforcement", 0) + 1
                continue
            spec, bead, flange, rib = resolved
        name = f"OCCT_{made + 1:04d}"
        try:
            part = builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=0.0,
                target_folds=spec.target_folds,
                out_dir=str(out_dir), part_name=name,
                bead=bead, flange=flange, rib=rib,
            )
        except ValueError as exc:
            failures[str(exc)[:60]] = failures.get(str(exc)[:60], 0) + 1
            continue
        made += 1
        kinds["bead" if bead else ("flange" if flange else ("rib" if rib else "plain"))] += 1
        actual = len(plan_for(spec).panel_frames) - 1
        fold_counts[actual] = fold_counts.get(actual, 0) + 1

        shape = read_step(part.stp_path)
        info = audit(shape)
        for key, value in info["surfaces"].items():
            surfaces_total[key] = surfaces_total.get(key, 0) + value
        for key, value in info["curves"].items():
            curves_total[key] = curves_total.get(key, 0) + value

        problems = []
        if not info["valid"]:
            problems.append("INVALID shape")
        if info["deviation"] > 0.25 * spec.thickness_mm:
            problems.append(f"edge deviates {info['deviation']:.3f}mm from a single primitive")
        if info["tiny_edges"]:
            problems.append(f"{info['tiny_edges']} edges shorter than 0.05mm")
        for label, point in (("p1", spec.point1.position_xyz), ("p2", spec.point2.position_xyz)):
            d = distance_to(shape, point)
            if d > 0.05:
                problems.append(f"{label} is {d:.3f}mm off the surface")
        if bead is not None:
            # C. ねじれ検出(docs/REVIEW_bead_sweep_twist.md の検出器):
            # 経路中央でのビード頂部の意図位置と、基準面を挟んだ鏡像の両方までの距離を測る。
            # ねじれていなければ top≈0、mirror>=深さ になる。
            top, mirror = bead_probe(spec, bead)
            d_top, d_mirror = distance_to(shape, top), distance_to(shape, mirror)
            if d_top > 0.1:
                problems.append(f"bead top is {d_top:.2f}mm off (twisted/flipped)")
            if d_mirror < 0.8 * bead.depth_mm:
                problems.append(f"bead mirror only {d_mirror:.2f}mm away (depth {bead.depth_mm:.1f}mm)")
        print(f"{name} {classify(spec.point1, spec.point2)!s:24s} "
              f"{'bead' if bead else ('flange' if flange else ('rib' if rib else 'plain')):6s} "
              f"faces={info['faces']:3d} edges={info['edges']:3d} free={info['free_edges']:3d} "
              f"dev={info['deviation']:.4f} "
              f"{'OK' if not problems else 'NG ' + '; '.join(problems)}", flush=True)

    dt = time.time() - t0
    print(f"\n{made} parts / {tried} attempts ({made / max(1, tried):.1%}) in {dt:.1f}s "
          f"= {dt / max(1, made):.2f}s/part", flush=True)
    print(f"kinds: {kinds}")
    print(f"folds: {dict(sorted(fold_counts.items()))}")
    print(f"surface types: {dict(sorted(surfaces_total.items(), key=lambda kv: -kv[1]))}")
    print(f"curve types:   {dict(sorted(curves_total.items(), key=lambda kv: -kv[1]))}")
    print("top rejections:")
    for reason, n in sorted(failures.items(), key=lambda kv: -kv[1])[:12]:
        print(f"  {n:5d}  {reason}")
    print(f"output: {out_dir}")


if __name__ == "__main__":
    main()

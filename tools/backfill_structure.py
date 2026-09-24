"""params に `structure` と座面比を書き足す(AMS 依頼 8 ④ と、返答 2026-09-10 §2 の契約)。

- `structure` : 全族共通キーの構造因子(`batch_generate.structure_of`)。occt21/22/23 は生成時に
                入っているので、無い族(occt11〜20、occt30)だけ計算する。値の無い因子は 0。
- `structure.seat_ratio_min`         : 座面比(**平らな円盤の定義**)。締結点から、部品の全エッジ
                                       (外形 + 折り線・ビードの足・フィレットの接線)までの最短距離 /
                                       必要座面半径 の最小値。生成時のゲート(2026-09-09 以降)と同じ定義
- `structure.seat_ratio_outline_min` : 同じく外形(自由エッジ)だけで測った値。2026-09-09 より前の
                                       ゲートの定義。旧世代のチャンクはこちらが ≥ 1 で保証されていた

座面比は AMS 側では正しく測れない(外形に面の所属が無いため)ので、こちらで計算して渡す契約
(AMS `REPLY_ML_seat_retraction_2026-09-10.md` §2)。形状(STEP)には触らない。

変種(variants/params)には `structure` だけ入れる(構造変種は構造が元と違うので、どの構造かを
明示する)。座面比は元の部品だけ(変種 5 万件の距離計算は重いので必要になったら足す)。

使い方: python tools/backfill_structure.py [--workers N] [チャンクのパス ...]
"""
from __future__ import annotations

import argparse
import io
import json
import multiprocessing as mp
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "synthetic_generator" / "src"))
sys.path.insert(0, str(ROOT / "tools"))


def _radii(meta: dict, n: int) -> tuple[list[float], str]:
    """締結点ごとの**設計時の**必要座面半径と、その出どころ。

    - `point_radii` が記録されていればそれ(box / panel / compose 第2期以降)
    - 合成族 第1期(occt30/chunk_01)は `point_radii` を記録していなかったが、タブ溶接の点だけ
      必要半径が小さい(`weld_bearing_mm`、7.5〜11)設計だった。点の並びは
      アンカー2 → 腕ごとの点(`arms[].points` の 'weld' / 'bolt')→ 基板上の点、なので復元できる
    - それ以外は部品で 1 つの `min_bearing_radius_mm`
    """
    spec = meta["spec"]
    b = spec["min_bearing_radius_mm"]
    for key in ("box", "panel", "compose"):
        blk = spec.get(key)
        if blk and blk.get("point_radii") and len(blk["point_radii"]) == n:
            return list(blk["point_radii"]), f"spec.{key}.point_radii"
    cp = spec.get("compose")
    if cp and cp.get("weld_bearing_mm") is not None:
        radii = [b, b]
        for arm in cp.get("arms", []):
            for pt in arm.get("points", []):
                radii.append(cp["weld_bearing_mm"] if pt[2] == "weld" else b)
        radii += [b] * (n - len(radii))
        if len(radii) == n:
            return radii, "reconstructed: compose.arms[].points + weld_bearing_mm"
    return [b] * n, "spec.min_bearing_radius_mm"


def _points(meta: dict) -> list[tuple]:
    spec = meta["spec"]
    pts = spec.get("annotated_points") or [spec["point1"], spec["point2"]]
    return [tuple(p["position_xyz"]) for p in pts]


def seat_ratios(stp: pathlib.Path, points, radii, welds: list | None = None) -> tuple[float, float]:
    """(全エッジでの最小座面比, 外形だけでの最小座面比)。

    半径が None の点(スポット溶接、2026-09-24)は座面比に入れず、welds が渡されていれば
    (自由縁まで, 他の辺まで) の実測を追記する。"""
    import synthetic_generator.occt_build as ob
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
    from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
    from OCC.Core.gp import gp_Pnt
    from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCC.Core.TopExp import topexp
    from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
    from OCC.Core.TopoDS import topods

    shape = ob._read_step(str(stp))
    amap = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, amap)
    edges = []
    for i in range(1, amap.Size() + 1):
        edges.append((topods.Edge(amap.FindKey(i)), amap.FindFromIndex(i).Size() == 1))
    worst_all = worst_out = float("inf")
    for p, r in zip(points, radii):
        v = BRepBuilderAPI_MakeVertex(gp_Pnt(*p)).Vertex()
        if r is None:
            if welds is not None:
                welds.append(ob.weld_distances(v, edges))
            continue
        d_all = d_out = float("inf")
        for e, free in edges:
            dist = BRepExtrema_DistShapeShape(v, e)
            dist.Perform()
            if not dist.IsDone():
                continue
            d = dist.Value()
            d_all = min(d_all, d)
            if free:
                d_out = min(d_out, d)
        worst_all = min(worst_all, d_all / r)
        worst_out = min(worst_out, d_out / r)
    if worst_all == float("inf"):      # ボルト点が無い(全部溶接)
        return None, None
    return round(worst_all, 4), round(worst_out, 4)


def _structure(meta: dict) -> dict:
    from synthetic_generator.batch_generate import structure_of
    from synthetic_generator.general_geometry import plan_for
    from synthetic_generator.variants import spec_from_meta
    spec, _b, _f, _r = spec_from_meta(meta)
    custom = any(getattr(spec, k, None) is not None
                 for k in ("branch", "channel", "drawn", "box", "panel"))
    custom = custom or getattr(spec, "plate_margin_mm", None) is not None
    plan = None
    if not custom:
        try:
            plan = plan_for(spec)
        except ValueError:
            plan = None
    return structure_of(spec, plan, meta.get("kind", ""))


def work_part(args) -> tuple[str, str]:
    chunk, pid = args
    chunk = pathlib.Path(chunk)
    path = chunk / "params" / f"{pid}.json"
    meta = json.load(io.open(path, encoding="utf-8"))
    st = dict(meta.get("structure") or _structure(meta))
    pts = _points(meta)
    stp = chunk / "mid" / f"{pid}_mid.stp"
    if stp.exists() and pts:
        try:
            radii, source = _radii(meta, len(pts))
            welds: list = []
            st["seat_ratio_min"], st["seat_ratio_outline_min"] = seat_ratios(stp, pts, radii, welds)
            st["seat_ratio_definition"] = "min(dist(point, nearest edge) / bearing_radius)"
            st["bearing_radii"] = [None if r is None else round(r, 4) for r in radii]
            if welds:     # スポット溶接(半径 None)は座面比から外し、縁距離の実測を渡す
                st["seat_ratio_scope"] = "bolt points only (spot welds excluded)"
                st["spot_weld_d_edge_mm"] = [round(e, 3) for e, _b in welds]
                st["spot_weld_d_bend_mm"] = [round(b_, 3) for _e, b_ in welds]
            st["bearing_radii_source"] = source
        except Exception as exc:     # 読めない STEP は値を入れず理由を残す
            st["seat_ratio_error"] = str(exc)[:120]
    meta["structure"] = st
    path.write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    return pid, "ok"


def work_variant(path: str) -> str:
    p = pathlib.Path(path)
    meta = json.load(io.open(p, encoding="utf-8"))
    if "structure" in meta:
        return "skip"
    try:
        meta["structure"] = _structure(meta)
    except Exception as exc:
        meta["structure"] = {"error": str(exc)[:120]}
    p.write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    return "ok"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("chunks", nargs="*")
    ap.add_argument("--workers", type=int, default=max(1, mp.cpu_count() - 1))
    a = ap.parse_args()
    chunks = [pathlib.Path(c) for c in a.chunks] or sorted((ROOT / "synthetic_parts").glob("*/chunk_*"))
    with mp.Pool(a.workers) as pool:
        for chunk in chunks:
            ids = [p.stem for p in sorted((chunk / "params").glob("*.json"))]
            done = pool.map(work_part, [(str(chunk), i) for i in ids], chunksize=8)
            vfiles = [str(p) for p in sorted((chunk / "variants" / "params").glob("*.json"))]
            vdone = pool.map(work_variant, vfiles, chunksize=64) if vfiles else []
            ratios = []
            for i in ids:
                m = json.load(io.open(chunk / "params" / f"{i}.json", encoding="utf-8"))
                v = m["structure"].get("seat_ratio_min")
                if v is not None:
                    ratios.append(v)
            ratios.sort()
            med = ratios[len(ratios) // 2] if ratios else float("nan")
            low = sum(1 for v in ratios if v < 1.0)
            print(f"{chunk.parent.name}/{chunk.name}: params {len(done)}  variants {len(vdone)}  "
                  f"seat_ratio_min 中央 {med:.3f} 最小 {min(ratios) if ratios else float('nan'):.3f}  "
                  f"<1.0 が {low}", flush=True)


if __name__ == "__main__":
    main()

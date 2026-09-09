"""同一締結点・同一 spec の変種どうしで、外形が締結点からの距離に応じてどれだけ散るかを測る。

AMS 依頼 7 §7(2026-09-10): モデルの角の誤差は最近傍の締結点からの距離で決まる
(近い 0.014〜0.032u、遠い 0.05〜0.07u)。それが**入力の限界**(締結点から遠い部分は
設計の自由度)なのか**モデルの精度**なのかを分けるには、同一入力で生成器を再実行した
部品群の散らばりが要る。variants/ がまさにそれなので、ここで測る。

  u        = 締結点の間隔(各点から最近傍の点までの距離の平均)
  散らばり = 元の部品と変種の外形どうしの距離(両方向。片側だと要素を削った変種で 0 になる)

使い方: python tools/measure_variant_spread.py [チャンク名 ...]
        既定は occt18 occt20。出力は最近傍の締結点までの距離(u 単位)で層別した中央値と p90。
"""

import collections
import glob
import io
import json
import math
import pathlib
import random
import statistics as st
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] /
                       "synthetic_generator" / "src"))
import synthetic_generator.occt_build as ob
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GCPnts import GCPnts_UniformAbscissa
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE
from OCC.Core.TopExp import topexp
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCC.Core.TopoDS import topods

SAMPLE_MM = 3.0          # 外形をこの間隔でサンプルする


def outline_points(path):
    shape = ob._read_step(str(path))
    amap = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, amap)
    pts = []
    for i in range(1, amap.Size() + 1):
        if amap.FindFromIndex(i).Size() != 1:
            continue
        curve = BRepAdaptor_Curve(topods.Edge(amap.FindKey(i)))
        disc = GCPnts_UniformAbscissa(curve, SAMPLE_MM)
        if not disc.IsDone():
            continue
        for k in range(1, disc.NbPoints() + 1):
            p = curve.Value(disc.Parameter(k))
            pts.append((p.X(), p.Y(), p.Z()))
    return pts


def spacing(points):
    if len(points) < 2:
        return 1.0
    out = []
    for i, p in enumerate(points):
        out.append(min(math.dist(p, q) for j, q in enumerate(points) if j != i))
    return st.mean(out)


def one_sided(a, b):
    """a の各点から b までの最短距離。"""
    return [min(math.dist(p, q) for q in b) for p in a]


def main():
    fams = sys.argv[1:] or ["occt18", "occt20"]
    for fam in fams:
        base = pathlib.Path("synthetic_parts") / fam / "chunk_01"
        mf = base / "variants" / "manifest.json"
        if not mf.exists():
            print(f"MISS {fam}")
            continue
        man = json.load(io.open(mf, encoding="utf-8"))
        ids = sorted(man["parts"])
        random.Random(11).shuffle(ids)
        bins = collections.defaultdict(list)
        n_pairs = 0
        for pid in ids:
            if n_pairs >= 40:
                break
            oks = [e for e in man["parts"][pid] if e.get("status") == "ok"]
            if len(oks) < 2:
                continue
            meta = json.load(io.open(base / "params" / f"{pid}.json", encoding="utf-8"))
            pts = [tuple(p["position_xyz"])
                   for p in (meta["spec"].get("annotated_points") or [])]
            if not pts:
                pts = [tuple(meta["spec"][k]["position_xyz"]) for k in ("point1", "point2")]
            u = spacing(pts)
            try:
                a = outline_points(base / "mid" / f"{pid}_mid.stp")
            except Exception:
                continue
            for e in oks[:6]:
                vp = base / "variants" / (e["name"] + "_mid.stp")
                if not vp.exists():
                    continue
                try:
                    b = outline_points(vp)
                except Exception:
                    continue
                # 両方向で測る(片側だと「要素を削った変種」で 0 になる)
                for src, dst in ((b, a), (a, b)):
                    for p, d in zip(src, one_sided(src, dst)):
                        r = min(math.dist(p, q) for q in pts) / u
                        bins[min(int(r * 2) / 2.0, 2.0)].append(d / u)
                n_pairs += 1
        print(f"RES {fam}: 対 {n_pairs}")
        for key in sorted(bins):
            v = bins[key]
            print("BIN %.1f-%.1f median %.4f p90 %.4f n %d"
                  % (key, key + 0.5, st.median(v), sorted(v)[int(0.9 * len(v))], len(v)))


if __name__ == "__main__":
    main()

"""実車の中立面STEPを、合成側でパラメータ化しやすいかで評価する(2026-09-04)。

締結点が少ない部品を拾い、その形状が「板を数回曲げた帯」に近いか(=いまの
2点締結ファミリの延長で作れるか)を測る。見ているのは:

* 締結点の数と種類(weld / bolt / mounting_hole。`other_hole` は締結ではないので別勘定)
* 面の種類の内訳 — 平面と円筒(曲げ)だけで出来ていれば解析的に再現できる。
  B-spline(自由曲面)が多い部品は絞りや複雑な絞り形状で、パラメータ化しにくい
* 外形の細長さ(bbox の長辺/短辺)— 帯状なら2点締結の骨格にそのまま乗る
* 曲げ半径の種類数 — 少ないほどパラメータが素直

使い方: python tools/survey_real_parts.py [締結数の上限]
"""
from __future__ import annotations

import collections
import json
import pathlib
import sys

from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.GProp import GProp_GProps
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods

ROOT = pathlib.Path(r"C:\Users\hide2\IdeaBox\PartMaker\fill_mid_surf")
ASSEMBLIES = ("A0072600002_AllCATPart", "A0072600529_AllCATPart", "A0072601285_AllCATPart")
GEO = {0: "平面", 1: "円筒", 2: "円錐", 3: "球", 4: "トーラス", 5: "bezier",
       6: "bspline", 7: "回転", 8: "押出", 9: "offset"}
FASTENERS = ("weld", "bolt", "mounting_hole")


def measure(path: pathlib.Path) -> dict | None:
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != 1:
        return None
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape is None or shape.IsNull():
        return None
    kinds: collections.Counter = collections.Counter()
    radii = []
    props = GProp_GProps()
    area = 0.0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = topods.Face(explorer.Current())
        surface = BRepAdaptor_Surface(face)
        kinds[GEO.get(surface.GetType(), str(surface.GetType()))] += 1
        if surface.GetType() == 1:
            radii.append(round(surface.Cylinder().Radius(), 1))
        brepgprop.SurfaceProperties(face, props)
        area += props.Mass()
        explorer.Next()
    box = Bnd_Box()
    brepbndlib.Add(shape, box)
    x0, y0, z0, x1, y1, z1 = box.Get()
    extents = sorted([x1 - x0, y1 - y0, z1 - z0], reverse=True)
    total = sum(kinds.values())
    return {
        "faces": total,
        "kinds": dict(kinds.most_common()),
        "free_form": (kinds.get("bspline", 0) + kinds.get("bezier", 0)) / max(1, total),
        "analytic": (kinds.get("平面", 0) + kinds.get("円筒", 0)) / max(1, total),
        "area_cm2": area / 100.0,
        "extents": [round(e) for e in extents],
        "slenderness": extents[0] / max(1e-6, extents[1]),
        "bend_radii": sorted(set(r for r in radii if 1.0 <= r <= 60.0)),
    }


def main() -> None:
    cap = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    rows = []
    for asm in ASSEMBLIES:
        annotation = ROOT / asm / "annotations" / "joints.json"
        if not annotation.exists():
            continue
        data = json.loads(annotation.read_text(encoding="utf-8"))
        per: dict = collections.defaultdict(collections.Counter)
        for joint in data["joints"]:
            for part_id in joint["parts"]:
                per[part_id][joint["type"]] += 1
        for entry in data["parts"]:
            counts = per.get(entry["part_id"], collections.Counter())
            fasteners = sum(counts[k] for k in FASTENERS)
            if not 2 <= fasteners <= cap:
                continue
            path = ROOT / asm / entry.get("fill_file", "")
            if not path.exists():
                continue
            shape = measure(path)
            if shape is None:
                continue
            rows.append({"asm": asm, "part": entry["part_id"], "fasteners": fasteners,
                         "counts": dict(counts), "thickness": entry.get("thickness_mm"),
                         "path": path, **shape})

    # 「作りやすさ」= 解析曲面の割合が高く、面数が少なく、細長い
    for row in rows:
        row["score"] = (row["analytic"] * 2.0
                        + min(1.0, 40.0 / max(1, row["faces"]))
                        + min(1.0, row["slenderness"] / 4.0))
    rows.sort(key=lambda r: -r["score"])

    print(f"締結点(weld/bolt/mounting_hole)が 2〜{cap} 個の部品: {len(rows)}件\n")
    header = (f"{'部品':<22} {'締結':>4} {'面数':>5} {'解析%':>6} {'自由%':>6} "
              f"{'細長さ':>6} {'寸法mm':>18} {'面積cm2':>8} 曲げR")
    print(header)
    print("-" * len(header))
    for row in rows:
        name = f"{row['asm'][:11]}/{row['part']}"
        radii = ",".join(f"{r:g}" for r in row["bend_radii"][:5]) or "-"
        print(f"{name:<22} {row['fasteners']:>4} {row['faces']:>5} "
              f"{row['analytic']:>5.0%} {row['free_form']:>6.0%} "
              f"{row['slenderness']:>6.1f} {str(row['extents']):>18} "
              f"{row['area_cm2']:>8.0f} {radii}")
    print("\n上位候補の面の内訳:")
    for row in rows[:5]:
        print(f"  {row['asm'][:11]}/{row['part']}: {row['kinds']}  t={row['thickness']}mm")
        print(f"      {row['path'].relative_to(ROOT)}")


if __name__ == "__main__":
    main()

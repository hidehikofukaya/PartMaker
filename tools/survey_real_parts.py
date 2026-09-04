"""実車部品が合成側の「断面スイープ」で再現できるかを測る(2026-09-05に全面改訂)。

決め手は**共通折り軸**があること。合成側は「全ての主曲げ軸が1本の方向 w に平行で、
部品は w に直交する平面内の中心線に沿って断面を掃引したもの」しか作れない。
フランジの根本Rとビードの足Rは断面の一部として掃引されるので、その軸は必ず
**w に直交**する。したがって:

    全ての曲げ円筒の軸が w に平行か直交のどちらか  <=>  掃引モデルに収まる

w は「法線が最も違う2つの締結点」の外積から取る(合成側の構築と同じ取り方)。
法線が全部揃っている部品は平板なので w は不定 — その場合は曲げの軸から取る。

旧版は解析曲面の割合と細長さで並べていたが、それは収まるかどうかを決めない。

使い方: python tools/survey_real_parts.py [締結数の上限]
"""
from __future__ import annotations

import collections
import json
import math
import pathlib
import sys

from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.GProp import GProp_GProps
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods

ROOT = pathlib.Path(__file__).resolve().parent.parent / "fill_mid_surf"
FASTENERS = ("weld", "bolt", "mounting_hole")
PLANE, CYLINDER = 0, 1
BEND_RADIUS_RANGE_MM = (1.0, 60.0)   # これを外れる円筒は穴・大R曲面として除く
PARALLEL_DEG, ORTHOGONAL_DEG = 10.0, 10.0


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def unit(a):
    n = math.sqrt(dot(a, a))
    return tuple(x / n for x in a) if n > 1e-12 else None


def angle_deg(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(dot(a, b))))))


def joints_of(assembly: pathlib.Path):
    """部品ID -> [{type, pos, normal}] と、部品ID -> STEPのパス。"""
    document = json.loads((assembly / "annotations" / "joints.json").read_text(encoding="utf-8"))
    per: dict = collections.defaultdict(list)
    for joint in document["joints"]:
        axis = joint.get("axis") or {}
        normal = unit(tuple(axis.get("direction_xyz", (0.0, 0.0, 1.0))))
        for entry in joint.get("per_part", []):
            per[entry["part_id"]].append({
                "type": joint["type"], "normal": normal,
                "pos": entry.get("hole_center_xyz") or entry.get("contact_xyz"),
            })
    files, thickness = {}, {}
    for entry in document.get("parts", []):
        # アセンブリによってキーが違う(fill_file / stp_file)。
        name = entry.get("fill_file") or entry.get("stp_file")
        if name and (assembly / name).exists():
            files[entry["part_id"]] = assembly / name
            thickness[entry["part_id"]] = entry.get("thickness_mm")
    return per, files, thickness


def measure(path: pathlib.Path, normals):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != 1:
        return None
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape is None or shape.IsNull():
        return None

    kinds: collections.Counter = collections.Counter()
    # 自由曲面の割合は**面積**で測る。中立面の抽出で出る極小のブレンドが面数だと
    # 効きすぎる(5面のうち2面がb-splineでも面積では数%、ということが起きる)。
    area: collections.Counter = collections.Counter()
    props = GProp_GProps()
    axes, planes = [], []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = topods.Face(explorer.Current())
        surface = BRepAdaptor_Surface(face)
        kinds[surface.GetType()] += 1
        brepgprop.SurfaceProperties(face, props)
        area[surface.GetType()] += props.Mass()
        if surface.GetType() == CYLINDER:
            radius = surface.Cylinder().Radius()
            if BEND_RADIUS_RANGE_MM[0] <= radius <= BEND_RADIUS_RANGE_MM[1]:
                d = surface.Cylinder().Axis().Direction()
                axes.append((unit((d.X(), d.Y(), d.Z())), radius))
        elif surface.GetType() == PLANE:
            d = surface.Plane().Axis().Direction()
            planes.append(unit((d.X(), d.Y(), d.Z())))
        explorer.Next()

    # 折り軸 w の候補: 締結法線の外積(合成側と同じ取り方)と、各曲げの軸。
    # 締結法線が全部揃っている部品では外積が取れないので、曲げの軸も候補に入れて
    # **はぐれ軸が最少になる w** を選ぶ — 「共通折り軸が存在するか」を測りたいのであって、
    # 特定の取り方で決め打ちしたいわけではない。
    spread, preferred = 0.0, []
    for i, a in enumerate(normals):
        for b in normals[i + 1:]:
            if a and b and angle_deg(a, b) > 1.0:
                spread = max(spread, angle_deg(a, b))
                c = unit(cross(a, b))
                if c:
                    preferred.append(c)
    # 締結法線から取った w を先に試す(合成側と同じ取り方で、主曲げと断面Rの区別が
    # 物理的に正しくなる)。同点なら先勝ちなのでこの順序が効く。
    candidates = preferred + [a for a, _ in axes]
    if not axes:      # 曲げが無い = 平板。掃引モデルには自明に収まる。
        box = Bnd_Box()
        brepbndlib.Add(shape, box)
        x0, y0, z0, x1, y1, z1 = box.Get()
        return {"faces": sum(kinds.values()),
                "free_form": ((area.get(6, 0.0) + area.get(5, 0.0))
                              / max(1e-9, sum(area.values()))),
                "area_cm2": sum(area.values()) / 100.0,
                "normal_spread_deg": spread, "folds": 0, "fold_radii": [],
                "section_bends": 0, "stray": 0,
                "extents": [round(e) for e in sorted([x1-x0, y1-y0, z1-z0], reverse=True)]}
    if not candidates:
        return None

    best = None
    for w in candidates:
        parallel = [r for a, r in axes if angle_deg(a, w) < PARALLEL_DEG]
        orthogonal = [r for a, r in axes if angle_deg(a, w) > 90.0 - ORTHOGONAL_DEG]
        stray = len(axes) - len(parallel) - len(orthogonal)
        if best is None or stray < best[0]:
            best = (stray, parallel, orthogonal, w)
    stray, parallel, orthogonal, w = best

    box = Bnd_Box()
    brepbndlib.Add(shape, box)
    x0, y0, z0, x1, y1, z1 = box.Get()
    extents = sorted([x1 - x0, y1 - y0, z1 - z0], reverse=True)
    total = sum(kinds.values())
    return {
        "faces": total,
        "free_form": ((area.get(6, 0.0) + area.get(5, 0.0))
                      / max(1e-9, sum(area.values()))),
        "area_cm2": sum(area.values()) / 100.0,
        "normal_spread_deg": spread,
        "folds": len(parallel), "fold_radii": sorted({round(r, 1) for r in parallel}),
        "section_bends": len(orthogonal), "stray": stray,
        "extents": [round(e) for e in extents],
    }


def main() -> None:
    cap = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    rows = []
    for assembly in sorted(ROOT.iterdir()):
        if not (assembly / "annotations" / "joints.json").exists():
            continue
        per, files, thickness = joints_of(assembly)
        for part_id, joints in per.items():
            fasteners = [j for j in joints if j["type"] in FASTENERS]
            if not 2 <= len(fasteners) <= cap or part_id not in files:
                continue
            shape = measure(files[part_id], [j["normal"] for j in fasteners])
            if shape is None:
                continue
            rows.append({
                "asm": assembly.name[:11], "part": part_id,
                "n": len(fasteners),
                "types": "/".join(f"{k}{v}" for k, v in
                                  sorted(collections.Counter(j["type"] for j in fasteners).items())),
                "t": thickness.get(part_id), **shape,
            })

    # 収まるか = はぐれ軸が無く、自由曲面が少なく、曲げが少ない
    for row in rows:
        # はぐれ軸ゼロ = 掃引モデルに収まる。自由曲面は面積で1割まで許す。
        row["fits"] = row["stray"] == 0 and row["free_form"] < 0.10
    rows.sort(key=lambda r: (not r["fits"], r["stray"], r["free_form"], r["folds"]))

    header = (f"{'部品':<16}{'締結':>4} {'種別':<22}{'面数':>5}{'自由%':>6}"
              f"{'法線差':>7}{'主曲げ':>6}{'断面R':>6}{'はぐれ':>6}  {'寸法mm':<18} 判定")
    print(f"締結点が2〜{cap}個の部品: {len(rows)}件（注釈のあるアセンブリ全て）\n")
    print(header)
    print("-" * len(header))
    for row in rows:
        name = f"{row['asm']}/{row['part']}"
        verdict = "収まる" if row["fits"] else ("はぐれ軸" if row["stray"] else "自由曲面が多い")
        print(f"{name:<16}{row['n']:>4} {row['types']:<22}{row['faces']:>5}"
              f"{row['free_form']:>6.0%}{row['normal_spread_deg']:>7.1f}{row['folds']:>6}"
              f"{row['section_bends']:>6}{row['stray']:>6}  {str(row['extents']):<18} {verdict}")

    print(f"\n収まる: {sum(r['fits'] for r in rows)}件 / {len(rows)}件")
    print("\n主曲げのR（収まる部品）:")
    for row in rows:
        if row["fits"]:
            print(f"  {row['asm']}/{row['part']}: {row['fold_radii']}  t={row['t']}mm")


if __name__ == "__main__":
    main()

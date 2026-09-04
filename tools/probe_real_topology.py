"""実車部品を「共通折り軸まわりの断面スイープ」で再現できるか測る(2026-09-04)。

合成側(occt_build)は、全ての曲げ軸が1本の方向 w に平行で、部品は w に直交する
平面内の中心線に沿って断面を掃引したもの、という構造しか作れない。実車の007/011が
その枠に収まるかを、次の観点で実測する:

 * 共通折り軸 w — 全ての曲げ円筒の軸が平行か(ばらつきの角度)
 * 各締結点が乗る面と、その法線 — 「2点が一致・1点が異なる」の裏取り
 * パネルの並びと折れ角・曲げR — 合成側のパラメータ範囲と比べる
 * 各面の w方向の幅 / 断面内の長さ — 幅の狭い付属パネル = フランジの判定
 * 締結点の (w, 断面内) 座標 — 2点がどれだけ隣接し、3点目がどこに居るか

使い方: python tools/probe_real_topology.py [部品ID ...]   (既定: 007 011)
"""
from __future__ import annotations

import collections
import json
import math
import pathlib
import sys

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.GProp import GProp_GProps
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_VERTEX
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCC.Core.TopoDS import topods
from OCC.Core.gp import gp_Pnt

ROOT = pathlib.Path(__file__).resolve().parent.parent / "fill_mid_surf"
ASSEMBLY = "A0072600002_AllCATPart"
PLANE, CYLINDER = 0, 1


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return tuple(x / n for x in a) if n > 1e-12 else a


def angle_deg(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(unit(a), unit(b))))))


def canonical(d):
    """向きの符号をそろえる(軸の平行判定用)。"""
    for c in d:
        if abs(c) > 1e-9:
            return tuple(x * (1.0 if c > 0 else -1.0) for x in d)
    return d


def read_faces(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != 1:
        raise SystemExit(f"読めない: {path}")
    reader.TransferRoots()
    shape = reader.OneShape()

    faces = []
    props = GProp_GProps()
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = topods.Face(explorer.Current())
        surface = BRepAdaptor_Surface(face)
        kind = surface.GetType()
        brepgprop.SurfaceProperties(face, props)
        centre = props.CentreOfMass()
        flip = -1.0 if face.Orientation() == TopAbs_REVERSED else 1.0
        entry = {
            "face": face, "kind": kind, "area": props.Mass(),
            "centre": (centre.X(), centre.Y(), centre.Z()),
            "corners": face_corners(face),
        }
        if kind == PLANE:
            d = surface.Plane().Axis().Direction()
            entry["normal"] = unit((d.X() * flip, d.Y() * flip, d.Z() * flip))
        elif kind == CYLINDER:
            cyl = surface.Cylinder()
            d = cyl.Axis().Direction()
            entry["axis"] = unit((d.X(), d.Y(), d.Z()))
            entry["radius"] = cyl.Radius()
        faces.append(entry)
        explorer.Next()
    return shape, faces


def face_corners(face):
    pts = []
    explorer = TopExp_Explorer(face, TopAbs_VERTEX)
    while explorer.More():
        p = BRep_Tool.Pnt(topods.Vertex(explorer.Current()))
        pts.append((p.X(), p.Y(), p.Z()))
        explorer.Next()
    return pts


def adjacency(shape, faces):
    """辺を共有する面の組を返す(索引の集合)。"""
    index = {f["face"]: i for i, f in enumerate(faces)}
    mapping = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, mapping)
    pairs = set()
    for i in range(1, mapping.Size() + 1):
        owners = [index.get(topods.Face(f)) for f in mapping.FindFromIndex(i)]
        owners = sorted(o for o in owners if o is not None)
        for a in range(len(owners)):
            for b in range(a + 1, len(owners)):
                pairs.add((owners[a], owners[b]))
    return pairs


def nearest_face(faces, point):
    vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(*point)).Shape()
    best, best_d = None, 1e18
    for i, f in enumerate(faces):
        d = BRepExtrema_DistShapeShape(vertex, f["face"])
        d.Perform()
        if d.IsDone() and d.Value() < best_d:
            best, best_d = i, d.Value()
    return best, best_d


def joints_for(part_id):
    raw = json.loads((ROOT / ASSEMBLY / "annotations" / "joints.json").read_text(encoding="utf-8"))
    out = []
    for joint in raw["joints"]:
        if part_id not in joint.get("parts", []):
            continue
        pos = None
        for entry in joint.get("per_part", []):
            if entry.get("part_id") == part_id:
                pos = entry.get("hole_center_xyz") or entry.get("contact_xyz")
        out.append({"id": joint["joint_id"], "type": joint["type"], "pos": pos,
                    "axis": unit(tuple(joint["axis"]["direction_xyz"])),
                    "dia": next((e.get("hole_diameter_mm") for e in joint["per_part"]
                                 if e.get("part_id") == part_id), None),
                    "partners": [p for p in joint["parts"] if p != part_id]})
    return out


def probe(part_id):
    path = next((ROOT / ASSEMBLY / "fill").glob(f"{part_id}_*.stp"))
    shape, faces = read_faces(path)
    joints = joints_for(part_id)

    print("=" * 78)
    print(f"部品 {part_id}   面 {len(faces)}   {path.name[:56]}")
    print("=" * 78)

    # --- 共通折り軸 w -------------------------------------------------------
    # 主曲げ = 面積が最大の曲げ。フランジ根本Rは小さく細いので基準にしない。
    bends = sorted((f for f in faces if f["kind"] == CYLINDER),
                   key=lambda f: -f["area"])
    if bends:
        ref = canonical(bends[0]["axis"])
        spread = [angle_deg(ref, canonical(b["axis"])) for b in bends]
        print(f"曲げ円筒 {len(bends)}枚  軸の平行性: 最大ずれ {max(spread):5.2f}deg "
              f"-> {'共通折り軸あり' if max(spread) < 5.0 else '共通折り軸なし(掃引不可)'}")
        w = ref
    else:
        w = (1.0, 0.0, 0.0)
        print("曲げ円筒なし")
    print(f"  w = ({w[0]:+.4f}, {w[1]:+.4f}, {w[2]:+.4f})")

    # 断面内の基底
    seed = (0.0, 0.0, 1.0) if abs(w[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = unit(cross(w, seed))
    v = unit(cross(w, u))

    def local(p):
        return (dot(p, w), dot(p, u), dot(p, v))

    # --- 面ごとの寸法 -------------------------------------------------------
    print()
    print(f"{'#':>2} {'種別':<6} {'面積mm2':>8} {'w幅':>7} {'断面長':>7} {'R/法線':<40} 備考")
    print("-" * 78)
    for i, f in enumerate(faces):
        ws = [local(p)[0] for p in f["corners"]] or [0.0]
        w_span = max(ws) - min(ws)
        # 断面内の広がり
        sec = [local(p)[1:] for p in f["corners"]] or [(0.0, 0.0)]
        sec_span = max(math.dist(a, b) for a in sec for b in sec) if len(sec) > 1 else 0.0
        if f["kind"] == PLANE:
            n = f["normal"]
            desc = f"n=({n[0]:+.2f},{n[1]:+.2f},{n[2]:+.2f}) wと{angle_deg(n, w):5.1f}deg"
            kind = "平面"
        elif f["kind"] == CYLINDER:
            a = f["axis"]
            desc = f"R={f['radius']:.2f} axis=({a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f})"
            kind = "曲げ"
        else:
            desc, kind = "", str(f["kind"])
        note = "細幅(フランジ候補)" if f["kind"] == PLANE and w_span < 20.0 else ""
        print(f"{i:>2} {kind:<6} {f['area']:>8.0f} {w_span:>7.1f} {sec_span:>7.1f} {desc:<40} {note}")

    # --- パネルの繋がりと折れ角 --------------------------------------------
    print()
    print("曲げでつながるパネル(折れ角 = 法線間の角):")
    pairs = adjacency(shape, faces)
    for bi, bend in enumerate(faces):
        if bend["kind"] != CYLINDER:
            continue
        touching = [j for (a, b) in pairs for j in (a, b)
                    if (a == faces.index(bend) or b == faces.index(bend))
                    and j != faces.index(bend) and faces[j]["kind"] == PLANE]
        if len(touching) == 2:
            n1, n2 = faces[touching[0]]["normal"], faces[touching[1]]["normal"]
            print(f"  面{touching[0]} --[曲げ{bi} R{bend['radius']:.1f}]-- 面{touching[1]}"
                  f"   折れ角 {angle_deg(n1, n2):5.1f}deg")
        else:
            print(f"  曲げ{bi} R{bend['radius']:.1f}: 接する平面 {touching}(2枚でない)")

    # --- 締結点 -------------------------------------------------------------
    print()
    print("締結点:")
    hosts = []
    for j in joints:
        idx, dist = nearest_face(faces, j["pos"])
        hosts.append(idx)
        lw, lu, lv = local(j["pos"])
        host = faces[idx]
        n = host.get("normal")
        agree = f"面法線と{angle_deg(j['axis'], n):5.1f}deg" if n else "(曲げ面上)"
        print(f"  {j['id']} {j['type']:<14} 面{idx}(距離{dist:5.2f}mm) {agree}"
              f"  局所(w={lw:8.1f}, u={lu:8.1f}, v={lv:8.1f})"
              f"  {'d=' + format(j['dia'], '.1f') if j['dia'] else '溶接'}")

    print()
    print("締結点どうし:")
    for a in range(len(joints)):
        for b in range(a + 1, len(joints)):
            pa, pb = joints[a]["pos"], joints[b]["pos"]
            d = tuple(y - x for x, y in zip(pa, pb))
            dw, du, dv = local(d)
            print(f"  {joints[a]['id']}-{joints[b]['id']}: 距離{norm(d):6.1f}mm "
                  f"(w方向{dw:+7.1f} / 断面内{math.hypot(du, dv):6.1f})  "
                  f"法線どうし {angle_deg(joints[a]['axis'], joints[b]['axis']):5.1f}deg "
                  f"{'<= 一致' if angle_deg(joints[a]['axis'], joints[b]['axis']) < 10 else ''}"
                  f"{'  同一面' if hosts[a] == hosts[b] else ''}")
    print()


def main():
    for part_id in (sys.argv[1:] or ["007", "011"]):
        probe(part_id)


if __name__ == "__main__":
    main()

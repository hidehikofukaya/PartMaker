"""部品を掃引方向に直交する平面で切り、断面プロファイルを出す(2026-09-04)。

合成側は「wに直交する平面内の中心線に沿って断面を掃引する」構造なので、実車を
同じ向きで切れば、合成の断面(基準面・フランジ壁・ビード)とそのまま突き合わせられる。
「両側にフランジ、真ん中にビード」といった特徴の有無は、面の一覧より断面で見る方が速い。

切る向き: 締結点の法線 n とその点が乗るパネルから w(折れ目軸)を決め、u = n x w を
断面の法線にする。プロファイルは (帯幅方向の位置, 基準面からの高さ) で出す。

使い方: python tools/probe_section_profile.py <部品ID> [切る締結点の番号]
"""
from __future__ import annotations

import collections
import json
import math
import pathlib
import sys

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods
from OCC.Core.gp import gp_Dir, gp_Pln, gp_Pnt

ROOT = pathlib.Path(__file__).resolve().parent.parent / "fill_mid_surf"
ASSEMBLY = "A0072600002_AllCATPart"
SAMPLES_PER_EDGE = 40


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def unit(a):
    n = math.sqrt(dot(a, a))
    return tuple(x / n for x in a) if n > 1e-12 else a


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
                    "normal": unit(tuple(joint["axis"]["direction_xyz"]))})
    return out


def slice_profile(shape, origin, w, u, n):
    """origin を通り u を法線とする平面で切り、(帯幅方向, 高さ) の点列を返す。"""
    section = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(*origin), gp_Dir(*u)))
    section.Approximation(True)
    section.Build()
    if not section.IsDone():
        return []
    points = []
    explorer = TopExp_Explorer(section.Shape(), TopAbs_EDGE)
    while explorer.More():
        edge = topods.Edge(explorer.Current())
        if not BRep_Tool.Degenerated(edge):
            curve = BRepAdaptor_Curve(edge)
            lo, hi = curve.FirstParameter(), curve.LastParameter()
            for k in range(SAMPLES_PER_EDGE + 1):
                p = curve.Value(lo + (hi - lo) * k / SAMPLES_PER_EDGE)
                d = (p.X() - origin[0], p.Y() - origin[1], p.Z() - origin[2])
                points.append((dot(d, w), dot(d, n)))
        explorer.Next()
    return sorted(points)


def draw(points, width=76, height=17):
    """プロファイルをASCIIで描く(縦横比は無視、形の把握用)。"""
    if not points:
        return ["(断面が取れなかった)"]
    xs = [p[0] for p in points]; ys = [p[1] for p in points]
    x0, x1 = min(xs), max(xs); y0, y1 = min(ys), max(ys)
    grid = [[" "] * width for _ in range(height)]
    for x, y in points:
        c = int((x - x0) / max(1e-9, x1 - x0) * (width - 1))
        r = height - 1 - int((y - y0) / max(1e-9, y1 - y0) * (height - 1))
        grid[r][c] = "#"
    rows = ["".join(g) for g in grid]
    rows.append(f"  帯幅 {x0:+.1f} 〜 {x1:+.1f}mm (幅 {x1-x0:.1f})   "
                f"高さ {y0:+.1f} 〜 {y1:+.1f}mm (振幅 {y1-y0:.1f})")
    return rows


def main() -> None:
    part_id = sys.argv[1] if len(sys.argv) > 1 else "014"
    path = next((ROOT / ASSEMBLY / "fill").glob(f"{part_id}_*.stp"))
    reader = STEPControl_Reader()
    reader.ReadFile(str(path))
    reader.TransferRoots()
    shape = reader.OneShape()

    joints = joints_for(part_id)
    # 折れ目軸 w = 法線が異なる2点の外積(合成側の掃引の前提と同じ取り方)
    base = joints[0]["normal"]
    other = next((j["normal"] for j in joints if abs(dot(j["normal"], base)) < 0.9), None)
    w = unit(cross(base, other))
    print(f"部品 {part_id}   締結点 {len(joints)}   w = "
          f"({w[0]:+.3f}, {w[1]:+.3f}, {w[2]:+.3f})")

    wanted = sys.argv[2:] or [j["id"] for j in joints]
    for joint in joints:
        if joint["id"] not in wanted:
            continue
        n = joint["normal"]
        u = unit(cross(n, w))          # 掃引方向(断面の法線)
        print()
        print(f"--- {joint['id']} ({joint['type']}) を通る断面 "
              f"[法線 ({n[0]:+.2f},{n[1]:+.2f},{n[2]:+.2f})] ---")
        for row in draw(slice_profile(shape, joint["pos"], w, u, n)):
            print("   " + row)


if __name__ == "__main__":
    main()

"""ハブ + 複数の腕(それぞれ別の折り軸)が1枚のシェルに縫えるかを確かめる(2026-09-05)。

実車026の骨格: 平面のハブ(面1、1453mm2)から3本の腕が出て、うち2本は同じ軸(帯)、
1本が75度ずれた軸で分岐する。折れ角は 91.0 / 90.0 / 30.3 度、根本の幅 60 / 73 / 32mm、
腕の長さ 47 / 23 / 31mm、曲げR 6.8 / 7.2 / 24.2。

既存の掃引は「全ての曲げ軸が1方向に平行」しか作れないので、分岐は作れない。
ハブを平面1枚として作り、各腕を**その根本エッジを軸に**回転掃引すれば、腕ごとに
軸が違ってよい。共有エッジは構築上まったく同じものになるので縫合できるはず — を確かめる。

使い方: python tools/probe_branch_hub.py [試行数]
"""
from __future__ import annotations

import collections
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from OCC.Core.BRepBuilderAPI import (  # noqa: E402
    BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire,
)
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeRevol  # noqa: E402
from OCC.Core.TopoDS import topods  # noqa: E402
from OCC.Core.gp import gp_Ax1, gp_Dir, gp_Pln, gp_Pnt, gp_Vec  # noqa: E402

import synthetic_generator.occt_build as ob  # noqa: E402


def unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def build(hub_xy, arms, out_dir, name):
    """hub_xy = ハブ多角形(z=0平面、反時計回り)。arms = {辺の番号: (折れ角deg, R, 腕長)}。"""
    faces: dict = {}
    normal = (0.0, 0.0, 1.0)
    pts = [(x, y, 0.0) for x, y in hub_xy]

    wire = BRepBuilderAPI_MakeWire()
    for i in range(len(pts)):
        wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*pts[i]),
                                         gp_Pnt(*pts[(i + 1) % len(pts)])).Edge())
    hub = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(*normal)), wire.Wire())
    if not hub.IsDone():
        raise ValueError("hub face failed")
    faces[hub.Face()] = "hub"

    for index, (angle_deg, radius, length) in arms.items():
        a, b = pts[index], pts[(index + 1) % len(pts)]
        axis_dir = unit((b[0] - a[0], b[1] - a[1], b[2] - a[2]))
        # 曲げ中心は根本エッジをハブ法線方向へ R ずらした線。折る側は法線の裏。
        centre = tuple(a[k] - radius * normal[k] for k in range(3))
        root = BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge()
        angle = math.radians(angle_deg)
        bend = BRepPrimAPI_MakeRevol(root, gp_Ax1(gp_Pnt(*centre), gp_Dir(*axis_dir)),
                                     angle).Shape()
        faces[topods.Face(bend)] = f"bend_{index}"
        # 腕は曲げの出口の接線方向へまっすぐ。出口の向きは -normal を angle 回した向き。
        outward = unit(cross(axis_dir, normal))          # ハブから外へ向かう向き
        tip = tuple(outward[k] * math.cos(angle) - normal[k] * math.sin(angle)
                    for k in range(3))
        # 曲げ後の根本エッジ(= 腕の付け根)
        moved_a = _rotate(a, centre, axis_dir, angle)
        moved_b = _rotate(b, centre, axis_dir, angle)
        edge = BRepBuilderAPI_MakeEdge(gp_Pnt(*moved_a), gp_Pnt(*moved_b)).Edge()
        arm = BRepPrimAPI_MakePrism(edge, gp_Vec(*(t * length for t in tip))).Shape()
        faces[topods.Face(arm)] = f"arm_{index}"

    shape, named = ob.OcctPartBuilder._sew(faces)
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stp = str(out_dir / f"{name}_mid.stp")
    ob._export_step(shape, named, stp)
    return stp, shape


def _rotate(p, centre, axis, angle):
    d = tuple(p[k] - centre[k] for k in range(3))
    c, s = math.cos(angle), math.sin(angle)
    dot = sum(d[k] * axis[k] for k in range(3))
    cr = cross(axis, d)
    return tuple(centre[k] + d[k] * c + cr[k] * s + axis[k] * dot * (1 - c) for k in range(3))


def main() -> None:
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    out = pathlib.Path(__file__).resolve().parent / "probe_output" / "branch"
    rng = random.Random(20260905)
    made = 0
    fails: collections.Counter = collections.Counter()
    for i in range(trials):
        # ハブ = 凸四角形。3辺に腕をつける(実車026は4辺のうち3辺)。
        w = rng.uniform(40.0, 80.0)
        h = rng.uniform(30.0, 70.0)
        skew = rng.uniform(-0.25, 0.25) * w
        hub = [(0.0, 0.0), (w, 0.0), (w + skew, h), (skew * 0.5, h)]
        arms = {}
        for edge in rng.sample(range(4), 3):
            arms[edge] = (rng.uniform(25.0, 110.0), rng.uniform(5.0, 20.0),
                          rng.uniform(20.0, 60.0))
        try:
            stp, shape = build(hub, arms, out, f"branch_{i:03d}")
            ob.check_shape(ob._read_step(stp))
            made += 1
        except Exception as exc:
            fails[f"{type(exc).__name__}: {str(exc)[:60]}"] += 1
    print(f"ハブ+腕3本: 成功 {made}/{trials} = {made / max(1, trials):.0%}")
    for k, v in fails.most_common(6):
        print(f"   {v:3d}  {k}")


if __name__ == "__main__":
    main()

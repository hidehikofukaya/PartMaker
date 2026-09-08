"""実車部品に「既存族の特徴」がいくつ入っているかを測る(2026-09-08)。

`tools/survey_real_parts.py` が「掃引モデルに収まるか」を測るのに対し、こちらは
**採用したい部品を選ぶ**ための検出器。結果は docs/PICK_real_parts_002_1285_2026-09-08.md。

族と特徴の対応:
  fold   主曲げ >=1            plain / bead / flange / rib（掃引の土台）
  fold3  主曲げ >=3            多段折り
  wall   細長い壁(フランジ)     flange / channel_seat の壁
  bead   小R断面が >=4         bead / rib
  arm    はぐれ軸 >=1          branch / compose の腕（自分の根元回りに折れる）
  drawn  トーラス/球のブレンド  drawn_tray（絞り隅）
  tab    小さな張り出し + 溶接  tab_bracket
  seat   向かい合う平行な座面   channel_seat
  norm3  締結法線 >=3 方向     多面締結
  mixed  締結種別 >=2          bolt/weld/mounting_hole の混在

使い方:
    python tools/pick_real_parts.py <出力JSON> [アセンブリ名 ...]
        アセンブリ名の既定は A0072600002_AllCATPart A0072601285_AllCATPart
"""

import collections
import json
import math
import pathlib
import sys

sys.path.insert(0, "tools")
import survey_real_parts as sp
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.GProp import GProp_GProps
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods

PLANE, CYL, CONE, SPHERE, TORUS, BEZ, BSPL = 0, 1, 2, 3, 4, 5, 6
DONE = {("A0072600002", p) for p in ("007", "011", "014", "026", "031", "057")} | \
       {("A0072601285", p) for p in ("18", "20")}


def face_table(shape):
    props = GProp_GProps()
    out = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = topods.Face(ex.Current())
        s = BRepAdaptor_Surface(f)
        brepgprop.SurfaceProperties(f, props)
        box = Bnd_Box()
        brepbndlib.Add(f, box)
        x0, y0, z0, x1, y1, z1 = box.Get()
        e = sorted([x1 - x0, y1 - y0, z1 - z0], reverse=True)
        rec = {"face": f, "kind": s.GetType(), "area": props.Mass(), "ext": e,
               "cen": ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), "axis": None}
        if s.GetType() == CYL:
            d = s.Cylinder().Axis().Direction()
            rec["axis"] = sp.unit((d.X(), d.Y(), d.Z()))
            rec["r"] = s.Cylinder().Radius()
        elif s.GetType() == PLANE:
            d = s.Plane().Axis().Direction()
            rec["axis"] = sp.unit((d.X(), d.Y(), d.Z()))
        out.append(rec)
        ex.Next()
    return out


def detect(path, joints):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != 1:
        return None
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape is None or shape.IsNull():
        return None
    faces = face_table(shape)
    m = sp.measure(path, [j["normal"] for j in joints])
    if m is None:
        return None
    total = sum(f["area"] for f in faces) or 1.0

    adj = sp._adjacency(shape, faces)
    nbr = collections.defaultdict(set)
    for a, b in adj:
        nbr[a].add(b)
        nbr[b].add(a)

    # 壁(フランジ): 細長い平面が、曲げを介して自分より十分大きい平面につながる
    walls = []
    for i, f in enumerate(faces):
        if f["kind"] != PLANE or f["area"] < 60:
            continue
        L, W = f["ext"][0], f["ext"][1]
        if not (5 <= W <= 45 and L >= max(30, 2.2 * W)):
            continue
        hit = False
        for j in nbr[i]:
            if faces[j]["kind"] != CYL or not (1.0 <= faces[j].get("r", 99) <= 25):
                continue
            for k in nbr[j]:
                if k != i and faces[k]["kind"] == PLANE and faces[k]["area"] > 1.8 * f["area"]:
                    hit = True
                    break
            if hit:
                break
        if hit:
            walls.append(i)

    # ビード/リブ: 小R(<=9)の円筒が、細い帯の両側に付く
    small = [i for i, f in enumerate(faces)
             if f["kind"] == CYL and 1.0 <= f.get("r", 99) <= 9.0]
    small_set = set(small)
    strip = 0
    for i, f in enumerate(faces):
        if f["kind"] != PLANE or f["ext"][1] > 30:
            continue
        if sum(1 for j in nbr[i] if j in small_set) >= 2:
            strip += 1

    blend = sum(1 for f in faces if f["kind"] in (SPHERE, TORUS))
    blend_area = sum(f["area"] for f in faces if f["kind"] in (SPHERE, TORUS)) / total

    # 座面(channel_seat): 向かい合う小さな平面2枚が平行・逆向きで離れている
    seats = [i for i, f in enumerate(faces)
             if f["kind"] == PLANE and f["area"] < 0.25 * total and f["ext"][0] < 120
             and f["area"] > 100]
    seat_pair = 0
    for a in range(len(seats)):
        for b in range(a + 1, len(seats)):
            fa, fb = faces[seats[a]], faces[seats[b]]
            if fa["axis"] and fb["axis"] and sp.angle_deg(fa["axis"], fb["axis"]) < 8:
                d = [fa["cen"][k] - fb["cen"][k] for k in range(3)]
                off = abs(sp.dot(d, fa["axis"]))
                lat = math.sqrt(max(0.0, sp.dot(d, d) - off * off))
                if off > 15 and lat < 80:
                    seat_pair += 1

    # タブ: 小さな面(面積<800、最大寸法<45)に溶接点が載る
    tabs = 0
    for j in joints:
        if j["type"] != "weld" or not j["pos"]:
            continue
        p = j["pos"]
        near = min(faces, key=lambda f: sum((f["cen"][k] - p[k]) ** 2 for k in range(3)))
        if near["area"] < 800 and near["ext"][0] < 45:
            tabs += 1

    normals = [j["normal"] for j in joints if j["normal"]]
    dirs = []
    for n in normals:
        if not any(sp.angle_deg(n, d) < 15 for d in dirs):
            dirs.append(n)

    feats = {
        "fold": m["folds"] >= 1,
        "fold3": m["folds"] >= 3,
        "wall": len(walls) >= 1,
        "bead": strip >= 1 and len(small) >= 4,
        "arm": m["stray"] >= 1,
        "drawn": blend >= 1 and blend_area > 0.005,
        "tab": tabs >= 1,
        "seat": seat_pair >= 1,
        "norm3": len(dirs) >= 3,
        "mixed": len({j["type"] for j in joints}) >= 2,
    }
    return {**m, "feats": feats, "score": sum(feats.values()), "walls": len(walls),
            "strips": strip, "small_r": len(small), "blend": blend,
            "blend_area": blend_area, "seat_pair": seat_pair, "tabs": tabs,
            "dirs": len(dirs)}


def main():
    root = pathlib.Path("fill_mid_surf")
    rows = []
    names = sys.argv[2:] or ["A0072600002_AllCATPart", "A0072601285_AllCATPart"]
    for asm_name in names:
        asm = root / asm_name
        per, files, thick = sp.joints_of(asm)
        for pid, joints in sorted(per.items()):
            if pid not in files:
                continue
            try:
                d = detect(files[pid], joints)
            except Exception as exc:
                print("ERR", asm_name[:11], pid, exc, flush=True)
                continue
            if d is None:
                continue
            d.update(asm=asm.name[:11], part=pid, n=len(joints), t=thick.get(pid),
                     types="/".join(f"{k}{v}" for k, v in sorted(
                         collections.Counter(j["type"] for j in joints).items())),
                     done=(asm.name[:11], pid) in DONE,
                     file=str(files[pid]))
            rows.append(d)
            print(f"{d['asm'][-4:]}/{pid:<4} n={d['n']:<3} score={d['score']} "
                  f"{','.join(k for k, v in d['feats'].items() if v)}", flush=True)
    out = pathlib.Path(sys.argv[1])
    out.write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    print("wrote", out, len(rows))


if __name__ == "__main__":
    main()

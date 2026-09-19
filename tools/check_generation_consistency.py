"""変種が「今の部品」と同じ世代で作られているかを照合する(AMS 依頼 8、2026-09-19)。

AMS 側で起きた事故(旧世代の部品と新世代の変種・params を組にしていた)がこちら側に
無いかを見る。変種の params は元の部品の spec を写して作るので、締結点が元の部品と
一致していれば同じ世代、ずれていれば**古い世代の部品に対して作られた変種**である。

照合項目(部品ごと):
  1. 変種の締結点(位置・法線)が元の部品と一致するか(1e-6 mm)
  2. 変種の STEP が元の部品の STEP より新しいか(mtime)
  3. params の generation.generated_at が manifest と一致するか(刻印後)

使い方: python tools/check_generation_consistency.py [チャンクのパス ...]
        既定は synthetic_parts/*/chunk_*。問題があれば終了コード 1。
"""
from __future__ import annotations

import collections
import io
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]


def points_of(meta: dict):
    spec = meta["spec"]
    pts = spec.get("annotated_points") or [spec["point1"], spec["point2"]]
    return [(tuple(p["position_xyz"]), tuple(p["normal_xyz"])) for p in pts]


def same_points(a, b, tol=1e-6) -> bool:
    if len(a) != len(b):
        return False
    return all(math.dist(p, q) < tol and math.dist(n, m) < tol for (p, n), (q, m) in zip(a, b))


def check_chunk(chunk: pathlib.Path) -> dict:
    out = collections.Counter()
    bad_examples = []
    man_path = chunk / "manifest.json"
    manifest = json.loads(man_path.read_text(encoding="utf-8")) if man_path.exists() else {}
    gen_at = manifest.get("generated_at")
    base_params = {p.stem: p for p in (chunk / "params").glob("*.json")}
    for pid, path in base_params.items():
        meta = json.load(io.open(path, encoding="utf-8"))
        g = (meta.get("generation") or {}).get("generated_at")
        if gen_at is not None:
            out["params_stamped" if g == gen_at else "params_stamp_mismatch"] += 1
    vman_path = chunk / "variants" / "manifest.json"
    if not vman_path.exists():
        return {"chunk": str(chunk), "variants": 0, **out}
    vman = json.load(io.open(vman_path, encoding="utf-8"))
    for pid, entries in vman["parts"].items():
        if pid not in base_params:
            out["variant_of_missing_part"] += 1
            continue
        base_meta = json.load(io.open(base_params[pid], encoding="utf-8"))
        base_pts = points_of(base_meta)
        base_stp = chunk / "mid" / f"{pid}_mid.stp"
        base_mtime = base_stp.stat().st_mtime if base_stp.exists() else None
        for e in entries:
            if e.get("status") != "ok":
                continue
            vp = chunk / "variants" / "params" / f"{e['name']}.json"
            vs = chunk / "variants" / f"{e['name']}_mid.stp"
            out["variants"] += 1
            if not vp.exists():
                out["variant_params_missing"] += 1
                continue
            vmeta = json.load(io.open(vp, encoding="utf-8"))
            if not same_points(points_of(vmeta), base_pts):
                out["points_differ"] += 1
                if len(bad_examples) < 3:
                    bad_examples.append(e["name"])
            if base_mtime is not None and vs.exists() and vs.stat().st_mtime < base_mtime:
                out["variant_older_than_part"] += 1
    return {"chunk": str(chunk).replace("\\", "/"), **out, "examples": bad_examples}


def main() -> int:
    args = [pathlib.Path(a) for a in sys.argv[1:]]
    chunks = args or sorted((ROOT / "synthetic_parts").glob("*/chunk_*"))
    problems = 0
    for c in chunks:
        r = check_chunk(c)
        bad = (r.get("points_differ", 0) + r.get("variant_older_than_part", 0)
               + r.get("variant_of_missing_part", 0) + r.get("params_stamp_mismatch", 0))
        problems += bad
        print(json.dumps(r, ensure_ascii=False))
    print(f"TOTAL_PROBLEMS {problems}")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())

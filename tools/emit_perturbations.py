"""締結点摂動ペアを作る(2026-09-06、AutoMetalSheet 依頼 B)。

部品 X の締結点を 1 つ選んで面内に動かし(座面半径の 0.5〜2 倍)、他の設計選択は
そのまま再生成した X' と、面ラベルの差分を記録する:

    <chunk>/perturb/<part_id>__p<k>_<dx>_<dy>_mid.stp
    <chunk>/perturb/<part_id>__p<k>_<dx>_<dy>.json
        {source_part_id, point_index, moved_from, moved_to, vector_xyz, joints,
         faces: {changed, unchanged, added, removed}}
    <chunk>/perturb/manifest.json

対象は 2 点族(bead/flange/rib/plain)と 007型(three_point)。011型/014型は掃引の
アンカーが実点ではなく、実点だけ動かしても面が変わらないので対象外(要アンカー再導出)。

    python tools/emit_perturbations.py synthetic_parts/occt11/chunk_01 --parts 100 --per-part 3
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "synthetic_generator" / "src"))

from synthetic_generator.occt_build import OcctPartBuilder  # noqa: E402
from synthetic_generator.variants import (  # noqa: E402
    build_variant, face_diff, part_rng, perturb_spec,
)

PERTURBABLE = {"bead", "flange", "rib", "plain", "three_point"}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("chunk")
    ap.add_argument("--parts", type=int, default=100)
    ap.add_argument("--per-part", type=int, default=3)
    ap.add_argument("--attempts", type=int, default=6, help="1 摂動あたりの引き直し上限")
    a = ap.parse_args()

    chunk = pathlib.Path(a.chunk)
    out = chunk / "perturb"
    out.mkdir(exist_ok=True)
    manifest_path = out / "manifest.json"
    manifest = (json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists()
                else {"schema": "partmaker_perturb/1", "parts": {}})
    ids = [p.stem for p in sorted((chunk / "params").glob("*.json"))]
    builder = OcctPartBuilder()
    done_parts = built = failed = 0
    t0 = time.time()
    for pid in ids:
        if done_parts >= a.parts:
            break
        meta = json.loads((chunk / "params" / f"{pid}.json").read_text(encoding="utf-8"))
        if meta["kind"] not in PERTURBABLE:
            continue
        before = json.loads((chunk / "features" / f"{pid}.json").read_text(encoding="utf-8"))["faces"]
        rng = part_rng(pid, "perturb")
        entries = manifest["parts"].get(pid, [])
        made = len([e for e in entries if e.get("status") == "ok"])
        tries = 0
        while made < a.per_part and tries < a.per_part * a.attempts:
            tries += 1
            got = perturb_spec(meta, rng)
            if got is None:
                break
            index, vector, variant, original = got
            name = f"{pid}__p{index}_{vector[0]:+.1f}_{vector[1]:+.1f}"
            if (out / f"{name}.json").exists():
                continue
            spec = variant.spec
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    part = build_variant(builder, variant, str(out), name)
            except Exception as exc:
                entries.append({"name": name, "status": "infeasible", "point_index": index,
                                "vector_xyz": list(vector), "reason": str(exc)[:200]})
                failed += 1
                continue
            moved = (spec.point1, spec.point2)[index]
            points = spec.annotated_points or (spec.point1, spec.point2, *spec.extra_points)
            record = {
                "schema": "partmaker_perturb/1", "part_id": name, "source_part_id": pid,
                "kind": meta["kind"], "point_index": index,
                "moved_from": list(original.position_xyz), "moved_to": list(moved.position_xyz),
                "vector_xyz": list(vector),
                "distance_over_bearing": (sum(c * c for c in vector) ** 0.5
                                          / spec.min_bearing_radius_mm),
                "joints": [{"position_xyz": list(p.position_xyz), "normal_xyz": list(p.normal_xyz)}
                           for p in points],
                "faces": face_diff(before, list(part.face_labels)),
                "face_labels": list(part.face_labels),
            }
            (out / f"{name}.json").write_text(json.dumps(record, ensure_ascii=False), encoding="utf-8")
            entries.append({"name": name, "status": "ok", "point_index": index,
                            "vector_xyz": list(vector), "file": f"perturb/{name}_mid.stp",
                            "changed_faces": len(record["faces"]["changed"]),
                            "added": len(record["faces"]["added"]),
                            "removed": len(record["faces"]["removed"])})
            made += 1
            built += 1
        manifest["parts"][pid] = entries
        done_parts += 1
        if done_parts % 10 == 0:
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            print(f"{chunk.parent.name} {done_parts}/{a.parts}: built {built}, failed {failed}, "
                  f"{time.time() - t0:.0f}s", flush=True)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
    print(f"{chunk}: parts {done_parts}, pairs {built}, infeasible {failed} -> {out}")


if __name__ == "__main__":
    main()

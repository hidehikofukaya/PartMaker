"""設計等価バリアントを既存チャンクに追加する(2026-09-06、AutoMetalSheet 依頼 A)。

締結点(位置・法線・座面半径)と板厚・曲げRを固定したまま、生成器が入力から決めていない
選択(フランジの側/高さ/根本R、折り位置、帯幅、ビード断面、リブ、余白、隅R、腕の長さ…)を
変えた部品を 1 部品あたり最大 --count 個作り、チャンクの隣に置く:

    <chunk>/variants/<part_id>__<knob>=<value>_mid.stp     形状
    <chunk>/variants/params/<name>.json                    spec + knob の記録
    <chunk>/variants/features/<name>.json                  面ラベル(features v2 の faces[])
    <chunk>/variants/<name>.infeasible                     不成立の記録(理由)
    <chunk>/variants/manifest.json                         部品ごとの一覧

既存の mid/ params/ features/ annotations/ は触らない。再実行は済んだ名前を飛ばす。

    python tools/emit_variants.py synthetic_parts/occt11/chunk_01 [--count 8]
        [--ids ID ...] [--holdout-first tools/_val_ids.json] [--limit N]
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
    build_variant, part_rng, propose_variants, variant_meta,
)

GENERATOR_VERSION = "2.5.0"


def load_manifest(path: pathlib.Path) -> dict:
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {"schema": "partmaker_variants/1", "generator_version": GENERATOR_VERSION,
            "note": "同じ締結点・座面半径・板厚・曲げRに対する等価な設計。knob と value が"
                    "何を変えたか、changed が前後の値。status=infeasible は成立しない組み合わせ",
            "parts": {}}


def order_ids(chunk: pathlib.Path, ids, holdout_path: str | None) -> list[str]:
    all_ids = [p.stem for p in sorted((chunk / "params").glob("*.json"))]
    if ids:
        return list(ids)
    if holdout_path:
        hold = json.loads(pathlib.Path(holdout_path).read_text(encoding="utf-8"))
        first = [i for i in hold.get(chunk.parent.name, []) if i in set(all_ids)]
        rest = [i for i in all_ids if i not in set(first)]
        return first + rest
    return all_ids


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("chunk")
    ap.add_argument("--count", type=int, default=8)
    ap.add_argument("--ids", nargs="*", default=None)
    ap.add_argument("--holdout-first", default=str(ROOT / "tools" / "_val_ids.json"))
    ap.add_argument("--limit", type=int, default=None, help="先頭から N 部品だけ")
    a = ap.parse_args()

    chunk = pathlib.Path(a.chunk)
    out = chunk / "variants"
    (out / "params").mkdir(parents=True, exist_ok=True)
    (out / "features").mkdir(exist_ok=True)
    manifest_path = out / "manifest.json"
    manifest = load_manifest(manifest_path)
    ids = order_ids(chunk, a.ids, a.holdout_first)
    if a.limit:
        ids = ids[:a.limit]

    builder = OcctPartBuilder()
    built = infeasible = skipped = 0
    t0 = time.time()
    for n, pid in enumerate(ids, 1):
        meta = json.loads((chunk / "params" / f"{pid}.json").read_text(encoding="utf-8"))
        try:
            variants = propose_variants(meta, count=a.count, rng=part_rng(pid))
        except Exception as exc:  # 候補が出せない(古い params 等)
            manifest["parts"][pid] = [{"status": "error", "reason": str(exc)[:200]}]
            continue
        # 既存の記録は残す(候補の規則が変わっても、ディスクにある変種を manifest から落とさない)。
        entries = [e for e in manifest["parts"].get(pid, [])
                   if e.get("name") and ((out / f"{e['name']}_mid.stp").exists()
                                         or (out / f"{e['name']}.infeasible").exists())]
        known = {e["name"] for e in entries}
        for v in variants:
            name = v.name(pid)
            if name in known:
                continue
            stp = out / f"{name}_mid.stp"
            bad = out / f"{name}.infeasible"
            entry = {"name": name, "knob": v.knob, "value": v.value, "changed": v.changed}
            if stp.exists():
                entry.update(status="ok", file=f"variants/{stp.name}")
                skipped += 1
            elif bad.exists():
                entry.update(status="infeasible", reason=bad.read_text(encoding="utf-8")[:200])
                skipped += 1
            else:
                try:
                    with contextlib.redirect_stdout(io.StringIO()):
                        part = build_variant(builder, v, str(out), name)
                    (out / "params" / f"{name}.json").write_text(
                        json.dumps(variant_meta(meta, v, name), ensure_ascii=False, indent=1),
                        encoding="utf-8")
                    (out / "features" / f"{name}.json").write_text(
                        json.dumps({"schema": "partmaker_features/2", "part_id": name,
                                    "source_part_id": pid, "kind": meta["kind"],
                                    "knob": v.knob, "value": v.value,
                                    "faces": list(part.face_labels)}, ensure_ascii=False),
                        encoding="utf-8")
                    entry.update(status="ok", file=f"variants/{stp.name}")
                    built += 1
                except Exception as exc:
                    reason = str(exc)[:300]
                    bad.write_text(reason, encoding="utf-8")
                    entry.update(status="infeasible", reason=reason[:200])
                    infeasible += 1
            entries.append(entry)
        manifest["parts"][pid] = entries
        if n % 25 == 0 or n == len(ids):
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            print(f"{chunk.parent.name} {n}/{len(ids)}: built {built}, infeasible {infeasible}, "
                  f"skipped {skipped}, {time.time() - t0:.0f}s", flush=True)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
    print(f"{chunk}: built {built}, infeasible {infeasible}, skipped {skipped} -> {out}")


if __name__ == "__main__":
    main()

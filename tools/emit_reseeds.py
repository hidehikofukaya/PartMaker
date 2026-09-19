"""同一入力(締結点 + spec)で生成器を別シードで再実行した部品群を作る(AMS 依頼 8 ⑤、2026-09-19)。

目的: 「締結点から遠い角の誤差は情報の限界か」を測る。knob を 1 つずつ動かす変種は元の部品の
近くに留まりやすいので、**締結点が決めていない生成器の乱数を全部同時に引き直した**部品を作る。
今は分岐族(occt18)だけ対応(`variants.reseed_branch`: 腕の長さと、ハブの隅R)。

出力は既存の変種の契約にそのまま載せる(knob = "reseed"、value = "s1".."sN"):

    <chunk>/variants/<part_id>__reseed=s<k>_mid.stp
    <chunk>/variants/params/<part_id>__reseed=s<k>.json      generation は元の部品の世代
    <chunk>/variants/features/<part_id>__reseed=s<k>.json
    <chunk>/variants/manifest.json                          knob="reseed" のエントリ

対象の部品は既定でホールドアウト(`tools/_val_ids.json` の族のキー、30 部品)。一覧に無い族は
20 部品おきの 30 部品(0001, 0021, …, 0581)。

使い方: python tools/emit_reseeds.py synthetic_parts/occt18/chunk_01 [--seeds 5] [--ids ...]
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "synthetic_generator" / "src"))
sys.path.insert(0, str(ROOT / "tools"))

import _generations  # noqa: E402
from synthetic_generator.occt_build import OcctPartBuilder  # noqa: E402
from synthetic_generator.variants import (  # noqa: E402
    Variant, build_variant, part_rng, reseed_branch, spec_from_meta, variant_meta,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("chunk")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--ids", nargs="*", default=None)
    a = ap.parse_args()

    chunk = pathlib.Path(a.chunk)
    family = chunk.parent.name
    out = chunk / "variants"
    for sub in ("params", "features"):
        (out / sub).mkdir(parents=True, exist_ok=True)
    base_manifest = json.loads((chunk / "manifest.json").read_text(encoding="utf-8"))
    base_gen, base_at = base_manifest.get("generation"), base_manifest.get("generated_at")
    man_path = out / "manifest.json"
    manifest = json.loads(man_path.read_text(encoding="utf-8"))
    if manifest.get("base_generated_at") not in (None, base_at):
        raise SystemExit(f"{out} の変種は別の世代の部品に対して作られている。")
    emitted_at = _generations.now_iso()

    holdout = json.loads((ROOT / "tools" / "_val_ids.json").read_text(encoding="utf-8"))
    # 一覧に無い族(occt18 など)は他族のホールドアウトと同じ 20 部品おき
    ids = a.ids or holdout.get(family) or [
        p.stem for p in sorted((chunk / "params").glob("*.json"))][::20][:30]
    builder = OcctPartBuilder()
    built = infeasible = 0
    for pid in ids:
        meta = json.loads((chunk / "params" / f"{pid}.json").read_text(encoding="utf-8"))
        spec, bead, flange, rib = spec_from_meta(meta)
        if spec.branch is None:
            raise SystemExit(f"{pid}: 別シードの再実行は今は分岐族だけ対応")
        entries = [e for e in manifest["parts"].get(pid, []) if e.get("knob") != "reseed"]
        for k in range(1, a.seeds + 1):
            rng = part_rng(pid, salt=f"reseed{k}")
            new = reseed_branch(spec, rng)
            changed = {"branch.arms.length_mm": [[x["length_mm"] for x in spec.branch["arms"]],
                                                  [x["length_mm"] for x in new.branch["arms"]]],
                       "branch.corner_radius": [spec.branch["corner_radius"],
                                                new.branch["corner_radius"]]}
            v = Variant("reseed", f"s{k}", new, bead, flange, rib, changed)
            name = v.name(pid)
            entry = {"name": name, "knob": "reseed", "value": f"s{k}", "changed": changed,
                     "seed_salt": f"reseed{k}"}
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    part = build_variant(builder, v, str(out), name)
                vmeta = variant_meta(meta, v, name)
                vmeta["generation"] = _generations.stamp(base_gen, base_at,
                                                         variant_emitted_at=emitted_at)
                (out / "params" / f"{name}.json").write_text(
                    json.dumps(vmeta, ensure_ascii=False, indent=1), encoding="utf-8")
                (out / "features" / f"{name}.json").write_text(
                    json.dumps({"schema": "partmaker_features/2", "part_id": name,
                                "source_part_id": pid, "kind": meta["kind"],
                                "knob": "reseed", "value": f"s{k}",
                                "faces": list(part.face_labels)}, ensure_ascii=False),
                    encoding="utf-8")
                entry.update(status="ok", file=f"variants/{name}_mid.stp")
                built += 1
            except Exception as exc:
                (out / f"{name}.infeasible").write_text(str(exc)[:300], encoding="utf-8")
                entry.update(status="infeasible", reason=str(exc)[:200])
                infeasible += 1
            entries.append(entry)
        manifest["parts"][pid] = entries
    manifest.update(base_generation=base_gen, base_generated_at=base_at)
    man_path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
    _generations.append_ledger({
        "event": "reseed", "family": family, "chunk": chunk.name,
        "base_generation": base_gen, "base_generated_at": base_at, "emitted_at": emitted_at,
        "parts": len(ids), "seeds": a.seeds, "built": built, "infeasible": infeasible,
        "git_commit": _generations.git_commit()})
    print(f"{family}/{chunk.name}: reseed {len(ids)} 部品 x {a.seeds} シード -> "
          f"built {built}, infeasible {infeasible}")


if __name__ == "__main__":
    main()

"""既存チャンクに世代を刻む(AMS 依頼 8、2026-09-19)。形状(STEP)には触らない。

世代管理を入れる前に作ったチャンクには `generation` / `generated_at` が無い。
ここで次を書き足す:

  manifest.json            generation, generated_at, git_commit=None, supersedes=[],
                           generation_backfilled=True
  params/<id>.json         generation = {n, generated_at, backfilled}
  variants/manifest.json   base_generation, base_generated_at
  variants/params/*.json   generation = {n, generated_at, backfilled}
  GENERATIONS.jsonl        event="backfill" を 1 チャンク 1 行

`generated_at` は `_COMPLETE` の更新時刻(= 今ディスクにある部品を作り終えた時刻)。
`generation` は族ごとの連番で、**同じチャンク名に上書きで作り直した回数**を数える
(2026-09-19 時点で分かっている履歴から。下の BACKFILL_GENERATION)。並存するチャンク
(occt30 の chunk_01 と chunk_02)は置き換えではないので、族の中の通し番号として 1, 2 を振る。

実行前に `tools/check_generation_consistency.py` で「変種が今の部品と同じ締結点で作られて
いること」を確かめておくこと(2026-09-19: 全 14 チャンク・変種 49,877 本で問題 0)。

使い方: python tools/backfill_generation.py [--dry-run]
"""
from __future__ import annotations

import io
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import _generations  # noqa: E402

# 同じチャンク名に上書きで作り直した回数(= 今のデータの世代番号)。根拠は各 PLAN / REPLY 文書と
# AMS 依頼 8 §1 の表。ここに無いチャンクは 1。
BACKFILL_GENERATION = {
    ("occt20", "chunk_01"): 2,   # 09-06 に作り直し(AMS の手元は 09:16、こちらの現行は 10:28)
    ("occt21", "chunk_01"): 2,   # 09-06 初版 -> 09-09 座面契約で作り直し
    ("occt22", "chunk_01"): 5,   # 09-08 600部品 -> 3群1,800 -> 第2期 -> 09-09 座面契約 -> 締結点上限16
    ("occt23", "chunk_01"): 3,   # 09-08 初版 -> 09-09 座面契約 -> 座面余白1.15倍
    ("occt30", "chunk_02"): 2,   # chunk_01 と並存(置き換えではない)。族の通し番号として 2
}
NOTE = ("2026-09-19 に後から刻んだ世代。generated_at は _COMPLETE の更新時刻。"
        "generation は同じチャンク名に上書きで作り直した回数(文書の記録からの推定)")


def rewrite(path: pathlib.Path, update, dry: bool) -> None:
    data = json.load(io.open(path, encoding="utf-8"))
    update(data)
    if not dry:
        indent = 1 if path.parent.name == "params" else None
        path.write_text(json.dumps(data, ensure_ascii=False, indent=indent), encoding="utf-8")


def main() -> None:
    dry = "--dry-run" in sys.argv
    for chunk in sorted(_generations.PARTS.glob("*/chunk_*")):
        family, name = chunk.parent.name, chunk.name
        man_path = chunk / "manifest.json"
        complete = chunk / "_COMPLETE"
        if not man_path.exists() or not complete.exists():
            print(f"SKIP {family}/{name}: 未完")
            continue
        manifest = json.load(io.open(man_path, encoding="utf-8"))
        if manifest.get("generated_at") and not manifest.get("generation_backfilled"):
            print(f"SKIP {family}/{name}: 生成時に刻印済み(generation {manifest['generation']})")
            continue
        gen = BACKFILL_GENERATION.get((family, name), 1)
        at = _generations.iso_of_mtime(complete)
        stamp = _generations.stamp(gen, at, backfilled=True)

        def upd_manifest(m):
            m.update(generation=gen, generated_at=at, git_commit=m.get("git_commit"),
                     supersedes=m.get("supersedes", []), generation_backfilled=True,
                     generation_note=NOTE)
        rewrite(man_path, upd_manifest, dry)

        n_params = 0
        for p in sorted((chunk / "params").glob("*.json")):
            rewrite(p, lambda m: m.__setitem__("generation", stamp), dry)
            n_params += 1

        n_var = 0
        vman = chunk / "variants" / "manifest.json"
        if vman.exists():
            rewrite(vman, lambda m: m.update(base_generation=gen, base_generated_at=at), dry)
            for p in sorted((chunk / "variants" / "params").glob("*.json")):
                rewrite(p, lambda m: m.__setitem__("generation", stamp), dry)
                n_var += 1

        if not dry:
            _generations.append_ledger({
                "event": "backfill", "family": family, "chunk": name, "generation": gen,
                "generated_at": at, "backfilled_at": _generations.now_iso(),
                "n_params": n_params, "n_variant_params": n_var, "note": NOTE})
        print(f"{'DRY ' if dry else ''}{family}/{name}: generation {gen}  generated_at {at}  "
              f"params {n_params}  variant params {n_var}")


if __name__ == "__main__":
    main()

"""生成物の世代管理(AMS 依頼 8、2026-09-19)。

背景: AMS の取り込みは「出力があればスキップ」する再開仕様だったため、同じ ID で
再生成された族は、部品(STEP)が旧世代のまま、変種と params だけ新世代という組になった
(occt20〜23)。世代を**明示**して、どのファイルがどの世代かを機械的に照合できるようにする。

- チャンクの `manifest.json` に `generation`(族ごとの連番)・`generated_at`(UTC)・`git_commit`
- 各 `params/<id>.json` に `generation: {"n", "generated_at"}`
- `variants/manifest.json` に `base_generation` / `base_generated_at`(どの世代の部品に対して作ったか)
- 追記専用の台帳 `synthetic_parts/GENERATIONS.jsonl`(1 行 1 イベント)= 「再生成の一報」の機械可読版

方針: **再生成は新しいチャンク番号で行う**(既存チャンクは上書きしない)。`run_occt_batch.py` は
既存の非空チャンクへの書き込みを拒否する。
"""
from __future__ import annotations

import datetime
import json
import pathlib
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
PARTS = ROOT / "synthetic_parts"
LEDGER = PARTS / "GENERATIONS.jsonl"


def now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def iso_of_mtime(path: pathlib.Path) -> str:
    return datetime.datetime.fromtimestamp(path.stat().st_mtime,
                                           datetime.timezone.utc).isoformat(timespec="seconds")


def git_commit() -> str | None:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                             capture_output=True, text=True, timeout=10)
        dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"],
                               cwd=ROOT, capture_output=True, text=True, timeout=10)
        sha = out.stdout.strip()
        return (sha + ("+dirty" if dirty.stdout.strip() else "")) if sha else None
    except Exception:
        return None


def read_ledger() -> list[dict]:
    if not LEDGER.exists():
        return []
    out = []
    for line in LEDGER.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            out.append(json.loads(line))
    return out


def next_generation(family: str) -> int:
    """その族で次に使う世代番号(台帳にある最大 + 1)。"""
    gens = [e.get("generation", 0) for e in read_ledger()
            if e.get("family") == family and e.get("event") in ("chunk", "backfill")]
    return max(gens, default=0) + 1


def append_ledger(entry: dict) -> None:
    PARTS.mkdir(parents=True, exist_ok=True)
    with LEDGER.open("a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def stamp(generation: int, generated_at: str, **extra) -> dict:
    """params に入れる世代の印。"""
    return {"n": generation, "generated_at": generated_at, **extra}

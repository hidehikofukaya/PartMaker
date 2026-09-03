"""フランジ部品だけを、多様性クォータ付きで1チャンク生成する(2026-08-26、SS18)。

既存491件のフランジ部品を分析したところ、**高さが10-12mmに50.7%集中**していた
(18-20mmは5.9%)。原因は凹側跨ぎの制約 h <= 基準面R - 2 で、基準面Rが10-23mmなので
上限が低く出やすいこと。到達可能性を実測すると高さ帯の比は 11:4:2:1.4:1 だったので、
リジェクションで均等化できる(純Python 46件/秒なので棄却は実質無料)。

配置クラスは gentle モードで parallel_same_offset 94% が構造的な上限。
orthogonal は原理的に不可能(法線90度は2つの折れで45度以上を要求するが、
フランジ条件は最大折れ角20度以下)。それでも oblique / coplanar_flat には
枠を確保して、parallel_same_offset 一色になるのを防ぐ。

使い方:
  python tools/run_flange_chunk.py <ルート名> <チャンク番号> <部品数> <seed>
"""
from __future__ import annotations

import json
import pathlib
import sys
import time
from collections import Counter

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.batch_generate import DEFAULT_OUTPUT_ROOT, generate_general_batch  # noqa: E402
from synthetic_generator.classify import classify  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

HEIGHT_BANDS = ((10.0, 12.0), (12.0, 14.0), (14.0, 16.0), (16.0, 18.0), (18.0, 100.0))
PSO = "parallel_same_offset"


def band_of(height_mm: float) -> str:
    for lo, hi in HEIGHT_BANDS:
        if lo <= height_mm < hi:
            return f"h{lo:.0f}-{min(hi, 20):.0f}"
    return "h?"


def build_quota(count: int) -> dict[tuple[str, str], int]:
    """高さ帯を均等化しつつ、各帯にクラス多様性の枠を確保する。"""
    per_band = count // len(HEIGHT_BANDS)
    other = max(1, round(per_band * 0.18))   # 実測到達率から18%が上限に近い
    quota = {}
    for lo, hi in HEIGHT_BANDS:
        b = f"h{lo:.0f}-{min(hi, 20):.0f}"
        quota[(b, "other")] = other
        quota[(b, "pso")] = per_band - other
    return quota


def main() -> None:
    root_name, chunk = sys.argv[1], int(sys.argv[2])
    count, seed = int(sys.argv[3]), int(sys.argv[4])
    out_dir = DEFAULT_OUTPUT_ROOT / root_name / f"chunk_{chunk:02d}"
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"{out_dir} は既に存在する。")

    # 第5引数にJSONを渡すとクォータを上書きできる(不足分の穴埋め用)。
    # キーは "帯|グループ" 形式(例 "h10-12|pso")。
    if len(sys.argv) > 5:
        raw = json.loads(pathlib.Path(sys.argv[5]).read_text(encoding="utf-8"))
        quota = {tuple(k.split("|")): v for k, v in raw.items()}
    else:
        quota = build_quota(count)
    produced: Counter = Counter()
    stats = {"seen": 0, "bead": 0, "rejected": 0}

    def key_of(spec, flange):
        cls = str(classify(spec.point1, spec.point2))
        return (band_of(flange.height_mm), "pso" if cls == PSO else "other")

    def accept(spec, bead, flange) -> bool:
        """判定のみ。**カウントはしない** — CATIAビルドが失敗した候補で枠を
        食い潰さないため(SS18で実測した不具合)。"""
        stats["seen"] += 1
        if flange is None:            # フランジ専用チャンクなのでビードは採らない
            stats["bead"] += 1
            return False
        if produced[key_of(spec, flange)] >= quota.get(key_of(spec, flange), 0):
            stats["rejected"] += 1
            return False
        return True

    def built(spec, bead, flange) -> None:
        if flange is not None:
            produced[key_of(spec, flange)] += 1

    builder = SyntheticPartBuilder()
    builder.catia.DisplayFileAlerts = False
    for i in range(builder.catia.Documents.Count, 0, -1):
        try:
            builder.catia.Documents.Item(i).Close()
        except Exception:
            pass

    print(f"chunk_{chunk:02d}: フランジ{count}部品 / seed {seed}", flush=True)
    print(f"クォータ: {dict(sorted(quota.items()))}", flush=True)
    t0 = time.time()
    records = generate_general_batch(
        builder, out_dir, count=count, seed=seed,
        reinforcement_probability=1.0,
        flange_aim_share=1.0,          # 常にフランジ帯狙い
        max_attempts_per_part=8000,    # 希少セルは純Pythonで数千回引く(1回~20ms)
        accept_filter=accept,
        on_part_built=built,
    )
    dt = time.time() - t0
    got = Counter()
    cls_c = Counter()
    for r in records:
        got[band_of(r.flange.height_mm)] += 1
        cls_c[str(classify(r.spec.point1, r.spec.point2))] += 1
    print(f"\nCHUNK DONE: {len(records)}部品 / {dt/60:.1f}分 ({dt/len(records):.1f}秒/部品)",
          flush=True)
    print(f"  純Python: {stats['seen']}候補 (ビード{stats['bead']} / クォータ外{stats['rejected']})",
          flush=True)
    print(f"  高さ帯: {dict(sorted(got.items()))}", flush=True)
    print(f"  配置クラス: {dict(cls_c.most_common())}", flush=True)
    print(f"  出力: {out_dir}", flush=True)
    print("\nCATIAを再起動してから次のチャンクを実行してください。", flush=True)


if __name__ == "__main__":
    main()

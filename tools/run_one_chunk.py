"""1チャンクだけ生成して終了する(2026-08-26、ユーザー指定の運用)。

CATIAセッションは時間とともに重くなる(SS5.1: 15.6->27.5秒/部品、メモリ795MB->1.78GB)。
チャンクごとに実行を止めてユーザーがCATIAを再起動する運用にしたため、
`run_long_production.py`(時間予算で複数チャンクを積む)ではなくこちらを使う。

既存ルートの続きを作れるよう、**そのチャンクのディレクトリだけ**の存在を確認する
(ルート全体ではない — run_long_productionはルートが空でないと拒否する)。

使い方:
  python tools/run_one_chunk.py <ルート名> <チャンク番号> <部品数> <seed> [補強確率] [クォータJSON]
例:
  python tools/run_one_chunk.py prod02 3 250 20263903 1.0 tools/quota_prod02.json
"""
import json
import pathlib
import sys
import time
from collections import Counter

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.batch_generate import DEFAULT_OUTPUT_ROOT, generate_general_batch  # noqa: E402
from synthetic_generator.classify import classify  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402


def main() -> None:
    root_name = sys.argv[1]
    chunk = int(sys.argv[2])
    count = int(sys.argv[3])
    seed = int(sys.argv[4])
    reinforcement_probability = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
    class_quota = None
    if len(sys.argv) > 6:
        class_quota = json.loads(pathlib.Path(sys.argv[6]).read_text(encoding="utf-8"))

    out_dir = DEFAULT_OUTPUT_ROOT / root_name / f"chunk_{chunk:02d}"
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"{out_dir} は既に存在する。先に削除するか別のチャンク番号を使うこと。")

    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    for i in range(catia.Documents.Count, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass

    print(f"chunk_{chunk:02d}: {count}部品 / seed {seed} / 補強確率 {reinforcement_probability}"
          + (f" / クォータ {class_quota}" if class_quota else ""), flush=True)
    t0 = time.time()
    records = generate_general_batch(
        builder, out_dir, count=count, seed=seed,
        reinforcement_probability=reinforcement_probability,
        class_quota=class_quota,
    )
    dt = time.time() - t0

    kinds = Counter("flange" if r.flange is not None else "bead" for r in records)
    classes = Counter(str(classify(r.spec.point1, r.spec.point2)) for r in records)
    print(f"\nCHUNK DONE: {len(records)}部品 / {dt / 60:.1f}分 ({dt / len(records):.1f}秒/部品)", flush=True)
    print(f"  補強: {dict(kinds)}", flush=True)
    print(f"  配置クラス: {dict(classes.most_common())}", flush=True)
    if class_quota:
        print("  クォータ達成: " + "  ".join(
            f"{c}={classes.get(c, 0)}/{n}" for c, n in class_quota.items()), flush=True)
    print(f"  出力: {out_dir}", flush=True)
    print("\nCATIAを再起動してから次のチャンクを実行してください。", flush=True)


if __name__ == "__main__":
    main()

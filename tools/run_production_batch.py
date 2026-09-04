"""本番の学習データ生成(2026-08-25、SS13完了後の初回本番バッチ)。

`generate_general_batch`(リゾルバ・メタデータ保存込みの本番経路)をそのまま使う。
バッチごとに独立したサブディレクトリへ出力する — AnnotationDocumentのjoints.jsonは
出力先ごとに1つで、同じディレクトリへ再実行すると上書きされるため。

使い方: python tools/run_production_batch.py <batch名> <部品数> <seed> [補強確率]
例:     python tools/run_production_batch.py batch02 100 20260827 0.5
補強の種類(フランジ/ビード)は基準面の幾何で自動選択(最大折れ角20度以下ならフランジ)。
"""
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.batch_generate import DEFAULT_OUTPUT_ROOT, generate_general_batch  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402


def main() -> None:
    batch_name = sys.argv[1]
    count = int(sys.argv[2])
    seed = int(sys.argv[3])
    reinforcement_probability = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
    out_dir = DEFAULT_OUTPUT_ROOT / batch_name
    if (out_dir / "annotations" / "joints.json").exists():
        raise SystemExit(f"{out_dir} には既にバッチが存在する。別のbatch名を使うこと。")

    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    for i in range(catia.Documents.Count, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass

    t0 = time.time()
    records = generate_general_batch(
        builder, out_dir, count=count, seed=seed,
        reinforcement_probability=reinforcement_probability
    )
    dt = time.time() - t0
    with_bead = sum(1 for r in records if r.bead is not None)
    with_flange = sum(1 for r in records if r.flange is not None)
    print(f"BATCH DONE: {len(records)}部品 / {dt / 60:.1f}分 "
          f"(1部品 {dt / len(records):.1f}秒)  ビード {with_bead} / フランジ {with_flange} "
          f"/ 補強なし {len(records) - with_bead - with_flange}", flush=True)
    print(f"出力: {out_dir}", flush=True)


if __name__ == "__main__":
    main()

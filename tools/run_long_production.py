"""長時間の本番生成をチャンク分割で回す(2026-08-25)。

`generate_general_batch`はjoints.jsonをバッチ末尾でまとめて書くため、数時間の単発実行だと
終盤でDELMIAが落ちたときにアノテーションを全損する(CATPart/STP/paramsは逐次書き込みなので
残る)。チャンクごとに独立した出力ディレクトリとjoints.jsonを持たせ、事故時の損失を
1チャンクに限定する。

- 時間予算を見ながらチャンクを積む。残り時間が1チャンク分に満たなければそこで終了
- チャンクが失敗しても次へ進む。ただしCOM接続が死んだ場合(RPCサーバー利用不可)は
  以降どうせ全滅するので即座に中断する
- 進捗は逐次flushして書き出す(長時間実行なので途中経過が見えることが重要)

使い方:
  python tools/run_long_production.py <ルート名> <時間[h]> <チャンク部品数> <seed>
                                      [補強確率] [最大チャンク数] [クォータJSON]
例:
  python tools/run_long_production.py prod01 7 220 20260902 1.0
  python tools/run_long_production.py prod02 6 250 20260903 1.0 4 quota.json

クォータJSONは配置クラスごとの**チャンクあたり**の目標件数
(例 {"orthogonal": 75, "parallel_opposite": 62})。カバレッジの穴を狙って埋めるため
(SS16)。計上は実際に生成できたクラスで行うので、到達不能な目標でも止まらない。
"""
import pathlib
import sys
import time
import traceback

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.batch_generate import DEFAULT_OUTPUT_ROOT, generate_general_batch  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

CONNECTION_DEAD_MARKERS = ("RPC", "-2147023174", "サーバーを利用できません")


def main() -> None:
    root_name = sys.argv[1]
    budget_hours = float(sys.argv[2])
    chunk_count = int(sys.argv[3])
    base_seed = int(sys.argv[4])
    reinforcement_probability = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
    max_chunks = int(sys.argv[6]) if len(sys.argv) > 6 else 0   # 0 = 時間予算まで
    class_quota = None
    if len(sys.argv) > 7:
        import json
        class_quota = json.loads(pathlib.Path(sys.argv[7]).read_text(encoding="utf-8"))

    root = DEFAULT_OUTPUT_ROOT / root_name
    if root.exists() and any(root.iterdir()):
        raise SystemExit(f"{root} は既に存在する。別のルート名を使うこと。")

    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    for i in range(catia.Documents.Count, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass

    deadline = time.time() + budget_hours * 3600.0
    print(f"予算 {budget_hours}h / チャンク {chunk_count}部品 / 補強確率 "
          f"{reinforcement_probability}"
          + (f" / 最大{max_chunks}チャンク" if max_chunks else "")
          + (f" / クォータ {class_quota}" if class_quota else ""), flush=True)

    chunk = 0
    total_parts = 0
    total_bead = total_flange = 0
    per_part = 15.0  # 初期見積り。実測で更新する
    while True:
        remaining = deadline - time.time()
        needed = chunk_count * per_part
        if remaining < needed:
            print(f"\n残り {remaining / 60:.0f}分 < 1チャンク所要 {needed / 60:.0f}分 のため終了",
                  flush=True)
            break
        if max_chunks and chunk >= max_chunks:
            print(f"\n最大チャンク数 {max_chunks} に到達したため終了", flush=True)
            break
        chunk += 1
        out_dir = root / f"chunk_{chunk:02d}"
        seed = base_seed + chunk * 1000
        print(f"\n=== chunk_{chunk:02d} 開始 (seed {seed}, 残り {remaining / 3600:.2f}h) ===",
              flush=True)
        t0 = time.time()
        try:
            records = generate_general_batch(
                builder, out_dir, count=chunk_count, seed=seed,
                reinforcement_probability=reinforcement_probability,
                class_quota=class_quota,
            )
        except Exception as exc:
            text = f"{exc}"
            traceback.print_exc()
            if any(marker in text for marker in CONNECTION_DEAD_MARKERS):
                print("\n*** CATIA/DELMIAへの接続が失われた。以降のチャンクも失敗するため中断 ***",
                      flush=True)
                break
            print(f"chunk_{chunk:02d} は失敗したが接続は生きている。次のチャンクへ進む", flush=True)
            continue
        dt = time.time() - t0
        per_part = dt / max(1, len(records))
        beads = sum(1 for r in records if r.bead is not None)
        flanges = sum(1 for r in records if r.flange is not None)
        total_parts += len(records)
        total_bead += beads
        total_flange += flanges
        print(f"=== chunk_{chunk:02d} 完了: {len(records)}部品 / {dt / 60:.1f}分 "
              f"({per_part:.1f}秒/部品) ビード{beads} フランジ{flanges} "
              f"[累計 {total_parts}部品] ===", flush=True)

    print(f"\nPRODUCTION DONE: {total_parts}部品 / {chunk}チャンク  "
          f"ビード{total_bead} フランジ{total_flange}", flush=True)
    print(f"出力: {root}", flush=True)


if __name__ == "__main__":
    main()

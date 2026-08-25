"""SS12改修後の本番経路(generate_general_batch + リゾルバ + メタデータ)の実機スモーク。"""
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.batch_generate import generate_general_batch  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402

OUT = pathlib.Path(__file__).resolve().parent / "probe_output" / "smoke_ss13"


def main() -> None:
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
        builder, OUT, count=int(sys.argv[1]) if len(sys.argv) > 1 else 10,
        seed=20260825, bead_probability=1.0,
    )
    dt = time.time() - t0
    n = len(records)
    print(f"\n{n}部品 / {dt/60:.1f}分 = 10部品あたり {dt/n*10/60:.1f}分", flush=True)
    params = sorted((OUT / "params").glob("*.json"))
    print(f"paramsファイル: {len(params)}件", flush=True)
    with_bead = sum(1 for r in records if r.bead is not None)
    print(f"ビード付き: {with_bead}/{n}", flush=True)


if __name__ == "__main__":
    main()

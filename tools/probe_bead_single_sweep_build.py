"""閉曲線1本の単発スイープ方式に切り替えたビードを、本番経路で実機検証する(2026-08-25)。

SS8.10: 壁4枚+BiTangent×3でコーナーを作る旧方式は、最終段(2つのL字リボンの統合)が
向き・trim・relimitation・半径のいずれを振っても成立しなかった。コーナーRを外形曲線に
織り込み、1回のMode=4スイープでバンドを作る方式へ置き換えた。
`build_general_two_point(bead=...)`をそのまま呼び、①〜⑦の全工程が通るか見る。
"""
import pathlib
import random
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

OUTPUT_DIR = pathlib.Path(__file__).resolve().parent / "probe_output"
TARGET = 6


def main() -> None:
    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    n = catia.Documents.Count
    for i in range(n, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass
    print(f"closed {n} document(s)", flush=True)
    OUTPUT_DIR.mkdir(exist_ok=True)

    rng = random.Random(20260825)
    made = 0
    reasons: dict[str, int] = {}
    samples: dict[str, list[str]] = {}
    for attempt in range(1, 121):
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        t0 = time.time()
        try:
            generated = builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                out_dir=str(OUTPUT_DIR),
                part_name=f"bead_single_sweep_{attempt:03d}",
                bead=bead,
            )
        except ValueError as exc:
            key = str(exc).split(" -- ")[0][:70]
            reasons[key] = reasons.get(key, 0) + 1
            samples.setdefault(key, []).append(str(exc))
            continue
        except Exception as exc:
            import traceback
            print(f"[{attempt:03d}] *** RAW *** {type(exc).__name__}: {str(exc)[:130]}", flush=True)
            print(traceback.format_exc()[-1200:], flush=True)
            reasons["RAW FAILURE"] = reasons.get("RAW FAILURE", 0) + 1
            continue
        made += 1
        print(f"[{attempt:03d}] SUCCESS ({time.time()-t0:.1f}s) depth={bead.depth_mm:.1f} "
              f"angle={bead.wall_angle_deg:.1f} topW={bead.top_width_mm:.1f} "
              f"ridgeR={bead.ridge_radius_mm:.1f} cornerR={bead.corner_radius_mm:.1f}", flush=True)
        print(f"        {generated.catpart_path}", flush=True)
        if made >= TARGET:
            break

    print(f"\n=== 成功 {made} / 試行 {attempt} ===", flush=True)
    for key, count in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print(f"  {count:3d}  {key}", flush=True)
    print(chr(10) + "=== 代表メッセージ ===", flush=True)
    for key in ("bead wall band: no draft angle put the top edge at the expected positi",
                "bead: could not build a usable offset surface for the bead top (depth="):
        for msg in samples.get(key, [])[:2]:
            print(f"  * {msg[:300]}", flush=True)


if __name__ == "__main__":
    main()

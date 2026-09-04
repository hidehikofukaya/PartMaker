"""ビード付きバッチを回し、成功物も**失敗直前の形状**も残す(2026-08-25、ユーザー依頼)。

通常の生成経路は`_discard_failed_attempt`で失敗ドキュメントを保存せず閉じる
(学習材料の選別を邪魔しないため、SS6.10)。ここでは診断のためにそれを差し替え、
閉じる前にCATPartとして書き出す。失敗理由をファイル名に入れて分類できるようにする。

Python側の事前チェックで弾かれた試行(free_fold_seedの不成立など)はCATIAに触る前に
終わるので形状は存在しない — 出力されるのはCATIA段階まで進んだ失敗だけ。
"""
import pathlib
import random
import re
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent / "probe_output" / "bead_batch"
OK_DIR = ROOT / "success"
NG_DIR = ROOT / "failed"
ATTEMPTS = int(sys.argv[1]) if len(sys.argv) > 1 else 60


def slug(text: str) -> str:
    """失敗理由をファイル名にできる短い識別子へ。"""
    text = text.split(" -- ")[0]
    text = re.sub(r"[^A-Za-z ]+", "", text)[:60].strip().replace(" ", "_")
    return text or "unknown"


def _label(saved_paths, reason: str, attempt: int) -> None:
    """保存済みの失敗CATPartに、判明した失敗理由をファイル名として付ける。"""
    for path in saved_paths:
        if not path.exists():
            continue
        target = path.with_name(f"{path.stem}__{reason}.CATPart")
        try:
            path.replace(target)
            print(f"[{attempt:03d}] 失敗形状: {target.name}", flush=True)
        except Exception as exc:
            print(f"[{attempt:03d}] リネーム失敗: {str(exc)[:70]}", flush=True)


def main() -> None:
    for d in (OK_DIR, NG_DIR):
        d.mkdir(parents=True, exist_ok=True)

    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    n = catia.Documents.Count
    for i in range(n, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass
    print(f"closed {n} stale document(s)", flush=True)

    # 失敗ドキュメントを、閉じる前にCATPartとして残す
    # 失敗理由は例外を捕まえるまで分からないので、いったん素の名前で保存しておき、
    # 呼び出し側で理由が判明してからリネームする。
    saved_paths: list[pathlib.Path] = []
    original_discard = builder._discard_failed_attempt

    def keep_then_discard(out_dir: str, basename: str, *docs):
        for index, doc in enumerate(docs):
            target = NG_DIR / f"{basename}{'' if index == 0 else f'_{index}'}.CATPart"
            try:
                doc.SaveAs(str(target))
                saved_paths.append(target)
            except Exception as exc:
                print(f"        失敗形状の保存に失敗: {str(exc)[:80]}", flush=True)
        return original_discard(out_dir, basename, *docs)

    builder._discard_failed_attempt = keep_then_discard

    rng = random.Random(20260825)
    made = 0
    kept = 0
    reasons: dict[str, int] = {}

    for attempt in range(1, ATTEMPTS + 1):
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        name = f"bead_{attempt:03d}"
        saved_paths.clear()
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
                out_dir=str(OK_DIR),
                part_name=name,
                bead=bead,
            )
        except ValueError as exc:
            key = slug(str(exc))
            reasons[key] = reasons.get(key, 0) + 1
            _label(saved_paths, key, attempt)
            continue
        except Exception as exc:
            key = "RAW_" + type(exc).__name__
            reasons[key] = reasons.get(key, 0) + 1
            print(f"[{attempt:03d}] *** RAW *** {str(exc)[:110]}", flush=True)
            _label(saved_paths, key, attempt)
            continue
        made += 1
        print(f"[{attempt:03d}] SUCCESS ({time.time()-t0:.1f}s) depth={bead.depth_mm:.1f} "
              f"angle={bead.wall_angle_deg:.1f} topW={bead.top_width_mm:.1f} "
              f"ridgeR={bead.ridge_radius_mm:.1f}", flush=True)

    kept = len(list(NG_DIR.glob("*.CATPart")))
    print(f"\n=== 成功 {made} / 試行 {ATTEMPTS}、失敗形状 {kept} 件を保存 ===", flush=True)
    for key, count in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print(f"  {count:3d}  {key}", flush=True)
    print(f"\n成功: {OK_DIR}\n失敗: {NG_DIR}", flush=True)


if __name__ == "__main__":
    main()

"""稜線R(BiTangent)の失敗が基準面由来かビード由来かを、直交行列で切り分ける
(2026-08-25、ユーザー指示)。

「成功しているシードの基準面でも、ビードのパラメータ次第で失敗しうるし、その逆もありうる」
という指摘のとおり、これまでの層別はランダムな(基準面, ビード)の組でしか見ておらず、
両者の寄与が混ざっていた。**基準面を固定してビードを振る / ビードを固定して基準面を振る**
の両方が同時に読める行列にする。

  行 = 基準面シード(過去のバッチで成功3件・稜線R失敗3件、いずれもhw>=21・3パネル)
  列 = ビード(深さ3 x 稜線R3。壁角度と頂部幅は固定してRと深さだけを動かす)

読み方:
  行がまるごと失敗   -> 基準面由来
  列がまるごと失敗   -> ビード由来
  まだら             -> 相互作用(両者の関係で決まる)
"""
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

import synthetic_generator.gsd_build as gsd_build  # noqa: E402
from synthetic_generator.bead import BeadParams, sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

SEED = 20260825
BASES = [int(a) for a in (sys.argv[1:] or ["28", "44", "63"])]

# 壁角度と頂部幅は固定し、深さと稜線Rだけを動かす。
# hf = 6 + depth/tan(55deg) <= 12.7、side_margin = 0.52*R + 2 <= 7.7 なので hw>=21 なら収まる。
import os
DEPTHS = tuple(float(x) for x in os.environ.get("FM_DEPTHS", "4.5,7.0,9.5").split(","))
RIDGES = tuple(float(x) for x in os.environ.get("FM_RIDGES", "5.0,8.0,11.0").split(","))
WALL_ANGLE = 55.0
TOP_WIDTH = 12.0
OUT = pathlib.Path(__file__).resolve().parent / "probe_output" / "fillet_matrix"


def spec_for(attempt: int):
    """バッチと同じrng列を再生して、指定試行のspecを取り出す。"""
    rng = random.Random(SEED)
    for i in range(1, attempt + 1):
        spec = sample_spec(rng)
        sample_bead(rng, spec.half_width_mm)  # rng列を合わせるため呼ぶだけ
    return spec


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    for i in range(catia.Documents.Count, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass

    cur: dict = {}
    original_bitangent = builder._bead_bitangent

    def watch(*args, **kwargs):
        label = args[-1]
        try:
            result = original_bitangent(*args, **kwargs)
            cur.setdefault("fillets", []).append((label, "OK"))
            return result
        except ValueError as exc:
            text = str(exc)
            inner = text[text.index("[") + 1:text.rindex("]")] if "[" in text else text
            cur.setdefault("fillets", []).append((label, inner))
            raise

    builder._bead_bitangent = watch

    print(f"列(ビード): 深さ{DEPTHS} x 稜線R{RIDGES}  壁角度{WALL_ANGLE}度 頂部幅{TOP_WIDTH}mm")
    print(f"行(基準面): 過去試行 {BASES}\n")
    results = {}
    for attempt in BASES:
        spec = spec_for(attempt)
        cells = []
        for depth in DEPTHS:
            for ridge in RIDGES:
                bead = BeadParams(depth_mm=depth, top_width_mm=TOP_WIDTH,
                                  wall_angle_deg=WALL_ANGLE, ridge_radius_mm=ridge)
                cur.clear()
                try:
                    builder.build_general_two_point(
                        spec.point1, spec.point2,
                        min_bearing_radius_mm=spec.min_bearing_radius_mm,
                        half_width_mm=spec.half_width_mm,
                        bend_radius_mm=spec.bend_radius_mm,
                        fold1_slack_mm=spec.fold1_slack_mm,
                        fold2_slack_mm=spec.fold2_slack_mm,
                        fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                        out_dir=str(OUT), part_name=f"fm_{attempt}_{depth:g}_{ridge:g}",
                        bead=bead,
                    )
                    mark = "O"
                except Exception as exc:
                    fil = cur.get("fillets", [])
                    failed = [l for l, res in fil if res != "OK"]
                    if failed:
                        mark = "t" if failed[0] == "top ridge" else "f"
                    elif "no draft angle" in str(exc):
                        mark = "w"      # 壁で落ちた
                    else:
                        mark = "."      # ビード以前で落ちた
                cells.append(mark)
                print(f"  base#{attempt:<4} depth={depth:4.1f} R={ridge:4.1f} -> {mark}", flush=True)
        results[attempt] = cells

    print("\n=== 行列 (O=成功 t=top ridge失敗 f=foot ridge失敗 w=壁失敗 .=それ以前) ===")
    import math
    ratio = lambda d, r: 2 * r * math.tan(math.radians(WALL_ANGLE) / 2) / (d / math.sin(math.radians(WALL_ANGLE)))
    header = "        " + "  ".join(f"{ratio(d, r):.2f} " for d in DEPTHS for r in RIDGES)
    print("列見出し = 2*R*tan(θ/2)/slant (足元R+頂稜線Rが壁の斜辺を食う比率)")
    print(header)
    for attempt, cells in results.items():
        print(f"base#{attempt:<4}" + "  ".join(f"{c:^5}" for c in cells))


if __name__ == "__main__":
    main()

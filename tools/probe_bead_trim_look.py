"""トリム後の外形曲線が掃引に通らないケースを、現物で見る(2026-08-25)。

長さ・位置は正しいのにMode=4スイープだけが落ちる。トリム無しの境界オフセットなら
同条件で全成功するので、Splitが作る端の切断エッジ周りに原因があるはず。
対象の試行を再現し、外形曲線が入った状態でCATPartを保存する。
"""
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

TARGETS = {4, 9}
OUT = pathlib.Path(__file__).resolve().parent / "probe_output" / "trim_look"


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

    state = {"attempt": 0}

    def stop_and_save(doc, part, hsf, spa, body, curve_ref, surface_ref, top_refs, bead, label):
        target = OUT / f"trim_{state['attempt']:03d}.CATPart"
        doc.SaveAs(str(target))
        print(f"saved {target.name}", flush=True)
        raise ValueError("look stop")

    builder._bead_wall = stop_and_save

    rng = random.Random(20260825)
    for attempt in range(1, max(TARGETS) + 1):
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        state["attempt"] = attempt
        if attempt not in TARGETS:
            continue
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                out_dir=str(OUT), part_name=f"trim_{attempt:03d}", bead=bead,
            )
        except Exception as exc:
            if "look stop" not in str(exc):
                print(f"[{attempt}] 脱落: {str(exc)[:80]}", flush=True)


if __name__ == "__main__":
    main()

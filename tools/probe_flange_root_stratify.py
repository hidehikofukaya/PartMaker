"""フランジ根本BiTangentのUpdate失敗を層別する(2026-08-25、ユーザー指示)。

SS14.4で本番経路4/6のうち2件が「根本BiTangentの全向きUpdate失敗」だった。
帯狙いサンプリング(ユーザー承認②)で母数を増やし、旧2件(seed 616161の走査を再現)と
合わせて変数を層別する。記録する変数はビードの稜線R調査(SS11)と同じ発想:
形状側(折れ角・傾き・基準面R・パネル長・凹凸)とフランジ側(h・根本R・side・direction)。
"""
import json
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.flange import fold_concavity_signs, max_fold_angle_deg  # noqa: E402
from synthetic_generator.general_geometry import plan_general_two_point  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import (  # noqa: E402
    resolve_reinforcement,
    sample as sample_spec,
)

OUT = pathlib.Path(__file__).resolve().parent / "probe_output" / "flange_root"
TARGET_BUILDS = 30


def collect(rng, *, gentle: bool, want: int, scan_cap: int):
    """resolve_reinforcementがフランジを返す(spec, flange)を集める。"""
    out = []
    scanned = 0
    while len(out) < want and scanned < scan_cap:
        scanned += 1
        spec = sample_spec(rng, gentle_folds=gentle)
        r = resolve_reinforcement(rng, spec)
        if r is None or r[2] is None:
            continue
        out.append((r[0], r[2]))
    return out


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    builder = SyntheticPartBuilder()
    builder.catia.DisplayFileAlerts = False
    for i in range(builder.catia.Documents.Count, 0, -1):
        try:
            builder.catia.Documents.Item(i).Close()
        except Exception:
            pass

    cases = []
    # 旧2件を含むseed 616161の走査(gentleなし、6件)を再現し、帯狙いで残りを足す
    cases += collect(random.Random(616161), gentle=False, want=6, scan_cap=2000)
    cases += collect(random.Random(818181), gentle=True, want=TARGET_BUILDS - len(cases),
                     scan_cap=2000)
    print(f"対象 {len(cases)} 件", flush=True)

    rows = []
    for k, (spec, flange) in enumerate(cases):
        plan = plan_general_two_point(
            spec.point1, spec.point2,
            min_bearing_radius_mm=spec.min_bearing_radius_mm,
            half_width_mm=spec.half_width_mm, bend_radius_mm=spec.bend_radius_mm,
            fold1_slack_mm=spec.fold1_slack_mm, fold2_slack_mm=spec.fold2_slack_mm,
            fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
            side_extension_mm=(
                flange.extension_mm if flange.side < 0 else 0.0,
                flange.extension_mm if flange.side > 0 else 0.0,
            ),
        )
        frames = plan.panel_frames
        concave = fold_concavity_signs(frames)
        row = dict(
            case=k,
            h=flange.height_mm, rootr=flange.root_radius_mm,
            side=flange.side, direction=flange.direction,
            ext=flange.extension_mm,
            maxfold=max_fold_angle_deg(frames),
            n_panels=len(frames),
            bendr=spec.bend_radius_mm, hw=spec.half_width_mm,
            min_panel=min(f.far_run_mm - f.near_run_mm for f in frames),
            tilt=max(abs(math.degrees(t)) for pair in plan.fold_tilts for t in pair),
            crosses_concave=any(s == flange.direction for s in concave),
            h_margin=spec.bend_radius_mm - flange.height_mm,
        )
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm, bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm, fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                out_dir=str(OUT), part_name=f"fr_{k:02d}", flange=flange)
            row["outcome"] = "OK"
        except ValueError as exc:
            text = str(exc)
            if "flange root" in text:
                row["outcome"] = "root_fillet"
            elif "flange wall" in text:
                row["outcome"] = "wall"
            else:
                row["outcome"] = "other"
            row["error"] = text[:220]
        rows.append(row)
        print(f"[{k:02d}] {row['outcome']:>11}  h={row['h']:.1f} R={row['rootr']:.1f} "
              f"dir={row['direction']:+d} side={row['side']:+d} 凹跨ぎ={row['crosses_concave']} "
              f"折れ{row['maxfold']:.0f}度 基準R={row['bendr']:.1f}", flush=True)

    (OUT / "rows.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1),
                                   encoding="utf-8")
    ok = sum(1 for r in rows if r["outcome"] == "OK")
    print(f"\n成功 {ok}/{len(rows)}  保存: {OUT / 'rows.json'}", flush=True)


if __name__ == "__main__":
    main()

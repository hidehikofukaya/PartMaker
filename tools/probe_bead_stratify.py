"""ビードの2大失敗要因を同一バッチで層別する(2026-08-25、ユーザー指示)。

  A. 壁バンドのMode=4スイープ失敗(どのドラフト角も頂部エッジを期待位置に置けない)
  B. 稜線R(BiTangent)の失敗 — 頂稜線(band x 頂面)と足元(hat x 基準面)

本番経路には手を入れず、`plan_bead_on_surface`(基準面側の形状変数が全部渡ってくる)と
`_bead_wall` / `_bead_bitangent` を包んで観測だけ取る。両者が投げる例外メッセージには
既に角度別・向き別の実測値が入っているので、それを構造化して拾う。
"""
import math
import pathlib
import random
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

import synthetic_generator.gsd_build as gsd_build  # noqa: E402
from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

ATTEMPTS = int(sys.argv[1]) if len(sys.argv) > 1 else 60
OUT = pathlib.Path(__file__).resolve().parent / "probe_output" / "stratify"


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


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

    rows: list[dict] = []
    cur: dict = {}

    original_plan = gsd_build.plan_bead_on_surface

    def capture_plan(panel_frames, bead, **kwargs):
        # 基準面側の形状変数はここに全部そろう
        folds = []
        for i in range(len(panel_frames) - 1):
            a, b = panel_frames[i], panel_frames[i + 1]
            folds.append(math.degrees(math.acos(max(-1.0, min(1.0, dot(a.u, b.u))))))
        cur["panels"] = len(panel_frames)
        cur["fold_deg"] = folds
        cur["tilt_deg"] = [math.degrees(t) for _, t in kwargs["fold_tilts"][:-1]]
        cur["panel_len"] = [f.far_run_mm - f.near_run_mm for f in panel_frames]
        cur["setback"] = [far for _, far in kwargs["fold_tangents"][:-1]]
        cur["hw"] = kwargs["half_width_mm"]
        cur["inset"] = kwargs["inset_mm"]
        plan = original_plan(panel_frames, bead, **kwargs)
        cur["offset"] = plan.outline_offset_mm
        # 頂部エッジのプローブがどのパネルに置かれたか。中央パネルに平坦区間が
        # 取れないと`top_panel_index[len//2]`が黙って端パネルを指す。
        cur["probe_panel"] = plan.wall_top_panel_index[0]
        cur["n_bead_panels"] = len(plan.top_panel_index)
        cur["probe_on_end"] = plan.wall_top_panel_index[0] in (0, len(panel_frames) - 1)
        # 各パネルの平坦区間の幅
        flats = []
        for i, f in enumerate(panel_frames):
            near_cut, far_cut = kwargs["fold_tangents"][i]
            flats.append((f.far_run_mm - far_cut - 1.0) - (f.near_run_mm + near_cut + 1.0))
        cur["flat"] = flats
        return plan

    gsd_build.plan_bead_on_surface = capture_plan

    original_wall = builder._bead_wall

    def watch_wall(*args, **kwargs):
        try:
            result = original_wall(*args, **kwargs)
            cur["wall"] = "OK"
            return result
        except ValueError as exc:
            text = str(exc)
            inner = text[text.index("[") + 1:text.rindex("]")] if "[" in text else text
            cur["wall"] = inner
            raise

    builder._bead_wall = watch_wall

    original_bitangent = builder._bead_bitangent

    def watch_bitangent(*args, **kwargs):
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

    builder._bead_bitangent = watch_bitangent

    rng = random.Random(20260825)
    for attempt in range(1, ATTEMPTS + 1):
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        cur.clear()
        cur.update(
            attempt=attempt, depth=bead.depth_mm, theta=bead.wall_angle_deg,
            topw=bead.top_width_mm, hf=bead.half_footprint_mm, ridger=bead.ridge_radius_mm,
            slant=bead.wall_slant_mm, wallrun=bead.wall_run_mm,
            setbk=bead.ridge_setback_mm, fits=bead.ridges_fit_on_wall,
            bendr=spec.bend_radius_mm, bearing=spec.min_bearing_radius_mm,
        )
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                out_dir=str(OUT), part_name=f"st_{attempt:03d}", bead=bead,
            )
            cur["outcome"] = "SUCCESS"
        except Exception as exc:
            cur.setdefault("outcome", type(exc).__name__ + ": " + str(exc)[:60])
            if "wall" not in cur and "fillets" not in cur:
                cur["outcome"] = "early: " + str(exc)[:70]
        rows.append(dict(cur))

    import json
    (OUT / "rows.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"\n=== {len(rows)} 行を {OUT / 'rows.json'} に保存 ===", flush=True)
    reached_wall = [r for r in rows if "wall" in r]
    print(f"CATIAのビード段階に到達: {len(reached_wall)}", flush=True)
    print(f"  壁OK: {sum(1 for r in reached_wall if r['wall'] == 'OK')}", flush=True)
    fil = [r for r in rows if r.get("fillets")]
    print(f"  稜線Rに到達: {len(fil)}", flush=True)
    print(f"  全成功: {sum(1 for r in rows if r['outcome'] == 'SUCCESS')}", flush=True)


if __name__ == "__main__":
    main()

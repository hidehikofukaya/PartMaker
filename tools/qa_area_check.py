"""生成部品の面積を実測し、解析的な期待値と比べて異常を検出する(2026-08-25)。

目視は視角に左右される(細長い部品は真横から見ると線に見える)ので、客観的な数値で
裏を取る。これまで見つかった欠陥(刃状の削れ・両側フランジ・途切れ・スリバー)は
すべて**面積の過不足**として現れるため、面積比が有効な指標になる。

期待面積:
  基準面 = sum(パネル長) * 2*half_width
           - 余肉カット4隅(または2隅) * (R^2 - pi*R^2/4)
  ビード = 壁バンド(周長 * 斜辺長) + 頂面(フットプリント相当) - 基準面から抜けた分
  フランジ = 壁(全長 * 高さ)
細部(フィレットの丸め・逃げ)までは追わないので、比の**分布からの外れ**を見る。

使い方: python tools/qa_area_check.py <batchディレクトリ>
"""
import json
import math
import pathlib
import statistics as st
import sys
import time

import win32com.client

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.classify import FasteningPoint  # noqa: E402
from synthetic_generator.general_geometry import plan_general_two_point  # noqa: E402


def expected_area_mm2(row: dict) -> float:
    """解析的な期待面積(粗い見積り)。"""
    spec = row["spec"]
    p1 = FasteningPoint(tuple(spec["point1"]["position_xyz"]), tuple(spec["point1"]["normal_xyz"]))
    p2 = FasteningPoint(tuple(spec["point2"]["position_xyz"]), tuple(spec["point2"]["normal_xyz"]))
    flange = row["flange"]
    ext = 0.0
    side_ext = (0.0, 0.0)
    if flange:
        ext = flange["root_radius_mm"] + 2.0
        side_ext = (ext if flange["side"] < 0 else 0.0, ext if flange["side"] > 0 else 0.0)
    plan = plan_general_two_point(
        p1, p2,
        min_bearing_radius_mm=spec["min_bearing_radius_mm"],
        half_width_mm=spec["half_width_mm"],
        bend_radius_mm=spec["bend_radius_mm"],
        fold1_slack_mm=spec["fold1_slack_mm"],
        fold2_slack_mm=spec["fold2_slack_mm"],
        fold1_tilt_perturbation_rad=spec["fold1_tilt_perturbation_rad"],
        side_extension_mm=side_ext,
    )
    run_total = sum(f.far_run_mm - f.near_run_mm for f in plan.panel_frames)
    width = 2.0 * spec["half_width_mm"] + ext
    area = run_total * width
    # 余肉カット: 1隅あたり R^2 - (pi/4)R^2 を落とす
    r = spec["min_bearing_radius_mm"]
    corners = 2 if flange else 4
    area -= corners * r * r * (1.0 - math.pi / 4.0)
    bead = row["bead"]
    if bead:
        theta = math.radians(bead["wall_angle_deg"])
        slant = bead["depth_mm"] / math.sin(theta)
        hf = bead["top_width_mm"] / 2.0 + bead["depth_mm"] / math.tan(theta)
        perimeter = 2.0 * run_total + 2.0 * 2.0 * hf
        area += perimeter * slant                      # 壁バンド
        area -= 2.0 * hf * run_total                   # ビード直下の基準面が抜ける
        area += bead["top_width_mm"] * run_total       # 頂面
    if flange:
        area += run_total * flange["height_mm"]        # フランジ壁
    return area


def main() -> None:
    # CATIAのDocuments.Openは**絶対パス**しか受け付けない(相対だと無言でOpen失敗)
    batch = pathlib.Path(sys.argv[1]).resolve()
    rows = {}
    for path in sorted((batch / "params").glob("*.json")):
        rows[path.stem] = json.loads(path.read_text(encoding="utf-8"))

    app = win32com.client.GetActiveObject("DELMIA.Application")
    app.DisplayFileAlerts = False
    for i in range(app.Documents.Count, 0, -1):
        try:
            app.Documents.Item(i).Close()
        except Exception:
            pass

    results = []
    for name, row in rows.items():
        path = batch / "mid" / f"{name}_mid.CATPart"
        if not path.exists():
            continue
        doc = None
        for _ in range(3):   # 初回Openが散発的に失敗する(既知のCOMの気まぐれ)
            try:
                doc = app.Documents.Open(str(path))
                break
            except Exception:
                time.sleep(0.5)
        if doc is None:
            print(f"{name}: Open失敗", flush=True)
            continue
        try:
            part = doc.Part
            body = part.HybridBodies.Item(1)
            shape = body.HybridShapes.Item(body.HybridShapes.Count)
            ref = part.CreateReferenceFromObject(shape)
            measurable = doc.GetWorkbench("SPAWorkbench").GetMeasurable(ref)
            measured = measurable.Area * 1e6
        except Exception as exc:
            print(f"{name}: 計測失敗 {str(exc)[:50]}", flush=True)
            doc.Close()
            continue
        doc.Close()
        expected = expected_area_mm2(row)
        kind = "flange" if row["flange"] else "bead"
        results.append((name, kind, measured, expected, measured / expected))

    print(f"\n計測 {len(results)} 件")
    for kind in ("bead", "flange"):
        ratios = [r[4] for r in results if r[1] == kind]
        if not ratios:
            continue
        med = st.median(ratios)
        # 中央値からの相対偏差でロバストに外れ値を拾う
        dev = st.median([abs(x - med) for x in ratios]) or 1e-6
        flagged = [r for r in results if r[1] == kind and abs(r[4] - med) > 6.0 * dev]
        print(f"\n{kind}: n={len(ratios)} 面積比 中央{med:.3f} "
              f"[{min(ratios):.3f}, {max(ratios):.3f}]  MAD={dev:.4f}")
        print(f"  外れ値(中央値から6MAD超): {len(flagged)}件 "
              f"({len(flagged) / len(ratios):.0%})")
        for name, _k, measured, expected, ratio in sorted(flagged, key=lambda x: -abs(x[4] - med))[:10]:
            print(f"    {name[-4:]}: 実測{measured:8.0f} 期待{expected:8.0f} 比{ratio:.3f}")


if __name__ == "__main__":
    main()

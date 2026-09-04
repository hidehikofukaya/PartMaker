"""サイドカーの特徴で、抽出済み bend_line にラベルが付くかを検証する(2026-08-30)。

依頼元の受け入れ条件C(「曲がっているのに曲げ線から遠い表面」の削減)を、生成側から
先に測っておくためのもの。抽出器の各 bend_line が、生成器が知っている特徴曲線
(折れ目の接線・ビードの稜線・フランジの根本・余肉カットの円弧)のどれで説明できるかを
距離で判定し、説明できない本数と全長比を出す。

使い方: python tools/qa_feature_label.py <chunkディレクトリ> [件数]
"""
from __future__ import annotations

import json
import pathlib
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, "C:/Users/hide2/IdeaBox/fill_volume/wireframe_app")
import extract as wf  # noqa: E402

TOL_MM = 2.0


def _seg_dist(P: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    L2 = float(ab @ ab) or 1e-9
    t = np.clip(((P - a) @ ab) / L2, 0.0, 1.0)
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def _poly_dist(P: np.ndarray, poly: np.ndarray) -> np.ndarray:
    if len(poly) < 2:
        return np.full(len(P), 1e9)
    return np.min(np.stack([_seg_dist(P, poly[i], poly[i + 1])
                            for i in range(len(poly) - 1)]), axis=0)


def curves_of(feat: dict) -> dict[str, np.ndarray]:
    """サイドカーから照合用の曲線群を組み立てる。"""
    out: dict[str, np.ndarray] = {}
    for fold in feat["folds"]:
        for t in fold["tangent_lines"]:
            out[f"bend_line|fold{fold['id']}side{t['side']}"] = np.array([t["p0"], t["p1"]])
    bead = feat.get("bead")
    if bead:
        n0 = np.array(feat["panels"][0]["normal"])
        lift = bead["top_ridge_lift_mm"]
        for tag, key in (("bead_foot", "foot_ridge_left"), ("bead_foot", "foot_ridge_right")):
            arr = np.array(bead[key], float)
            if len(arr) >= 2:
                out[f"{tag}|{key}"] = arr
        for key in ("top_ridge_left", "top_ridge_right"):
            arr = np.array(bead[key], float)
            if len(arr) < 2:
                continue
            # 頂稜線の持ち上がり向きは params に残っていないので両符号を候補にする
            out[f"bead_top|{key}+"] = arr + n0 * lift
            out[f"bead_top|{key}-"] = arr - n0 * lift
    flange = feat.get("flange")
    if flange:
        arr = np.array(flange["root_curve"], float)
        if len(arr) >= 2:
            out["flange_root|base"] = arr
    for c in feat["corner_relief"]:
        arr = np.array(c["arc"], float)
        if len(arr) >= 2:
            out[f"corner_relief|{c['label']}"] = arr
    return out


def label_part(feat: dict, wire: dict, tol: float = TOL_MM):
    """辺を特徴に割り当てる。

    折れ目の接線は「線」なので厳密照合でよいが、ビード/フランジの稜線は
    **フットプリント全周の閉ループ**(端キャップ・四隅・フィレット横断を含む)なので
    曲線の再現は割に合わない。中心線からの距離による**領域判定**にする —
    ビード領域に入る曲げ線はビード由来、という判定はラベルとしては十分で、
    かつ端キャップも四隅も自動的に含む。
    """
    curves = curves_of(feat)
    bead = feat.get("bead")
    bead_centre = np.array(bead["centreline"], float) if bead else None
    bead_reach = (bead["half_footprint_mm"] + bead["ridge_setback_mm"] + 2.0) if bead else 0.0
    flange = feat.get("flange")
    flange_root = np.array(flange["root_curve"], float) if flange else None
    flange_reach = (flange["root_radius_mm"] + flange["height_mm"] + 2.0) if flange else 0.0
    fold_curves = {k: v for k, v in curves.items() if k.startswith("bend_line|")}
    relief_curves = {k: v for k, v in curves.items() if k.startswith("corner_relief|")}
    hits = Counter()
    explained_len = unexplained_len = 0.0
    residuals = []
    for e in wire["edges"]:
        if e["type"] != "bend_line":
            continue
        P = np.asarray(e["polyline"], float)
        # 1) 折れ目の接線(線どうしの厳密照合)
        best, bestd = None, 1e9
        for name, poly in list(fold_curves.items()) + list(relief_curves.items()):
            dd = float(np.median(_poly_dist(P, poly)))
            if dd < bestd:
                best, bestd = name, dd
        label = best.split("|")[0] if bestd <= tol else None
        residuals.append(bestd)
        # 2) ビード/フランジは領域判定
        if label is None and bead_centre is not None and len(bead_centre) >= 2:
            if float(np.median(_poly_dist(P, bead_centre))) <= bead_reach:
                label = "bead"
        if label is None and flange_root is not None and len(flange_root) >= 2:
            if float(np.median(_poly_dist(P, flange_root))) <= flange_reach:
                label = "flange"
        if label is None:
            hits["UNEXPLAINED"] += 1
            unexplained_len += e["length_mm"]
        else:
            hits[label] += 1
            explained_len += e["length_mm"]
    return hits, explained_len, unexplained_len, residuals


def main() -> None:
    chunk = pathlib.Path(sys.argv[1]).resolve()
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    total = Counter()
    exp = unexp = 0.0
    res_all = []
    n = 0
    for fp in sorted((chunk / "features").glob("*.json"))[:limit]:
        feat = json.loads(fp.read_text(encoding="utf-8"))
        stp = chunk / "mid" / f"{fp.stem}_mid.stp"
        if not stp.exists():
            continue
        wire = wf.extract_wireframe(stp)
        hits, e, u, res = label_part(feat, wire)
        total.update(hits)
        exp += e
        unexp += u
        res_all.extend(res)
        n += 1
    tot = sum(total.values())
    print(f"\n{n}部品 / bend_line {tot}本")
    for k, v in total.most_common():
        print(f"  {k:16s} {v:5d} ({v / tot:5.1%})")
    print(f"\n説明できた全長比: {exp / (exp + unexp):.1%}")
    r = sorted(res_all)
    print(f"最寄り特徴までの距離: 中央{r[len(r)//2]:.2f}mm  "
          f"p90 {r[int(0.9*len(r))]:.2f}mm  最大{r[-1]:.1f}mm")


if __name__ == "__main__":
    main()

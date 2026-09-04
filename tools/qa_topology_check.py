"""STPの面エンティティを数えて、ビード/フランジの崩壊を検出する(2026-08-26)。

CATIAを一切使わない軽量検査。ビードやフランジが崩壊すれば**面が消える**ので、
STEPの面エンティティ数(ADVANCED_FACE と各種サーフェス)が直接の指標になる。
面積検査(qa_area_check.py)がCATIAで1部品あたり数秒かかるのに対し、こちらは
ファイル走査だけで1部品あたり10ms程度。

判定: 補強種別ごとに面数の分布を取り、**下側の外れ値**を崩壊候補として拾う
(崩壊は面が減る方向にしか出ない。上振れは細分化なので無害)。

使い方: python tools/qa_topology_check.py <ルートディレクトリ...>
"""
import json
import pathlib
import re
import statistics as st
import sys
from collections import Counter

FACE_PATTERN = re.compile(rb"=\s*ADVANCED_FACE\(")
SURF_PATTERN = re.compile(
    rb"=\s*(B_SPLINE_SURFACE_WITH_KNOTS|CYLINDRICAL_SURFACE|PLANE|TOROIDAL_SURFACE|"
    rb"CONICAL_SURFACE|SPHERICAL_SURFACE)\("
)


def scan(root: pathlib.Path):
    rows = []
    for chunk in sorted(root.glob("chunk_*")):
        for params_path in sorted((chunk / "params").glob("*.json")):
            meta = json.loads(params_path.read_text(encoding="utf-8"))
            stp = chunk / "mid" / f"{params_path.stem}_mid.stp"
            if not stp.exists():
                print(f"  STP欠損: {params_path.stem}", flush=True)
                continue
            blob = stp.read_bytes()
            kind = "flange" if meta["flange"] else ("bead" if meta["bead"] else "plain")
            rows.append({
                "id": params_path.stem,
                "root": root.name,
                "chunk": chunk.name,
                "kind": kind,
                "faces": len(FACE_PATTERN.findall(blob)),
                "surfs": len(SURF_PATTERN.findall(blob)),
                "bytes": len(blob),
            })
    return rows


def report(rows, metric: str, label: str):
    print(f"\n■{label}(補強種別ごと)")
    flagged = []
    for kind in ("bead", "flange", "plain"):
        vals = [r[metric] for r in rows if r["kind"] == kind]
        if not vals:
            continue
        med = st.median(vals)
        mad = st.median([abs(v - med) for v in vals]) or 1.0
        # 崩壊は「面が減る」方向にしか出ないので下側だけを見る
        low = [r for r in rows if r["kind"] == kind and (med - r[metric]) > 6.0 * mad]
        vs = sorted(vals)
        print(f"  {kind:7s} n={len(vals):4d}  中央{med:6.1f}  MAD={mad:5.1f}  "
              f"最小{vs[0]:4d} p10 {vs[len(vs)//10]:4d} p90 {vs[9*len(vs)//10]:4d} 最大{vs[-1]:4d}")
        print(f"          下側外れ値(中央-6MAD未満): {len(low)}件 ({len(low)/len(vals):.1%})")
        flagged.extend(low)
    return flagged


def main() -> None:
    rows = []
    for arg in sys.argv[1:]:
        root = pathlib.Path(arg).resolve()
        got = scan(root)
        print(f"{root.name}: {len(got)}件 走査", flush=True)
        rows.extend(got)
    print(f"\n合計 {len(rows)}件  内訳 {dict(Counter(r['kind'] for r in rows))}")

    flagged = report(rows, "faces", "面数 ADVANCED_FACE")
    report(rows, "surfs", "サーフェス数")

    if flagged:
        print(f"\n■崩壊候補 {len(flagged)}件(面数が下側に外れたもの)")
        for r in sorted(flagged, key=lambda x: x["faces"])[:20]:
            print(f"  {r['root']}/{r['chunk']}/{r['id'][-4:]}  {r['kind']:7s} "
                  f"面{r['faces']:4d} サーフェス{r['surfs']:4d}")
        out = pathlib.Path("tools/probe_output/topology_flagged.json")
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(flagged, ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"  -> {out}")
    else:
        print("\n崩壊候補なし")


if __name__ == "__main__":
    main()

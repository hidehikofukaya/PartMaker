"""OCCTバックエンドで1チャンクを生成する(2026-09-04、PartMaker刷新)。

CATIA/DELMIAは不要。ライセンス・GUI・クリップボード・チャンク毎の再起動も不要で、
`multiprocessing`で並列化もできる(このスクリプトは単プロセス)。

出力は既存の契約(引継ぎ書 §2.4)を維持したうえで、同 §4.2/§4.6 の追加物を出す:

  synthetic_parts/<族>/<chunk>/
    mid/<part_id>_mid.stp        1 OPEN_SHELL、mm、AP214、XCAFで面名つき
    params/<part_id>.json        spec(7キー)+ bead|flange + backend
    features/<part_id>.json      partmaker_features/2 = v1 + faces[] + backend
    annotations/joints.json      schema 1.1(途中でも書く)
    manifest.json                partmaker_manifest/1(族・容量・クラス内訳)
    _COMPLETE                    完了マーカー(部品数)

使い方:
  python tools/run_occt_batch.py <族名> <チャンク番号> <部品数> <seed>
                                 [補強確率] [フランジ狙い比率]
例:
  python tools/run_occt_batch.py occt01 1 250 20260904
"""
from __future__ import annotations

import collections
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import emit_feature_truth  # noqa: E402
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE  # noqa: E402
from OCC.Core.TopExp import topexp  # noqa: E402
from OCC.Core.TopTools import TopTools_IndexedMapOfShape  # noqa: E402

from synthetic_generator.batch_generate import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    generate_recipe_batch,
)
from synthetic_generator.classify import classify  # noqa: E402
from synthetic_generator.occt_build import OcctPartBuilder  # noqa: E402

GENERATOR_VERSION = "2.0.0"


def shape_capacity(stp_path: str) -> tuple[int, int]:
    from OCC.Core.STEPControl import STEPControl_Reader
    reader = STEPControl_Reader()
    reader.ReadFile(stp_path)
    reader.TransferRoots()
    shape = reader.OneShape()
    counts = []
    for kind in (TopAbs_FACE, TopAbs_EDGE):
        seen = TopTools_IndexedMapOfShape()   # 共有エッジを二重に数えない
        topexp.MapShapes(shape, kind, seen)
        counts.append(seen.Size())
    return counts[0], counts[1]


def main() -> None:
    """使い方: python tools/run_occt_batch.py <レシピJSON>

    レシピが「何をどれだけどんな多様性で作るか」を全部持つ(2026-09-04、族別生成)。
    例は tools/recipes/default.json。
    """
    recipe_path = pathlib.Path(sys.argv[1])
    recipe = json.loads(recipe_path.read_text(encoding="utf-8"))
    family, chunk, seed = recipe["family"], int(recipe["chunk"]), int(recipe["seed"])

    out_dir = DEFAULT_OUTPUT_ROOT / family / f"chunk_{chunk:02d}"
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"{out_dir} は既に存在する。")

    wanted = {e["kind"]: e["count"] for e in recipe["parts"]}
    print(f"{family}/chunk_{chunk:02d}: {sum(wanted.values())}部品 {wanted} / seed {seed}",
          flush=True)
    labels_by_part: dict[str, tuple] = {}

    class _Recording(OcctPartBuilder):
        """面ラベルはビルド時にしか手元に無いので、その場で拾っておく。"""

        def build_general_two_point(self, *args, **kwargs):
            part = super().build_general_two_point(*args, **kwargs)
            labels_by_part[kwargs["part_name"]] = part.face_labels
            return part

    attempts: collections.Counter = collections.Counter()
    tries: collections.Counter = collections.Counter()

    def on_part(kind, spec, bead, flange, rib, used):
        attempts[kind] += 1
        tries[kind] += used

    t0 = time.time()
    records = generate_recipe_batch(_Recording(), out_dir, recipe["parts"],
                                    seed=seed, on_part=on_part)
    build_seconds = time.time() - t0

    features_dir = out_dir / "features"
    features_dir.mkdir(exist_ok=True)
    classes: collections.Counter = collections.Counter()
    kinds: collections.Counter = collections.Counter()
    faces_max = edges_max = 0
    counts: list[int] = []   # 締結点の数(3点族で3になる)
    for record in records:
        params_path = out_dir / "params" / f"{record.part_id}.json"
        meta = json.loads(params_path.read_text(encoding="utf-8"))
        meta["backend"] = "occt"
        meta["generator_version"] = GENERATOR_VERSION
        params_path.write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")

        feature = emit_feature_truth.build(meta)
        feature["schema"] = "partmaker_features/2"
        feature["backend"] = "occt"
        feature["kind"] = meta["kind"]
        feature["faces"] = list(labels_by_part[record.part_id])
        (features_dir / f"{record.part_id}.json").write_text(
            json.dumps(feature, ensure_ascii=False), encoding="utf-8")

        n_faces, n_edges = shape_capacity(record.stp_path)
        faces_max, edges_max = max(faces_max, n_faces), max(edges_max, n_edges)
        classes[str(classify(record.spec.point1, record.spec.point2))] += 1
        kinds[meta["kind"]] += 1
        annotated = getattr(record.spec, "annotated_points", None)
        counts.append(len(annotated) if annotated
                      else 2 + len(getattr(record.spec, "extra_points", ())))

    manifest = {
        "schema": "partmaker_manifest/1",
        "family": family,
        "chunk": f"chunk_{chunk:02d}",
        "generator_version": GENERATOR_VERSION,
        "backend": "occt",
        "seed": seed,
        "complete": True,
        "n_parts": len(records),
        "recipe": recipe["parts"],
        "fastener_count": {"min": min(counts or [2]), "max": max(counts or [2])},
        "capacity": {"faces_max": faces_max, "edges_max": edges_max, "loops_max": 1},
        "config_classes": dict(classes),
        "reinforcement": dict(kinds),
        "spec_keys": ["thickness_mm", "half_width_mm", "bend_radius_mm", "fold1_slack_mm",
                      "fold2_slack_mm", "min_bearing_radius_mm", "hole_diameter_mm"],
        "holes": False,
        "seconds_per_part": round(build_seconds / max(1, len(records)), 3),
        "files": {"mid": "mid/{id}_mid.stp", "params": "params/{id}.json",
                  "features": "features/{id}.json", "joints": "annotations/joints.json"},
    }
    (out_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8")
    (out_dir / "_COMPLETE").write_text(f"{len(records)}\n", encoding="utf-8")

    total = time.time() - t0
    print(f"\nCHUNK DONE: {len(records)}部品 / {total / 60:.1f}分 "
          f"(生成 {build_seconds / max(1, len(records)):.2f}秒/部品)", flush=True)
    print(f"  族: {dict(kinds)}")
    print(f"  族ごとの平均試行: " +
          ", ".join(f"{k} {tries[k] / max(1, attempts[k]):.1f}回" for k in sorted(attempts)))
    print(f"  配置クラス: {dict(classes.most_common())}")
    print(f"  容量: 面 最大{faces_max} / エッジ 最大{edges_max}")
    print(f"  出力: {out_dir}")


if __name__ == "__main__":
    main()

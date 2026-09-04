"""params から joints.json を復旧する(2026-08-26、SS18)。

`generate_general_batch` は joints.json をバッチ末尾でまとめて書くため、途中で
RuntimeError が出ると **部品は残るがアノテーションが無い** 状態になる
(フランジ専用チャンクのクォータ枯渇で実際に発生)。params には spec 全体が
入っているので、バッチと同じ手順で書き直せる。

使い方: python tools/rebuild_joints.py <chunkディレクトリ...>
"""
from __future__ import annotations

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.annotate import build_two_joint_pair  # noqa: E402
from synthetic_generator.annotation_schema import AnnotationDocument, PartEntry  # noqa: E402
from synthetic_generator.classify import FasteningPoint  # noqa: E402


def main() -> None:
    for arg in sys.argv[1:]:
        chunk = pathlib.Path(arg).resolve()
        params = sorted((chunk / "params").glob("*.json"))
        if not params:
            print(f"{chunk.name}: params無し、スキップ", flush=True)
            continue
        doc = AnnotationDocument(assembly_dir=chunk, full_assembly_stp="synthetic")
        n = 0
        for pp in params:
            meta = json.loads(pp.read_text(encoding="utf-8"))
            spec = meta["spec"]
            part_id = meta["part_id"]
            if not (chunk / "mid" / f"{part_id}_mid.stp").exists():
                continue          # ビルドされなかった部品(paramsだけ)は除外
            doc.parts[part_id] = PartEntry(
                part_id=part_id,
                stp_file=f"mid/{part_id}_mid.stp",
                vtp_file="",
                tag="sheet_metal",
                thickness_mm=spec["thickness_mm"],
                thickness_source="synthetic_generator",
            )
            p1 = FasteningPoint(tuple(spec["point1"]["position_xyz"]),
                                tuple(spec["point1"]["normal_xyz"]))
            p2 = FasteningPoint(tuple(spec["point2"]["position_xyz"]),
                                tuple(spec["point2"]["normal_xyz"]))
            for joint in build_two_joint_pair(part_id, p1, p2, spec["hole_diameter_mm"]):
                doc.add_joint(joint)
            n += 1
        doc.save()
        print(f"{chunk.parent.name}/{chunk.name}: {n}部品ぶんの joints.json を復旧", flush=True)


if __name__ == "__main__":
    main()

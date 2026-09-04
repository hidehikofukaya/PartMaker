"""OCCTバックエンドの幾何合格テスト(2026-09-04)。

「例外が出ない」だけでは不足(docs/REVIEW_bead_sweep_twist.md の指摘)なので、
出来た形を測る: シェルの妥当性、エッジが単一のプリミティブに載ること、
ゴミエッジが無いこと、締結点が面上に残っていること、ビードがねじれていないこと。

判定ロジックは `tools/occt_smoke.py` と共有する(広いサンプルでの計測はそちら:
`python tools/occt_smoke.py 200 <seed>`)。
"""
from __future__ import annotations

import pathlib
import random
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))

from occt_smoke import audit, bead_probe, distance_to, read_step  # noqa: E402

from synthetic_generator.occt_build import OcctPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import resolve_reinforcement  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_general  # noqa: E402


def _build(rng, out_dir, name, want):
    """`want` ("bead" / "flange" / "rib" / None) の部品を1つ作る。作れなければNone。"""
    builder = OcctPartBuilder()
    for _ in range(2000):
        try:
            # リブは「締結点が近すぎてビードが置けない」ときだけ出るので、短い部品が
            # 出やすい単曲げ族を狙う(そうしないと試行のほとんどがビードになる)。
            spec = sample_general(rng, gentle_folds=(want == "flange"),
                                  target_folds=1 if want == "rib" else None)
        except ValueError:
            continue
        bead = flange = rib = None
        if want is not None:
            resolved = resolve_reinforcement(rng, spec)
            if resolved is None:
                continue
            spec, bead, flange, rib = resolved
            got = "bead" if bead else ("flange" if flange else ("rib" if rib else None))
            if got != want:
                continue
        try:
            part = builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                out_dir=str(out_dir), part_name=name,
                target_folds=spec.target_folds,
                bead=bead, flange=flange, rib=rib,
            )
        except ValueError:
            continue
        return part, spec, bead, flange, rib
    return None


@pytest.mark.parametrize("want", ["bead", "flange", "rib", None])
def test_occt_part_is_geometrically_sound(tmp_path, want):
    made = _build(random.Random(4242), tmp_path, f"test_{want}", want)
    assert made is not None, f"could not sample a {want} part in 2000 attempts"
    part, spec, _bead, _flange, _rib = made

    shape = read_step(part.stp_path)
    info = audit(shape)
    assert info["valid"], "the sewn shell is not valid"
    assert info["deviation"] <= 0.25 * spec.thickness_mm, (
        f"an edge deviates {info['deviation']:.4f}mm from a single line/arc "
        f"(gate A allows {0.25 * spec.thickness_mm:.4f}mm)")
    assert info["tiny_edges"] == 0, f"{info['tiny_edges']} junk edges under 0.05mm"
    assert info["free_edges"] > 0, "an open shell must have an outline"
    assert info["boundary_loops"] == 1, (
        f"the outline is {info['boundary_loops']} closed loops, not 1 -- the shape collapsed")
    assert info["dihedral_deg"] <= 150.0, (
        f"two adjacent faces turn {info['dihedral_deg']:.0f}deg -- the surface folds back")

    for label, point in (("point1", spec.point1.position_xyz),
                         ("point2", spec.point2.position_xyz)):
        assert distance_to(shape, point) < 0.1, f"{label} is not on the surface"

    assert part.face_labels, "no face labels were produced"
    assert all(entry["name"] for entry in part.face_labels)


def test_bead_is_not_twisted(tmp_path):
    """ビード頂部の意図位置とその鏡像までの距離で、掃引のねじれを検出する
    (Gemini版が螺旋リボンになった不具合の再発検知)。"""
    made = _build(random.Random(99), tmp_path, "test_twist", "bead")
    assert made is not None
    part, spec, bead, _, _ = made
    shape = read_step(part.stp_path)
    top, mirror = bead_probe(spec, bead)
    assert distance_to(shape, top) <= 0.1, "the bead top is not where it was planned"
    assert distance_to(shape, mirror) >= 0.8 * bead.depth_mm, "the bead is flipped"

"""設計等価バリアント(依頼A)と締結点摂動(依頼B)の不変条件。2026-09-06。

OCCT を呼ばない範囲: 候補の列挙が締結点・座面半径・板厚・曲げR を変えないこと。
"""
from __future__ import annotations

import dataclasses
import random

from synthetic_generator.families import (
    Knobs, bead_part, branch_part, channel_seat_part, flat_plate_part, flange_part, rib_part,
    three_point_part,
)
from synthetic_generator.variants import face_diff, perturb_spec, propose_variants, spec_from_meta


def _meta(kind, generator, seed=20260906):
    rng = random.Random(seed)
    knobs = Knobs(gentle=(kind == "flange"))
    while True:
        got = generator(rng, knobs)
        if got is not None:
            spec, bead, flange, rib = got
            return {"part_id": f"T_{kind}", "kind": kind, "spec": dataclasses.asdict(spec),
                    "bead": dataclasses.asdict(bead) if bead else None,
                    "flange": dataclasses.asdict(flange) if flange else None,
                    "rib": dataclasses.asdict(rib) if rib else None}


def _fixed(spec):
    return (spec.point1, spec.point2, spec.extra_points, spec.annotated_points,
            spec.min_bearing_radius_mm, spec.thickness_mm, spec.bend_radius_mm)


def test_variants_keep_fastening_points_and_continuous_spec():
    for kind, gen in (("bead", bead_part), ("flange", flange_part), ("rib", rib_part),
                      ("three_point", three_point_part), ("flat_plate", flat_plate_part),
                      ("branch", branch_part), ("channel_seat", channel_seat_part)):
        meta = _meta(kind, gen)
        spec, *_ = spec_from_meta(meta)
        variants = propose_variants(meta, count=8)
        assert variants, f"{kind}: 候補が1つも出ない"
        names = {v.name(meta["part_id"]) for v in variants}
        assert len(names) == len(variants), "名前が衝突"
        for v in variants:
            assert _fixed(v.spec) == _fixed(spec), f"{kind}/{v.knob}: 固定すべき値が動いた"
            assert v.changed, "何を変えたかが記録されていない"


def test_two_fold_bead_part_gets_slack_and_bead_and_width_knobs():
    rng = random.Random(3)
    while True:
        got = bead_part(rng, Knobs(fold_weights=((2, 1.0),)))
        if got is not None and got[0].target_folds == 2:
            break
    spec, bead, flange, rib = got
    meta = {"part_id": "T2", "kind": "bead", "spec": dataclasses.asdict(spec),
            "bead": dataclasses.asdict(bead), "flange": None, "rib": None}
    knobs = {v.knob for v in propose_variants(meta, count=12)}
    assert {"slack", "bead", "width"} <= knobs


def test_perturbation_moves_exactly_one_point_in_plane():
    meta = _meta("bead", bead_part)
    spec, *_ = spec_from_meta(meta)
    index, vector, variant, original = perturb_spec(meta, random.Random(1))
    moved = (variant.spec.point1, variant.spec.point2)[index]
    other = (variant.spec.point2, variant.spec.point1)[index]
    assert other == (spec.point2, spec.point1)[index]
    assert abs(sum(v * n for v, n in zip(vector, original.normal_xyz))) < 1e-9, "面内移動"
    dist = sum(c * c for c in vector) ** 0.5
    assert 0.5 * spec.min_bearing_radius_mm <= dist <= 2.0 * spec.min_bearing_radius_mm
    assert moved.normal_xyz == original.normal_xyz


def test_face_diff_reports_moved_added_removed():
    before = [{"name": "a", "area_mm2": 100.0, "centroid": [0, 0, 0]},
              {"name": "b", "area_mm2": 100.0, "centroid": [0, 0, 0]},
              {"name": "gone", "area_mm2": 5.0, "centroid": [0, 0, 0]}]
    after = [{"name": "a", "area_mm2": 100.5, "centroid": [0.1, 0, 0]},
             {"name": "b", "area_mm2": 130.0, "centroid": [0, 0, 0]},
             {"name": "new", "area_mm2": 5.0, "centroid": [0, 0, 0]}]
    d = face_diff(before, after)
    assert d == {"changed": ["b"], "unchanged": ["a"], "added": ["new"], "removed": ["gone"]}

import dataclasses
import json
import pathlib

import pytest

from synthetic_generator.batch_generate import generate_batch, generate_general_batch


@dataclasses.dataclass(frozen=True)
class _FakeGeneratedPart:
    stp_path: str
    catpart_path: str


class _FakeBuilder:
    """CATIAを呼ばないスタブ。オーケストレーション(サンプリング〜joints.json書き出し)だけを検証する。"""

    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def build_parallel_same_offset(self, spec, reinforcement, out_dir, part_name):
        self.calls.append((spec, reinforcement, out_dir, part_name))
        return _FakeGeneratedPart(
            stp_path=f"{out_dir}/{part_name}_mid.stp", catpart_path=f"{out_dir}/{part_name}_mid.CATPart"
        )


def test_generate_batch_writes_joints_json_and_calls_builder_per_part(tmp_path: pathlib.Path) -> None:
    builder = _FakeBuilder()

    records = generate_batch(builder, tmp_path, count=3, seed=123)

    assert len(records) == 3
    assert len(builder.calls) == 3
    assert all(r.catpart_path.endswith(".CATPart") for r in records)

    joints_path = tmp_path / "annotations" / "joints.json"
    assert joints_path.exists()
    data = json.loads(joints_path.read_text(encoding="utf-8"))
    assert len(data["parts"]) == 3
    assert len(data["joints"]) == 6  # 締結点2点 x 3部品
    assert all(j["confidence"] == "synthetic" for j in data["joints"])
    assert all(p["tag"] == "sheet_metal" for p in data["parts"])


def test_generate_batch_is_deterministic_for_same_seed(tmp_path: pathlib.Path) -> None:
    records_a = generate_batch(_FakeBuilder(), tmp_path / "a", count=2, seed=99)
    records_b = generate_batch(_FakeBuilder(), tmp_path / "b", count=2, seed=99)

    assert [r.spec for r in records_a] == [r.spec for r in records_b]
    assert [r.reinforcement for r in records_a] == [r.reinforcement for r in records_b]


class _FlakyBuilder:
    """最初のN回はInfeasible(ValueError)、その後成功するスタブ(2026-08-07 skip-and-retry検証用)。"""

    def __init__(self, fail_first_n: int) -> None:
        self.fail_first_n = fail_first_n
        self.call_count = 0

    def build_parallel_same_offset(self, spec, reinforcement, out_dir, part_name):
        self.call_count += 1
        if self.call_count <= self.fail_first_n:
            raise ValueError("infeasible (simulated)")
        return _FakeGeneratedPart(
            stp_path=f"{out_dir}/{part_name}_mid.stp", catpart_path=f"{out_dir}/{part_name}_mid.CATPart"
        )


def test_generate_batch_skips_infeasible_and_still_reaches_count(tmp_path: pathlib.Path) -> None:
    builder = _FlakyBuilder(fail_first_n=3)

    records = generate_batch(builder, tmp_path, count=2, seed=1)

    assert len(records) == 2
    assert builder.call_count == 5  # 3回失敗 + 2回成功


class _AlwaysCrashingBuilder:
    """CATIA自体の失敗(ValueError以外)を模す — バッチ全体を中断すべき。"""

    def build_parallel_same_offset(self, spec, reinforcement, out_dir, part_name):
        raise RuntimeError("simulated CATIA-level failure")


def test_generate_batch_does_not_swallow_non_value_error(tmp_path: pathlib.Path) -> None:
    with pytest.raises(RuntimeError, match="simulated CATIA-level failure"):
        generate_batch(_AlwaysCrashingBuilder(), tmp_path, count=2, seed=1)


class _AlwaysInfeasibleBuilder:
    def build_parallel_same_offset(self, spec, reinforcement, out_dir, part_name):
        raise ValueError("always infeasible")


def test_generate_batch_gives_up_after_max_attempts(tmp_path: pathlib.Path) -> None:
    with pytest.raises(RuntimeError, match="Infeasible"):
        generate_batch(_AlwaysInfeasibleBuilder(), tmp_path, count=1, seed=1, max_attempts_per_part=5)


class _FakeGeneralBuilder:
    """CATIAを呼ばないスタブ。generate_general_batch用(2026-08-10)。"""

    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def build_general_two_point(
        self, point1, point2, *, min_bearing_radius_mm, half_width_mm, bend_radius_mm,
        fold1_slack_mm, fold2_slack_mm, fold1_tilt_perturbation_rad, out_dir, part_name,
        bead=None, flange=None,
    ):
        self.calls.append((point1, point2, out_dir, part_name, bead))
        return _FakeGeneratedPart(
            stp_path=f"{out_dir}/{part_name}_mid.stp", catpart_path=f"{out_dir}/{part_name}_mid.CATPart"
        )


def test_generate_general_batch_writes_joints_json_and_calls_builder_per_part(tmp_path: pathlib.Path) -> None:
    builder = _FakeGeneralBuilder()

    records = generate_general_batch(builder, tmp_path, count=3, seed=123)

    assert len(records) == 3
    assert len(builder.calls) == 3
    assert all(r.catpart_path.endswith(".CATPart") for r in records)

    joints_path = tmp_path / "annotations" / "joints.json"
    assert joints_path.exists()
    data = json.loads(joints_path.read_text(encoding="utf-8"))
    assert len(data["parts"]) == 3
    assert len(data["joints"]) == 6  # 締結点2点 x 3部品


def test_generate_general_batch_is_deterministic_for_same_seed(tmp_path: pathlib.Path) -> None:
    records_a = generate_general_batch(_FakeGeneralBuilder(), tmp_path / "a", count=2, seed=99)
    records_b = generate_general_batch(_FakeGeneralBuilder(), tmp_path / "b", count=2, seed=99)

    assert [r.spec for r in records_a] == [r.spec for r in records_b]


class _FlakyGeneralBuilder:
    def __init__(self, fail_first_n: int) -> None:
        self.fail_first_n = fail_first_n
        self.call_count = 0

    def build_general_two_point(self, point1, point2, **kwargs):
        self.call_count += 1
        if self.call_count <= self.fail_first_n:
            raise ValueError("infeasible (simulated)")
        out_dir = kwargs["out_dir"]
        part_name = kwargs["part_name"]
        return _FakeGeneratedPart(
            stp_path=f"{out_dir}/{part_name}_mid.stp", catpart_path=f"{out_dir}/{part_name}_mid.CATPart"
        )


def test_generate_general_batch_skips_infeasible_and_still_reaches_count(tmp_path: pathlib.Path) -> None:
    builder = _FlakyGeneralBuilder(fail_first_n=3)

    records = generate_general_batch(builder, tmp_path, count=2, seed=1)

    assert len(records) == 2
    assert builder.call_count == 5


def test_generate_general_batch_writes_params_json_per_part(tmp_path: pathlib.Path) -> None:
    """部品ごとの生成パラメータ(spec・ビード・傾き実績値)がparams/に保存される(SS12)。"""
    builder = _FakeGeneralBuilder()
    records = generate_general_batch(builder, tmp_path, count=2, seed=123)

    for record in records:
        params_path = tmp_path / "params" / f"{record.part_id}.json"
        assert params_path.exists()
        data = json.loads(params_path.read_text(encoding="utf-8"))
        assert data["part_id"] == record.part_id
        assert data["bead"] is None and data["flange"] is None  # 補強確率0(既定)
        assert "fold_tilts_deg" in data and "geometry_label" in data
        # specはそのまま形状を再構築できる完全な記録であること
        assert data["spec"]["half_width_mm"] == record.spec.half_width_mm
        assert data["spec"]["point1"]["position_xyz"] == list(record.spec.point1.position_xyz)


def test_resolve_bead_slacks_returns_feasible_combination() -> None:
    """リゾルバの返すslack/ビードは、builderが使うのと同一の権威チェックを通る(SS12)。"""
    import random

    from synthetic_generator.bead import sample_bead
    from synthetic_generator.general_geometry import check_bead_feasible, plan_general_two_point
    from synthetic_generator.templates.general_two_point import (
        resolve_bead_slacks,
        sample as sample_general,
    )

    rng = random.Random(20260825)
    resolved_count = 0
    for _ in range(40):
        spec = sample_general(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        result = resolve_bead_slacks(rng, spec, bead)
        if result is None:
            continue
        resolved_count += 1
        new_spec, new_bead = result
        # 締結点は不変(slackとビードだけが差し替わる)
        assert new_spec.point1 == spec.point1 and new_spec.point2 == spec.point2
        plan = plan_general_two_point(
            new_spec.point1,
            new_spec.point2,
            min_bearing_radius_mm=new_spec.min_bearing_radius_mm,
            half_width_mm=new_spec.half_width_mm,
            bend_radius_mm=new_spec.bend_radius_mm,
            fold1_slack_mm=new_spec.fold1_slack_mm,
            fold2_slack_mm=new_spec.fold2_slack_mm,
            fold1_tilt_perturbation_rad=new_spec.fold1_tilt_perturbation_rad,
        )
        check_bead_feasible(plan, new_bead)  # 通らなければValueErrorで落ちる
    assert resolved_count > 0, "40試行で1件も解決できないのはサンプラーが破綻している"

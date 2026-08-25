"""締結点2点部品をバッチ生成する(Phase 1 PoCのエントリポイント)。

2つの生成経路がある: `generate_batch`(法線がほぼ一致するparallel_same_offsetクラス専用、
フランジ・Phase 1.5トリム対応)と`generate_general_batch`(任意の法線・任意の位置、
roadmap SS6.20〜6.22。フランジ・トリムは未対応、まずメイン形状生成の成功を優先する
2026-08-10の方針転換による)。

実機(CATIA V5-6)で動作確認済み。builder引数にスタブを渡せば、サンプリング〜
joints.json書き出しまでのオーケストレーションだけをCATIA無しでテストできる
(tests/test_batch_generate.py参照)。

生成物(STP/CATPart/joints.json)は既定で`synthetic_parts/`配下に出力する(2026-08-10、
本パッケージ自体をAutoMetalSheetから独立したPartMakerリポジトリへ移管したため、
コード・生成物とも本リポジトリ内で完結する)。
"""

from __future__ import annotations

import dataclasses
import pathlib
import random
from typing import Protocol

from synthetic_generator.annotate import build_two_joint_pair
from synthetic_generator.bead import BeadParams, sample_bead
from synthetic_generator.annotation_schema import AnnotationDocument, PartEntry
from synthetic_generator.reinforcement import ReinforcementParams, sample_reinforcement
from synthetic_generator.templates.general_two_point import GeneralTwoJointSpec
from synthetic_generator.templates.general_two_point import sample as sample_general_two_point
from synthetic_generator.templates.parallel_same_offset import TwoJointSpec
from synthetic_generator.templates.parallel_same_offset import sample as sample_two_joint_spec

DEFAULT_OUTPUT_ROOT = pathlib.Path(r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_parts")


class GeneratedPartLike(Protocol):
    stp_path: str
    catpart_path: str


class PartBuilder(Protocol):
    def build_parallel_same_offset(
        self, spec: TwoJointSpec, reinforcement: ReinforcementParams, out_dir: str, part_name: str
    ) -> GeneratedPartLike: ...


class GeneralPartBuilder(Protocol):
    def build_general_two_point(
        self,
        point1,
        point2,
        *,
        min_bearing_radius_mm: float,
        half_width_mm: float,
        bend_radius_mm: float,
        fold1_slack_mm: float,
        fold2_slack_mm: float,
        fold1_tilt_perturbation_rad: float,
        out_dir: str,
        part_name: str,
        bead: BeadParams | None = None,
    ) -> GeneratedPartLike: ...


@dataclasses.dataclass(frozen=True)
class GeneratedPartRecord:
    part_id: str
    spec: TwoJointSpec
    reinforcement: ReinforcementParams
    stp_path: str
    catpart_path: str


def generate_batch(
    builder: PartBuilder,
    out_dir: pathlib.Path = DEFAULT_OUTPUT_ROOT,
    *,
    count: int,
    seed: int,
    max_attempts_per_part: int = 50,
) -> list[GeneratedPartRecord]:
    """count件の成功パーツを生成する。

    2026-08-07変更: sample()側の事前チェック(bearing radius/幅方向/ジョグ折れ重なり等)
    によるInfeasible(ValueError)は、CATIA自体の不具合ではなく正常なランダムサンプリングの
    一部として頻繁に発生するようになった(§6.15/6.16参照、歩留まり40〜60%程度)。以前の
    「失敗したらアボートでいい」方針はInfeasible率がほぼ0%だった頃のものだったため、
    ValueErrorはスキップして次のシード(rngは引き続き進む)で再試行するよう変更した。
    CATIA自体の失敗(com_error等、ValueError以外の例外)は従来通りバッチ全体を中断する
    (これは実際のバグの兆候であり握りつぶすべきではないため)。1パーツあたり
    max_attempts_per_part回試行しても成功しない場合は異常事態としてRuntimeErrorを送出する。
    """
    rng = random.Random(seed)
    out_dir = pathlib.Path(out_dir)

    doc = AnnotationDocument(assembly_dir=out_dir, full_assembly_stp="synthetic")
    records: list[GeneratedPartRecord] = []

    for i in range(1, count + 1):
        part_id = f"SYN_parallel_same_offset_{i:04d}"

        for attempt in range(max_attempts_per_part):
            spec = sample_two_joint_spec(rng)
            reinforcement = sample_reinforcement(spec.thickness_mm, rng)
            try:
                generated = builder.build_parallel_same_offset(spec, reinforcement, str(out_dir / "mid"), part_id)
                break
            except ValueError:
                continue
        else:
            raise RuntimeError(
                f"{part_id}: {max_attempts_per_part}回連続でInfeasibleだった。"
                "サンプリング範囲自体が破綻している可能性がある。"
            )

        doc.parts[part_id] = PartEntry(
            part_id=part_id,
            stp_file=f"mid/{part_id}_mid.stp",
            vtp_file="",
            tag="sheet_metal",
            thickness_mm=spec.thickness_mm,
            thickness_source="synthetic_generator",
        )
        for joint in build_two_joint_pair(part_id, spec.point1, spec.point2, spec.hole_diameter_mm):
            doc.add_joint(joint)

        records.append(
            GeneratedPartRecord(
                part_id=part_id,
                spec=spec,
                reinforcement=reinforcement,
                stp_path=generated.stp_path,
                catpart_path=generated.catpart_path,
            )
        )

    doc.save()
    return records


@dataclasses.dataclass(frozen=True)
class GeneratedGeneralPartRecord:
    part_id: str
    spec: GeneralTwoJointSpec
    stp_path: str
    catpart_path: str
    bead: BeadParams | None = None


def generate_general_batch(
    builder: GeneralPartBuilder,
    out_dir: pathlib.Path = DEFAULT_OUTPUT_ROOT,
    *,
    count: int,
    seed: int,
    max_attempts_per_part: int = 50,
    bead_probability: float = 0.0,
) -> list[GeneratedGeneralPartRecord]:
    """任意の法線・任意の位置の締結点ペア(roadmap SS6.20〜6.22)でcount件の成功パーツを
    生成する。`generate_batch`(parallel_same_offsetクラス専用)と同じskip-and-retry
    方針(ValueErrorはスキップして次のシードで再試行、それ以外の例外はバッチ全体を中断)。
    フランジ・Phase 1.5トリムは未対応(`build_general_two_point`自体が未対応、SS6.21参照)。
    """
    rng = random.Random(seed)
    out_dir = pathlib.Path(out_dir)

    doc = AnnotationDocument(assembly_dir=out_dir, full_assembly_stp="synthetic")
    records: list[GeneratedGeneralPartRecord] = []

    for i in range(1, count + 1):
        part_id = f"SYN_general_two_point_{i:04d}"

        for attempt in range(max_attempts_per_part):
            spec = sample_general_two_point(rng)
            # bead_probability=0(既定)ではrngを一切消費しない — 既存バッチのシード列を
            # そのまま再現できるようにするため。
            bead = (
                sample_bead(rng, spec.half_width_mm)
                if bead_probability > 0.0 and rng.random() < bead_probability
                else None
            )
            try:
                generated = builder.build_general_two_point(
                    spec.point1,
                    spec.point2,
                    min_bearing_radius_mm=spec.min_bearing_radius_mm,
                    half_width_mm=spec.half_width_mm,
                    bend_radius_mm=spec.bend_radius_mm,
                    fold1_slack_mm=spec.fold1_slack_mm,
                    fold2_slack_mm=spec.fold2_slack_mm,
                    fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                    out_dir=str(out_dir / "mid"),
                    part_name=part_id,
                    bead=bead,
                )
                break
            except ValueError:
                continue
        else:
            raise RuntimeError(
                f"{part_id}: {max_attempts_per_part}回連続でInfeasibleだった。"
                "サンプリング範囲自体が破綻している可能性がある。"
            )

        doc.parts[part_id] = PartEntry(
            part_id=part_id,
            stp_file=f"mid/{part_id}_mid.stp",
            vtp_file="",
            tag="sheet_metal",
            thickness_mm=spec.thickness_mm,
            thickness_source="synthetic_generator",
        )
        for joint in build_two_joint_pair(part_id, spec.point1, spec.point2, spec.hole_diameter_mm):
            doc.add_joint(joint)

        records.append(
            GeneratedGeneralPartRecord(
                part_id=part_id,
                spec=spec,
                stp_path=generated.stp_path,
                catpart_path=generated.catpart_path,
                bead=bead,
            )
        )

    doc.save()
    return records

"""生成パラメータから joints.json 用の Joint を直接組み立てる。

人手アノテーションと異なり生成時に真値が既知なので、annotation_schema(2026-08-10、
AutoMetalSheetのannotation_tool.schemaからPartMaker独立のためベンダーコピーしたもの)を
再利用してそのまま書き出せる。各締結点は1部品のみを参照する単独joint
(mounting_holeの実データパターンと同じ、schema.pyのJointはparts>=1を許容)として
記録する — 合成生成では相手部品(締結相手)そのものは生成対象外のため。
"""

from __future__ import annotations

from synthetic_generator.annotation_schema import Axis, Joint, PartRef

from synthetic_generator.classify import FasteningPoint


def build_fastening_joint(joint_id: str, part_id: str, point: FasteningPoint, hole_diameter_mm: float) -> Joint:
    return Joint(
        joint_id=joint_id,
        type="mounting_hole",
        parts=[part_id],
        axis=Axis(
            start_xyz=point.position_xyz,
            direction_xyz=point.normal_xyz,
            length_mm=hole_diameter_mm,
        ),
        per_part=[
            PartRef(
                part_id=part_id,
                hole_center_xyz=point.position_xyz,
                hole_diameter_mm=hole_diameter_mm,
                detection_method="manual",
            )
        ],
        confidence="synthetic",
    )


def build_two_joint_pair(part_id: str, point1: FasteningPoint, point2: FasteningPoint, hole_diameter_mm: float) -> list[Joint]:
    return [
        build_fastening_joint(f"{part_id}_j0001", part_id, point1, hole_diameter_mm),
        build_fastening_joint(f"{part_id}_j0002", part_id, point2, hole_diameter_mm),
    ]

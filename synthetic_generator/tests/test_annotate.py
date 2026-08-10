from synthetic_generator.annotate import build_two_joint_pair
from synthetic_generator.classify import FasteningPoint


def test_build_two_joint_pair_produces_synthetic_confidence_joints() -> None:
    point1 = FasteningPoint(position_xyz=(0.0, 0.0, 0.0), normal_xyz=(0.0, 0.0, 1.0))
    point2 = FasteningPoint(position_xyz=(50.0, 0.0, 20.0), normal_xyz=(0.0, 0.0, 1.0))

    joints = build_two_joint_pair("SYN_parallel_same_offset_0001", point1, point2, hole_diameter_mm=8.0)

    assert len(joints) == 2
    for joint, point in zip(joints, (point1, point2)):
        assert joint.confidence == "synthetic"
        assert joint.parts == ["SYN_parallel_same_offset_0001"]
        assert joint.axis.start_xyz == point.position_xyz
        assert joint.axis.direction_xyz == point.normal_xyz
        assert joint.per_part[0].hole_diameter_mm == 8.0

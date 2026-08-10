"""2締結点のconfiguration classを法線関係+同一平面判定で分類する。

閾値は docs/synthetic_two_joint_generation_roadmap.md SS3.1 で実データ
(3アセンブリ、31,846組の締結点ペア)から検証済み: 内積の自然なギャップが
+-0.9 / +-0.15 付近にあり、parallel_same 内の同一平面クラスタは
オフセット 1.0mm 未満に集中する。
"""

from __future__ import annotations

import dataclasses
import math
from typing import Literal

ConfigClass = Literal[
    "coplanar_flat",
    "parallel_same_offset",
    "parallel_opposite",
    "orthogonal",
    "oblique",
]

Vec3 = tuple[float, float, float]

DOT_PARALLEL_THRESHOLD = 0.9
DOT_ORTHOGONAL_THRESHOLD = 0.15
COPLANAR_OFFSET_TOLERANCE_MM = 1.0

# coplanar_flat のみが非強度部品(docs SS3: 「強度部品でないのは締結点が同一平面の場合だけ」)
NON_STRENGTH_CLASSES: frozenset[ConfigClass] = frozenset({"coplanar_flat"})

# ユーザー製造制約(2026-08-06): 中立面Rの最小値。ジョグの折れ目・フランジ根本の折れ目を
# 含む全てのフィレット半径に適用する。templates/parallel_same_offset.pyとreinforcement.py
# の両方が参照するため、どちらにも依存しないこのモジュールに置く。
MIN_NEUTRAL_PLANE_RADIUS_MM = 4.0


def _normalize(v: Vec3) -> Vec3:
    length = math.sqrt(sum(c * c for c in v))
    if length < 1e-9:
        raise ValueError(f"cannot normalize near-zero vector: {v}")
    return (v[0] / length, v[1] / length, v[2] / length)


def _dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


@dataclasses.dataclass(frozen=True)
class FasteningPoint:
    """締結点1つ分。normal_xyz は joints.json の axis.direction_xyz(板厚方向)に対応。"""

    position_xyz: Vec3
    normal_xyz: Vec3


def classify(point1: FasteningPoint, point2: FasteningPoint) -> ConfigClass:
    n1 = _normalize(point1.normal_xyz)
    n2 = _normalize(point2.normal_xyz)
    d = _dot(n1, n2)

    if d > DOT_PARALLEL_THRESHOLD:
        offset_mm = abs(_dot(_sub(point2.position_xyz, point1.position_xyz), n1))
        if offset_mm < COPLANAR_OFFSET_TOLERANCE_MM:
            return "coplanar_flat"
        return "parallel_same_offset"
    if d < -DOT_PARALLEL_THRESHOLD:
        return "parallel_opposite"
    if -DOT_ORTHOGONAL_THRESHOLD < d < DOT_ORTHOGONAL_THRESHOLD:
        return "orthogonal"
    return "oblique"


def is_strength_part(config_class: ConfigClass) -> bool:
    return config_class not in NON_STRENGTH_CLASSES


def tangent_length_for_bend_angle_rad(bend_angle_rad: float, radius_mm: float) -> float:
    """ある折れ目(外向きの曲がり角 bend_angle_rad)にradius_mmのエッジフィレットを付けたとき、
    理論上のシャープな角から実際にフィレットが始まる点までの距離(接線長)。

    bend_angle_rad=90度(垂直な折れ)のとき接線長はradius_mmそのものに一致する。
    ジョグの折れ(fold_tangent_length_mm)にもフランジ根本の折れ(flange_angle_degが
    そのままbend_angleに対応する — flat_flangeのdirection = sin(angle)*normal +
    cos(angle)*outward_tangentという定義上、angle=0は折れなしの平坦延長、angle=90度は
    垂直な折れで、ジョグのalpha = atan2(offset, ramp_extent)と同じ意味を持つ)にも
    同じ式を使う。
    """
    return radius_mm * math.tan(bend_angle_rad / 2.0)


def fold_tangent_length_mm(offset_mm: float, ramp_extent_mm: float, radius_mm: float) -> float:
    """ジョグの折れ目(flat-ランプ間)にradius_mmのエッジフィレットを付けたときの接線長。
    ランプの傾き角(水平からの角度)を alpha = atan2(offset_mm, ramp_extent_mm) として
    tangent_length_for_bend_angle_rad(alpha, radius_mm) を返す。

    templates/parallel_same_offset.py(締結点の必要最小半径を侵さない範囲でbend_radius_mm
    を実行可能サンプリングするため)とgsd_build.py(実際の構築前の最終安全確認)の両方から
    同じ式を使う必要があるため、どちらにも依存しないこのモジュールに置いている。
    """
    alpha = math.atan2(offset_mm, ramp_extent_mm)
    return tangent_length_for_bend_angle_rad(alpha, radius_mm)

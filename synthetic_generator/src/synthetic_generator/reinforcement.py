"""強度部品(coplanar_flat以外)の補強構造パラメータをサンプリングする。

Phase 1スコープ(docs/synthetic_two_joint_generation_roadmap.md SS6): parallel_same_offset
クラス + フランジ補強のみ。一般面の45度刻みファセット、ビード補強はPhase 1の
スコープ外(未実装)。

flange_angle_degは既定90度を主としつつ、稀に90度以外(斜めフランジ)もサンプリングする
ように変更した(2026-08-03)。ただし斜めフランジ時の「戻りフランジ」(先端に一般面と
平行なもう一つのフランジを追加する構造)はまだ未実装 — gsd_build.flat_flangeは
現状、角度可変の単純な片フランジのみを生成する。戻りフランジは次の増分。

ジョグ折れ目のbend_radius_mmは、締結点の必要最小半径を侵さない実行可能範囲でしか
成立しないため、このモジュールではなくtemplates/parallel_same_offset.py側
(ジョグ幾何と同時にサンプリングできる場所)に移した(2026-08-06、歩留まり改善)。

flange_bend_radius_mm(フランジ根本の折れ目半径、2026-08-06追加、SS6.11 Step2):
ユーザー確定方針により、メイン形状のbend_radius_mm(R4〜50mm、応力集中回避重視)とは
別に、板厚の0〜3倍程度の小さめのレンジでサンプリングする(実務のフランジは剛性の
ビームフランジとして機能するようシャープさを保つのが基本のため)。flange_height_mmを
超えないようクランプする(隣接パネル長を超えるフィレット半径は実機でCATIAのUpdateが
失敗することがSS6.9で判明したため、同じ考え方を踏襲)。締結点の必要最小半径の保護
(flange_margin拡張)はgsd_build.py側で行う — flange_angle_degとmin_bearing_radius_mmが
両方揃わないと計算できないため。

ユーザー製造制約(2026-08-06): 中立面Rは全て最小R4(MIN_NEUTRAL_PLANE_RADIUS_MM、
classify.py)を守る。flange_height_mmが小さくR4以上のフィレットが物理的に成立しない
場合はあえて4mmを引いておき、gsd_build.py側の事前チェックでInfeasibleとして
明示的に弾かれるようにする。
"""

from __future__ import annotations

import dataclasses
import random

from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM

CORNER_RADIUS_MIN_MM = 5.0  # ユーザー製造制約: コーナー最小Rは外R5
FLANGE_ANGLE_DEFAULT_DEG = 90.0  # ユーザー訂正: フランジは90度で立てるのが基本
FLANGE_ANGLE_DEFAULT_PROBABILITY = 0.85  # 90度が主、稀に斜めフランジ
FLANGE_ANGLE_OBLIQUE_RANGE_DEG = (60.0, 120.0)  # 90度以外を引いた場合のサンプリング範囲
FLANGE_BEND_RADIUS_RATIO_CAP = 3.0  # ユーザー確定: 板厚の1〜3倍程度の小さめのR


@dataclasses.dataclass(frozen=True)
class ReinforcementParams:
    reinforcement_direction_deg: float
    flange_height_mm: float
    flange_angle_deg: float
    corner_radius_mm: float
    flange_bend_radius_mm: float


def sample_reinforcement(
    thickness_mm: float,
    rng: random.Random,
    *,
    flange_height_ratio_range: tuple[float, float] = (3.0, 8.0),
    corner_radius_max_mm: float = 15.0,
) -> ReinforcementParams:
    if thickness_mm <= 0:
        raise ValueError(f"thickness_mm must be positive, got {thickness_mm}")
    if corner_radius_max_mm < CORNER_RADIUS_MIN_MM:
        raise ValueError(
            f"corner_radius_max_mm ({corner_radius_max_mm}) must be >= "
            f"CORNER_RADIUS_MIN_MM ({CORNER_RADIUS_MIN_MM})"
        )

    height_ratio = rng.uniform(*flange_height_ratio_range)
    flange_height = height_ratio * thickness_mm
    if rng.random() < FLANGE_ANGLE_DEFAULT_PROBABILITY:
        flange_angle = FLANGE_ANGLE_DEFAULT_DEG
    else:
        flange_angle = rng.uniform(*FLANGE_ANGLE_OBLIQUE_RANGE_DEG)

    # flange_height_mmを超えるフィレット半径は隣接パネル長を超えて破綻しうるため
    # (SS6.9のbend_radius_mm vs half_widthの教訓と同じ考え方)、0.9倍を上限にクランプする。
    max_flange_bend_radius = min(FLANGE_BEND_RADIUS_RATIO_CAP * thickness_mm, 0.9 * flange_height)
    # 中立面R最小4mmを厳守。max_flange_bend_radiusが4mm未満(flange_height_mmが小さすぎる)
    # 場合はあえて4mmを引いておき、gsd_build.py側の事前チェックで明示的に弾かれるようにする。
    flange_bend_radius = rng.uniform(
        MIN_NEUTRAL_PLANE_RADIUS_MM, max(MIN_NEUTRAL_PLANE_RADIUS_MM, max_flange_bend_radius)
    )

    return ReinforcementParams(
        reinforcement_direction_deg=rng.uniform(0.0, 180.0),
        flange_height_mm=flange_height,
        flange_angle_deg=flange_angle,
        corner_radius_mm=rng.uniform(CORNER_RADIUS_MIN_MM, corner_radius_max_mm),
        flange_bend_radius_mm=flange_bend_radius,
    )

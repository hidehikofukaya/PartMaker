"""部品族ごとの生成器(2026-09-04、ユーザー提案で導入)。

**何を・どれだけ・どんな多様性で作るかは、この外側(レシピ)が決める。**

それまでは `templates.general_two_point.resolve_reinforcement` が
「フランジ条件なら…、駄目ならビード、それも駄目ならリブ」と1つの決定論で全部を
決めていた。これは片方を直すと片方が消える構造で、実際に

* ビードの置き場所探索を賢くする → リブが0%になる
* リブの条件を緩める → ビードが置ける部品にまでリブが載る

という往復が起きた(2026-09-04)。族ごとに「その特徴が成立する配置を**狙って**引く」
生成器に分ければ、比率は外から与えるだけで済み、互いに干渉しない。

各生成器は `(spec, bead, flange, rib)` を返す(該当しないものは None)。
成立しなければ None を返すので、呼び出し側は次のシードで引き直す。
"""

from __future__ import annotations

import dataclasses
import math
import random

from synthetic_generator.bead import BeadParams, sample_bead
from synthetic_generator.classify import classify
from synthetic_generator.flange import (
    FLANGE_MAX_FOLD_ANGLE_DEG,
    FlangeParams,
    max_fold_angle_deg,
    plan_flange_on_surface,
    sample_flange,
)
from synthetic_generator.general_geometry import (
    bead_room_mm,
    check_bead_feasible_occt,
    plan_for,
)
from synthetic_generator.rib import (
    RIB_BEND_RADIUS_MM,
    RIB_MAX_BEAD_ROOM_MM,
    RIB_MIN_FOLD_ANGLE_DEG,
    RibParams,
    leg_room_mm,
    sample_rib,
)
from synthetic_generator.templates.general_two_point import (
    FOLD_SLACK_RANGE_MM,
    GeneralTwoJointSpec,
    draw_fold_count,
)
from synthetic_generator.templates.general_two_point import sample as sample_spec

# ビードが通る折れ目位置を探すときの slack の引き直し回数
# (当初の `resolve_bead_slacks` と同じ狙い: 置き場所ではなく折れ目の位置で解決する)。
SLACK_ATTEMPTS = 16


@dataclasses.dataclass(frozen=True)
class Knobs:
    """族ごとの多様性つまみ。レシピ(JSON)からそのまま作れる。"""

    # 締結点間距離(w直交断面で測る)。Noneならサンプラーの既定。
    distance_mm: tuple[float, float] | None = None
    # 座面の必要半径(=締結点まわりに要る平地の半径)。
    bearing_radius_mm: tuple[float, float] | None = None
    # 曲げ本数の重み [[本数, 重み], ...]。Noneなら既定の配分。
    fold_weights: tuple[tuple[int, float], ...] | None = None
    # 配置クラスの絞り込み(classify()の文字列)。Noneなら制限なし。
    classes: frozenset[str] | None = None
    # 緩い折れを狙う(フランジ帯)。
    gentle: bool = False
    # 1部品あたりの純Python試行の上限。
    attempts: int = 200

    @staticmethod
    def from_dict(data: dict) -> "Knobs":
        return Knobs(
            distance_mm=tuple(data["distance_mm"]) if data.get("distance_mm") else None,
            bearing_radius_mm=(tuple(data["bearing_radius_mm"])
                               if data.get("bearing_radius_mm") else None),
            fold_weights=(tuple((int(n), float(w)) for n, w in data["fold_weights"])
                          if data.get("fold_weights") else None),
            classes=frozenset(data["classes"]) if data.get("classes") else None,
            gentle=bool(data.get("gentle", False)),
            attempts=int(data.get("attempts", 200)),
        )


Result = tuple[GeneralTwoJointSpec, BeadParams | None, FlangeParams | None, RibParams | None]


def _draw_spec(rng: random.Random, knobs: Knobs, *, folds: int | None = None):
    weights = knobs.fold_weights
    target = folds if folds is not None else (
        _draw_weighted(rng, weights) if weights else draw_fold_count(rng))
    return sample_spec(
        rng,
        gentle_folds=knobs.gentle,
        target_folds=target,
        section_distance_mm=knobs.distance_mm,
        bearing_radius_mm=knobs.bearing_radius_mm,
        target_classes=knobs.classes,
    )


def _draw_weighted(rng: random.Random, weights) -> int:
    roll, cumulative = rng.random(), 0.0
    for value, weight in weights:
        cumulative += weight
        if roll < cumulative:
            return value
    return weights[-1][0]


# ---------------------------------------------------------------- 族ごとの生成器


def bead_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """ビード部品: ビードが**全長を貫通して折れ目を全てまたぐ**配置を狙う。

    当初からの設計(roadmap Step 3)どおり、締結点の座面 2×bearing半径 を両端で
    避けたうえでビードを通しきる。通らない場合は置き場所を動かすのではなく
    **折れ目の位置(slack)を選び直す** — 折れ目の位置は
    `座面半径 + フィレット接線長 + slack` で決まるので、slack がビードの
    走行長を直接左右する。
    """
    for _ in range(knobs.attempts):
        try:
            spec = _draw_spec(rng, knobs)
        except ValueError:
            continue
        for attempt in range(SLACK_ATTEMPTS):
            candidate = spec if attempt == 0 else dataclasses.replace(
                spec,
                fold1_slack_mm=rng.uniform(*FOLD_SLACK_RANGE_MM),
                fold2_slack_mm=rng.uniform(*FOLD_SLACK_RANGE_MM),
            )
            try:
                plan = plan_for(candidate)
            except ValueError:
                continue
            if bead_room_mm(plan, candidate.bend_radius_mm) < RIB_MAX_BEAD_ROOM_MM:
                continue      # 短すぎてまともなビードにならない -> リブ族の領分
            for _ in range(12):
                bead = sample_bead(rng, candidate.half_width_mm)
                try:
                    check_bead_feasible_occt(plan, bead, candidate.bend_radius_mm)
                except ValueError:
                    continue
                return candidate, bead, None, None
            if candidate.target_folds in (0, 1):
                break         # slackが幾何を動かさない族。次のspecへ
    return None


def flange_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """フランジ部品: 最大折れ角20度以下(フランジが成立する帯)を狙う。"""
    aimed = dataclasses.replace(knobs, gentle=True)
    for _ in range(knobs.attempts):
        try:
            spec = _draw_spec(rng, aimed)
            plan = plan_for(spec)
        except ValueError:
            continue
        if max_fold_angle_deg(plan.panel_frames) > FLANGE_MAX_FOLD_ANGLE_DEG:
            continue
        flange = sample_flange(
            rng, plan.panel_frames, plan.fold_tilts,
            spec.half_width_mm, spec.bend_radius_mm, spec.point1.normal_xyz,
        )
        if flange is None:
            continue
        extension = (flange.extension_mm if flange.side < 0 else 0.0,
                     flange.extension_mm if flange.side > 0 else 0.0)
        try:
            wide = plan_for(spec, side_extension_mm=extension)
            plan_flange_on_surface(wide.panel_frames, flange,
                                   half_width_mm=spec.half_width_mm,
                                   fold_tangents=wide.fold_tangents)
        except ValueError:
            continue
        return spec, None, flange, None
    return None


def fold_angle_deg(frames, index: int) -> float:
    """折れ目 index の外向きの折れ角[度]。"""
    a, b = frames[index], frames[index + 1]
    cosine = max(-1.0, min(1.0, sum(a.u[i] * b.u[i] for i in range(3))))
    return math.degrees(math.acos(cosine))


def rib_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """リブ部品: 締結点が近く(ビードがまともに載らない)、折れ角がきつい配置を狙う。

    曲げRは中立面R最小に固定する(ユーザー指定の工程 — シャープな折れのままリブを
    作り、そのあとリブ幅の外側だけを最小Rでフィレット)。
    """
    for _ in range(knobs.attempts):
        try:
            spec = _draw_spec(rng, knobs, folds=_draw_weighted(
                rng, knobs.fold_weights) if knobs.fold_weights else 1)
            spec = dataclasses.replace(spec, bend_radius_mm=RIB_BEND_RADIUS_MM)
            plan = plan_for(spec)
        except ValueError:
            continue
        folds = len(plan.panel_frames) - 1
        if folds < 1 or max_fold_angle_deg(plan.panel_frames) < RIB_MIN_FOLD_ANGLE_DEG:
            continue
        if bead_room_mm(plan, spec.bend_radius_mm) >= RIB_MAX_BEAD_ROOM_MM:
            continue          # ビードがまともに載る -> ビード族の領分
        best = max(range(folds), key=lambda i: min(leg_room_mm(plan, i)))
        angle = math.radians(fold_angle_deg(plan.panel_frames, best))
        rib = sample_rib(rng, half_width_mm=spec.half_width_mm, fold_index=best,
                         leg_room_mm=leg_room_mm(plan, best), fold_angle_rad=angle)
        if rib is not None:
            return spec, None, None, rib
    return None


def plain_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """補強なしの部品(パッチ)。**明確なルールで作る**(ユーザー指定 2026-09-04):

    1. 締結点同士がリブ部品と同等かそれ以上に短い
       — リブと同じ構造的判定 `bead_room_mm < RIB_MAX_BEAD_ROOM_MM` を使う。
    2. 曲げが0〜1回で、リブを生成する余地がない
       — 折れ角が45度未満、またはリブの寸法が取れない。

    つまり「小さすぎて何も入らない当て板」に相当する部品だけを作る。
    偶然の無補強ではなく、この条件を満たすものだけを通す。
    """
    for _ in range(knobs.attempts):
        folds = _draw_weighted(rng, knobs.fold_weights) if knobs.fold_weights else (
            0 if rng.random() < 0.3 else 1)
        if folds > 1:
            continue                      # 条件2: 曲げは0〜1回まで
        try:
            spec = _draw_spec(rng, knobs, folds=folds)
            spec = dataclasses.replace(spec, bend_radius_mm=RIB_BEND_RADIUS_MM)
            plan = plan_for(spec)
        except ValueError:
            continue
        # 条件1: 締結点が近い(ビードがまともに載らない)
        if bead_room_mm(plan, spec.bend_radius_mm) >= RIB_MAX_BEAD_ROOM_MM:
            continue
        # 条件2: リブの余地が無い
        actual_folds = len(plan.panel_frames) - 1
        if actual_folds >= 1:
            angle = fold_angle_deg(plan.panel_frames, 0)
            if angle >= RIB_MIN_FOLD_ANGLE_DEG:
                rib = sample_rib(rng, half_width_mm=spec.half_width_mm, fold_index=0,
                                 leg_room_mm=leg_room_mm(plan, 0),
                                 fold_angle_rad=math.radians(angle))
                if rib is not None:
                    continue      # リブが載る -> リブ族の領分
        return spec, None, None, None
    return None


FAMILIES = {
    "bead": bead_part,
    "flange": flange_part,
    "rib": rib_part,
    "plain": plain_part,
}


def kind_of(bead, flange, rib) -> str:
    return "bead" if bead else ("flange" if flange else ("rib" if rib else "plain"))


def classify_spec(spec) -> str:
    return str(classify(spec.point1, spec.point2))

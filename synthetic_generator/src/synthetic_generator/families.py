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
from synthetic_generator.classify import FasteningPoint, classify
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
    # 単曲げの折れ角レンジ[deg]。Noneならサンプラーの既定(20〜90)。
    turn_range_deg: tuple[float, float] | None = None
    # 帯の半幅 / 座面半径 の比。Noneなら既定(1.0〜1.3)。
    half_width_ratio: tuple[float, float] | None = None
    # 半幅の上限[mm]。Noneなら既定(25 = 帯幅50mm)。
    max_half_width_mm: float | None = None
    # 基準面の曲げR[mm]。Noneなら既定(5〜50、ただし半幅の0.9倍が上限)。
    bend_radius_mm: tuple[float, float] | None = None

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
            turn_range_deg=(tuple(data["turn_range_deg"])
                            if data.get("turn_range_deg") else None),
            half_width_ratio=(tuple(data["half_width_ratio"])
                              if data.get("half_width_ratio") else None),
            max_half_width_mm=(float(data["max_half_width_mm"])
                               if data.get("max_half_width_mm") else None),
            bend_radius_mm=(tuple(data["bend_radius_mm"])
                            if data.get("bend_radius_mm") else None),
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
        turn_range_deg=knobs.turn_range_deg,
        half_width_ratio_range=knobs.half_width_ratio,
        max_half_width_mm=knobs.max_half_width_mm,
        bend_radius_range_mm=knobs.bend_radius_mm,
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


# --- 3締結点(実車007/011の再現、2026-09-04) --------------------------------
# 実測(docs/PLAN_three_point_2026-09-04.md): 主曲げ1本・折れ角90度・法線が一致する
# 2点は同一パネル・側端フランジが曲げを跨ぐ、という骨格が両部品で共通していた。
THREE_POINT_TURN_RANGE_DEG = (45.0, 90.0)          # 実車 007=90.5 / 011=90.0
THREE_POINT_PAIR_DISTANCE_RANGE_MM = (20.0, 45.0)  # 実車 007=24.1 / 011=36.8
# 帯幅70mmまで許す(ユーザー決定 2026-09-04)。011の「対が帯幅方向に36.8mm離れる」型は
# 帯幅61mmを要するため。余白ルール(half_width >= bearing_radius)は不変で、帯を
# 広げるだけなので必要平面は一切減らない。
THREE_POINT_HALF_WIDTH_RATIO = (1.0, 2.8)
THREE_POINT_MAX_HALF_WIDTH_MM = 35.0
# 曲げRは既定では半幅から導かれるので、帯を70mmに広げるとRまで太る(実測R19.7〜27.8)。
# 実車は007=R10.83 / 011=R10.85 なので、この族だけ帯と切り離して実車帯に載せる。
THREE_POINT_BEND_RADIUS_RANGE_MM = (8.0, 18.0)
PAIR_PLACE_ATTEMPTS = 24


def _place_pair_mate(rng: random.Random, spec, plan) -> tuple[int, FasteningPoint] | None:
    """法線が一致する相方を、既存の締結点と**同じパネル**の上に置く。

    余白は従来ルールを踏襲(2026-09-04ユーザー指示): 帯端・曲げの接線・パネル端の
    いずれからも `min_bearing_radius_mm` 以上を空ける。単曲げの版組みでは
    point1 が panel0 の run=0、point2 が panel1 の run=0 に来る。
    """
    bearing = spec.min_bearing_radius_mm
    hosts = [(0, spec.point1), (1, spec.point2)]
    rng.shuffle(hosts)
    for index, anchor in hosts:
        frame = plan.panel_frames[index]
        near_cut, far_cut = plan.fold_tangents[index]
        lo = frame.near_run_mm + near_cut + bearing
        hi = frame.far_run_mm - far_cut - bearing
        t_max = spec.half_width_mm - bearing
        if hi <= lo or t_max <= 0.0:
            continue
        for _ in range(PAIR_PLACE_ATTEMPTS):
            distance = rng.uniform(*THREE_POINT_PAIR_DISTANCE_RANGE_MM)
            # 分離の向きそのものが多様性の軸: 0 = 全て掃引方向(007型)、
            # 1 = 全て帯幅方向(011型)。実測 007=w49% / 011=w100%。
            # 帯幅方向を**先に**、しかも実際に入る範囲から引く(share_wを一様に引いて
            # から棄却すると、入る範囲の狭い側に寄って中央値が実車の半分以下になる)。
            t = rng.uniform(0.0, min(t_max, distance)) * rng.choice((-1.0, 1.0))
            s = math.sqrt(max(0.0, distance * distance - t * t))
            s = s if abs(hi) > abs(lo) else -s      # パネルの伸びている側へ
            if not (lo <= s <= hi):
                continue
            position = tuple(frame.origin[i] + s * frame.u[i] + t * frame.v[i]
                             for i in range(3))
            return index, FasteningPoint(position_xyz=position,
                                         normal_xyz=anchor.normal_xyz)
    return None


def three_point_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """3締結点部品: 法線の一致する2点が同一パネル、3点目が曲げの向こう側。
    その曲げを側端フランジが跨いで補強する(実車007/011の骨格)。

    フランジの折れ角20度上限(`FLANGE_MAX_FOLD_ANGLE_DEG`、CATIA時代の申し送り)は
    **適用しない**。実車のフランジは90度の曲げに載っており、ユーザー裁定
    (2026-09-04「実車が正」)。OCCT版の実拘束は凹側クリアランス R - h >= 2mm だけで、
    それは `occt_build._check_flange_radii` が見ている。
    """
    aimed = dataclasses.replace(
        knobs,
        turn_range_deg=knobs.turn_range_deg or THREE_POINT_TURN_RANGE_DEG,
        half_width_ratio=knobs.half_width_ratio or THREE_POINT_HALF_WIDTH_RATIO,
        max_half_width_mm=knobs.max_half_width_mm or THREE_POINT_MAX_HALF_WIDTH_MM,
        bend_radius_mm=knobs.bend_radius_mm or THREE_POINT_BEND_RADIUS_RANGE_MM,
    )
    for _ in range(knobs.attempts):
        try:
            spec = _draw_spec(rng, aimed, folds=1)
            plan = plan_for(spec)
        except ValueError:
            continue
        flange = sample_flange(
            rng, plan.panel_frames, plan.fold_tilts,
            spec.half_width_mm, spec.bend_radius_mm, spec.point1.normal_xyz,
        )
        if flange is None:
            continue
        placed = _place_pair_mate(rng, spec, plan)
        if placed is None:
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
        return dataclasses.replace(spec, extra_points=(placed[1],)), None, flange, None
    return None


FAMILIES = {
    "bead": bead_part,
    "flange": flange_part,
    "rib": rib_part,
    "plain": plain_part,
    "three_point": three_point_part,
}


def kind_of(bead, flange, rib) -> str:
    return "bead" if bead else ("flange" if flange else ("rib" if rib else "plain"))


def classify_spec(spec) -> str:
    return str(classify(spec.point1, spec.point2))

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
from synthetic_generator.occt_build import (
    ARM_TIP_RELIEF_MM, branch_frames, channel_seat_frames, compose_arm_frame, drawn_notch_mm,
    drawn_tray_frames, step_frame, sweep_steps, tab_footprint_mm,
)
from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM
from synthetic_generator.flange import (
    FLANGE_MAX_FOLD_ANGLE_DEG,
    FlangeParams,
    max_fold_angle_deg,
    plan_flange_on_surface,
    sample_flange,
)
from synthetic_generator.general_geometry import (
    BEAD_MIN_BODY_MM,
    BEAD_MIN_RUNOUT_MM,
    bead_room_mm,
    check_bead_feasible_occt,
    path_spans,
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
    _flange_feasible,
    draw_fold_count,
    resolve_bead_slacks,
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
    # 単曲げの脚の上乗せ[mm]。Noneなら既定(0〜80)。
    leg_slack_mm: tuple[float, float] | None = None

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
            leg_slack_mm=(tuple(data["leg_slack_mm"])
                          if data.get("leg_slack_mm") else None),
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
        leg_slack_mm=knobs.leg_slack_mm,
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


# --- 3締結点・三角形分布(実車011型、2026-09-04) ----------------------------
# 007型(three_point)は3点がほぼ一直線に並ぶ — 実測で三角形らしさ 中央0.128・最大0.286、
# 0.30以上は0%だった。実車011は近正三角形(0.765)で、この型は作れていない。
#
# 近正三角形にするには対を**帯幅方向にだけ**離す必要がある(実車011は断面内の隔たり
# 0.2mm・帯幅方向36.8mm)。しかも現実的な帯幅に収めるには対が中心線を**またぐ**必要が
# ある: 片方を中心線に固定すると Δw <= half_width - bearing にしかならず、36.8mmには
# 帯幅98mmが要る。またげば Δw <= 2*(half_width - bearing) で、帯幅66mm(実車61mm)で足りる。
#
# そこで掃引のアンカー(point1 か point2)を対の**中点**として使い、実際の締結点2つを
# ±Δw/2 に置く。アンカー自身は締結点ではないので annotated_points から外す。
THREE_POINT_TRI_TURN_RANGE_DEG = (45.0, 90.0)
THREE_POINT_TRI_LEG_SLACK_MM = (0.0, 10.0)      # 脚が長いと対が届かず細長い三角になる
THREE_POINT_TRI_BEARING_MM = (12.5, 16.0)       # 帯幅を対に譲る(従来ルールの下端)
THREE_POINT_TRI_HALF_WIDTH_RATIO = (2.2, 2.8)
THREE_POINT_TRI_MAX_HALF_WIDTH_MM = 35.0
THREE_POINT_TRI_BEND_RADIUS_MM = (8.0, 14.0)    # Rが大きいと接線長で脚が伸びる
THREE_POINT_TRI_SPREAD_RATIO = (1.0, 1.3)       # Δw / 単独点までの距離。1.155で正三角形
# 対の置き方(011型・014型で共通)。対称に置くと二等辺三角形しか出ないので、
# 左右の振り分けと掃引方向のずらしを振る(ユーザー指示 2026-09-04)。
PAIR_SHARE_RANGE = (0.30, 0.70)                 # 中心線の左右への振り分け。0.5で対称
PAIR_RUN_REACH_RATIO = 0.55                     # 掃引方向のずらし幅 / 対の広がり
LONE_OFFSET_RATIO = 0.85                        # 単独点の帯幅方向のずらし / 使える片側
THREE_POINT_TRI_MIN_TRIANGULARITY = 0.60        # 実車011=0.765、007型=最大0.286
# 上の3つは総当たりで決めた(2026-09-04、200件ずつ)。閾値0.45/R8-18/slack0-18 では
# 辺の中央値が [38,56,57] と細長く、実車011の [37,38,41] から外れる。この組では
# [36,47,47]・三角形らしさ中央0.73 まで寄る。折れ角を60-90度に狭めても改善しないので
# 45-90度の多様性は残す。


def triangularity(positions) -> float:
    """3点の三角形らしさ = 最小高さ / 最長辺。0 = 一直線、0.866 = 正三角形。"""
    a = math.dist(positions[0], positions[1])
    b = math.dist(positions[1], positions[2])
    c = math.dist(positions[0], positions[2])
    half = (a + b + c) / 2.0
    area = math.sqrt(max(0.0, half * (half - a) * (half - b) * (half - c)))
    longest = max(a, b, c)
    return (2.0 * area / longest) / longest if longest > 1e-9 else 0.0


def _panel_room(spec, plan, index):
    """パネル index の上で締結点を置ける範囲 (掃引方向の下限, 上限, 帯幅方向の片側)。

    余白は従来ルールを踏襲 — 帯端・曲げの接線・パネル端のいずれからも
    `min_bearing_radius_mm` 以上を空ける。
    """
    bearing = spec.min_bearing_radius_mm
    frame = plan.panel_frames[index]
    near_cut, far_cut = plan.fold_tangents[index]
    return (frame.near_run_mm + near_cut + bearing,
            frame.far_run_mm - far_cut - bearing,
            spec.half_width_mm - bearing)


def _on_panel(frame, run: float, offset: float, normal) -> FasteningPoint:
    position = tuple(frame.origin[i] + run * frame.u[i] + offset * frame.v[i]
                     for i in range(3))
    return FasteningPoint(position_xyz=position, normal_xyz=normal)


def _straddle_pair(rng: random.Random, spec, plan, spread_ratio=None,
                   share_range=None, run_reach_ratio=None, host_index=None,
                   min_offset_mm=0.0, lone_room_mm=None, lone_run_limit_mm=None):
    """締結点3つをパネルの上に置く。戻り値は (対の2点, 単独の1点)。

    掃引アンカー(point1 / point2)は中心線上の**基準にすぎず、締結点ではない**。
    対を中心線の両側にまたがせることで、片側固定なら帯幅98mmを要する広がりが
    帯幅66mmで出せる(実車011)。

    対称に置く必要はない(ユーザー指示 2026-09-04)。左右の振り分け・掃引方向の
    ずらしに加え、**単独点も中心線から外す** — 単独点を中心線に固定すると
    長い2辺がほぼ等しくなり、二等辺三角形しか出ない(実測: 二等辺からのずれが
    中央0.04止まり。実車は007=0.27 / 011=0.07 / 014=0.02)。
    """
    hosts = [(0, spec.point1, spec.point2), (1, spec.point2, spec.point1)]
    rng.shuffle(hosts)
    if host_index is not None:      # 掃引の向きを固定したい族(014型)
        hosts = [h for h in hosts if h[0] == host_index]
    share_lo, share_hi = share_range or PAIR_SHARE_RANGE
    reach_ratio = PAIR_RUN_REACH_RATIO if run_reach_ratio is None else run_reach_ratio
    for index, anchor, lone in hosts:
        lo, hi, half_room = _panel_room(spec, plan, index)
        lone_lo, lone_hi, lone_room = _panel_room(spec, plan, 1 - index)
        if lone_room_mm is not None:
            # 孤立点の側が絞られている族(014型)。横ずれは**絞ったあとの**帯で決める。
            lone_room = lone_room_mm
        if hi < lo or lone_hi < lone_lo or half_room <= 0.0 or lone_room <= 0.0:
            continue
        frame = plan.panel_frames[index]
        span = math.dist(anchor.position_xyz, lone.position_xyz)
        share = rng.uniform(share_lo, share_hi)
        spread = min(rng.uniform(*(spread_ratio or THREE_POINT_TRI_SPREAD_RATIO)) * span,
                     half_room / max(share, 1.0 - share))
        if min_offset_mm > 0.0:      # ビードの足の外へ出す(014型)
            spread = max(spread, min_offset_mm / min(share, 1.0 - share))
            if spread * max(share, 1.0 - share) > half_room:
                continue
        reach = hi if abs(hi) > abs(lo) else lo
        limit = min(abs(reach), reach_ratio * spread)
        mates = tuple(
            _on_panel(frame,
                      math.copysign(rng.uniform(0.0, limit), reach) if abs(reach) > 1e-9 else 0.0,
                      offset, anchor.normal_xyz)
            for offset in (spread * share, -spread * (1.0 - share)))
        # 単独点も帯の中心から外す。長い2辺が一次で変わるので三角形が不等辺になる。
        lone_reach = lone_hi if abs(lone_hi) > abs(lone_lo) else lone_lo
        # 掃引方向の可動域。014型はビードが孤立点の手前で平地に戻るので、その先
        # (座面ぶん)より奥へは行かせない — 行くと座面が走り終いの壁に乗る。
        lone_limit = min(abs(lone_reach), reach_ratio * spread,
                         lone_run_limit_mm if lone_run_limit_mm is not None else math.inf)
        lone_point = _on_panel(
            plan.panel_frames[1 - index],
            math.copysign(rng.uniform(0.0, lone_limit), lone_reach)
            if abs(lone_reach) > 1e-9 else 0.0,
            rng.uniform(-1.0, 1.0) * lone_room * LONE_OFFSET_RATIO,
            lone.normal_xyz)
        return mates, lone_point
    return None


def three_point_tri_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """3締結点・三角形分布(実車011型)。法線が一致する対が曲げ線と平行に並び、
    3点目が曲げの向こう側。その曲げを側端フランジが跨ぐ。"""
    aimed = dataclasses.replace(
        knobs,
        turn_range_deg=knobs.turn_range_deg or THREE_POINT_TRI_TURN_RANGE_DEG,
        bearing_radius_mm=knobs.bearing_radius_mm or THREE_POINT_TRI_BEARING_MM,
        half_width_ratio=knobs.half_width_ratio or THREE_POINT_TRI_HALF_WIDTH_RATIO,
        max_half_width_mm=knobs.max_half_width_mm or THREE_POINT_TRI_MAX_HALF_WIDTH_MM,
        bend_radius_mm=knobs.bend_radius_mm or THREE_POINT_TRI_BEND_RADIUS_MM,
        leg_slack_mm=knobs.leg_slack_mm or THREE_POINT_TRI_LEG_SLACK_MM,
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
        placed = _straddle_pair(rng, spec, plan)
        if placed is None:
            continue
        mates, lone = placed
        annotated = (lone, *mates)
        if triangularity([p.position_xyz for p in annotated]) < THREE_POINT_TRI_MIN_TRIANGULARITY:
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
        return (dataclasses.replace(spec, extra_points=annotated, annotated_points=annotated),
                None, flange, None)
    return None


# --- 3締結点・単独点が遠い型(実車014、2026-09-04) --------------------------
# 007(0.254) と 011(0.765) の三角形らしさの間に 014(0.465) が入る。決め手は
# 「単独点までの距離 / 対の間隔」で、011=1.05 に対し 014=2.04 と2倍離れている。
# 離れた分だけ曲げをまたぐ荷重が増えるので、実車は**両側にフランジ + 帯の中央に
# ビード**を入れて1点と2点の間の剛性を確保している(断面実測: 対のパネルで
# ビード深さ3.5mm・フランジ4.6mm、単独点のパネルではビードが消えて平地に戻る)。
#
# 1部品1特徴の唯一の例外(ユーザー決定 2026-09-04)。フランジ高さも実車の4.6〜5.8mmに
# 寄せてこの族だけ6〜12mmにする(成立率はどちらでも100%で、忠実度だけの選択)。
THREE_POINT_SPAN_TURN_RANGE_DEG = (45.0, 90.0)
THREE_POINT_SPAN_LEG_SLACK_MM = (10.0, 40.0)     # 対の広がりが大きい分、単独点も遠い
# 脚を伸ばすと単独点までの距離だけが伸び、対の広がりは帯幅(2*(半幅-座面R))で頭打ちに
# なるので三角形が潰れる。150件ずつの実測: slack(25,70)で三角形らしさ中央0.23、
# slack(5,25)で0.34、slack(0,15)で0.37(実車014は0.465)。
# 実車014の対は**溶接**(座面クリアランス7.6mm)。ボルト前提の12.5mm下限を当てると、
# 「ビードの足 + 座面R」を対の外に確保するのに帯幅が実車の1.5倍(71〜80mm)要る。
# ユーザー裁定(2026-09-04): この族の対だけ溶接相当まで下げる。他族は12.5〜25mm据え置き。
THREE_POINT_SPAN_BEARING_MM = (7.5, 11.0)
THREE_POINT_SPAN_HALF_WIDTH_RATIO = (2.8, 3.8)
# 半幅 >= ビードの足 + 2*座面R が要る(対をビードの足の外へ出すため)。
# 座面7.5〜11・足8〜13 なら半幅25〜34mm、帯幅50〜68mmで実車の50mmに重なる。
THREE_POINT_SPAN_MAX_HALF_WIDTH_MM = 34.0
THREE_POINT_SPAN_BEND_RADIUS_MM = (12.0, 18.0)
THREE_POINT_SPAN_SPREAD_RATIO = (0.38, 0.62)     # Δw / 単独点までの距離。実車014は0.50
THREE_POINT_SPAN_FLANGE_HEIGHT_MM = (6.0, 12.0)  # 実車014は4.6〜5.8mm
# 実車014は開口7.8mm程度の「細長いビード」。既定の頂部幅下限10mmでは太すぎて、
# 対の座面がビードに乗ってしまう(実測: 足が片側11.9〜24.9mm)。
THREE_POINT_SPAN_BEAD_TOP_WIDTH_MM = (4.0, 12.0)
# 深さが壁の投影長 depth/tan(θ) を決め、それが足の幅を支配する。実車014は3.5〜4.4mm。
THREE_POINT_SPAN_BEAD_DEPTH_MM = (4.0, 5.5)
THREE_POINT_SPAN_TRIANGULARITY = (0.30, 0.59)    # 007型(<=0.286)と011型(>=0.60)の隙間
THREE_POINT_SPAN_NARROW_MARGIN_MM = 1.5   # 絞った端がビードの足を飲み込むための余白
BEAD_DRAW_ATTEMPTS = 12


def _bead_half_footprint_mm(bead) -> float:
    """ビードが幅方向に占める片側の量(足Rの後退量まで含む)。断面のブレーク点と同じ値。"""
    return bead.half_footprint_mm + bead.ridge_radius_mm * math.tan(
        math.radians(bead.wall_angle_deg) / 2.0)


def three_point_span_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """3締結点・単独点が対から遠い型(実車014)。両側フランジ + 中央ビード。"""
    aimed = dataclasses.replace(
        knobs,
        turn_range_deg=knobs.turn_range_deg or THREE_POINT_SPAN_TURN_RANGE_DEG,
        bearing_radius_mm=knobs.bearing_radius_mm or THREE_POINT_SPAN_BEARING_MM,
        half_width_ratio=knobs.half_width_ratio or THREE_POINT_SPAN_HALF_WIDTH_RATIO,
        max_half_width_mm=knobs.max_half_width_mm or THREE_POINT_SPAN_MAX_HALF_WIDTH_MM,
        bend_radius_mm=knobs.bend_radius_mm or THREE_POINT_SPAN_BEND_RADIUS_MM,
        leg_slack_mm=knobs.leg_slack_mm or THREE_POINT_SPAN_LEG_SLACK_MM,
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
            height_range_mm=THREE_POINT_SPAN_FLANGE_HEIGHT_MM, both_sides=True,
        )
        if flange is None:
            continue
        # ビードは載る置き方が引けるまで引き直す(ビード族と同じ事前判定)。
        bead = narrow = None
        for _ in range(BEAD_DRAW_ATTEMPTS):
            candidate = sample_bead(rng, spec.half_width_mm,
                                    THREE_POINT_SPAN_BEAD_TOP_WIDTH_MM,
                                    THREE_POINT_SPAN_BEAD_DEPTH_MM)
            # 対の座面(半径 min_bearing_radius_mm の平地)がビードに乗らないこと。
            # 対は中心線の両側なので、片側に「ビードの足 + 座面R」が要る。
            if _bead_half_footprint_mm(candidate) + 2.0 * spec.min_bearing_radius_mm                     > spec.half_width_mm:
                continue
            try:
                check_bead_feasible_occt(plan, candidate, spec.bend_radius_mm)
            except ValueError:
                continue
            bead = candidate
            break
        if bead is None:
            continue
        # 孤立点の側は必要平面ぶんまで絞る(実車014は50.1 -> 27.4mm)。
        narrow = max(spec.min_bearing_radius_mm,
                     _bead_half_footprint_mm(bead) + THREE_POINT_SPAN_NARROW_MARGIN_MM)
        if narrow >= spec.half_width_mm - 2.0:
            continue                      # 絞る余地が無い
        # 対を必ず panel 0 に載せる = 掃引が「対 -> 孤立点」の向きに揃う。
        placed = _straddle_pair(
            rng, spec, plan, THREE_POINT_SPAN_SPREAD_RATIO, host_index=0,
            min_offset_mm=_bead_half_footprint_mm(bead) + spec.min_bearing_radius_mm,
            lone_room_mm=max(0.0, narrow - spec.min_bearing_radius_mm),
            lone_run_limit_mm=spec.min_bearing_radius_mm)
        if placed is None:
            continue
        mates, lone = placed
        annotated = (lone, *mates)
        low, high = THREE_POINT_SPAN_TRIANGULARITY
        if not low <= triangularity([p.position_xyz for p in annotated]) <= high:
            continue
        return (dataclasses.replace(spec, extra_points=annotated, annotated_points=annotated,
                                    taper_half_width_mm=narrow),
                bead, flange, None)
    return None


# --- 平板 x 多点締結(実車031 / 1285-20、2026-09-05) -------------------------
# 実測: どちらも曲げゼロの平板で締結点は4つ、法線は全て同じ。外形は「締結点の配置に
# 沿った多角形 + 隅R」(031は隅R4.5〜8、20はR5)。面内の広がりは031が87x54mm、
# 20が63x62mm、点間距離は16〜88mm、板厚1.275〜1.76mm。
# 掃引モデルに収まる実車部品は19件中4件しかなく、うち2件がこの平板だった。
FLAT_PLATE_POINT_WEIGHTS = ((4, 0.60), (5, 0.25), (6, 0.15))
FLAT_PLATE_SPREAD_MM = (55.0, 95.0)        # 面内の広がり(実車 54〜87mm)
FLAT_PLATE_MIN_DISTANCE_MM = 20.0          # 点間の最小距離(実車の最小は16mm)
FLAT_PLATE_MARGIN_RATIO = (1.0, 1.8)       # 外形の余白 / 必要平面R(実車は18〜22mm)
FLAT_PLATE_MIN_TRIANGLE_MM2 = 400.0        # 一直線に近い配置を弾く
FLAT_PLATE_CORNER_RADIUS_MM = (5.0, 12.0)  # 外形の隅R。余白とは別物(実車は4.5〜8mm)


def _plane_basis(normal):
    seed = (0.0, 0.0, 1.0) if abs(normal[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = _unit(_cross3(normal, seed))
    return u, _cross3(normal, u)


def _cross3(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a):
    n = math.sqrt(sum(x * x for x in a))
    return tuple(x / n for x in a) if n > 1e-12 else a


def _largest_triangle_mm2(xy) -> float:
    """点集合が張る最大の三角形の面積。一直線に近い配置ほど 0 に近づく。"""
    best = 0.0
    for i in range(len(xy)):
        for j in range(i + 1, len(xy)):
            for k in range(j + 1, len(xy)):
                a, b, c = xy[i], xy[j], xy[k]
                best = max(best, abs((b[0] - a[0]) * (c[1] - a[1])
                                     - (b[1] - a[1]) * (c[0] - a[0])) / 2.0)
    return best


def flat_plate_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """平板 x 多点締結。曲げゼロ・法線が全点で一致し、外形は締結点の凸包の外側
    オフセット(隅Rは余白の半径そのもの)。特徴(ビード/フランジ/リブ)は持たない。"""
    for _ in range(knobs.attempts):
        try:
            spec = _draw_spec(rng, knobs, folds=0)
        except ValueError:
            continue
        normal = spec.point1.normal_xyz
        u, v = _plane_basis(normal)
        count = _draw_weighted(rng, knobs.fold_weights or FLAT_PLATE_POINT_WEIGHTS)
        width = rng.uniform(*FLAT_PLATE_SPREAD_MM)
        height = rng.uniform(*FLAT_PLATE_SPREAD_MM)
        xy = [(rng.uniform(0.0, width), rng.uniform(0.0, height)) for _ in range(count)]
        if min(math.dist(a, b) for a in xy for b in xy if a is not b)                 < FLAT_PLATE_MIN_DISTANCE_MM:
            continue
        if _largest_triangle_mm2(xy) < FLAT_PLATE_MIN_TRIANGLE_MM2:
            continue
        base = spec.point1.position_xyz
        points = tuple(
            FasteningPoint(
                position_xyz=tuple(base[i] + x * u[i] + y * v[i] for i in range(3)),
                normal_xyz=normal)
            for x, y in xy)
        margin = spec.min_bearing_radius_mm * rng.uniform(*FLAT_PLATE_MARGIN_RATIO)
        return (dataclasses.replace(
            spec, point1=points[0], point2=points[1], extra_points=(),
            annotated_points=points, plate_margin_mm=margin, target_folds=0,
            plate_corner_radius_mm=rng.uniform(*FLAT_PLATE_CORNER_RADIUS_MM)),
            None, None, None)
    return None


# --- 分岐(実車026、2026-09-05) ------------------------------------------------
# 「締結したい面が3つ以上あるとき、それらを1枚の板でどう繋ぐか」を学習させる族。
# 平面のハブから辺ごとに別の軸で腕を折り出す。実車026: ハブ1453mm2、腕3本
# (折れ角 91.0 / 90.0 / 30.3度、根本幅 32〜73mm、腕長 23〜47mm、曲げR 6.8〜24.2)。
#
# 一枚板から作れるための条件(docs/PLAN_branch_2026-09-05.md):
#   展開可能 = 構築上保証 / 展開図の自己交差なし = 凸ハブで構築上保証 /
#   ハブ角のコーナー = 頂点中心の円弧で腕の側端と滑らかに繋ぐ / 腕どうしの干渉 = 実測。
#
# ガセット: 隣り合う2本の腕を90度に揃え、片方から折り出したタブをもう片方に重ねる。
# 両側を曲げで留めると角の3枚の扇形角が 90+90+ハブ角 < 360 で展開できない(実車026は
# ここを絞りで埋めている)。重ねならタブの端が自由エッジになり外形線は1本のまま。
BRANCH_ARM_COUNT_WEIGHTS = ((3, 0.60), (4, 0.40))
BRANCH_HUB_WIDTH_MM = (60.0, 110.0)
BRANCH_HUB_HEIGHT_MM = (50.0, 95.0)
BRANCH_HUB_SKEW = 0.20                    # 四角形の歪み(上辺のずれ / 幅)
BRANCH_FOLD_DEG = (25.0, 110.0)           # 腕ごとに独立(実車 30.3 / 90.0 / 91.0)
BRANCH_BEND_R_MM = (5.0, 25.0)            # 実車 6.8 / 7.2 / 24.2
BRANCH_ARM_LENGTH_MM = (30.0, 70.0)       # 実車 23 / 31 / 47
BRANCH_CORNER_R_MM = (6.0, 12.0)          # ハブ角のくぼみR
# ガセットは無効(2026-09-05 ユーザー裁定)。タブ重ねでも角に穴が残って見えるため、
# 生成には入れない。機構は残してあるので、埋め方が決まれば比率を戻せる。
BRANCH_GUSSET_PROB = 0.0
BRANCH_GUSSET_FOLD_DEG = 90.0             # ガセットを挟む2本の腕はどちらも90度
BRANCH_ARM_POINT_WEIGHTS = ((1, 0.60), (2, 0.40))   # 腕先の締結点数
BRANCH_HUB_POINT_PROB = 0.50              # ハブに1点置く確率
BRANCH_POINT_ATTEMPTS = 20


def _dist_to_segment(p, a, b) -> float:
    ab = (b[0] - a[0], b[1] - a[1])
    ap = (p[0] - a[0], p[1] - a[1])
    denom = ab[0] * ab[0] + ab[1] * ab[1]
    t = 0.0 if denom < 1e-12 else max(0.0, min(1.0, (ap[0] * ab[0] + ap[1] * ab[1]) / denom))
    return math.hypot(ap[0] - t * ab[0], ap[1] - t * ab[1])


def branch_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """分岐部品: 凸四角形のハブに腕3〜4本、30%でガセット、腕先1〜2点＋ハブ0〜1点。"""
    for _ in range(knobs.attempts):
        try:
            spec = _draw_spec(rng, knobs, folds=0)      # 板厚・座面R・向きだけ借りる
        except ValueError:
            continue
        bearing = spec.min_bearing_radius_mm
        normal = spec.point1.normal_xyz
        u, v = _plane_basis(normal)
        origin = spec.point1.position_xyz

        w = rng.uniform(*BRANCH_HUB_WIDTH_MM)
        h = rng.uniform(*BRANCH_HUB_HEIGHT_MM)
        skew = rng.uniform(-BRANCH_HUB_SKEW, BRANCH_HUB_SKEW) * w
        hub_xy = [(0.0, 0.0), (w, 0.0), (w + skew, h), (skew * 0.5, h)]
        edges = sorted(rng.sample(range(4), _draw_weighted(rng, BRANCH_ARM_COUNT_WEIGHTS)))
        arms = [{"edge": e,
                 "fold_deg": rng.uniform(*BRANCH_FOLD_DEG),
                 "radius_mm": rng.uniform(*BRANCH_BEND_R_MM),
                 "length_mm": rng.uniform(*BRANCH_ARM_LENGTH_MM),
                 "relief_mm": ARM_TIP_RELIEF_MM} for e in edges]
        corner_radius = rng.uniform(*BRANCH_CORNER_R_MM)

        # ガセット: 隣り合う腕2本の角を1つ選び、両腕を90度に揃える(意図的に用意する)。
        gussets: list = []
        if rng.random() < BRANCH_GUSSET_PROB:
            corners = [vtx for vtx in range(4) if (vtx - 1) % 4 in edges and vtx in edges]
            if corners:
                vtx = rng.choice(corners)
                for arm in arms:
                    if arm["edge"] in ((vtx - 1) % 4, vtx):
                        arm["fold_deg"] = BRANCH_GUSSET_FOLD_DEG
                gussets = [vtx]

        try:
            layout = branch_frames(hub_xy, arms, origin=origin, hub_u=u, hub_v=v,
                                   corner_radius=corner_radius)
        except ValueError:
            continue

        # 締結点: 腕先に1〜2点(座面ぶんの余白を四方に)、ハブに0〜1点。
        points: list = []
        feasible = True
        for arm in arms:
            fr = layout["arms"][arm["edge"]]
            width = fr["width"]
            length = arm["length_mm"] - ARM_TIP_RELIEF_MM
            if width < 2.0 * bearing + 1.0 or length < 2.0 * bearing + 1.0:
                feasible = False
                break
            wanted = _draw_weighted(rng, BRANCH_ARM_POINT_WEIGHTS)
            got: list = []
            for _ in range(BRANCH_POINT_ATTEMPTS):
                run = rng.uniform(bearing, length - bearing)
                across = rng.uniform(bearing, width - bearing)
                if all(math.hypot(run - r2, across - a2) >= FLAT_PLATE_MIN_DISTANCE_MM
                       for r2, a2 in got):
                    got.append((run, across))
                if len(got) == wanted:
                    break
            if not got:
                feasible = False
                break
            for run, across in got:
                position = tuple(fr["a"][i] + run * fr["tip"][i] + across * fr["axis"][i]
                                 for i in range(3))
                points.append(FasteningPoint(position_xyz=position, normal_xyz=fr["normal"]))
        if not feasible:
            continue
        if rng.random() < BRANCH_HUB_POINT_PROB:
            cx = sum(p[0] for p in hub_xy) / 4.0
            cy = sum(p[1] for p in hub_xy) / 4.0
            if all(_dist_to_segment((cx, cy), hub_xy[i], hub_xy[(i + 1) % 4]) >= bearing
                   for i in range(4)):
                points.append(FasteningPoint(position_xyz=layout["to_space"]((cx, cy)),
                                             normal_xyz=layout["normal"]))
        if len(points) < 3:
            continue
        branch = {"hub_xy": hub_xy, "arms": arms, "origin": list(origin),
                  "hub_u": list(u), "hub_v": list(v),
                  "corner_radius": corner_radius, "gussets": gussets}
        return (dataclasses.replace(spec, point1=points[0], point2=points[1], extra_points=(),
                                    annotated_points=tuple(points), branch=branch,
                                    target_folds=None),
                None, None, None)
    return None


# ---------------------------------------------------------------- 実車144型(チャンネル + 座面)
# ウェブ(台の平面)の両長辺に壁、壁の下辺は斜め、そこから座面を外へ折る。BIWと外装の接点。
# 実車: ウェブ 66.8x31.5、壁 88.0度 R3.4、斜辺 43.5度、座面 44.8x31 (86.1度 R2.6)、t=0.6。
CHANNEL_THICKNESS_MM = 0.6
CHANNEL_MIN_R_MM = 4.0                     # この族の最小R(指示: 4)
CHANNEL_BEARING_MM = (12.5, 15.0)
CHANNEL_WEB_LENGTH_MM = (55.0, 90.0)       # 実車 66.8
CHANNEL_WEB_WIDTH_MM = (28.0, 45.0)        # 実車 31.5
CHANNEL_WALL_FOLD_DEG = (60.0, 88.0)       # 上限88: 壁は常に外へ開く(座面側が長辺の台形)
CHANNEL_WALL_R_MM = (4.0, 8.0)
CHANNEL_WALL_DEPTH0_MM = (25.0, 50.0)      # 斜辺の浅い側の深さ(実車 45)
CHANNEL_DIAG_DEG = (20.0, 60.0)            # 斜辺の角度(実車 43.5)
CHANNEL_DIAG_OVERHANG_MM = 8.0             # 斜辺の浅い端がウェブ端より外に出てよい量(実車 9.5)
CHANNEL_SEAT_WIDTH_MM = (45.0, 70.0)       # 斜辺の長さ = 座面の幅(実車 44.8)
CHANNEL_SEAT_DEPTH_MM = (28.0, 45.0)       # 実車 31
CHANNEL_SEAT_R_MM = (4.0, 6.0)
CHANNEL_SEAT_FOLD_RANGE_DEG = (55.0, 100.0)  # 解いた γ がここを外れたら引き直す
CHANNEL_SEAT_POINT_GAP_MM = 20.0           # 座面2点の間隔(実車 31)


def channel_seat_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """実車144型: 座面(同一法線)各1〜2点 + ウェブ1点(必須)。"""
    for _ in range(knobs.attempts):
        try:
            spec = _draw_spec(rng, knobs, folds=0)      # 向きだけ借りる
        except ValueError:
            continue
        bearing = rng.uniform(*(knobs.bearing_radius_mm or CHANNEL_BEARING_MM))
        normal = spec.point1.normal_xyz
        u, v = _plane_basis(normal)
        origin = spec.point1.position_xyz

        length = rng.uniform(*CHANNEL_WEB_LENGTH_MM)
        width = rng.uniform(*CHANNEL_WEB_WIDTH_MM)
        beta = math.radians(rng.uniform(*CHANNEL_DIAG_DEG))
        seat_w = rng.uniform(*CHANNEL_SEAT_WIDTH_MM)
        run = seat_w * math.cos(beta)
        if run > length + CHANNEL_DIAG_OVERHANG_MM - 2.0:
            continue
        diag_start = rng.uniform(-CHANNEL_DIAG_OVERHANG_MM, length - run - 2.0)
        depth0 = rng.uniform(*CHANNEL_WALL_DEPTH0_MM)
        geom = {
            "origin": list(origin), "hub_u": list(u), "hub_v": list(v),
            "length_mm": length, "width_mm": width,
            "wall_fold_deg": rng.uniform(*CHANNEL_WALL_FOLD_DEG),
            "wall_radius_mm": rng.uniform(*CHANNEL_WALL_R_MM),
            "wall_depth0_mm": depth0, "wall_depth1_mm": depth0 + seat_w * math.sin(beta),
            "diag_start_mm": diag_start, "diag_end_mm": diag_start + run,
            "seat_fold_deg": None,
            "seat_radius_mm": rng.uniform(*CHANNEL_SEAT_R_MM),
        }
        lay = channel_seat_frames(**geom)
        wall = lay["walls"]["A"]
        gamma = wall["seat_fold_deg"]
        if not CHANNEL_SEAT_FOLD_RANGE_DEG[0] <= gamma <= CHANNEL_SEAT_FOLD_RANGE_DEG[1]:
            continue
        geom["seat_fold_deg"] = gamma
        seat_d = rng.uniform(*CHANNEL_SEAT_DEPTH_MM)
        if seat_d < 2.0 * bearing + 1.0 or width < 2.0 * bearing + 1.0:
            continue

        # 座面: 幅が許せば2点(実車)、無理なら1点。両座面で同じ局所座標 = 鏡像配置。
        two = seat_w >= 2.0 * bearing + CHANNEL_SEAT_POINT_GAP_MM + 1.0
        across = rng.uniform(bearing, seat_d - bearing)
        if two:
            gap = rng.uniform(CHANNEL_SEAT_POINT_GAP_MM, seat_w - 2.0 * bearing)
            centre = rng.uniform(bearing + gap / 2.0, seat_w - bearing - gap / 2.0)
            along = [centre - gap / 2.0, centre + gap / 2.0]
        else:
            along = [rng.uniform(bearing, seat_w - bearing)]
        points: list = []
        for key in ("A", "B"):
            w = lay["walls"][key]
            for s_ in along:
                position = tuple(w["seat_a"][i] + s_ * w["diag"][i] + across * w["seat_out"][i]
                                 for i in range(3))
                points.append(FasteningPoint(position_xyz=position, normal_xyz=w["seat_normal"]))
        # ウェブ: 1点必須
        wx = rng.uniform(bearing, length - bearing)
        wy = rng.uniform(bearing, width - bearing)
        points.append(FasteningPoint(position_xyz=lay["at"](wx, wy), normal_xyz=lay["normal"]))

        channel = dict(geom, seat_depth_mm=seat_d, seat_corner_mm=CHANNEL_MIN_R_MM,
                       diag_deg=math.degrees(beta), seat_width_mm=seat_w)
        return (dataclasses.replace(spec, point1=points[0], point2=points[1], extra_points=(),
                                    annotated_points=tuple(points), channel=channel,
                                    target_folds=None, thickness_mm=CHANNEL_THICKNESS_MM,
                                    min_bearing_radius_mm=bearing),
                None, None, None)
    return None


# ---------------------------------------------------------------- 実車1285-18(タブ付きブラケット)
# 台の平面の長辺を上へ折った低い立ち上がり(溶接タブ2つ)と、台の角を斜めに切った折り線から
# 反対側(下)へ折ったフランジ(ボルト1点)。締結面の法線が3方向(台 / 立ち上がり / フランジ)。
# 実車: 台 58x27、立ち上がり 88度 R3 高さ9 タブR9 x2、斜辺33度、フランジ 85度 R4.1 高さ22、
# ボルト穴6.9、台にクリップ穴2(18.2 / 8.0)、t=1.5。ユーザー裁定(2026-09-06): 必要半径と
# 最小Rは実車相当、台の点は1〜2点、フランジは矩形/台形。
TAB_MIN_R_MM = 3.0                          # 実車 R3(2t) / R4.1
TAB_BEARING_MM = (7.5, 11.0)                # 溶接相当。タブ半径 = 溶接の必要半径(実車 9)
TAB_PLATE_LENGTH_MM = (50.0, 75.0)          # 実車 58
TAB_PLATE_WIDTH_MM = (22.0, 35.0)           # 実車 27
TAB_PLATE_CORNER_R_MM = (5.0, 10.0)         # 実車 8
TAB_RELIEF_GAP_MM = (2.0, 5.0)              # 立ち上がりとフランジの折り線の間の逃げ(実車 2)
TAB_HINGE_DEG = (20.0, 50.0)                # フランジ折り線と立ち上がりの角度(実車 33)
TAB_UPSTAND_FOLD_DEG = (80.0, 90.0)         # 実車 88
TAB_UPSTAND_R_MM = (3.0, 5.0)               # 実車 3
TAB_UPSTAND_EXTRA_HEIGHT_MM = (0.0, 3.0)    # 高さ = 必要半径 + これ(実車 9 = 必要半径)
TAB_WELD_COUNT = 2                          # 実車 2
TAB_FLANGE_FOLD_DEG = (80.0, 90.0)          # 実車 85
TAB_FLANGE_R_MM = (3.0, 6.0)                # 実車 4.1
TAB_FLANGE_HEIGHT_MM = (18.0, 30.0)         # 実車 22
TAB_FLANGE_TRAPEZOID_PROB = 0.5             # 残りは矩形
TAB_FLANGE_SHRINK_RATIO = 0.25              # 台形の片側の縮み / 折り線長の上限
TAB_PLATE_POINT_WEIGHTS = ((1, 0.5), (2, 0.5))
TAB_POINT_ATTEMPTS = 80
TAB_PLATE_MIN_DISTANCE_MM = 16.0          # 台の2点の最小間隔(実車 26)。必要半径2つ分が優先


def _inside_convex(p, xy) -> bool:
    n = len(xy)
    for i in range(n):
        a, b = xy[i], xy[(i + 1) % n]
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < 0.0:
            return False
    return True


def tab_bracket_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """実車1285-18: 台 + 溶接タブ付き立ち上がり(上) + 斜め折り線のフランジ(下)。"""
    for _ in range(knobs.attempts):
        try:
            spec = _draw_spec(rng, knobs, folds=0)      # 板厚・向きだけ借りる
        except ValueError:
            continue
        bearing = rng.uniform(*(knobs.bearing_radius_mm or TAB_BEARING_MM))
        normal = spec.point1.normal_xyz
        u, v = _plane_basis(normal)
        origin = spec.point1.position_xyz

        length = rng.uniform(*TAB_PLATE_LENGTH_MM)
        width = rng.uniform(max(TAB_PLATE_WIDTH_MM[0], 2.0 * bearing + 2.0), TAB_PLATE_WIDTH_MM[1])
        gap = rng.uniform(*TAB_RELIEF_GAP_MM)
        beta = math.radians(rng.uniform(*TAB_HINGE_DEG))
        corner_r = rng.uniform(*TAB_PLATE_CORNER_R_MM)
        dx = (width - gap) / math.tan(beta)
        if dx > length - corner_r - 6.0:
            continue                          # 上辺が残らない
        hinge = math.hypot(dx, width - gap)
        if hinge < 2.0 * bearing + 2.0:
            continue
        # 頂点(反時計回り): 0 台の左下, 1 右下, 2 逃げの上, 3 斜辺の上端, 4 左上(フィレット)
        hub_xy = [(0.0, 0.0), (length, 0.0), (length, gap), (length - dx, width), (0.0, width)]

        # 立ち上がり: 辺0 を表側(+法線)へ。高さ = 必要半径 + α、タブは半径 = 必要半径。
        up_h = bearing + rng.uniform(*TAB_UPSTAND_EXTRA_HEIGHT_MM)
        foot = tab_footprint_mm(bearing, TAB_MIN_R_MM)      # タブ + 根元R が上端で占める片側幅
        span = length - 2.0 * (foot + 0.6)
        if span < (TAB_WELD_COUNT - 1) * (2.0 * foot + 2.0):
            continue
        s_first = rng.uniform(foot + 0.6, length - foot - 0.6 - (TAB_WELD_COUNT - 1) * (2.0 * foot + 2.0))
        pitch = rng.uniform(2.0 * foot + 2.0, (length - foot - 0.6 - s_first) / max(1, TAB_WELD_COUNT - 1))
        tabs = [[s_first + k * pitch, bearing] for k in range(TAB_WELD_COUNT)]
        upstand = {"edge": 0, "fold_deg": rng.uniform(*TAB_UPSTAND_FOLD_DEG),
                   "radius_mm": rng.uniform(*TAB_UPSTAND_R_MM), "side": 1, "length_mm": up_h,
                   "relief_mm": TAB_MIN_R_MM,
                   "outline": {"kind": "tabs", "height_mm": up_h, "tabs": tabs,
                               "root_r_mm": TAB_MIN_R_MM}}
        # フランジ: 辺2(斜辺)を裏側(-法線)へ。矩形 or 台形。
        fl_h = rng.uniform(max(TAB_FLANGE_HEIGHT_MM[0], 2.0 * bearing + 2.0), TAB_FLANGE_HEIGHT_MM[1])
        if rng.random() < TAB_FLANGE_TRAPEZOID_PROB:
            cap = min(TAB_FLANGE_SHRINK_RATIO * hinge, (hinge - 2.0 * bearing - 2.0) / 2.0)
            shrink = (rng.uniform(0.0, max(0.0, cap)), rng.uniform(0.0, max(0.0, cap)))
        else:
            shrink = (0.0, 0.0)
        flange = {"edge": 2, "fold_deg": rng.uniform(*TAB_FLANGE_FOLD_DEG),
                  "radius_mm": rng.uniform(*TAB_FLANGE_R_MM), "side": -1, "length_mm": fl_h,
                  "relief_mm": TAB_MIN_R_MM,
                  "outline": {"kind": "trapezoid", "shrink_a_mm": shrink[0], "shrink_b_mm": shrink[1]}}
        arms = [upstand, flange]
        fillet = {4: corner_r}
        try:
            layout = branch_frames(hub_xy, arms, origin=origin, hub_u=u, hub_v=v,
                                   corner_radius=TAB_MIN_R_MM, fillet_radius=fillet)
        except ValueError:
            continue

        points: list = []
        fr = layout["arms"][0]
        for s_c, _r in tabs:                  # 溶接 = タブの円の中心
            position = tuple(fr["a"][i] + up_h * fr["tip"][i] + s_c * fr["axis"][i] for i in range(3))
            points.append(FasteningPoint(position_xyz=position, normal_xyz=fr["normal"]))
        fr = layout["arms"][2]
        t = rng.uniform(bearing, fl_h - bearing)
        # 台形の高さ t での有効幅の中央に置く
        lo = shrink[0] * t / fl_h
        hi = fr["width"] - shrink[1] * t / fl_h
        if hi - lo < 2.0 * bearing:
            continue
        s_b = rng.uniform(lo + bearing, hi - bearing)
        position = tuple(fr["a"][i] + t * fr["tip"][i] + s_b * fr["axis"][i] for i in range(3))
        points.append(FasteningPoint(position_xyz=position, normal_xyz=fr["normal"]))
        # 台の点(1〜2)
        wanted = _draw_weighted(rng, TAB_PLATE_POINT_WEIGHTS)
        got: list = []
        for _ in range(TAB_POINT_ATTEMPTS):
            cand = (rng.uniform(bearing, length - bearing), rng.uniform(bearing, width - bearing))
            if not _inside_convex(cand, hub_xy):
                continue
            if any(_dist_to_segment(cand, hub_xy[i], hub_xy[(i + 1) % 5]) < bearing for i in range(5)):
                continue
            if all(math.dist(cand, o) >= max(TAB_PLATE_MIN_DISTANCE_MM, 2.0 * bearing) for o in got):
                got.append(cand)
            if len(got) == wanted:
                break
        if not got:
            continue
        for cand in got:
            points.append(FasteningPoint(position_xyz=layout["to_space"](cand),
                                         normal_xyz=layout["normal"]))
        branch = {"hub_xy": hub_xy, "arms": arms, "origin": list(origin),
                  "hub_u": list(u), "hub_v": list(v), "corner_radius": TAB_MIN_R_MM,
                  "gussets": [], "fillet_radius": {4: corner_r},
                  "hinge_deg": math.degrees(beta), "hinge_mm": hinge, "gap_mm": gap}
        return (dataclasses.replace(spec, point1=points[0], point2=points[1], extra_points=(),
                                    annotated_points=tuple(points), branch=branch,
                                    target_folds=None, min_bearing_radius_mm=bearing),
                None, None, None)
    return None


# ---------------------------------------------------------------- 実車002-057(絞りの角 + 曲げタブ)
# ユーザーの展開図(2026-09-06): 赤=外形、緑=絞り線(ハブの角から Y 字: ハブ–壁A、ハブ–壁B、
# 壁どうしの継ぎ目)、青=曲げ線(タブ2つ)。まず緑を絞ると、ハブに対して斜めの継ぎ目で
# つながった壁2枚ができる。次にハブが器の底になる向きにタブ2つを曲げる。
# 3D では「つないでからフィレット」: ハブ・壁A・壁B を鋭いエッジで縫合し、3本の共有エッジに
# フィレット(頂点はブレンド = 絞り)。壁の角度は独立、締結は絞り壁にタブ溶接・曲げタブに
# 溶接1〜2・ハブに穴2、壁の上端は実車相当(高い方の壁が継ぎ目へ向けて細る)。裁定 2026-09-06。
DRAWN_BEARING_MM = (7.5, 11.0)             # 溶接相当(タブ半径 = 必要半径)
DRAWN_FRONT_MM = (60.0, 95.0)              # ハブの前辺(壁B の根本、実車 74)
DRAWN_DEPTH_MM = (45.0, 70.0)              # ハブの奥行(壁A の根本、実車 52)
DRAWN_RIGHT_SKEW_DEG = (0.0, 30.0)         # 壁A の根本の傾き(実車 24)
DRAWN_WALL_FOLD_DEG = (30.0, 90.0)         # 壁A/B 独立(実車 85 / 32)
DRAWN_WALL_HEIGHT_MM = (25.0, 45.0)        # 実車 43 / 37
DRAWN_WALL_FILLET_STEEP_MM = (6.0, 15.0)   # 折れ角 > 60 度の壁の根本R(実車 8.7)
DRAWN_WALL_FILLET_SHALLOW_MM = (6.0, 40.0) # 折れ角 <= 60 度の壁の根本R(実車 38.7)
DRAWN_SEAM_FILLET_MM = (5.0, 10.0)         # 継ぎ目のR(実車 6〜9)
DRAWN_TAPER_SHARE = (0.25, 0.5)            # 細りを始める位置(継ぎ目から壁幅の何割か)
DRAWN_TAB_COUNT_WEIGHTS = ((2, 0.6), (3, 0.4))
DRAWN_TAB_JITTER = 0.30
DRAWN_ARM_FOLD_DEG = (65.0, 90.0)          # 曲げタブ(実車の後壁 72、左壁 90)
DRAWN_ARM_R_MM = (6.0, 15.0)
DRAWN_ARM_LENGTH_MM = (25.0, 45.0)
DRAWN_ARM_GAP_MM = (3.0, 8.0)              # タブの根本と絞り壁・角との逃げ
DRAWN_ARM_POINT_WEIGHTS = ((1, 0.5), (2, 0.5))
DRAWN_HUB_POINTS = 2
DRAWN_POINT_ATTEMPTS = 80


def _tabs_on(rng, b, lo, hi, root_r=MIN_NEUTRAL_PLANE_RADIUS_MM):
    """[lo, hi] の区間に半径 b のタブ(根元R root_r つき)を 2〜3(入らなければ減らす)。
    位置は余白を弱い乱数で配る。占有幅はタブ + 根元R の片側幅 f = sqrt(b^2 + 2 b root_r)。"""
    f = tab_footprint_mm(b, root_r)
    usable = hi - lo
    count = _draw_weighted(rng, DRAWN_TAB_COUNT_WEIGHTS)
    while count > 1 and usable < count * 2.0 * f + (count - 1) * 1.0 + 1.2:
        count -= 1
    slack = usable - (count * 2.0 * f + (count - 1) * 1.0 + 1.2)
    if slack < 0.0:
        return None
    weights = [1.0 + rng.uniform(-DRAWN_TAB_JITTER, DRAWN_TAB_JITTER) for _ in range(count + 1)]
    total = sum(weights)
    tabs, cursor = [], lo + 0.6
    for k in range(count):
        cursor += slack * weights[k] / total
        tabs.append([cursor + f, b])
        cursor += 2.0 * f + 1.0
    return tabs


def drawn_tray_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """実車002-057 の簡略版(展開図第2版): ハブ + 絞り壁3枚(前B・右A・左C、継ぎ目2本) +
    曲げタブ1枚(後)。溶接 = 壁のタブ中心(各2〜3) + 曲げタブ 1〜2、ハブに穴2。"""
    for _ in range(knobs.attempts):
        try:
            spec = _draw_spec(rng, knobs, folds=0)
        except ValueError:
            continue
        b = rng.uniform(*(knobs.bearing_radius_mm or DRAWN_BEARING_MM))
        normal = spec.point1.normal_xyz
        u, v = _plane_basis(normal)
        origin = spec.point1.position_xyz

        depth = rng.uniform(max(DRAWN_DEPTH_MM[0], 5.0 * b + 14.0), DRAWN_DEPTH_MM[1])
        front = rng.uniform(max(DRAWN_FRONT_MM[0], 7.0 * b + 14.0), DRAWN_FRONT_MM[1])
        skew = depth * math.tan(math.radians(rng.uniform(*DRAWN_RIGHT_SKEW_DEG)))
        if front - skew < 4.0 * b + 10.0:
            continue
        hub_xy = [(0.0, 0.0), (front, 0.0), (front - skew, depth), (0.0, depth)]

        # 3本のフィレットは同じ半径にする: 半径が違うと頂点のブレンドの縁が自由曲線になり、
        # 「エッジは直線と円弧だけ」の出力契約(ゲートA)を破る(2026-09-06 実測)。
        fillet = rng.uniform(*DRAWN_WALL_FILLET_STEEP_MM)
        walls = {}
        for key in ("A", "B", "C"):
            fold = rng.uniform(*DRAWN_WALL_FOLD_DEG)
            tangent = fillet * math.tan(math.radians(fold) / 2.0)
            height = rng.uniform(max(DRAWN_WALL_HEIGHT_MM[0], tangent + b + 4.0), DRAWN_WALL_HEIGHT_MM[1])
            walls[key] = {"fold_deg": fold, "height_mm": height, "fillet_mm": fillet,
                          "tangent_mm": tangent, "tabs": [], "taper_from_mm": {},
                          "corner_square_mm": tangent + 2.0,
                          "tab_root_r_mm": MIN_NEUTRAL_PLANE_RADIUS_MM}
        try:
            lay = drawn_tray_frames(hub_xy, walls, origin=origin, hub_u=u, hub_v=v)
        except ValueError:
            continue
        # 細りの始点(継ぎ目側)と、タブを置ける区間 = 根本の上で、細りとノッチを避けた部分
        ok = True
        for key, fr in lay["walls"].items():
            w = fr["width"]
            lo, hi = 0.0, w
            for side in ("left", "right"):
                end = fr["ends"][side]
                if end is None:
                    continue
                notch = drawn_notch_mm(fillet, end["turn"])
                perp_s = end["dt"] if side == "left" else -end["dt"]
                q_s = end["s"] + notch * perp_s
                share = rng.uniform(*DRAWN_TAPER_SHARE)
                if end["tapers"]:
                    s_taper = end["s"] + (share * w if side == "left" else -share * w)
                    # ノッチの先より内側に始点を置く
                    s_taper = max(s_taper, q_s + 0.5) if side == "left" else min(s_taper, q_s - 0.5)
                    walls[key]["taper_from_mm"][side] = s_taper
                else:
                    s_taper = q_s
                if side == "left":
                    lo = max(lo, s_taper)
                else:
                    hi = min(hi, s_taper)
            tabs = _tabs_on(rng, b, lo, hi)
            if tabs is None or len(tabs) < 2:        # 裁定: 各壁のタブは 2〜3
                ok = False
                break
            walls[key]["tabs"] = tabs
        if not ok:
            continue

        # 曲げタブ: 辺2(後、v2->v3)。根本は壁A/C のフィレット帯と v2 の直角短辺を避ける。
        edge_len = math.dist(hub_xy[2], hub_xy[3])
        root_from = walls["A"]["tangent_mm"] + walls["A"]["corner_square_mm"] * math.sin(math.atan2(skew, depth)) + 3.0
        root_to = edge_len - walls["C"]["tangent_mm"] - 3.0
        width = root_to - root_from
        if width < 2.0 * b + 2.0:
            continue
        length = rng.uniform(max(DRAWN_ARM_LENGTH_MM[0], 2.0 * b + 2.0), DRAWN_ARM_LENGTH_MM[1])
        cap = max(0.0, (width - 2.0 * b - 2.0) / 2.0)
        outline = ({"kind": "trapezoid", "shrink_a_mm": rng.uniform(0.0, min(cap, 0.25 * width)),
                    "shrink_b_mm": rng.uniform(0.0, min(cap, 0.25 * width))}
                   if rng.random() < 0.5 else {"kind": "rect"})
        arms = [{"edge": 2, "fold_deg": rng.uniform(*DRAWN_ARM_FOLD_DEG),
                 "radius_mm": rng.uniform(*DRAWN_ARM_R_MM), "length_mm": length,
                 "relief_mm": ARM_TIP_RELIEF_MM, "root_from_mm": root_from,
                 "root_to_mm": root_to, "outline": outline}]

        # ---- 締結点
        points: list = []
        for key, fr in lay["walls"].items():          # 絞り壁の溶接 = タブの円の中心
            for s_c, _r in walls[key]["tabs"]:
                pos = tuple(fr["a"][i] + fr["height"] * fr["tip"][i] + s_c * fr["axis"][i] for i in range(3))
                points.append(FasteningPoint(position_xyz=pos, normal_xyz=fr["normal"]))
        for arm in arms:                              # 曲げタブの溶接 1〜2
            i0, i1 = arm["edge"], (arm["edge"] + 1) % 4
            a_xy, b_xy = hub_xy[i0], hub_xy[i1]
            d2 = _unit((b_xy[0] - a_xy[0], b_xy[1] - a_xy[1]))
            a = lay["to_space"]((a_xy[0] + d2[0] * arm["root_from_mm"], a_xy[1] + d2[1] * arm["root_from_mm"]))
            bpt = lay["to_space"]((a_xy[0] + d2[0] * arm["root_to_mm"], a_xy[1] + d2[1] * arm["root_to_mm"]))
            axis = tuple((bpt[i] - a[i]) / math.dist(a, bpt) for i in range(3))
            angle = math.radians(arm["fold_deg"])
            outward = _unit(_cross3(axis, normal))
            tip = tuple(outward[i] * math.cos(angle) - normal[i] * math.sin(angle) for i in range(3))
            centre = tuple(a[i] - normal[i] * arm["radius_mm"] for i in range(3))
            a2 = _rotate3(a, centre, axis, angle)
            arm_normal = _unit(_cross3(tip, axis))
            wanted = _draw_weighted(rng, DRAWN_ARM_POINT_WEIGHTS)
            t = rng.uniform(b, arm["length_mm"] - b)
            sh_a = arm["outline"].get("shrink_a_mm", 0.0) * t / arm["length_mm"]
            sh_b = arm["outline"].get("shrink_b_mm", 0.0) * t / arm["length_mm"]
            lo, hi = sh_a + b, width - sh_b - b
            if hi - lo < 0.0:
                ok = False
                break
            if wanted == 2 and hi - lo >= 2.0 * b + 2.0:
                svals = [lo + rng.uniform(0.0, 0.2) * (hi - lo), hi - rng.uniform(0.0, 0.2) * (hi - lo)]
            else:
                svals = [rng.uniform(lo, hi)]
            for s_ in svals:
                pos = tuple(a2[i] + t * tip[i] + s_ * axis[i] for i in range(3))
                points.append(FasteningPoint(position_xyz=pos, normal_xyz=arm_normal))
        if not ok:
            continue
        # ハブの穴2: 絞り壁側はフィレット帯 + 座面、タブ側は座面 + 逃げ
        margin = {0: walls["B"]["tangent_mm"] + b + 1.0, 1: walls["A"]["tangent_mm"] + b + 1.0,
                  2: b + 2.0, 3: walls["C"]["tangent_mm"] + b + 1.0}
        got: list = []
        for _ in range(DRAWN_POINT_ATTEMPTS):
            cand = (rng.uniform(0.0, front), rng.uniform(0.0, depth))
            if not _inside_convex(cand, hub_xy):
                continue
            if any(_dist_to_segment(cand, hub_xy[i], hub_xy[(i + 1) % 4]) < margin[i] for i in range(4)):
                continue
            if all(math.dist(cand, o) >= 2.0 * b + 2.0 for o in got):
                got.append(cand)
            if len(got) == DRAWN_HUB_POINTS:
                break
        if len(got) < DRAWN_HUB_POINTS:
            continue
        for cand in got:
            points.append(FasteningPoint(position_xyz=lay["to_space"](cand), normal_xyz=lay["normal"]))

        drawn = {"hub_xy": hub_xy, "walls": walls, "arms": arms, "origin": list(origin),
                 "hub_u": list(u), "hub_v": list(v), "seam_fillet_mm": fillet,
                 "corner_radius": ARM_TIP_RELIEF_MM, "skew_mm": skew,
                 "seams": {k: {"end": list(sm["end"]), "turn_deg": sm["turn_deg"]}
                           for k, sm in lay["seams"].items()}}
        return (dataclasses.replace(spec, point1=points[0], point2=points[1], extra_points=(),
                                    annotated_points=tuple(points), drawn=drawn,
                                    target_folds=None, min_bearing_radius_mm=b),
                None, None, None)
    return None

def _cross3(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a):
    k = math.sqrt(sum(c * c for c in a)) or 1.0
    return tuple(c / k for c in a)


def _rotate3(p, centre, axis, angle):
    d = tuple(p[k] - centre[k] for k in range(3))
    c, s_ = math.cos(angle), math.sin(angle)
    dot = sum(d[k] * axis[k] for k in range(3))
    cr = _cross3(axis, d)
    return tuple(centre[k] + d[k] * c + cr[k] * s_ + axis[k] * dot * (1 - c) for k in range(3))


# ---------------------------------------------------------------- 合成族(AutoMetalSheet 依頼 2026-09-06)
# 因子(折り数・断面・壁・腕・タブ・基板の形・締結点数)を 1 部品ごとに独立にサンプルし、族名で
# 構造が予測できない教師にする。土台は掃引(2 点族)で、パネルの側辺に分岐族の腕を縫合する。
# 成立しない組合せは**その因子だけ**引き直す(他の因子は保つ)。
# 裁定(2026-09-06): 折り 3 本は見送り(0〜2)、フランジの折れ角 20 度制限は外す、
# 構造変種は腕上に締結点の無い部品だけ、切欠きは円弧と矩形、タブ溶接は実車相当(7.5〜11)。
# 第2期(ML 返答 2026-09-06 夜): 壁は**短い壁腕**にして腕と共存させる(壁–腕の依存を解消)、
# ビードを区間にして基板上に締結点を置く、理由のある非凸(ベンドリリーフ・くびれ)だけにする、
# 非対称余白を 30% に、座面余裕をゲートで保証する。
COMPOSE_FOLDS = (0, 1, 2)
COMPOSE_SECTIONS = ("none", "bead", "rib")
COMPOSE_WALLS = ("none", "one", "both")
COMPOSE_ARMS = (0, 1, 2, 3, 4)
COMPOSE_TABS = (0, 1, 2)
COMPOSE_POINTS = (2, 3, 4, 5, 6, 7, 8)
COMPOSE_BASE_KINDS = (("convex", 0.35), ("asym", 0.30), ("relief", 0.20), ("waist", 0.15))
COMPOSE_WELD_BEARING_MM = (7.5, 11.0)      # タブ溶接(実車相当)
COMPOSE_ARM_WIDTH_MM = (20.0, 50.0)
COMPOSE_ARM_FOLD_DEG = (60.0, 110.0)      # 依頼は 60〜120。110 超は帯の裏へ回り込んで干渉が増える
COMPOSE_ARM_R_MM = (5.0, 15.0)
COMPOSE_ARM_LENGTH_MM = (25.0, 60.0)
COMPOSE_ARM_GAP_MM = 4.0                    # 同じ側辺の腕どうし・腕と切欠きの隙間
COMPOSE_END_MARGIN_MM = 3.0                 # 曲げの接線点からの逃げ
COMPOSE_WALL_HEIGHT_MM = (10.0, 25.0)       # 壁腕(短いフランジ)
COMPOSE_WALL_FOLD_DEG = (75.0, 90.0)
COMPOSE_WALL_SHARE = (0.4, 0.8)             # 壁腕の根本 / 空き区間
COMPOSE_RELIEF_DEPTH_MM = (3.0, 5.0)        # ベンドリリーフ(折り線の端の逃げ)
COMPOSE_RELIEF_LEN_MM = (5.0, 9.0)
COMPOSE_WAIST_R_MM = (5.0, 15.0)            # くびれ(締結点群の間で余白を絞る)
COMPOSE_ASYM_EXT_MM = (4.0, 15.0)
COMPOSE_HALF_WIDTH_RATIO = (1.0, 2.4)
COMPOSE_MAX_HALF_WIDTH_MM = 40.0
COMPOSE_FACTOR_RETRIES = 6
COMPOSE_PLACE_ATTEMPTS = 30
COMPOSE_BEAD_SPAN_TRIES = 20


def _compose_free_segments(steps, taken, bearing, n_panels):
    """(panel, side) ごとの空き区間 [(lo, hi), ...]。帯の端は座面 + 角の逃げ、曲げの
    接線点は少しだけ空ける。taken = {(panel, side): [(t0, t1), ...]}。"""
    out = {}
    for k, st in enumerate(steps):
        lo = (bearing + COMPOSE_END_MARGIN_MM + 2.0) if k == 0 else COMPOSE_END_MARGIN_MM
        hi = st["length"] - ((bearing + COMPOSE_END_MARGIN_MM + 2.0) if k == n_panels - 1
                             else COMPOSE_END_MARGIN_MM)
        for side in (-1, 1):
            segs = [(lo, hi)]
            for a, b in sorted(taken.get((k, side), [])):
                nxt = []
                for s0, s1 in segs:
                    if b + COMPOSE_ARM_GAP_MM <= s0 or a - COMPOSE_ARM_GAP_MM >= s1:
                        nxt.append((s0, s1))
                    else:
                        if a - COMPOSE_ARM_GAP_MM > s0:
                            nxt.append((s0, a - COMPOSE_ARM_GAP_MM))
                        if b + COMPOSE_ARM_GAP_MM < s1:
                            nxt.append((b + COMPOSE_ARM_GAP_MM, s1))
                segs = nxt
            out[(k, side)] = [(s0, s1) for s0, s1 in segs if s1 - s0 > 1.0]
    return out


def _compose_bead_span(rng, plan, spec, bead):
    """区間ビード: 両端の座面(2b)を避け、ランアウトが直線区間に収まる [s0, s3] を引く。
    全長ビードも 1/3 の確率で出す(既存 2 点族との連続性)。"""
    spans, total = path_spans(plan, spec.bend_radius_mm)
    inset = 2.0 * spec.min_bearing_radius_mm
    runout = max(BEAD_MIN_RUNOUT_MM, 2.0 * bead.depth_mm)
    straights = [(a, b) for a, b, st in spans if st]

    def ok(s0, s3):
        return (any(a - 1e-6 <= s0 and s0 + runout <= b + 1e-6 for a, b in straights)
                and any(a - 1e-6 <= s3 - runout and s3 <= b + 1e-6 for a, b in straights)
                and s3 - s0 >= 2.0 * runout + BEAD_MIN_BODY_MM)
    full = (inset, total - inset)
    if rng.random() < 1.0 / 3.0:
        return full if ok(*full) else None
    for _ in range(COMPOSE_BEAD_SPAN_TRIES):
        s0 = rng.uniform(inset, total - inset - 2.0 * runout - BEAD_MIN_BODY_MM)
        s3 = rng.uniform(s0 + 2.0 * runout + BEAD_MIN_BODY_MM, total - inset)
        if ok(s0, s3):
            return (s0, s3)
    return full if ok(*full) else None


def compose_part(rng: random.Random, knobs: Knobs) -> Result | None:
    """合成族: 因子を独立に引き、成立しない因子だけ引き直す。"""
    for _ in range(knobs.attempts):
        factors = {
            "folds": rng.choice(COMPOSE_FOLDS),
            "section": rng.choice(COMPOSE_SECTIONS),
            "walls": rng.choice(COMPOSE_WALLS),
            "arms": rng.choice(COMPOSE_ARMS),
            "tabs": rng.choice(COMPOSE_TABS),
            "base": _draw_weighted_label(rng, COMPOSE_BASE_KINDS),
            "points": rng.choice(COMPOSE_POINTS),
        }
        realised = dict(factors)
        # 帯は既定より広めに引く(座面半径の 1.0〜2.4 倍、上限 40)。切欠き・非対称余白・
        # 基板上の追加点に幅方向の余地が要るため。
        wide = dataclasses.replace(knobs, half_width_ratio=knobs.half_width_ratio or COMPOSE_HALF_WIDTH_RATIO,
                                   max_half_width_mm=knobs.max_half_width_mm or COMPOSE_MAX_HALF_WIDTH_MM)
        try:
            spec = _draw_spec(rng, wide, folds=factors["folds"])
            plan = plan_for(spec)
        except ValueError:
            continue
        bearing = spec.min_bearing_radius_mm
        weld = rng.uniform(*COMPOSE_WELD_BEARING_MM)

        # ---- 断面(その因子だけ引き直す: bead -> rib -> none)
        bead = rib = None
        bead_span = None
        section = factors["section"]
        for _try in range(COMPOSE_FACTOR_RETRIES):
            if section == "bead":
                got = resolve_bead_slacks(rng, spec, sample_bead(rng, spec.half_width_mm))
                if got is not None:
                    spec, bead = got
                    break
                section = rng.choice(("rib", "none"))
            elif section == "rib":
                got = _compose_rib(rng, spec) if spec.target_folds != 0 else None
                if got is not None:
                    spec, rib = got
                    break
                section = rng.choice(("bead", "none"))
            else:
                break
        realised["section"] = "bead" if bead else "rib" if rib else "none"
        try:
            plan = plan_for(spec)
            steps, _w = sweep_steps(plan, spec.bend_radius_mm)
        except ValueError:
            continue
        n_panels = len(steps)
        spans, total = path_spans(plan, spec.bend_radius_mm)
        straight_s = [a for a, b, st in spans if st]          # パネル k の弧長の始点
        if bead is not None:
            bead_span = _compose_bead_span(rng, plan, spec, bead)
            if bead_span is None:
                continue

        # ---- 基板の形: 非対称余白(断面と共存、フランジは使わない)
        ext = [0.0, 0.0]
        base = factors["base"]
        if base == "relief" and factors["folds"] == 0:
            base = rng.choice(("convex", "asym"))
        if base == "asym":
            ext[rng.choice((0, 1))] = rng.uniform(*COMPOSE_ASYM_EXT_MM)

        taken: dict = {}
        arms: list = []

        def free_slots():
            free = _compose_free_segments(steps, taken, bearing, n_panels)
            return [(key, seg) for key, segs in free.items() for seg in segs]

        # ---- 壁腕(短いフランジ)。none/one/both。both は反対側の辺を優先。
        n_walls = {"none": 0, "one": 1, "both": 2}[factors["walls"]]
        used_sides: set = set()
        for _wi in range(n_walls):
            slots = [(key, seg) for key, seg in free_slots() if seg[1] - seg[0] >= 25.0]
            prefer = [x for x in slots if x[0][1] not in used_sides] if used_sides else slots
            if not (prefer or slots):
                break
            (k, side), (s0, s1) = rng.choice(prefer or slots)
            width = max(25.0, (s1 - s0) * rng.uniform(*COMPOSE_WALL_SHARE))
            t0 = rng.uniform(s0, s1 - width)
            height = rng.uniform(*COMPOSE_WALL_HEIGHT_MM)
            arms.append({"panel": k, "side": side, "t0_mm": t0, "t1_mm": t0 + width,
                         "fold_deg": rng.uniform(*COMPOSE_WALL_FOLD_DEG),
                         "radius_mm": rng.uniform(*COMPOSE_ARM_R_MM),
                         "length_mm": height,
                         # 低い壁は先端の隅Rを高さに合わせて小さくする(R5 が高さ 10 の半分を食う)
                         "relief_mm": min(ARM_TIP_RELIEF_MM, 0.5 * height - 1.0),
                         "outline": {"kind": "rect"}, "role": "wall", "points": []})
            taken.setdefault((k, side), []).append((t0, t0 + width))
            used_sides.add(side)
        realised["walls"] = {0: "none", 1: "one", 2: "both"}[len(arms)]
        n_wall_arms = len(arms)

        # ---- 腕(側辺の空き区間に置く)。タブは腕に配る。
        tab_left = min(factors["tabs"], factors["arms"])
        for i in range(factors["arms"]):
            placed = False
            for _try in range(COMPOSE_PLACE_ATTEMPTS):
                slots = free_slots()
                if not slots:
                    break
                (k, side), (s0, s1) = rng.choice(slots)
                width = rng.uniform(*COMPOSE_ARM_WIDTH_MM)
                if s1 - s0 < width:
                    continue
                t0 = rng.uniform(s0, s1 - width)
                length = rng.uniform(*COMPOSE_ARM_LENGTH_MM)
                with_tab = tab_left > 0 and rng.random() < 0.7
                if with_tab:
                    foot = tab_footprint_mm(weld, ARM_TIP_RELIEF_MM)
                    if width < 2.0 * foot + 1.2:
                        continue
                    s_c = rng.uniform(foot + 0.6, width - foot - 0.6)
                    height = weld + rng.uniform(0.0, 4.0)
                    outline = {"kind": "tabs", "height_mm": height, "tabs": [[s_c, weld]],
                               "root_r_mm": ARM_TIP_RELIEF_MM}
                    length = height
                elif rng.random() < 0.4:
                    cap = max(0.0, (width - 2.0 * ARM_TIP_RELIEF_MM - 1.5) / 2.0)
                    outline = {"kind": "trapezoid", "shrink_a_mm": rng.uniform(0.0, min(cap, 0.25 * width)),
                               "shrink_b_mm": rng.uniform(0.0, min(cap, 0.25 * width))}
                else:
                    outline = {"kind": "rect"}
                arms.append({"panel": k, "side": side, "t0_mm": t0, "t1_mm": t0 + width,
                             "fold_deg": rng.uniform(*COMPOSE_ARM_FOLD_DEG),
                             "radius_mm": rng.uniform(*COMPOSE_ARM_R_MM), "length_mm": length,
                             "relief_mm": ARM_TIP_RELIEF_MM, "outline": outline,
                             "role": "arm", "points": []})
                taken.setdefault((k, side), []).append((t0, t0 + width))
                if with_tab:
                    tab_left -= 1
                placed = True
                break
            if not placed:
                break
        realised["arms"] = len(arms) - n_wall_arms
        realised["tabs"] = sum(1 for a in arms if a["outline"]["kind"] == "tabs")

        # ---- 締結点: アンカー 2 + 腕の上 + 基板の上(座面半径も記録)
        points = [spec.point1, spec.point2]
        radii = [bearing, bearing]
        for arm in arms:
            if arm["role"] != "arm":
                continue
            width_side = spec.half_width_mm + (ext[1] if arm["side"] > 0 else ext[0])
            fr = compose_arm_frame(steps[arm["panel"]], arm["side"], arm["t0_mm"], arm["t1_mm"],
                                   width_side, arm["fold_deg"], arm["radius_mm"])
            local = []
            if arm["outline"]["kind"] == "tabs":
                for s_c, r in arm["outline"]["tabs"]:
                    local.append((arm["length_mm"], s_c, "weld"))
            else:
                wanted = rng.choice((0, 1, 1, 2))
                w_a = arm["t1_mm"] - arm["t0_mm"]
                usable_l = arm["length_mm"] - ARM_TIP_RELIEF_MM
                sh_a = arm["outline"].get("shrink_a_mm", 0.0)
                sh_b = arm["outline"].get("shrink_b_mm", 0.0)
                for _p in range(COMPOSE_PLACE_ATTEMPTS):
                    if len(local) >= wanted or usable_l < 2.0 * bearing:
                        break
                    run = rng.uniform(bearing, usable_l - bearing)
                    # 台形は高さ run での有効幅の中に置く(斜辺からも座面ぶん離す)
                    frac = run / arm["length_mm"]
                    lo, hi = sh_a * frac + bearing, w_a - sh_b * frac - bearing
                    if hi - lo < 0.0:
                        continue
                    across = rng.uniform(lo, hi)
                    if all(math.hypot(run - r2, across - a2) >= 2.0 * bearing + 2.0 for r2, a2, _k in local):
                        local.append((run, across, "bolt"))
            for run, across, kind in local:
                pos = tuple(fr["a"][j] + run * fr["tip"][j] + across * fr["axis"][j] for j in range(3))
                points.append(FasteningPoint(position_xyz=pos, normal_xyz=fr["normal"]))
                radii.append(weld if kind == "weld" else bearing)
            arm["points"] = [[run, across, kind] for run, across, kind in local]
        n_base = max(0, factors["points"] - len(points))
        base_points: list = []
        base_local: list = []              # (panel, t, y) — くびれの配置に使う
        # 断面が占める弧長の区間(ここには基板の点を置かない)
        blocked_s = []
        if bead is not None and bead_span is not None:
            blocked_s.append((bead_span[0] - bearing, bead_span[1] + bearing))
        if rib is not None:
            fold_s = [b for a, b, st in spans if st][rib.fold_index]      # 折れ目の始まりの弧長
            reach = rib.reach_mm(math.radians(fold_angle_deg(plan.panel_frames, rib.fold_index)))
            blocked_s.append((fold_s - reach - bearing, fold_s + spans[2 * rib.fold_index + 1][1]
                              - spans[2 * rib.fold_index + 1][0] + reach + bearing))
        for _p in range(COMPOSE_PLACE_ATTEMPTS * 3):
            if len(base_points) >= n_base:
                break
            k = rng.randrange(n_panels)
            st = steps[k]
            lo = (bearing + 2.0) if k == 0 else 0.0
            hi = st["length"] - ((bearing + 2.0) if k == n_panels - 1 else 0.0)
            if hi - lo < 2.0 * bearing:
                continue
            t = rng.uniform(lo + bearing, hi - bearing)
            s_arc = straight_s[k] + t
            if any(a <= s_arc <= b for a, b in blocked_s):
                continue
            y_max = spec.half_width_mm - bearing
            if y_max <= 0.0:
                break
            y = rng.uniform(-y_max, y_max)
            pos = tuple(st["origin"][j] + t * st["direction"][j] + y * st["ey"][j] for j in range(3))
            if any(math.dist(pos, q.position_xyz) < 2.0 * bearing + 2.0 for q in points + base_points):
                continue
            base_points.append(FasteningPoint(position_xyz=pos, normal_xyz=tuple(st["ez"])))
            base_local.append((k, t, y))
        points.extend(base_points)
        radii.extend([bearing] * len(base_points))

        # ---- 理由のある非凸: ベンドリリーフ(折り線の端) / くびれ(締結点群の間)
        notches: list = []
        room = spec.half_width_mm - bearing - 1.0
        if base == "waist":
            # 同じパネル上で走行方向に離れた 2 点(アンカー含む)の中間、両側に円弧の絞り
            locs = []
            if n_panels >= 1:
                locs.append((0, bearing))                                  # point1 は panel0 の run=bearing
                locs.append((n_panels - 1, steps[-1]["length"] - bearing))  # point2 は末尾
            locs += [(k, t) for k, t, _y in base_local]
            by_panel: dict = {}
            for k, t in locs:
                by_panel.setdefault(k, []).append(t)
            cands = []
            for k, ts in by_panel.items():
                ts.sort()
                for a, b in zip(ts, ts[1:]):
                    if b - a >= 2.0 * bearing + 2.0 * COMPOSE_WAIST_R_MM[0] + 2.0:
                        cands.append((k, a, b))
            if cands and room >= COMPOSE_WAIST_R_MM[0]:
                k, a, b = rng.choice(cands)
                r = min(rng.uniform(*COMPOSE_WAIST_R_MM), room, (b - a - 2.0 * bearing) / 2.0 - 0.5)
                t_c = (a + b) / 2.0
                for side in (-1, 1):
                    free = _compose_free_segments(steps, taken, bearing, n_panels)[(k, side)]
                    if any(s0 <= t_c - r and t_c + r <= s1 for s0, s1 in free):
                        notches.append({"panel": k, "side": side, "t_mm": t_c, "kind": "arc",
                                        "radius_mm": r, "half_span_mm": r, "depth_mm": r,
                                        "reason": "waist"})
                        taken.setdefault((k, side), []).append((t_c - r, t_c + r))
            if not notches:
                base = "convex"
        elif base == "relief":
            # 折り線の端の逃げ: 曲げの接線点のすぐ隣(平坦区間の中)に矩形の小さな切欠き
            depth = rng.uniform(*COMPOSE_RELIEF_DEPTH_MM)
            length = rng.uniform(*COMPOSE_RELIEF_LEN_MM)
            folds_to_relieve = list(range(n_panels - 1))
            rng.shuffle(folds_to_relieve)
            for kf in folds_to_relieve[: rng.choice((1, 1, 2))]:
                for k, t_c in ((kf, steps[kf]["length"] - length / 2.0 - 0.6), (kf + 1, length / 2.0 + 0.6)):
                    for side in (-1, 1):
                        free = _compose_free_segments(steps, taken, bearing, n_panels)[(k, side)]
                        # 空き区間は接線点から 3mm 空くので、リリーフはその外側に置けるよう判定を緩める
                        span_ok = any(s0 - COMPOSE_END_MARGIN_MM <= t_c - length / 2.0 and
                                      t_c + length / 2.0 <= s1 + COMPOSE_END_MARGIN_MM for s0, s1 in free)
                        if span_ok and not any(abs(t_c - tt) < length / 2.0 + bearing for kk, tt, yy in base_local
                                               if kk == k and side * yy > spec.half_width_mm - depth - bearing):
                            notches.append({"panel": k, "side": side, "t_mm": t_c, "kind": "rect",
                                            "depth_mm": depth, "length_mm": length,
                                            "corner_r_mm": min(ARM_TIP_RELIEF_MM, depth * 0.45),
                                            "half_span_mm": length / 2.0, "reason": "bend_relief"})
                            taken.setdefault((k, side), []).append((t_c - length / 2.0, t_c + length / 2.0))
            if not notches:
                base = "convex"
        realised["base"] = base
        realised["notches"] = len(notches)
        realised["points"] = len(points)
        realised["base_points"] = len(base_points)
        realised["arm_points"] = sum(len(a["points"]) for a in arms)

        compose = {"factors_sampled": factors, "factors": realised, "arms": arms,
                   "notches": notches, "side_extension_mm": ext, "weld_bearing_mm": weld,
                   "n_panels": n_panels, "bead_span": list(bead_span) if bead_span else None,
                   "point_radii": radii}
        return (dataclasses.replace(spec, annotated_points=tuple(points), compose=compose),
                bead, None, rib)
    return None

def _compose_rib(rng: random.Random, spec):
    """リブ付きの spec(曲げRを最小に固定)とリブ。rib_part と同じ手順。載らなければ None。"""
    candidate = dataclasses.replace(spec, bend_radius_mm=RIB_BEND_RADIUS_MM)
    try:
        plan = plan_for(candidate)
    except ValueError:
        return None
    folds = len(plan.panel_frames) - 1
    if folds < 1 or max_fold_angle_deg(plan.panel_frames) < RIB_MIN_FOLD_ANGLE_DEG:
        return None
    best = max(range(folds), key=lambda i: min(leg_room_mm(plan, i)))
    angle = math.radians(fold_angle_deg(plan.panel_frames, best))
    for _ in range(8):
        rib = sample_rib(rng, half_width_mm=candidate.half_width_mm, fold_index=best,
                         leg_room_mm=leg_room_mm(plan, best), fold_angle_rad=angle)
        if rib is not None:
            return candidate, rib
    return None


def _draw_weighted_label(rng: random.Random, weights):
    roll, cumulative = rng.random(), 0.0
    for value, weight in weights:
        cumulative += weight
        if roll < cumulative:
            return value
    return weights[-1][0]


FAMILIES = {
    "bead": bead_part,
    "flange": flange_part,
    "rib": rib_part,
    "plain": plain_part,
    "three_point": three_point_part,
    "three_point_tri": three_point_tri_part,
    "three_point_span": three_point_span_part,
    "flat_plate": flat_plate_part,
    "branch": branch_part,
    "channel_seat": channel_seat_part,
    "tab_bracket": tab_bracket_part,
    "drawn_tray": drawn_tray_part,
    "compose": compose_part,
}


def kind_of(bead, flange, rib) -> str:
    """特徴の名前。ビード + フランジの併存(実車014型)は "bead+flange"。"""
    if rib:
        return "rib"
    present = [name for name, value in (("bead", bead), ("flange", flange)) if value]
    return "+".join(present) or "plain"


def classify_spec(spec) -> str:
    return str(classify(spec.point1, spec.point2))

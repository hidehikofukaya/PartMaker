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
from synthetic_generator.occt_build import ARM_TIP_RELIEF_MM, branch_frames, channel_seat_frames
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
}


def kind_of(bead, flange, rib) -> str:
    """特徴の名前。ビード + フランジの併存(実車014型)は "bead+flange"。"""
    if rib:
        return "rib"
    present = [name for name, value in (("bead", bead), ("flange", flange)) if value]
    return "+".join(present) or "plain"


def classify_spec(spec) -> str:
    return str(classify(spec.point1, spec.point2))

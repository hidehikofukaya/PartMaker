"""任意の法線・任意の位置の締結点2点ペアをサンプリングする(roadmap SS6.20〜6.21)。

`parallel_same_offset.py`(法線がほぼ一致する`parallel_same_offset`クラス専用)を
一般化したもの: 法線n1・n2は完全にランダムな相対角度(0〜180度)を取り、締結点2の
位置も締結点1から見て完全にランダムな3D方向・距離を取る。`classify()`による分類は
5クラスいずれになってもよい(`coplanar_flat`のみ、締結点が同一平面上で強度部品扱いに
ならないクラスだが、メイン形状生成自体はこのモジュールの対象外にしない — 剛性方向の
補強が要らないだけで、面を繋ぐこと自体は引き続き必要なため)。

フランジ・Phase 1.5トリムは未対応(`include_flanges`を持たない`build_general_two_point`
専用、SS6.20参照)。

2026-08-24(docs/catia_bead_fillet_investigation_log.md SS8): 基準面の作り方を
「座面2枚の交線+測地線」方式(自由折れ目チェーン、`classify.free_fold_seed`/
`solve_free_fold`)に置き換えた。中間折れ目の位置は締結点の最小必要平面(bearing半径)+
フィレット接線長+ランダムslackで決まり、接線長は基準面フィレット半径Rに依存するため、
**Rを他のどのパラメータより先にサンプリングする必要がある**(ユーザー確定方針)。
実際の閉合ソルブ(ホモトピー継続、フィレット干渉・曲げ角上限などの権威あるチェック)は
`gsd_build.build_general_two_point`側で行う(`parallel_same_offset.py`と同じ
「サンプラー側はおおまかに、gsd_build.py側で権威あるチェック」の分担)。
"""

from __future__ import annotations

import dataclasses
import math
import random

from synthetic_generator.classify import (
    MAX_FOLD_ANGLE_DEG,
    MIN_BASE_BEND_RADIUS_MM,
    FasteningPoint,
    Vec3,
    MAX_FOLD_TILT_DEG,
    free_fold_seed,
    rotate_about_axis,
    solve_free_fold,
    two_point_frame,
)
from synthetic_generator.bead import BeadParams, sample_bead
from synthetic_generator.flange import (
    FLANGE_MAX_FOLD_ANGLE_DEG,
    FlangeParams,
    max_fold_angle_deg,
    plan_flange_on_surface,
    sample_flange,
)
from synthetic_generator.general_geometry import check_bead_feasible, plan_general_two_point

# ユーザー確定(2026-08-24): 締結点1つに必要な最小平面は**直径25mm**(=半径12.5mm)。
# それ以上大きくしても利点は乏しいので、板金の幅は25〜50mmに抑える。
# したがって bearing_radius(=締結点まわりに要る平地の半径)は 12.5〜25mm、
# half_width(=板幅の半分)は bearing_radius 以上・25mm以下。
MIN_BEARING_RADIUS_RANGE_MM = (12.5, 25.0)
HALF_WIDTH_MARGIN_RATIO_RANGE = (1.0, 1.3)  # bearing_radiusに対する幅方向の余裕
MAX_HALF_WIDTH_MM = 25.0  # 板幅の上限50mmの半分
BEND_RADIUS_CAP_MM = 50.0  # ユーザー知見: 小部品でもR40〜50まであり得る、の上限(parallel_same_offset.pyと同じ)

# 中間折れ目位置(bearing半径+接線長からの追加距離)のランダムslack。SS8.4の実測で
# この範囲(0〜60mm)を使い、総合成立率70%前後・帯全長中央値200mm前後という
# 妥当な結果を確認済み。
FOLD_SLACK_RANGE_MM = (0.0, 60.0)

# fold1の傾き(a1)を、free_fold_seedが返す「必要最小限の傾き」からどれだけ追加で
# 動かすか(ユーザー①の「中間折れ位置とベクトルに若干のランダム性」に対応)。
# 2026-08-25の層別分析: `solve_free_fold`が収束しないケースの判別変数はこの摂動幅
# だった(失敗時の中央値13.8度 vs 成功時8.6度)。摂動が大きいとホモトピー継続が
# 追従できず、傾き上限も超えやすい。±20度→±8度で通過率が68%→80%に上がる。
# seed自身の傾きa1(中央値約5度)と同程度なので、多様性は十分残る。
# 実測の通過率: ±20度 68% / ±8度 78.7% / ±6度 82.1% / ±4度 85.5%。
# ユーザー目標80%を満たす範囲で多様性を最大化する±6度を採る。
# 傾き摂動の幅。上限5度に対して±6度では探索候補の大半が上限側でクランプされて
# 無駄になるため、上限内に収まる±3度とする(2026-08-25、傾き上限30->5度の変更に追随)。
FOLD_TILT_PERTURBATION_RANGE_DEG = (-3.0, 3.0)

# 締結点2の配置(2026-08-25、層別分析にもとづく)。
#
# 以前は「3D一様な方向へ60〜200mm」だったが、折れ目は全て共通方向wに平行なので、
# **wに沿った変位(幅方向ズレ)は中間折れ目の配置予算を一切消費せず、代わりに
# 折れ目を傾けさせる**。3D一様だとこのズレが板幅を大きく超え、パネルが激しく
# シアーして成立しなくなる。層別実測:
#
#   ズレ比 dw/断面距離  <0.1  0.25-0.5  0.5-1   1-2   >=2
#   通過率              63%     59%     43%     9%    1%
#   断面距離            <60mm  90-120  120-150  >=150
#   通過率                0%     39%     52%     66%
#
# そこで p2 は **w直交平面内の距離(断面距離)と、wに沿ったズレ比** で指定する。
# 法線が平行で w が定まらない場合だけ、従来の3D一様方向へフォールバックする。
SECTION_DISTANCE_RANGE_MM = (120.0, 220.0)  # w直交平面内で測った締結点間距離
# 横ズレ比の上限。傾きの上限MAX_FOLD_TILT_DEG=5度(tan5deg=0.087)が実質的に許すのは
# 比0.1前後まで — 実測(2026-08-25、300試行): 比0.15-0.45の帯は84%が傾き棄却で
# OK率4%、比<0.09なら傾き棄却11%・OK率36%。0.40のままでは試行の65%が最初から
# 通らない帯に置かれる(feasible-by-construction違反)。出力分布は変わらない
# (上限が既に排除している領域なので)。入力カバレッジを広げたくなったら、傾き上限と
# セットで再検討すること。
MAX_LATERAL_OFFSET_RATIO = 0.10             # |dw| / 断面距離 の上限
OFFSET_DISTANCE_RANGE_MM = (120.0, 220.0)   # 法線平行時のフォールバック用

# 中間折れ目の間に最低限残すランプの長さ。slack予算の計算に使う。
MIN_RAMP_LENGTH_MM = 20.0

# 折れ角の配分の狙い(2026-08-25の層別分析)。中間折れ目の位置(slack)はランプ方向を
# 決め、それが2つの折れ角の配分を決める。配分が偏る(片方がほぼ0度)と、閉合条件が
# もう片方の傾きa2に大きな値を要求して上限を超える(残り失敗の9%)。逆に両方が急だと
# ホモトピー継続が収束しない(同6%)。実測の層別:
#   小さい方の折れ角 <10度 -> 通過66%(傾き上限超28%) / 20〜40度 -> 89%
#   大きい方の折れ角 >=110度 -> 通過71%以下(solve未収束19%) / <70度 -> 92%
# slackを0〜60mmで振ると小さい方は中央値40度・大きい方は43.5度動かせ、**97%のケースで
# 両方の狙いを満たす組合せが存在する**ので、法線相対角の分布に手を付けず(=直交などの
# 配置クラスの多様性を保ったまま)配分だけで是正できる。
TARGET_MIN_FOLD_ANGLE_DEG = 25.0
TARGET_MAX_FOLD_ANGLE_DEG = 95.0
FOLD_SPLIT_SEARCH_ATTEMPTS = 8

# 法線の相対角のうち、実測で通過率が落ち込む帯(2026-08-25)。
#   <30度 91% / 30〜50度 89% / 50〜70度 84% / **70〜110度 78〜80%** / 110〜130度 86% / >=130度 89%
# 部品が大きく向きを変える必要があり、折れ角が急になるため。単調ではなく中央が谷なので
# 「上限を絞る」のではなく**帯の重みだけ下げる**。除外すると直交(orthogonal)クラスを
# 丸ごと失うため、引き直しは1回だけにして帯自体は残す(ユーザー指示「ほどほどに」)。
NORMAL_ANGLE_DIP_RANGE_DEG = (70.0, 110.0)
NORMAL_ANGLE_DIP_RESAMPLE_PROB = 0.5

# 傾き摂動の候補数。残る失敗(傾き上限超・solve未収束)はどちらも「実際に閉合を解いて
# みないと分からない」量なので、サンプラー側で`solve_free_fold`を数回試し、収束して
# かつ傾きが上限内に収まる摂動を選ぶ(このプロジェクト共通のfeasible-by-construction)。
# CATIAの1ビルドが数秒かかるのに対しここは純Pythonで軽いので、先に潰す方が得。
TILT_PERTURBATION_SEARCH_ATTEMPTS = 6


@dataclasses.dataclass(frozen=True)
class GeneralTwoJointSpec:
    point1: FasteningPoint
    point2: FasteningPoint
    thickness_mm: float
    hole_diameter_mm: float
    min_bearing_radius_mm: float
    half_width_mm: float
    bend_radius_mm: float  # 基準面フィレットR(fold1・fold2共通)。他の何よりも先に決まる
    fold1_slack_mm: float  # 中間折れ目1(fold1)の位置: bearing半径+接線長+この値
    fold2_slack_mm: float  # 同上、fold2側
    fold1_tilt_perturbation_rad: float  # fold1の傾きをfree_fold_seedの既定値から動かす量


def _random_unit_vector(rng: random.Random) -> Vec3:
    z = rng.uniform(-1.0, 1.0)
    theta = rng.uniform(0.0, 2 * math.pi)
    r = math.sqrt(max(0.0, 1.0 - z * z))
    return (r * math.cos(theta), r * math.sin(theta), z)


def _place_second_point(rng: random.Random, n1: Vec3, n2: Vec3, p1: Vec3) -> Vec3:
    """締結点2を、共通折れ目方向 w に対して制御した位置へ置く。

    w = n1 x n2 は全ての折れ目に共通の方向で、**w方向の変位は中間折れ目の配置予算を
    消費せず、代わりに折れ目を傾けさせる**。したがって「w直交平面内でどれだけ離すか
    (断面距離)」と「wへどれだけずらすか(ズレ比)」を別々に制御するのが正しい。
    法線が平行で w が定まらない場合のみ、従来の3D一様方向へフォールバックする。
    """
    cross = (
        n1[1] * n2[2] - n1[2] * n2[1],
        n1[2] * n2[0] - n1[0] * n2[2],
        n1[0] * n2[1] - n1[1] * n2[0],
    )
    cross_len = math.sqrt(sum(c * c for c in cross))
    if cross_len < 1e-9:
        distance = rng.uniform(*OFFSET_DISTANCE_RANGE_MM)
        direction = _random_unit_vector(rng)
        return tuple(p1[i] + distance * direction[i] for i in range(3))

    w = tuple(c / cross_len for c in cross)
    helper = (1.0, 0.0, 0.0) if abs(w[0]) < 0.9 else (0.0, 1.0, 0.0)
    e1_raw = (
        w[1] * helper[2] - w[2] * helper[1],
        w[2] * helper[0] - w[0] * helper[2],
        w[0] * helper[1] - w[1] * helper[0],
    )
    e1_len = math.sqrt(sum(c * c for c in e1_raw))
    e1 = tuple(c / e1_len for c in e1_raw)
    e2 = (
        w[1] * e1[2] - w[2] * e1[1],
        w[2] * e1[0] - w[0] * e1[2],
        w[0] * e1[1] - w[1] * e1[0],
    )

    theta = rng.uniform(0.0, 2.0 * math.pi)
    section = rng.uniform(*SECTION_DISTANCE_RANGE_MM)
    lateral = section * rng.uniform(-MAX_LATERAL_OFFSET_RATIO, MAX_LATERAL_OFFSET_RATIO)
    return tuple(
        p1[i] + section * (math.cos(theta) * e1[i] + math.sin(theta) * e2[i]) + lateral * w[i]
        for i in range(3)
    )


def _choose_fold_slacks(
    rng: random.Random,
    point1: FasteningPoint,
    point2: FasteningPoint,
    slack_budget_mm: float,
    *,
    bearing_radius: float,
    bend_radius: float,
) -> tuple[float, float]:
    """折れ角の配分が偏らないslackの組を選ぶ。

    候補をいくつか引いて`free_fold_seed`で実際の折れ角を評価し、
    「小さい方 >= TARGET_MIN / 大きい方 <= TARGET_MAX」を満たす最初の組を採る。
    どれも満たさない場合は、狙いからの逸脱が最小の組を採る(下流で弾かれてよい)。
    """
    best: tuple[float, float] | None = None
    best_penalty = float("inf")
    for _ in range(FOLD_SPLIT_SEARCH_ATTEMPTS):
        total = rng.uniform(0.0, min(slack_budget_mm, 2.0 * FOLD_SLACK_RANGE_MM[1]))
        share = rng.uniform(0.2, 0.8)
        # 予算は2本分まとめて配分するが、1本あたりの上限は守る。
        candidate = (
            min(total * share, FOLD_SLACK_RANGE_MM[1]),
            min(total * (1.0 - share), FOLD_SLACK_RANGE_MM[1]),
        )
        seed = free_fold_seed(
            point1, point2, bend_radius_mm=bend_radius, min_bearing_radius_mm=bearing_radius,
            fold1_slack_mm=candidate[0], fold2_slack_mm=candidate[1],
            max_fold_deg=MAX_FOLD_ANGLE_DEG,
        )
        if seed is None:
            penalty = float("inf")
        else:
            angles = (math.degrees(seed.fold1_angle_rad), math.degrees(seed.fold2_angle_rad))
            penalty = (
                max(0.0, TARGET_MIN_FOLD_ANGLE_DEG - min(angles))
                + max(0.0, max(angles) - TARGET_MAX_FOLD_ANGLE_DEG)
            )
            if penalty == 0.0:
                return candidate
        if penalty < best_penalty:
            best_penalty, best = penalty, candidate
    return best if best is not None else (0.0, 0.0)


def _choose_tilt_perturbation(
    rng: random.Random,
    point1: FasteningPoint,
    point2: FasteningPoint,
    *,
    bearing_radius: float,
    bend_radius: float,
    fold1_slack: float,
    fold2_slack: float,
) -> float:
    """閉合が実際に解けて、かつ傾きが上限内に収まる摂動を選ぶ。

    残りの失敗(`solve_free_fold`未収束、解けた傾きa2が上限超)はどちらも解いてみないと
    分からない。a1は上限へクランプできるがa2は閉合条件で決まるため制御できないので、
    ここで候補をいくつか試して通るものを採る。どれも通らなければ最初の候補を返す
    (下流の`build_general_two_point`が明示的にInfeasibleとして弾く)。
    """
    seed = free_fold_seed(
        point1, point2, bend_radius_mm=bend_radius, min_bearing_radius_mm=bearing_radius,
        fold1_slack_mm=fold1_slack, fold2_slack_mm=fold2_slack,
        max_fold_deg=MAX_FOLD_ANGLE_DEG,
    )
    first: float | None = None
    if seed is None:
        return math.radians(rng.uniform(*FOLD_TILT_PERTURBATION_RANGE_DEG))
    cap = math.radians(MAX_FOLD_TILT_DEG)
    for _ in range(TILT_PERTURBATION_SEARCH_ATTEMPTS):
        perturbation = math.radians(rng.uniform(*FOLD_TILT_PERTURBATION_RANGE_DEG))
        if first is None:
            first = perturbation
        target = max(-cap, min(cap, seed.a1_rad + perturbation))
        chain = solve_free_fold(seed, point1, point2, target_a1_rad=target)
        if chain is None:
            continue
        if max(abs(chain.a1_rad), abs(chain.a2_rad)) <= cap:
            return perturbation
    return first if first is not None else 0.0


def sample(
    rng: random.Random,
    *,
    thickness_range_mm: tuple[float, float] = (1.0, 2.5),
    hole_diameter_range_mm: tuple[float, float] = (6.0, 14.0),
) -> GeneralTwoJointSpec:
    """任意の法線・任意の位置の締結点ペアを1組サンプリングする。"""
    n1 = _random_unit_vector(rng)
    rotation_axis = _random_unit_vector(rng)
    rotation_angle = rng.uniform(0.0, math.pi)  # 0〜180度、全configuration classをカバー
    dip_low, dip_high = (math.radians(a) for a in NORMAL_ANGLE_DIP_RANGE_DEG)
    if dip_low <= rotation_angle <= dip_high and rng.random() < NORMAL_ANGLE_DIP_RESAMPLE_PROB:
        rotation_angle = rng.uniform(0.0, math.pi)  # 谷の帯は重みを下げる(1回だけ引き直す)
    n2 = rotate_about_axis(n1, rotation_axis, rotation_angle)

    p1: Vec3 = (0.0, 0.0, 0.0)
    p2 = _place_second_point(rng, n1, n2, p1)

    point1 = FasteningPoint(position_xyz=p1, normal_xyz=n1)
    point2 = FasteningPoint(position_xyz=p2, normal_xyz=n2)

    thickness = rng.uniform(*thickness_range_mm)
    hole_diameter = rng.uniform(*hole_diameter_range_mm)
    bearing_radius = rng.uniform(*MIN_BEARING_RADIUS_RANGE_MM)
    half_width = min(MAX_HALF_WIDTH_MM, bearing_radius * rng.uniform(*HALF_WIDTH_MARGIN_RATIO_RANGE))

    # bend_radius_mmはhalf_widthの0.9倍を超えられない(gsd_build.build_general_two_point
    # の既存チェックと同じ)。これはhalf_widthだけで決まる独立な制約なので、ここで
    # 先に反映しておけば無駄なInfeasible試行を減らせる — 中間折れ位置(fold*_slack_mm)
    # とは無関係にhalf_widthさえ分かれば計算できるため、SS8.3の「Rを先に決める」原則
    # (T/L1/L3の相互依存)には抵触しない。
    max_bend_radius = min(BEND_RADIUS_CAP_MM, 0.9 * half_width)
    bend_radius = rng.uniform(MIN_BASE_BEND_RADIUS_MM, max(MIN_BASE_BEND_RADIUS_MM, max_bend_radius))

    # 中間折れ目の位置は「bearing半径 + フィレット接線長 + slack」で決まる。両側の合計が
    # 締結点間の距離を食い尽くすとランプが逆走し、折れ角180度で成立しなくなる。
    # slackを独立にサンプリングすると実測でseed成立率36.8%まで落ちるので、**実際に
    # 使える予算を距離から逆算してからその中でサンプリングする**(このプロジェクト共通の
    # feasible-by-construction方針)。これだけで74.9%、距離下限90mmと併せて80.8%になる。
    #
    # 距離はw直交断面へ射影して測る — 折れ目は全てwに平行なので、w方向の変位は
    # 中間折れ目の配置予算を消費しない(free_fold_seedの内部と同じ射影)。
    # two_point_frameは法線が完全平行かつ横変位ゼロという測度ゼロの縮退でのみ失敗する。
    # その場合はslackを0にしてfree_fold_seed側の判断に委ねる。
    try:
        frame = two_point_frame(point1, point2)
    except ValueError:
        frame = None

    if frame is None:
        slack_budget = 0.0
    else:
        def _project_out_w(point: Vec3) -> Vec3:
            along_w = sum(point[i] * frame.w[i] for i in range(3))
            return tuple(point[i] - along_w * frame.w[i] for i in range(3))

        section_gap = math.dist(_project_out_w(p1), _project_out_w(p2))
        # 接線長は折れ角が決まらないと定まらない(SS8.3の相互依存)。90度相当のT=Rで概算。
        slack_budget = section_gap - MIN_RAMP_LENGTH_MM - 2.0 * bearing_radius - 2.0 * bend_radius

    if slack_budget <= 0.0:
        fold1_slack = fold2_slack = 0.0  # 距離が足りない。free_fold_seed側で弾かれる
    else:
        fold1_slack, fold2_slack = _choose_fold_slacks(
            rng, point1, point2, slack_budget,
            bearing_radius=bearing_radius, bend_radius=bend_radius,
        )
    fold1_tilt_perturbation = _choose_tilt_perturbation(
        rng, point1, point2,
        bearing_radius=bearing_radius, bend_radius=bend_radius,
        fold1_slack=fold1_slack, fold2_slack=fold2_slack,
    )

    return GeneralTwoJointSpec(
        point1=point1,
        point2=point2,
        thickness_mm=thickness,
        hole_diameter_mm=hole_diameter,
        min_bearing_radius_mm=bearing_radius,
        half_width_mm=half_width,
        bend_radius_mm=bend_radius,
        fold1_slack_mm=fold1_slack,
        fold2_slack_mm=fold2_slack,
        fold1_tilt_perturbation_rad=fold1_tilt_perturbation,
    )


def resolve_bead_slacks(
    rng: random.Random,
    spec: GeneralTwoJointSpec,
    bead: BeadParams,
    *,
    slack_attempts: int = 20,
    bead_resample_attempts: int = 20,
) -> tuple[GeneralTwoJointSpec, BeadParams] | None:
    """この締結点ペアで**ビードまで成立する**slack(必要ならビードも)を探して返す。

    棄却は締結点自体の性質(折れ目の傾き)にだけ適用すべきで、折れ目の置き方の性質
    (パネルの平坦区間・逃げ位置・フットプリント)は棄却ではなくslackの選び直しで
    解決できる — 実測(2026-08-25、300試行): 平坦区間不足の棄却42件のうち64%、
    その他のビード計画棄却18件のうち50%が、同じ締結点のまま回収できた
    (必要なslack候補数の中央値は1)。棄却のままだと「短い中間パネルを強いる締結点
    配置にはビード付き学習データが存在しない」という分布の穴が空く。

    判定は`plan_general_two_point`+`check_bead_feasible` — gsd_buildが実機構築前に
    行う権威チェックと同一の関数なので、ここを通ればCATIA非依存の理由では落ちない。

    戻り値はslackを差し替えたspec(とビード)。どうしても成立しなければNone
    (傾き上限など締結点の性質で落ちるケース)。純Python(1判定あたり数ms)。
    """

    def feasible(slack1: float, slack2: float, candidate: BeadParams) -> bool:
        try:
            plan = plan_general_two_point(
                spec.point1,
                spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=slack1,
                fold2_slack_mm=slack2,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
            )
            check_bead_feasible(plan, candidate)
        except ValueError:
            return False
        return True

    if feasible(spec.fold1_slack_mm, spec.fold2_slack_mm, bead):
        return spec, bead
    for _ in range(slack_attempts):
        slack1 = rng.uniform(*FOLD_SLACK_RANGE_MM)
        slack2 = rng.uniform(*FOLD_SLACK_RANGE_MM)
        if feasible(slack1, slack2, bead):
            return dataclasses.replace(spec, fold1_slack_mm=slack1, fold2_slack_mm=slack2), bead
    # slackだけで駄目なら、ビード側も引き直す(フットプリントが板幅に対して大きすぎる等、
    # ビードの性質で落ちているケースはこちらで救う)
    for _ in range(bead_resample_attempts):
        slack1 = rng.uniform(*FOLD_SLACK_RANGE_MM)
        slack2 = rng.uniform(*FOLD_SLACK_RANGE_MM)
        candidate = sample_bead(rng, spec.half_width_mm)
        if feasible(slack1, slack2, candidate):
            return dataclasses.replace(spec, fold1_slack_mm=slack1, fold2_slack_mm=slack2), candidate
    return None


def _flange_feasible(spec: GeneralTwoJointSpec, flange: FlangeParams) -> bool:
    """このspec+フランジで、拡張幅込みの基準面計画とフランジ計画の両方が通るか。"""
    try:
        plan = plan_general_two_point(
            spec.point1,
            spec.point2,
            min_bearing_radius_mm=spec.min_bearing_radius_mm,
            half_width_mm=spec.half_width_mm,
            bend_radius_mm=spec.bend_radius_mm,
            fold1_slack_mm=spec.fold1_slack_mm,
            fold2_slack_mm=spec.fold2_slack_mm,
            fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
            side_extension_mm=(
                flange.extension_mm if flange.side < 0 else 0.0,
                flange.extension_mm if flange.side > 0 else 0.0,
            ),
        )
        plan_flange_on_surface(
            plan.panel_frames, flange,
            half_width_mm=spec.half_width_mm, fold_tangents=plan.fold_tangents,
        )
    except ValueError:
        return False
    return True


def resolve_reinforcement(
    rng: random.Random,
    spec: GeneralTwoJointSpec,
    *,
    slack_attempts: int = 20,
) -> tuple[GeneralTwoJointSpec, BeadParams | None, FlangeParams | None] | None:
    """補強の種類を選び、成立するspec(slack差し替え済み)と補強パラメータを返す。

    使い分けはユーザー指定(2026-08-25): **最大折れ角20度以下ならフランジ、
    それ以外(急でフランジ不成立)はビード**。種類が基準面の幾何の決定的な関数なので、
    学習器は「締結点 -> 補強の種類」も学べる。

    フランジがそのslackで成立しない場合はslackを選び直し(種類の判定は最初の成立plan
    での折れ角を保つ)、それでも駄目ならビードへフォールバックする。
    戻り値Noneは基準面自体が不成立(呼び出し側は次のspecへ)。
    """
    try:
        plan = plan_general_two_point(
            spec.point1,
            spec.point2,
            min_bearing_radius_mm=spec.min_bearing_radius_mm,
            half_width_mm=spec.half_width_mm,
            bend_radius_mm=spec.bend_radius_mm,
            fold1_slack_mm=spec.fold1_slack_mm,
            fold2_slack_mm=spec.fold2_slack_mm,
            fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
        )
    except ValueError:
        return None

    if max_fold_angle_deg(plan.panel_frames) <= FLANGE_MAX_FOLD_ANGLE_DEG:
        flange = sample_flange(
            rng, plan.panel_frames, plan.fold_tilts,
            spec.half_width_mm, spec.bend_radius_mm,
        )
        if flange is not None and _flange_feasible(spec, flange):
            return spec, None, flange
        # slackを選び直して再試行(平坦区間不足などはslackの性質、SS12と同じ理屈)
        for _ in range(slack_attempts):
            slack1 = rng.uniform(*FOLD_SLACK_RANGE_MM)
            slack2 = rng.uniform(*FOLD_SLACK_RANGE_MM)
            candidate = dataclasses.replace(spec, fold1_slack_mm=slack1, fold2_slack_mm=slack2)
            try:
                plan2 = plan_general_two_point(
                    candidate.point1,
                    candidate.point2,
                    min_bearing_radius_mm=candidate.min_bearing_radius_mm,
                    half_width_mm=candidate.half_width_mm,
                    bend_radius_mm=candidate.bend_radius_mm,
                    fold1_slack_mm=slack1,
                    fold2_slack_mm=slack2,
                    fold1_tilt_perturbation_rad=candidate.fold1_tilt_perturbation_rad,
                )
            except ValueError:
                continue
            if max_fold_angle_deg(plan2.panel_frames) > FLANGE_MAX_FOLD_ANGLE_DEG:
                continue  # 種類の判定を跨ぐslackは採らない(フランジ条件の部品はフランジのまま)
            flange = sample_flange(
                rng, plan2.panel_frames, plan2.fold_tilts,
                candidate.half_width_mm, candidate.bend_radius_mm,
            )
            if flange is not None and _flange_feasible(candidate, flange):
                return candidate, None, flange
        # フランジ不成立 -> ビードへフォールバック(ユーザー指定の使い分け)

    bead = sample_bead(rng, spec.half_width_mm)
    resolved = resolve_bead_slacks(rng, spec, bead)
    if resolved is None:
        return None
    new_spec, new_bead = resolved
    return new_spec, new_bead, None

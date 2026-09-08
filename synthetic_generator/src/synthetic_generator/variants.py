"""設計等価バリアントと締結点摂動(2026-09-06、AutoMetalSheet 依頼 A/B)。

依頼の根拠(AutoMetalSheet 実測): 締結点は設計の全部を決めない。同じ締結点に対して
生成器が**自由に選んだ**選択(フランジの側、折り位置、帯幅、ビード断面…)を変えた部品は
等価な正解で、評価と学習(集合値の教師)の両方で誤差が下がった。

ここでは params/<part_id>.json(生成時の spec とパラメータ)から部品を復元し、
**締結点(位置・法線・座面半径)と連続スペック(板厚・曲げR)を固定したまま**、
入力から決まらない選択だけを変えた候補を列挙する。成立するかはビルドと同じゲート
(`check_shape` + check_points)で決めるので、ここでは安価な事前判定だけ行う。

族ごとに何が自由か(2026-09-06 の調査):

| 族 | 自由な選択 | 固定(締結点が動くので変えられない) |
|---|---|---|
| 2曲げ(bead/flange, occt11) | 折り位置(slack 2本)、帯の半幅、ビード断面、フランジの側/高さ/根本R | 折れ角(slack と点で決まる) |
| 1曲げ(rib/plain/3点族) | 帯の半幅、ビード断面、フランジの側/高さ/根本R、リブの幅/膨らみ | 折り位置(2点と法線の交線で一意) |
| 平板 | 余白、隅R | 凸包(締結点で決まる) |
| 分岐 | ハブ角のくぼみR、腕の長さ(締結点+座面を含む範囲で) | ハブの形・腕の幅・折れ角(腕上の締結点が動く) |
| チャンネル+座面 | ウェブ長(奥側)、座面の奥行き | 幅・深さ・壁角・斜辺・座面の側(座面上の締結点が動く) |
"""
from __future__ import annotations

import dataclasses
import math
import random
import zlib

from synthetic_generator.bead import BeadParams, sample_bead
from synthetic_generator.classify import FasteningPoint
from synthetic_generator.families import (
    BRANCH_ARM_LENGTH_MM, BRANCH_CORNER_R_MM, CHANNEL_SEAT_DEPTH_MM, CHANNEL_WEB_LENGTH_MM,
    FLAT_PLATE_CORNER_RADIUS_MM, FLAT_PLATE_MARGIN_RATIO, THREE_POINT_MAX_HALF_WIDTH_MM,
    THREE_POINT_SPAN_BEAD_DEPTH_MM, THREE_POINT_SPAN_BEAD_TOP_WIDTH_MM,
    THREE_POINT_SPAN_FLANGE_HEIGHT_MM, THREE_POINT_SPAN_MAX_HALF_WIDTH_MM,
    THREE_POINT_SPAN_NARROW_MARGIN_MM, _bead_half_footprint_mm, fold_angle_deg,
)
from synthetic_generator.flange import (
    FLANGE_HEIGHT_RANGE_MM, FLANGE_ROOT_RADIUS_RANGE_MM, FlangeParams, max_flange_height_mm,
)
from synthetic_generator.general_geometry import check_bead_feasible_occt, plan_for
from synthetic_generator.occt_build import (
    ARM_TIP_RELIEF_MM, branch_frames, channel_seat_frames, sweep_steps,
)
from synthetic_generator.families import _compose_rib
from synthetic_generator.templates.general_two_point import resolve_bead_slacks
from synthetic_generator.rib import RibParams, leg_room_mm, sample_rib
from synthetic_generator.templates.general_two_point import (
    FOLD_SLACK_RANGE_MM, MAX_HALF_WIDTH_MM, GeneralTwoJointSpec, _flange_feasible,
)

SWEEP_KINDS = {"bead", "flange", "rib", "plain", "three_point", "three_point_tri",
               "three_point_span"}
NEAR_MM = 1.0            # 元の値とこれ未満しか違わない候補は「同じ設計」とみなして捨てる
QUANTILES = (0.15, 0.5, 0.85)


@dataclasses.dataclass(frozen=True)
class Variant:
    knob: str
    value: str
    spec: GeneralTwoJointSpec
    bead: BeadParams | None
    flange: FlangeParams | None
    rib: RibParams | None
    changed: dict

    def name(self, part_id: str) -> str:
        return f"{part_id}__{self.knob}={self.value}"


# ---------------------------------------------------------------- params.json の復元

def _point(d) -> FasteningPoint:
    return FasteningPoint(tuple(d["position_xyz"]), tuple(d["normal_xyz"]))


def spec_from_meta(meta: dict):
    """params/<id>.json から (spec, bead, flange, rib) を復元する。"""
    s = dict(meta["spec"])
    s["point1"], s["point2"] = _point(s["point1"]), _point(s["point2"])
    s["extra_points"] = tuple(_point(p) for p in (s.get("extra_points") or ()))
    ann = s.get("annotated_points")
    s["annotated_points"] = tuple(_point(p) for p in ann) if ann else None
    known = {f.name for f in dataclasses.fields(GeneralTwoJointSpec)}
    spec = GeneralTwoJointSpec(**{k: v for k, v in s.items() if k in known})
    bead = BeadParams(**meta["bead"]) if meta.get("bead") else None
    flange = FlangeParams(**meta["flange"]) if meta.get("flange") else None
    rib = RibParams(**meta["rib"]) if meta.get("rib") else None
    return spec, bead, flange, rib


def part_rng(part_id: str, salt: str = "variants") -> random.Random:
    """部品IDから決まる乱数列(再実行しても同じ候補が出る)。"""
    return random.Random(zlib.crc32(f"{part_id}:{salt}".encode()))


# ---------------------------------------------------------------- 共通の小道具

def _fmt(x: float) -> str:
    return f"{x:.1f}"


def _quantile_values(lo: float, hi: float, original: float, count: int = 3):
    """[lo, hi] の分位点から、元の値から NEAR_MM 以上離れたものを返す。"""
    if hi - lo < NEAR_MM:
        return []
    out = []
    for q in QUANTILES[:count]:
        v = lo + (hi - lo) * q
        if abs(v - original) >= NEAR_MM and all(abs(v - o) >= NEAR_MM for o in out):
            out.append(v)
    return out


def _plan_ok(spec, flange=None) -> bool:
    try:
        if flange is not None:
            return _flange_feasible(spec, flange)
        plan_for(spec)
        return True
    except ValueError:
        return False


def _bead_ok(spec, bead) -> bool:
    try:
        check_bead_feasible_occt(plan_for(spec), bead, spec.bend_radius_mm)
        return True
    except ValueError:
        return False


def _feasible(spec, bead, flange, rib) -> bool:
    """安価な事前判定(計画レベル)。最終判定はビルドのゲートに任せる。"""
    if bead is not None:
        return _bead_ok(spec, bead)
    if flange is not None:
        return _plan_ok(spec, flange)
    if rib is not None:
        try:
            plan = plan_for(spec)
        except ValueError:
            return False
        return rib.half_width_mm <= spec.half_width_mm - 3.0 and len(plan.panel_frames) > rib.fold_index + 1
    return _plan_ok(spec)


# ---------------------------------------------------------------- 掃引族の knob

def _slack_variants(spec, bead, flange, rib, rng, count: int):
    """折り位置。2曲げ族だけ自由(1曲げは2点と法線の交線で折り位置が一意)。"""
    if spec.target_folds != 2:
        return []
    out = []
    for _ in range(count * 6):
        if len(out) >= count:
            break
        s1, s2 = rng.uniform(*FOLD_SLACK_RANGE_MM), rng.uniform(*FOLD_SLACK_RANGE_MM)
        if abs(s1 - spec.fold1_slack_mm) < 3.0 and abs(s2 - spec.fold2_slack_mm) < 3.0:
            continue
        cand = dataclasses.replace(spec, fold1_slack_mm=s1, fold2_slack_mm=s2)
        if _feasible(cand, bead, flange, rib):
            out.append(Variant("slack", f"{s1:.0f}_{s2:.0f}", cand, bead, flange, rib,
                               {"fold1_slack_mm": [spec.fold1_slack_mm, s1],
                                "fold2_slack_mm": [spec.fold2_slack_mm, s2]}))
    return out


def _width_cap(kind: str) -> float:
    if kind == "three_point_span":
        return THREE_POINT_SPAN_MAX_HALF_WIDTH_MM
    if kind.startswith("three_point"):
        return THREE_POINT_MAX_HALF_WIDTH_MM
    return MAX_HALF_WIDTH_MM


def _lateral_need_mm(spec) -> float:
    """帯の半幅の下限: 追加の締結点(3点族の3点目など)の幅方向オフセット + 座面半径。
    必要平面のルールを守る(帯の端ぎりぎりに点が乗るのは不可)。"""
    points = list(spec.extra_points or ()) + list(spec.annotated_points or ())
    if not points:
        return 0.0
    try:
        frames = plan_for(spec).panel_frames
    except ValueError:
        return 0.0
    need = 0.0
    for p in points:
        best = None
        for fr in frames:
            n = _normal_of(fr)
            off = abs(sum((p.position_xyz[i] - fr.origin[i]) * n[i] for i in range(3)))
            lat = abs(sum((p.position_xyz[i] - fr.origin[i]) * fr.v[i] for i in range(3)))
            if best is None or off < best[0]:
                best = (off, lat)
        if best is not None and best[0] < 1.0:
            need = max(need, best[1] + spec.min_bearing_radius_mm)
    return need


def _normal_of(frame):
    u, v = frame.u, frame.v
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    k = math.sqrt(sum(c * c for c in n)) or 1.0
    return tuple(c / k for c in n)


def _width_variants(kind, spec, bead, flange, rib, count: int):
    """帯の半幅。下限は座面半径・絞り幅+2・追加点の幅方向の必要平面、上限は族の上限。"""
    lo = max(spec.min_bearing_radius_mm, _lateral_need_mm(spec))
    if getattr(spec, "taper_half_width_mm", None):
        lo = max(lo, spec.taper_half_width_mm + 2.0)
    if rib is not None:
        lo = max(lo, rib.half_width_mm + 3.0)
    out = []
    for w in _quantile_values(lo, _width_cap(kind), spec.half_width_mm, count):
        cand = dataclasses.replace(spec, half_width_mm=w)
        if _feasible(cand, bead, flange, rib):
            out.append(Variant("width", _fmt(w), cand, bead, flange, rib,
                               {"half_width_mm": [spec.half_width_mm, w]}))
    return out


def _bead_variants(kind, spec, bead, flange, rib, rng, count: int):
    if bead is None:
        return []
    top = THREE_POINT_SPAN_BEAD_TOP_WIDTH_MM if kind == "three_point_span" else None
    depth = THREE_POINT_SPAN_BEAD_DEPTH_MM if kind == "three_point_span" else None
    out = []
    for _ in range(count * 40):      # 成立率が1〜2割の族があるので多めに引く(1回数ms)
        if len(out) >= count:
            break
        cand = sample_bead(rng, spec.half_width_mm, top, depth)
        foot = _bead_half_footprint_mm(cand)
        if kind == "three_point_span":
            if foot + 2.0 * spec.min_bearing_radius_mm > spec.half_width_mm:
                continue
            if foot + THREE_POINT_SPAN_NARROW_MARGIN_MM > (spec.taper_half_width_mm or 0.0):
                continue      # 絞った端がビードの足を飲み込めない
        if (abs(cand.top_width_mm - bead.top_width_mm) < 2.0
                and abs(cand.depth_mm - bead.depth_mm) < 1.0):
            continue
        if any(abs(cand.top_width_mm - v.bead.top_width_mm) < 2.0
               and abs(cand.depth_mm - v.bead.depth_mm) < 1.0 for v in out):
            continue
        if _bead_ok(spec, cand):
            out.append(Variant("bead", f"{cand.top_width_mm:.0f}x{cand.depth_mm:.1f}",
                               spec, cand, flange, rib,
                               {"bead": [dataclasses.asdict(bead), dataclasses.asdict(cand)]}))
    return out


def _flange_variants(kind, spec, bead, flange, rib):
    if flange is None:
        return []
    out = []
    # 側(両側フランジの族には無い)
    if not flange.both_sides:
        alt = dataclasses.replace(flange, side=-flange.side)
        if _plan_ok(spec, alt):
            out.append(Variant("side", f"{alt.side:+d}", spec, bead, alt, rib,
                               {"flange.side": [flange.side, alt.side]}))
    # 高さ: 凹側クリアランスの上限 min(族の上限, R_bend - 2) の中で分位点
    limits = (THREE_POINT_SPAN_FLANGE_HEIGHT_MM if kind == "three_point_span"
              else FLANGE_HEIGHT_RANGE_MM)
    try:
        plan = plan_for(spec)
        h_max = min(limits[1], max_flange_height_mm(plan.panel_frames, flange.direction,
                                                    spec.bend_radius_mm))
    except ValueError:
        return out
    for h in _quantile_values(limits[0], h_max, flange.height_mm):
        root = min(flange.root_radius_mm, max(FLANGE_ROOT_RADIUS_RANGE_MM[0], h - 2.0))
        alt = dataclasses.replace(flange, height_mm=h, root_radius_mm=root)
        if _plan_ok(spec, alt):
            out.append(Variant("height", _fmt(h), spec, bead, alt, rib,
                               {"flange.height_mm": [flange.height_mm, h]}))
    # 根本R: 範囲の反対側の端
    lo, hi = FLANGE_ROOT_RADIUS_RANGE_MM
    root = hi if flange.root_radius_mm < (lo + hi) / 2.0 else lo
    root = min(root, flange.height_mm - 2.0)
    if abs(root - flange.root_radius_mm) >= NEAR_MM:
        alt = dataclasses.replace(flange, root_radius_mm=root)
        if _plan_ok(spec, alt):
            out.append(Variant("root_r", _fmt(root), spec, bead, alt, rib,
                               {"flange.root_radius_mm": [flange.root_radius_mm, root]}))
    return out


def _rib_variants(spec, bead, flange, rib, rng, count: int):
    if rib is None:
        return []
    try:
        plan = plan_for(spec)
    except ValueError:
        return []
    angle = math.radians(fold_angle_deg(plan.panel_frames, rib.fold_index))
    room = leg_room_mm(plan, rib.fold_index)
    out = []
    for _ in range(count * 8):
        if len(out) >= count:
            break
        cand = sample_rib(rng, half_width_mm=spec.half_width_mm, fold_index=rib.fold_index,
                          leg_room_mm=room, fold_angle_rad=angle,
                          bend_radius_mm=spec.bend_radius_mm)
        if cand is None:
            continue
        if (abs(cand.half_width_mm - rib.half_width_mm) < 2.0
                and abs(cand.bulge_radius_mm - rib.bulge_radius_mm) < 2.0):
            continue
        if any(abs(cand.half_width_mm - v.rib.half_width_mm) < 2.0
               and abs(cand.bulge_radius_mm - v.rib.bulge_radius_mm) < 2.0 for v in out):
            continue
        out.append(Variant("rib", f"c{cand.half_width_mm:.0f}_r{cand.bulge_radius_mm:.0f}",
                           spec, bead, flange, cand,
                           {"rib": [dataclasses.asdict(rib), dataclasses.asdict(cand)]}))
    return out


# ---------------------------------------------------------------- 専用ビルダー族の knob

def _plate_variants(spec, count: int):
    b = spec.min_bearing_radius_mm
    out = []
    for m in _quantile_values(FLAT_PLATE_MARGIN_RATIO[0] * b, FLAT_PLATE_MARGIN_RATIO[1] * b,
                              spec.plate_margin_mm, count):
        out.append(Variant("margin", _fmt(m), dataclasses.replace(spec, plate_margin_mm=m),
                           None, None, None, {"plate_margin_mm": [spec.plate_margin_mm, m]}))
    for r in _quantile_values(*FLAT_PLATE_CORNER_RADIUS_MM, spec.plate_corner_radius_mm, count):
        out.append(Variant("corner_r", _fmt(r),
                           dataclasses.replace(spec, plate_corner_radius_mm=r), None, None, None,
                           {"plate_corner_radius_mm": [spec.plate_corner_radius_mm, r]}))
    return out


def _branch_variants(spec, count: int):
    br = spec.branch
    out = []
    edges = {a["edge"] for a in br["arms"]}
    n = len(br["hub_xy"])
    has_corner = any((i - 1) % n in edges and i in edges for i in range(n))
    fillet = {int(k): v for k, v in (br.get("fillet_radius") or {}).items()}
    for r in (_quantile_values(*BRANCH_CORNER_R_MM, br["corner_radius"], count) if has_corner else []):
        alt = dict(br, corner_radius=r)
        out.append(Variant("corner_r", _fmt(r), dataclasses.replace(spec, branch=alt),
                           None, None, None, {"branch.corner_radius": [br["corner_radius"], r]}))
    # 腕の長さ: 腕上の締結点 + 座面 + 先端逃げ を含む最小値から先を振る
    lay = branch_frames(br["hub_xy"], br["arms"], origin=tuple(br["origin"]),
                        hub_u=tuple(br["hub_u"]), hub_v=tuple(br["hub_v"]),
                        corner_radius=br["corner_radius"], fillet_radius=fillet or None)
    need = {}
    for arm in br["arms"]:
        fr = lay["arms"][arm["edge"]]
        runs = [sum((p.position_xyz[i] - fr["a"][i]) * fr["tip"][i] for i in range(3))
                for p in spec.annotated_points]
        on_arm = [r for r, p in zip(runs, spec.annotated_points)
                  if abs(sum((p.position_xyz[i] - fr["a"][i]) * fr["normal"][i]
                             for i in range(3))) < 0.5 and r > 0.0]
        need[arm["edge"]] = (max(on_arm) if on_arm else 0.0) + spec.min_bearing_radius_mm + ARM_TIP_RELIEF_MM
    # 台の角の凸フィレット(実車1285-18)も自由
    for vtx, r0 in fillet.items():
        for r in _quantile_values(5.0, 10.0, r0, 2):
            alt = dict(fillet); alt[vtx] = r
            out.append(Variant("fillet_r", _fmt(r),
                               dataclasses.replace(spec, branch=dict(br, fillet_radius=alt)),
                               None, None, None, {f"branch.fillet_radius.{vtx}": [r0, r]}))
    lo = BRANCH_ARM_LENGTH_MM[0] if all((a.get("outline") or {}).get("kind", "rect") == "rect"
                                        for a in br["arms"]) else 0.0
    for delta in (0.0, 12.0, 24.0):
        arms = [a if (a.get("outline") or {}).get("kind") == "tabs" else
                dict(a, length_mm=min(max(need[a["edge"]] + delta, lo), BRANCH_ARM_LENGTH_MM[1] + 10.0))
                for a in br["arms"]]
        if all(abs(a["length_mm"] - o["length_mm"]) < NEAR_MM for a, o in zip(arms, br["arms"])):
            continue
        out.append(Variant("arm_len", f"d{delta:.0f}", dataclasses.replace(spec, branch=dict(br, arms=arms)),
                           None, None, None,
                           {"branch.arms.length_mm": [[a["length_mm"] for a in br["arms"]],
                                                      [a["length_mm"] for a in arms]]}))
    return out


def _channel_variants(spec, count: int):
    ch = spec.channel
    geom = {k: (tuple(v) if isinstance(v, list) else v) for k, v in ch.items()
            if k not in ("seat_depth_mm", "seat_corner_mm", "diag_deg", "seat_width_mm")}
    lay = channel_seat_frames(**geom)
    b = spec.min_bearing_radius_mm
    out = []
    # ウェブ長: 斜辺の端 + 2、ウェブ点 + 座面 を含む最小値から奥へ
    web = [p for p in spec.annotated_points
           if abs(sum(p.normal_xyz[i] * lay["normal"][i] for i in range(3))) > 0.999]
    wx = max((sum((p.position_xyz[i] - ch["origin"][i]) * ch["hub_u"][i] for i in range(3))
              for p in web), default=0.0)
    need_len = max(ch["diag_end_mm"] + 2.0, wx + b)
    for L in _quantile_values(need_len, max(need_len, CHANNEL_WEB_LENGTH_MM[1]), ch["length_mm"], count):
        out.append(Variant("length", _fmt(L), dataclasses.replace(spec, channel=dict(ch, length_mm=L)),
                           None, None, None, {"channel.length_mm": [ch["length_mm"], L]}))
    # 座面の奥行き: 座面上の締結点 + 座面 を含む最小値から
    across = 0.0
    for key, w in lay["walls"].items():
        for p in spec.annotated_points:
            if abs(sum(p.normal_xyz[i] * w["seat_normal"][i] for i in range(3))) > 0.999:
                across = max(across, sum((p.position_xyz[i] - w["seat_a"][i]) * w["seat_out"][i]
                                         for i in range(3)))
    need_depth = across + b
    for d in _quantile_values(need_depth, max(need_depth, CHANNEL_SEAT_DEPTH_MM[1]),
                              ch["seat_depth_mm"], count):
        out.append(Variant("seat_depth", _fmt(d),
                           dataclasses.replace(spec, channel=dict(ch, seat_depth_mm=d)),
                           None, None, None, {"channel.seat_depth_mm": [ch["seat_depth_mm"], d]}))
    return out


def _drawn_variants(spec, count: int):
    """絞りトレイ: 曲げタブの長さ(締結点 + 座面を含む最小値から)と継ぎ目のR。"""
    dr = spec.drawn
    b = spec.min_bearing_radius_mm
    out = []
    # フィレットは3本(継ぎ目と壁の根本)を同じ半径で振る(半径が違うと縁が自由曲線になる)。
    # 大きくするとハブの点がフィレット帯に掛かりうるので、小さい側だけ振る。
    for r in _quantile_values(6.0, dr["seam_fillet_mm"], dr["seam_fillet_mm"], count):
        walls = {k: dict(w, fillet_mm=r, tangent_mm=r * math.tan(math.radians(w["fold_deg"]) / 2.0))
                 for k, w in dr["walls"].items()}
        out.append(Variant("fillet_r", _fmt(r),
                           dataclasses.replace(spec, drawn=dict(dr, seam_fillet_mm=r, walls=walls)),
                           None, None, None, {"drawn.fillet_mm": [dr["seam_fillet_mm"], r]}))
    need = {}
    for arm in dr["arms"]:
        # 腕上の点: 腕面の法線と一致する点の、根本からの高さの最大 + 座面 + 先端逃げ
        need[arm["edge"]] = 2.0 * b + 2.0
    for delta in (0.0, 10.0, 20.0):
        arms = [dict(a, length_mm=min(max(need[a["edge"]] + delta, 25.0), 55.0)) for a in dr["arms"]]
        if all(abs(a["length_mm"] - o["length_mm"]) < NEAR_MM for a, o in zip(arms, dr["arms"])):
            continue
        out.append(Variant("arm_len", f"d{delta:.0f}", dataclasses.replace(spec, drawn=dict(dr, arms=arms)),
                           None, None, None,
                           {"drawn.arms.length_mm": [[a["length_mm"] for a in dr["arms"]],
                                                     [a["length_mm"] for a in arms]]}))
    return out


def _box_variants(spec, count: int):
    """多面ブラケット: 締結点が決めていない選択 = 角のR・腕の長さ・深さ2フランジの長さ・
    締結点の載っていない壁の高さ。点を保持している要素は動かさない(`point_owners`)。"""
    bx = spec.box
    b = spec.min_bearing_radius_mm
    owners = set(bx.get("point_owners") or ())
    out = []
    # 角のR: 大きくするとハブ上の点がフィレット帯に掛かりうるので小さい側だけ振る
    for r in _quantile_values(4.0, bx["corner_r_mm"], bx["corner_r_mm"], count):
        walls = {k: dict(w, fillet_mm=r,
                         tangent_mm=r * math.tan(math.radians(w["fold_deg"]) / 2.0))
                 for k, w in bx["walls"].items()}
        out.append(Variant("corner_r", _fmt(r),
                           dataclasses.replace(spec, box=dict(bx, corner_r_mm=r, walls=walls)),
                           None, None, None,
                           {"box.corner_r_mm": [bx["corner_r_mm"], r]}))
    # 腕の長さ(点が載っていれば座面が入る最小長は保つ)
    if bx["arms"]:
        for delta in (0.0, 10.0, 20.0):
            arms = []
            for a in bx["arms"]:
                floor = 2.0 * b + 2.0 if f"arm_{a['edge']}" in owners else 15.0
                arms.append(dict(a, length_mm=min(max(floor + delta, 18.0), 55.0)))
            if all(abs(a["length_mm"] - o["length_mm"]) < NEAR_MM
                   for a, o in zip(arms, bx["arms"])):
                continue
            out.append(Variant("arm_len", f"d{delta:.0f}",
                               dataclasses.replace(spec, box=dict(bx, arms=arms)),
                               None, None, None,
                               {"box.arms.length_mm": [[a["length_mm"] for a in bx["arms"]],
                                                       [a["length_mm"] for a in arms]]}))
    # 深さ2フランジの長さ
    if bx["flanges"]:
        for delta in (0.0, 8.0, 16.0):
            fls = []
            for i, f in enumerate(bx["flanges"]):
                floor = 2.0 * b + 2.0 if f"flange_{i}" in owners else 12.0
                fls.append(dict(f, length_mm=min(max(floor + delta, 12.0), 45.0)))
            if all(abs(f["length_mm"] - o["length_mm"]) < NEAR_MM
                   for f, o in zip(fls, bx["flanges"])):
                continue
            out.append(Variant("flange_len", f"d{delta:.0f}",
                               dataclasses.replace(spec, box=dict(bx, flanges=fls)),
                               None, None, None,
                               {"box.flanges.length_mm": [[f["length_mm"] for f in bx["flanges"]],
                                                          [f["length_mm"] for f in fls]]}))
    # 締結点の載っていない壁の高さ(タブが載っている壁は動かせない)
    free = [k for k, w in bx["walls"].items()
            if f"wall_{k}" not in owners and not w["tabs"]
            and not any(str(f["wall"]) == str(k) for f in bx["flanges"])]
    if free:
        for scale in (0.6, 1.4):
            walls = dict(bx["walls"])
            moved = False
            for k in free:
                w = bx["walls"][k]
                h = min(max(w["height_mm"] * scale, w["tangent_mm"] + 8.0), 60.0)
                if abs(h - w["height_mm"]) >= NEAR_MM:
                    walls[k] = dict(w, height_mm=h)
                    moved = True
            if moved:
                out.append(Variant("wall_h", f"x{scale:.1f}",
                                   dataclasses.replace(spec, box=dict(bx, walls=walls)),
                                   None, None, None,
                                   {"box.walls.height_mm": [
                                       {k: bx["walls"][k]["height_mm"] for k in free},
                                       {k: walls[k]["height_mm"] for k in free}]}))
    return out


def _panel_variants(spec, count: int):
    """大型パネル: 締結点が決めていない選択 = ビードの断面、腕の長さ、帯幅(広げる側だけ)。"""
    pn = spec.panel
    b = spec.min_bearing_radius_mm
    out = []
    beads = pn["beads"]
    if beads:
        base = dict(beads[0][1])
        for scale in (0.7, 1.3):
            depth = base["depth_mm"] * scale
            sb = base["ridge_radius_mm"] * math.tan(math.radians(base["wall_angle_deg"]) / 2.0)
            if depth - 2.0 * sb * math.sin(math.radians(base["wall_angle_deg"])) <= 0.5:
                continue
            if abs(depth - base["depth_mm"]) < NEAR_MM:
                continue
            new = [[y, dict(bd, depth_mm=depth)] for y, bd in beads]
            out.append(Variant("bead_depth", _fmt(depth),
                               dataclasses.replace(spec, panel=dict(pn, beads=new)),
                               None, None, None,
                               {"panel.bead.depth_mm": [base["depth_mm"], depth]}))
    if pn["arms"]:
        for delta in (0.0, 10.0, 20.0):
            arms = []
            for a in pn["arms"]:
                floor = (2.0 * b + 2.0) if a.get("points") else 12.0
                arms.append(dict(a, length_mm=min(max(floor + delta, 12.0), 60.0)))
            if all(abs(a["length_mm"] - o["length_mm"]) < NEAR_MM
                   for a, o in zip(arms, pn["arms"])):
                continue
            out.append(Variant("arm_len", f"d{delta:.0f}",
                               dataclasses.replace(spec, panel=dict(pn, arms=arms)),
                               None, None, None,
                               {"panel.arms.length_mm": [[a["length_mm"] for a in pn["arms"]],
                                                         [a["length_mm"] for a in arms]]}))
    # 帯幅は広げる方向だけ(狭めると縁の締結点が座面を失う)
    for extra in (6.0, 14.0):
        hw = min(pn["half_width_mm"] + extra, 70.0)
        if hw - pn["half_width_mm"] < NEAR_MM:
            continue
        out.append(Variant("half_width", _fmt(hw),
                           dataclasses.replace(spec, half_width_mm=hw,
                                               panel=dict(pn, half_width_mm=hw)),
                           None, None, None,
                           {"half_width_mm": [pn["half_width_mm"], hw]}))
    return out


def _panel_structure_variants(spec):
    """大型パネルの**構造**変種(AMS 依頼 7 §3-2)。締結点を保持していない要素だけ落とす。
    ビードは平地に点を置く設計なので、外しても締結点は保持される。"""
    pn = spec.panel
    out = []

    def has_point(arm):
        return bool(arm.get("points"))

    def add(label, beads=False, walls=False, arms=False):
        new_beads = [] if beads else pn["beads"]
        keep = [a for a in pn["arms"]
                if has_point(a) or (a["role"] == "wall" and not walls)
                or (a["role"] == "arm" and not arms)]
        if len(new_beads) == len(pn["beads"]) and len(keep) == len(pn["arms"]):
            return
        new = dict(pn, beads=new_beads, arms=keep,
                   bead_spans=[] if beads else pn["bead_spans"])
        out.append(Variant("structure", label, dataclasses.replace(spec, panel=new),
                           None, None, None,
                           {"structure": [{"beads": len(pn["beads"]), "arms": len(pn["arms"])},
                                          {"beads": len(new_beads), "arms": len(keep)}]}))

    add("nobeads", beads=True)
    add("nowall", walls=True)
    add("noarms", arms=True)
    add("plain", beads=True, walls=True, arms=True)
    return out


def _box_structure_variants(spec):
    """多面ブラケットの**構造**変種(AMS 依頼 7 §3-2、2026-09-09)。同じ締結点に対して
    構造だけを変える。締結点を保持している要素は外せないので、`point_owners` を見て
    「点の載っていない要素」だけを落とす。"""
    bx = spec.box
    owners = set(bx.get("point_owners") or ())
    out = []

    def drop(label, walls=False, arms=False, flanges=False, rib=False):
        w = dict(bx["walls"])
        closed = list(bx["closed"])
        if walls:
            w = {k: v for k, v in w.items()
                 if f"wall_{k}" in owners or v["tabs"]
                 or any(str(f["wall"]) == str(k) for f in bx["flanges"])}
            closed = [c for c in closed if len(w) >= 2]
        fl = bx["flanges"]
        dp = bx["deep"]
        if flanges:
            keep = [i for i, f in enumerate(fl)
                    if f"flange_{i}" in owners
                    or any(d["flange"] == i and f"deep_{j}" in owners
                           for j, d in enumerate(dp))]
            fl = [fl[i] for i in keep]
            remap = {old: new for new, old in enumerate(keep)}
            dp = [dict(d, flange=remap[d["flange"]]) for d in dp if d["flange"] in remap]
        a = bx["arms"]
        if arms:
            a = [x for x in a if f"arm_{x['edge']}" in owners]
        r = None if rib else bx.get("rib")
        new = dict(bx, walls=w, closed=closed, flanges=fl, deep=dp, arms=a, rib=r)
        changed = {"structure": [
            {"walls": len(bx["walls"]), "flanges": len(bx["flanges"]),
             "arms": len(bx["arms"]), "rib": 1 if bx.get("rib") else 0},
            {"walls": len(w), "flanges": len(fl), "arms": len(a), "rib": 0 if rib else
             (1 if bx.get("rib") else 0)}]}
        if changed["structure"][0] == changed["structure"][1]:
            return
        out.append(Variant("structure", label, dataclasses.replace(spec, box=new),
                           None, None, None, changed))

    drop("nowall", walls=True)
    drop("noarms", arms=True)
    drop("noflange", flanges=True)
    if bx.get("rib"):
        drop("norib", rib=True)
    drop("plain", walls=True, arms=True, flanges=True, rib=True)
    return out


def _structure_variants(spec, bead, flange, rib, rng):
    """合成族の構造変種(依頼 原則 C)。同じ締結点に対して構造だけを変える。
    腕を消す変種は腕の上に締結点が無い部品だけ(裁定 2026-09-06)。切欠き・壁腕・断面の
    付け外しは締結点に依らない(第2期、ML 返答)。"""
    cp = spec.compose
    arm_points = any(a["points"] for a in cp["arms"] if a.get("role", "arm") == "arm")
    out = []

    def variant(label, comp, b=None, f=None, r=None, changed=None):
        c2 = dict(comp)
        c2["factors"] = dict(comp["factors"], structure_variant=label)
        return Variant("structure", label, dataclasses.replace(spec, compose=c2), b, f, r, changed or {})

    walls = [a for a in cp["arms"] if a.get("role") == "wall"]
    real_arms = [a for a in cp["arms"] if a.get("role", "arm") == "arm"]
    # plain: 断面・壁・腕・切欠き無し(腕上に点が無いときだけ)
    if not arm_points and (bead or rib or cp["arms"] or cp["notches"]):
        out.append(variant("plain", dict(cp, arms=[], notches=[], side_extension_mm=[0.0, 0.0]),
                           changed={"structure": ["original", "plain"]}))
    # 切欠き
    if cp["notches"]:
        out.append(variant("nonotch", dict(cp, notches=[]), bead, None, rib,
                           {"notches": [len(cp["notches"]), 0]}))
    # 壁腕
    if walls:
        out.append(variant("nowall", dict(cp, arms=real_arms), bead, None, rib, {"walls": [len(walls), 0]}))
    # 腕
    if real_arms and not arm_points:
        out.append(variant("noarms", dict(cp, arms=walls), bead, None, rib, {"arms": [len(real_arms), 0]}))
    # 断面の付け外し
    if bead is not None:
        out.append(variant("nobead", dict(cp, bead_span=None), None, None, None, {"section": ["bead", "none"]}))
    elif rib is not None:
        out.append(variant("norib", cp, None, None, None, {"section": ["rib", "none"]}))
    else:
        got = None
        for _ in range(6):
            got = resolve_bead_slacks(rng, spec, sample_bead(rng, spec.half_width_mm))
            if got is not None:
                break
        if got is not None and got[0] == spec:            # slack を動かさずに載るビードだけ
            out.append(variant("bead", cp, got[1], None, None, {"section": ["none", "bead"]}))
        got = _compose_rib(rng, spec) if spec.target_folds != 0 else None
        if got is not None and abs(got[0].bend_radius_mm - spec.bend_radius_mm) < 1e-9:
            out.append(variant("rib", cp, None, None, got[1], {"section": ["none", "rib"]}))
    return out

# ---------------------------------------------------------------- 入口

def propose_variants(meta: dict, *, count: int = 8, rng: random.Random | None = None):
    """1部品の等価バリアント候補を最大 count 個。knob を交互に並べて、count が小さくても
    全ての自由度が1つずつは入るようにする。"""
    kind = meta["kind"]
    rng = rng or part_rng(meta["part_id"])
    spec, bead, flange, rib = spec_from_meta(meta)
    groups: list[list[Variant]] = []
    if kind in SWEEP_KINDS:
        groups.append(_flange_variants(kind, spec, bead, flange, rib))
        groups.append(_slack_variants(spec, bead, flange, rib, rng, 3))
        groups.append(_width_variants(kind, spec, bead, flange, rib, 3))
        groups.append(_bead_variants(kind, spec, bead, flange, rib, rng, 3))
        groups.append(_rib_variants(spec, bead, flange, rib, rng, 3))
    elif spec.plate_margin_mm is not None:        # flat_plate
        groups.append(_plate_variants(spec, 3))
    elif spec.branch is not None:                 # branch / tab_bracket(分岐ビルダーの拡張)
        groups.append(_branch_variants(spec, 3))
    elif spec.channel is not None:                # channel_seat
        groups.append(_channel_variants(spec, 3))
    elif spec.drawn is not None:                  # drawn_tray
        groups.append(_drawn_variants(spec, 3))
    elif spec.box is not None:                    # box_bracket(多面ブラケット)
        groups.append(_box_variants(spec, 3))
        groups.append(_box_structure_variants(spec))
    elif spec.panel is not None:                  # 大型パネル
        groups.append(_panel_variants(spec, 3))
        groups.append(_panel_structure_variants(spec))
    elif spec.compose is not None:                # 合成族: 構造レベルの変種
        groups.append(_structure_variants(spec, bead, flange, rib, rng))
    else:
        raise ValueError(f"unknown kind {kind}")
    # 交互に取る(側 -> slack -> 幅 -> ビード -> リブ -> 側の2つ目 ...)
    out: list[Variant] = []
    queues = [list(g) for g in groups if g]
    while queues and len(out) < count:
        for q in list(queues):
            if len(out) >= count:
                break
            out.append(q.pop(0))
            if not q:
                queues.remove(q)
    return out


def build_variant(builder, variant: Variant, out_dir: str, name: str):
    """バリアントをビルドする。フランジのキラリティ再試行(`build_general_part`)は
    knob を勝手に変えるので**使わない** — 指定した設計がそのまま成立するかだけを見る。"""
    spec = variant.spec
    if spec.box is not None:
        bx = spec.box
        return builder.build_box_bracket(
            [tuple(q) for q in bx["hub_xy"]], {int(k): w for k, w in bx["walls"].items()},
            set(bx["closed"]), bx["arms"], origin=tuple(bx["origin"]),
            hub_u=tuple(bx["hub_u"]), hub_v=tuple(bx["hub_v"]),
            corner_r_mm=bx["corner_r_mm"], out_dir=out_dir, part_name=name,
            flanges=bx["flanges"], rib=bx.get("rib"), deep=bx.get("deep", ()),
            check_points=spec.annotated_points,
            check_radii=tuple(bx.get("point_radii", ())))
    if spec.drawn is not None:
        dr = spec.drawn
        return builder.build_drawn_tray(
            dr["hub_xy"], dr["walls"], dr["arms"], origin=tuple(dr["origin"]),
            hub_u=tuple(dr["hub_u"]), hub_v=tuple(dr["hub_v"]), seam_fillet_mm=dr["seam_fillet_mm"],
            corner_radius=dr["corner_radius"], out_dir=out_dir, part_name=name,
            check_points=spec.annotated_points,
            check_radii=(spec.min_bearing_radius_mm,) * len(spec.annotated_points))
    if spec.channel is not None:
        geom = {k: (tuple(v) if isinstance(v, list) else v) for k, v in spec.channel.items()
                if k not in ("diag_deg", "seat_width_mm")}
        return builder.build_channel_seat(out_dir=out_dir, part_name=name,
                                          check_points=spec.annotated_points, **geom)
    if spec.branch is not None:
        br = spec.branch
        fillet = {int(k): v for k, v in (br.get("fillet_radius") or {}).items()} or None
        return builder.build_branch_part(
            br["hub_xy"], br["arms"], origin=tuple(br["origin"]), hub_u=tuple(br["hub_u"]),
            hub_v=tuple(br["hub_v"]), corner_radius=br["corner_radius"], out_dir=out_dir,
            part_name=name, check_points=spec.annotated_points, gussets=br["gussets"],
            fillet_radius=fillet)
    if spec.plate_margin_mm is not None:
        return builder.build_flat_plate(spec.annotated_points, margin_mm=spec.plate_margin_mm,
                                        corner_radius_mm=spec.plate_corner_radius_mm,
                                        out_dir=out_dir, part_name=name)
    cp = spec.compose or {}
    pn = spec.panel or {}
    if pn:
        from synthetic_generator.bead import BeadParams as _BP
        return builder.build_general_two_point(
            spec.point1, spec.point2, min_bearing_radius_mm=spec.min_bearing_radius_mm,
            half_width_mm=spec.half_width_mm, bend_radius_mm=spec.bend_radius_mm,
            fold1_slack_mm=spec.fold1_slack_mm, fold2_slack_mm=spec.fold2_slack_mm,
            fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
            target_folds=spec.target_folds, check_points=spec.annotated_points,
            out_dir=out_dir, part_name=name,
            beads=[(y, _BP(**bd)) for y, bd in pn["beads"]], arms=pn["arms"],
            check_radii=tuple(pn.get("point_radii", ())))
    return builder.build_general_two_point(
        spec.point1, spec.point2, min_bearing_radius_mm=spec.min_bearing_radius_mm,
        half_width_mm=spec.half_width_mm, bend_radius_mm=spec.bend_radius_mm,
        fold1_slack_mm=spec.fold1_slack_mm, fold2_slack_mm=spec.fold2_slack_mm,
        fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
        target_folds=spec.target_folds, extra_points=spec.extra_points,
        check_points=spec.annotated_points, taper_half_width_mm=spec.taper_half_width_mm,
        out_dir=out_dir, part_name=name, bead=variant.bead, flange=variant.flange,
        rib=variant.rib, arms=cp.get("arms", ()), notches=cp.get("notches", ()),
        side_extension_mm=tuple(cp.get("side_extension_mm", (0.0, 0.0))),
        bead_span=(tuple(cp["bead_span"]) if cp.get("bead_span") and variant.bead is not None else None),
        check_radii=tuple(cp.get("point_radii", ())))


def variant_meta(meta: dict, variant: Variant, name: str) -> dict:
    """バリアントの params(元の params と同じ形 + knob の記録)。"""
    return {
        "part_id": name, "source_part_id": meta["part_id"], "kind": meta["kind"],
        "knob": variant.knob, "value": variant.value, "changed": variant.changed,
        "spec": dataclasses.asdict(variant.spec),
        "bead": dataclasses.asdict(variant.bead) if variant.bead else None,
        "flange": dataclasses.asdict(variant.flange) if variant.flange else None,
        "rib": dataclasses.asdict(variant.rib) if variant.rib else None,
    }


# ---------------------------------------------------------------- 依頼 B: 締結点の摂動

def _in_plane_direction(rng: random.Random, normal) -> tuple:
    while True:
        g = tuple(rng.gauss(0.0, 1.0) for _ in range(3))
        along = sum(g[i] * normal[i] for i in range(3))
        u = tuple(g[i] - along * normal[i] for i in range(3))
        n = math.sqrt(sum(c * c for c in u))
        if n > 1e-3:
            return tuple(c / n for c in u)


def perturb_spec(meta: dict, rng: random.Random, *, ratio_range=(0.5, 2.0)):
    """締結点を1つ選んで面内に動かした spec を返す(index, 移動ベクトル, spec)。

    2点族は point1/point2、007型は point1/point2(=実点)を動かす。011型/014型は掃引の
    アンカーが実点ではないので、実点だけ動かしても面が変わらない(=対にならない)。
    その族は None を返す(アンカーの再導出が要る。未対応)。
    """
    spec, bead, flange, rib = spec_from_meta(meta)
    kind = meta["kind"]
    if kind not in ("bead", "flange", "rib", "plain", "three_point"):
        return None
    index = rng.choice((0, 1))
    point = (spec.point1, spec.point2)[index]
    direction = _in_plane_direction(rng, point.normal_xyz)
    distance = spec.min_bearing_radius_mm * rng.uniform(*ratio_range)
    moved = FasteningPoint(tuple(point.position_xyz[i] + distance * direction[i] for i in range(3)),
                           point.normal_xyz)
    fields = {"point1": moved} if index == 0 else {"point2": moved}
    if spec.annotated_points:
        ann = list(spec.annotated_points)
        for j, p in enumerate(ann):
            if p == point:
                ann[j] = moved
        fields["annotated_points"] = tuple(ann)
    cand = dataclasses.replace(spec, **fields)
    vector = tuple(distance * c for c in direction)
    return index, vector, Variant("perturb", "", cand, bead, flange, rib, {}), point


def face_diff(before: list, after: list, *, area_tol: float = 0.01, move_tol_mm: float = 0.5):
    """features v2 の faces[] 同士の差分。名前で対応づけ、面積または重心が動いた面、
    増えた面、消えた面を返す。"""
    a = {f["name"]: f for f in before}
    b = {f["name"]: f for f in after}
    changed, same = [], []
    for name in sorted(set(a) & set(b)):
        fa, fb = a[name], b[name]
        d_area = abs(fa["area_mm2"] - fb["area_mm2"]) / max(1e-9, fa["area_mm2"])
        move = math.dist(fa["centroid"], fb["centroid"])
        (changed if d_area > area_tol or move > move_tol_mm else same).append(name)
    return {"changed": changed, "unchanged": same,
            "added": sorted(set(b) - set(a)), "removed": sorted(set(a) - set(b))}

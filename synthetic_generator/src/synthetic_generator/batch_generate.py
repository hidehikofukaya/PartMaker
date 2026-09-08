"""締結点2点部品をバッチ生成する(Phase 1 PoCのエントリポイント)。

2つの生成経路がある: `generate_batch`(法線がほぼ一致するparallel_same_offsetクラス専用、
フランジ・Phase 1.5トリム対応)と`generate_general_batch`(任意の法線・任意の位置、
roadmap SS6.20〜6.22。フランジ・トリムは未対応、まずメイン形状生成の成功を優先する
2026-08-10の方針転換による)。

実機(CATIA V5-6)で動作確認済み。builder引数にスタブを渡せば、サンプリング〜
joints.json書き出しまでのオーケストレーションだけをCATIA無しでテストできる
(tests/test_batch_generate.py参照)。

生成物(STP/CATPart/joints.json)は既定で`synthetic_parts/`配下に出力する(2026-08-10、
本パッケージ自体をAutoMetalSheetから独立したPartMakerリポジトリへ移管したため、
コード・生成物とも本リポジトリ内で完結する)。
"""

from __future__ import annotations

import dataclasses
import json
import math
import pathlib
import random
from typing import Protocol

from synthetic_generator.annotate import build_joints, build_two_joint_pair
from synthetic_generator.bead import BeadParams, sample_bead
from synthetic_generator.annotation_schema import AnnotationDocument, PartEntry
from synthetic_generator.reinforcement import ReinforcementParams, sample_reinforcement
from synthetic_generator.classify import classify
from synthetic_generator.flange import FlangeParams, chirality_candidates
from synthetic_generator.families import FAMILIES, Knobs, kind_of
from synthetic_generator.rib import RibParams
from synthetic_generator.general_geometry import plan_for
from synthetic_generator.templates.general_two_point import (
    GeneralTwoJointSpec,
    draw_fold_count,
    resolve_reinforcement,
)
from synthetic_generator.templates.general_two_point import sample as sample_general_two_point
from synthetic_generator.templates.parallel_same_offset import TwoJointSpec
from synthetic_generator.templates.parallel_same_offset import sample as sample_two_joint_spec

DEFAULT_OUTPUT_ROOT = pathlib.Path(r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_parts")

# 通常モードでは実用的な頻度で出ないクラス(2026-08-26実測: 通常1500件中
# coplanar_flatは0件、gentleでは2.5%)。クォータ狙い時はgentleモードで引く。
GENTLE_ONLY_CLASSES = frozenset({"coplanar_flat", "parallel_same_offset"})
# クォータ狙い時のgentle slack目標[deg]。既定18だと折れ角20〜30度の帯が薄くなる
# (prod01実測3.5%)ため、少し上げてこの帯も埋める。
mid_fold_target = 26.0


class GeneratedPartLike(Protocol):
    stp_path: str
    catpart_path: str


class PartBuilder(Protocol):
    def build_parallel_same_offset(
        self, spec: TwoJointSpec, reinforcement: ReinforcementParams, out_dir: str, part_name: str
    ) -> GeneratedPartLike: ...


class GeneralPartBuilder(Protocol):
    def build_general_two_point(
        self,
        point1,
        point2,
        *,
        min_bearing_radius_mm: float,
        half_width_mm: float,
        bend_radius_mm: float,
        fold1_slack_mm: float,
        fold2_slack_mm: float,
        fold1_tilt_perturbation_rad: float,
        out_dir: str,
        part_name: str,
        bead: BeadParams | None = None,
        flange: FlangeParams | None = None,
        rib: RibParams | None = None,
    ) -> GeneratedPartLike: ...


@dataclasses.dataclass(frozen=True)
class GeneratedPartRecord:
    part_id: str
    spec: TwoJointSpec
    reinforcement: ReinforcementParams
    stp_path: str
    catpart_path: str


def generate_batch(
    builder: PartBuilder,
    out_dir: pathlib.Path = DEFAULT_OUTPUT_ROOT,
    *,
    count: int,
    seed: int,
    max_attempts_per_part: int = 50,
) -> list[GeneratedPartRecord]:
    """count件の成功パーツを生成する。

    2026-08-07変更: sample()側の事前チェック(bearing radius/幅方向/ジョグ折れ重なり等)
    によるInfeasible(ValueError)は、CATIA自体の不具合ではなく正常なランダムサンプリングの
    一部として頻繁に発生するようになった(§6.15/6.16参照、歩留まり40〜60%程度)。以前の
    「失敗したらアボートでいい」方針はInfeasible率がほぼ0%だった頃のものだったため、
    ValueErrorはスキップして次のシード(rngは引き続き進む)で再試行するよう変更した。
    CATIA自体の失敗(com_error等、ValueError以外の例外)は従来通りバッチ全体を中断する
    (これは実際のバグの兆候であり握りつぶすべきではないため)。1パーツあたり
    max_attempts_per_part回試行しても成功しない場合は異常事態としてRuntimeErrorを送出する。
    """
    rng = random.Random(seed)
    out_dir = pathlib.Path(out_dir)

    doc = AnnotationDocument(assembly_dir=out_dir, full_assembly_stp="synthetic")
    records: list[GeneratedPartRecord] = []

    for i in range(1, count + 1):
        part_id = f"SYN_parallel_same_offset_{i:04d}"

        for attempt in range(max_attempts_per_part):
            spec = sample_two_joint_spec(rng)
            reinforcement = sample_reinforcement(spec.thickness_mm, rng)
            try:
                generated = builder.build_parallel_same_offset(spec, reinforcement, str(out_dir / "mid"), part_id)
                break
            except ValueError:
                continue
        else:
            raise RuntimeError(
                f"{part_id}: {max_attempts_per_part}回連続でInfeasibleだった。"
                "サンプリング範囲自体が破綻している可能性がある。"
            )

        doc.parts[part_id] = PartEntry(
            part_id=part_id,
            stp_file=f"mid/{part_id}_mid.stp",
            vtp_file="",
            tag="sheet_metal",
            thickness_mm=spec.thickness_mm,
            thickness_source="synthetic_generator",
        )
        for joint in build_two_joint_pair(part_id, spec.point1, spec.point2, spec.hole_diameter_mm):
            doc.add_joint(joint)

        records.append(
            GeneratedPartRecord(
                part_id=part_id,
                spec=spec,
                reinforcement=reinforcement,
                stp_path=generated.stp_path,
                catpart_path=generated.catpart_path,
            )
        )

    doc.save()
    return records


def _int_keys(d):
    """JSON 往復で文字列になった頂点番号のキーを int に戻す。"""
    return {int(k): v for k, v in d.items()} if d else None


def build_general_part(builder, spec, bead, flange, out_dir: str, part_name: str, rib=None):
    """1部品をビルドする。フランジの根本フィレットが落ちた場合はキラリティ
    (側x方向)の反転候補で再試行する(SS14.6: 成立性はキラリティ依存で、失敗7件の
    全てが反転で成立した)。戻り値は(GeneratedPart, 実際に使ったflange)。"""

    # 絞りの角 + 曲げタブ(実車057の簡略版)。
    drawn = getattr(spec, "drawn", None)
    if drawn is not None:
        return builder.build_drawn_tray(
            drawn["hub_xy"], drawn["walls"], drawn["arms"], origin=tuple(drawn["origin"]),
            hub_u=tuple(drawn["hub_u"]), hub_v=tuple(drawn["hub_v"]),
            seam_fillet_mm=drawn["seam_fillet_mm"], corner_radius=drawn["corner_radius"],
            out_dir=out_dir, part_name=part_name, check_points=spec.annotated_points), None
    # チャンネル + 座面(実車144)。
    channel = getattr(spec, "channel", None)
    if channel is not None:
        geom = {k: (tuple(v) if isinstance(v, list) else v) for k, v in channel.items()
                if k not in ("diag_deg", "seat_width_mm")}
        return builder.build_channel_seat(out_dir=out_dir, part_name=part_name,
                                          check_points=spec.annotated_points, **geom), None
    # 分岐はハブ + 腕(実車026)。
    branch = getattr(spec, "branch", None)
    if branch is not None:
        return builder.build_branch_part(
            branch["hub_xy"], branch["arms"], origin=tuple(branch["origin"]),
            hub_u=tuple(branch["hub_u"]), hub_v=tuple(branch["hub_v"]),
            corner_radius=branch["corner_radius"], out_dir=out_dir, part_name=part_name,
            check_points=spec.annotated_points, gussets=branch["gussets"],
            fillet_radius=_int_keys(branch.get("fillet_radius"))), None
    # 平板は掃引ではなく外形ワイヤから作る(実車031/1285-20)。
    if getattr(spec, "plate_margin_mm", None) is not None:
        return builder.build_flat_plate(
            spec.annotated_points, margin_mm=spec.plate_margin_mm,
            corner_radius_mm=spec.plate_corner_radius_mm,
            out_dir=out_dir, part_name=part_name), None

    compose = getattr(spec, "compose", None) or {}

    def attempt(candidate_flange):
        return builder.build_general_two_point(
            spec.point1,
            spec.point2,
            min_bearing_radius_mm=spec.min_bearing_radius_mm,
            half_width_mm=spec.half_width_mm,
            bend_radius_mm=spec.bend_radius_mm,
            fold1_slack_mm=spec.fold1_slack_mm,
            fold2_slack_mm=spec.fold2_slack_mm,
            fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
            target_folds=spec.target_folds,
            extra_points=getattr(spec, "extra_points", ()),
            check_points=getattr(spec, "annotated_points", None),
            taper_half_width_mm=getattr(spec, "taper_half_width_mm", None),
            out_dir=out_dir,
            part_name=part_name,
            bead=bead,
            flange=candidate_flange,
            rib=rib,
            # 合成族だけ側辺の腕・切欠き・非対称余白を渡す(他の族のビルダー呼び出しは不変)
            **({"arms": compose.get("arms", ()), "notches": compose.get("notches", ()),
                "side_extension_mm": tuple(compose.get("side_extension_mm", (0.0, 0.0))),
                "bead_span": (tuple(compose["bead_span"]) if compose.get("bead_span") else None),
                "check_radii": tuple(compose.get("point_radii", ()))}
               if compose else {}),
        )

    # 合成族はフランジの側を変えると腕の側と衝突するので、キラリティ再試行をしない。
    if flange is None or compose:
        return attempt(flange), flange
    plan = plan_for(spec)
    last_error: ValueError | None = None
    for candidate in chirality_candidates(flange, plan.panel_frames, spec.bend_radius_mm):
        try:
            return attempt(candidate), candidate
        except ValueError as exc:
            if "flange" not in str(exc):
                raise  # フランジ以外の失敗(基準面など)は反転しても直らない
            last_error = exc
    raise last_error if last_error is not None else ValueError(
        "no feasible flange chirality candidate"
    )


@dataclasses.dataclass(frozen=True)
class GeneratedGeneralPartRecord:
    part_id: str
    spec: GeneralTwoJointSpec
    stp_path: str
    catpart_path: str
    bead: BeadParams | None = None
    flange: FlangeParams | None = None
    rib: RibParams | None = None


def generate_general_batch(
    builder: GeneralPartBuilder,
    out_dir: pathlib.Path = DEFAULT_OUTPUT_ROOT,
    *,
    count: int,
    seed: int,
    max_attempts_per_part: int = 50,
    reinforcement_probability: float = 0.0,
    flange_aim_share: float = 0.4,
    class_quota: dict[str, int] | None = None,
    accept_filter=None,
    on_part_built=None,
) -> list[GeneratedGeneralPartRecord]:
    """任意の法線・任意の位置の締結点ペア(roadmap SS6.20〜6.22)でcount件の成功パーツを
    生成する。`generate_batch`(parallel_same_offsetクラス専用)と同じskip-and-retry
    方針(ValueErrorはスキップして次のシードで再試行、それ以外の例外はバッチ全体を中断)。
    フランジ・Phase 1.5トリムは未対応(`build_general_two_point`自体が未対応、SS6.21参照)。
    """
    rng = random.Random(seed)
    out_dir = pathlib.Path(out_dir)

    doc = AnnotationDocument(assembly_dir=out_dir, full_assembly_stp="synthetic")
    records: list[GeneratedGeneralPartRecord] = []
    # クォータの計上は**実際に生成できたクラス**で行う(狙いが外れても止まらない)
    produced_classes: dict[str, int] = {c: 0 for c in (class_quota or {})}

    for i in range(1, count + 1):
        part_id = f"SYN_general_two_point_{i:04d}"

        for attempt in range(max_attempts_per_part):
            # reinforcement_probability=0(既定)では追加のrngを消費しない — 既存バッチの
            # シード列をそのまま再現できるようにするため。
            reinforce = reinforcement_probability > 0.0 and rng.random() < reinforcement_probability
            # フランジ帯狙い(SS14.5、ユーザー承認②): 既定サンプラーは折れ角>=25度を
            # 狙うため、フランジ対象(<=20度)は3.6%しか出ない。補強部品の一部を
            # 「法線角<=35度+slack目標18度以下」の帯狙いで引く。
            aim_flange = reinforce and rng.random() < flange_aim_share
            target = None
            if class_quota:
                # 不足の一番大きいクラスを狙う。クラスは法線と位置だけで決まるので、
                # サンプラー側で重い探索の前に足切りできる(SS16)。
                deficits = {c: n - produced_classes[c] for c, n in class_quota.items()}
                target = max(deficits, key=deficits.get) if max(deficits.values()) > 0 else None
            if target is not None:
                # 事前測定(2026-08-26)にもとづくモード選択: coplanar_flatと
                # parallel_same_offsetはgentleモードでしか実用的な頻度で出ない
                # (通常モードのcoplanar_flatは1500件中0件)。
                aim_flange = target in GENTLE_ONLY_CLASSES
                spec = sample_general_two_point(
                    rng, gentle_folds=aim_flange, target_classes={target},
                    gentle_target_max_fold_deg=mid_fold_target,
                )
            else:
                # 曲げ本数は設計変数(2026-09-04、D1)。狙いが外れたら実測値をparamsへ。
                spec = sample_general_two_point(
                    rng, gentle_folds=aim_flange, target_folds=draw_fold_count(rng))
            bead: BeadParams | None = None
            flange: FlangeParams | None = None
            rib: RibParams | None = None
            if reinforce:
                # 補強の種類は基準面の幾何で決まる(ユーザー指定、2026-08-25):
                # 最大折れ角20度以下ならフランジ、それ以外(急でフランジ不成立)はビード。
                # CATIAに触る前に純Pythonで種類選定と成立可否を解決する(SS12/SS14)。
                resolved = resolve_reinforcement(rng, spec)
                if resolved is None:
                    continue
                spec, bead, flange, rib = resolved
                # カバレッジ補正用のフィルタ(SS18)。CATIAに触る前の純Python判定なので
                # 棄却は実質無料。クォータを満たす組だけをビルドへ送る。
                if accept_filter is not None and not accept_filter(spec, bead, flange):
                    continue
            try:
                generated, flange = build_general_part(
                    builder, spec, bead, flange, str(out_dir / "mid"), part_id, rib=rib
                )
                break
            except ValueError:
                continue
        else:
            raise RuntimeError(
                f"{part_id}: {max_attempts_per_part}回連続でInfeasibleだった。"
                "サンプリング範囲自体が破綻している可能性がある。"
            )

        # 部品ごとの生成パラメータを保存する(2026-08-25)。joints.jsonは締結点と板厚しか
        # 持たないため、再現やビード有無の条件付き学習に必要な情報(slack・ビード寸法・
        # 折れ目の傾き実績値)がどこにも残らなかった。specはビルダー成功時の値
        # (リゾルバがslackを差し替えた後)なので、これだけで形状を再構築できる。
        plan = plan_for(spec)
        params_dir = out_dir / "params"
        params_dir.mkdir(parents=True, exist_ok=True)
        (params_dir / f"{part_id}.json").write_text(
            json.dumps(
                {
                    "part_id": part_id,
                    "attempts_used": attempt + 1,
                    "geometry_label": plan.geometry_label,
                    # 実際の曲げ本数(狙いが外れても実測値を残す)
                    "folds": len(plan.panel_frames) - 1,
                    "fold_tilts_deg": [
                        [math.degrees(a), math.degrees(b)] for a, b in plan.fold_tilts
                    ],
                    "spec": dataclasses.asdict(spec),
                    "bead": dataclasses.asdict(bead) if bead is not None else None,
                    "flange": dataclasses.asdict(flange) if flange is not None else None,
                    "rib": dataclasses.asdict(rib) if rib is not None else None,
                },
                ensure_ascii=False,
                indent=1,
            ),
            encoding="utf-8",
        )

        doc.parts[part_id] = PartEntry(
            part_id=part_id,
            stp_file=f"mid/{part_id}_mid.stp",
            vtp_file="",
            tag="sheet_metal",
            thickness_mm=spec.thickness_mm,
            thickness_source="synthetic_generator",
        )
        for joint in build_two_joint_pair(part_id, spec.point1, spec.point2, spec.hole_diameter_mm):
            doc.add_joint(joint)

        if class_quota:
            actual = str(classify(spec.point1, spec.point2))
            if actual in produced_classes:
                produced_classes[actual] += 1

        # クォータの計上は**ビルド成功後**に行うこと。採択時に数えると、CATIA側の
        # 失敗(フランジで約1/3)が枠を食い潰し、最後の数十部品で「どのセルも満杯」に
        # なって8000回連続Infeasibleで落ちる(2026-08-26に実測: 100部品の指定に対し
        # 74-80部品で停止した)。
        if on_part_built is not None:
            on_part_built(spec, bead, flange)

        # joints.jsonは**途中でも書く**。末尾でまとめて書くと、バッチが例外で落ちた
        # ときに「部品は残るがアノテーションが無い」状態になる(2026-08-26に315部品で
        # 実際に発生し、tools/rebuild_joints.py で復旧した)。
        if len(records) % 25 == 0:
            doc.save()

        records.append(
            GeneratedGeneralPartRecord(
                part_id=part_id,
                spec=spec,
                stp_path=generated.stp_path,
                catpart_path=generated.catpart_path,
                bead=bead,
                flange=flange,
                rib=rib,
            )
        )

    doc.save()
    return records


def generate_recipe_batch(
    builder,
    out_dir: pathlib.Path,
    recipe: list[dict],
    *,
    seed: int,
    on_part=None,
) -> list[GeneratedGeneralPartRecord]:
    """レシピどおりに部品を作る(2026-09-04、ユーザー提案の族別生成)。

    `recipe` は `[{"kind": "bead", "count": 180, "knobs": {...}}, ...]`。
    **何をどれだけどんな多様性で作るかは、ここではなく呼び出し側が決める。**
    族ごとに独立した生成器(`families.FAMILIES`)が「その特徴が成立する配置を狙って」
    引くので、族どうしが干渉しない(1つの決定論で振り分けていた頃は、ビードの
    置き場所探索を改善するとリブが0%になる、といった往復が起きた)。

    ビルドが落ちた部品はその族の中で引き直す。族ごとの試行回数と成功数は
    `on_part(kind, spec, bead, flange, rib, attempts)` で観測できる。
    """
    rng = random.Random(seed)
    out_dir = pathlib.Path(out_dir)
    doc = AnnotationDocument(assembly_dir=out_dir, full_assembly_stp="synthetic")
    records: list[GeneratedGeneralPartRecord] = []
    index = 0

    # **族を混ぜる。**レシピ順にまとめて作ると part_id が族ごとに連続し、
    # IDで train/val を切ると片方が1族だけになる(2026-09-04にユーザー指摘)。
    order = [entry for entry in recipe for _ in range(int(entry["count"]))]
    rng.shuffle(order)

    for entry in order:
        kind = entry["kind"]
        generator = FAMILIES[kind]
        knobs = Knobs.from_dict(entry.get("knobs", {}))
        made = 0
        while made < 1:
            for attempt in range(entry.get("max_attempts", 400)):
                drawn = generator(rng, knobs)
                if drawn is None:
                    continue
                spec, bead, flange, rib = drawn
                index += 1
                part_id = f"SYN_general_two_point_{index:04d}"
                try:
                    generated, flange = build_general_part(
                        builder, spec, bead, flange, str(out_dir / "mid"), part_id, rib=rib
                    )
                except ValueError:
                    index -= 1
                    continue
                break
            else:
                raise RuntimeError(
                    f"{kind}: {entry.get('max_attempts', 400)}回試しても1件も作れなかった。"
                    "レシピのつまみが実行可能範囲を外している。"
                )
            made += 1

            plate = getattr(spec, "plate_margin_mm", None) is not None
            branch = getattr(spec, "branch", None)
            channel = getattr(spec, "channel", None)
            drawn = getattr(spec, "drawn", None)
            custom = plate or branch is not None or channel is not None or drawn is not None
            plan = None if custom else plan_for(spec)
            params_dir = out_dir / "params"
            params_dir.mkdir(parents=True, exist_ok=True)
            (params_dir / f"{part_id}.json").write_text(
                json.dumps(
                    {
                        "part_id": part_id,
                        "kind": kind,
                        "feature": kind_of(bead, flange, rib),
                        "attempts_used": attempt + 1,
                        "geometry_label": (
                            f"flat plate, {len(spec.annotated_points)} joints" if plate
                            else (f"branch, {len(branch['arms'])} arms, "
                                  f"{len(branch['gussets'])} gussets, "
                                  f"{len(spec.annotated_points)} joints") if branch
                            else (f"channel seat, wall {channel['wall_fold_deg']:.0f} deg, "
                                  f"seat {channel['seat_fold_deg']:.0f} deg, "
                                  f"{len(spec.annotated_points)} joints") if channel
                            else (f"drawn tray, walls {drawn['walls']['A']['fold_deg']:.0f}/"
                                  f"{drawn['walls']['B']['fold_deg']:.0f} deg, "
                                  f"{len(spec.annotated_points)} joints") if drawn
                            else plan.geometry_label),
                        "folds": (0 if plate else len(branch["arms"]) if branch
                                  else 4 if channel else 4 if drawn
                                  else len(plan.panel_frames) - 1),
                        "fold_tilts_deg": [] if custom else [
                            [math.degrees(a), math.degrees(b)] for a, b in plan.fold_tilts
                        ],
                        "spec": dataclasses.asdict(spec),
                        "bead": dataclasses.asdict(bead) if bead is not None else None,
                        "flange": dataclasses.asdict(flange) if flange is not None else None,
                        "rib": dataclasses.asdict(rib) if rib is not None else None,
                    },
                    ensure_ascii=False,
                    indent=1,
                ),
                encoding="utf-8",
            )
            doc.parts[part_id] = PartEntry(
                part_id=part_id,
                stp_file=f"mid/{part_id}_mid.stp",
                vtp_file="",
                tag="sheet_metal",
                thickness_mm=spec.thickness_mm,
                thickness_source="synthetic_generator",
            )
            # 011型は掃引アンカーが締結点ではないので annotated_points が優先する。
            points = (getattr(spec, "annotated_points", None)
                      or (spec.point1, spec.point2, *getattr(spec, "extra_points", ())))
            for joint in build_joints(part_id, points, spec.hole_diameter_mm):
                doc.add_joint(joint)
            if len(records) % 25 == 0:
                doc.save()
            if on_part is not None:
                on_part(kind, spec, bead, flange, rib, attempt + 1)
            records.append(
                GeneratedGeneralPartRecord(
                    part_id=part_id, spec=spec,
                    stp_path=generated.stp_path, catpart_path=generated.catpart_path,
                    bead=bead, flange=flange, rib=rib,
                )
            )

    doc.save()
    return records

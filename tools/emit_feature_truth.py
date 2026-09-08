"""生成器のグラウンドトゥルースを特徴サイドカーとして書き出す(2026-08-30、SS17)。

依頼元(AutoMetalSheet の生成モデル側、docs/requests/2026-08-30_wireframe_extraction.md)は
`fold` 識別子・`angle`・`radius`、および `bead` / `corner_relief` クラスを求めている。
これらは**B-Repから推定するより、作った側が出すほうが正確**である — 生成器は各折れ目の
角度・半径・傾き、ビード/フランジの全寸法、余肉カットのRを厳密に知っている。

各部品の params/*.json から `plan_general_two_point` で構成を再現し(決定的)、
抽出済みワイヤーフレームの辺にラベルを付けられる形の3D幾何を出力する。

出力: <chunk>/features/<part_id>.json
  panels[]        パネルのローカルフレームと範囲(基準面の構造)
  folds[]         折れ目: 角度・半径・傾き・シャープ線・**接線2本(side 0/1)**
                  -> 抽出器の bend_line 対がどの折れ目のどちら側かが確定する
  bead            ビードの全寸法 + 中心線ポリライン + 稜線のオフセット量
                  -> 中心線からの距離で bead 稜線を同定できる
  flange          フランジの全寸法 + 根本曲線
  corner_relief[] 余肉カットの円弧(外形側の特徴)

使い方: python tools/emit_feature_truth.py <chunkディレクトリ...>
"""
from __future__ import annotations

import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import BeadParams, _cross, _normalize  # noqa: E402
from synthetic_generator.classify import FasteningPoint  # noqa: E402
from synthetic_generator.corner_relief import plan_corner_relief  # noqa: E402
from synthetic_generator.flange import FlangeParams, plan_flange_on_surface  # noqa: E402
from synthetic_generator.general_geometry import plan_general_two_point  # noqa: E402
from synthetic_generator.rib import RibParams  # noqa: E402

SCHEMA = "partmaker_features/1"
TILTS: list = []


def _at(frame, run: float, width: float):
    return [frame.origin[i] + run * frame.u[i] + width * frame.v[i] for i in range(3)]


def _flat_polyline(frames, fold_tangents, width_mm: float, samples: int = 7,
                   clip: tuple[float, float] | None = None):
    """各パネルの平坦区間で、中心線から幅width_mmの位置を通るポリライン。

    抽出された辺にラベルを付けるための照合曲線。曲げフィレット上は採らない
    (そこは`bend_line`の領域なので、稜線の照合には不要)。傾いた折れ目では
    幅座標wでの境界runが w*tan(tilt) ずれる(SS8.8)。
    """
    pts = []
    for idx, frame in enumerate(frames):
        near_cut, far_cut = fold_tangents[idx]
        near_tilt, far_tilt = TILTS[idx]
        lo = frame.near_run_mm + width_mm * math.tan(near_tilt) + near_cut + 1.0
        hi = frame.far_run_mm + width_mm * math.tan(far_tilt) - far_cut - 1.0
        if clip:
            lo, hi = max(lo, clip[0]), min(hi, clip[1])
        if hi - lo < 1.0:
            continue
        for k in range(samples):
            pts.append(_at(frame, lo + (hi - lo) * k / (samples - 1), width_mm))
    return pts


def _build_branch(meta: dict, spec: dict) -> dict:
    """分岐部品(実車026)。ハブ + 腕 + ガセットのパラメータをそのまま真値にする。"""
    br = spec["branch"]
    points = spec.get("annotated_points") or []
    return {
        "schema": SCHEMA,
        "part_id": meta["part_id"],
        "geometry_label": meta["geometry_label"],
        "half_width_mm": None,
        "min_bearing_radius_mm": spec["min_bearing_radius_mm"],
        "thickness_mm": spec["thickness_mm"],
        "folds": len(br["arms"]),
        "panels": 1 + len(br["arms"]) + len(br["gussets"]),
        "has_inflection": False,
        "bead": None, "flange": None, "rib": None,
        "corner_relief": None,
        "branch": {
            "hub_xy": br["hub_xy"], "hub_normal_xyz": None,
            "arms": [{"edge": a["edge"], "fold_deg": a["fold_deg"],
                      "bend_radius_mm": a["radius_mm"], "length_mm": a["length_mm"],
                      "side": a.get("side", -1), "outline": a.get("outline")}
                     for a in br["arms"]],
            "gussets": br["gussets"],
            "corner_radius_mm": br["corner_radius"],
            "fillet_radius_mm": br.get("fillet_radius"),
            "hinge_deg": br.get("hinge_deg"),
            "joints": len(points),
            "joint_positions_xyz": [list(p["position_xyz"]) for p in points],
            "joint_normals_xyz": [list(p["normal_xyz"]) for p in points],
        },
        "notes": "branch: planar hub with arms folded about their own root edges; "
                 "gusset = tab folded from one arm lapped onto the neighbour",
    }


def _build_channel(meta: dict, spec: dict) -> dict:
    """チャンネル + 座面(実車144)。ウェブ・壁・座面のパラメータをそのまま真値にする。"""
    ch = spec["channel"]
    points = spec.get("annotated_points") or []
    return {
        "schema": SCHEMA,
        "part_id": meta["part_id"],
        "geometry_label": meta["geometry_label"],
        "half_width_mm": None,
        "min_bearing_radius_mm": spec["min_bearing_radius_mm"],
        "thickness_mm": spec["thickness_mm"],
        "folds": 4,
        "panels": 5,
        "has_inflection": False,
        "bead": None, "flange": None, "rib": None,
        "corner_relief": None,
        "channel": {
            "web_length_mm": ch["length_mm"], "web_width_mm": ch["width_mm"],
            "wall_fold_deg": ch["wall_fold_deg"], "wall_bend_radius_mm": ch["wall_radius_mm"],
            "wall_depth_mm": [ch["wall_depth0_mm"], ch["wall_depth1_mm"]],
            "diag_deg": ch["diag_deg"],
            "seat_fold_deg": ch["seat_fold_deg"], "seat_bend_radius_mm": ch["seat_radius_mm"],
            "seat_width_mm": ch["seat_width_mm"], "seat_depth_mm": ch["seat_depth_mm"],
            "joints": len(points),
            "joint_positions_xyz": [list(p["position_xyz"]) for p in points],
            "joint_normals_xyz": [list(p["normal_xyz"]) for p in points],
        },
        "notes": "channel seat: web hub, two walls folded about the long edges, "
                 "seats folded outward about each wall's diagonal bottom edge",
    }


def _build_drawn(meta: dict, spec: dict) -> dict:
    """絞りの角 + 曲げタブ(実車057の簡略版)。壁・継ぎ目・タブのパラメータをそのまま真値にする。"""
    dr = spec["drawn"]
    points = spec.get("annotated_points") or []
    return {
        "schema": SCHEMA,
        "part_id": meta["part_id"],
        "geometry_label": meta["geometry_label"],
        "half_width_mm": None,
        "min_bearing_radius_mm": spec["min_bearing_radius_mm"],
        "thickness_mm": spec["thickness_mm"],
        "folds": 4,
        "panels": 5,
        "has_inflection": False,
        "bead": None, "flange": None, "rib": None,
        "corner_relief": None,
        "drawn": {
            "hub_xy": dr["hub_xy"],
            "walls": {k: {"fold_deg": w["fold_deg"], "height_mm": w["height_mm"],
                          "fillet_mm": w["fillet_mm"], "tabs": w["tabs"],
                          "taper_from_mm": w.get("taper_from_mm")} for k, w in dr["walls"].items()},
            "seam_fillet_mm": dr["seam_fillet_mm"], "seams": dr.get("seams"),
            "arms": [{"edge": a["edge"], "fold_deg": a["fold_deg"], "bend_radius_mm": a["radius_mm"],
                      "length_mm": a["length_mm"], "root_mm": [a["root_from_mm"], a["root_to_mm"]],
                      "outline": a.get("outline")} for a in dr["arms"]],
            "joints": len(points),
            "joint_positions_xyz": [list(p["position_xyz"]) for p in points],
            "joint_normals_xyz": [list(p["normal_xyz"]) for p in points],
        },
        "notes": "drawn tray: hub + two walls sharing a drawn corner (sharp shell, then OCCT "
                 "fillets on hub-A / hub-B / seam; the vertex blend is the draw), + two bent tabs",
    }


def _build_box(meta: dict, spec: dict) -> dict:
    """多面ブラケット(実車002-024)。ハブ多角形・壁・角の連結・深さ2フランジ・腕を真値にする。"""
    bx = spec["box"]
    points = spec.get("annotated_points") or []
    walls = bx["walls"]
    return {
        "schema": SCHEMA,
        "part_id": meta["part_id"],
        "geometry_label": meta["geometry_label"],
        "half_width_mm": None,
        "min_bearing_radius_mm": spec["min_bearing_radius_mm"],
        "thickness_mm": spec["thickness_mm"],
        # 折り = 壁 + 腕 + 深さ2フランジ + 深さ3パネル + リブ(4本)。
        "folds": (len(walls) + len(bx["arms"]) + len(bx["flanges"])
                  + len(bx.get("deep", ())) + (4 if bx.get("rib") else 0)),
        "panels": (1 + len(walls) + len(bx["arms"]) + len(bx["flanges"])
                   + len(bx.get("deep", ())) + (4 if bx.get("rib") else 0)),
        "has_inflection": False,
        "bead": None, "flange": None, "rib": None,
        "corner_relief": None,
        "box": {
            "hub_xy": bx["hub_xy"],
            "corner_r_mm": bx["corner_r_mm"],
            "closed_corners": bx["closed"],
            "walls": {k: {"fold_deg": w["fold_deg"], "height_mm": w["height_mm"],
                          "tabs": w["tabs"], "taper_from_mm": w.get("taper_from_mm")}
                      for k, w in walls.items()},
            "flanges": [{"wall": f["wall"], "root_mm": [f["from_mm"], f["to_mm"]],
                         "side": f["side"], "fold_deg": f["fold_deg"],
                         "bend_radius_mm": f["radius_mm"], "length_mm": f["length_mm"]}
                        for f in bx["flanges"]],
            "arms": [{"edge": a["edge"], "fold_deg": a["fold_deg"],
                      "bend_radius_mm": a["radius_mm"], "length_mm": a["length_mm"],
                      "root_mm": [a["root_from_mm"], a["root_to_mm"]],
                      "outline": a.get("outline")} for a in bx["arms"]],
            "rib": bx.get("rib"),
            "deep": [{"flange": d["flange"], "root_mm": [d["from_mm"], d["to_mm"]],
                      "side": d["side"], "fold_deg": d["fold_deg"],
                      "bend_radius_mm": d["radius_mm"], "length_mm": d["length_mm"]}
                     for d in bx.get("deep", ())],
            "relief_corners": bx.get("relief_corners"),
            "notch_edges": bx.get("notch_edges"),
            "factors": bx.get("factors"),
            "point_radii": bx.get("point_radii"),
            "point_owners": bx.get("point_owners"),
            "joints": len(points),
            "joint_positions_xyz": [list(p["position_xyz"]) for p in points],
            "joint_normals_xyz": [list(p["normal_xyz"]) for p in points],
        },
        "notes": "box bracket: a polygon hub with walls on any of its edges. Adjacent walls "
                 "either meet at a sewn corner (filleted; the vertex blend is the draw) or "
                 "are separated by a relief notch cut perpendicular to both wall edges "
                 "(pure bending). Plus depth-2 flanges on wall tops, depth-3 panels on "
                 "flange tips, arms on wall-free edges, and a trapezoidal rib across the hub",
    }


def _build_flat_plate(meta: dict, spec: dict) -> dict:
    """平板×多点締結(実車031 / 1285-20)。掃引の計画が無いので真値も別立てで出す。"""
    points = spec.get("annotated_points") or []
    return {
        "schema": SCHEMA,
        "part_id": meta["part_id"],
        "geometry_label": meta["geometry_label"],
        "half_width_mm": None,
        "min_bearing_radius_mm": spec["min_bearing_radius_mm"],
        "thickness_mm": spec["thickness_mm"],
        "folds": 0,
        "panels": 1,
        "has_inflection": False,
        "bead": None, "flange": None, "rib": None,
        "corner_relief": None,
        "plate": {
            "joints": len(points),
            "margin_mm": spec["plate_margin_mm"],
            "corner_radius_mm": spec.get("plate_corner_radius_mm"),
            "normal_xyz": list(points[0]["normal_xyz"]) if points else None,
            "joint_positions_xyz": [list(p["position_xyz"]) for p in points],
        },
        "notes": "flat plate: outline is the fastening hull offset outward by margin",
    }


def build(meta: dict) -> dict:
    spec = meta["spec"]
    if spec.get("plate_margin_mm") is not None:
        return _build_flat_plate(meta, spec)
    if spec.get("branch") is not None:
        return _build_branch(meta, spec)
    if spec.get("channel") is not None:
        return _build_channel(meta, spec)
    if spec.get("drawn") is not None:
        return _build_drawn(meta, spec)
    if spec.get("box") is not None:
        return _build_box(meta, spec)
    truth = _build_sweep(meta, spec)
    if spec.get("compose") is not None:
        cp = spec["compose"]
        truth["compose"] = {
            "factors": cp["factors"], "factors_sampled": cp.get("factors_sampled"),
            "arms": cp["arms"], "notches": cp["notches"],
            "side_extension_mm": cp["side_extension_mm"], "weld_bearing_mm": cp.get("weld_bearing_mm"),
            "joints": len(spec.get("annotated_points") or []),
            "joint_positions_xyz": [list(p["position_xyz"]) for p in (spec.get("annotated_points") or [])],
            "joint_normals_xyz": [list(p["normal_xyz"]) for p in (spec.get("annotated_points") or [])],
        }
    return truth


def _build_sweep(meta: dict, spec: dict) -> dict:
    p1 = FasteningPoint(tuple(spec["point1"]["position_xyz"]), tuple(spec["point1"]["normal_xyz"]))
    p2 = FasteningPoint(tuple(spec["point2"]["position_xyz"]), tuple(spec["point2"]["normal_xyz"]))
    flange = FlangeParams(**meta["flange"]) if meta["flange"] else None
    side_ext = (0.0, 0.0)
    if flange:
        side_ext = (flange.extension_mm if flange.side < 0 else 0.0,
                    flange.extension_mm if flange.side > 0 else 0.0)
    plan = plan_general_two_point(
        p1, p2,
        min_bearing_radius_mm=spec["min_bearing_radius_mm"],
        half_width_mm=spec["half_width_mm"],
        bend_radius_mm=spec["bend_radius_mm"],
        fold1_slack_mm=spec["fold1_slack_mm"],
        fold2_slack_mm=spec["fold2_slack_mm"],
        fold1_tilt_perturbation_rad=spec["fold1_tilt_perturbation_rad"],
        side_extension_mm=side_ext,
        target_folds=spec.get("target_folds"),
    )
    frames = plan.panel_frames
    global TILTS
    TILTS = plan.fold_tilts
    hw = spec["half_width_mm"]
    out: dict = {
        "schema": SCHEMA,
        "part_id": meta["part_id"],
        "geometry_label": meta["geometry_label"],
        "half_width_mm": hw,
        "min_bearing_radius_mm": spec["min_bearing_radius_mm"],
        "thickness_mm": spec["thickness_mm"],
        "panels": [],
        "folds": [],
        "bead": None,
        "flange": None,
        "rib": None,
        "corner_relief": [],
        "notes": [],
    }
    for f in frames:
        out["panels"].append({
            "origin": list(f.origin), "u": list(f.u), "v": list(f.v),
            "normal": list(_normalize(_cross(f.u, f.v))),
            "near_run_mm": f.near_run_mm, "far_run_mm": f.far_run_mm,
        })

    # --- 折れ目: シャープ線 + 接線2本(= 抽出器が出す bend_line の対) ---
    corners = plan.panel_corner_sets
    for i in range(len(frames) - 1):
        a, b = frames[i], frames[i + 1]
        na, nb = _normalize(_cross(a.u, a.v)), _normalize(_cross(b.u, b.v))
        angle = math.degrees(math.acos(max(-1.0, min(1.0, sum(a.u[k] * b.u[k] for k in range(3))))))
        far_cut = plan.fold_tangents[i][1]
        near_cut = plan.fold_tangents[i + 1][0]
        far_tilt = plan.fold_tilts[i][1]
        near_tilt = plan.fold_tilts[i + 1][0]
        # 傾いた折れ目では幅座標wでの境界runが w*tan(tilt) ずれる(SS8.8)
        wneg = -(hw + side_ext[0])
        wpos = hw + side_ext[1]
        radius = spec["bend_radius_mm"]
        out["folds"].append({
            "id": i,
            "angle_deg": angle,
            "radius_mm": radius,
            "tilt_deg": math.degrees(far_tilt),
            "sharp_line": [list(corners[i][2]), list(corners[i][3])],
            "tangent_lines": [
                {"side": 0, "panel": i,
                 "p0": _at(a, a.far_run_mm + wneg * math.tan(far_tilt) - far_cut, wneg),
                 "p1": _at(a, a.far_run_mm + wpos * math.tan(far_tilt) - far_cut, wpos)},
                {"side": 1, "panel": i + 1,
                 "p0": _at(b, b.near_run_mm + wneg * math.tan(near_tilt) + near_cut, wneg),
                 "p1": _at(b, b.near_run_mm + wpos * math.tan(near_tilt) + near_cut, wpos)},
            ],
            # 法線側が凹か(フランジの高さ制約と同じ判定)
            "concave_side": 1 if sum(b.u[k] * na[k] for k in range(3)) > 0 else -1,
        })

    if meta["bead"]:
        bead = BeadParams(**meta["bead"])
        inset = 2.0 * spec["min_bearing_radius_mm"]
        run_clip = (frames[0].near_run_mm + inset, frames[-1].far_run_mm - inset)
        # OCCT版は `plan_bead_on_surface`(CATIAのprobe-and-select用)を使わない。
        # あれは「全パネルに平坦なビード区間があること」を要求するが、断面掃引には
        # 不要な条件で、実際に作れるビードまで落としてしまう(2026-09-04)。
        centreline = _flat_polyline(frames, plan.fold_tangents, 0.0, clip=run_clip)
        out["bead"] = {
            **meta["bead"],
            "half_footprint_mm": bead.half_footprint_mm,
            "wall_run_mm": bead.wall_run_mm,
            "wall_slant_mm": bead.wall_slant_mm,
            "ridge_setback_mm": bead.ridge_setback_mm,
            "inset_mm": inset,
            "centreline": centreline,
            # 中心線からの測地オフセット。抽出辺の距離がこれに一致すれば当該稜線
            # 稜線フィレットの**接線**位置。シャープ角から後退量 R*tan(θ/2) だけずれる:
            # 足元は外へ(基準面上)、頂稜線は内へ(頂面上)。抽出器の bend_line は
            # 平面↔曲面の境界なので、照合すべきはシャープ角ではなくこの接線。
            "foot_ridge_offset_mm": bead.half_footprint_mm + bead.ridge_setback_mm,
            "top_ridge_offset_mm": bead.top_width_mm / 2.0 - bead.ridge_setback_mm,
            # 照合用: 平坦区間での稜線位置(基準面上)。top側は実際には深さぶん
            # 持ち上がるが、法線方向にdepthだけ動かせば得られる(normalはpanels[]にある)
            "foot_ridge_left": _flat_polyline(frames, plan.fold_tangents,
                -(bead.half_footprint_mm + bead.ridge_setback_mm), clip=run_clip),
            "foot_ridge_right": _flat_polyline(frames, plan.fold_tangents,
                bead.half_footprint_mm + bead.ridge_setback_mm, clip=run_clip),
            "top_ridge_left": _flat_polyline(frames, plan.fold_tangents,
                -(bead.top_width_mm / 2.0 - bead.ridge_setback_mm), clip=run_clip),
            "top_ridge_right": _flat_polyline(frames, plan.fold_tangents,
                bead.top_width_mm / 2.0 - bead.ridge_setback_mm, clip=run_clip),
            # 頂稜線は基準面から depth だけ法線方向に持ち上がる。向きは構築時に
            # CATIA側のプローブで決まるため params に残っていない -> 両符号を出す
            "top_ridge_lift_mm": bead.depth_mm,
            "run_out": {"start": centreline[0] if centreline else None,
                        "end": centreline[-1] if centreline else None},
        }
    if flange:
        fplan = plan_flange_on_surface(
            frames, flange, half_width_mm=hw, fold_tangents=plan.fold_tangents)
        out["flange"] = {
            **meta["flange"],
            "extension_mm": flange.extension_mm,
            "edge_offset_mm": fplan.edge_offset_mm,
            "centreline": [list(p) for p in fplan.centreline_points],
            # 根本フィレットの接線は、壁の立ち上がり位置(edge_offset)から
            # 内側へ R*tan(90/2)=R だけ後退する
            "root_tangent_offset_mm": fplan.edge_offset_mm - flange.root_radius_mm,
            "root_curve": _flat_polyline(
                frames, plan.fold_tangents,
                flange.side * (fplan.edge_offset_mm - flange.root_radius_mm)),
            "root_probe": list(fplan.side_probe),
            "wall_top_probe": list(fplan.wall_top_probe),
        }

    # --- リブ(可変半径のコーナーブレンド、2026-09-04 D3再改訂) ---
    if meta.get("rib"):
        rib = RibParams(**meta["rib"])
        index = min(rib.fold_index, max(0, len(out["folds"]) - 1))
        fold = out["folds"][index] if out["folds"] else None
        angle = math.radians(fold["angle_deg"]) if fold else 0.0
        out["rib"] = {
            **meta["rib"],
            # 曲げのコーナーを、幅 |y|<=c の範囲だけ大きな半径で丸めたもの。
            # 半径は中央 bulge_radius_mm から、|y|=c で基準面の曲げRへ線形に落ちる。
            "fold_id": index,
            "sharp_line": fold["sharp_line"] if fold else None,
            "concave_side": fold["concave_side"] if fold else 0,
            "base_bend_radius_mm": spec["bend_radius_mm"],
            # 領域判定用: 曲げ線から走行方向/幅方向にこの距離まで
            "reach_along_mm": rib.reach_mm(angle),
            "reach_across_mm": rib.half_width_mm,
        }

    # --- 余肉カット(外形側の特徴) ---
    try:
        cuts = plan_corner_relief(
            frames, half_width_mm=hw, radius_mm=spec["min_bearing_radius_mm"],
            fold_tangents=plan.fold_tangents,
            exclude_side=flange.side if flange else None)
        for c in cuts:
            out["corner_relief"].append({
                "label": c.label, "radius_mm": spec["min_bearing_radius_mm"],
                "arc": [list(p) for p in c.arc_points],
            })
    except ValueError as exc:
        out["notes"].append(f"corner_relief再現不可: {exc}")

    # 依頼元が求める `inflection` について
    signs = [f["concave_side"] for f in out["folds"]]
    out["has_inflection"] = len(set(signs)) > 1
    if out["has_inflection"]:
        out["notes"].append(
            "折れ目の凹凸が反転する(S字)。ただし基準面は平面+円筒フィレットの区分構成で、"
            "平坦部の曲率は厳密に0なので『曲率の符号が変わる線』は存在しない — "
            "反転は2つの折れ目の間の平坦パネル全体で起きる")
    return out


def main() -> None:
    total = 0
    for arg in sys.argv[1:]:
        chunk = pathlib.Path(arg).resolve()
        dst = chunk / "features"
        dst.mkdir(exist_ok=True)
        n = 0
        for pp in sorted((chunk / "params").glob("*.json")):
            meta = json.loads(pp.read_text(encoding="utf-8"))
            try:
                feat = build(meta)
            except Exception as exc:
                print(f"  {pp.stem}: 再現失敗 {type(exc).__name__}: {str(exc)[:70]}", flush=True)
                continue
            (dst / f"{pp.stem}.json").write_text(
                json.dumps(feat, ensure_ascii=False), encoding="utf-8")
            n += 1
        print(f"{chunk.parent.name}/{chunk.name}: {n}件", flush=True)
        total += n
    print(f"\n合計 {total}件 -> <chunk>/features/")


if __name__ == "__main__":
    main()

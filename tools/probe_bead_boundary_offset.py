"""外形境界→平行曲線でビードの起点閉曲線を作る案を実機で検証する(2026-08-25、ユーザー提案)。

現行は「Pythonで解析的に点を打つ→スプライン→AddNewProjectで基準面へ投影」だが、
フィレットを横切る区間が面から浮いた弦になり、投影が分断してMode=4スイープが
全ドラフト角・全長で落ちる(41件中14件で実測)。

代案: 基準面そのものの境界を接線連続で取り、支持面付きの平行曲線で内側へ寄せる。
曲線は構築上つねに面の上にあるので、投影という経路が丸ごと消える。
決着させたい3点:
  (a) AddNewBoundaryOfSurfaceがJoin済み基準面に効くか(BRep参照の地雷を踏まないか)
  (b) AddNewCurveParがその境界を内側へオフセットして閉曲線を返すか
  (c) その閉曲線からMode=4のドラフトスイープが通るか(内向き・外向きとも)
端の逃げ(走行方向のトリム)はこの3点が通ってから考える。
"""
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "synthetic_generator" / "src"))

from synthetic_generator.bead import sample_bead  # noqa: E402
from synthetic_generator.gsd_build import SyntheticPartBuilder  # noqa: E402
from synthetic_generator.templates.general_two_point import sample as sample_spec  # noqa: E402

ATTEMPTS = 12


def main() -> None:
    builder = SyntheticPartBuilder()
    catia = builder.catia
    catia.DisplayFileAlerts = False
    for i in range(catia.Documents.Count, 0, -1):
        try:
            catia.Documents.Item(i).Close()
        except Exception:
            pass

    state: dict = {}

    def experiment(doc, part, hsf, spa, body, surface, *, bead, panel_frames, half_width_mm,
                   min_bearing_radius_mm, fold_tangents, fold_tilts):
        n = state["attempt"]
        surface_ref = part.CreateReferenceFromObject(surface)

        def length_of(shape):
            return spa.GetMeasurable(part.CreateReferenceFromObject(shape)).Length * 1000.0

        # (a) 境界
        boundary = None
        for name, args in (
            ("AddNewBoundaryOfSurface", (surface_ref,)),
            ("AddNewBoundary", (surface_ref, 1, None, None)),
        ):
            try:
                boundary = getattr(hsf, name)(*args)
                body.AppendHybridShape(boundary)
                part.Update()
                print(f"[{n:02d}] boundary OK via {name}: {length_of(boundary):.1f}mm", flush=True)
                break
            except Exception as exc:
                print(f"[{n:02d}] boundary NG via {name}: {str(exc)[:70]}", flush=True)
                if boundary is not None:
                    builder._delete_feature(doc, part, boundary)
                boundary = None
        if boundary is None:
            raise ValueError("probe stop")
        boundary_ref = part.CreateReferenceFromObject(boundary)

        # (b) 平行曲線で内側へ
        offset = max(1.0, half_width_mm - bead.half_footprint_mm)
        outline = None
        for reverse in (False, True):
            candidate = hsf.AddNewCurvePar(boundary_ref, surface_ref, offset, reverse, True)
            body.AppendHybridShape(candidate)
            try:
                part.Update()
            except Exception as exc:
                print(f"[{n:02d}]   CurvePar rev={reverse} NG: {str(exc)[:60]}", flush=True)
                builder._delete_feature(doc, part, candidate)
                continue
            print(f"[{n:02d}]   CurvePar rev={reverse} OK offset={offset:.1f} "
                  f"len={length_of(candidate):.1f}mm", flush=True)
            outline = candidate
            break
        if outline is None:
            raise ValueError("probe stop")
        outline_ref = part.CreateReferenceFromObject(outline)

        # (c) Mode=4スイープ
        theta = bead.wall_angle_deg
        for length in (1.0, bead.wall_slant_mm):
            row = []
            for angle in (theta, 180.0 - theta, -theta, -(180.0 - theta)):
                sweep = hsf.AddNewSweepLine(outline_ref)
                sweep.Mode = 4
                sweep.FirstGuideSurf = surface_ref
                sweep.SetAngle(1, angle)
                sweep.SetLength(1, length)
                body.AppendHybridShape(sweep)
                try:
                    part.Update()
                    row.append(f"{angle:+.0f}:OK")
                except Exception:
                    row.append(f"{angle:+.0f}:X")
                builder._delete_feature(doc, part, sweep)
            print(f"[{n:02d}]   sweep len={length:5.1f}  " + "  ".join(row), flush=True)
        raise ValueError("probe stop")

    builder._add_bead_to_surface = experiment

    rng = random.Random(20260825)
    for attempt in range(1, ATTEMPTS + 1):
        spec = sample_spec(rng)
        bead = sample_bead(rng, spec.half_width_mm)
        state["attempt"] = attempt
        try:
            builder.build_general_two_point(
                spec.point1, spec.point2,
                min_bearing_radius_mm=spec.min_bearing_radius_mm,
                half_width_mm=spec.half_width_mm,
                bend_radius_mm=spec.bend_radius_mm,
                fold1_slack_mm=spec.fold1_slack_mm,
                fold2_slack_mm=spec.fold2_slack_mm,
                fold1_tilt_perturbation_rad=spec.fold1_tilt_perturbation_rad,
                out_dir=str(pathlib.Path(__file__).resolve().parent / "probe_output"),
                part_name=f"bnd_{attempt:03d}",
                bead=bead,
            )
        except Exception as exc:
            if "probe stop" not in str(exc):
                print(f"[{attempt:02d}] 事前に脱落: {str(exc)[:80]}", flush=True)


if __name__ == "__main__":
    main()

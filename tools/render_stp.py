"""Render batch STEP output to a PNG contact sheet for quick visual review.

Usage:
    python tools/render_stp.py synthetic_parts/general_two_point_batch1
    python tools/render_stp.py <dir-or-stp> -o out.png --cols 5 --limit 12 --elev 25 --azim -60

Joint points from annotations/joints.json (if present next to the mid/ dir) are
overlaid as red dots with their axis direction, so "does the fastening point sit
on the surface" is checkable at a glance.
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import topods

# ponytail: fixed 1mm tessellation, plenty for a thumbnail; expose if a part ever looks faceted
DEFLECTION_MM = 0.4
LIGHT_DIR = np.array([0.4, 0.5, 0.75])  # fixed key light for lambertian shading


def load_triangles(stp_path: Path) -> np.ndarray:
    """Return (n_tri, 3, 3) triangle vertex array for a STEP file."""
    reader = STEPControl_Reader()
    if reader.ReadFile(str(stp_path)) != 1:
        raise ValueError(f"failed to read {stp_path}")
    reader.TransferRoots()
    shape = reader.OneShape()
    BRepMesh_IncrementalMesh(shape, DEFLECTION_MM, False, 0.5, True)

    tris = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = topods.Face(exp.Current())
        loc = TopLoc_Location()
        mesh = BRep_Tool.Triangulation(face, loc)
        if mesh is not None:
            trsf = loc.Transformation()
            nodes = [mesh.Node(i).Transformed(trsf) for i in range(1, mesh.NbNodes() + 1)]
            pts = np.array([[p.X(), p.Y(), p.Z()] for p in nodes])
            for i in range(1, mesh.NbTriangles() + 1):
                a, b, c = mesh.Triangle(i).Get()
                tris.append(pts[[a - 1, b - 1, c - 1]])
        exp.Next()
    if not tris:
        raise ValueError(f"no triangulation in {stp_path}")
    return np.array(tris)


def joints_by_part(batch_dir: Path) -> dict[str, list[dict]]:
    path = batch_dir / "annotations" / "joints.json"
    if not path.exists():
        return {}
    out: dict[str, list[dict]] = {}
    for joint in json.loads(path.read_text())["joints"]:
        for part_id in joint["parts"]:
            out.setdefault(part_id, []).append(joint)
    return out


def draw(ax, tris: np.ndarray, joints: list[dict], elev: float, azim: float) -> None:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    shade = 0.35 + 0.65 * np.abs(n @ (LIGHT_DIR / np.linalg.norm(LIGHT_DIR)))
    base = np.array([0.42, 0.56, 0.75])
    colors = np.clip(shade[:, None] * base, 0, 1)
    # ponytail: edgecolor==facecolor hides the white seams between adjacent facets
    ax.add_collection3d(
        Poly3DCollection(tris, facecolors=colors, edgecolors=colors, linewidths=0.3)
    )
    pts = tris.reshape(-1, 3)
    for joint in joints:
        p = np.array(joint["axis"]["start_xyz"])
        d = np.array(joint["axis"]["direction_xyz"])
        span = float(np.ptp(pts, axis=0).max())
        ax.quiver(*p, *(d * span * 0.15), color="#d94a4a", linewidth=1.2)
        ax.scatter(*p, color="#d94a4a", s=26, depthshade=False)
        pts = np.vstack([pts, p])

    center = (pts.max(axis=0) + pts.min(axis=0)) / 2
    radius = float(np.ptp(pts, axis=0).max()) / 2 * 1.1
    for setlim, c in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center):
        setlim(c - radius, c + radius)
    ax.set_box_aspect((1, 1, 1), zoom=1.45)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("target", type=Path, help="batch dir (with mid/) or a single .stp")
    ap.add_argument("-o", "--out", type=Path, help="output PNG (default: <target>_render.png)")
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--limit", type=int, default=0, help="render only N parts (paging, with --offset)")
    ap.add_argument("--offset", type=int, default=0, help="skip the first N parts")
    ap.add_argument("--elev", type=float, default=22.0)
    ap.add_argument("--azim", type=float, default=-58.0)
    args = ap.parse_args()

    target = args.target
    if target.is_dir():
        batch_dir = target / "mid" if (target / "mid").is_dir() else target
        files = sorted(batch_dir.glob("*.stp"))
        joints = joints_by_part(batch_dir.parent)
        out = args.out or target / f"{target.name}_render.png"
    else:
        files = [target]
        joints = joints_by_part(target.parent.parent)
        out = args.out or target.with_suffix(".png")
    files = files[args.offset :]
    if args.limit:
        files = files[: args.limit]
    if not files:
        raise SystemExit(f"no .stp under {target}")

    cols = min(args.cols, len(files))
    rows = -(-len(files) // cols)
    fig = plt.figure(figsize=(3.2 * cols, 3.2 * rows), dpi=110)
    for i, path in enumerate(files, start=1):
        ax = fig.add_subplot(rows, cols, i, projection="3d")
        part_id = path.stem.removesuffix("_mid")
        draw(ax, load_triangles(path), joints.get(part_id, []), args.elev, args.azim)
        ax.set_title(part_id.replace("SYN_", ""), fontsize=7, pad=0)
        print(f"[{i}/{len(files)}] {part_id}", flush=True)

    fig.subplots_adjust(left=0.01, right=0.99, top=0.97, bottom=0.01, wspace=0.02, hspace=0.05)
    fig.savefig(out, facecolor="white")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

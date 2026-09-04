"""STEPをヘッドレスで4方向レンダリングしてPNGにする(2026-09-04)。

既存の `tools/render_stp.py`(ディレクトリのコンタクトシート + 締結点オーバーレイ)とは別物。
こちらは1部品を4方向から見る/複数部品を1方向で並べる用途。

`tools/step_viewer.py`(Flaskの対話ビューワ)はユーザー確認用。こちらは
エージェント/CIが形を目で確かめるための静止画。OCCTでメッシュ化して
matplotlibのPoly3DCollectionで描く(GUI不要)。

使い方:
  python tools/render_views.py <出力PNG> <STEP...>            # 1部品4方向
  python tools/render_views.py <出力PNG> --grid <STEP...>     # 複数部品を1方向で並べる
"""
from __future__ import annotations

import math
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

from OCC.Core.BRep import BRep_Tool  # noqa: E402
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh  # noqa: E402
from OCC.Core.STEPControl import STEPControl_Reader  # noqa: E402
from OCC.Core.TopAbs import TopAbs_FACE  # noqa: E402
from OCC.Core.TopExp import TopExp_Explorer  # noqa: E402
from OCC.Core.TopLoc import TopLoc_Location  # noqa: E402
from OCC.Core.TopoDS import topods  # noqa: E402

VIEWS = ((28, -60), (28, 30), (80, -90), (5, 0))
AUTO_TILT_DEG = 28.0   # 真正面すぎると立体感が出ないので少し振る


def auto_view(tris):
    """部品が一番よく見える視線(elev, azim)。

    メッシュ点の共分散で一番薄い方向(平板なら法線)を求め、そこから
    AUTO_TILT_DEG だけ振った向きから見る。固定アングルだと平板が真横になって
    線にしか見えないことがある。
    """
    points = tris.reshape(-1, 3)
    centred = points - points.mean(axis=0)
    _values, vectors = np.linalg.eigh(np.cov(centred.T))
    thin, fat = vectors[:, 0], vectors[:, -1]
    tilt = math.radians(AUTO_TILT_DEG)
    d = thin * math.cos(tilt) + fat * math.sin(tilt)
    d = d / np.linalg.norm(d)
    return math.degrees(math.asin(max(-1.0, min(1.0, d[2])))), math.degrees(math.atan2(d[1], d[0]))


def triangles(path: str):
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != 1:
        raise RuntimeError(f"cannot read {path}")
    reader.TransferRoots()
    shape = reader.OneShape()
    BRepMesh_IncrementalMesh(shape, 0.3, False, 0.3, True)
    tris = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = topods.Face(explorer.Current())
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation(face, location)
        if mesh is not None:
            transform = location.Transformation()
            nodes = [mesh.Node(i + 1).Transformed(transform) for i in range(mesh.NbNodes())]
            for i in range(1, mesh.NbTriangles() + 1):
                a, b, c = mesh.Triangle(i).Get()
                tris.append([
                    (nodes[a - 1].X(), nodes[a - 1].Y(), nodes[a - 1].Z()),
                    (nodes[b - 1].X(), nodes[b - 1].Y(), nodes[b - 1].Z()),
                    (nodes[c - 1].X(), nodes[c - 1].Y(), nodes[c - 1].Z()),
                ])
        explorer.Next()
    if not tris:
        raise RuntimeError(f"{path}: no triangulation")
    return np.array(tris)


def draw(ax, tris, elev, azim, title: str) -> None:
    ax.add_collection3d(Poly3DCollection(
        tris, facecolor="#9fb4c7", edgecolor="#20303c", linewidths=0.15, alpha=1.0))
    points = tris.reshape(-1, 3)
    centre = points.mean(axis=0)
    reach = float(np.abs(points - centre).max()) * 1.05
    ax.set_xlim(centre[0] - reach, centre[0] + reach)
    ax.set_ylim(centre[1] - reach, centre[1] + reach)
    ax.set_zlim(centre[2] - reach, centre[2] + reach)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(title, fontsize=7)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass


def main() -> None:
    out = pathlib.Path(sys.argv[1])
    args = sys.argv[2:]
    grid = args and args[0] == "--grid"
    files = args[1:] if grid else args

    if grid:
        columns = min(4, len(files))
        rows = math.ceil(len(files) / columns)
        fig = plt.figure(figsize=(3.2 * columns, 3.0 * rows), dpi=150)
        for index, path in enumerate(files):
            ax = fig.add_subplot(rows, columns, index + 1, projection="3d")
            tris = triangles(path)
            elev, azim = auto_view(tris)
            draw(ax, tris, elev, azim, pathlib.Path(path).stem)
    else:
        tris = triangles(files[0])
        fig = plt.figure(figsize=(11, 10), dpi=150)
        for index, (elev, azim) in enumerate(VIEWS):
            ax = fig.add_subplot(2, 2, index + 1, projection="3d")
            draw(ax, tris, elev, azim, f"elev={elev} azim={azim}")
        fig.suptitle(pathlib.Path(files[0]).stem, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight", facecolor="white")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

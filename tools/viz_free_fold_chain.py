# -*- coding: utf-8 -*-
"""自由折れ目チェーン(classify.py, SS8.8)で生成した3パネル形状を可視化する。

CATIAを使わない純粋幾何の段階での見た目確認用(ユーザーはスマホ環境のため画像で確認)。
締結点ペアをいくつかサンプルし、成立したものをワイヤーフレーム(3パネル+締結点+中心線)
として描画してPNGに保存する。
"""
import math
import random
import sys

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, "synthetic_generator/src")
from synthetic_generator.classify import (
    FasteningPoint,
    free_fold_seed,
    sheared_panel_corners,
    solve_free_fold,
)


def _random_unit_vector(rng: random.Random):
    z = rng.uniform(-1.0, 1.0)
    theta = rng.uniform(0.0, 2 * math.pi)
    r = math.sqrt(max(0.0, 1.0 - z * z))
    return (r * math.cos(theta), r * math.sin(theta), z)


def _rotate(v, axis, angle):
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    dot = sum(v[i] * axis[i] for i in range(3))
    cross = (
        axis[1] * v[2] - axis[2] * v[1],
        axis[2] * v[0] - axis[0] * v[2],
        axis[0] * v[1] - axis[1] * v[0],
    )
    return tuple(v[i] * cos_a + cross[i] * sin_a + axis[i] * dot * (1 - cos_a) for i in range(3))


def generate_one(rng: random.Random, half_width_mm: float = 20.0, max_attempts: int = 200):
    for _ in range(max_attempts):
        n1 = _random_unit_vector(rng)
        n2 = _rotate(n1, _random_unit_vector(rng), rng.uniform(0.3, math.pi - 0.3))
        p1 = (0.0, 0.0, 0.0)
        p2 = tuple(rng.uniform(80.0, 200.0) * c for c in _random_unit_vector(rng))
        point1, point2 = FasteningPoint(p1, n1), FasteningPoint(p2, n2)
        bearing = rng.uniform(12.5, 25.0)
        seed = free_fold_seed(
            point1,
            point2,
            bend_radius_mm=rng.uniform(10.0, 25.0),
            min_bearing_radius_mm=bearing,
            fold1_slack_mm=rng.uniform(0.0, 60.0),
            fold2_slack_mm=rng.uniform(0.0, 60.0),
        )
        if seed is None:
            continue
        target_a1 = seed.a1_rad + math.radians(rng.uniform(-20.0, 20.0))
        chain = solve_free_fold(seed, point1, point2, target_a1_rad=target_a1)
        if chain is None:
            continue
        return point1, point2, chain, bearing
    return None


def panels_of(chain, bearing_mm: float, half_width_mm: float):
    panel1 = sheared_panel_corners(
        chain.panel1.origin, chain.panel1.u, chain.panel1.v,
        -bearing_mm, chain.L1_mm, half_width_mm, near_tilt_rad=0.0, far_tilt_rad=chain.a1_rad,
    )
    panel_mid = sheared_panel_corners(
        chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v,
        0.0, chain.L2_mm, half_width_mm, near_tilt_rad=chain.a1_rad, far_tilt_rad=chain.a2_rad,
    )
    panel3 = sheared_panel_corners(
        chain.panel3.origin, chain.panel3.u, chain.panel3.v,
        0.0, chain.L3_mm + bearing_mm, half_width_mm, near_tilt_rad=chain.a2_rad, far_tilt_rad=0.0,
    )
    return panel1, panel_mid, panel3


def plot_case(ax, point1, point2, chain, bearing_mm, half_width_mm, title):
    panel1, panel_mid, panel3 = panels_of(chain, bearing_mm, half_width_mm)
    colors = ["#4C72B0", "#DD8452", "#55A868"]
    for corners, color in zip((panel1, panel_mid, panel3), colors):
        poly = Poly3DCollection([corners], alpha=0.5, facecolor=color, edgecolor="black", linewidths=1.0)
        ax.add_collection3d(poly)

    centerline = [point1.position_xyz, chain.panel_mid.origin, chain.panel3.origin, point2.position_xyz]
    xs, ys, zs = zip(*centerline)
    ax.plot(xs, ys, zs, color="red", linewidth=2.0, marker="o", markersize=4, label="centreline")

    for p, label in ((point1.position_xyz, "p1"), (point2.position_xyz, "p2")):
        ax.scatter(*p, color="black", s=30)
        ax.text(*p, label, fontsize=9)

    all_pts = panel1 + panel_mid + panel3
    xs_all = [p[0] for p in all_pts]
    ys_all = [p[1] for p in all_pts]
    zs_all = [p[2] for p in all_pts]
    span = max(max(xs_all) - min(xs_all), max(ys_all) - min(ys_all), max(zs_all) - min(zs_all), 1.0)
    cx, cy, cz = sum(xs_all) / len(xs_all), sum(ys_all) / len(ys_all), sum(zs_all) / len(zs_all)
    ax.set_xlim(cx - span / 1.6, cx + span / 1.6)
    ax.set_ylim(cy - span / 1.6, cy + span / 1.6)
    ax.set_zlim(cz - span / 1.6, cz + span / 1.6)
    ax.set_title(title, fontsize=8, pad=2)
    ax.set_box_aspect((1, 1, 1))


def main():
    rng = random.Random(31)
    half_width_mm = 20.0
    fig = plt.figure(figsize=(15, 11))
    n_rows, n_cols = 2, 3
    plotted = 0
    for idx in range(n_rows * n_cols):
        result = generate_one(rng, half_width_mm)
        if result is None:
            continue
        point1, point2, chain, bearing = result
        ax = fig.add_subplot(n_rows, n_cols, idx + 1, projection="3d")
        gamma_note = f"a1={math.degrees(chain.a1_rad):.0f}deg a2={math.degrees(chain.a2_rad):.0f}deg"
        fold_note = f"fold1={math.degrees(chain.fold1_angle_rad):.0f}deg fold2={math.degrees(chain.fold2_angle_rad):.0f}deg"
        plot_case(ax, point1, point2, chain, bearing, half_width_mm, f"{gamma_note}\n{fold_note}")
        plotted += 1
    fig.suptitle(
        "Free-fold chain samples (blue=panel1, orange=mid, green=panel3, red=centreline/bead axis)",
        fontsize=10,
    )
    fig.subplots_adjust(left=0.02, right=0.98, top=0.90, bottom=0.03, wspace=0.25, hspace=0.35)
    out_path = "tools/probe_output/free_fold_chain_samples.png"
    fig.savefig(out_path, dpi=140)
    print(f"saved {out_path} ({plotted} panels plotted)")


if __name__ == "__main__":
    main()

# -*- coding: utf-8 -*-
"""定理の検証(順方向・高速):
   「帯が全ての折れ目に垂直な可展帯」は、折れ目の枚数・方向によらず
    必ず 全折れ目が同一方向 g に平行 かつ g ∥ n_start x n_end かつ dot(p_end - p_start, g) = 0
    を満たす。

証明: 折れ目軸 g1 = n1 x u1 まわりの回転 R1 は g1 を固定するので、
      次のパネルでの折れ目軸 g2 = n_m x u_m = R1(n1 x u1) = R1 g1 = g1。帰納的に全て同一。
      全ての法線は g1 に垂直、全ての中心線方向も g1 に垂直 -> 中心線は g1 に垂直な平面内。
帰結: 締結点ペアの幅方向ズレ dw = dot(p2-p1, n1 x n2) が 0 でない限り、
      gamma=0 (帯が折れ目に垂直) な基準面は**存在しない**。折れ数をいくら増やしても同じ。
"""
import math, random, sys
import numpy as np
sys.path.insert(0, "synthetic_generator/src")

def rot(v, axis, ang):
    return v*math.cos(ang) + np.cross(axis, v)*math.sin(ang) + axis*np.dot(axis, v)*(1-math.cos(ang))
def unit(v): return v/np.linalg.norm(v)

def forward_perpendicular_chain(rng, n_folds):
    """折れ目が全て中心線に垂直な帯をランダムに作り、両端の(位置,法線)を返す。"""
    n = unit(np.array([rng.gauss(0,1) for _ in range(3)]))
    tmp = np.array([1.0,0,0]) if abs(n[0]) < 0.9 else np.array([0,1.0,0])
    u = unit(np.cross(n, tmp))
    P = np.zeros(3); p1, n1 = P.copy(), n.copy()
    axes = []
    for _ in range(n_folds):
        P = P + rng.uniform(20, 100)*u
        g = unit(np.cross(n, u))            # 中心線に垂直な折れ目軸
        axes.append(g.copy())
        phi = rng.uniform(-2.0, 2.0)
        u = unit(rot(u, g, phi)); n = unit(rot(n, g, phi))
    P = P + rng.uniform(20, 100)*u
    return p1, n1, P, n, axes

if __name__ == "__main__":
    rng = random.Random(31415)
    worst_dw = 0.0; worst_parallel = 0.0; worst_wg = 0.0; n = 0
    for n_folds in (1, 2, 3, 5):
        for _ in range(3000):
            p1, n1, p2, n2, axes = forward_perpendicular_chain(rng, n_folds)
            if np.linalg.norm(np.cross(n1, n2)) < 1e-3: continue   # 法線平行は w が定義できない
            n += 1
            w = unit(np.cross(n1, n2))
            worst_dw = max(worst_dw, abs(float(np.dot(p2-p1, w))))
            for g in axes[1:]:
                worst_parallel = max(worst_parallel, float(np.linalg.norm(np.cross(g, axes[0]))))
            worst_wg = max(worst_wg, float(np.linalg.norm(np.cross(w, axes[0]))))
    print(f"検証数 {n} (折れ目1/2/3/5枚)")
    print(f"  全折れ目が同一方向か  |g_k x g_1|             最大 {worst_parallel:.3e}")
    print(f"  その方向が w=n1xn2 か |w x g_1|               最大 {worst_wg:.3e}")
    print(f"  両端の幅方向ズレ      |dot(p2-p1, w)|         最大 {worst_dw:.3e} mm")
    assert worst_parallel < 1e-9 and worst_wg < 1e-9 and worst_dw < 1e-9
    print("\n定理成立: 帯が折れ目に垂直な基準面は dw=0 のときしか存在しない。")
    print("         => 折れ目方向を自由にしても gamma=0 にはできない(折れ数を増やしても不可)。")

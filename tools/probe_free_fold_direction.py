# -*- coding: utf-8 -*-
"""折れ目方向を自由にした帯(ユーザー選択 2026-08-24)の成立性を検証する。

検証したい2つの主張:
  X. 「帯を全ての折れ目に垂直(gamma=0)にできる」  <- §8.6で未検証の見込みとして書いたもの
  Y. 「折れ目方向を自由にすると自由度が増え、形状多様性が上がる」

パラメータ化(尖り角。フィレットは接線を動かさないので閉合式は同じ):
  p1 で法線 n1、中心線方向 u1 = u1(psi)。
  L1 進んで A。A を通る折れ目軸 g1 = cos(a1)*v1 + sin(a1)*u1  (v1 = n1 x u1、a1=0 が中心線に垂直)。
  g1 まわりに phi1 回転 -> 中間パネル。L2 進んで B。同様に g2, phi2 で回転。L3 進んで終点 E。
拘束: E == p2 (3本)、n3 == ±n2 (2本)。 合計5本。
未知数: psi, L1, a1, phi1, L2, a2, phi2, L3 の8個 -> 3パラメータ族(のはず)。
"""
import math, random, sys
import numpy as np

sys.path.insert(0, "synthetic_generator/src")
from synthetic_generator.classify import FasteningPoint, two_point_frame
from synthetic_generator.templates.general_two_point import _random_unit_vector, _rotate_about_axis

def rot(v, axis, ang):
    v = np.asarray(v, float); axis = np.asarray(axis, float)
    return v*math.cos(ang) + np.cross(axis, v)*math.sin(ang) + axis*np.dot(axis, v)*(1-math.cos(ang))

def unit(v):
    v = np.asarray(v, float); return v/np.linalg.norm(v)

def chain(p1, n1, e_a, e_b, psi, L1, a1, phi1, L2, a2, phi2, L3):
    """折れ目2枚の帯を辿り、終点Eと終端法線n3、各折れ目での中心線の交差角を返す。"""
    u = unit(math.cos(psi)*e_a + math.sin(psi)*e_b)
    n = np.asarray(n1, float)
    P = np.asarray(p1, float)
    cross_angles = []
    for (L, a, phi) in ((L1, a1, phi1), (L2, a2, phi2)):
        P = P + L*u
        v = np.cross(n, u)                       # 中心線に垂直な、パネル内の方向
        g = unit(math.cos(a)*v + math.sin(a)*u)  # 折れ目軸
        # 中心線と折れ目の交差角(a=0 なら垂直=0deg)
        cross_angles.append(abs(math.degrees(a)))
        u = unit(rot(u, g, phi)); n = unit(rot(n, g, phi))
    P = P + L3*u
    return P, n, cross_angles

def residual(x, p1, n1, p2, n2, e_a, e_b, fixed):
    psi, a1, a2 = fixed
    L1, phi1, L2, phi2, L3 = x
    E, n3, _ = chain(p1, n1, e_a, e_b, psi, L1, a1, phi1, L2, a2, phi2, L3)
    b1 = unit(np.cross(n2, e_a if abs(np.dot(n2, e_a)) < 0.9 else e_b))
    b2 = unit(np.cross(n2, b1))
    return np.array([*(E - p2), np.dot(n3, b1), np.dot(n3, b2)])

def solve(p1, n1, p2, n2, e_a, e_b, fixed, x0, iters=80):
    x = np.array(x0, float)
    for _ in range(iters):
        r = residual(x, p1, n1, p2, n2, e_a, e_b, fixed)
        if np.linalg.norm(r) < 1e-9:
            return x, np.linalg.norm(r)
        J = np.zeros((5, 5))
        for k in range(5):
            h = 1e-6*max(1.0, abs(x[k]))
            xp = x.copy(); xp[k] += h
            J[:, k] = (residual(xp, p1, n1, p2, n2, e_a, e_b, fixed) - r)/h
        try:
            step = np.linalg.solve(J, -r)
        except np.linalg.LinAlgError:
            step = -np.linalg.pinv(J) @ r
        nrm = np.linalg.norm(step)
        if nrm > 60.0: step *= 60.0/nrm     # ダンピング
        x = x + step
    return x, np.linalg.norm(residual(x, p1, n1, p2, n2, e_a, e_b, fixed))

def make_case(rng):
    n1 = np.array(_random_unit_vector(rng))
    n2 = np.array(_rotate_about_axis(tuple(n1), _random_unit_vector(rng), rng.uniform(0.3, math.pi-0.3)))
    p1 = np.zeros(3)
    p2 = np.array(_random_unit_vector(rng))*rng.uniform(80.0, 200.0)
    e_a = unit(np.cross(n1, [1,0,0]) if abs(n1[0]) < 0.9 else np.cross(n1, [0,1,0]))
    e_b = np.cross(n1, e_a)
    frame = two_point_frame(FasteningPoint(tuple(p1), tuple(n1)), FasteningPoint(tuple(p2), tuple(n2)))
    dw = abs(float(np.dot(p2-p1, frame.w)))
    return p1, n1, p2, n2, e_a, e_b, dw

def test_X(n=400):
    """主張X: 全ての折れ目に垂直(a1=a2=0)な帯は作れるか。"""
    rng = random.Random(11)
    solved_dw, unsolved_dw, ok = [], [], 0
    for _ in range(n):
        p1,n1,p2,n2,e_a,e_b,dw = make_case(rng)
        best = 1e18
        for trial in range(6):   # 初期値を変えて複数回試す
            x0 = [rng.uniform(20,80), rng.uniform(-2,2), rng.uniform(20,120), rng.uniform(-2,2), rng.uniform(20,80)]
            _, r = solve(p1,n1,p2,n2,e_a,e_b, (rng.uniform(0,2*math.pi), 0.0, 0.0), x0)
            best = min(best, r)
        if best < 1e-6: ok += 1; solved_dw.append(dw)
        else: unsolved_dw.append(dw)
    print(f"[主張X] 帯を全折れ目に垂直(gamma=0)にする解が見つかった: {ok}/{n} = {100*ok/n:.1f}%")
    if solved_dw:
        print(f"         解けたケースの |dw| : 中央値 {sorted(solved_dw)[len(solved_dw)//2]:.2f}mm  最大 {max(solved_dw):.2f}mm")
    if unsolved_dw:
        print(f"         解けなかったケースの |dw|: 中央値 {sorted(unsolved_dw)[len(unsolved_dw)//2]:.2f}mm  最小 {min(unsolved_dw):.2f}mm")

def test_Y(n=400):
    """主張Y: 折れ目方向 a1,a2 を自由に振ったとき、閉合解が存在するか(=3パラメータ族か)。"""
    rng = random.Random(22)
    ok = 0; angs = []; lens = []
    for _ in range(n):
        p1,n1,p2,n2,e_a,e_b,dw = make_case(rng)
        psi = rng.uniform(0, 2*math.pi)
        a1 = math.radians(rng.uniform(-40, 40)); a2 = math.radians(rng.uniform(-40, 40))
        best = (1e18, None)
        for trial in range(8):
            x0 = [rng.uniform(20,80), rng.uniform(-2.5,2.5), rng.uniform(20,120), rng.uniform(-2.5,2.5), rng.uniform(20,80)]
            x, r = solve(p1,n1,p2,n2,e_a,e_b,(psi,a1,a2), x0)
            if r < best[0]: best = (r, x)
        if best[0] < 1e-6 and all(v > 0 for v in (best[1][0], best[1][2], best[1][4])):
            ok += 1
            angs += [abs(math.degrees(a1)), abs(math.degrees(a2))]
            lens.append(best[1][0]+best[1][2]+best[1][4])
    print(f"[主張Y] 折れ目方向を任意に指定しても閉合解が存在し全区間が正: {ok}/{n} = {100*ok/n:.1f}%")
    if lens:
        s = sorted(lens); print(f"         そのときの帯全長: 中央値 {s[len(s)//2]:.1f}mm  p90 {s[int(.9*len(s))]:.1f}mm")

if __name__ == "__main__":
    test_X(60)
    test_Y(120)

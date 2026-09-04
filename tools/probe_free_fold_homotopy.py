# -*- coding: utf-8 -*-
"""折れ目方向自由版を「w平行解からのホモトピー」で解く(ユーザー確定方針 2026-08-24)。

素朴に (psi, a1, a2) をランダムに振って L を解くと成立5%・全長中央値487mmで実用外だった。
理由: psi(帯がp1を出る方向)を勝手に決めると、閉合のしわ寄せが全部 L に行くため。

正しい変数の取り方(ユーザー①「中間折れ位置は bearing クリアランス+フィレット寸法から決める」):
  固定する: L1, L3 (= bearing + 接線長 + slack)、および a1 (折れ目1の傾き = ランダム性の源)
  解く    : psi, phi1, L2, a2, phi2   (5未知数 = 5拘束)
初期値には w平行解(閉形式)を使う。w平行解は自由度族の一員なので必ず解になっている。
そこから a1 を摂動して連続的に追いかける = ホモトピー継続。
"""
import math, random, sys
import numpy as np
sys.path.insert(0, "synthetic_generator/src")
from synthetic_generator.classify import (
    FasteningPoint, two_point_frame, tangent_length_for_bend_angle_rad,
)
from synthetic_generator.templates.general_two_point import _random_unit_vector, _rotate_about_axis

MAX_FOLD_DEG = 135.0
MAX_CROSS_DEG = 60.0   # 折れ目が帯の軸となす角の上限(90degで縮退)
BR_RANGE = (12.5, 25.0)
R_RANGE = (10.0, 25.0)
SLACK_MAX = 80.0

def rot(v, axis, ang):
    return v*math.cos(ang) + np.cross(axis, v)*math.sin(ang) + axis*np.dot(axis, v)*(1-math.cos(ang))
def unit(v): return np.asarray(v, float)/np.linalg.norm(v)

# ---------------------------------------------------------------- 自由折れ目版の順方向チェーン
def chain(p1, n1, e_a, e_b, x_all):
    psi, L1, a1, phi1, L2, a2, phi2, L3 = x_all
    u = unit(math.cos(psi)*e_a + math.sin(psi)*e_b)
    n = np.asarray(n1, float); P = np.asarray(p1, float)
    folds = []
    for (L, a, phi) in ((L1, a1, phi1), (L2, a2, phi2)):
        P = P + L*u
        g = unit(math.cos(a)*np.cross(n, u) + math.sin(a)*u)
        folds.append((P.copy(), g.copy(), phi))
        u = unit(rot(u, g, phi)); n = unit(rot(n, g, phi))
    return P + L3*u, n, folds

def residual(x5, p1, n1, p2, n2, e_a, e_b, L1, a1, L3, basis2):
    psi, phi1, L2, a2, phi2 = x5
    E, n3, _ = chain(p1, n1, e_a, e_b, (psi, L1, a1, phi1, L2, a2, phi2, L3))
    return np.array([*(E - p2), np.dot(n3, basis2[0]), np.dot(n3, basis2[1])])

def solve(x0, args, iters=30):
    x = np.array(x0, float)
    for _ in range(iters):
        r = residual(x, *args)
        if np.linalg.norm(r) < 1e-9: return x, 0.0
        J = np.empty((5,5))
        for k in range(5):
            h = 1e-7*max(1.0, abs(x[k])); xp = x.copy(); xp[k] += h
            J[:,k] = (residual(xp, *args) - r)/h
        try: step = np.linalg.solve(J, -r)
        except np.linalg.LinAlgError: step = -np.linalg.pinv(J) @ r
        nr = np.linalg.norm(step)
        if nr > 30.0: step *= 30.0/nr
        x = x + step
    return x, float(np.linalg.norm(residual(x, *args)))

# ---------------------------------------------------------------- w平行解(閉形式)
def w_parallel_solution(p1, n1, p2, n2, R, br1, br2, s1, s2):
    """§8.2/8.3 のw平行版。成立すれば自由折れ目版の8変数に変換して返す。"""
    fr = two_point_frame(FasteningPoint(tuple(p1), tuple(n1)), FasteningPoint(tuple(p2), tuple(n2)))
    u1, u2, w = np.array(fr.u1), np.array(fr.u2), np.array(fr.w)
    # 断面(w直交面)へ射影してからプロファイルを作る(3Dのまま B-A を取ると w 成分が乗る)
    cs = lambda p: np.asarray(p, float) - w*float(np.dot(p, w))
    q1, q2 = cs(p1), cs(p2)
    T1 = T2 = R
    for _ in range(10):
        a, b = br1 + T1 + s1, br2 + T2 + s2
        A, B = q1 + a*u1, q2 - b*u2
        ab = B - A; Lr = np.linalg.norm(ab)
        if Lr < 1e-6: return None
        d = ab/Lr
        th1 = math.acos(max(-1, min(1, float(np.dot(u1, d)))))
        th2 = math.acos(max(-1, min(1, float(np.dot(d, u2)))))
        nT1, nT2 = tangent_length_for_bend_angle_rad(th1, R), tangent_length_for_bend_angle_rad(th2, R)
        if abs(nT1-T1) < 1e-6 and abs(nT2-T2) < 1e-6: T1, T2 = nT1, nT2; break
        T1, T2 = nT1, nT2
    if max(th1, th2) > math.radians(MAX_FOLD_DEG) or Lr < T1+T2: return None
    S = a + Lr + b
    dw = float(np.dot(p2-p1, w))
    gamma = math.atan2(dw, S)
    cg, sg = math.cos(gamma), math.sin(gamma)

    # 帯の中心線方向を各パネル上で求める(展開面上の一定方向 (cos g, sin g) を3Dへ)。
    # 断面方向は u1 -> ramp_dir -> u2、w方向成分は共通。
    u1s = unit(cg*u1 + sg*w)
    u_m = unit(cg*d  + sg*w)
    u3s = unit(cg*u2 + sg*w)

    def signed_angle_about(v_from, v_to, axis):
        return math.atan2(float(np.dot(np.cross(v_from, v_to), axis)), float(np.dot(v_from, v_to)))

    # 各折れ目は w まわりの回転。法線はその回転で運ばれる(剛体回転)。
    phi1_w = signed_angle_about(u1, d,  w)
    phi2_w = signed_angle_about(d,  u2, w)
    n_m = unit(rot(np.asarray(n1, float), w, phi1_w))
    n_3 = unit(rot(n_m, w, phi2_w))

    # (alpha, phi) パラメータ化へ変換。
    # 注意: 回転角は「回転軸に垂直な成分」で測らないといけない。u1s は w 成分 sin(gamma) を
    # 持つので signed_angle_about(u1s, u_m, w) は誤り。w に垂直な u1->d で測った phi*_w が正しい。
    # 折れ目軸 g は ±w のどちらでもよい(g=-w なら phi の符号が反転する)。|alpha| が小さい側を選ぶ。
    def to_alpha_phi(u_in, n_in, u_out, n_out, phi_w):
        best = None
        for sgn in (1.0, -1.0):
            g = sgn*w
            phi = sgn*phi_w
            if np.linalg.norm(rot(n_in, g, phi) - n_out) > 1e-7: continue
            if np.linalg.norm(rot(u_in, g, phi) - u_out) > 1e-7: continue
            v_in = np.cross(n_in, u_in)
            alpha = math.atan2(float(np.dot(g, u_in)), float(np.dot(g, v_in)))
            if best is None or abs(alpha) < abs(best[0]): best = (alpha, phi)
        return best
    c1 = to_alpha_phi(u1s, np.asarray(n1, float), u_m, n_m, phi1_w)
    c2 = to_alpha_phi(u_m, n_m, u3s, n_3, phi2_w)
    if c1 is None or c2 is None: return None
    a1, phi1 = c1
    a2, phi2 = c2
    return dict(u1s=u1s, L1=a/cg, L2=Lr/cg, L3=b/cg, a1=a1, a2=a2, phi1=phi1, phi2=phi2,
                gamma=math.degrees(abs(gamma)), th=(math.degrees(th1), math.degrees(th2)),
                T=(T1, T2), br=(br1, br2), S=S, D=math.hypot(S, dw), n3=n_3)

def run(n=400, perturb_deg=20.0):
    rng = random.Random(9001)
    base_ok = conv_ok = 0; tried = 0
    res_base = []; d_alpha = []; lens = []; angs = []; cross = []
    while tried < n:
        n1 = np.array(_random_unit_vector(rng))
        n2 = np.array(_rotate_about_axis(tuple(n1), _random_unit_vector(rng), rng.uniform(0.3, math.pi-0.3)))
        p1 = np.zeros(3); p2 = np.array(_random_unit_vector(rng))*rng.uniform(80.0, 200.0)
        R = rng.uniform(*R_RANGE); br1 = rng.uniform(*BR_RANGE); br2 = rng.uniform(*BR_RANGE)
        sol = None
        for _ in range(20):   # slack の実行可能領域からサンプリング
            sol = w_parallel_solution(p1, n1, p2, n2, R, br1, br2,
                                      rng.uniform(0, SLACK_MAX), rng.uniform(0, SLACK_MAX))
            if sol: break
        if not sol: continue
        tried += 1; base_ok += 1
        e_a = unit(np.cross(n1, [1,0,0]) if abs(n1[0]) < 0.9 else np.cross(n1, [0,1,0]))
        e_b = np.cross(n1, e_a)
        psi0 = math.atan2(float(np.dot(sol["u1s"], e_b)), float(np.dot(sol["u1s"], e_a)))
        nn = sol["n3"]
        b1 = unit(np.cross(nn, e_a if abs(np.dot(nn, e_a)) < 0.9 else e_b)); b2 = unit(np.cross(nn, b1))
        # w平行解が本当に自由折れ目版の解になっているか(変換の正当性チェック)
        x0 = np.array([psi0, sol["phi1"], sol["L2"], sol["a2"], sol["phi2"]])
        r0 = np.linalg.norm(residual(x0, p1, n1, p2, n2, e_a, e_b, sol["L1"], sol["a1"], sol["L3"], (b1, b2)))
        res_base.append(r0)
        # a1 を摂動してホモトピー継続(3段階)
        target = sol["a1"] + math.radians(rng.uniform(-perturb_deg, perturb_deg))
        x = x0.copy(); ok = True
        for k in (1/3, 2/3, 1.0):
            a1k = sol["a1"] + k*(target - sol["a1"])
            x, err = solve(x, (p1, n1, p2, n2, e_a, e_b, sol["L1"], a1k, sol["L3"], (b1, b2)))
            if err > 1e-7: ok = False; break
        if not ok: continue
        psi, phi1, L2, a2, phi2 = x
        # 折れ目軸 g と -g は同一直線なので、交差角は (-90, 90] に畳む。
        # ソルバは a2 を 2pi の整数倍だけ流すことがある(実測 max 3171deg)ので必須。
        a2w = math.atan2(math.sin(a2), math.cos(a2))
        if a2w > math.pi/2: a2w -= math.pi
        elif a2w <= -math.pi/2: a2w += math.pi
        if L2 < sol["T"][0] + sol["T"][1]: continue
        if max(abs(phi1), abs(phi2)) > math.radians(MAX_FOLD_DEG): continue
        # 折れ目が帯の軸に近づきすぎると、その折れ目は帯を横切れずパネルが縮退する。
        # 帯半幅 hw に対し折れ目が s 方向に食う長さは hw*tan|alpha| なので上限を置く。
        if max(abs(target), abs(a2w)) > math.radians(MAX_CROSS_DEG): continue
        conv_ok += 1
        d_alpha.append(abs(math.degrees(target - sol["a1"])))
        lens.append(sol["L1"] + L2 + sol["L3"])
        angs += [abs(math.degrees(phi1)), abs(math.degrees(phi2))]
        cross += [abs(math.degrees(target)), abs(math.degrees(a2w))]
    st = lambda v: (f"med={sorted(v)[len(v)//2]:.1f} p90={sorted(v)[int(.9*len(v))]:.1f} max={max(v):.1f}" if v else "n=0")
    print(f"w平行解の残差(変換の正当性)  最大 {max(res_base):.3e}   <- 0ならw平行解は自由族の一員")
    print(f"a1を±{perturb_deg:.0f}deg摂動しての収束: {conv_ok}/{tried} = {100*conv_ok/tried:.1f}%")
    print(f"  実際に動かせた折れ目1の傾き量[deg] {st(d_alpha)}")
    print(f"  帯全長[mm]        {st(lens)}")
    print(f"  曲げ角[deg]       {st(angs)}")
    print(f"  折れ目の交差角[deg] {st(cross)}")

if __name__ == "__main__":
    for pd in (20.0, 35.0):
        print(f"\n===== 摂動幅 +-{pd:.0f}deg =====")
        run(120, pd)

# -*- coding: utf-8 -*-
"""展開写像 phi(s,t) の実機検証。

主張: 折れ目を全て w=n1xn2 に平行に保つと、基準面は「断面プロファイル x w」の可展面になり、
      phi(s,t) = profile3d(s) + t*w  が等長写像(展開)になる。
      よって展開面上で直線に引いた一定幅の帯は、3D上でも幅が一定 = カニ歩きが起きない。

ここで検証する(assertで落ちる):
  1. phi(s1,t1) == p1、phi(s2,t2) == p2  … 帯の両端がちゃんと締結点に着地するか(閉合)
  2. 各パネルの法線が n1 / n2 に一致するか … 座面が締結点法線を満たすか
  3. フィレット部の弧長を使うと展開長が正しいか … 尖り角の 2T ではなく R*theta
  4. 帯の幅が3D上でも一定か … カニ歩きの解消
"""
import math, random, sys
sys.path.insert(0, "synthetic_generator/src")
from synthetic_generator.classify import (
    FasteningPoint, two_point_frame, tangent_length_for_bend_angle_rad,
)
from synthetic_generator.templates.general_two_point import _random_unit_vector, _rotate_about_axis

def dot(a,b): return sum(x*y for x,y in zip(a,b))
def sub(a,b): return tuple(a[i]-b[i] for i in range(3))
def add(a,b): return tuple(a[i]+b[i] for i in range(3))
def mul(v,k): return tuple(c*k for c in v)
def norm(v): return math.sqrt(dot(v,v))
def unit(v): 
    l = norm(v); return tuple(c/l for c in v)
def cross(a,b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def build_profile(p1, p2, u1, u2, w, a, b, R, back1, fwd2):
    """断面プロファイル(フィレット入り)を、弧長パラメータ付きの区間リストとして返す。

    区間は ('line', 始点, 方向, 長さ) または ('arc', 中心, 半径, 始半径ベクトル, 回転軸, 角度)。
    弧長 s は「c(p1) の位置を s=0」として測る。
    """
    c = lambda p: sub(p, mul(w, dot(p, w)))     # w成分を落とす = 断面への射影
    q1, q2 = c(p1), c(p2)
    A = add(q1, mul(u1, a))
    B = sub(q2, mul(u2, b))
    ramp = unit(sub(B, A)); ramp_len = norm(sub(B, A))
    th1 = math.acos(max(-1,min(1,dot(u1, ramp))))
    th2 = math.acos(max(-1,min(1,dot(ramp, u2))))
    T1 = tangent_length_for_bend_angle_rad(th1, R)
    T2 = tangent_length_for_bend_angle_rad(th2, R)

    segs = []
    s = -back1
    # パネル1: q1-back1*u1 から A-T1*u1 まで
    start = sub(q1, mul(u1, back1))
    L = back1 + a - T1
    segs.append(("line", start, u1, s, L)); s += L
    # フィレット1
    for (corner, din, dout, th, T) in ((A, u1, ramp, th1, T1), (B, ramp, u2, th2, T2)):
        if th > 1e-9:
            axis = unit(cross(din, dout))                 # +w か -w のどちらか
            tang_in = sub(corner, mul(din, T))            # 接点
            center = add(tang_in, mul(unit(cross(axis, din)), R))
            r0 = sub(tang_in, center)
            segs.append(("arc", center, R, r0, axis, s, th)); s += R*th
        if corner is A:
            L = ramp_len - T1 - T2
            segs.append(("line", add(A, mul(ramp, T1)), ramp, s, L)); s += L
    L = b - T2 + fwd2
    segs.append(("line", add(B, mul(u2, T2)), u2, s, L)); s += L
    return segs, s - (-back1), (th1, th2, T1, T2, ramp_len)

def profile_point(segs, s):
    """弧長 s の断面プロファイル上の点(3D、w成分は c(p1) と同じ)。"""
    for seg in segs:
        if seg[0] == "line":
            _, start, d, s0, L = seg
            if s0 - 1e-9 <= s <= s0 + L + 1e-9:
                return add(start, mul(d, s - s0))
        else:
            _, center, R, r0, axis, s0, th = seg
            if s0 - 1e-9 <= s <= s0 + R*th + 1e-9:
                ang = (s - s0)/R
                ca, sa = math.cos(ang), math.sin(ang)
                rot = add(add(mul(r0, ca), mul(cross(axis, r0), sa)),
                          mul(axis, dot(axis, r0)*(1-ca)))
                return add(center, rot)
    raise ValueError(f"s={s} out of range")

def check_one(rng, verbose=False):
    n1 = _random_unit_vector(rng)
    n2 = _rotate_about_axis(n1, _random_unit_vector(rng), rng.uniform(0.3, math.pi-0.3))
    p1 = (0.0,0.0,0.0); p2 = mul(_random_unit_vector(rng), rng.uniform(80.0, 200.0))
    f1, f2 = FasteningPoint(p1,n1), FasteningPoint(p2,n2)
    frame = two_point_frame(f1, f2)
    u1,u2,w = frame.u1, frame.u2, frame.w
    R = rng.uniform(10.0, 25.0); br = rng.uniform(12.5, 25.0)
    a = br + R + rng.uniform(0, 60); b = br + R + rng.uniform(0, 60)
    segs, total, (th1,th2,T1,T2,ramp_len) = build_profile(p1,p2,u1,u2,w,a,b,R,br,br)
    if ramp_len < T1+T2 or max(th1,th2) > math.radians(120): return None

    t1, t2 = dot(p1,w), dot(p2,w)
    phi = lambda s,t: add(profile_point(segs, s), mul(w, t))

    # 1. 閉合: phi(0,t1)==p1, phi(s2,t2)==p2
    s2 = total - br - br                       # c(p1)->c(p2) のフィレット込み弧長
    e1 = norm(sub(phi(0.0, t1), p1))
    e2 = norm(sub(phi(s2, t2), p2))
    # 2. 座面の法線: パネル1・2の面法線
    d = 1e-3
    nrm1 = unit(cross(sub(phi(-br+d,t1), phi(-br,t1)), mul(w,1.0)))
    nrm2 = unit(cross(sub(phi(s2+d,t2), phi(s2,t2)), mul(w,1.0)))
    a1 = min(abs(1-abs(dot(nrm1,n1))), 1.0); a2 = min(abs(1-abs(dot(nrm2,n2))), 1.0)
    # 3. 等長性そのものを計量テンソルで直接確認する。
    #    phi が等長 <=> |d(phi)/ds|=1, |d(phi)/dt|=1, 内積=0 (第一基本形式が単位行列)。
    #    これが成り立てば、展開面(s,t)に描いた図形は基準面上で「板を曲げただけ」の合同像になる
    #    = 一定幅の帯は展開幅が厳密に一定 = カニ歩きは起きない。
    #    (折れ目をまたぐ2点の"直線弦"が縮むのは板金として正常であり、欠陥ではない。)
    h = 1e-6
    metric_err = 0.0
    for k in range(1, 20):
        sc = -br + (total - 1e-6) * k/20.0
        tc = t1 + (t2-t1) * k/20.0
        ds_v = mul(sub(phi(sc+h, tc), phi(sc-h, tc)), 1/(2*h))
        dt_v = mul(sub(phi(sc, tc+h), phi(sc, tc-h)), 1/(2*h))
        metric_err = max(metric_err, abs(norm(ds_v)-1), abs(norm(dt_v)-1), abs(dot(ds_v, dt_v)))
    # 4. 折れ目をまたがない区間では、直線弦もそのまま一定幅になることを確認
    hw = 20.0
    dsv, dtv = s2, t2 - t1
    L = math.hypot(dsv, dtv); ns, nt = -dtv/L, dsv/L
    flat_w = 0.0
    for k in range(1, 10):
        sc, tc = dsv*k/10.0, t1 + dtv*k/10.0
        lo, hi = sc - hw*abs(ns), sc + hw*abs(ns)
        # パネル1の平坦区間(-br .. a-T1)に完全に収まる幅線だけを見る
        if not (-br <= lo and hi <= a - T1): continue
        d3 = norm(sub(phi(sc+hw*ns, tc+hw*nt), phi(sc-hw*ns, tc-hw*nt)))
        flat_w = max(flat_w, abs(d3 - 2*hw))
    return e1, e2, a1, a2, metric_err, flat_w

if __name__ == "__main__":
    rng = random.Random(4242)
    worst = [0.0]*6; n = 0
    while n < 3000:
        r = check_one(rng)
        if r is None: continue
        n += 1
        for i in range(6): worst[i] = max(worst[i], r[i])
    print(f"検証数 {n}")
    print(f"  1. 閉合誤差  |phi(0,t1)-p1|      最大 {worst[0]:.3e} mm")
    print(f"     閉合誤差  |phi(s2,t2)-p2|     最大 {worst[1]:.3e} mm")
    print(f"  2. 座面法線ずれ |1-|n.n1||       最大 {worst[2]:.3e}")
    print(f"     座面法線ずれ |1-|n.n2||       最大 {worst[3]:.3e}")
    print(f"  3. 第一基本形式の単位行列からのずれ 最大 {worst[4]:.3e}")
    print(f"  4. 平坦区間での帯幅誤差(公称40mm)   最大 {worst[5]:.3e} mm")
    assert worst[0] < 1e-6 and worst[1] < 1e-6, "閉合しない"
    assert worst[2] < 1e-6 and worst[3] < 1e-6, "座面法線が締結点法線と一致しない"
    assert worst[4] < 1e-5, "phiが等長写像になっていない"
    assert worst[5] < 1e-6, "平坦区間ですら帯幅が一定でない"
    print("OK: 展開写像は等長、帯幅は3Dで厳密に一定 -> カニ歩きは原理的に発生しない")

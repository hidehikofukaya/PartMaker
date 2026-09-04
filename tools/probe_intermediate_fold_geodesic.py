# -*- coding: utf-8 -*-
"""中間折れ+測地線バンド方式(ユーザー指示 2026-08-24 ①②)の純粋幾何検証。

検証したい主張:
  A. 「折れ目を全てw=n1xn2に平行に保ったまま、バンド(帯)を"展開面上の直線"として引く」と
     カニ歩きは原理的に消える。帯の傾き角 gamma = atan(dw / 断面プロファイル長)。
  B. 単曲げ(交線L)が遠すぎるケースは、中間折れ(=既存のランプ)を1枚挟めば
     プロファイル長を自由に選べるので、常に短く作れる。
  C. 中間折れ位置は「bearing半径 + フィレット接線長 + ランダムslack」で決まる。
     したがって基準面フィレットRを先にサンプリングする必要がある。
  D. 締結点間には最低限の距離が要る。効くのは3D距離ではなく**断面内(w直交面内)の距離**。
"""
import math, random, statistics, sys, collections

sys.path.insert(0, "synthetic_generator/src")
from synthetic_generator.classify import (
    MIN_BASE_BEND_RADIUS_MM, FasteningPoint, classify, two_point_frame,
    tangent_length_for_bend_angle_rad,
)
from synthetic_generator.templates.general_two_point import _random_unit_vector, _rotate_about_axis

BR_RANGE = (12.5, 25.0)          # bearing radius (最小必要平面の半径)
R_RANGE = (10.0, 25.0)           # 基準面フィレットR(先にサンプリング)
SLACK_RANGE = (0.0, 30.0)        # 中間折れ位置のランダムslack
MAX_FOLD_DEG = 120.0             # 1つの折れ目の曲げ角上限
MAX_TOTAL_LEN = 300.0            # 部品全長(展開)の上限
OFFSET_RANGE = (60.0, 200.0)

def dot(a, b): return sum(x*y for x, y in zip(a, b))
def sub(a, b): return tuple(a[i]-b[i] for i in range(3))
def add(a, b): return tuple(a[i]+b[i] for i in range(3))
def mul(v, k): return tuple(c*k for c in v)
def norm(v): return math.sqrt(dot(v, v))

def sample_pair(rng, min_cross_sep=0.0):
    """締結点ペアを1組。min_cross_sep>0なら断面内距離での棄却を行う。"""
    for _ in range(200):
        n1 = _random_unit_vector(rng)
        n2 = _rotate_about_axis(n1, _random_unit_vector(rng), rng.uniform(0, math.pi))
        p1 = (0.0, 0.0, 0.0)
        d = rng.uniform(*OFFSET_RANGE)
        p2 = mul(_random_unit_vector(rng), d)
        f1, f2 = FasteningPoint(p1, n1), FasteningPoint(p2, n2)
        try:
            frame = two_point_frame(f1, f2)
        except ValueError:
            continue
        delta = sub(p2, p1)
        dw = dot(delta, frame.w)
        cross_sep = math.sqrt(max(0.0, dot(delta, delta) - dw*dw))
        if cross_sep >= min_cross_sep:
            return f1, f2, frame, dw, cross_sep
    return None

def two_fold_layout(f1, f2, frame, rng, R, br1, br2):
    """中間折れ(w平行)を1枚挟む断面プロファイル q1 -> A -> B -> q2 を作る。
    a,b は bearing + 接線長 + slack から決める(接線長は角度に依存するので不動点反復)。
    戻り: dict または None(不成立)。"""
    u1, u2, w = frame.u1, frame.u2, frame.w
    p1, p2 = f1.position_xyz, f2.position_xyz
    s1, s2 = rng.uniform(*SLACK_RANGE), rng.uniform(*SLACK_RANGE)
    T1 = T2 = R  # 初期推定: 曲げ角90度 -> 接線長=R
    for _ in range(6):
        a = br1 + T1 + s1
        b = br2 + T2 + s2
        A = add(p1, mul(u1, a))
        B = sub(p2, mul(u2, b))
        ab = sub(B, A)
        ab_len = norm(ab)
        if ab_len < 1e-6:
            return None
        ab_dir = mul(ab, 1.0/ab_len)
        th1 = math.acos(max(-1.0, min(1.0, dot(u1, ab_dir))))
        th2 = math.acos(max(-1.0, min(1.0, dot(ab_dir, u2))))
        nT1 = tangent_length_for_bend_angle_rad(th1, R)
        nT2 = tangent_length_for_bend_angle_rad(th2, R)
        if abs(nT1-T1) < 1e-4 and abs(nT2-T2) < 1e-4:
            T1, T2 = nT1, nT2
            break
        T1, T2 = nT1, nT2
    if max(th1, th2) > math.radians(MAX_FOLD_DEG):
        return None, "fold_angle"
    if ab_len < T1 + T2:
        return None, "ramp_overlap"
    S = a + ab_len + b                       # 断面プロファイル全長
    dw = dot(sub(p2, p1), w)
    D = math.hypot(S, dw)                    # 展開後のバンド中心線長
    gamma = math.degrees(math.atan2(abs(dw), S))
    if D > MAX_TOTAL_LEN:
        return None, "too_long"
    return dict(a=a, b=b, ramp=ab_len, S=S, D=D, gamma=gamma,
                th1=math.degrees(th1), th2=math.degrees(th2), R=R), None

def single_fold(f1, f2, frame, R, br1, br2):
    """交線Lでの単曲げ。成立しなければ理由を返す。"""
    n1, n2, u1, u2, w = frame.n1, frame.n2, frame.u1, frame.u2, frame.w
    if abs(abs(dot(n1, n2)) - 1.0) < 1e-6:
        return None, "parallel"
    den = dot(u1, n2)
    if abs(den) < 1e-9:
        return None, "degenerate"
    delta = sub(f2.position_xyz, f1.position_xyz)
    d1 = dot(delta, n2) / den
    fold_pt = add(f1.position_xyz, mul(u1, d1))
    d2 = dot(sub(f2.position_xyz, fold_pt), u2)
    if d1 <= 0 or d2 <= 0:
        return None, "behind"
    th = math.acos(max(-1.0, min(1.0, dot(u1, u2))))
    T = tangent_length_for_bend_angle_rad(th, R)
    if d1 - T < br1 or d2 - T < br2:
        return None, "bearing"
    S = d1 + d2
    dw = dot(delta, w)
    D = math.hypot(S, dw)
    return dict(d1=d1, d2=d2, S=S, D=D, gamma=math.degrees(math.atan2(abs(dw), S)),
                th=math.degrees(th)), None

def pct(x, n): return f"{100.0*x/n:5.1f}%"
def stats(vals):
    if not vals: return "  (n=0)"
    v = sorted(vals)
    q = lambda p: v[min(len(v)-1, int(p*len(v)))]
    return f"p10={q(.1):7.1f} med={q(.5):7.1f} p90={q(.9):7.1f} max={v[-1]:7.1f}"

def run(n, min_cross_sep, label):
    rng = random.Random(20260824 + int(min_cross_sep))
    ok1 = ok2 = 0
    reasons1 = collections.Counter(); reasons2 = collections.Counter()
    gam1, gam2, D1, D2, S1, S2, cross, dws = [], [], [], [], [], [], [], []
    fold_angles = []
    cls = collections.Counter()
    total = 0
    while total < n:
        s = sample_pair(rng, min_cross_sep)
        if s is None: continue
        total += 1
        f1, f2, frame, dw, cs = s
        cross.append(cs); dws.append(abs(dw))
        R = rng.uniform(*R_RANGE)
        br1 = rng.uniform(*BR_RANGE); br2 = rng.uniform(*BR_RANGE)
        r1 = single_fold(f1, f2, frame, R, br1, br2)
        if r1[0]:
            ok1 += 1; gam1.append(r1[0]["gamma"]); D1.append(r1[0]["D"]); S1.append(r1[0]["S"])
        else:
            reasons1[r1[1]] += 1
        r2 = two_fold_layout(f1, f2, frame, rng, R, br1, br2)
        if isinstance(r2, tuple) and r2[0]:
            ok2 += 1; gam2.append(r2[0]["gamma"]); D2.append(r2[0]["D"]); S2.append(r2[0]["S"])
            fold_angles += [r2[0]["th1"], r2[0]["th2"]]
            cls[classify(f1, f2)] += 1
        else:
            reasons2[r2[1] if isinstance(r2, tuple) else "none"] += 1
    print(f"\n===== {label}  (n={n}, 断面内最小距離={min_cross_sep}mm) =====")
    print(f"  断面内距離   {stats(cross)}")
    print(f"  幅方向ズレ|dw| {stats(dws)}")
    print(f"  [単曲げ(交線L)] 成立 {pct(ok1,n)}   不成立理由 {dict(reasons1)}")
    if D1: print(f"      全長D {stats(D1)}   帯の傾きgamma[deg] {stats(gam1)}")
    print(f"  [中間折れ1枚]  成立 {pct(ok2,n)}   不成立理由 {dict(reasons2)}")
    if D2:
        print(f"      全長D {stats(D2)}   帯の傾きgamma[deg] {stats(gam2)}")
        print(f"      断面プロファイル長S {stats(S2)}   曲げ角[deg] {stats(fold_angles)}")
        print(f"      成立後クラス分布 { {k: pct(v, ok2) for k, v in cls.most_common()} }")

if __name__ == "__main__":
    run(4000, 0.0, "現状の分布(断面内距離の下限なし)")
    for sep in (40.0, 60.0, 80.0):
        run(4000, sep, "断面内距離に下限を入れる")

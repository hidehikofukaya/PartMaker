# -*- coding: utf-8 -*-
"""中間折れ+測地線バンド v2: 中間折れ位置を「実行可能範囲を先に求めてから」サンプリングする。

v1 は a=br+T+slack (slack~U(0,30)) と決め打ちしたため曲げ角超過で39%落ちた。
本プロジェクトの既定方針(parallel_same_offset.py/general_two_point.py と同じ)に合わせ、
(a,b) の実行可能領域をグリッドで求め、その広さ(=形状多様性の余地)も併せて測る。
"""
import math, random, statistics, sys, collections
sys.path.insert(0, "synthetic_generator/src")
from synthetic_generator.classify import (
    FasteningPoint, classify, two_point_frame, tangent_length_for_bend_angle_rad,
    MIN_PANEL_CLEARANCE_MM, _segment_distance_2d,
)
from synthetic_generator.templates.general_two_point import _random_unit_vector, _rotate_about_axis

BR_RANGE = (12.5, 25.0)
R_RANGE = (10.0, 25.0)
MAX_FOLD_DEG = 135.0
MAX_TOTAL_LEN = 300.0
SINGLE_FOLD_MAX_LEN = 250.0     # これ以下なら単曲げを使う。超えたら中間折れを入れる
OFFSET_RANGE = (60.0, 200.0)
SLACK_MAX = 80.0                # 中間折れ位置の探索上限(bearing+接線長からの追加距離)
GRID = 12

def dot(a,b): return sum(x*y for x,y in zip(a,b))
def sub(a,b): return tuple(a[i]-b[i] for i in range(3))
def add(a,b): return tuple(a[i]+b[i] for i in range(3))
def mul(v,k): return tuple(c*k for c in v)
def norm(v): return math.sqrt(dot(v,v))

def two_fold_eval(f1, f2, frame, R, br1, br2, s1, s2):
    """slack (s1,s2) を与えて中間折れレイアウトを評価。成立なら dict、不成立なら理由。"""
    u1,u2,w = frame.u1, frame.u2, frame.w
    p1,p2 = f1.position_xyz, f2.position_xyz
    # 断面(w直交面)へ射影してからプロファイルを作る。p1,p2 は w 座標が dw だけ違うので、
    # 3Dのまま B-A を取ると w 成分が乗り、ランプ面が w に平行でなくなる(2026-08-24の実測バグ)。
    c = lambda p: sub(p, mul(w, dot(p, w)))
    q1, q2 = c(p1), c(p2)
    T1 = T2 = R
    for _ in range(8):
        a, b = br1+T1+s1, br2+T2+s2
        A, B = add(q1, mul(u1,a)), sub(q2, mul(u2,b))
        ab = sub(B,A); L = norm(ab)
        if L < 1e-6: return None, "degenerate"
        d = mul(ab, 1.0/L)
        th1 = math.acos(max(-1,min(1,dot(u1,d))))
        th2 = math.acos(max(-1,min(1,dot(d,u2))))
        n1t = tangent_length_for_bend_angle_rad(th1,R); n2t = tangent_length_for_bend_angle_rad(th2,R)
        if abs(n1t-T1)<1e-4 and abs(n2t-T2)<1e-4: T1,T2 = n1t,n2t; break
        T1,T2 = n1t,n2t
    if max(th1,th2) > math.radians(MAX_FOLD_DEG): return None, "fold_angle"
    if L < T1+T2: return None, "ramp_overlap"
    # 断面(w直交面)でのプロファイル自己干渉: パネル1の帯 vs パネル2の帯
    def proj(p):
        r = sub(p, p1); return (dot(r, u1), dot(r, frame.n1))
    seg1 = (proj(sub(p1, mul(u1, br1))), proj(A))
    seg2 = (proj(B), proj(add(p2, mul(u2, br2))))
    if _segment_distance_2d(*seg1, *seg2) < MIN_PANEL_CLEARANCE_MM: return None, "interference"
    S = a + L + b
    dw = dot(sub(p2,p1), w)
    D = math.hypot(S, dw)
    if D > MAX_TOTAL_LEN: return None, "too_long"
    return dict(a=a,b=b,ramp=L,S=S,D=D,gamma=math.degrees(math.atan2(abs(dw),S)),
                th1=math.degrees(th1), th2=math.degrees(th2)), None

def single_fold(f1,f2,frame,R,br1,br2):
    n1,n2,u1,u2,w = frame.n1,frame.n2,frame.u1,frame.u2,frame.w
    if abs(abs(dot(n1,n2))-1.0) < 1e-6: return None,"parallel"
    den = dot(u1,n2)
    if abs(den) < 1e-9: return None,"degenerate"
    delta = sub(f2.position_xyz, f1.position_xyz)
    d1 = dot(delta,n2)/den
    fp = add(f1.position_xyz, mul(u1,d1))
    d2 = dot(sub(f2.position_xyz,fp), u2)
    if d1<=0 or d2<=0: return None,"behind"
    th = math.acos(max(-1,min(1,dot(u1,u2))))
    T = tangent_length_for_bend_angle_rad(th,R)
    if d1-T<br1 or d2-T<br2: return None,"bearing"
    S = d1+d2; dw = dot(delta,w); D = math.hypot(S,dw)
    return dict(S=S,D=D,gamma=math.degrees(math.atan2(abs(dw),S)),th=math.degrees(th)),None

def stats(v):
    if not v: return "(n=0)"
    v=sorted(v); q=lambda p: v[min(len(v)-1,int(p*len(v)))]
    return f"p10={q(.1):7.1f} med={q(.5):7.1f} p90={q(.9):7.1f} max={v[-1]:7.1f}"
def pct(x,n): return f"{100.0*x/n:5.1f}%"

def run(n, min_cross_sep):
    rng = random.Random(777 + int(min_cross_sep))
    tot=0; use_single=0; use_two=0; fail=0
    reasons=collections.Counter(); feas_frac=[]; Ds=[]; gams=[]; angs=[]; cls=collections.Counter()
    while tot < n:
        n1 = _random_unit_vector(rng)
        n2 = _rotate_about_axis(n1, _random_unit_vector(rng), rng.uniform(0,math.pi))
        p1=(0.0,0.0,0.0); p2 = mul(_random_unit_vector(rng), rng.uniform(*OFFSET_RANGE))
        f1,f2 = FasteningPoint(p1,n1), FasteningPoint(p2,n2)
        try: frame = two_point_frame(f1,f2)
        except ValueError: continue
        delta = sub(p2,p1); dw = dot(delta, frame.w)
        if math.sqrt(max(0.0, dot(delta,delta)-dw*dw)) < min_cross_sep: continue
        tot += 1
        R = rng.uniform(*R_RANGE); br1=rng.uniform(*BR_RANGE); br2=rng.uniform(*BR_RANGE)
        sf,_r = single_fold(f1,f2,frame,R,br1,br2)
        if sf and sf["D"] <= SINGLE_FOLD_MAX_LEN:
            use_single += 1; Ds.append(sf["D"]); gams.append(sf["gamma"]); angs.append(sf["th"])
            cls[classify(f1,f2)] += 1
            continue
        # 中間折れ: (a,b)の実行可能領域をグリッドで探す
        feas=[]; last=None
        for i in range(GRID):
            for j in range(GRID):
                s1 = SLACK_MAX*i/(GRID-1); s2 = SLACK_MAX*j/(GRID-1)
                r,why = two_fold_eval(f1,f2,frame,R,br1,br2,s1,s2)
                if r: feas.append(r)
                else: last = why
        if feas:
            use_two += 1; feas_frac.append(100.0*len(feas)/(GRID*GRID))
            pick = rng.choice(feas)
            Ds.append(pick["D"]); gams.append(pick["gamma"]); angs += [pick["th1"],pick["th2"]]
            cls[classify(f1,f2)] += 1
        else:
            fail += 1; reasons[last] += 1
    print(f"\n===== 断面内最小距離={min_cross_sep}mm  (n={n}) =====")
    print(f"  単曲げ(交線L)で足りた      {pct(use_single,n)}")
    print(f"  中間折れ1枚を入れて成立    {pct(use_two,n)}")
    print(f"  総合成立率                 {pct(use_single+use_two,n)}   不成立 {pct(fail,n)} {dict(reasons)}")
    print(f"  部品全長D(展開)  {stats(Ds)}")
    print(f"  帯の傾きgamma[deg] {stats(gams)}")
    print(f"  曲げ角[deg]       {stats(angs)}")
    if feas_frac: print(f"  中間折れの実行可能領域の広さ[%of grid] {stats(feas_frac)}  <- 形状多様性の余地")
    print(f"  クラス分布 { {k: pct(v, use_single+use_two) for k,v in cls.most_common()} }")

if __name__ == "__main__":
    for sep in (0.0, 40.0, 60.0, 80.0):
        run(2000, sep)

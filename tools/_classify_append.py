

# ---------------------------------------------------------------- 自由折れ目チェーン(2026-08-24)
#
# docs/catia_bead_fillet_investigation_log.md SS8参照。two_point_frameのw平行制約
# (全ての折れ目がn1×n2に平行)を緩め、中間折れ目1本を「任意方向に傾けられる」ようにした
# 一般化。p1-panel1-fold1-panel_mid-fold2-panel3-p2の3パネル構成で、各折れ目の傾き
# (a1/a2、パネル固有の幅方向vに対する折れ目軸の角度)にランダム性を持たせることで
# 締結点ペアごとに複数の形状(3パラメータ族: a1, a2, および展開全長に対応する自由度)が
# 作れる(ユーザー指示、2026-08-24)。
#
# SS8.7で証明済み: 中心線が全折れ目に垂直(gamma=0)な構成は、締結点の幅方向ズレ
# dw=dot(p2-p1, n1xn2)が非ゼロである限り存在しない(折れ目の枚数・方向によらず)。
# したがって帯が折れ目を斜めに横切ること自体は避けられない設計上の性質であり、
# 折れ目方向を自由にする利点はgammaの削減ではなく形状多様性の拡大にある。


def _rotate_about_axis(v: Vec3, axis: Vec3, angle_rad: float) -> Vec3:
    """ロドリゲスの回転公式でv(任意ベクトル)をaxis(単位ベクトル)まわりにangle_rad回転する。

    templates/general_two_point.pyの乱数生成用と用途は異なるが式は同じなので、
    自由折れ目チェーンの構築(法線・走行方向を折れ目軸まわりに回す)にはこちらを使う。
    """
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    dot = _dot(v, axis)
    cross = _cross(axis, v)
    return tuple(v[i] * cos_a + cross[i] * sin_a + axis[i] * dot * (1 - cos_a) for i in range(3))


def _angle_between(a: Vec3, b: Vec3) -> float:
    return math.acos(max(-1.0, min(1.0, _dot(a, b))))


def _solve_linear(matrix: list[list[float]], rhs: list[float]) -> list[float] | None:
    """n元連立1次方程式を部分ピボット付きガウス消去法で解く(自由折れ目チェーンの
    ニュートン法で使う数値ヤコビアンの求解専用。5x5程度の小規模行列のみを想定)。
    特異に近い場合はNoneを返す(呼び出し側でダンピング等の対処に回す)。
    """
    n = len(rhs)
    aug = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot_row = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot_row][col]) < 1e-14:
            return None
        aug[col], aug[pivot_row] = aug[pivot_row], aug[col]
        pivot = aug[col][col]
        aug[col] = [x / pivot for x in aug[col]]
        for r in range(n):
            if r == col:
                continue
            factor = aug[r][col]
            if factor != 0.0:
                aug[r] = [aug[r][k] - factor * aug[col][k] for k in range(n + 1)]
    return [aug[i][n] for i in range(n)]


@dataclasses.dataclass(frozen=True)
class ChainPanelFrame:
    """自由折れ目チェーンの1パネル分のローカルフレーム。originはこのパネルの走行座標
    run=0の点(パネル1ならp1、パネル中間・パネル3なら手前側の折れ目点)。
    """

    origin: Vec3
    u: Vec3  # 走行方向(単位ベクトル)
    n: Vec3  # 面法線(単位ベクトル)

    @property
    def v(self) -> Vec3:
        """幅方向(単位ベクトル) = n x u。(u,v,n)はこの順で右手系の正規直交基底になる
        (u x v = n)。折れ目軸g(a) = cos(a)*v + sin(a)*u の基準。"""
        return _cross(self.n, self.u)


@dataclasses.dataclass(frozen=True)
class FreeFoldChain:
    """p1-panel1-fold1-panel_mid-fold2-panel3-p2 の3パネル構成。

    fold1/fold2の位置はpanel_mid.origin/panel3.originそのもの(=中心線が折れ目を
    横切る点)。中心線(ビード中心線にもなる、SS8.6)は p1 -> panel_mid.origin ->
    panel3.origin -> p2 の3線分ポリラインで、chain構築時の閉合条件により
    panel3.origin + L3_mm*panel3.u は厳密に point2.position_xyz に一致する
    (`solve_free_fold`のニュートン法収束条件そのもの)。
    """

    panel1: ChainPanelFrame
    panel_mid: ChainPanelFrame
    panel3: ChainPanelFrame
    L1_mm: float  # panel1: p1からfold1までの走行距離
    L2_mm: float  # panel_mid: fold1からfold2までの走行距離
    L3_mm: float  # panel3: fold2からp2までの走行距離
    a1_rad: float  # fold1の傾き(panel1.vに対するfold1の折れ目軸の角度。0ならv平行=旧w平行版)
    a2_rad: float  # fold2の傾き(panel_mid.vに対する角度)

    @property
    def fold1_angle_rad(self) -> float:
        """fold1の二面角(=外向きの曲がり角、tangent_length_for_bend_angle_radにそのまま渡せる)。
        回転軸(折れ目軸)は常にpanel1.n・panel_mid.nの両方に直交するため、この角度は
        a1_radの値によらずpanel1.nとpanel_mid.nのなす角に厳密に一致する(SS8.8で証明・検証済み:
        折れ目軸まわりの回転はどんなa1でも法線をその回転角だけ運ぶ)。"""
        return _angle_between(self.panel1.n, self.panel_mid.n)

    @property
    def fold2_angle_rad(self) -> float:
        return _angle_between(self.panel_mid.n, self.panel3.n)

    @property
    def end_point(self) -> Vec3:
        return tuple(self.panel3.origin[i] + self.L3_mm * self.panel3.u[i] for i in range(3))


def sheared_panel_corners(
    origin: Vec3,
    u: Vec3,
    v: Vec3,
    near_run_mm: float,
    far_run_mm: float,
    half_width_mm: float,
    *,
    near_tilt_rad: float = 0.0,
    far_tilt_rad: float = 0.0,
) -> list[Vec3]:
    """自由折れ目パネルの4隅を返す(end_panel_cornersの一般化)。

    境界(near/far)がそれぞれ折れ目軸の傾き(near_tilt_rad/far_tilt_rad)を持つ場合、
    幅方向座標widthでの境界上の点は run = boundary_run + width*tan(tilt) になる
    (SS8.8で導出・検証済み: 隣接パネルの折れ目軸gが自パネルの幅方向vとなす角は
    両パネルで厳密に同じ値になるため、隣接パネル同士は同じtilt値で計算すれば
    境界の4隅の座標が寸分違わず一致する)。tilt=0ならend_panel_cornersと同じ矩形になる
    (境界の自由端— 締結点まわりのbearing margin — は常にtilt=0)。

    順序は[near,-hw],[near,hw],[far,hw],[far,-hw](end_panel_cornersと同じ規約)。
    """

    def pt(run: float, width: float, tilt: float) -> Vec3:
        shifted_run = run + width * math.tan(tilt)
        return tuple(origin[i] + shifted_run * u[i] + width * v[i] for i in range(3))

    return [
        pt(near_run_mm, -half_width_mm, near_tilt_rad),
        pt(near_run_mm, half_width_mm, near_tilt_rad),
        pt(far_run_mm, half_width_mm, far_tilt_rad),
        pt(far_run_mm, -half_width_mm, far_tilt_rad),
    ]


def _advance_chain_link(
    origin: Vec3, u: Vec3, n: Vec3, run_mm: float, tilt_rad: float, fold_rotation_rad: float
) -> tuple[Vec3, Vec3, Vec3]:
    """originからu方向にrun_mm進んだ点を折れ目点とし、そこで折れ目軸
    g=cos(tilt_rad)*v+sin(tilt_rad)*u まわりにfold_rotation_rad回転して次パネルの
    (走行方向, 法線)を返す。戻り値は(fold_point, next_u, next_n)。"""
    v = _cross(n, u)
    g = tuple(math.cos(tilt_rad) * v[i] + math.sin(tilt_rad) * u[i] for i in range(3))
    fold_point = tuple(origin[i] + run_mm * u[i] for i in range(3))
    next_u = _rotate_about_axis(u, g, fold_rotation_rad)
    next_n = _rotate_about_axis(n, g, fold_rotation_rad)
    return fold_point, next_u, next_n


def _chain_from_params(
    p1: Vec3,
    n1: Vec3,
    e_a: Vec3,
    e_b: Vec3,
    psi_rad: float,
    L1_mm: float,
    a1_rad: float,
    phi1_rad: float,
    L2_mm: float,
    a2_rad: float,
    phi2_rad: float,
    L3_mm: float,
) -> tuple[FreeFoldChain, Vec3, Vec3]:
    """8パラメータ(psi,L1,a1,phi1,L2,a2,phi2,L3)からFreeFoldChainを組み立てる。
    e_a/e_bはp1での法線n1に直交する正規直交基底(u1の向きpsiの基準)。
    戻り値は(chain, 実際の終点座標, 終端法線) — 終点がpoint2と一致するかは
    呼び出し側(free_fold_seed/solve_free_fold)の閉合条件チェックに委ねる。
    """
    u1 = tuple(math.cos(psi_rad) * e_a[i] + math.sin(psi_rad) * e_b[i] for i in range(3))
    fold1_pt, u_mid, n_mid = _advance_chain_link(p1, u1, n1, L1_mm, a1_rad, phi1_rad)
    fold2_pt, u3, n3 = _advance_chain_link(fold1_pt, u_mid, n_mid, L2_mm, a2_rad, phi2_rad)
    end_pt = tuple(fold2_pt[i] + L3_mm * u3[i] for i in range(3))
    chain = FreeFoldChain(
        panel1=ChainPanelFrame(origin=p1, u=u1, n=n1),
        panel_mid=ChainPanelFrame(origin=fold1_pt, u=u_mid, n=n_mid),
        panel3=ChainPanelFrame(origin=fold2_pt, u=u3, n=n3),
        L1_mm=L1_mm,
        L2_mm=L2_mm,
        L3_mm=L3_mm,
        a1_rad=a1_rad,
        a2_rad=a2_rad,
    )
    return chain, end_pt, n3


def _in_plane_basis(n: Vec3) -> tuple[Vec3, Vec3]:
    """n(単位法線)に直交する任意の正規直交基底を1組返す。"""
    reference = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    e_a = _normalize(_cross(n, reference))
    e_b = _cross(n, e_a)
    return e_a, e_b


def free_fold_seed(
    point1: FasteningPoint,
    point2: FasteningPoint,
    *,
    bend_radius_mm: float,
    min_bearing_radius_mm: float,
    fold1_slack_mm: float,
    fold2_slack_mm: float,
    max_fold_deg: float = 135.0,
) -> FreeFoldChain | None:
    """折れ目を全てw=n1xn2に平行に保つ版(a1=a2=0)の閉形式解(SS8.2/8.3)。

    two_point_frameを再利用してu1/u2/wを求め、中間折れ目位置
    a = min_bearing_radius_mm + T1 + fold1_slack_mm(bはfold2側も同様)を、
    T(接線長)とa/bの相互依存を不動点反復で解く(T=R*tan(theta/2)のthetaはa/bが
    決まらないと定まらないため、SS8.3参照)。

    このw平行解は自由折れ目族の厳密な一員であり(`solve_free_fold`のホモトピー継続の
    初期値としてそのまま使える。実測で変換残差9.2e-14、SS8.8)、n1・n2が平行に近い
    (two_point_frameのフォールバック分岐)場合や、折れ角が実現不能に大きい場合はNoneを返す。
    """
    n1 = _normalize(point1.normal_xyz)
    frame = two_point_frame(point1, point2)
    u1, u2, w = frame.u1, frame.u2, frame.w
    p1, p2 = point1.position_xyz, point2.position_xyz

    # w直交断面へ射影してからプロファイルを作る。p1とp2はw座標が違うため、3Dのまま
    # B-Aを取るとランプ方向にw成分が乗ってしまう(2026-08-24に実測で発見したバグ、SS8.4参照)。
    def project_out_w(p: Vec3) -> Vec3:
        return _sub(p, _scale(w, _dot(p, w)))

    q1, q2 = project_out_w(p1), project_out_w(p2)

    tangent1 = tangent2 = bend_radius_mm
    for _ in range(20):
        a = min_bearing_radius_mm + tangent1 + fold1_slack_mm
        b = min_bearing_radius_mm + tangent2 + fold2_slack_mm
        point_a = _add(q1, _scale(u1, a))
        point_b = _sub(q2, _scale(u2, b))
        ramp = _sub(point_b, point_a)
        ramp_len = math.sqrt(_dot(ramp, ramp))
        if ramp_len < 1e-6:
            return None
        ramp_dir = _scale(ramp, 1.0 / ramp_len)
        fold1_angle = _angle_between(u1, ramp_dir)
        fold2_angle = _angle_between(ramp_dir, u2)
        new_tangent1 = tangent_length_for_bend_angle_rad(fold1_angle, bend_radius_mm)
        new_tangent2 = tangent_length_for_bend_angle_rad(fold2_angle, bend_radius_mm)
        if abs(new_tangent1 - tangent1) < 1e-7 and abs(new_tangent2 - tangent2) < 1e-7:
            tangent1, tangent2 = new_tangent1, new_tangent2
            break
        tangent1, tangent2 = new_tangent1, new_tangent2
    if max(fold1_angle, fold2_angle) > math.radians(max_fold_deg):
        return None
    if ramp_len < tangent1 + tangent2:
        return None

    # a1=a2=0(fold軸=v)になるよう、各パネルのvがu1/u_mid/u2からcross(n,u)で
    # 直接計算されることを利用する(w平行かどうかを別途チェックする必要が無い設計)。
    e_a, e_b = _in_plane_basis(n1)
    psi = math.atan2(_dot(u1, e_b), _dot(u1, e_a))
    return _chain_from_params(p1, n1, e_a, e_b, psi, a, 0.0, fold1_angle, ramp_len, 0.0, fold2_angle, b)[0]


def solve_free_fold(
    seed: FreeFoldChain,
    point1: FasteningPoint,
    point2: FasteningPoint,
    *,
    target_a1_rad: float,
    target_a2_rad: float,
    homotopy_steps: int = 3,
    newton_iters: int = 40,
    max_step_mm: float = 30.0,
) -> FreeFoldChain | None:
    """seed(w平行解、a1=a2=0)から、折れ目の傾き(a1,a2)をtarget値まで連続的に動かして
    閉合を保ったまま解く(ホモトピー継続、SS8.8)。

    未知数(psi, phi1, L2, a2, phi2)の5変数を、拘束(終点==point2の3本 + 終端法線==
    ±point2法線の2本)の5本でニュートン法により解く。L1・L3・a1はseedの値を保ったまま
    a1のみを目標値まで段階的に動かす(ユーザー①の設計方針: 中間折れ位置=bearing半径+
    接線長+ランダムslackで決まるL1・L3は固定し、折れ目の傾きだけをランダム性の源にする)。

    素朴にpsi/a1/a2を独立乱数で振ると閉合の解が実質存在しない(実測 成立5.0%、
    帯全長 中央値486mm — SS8.8)。w平行解を初期値にしたホモトピー継続でのみ安定に解ける
    (実測 収束率67.5〜80.0%、SS8.8)。
    """
    n1 = seed.panel1.n
    e_a, e_b = _in_plane_basis(n1)
    p1, p2 = point1.position_xyz, point2.position_xyz
    n2 = _normalize(point2.normal_xyz)

    psi0 = math.atan2(_dot(seed.panel1.u, e_b), _dot(seed.panel1.u, e_a))
    phi1_0 = (
        seed.fold1_angle_rad
        if _dot(_cross(seed.panel1.n, seed.panel_mid.n), seed.panel1.v) >= 0
        else -seed.fold1_angle_rad
    )
    phi2_0 = (
        seed.fold2_angle_rad
        if _dot(_cross(seed.panel_mid.n, seed.panel3.n), seed.panel_mid.v) >= 0
        else -seed.fold2_angle_rad
    )
    x = [psi0, phi1_0, seed.L2_mm, 0.0, phi2_0]

    b1, b2 = _in_plane_basis(n2)

    def residual(vec: list[float], a1: float) -> list[float]:
        psi, phi1, L2, a2, phi2 = vec
        _, end_pt, n3 = _chain_from_params(
            p1, n1, e_a, e_b, psi, seed.L1_mm, a1, phi1, L2, a2, phi2, seed.L3_mm
        )
        diff = _sub(end_pt, p2)
        return [diff[0], diff[1], diff[2], _dot(n3, b1), _dot(n3, b2)]

    for step in range(1, homotopy_steps + 1):
        a1 = seed.a1_rad + (target_a1_rad - seed.a1_rad) * step / homotopy_steps
        converged = False
        for _ in range(newton_iters):
            r = residual(x, a1)
            err = math.sqrt(sum(c * c for c in r))
            if err < 1e-7:
                converged = True
                break
            jac = [[0.0] * 5 for _ in range(5)]
            for k in range(5):
                h = 1e-7 * max(1.0, abs(x[k]))
                xp = x[:]
                xp[k] += h
                rp = residual(xp, a1)
                for row in range(5):
                    jac[row][k] = (rp[row] - r[row]) / h
            step_vec = _solve_linear(jac, [-c for c in r])
            if step_vec is None:
                return None
            norm = math.sqrt(sum(c * c for c in step_vec))
            if norm > max_step_mm:
                step_vec = [c * max_step_mm / norm for c in step_vec]
            x = [x[i] + step_vec[i] for i in range(5)]
        if not converged:
            return None

    psi, phi1, L2, a2_raw, phi2 = x
    # 折れ目軸gと-gは同一直線なので、a2は(-90,90]度に畳む(ソルバが2piの整数倍だけ
    # 流すことがある。実測 修正前max 3171度、SS8.8)。
    a2 = math.atan2(math.sin(a2_raw), math.cos(a2_raw))
    if a2 > math.pi / 2:
        a2 -= math.pi
    elif a2 <= -math.pi / 2:
        a2 += math.pi
    if L2 <= 0.0:
        return None
    chain, end_pt, n3 = _chain_from_params(
        p1, n1, e_a, e_b, psi, seed.L1_mm, target_a1_rad, phi1, L2, a2, phi2, seed.L3_mm
    )
    if math.sqrt(sum((end_pt[i] - p2[i]) ** 2 for i in range(3))) > 1e-4:
        return None
    return chain


def _segment_distance_3d(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> float:
    """3D線分ab-cd間の最小距離(交差を含む)。標準的な最近接点法(クランプ付き)。"""
    u = _sub(b, a)
    v = _sub(d, c)
    w0 = _sub(a, c)
    aa, bb, cc = _dot(u, u), _dot(u, v), _dot(v, v)
    dd, ee = _dot(u, w0), _dot(v, w0)
    denom = aa * cc - bb * bb
    s = 0.0 if denom < 1e-12 else max(0.0, min(1.0, (bb * ee - cc * dd) / denom))
    t = (bb * s + ee) / cc if cc > 1e-12 else 0.0
    if t < 0.0:
        t = 0.0
        s = 0.0 if aa < 1e-12 else max(0.0, min(1.0, -dd / aa))
    elif t > 1.0:
        t = 1.0
        s = 0.0 if aa < 1e-12 else max(0.0, min(1.0, (bb - dd) / aa))
    closest_on_ab = tuple(a[i] + s * u[i] for i in range(3))
    closest_on_cd = tuple(c[i] + t * v[i] for i in range(3))
    return math.sqrt(sum((closest_on_ab[i] - closest_on_cd[i]) ** 2 for i in range(3)))


def panel_quad_clearance_mm(corners_a: list[Vec3], corners_b: list[Vec3]) -> float:
    """2つの平面四角形(sheared_panel_cornersが返す4隅、閉ループ順)の最小距離を、
    全辺同士の3D線分距離の最小値として求める。panel1とpanel3のような非隣接パネル同士の
    干渉チェックに使う(w平行版の`flat_panels_clearance_mm`の一般化、SS8.8)。
    """
    na, nb = len(corners_a), len(corners_b)
    best = math.inf
    for i in range(na):
        a1, a2 = corners_a[i], corners_a[(i + 1) % na]
        for j in range(nb):
            b1, b2 = corners_b[j], corners_b[(j + 1) % nb]
            best = min(best, _segment_distance_3d(a1, a2, b1, b2))
    return best

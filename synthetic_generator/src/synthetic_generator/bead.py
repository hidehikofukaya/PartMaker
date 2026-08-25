"""ビード補強(線状プレス補強)の断面パラメータとセル分割(roadmap SS4.4 / SS6.25)。

フランジ(境界エッジを曲げる)とは別種のプリミティブで、一般面の内部に入れる線状の
段差である。中立面表現では「面の局所的なオフセット」として現れるため、既存のジョグ
構築と同じ原理 — 平面セルをシャープに結合してからエッジフィレット — で作れる。

構成(ユーザー確定、2026-08-21): 走行方向に走る1本のビードを幅方向中央に置き、断面は
台形(平地・立上り・頂部・立下り・平地の5ストリップ)。曲げをまたいで走らせるのが目的
なので、各パネル(flat1/ランプ/flat2)を同じ5ストリップに分割し、頂部だけを深さぶん
オフセットする。

**曲げ部での頂部の折れ目位置**: 隣接する2パネルの法線をna/nbとすると、両方の面を
深さdだけオフセットした平面同士の交線は、元の折れ目から `d * (na+nb)/(1+na.nb)`
(マイターオフセット)だけずれる。この式は na=nb(折れなし)で d*na に、直角で
d*(na+nb)(長さ d*sqrt2 = d/cos45度)に退化する、通常のオフセットポリラインの
マイター式そのもの。

**セルの平面性**: 平地・頂部の各セルは常に平面(元パネル/オフセット面のどちらか一方に
4隅とも乗るため)。立上り・立下りのセルも、パネルの走行方向が幅方向成分を持たない限り
平面になる。

例外はランプ(横方向オフセットを吸収するため台形になっているパネル)の壁セルで、ここは
**幾何的に捻れる**(双曲放物面になる)。ランプの走行方向は横方向オフセットを吸収する
ぶん幅方向成分を持つ一方、頂部のマイターオフセットは各パネルの法線の和で決まるため
幅方向成分を持たない — この差だけ、壁の上辺と下辺の向きがずれる。実物のビードが
曲げをまたぐ場合も壁は同様に捻れるので、これは近似誤差ではなく正しい幾何である
(実測で1〜2mm程度)。CATIAの`Fill`が非平面の4辺輪郭を受けられるかは実機で検証する。
"""

from __future__ import annotations

import dataclasses
import math
import random

from synthetic_generator.classify import MIN_NEUTRAL_PLANE_RADIUS_MM, Vec3

# ユーザー確定(2026-08-21)のサンプリングレンジ。中立面表現なので深さ=中立面のオフセット量、
# 壁角度は一般面からの立ち上がり角(ジョグのランプ角と同じ意味)。
BEAD_DEPTH_RANGE_MM = (4.0, 10.0)

# ユーザー確定(2026-08-24): ビード壁のドラフト角の一般的な設計狙い値は45〜70度。
# 垂直(90度)はプレス成形上まず成立しないため、上限は70度に留める。
BEAD_WALL_ANGLE_RANGE_DEG = (45.0, 70.0)

# 頂部の幅。実際の上限は板幅から逆算する(sample_bead参照)ので、ここは名目レンジ。
# 板幅が25〜50mmに絞られた(2026-08-24)ため、下限も従来の20mmから引き下げてある。
BEAD_TOP_WIDTH_RANGE_MM = (10.0, 40.0)

# 足元フィレットの後退量の**外側**に、さらに残しておく平地のクリアランス。
# フットプリント外側に要る余白は「足元フィレットの後退量 + これ」= BeadParams.side_margin_mm。
BEAD_SIDE_CLEARANCE_MM = 2.0

# 端の幅ガイドをフットプリントより外へ出す量。gsd_buildと事前判定(general_geometry.
# check_bead_feasible)が同じ値を使うよう、ここで一元定義する(2026-08-25に
# gsd_buildのクラス属性から移動)。
BEAD_GUIDE_MARGIN_MM = 20.0

# 足元Rと頂稜線Rが壁の斜辺を食ってよい割合の上限 2*R*tan(θ/2) <= K*slant。
# 幾何としてはK=1で「2つのフィレットが重ならない」を満たすが、実測ではそれでは全く
# 足りなかった(2026-08-25、6つの基準面 x 消費率10段階=60ビルドの直交実験):
#   消費率 0.61/0.73 -> 12/12成功、0.85 -> 11/12、0.91〜1.40 -> 基準面依存で3〜4/6が失敗、
#   1.58 -> 0/6。しかも中間帯は単調ですらない(#55は0.91-1.04で失敗し1.10-1.40で成功)。
# 中間帯は幾何が既に退化していてCATIA内部処理の当たり外れになっているので、設計として
# 使ってはいけない。全基準面で成功した0.73を上限に採る。
BEAD_RIDGE_WALL_CONSUMPTION_MAX = 0.73

# ビード中心線のスプラインを張るとき、1パネルあたり何点サンプリングするか(SS8.9)。
# 平坦区間は直線なので本来2点で足りるが、投影(AddNewProject)がフィレット領域で
# 素直に収束するよう、余裕をみて5点取る。
CENTRELINE_POINTS_PER_PANEL = 5


@dataclasses.dataclass(frozen=True)
class BeadParams:
    depth_mm: float
    top_width_mm: float
    wall_angle_deg: float
    ridge_radius_mm: float   # 断面の稜線R(頂稜線=手順⑥・足元=手順⑦に共通)

    @property
    def wall_run_mm(self) -> float:
        """壁の幅方向への投影長(平地から頂部までに幅方向で消費する距離)。"""
        return self.depth_mm / math.tan(math.radians(self.wall_angle_deg))

    @property
    def half_footprint_mm(self) -> float:
        """ビードが幅方向に占める範囲の半分(頂部の半幅 + 壁の投影長)。"""
        return self.top_width_mm / 2.0 + self.wall_run_mm

    @property
    def wall_slant_mm(self) -> float:
        """壁そのものの長さ(斜辺)。稜線フィレットの後退量がこれを超えると成立しない。"""
        return self.depth_mm / math.sin(math.radians(self.wall_angle_deg))

    @property
    def ridge_setback_mm(self) -> float:
        """稜線フィレット1つが壁の斜辺方向に食う長さ。

        平地と壁(あるいは壁と頂部)が成す材料側の角は 180-wall_angle なので、
        半径Rの円が接する点は頂点から `R / tan((180-θ)/2) = R * tan(θ/2)` だけ離れる。
        足元・頂稜線のどちらも同じ式になる。
        """
        return self.ridge_radius_mm * math.tan(math.radians(self.wall_angle_deg) / 2.0)

    @property
    def side_margin_mm(self) -> float:
        """フットプリントの外側に必要な平地の幅。

        足元フィレットが基準面側に食う後退量(ridge_setback_mm)に、さらに
        BEAD_SIDE_CLEARANCE_MM の余白を足したもの。
        """
        return self.ridge_setback_mm + BEAD_SIDE_CLEARANCE_MM

    @property
    def ridges_fit_on_wall(self) -> bool:
        """壁の斜辺に、足元と頂稜線の2つのフィレットが重ならず載るか。"""
        return 2.0 * self.ridge_setback_mm <= self.wall_slant_mm


def sample_bead(rng: random.Random, half_width_mm: float) -> BeadParams:
    """板の半幅`half_width_mm`に収まるビード断面を1つサンプリングする。

    このプロジェクトで一貫している「実行可能な上限を計算してから、その中でサンプリング
    する」方針を、互いに依存する3つの寸法へ順に適用する。依存の向きが一方通行なので、
    リトライ無しで(ほぼ)実行可能な組み合わせが得られる:

      1. `(wall_angle, depth)` — 実行可能領域上で同時に一様(リジェクションサンプリング。
         稜線Rが中立面R最小を下回れない条件が、壁が立つほど深さの下限を押し上げる)
      2. `ridge_radius` — 壁の斜辺を2つのフィレットが食う割合の上限
         `2*R*tan(θ/2) <= K*depth/sin(θ)` から上限が決まる
      3. `top_width` — 2で決まる`side_margin_mm`を引いた残り幅に収まる条件から上限が決まる

    平面視のコーナーRはサンプリングしない。外形の閉曲線は基準面の境界を内側へ
    平行オフセットして得るので、コーナーの丸めはCATIA側のオフセット処理が決める
    (2026-08-25、SS8.14)。

    全ての半径は中立面R最小(`MIN_NEUTRAL_PLANE_RADIUS_MM`)を下回らない。深さの下限を
    壁角度から逆算しているので、稜線Rについては構造上つねに成立する。頂部幅だけは板幅
    次第で成立しないことがあり、その場合は最小値を返して`plan_bead_on_surface`側で
    Infeasibleとして弾かれる(templates/*.pyと同じ「成立しない値をあえて返して下流で
    弾く」設計)。
    """
    # 1. (壁角度, 深さ): 実行可能領域の上で**同時に一様**にサンプリングする。
    #    実行可能条件は、稜線Rの最小値(中立面R最小)と消費率上限から来る深さ下限
    #    2*R0*tan(θ/2) <= K*slant = K*depth/sin(θ)  ->  depth >= 2*R0*tan(θ/2)*sin(θ)/K
    #    (壁が立つほど下限が上がり、70度では9.0mm)。
    #    以前は「角度を振ってから深さを下限つき一様で振る」逐次方式だったが、下限が
    #    角度の関数なので**物理が要求する以上に強い相関 r=+0.81 がデータに入り**、
    #    「急な壁 ⇒ 必ず深い」という板金設計には存在しない規則を学習させる恐れがあった
    #    (2026-08-25の多様性検査で発覚)。矩形からのリジェクションサンプリングなら
    #    領域上で正確に一様になり、残る相関は実行可能領域の形が強制する分だけになる。
    #    実行可能率は約60%なので平均1.7回で当たる。
    for _ in range(1000):
        wall_angle = rng.uniform(*BEAD_WALL_ANGLE_RANGE_DEG)
        depth = rng.uniform(*BEAD_DEPTH_RANGE_MM)
        theta = math.radians(wall_angle)
        half_angle_tangent = math.tan(theta / 2.0)
        min_depth = (
            2.0 * MIN_NEUTRAL_PLANE_RADIUS_MM * half_angle_tangent * math.sin(theta)
            / BEAD_RIDGE_WALL_CONSUMPTION_MAX
        )
        if depth >= min_depth:
            break
    else:  # 全角度で下限<=上限(70度でも9.0<10.0)なので実際には到達しない
        raise RuntimeError("bead (angle, depth) rejection sampling did not terminate")

    # 2. 稜線R: 2*R*tan(θ/2) <= K*slant を R について解く。1で下限を効かせてあるので
    #    上限は必ず中立面R最小以上になる。
    slant = depth / math.sin(theta)
    max_ridge = BEAD_RIDGE_WALL_CONSUMPTION_MAX * slant / (2.0 * half_angle_tangent)
    ridge_radius = rng.uniform(MIN_NEUTRAL_PLANE_RADIUS_MM, max_ridge)

    # 3. 頂部幅: half_footprint + side_margin <= half_width
    #    half_footprint = top_width/2 + wall_run なので top_width の上限が決まる。
    wall_run = depth / math.tan(theta)
    side_margin = ridge_radius * half_angle_tangent + BEAD_SIDE_CLEARANCE_MM
    max_top_half = half_width_mm - side_margin - wall_run
    min_top_half = BEAD_TOP_WIDTH_RANGE_MM[0] / 2.0
    max_top_half = min(max_top_half, BEAD_TOP_WIDTH_RANGE_MM[1] / 2.0)
    top_width = 2.0 * rng.uniform(min_top_half, max(min_top_half, max_top_half))

    return BeadParams(
        depth_mm=depth,
        top_width_mm=top_width,
        wall_angle_deg=wall_angle,
        ridge_radius_mm=ridge_radius,
    )


@dataclasses.dataclass(frozen=True)
class BeadPanelFrame:
    """ビード配置計算(および中心線の複数平面交線構築、SS8.6)に使う1パネル分の
    ローカルフレーム。origin/u/v・near_run_mm/far_run_mmは、gsd_build.pyが実際に
    sheared_panel_corners/end_panel_cornersへ渡すのと同じ絶対run座標系を共有する
    設計にしている — ビードのプローブ点や中心線が、実際に構築されるパネル形状と
    寸分違わず一致することを保証するため(座標系がズレると、フィレット領域を平坦と
    誤認するなどしてprobe-and-selectが全滅する、2026-08-24に自由折れ目チェーン
    導入時に実測)。
    """

    origin: Vec3
    u: Vec3  # 走行方向(単位ベクトル)
    v: Vec3  # 幅方向(単位ベクトル) = n x u
    near_run_mm: float
    far_run_mm: float


@dataclasses.dataclass(frozen=True)
class BeadSurfacePlan:
    """完成した基準面の上にビードを載せるための配置情報(CATIA非依存)。

    BiTangent方式(docs/bead_construction_flowchart.md SS6)のビルダーが必要とする
    座標だけを、パネルのローカルフレーム列から解析的に求めたもの。「頂面側」の点は
    基準面上の投影として持ち、実際の立ち上がり方向は面のオフセットに問い合わせて決める
    (谷折り面ではパネルごとに法線の符号が反転するため、解析的に決め打ちできない)。
    """

    start_guide: tuple[Vec3, Vec3]  # 始端の幅ガイド線の両端(オーバーサイズ済み)
    end_guide: tuple[Vec3, Vec3]    # 終端の幅ガイド線の両端(同上)
    base_keep: list[Vec3]           # 基準面上で残るべき点
    base_remove: list[Vec3]         # 基準面上で消えるべき点(ビード直下)
    top_keep_bases: list[Vec3]      # 頂面で残るべき点の、基準面上への投影
    top_remove_bases: list[Vec3]    # 頂面で消えるべき点の、基準面上への投影
    top_panel_index: list[int]      # top_keep_bases各点が乗るパネルの番号
    top_remove_panel_index: list[int]
    wall_root_probes: list[Vec3]    # 壁の根元(左右まとめ。バンド完成後のkeep判定用)
    # 左右それぞれの壁の根元プローブ。BiTangentでコーナーを作る途中段階では、まだ
    # 反対側の壁が存在しないので、その段階のkeep判定に全点を渡すと必ず不合格になる
    # (2026-08-24に実機で発覚した既存バグ)。段階に応じて片側だけを渡すために分けている。
    wall_root_probes_left: list[Vec3]
    wall_root_probes_right: list[Vec3]
    # 壁4枚(左・始端・右・終端)の頂部エッジ中点の、基準面上への投影と、乗るパネル番号。
    # 実際の頂部位置は base + rise_sign * depth * panel_normal で求める(符号は実測)。
    wall_top_bases: list[Vec3]
    wall_top_panel_index: list[int]
    # ビード中心線を張るための標本点(走行順)。各パネルの平坦区間(曲げフィレットの外側)
    # からのみ採るので、全点が厳密に基準面上に乗る。CATIA側はこれをスプラインで結び、
    # 基準面へ投影して中心線にする(SS8.9)。フィレット領域はスプラインの補間+投影に任せる。
    centreline_points: list[Vec3]
    # ビード外形(フットプリント)の閉曲線を、CATIA側で基準面から導出するための指示。
    # 基準面をtrim_sections(各3点で表す平面)で両端Splitし、その断片の境界を
    # outline_offset_mmだけ内側へ平行オフセットすると、フットプリントの閉曲線になる。
    # Splitの残す側と平行曲線の向きは形状依存なので決め打ちせず、得られた曲線が
    # outline_probesを通ることを実測して選ぶ(SS8.14)。
    outline_offset_mm: float
    trim_sections: list[tuple[Vec3, Vec3, Vec3]]
    trim_keep_probe: Vec3
    trim_remove_probes: list[Vec3]
    outline_probes: list[Vec3]


def plan_bead_on_surface(
    panel_frames: list[BeadPanelFrame],
    bead: BeadParams,
    *,
    inset_mm: float,
    guide_margin_mm: float,
    half_width_mm: float,
    fold_tangents: list[tuple[float, float]],
    fold_tilts: list[tuple[float, float]],
) -> BeadSurfacePlan:
    """ビードの配置と、向き選定に使う幾何プローブ点を決める。

    2026-08-24(docs/catia_bead_fillet_investigation_log.md SS8.6)より、自由折れ目
    チェーン(`classify.FreeFoldChain`)に対応するため設計を変更した。ビードは各パネルの
    中心線(v=0のu軸)にそのまま沿って走る — 締結点はチェーン構築の時点で既にこの軸上
    (run=near_run_mm/far_run_mmの端)にあるので、旧来の「全パネル共通の幅座標を探して
    各パネルの端辺で補間する」処理が丸ごと不要になった(SS7.3のカニ歩き問題そのものが、
    自由折れ目チェーンの導入によって解消されたため)。板幅は物理的に単一(全パネル共通の
    `half_width_mm`)なので、フィット判定も1回で済む。

    `fold_tangents`はパネルごとの(近端, 遠端)で曲げフィレットが食う長さ。プローブ点は
    必ずこの外側(=本当に平坦な領域)に置く。フィレットの円筒面上に置くと、平面前提で
    計算した座標が実際の面から数mmずれ、向き判定が誤って全滅する(2026-08-24に実測)。
    """
    if not panel_frames:
        raise ValueError("bead: no panels given")

    required = bead.half_footprint_mm + bead.side_margin_mm
    if half_width_mm < required:
        raise ValueError(
            f"bead (half footprint {bead.half_footprint_mm:.1f}mm + margin "
            f"{bead.side_margin_mm:.1f}mm = {required:.1f}mm) does not fit within the panel "
            f"half width {half_width_mm:.1f}mm. Infeasible; not attempting construction."
        )

    def at(panel_index: int, run_mm: float, width_from_centre_mm: float) -> Vec3:
        """パネル上の点を (絶対走行座標, ビード中心線からの幅方向距離) で指定する。

        各パネルの中心線(v=0)はチェーン構築そのものによってrun軸上に厳密に乗るので、
        旧来のような平面交線や補間は不要 — originを基準にu/vへ射影するだけでよい。
        """
        frame = panel_frames[panel_index]
        return tuple(
            frame.origin[i] + run_mm * frame.u[i] + width_from_centre_mm * frame.v[i] for i in range(3)
        )

    last_index = len(panel_frames) - 1
    first_near, first_far = panel_frames[0].near_run_mm, panel_frames[0].far_run_mm
    last_near, last_far = panel_frames[last_index].near_run_mm, panel_frames[last_index].far_run_mm
    if (first_far - first_near) <= inset_mm or (last_far - last_near) <= inset_mm:
        raise ValueError(
            f"bead run-out inset ({inset_mm:.1f}mm) is longer than the end panel itself "
            f"({min(first_far - first_near, last_far - last_near):.1f}mm). "
            "Infeasible; not attempting construction."
        )

    # 端の幅ガイドはコーナーRで削られる前提でフットプリントより外へ出す(オーバーサイズ)が、
    # **板幅を超えてはいけない**。超えるとガイド線が基準面から外れ、Mode=4のスイープ
    # (曲線が案内サーフェス上にあることを要求する)が全ドラフト角で失敗する
    # (2026-08-24に実機で確認: half_footprint12.5+margin20=32.5mm に対し半幅25mm)。
    # ビードの走行範囲(両端の締結点から逃げを取った区間)。外形の閉曲線もここに収まる。
    start_run = first_near + inset_mm
    end_run = last_far - inset_mm

    # ビードの端(逃げ位置)は曲げフィレットの外=本当に平坦な区間になければならない。
    # 端パネルが短いと、逃げを取った位置が曲げのR上に乗ってしまう。そうなると
    # (a) 端の逃げが曲げに重なるという形状上の不正、(b) 平面前提で計算した解析座標が
    # 実際の面から数百um外れ、頂面オフセットの向き判定が落ちる、の両方が起きる
    # (2026-08-25に実測: 終端パネル長21.8mmに対しフィレット接線8.5mm、逃げ位置が
    # わずか0.12mmしか外に出ておらず、0.19〜0.35mmのずれとして現れた)。
    FLAT_CLEARANCE_MM = 1.0  # フィレットの接点からさらに離す余裕
    for label, run_mm, index in (
        ("start", start_run, 0),
        ("end", end_run, len(panel_frames) - 1),
    ):
        frame = panel_frames[index]
        near_cut, far_cut = fold_tangents[index]
        low = frame.near_run_mm + near_cut + FLAT_CLEARANCE_MM
        high = frame.far_run_mm - far_cut - FLAT_CLEARANCE_MM
        if not low <= run_mm <= high:
            raise ValueError(
                f"bead {label} run-out at run={run_mm:.1f}mm falls outside the flat region "
                f"[{low:.1f}, {high:.1f}]mm of its end panel -- the run-out would sit on the "
                "bend fillet. Infeasible; not attempting construction."
            )

    guide_half = min(bead.half_footprint_mm + guide_margin_mm, half_width_mm * 0.98)
    start_guide = (at(0, first_near + inset_mm, -guide_half), at(0, first_near + inset_mm, guide_half))
    end_guide = (
        at(last_index, last_far - inset_mm, -guide_half),
        at(last_index, last_far - inset_mm, guide_half),
    )

    # --- 幾何プローブ ---
    outside = bead.half_footprint_mm + bead.side_margin_mm
    base_keep: list[Vec3] = []
    base_remove: list[Vec3] = []
    top_keep_bases: list[Vec3] = []
    top_panel_index: list[int] = []
    top_remove_bases: list[Vec3] = []
    top_remove_panel_index: list[int] = []
    wall_root_probes_left: list[Vec3] = []
    wall_root_probes_right: list[Vec3] = []
    probe_run_of: dict[int, float] = {}

    for index, frame in enumerate(panel_frames):
        near_run, far_run = frame.near_run_mm, frame.far_run_mm

        # 曲げフィレットの外側=本当に平坦な区間。ここにしかプローブを置かない。
        near_cut, far_cut = fold_tangents[index]
        flat_low = near_run + near_cut + FLAT_CLEARANCE_MM
        flat_high = far_run - far_cut - FLAT_CLEARANCE_MM
        if flat_high - flat_low < 2.0:
            continue  # このパネルには平坦なプローブ余地が無い(ランプが短い等)

        # 残るべき基準面: フットプリントのすぐ外側と、パネル外形のすぐ内側の両方を見る。
        # 外側1点だけだと、角に残ったスリバーに当たって誤合格する(SS6.2)。
        for fraction in (0.25, 0.75):
            run_mm = flat_low + fraction * (flat_high - flat_low)
            for side in (-1.0, 1.0):
                base_keep.append(at(index, run_mm, side * outside))
                base_keep.append(at(index, run_mm, side * half_width_mm * 0.95))

        # ビードが実際に存在する走行区間(端パネルは逃げのぶん内側から)
        run_out = inset_mm + bead.wall_run_mm + bead.ridge_setback_mm
        low = max(flat_low, near_run + run_out if index == 0 else flat_low)
        high = min(flat_high, far_run - run_out if index == last_index else flat_high)
        if high - low < 1.0:
            continue
        mid = (low + high) / 2.0
        probe_run_of[index] = mid
        base_remove.append(at(index, mid, 0.0))       # ビード直下: 消えるべき
        top_keep_bases.append(at(index, mid, 0.0))    # ビード頂面: 残るべき
        top_panel_index.append(index)
        top_remove_bases.append(at(index, mid, outside))  # 頂面のビード外側: 消えるべき
        top_remove_panel_index.append(index)
        wall_root_probes_left.append(at(index, mid, -bead.half_footprint_mm))
        wall_root_probes_right.append(at(index, mid, bead.half_footprint_mm))

    # 逃げより外側(ビードが無い領域)の基準面も残るべき
    base_keep.append(at(0, first_near + inset_mm * 0.4, 0.0))
    base_keep.append(at(last_index, last_far - inset_mm * 0.4, 0.0))

    if not top_keep_bases:
        raise ValueError(
            "bead: the run-out insets leave no room for the bead itself. "
            "Infeasible; not attempting construction."
        )

    # ビードは**全てのパネル**に平坦区間を持っていなければならない。曲げが急でRが大きいと
    # フィレットがパネルを食い尽くし、そのパネルには平坦区間が残らない。外形の閉曲線は
    # それでも端から端まで走るので、ビードの大半がフィレットの上に乗ることになり、
    # 平面前提で計算した頂部エッジの解析位置が実物と合わなくなる。
    # 2026-08-25の実測(240試行): 3パネルとも平坦区間があれば壁の掃引は28/28成功、
    # 2パネルなら8/15、1パネルなら2/11。稜線Rの失敗も同じ変数で分離する
    # (平坦区間の最小値: 成功53.7mm / 失敗43.7mm)。
    if len(top_panel_index) < len(panel_frames):
        missing = [i for i in range(len(panel_frames)) if i not in top_panel_index]
        raise ValueError(
            f"bead has no flat bearing stretch on panel(s) {missing} of {len(panel_frames)} "
            "-- the bend fillets consume those panels, so most of the bead would sit on "
            "curvature. Infeasible; not attempting construction."
        )

    # 壁4枚の頂部エッジ中点(基準面への投影)。壁は根元から内側へ wall_run だけ倒れる。
    top_half = bead.top_width_mm / 2.0
    middle_index = top_panel_index[len(top_panel_index) // 2]
    # 左右の壁の頂部は、そのパネルの「ビードが載っている平坦区間」の中央で見る。
    middle_run = probe_run_of[middle_index]
    # 外形の閉曲線を一括で掃引するようになった(SS8.10)ので、壁は4枚まとめて同じ向きに
    # 倒れる。ドラフト角の判定には長辺(左右)の頂部2点だけで足りる。
    # 旧方式で使っていた始端/終端の壁頂部は、逃げ位置にあるぶん曲げフィレットやパネル端に
    # 近く、頂面オフセットの向き判定を落とす主因になっていた(2026-08-25の実測で、
    # 「sample 1 が0.20〜0.80mmずれ」「sample 9 が4.7〜9.3mmずれ」として現れていた)。
    wall_top_bases = [
        at(middle_index, middle_run, -top_half),   # 左の壁の頂部
        at(middle_index, middle_run, top_half),    # 右の壁の頂部
    ]
    wall_top_panel_index = [middle_index, middle_index]

    # ビード中心線の標本点(SS8.9)。各パネルの平坦区間からのみ採るので全点が厳密に
    # 基準面上にある。曲げフィレット領域はCATIA側でスプラインが補間し、投影が実際の
    # 曲面へ吸着させる。平坦区間が取れないパネルでもスプラインが素直に走るよう、
    # 最低1点(パネル中央)は必ず入れる。
    centreline_points: list[Vec3] = []
    for index, frame in enumerate(panel_frames):
        near_cut, far_cut = fold_tangents[index]
        lo = frame.near_run_mm + near_cut + FLAT_CLEARANCE_MM
        hi = frame.far_run_mm - far_cut - FLAT_CLEARANCE_MM
        if hi - lo < 2.0:
            centreline_points.append(at(index, (frame.near_run_mm + frame.far_run_mm) / 2.0, 0.0))
            continue
        for k in range(CENTRELINE_POINTS_PER_PANEL):
            run = lo + (hi - lo) * k / (CENTRELINE_POINTS_PER_PANEL - 1)
            centreline_points.append(at(index, run, 0.0))

    # --- ビード外形(フットプリント)の起点 ---
    # 外形曲線はPython側では作らず、CATIA側で基準面から導出する(2026-08-25、ユーザー提案)。
    #
    # 旧方式は「解析的に点を打つ -> 閉スプライン -> AddNewProjectで基準面へ投影」だったが、
    # 長辺の標本は曲げフィレットの外(平坦区間)からしか採れないため、フィレットを横切る
    # 区間が**基準面から離れた3Dの弦**になる。そのまま投影すると曲線が分断し、Mode=4の
    # ドラフトスイープが全角度・全長さで落ちた(2026-08-25、41件中14件で実測)。
    #
    # 確定方式: 基準面を両端でSplitし、その断片の境界(AddNewBoundaryOfSurface)を
    # `AddNewCurvePar`で内側へ一様オフセットする。曲線は構築上つねに面の上にあるので
    # 投影という経路が丸ごと消える。実機12件で境界・平行曲線・スイープとも全成功
    # (旧方式は曲線66%・内向きスイープ67%)。
    #
    # オフセット量dは横方向・走行方向で共通なので、走行方向の切断位置はd手前に置く:
    #   長辺  : 半幅 - d = フットプリント半幅  -> d = half_width - half_footprint
    #   端    : (start_run - d) + d = start_run
    outline_offset_mm = half_width_mm - bead.half_footprint_mm
    if outline_offset_mm <= 0.0:
        raise ValueError(
            f"bead footprint half-width ({bead.half_footprint_mm:.1f}mm) leaves no room to "
            f"offset inward from the plate edge ({half_width_mm:.1f}mm). Infeasible."
        )
    # 切断位置は逃げ位置よりさらにd手前なので、逃げが平坦区間にあっても切断が曲げの
    # フィレット上に落ちうる。フィレットを斜めに切ると境界の隅が接線的に潰れ、内側へ
    # オフセットした曲線が掃引の入力として不正になる。逃げ位置と同じ判定を掛ける。
    for label, run_mm, index in (
        ("start", start_run - outline_offset_mm, 0),
        ("end", end_run + outline_offset_mm, last_index),
    ):
        frame = panel_frames[index]
        near_cut, far_cut = fold_tangents[index]
        low = frame.near_run_mm + near_cut + FLAT_CLEARANCE_MM
        high = frame.far_run_mm - far_cut - FLAT_CLEARANCE_MM
        if not low <= run_mm <= high:
            raise ValueError(
                f"bead {label} trim section at run={run_mm:.1f}mm falls outside the flat region "
                f"[{low:.1f}, {high:.1f}]mm of its end panel -- the split would cut through the "
                "bend fillet. Infeasible; not attempting construction."
            )

    def trim_section(index: int, run_mm: float) -> tuple[Vec3, Vec3, Vec3]:
        """基準面を切る平面を3点で表す。パネルに垂直で、幅方向vと法線nを含む。"""
        frame = panel_frames[index]
        normal = _normalize(_cross(frame.u, frame.v))
        base = at(index, run_mm, 0.0)
        return (
            base,
            at(index, run_mm, half_width_mm),
            tuple(base[i] + 10.0 * normal[i] for i in range(3)),
        )

    # 切断平面は無限に広がるので、意図した端パネル以外も切ってしまうことがある。
    # そうなるとSplitの結果が離れた2枚になり(ビード区間側の点は乗ったままなので
    # keepプローブだけでは検出できない)、境界が2ループになって掃引が全角度で落ちる。
    # 2026-08-25の実測: トリム無しの境界オフセットなら12件×4角度すべて成功するのに、
    # トリムを挟むと5/12が落ちた。他パネルを横切る配置はここで弾く。
    TRIM_PANEL_CLEARANCE_MM = 1.0

    def cuts_other_panels(cut_index: int, run_mm: float) -> int | None:
        frame = panel_frames[cut_index]
        normal_u = frame.u
        origin = at(cut_index, run_mm, 0.0)
        for index, other in enumerate(panel_frames):
            if index == cut_index:
                continue
            near_tilt, far_tilt = fold_tilts[index]
            signed = [
                _dot(_sub(at(index, run + width * math.tan(tilt), width), origin), normal_u)
                for width in (-half_width_mm, half_width_mm)
                for run, tilt in ((other.near_run_mm, near_tilt), (other.far_run_mm, far_tilt))
            ]
            if min(signed) < -TRIM_PANEL_CLEARANCE_MM and max(signed) > TRIM_PANEL_CLEARANCE_MM:
                return index
        return None

    trim_sections = []
    for cut_index, run_mm in ((0, start_run - outline_offset_mm),
                              (last_index, end_run + outline_offset_mm)):
        crossed = cuts_other_panels(cut_index, run_mm)
        if crossed is not None:
            raise ValueError(
                f"bead trim section on panel {cut_index} (run={run_mm:.1f}mm) also cuts through "
                f"panel {crossed} -- the split would leave two disjoint pieces and the boundary "
                "would not be a single loop. Infeasible; not attempting construction."
            )
        trim_sections.append(trim_section(cut_index, run_mm))
    # 平行曲線の向きとSplitの残す側は決め打ちできない(形状依存)。得られた閉曲線が
    # ここを通ることを実測して選ぶ。コーナーの丸め方に左右されないよう、四隅ではなく
    # 長辺の中点と両端キャップの中点を使う。
    outline_probes = (
        wall_root_probes_left + wall_root_probes_right
        + [at(0, start_run, 0.0), at(last_index, end_run, 0.0)]
    )
    # Splitで残す側の判定に使う点。「ビード区間の内側が残る」だけでは足りない —
    # 平面は無限に広がるので切り落とし側に離れた別片が残ることがあり、keepプローブ
    # だけだと素通りする。境界が2ループになって掃引が全角度で落ちるので、切り落とし側
    # (板の両端すぐ内側)が実際に消えたことも要求する
    # (`_bead_bitangent`が同じ失敗で学んだのと同じパターン)。
    trim_keep_probe = at(last_index // 2, (start_run + end_run) / 2.0, 0.0)
    trim_remove_probes = [
        at(0, panel_frames[0].near_run_mm + FLAT_CLEARANCE_MM, 0.0),
        at(last_index, panel_frames[last_index].far_run_mm - FLAT_CLEARANCE_MM, 0.0),
    ]

    return BeadSurfacePlan(
        start_guide=start_guide,
        end_guide=end_guide,
        base_keep=base_keep,
        base_remove=base_remove,
        top_keep_bases=top_keep_bases,
        top_remove_bases=top_remove_bases,
        top_panel_index=top_panel_index,
        top_remove_panel_index=top_remove_panel_index,
        wall_root_probes=wall_root_probes_left + wall_root_probes_right,
        wall_root_probes_left=wall_root_probes_left,
        wall_root_probes_right=wall_root_probes_right,
        wall_top_bases=wall_top_bases,
        wall_top_panel_index=wall_top_panel_index,
        centreline_points=centreline_points,
        outline_offset_mm=outline_offset_mm,
        trim_sections=trim_sections,
        trim_keep_probe=trim_keep_probe,
        trim_remove_probes=trim_remove_probes,
        outline_probes=outline_probes,
    )


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: Vec3, k: float) -> Vec3:
    return (v[0] * k, v[1] * k, v[2] * k)


def _dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(v: Vec3) -> Vec3:
    length = math.sqrt(_dot(v, v))
    if length < 1e-9:
        raise ValueError(f"cannot normalize near-zero vector: {v}")
    return _scale(v, 1.0 / length)


@dataclasses.dataclass(frozen=True)
class BeadLayout:
    """ビード入りのセル群と、フィレットを当てる折れ目の中点。

    `cells`は`classify.end_panel_corners`と同じ4隅の規約([near,-hw],[near,+hw],
    [far,+hw],[far,-hw])で、そのまま`rect_fill`+`join`に流せる。
    `fold_mids`はメイン形状の折れ目(ビードで5本に分断されている)、`bead_mids`は
    ビード自身の折れ目(縦4本 x パネル数 + 端の逃げ4本 x 端の数)。
    """

    cells: list[list[Vec3]]
    fold_mids: list[Vec3]
    bead_mids: list[Vec3]



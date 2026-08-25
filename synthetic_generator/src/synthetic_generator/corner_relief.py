"""基準面の隅の余肉カット(CATIA非依存の計画)。2026-08-25、SS15、ユーザー指定。

補強(フランジ/ビード)の生成後、基準面の隅を R = 締結点の必要最小半径
(min_bearing_radius_mm)で丸めて落とす。締結点まわりに必要なのはベアリング円だけで、
矩形の隅はどの構造にも寄与しない死重のため(実務の重量低減の余肉カット)。

- ビード部品: 四隅すべて(ビードは両端から逃げているので四隅は空いている)
- フランジ部品: **反フランジ側の2点のみ**(フランジ側の隅はフランジが連続する構造部)

カット円弧は端パネルのベアリング平坦区間に載る(解析座標が厳密に面上)。
R = bearing半径のとき、側辺の接点はちょうど締結点の走行位置(run=0)に来る —
座面を侵さないぎりぎりまで削る、実務の丸タブ端形状になる。
"""

from __future__ import annotations

import dataclasses
import math

from synthetic_generator.bead import BeadPanelFrame
from synthetic_generator.classify import Vec3

ARC_POINTS = 7          # 四分円の標本点数
FLAT_CLEARANCE_MM = 1.0


@dataclasses.dataclass(frozen=True)
class CornerCut:
    """1つの隅のカット。arc_pointsを結んだ曲線でSplitし、remove側を捨てる。"""

    arc_points: list[Vec3]   # 側辺の接点 -> 端辺の接点 の四分円(面上、走行順)
    remove_probe: Vec3       # 捨てられるべき点(元の隅)
    label: str


def plan_corner_relief(
    panel_frames: list[BeadPanelFrame],
    *,
    half_width_mm: float,
    radius_mm: float,
    fold_tangents: list[tuple[float, float]],
    exclude_side: int | None = None,
) -> list[CornerCut]:
    """余肉カットの計画。`exclude_side`はフランジ側(+1/-1、カットしない側)。

    円弧が端パネルの平坦区間からはみ出す(基準面Rの上に乗る)場合はValueError —
    ベアリング設計上は起きないはずの構成なので、明示的なInfeasibleとして扱う。
    """
    if radius_mm > half_width_mm + 1e-9:
        raise ValueError(
            f"corner relief radius ({radius_mm:.1f}mm) exceeds the half width "
            f"({half_width_mm:.1f}mm). Infeasible."
        )

    def at(frame: BeadPanelFrame, run: float, width: float) -> Vec3:
        return tuple(frame.origin[i] + run * frame.u[i] + width * frame.v[i] for i in range(3))

    cuts: list[CornerCut] = []
    ends = (
        # (フレーム, 端のrun, 走行の内向き符号, 平坦区間の奥行き, ラベル)
        (panel_frames[0], panel_frames[0].near_run_mm, 1.0,
         panel_frames[0].far_run_mm - fold_tangents[0][1] - FLAT_CLEARANCE_MM, "p1"),
        (panel_frames[-1], panel_frames[-1].far_run_mm, -1.0,
         panel_frames[-1].near_run_mm + fold_tangents[-1][0] + FLAT_CLEARANCE_MM, "p2"),
    )
    for frame, end_run, inward, flat_limit, end_label in ends:
        reach = end_run + inward * radius_mm
        if inward > 0 and reach > flat_limit or inward < 0 and reach < flat_limit:
            raise ValueError(
                f"corner relief at the {end_label} end (R={radius_mm:.1f}mm) reaches past the "
                f"flat bearing stretch into the bend fillet. Infeasible."
            )
        for side in (-1, 1):
            if exclude_side is not None and side == exclude_side:
                continue
            # 円弧: 側辺の接点 A=(end+inward*R, side*hw) -> 端辺の接点 B=(end, side*(hw-R))
            # 中心 C=(end+inward*R, side*(hw-R))
            centre_run = end_run + inward * radius_mm
            centre_w = side * (half_width_mm - radius_mm)
            pts = []
            for k in range(ARC_POINTS):
                phi = math.pi / 2.0 * k / (ARC_POINTS - 1)
                run = centre_run - inward * radius_mm * math.sin(phi)
                width = centre_w + side * radius_mm * math.cos(phi)
                pts.append(at(frame, run, width))
            cuts.append(CornerCut(
                arc_points=pts,
                remove_probe=at(frame, end_run, side * half_width_mm),
                label=f"{end_label}/{'+' if side > 0 else '-'}v",
            ))
    return cuts

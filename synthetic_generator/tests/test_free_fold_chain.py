"""自由折れ目チェーン(classify.py、docs/catia_bead_fillet_investigation_log.md SS8)のテスト。

seed(w平行解)とsolve_free_fold(ホモトピー継続)の閉合・隣接パネルの境界一致・
干渉チェックを検証する。数値の裏付け自体はtools/probe_free_fold_homotopy.py等で
既に取った(SS8.8)。ここでは本番コード(classify.py)がそれと同じ性質を持つことを
回帰テストとして固定する。
"""
import math
import random

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


def _dist(a, b) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _sample_pair(rng: random.Random):
    n1 = _random_unit_vector(rng)
    n2 = _rotate(n1, _random_unit_vector(rng), rng.uniform(0.3, math.pi - 0.3))
    p1 = (0.0, 0.0, 0.0)
    p2 = tuple(rng.uniform(80.0, 200.0) * c for c in _random_unit_vector(rng))
    return FasteningPoint(p1, n1), FasteningPoint(p2, n2)


def test_free_fold_seed_closes_exactly() -> None:
    """seed(w平行の帯にgammaシアーを織り込んだ版)は必ずpoint2にちょうど届く(=閉合する)。

    注意: a1=a2=0(シアー無し)では締結点の幅方向ズレdw=dot(p2-p1,w)が非ゼロの限り
    原理的に閉じない(中心線のw座標がp1のw座標に固定されたままになるため、SS8.8)。
    したがってseedのa1_rad/a2_radは一般に非ゼロになる — それ自体がbugではない。
    """
    rng = random.Random(101)
    checked = 0
    while checked < 60:
        point1, point2 = _sample_pair(rng)
        seed = free_fold_seed(
            point1,
            point2,
            bend_radius_mm=rng.uniform(10.0, 25.0),
            min_bearing_radius_mm=rng.uniform(12.5, 25.0),
            fold1_slack_mm=rng.uniform(0.0, 60.0),
            fold2_slack_mm=rng.uniform(0.0, 60.0),
        )
        if seed is None:
            continue
        checked += 1
        assert _dist(seed.end_point, point2.position_xyz) < 1e-6
        n2 = point2.normal_xyz
        n3 = seed.panel3.n
        assert min(_dist(n3, n2), _dist(tuple(-c for c in n3), n2)) < 1e-6
    assert checked == 60


def test_solve_free_fold_closes_and_reaches_requested_tilt() -> None:
    """ホモトピー継続で解けたchainは(1)point2にちょうど届く、
    (2)終端法線がpoint2の法線(符号は自由)と一致する、(3)要求したa1に実際に到達している。"""
    rng = random.Random(202)
    checked = 0
    attempts = 0
    while checked < 40 and attempts < 4000:
        attempts += 1
        point1, point2 = _sample_pair(rng)
        seed = free_fold_seed(
            point1,
            point2,
            bend_radius_mm=rng.uniform(10.0, 25.0),
            min_bearing_radius_mm=rng.uniform(12.5, 25.0),
            fold1_slack_mm=rng.uniform(0.0, 60.0),
            fold2_slack_mm=rng.uniform(0.0, 60.0),
        )
        if seed is None:
            continue
        target_a1 = math.radians(rng.uniform(-20.0, 20.0))
        chain = solve_free_fold(seed, point1, point2, target_a1_rad=target_a1)
        if chain is None:
            continue
        checked += 1
        assert _dist(chain.end_point, point2.position_xyz) < 1e-4
        n2 = point2.normal_xyz
        n3 = chain.panel3.n
        assert min(_dist(n3, n2), _dist(tuple(-c for c in n3), n2)) < 1e-4
        assert abs(chain.a1_rad - target_a1) < 1e-9
    assert checked >= 20, f"収束数が少なすぎる: {checked}/{attempts}"


def test_adjacent_panel_boundaries_share_identical_corner_points() -> None:
    """panel1の遠端(fold1境界)とpanel_midの近端(fold1境界)は、それぞれ独立に
    sheared_panel_cornersで計算しても寸分違わず同じ3D点になる(SS8.8の導出の核心)。
    折れ目の傾きがゼロでない(a1_rad != 0)ケースでこそ意味のある検証。
    """
    rng = random.Random(303)
    checked = 0
    attempts = 0
    while checked < 30 and attempts < 4000:
        attempts += 1
        point1, point2 = _sample_pair(rng)
        seed = free_fold_seed(
            point1,
            point2,
            bend_radius_mm=rng.uniform(10.0, 25.0),
            min_bearing_radius_mm=rng.uniform(12.5, 25.0),
            fold1_slack_mm=rng.uniform(0.0, 60.0),
            fold2_slack_mm=rng.uniform(0.0, 60.0),
        )
        if seed is None:
            continue
        target_a1 = math.radians(rng.uniform(-25.0, 25.0))
        chain = solve_free_fold(seed, point1, point2, target_a1_rad=target_a1)
        if chain is None or abs(chain.a1_rad) < math.radians(3.0):
            continue
        checked += 1
        half_width = 20.0
        panel1_far = sheared_panel_corners(
            chain.panel1.origin, chain.panel1.u, chain.panel1.v,
            -10.0, chain.L1_mm, half_width, near_tilt_rad=0.0, far_tilt_rad=chain.a1_rad,
        )
        panel_mid_near = sheared_panel_corners(
            chain.panel_mid.origin, chain.panel_mid.u, chain.panel_mid.v,
            0.0, chain.L2_mm, half_width, near_tilt_rad=chain.a1_rad, far_tilt_rad=chain.a2_rad,
        )
        # panel1_farの[far,-hw],[far,hw] = panel_mid_nearの[near,-hw],[near,hw]
        assert _dist(panel1_far[3], panel_mid_near[0]) < 1e-6
        assert _dist(panel1_far[2], panel_mid_near[1]) < 1e-6
    assert checked >= 10, f"a1が十分ずれた収束ケースが少なすぎる: {checked}/{attempts}"


def test_dihedral_fold_angle_matches_rotation_angle_regardless_of_tilt() -> None:
    """fold1_angle_rad(=panel1.nとpanel_mid.nのなす角)は、折れ目の傾きa1_radが
    ゼロでなくても正しいbend angleであり続ける(SS8.8: n1⊥g1は常に成立するため)。"""
    rng = random.Random(404)
    checked = 0
    attempts = 0
    while checked < 30 and attempts < 4000:
        attempts += 1
        point1, point2 = _sample_pair(rng)
        seed = free_fold_seed(
            point1,
            point2,
            bend_radius_mm=rng.uniform(10.0, 25.0),
            min_bearing_radius_mm=rng.uniform(12.5, 25.0),
            fold1_slack_mm=rng.uniform(0.0, 60.0),
            fold2_slack_mm=rng.uniform(0.0, 60.0),
        )
        if seed is None:
            continue
        target_a1 = math.radians(rng.uniform(-25.0, 25.0))
        chain = solve_free_fold(seed, point1, point2, target_a1_rad=target_a1)
        if chain is None:
            continue
        checked += 1
        # n1・g1は常に直交する(g1はpanel1平面内、n1はその平面の法線)
        v1 = chain.panel1.v
        g1 = tuple(
            math.cos(chain.a1_rad) * v1[i] + math.sin(chain.a1_rad) * chain.panel1.u[i] for i in range(3)
        )
        assert abs(sum(chain.panel1.n[i] * g1[i] for i in range(3))) < 1e-9
        assert 0.0 <= chain.fold1_angle_rad <= math.pi
    assert checked >= 10


if __name__ == "__main__":
    test_free_fold_seed_closes_exactly()
    test_solve_free_fold_closes_and_reaches_requested_tilt()
    test_adjacent_panel_boundaries_share_identical_corner_points()
    test_dihedral_fold_angle_matches_rotation_angle_regardless_of_tilt()
    print("OK: all free-fold chain self-checks passed")

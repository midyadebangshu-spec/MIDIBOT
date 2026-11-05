import math

import numpy as np

from midibot_locomotion.dcm import capture_step, dcm, dcm_after, foothold_correction, omega
from midibot_locomotion.gait import GaitParams, WalkGenerator

ZC = 0.71


def test_capture_step_stops_the_divergence():
    w = omega(ZC)
    c, v = np.array([0.05, 0.0]), np.array([0.5, 0.0])
    xi = dcm(c, v, ZC)
    stance = np.array([0.0, 0.0])
    foot = capture_step(xi, stance, 0.3, ZC)
    # simulate: ZMP at the stance foot for 0.3 s, then at the new foot
    dt = 0.001
    for _ in range(300):
        a = w * w * (c - stance)
        v += a * dt
        c += v * dt
    # at landing the DCM sits on the new foot; from there it only moves if the ZMP leaves the foot
    assert np.allclose(dcm(c, v, ZC), foot, atol=0.005)
    for _ in range(200):  # another 0.2 s with the ZMP on the new foot: the DCM stays put
        a = w * w * (c - foot)
        v += a * dt
        c += v * dt
    assert np.allclose(dcm(c, v, ZC), foot, atol=0.01)


def test_dcm_after_matches_closed_form():
    xi = dcm_after([0.1, 0.0], [0.0, 0.0], 0.2, ZC)
    assert abs(xi[0] - 0.1 * math.exp(omega(ZC) * 0.2)) < 1e-9


def test_foothold_correction_deadband_and_limit():
    assert not foothold_correction([0.01, 0], [0, 0], ZC, 0.3, 0.5, deadband=0.02).any()
    dp = foothold_correction([0.5, 0], [0, 0], ZC, 0.3, 0.5, limit=0.08)
    assert abs(np.linalg.norm(dp) - 0.08) < 1e-9
    # the longer the wait until touchdown, the larger the shift needed for the same error
    a = np.linalg.norm(foothold_correction([0.01, 0], [0, 0], ZC, 0.1, 0.5, limit=9))
    b = np.linalg.norm(foothold_correction([0.01, 0], [0, 0], ZC, 0.4, 0.5, limit=9))
    assert b > 2 * a


def walking_gen(seconds=3.2):
    """A generator part-way through its first swing (feet 0.15 m apart, walking forward)."""
    gen = WalkGenerator(GaitParams(zc=ZC))
    for _ in range(int(seconds / gen.p.dt)):
        gen.tick((0.1, 0.0, 0.0))
    return gen


def swing_step(gen):
    st = gen.steps[gen.k]
    assert st.kind == "walk"
    return st


def test_adjust_foothold_moves_landing_rate_limited_and_replans():
    gen = WalkGenerator(GaitParams(zc=ZC))
    while gen.steps[gen.k].kind != "walk":
        gen.tick((0.1, 0.0, 0.0))
    st = swing_step(gen)
    nominal = np.array(st.nominal[:2])
    next_before = gen.steps[gen.k + 1].stance_to
    gen.adjust_foothold([0.08, 0.0], rate=0.6)
    moved = np.array(st.target[:2]) - nominal
    assert 0 < moved[0] <= 0.6 * gen.p.dt + 1e-9, "one tick may move the landing by at most rate*dt"
    for _ in range(100):
        gen.adjust_foothold([0.08, 0.0], rate=0.6)
    assert abs((st.target[0] - nominal[0]) - 0.08) < 1e-6
    # the ZMP reference of the next step follows the adjusted foot
    assert gen.steps[gen.k + 1].stance_to != next_before
    assert abs(gen.steps[gen.k + 1].stance_to[0] - st.target[0]) < 1e-9


def test_adjust_foothold_respects_width_and_reach_and_freezes_late():
    gen = WalkGenerator(GaitParams(zc=ZC))
    while gen.steps[gen.k].kind != "walk":
        gen.tick((0.1, 0.0, 0.0))
    st = swing_step(gen)
    stance = "left" if st.swing == "right" else "right"
    sx, sy, _ = gen.feet[stance]
    for _ in range(400):
        gen.adjust_foothold([0.0, 0.5 if st.swing == "right" else -0.5], rate=5.0)  # try to cross the stance foot
    assert abs(st.target[1] - sy) >= gen.p.min_width - 1e-6
    for _ in range(400):
        gen.adjust_foothold([2.0, 0.0], rate=5.0)
    assert np.hypot(st.target[0] - sx, st.target[1] - sy) <= gen.p.max_reach + 1e-6
    # late in the swing the landing no longer moves
    while gen.t < st.t_ds_end + 0.9 * (st.t1 - st.t_ds_end):
        gen.tick((0.1, 0.0, 0.0))
    before = st.target
    gen.adjust_foothold([-0.1, 0.0], rate=5.0)
    assert st.target == before


def test_adjust_foothold_is_ignored_while_standing():
    gen = WalkGenerator(GaitParams(zc=ZC))
    gen.tick((0.0, 0.0, 0.0))
    gen.adjust_foothold([0.05, 0.0])
    gen.dcm_feedback([0.3, 0.0], [0.0, 0.0])  # no swing step: nothing to adjust
    assert gen.steps[gen.k].kind == "hold"

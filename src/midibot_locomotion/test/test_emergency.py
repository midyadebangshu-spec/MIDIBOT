import math

import numpy as np

from midibot_locomotion.gait import GaitParams, WalkGenerator

ZC = 0.71


def gen_in_swing(frac):
    """Walking forward; stop the clock part-way through a swing (frac of the single support)."""
    gen = WalkGenerator(GaitParams(zc=ZC))
    while True:
        gen.tick((0.1, 0.0, 0.0))
        st = gen.steps[gen.k]
        if st.kind == "walk" and gen.t >= st.t_ds_end + frac * (st.t1 - st.t_ds_end):
            return gen


def test_emergency_landing_is_continuous_monotonic_and_ends_feet_together():
    gen = gen_in_swing(0.4)
    st = gen.steps[gen.k]
    swing, stance = st.swing, ("left" if st.swing == "right" else "right")
    before = np.array(gen.state[swing])
    assert before[2] > 0.01, "the foot must be in the air for this test"
    assert gen.emergency_stop(t_land=0.18)
    now = np.array(gen.state[swing])
    assert np.linalg.norm(now[:3] - before[:3]) < 1e-9, "no jump at the moment of the reflex"
    zs, pos = [now[2]], [now[:2]]
    for _ in range(40):
        gen.tick((0.0, 0.0, 0.0))
        s = gen.state[swing]
        zs.append(s[2])
        pos.append(np.array(s[:2]))
    zs = np.array(zs)
    assert (np.diff(zs) <= 1e-9).all(), "the foot only goes down"
    assert np.max(np.linalg.norm(np.diff(np.array(pos), axis=0), axis=1)) < 0.03, "no jumps while landing"
    assert zs[19] < 1e-6 and gen.t - 0.0 > 0  # landed within t_land (+ a tick)
    f_l, f_r = gen.state["left"], gen.state["right"]
    gap = np.array(f_l[:2]) - np.array(f_r[:2])
    assert abs(np.linalg.norm(gap) - gen.p.foot_width) < 1e-6, "feet together at the nominal width"


def test_emergency_then_stand_settles():
    gen = gen_in_swing(0.3)
    gen.emergency_stop()
    for _ in range(int(4.0 / gen.p.dt)):
        gen.tick((0.0, 0.0, 0.0))
    assert gen.settled


def test_emergency_during_weight_shift_becomes_a_stand():
    gen = WalkGenerator(GaitParams(zc=ZC))
    while True:
        gen.tick((0.1, 0.0, 0.0))
        st = gen.steps[gen.k]
        if st.kind == "walk" and gen.t < st.t_ds_end - 0.05:
            break
    assert gen.emergency_stop()
    assert gen.steps[gen.k].kind == "hold"
    for _ in range(int(3.0 / gen.p.dt)):
        gen.tick((0.0, 0.0, 0.0))
    assert gen.settled


def test_emergency_is_a_noop_when_standing_or_repeated():
    gen = WalkGenerator(GaitParams(zc=ZC))
    gen.tick((0.0, 0.0, 0.0))
    assert not gen.emergency_stop()
    g2 = gen_in_swing(0.5)
    assert g2.emergency_stop() and not g2.emergency_stop()

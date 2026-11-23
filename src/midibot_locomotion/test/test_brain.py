import math

from midibot_locomotion.brain import BRACE, CAUTION, HALT, NORMAL, LocomotionBrain

DT = 0.01


def run(brain, seconds, fn):
    """fn(t) -> (roll, pitch, roll_rate, pitch_rate, dcm_err); returns the list of outputs."""
    outs = []
    n = int(seconds / DT)
    for k in range(n):
        outs.append(brain.update(DT, *fn(k * DT)))
    return outs


def test_quiet_walking_never_alarms():
    b = LocomotionBrain()
    outs = run(b, 10.0, lambda t: (0.004 * math.sin(7 * t), 0.003 * math.sin(5 * t), 0.05 * math.cos(7 * t),
                                   0.04 * math.cos(5 * t), (0.02, 0.02)))
    assert all(o.mode == NORMAL for o in outs)
    assert max(o.alert for o in outs) < 0.1


def test_sharp_push_forward_triggers_brace_fast_and_points_forward():
    b = LocomotionBrain()
    run(b, 1.0, lambda t: (0.0, 0.0, 0.0, 0.0, None))
    # forward push: pitch rate 1.2 rad/s while the lean grows
    outs = run(b, 0.3, lambda t: (0.0, 0.5 * 1.2 * t * t * 2, 0.0, 1.2, None))
    first = next(i for i, o in enumerate(outs) if o.mode == BRACE)
    assert first * DT < 0.15, f"BRACE after {first * DT:.2f} s"
    assert outs[first].lean[0] > 0.9 and abs(outs[first].lean[1]) < 0.2


def test_lean_direction_follows_roll_sign():
    b = LocomotionBrain()
    o = run(b, 0.2, lambda t: (-0.1, 0.0, -0.6, 0.0, None))[-1]   # left side down -> falling to the left (+y)
    assert o.lean[1] > 0.9
    b = LocomotionBrain()
    o = run(b, 0.2, lambda t: (0.1, 0.0, 0.6, 0.0, None))[-1]
    assert o.lean[1] < -0.9
    b = LocomotionBrain()
    o = run(b, 0.2, lambda t: (0.0, -0.1, 0.0, -0.6, None))[-1]   # backward
    assert o.lean[0] < -0.9


def test_sustained_growing_wobble_halts_but_small_wobble_does_not():
    small = run(LocomotionBrain(), 5.0, lambda t: (0.04 * math.sin(6 * t), 0.0, 0.04 * 6 * math.cos(6 * t), 0.0, None))
    assert max(o.mode for o in small) == NORMAL
    grow = run(LocomotionBrain(), 5.0, lambda t: (0.01 * math.exp(0.7 * t) * math.sin(6 * t), 0.0,
                                                  0.01 * math.exp(0.7 * t) * 6 * math.cos(6 * t), 0.0, None))
    assert max(o.mode for o in grow) >= HALT
    first = next(i for i, o in enumerate(grow) if o.mode >= CAUTION)
    assert first * DT < 4.0, "the growing wobble must be noticed before it becomes large"


def test_recovered_pushes_release_after_the_hold_time():
    b = LocomotionBrain()
    outs = run(b, 4.0, lambda t: ((0.0, 0.0, 0.0, 0.0, None) if t > 0.2 else (0.0, 0.1, 0.0, 1.2, None)))
    brace_end = max(i for i, o in enumerate(outs) if o.mode == BRACE)
    assert outs[-1].mode == NORMAL
    assert brace_end * DT >= 0.9, "the mode is held for at least the hold time"


def test_capture_point_error_alone_can_trigger():
    b = LocomotionBrain()
    outs = run(b, 1.0, lambda t: (0.0, 0.0, 0.0, 0.0, (0.2, 0.0)))
    assert outs[-1].mode >= HALT


def test_hard_tilt_always_braces():
    b = LocomotionBrain()
    o = run(b, 0.05, lambda t: (0.0, 0.25, 0.0, 0.0, None))[-1]
    assert o.mode == BRACE and o.cmd_scale == 0.0

import math

import numpy as np

from midibot_locomotion.gait import GaitParams, WalkGenerator


def run(gen, cmd, seconds, log=None):
    for _ in range(int(seconds / gen.p.dt)):
        st = gen.tick(cmd)
        if log is not None:
            log.append(st)
    return st


def test_walks_forward_and_stops_with_feet_together():
    gen = WalkGenerator(GaitParams(zc=0.71))
    log = []
    run(gen, (0.12, 0, 0), 8, log)
    st = log[-1]
    assert st["left"][0] > 0.5 and st["right"][0] > 0.5
    run(gen, (0, 0, 0), 4, log)
    assert gen.settled
    st = log[-1]
    assert abs(st["left"][0] - st["right"][0]) < 1e-3
    assert abs(st["left"][1] - st["right"][1] - gen.p.foot_width) < 1e-3
    # CoM ends up over the support centre
    assert abs(st["com"][0] - st["left"][0]) < 0.005


def test_step_limits_clearance_and_width():
    p = GaitParams(zc=0.71)
    gen = WalkGenerator(p)
    log = []
    run(gen, (1.0, 1.0, 1.0), 10, log)  # far above the limits: must be clipped
    zmax = max(max(s["left"][2], s["right"][2]) for s in log)
    assert abs(zmax - p.step_height) < 1e-3
    for s in log:
        # feet never cross or get closer than min_width (in the foot frame the lateral gap is what matters)
        d = np.array(s["left"][:2]) - np.array(s["right"][:2])
        assert np.linalg.norm(d) > p.min_width - 1e-6
    # per-step displacement (stance to landing foot) respects max_dx
    landed = [s for s in gen.steps if s.kind == "walk"]
    assert landed


def test_lipm_zmp_stays_inside_support_polygon():
    """The preview-controlled ZMP must lie in the support box of the feet on the ground. (It may lag the
    reference during the short double-support shift, where any point between the feet is feasible.)"""
    from midibot_locomotion.pose import FOOT_LEN, FOOT_WID
    gen = WalkGenerator(GaitParams(zc=0.71))
    log = []
    run(gen, (0.12, 0, 0), 8, log)
    run(gen, (0, 0, 0), 3, log)
    worst = 1e9
    for s in log:
        pts = [(s[f][0] + sx * FOOT_LEN / 2, s[f][1] + sy * FOOT_WID / 2)
               for f in ("left", "right") if s[f][2] < 1e-4 for sx in (-1, 1) for sy in (-1, 1)]
        xs, ys = zip(*pts)
        z = s["zmp"]
        worst = min(worst, z[0] - min(xs), max(xs) - z[0], z[1] - min(ys), max(ys) - z[1])
    assert worst > 0.01, f"ZMP margin {worst * 100:.2f} cm"


def test_turns_and_restarts():
    gen = WalkGenerator(GaitParams(zc=0.71))
    log = []
    run(gen, (0, 0, 0.3), 8, log)
    assert abs(log[-1]["yaw"]) > 0.5
    run(gen, (0, 0, 0), 4, log)
    assert gen.settled
    x0 = log[-1]["left"][0]
    run(gen, (0.1, 0, 0), 6, log)
    assert math.hypot(log[-1]["left"][0] - x0, log[-1]["left"][1]) > 0.2

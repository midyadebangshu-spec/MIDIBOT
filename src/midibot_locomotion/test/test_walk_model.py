"""Whole-body checks on the real MIDIBOT model: IK convergence, limits and ZMP from inverse dynamics."""
import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")

from midibot_locomotion.pose import FOOT_LEN, FOOT_WID, LEG_JOINTS  # noqa: E402
from midibot_locomotion.walker import WalkPattern, yaw_matrix  # noqa: E402

CASES = {"forward": (0.12, 0, 0), "backward": (-0.08, 0, 0), "turn": (0, 0, 0.3), "side": (0, 0.06, 0)}


def trajectory(model, cmd, walk_s=7.0, total_s=14.0):
    w = WalkPattern(model)
    dt = w.params.dt
    qs, sts, conv = [], [], True
    for k in range(int(total_s / dt)):
        _, st = w.step(cmd if k * dt < walk_s else (0, 0, 0))
        conv &= w.converged
        q = w.q.copy()
        q[0:3] = st["pelvis"]
        q[3:7] = pin.Quaternion(pin.rpy.rpyToMatrix(0, 0, st["yaw"])).coeffs()
        qs.append(q)
        sts.append(st)
    return w, np.array(qs), sts, conv


def zmp_margin(model, qs, sts, dt):
    d = model.createData()
    T = len(qs)
    v = np.array([pin.difference(model, qs[k], qs[k + 1]) / dt for k in range(T - 1)])
    a = np.array([(v[k + 1] - v[k]) / dt for k in range(T - 2)])
    worst = 1e9
    for k in range(1, T - 2):
        tau = pin.rnea(model, d, qs[k], v[k], a[k - 1])
        R = pin.Quaternion(qs[k][3:7]).toRotationMatrix()
        f = R @ tau[:3]
        M = R @ tau[3:6] + np.cross(qs[k][:3], f)
        zmp = np.array([-M[1] / f[2], M[0] / f[2]])
        st = sts[k]
        pts = []
        for s in ("left", "right"):
            if st[s][2] < 1e-4:  # sole on the ground
                x, y, _, yaw = st[s]
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        pts.append(np.array([x, y]) + yaw_matrix(yaw) @ np.array([sx * FOOT_LEN / 2, sy * FOOT_WID / 2]))
        pts = np.array(pts)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        worst = min(worst, zmp[0] - lo[0], hi[0] - zmp[0], zmp[1] - lo[1], hi[1] - zmp[1])
    return worst, v


@pytest.mark.parametrize("name", list(CASES))
def test_walk_is_kinematically_and_dynamically_feasible(model, name):
    w, qs, sts, conv = trajectory(model, CASES[name])
    assert conv, "IK did not converge"
    leg = [model.joints[model.getJointId(j)].idx_q for js in LEG_JOINTS.values() for j in js]
    lo, hi = model.lowerPositionLimit[leg], model.upperPositionLimit[leg]
    margin = min((qs[:, leg] - lo).min(), (hi - qs[:, leg]).min())
    assert margin > 0.02, f"leg joint within {margin:.3f} rad of a limit"
    margin_zmp, v = zmp_margin(model, qs, sts, w.params.dt)
    vleg = [model.joints[model.getJointId(j)].idx_v for js in LEG_JOINTS.values() for j in js]
    assert (np.abs(v[:, vleg]) / model.velocityLimit[vleg]).max() < 0.6
    assert margin_zmp > 0.005, f"ZMP margin {margin_zmp * 100:.2f} cm"
    # the whole-body CoM follows the LIPM CoM that the pattern was built on
    d = model.createData()
    err = max(np.linalg.norm(pin.centerOfMass(model, d, qs[k])[:2] - sts[k]["com"]) for k in range(0, len(qs), 5))
    assert err < 0.002


def test_ready_pose_matches_start_of_walk(model):
    """The first IK output must be (almost) the READY posture so the hand-over is smooth."""
    from midibot_locomotion.pose import READY
    w = WalkPattern(model)
    joints, _ = w.step((0, 0, 0))
    diff = max(abs(joints[j] - READY.get(j, 0.0)) for j in joints if "hip" in j or "knee" in j or "ankle" in j)
    assert diff < 0.05

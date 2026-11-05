"""Base-estimator checks on synthetic sensor data built from the real walking pattern."""
import numpy as np
import pytest

from midibot_locomotion.estimator import BaseEstimator, G

pin = pytest.importorskip("pinocchio")

from midibot_locomotion.walker import WalkPattern, yaw_matrix  # noqa: E402


def rot_z(yaw):
    R2 = yaw_matrix(yaw)
    return np.array([[R2[0, 0], R2[0, 1], 0.0], [R2[1, 0], R2[1, 1], 0.0], [0.0, 0.0, 1.0]])


def synth(model, cmd, walk_s=6.0, total_s=12.0, accel_bias=(0.15, -0.1, 0.05), seed=0):
    rng = np.random.default_rng(seed)
    w = WalkPattern(model)
    dt = w.params.dt
    sts = []
    for k in range(int(total_s / dt)):
        _, st = w.step(cmd if k * dt < walk_s else (0, 0, 0))
        sts.append(st)
    P = np.array([s["pelvis"] for s in sts])
    acc = np.zeros_like(P)
    acc[1:-1] = (P[2:] - 2 * P[1:-1] + P[:-2]) / dt ** 2
    data = []
    for k, st in enumerate(sts):
        R = rot_z(st["yaw"])
        f_body = R.T @ (acc[k] - G) + np.array(accel_bias) + rng.normal(0, 0.05, 3)
        foot_rel = {s: R.T @ (np.array([st[s][0], st[s][1], st[s][2]]) - P[k]) + rng.normal(0, 0.003, 3)
                    for s in ("left", "right")}
        contact = {s: st[s][2] < 1e-4 for s in ("left", "right")}
        data.append((R, f_body, foot_rel, contact))
    return dt, P, np.array([s["yaw"] for s in sts]), data


@pytest.mark.parametrize("name,cmd", [("forward", (0.12, 0, 0)), ("turn", (0, 0, 0.3)), ("side", (0, 0.06, 0))])
def test_tracks_pelvis_through_a_walk(model, name, cmd):
    dt, P, yaw, data = synth(model, cmd)
    est = BaseEstimator()
    est.initialise(data[0][0], data[0][2], p0=P[0])
    err_p, err_v = [], []
    V = np.gradient(P, dt, axis=0)
    for k, (R, f, fr, c) in enumerate(data):
        est.step(dt, R, f, fr, c)
        err_p.append(np.linalg.norm(est.p - P[k]))
        err_v.append(np.linalg.norm(est.v - V[k]))
    assert max(err_p) < 0.02, f"{name}: max position error {max(err_p) * 100:.1f} cm"
    assert np.sqrt(np.mean(np.square(err_v[100:]))) < 0.04, f"{name}: velocity rms {np.sqrt(np.mean(np.square(err_v[100:]))):.3f}"
    assert np.linalg.norm(est.bias - np.array([0.15, -0.1, 0.05])) < 0.08  # accelerometer bias recovered


def test_standing_with_biased_accelerometer_does_not_drift():
    """A biased accelerometer alone would drift quadratically; the foot constraint must hold the base."""
    est = BaseEstimator()
    R = np.eye(3)
    foot_rel = {"left": np.array([0.0, 0.075, -0.74]), "right": np.array([0.0, -0.075, -0.74])}
    est.initialise(R, foot_rel, p0=(0.0, 0.0, 0.74))
    contact = {"left": True, "right": True}
    f = -G + np.array([0.3, 0.0, 0.0])  # at rest + bias
    for _ in range(3000):
        est.step(0.01, R, f, foot_rel, contact)
    assert np.linalg.norm(est.p - np.array([0.0, 0.0, 0.74])) < 0.005
    assert np.linalg.norm(est.v) < 0.01
    assert abs(est.bias[0] - 0.3) < 0.05

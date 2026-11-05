"""Divergent component of motion (capture point) helpers for the LIPM.

xi = c + c_dot / omega,   omega = sqrt(g / zc).  With the ZMP at p:  xi_dot = omega (xi - p), so xi diverges
away from p; the stable part of the CoM follows xi.  Keeping xi under control is what balances the robot.
"""
import math

import numpy as np

G = 9.81


def omega(zc):
    return math.sqrt(G / zc)


def dcm(c, c_dot, zc):
    """Instantaneous capture point of CoM position c and velocity c_dot (any shape, e.g. xy)."""
    return np.asarray(c, dtype=float) + np.asarray(c_dot, dtype=float) / omega(zc)


def dcm_after(xi0, zmp, t, zc):
    """DCM after time t with the ZMP held at `zmp`."""
    e = math.exp(omega(zc) * t)
    return np.asarray(zmp, dtype=float) + (np.asarray(xi0, dtype=float) - np.asarray(zmp, dtype=float)) * e


def capture_step(xi_now, stance, t_land, zc, margin=0.0):
    """Foot position that stops the divergence if it lands at `t_land` while the ZMP stays at `stance`:
    the DCM at landing time. `margin` shifts it further along the motion (safety)."""
    xi = dcm_after(xi_now, stance, t_land, zc)
    d = xi - np.asarray(stance, dtype=float)
    n = np.linalg.norm(d)
    return xi if n < 1e-9 else xi + d / n * margin


def foothold_correction(xi_meas, xi_plan, zc, t_land, t_next, limit=0.10, deadband=0.0):
    """Foothold shift that cancels a DCM tracking error.

    A foothold only changes the ZMP once the foot touches down, so the error is first predicted to the
    landing time (it diverges like exp(omega t) while the ZMP is unchanged): e_land = e exp(omega t_land).
    With the ZMP then shifted by dp for the next support phase of length `t_next`, the error evolves as
    e(t) = dp + (e_land - dp) exp(omega t); it is zero at t_next for dp = e_land / (1 - exp(-omega t_next)).
    Errors inside `deadband` (metres, estimator noise) are ignored; the result is limited to `limit` metres."""
    w = omega(zc)
    e = np.asarray(xi_meas, dtype=float) - np.asarray(xi_plan, dtype=float)
    n = np.linalg.norm(e)
    if n <= deadband:
        return np.zeros_like(e)
    e = e * ((n - deadband) / n)
    dp = e * math.exp(w * max(t_land, 0.0)) / (1.0 - math.exp(-w * max(t_next, 1e-3)))
    n = np.linalg.norm(dp)
    return dp if n <= limit else dp / n * limit

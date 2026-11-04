"""LIPM ZMP preview controller (Kajita et al., ICRA 2003).

Cart-table model: p = c - (zc/g) * c_ddot, with the CoM jerk as the control input. The controller
tracks a future ZMP reference with an integral-of-error term plus state feedback and a preview
of the next `n_preview` reference samples. One instance handles one horizontal axis; walking
uses two (x and y) that share the same gains.
"""
import numpy as np

G = 9.81


class PreviewGains:
    """Solves the discrete algebraic Riccati equation once and keeps the gains."""

    def __init__(self, zc, dt=0.01, preview_time=1.0, qe=1.0, r=1e-6):
        self.zc, self.dt = zc, dt
        self.n_preview = int(round(preview_time / dt))
        A = np.array([[1, dt, dt * dt / 2], [0, 1, dt], [0, 0, 1]])
        B = np.array([[dt ** 3 / 6], [dt * dt / 2], [dt]])
        C = np.array([[1.0, 0.0, -zc / G]])
        self.A, self.B, self.C = A, B, C
        # augmented system with the integral of the ZMP error as an extra state
        Ab = np.block([[np.eye(1), C @ A], [np.zeros((3, 1)), A]])
        Bb = np.vstack([C @ B, B])
        Ib = np.array([[1.0], [0.0], [0.0], [0.0]])
        Q = np.zeros((4, 4))
        Q[0, 0] = qe
        P = Q.copy()
        for _ in range(200000):  # fixed-point iteration of the Riccati recursion
            S = r + (Bb.T @ P @ Bb).item()
            Pn = Q + Ab.T @ P @ Ab - (Ab.T @ P @ Bb) @ (Bb.T @ P @ Ab) / S
            if np.max(np.abs(Pn - P)) < 1e-12:
                P = Pn
                break
            P = Pn
        S = r + (Bb.T @ P @ Bb).item()
        K = (Bb.T @ P @ Ab) / S
        self.gi = K[0, 0]
        self.gx = K[0, 1:]
        Ac = Ab - Bb @ K
        X = -Ac.T @ P @ Ib
        gd = np.empty(self.n_preview)
        gd[0] = -self.gi
        for i in range(1, self.n_preview):
            gd[i] = ((Bb.T @ X) / S).item()
            X = Ac.T @ X
        self.gd = gd


class PreviewAxis:
    """State of one axis: CoM position/velocity/acceleration and the ZMP error integral."""

    def __init__(self, gains, c0=0.0):
        self.g = gains
        self.x = np.array([c0, 0.0, 0.0])
        self.err_sum = 0.0

    @property
    def zmp(self):
        return (self.g.C @ self.x).item()

    def step(self, zmp_ref_window):
        """Advance one sample. `zmp_ref_window[0]` is the reference now, `[1:]` the preview
        (at least `n_preview` further samples; the last value is repeated if shorter)."""
        g = self.g
        ref = np.asarray(zmp_ref_window, dtype=float)
        if len(ref) < g.n_preview + 1:
            ref = np.concatenate([ref, np.full(g.n_preview + 1 - len(ref), ref[-1])])
        self.err_sum += self.zmp - ref[0]
        u = -g.gi * self.err_sum - g.gx @ self.x - g.gd @ ref[1:g.n_preview + 1]
        self.x = g.A @ self.x + g.B[:, 0] * u
        return self.x[0]

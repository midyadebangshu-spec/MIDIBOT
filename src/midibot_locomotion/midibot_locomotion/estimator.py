"""Legged-odometry Kalman filter for the floating base (Bloesch et al., "State estimation for legged robots").

Pure numpy (no ROS, no Pinocchio) so it can be tested offline. The caller supplies the pelvis orientation
(from the IMU), the accelerometer reading, and the sole positions in the pelvis frame from forward kinematics.

State x (15): pelvis position p (3), velocity v (3), left/right sole positions c_l, c_r (3 each), accelerometer
bias b (3), all in the world frame (origin: where the filter was started, z up).

  predict:  a = R (f - b) + g ;  p += v dt + a dt^2 / 2 ;  v += a dt ;  c, b constant
            (foot positions get a small process noise while in contact and a huge one in swing, so a swinging
            foot does not constrain the base)
  update:   for each foot,  R * foot_rel  =  c - p            (kinematic measurement)
            for each stance foot,  c_z = 0                    (flat ground)

Orientation is not estimated here: R comes from the IMU's own attitude filter.
"""
import numpy as np

G = np.array([0.0, 0.0, -9.81])


class BaseEstimator:
    def __init__(self, p0=(0.0, 0.0, 0.0), accel_noise=0.2, accel_bias_walk=0.01, foot_stance_noise=0.002,
                 foot_swing_noise=10.0, kin_noise=0.01, ground_noise=0.005):
        self.x = np.zeros(15)
        self.x[0:3] = p0
        self.P = np.eye(15) * 1e-4
        self.sa, self.sb = accel_noise, accel_bias_walk
        self.sc_stance, self.sc_swing = foot_stance_noise, foot_swing_noise
        self.skin, self.sgnd = kin_noise, ground_noise
        self.initialised = False

    # ------------------------------------------------------------------ properties
    @property
    def p(self):
        return self.x[0:3].copy()

    @property
    def v(self):
        return self.x[3:6].copy()

    @property
    def bias(self):
        return self.x[12:15].copy()

    def foot(self, side):
        i = 6 if side == "left" else 9
        return self.x[i:i + 3].copy()

    # ------------------------------------------------------------------ steps
    def initialise(self, R, foot_rel, p0=None):
        """Anchor the world frame: feet start where forward kinematics puts them."""
        if p0 is not None:
            self.x[0:3] = p0
        self.x[3:6] = 0.0
        self.x[6:9] = self.x[0:3] + R @ foot_rel["left"]
        self.x[9:12] = self.x[0:3] + R @ foot_rel["right"]
        self.initialised = True

    def predict(self, dt, R, f_body, contact):
        """IMU propagation. `f_body`: accelerometer reading (specific force, m/s^2, body frame)."""
        x, P = self.x, self.P
        a = R @ (f_body - x[12:15]) + G
        x[0:3] += x[3:6] * dt + 0.5 * a * dt * dt
        x[3:6] += a * dt
        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        F[0:3, 12:15] = -0.5 * R * dt * dt
        F[3:6, 12:15] = -R * dt
        Q = np.zeros((15, 15))
        Q[0:3, 0:3] = np.eye(3) * (0.5 * self.sa * dt * dt) ** 2
        Q[3:6, 3:6] = np.eye(3) * (self.sa * dt) ** 2
        for side, i in (("left", 6), ("right", 9)):
            s = self.sc_stance if contact.get(side, False) else self.sc_swing
            Q[i:i + 3, i:i + 3] = np.eye(3) * (s ** 2) * dt
        Q[12:15, 12:15] = np.eye(3) * (self.sb ** 2) * dt
        self.P = F @ P @ F.T + Q

    def _update(self, H, z, r_var):
        S = H @ self.P @ H.T + np.eye(len(z)) * r_var
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ (z - H @ self.x)
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ (np.eye(len(z)) * r_var) @ K.T  # Joseph form

    def update(self, R, foot_rel, contact):
        """Kinematic measurements (both feet) and the flat-ground constraint (stance feet)."""
        for side, i in (("left", 6), ("right", 9)):
            H = np.zeros((3, 15))
            H[:, i:i + 3] = np.eye(3)
            H[:, 0:3] = -np.eye(3)
            self._update(H, R @ foot_rel[side], self.skin ** 2)
            if contact.get(side, False):
                Hz = np.zeros((1, 15))
                Hz[0, i + 2] = 1.0
                self._update(Hz, np.array([0.0]), self.sgnd ** 2)

    def step(self, dt, R, f_body, foot_rel, contact):
        if not self.initialised:
            self.initialise(R, foot_rel)
        self.predict(dt, R, f_body, contact)
        self.update(R, foot_rel, contact)
        return self.p, self.v

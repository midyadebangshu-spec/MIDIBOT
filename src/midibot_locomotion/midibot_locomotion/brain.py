"""Fly-inspired locomotion "brain": fall awareness and graded reflex decisions.

This is NOT a model of the real fly connectome. It borrows the fly's *structure* for fast, reliable locomotion
control, in a form small enough to run in the 100 Hz control loop:

  sensory units      gyro-like rate sensing ("halteres"), tilt ("body lean"), capture-point error
                     ("proprioceptive mismatch"). Each is rectified and normalised to 0..1.
  interneuron        THREAT = noisy-OR of the sensory units (any one alarming signal is enough).
  ALERT unit         a leaky integrator of THREAT with a fast attack and a slow decay, like a membrane
                     potential: a brief glitch does not trigger a reflex, a sustained or sharp rise does.
  descending units   SLOW / HALT / BRACE command neurons with hysteresis (Schmitt triggers): they switch on at a
                     high ALERT and off at a lower one, and are held for a minimum time so they do not chatter.

The brain only *watches locomotion*. Its outputs (mode, alert, fall direction, command scale) are used by the
walking controller; it never drives joints itself.

Fall direction. The body's up-axis leans towards the direction the robot is falling. With roll r and pitch p (yaw
removed) the lean vector is (sin p cos r, -sin r) in the heading frame (x forward, y left): pitching forward leans
+x, rolling to the left side down leans +y.
"""
import math
from dataclasses import dataclass, field

NORMAL, CAUTION, HALT, BRACE = 0, 1, 2, 3
MODE_NAMES = {NORMAL: "NORMAL", CAUTION: "CAUTION", HALT: "HALT", BRACE: "BRACE"}


@dataclass
class BrainParams:
    # sensory thresholds: (starts to respond, saturates)
    tilt: tuple = (0.03, 0.12)       # rad of lean (1.7 .. 7 deg)
    rate: tuple = (0.25, 1.0)        # rad/s, lean rate towards the lean direction
    dcm: tuple = (0.06, 0.15)        # m, measured-minus-planned capture point error
    # ALERT unit time constants (s)
    tau_up: float = 0.03
    tau_down: float = 0.30
    # descending units: (switch-on, switch-off) ALERT levels and the minimum hold time (s)
    caution: tuple = (0.25, 0.15)
    halt: tuple = (0.50, 0.30)
    brace: tuple = (0.80, 0.50)
    hold: float = 1.0
    # hard limit: a lean beyond this always means BRACE
    tilt_hard: float = 0.20
    # command scale for each mode
    scale: dict = field(default_factory=lambda: {NORMAL: 1.0, CAUTION: 0.5, HALT: 0.0, BRACE: 0.0})


@dataclass
class BrainOut:
    mode: int = NORMAL
    alert: float = 0.0
    threat: float = 0.0
    sensors: tuple = (0.0, 0.0, 0.0)   # tilt, rate, dcm units
    lean: tuple = (0.0, 0.0)           # unit vector of the fall direction (heading frame), (0, 0) if upright
    lean_mag: float = 0.0              # rad
    cmd_scale: float = 1.0

    @property
    def name(self):
        return MODE_NAMES[self.mode]


def _unit(x, lo, hi):
    return min(max((x - lo) / (hi - lo), 0.0), 1.0)


class LocomotionBrain:
    def __init__(self, params=None):
        self.p = params or BrainParams()
        self.reset()

    def reset(self):
        self.alert = 0.0
        self.mode = NORMAL
        self.t = 0.0
        self.t_mode = -1e9   # time the current mode was entered
        self.out = BrainOut()

    def update(self, dt, roll, pitch, roll_rate, pitch_rate, dcm_err=None):
        """One sample. `roll`, `pitch` in rad, rates in rad/s (body frame), `dcm_err` an (x, y) error in metres or
        None when the capture point is not available. Returns a BrainOut."""
        p = self.p
        self.t += dt
        lean = (math.sin(pitch) * math.cos(roll), -math.sin(roll))
        lean_mag = math.hypot(*lean)
        lean_rate = (pitch_rate, -roll_rate)
        if lean_mag > 1e-3:
            u = (lean[0] / lean_mag, lean[1] / lean_mag)
            rate_out = lean_rate[0] * u[0] + lean_rate[1] * u[1]   # growth of the lean (negative = recovering)
        else:
            u, rate_out = (0.0, 0.0), math.hypot(*lean_rate)
        s_tilt = _unit(lean_mag, *p.tilt)
        s_rate = _unit(rate_out, *p.rate)
        s_dcm = _unit(math.hypot(*dcm_err), *p.dcm) if dcm_err is not None else 0.0
        threat = 1.0 - (1.0 - s_tilt) * (1.0 - s_rate) * (1.0 - s_dcm)
        tau = p.tau_up if threat > self.alert else p.tau_down
        self.alert += (threat - self.alert) * min(dt / tau, 1.0)

        # descending units with hysteresis and a minimum hold time
        a = self.alert
        mode = self.mode
        want = NORMAL
        for m, (on, off) in ((CAUTION, p.caution), (HALT, p.halt), (BRACE, p.brace)):
            level = off if self.mode >= m else on   # already at or above this level: use the lower (off) threshold
            if a >= level:
                want = m
        if lean_mag > p.tilt_hard:
            want = BRACE
        if want > mode:
            mode = want
            self.t_mode = self.t
        elif want < mode and self.t - self.t_mode >= p.hold:
            mode = want
            self.t_mode = self.t
        self.mode = mode
        self.out = BrainOut(mode=mode, alert=self.alert, threat=threat, sensors=(s_tilt, s_rate, s_dcm), lean=u,
                            lean_mag=lean_mag, cmd_scale=p.scale[mode])
        return self.out

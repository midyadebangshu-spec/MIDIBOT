"""Footstep planner + ZMP preview walking pattern generator.

All quantities live in a planning world frame W: origin at the support centre (mid of the soles) when
walking starts, x forward, y left, z up, ground at z = 0. Poses are the *sole centres* and their yaw.

Timeline. The generator works on a list of steps. Step k has
    [t0, t0 + t_ds)   double support: ZMP moves from the old to the new stance foot
    [t0 + t_ds, t1)   single support: ZMP on the stance foot, the other foot swings to its target
A step of kind "hold" has no swing: the ZMP rests at the mid of both feet (standing). The planner
looks `lookahead` steps ahead; a new velocity command changes steps that have not started.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from .preview import PreviewAxis, PreviewGains


@dataclass
class GaitParams:
    zc: float = 0.60            # CoM height above the ground (m)
    dt: float = 0.01            # pattern generator sample time (s)
    t_ss: float = 0.30          # single support time (s)
    t_ds: float = 0.20          # double support time between steps (s)
    t_ds_first: float = 0.60    # longer weight shift before the first step after standing (s)
    t_hold: float = 1.0         # initial standing time before the first step (s)
    step_height: float = 0.03   # swing foot apex (m)
    foot_width: float = 0.15    # nominal distance between the soles' centres (m)
    max_dx: float = 0.12        # per-step limits (m, m, rad)
    max_dy: float = 0.05
    max_dyaw: float = 0.20
    min_width: float = 0.12     # feet never closer than this laterally (m)
    max_vx: float = 0.15        # command limits (m/s, m/s, rad/s)
    max_vy: float = 0.08
    max_wz: float = 0.35
    lookahead: int = 3          # planned steps ahead of the current one
    preview_time: float = 1.0
    cmd_deadband: float = 0.01  # |cmd| below this counts as "stop"
    first_swing: str = "right"


@dataclass
class Step:
    kind: str                   # "hold" | "walk"
    t0: float
    t_ds_end: float
    t1: float
    swing: str = ""             # "left" | "right"
    start: tuple = (0.0, 0.0, 0.0)   # swing foot (x, y, yaw) at the start
    target: tuple = (0.0, 0.0, 0.0)  # swing foot (x, y, yaw) at the end
    stance_from: tuple = (0.0, 0.0)  # ZMP at t0 (xy)
    stance_to: tuple = (0.0, 0.0)    # ZMP during single support (xy)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def _cycloid(s):
    """Position profile with zero velocity at both ends."""
    s = min(max(s, 0.0), 1.0)
    return s - math.sin(2 * math.pi * s) / (2 * math.pi)


class WalkGenerator:
    """Tick-driven walking pattern. Call `tick(cmd)` once per `dt`; read `state`."""

    def __init__(self, params=None):
        self.p = params or GaitParams()
        self.gains = PreviewGains(self.p.zc, self.p.dt, self.p.preview_time)
        self.reset()

    # ------------------------------------------------------------------ setup
    def reset(self, left=None, right=None):
        """Standing start: soles flat at +-foot_width/2 around the origin, CoM at the origin."""
        p = self.p
        w = p.foot_width / 2
        self.feet = {"left": left or (0.0, w, 0.0), "right": right or (0.0, -w, 0.0)}
        self.t = 0.0
        self.k = 0  # index of the current step
        self.x_axis = PreviewAxis(self.gains, self._mid()[0])
        self.y_axis = PreviewAxis(self.gains, self._mid()[1])
        self.steps = [Step("hold", 0.0, 0.0, p.t_hold, stance_from=self._mid(), stance_to=self._mid())]
        self.cmd = (0.0, 0.0, 0.0)
        self.com = np.array([self.x_axis.x[0], self.y_axis.x[0]])
        self.done_walking = True
        self._waiting = False
        self._extend_plan()

    def _mid(self):
        l, r = self.feet["left"], self.feet["right"]
        return ((l[0] + r[0]) / 2, (l[1] + r[1]) / 2)

    # --------------------------------------------------------------- planning
    def _clip_cmd(self, cmd):
        p = self.p
        vx = min(max(cmd[0], -p.max_vx), p.max_vx)
        vy = min(max(cmd[1], -p.max_vy), p.max_vy)
        wz = min(max(cmd[2], -p.max_wz), p.max_wz)
        return vx, vy, wz

    def _moving(self, cmd):
        return max(abs(cmd[0]), abs(cmd[1]), abs(cmd[2])) > self.p.cmd_deadband

    def _plan_next(self, prev, feet):
        """Plan the step after `prev`. `feet` = foot poses after `prev` has finished (updated in place)."""
        p = self.p
        t_step = p.t_ss + p.t_ds
        vx, vy, wz = self._clip_cmd(self.cmd)
        moving = self._moving(self.cmd)
        # swing foot: alternate; the first step swings `first_swing`
        if prev.kind == "walk":
            swing = "left" if prev.swing == "right" else "right"
        else:
            swing = p.first_swing
        stance = "left" if swing == "right" else "right"
        sx, sy, syaw = feet[stance]
        cur = feet[swing]
        mid = ((feet["left"][0] + feet["right"][0]) / 2, (feet["left"][1] + feet["right"][1]) / 2)
        # feet are "together" when the left sole sits one foot_width to the left of the right sole,
        # measured in the right sole's frame (world-axis checks fail once the robot has turned)
        ry = feet["right"][2]
        rel = _rot(-ry) @ np.array([feet["left"][0] - feet["right"][0], feet["left"][1] - feet["right"][1]])
        closed = (abs(rel[0]) < 1e-3 and abs(rel[1] - p.foot_width) < 1e-3
                  and abs(_wrap(feet["left"][2] - feet["right"][2])) < 1e-3)
        if not moving and closed:
            # standing: hold
            t0 = prev.t1
            st = Step("hold", t0, t0, t0 + t_step, stance_from=mid, stance_to=mid)
            return st
        sign = 1.0 if swing == "left" else -1.0
        if moving:
            dx = min(max(vx * t_step, -p.max_dx), p.max_dx)
            dy = min(max(vy * t_step, -p.max_dy), p.max_dy)
            dyaw = min(max(wz * t_step, -p.max_dyaw), p.max_dyaw)
        else:
            dx = dy = dyaw = 0.0  # closing step: bring the swing foot beside the stance foot
        # the foot lands in the stance foot's frame, rotated by the heading change of this step
        yaw_t = _wrap(syaw + dyaw)
        lat = sign * p.foot_width + dy
        if sign * lat < p.min_width:
            lat = sign * p.min_width
        # while turning, a step of the outside foot is longer; keep it simple: both feet use the same dx
        c, s_ = math.cos(syaw), math.sin(syaw)
        # offset expressed in the stance foot frame (x forward, y left)
        ox, oy = dx, lat
        tx = sx + c * ox - s_ * oy
        ty = sy + s_ * ox + c * oy
        if not moving:
            tx, ty, yaw_t = sx - s_ * (sign * p.foot_width), sy + c * (sign * p.foot_width), syaw
        t0 = prev.t1
        t_ds = p.t_ds_first if prev.kind == "hold" else p.t_ds
        st = Step("walk", t0, t0 + t_ds, t0 + t_ds + p.t_ss, swing=swing, start=cur, target=(tx, ty, yaw_t),
                  stance_from=(cur[0], cur[1]) if prev.kind == "walk" else mid,
                  stance_to=(sx, sy))
        return st

    def _feet_after(self, step, feet):
        if step.kind == "walk":
            feet = dict(feet)
            feet[step.swing] = step.target
        return feet

    def _extend_plan(self):
        """Make sure `lookahead` steps exist after the current one (replans the not-yet-started ones)."""
        cur = self.steps[self.k]
        del self.steps[self.k + 1:]
        feet_after = self.feet_after_current()
        prev = cur
        feet = feet_after
        for _ in range(self.p.lookahead):
            nxt = self._plan_next(prev, feet)
            self.steps.append(nxt)
            feet = self._feet_after(nxt, feet)
            prev = nxt

    def feet_after_current(self):
        cur = self.steps[self.k]
        return self._feet_after(cur, self.feet)

    # ------------------------------------------------------------------ ZMP
    def zmp_ref(self, t):
        """Reference ZMP at absolute time t from the planned steps."""
        for st in self.steps:
            if t < st.t1:
                break
        else:
            st = self.steps[-1]
        if st.kind == "hold":
            return st.stance_to
        tau = t - st.t0
        t_ds = st.t_ds_end - st.t0
        if tau < t_ds and t_ds > 0:
            a = tau / t_ds
            a = a * a * (3 - 2 * a)
            return (st.stance_from[0] + a * (st.stance_to[0] - st.stance_from[0]),
                    st.stance_from[1] + a * (st.stance_to[1] - st.stance_from[1]))
        return st.stance_to

    # --------------------------------------------------------------- stepping
    @property
    def stance_foot(self):
        """Foot that carries the weight during the current step's single support ('' for a hold)."""
        st = self.steps[self.k]
        return "" if st.kind != "walk" else ("left" if st.swing == "right" else "right")

    @property
    def waiting_for_load(self):
        """True when the double-support shift is complete and only the load-transfer gate holds the swing."""
        return self._waiting

    def tick(self, cmd=(0.0, 0.0, 0.0), load_ok=True):
        """Advance one sample with the current velocity command (vx, vy, wz).

        `load_ok=False` freezes the pattern clock at the end of a double-support shift (the preview
        controller keeps running with the ZMP on the stance foot) until the stance foot really carries
        the weight, so the swing foot never lifts while the body is still over it."""
        p = self.p
        self.cmd = tuple(cmd)
        cur = self.steps[self.k]
        crossing = self.t < cur.t_ds_end - 1e-6 and self.t + p.dt >= cur.t_ds_end - 1e-6
        self._waiting = (not load_ok) and cur.kind == "walk" and crossing
        n = self.gains.n_preview
        ts = self.t + np.arange(n + 1) * p.dt
        refs = np.array([self.zmp_ref(t) for t in ts])
        self.last_ref = tuple(refs[0])
        self.x_axis.step(refs[:, 0])
        self.y_axis.step(refs[:, 1])
        self.com = np.array([self.x_axis.x[0], self.y_axis.x[0]])
        if not self._waiting:
            self.t += p.dt
        if self.t >= self.steps[self.k].t1 - 1e-9:
            self.feet = self.feet_after_current()
            self.k += 1
            if self.k >= len(self.steps):
                raise RuntimeError("plan exhausted")
            self._extend_plan()
            # drop old history to keep the list short
            if self.k > 4:
                del self.steps[:self.k - 1]
                self.k = 1
        self.done_walking = self.steps[self.k].kind == "hold" and not self._moving(self.cmd)
        return self.state

    @property
    def settled(self):
        """Standing still: a hold step, feet together, CoM at rest over the support centre."""
        m = self._mid()
        return (self.done_walking and np.hypot(self.com[0] - m[0], self.com[1] - m[1]) < 0.003
                and np.hypot(self.x_axis.x[1], self.y_axis.x[1]) < 0.01)

    @property
    def state(self):
        """Current targets: dict with com (xy), left/right sole (x, y, z, yaw), pelvis yaw, phase."""
        st = self.steps[self.k]
        feet = {"left": (*self.feet["left"][:2], 0.0, self.feet["left"][2]),
                "right": (*self.feet["right"][:2], 0.0, self.feet["right"][2])}
        phase = "hold"
        if st.kind == "walk":
            phase = "ds"
            if self.t >= st.t_ds_end:
                phase = "ss"
                s = (self.t - st.t_ds_end) / (st.t1 - st.t_ds_end)
                u = _cycloid(s)
                z = self.p.step_height * 0.5 * (1 - math.cos(2 * math.pi * min(max(s, 0.0), 1.0)))
                x0, y0, a0 = st.start
                x1, y1, a1 = st.target
                feet[st.swing] = (x0 + u * (x1 - x0), y0 + u * (y1 - y0), z, a0 + u * _wrap(a1 - a0))
        yaw = math.atan2(math.sin(feet["left"][3]) + math.sin(feet["right"][3]),
                         math.cos(feet["left"][3]) + math.cos(feet["right"][3]))
        return {"com": self.com.copy(), "left": feet["left"], "right": feet["right"], "yaw": yaw,
                "phase": phase, "zmp": (self.x_axis.zmp, self.y_axis.zmp),
                "zmp_ref": getattr(self, "last_ref", self._mid()), "t": self.t,
                "walking": not self.done_walking}

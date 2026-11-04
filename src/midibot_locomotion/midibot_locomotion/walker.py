"""Walking pattern -> joint targets: gait generator + pelvis placement + leg IK.

The pelvis is held at a constant height and level; its horizontal position is chosen so that the
*whole-body* centre of mass (Pinocchio, with the actual swing-leg posture) follows the LIPM CoM from
the preview controller. The upper body stays in the READY posture.
"""
import math
from dataclasses import replace

import numpy as np
import pinocchio as pin

from .gait import GaitParams, WalkGenerator
from .ik import LegIK
from .pose import BODY_JOINTS, READY


def yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class WalkPattern:
    def __init__(self, model, params=None, com_iterations=2):
        self.model = model
        self.ik = LegIK(model)
        self.com_iterations = com_iterations
        q0 = self.ik.base_q({j: READY.get(j, 0.0) for j in BODY_JOINTS})
        # pelvis height and CoM height of the READY stance (soles on the ground, pelvis level)
        self.pelvis_z = self._sole_height_below_pelvis(q0)
        com0 = self.ik.com(q0)
        zc = self.pelvis_z + com0[2]
        p = params or GaitParams()
        self.params = replace(p, zc=zc)
        self.gen = WalkGenerator(self.params)
        self.q = q0
        self.q_ready = q0.copy()
        self.converged = True
        self.reset()

    def _sole_height_below_pelvis(self, q):
        pin.forwardKinematics(self.model, self.ik.data, q)
        pin.updateFramePlacements(self.model, self.ik.data)
        M = self.ik.data.oMf[self.ik.frame["left"]]
        return -(M.translation[2] + (M.rotation @ np.array([0.0, 0.0, -0.06]))[2])

    def reset(self, q_current=None):
        """Restart standing at the origin. The pelvis starts above the support centre."""
        self.gen.reset()
        self.q = self.q_ready.copy() if q_current is None else q_current.copy()

    def step(self, cmd):
        """One pattern sample. Returns (joint dict for BODY_JOINTS, generator state)."""
        st = self.gen.tick(cmd)
        com_xy = st["com"]
        yaw = st["yaw"]
        R2 = yaw_matrix(yaw)
        pelvis_xy = com_xy.copy()
        q = self.q
        ok = True
        for _ in range(max(1, self.com_iterations)):
            pelvis = np.array([pelvis_xy[0], pelvis_xy[1], self.pelvis_z])
            Rw = np.array([[R2[0, 0], R2[0, 1], 0], [R2[1, 0], R2[1, 1], 0], [0, 0, 1]])
            targets = {}
            for s in ("left", "right"):
                x, y, z, a = st[s]
                targets[s] = (Rw.T @ (np.array([x, y, z]) - pelvis), a - yaw)
            ok &= self.ik.solve(q, targets)
            c_p = self.ik.com(q)
            pelvis_xy = com_xy - R2 @ c_p[:2]
        self.converged = ok
        self.q = q
        out = {}
        for jid in range(2, self.model.njoints):
            n = self.model.names[jid]
            if n in BODY_JOINTS:
                out[n] = float(q[self.model.joints[jid].idx_q])
        st = dict(st)
        st["pelvis"] = np.array([pelvis_xy[0], pelvis_xy[1], self.pelvis_z])
        return out, st

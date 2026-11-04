"""Leg inverse kinematics for walking (damped least squares on the Pinocchio model).

Everything is solved in the pelvis frame (floating base at the identity): the foot targets are the
planned sole poses expressed relative to the planned pelvis pose.
"""
import numpy as np
import pinocchio as pin

from .pose import ANKLE_H, FOOT_X, LEG_JOINTS

SOLE_IN_FOOT = np.array([FOOT_X, 0.0, -ANKLE_H])  # sole centre in the foot frame


class LegIK:
    def __init__(self, model, damping=1e-4, tol=1e-6, max_iter=30):
        self.model = model
        self.data = model.createData()
        self.damping, self.tol, self.max_iter = damping, tol, max_iter
        self.frame = {s: model.getFrameId(f"{s}_foot") for s in ("left", "right")}
        self.idx_q, self.idx_v = {}, {}
        for s, names in LEG_JOINTS.items():
            ids = [model.getJointId(n) for n in names]
            self.idx_q[s] = [model.joints[i].idx_q for i in ids]
            self.idx_v[s] = [model.joints[i].idx_v for i in ids]
        self.lo, self.hi = model.lowerPositionLimit, model.upperPositionLimit
        self.last_iters = 0

    def base_q(self, q_joints):
        """Full configuration with the base at the identity. `q_joints` maps joint name -> angle."""
        q = pin.neutral(self.model)
        for jid in range(2, self.model.njoints):
            n = self.model.names[jid]
            if n in q_joints:
                q[self.model.joints[jid].idx_q] = q_joints[n]
        return q

    def solve(self, q, targets):
        """Move the leg joints of `q` (in place) so each sole matches `targets[side] = (pos3, yaw)` given in
        the pelvis frame (sole centre position and flat-foot yaw about z, plus optional (roll, pitch)).
        Returns True if both legs converged."""
        m, d = self.model, self.data
        goals = {}
        for s, (pos, yaw) in targets.items():
            R = pin.rpy.rpyToMatrix(0.0, 0.0, yaw)
            goals[s] = (np.asarray(pos) - R @ SOLE_IN_FOOT, R)
        ok = {s: False for s in goals}
        for it in range(self.max_iter):
            pin.forwardKinematics(m, d, q)
            pin.updateFramePlacements(m, d)
            all_ok = True
            for s, (p_goal, R_goal) in goals.items():
                M = d.oMf[self.frame[s]]
                e = np.concatenate([p_goal - M.translation, pin.log3(R_goal @ M.rotation.T)])
                if np.linalg.norm(e) < self.tol:
                    ok[s] = True
                    continue
                all_ok = False
                ok[s] = False
                J = pin.computeFrameJacobian(m, d, q, self.frame[s], pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                J = J[:, self.idx_v[s]]
                dq = J.T @ np.linalg.solve(J @ J.T + self.damping * np.eye(6), e)
                for i, qi in enumerate(self.idx_q[s]):
                    q[qi] = min(max(q[qi] + dq[i], self.lo[qi]), self.hi[qi])
            if all_ok:
                self.last_iters = it
                return True
        self.last_iters = self.max_iter
        return all(ok.values())

    def com(self, q):
        return pin.centerOfMass(self.model, self.data, q).copy()

import numpy as np


class PDController:
    def __init__(self, _skel, _kp, _kd):
        self.skel = _skel
        self.ndofs = self.skel.ndofs
        self.kp = np.array( [_kp] * self.ndofs )
        self.kd = np.array( [_kd] * self.ndofs )
        self.target = None

    def control(self):
        q = self.skel.q
        qdot = self.skel.qdot
        
        tau = np.zeros( self.ndofs )
        for i in range(6, self.ndofs):
            tau[i] = -self.kp[i] * (q[i] - self.target[i]) - self.kd[i] * qdot[i]
            # tau[i] = confine(tau[i], -self.maxTorque, self.maxTorque)
        return tau


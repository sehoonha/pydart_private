import numpy as np

def confine(x, lo, hi):
    return min(max(lo, x), hi)

class PDController:
    def __init__(self, _ndofs, _kp, _kd):
        self.ndofs = _ndofs
        self.kp = np.array( [_kp] * self.ndofs )
        self.kd = np.array( [_kd] * self.ndofs )
        self.maxTorque = 0.3 * 1.5
        self.target = None


    def control(self, q, qdot):
        tau = np.zeros( self.ndofs )
        for i in range(6, self.ndofs):
            tau[i] = -self.kp[i] * (q[i] - self.target[i]) - self.kd[i] * qdot[i]
            tau[i] = confine(tau[i], -self.maxTorque, self.maxTorque)
        return tau


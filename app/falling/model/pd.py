import numpy as np


def confine(x, lo, hi):
    return min(max(lo, x), hi)


class PDController:
    def __init__(self, _skel, _kp, _kd, _maxTorque):
        self.skel = _skel
        self.ndofs = self.skel.ndofs
        self.kp = np.array([_kp] * self.ndofs)
        self.kd = np.array([_kd] * self.ndofs)
        self.target = None
        self.maxTorque = _maxTorque
        self.step_counter = 0  # For debug

    def verbose(self):
        return False
        # return (self.step_counter % 100 == 0)

    def control(self):
        q = self.skel.q
        qdot = self.skel.qdot

        tau = np.zeros(self.ndofs)
        tau_lo = self.skel.tau_lo
        tau_hi = self.skel.tau_hi
        if self.verbose():
            print 'step!!!'
        for i in range(6, self.ndofs):
            tau[i] = -self.kp[i] * (q[i] - self.target[i]) \
                     - self.kd[i] * qdot[i]
            # tau[i] = confine(tau[i], -self.maxTorque, self.maxTorque)
            tau[i] = confine(tau[i], tau_lo[i], tau_hi[i])
            if self.verbose():
                print i, "%.4f %.4f %.4f %.4f" % (q[i], self.target[i],
                                                  q[i] - self.target[i],
                                                  tau[i])
        self.step_counter += 1
        return tau

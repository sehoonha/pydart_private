import numpy as np

class JTController:
    def __init__(self, _skel):
        self.skel = _skel

    def control(self, bodynames, f):
        if not isinstance(bodynames, list):
            bodynames = [bodynames]

        tau = np.zeros(self.skel.ndofs)
        for bodyname in bodynames:
            # J = self.skel.getBodyNodeWorldLinearJacobian(bodyname)
            J = self.skel.body(bodyname).world_linear_jacobian()
            JT = np.transpose(J)
            tau += JT.dot(f)
        return tau



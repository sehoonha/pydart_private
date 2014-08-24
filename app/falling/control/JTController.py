import numpy as np

class JTController:
    def __init__(self, _world):
        self.world = _world

    def control(self, bodynames, f):
        if not isinstance(bodynames, list):
            bodynames = [bodynames]

        tau = np.zeros(self.world.ndofs)
        for bodyname in bodynames:
            J = self.world.getBodyNodeWorldLinearJacobian(bodyname)
            JT = np.transpose(J)
            tau += JT.dot(f)
        return tau



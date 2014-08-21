import numpy as np

class JTController:
    def __init__(self, _world):
        self.world = _world

    def control(self, bodyname, f):
        J = self.world.getBodyNodeWorldLinearJacobian(bodyname)
        JT = np.transpose(J)
        return JT.dot(f)



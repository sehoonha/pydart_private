import numpy as np
from numpy.linalg import norm

from scipy.optimize import minimize


class ObjTIP:
    def __init__(self, _tip):
        self.target = [0.15, 0.15, 1.57]
        # self.target = [0.104, 0.13, 1.27]
        self.tip = _tip

    def cost(self):
        state = self.tip.getState()
        return norm(state - self.target) ** 2
        
class IK:
    def __init__(self, _world):
        self.world = _world
        self.res = None

        # Dimensions and weights
        self.dim = 5
        self.weights = np.array( [1.0, 1.0, 0.5, 1.0, 0.05] )

        self.objs = [ ObjTIP(self.world.tip) ]

    def expand(self, x):
        q = self.world.getPositions()
        return list(q[0:6]) + [ 0.0, x[0], 0.0, x[0], x[1], 0.0, x[1], 0.0 ] + [ x[2], x[3], x[2], x[3], x[4], x[4], 0.0, 0.0 ]

    # def getCOMToBodyPoint(self):
    #     return self.objs[0]

    def evaluate(self, x = None):
        if x is not None:
            # 0 : shoulders 1 : hips 2: knees 3: hands 4: ankles
            self.world.setPositions( self.expand(x * self.weights) )
                       
        return sum([ obj.cost() for obj in self.objs])


    def optimize(self, x0 = None, restore = True):
        saved_pose = self.world.getPositions()
        
        if x0 is None:
            x0 = np.zeros(self.dim)
        self.res = minimize(lambda x : self.evaluate(x), x0, method='nelder-mead')

        print self.res
        if restore:
            self.world.setPositions(saved_pose)

        return self.expand(self.res["x"])
            

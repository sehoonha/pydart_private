import numpy as np
import numpy.linalg as LA

from scipy.optimize import minimize

class COMToBodyPoint:
    def __init__(self, _world, _bodynames, _local):
        self.world = _world
        self.target = 0.10
        self.bodynames = _bodynames
        self.local = _local
        
    def cost(self):
        C = self.world.getCOM()

        positions = []
        for bodyname in self.bodynames:
            T = self.world.getBodyNodeTransformation(bodyname)
            P = T.dot( np.append(self.local, [1.0]) )
            positions += [P]
        avg = (sum(positions) / len(positions))[0:3]
        diff = (LA.norm(C - avg) - self.target) ** 2
        return diff

class IK:
    def __init__(self, _world):
        self.world = _world
        self.res = None

        # Dimensions and weights
        self.dim = 5
        self.weights = np.array( [1.0, 1.0, 0.5, 1.0, 0.05] )

        # Objectives
        self.objs = [ COMToBodyPoint(self.world, ["l_foot", "r_foot"], np.array([-0.05, 0.025, 0.0]) ) ]

    def expand(self, x):
        q = self.world.getPositions()
        return list(q[0:6]) + [ 0.0, x[0], 0.0, x[0], x[1], 0.0, x[1], 0.0 ] + [ x[2], x[3], x[2], x[3], x[4], x[4], 0.0, 0.0 ]

    def getCOMToBodyPoint(self):
        return self.objs[0]

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
            

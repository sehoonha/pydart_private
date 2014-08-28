import numpy as np
from numpy.linalg import norm

from scipy.optimize import minimize


class ObjTIP:
    def __init__(self, _tip):
        # self.target = [0.15, 0.15, 1.57]
        # self.target = [0.091406, 0.08, 2.6367]
        self.target = [0.17, 0.17, 2.6367]
        self.tip = _tip

    def cost(self):
        state = self.tip.get_state()
        return norm( (state - self.target) * [1.0, 1.0, 0.1] ) ** 2
        
class IK:
    def __init__(self, _sim):
        self.sim = _sim

        self.res = None

        # Dimensions and weights
        self.dim = 5
        self.weights = np.array( [1.0, 1.0, 0.5, 1.0, 0.05] )

        self.objs = [ ObjTIP(self.sim.tip) ]
        self.objs[0].target = self.sim.abstract_tip.commands()
        print 'objs[0].target = ', self.objs[0].target
        
    def expand(self, x):
        q = self.sim.skel.q
        return list(q[0:6]) + [ 0.0, x[0], 0.0, x[0], x[1], 0.0, x[1], 0.0 ] + [ x[2], x[3], x[2], x[3], x[4], x[4], 0.0, 0.0 ]

    # def getCOMToBodyPoint(self):
    #     return self.objs[0]

    def evaluate(self, x = None):
        if x is not None:
            # 0 : shoulders 1 : hips 2: knees 3: hands 4: ankles
            self.sim.skel.q = self.expand(x * self.weights)
                       
        return sum([ obj.cost() for obj in self.objs])


    def optimize(self, x0 = None, restore = True):
        saved_pose = self.sim.skel.q
        
        if x0 is None:
            x0 = np.zeros(self.dim)

        print "==== ik.IK optimize...."
        self.res = minimize(lambda x : self.evaluate(x), x0, method='nelder-mead')

        print "==== result\n", self.res
        print "==== ik.IK optimize....OK"
        if restore:
            self.sim.skel.q = saved_pose

        return self.expand(self.res["x"])
            

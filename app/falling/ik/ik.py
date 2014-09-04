import numpy as np
from numpy.linalg import norm

from scipy.optimize import minimize
import cma

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

    def evaluate_fullbody(self, x):
        self.sim.pd.target = self.expand(x)
        self.sim.reset()
        while not self.sim.step():
            pass
        # v = -self.sim.skel.Cdot[1]
        v = -self.sim.skel.C[2]

        contacts = set(self.sim.skel.contacted_body_names())
        allowed  = set(['l_foot', 'r_foot', 'l_hand', 'r_hand'])
        if len(contacts - allowed) > 0:
            v += 10.0
        print x, " --> ", v
        return v 

    def optimize_with_fullbody_motion(self):
        # print self.evaluate_fullbody([ 1., 0.09637636, 0.93696491, -0.92534248, -0.77468035])
        print self.evaluate_fullbody([ 0.80067996,  0.91354763,  0.87398188, -0.5739955,  -0.05695692]) # Min dist touch
        # print self.evaluate_fullbody([0.99941362, -0.63121823, 0.58667322, 0.64432841, -0.79191961]) # Max dist touch

        # lo = np.array([-1.0] * 5)
        # hi = np.array([ 1.0] * 5)
        # opt = {'verb_time':0,  'boundary_handling': 'BoundPenalty', \
        #        'bounds': [lo, hi], 'tolfun' : 0.001}
        # print "==== abstract.model.TIP optimize...."
        # self.res = cma.fmin(self.evaluate_fullbody, 0.5 * (lo + hi), 1.0, opt)

        # self.control = self.res[0]

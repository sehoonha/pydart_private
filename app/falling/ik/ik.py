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
        # self.weights = np.array( [1.0, 1.0, 0.5, 1.0, 0.05] )

        self.param_desc = [ (0, 'l_shoulder', 1.0),
                            (0, 'r_shoulder', 1.0),
                            (1, 'l_hand', 1.0),
                            (1, 'r_hand', 1.0),
                            (2, 'l_thigh', 1.0),
                            (3, 'r_thigh', 1.0),
                            (4, 'l_shin', 0.5),
                            (5, 'r_shin', 0.5),
                            (6, 'l_heel', 0.05),
                            (7, 'r_heel', 0.05) ]
                                    
        self.dim = max([i for i, dof, w in self.param_desc]) + 1

        self.objs = [ ObjTIP(self.sim.tip) ]
        self.objs[0].target = self.sim.abstract_tip.commands()
        print 'objs[0].target = ', self.objs[0].target

        # self.objs = [ ObjTIP(self.sim.tip) ]
        # self.objs[0].target = [0.14, 0.08, 2.7]
        # print 'objs[0].target = ', self.objs[0].target

        # self.objs += [ ObjTIP(self.sim.tip2) ]
        # self.objs[1].target = [0.08, 0.17, 1.0]
        # print 'objs[1].target = ', self.objs[1].target

        
    def expand(self, x):
        q = self.sim.skel.q
        for x_i, dof_name, w in self.param_desc:
            dof_i = self.sim.skel.dof_index(dof_name)
            q[dof_i] = w * x[x_i]
        return q

    def evaluate(self, x = None):
        if x is not None:
            if np.max(np.fabs(x)) > 1.0:
                return np.max(np.fabs(x))
            self.sim.skel.q = self.expand(x)
                       
        return sum([ obj.cost() for obj in self.objs])


    def optimize(self, x0 = None, restore = True):
        saved_pose = self.sim.skel.q
        
        if x0 is None:
            x0 = np.zeros(self.dim)

        print "==== ik.IK optimize...."
        self.res = minimize(lambda x : self.evaluate(x), x0, method='nelder-mead', tol=0.000001, options={'maxiter':3000})

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

        data = self.sim.history.get_frame_at(-2)

        # v = -data['P.y']
        # v = sum(self.sim.history.vertical_impulses())
        v = -data['C.y']

        contacts = set(self.sim.skel.contacted_body_names())
        allowed  = set(['l_foot', 'r_foot', 'l_hand', 'r_hand'])
        if len(contacts - allowed) > 0:
            v += 10.0
        print repr(x), " --> ", v
        return v 

    def optimize_with_fullbody_motion(self):
        # print self.evaluate_fullbody([ 0.98677002,  0.79964316,  0.58797879, -0.63084664, -0.53535414]) # Max P.y
        # print self.evaluate_fullbody([0.05952294,  0.27164898,  1.        , -0.6327223 , -0.67808456]) # Min Impulse

        # print self.evaluate_fullbody([ 0.80067996,  0.91354763,  0.87398188, -0.5739955,  -0.05695692]) # Min dist touch
        # print self.evaluate_fullbody([0.99941362, -0.63121823, 0.58667322, 0.64432841, -0.79191961]) # Max dist touch


        lo = np.array([-1.0] * 5)
        hi = np.array([ 1.0] * 5)
        opt = {'verb_time':0,  'boundary_handling': 'BoundPenalty', \
               'bounds': [lo, hi], 'tolfun' : 0.001}
        print "==== abstract.model.TIP optimize...."
        self.res = cma.fmin(self.evaluate_fullbody, 0.5 * (lo + hi), 1.0, opt)

        self.control = self.res[0]

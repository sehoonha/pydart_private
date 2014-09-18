import numpy as np
from numpy.linalg import norm

from scipy.optimize import minimize
import cma


class ObjTIP:
    def __init__(self, _tip):
        self.target = None
        self.tip = _tip

    def state(self):
        return self.tip.get_state()

    def cost(self):
        state = self.tip.get_state()
        return norm((state - self.target) * [1.0, 1.0, 0.1] ) ** 2


class ObjTWOTIP:
    def __init__(self, _tips):
        self.target = None
        self.tips = _tips

    def state(self):
        th1 = self.tips[0].theta()
        state = np.concatenate([[th1]] + [t.get_state() for t in self.tips])
        return state

    def cost(self):
        w = [10.0, 1.0, 1.0, 0.5, 1.0, 1.0, 0.5]
        return norm((self.state() - self.target) * w) ** 2


class IK:
    def __init__(self, _sim):
        self.sim = _sim

        self.res = None

        # Dimensions and weights
        # self.weights = np.array( [1.0, 1.0, 0.5, 1.0, 0.05] )

        # self.param_desc = [(0, 'l_shoulder', 1.0),
        #                    (1, 'r_shoulder', 1.0),
        #                    (2, 'l_hand', 1.0),
        #                    (3, 'r_hand', 1.0),
        #                    (4, 'l_thigh', 1.0),
        #                    (5, 'r_thigh', 1.0),
        #                    (6, 'l_shin', 0.5),
        #                    (7, 'r_shin', 0.5),
        #                    (8, 'l_heel', 0.05),
        #                    (9, 'r_heel', 0.05) ]

        self.param_desc = [(0, 'l_hip', 1.0),
                           (1, 'r_hip', 1.0),
                           (2, 'l_shoulder', 1.0),
                           (3, 'r_shoulder', 1.0),
                           (4, 'l_hand', 1.0),
                           (5, 'r_hand', 1.0),
                           (6, 'l_thigh', 1.0),
                           (7, 'r_thigh', 1.0),
                           (8, 'l_shin', 0.5),
                           (9, 'r_shin', 0.5),
                           (10, 'l_heel', 0.05),
                           (11, 'r_heel', 0.05) ]

        self.dim = max([i for i, dof, w in self.param_desc]) + 1

        # self.objs = [ObjTIP(self.sim.tip)]
        # self.objs[0].target = self.sim.abstract_tip.commands()
        # print 'objs[0].target = ', self.objs[0].target

        # self.objs = [ ObjTIP(self.sim.tip) ]
        # self.objs[0].target = [0.14, 0.08, 2.7]
        # print 'objs[0].target = ', self.objs[0].target

        # self.objs += [ ObjTIP(self.sim.tip2) ]
        # self.objs[1].target = [0.08, 0.17, 1.0]
        # print 'objs[1].target = ', self.objs[1].target

        # self.objs = [ ObjTWOTIP(self.sim.tips) ]
        # self.objs[0].target = self.sim.abstract_twotip.commands()
        # print 'objs[0].target = ', self.objs[0].target

    def expand(self, x):
        q = self.sim.skel.q
        for x_i, dof_name, w in self.param_desc:
            dof_i = self.sim.skel.dof_index(dof_name)
            q[dof_i] = w * x[x_i]
        return q

    def evaluate_orientation(self, x=None):
        q = self.sim.skel.q
        q[0:3] = x
        self.sim.skel.q = q
        lhs = self.sim.skel.body("torso").T
        lhs[0][3] = lhs[1][3] = lhs[2][3] = 0.0
        rhs = np.array([[1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]])
        return norm(lhs - rhs)

    def evaluate(self, x=None):
        if x is not None:
            if np.max(np.fabs(x)) > 1.57:
                return np.max(np.fabs(x))
            self.sim.skel.q = self.expand(x)

        return sum([obj.cost() for obj in self.objs])

    def optimize(self, x0=None, restore=True):
        saved_pose = self.sim.skel.q
        saved_vel = self.sim.skel.qdot

        self.dim = 3

        if x0 is None:
            # x0 = np.zeros(self.dim)
            x0 = np.random.rand(self.dim)

        print "==== ik.IK optimize...."

        # self.res = { "x" : x, "value" : self.evaluate(x) }
        # self.res = minimize(lambda x: self.evaluate_orientation(x), x0,
        self.res = minimize(lambda x: self.evaluate(x), x0,
                            method='nelder-mead', tol=0.000001,
                            options={'maxiter': 10000, 'maxfev': 10000,
                                     'xtol': 10e-8, 'ftol': 10e-8})

        print "==== result\n", self.res
        print "state = ", self.objs[0].state()
        print "==== ik.IK optimize....OK"

        if restore:
            self.sim.skel.q = saved_pose
            self.sim.skel.qdot = saved_vel

        return self.expand(self.res["x"])

    def evaluate_fullbody(self, x):
        self.sim.reset()
        self.sim.pd.target = self.expand(x)
        # self.sim.reset()
        while not self.sim.step():
            pass

        data = self.sim.history.get_frame_at(-2)

        # v = -data['P.y']
        # v = sum(self.sim.history.vertical_impulses())
        # v = -data['C.y']
        v = data['max_impulse']

        # contacts = set(self.sim.skel.contacted_body_names())
        # allowed  = set(['l_foot', 'r_foot', 'l_hand', 'r_hand'])
        # if len(contacts - allowed) > 0:
        #     v += 10.0
        print repr(x), " --> ", v
        return v

    def optimize_with_fullbody_motion(self):
        # x = np.random.rand(self.dim)
        x = np.array([-0.52563663, -1.57, -1.57, 0.91604437, -0.67324443,
                      -0.43524684, -0.2820, 0.67682579, -1.199266, -0.864372,
                      -1.57, 0.79398899])  # 0.476501049002
        return self.evaluate_fullbody(x)

        lo = np.array([-1.57] * self.dim)
        hi = np.array([1.57] * self.dim)
        opt = {'verb_time': 0, 'boundary_handling': 'BoundPenalty',
               'bounds': [lo, hi], 'tolfun': 0.001}
        print "==== abstract.model.TIP optimize...."
        self.res = cma.fmin(self.evaluate_fullbody, 0.5 * (lo + hi), 1.0, opt)

        self.control = self.res[0]

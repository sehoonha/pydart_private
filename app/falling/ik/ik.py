import numpy as np
from numpy.linalg import norm

from scipy.optimize import minimize
import cma


class ObjC:
    def __init__(self, _skel, _target):
        self.skel = _skel
        self.target = _target

    def cost(self):
        C = self.skel.C
        C = np.array([C[2], C[1]])
        return norm((C - self.target) * [2.0, 2.0] ) ** 2


class ObjPt:
    def __init__(self, _con, _target):
        self.con = _con
        self.target = _target

    def cost(self):
        P = self.con.p
        P = np.array([P[2], P[1]])
        return norm((P - self.target) * [1.0, 1.0] ) ** 2


class IK:
    def __init__(self, _sim, _plan):
        self.sim = _sim
        self.plan = _plan

        self.res = None

        # Dimensions and weights
        # self.weights = np.array( [1.0, 1.0, 0.5, 1.0, 0.05] )

        self.param_desc = [(0, 'l_shoulder', 1.0),
                           (0, 'r_shoulder', 1.0),
                           (1, 'l_hand', 1.0),
                           (1, 'r_hand', 1.0),
                           (2, 'l_thigh', 1.0),
                           (3, 'r_thigh', 1.0),
                           (4, 'l_shin', 0.5),
                           (5, 'r_shin', 0.5),
                           (6, 'l_heel', 0.5),
                           (7, 'r_heel', 0.05) ]

        self.dim = max([i for i, dof, w in self.param_desc]) + 1

        skel = self.sim.skel
        plan = self.plan
        prob = self.sim.prob

        self.objs = []
        self.objs += [ObjC(skel, plan.C())]
        self.objs += [ObjPt(prob.contact('r_toe'), plan.P(0))]
        self.objs += [ObjPt(prob.contact('l_heel'), plan.P(1))]
        self.objs += [ObjPt(prob.contact('l_toe'), plan.P(2))]

        print 'costs = ', [o.cost() for o in self.objs]

    def expand(self, x):
        q = self.sim.skel.q
        for x_i, dof_name, w in self.param_desc:
            dof_i = self.sim.skel.dof_index(dof_name)
            q[dof_i] = w * x[x_i]
        return q

    def evaluate(self, x=None):
        if x is not None:

            q = self.expand(x)
            dq = q - self.q0

            # Check the pose
            if np.max(np.fabs(q)) > 1.57:
                return np.max(np.fabs(q))
            # Check the change of pose
            if np.max(np.fabs(dq)) > 0.8:
                return 10.0 * np.max(np.fabs(dq))

            self.sim.skel.q = q

        costs = [obj.cost() for obj in self.objs]
        # print sum(costs), costs
        return sum(costs)

    def optimize(self, restore=True):
        saved_pose = self.sim.skel.q
        saved_vel = self.sim.skel.qdot

        self.q0 = self.sim.skel.q

        print "==== ik.IK optimize...."
        self.res = None
        for i in range(10):
            x0 = np.random.rand(self.dim)
            res = minimize(lambda x: self.evaluate(x), x0,
                           method='nelder-mead', tol=0.00001,
                           # method='SLSQP', tol=0.00001,
                           options={'maxiter': 10000, 'maxfev': 10000,
                                    'xtol': 10e-8, 'ftol': 10e-8})
            if self.res is None or res['fun'] < self.res['fun']:
                self.res = res
            print i, self.res['fun']

        self.sim.skel.q = self.expand(self.res['x'])

        print "==== result\n", self.res
        print 'final costs = ', [obj.cost() for obj in self.objs]
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

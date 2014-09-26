import numpy as np
from numpy.linalg import norm, det

from scipy.optimize import minimize
import cma


class ObjTIP:
    def __init__(self, _tip):
        self.target = None
        self.tip = _tip

    def state(self):
        th1 = self.tip.th1
        return np.concatenate([[th1], self.tip.pose()])

    def cost(self):
        state = self.state()
        return norm((state - self.target) * [10.0, 1.0, 1.0, 0.1] ) ** 2


class ObjTWOTIP:
    def __init__(self, _tips):
        self.target = None
        self.tips = _tips

    def state(self):
        th1 = self.tips[0].th1
        state = np.concatenate([[th1]] + [t.pose() for t in self.tips])
        return state

    def cost(self):
        w = np.array([10.0, 1.0, 1.0, 0.5, 1.0, 1.0, 0.5])
        return norm((self.state() - self.target) * w) ** 2


class ObjHidePoint:
    """ p1 should be hide from the line (o, p0) """
    def __init__(self, _o, _p0, _p1):
        self.o = _o
        self.p0 = _p0
        self.p1 = _p1

    def cost(self):
        # a0 = self.p0.angle(self.o)
        # a1 = self.p1.angle(self.o)
        # print 'a0, a1:', a0, a1
        # return 1.0 if (a0 - 0.2) < a1 else 0.0

        # http://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        a = self.o.p
        n = self.p0.p - a
        n /= norm(n)
        ap = self.p1.p - a
        d = norm(ap - (ap.dot(n)) * n)

        # determinant to test which side
        side = det([[n[2], ap[2]], [n[1], ap[1]]])
        # print side
        return -0.1 * d if side > 0 else 1.0 + d


class IK:
    def __init__(self, _sim):
        self.sim = _sim

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

        if len(self.sim.tips) == 1:
            self.objs = [ObjTIP(self.sim.tip)]
            self.objs[0].target = self.sim.abstract_tip.commands()
            print 'tip.objs[0].target = ', self.objs[0].target
        else:
            self.objs = [ObjTWOTIP(self.sim.tips)]
            self.objs[0].target = self.sim.abstract_tip.commands()
            print 'twotips.objs[0].target = ', self.objs[0].target

        r_toe = self.sim.prob.contact('r_toe')
        l_heel = self.sim.prob.contact('l_heel')
        l_toe = self.sim.prob.contact('l_toe')
        obj = ObjHidePoint(r_toe, l_toe, l_heel)
        self.objs += [obj]

        # hands = self.sim.prob.contact('hands')
        # obj0 = ObjHidePoint(r_toe, hands, l_toe)
        # obj1 = ObjHidePoint(r_toe, hands, l_heel)
        # self.objs += [obj0, obj1]
        print 'costs = ', [o.cost() for o in self.objs]

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
        # x = np.array([  0.797,   9.20728001e-04,   9.37700050e-01,
        #                 -3.45819731e-01,   2.15869410e-01,   4.95856065e-01,
        #                 -20.3038,   1.57000000e+00])
        # self.res = { "x" : x, "value" : self.evaluate(x) }
        # self.res = minimize(lambda x: self.evaluate_orientation(x), x0,
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
        # opt = {'popsize': 32}
        # self.res = cma.fmin(self.evaluate, x0, 1.0, opt)
        # self.res = {"x": self.res[0]}

        self.sim.skel.q = self.expand(self.res['x'])

        print "==== result\n", self.res
        print "state = ", self.objs[0].state()
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

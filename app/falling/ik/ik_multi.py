import numpy as np
from numpy.linalg import norm

from scipy.optimize import minimize


class ObjC:
    def __init__(self, _skel, _target, _index):
        self.skel = _skel
        self.target = _target
        self.index = _index

    def cost(self):
        C = self.skel.C
        C = np.array([C[2], C[1]])
        return norm((C - self.target) * [2.0, 2.0] ) ** 2


class ObjPt:
    def __init__(self, _con, _target, _index):
        self.con = _con
        self.target = _target
        self.index = _index

    def cost(self):
        P = self.con.p
        P = np.array([P[2], P[1]])
        return norm((P - self.target) * [1.0, 1.0] ) ** 2


class ObjSmooth:
    def __init__(self):
        self.index = None

    def cost(self, poses):
        n = len(poses)
        v = 0.0
        for i in range(n - 1):
            q_0 = poses[i]
            q_1 = poses[i + 1]
            diff = q_0 - q_1
            diff[:3] = 0.0
            v += norm(diff) ** 2
        return 0.1 * v


class IKMulti(object):
    def __init__(self, _sim, _plan):
        self.sim = _sim
        self.plan = _plan

        # Parameter descriptions
        # param_desc: ( [{dof_index or dof_name}, {weight}] )
        desc = []
        desc.append([(0, 1.0)])  # Orientation
        desc.append([(4, 1.0)])  # X-Y
        desc.append([(5, 1.0)])  # X-Y

        desc.append([('l_shoulder', 1.0), ('r_shoulder', 1.0), ])
        desc.append([('l_hand', 1.0), ('r_hand', 1.0), ])
        desc.append([('l_thigh', 1.0), ])
        desc.append([('r_thigh', 1.0), ])
        desc.append([('l_shin', 0.5), ])
        desc.append([('r_shin', 0.5), ])
        desc.append([('l_heel', 0.05), ])
        desc.append([('r_heel', 0.05), ])
        self.desc = desc

        # Dimensions
        self.n = self.plan.n
        self.dim = len(self.desc)
        self.totaldim = len(self.desc) * self.plan.n
        print 'n=', self.n,
        print 'dim=', self.dim,
        print 'totaldim=', self.totaldim

        # Costs
        self.objs = []
        for i in range(self.n):
            print '== %d th impact' % i
            c_1 = self.plan.contact1(i)
            con_1 = self.prob.contact(c_1)
            p_1 = np.array([0.0, 0.0])
            c_2 = self.plan.contact2(i)
            con_2 = self.prob.contact(c_2)
            p_2 = self.plan.P(i)
            C = self.plan.C(i)
            print 'p_1 = ', c_1, con_1.name, p_1
            print 'p_2 = ', c_2, con_2.name, p_2
            print 'C = ', C

            self.objs += [ObjPt(con_1, p_1, i)]
            self.objs += [ObjPt(con_2, p_2, i)]
            self.objs += [ObjC(self.skel, C, i)]
        self.objs += [ObjSmooth()]
        print '# objs = ', len(self.objs)
        print 'objs = ', self.objs

    @property
    def prob(self):
        return self.sim.prob

    @property
    def skel(self):
        return self.sim.skel

    def expand(self, x):
        q = self.sim.skel.q
        for i, dofs in enumerate(self.desc):
            x_i = x[i]
            for (d, w) in dofs:
                index = d if isinstance(d, int) else self.skel.dof_index(d)
                q[index] = w * x_i
        return q

    def expand_all(self, x):
        poses = []
        for i in range(self.n):
            x_i = x[i * self.dim:(i + 1) * self.dim]
            q_i = self.expand(x_i)
            poses.append(q_i)
        return poses

    def get_costs(self, x):
        poses = self.expand_all(x)
        costs = []
        for i in range(self.n):
            self.skel.q = poses[i]
            obj_i = [o for o in self.objs if o.index == i]
            costs += [o.cost() for o in obj_i]

        obj_other = [o for o in self.objs if o.index is None]
        costs += [o.cost(poses) for o in obj_other]
        return costs

    def evaluate(self, x):
        return sum(self.get_costs(x))

    def optimize(self, restore=True):
        saved_pose = self.sim.skel.q
        saved_vel = self.sim.skel.qdot

        self.q0 = self.sim.skel.q

        print "==== ik.IKMulti optimize...."
        self.res = None
        for i in range(5):
            x0 = np.random.rand(self.totaldim)
            res = minimize(lambda x: self.evaluate(x), x0,
                           method='nelder-mead', tol=0.00001,
                           # method='SLSQP', tol=0.00001,
                           options={'maxiter': 10000, 'maxfev': 10000,
                                    'xtol': 10e-8, 'ftol': 10e-8})
            if self.res is None or res['fun'] < self.res['fun']:
                self.res = res
            print i, self.res['fun']

        x = self.res['x']
        self.target_index = 0
        self.targets = self.expand_all(x)

        print "==== result\n", self.res
        print 'final costs:', self.get_costs(x)
        print "==== ik.IKMulti optimize....OK"

        if restore:
            self.sim.skel.q = saved_pose
            self.sim.skel.qdot = saved_vel
        return self.targets

    def next_target(self):
        self.skel.q = self.targets[self.target_index]
        self.target_index = (self.target_index + 1) % self.n

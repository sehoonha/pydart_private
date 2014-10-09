import numpy as np
from numpy.linalg import norm

from scipy.optimize import minimize


class ObjC(object):
    def __init__(self, _skel, _target, _index):
        self.skel = _skel
        self.target = _target
        self.index = _index

    @property
    def C(self):
        C = self.skel.C()
        return np.array([C[2], C[1]])
        # return C

    def cost(self):
        return norm((self.C - self.target) * [1.0, 1.0] ) ** 2

    def __str__(self):
        return '[ObjC: %.6f (%r, %r)]' % (self.cost(), self.C, self.target)


class ObjCRel(object):
    def __init__(self, _tip, _target, _index):
        self.tip = _tip
        self.target = _target
        self.index = _index

    @property
    def C(self):
        C = self.tip.C_rel()
        return np.array([C[2], C[1]])
        # return C

    def cost(self):
        return norm((self.C - self.target) * [0.3, 0.3] ) ** 2

    def __str__(self):
        return '[ObjCRel: %.6f (%r, %r)]' % (self.cost(), self.C, self.target)


class ObjBaseDist(object):
    def __init__(self, _tip, _target, _index):
        self.tip = _tip
        self.target = _target
        self.index = _index

    @property
    def d(self):
        return self.tip.base_dist()

    def cost(self):
        diff = self.d - self.target
        return (diff * 1.0) ** 2

    def __str__(self):
        return '[ObjBaseDist: %.6f (%.4f, %.4f)]' % (self.cost(),
                                                     self.d, self.target)


class ObjPt(object):
    def __init__(self, _con, _target, _index):
        self.con = _con
        self.target = _target
        self.index = _index

    @property
    def P(self):
        P = self.con.p
        return np.array([P[2], P[1]])
        # return P

    def cost(self):
        return norm((self.P - self.target) * [1.0, 1.0] ) ** 2

    def __str__(self):
        return '[ObjPt.%s: %.6f (%r, %r)]' % (self.con.name, self.cost(),
                                              self.P, self.target)


class ObjSmooth:
    def __init__(self, _init_q):
        self.index = None
        self.last_cost = 0.0
        self.init_q = _init_q

    def cost(self, poses):
        poses = [self.init_q] + poses
        n = len(poses)
        v = 0.0
        for i in range(n - 1):
            q_0 = poses[i]
            q_1 = poses[i + 1]
            diff = q_0 - q_1
            diff[:6] = 0.0
            # v += norm(diff) ** 2
            v += diff.dot(diff)
        self.last_cost = 0.001 * v
        return self.last_cost

    def __str__(self):
        return '[ObjSmooth: %.6f]' % (self.last_cost)


class IKMulti(object):
    def __init__(self, _sim, _plan):
        self.sim = _sim
        self.plan = _plan

        # Parameter descriptions
        # param_desc: ( [{dof_index or dof_name}, {weight}] )
        desc = []

        cfg_name = self.sim.cfg.name
        leg_symmetry = cfg_name in ['lean', 'back']
        print 'leg_symmetry:', leg_symmetry

        if self.sim.is_bioloid():
            desc.append([('l_shoulder', 1.0), ('r_shoulder', 1.0), ])
            desc.append([('l_hand', 1.0), ('r_hand', 1.0), ])
            if leg_symmetry:
                desc.append([('l_thigh', 1.0), ('r_thigh', 1.0), ])
                desc.append([('l_shin', 0.5), ('r_shin', 0.5), ])
                desc.append([('l_heel', 0.05), ('r_heel', 0.05), ])
            else:
                desc.append([('l_thigh', 1.0), ])
                desc.append([('r_thigh', 1.0), ])
                desc.append([('l_shin', 0.5), ])
                desc.append([('r_shin', 0.5), ])
                desc.append([('l_heel', 0.05), ])
                desc.append([('r_heel', 0.05), ])
        else:
            desc.append([('l_arm_shy', 1.0), ('r_arm_shy', 1.0), ])
            desc.append([('l_arm_shx', 1.0), ('r_arm_shx', -1.0), ])
            desc.append([('l_arm_elx', 1.0), ('r_arm_elx', -1.0), ])
            desc.append([('back_bky', 1.0), ])
            desc.append([('l_leg_hpy', 1.0), ('r_leg_hpy', 1.0), ])
            desc.append([('l_leg_kny', 1.0), ('r_leg_kny', 1.0), ])
            desc.append([('l_leg_aky', 1.0), ('r_leg_aky', 1.0), ])

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
            c1 = self.plan.contact1(i)
            con1 = self.prob.contact(c1)
            p1 = np.array([0.0, 0.0, 0.0])
            c2 = self.plan.contact2(i)
            con2 = self.prob.contact(c2)
            p2 = self.plan.P(i)
            C = self.plan.C_rel(i)
            bd = self.plan.base_dist(i)
            e = self.prob.next_e[c1][c2]
            tip = self.prob.tips[e]
            print 'p_1 = ', c1, con1.name, p1
            print 'p_2 = ', c2, con2.name, p2
            print 'edge = ', e, tip
            print 'C_rel = ', C, 'base_dist', bd

            # self.objs += [ObjPt(con1, p1, i)]
            # self.objs += [ObjPt(con2, p2, i)]
            # self.objs += [ObjC(self.skel, C, i)]
            self.objs += [ObjCRel(self.prob.tips[e], C, i)]
            self.objs += [ObjBaseDist(self.prob.tips[e], bd, i)]
        self.objs += [ObjSmooth(self.skel.q)]
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

    def get_costs(self, x, verbose=False):
        poses = self.expand_all(x)
        # Check the validity
        for q in poses:
            # Check the pose
            max_joint = np.max(np.fabs(q[6:]))
            if max_joint > 2.0:
                return [max_joint] * len(self.objs)
            # # Check the change of pose
            # dq = q - self.q0
            # if np.max(np.fabs(dq)) > 0.8:
            #     return [10.0 * np.max(np.fabs(dq))] * len(self.objs)

        # Collect all the costs
        costs = []
        obj_strs = []
        for i in range(self.n):
            self.skel.q = poses[i]
            obj_i = [o for o in self.objs if o.index == i]
            costs += [o.cost() for o in obj_i]
            if verbose:
                obj_strs += [str(o) for o in obj_i]

        obj_other = [o for o in self.objs if o.index is None]
        costs += [o.cost(poses) for o in obj_other]
        obj_strs += [str(o) for o in obj_other]
        if verbose:
            for i, obj_str in enumerate(obj_strs):
                print i, obj_str
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
            res = minimize(self.evaluate, x0,
                           method='nelder-mead', tol=0.00001,
                           # method='SLSQP', tol=0.00001,
                           options={'maxiter': 100000, 'maxfev': 100000,
                                    'xtol': 10e-8, 'ftol': 10e-8})
            if self.res is None or res['fun'] < self.res['fun']:
                self.res = res
            print i, self.res['fun']

        x = self.res['x']
        self.target_index = 0
        self.targets = self.expand_all(x)

        print "==== result\n", self.res
        print '[final costs]:'
        self.get_costs(x, True)
        print "==== ik.IKMulti optimize....OK"

        if restore:
            self.sim.skel.q = saved_pose
            self.sim.skel.qdot = saved_vel
        return self.targets

    def next_target(self):
        print 'next_target: ik.target_index:', self.target_index
        self.skel.q = self.targets[self.target_index]
        self.target_index = (self.target_index + 1) % self.n

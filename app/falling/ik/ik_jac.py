import math
import numpy as np
from numpy.linalg import norm

import scipy.optimize


class Obj(object):
    def __init__(self, _name, _pi, _f, _v, _w=1.0):
        self.name = _name
        self.pose_index = _pi
        self.f = _f
        self.v = _v
        self.w = _w

    def __str__(self):
        return '[%s : (%.4f %.4f)]' % (self.name, self.f(), self.v)


class IKJac(object):
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
            desc.append([('l_arm_shy', 1.0), ('r_arm_shy', 1.0),
                         ('l_arm_shx', 0.0), ('r_arm_shx', 0.0), ])
            desc.append([('back_bky', 1.0), ])
            desc.append([('l_leg_hpy', 1.0), ('r_leg_hpy', 1.0), ])
            desc.append([('l_leg_kny', 1.0), ('r_leg_kny', 1.0), ])
            desc.append([('l_leg_aky', 1.0), ('r_leg_aky', 1.0), ])
        self.desc = desc

        # Dimensions
        self.n = self.plan.n
        self.n = 2
        self.dim = len(self.desc)
        self.totaldim = len(self.desc) * self.n
        print 'n=', self.n,
        print 'dim=', self.dim,
        print 'totaldim=', self.totaldim

        # Initialize the objectives and constraints
        self.objs = []
        self.con_eqs = []
        self.con_ineqs = []

        for i in range(self.n):
            print '== %d th impact' % i
            c1 = self.plan.contact1(i)
            c2 = self.plan.contact2(i)
            e = self.prob.next_e[c1][c2]
            tip = self.prob.tips[e]

            r1 = self.plan.r1(i)
            r2 = self.plan.r2(i)
            th2 = self.plan.th2(i)
            print 'target r1, r2, th2: ', r1, r2, th2

            # Put objectives related to TIPs
            self.con_eqs += [Obj("r1_%d" % i, i, tip.r1, r1)]
            self.con_eqs += [Obj("r2_%d" % i, i, tip.r2, r2)]
            self.objs += [Obj("th2_%d" % i, i, tip.th2, th2)]

        # Summarize objectives and constraints
        self.print_objs()

    @property
    def prob(self):
        return self.sim.prob

    @property
    def skel(self):
        return self.sim.skel

    def print_objs(self):
        print 'Objectives: '
        for o in self.objs:
            print o
        print 'Equality constraints: '
        for o in self.con_eqs:
            result = 'O' if math.fabs(o.f() - o.v) < 1e-4 else 'X'
            print o, result

    def expand(self, x):
        q = self.sim.skel.q
        for i, dofs in enumerate(self.desc):
            x_i = x[i]
            for (d, w) in dofs:
                index = d if isinstance(d, int) else self.skel.dof_index(d)
                if d == 'l_arm_shx':
                    q[index] = -0.5 - math.cos(x_i / 1.57)
                elif d == 'r_arm_shx':
                    q[index] = 0.5 + math.cos(x_i / 1.57)
                else:
                    q[index] = w * x_i
        return q

    def expand_all(self, x):
        poses = []
        for i in range(self.n):
            x_i = x[i * self.dim:(i + 1) * self.dim]
            q_i = self.expand(x_i)
            poses.append(q_i)
        return poses

    def update_pose(self, x, pose_index):
        poses = self.expand_all(x)
        q = poses[pose_index]
        diff = norm(self.skel.q - q)
        if diff > 1e-16:
            # print 'update pose!!'
            self.skel.q = q

    def obj(self, x):
        value = 0.0
        for obj in self.objs:
            self.update_pose(x, obj.pose_index)
            value_now = 0.5 * (obj.f() - obj.v) ** 2
            value += obj.w * value_now
        # print 'obj:', obj, x, value
        return value

    def constraint(self, x, obj):
        self.update_pose(x, obj.pose_index)
        # print 'eq:', obj, x, obj.pose_index, obj.f() - obj.v
        return obj.f() - obj.v

    def optimize(self, restore=True):
        saved_state = self.skel.x

        cons = []
        for i, o in enumerate(self.con_eqs):
            print 'objective:', i, o
            cons += [{'type': 'eq', 'fun': self.constraint, 'args': [o]}]

        print "==== ik.IKJac optimize...."
        x0 = (np.random.rand(self.totaldim) - 0.5) * 3.14
        options = {'maxiter': 100000, 'maxfev': 100000,
                   'xtol': 10e-8, 'ftol': 10e-8}
        self.res = scipy.optimize.minimize(self.obj, x0,
                                           method='SLSQP',
                                           constraints=cons,
                                           options=options)

        print "==== result"
        print self.res
        self.print_objs()
        print "==== ik.IKJac optimize....OK"

        if restore:
            self.skel.x = saved_state

        x = self.res['x']
        self.targets = self.expand_all(x)
        return self.targets

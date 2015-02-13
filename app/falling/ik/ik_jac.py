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
        if isinstance(self.v, float):
            return '[%s (%d): (%.4f %.4f)]' % (self.name, self.pose_index,
                                               self.f(), self.v)
        else:
            diff = self.f() - self.v
            return '[%s (%d): %.4f]' % (self.name, self.pose_index,
                                        0.5 * norm(diff) ** 2)


class IKJac(object):
    def __init__(self, _sim, _plan, _ik_index=None, _saved_targets=None):
        self.sim = _sim
        self.plan = _plan

        self.ik_index = _ik_index
        self.saved_targets = _saved_targets
        # Parameter descriptions
        # param_desc: ( [{dof_index or dof_name}, {weight}] )
        desc = []
        cfg_name = self.sim.cfg.name
        # leg_symmetry = cfg_name in ['lean', 'back']
        leg_symmetry = ('lean' in cfg_name) or ('back' in cfg_name)
        print 'cfg_name:', cfg_name
        print 'leg_symmetry:', leg_symmetry

        if self.sim.is_bioloid():
            desc.append([('l_shoulder', 1.0), ('r_shoulder', 1.0), ])
            desc.append([('l_hand', 1.0), ('r_hand', 1.0), ])
            if leg_symmetry:
                desc.append([('l_thigh', 1.0), ('r_thigh', 1.0), ])
                desc.append([('l_shin', 1.0), ('r_shin', 1.0), ])
                desc.append([('l_heel', 1.0), ('r_heel', 1.0), ])
            else:
                desc.append([('l_thigh', 1.0), ])
                desc.append([('r_thigh', 1.0), ])
                desc.append([('l_shin', 1.0), ])
                desc.append([('r_shin', 1.0), ])
                desc.append([('l_heel', 1.0), ])
                desc.append([('r_heel', 1.0), ])
        else:
            desc.append([('back_bky', 1.0), ])
            if leg_symmetry:
                desc.append([('l_leg_hpy', 1.0), ('r_leg_hpy', 1.0), ])
                desc.append([('l_leg_kny', 1.0), ('r_leg_kny', 1.0), ])
                desc.append([('l_leg_aky', 1.0), ('r_leg_aky', 1.0), ])
            else:
                desc.append([('l_leg_hpy', 1.0), ])
                desc.append([('r_leg_hpy', 1.0), ])
                desc.append([('l_leg_kny', 1.0), ])
                desc.append([('r_leg_kny', 1.0), ])
                desc.append([('l_leg_aky', 1.0), ])
                desc.append([('r_leg_aky', 1.0), ])
            desc.append([('l_arm_shx', 1.0), ('r_arm_shx', -1.0), ])
            desc.append([('l_arm_shy', 1.0), ('r_arm_shy', 1.0), ])
            desc.append([('l_arm_elx', 1.0), ('r_arm_elx', -1.0), ])
        self.desc = desc

        # Dimensions
        self.n = self.plan.n
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
            # self.con_eqs += [Obj("r1_%d" % i, i, tip.r1, r1)]
            # self.con_eqs += [Obj("r2_%d" % i, i, tip.r2, r2)]
            self.objs += [Obj("r1_%d" % i, i, tip.r1, r1, 100.0)]
            self.objs += [Obj("r2_%d" % i, i, tip.r2, r2, 100.0)]
            self.objs += [Obj("th2_%d" % i, i, tip.th2, th2, 2.0)]
            # if i > 0:
            #     self.objs += [Obj("r2_%d" % i, i - 1, tip.r2, r2, 0.5)]
            #     self.objs += [Obj("th2_%d" % i, i - 1, tip.th2, th2, 0.1)]

        # Summarize objectives and constraints
        self.print_objs(self.objs, self.con_eqs, self.con_ineqs)

    @property
    def prob(self):
        return self.sim.prob

    @property
    def skel(self):
        return self.sim.skel

    def pose(self):
        return self.skel.q

    def print_objs(self, objs, con_eqs, con_ineqs):
        print 'Objectives: '
        for o in objs:
            if isinstance(o.v, float):
                value_now = 0.5 * (o.f() - o.v) ** 2
            else:
                value_now = 0.5 * norm(o.f() - o.v) ** 2
            print o, o.w * value_now
        print 'Equality constraints: '
        for o in con_eqs:
            result = 'O' if math.fabs(o.f() - o.v) < 1e-4 else 'X'
            print o, result
        print 'Inequality constraints: '
        for o in con_ineqs:
            result = 'O' if o.f() > o.v else 'X'
            print o, result

    def expand(self, x):
        q = self.sim.skel.q
        lo = self.skel.q_lo
        hi = self.skel.q_hi
        lo[:6] = np.zeros(6)
        hi[:6] = np.zeros(6)
        # Readjusts the range of the motion
        mi = 0.5 * (lo + hi)
        rng = 0.5 * (hi - lo)  # lo = mi - rng, hi = mi + rng
        # lo = mi - 0.5 * rng
        # hi = mi + 0.5 * rng
        lo = mi - 0.9 * rng
        hi = mi + 0.9 * rng

        for i, dofs in enumerate(self.desc):
            v = x[i]
            for (d, w) in dofs:
                index = d if isinstance(d, int) else self.skel.dof_index(d)
                vv = v if w > 0.0 else 1.0 - v
                q[index] = lo[index] + (hi[index] - lo[index]) * vv
        return q

    def update_pose(self, x):
        q = self.expand(x)
        diff = norm(self.skel.q - q)
        if diff > 1e-16:
            # print 'update pose!!'
            self.skel.q = q

    def obj(self, x, objs):
        value = 0.0
        for obj in objs:
            self.update_pose(x)
            if isinstance(obj.v, float):
                value_now = 0.5 * (obj.f() - obj.v) ** 2
            else:
                value_now = 0.5 * norm(obj.f() - obj.v) ** 2
            # if 'r2' in obj.name and obj.f() < obj.v:
            #     value_now *= 4.0
            value += obj.w * value_now
        # print 'obj:', obj, x, value
        return value

    def constraint(self, x, obj):
        self.update_pose(x)
        # print 'eq:', obj, x, obj.pose_index, obj.f() - obj.v
        return obj.f() - obj.v

    def optimize_index(self, index):
        if self.ik_index is not None and index != self.ik_index:
            return self.saved_targets[index]
        print
        print '======== optimize_index', index
        objs = [o for o in self.objs if o.pose_index == index]
        con_eqs = [o for o in self.con_eqs if o.pose_index == index]
        con_ineqs = [o for o in self.con_ineqs if o.pose_index == index]

        # Add pose constraint
        obj_pose = Obj("q%d" % index, index, self.pose,
                       self.prev_target, 0.0001)
        objs += [obj_pose]

        # Make constraints
        cons = []
        for i, o in enumerate(con_eqs):
            cons += [{'type': 'eq', 'fun': self.constraint, 'args': [o]}]
        for i, o in enumerate(con_ineqs):
            cons += [{'type': 'ineq', 'fun': self.constraint, 'args': [o]}]
        options = {'maxiter': 100000, 'maxfev': 100000,
                   'xtol': 10e-8, 'ftol': 10e-8}
        bnds = [(0.0, 1.0)] * self.dim

        print "==== ik.IKJac optimize...."
        res = None
        for i in range(5):
            # x0 = (np.random.rand(self.dim) - 0.5) * 3.14
            x0 = (np.random.rand(self.dim) - 0.5)
            now = scipy.optimize.minimize(self.obj, x0,
                                          args=(objs,),
                                          method='SLSQP',
                                          # method='L-BFGS-B',
                                          bounds=bnds,
                                          constraints=cons,
                                          options=options)
            if res is None or now['fun'] < res['fun']:
                res = now
            print i, res['fun'], res['success']
        print "==== result"
        x = res['x']
        self.update_pose(x)
        target = self.expand(x)
        print res
        self.print_objs(objs, con_eqs, con_ineqs)
        print "==== ik.IKJac optimize....OK"
        # print 'target:', target
        return target

    def optimize(self, restore=True):
        saved_state = self.skel.x

        self.targets = []
        self.prev_target = self.skel.q
        for i in range(self.n):
            t = self.optimize_index(i)
            print 'check_validity:', self.is_valid_pose(t)
            self.targets += [t]
            self.prev_target = t

        if restore:
            self.skel.x = saved_state

        # # Debug routine
        # self.skel.x = saved_state
        # print 'ignore ik and return the intial pose....'
        # self.targets = [self.skel.q]

        return self.targets

    def is_valid_pose(self, q):
        """ For debuging """
        lo = self.skel.q_lo
        hi = self.skel.q_hi

        for i in range(6, self.skel.ndofs):
            if q[i] < lo[i] or hi[i] < q[i]:
                return False
        return True

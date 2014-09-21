import math
import numpy as np
from numpy.linalg import norm


def is_similar(lhs, rhs):
    """ lhs and rhs are poses as np.array """
    return (norm(lhs - rhs) < 0.05)


class StopperSet(object):
    def __init__(self, _states=None):
        self.states = []
        if _states is not None:
            self.states = _states
        self.w = np.array([1.0, 0.1, 1.0])

    def insert(self, x):
        self.states.append(np.array(x))

    def is_new(self, lhs):
        for rhs in self.states:
            if is_similar(lhs, rhs):
                return False
        return True

    def insert_if_new(self, x):
        x = x * self.w
        if self.is_new(x):
            self.insert(x)

    def __len__(self):
        return len(self.states)

    def __repr__(self):
        return 'StopperSet(%r)' % self.states


class RangeChecker(object):
    def __init__(self, _sim):
        self.sim = _sim

    @property
    def skel(self):
        return self.sim.skel

    @property
    def prob(self):
        return self.sim.prob

    def check_all(self):
        print '==== start the range check ===='
        self.check_init_angle()
        # self.check_kinematic()
        self.check_kinematic_cached()
        for tip, ss in zip(self.prob.tips, self.stop_sets):
            print 'TIP ', str(tip),
            print ' sampled set is', len(ss), id(ss)

    def check_init_angle(self):
        self.init_angles = [tip.th2 for tip in self.prob.tips]
        print self.init_angles

    def check_kinematic(self):
        # Each edge has a set of stoppers
        self.stop_sets = [StopperSet() for _ in range(self.prob.m)]

        for i in range(1000):
            self.set_random_pose()
            for tip, ss in zip(self.prob.tips, self.stop_sets):
                x = tip.pose()
                if x[-1] > math.pi:
                    continue
                ss.insert_if_new(x)

        fp = open('cached_stopper_sets.py', 'w+')
        fp.write('from range_checker import StopperSet\n')
        fp.write('from numpy import array\n')
        fp.write('cached_sets=%s' % repr(self.stop_sets))
        fp.close()

    def check_kinematic_cached(self):
        import cached_stopper_sets
        self.stop_sets = cached_stopper_sets.cached_sets

    def set_random_pose(self):
        param_desc = [(0, 'l_shoulder', 1.0),
                      (0, 'r_shoulder', 1.0),
                      (1, 'l_hand', 1.0),
                      (1, 'r_hand', 1.0),
                      (2, 'l_thigh', 1.0),
                      (3, 'r_thigh', 1.0),
                      (4, 'l_shin', 0.5),
                      (5, 'r_shin', 0.5),
                      (6, 'l_heel', 0.05),
                      (7, 'r_heel', 0.05) ]
        dim = max([i for i, dof, w in param_desc]) + 1
        lo = np.array([-1.57] * dim)
        hi = np.array([1.57] * dim)
        x = lo + np.random.rand(dim) * (hi - lo)
        q = self.skel.q
        for x_i, dof_name, _ in param_desc:
            dof_i = self.skel.dof_index(dof_name)
            q[dof_i] = x[x_i]
        # Validity check?
        self.skel.q = q

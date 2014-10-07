
import time
import math
import numpy as np
from numpy.linalg import norm
from nearpy import Engine
from nearpy.hashes import RandomBinaryProjections


THRESHOLD = 0.03


def is_similar(lhs, rhs):
    """ lhs and rhs are poses as np.array """
    return (norm(lhs - rhs) < THRESHOLD)


class StopperSet(object):
    def __init__(self, _states=None):
        # Weight
        # initialize "nearby" library
        self.dim = 3

        self.lo = [0.05, 0.05, 0.0]
        self.hi = [0.20, 0.20, 3.0]
        # self.step = [0.005, 0.005, 0.1]
        self.step = [0.005, 0.005, 0.1]
        self.num = [int((self.hi[i] - self.lo[i]) / self.step[i])
                    for i in range(self.dim)]

        self.data = np.zeros(shape=self.num, dtype=int)

        # performance counter
        self.counter = 0
        self.th2_range = (2 * math.pi, -2 * math.pi)
        self.time_counter = 0.0

        # # load data if necessary
        # if _states is not None:
        #     for x in _states:
        #         self.insert(x)

    def to_index(self, x):
        return [int((x[i] - self.lo[i]) / self.step[i] )
                for i in range(self.dim)]

    def is_valid(self, index):
        for i in range(self.dim):
            if index[i] < 0:
                return False
            if index[i] >= self.num[i]:
                return False
        return True

    def insert(self, x):
        index = self.to_index(x)
        if not self.is_valid(index):
            return
        self.data[tuple(index)] += 1
        # Other checking
        self.counter += 1
        self.th2_range = (min(self.th2_range[0], x[-1]),
                          max(self.th2_range[1], x[-1]))

    def is_new(self, x):
        t0 = time.time()
        index = self.to_index(x)
        if not self.is_valid(index):
            self.time_counter += (time.time() - t0)
            return True
        self.time_counter += (time.time() - t0)
        # print x, index, self.data[tuple(index)]
        return (self.data[tuple(index)] == 0)

    def insert_if_new(self, x):
        if self.is_new(x):
            self.insert(x)

    def __len__(self):
        return self.counter

    def __repr__(self):
        return 'StopperSet(%r)' % (self.data)


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
        self.check_kinematic(False)
        # self.check_kinematic_cached()
        for tip, ss in zip(self.prob.tips, self.stop_sets):
            print 'TIP ', str(tip),
            print ' Len:', len(ss), 'Time: %.4f' % ss.time_counter
            print ' th2_range = ', ss.th2_range
            # print ' lo = ', ss.lo
            # print ' hi = ', ss.hi

    def check_init_angle(self):
        self.init_angles = [tip.th2 for tip in self.prob.tips]
        print self.init_angles

    def check_kinematic(self, generate_cache=True):
        saved_x = self.skel.x

        # Each edge has a set of stoppers
        self.stop_sets = [StopperSet() for _ in range(self.prob.m)]

        for i in range(10000):
            self.set_random_pose()
            for tip, ss in zip(self.prob.tips, self.stop_sets):
                x = tip.pose()
                if x[-1] > math.pi:
                    continue
                ss.insert_if_new(x)

        if generate_cache:
            fp = open('cached_stopper_sets.py', 'w+')
            fp.write('from range_checker import StopperSet\n')
            fp.write('from numpy import array\n')
            fp.write('cached_sets=%s' % repr(self.stop_sets))
            fp.close()
        self.skel.x = saved_x

    def check_kinematic_cached(self):
        import cached_stopper_sets
        self.stop_sets = cached_stopper_sets.cached_sets

    def query_dynamics(self, t, c1, c2, th2):
        e = self.prob.next_e[c1][c2]
        th2_0 = self.init_angles[e]
        vel = 1.57  # maximum velocity of the th2
        lo, hi = (th2_0 - t * vel, th2_0 + t * vel)
        return (lo < th2) and (th2 < hi)

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

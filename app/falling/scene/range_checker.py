
import time
import math
import numpy as np
from numpy.linalg import norm
# from nearpy import Engine
# from nearpy.hashes import RandomBinaryProjections

# Plotting
import plotly.plotly as py
import plotly.graph_objs as pyg


THRESHOLD = 0.03


def is_similar(lhs, rhs):
    """ lhs and rhs are poses as np.array """
    return (norm(lhs - rhs) < THRESHOLD)


class StopperSet(object):
    def __init__(self, humansize=False):
        # Weight
        # initialize "nearby" library
        self.dim = 3

        if not humansize:
            self.lo = [0.05, 0.05, 0.0]
            self.hi = [0.20, 0.20, 3.0]
            self.step = [0.005, 0.005, 0.1]
            # self.step = [0.01, 0.01, 0.1]
        else:
            self.lo = [0.40, 0.40, 0.0]
            self.hi = [1.20, 1.20, 3.0]
            self.step = [0.02, 0.02, 0.1]

        self.num = [int((self.hi[i] - self.lo[i]) / self.step[i])
                    for i in range(self.dim)]

        self.data = np.zeros(shape=self.num, dtype=int)

        # performance counter
        self.counter = 0
        self.th2_range = (2 * math.pi, -2 * math.pi)
        self.time_counter = 0.0

        self.states = []

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
        # self.states += [x]  # Only for plotting....
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
            return True
        return False

    def __len__(self):
        return self.counter

    def __repr__(self):
        return 'StopperSet(%r)' % (self.data)

    def get_trace(self, _name):
        x = []
        y = []
        for r1, r2, th2 in self.states:
            x1 = 0.0
            y1 = r1
            x2 = x1 + r2 * math.sin(th2)
            y2 = y1 + r2 * math.cos(th2)
            x += [0.0, x1, x2, x1, 0.0]
            y += [0.0, y1, y2, y1, 0.0]
        return pyg.Scatter(x=x, y=y, name=_name, opacity=0.5)


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
        # self.plot()

    def plot(self):
        traces = []
        for tip, ss in zip(self.prob.tips, self.stop_sets):
            print 'Plot TIP ', str(tip),
            name = str(tip)
            traces += [ss.get_trace(name)]
        data = pyg.Data(traces)
        lo = -0.01 if self.sim.is_bioloid() else -0.1
        hi = 0.39 if self.sim.is_bioloid() else 1.9
        layout = pyg.Layout(xaxis=pyg.XAxis(range=[lo, hi]),
                            yaxis=pyg.YAxis(range=[lo, hi]))
        unique_url = py.plot({'data': data, 'layout': layout},
                             filename='Ranges')
        print 'url =', unique_url

    def check_init_angle(self):
        self.init_angles = [tip.th2() for tip in self.prob.tips]
        print self.init_angles

    def check_kinematic(self, generate_cache=True):
        saved_x = self.skel.x

        humansize = not (self.sim.is_bioloid())

        # Each edge has a set of stoppers
        self.stop_sets = [StopperSet(humansize) for _ in range(self.prob.m)]

        for i in range(10000):
            self.set_random_pose()
            for tip, ss in zip(self.prob.tips, self.stop_sets):
                x = tip.pose()
                if x[-1] > math.pi:
                    continue
                inserted = ss.insert_if_new(x)
                # print 'insert', x, inserted
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

    def is_valid_pose(self):
        """ For debuging """
        q = self.skel.q
        lo = self.skel.q_lo
        hi = self.skel.q_hi

        for i in range(6, self.skel.ndofs):
            if q[i] < lo[i] or hi[i] < q[i]:
                return False
        return True

    def set_random_pose(self):
        if self.sim.is_bioloid():
            q = self.skel.q
            lo = np.array(self.skel.q_lo)
            hi = np.array(self.skel.q_hi)
            lo[:6] = np.zeros(6)
            hi[:6] = np.zeros(6)
            # Readjusts the range of the motion
            mi = 0.5 * (lo + hi)
            rng = 0.5 * (hi - lo)  # lo = mi - rng, hi = mi + rng
            lo = mi - 0.5 * rng
            hi = mi + 0.5 * rng
            # param_desc = [(0, 'l_shoulder', 1.0),
            #               (0, 'r_shoulder', 1.0),
            #               (1, 'l_hand', 1.0),
            #               (1, 'r_hand', 1.0),
            #               (2, 'l_thigh', 1.0),
            #               (2, 'r_thigh', 1.0),
            #               (3, 'l_shin', 1.0),
            #               (3, 'r_shin', 1.0),
            #               (4, 'l_heel', 1.0),
            #               (4, 'r_heel', 1.0) ]
            param_desc = [(0, 'l_shoulder', 1.0),
                          (0, 'r_shoulder', 1.0),
                          (1, 'l_hand', 1.0),
                          (1, 'r_hand', 1.0),
                          (2, 'l_thigh', 1.0),
                          (3, 'r_thigh', 1.0),
                          (4, 'l_shin', 1.0),
                          (5, 'r_shin', 1.0),
                          (6, 'l_heel', 1.0),
                          (7, 'r_heel', 1.0) ]
            dim = max([d[0] for d in param_desc]) + 1
            # params = 0.2 + (1.0 - 2 * 0.2) * np.random.rand(dim)
            # params = 0.4 + (1.0 - 2 * 0.4) * np.random.rand(dim)
            params = np.random.rand(dim)
            for x_i, dof_name, w in param_desc:
                v = params[x_i]
                if w < 0.0:
                    v = 1.0 - v
                i = self.skel.dof_index(dof_name)
                # print dof_name, i, v
                q[i] = lo[i] + (hi[i] - lo[i]) * v
            self.skel.q = q
        else:
            q = self.skel.q
            lo = self.skel.q_lo
            hi = self.skel.q_hi
            # param_desc = [(0, 'back_bky', 1.0),
            #               (1, 'l_leg_hpy', 1.0),
            #               (1, 'r_leg_hpy', 1.0),
            #               (2, 'l_leg_kny', 1.0),
            #               (2, 'r_leg_kny', 1.0),
            #               (3, 'l_leg_aky', 1.0),
            #               (3, 'r_leg_aky', 1.0),
            #               (4, 'l_arm_shx', 1.0),
            #               (4, 'r_arm_shx', -1.0),
            #               (5, 'l_arm_shy', 1.0),
            #               (5, 'r_arm_shy', 1.0),
            #               (6, 'l_arm_elx', 1.0),
            #               (6, 'r_arm_elx', -1.0), ]
            param_desc = [(0, 'back_bky', 1.0),
                          (1, 'l_leg_hpy', 1.0),
                          (2, 'r_leg_hpy', 1.0),
                          (3, 'l_leg_kny', 1.0),
                          (4, 'r_leg_kny', 1.0),
                          (5, 'l_leg_aky', 1.0),
                          (6, 'r_leg_aky', 1.0),
                          (7, 'l_arm_shx', 1.0),
                          (7, 'r_arm_shx', -1.0),
                          (8, 'l_arm_shy', 1.0),
                          (8, 'r_arm_shy', 1.0),
                          (9, 'l_arm_elx', 1.0),
                          (9, 'r_arm_elx', -1.0), ]
            dim = max([d[0] for d in param_desc]) + 1
            params = np.random.rand(dim)
            for x_i, dof_name, w in param_desc:
                v = params[x_i]
                if w < 0.0:
                    v = 1.0 - v
                i = self.skel.dof_index(dof_name)
                # print dof_name, i, v
                q[i] = lo[i] + (hi[i] - lo[i]) * v
            self.skel.q = q

        # else:
        #     q = self.skel.q
        #     lo = self.skel.q_lo
        #     hi = self.skel.q_hi
        #     rand_q = np.random.rand(len(q)) * (hi - lo) + lo
        #     q[6:] = rand_q[6:]
        #     self.skel.q = q

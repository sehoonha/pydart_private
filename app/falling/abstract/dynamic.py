import sys
import math
from math import sin, cos
import numpy as np
# from collections import namedtuple
from scipy.integrate import odeint
from state_db import State, Control, get_points, StateDB

g_inf = float("inf")


class DynamicTIP:
    def __init__(self, _prob, _range):
        # strucures
        self.prob = _prob
        self.rng = _range
        self.db = StateDB(self.prob.n)
        # member variables
        self.eval_counter = 0
        self.N_GRID = 11
        # math quantities
        self.m = 1.08
        self.I = 0.0080
        self.g = -9.8
        (self.lo_dr, self.hi_dr) = (-0.01, 0.01)

    def set_x0(self, tips):
        t0 = tips[0]
        self.x0 = State(t0.th1, t0.dth1, t0.r1, 0, 0, 0.0)
        print 'set abstract.DynamicTIP.x0 = ', self.x0

    def deriv(self, _state, _t):
        x = State(*_state)
        (th1, dth1, r1, dr1) = (x.th1, x.dth1, x.r1, x.dr1)

        (m, g) = (self.m, self.g)
        ddth1 = (m * r1 * (2 * dr1 * dth1 - g * sin(th1))) / (m * (r1 ** 2))
        return [dth1, ddth1, dr1, 0, 0, 1.0]

    def step(self, x):
        # time = np.arange(0.0, 0.01, 0.005)
        time = np.array([0.0, 0.005])
        x0 = np.array(x)
        X = odeint(self.deriv, x0, time)
        return State(*X[-1])

    def is_stopped(self, x):
        # return (x.c1 != 0)
        return (x.dth1 < 0)

    def is_grounded(self, x):
        return (x.th1 > 0.5 * math.pi)

    def impact(self, x, u):
        # Fetch the required quantities
        p = get_points(x, u)
        (x1, x2, dy2) = (p.x1, p.x2, p.dy2)
        (m, I) = (self.m, self.I)
        # Estimate the impulse
        j = (-dy2) / ((1 / m) + (1 / I) * ((x2 - x1) ** 2))

        # Estimate the next velocity
        n_th1 = x.th1 + u.th2 - math.pi
        n_dth1 = x.dth1 - (1 / I) * (x2 - x1) * j
        n_r1 = u.r2
        n_dr1 = u.n_dr1
        # n_c = x.c1 + 1
        n_c = u.c2
        n_t = x.t
        n_x = State(n_th1, n_dth1, n_r1, n_dr1, n_c, n_t)
        return (n_x, j)

    def stoppers(self, x):
        (th1, r1, c1, t) = (x.th1, x.r1, int(x.c1), x.t )
        stoppers = []
        # Generate all feasible stoppers
        # For all possible next contact
        # print '-- check :', x
        for c2 in self.prob.next_v[c1]:
            # if c2 != 1:
            #     continue
            e = self.prob.next_e[c1][c2]
            ss = self.rng.stop_sets[e]
            (min_th2, max_th2) = ss.th2_range
            # th2_0 = self.rng.init_angles[e]
            # print c1, c2, 'e', e, 'rng', min_th2, max_th2, 'th2', th2_0
            for th2 in np.linspace(min_th2, max_th2, self.N_GRID):
                # Condition  y2 = r1 * cos(th1) + r2 * cos(th1 + th2) = 0
                r2 = r1 * cos(th1) / -cos(th1 + th2)
                # Check the dynamics
                if not self.rng.query_dynamics(t, c1, c2, th2):
                    # print 'reject the bad dynamics'
                    continue
                # Check the kinematics
                query = (r1, r2, th2 / 10.0)
                if ss.is_new(query, 0.02):
                    # print 'reject the unseen query:', query, ss.min_d(query)
                    continue
                # print 'accept the seen query:', query
                # If the stopper is feasible
                for n_dr1 in np.linspace(self.lo_dr, self.hi_dr, self.N_GRID):
                    stoppers += [Control(th2, r2, n_dr1, c2)]

        # exit(0)
        return stoppers

    def commands(self):
        return [0.2064,
                0.19035830724994546, 0.22504997962813594, 1.8550685725083569]
        # return [0.2064,
        #         0.19116964009964868, 0.18624620237610856, 2.1633800444288918]
        # return [0.2064,
        #         0.1908196400996486, 0.15032234957068108, 2.2683800444288917,
        #         0.15032234957068108, 0.12082146452513869, 1.6606811609717798]

    def plan_initial(self):
        self.upper_bound = g_inf
        j = g_inf
        x = None
        for dr1 in np.linspace(self.lo_dr, self.hi_dr, self.N_GRID):
            x_ = State(self.x0.th1, self.x0.dth1, self.x0.r1, dr1, 0, 0.0)
            x_, j_ = self.plan(x_, 0)
            if j_ < j:
                (x, j) = (x_, j_)
            break
        print 'best: ', x, ':', j
        print '# evals = ', self.eval_counter

        # path = self.db.trace(x)
        # for i, (x, v, u) in enumerate(path):
        #     print i, x, v, u
        self.db.plot_trace(x)
        return j

    def plan(self, x, j):
        # print 'plan()', x, j
        # If the current state has negative velocity
        if self.is_stopped(x):
            # return x, j
            if int(x.c1) == 2:  # If this is the second contact
                return (x, j)
            else:
                return (x, g_inf)  # If this is not the second

        if self.is_grounded(x):  # If the rod falls to the ground
            return (x, g_inf)

        # if int(x.c1) >= self.prob.n:  # Exceed the maximum contacts
        if int(x.c1) >= 2:  # Exceed the maximum contacts
            return (x, g_inf)

        if j > self.upper_bound:  # Not promising state
            return (x, g_inf)

        x_prime, j_prime = self.db.lookup(x)  # Lookup the similar states
        if x_prime is not None:
            # print x, ' == ', x_prime, ':', j, 'vs. ', j_prime
            return (x_prime, j_prime)

        if self.eval_counter % 1000 == 0:
            # print '*',
            # print '\reval_counter:', self.eval_counter, self.upper_bound
            sys.stdout.write('\reval_counter: %r %r'
                             % (self.eval_counter, self.upper_bound))
            sys.stdout.flush()
            if self.eval_counter % 10000 == 0:
                print

        self.eval_counter += 1

        # Case 1: keep falling
        (best_x, best_j) = self.plan(self.step(x), j)
        best_u = None

        # Case 2: use all feasible stoppers
        for u in self.stoppers(x):
            x_impact, j_impact = self.impact(x, u)
            (x_, j_) = self.plan(x_impact, max(j, j_impact))
            if j_ < best_j:
                best_j = j_
                best_x = x_
                best_u = u

        # print x, '->', best_x, best_j, best_u
        self.upper_bound = min(self.upper_bound, best_j)
        self.db.add(x, best_x, best_j, best_u)
        return (x, best_j)

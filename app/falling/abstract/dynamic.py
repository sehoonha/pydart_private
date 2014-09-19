import math
from math import sin, cos
import numpy as np
# from collections import namedtuple
from scipy.integrate import odeint
from state_db import State, Control, get_points, StateDB

g_inf = float("inf")


class DynamicTIP:
    def __init__(self):
        self.eval_counter = 0
        self.N_GRID = 11

        self.m = 1.08
        self.I = 0.0080
        self.g = -9.8

        self.db = StateDB()

    def set_x0(self, tips):
        t0 = tips[0]
        self.x0 = State(t0.theta(), t0.dtheta(), t0.d01(), 0, 0)
        print 'set abstract.DynamicTIP.x0 = ', self.x0

    def set_bounds(self, tips):
        """Bound for control signals: [(dr1, th2, r2), (dr2, th3, r3)]"""
        (self.lo0, self.hi0) = (-0.01, 0.01)
        self.lo = []
        self.hi = []

        # (a, b, c) = (2.3284, 0.1522, -0.1000)
        # (d, e, f) = (1.7107, 0.1108, -0.1000)
        # self.lo = [Control(a, b - g_eps, c), Control(d, e - g_eps, f)]
        # self.hi = [Control(a, b + g_eps, c), Control(d, e + g_eps, f)]

        for i, tip in enumerate(tips):
            (a, d) = (tip.angle(), tip.d12())
            if i == 0:
                self.lo += [Control(a - 1.0, d - 0.02, -0.01)]
                self.hi += [Control(a + 0.05, d + 0.02, 0.01)]
            else:
                self.lo += [Control(a - 1.0, d - 0.02, -0.01)]
                self.hi += [Control(a + 1.0, d + 0.02, 0.01)]

            print 'set abstract.DynamicTIP.lo = ', self.lo[i]
            print 'set abstract.DynamicTIP.hi = ', self.hi[i]

    def test_control(self):
        u = np.array([0.0054, 2.255, 0.1576, -0.00997, 1.583, 0.1215])
        e = 0.01
        self.N_GRID = 1
        (self.lo0, self.hi0) = (u[0], u[0])
        self.lo = [Control(u[1], u[2] - e, u[3]), Control(u[4], u[5] - e, 0.0)]
        self.hi = [Control(u[1], u[2] + e, u[3]), Control(u[4], u[5] + e, 0.0)]

    def deriv(self, _state, _t):
        x = State(*_state)
        (th1, dth1, r1, dr1) = (x.th1, x.dth1, x.r1, x.dr1)

        (m, g) = (self.m, self.g)
        ddth1 = (m * r1 * (2 * dr1 * dth1 - g * sin(th1))) / (m * (r1 ** 2))
        return [dth1, ddth1, dr1, 0, 0]

    def step(self, x):
        # time = np.arange(0.0, 0.01, 0.005)
        time = np.array([0.0, 0.005])
        x0 = np.array(x)
        X = odeint(self.deriv, x0, time)
        return State(*X[-1])

    def is_stopped(self, x):
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
        n_c = x.c + 1
        n_x = State(n_th1, n_dth1, n_r1, n_dr1, n_c)
        return (n_x, j)

    def stoppers(self, x):
        (th1, r1) = (x.th1, x.r1)
        lo = self.lo[int(x.c)]
        hi = self.hi[int(x.c)]
        stoppers = []
        # Generate all feasible stoppers
        for th2 in np.linspace(lo.th2, hi.th2, self.N_GRID):
            # Condition  y2 = r1 * cos(th1) + r2 * cos(th1 + th2) = 0
            r2 = r1 * cos(th1) / -cos(th1 + th2)
            # if int(x.c) == 0:
            #     print 'check', th2, r2, 'is in', lo.r2, hi.r2, 'for', x
            if r2 < lo.r2 or hi.r2 < r2:
                continue
            # If the stopper is feasible
            for n_dr1 in np.linspace(lo.n_dr1, hi.n_dr1, self.N_GRID):
                stoppers += [Control(th2, r2, n_dr1)]
            # stoppers += [Control(th2, r2, 0.0)]
        return stoppers

    def commands(self):
        # return [0.2064,
        #         0.19116964009964868, 0.18624620237610856, 2.1633800444288918]
        return [0.2064,
                0.1908196400996486, 0.15032234957068108, 2.2683800444288917,
                0.15032234957068108, 0.12082146452513869, 1.6606811609717798]

    def plan_initial(self):
        self.upper_bound = g_inf
        j = g_inf
        x = None
        for dr1 in np.linspace(self.lo0, self.hi0, self.N_GRID):
            x_ = State(self.x0.th1, self.x0.dth1, self.x0.r1, dr1, 0)
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
        if self.is_stopped(x):  # If the current state has negative velocity
            if int(x.c) == 1:  # If this is the second contact
                return (x, j)
            else:
                return (x, g_inf)  # If this is not the second

        if self.is_grounded(x):  # If the rod falls to the ground
            return (x, g_inf)

        if int(x.c) == 1:  # Exceed the maximum contacts
            return (x, g_inf)

        if j > self.upper_bound:  # Not promising state
            return (x, g_inf)

        x_prime, j_prime = self.db.lookup(x)  # Lookup the similar states
        if x_prime is not None:
            # print x, ' == ', x_prime, ':', j, 'vs. ', j_prime
            return (x_prime, j_prime)

        if self.eval_counter % 10000 == 0:
            print 'eval_counter:', self.eval_counter, self.upper_bound
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

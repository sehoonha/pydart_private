import math
from math import sin, cos
import numpy as np
from collections import namedtuple
from scipy.integrate import odeint

State = namedtuple('State', ['th1', 'dth1', 'r1', 'dr1', 'c'])
Control = namedtuple('Control', ['th2', 'r2', 'next_dr1'])
Points = namedtuple('Points', ['x1', 'y1', 'dx1', 'dy1',
                               'x2', 'y2', 'dx2', 'dy2'])

g_inf = float("inf")


def get_points(x, u):
    (th1, dth1, r1, dr1) = (x.th1, x.dth1, x.r1, x.dr1)
    (th2, r2) = (u.th2, u.r2)

    x1 = r1 * sin(th1)
    y1 = r1 * cos(th1)
    dx1 = dr1 * sin(th1) + r1 * cos(th1) * dth1
    dy1 = dr1 * cos(th1) - r1 * sin(th1) * dth1

    x2 = x1 + r2 * sin(th1 + th2)
    y2 = y1 + r2 * cos(th1 + th2)
    dx2 = dx1 + r2 * cos(th1 + th2) * dth1
    dy2 = dy1 - r2 * sin(th1 + th2) * dth1
    return Points(x1, y1, dx1, dy1, x2, y2, dx2, dy2)


class DynamicTIP:
    def __init__(self):
        self.eval_counter = 0

        self.m = 1.08
        self.I = 0.0080
        self.g = -9.8

    def set_x0(self, tips):
        t0 = tips[0]
        self.x0 = State(t0.theta(), t0.dtheta(), t0.d01(), 0, 0)
        print 'set abstract.DynamicTIP.x0 = ', self.x0

    def set_bounds(self, tips):
        """Bound for control signals: [(dr1, th2, r2), (dr2, th3, r3)]"""
        self.lo = []
        self.hi = []

        for tip in tips:
            self.lo += [Control(tip.angle() - 1.0, tip.d12() - 0.03, -0.1)]
            self.hi += [Control(tip.angle() + 0.5, tip.d12() + 0.03, 0.1)]

            print 'set abstract.DynamicTIP.lo = ', self.lo[-1]
            print 'set abstract.DynamicTIP.hi = ', self.hi[-1]

    def deriv(self, _state, _t):
        x = State(*_state)
        (th1, dth1, r1, dr1) = (x.th1, x.dth1, x.r1, x.dr1)

        (m, g) = (self.m, self.g)
        ddth1 = (m * r1 * (2 * dr1 * dth1 - g * sin(th1))) / (m * (r1 ** 2))
        return [dth1, ddth1, dr1, 0, 0]

    def step(self, x):
        time = np.arange(0.0, 0.01, 0.005)
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
        n_dr1 = u.next_dr1
        n_c = x.c + 1
        n_x = State(n_th1, n_dth1, n_r1, n_dr1, n_c)
        return (n_x, j)

    def stoppers(self, x):
        (th1, r1) = (x.th1, x.r1)
        lo = self.lo[int(x.c)]
        hi = self.hi[int(x.c)]
        stoppers = []
        # Generate all feasible stoppers
        for th2 in np.linspace(lo.th2, hi.th2, 11):
            # Condition  y2 = r1 * cos(th1) + r2 * cos(th1 + th2) = 0
            r2 = r1 * cos(th1) / -cos(th1 + th2)
            if r2 < lo.r2 or hi.r2 < r2:
                continue
            # If the stopper is feasible
            for next_dr1 in np.linspace(lo.next_dr1, hi.next_dr1, 11):
                stoppers += [Control(th2, r2, next_dr1)]
        return stoppers

    def plan_initial(self):
        j = g_inf
        x = None
        for dr1 in np.linspace(-0.1, 0.1, 11):
            x_ = State(self.x0.th1, self.x0.dth1, self.x0.r1, dr1, 0)
            j_ = self.plan(x_, 0)
            if j_ < j:
                j = j_
                x = x_
        print 'best: ', x, ':', j
        print '# evals = ', self.eval_counter
        return j

    def plan(self, x, j):
        if self.is_stopped(x):
            return j

        if self.is_grounded(x) or int(x.c) == 1:
            return g_inf

        self.eval_counter += 1

        # Case 1: keep falling
        best_x = self.step(x)
        best_j = self.plan(best_x, j)

        # Case 2: use all feasible stoppers
        for u in self.stoppers(x):
            x_impact, j_impact = self.impact(x, u)
            j_ = self.plan(x_impact, max(j, j_impact))
            if j_ < best_j:
                best_j = j_
                best_x = x_impact
        print x, '->', best_j
        return best_j

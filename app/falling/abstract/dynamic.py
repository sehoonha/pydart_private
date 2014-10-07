import sys
import math
from math import sin, cos
import numpy as np
# from collections import namedtuple
# from scipy.integrate import odeint
from state_db import State, Control, PathEntry, get_points, StateDB

g_inf = float("inf")


class DynamicTIP:
    def __init__(self, _prob, _range):
        # strucures
        self.prob = _prob
        self.rng = _range
        self.db = StateDB(self.prob.n)
        # member variables
        self.eval_counter = 0
        self.kill_counter = 0
        self.N_GRID = 11
        # math quantities
        self.m = 1.08
        self.I = 0.0080
        self.g = -9.8
        (self.lo_dr, self.hi_dr) = (-0.1, 0.1)

    def set_x0(self, tips):
        t0 = tips[0]
        self.x0 = State(t0.th1, t0.dth1, t0.r1, 0, 0, 0.0)
        print 'set abstract.DynamicTIP.x0 = ', self.x0

    def deriv(self, _state, _t):
        x = State(*_state)
        (th1, dth1, r1, dr1) = (x.th1, x.dth1, x.r1, x.dr1)

        (m, g) = (self.m, self.g)
        ddth1 = (m * r1 * (2 * dr1 * dth1 - g * sin(th1))) / (m * (r1 ** 2))
        return np.array([dth1, ddth1, dr1, 0, 0, 1.0])

    def step(self, x):
        # time = np.array([0.0, 0.005])
        # x0 = np.array(x)
        # X = odeint(self.deriv, x0, time)
        # return State(*X[-1])
        x0 = np.array(x)
        dx = self.deriv(x0, 0.0)
        x1 = x0 + 0.005 * dx
        return State(*x1)

    def is_stopped(self, x):
        # return (x.c1 != 0)
        return (x.dth1 < 0)

    def is_grounded(self, x):
        return (x.th1 < -0.5 * math.pi) or (0.5 * math.pi < x.th1)

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
        # n_c = x.c1 + 1
        n_c = u.c2
        n_t = x.t
        n_x = State(n_th1, n_dth1, n_r1, None, n_c, n_t)
        return (n_x, j)

    def stoppers(self, x):
        if x.dr1 is None:
            return []

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
                if ss.is_new(query):
                    continue
                stoppers += [Control(th2, r2, c2)]
                # print 'stopper:', stoppers[-1]

        # exit(0)
        return stoppers

    def saved_plan_initial(self):
        # self.x0 = State(0.1992, 0.5370, 0.1909, None, 0, 0.000)
        # self.path =  [PathEntry(x=State(th1=0.1992005703918279, dth1=0.53697920662012044, r1=0.19091957453940681, dr1=None, c1=0, t=0.0), nx_0=State(th1=0.25407882560837419, dth1=1.2308237060282521, r1=0.18571957453940666, dr1=-0.080000000000000002, c1=0.0, t=0.064999999999999988), nx_1=State(th1=-0.37423970510958426, dth1=-0.032748116196193111, r1=0.19312401206426241, dr1=None, c1=1, t=0.064999999999999988), v=0.14318221596194239, v_max=0.14318221596194239, u=Control(th2=2.5132741228718345, r2=0.19312401206426241, c2=1))]

        # self.x0 =  State(0.2011, 1.2389, 0.1906, None, 0, 0.000)
        # self.path =  [PathEntry(x=State(th1=0.2011022918247842, dth1=1.2388944275458982, r1=0.19064937106519639, dr1=None, c1=0, t=0.0), nx_0=State(th1=0.43125501868356747, dth1=3.0588125288227586, r1=0.19754937106519627, dr1=0.059999999999999998, c1=0.0, t=0.11500000000000003), nx_1=State(th1=-0.19706351203439132, dth1=1.7903616252677912, r1=0.18300402394716228, dr1=None, c1=1, t=0.11500000000000003), v=0.28321179706577121, v_max=0.28321179706577121, u=Control(th2=2.5132741228718345, r2=0.18300402394716228, c2=1)), PathEntry(x=State(th1=-0.19706351203439132, dth1=1.7903616252677912, r1=0.18300402394716228, dr1=None, c1=1, t=0.11500000000000003), nx_0=State(th1=0.015404109237411336, dth1=0.71455538985312073, r1=0.16300402394716226, dr1=-0.10000000000000001, c1=1.0, t=0.31500000000000017), nx_1=State(th1=-0.61291442148054731, dth1=-0.31021080280269575, r1=0.19925388009207154, dr1=None, c1=2, t=0.31500000000000017), v=0.071523453350854513, v_max=0.28321179706577121, u=Control(th2=2.5132741228718345, r2=0.19925388009207154, c2=2))]

        self.x0 = State(0.2235, 1.9825, 0.1473, None, 0, 0.000)
        self.path = [PathEntry(x=State(th1=0.2235014833241239, dth1=1.9825153508725917, r1=0.14729444488004678, dr1=None, c1=0, t=0.0), nx_0=State(th1=0.35962907556464102, dth1=3.1569615127400725, r1=0.15279444488004679, dr1=0.10000000000000001, c1=0.0, t=0.054999999999999993), nx_1=State(th1=-0.26868945515331744, dth1=2.2756682929402965, r1=0.14834236981175702, dr1=None, c1=1, t=0.054999999999999993), v=0.1790328772549506, v_max=0.1790328772549506, u=Control(th2=2.5132741228718345, r2=0.14834236981175702, c2=1)), PathEntry(x=State(th1=-0.26868945515331744, dth1=2.2756682929402965, r1=0.14834236981175702, dr1=None, c1=1, t=0.054999999999999993), nx_0=State(th1=-0.321641757097685, dth1=-2.7727464259573318, r1=0.11204236981175766, dr1=-0.060000000000000005, c1=1.0, t=0.66000000000000048), nx_1=State(th1=-0.94996028781564368, dth1=-1.4779127161628589, r1=0.18272952825988312, dr1=None, c1=2, t=0.66000000000000048), v=-0.069693959524558632, v_max=0.1790328772549506, u=Control(th2=2.5132741228718345, r2=0.18272952825988312, c2=2))]

        return 0.4264

    def plan_initial(self):
        # return self.saved_plan_initial()

        self.upper_bound = g_inf
        # self.upper_bound = 0.52
        x = State(self.x0.th1, self.x0.dth1, self.x0.r1, None, 0, 0.0)
        x_, j = self.plan(x, 0)
        print 'best: ', x, ':', j
        print '# evals = ', self.eval_counter, self.kill_counter

        # path = self.db.trace(x)
        # for i, (x, v, u) in enumerate(path):
        #     print i, x, v, u
        self.db.plot_trace(x)
        # exit(0)

        self.x0 = x
        # self.path = self.db.trace_impacts(x)
        self.path = self.db.trace(x)
        return j

    def plan(self, x, j):
        # If the current state has negative velocity
        if self.is_stopped(x):
            return x, j
            # if int(x.c1) == 2:  # If this is the second contact
            #     return (x, j)
            # else:
            #     return (x, g_inf)  # If this is not the second

        if self.is_grounded(x):  # If the rod falls to the ground
            return (x, g_inf)

        # if int(x.c1) >= 2:  # Exceed the maximum contacts
        #     return (x, g_inf)

        if j > self.upper_bound:  # Not promising state
            return (x, g_inf)

        # print
        # print '>> lookup', x
        x_prime, j_prime = self.db.lookup(x)  # Lookup the similar states
        # print '>> x_prime = ', x_prime
        # print
        if x_prime is not None:
            self.kill_counter += 1
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
        print self.eval_counter, 'plan()', x, j, len(self.db)

        # (best_x, best_max_j, best_now_j, best_u) = (None, g_inf, None, None)
        best_j = g_inf
        best_entry = PathEntry(x, None, None, g_inf, g_inf, None)
        for n_dr1 in np.linspace(self.lo_dr, self.hi_dr, self.N_GRID):
            x_now = State(x.th1, x.dth1, x.r1, n_dr1, x.c1, x.t)
            while not self.is_grounded(x_now):
                for u in self.stoppers(x_now):
                    x_impact, j_impact = self.impact(x_now, u)
                    # print u, x_impact, j_impact
                    (x_, j_) = self.plan(x_impact, max(j, j_impact))
                    if j_ < best_j:
                        best_j = j_
                        best_entry = PathEntry(x, x_now, x_,
                                               j_impact, j_, u)

                x_now = self.step(x_now)
        self.upper_bound = min(self.upper_bound, best_j)
        self.db.add(x, best_entry)
        return (x, best_j)

import math
import tip
import numpy as np

import cma
import plotly.plotly as py
from plotly.graph_objs import *

class TWOTIP:
    def __init__(self):
        self.tips = [tip.TIP(), tip.TIP()]
        self.first_tip = self.tips[0]
        self.second_tip = self.tips[1]

    def set_x0(self, tips):
        (t0, t1) = (tips[0], tips[1])
        self.x0 = [t0.theta(), t0.dtheta(), t0.d01(), 0]
        self.first_tip.control0 = [0.0, t0.angle(), t0.d12()]
        self.second_tip.control0 = [0.0, t1.angle(), t1.d12()]

    def set_bounds(self, tips):
        """Bound for control signals: [(dr1, th2, r2), (dr2, th3, r3)]"""
        (t0, t1) = (tips[0], tips[1])
        self.lo = [-0.01, t0.angle() - 1.0, t0.d12() - 0.01,
                   -0.01, t1.angle() - 1.0, t1.d12() - 0.02,]
        self.hi = [ 0.01, t0.angle() + 0.03, t0.d12() + 0.01,
                    0.01, t1.angle() + 1.0, t1.d12() + 0.02, ]
        self.lo = np.array(self.lo)
        self.hi = np.array(self.hi)
        print 'set abstract.model.TIP.lo = ', self.lo
        print 'set abstract.model.TIP.hi = ', self.hi

    def simulate(self):
        # Simulate the first TIP
        print
        print '> new simulation'
        print self.control
        self.first_tip.x0 = self.x0
        X1 = self.first_tip.simulate()
        self.first_tip.bake_states(X1)

        # Estimate the next tip
        u = self.first_tip.control
        x_f = X1[-1] # final frame
        n_th1 = self.first_tip.th1(x_f, u) + self.first_tip.th2(x_f, u) - math.pi
        n_dth1 = self.first_tip.estimate_next_dtheta(x_f)
        n_r1 = self.first_tip.r2(x_f, u)
        print 'estimated next th_1:', n_dth1


        print "x_1 : ", x_f, "j_1 : ", self.first_tip.estimate_impact(x_f)
        print 'first_tip = ', self.first_tip.to_str(x_f, u)
        # Simulate the second TIP
        self.second_tip.x0 = (n_th1, n_dth1, n_r1, 0)
        X2 = self.second_tip.simulate()
        u2 = self.second_tip.control
        x_f2 = X2[-1]
        self.second_tip.bake_states(X2)
        print "x_2 : ", X2[-1], "j_2 : " , self.second_tip.estimate_impact(X2[-1])
        print 'second_tip = ', self.second_tip.to_str(x_f2, u2)
        n_dth1_2  = self.second_tip.estimate_next_dtheta(x_f2)
        print 'estimated next th_2:', n_dth1_2
        print

        return np.concatenate((X1, X2))

    def simulate_random(self):
        # u = np.concatenate((self.first_tip.control0, self.second_tip.control0))
        # u = np.random.rand(6) * (self.hi - self.lo) + self.lo
        u = np.array([-8.61745591e-03,   1.61526275e+00,   1.57589978e-01,  -2.85106538e-03, 3.21096862e+00,   1.21509601e-01])
        self.control = u
        self.first_tip.control = np.array(u[:3])
        self.second_tip.control = np.array(u[3:])
        print 'u_0 = ', self.first_tip.control
        print 'u_1 = ', self.second_tip.control
        self.simulate()
        self.plot_poses()

    def evaluate(self, u):
        self.control = u
        self.first_tip.control = np.array(u[:3])
        self.second_tip.control = np.array(u[3:])
        penalty = 0
        for i, (lo, hi) in enumerate(zip(self.lo, self.hi)):
            if u[i] < lo:
                penalty += (lo - u[i]) ** 2
            if u[i] > hi:
                penalty += (hi - u[i]) ** 2
        if penalty > 0:
            return 10.0 + penalty

        X = self.simulate()
        j = self.second_tip.estimate_impact(X[-1])
        cost = j
        print 'cost = ', cost
        return cost

    def optimize(self):
        opt = {'verb_time':0,
               # 'boundary_handling': 'BoundPenalty', 'bounds': [self.lo, self.hi],
               'tolfun' : 0.001}
        print "==== abstract.model.TWOTIP optimize...."

        self.res = cma.fmin(self.evaluate, 0.5 * (self.lo + self.hi), 0.01, opt)
        print self.evaluate(self.control)
        print "==== result\n ", self.res
        print 'solution = ', self.control
        print "==== abstract.model.TWOTIP optimize.... OK"
        self.plot_poses()

    def commands(self):
        return np.concatenate( ([self.x0[0]], self.first_tip.commands(), self.second_tip.commands()) )
        
    def plot_poses(self):
        print '==== plot_poses ...'
        traces = []
        traces += self.first_tip.plot_poses(0.0, True)
        x2 = self.first_tip.x2(self.first_tip.data[-1], self.first_tip.control)
        traces += self.second_tip.plot_poses(x2, True)
        data = Data(traces)
        layout = Layout(xaxis=XAxis(range=[0.0, 0.4]),  yaxis=YAxis(range=[0.0, 0.4]) )

        py.image.save_as({'data': data, 'layout':layout}, 'abstract_twotip.png', height = 900, width = 1000)
        print '==== plot_poses OK'

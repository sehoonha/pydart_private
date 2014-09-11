import numpy as np
import math
from math import sin, cos
from numpy.linalg import norm
from scipy.integrate import odeint
from collections import namedtuple

Control = namedtuple('Control', ['dr1', 'th2', 'r2'])
State = namedtuple('State', ['th1', 'dth1', 'r1', 'stop'])
Quantities = namedtuple('Quantities', ['x1', 'y1', 'dx1', 'dy1',
                                       'x2', 'y2', 'dx2', 'dy2'])

class TIPv2:
    def __init__(self):
        self.data = None
        # Important quantities
        self.m = 1.08
        self.I = 0.0080
        self.g = -9.8
        self.h = 0.0005
        

    def set_x0(self, tip):
        self.x0 = State(tip.theta(), tip.dtheta(), tip.d01(), 0)
        self.control0 = Control(0.0, tip.angle(), tip.d12())
        print 'set abstract.model.TIPv2.x0 = ', self.x0
        print 'set abstract.model.TIPv2.control0 = ', self.control0


    def set_bounds(self, tip):
        self.lo = Control(-0.1, tip.angle() - 1.0, tip.d12() - 0.03)
        self.hi = Control( 0.1, tip.angle() + 0.5, tip.d12() + 0.03)

        print 'set abstract.model.TIPv2.lo = ', self.lo
        print 'set abstract.model.TIPv2.hi = ', self.hi

    def deriv(self, _state, _t):
        x = State(*_state)
        (th1, dth1, r1, stop) = (x.th1, x.dth1, x.r1, x.stop)

        # When it hits the ground
        if stop > 0 or th1 > (0.5 * math.pi):
            return [0, 0, 0, 1]

        u = self.control
        (dr1, th2, r2) = (u.dr1, u.th2, u.r2)

        (m, I, g, h) = (self.m, self.I, self.g, self.h)
        ddth1 = (m * r1 * (2 * dr1 * dth1 - g * sin(th1))) / (m * (r1**2))
        return [dth1, ddth1, dr1, 0]

    def quantities(self, x, u):
        (th1, dth1, r1, stop) = (x.th1, x.dth1, x.r1, x.stop)
        (dr1, th2, r2) = (u.dr1, u.th2, u.r2)

        x1 = r1 * sin(th1)
        y1 = r1 * cos(th1)
        dx1 = dr1 * sin(th1) + r1 * cos(th1) * dth1
        dy1 = dr1 * cos(th1) - r1 * sin(th1) * dth1

        x2 = x1 + r2 * sin(th1 + th2)
        y2 = y1 + r2 * cos(th1 + th2)
        dx2 = dx1 + r2 * cos(th1 + th2) * dth1;
        dy2 = dy1 - r2 * sin(th1 + th2) * dth1;
        return Quantities(x1, y1, dx1, dy1, x2, y2, dx2, dy2)
        
        
    def estimate_impact(self, x, u):
        q = self.quantities(x, u)
        (x1, x2, dy2) = (q.x1, q.x2, q.dy2)
        (m, I) = (self.m, self.I)
        j = (-dy2) / ( (1/m) + (1/I) * ((x2 - x1) ** 2) )
        return j
        

    def check_as_final_state(self, _state, _dr1):
        x = _state if isinstance(_state, State) else State(*_state)
        (th1, r1) = (x.th1, x.r1)

        tests = []
        for th2 in np.linspace(self.lo.th2, self.hi.th2, 11):
            ### Condition  y2 = r1 * cos(th1) + r2 * cos(th1 + th2) = 0
            r2 = r1 * cos(th1) / -cos(th1 + th2)
            if r2 < self.lo.r2 or self.hi.r2 < r2:
                continue
            
            # Okay, the acceptable final state
            u = Control(_dr1, th2, r2)
            j = self.estimate_impact(x, u)
            # print x, u, '-->', j
            tests += [(j, x, u)]
        return tests
        
    def simulate(self):
        time = np.arange(0.0, 5.0, self.h)
        X = odeint(self.deriv, self.x0, time)
        X = np.array([x for x in X if x[-1] == 0]) 
        return X

    def optimize(self):
        print "==== abstract.model.TIPv2 optimize...."
        tests = []
        for dr1 in np.linspace(self.lo.dr1, self.hi.dr1, 11):
            print 'dr1 = ', dr1
            self.control = Control(dr1=dr1, th2=0.0, r2=1.8)
            X = self.simulate()
            print X
            for x in X:
                tests += self.check_as_final_state(x, dr1)
        optimal_index = np.argmin([j for (j, x, u) in tests])
        self.optimal_test = tests[optimal_index]
        print 'best control = ', self.control
        print "==== abstract.model.TIPv2 optimize.... OK"

    def commands(self):
        (j, x, u) = self.optimal_test
        return (x.r1, u.r2, u.th2)


    

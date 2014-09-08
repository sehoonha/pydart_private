import csv

from math import sin, cos
import numpy as np
from numpy.linalg import norm
from scipy.integrate import odeint
from scipy.optimize import minimize
import cma

import plotly.plotly as py
from plotly.graph_objs import *

class TIP:
    def __init__(self):
        self.data = None
        self.m = 1.08
        self.I = 0.0093
        self.g = -9.8
        self.h = 0.0005
        self.x0 = [0.123, 0, 0.18, 0]
        self.header = ['t', 'th', 'dth', 'r', 'C.x', 'C.y', 'dr', 'alpha', 'l']
        self.index = dict(zip(self.header, range(len(self.header))))
        pass

    def load_history(self, _csvfilename):
        self.data = []
        with open(_csvfilename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.data += [[float(x) for x in row ]]
        # print self.data
        # print [self.data[i][self.index['t']] for i in range(len(self.data))]

    def deriv(self, state, t):
        (th, dth, r, stop) = tuple(state)
        (x, y, Px, Py, x2, y2, dx2, dy2) = self.quantities(state)

        if stop > 0 or y2 < 0:
            return [0, 0, 0, 1]
            
        (dr, th2, r2) = tuple(self.control)
        ddth = (self.m*r*(2*dr*dth - self.g*sin(th))) / (self.m*(r**2))
        return [dth, ddth, dr, 0]

    def quantities(self, state):
        (th, dth, r, stop) = tuple(state)
        (dr, th2, r2) = tuple(self.control)
        
        x = r * sin(th)
        y = r * cos(th)
        dx = dr * sin(th) + r * cos(th) * dth
        dy = dr * cos(th) - r * sin(th) * dth
        x2 = x + r2 * sin(th + th2)
        y2 = y + r2 * cos(th + th2)

        dx2 = dx + r2 * cos(th + th2) * dth;
        dy2 = dy - r2 * sin(th + th2) * dth;

        Px, Py = (self.m * dx, self.m * dy)
        return (x, y, Px, Py, x2, y2, dx2, dy2)

    def set_x0(self, tip):
        self.x0 = [tip.theta(), tip.dtheta(), tip.d01(), 0]
        self.control0 = [0.0, tip.angle(), tip.d12()]
        print 'set abstract.model.TIP.x0 = ', self.x0

    def set_bounds(self, tip):
        self.lo = np.array([-0.1, tip.angle() - 1.0, tip.d12() - 0.03])
        self.hi = np.array([ 0.1, tip.angle() + 0.5, tip.d12() + 0.03])
        # self.lo = np.array([-0.2, tip.angle() - 1.0, tip.d12() - 0.04])
        # self.hi = np.array([ 0.2, tip.angle() + 1.2, tip.d12() + 0.07])
        print 'set abstract.model.TIP.lo = ', self.lo
        print 'set abstract.model.TIP.hi = ', self.hi

        
    def simulate(self):
        # x0 = [0.123, 0, 0.18, 0] # Last zeromeans state is not terminated
        x0 = self.x0
        time = np.arange(0.0, 5.0, self.h)
        X = odeint(self.deriv, x0, time)
        X = np.array([x for x in X if x[-1] == 0]) 
        return X

    def estimate_impact(self, x):
        (th, dth, r, stop) = tuple(x)
        (dr, th2, r2) = tuple(self.control)
        (x1, y1, Px, Py, x2, y2, dx2, dy2) = self.quantities(x)
        m = self.m
        I = self.I
        # j = dy2^{-} / ( (1/m) + (1/I) * (x2 - x1)^2 )
        j = (-dy2) / ( (1/m) + (1/I) * ((x2 - x1) ** 2) )
        return j
    def estimate_next_dtheta(self, x):
        j = self.estimate_impact(x)
        (th, dth, r, stop) = tuple(x)
        (x1, y1, Px, Py, x2, y2, dx2, dy2) = self.quantities(x)
        I = self.I
        # dth^{+} = dth^{-} - (1/I) (x2 - x1)
        dth_next = dth - (1 / I) * (x2 - x1) * j
        return dth_next

    def evaluate(self, x):
        self.control = x
        penalty = 0
        for i, (lo, hi) in enumerate([(-0.2, 0.2), (0.0, 3.0), (0.10, 0.17)]):
            if x[i] < lo:
                penalty += (lo - x[i]) ** 2
            if x[i] > hi:
                penalty += (hi - x[i]) ** 2

        
        if penalty > 0:
            return 10.0 + penalty
                
        X = self.simulate()
        (x, y, Px, Py, x2, y2, dx2, dy2) = self.quantities(X[-1])
        (r1, r2, th2) = (X[-1][2], self.control[2], self.control[1])
        # if r1 + r2 > 0.32:
        #     return 9.0 + (r1 + r2)

        # cost = -1.0 * Py
        # cost = self.estimate_impact(X[-1])
        cost = -1.0 * y

        # cost = -1.0 * dy2
        # cost = -x2
        # print self.control, 0.0005 * len(X), X[-1], -1.0 * cost
        # print -1.0 * Py, self.estimate_impact(X[-1]), -1.0 * y
        print '>> th1 = %.4f dth1 = %.4f' % (X[-1][0], X[-1][1]),
        print 'r1 = %.4f r2 = %.4f th2 = %.4f' % (X[-1][2], self.control[2], self.control[1])
        print 'P.y = %.4f Impact = %.4f C.y = %.4f ' % (Py, self.estimate_impact(X[-1]), y),
        print 'Dtheta_next = %.4f' % (self.estimate_next_dtheta(X[-1]))

        return cost

    def optimize(self):
        # lo = np.array([-0.2, 0.0, 0.10])
        # hi = np.array([ 0.2, 3.0, 0.17])

        opt = {'verb_time':0,  'boundary_handling': 'BoundPenalty', \
               'bounds': [self.lo, self.hi], 'tolfun' : 0.001}
        print "==== abstract.model.TIP optimize...."

        self.res = cma.fmin(self.evaluate, 0.5 * (self.lo + self.hi), 0.03, opt)

        self.control = self.res[0]
        print "==== result\n ", self.res
        print 'solution = ', self.control
        print "==== abstract.model.TIP optimize.... OK"

        ## Put simulation result into self.data
        (dr, th2, r2) = tuple(self.control)
        X = self.simulate()
        self.data = []
        for i, x in enumerate(X):
            t = self.h * i
            (Cx, Cy, Px, Py, x2, y2, dx2, dy2) = self.quantities(x)
            self.data += [ [t, x[0], x[1], x[2], Cx, Cy, dr, th2, r2] ]
        print 'Estimated impact = ', self.estimate_impact(X[-1])
        # self.plotData()

    def column(self, name):
        return [self.data[i][self.index[name]] for i in range(len(self.data))]
        
    def commands(self):
        if self.data is None:
            return [0.0, 0.0,0.0]
        else:
            return [self.data[-1][self.index[name]] for name in ['r', 'l', 'alpha']]
                
    def plot(self, colors):
        if self.data is None:
            return []
        x = self.column('t')
        traces = []

        for i, name in enumerate(['th', 'r', 'C.x', 'C.y']):
            y = self.column(name)
            line = Line(dash='dash', color = colors[i])
            traces += [ Scatter(x=x,y=y,name='TIP_%s' % name, line=line) ]
        return traces

    def plotData(self):
        traces = []
        for i, ctrl in [(0, self.control0), (len(self.data) - 1, self.control)]:
            self.control = ctrl
            state = self.data[i][1:4] + [0]
            print i, state, self.quantities(state)
            (Cx, Cy, Px, Py, x2, y2, dx2, dy2) = self.quantities(state)
            x = [0, Cx, x2]
            y = [0, Cy, y2]
            traces += [ Scatter(x=x,y=y,name='Frame%d' % i)]
        data = Data(traces)
        layout = Layout(xaxis=XAxis(range=[0.0, 0.3]),  yaxis=YAxis(range=[0.0, 0.3]) )

        py.image.save_as({'data': data, 'layout':layout}, 'abstract_tip.png', height = 900, width = 1000)
        # unique_url = py.plot({'data': data, 'layout':layout}, filename = 'Abstract TIP')

            

        

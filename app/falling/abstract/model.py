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
        self.g = -9.8
        self.h = 0.0005
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
        (x, y, Px, Py, x2, y2) = self.quantities(state)

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

        Px, Py = (self.m * dx, self.m * dy)
        return (x, y, Px, Py, x2, y2)
        
    def simulate(self):
        x0 = [0.123, 0, 0.18, 0] # Last zeromeans state is not terminated
        time = np.arange(0.0, 5.0, self.h)
        X = odeint(self.deriv, x0, time)
        X = np.array([x for x in X if x[-1] == 0]) 
        return X

    def evaluate(self, x):
        penalty = 0
        for i, (lo, hi) in enumerate([(-0.2, 0.2), (0.0, 3.0), (0.10, 0.17)]):
            if x[i] < lo:
                penalty += (lo - x[i]) ** 2
            if x[i] > hi:
                penalty += (hi - x[i]) ** 2
        if penalty > 0:
            return 10.0 + penalty
                
        self.control = x
        X = self.simulate()
        (x, y, Px, Py, x2, y2) = self.quantities(X[-1])
        cost = -1.0 * Py
        # print self.control, 0.0005 * len(X), X[-1], -1.0 * cost
        return cost

    def optimize(self):
        lo = np.array([-0.2, 0.0, 0.10])
        hi = np.array([ 0.2, 3.0, 0.17])
        opt = {'verb_time':0,  'boundary_handling': 'BoundPenalty', \
               'bounds': [lo, hi], 'tolfun' : 0.001}
        self.res = cma.fmin(self.evaluate, 0.5 * (lo + hi), 0.03, opt)

        self.control = self.res[0]
        print "== abstract.model.TIP optimize. resultf ==\n ", self.res
        print 'solution = ', self.control

        (dr, th2, r2) = tuple(self.control)
        X = self.simulate()
        self.data = []
        for i, x in enumerate(X):
            t = self.h * i
            (Cx, Cy, Px, Py, x2, y2) = self.quantities(X[-1])
            self.data += [ [t, x[0], x[1], x[2], Cx, Cy, dr, th2, r2] ]

    def column(self, name):
        return [self.data[i][self.index[name]] for i in range(len(self.data))]
        
    def commands(self):
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
            

        

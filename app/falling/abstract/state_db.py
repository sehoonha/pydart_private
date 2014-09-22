from collections import namedtuple
from math import sin, cos, sqrt
import numpy as np
import plotly.plotly as py
import plotly.graph_objs as pyg


class State(namedtuple('State', ['th1', 'dth1', 'r1', 'dr1', 'c', 't'])):
    __slots__ = ()

    def __str__(self):
        return 'State(%.4f, %.4f, %.4f, %.4f, %d, %.3f)' % self


class Control(namedtuple('Control', ['th2', 'r2', 'n_dr1'])):
    __slots__ = ()

    def __str__(self):
        return 'Control(%.4f, %.4f, %.4f)' % self

Points = namedtuple('Points', ['x1', 'y1', 'dx1', 'dy1',
                               'x2', 'y2', 'dx2', 'dy2'])


def get_first_point(x):
    (th1, dth1, r1, dr1) = (x.th1, x.dth1, x.r1, x.dr1)
    x1 = r1 * sin(th1)
    y1 = r1 * cos(th1)
    dx1 = dr1 * sin(th1) + r1 * cos(th1) * dth1
    dy1 = dr1 * cos(th1) - r1 * sin(th1) * dth1
    return Points(x1, y1, dx1, dy1, 0, 0, 0, 0)


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


class StateDB(object):
    def __init__(self):
        self.info = {}

    def reset(self):
        self.info = {}

    def __len__(self):
        return len(self.info)

    def add(self, x, next_x, v, u=None):
        self.info[x] = (next_x, v, u)

    def trace(self, x):
        if x not in self.info:
            return [(x, None, None)]
        (next_x, v, u) = self.info[x]
        return [(x, v, u)] + self.trace(next_x)

    def lookup(self, x):
        w = np.array([1.0, 0.1, 1.0, 100.0, 100.0])
        for s, (next_s, v, u) in self.info.iteritems():
            lhs = np.array(x)
            rhs = np.array(s)
            d = w.dot((lhs - rhs) ** 2)
            if d < 0.01:
                return (s, v)
        return (None, None)

    def foo(self):
        states = [n_x for n_x, v, u in self.info.values() if int(n_x.c) == 1]
        states.sort()
        for s in states:
            print s
        print '# total %s states' % len(states)
        exit(0)

    def plot_trace(self, x):
        # self.foo()
        path = self.trace(x)
        traces = []
        x_offset = 0.0
        cmds = []
        for i, (x, v, u) in enumerate(path):
            print i, x, v, u
            if u is not None:
                p = get_points(x, u)
                x = np.array([0.0, p.x1, p.x2]) + x_offset
                y = np.array([0.0, p.y1, p.y2])
                print 'line:', zip(x, y)
                traces += [pyg.Scatter(x=x, y=y, name='TIP%d' % i)]
                x_offset = x[-1]
                print 'x_offset:', x_offset

                r1 = sqrt(p.x1 ** 2 + p.y1 ** 2)
                r2 = sqrt((p.x1 - p.x2) ** 2 + (p.y1 - p.y2) ** 2)
                th2 = u.th2
                cmds += [r1, r2, th2]
            elif i == 0 or i == len(path) - 1:
                p = get_first_point(x)
                x = np.array([0.0, p.x1]) + x_offset
                y = np.array([0.0, p.y1])
                print 'line: ', zip(x, y)
                traces += [pyg.Scatter(x=x, y=y, name='TIP%d' % i)]

        data = pyg.Data(traces)
        layout = pyg.Layout(xaxis=pyg.XAxis(range=[-0.1, 0.4]),
                            yaxis=pyg.YAxis(range=[-0.1, 0.4]))

        unique_url = py.plot({'data': data, 'layout': layout},
                             filename='Abstract TIP')
        print '==== plot_trace OK : ', unique_url
        print 'collected commands = ', cmds

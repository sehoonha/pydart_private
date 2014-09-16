from collections import namedtuple
from math import sin, cos
import numpy as np
import plotly.plotly as py
import plotly.graph_objs as pyg


class State(namedtuple('State', ['th1', 'dth1', 'r1', 'dr1', 'c'])):
    __slots__ = ()

    def __str__(self):
        return 'State(%.4f, %.4f, %.4f, %.4f, %d)' % self


class Control(namedtuple('Control', ['th2', 'r2', 'next_dr1'])):
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
        pass

    def plot_trace(self, x):
        path = self.trace(x)
        traces = []
        for i, (x, v, u) in enumerate(path):
            print i, x, v, u
            if i == 0:
                p = get_first_point(x)
                x = np.array([0.0, p.x1])
                y = np.array([0.0, p.y1])
                print 'line: ', x, y
                traces += [pyg.Scatter(x=x, y=y, name='TIP%d' % i)]
            elif u is not None:
                p = get_points(x, u)
                x = np.array([0.0, p.x1, p.x2])
                y = np.array([0.0, p.y1, p.y2])
                print 'line: ', x, y
                traces += [pyg.Scatter(x=x, y=y, name='TIP%d' % i)]

        data = pyg.Data(traces)
        layout = pyg.Layout(xaxis=pyg.XAxis(range=[0.0, 0.3]),
                            yaxis=pyg.YAxis(range=[0.0, 0.3]))

        unique_url = py.plot({'data': data, 'layout': layout},
                             filename='Abstract TIP')
        print '==== plot_trace OK : ', unique_url

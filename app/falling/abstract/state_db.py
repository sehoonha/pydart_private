from collections import namedtuple
from math import sin, cos, sqrt
import numpy as np
import plotly.plotly as py
import plotly.graph_objs as pyg
from nearpy import Engine
from nearpy.hashes import RandomBinaryProjections


class State(namedtuple('State', ['th1', 'dth1', 'r1', 'dr1', 'c1', 't'])):
    __slots__ = ()

    def __str__(self):
        if self.dr1 is None:
            return 'State(%.4f, %.4f, %.4f, %r, %d, %.3f)' % self
        else:
            return 'State(%.4f, %.4f, %.4f, %.4f, %d, %.3f)' % self


class Control(namedtuple('Control', ['th2', 'r2', 'c2'])):
    __slots__ = ()

    def __str__(self):
        return 'Control(%.4f, %.4f, %d)' % self

Points = namedtuple('Points', ['x1', 'y1', 'dx1', 'dy1',
                               'x2', 'y2', 'dx2', 'dy2'])


def get_first_point(x):
    (th1, dth1, r1, dr1) = (x.th1, x.dth1, x.r1, x.dr1)
    x1 = r1 * sin(th1)
    y1 = r1 * cos(th1)

    (dx1, dy1) = (0.0, 0.0)
    if dr1 is not None:
        dx1 = dr1 * sin(th1) + r1 * cos(th1) * dth1
        dy1 = dr1 * cos(th1) - r1 * sin(th1) * dth1
    return Points(x1, y1, dx1, dy1, 0, 0, 0, 0)


def get_points(x, u):
    (th1, dth1, r1, dr1) = (x.th1, x.dth1, x.r1, x.dr1)
    (th2, r2) = (u.th2, u.r2)

    x1 = r1 * sin(th1)
    y1 = r1 * cos(th1)
    (dx1, dy1) = (0.0, 0.0)
    if dr1 is not None:
        dx1 = dr1 * sin(th1) + r1 * cos(th1) * dth1
        dy1 = dr1 * cos(th1) - r1 * sin(th1) * dth1

    x2 = x1 + r2 * sin(th1 + th2)
    y2 = y1 + r2 * cos(th1 + th2)
    (dx2, dy2) = (0.0, 0.0)
    if dr1 is not None:
        dx2 = dx1 + r2 * cos(th1 + th2) * dth1
        dy2 = dy1 - r2 * sin(th1 + th2) * dth1
    return Points(x1, y1, dx1, dy1, x2, y2, dx2, dy2)


class StateDBEngine(object):
    def __init__(self):
        # initialize "nearby" library
        self.dim = 4
        self.rbp = RandomBinaryProjections('rbp', 100)
        self.engine = Engine(self.dim, lshashes=[self.rbp])
        # performance counter
        self.counter = 0

    def add(self, x, data):
        self.engine.store_vector(x, data)
        self.counter += 1

    def lookup(self, x, THRESHOLD=0.01):
        naver = self.engine.neighbours(x)
        for pt, data, d in naver:
            if d < THRESHOLD:
                return data
        return None


class PathEntry(namedtuple('PathEntry',
                           ['x', 'nx_0', 'nx_1', 'v', 'v_max', 'u'])):
    __slots__ = ()

    def __str__(self):
        return 'PathEntry(%r, %r, %r, %.4f, %.4f, %r)' % self


class StateDB(object):
    def __init__(self, _n):
        self.n = _n
        self.reset()

    def reset(self):
        n = self.n
        self.info = {}
        self.w = np.array([1.0, 0.1, 1.0, 1.0])
        self.engines = [StateDBEngine() for _ in range(n)]

    def __len__(self):
        return len(self.info)

    def find_engine(self, x):
        c = int(x.c1)
        return self.engines[c]

    def to_query(self, x):
        t = min(x.t, 0.2)
        return self.w * np.array([x.th1, x.dth1, x.r1, t])

    # def add(self, x, next_x, v, u=None):
    # def add(self, x, nx_0, nx_1, v, v_max, u=None):
    def add(self, x, entry):
        """
        x: the current tip at begining of the falling
        nx_0: the curent tip right before the impact
        nx_1: the next tip right after the impact (woudl be next x)
        v: current impulse
        v_max: maximul impulse to the end
        """
        q = self.to_query(x)
        # entry = PathEntry(x, nx_0, nx_1, v, v_max, u)
        self.find_engine(x).add(q, entry)
        self.info[x] = entry

    def trace(self, x):
        if x not in self.info:
            return []
        entry = self.info[x]
        return [entry] + self.trace(entry.nx_1)

    def trace_impacts(self, x0):
        path = [entry for entry in self.trace(x0) if entry.u is not None]
        return path

    def lookup(self, x):
        q = self.to_query(x)
        entry = self.find_engine(x).lookup(q)
        if entry is None:
            return (None, None)
        return (entry.x, entry.v_max)

        # d = self.find_engine(x).min_d(lhs)
        # if d < 0.01:
        #     return True
        # for s, (next_s, v, u) in self.info.iteritems():
        #     lhs = np.array(x)
        #     rhs = np.array(s)
        #     d = w.dot((lhs - rhs) ** 2)
        #     if d < 0.01:
        #         return (s, v)
        # return (None, None)

    def plot_trace(self, x):
        path = self.trace(x)
        traces = []
        x_offset = 0.0
        cmds = []
        for i, entry in enumerate(path):
            print
            print 'path entry ', i
            print 'impulse:', entry.v
            print 'x0:', entry.x
            print 'nx_0:', entry.nx_0
            print 'nx_1:', entry.nx_1

            if entry.u is not None:
                p0 = get_first_point(entry.x)
                p = get_points(entry.nx_0, entry.u)
                x = np.array([p0.x1, 0.0, p.x1, p.x2]) + x_offset
                y = np.array([p0.y1, 0.0, p.y1, p.y2])
                print 'line:', zip(x, y)
                traces += [pyg.Scatter(x=x, y=y, name='TIP%d' % i)]
                x_offset = x[-1]
                print 'x_offset:', x_offset

                r1 = sqrt(p.x1 ** 2 + p.y1 ** 2)
                r2 = sqrt((p.x1 - p.x2) ** 2 + (p.y1 - p.y2) ** 2)
                th2 = entry.u.th2
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

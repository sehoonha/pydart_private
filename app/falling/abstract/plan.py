import numpy as np
import math
from state_db import State, get_points
# Plotting
import plotly.plotly as py
import plotly.graph_objs as pyg


class Plan:
    def __init__(self, _x0, _path):
        self.x0 = _x0
        self.path = _path
        self.initialize()

    def initialize(self):
        self.states = [self.x0] + [entry.nx_0 for entry in self.path]
        self.controls = [entry.u for entry in self.path]

    @property
    def n(self):
        """Num contacts"""
        return len(self.controls)

    def state(self, index):
        if index == 0:
            # x0 = self.states[index]
            # x1 = self.states[index + 1]
            # return State(x0.th1, 0, x1.r1, 0, 0, 0)
            x1 = self.states[index + 1]
            return State(x1.th1, 0, x1.r1, 0, 0, 0)
        else:
            # x0 = self.states[index]
            # x1 = self.states[index + 1]
            # u0 = self.controls[index - 1]
            # th1 = x0.th1 + u0.th2 - math.pi
            # return State(th1, 0, x1.r1, 0, 0, 0)
            x1 = self.states[index + 1]
            return State(x1.th1, 0, x1.r1, 0, 0, 0)

    def control(self, index):
        return self.controls[index]

    def C(self, index):
        (x, u) = self.state(index), self.control(index)
        p = get_points(x, u)
        return np.array([p.x1, p.y1])

    def C_rel(self, index):
        c = self.C(index)
        p = self.P(index)
        d = p[0]
        return c / d

    def base_dist(self, index):
        p = self.P(index)
        d = p[0]
        return d

    def P(self, index):
        (x, u) = self.state(index), self.control(index)
        p = get_points(x, u)
        return np.array([p.x2, p.y2])

    def J(self, index):
        impulses = [entry.v for entry in self.path]
        return impulses[index]

    def contact1(self, index):
        return int(self.states[index + 1].c1)

    def contact2(self, index):
        return int(self.controls[index].c2)

    def __str__(self):
        ret = "== Plan ==\n"
        for i, entry in enumerate(self.path):
            ret += '\n'
            ret += 'Contact %d' % i + '\n'
            ret += 'impulse: ' + str(entry.v) + '\n'
            ret += 'x0: ' + str(entry.x) + '\n'
            ret += 'nx_0: ' + str(entry.nx_0) + '\n'
            ret += 'u: ' + str(entry.u) + '\n'
            ret += 'nx_1: ' + str(entry.nx_1) + '\n'
        return ret

    def plot(self):
        """Plot the configuration with the given index of collision"""
        traces = []
        min_offset = 0.0
        max_offset = 0.0
        x_offset = 0.0
        for i in range(self.n):
            c = self.C(i)
            p = self.P(i)
            j = self.J(i)
            x = np.array([0.0, c[0], p[0], p[0]]) + x_offset
            y = np.array([0.0, c[1], p[1], j * 0.1])
            text = [None, None, None, "%.4f" % j]
            x_offset = x[-1]

            max_offset = max(max_offset, max(x))
            max_offset = max(max_offset, max(y))
            min_offset = min(min_offset, min(x))
            min_offset = min(min_offset, min(y))

            traces += [pyg.Scatter(x=x, y=y, text=text, name='L%d' % i)]
        data = pyg.Data(traces)

        origin = min_offset - 0.05
        size = (max_offset - min_offset) + 0.1
        layout = pyg.Layout(xaxis=pyg.XAxis(range=[origin, size]),
                            yaxis=pyg.YAxis(range=[origin, size]))

        unique_url = py.plot({'data': data, 'layout': layout},
                             filename='TIP Pose')
        print '==== plot_trace OK : ', unique_url

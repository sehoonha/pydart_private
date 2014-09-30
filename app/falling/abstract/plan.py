import numpy as np
import math
from state_db import State, get_points
# Plotting
import plotly.plotly as py
import plotly.graph_objs as pyg


class Plan:
    def __init__(self, _x0, _path):
        # self.x0 = _x0
        # self.path = _path
        self.states = [_x0] + [x for (x, v, u) in _path]
        self.controls = [u for (x, v, u) in _path]

    @property
    def n(self):
        """Num contacts"""
        return len(self.controls)

    def state(self, index):
        if index == 0:
            x0 = self.states[index]
            x1 = self.states[index + 1]
            return State(x0.th1, 0, x1.r1, 0, 0, 0)
        else:
            x0 = self.states[index]
            x1 = self.states[index + 1]
            u0 = self.controls[index - 1]
            th1 = x0.th1 + u0.th2 - math.pi
            return State(th1, 0, x1.r1, 0, 0, 0)

    def control(self, index):
        return self.controls[index]

    def C(self, index):
        (x, u) = self.state(index), self.control(index)
        p = get_points(x, u)
        return np.array([p.x1, p.y1])

    def P(self, index):
        (x, u) = self.state(index), self.control(index)
        p = get_points(x, u)
        return np.array([p.x2, p.y2])

    def contact1(self, index):
        return int(self.states[index + 1].c1)

    def contact2(self, index):
        return int(self.controls[index].c2)

    # def C(self, collision_index=-1):
    #     x = self.x0
    #     (th1, r1) = (x.th1, x.r1)
    #     x1 = r1 * sin(th1)
    #     y1 = r1 * cos(th1)
    #     return np.array([x1, y1])

    # def P(self, index, collision_index=-1):
    #     if index == 0:
    #         return np.array([0.0, 0.0])
    #     else:
    #         theta = pi + self.x0.th1
    #         r2 = None
    #         for u in self.controls[:index]:
    #             theta -= (pi - u.th2)
    #             r2 = u.r2
    #         x2, y2 = r2 * sin(theta), r2 * cos(theta)
    #         p = self.C() + np.array([x2, y2])
    #         return p

    # def __repr__(self):
    #     pass

    def plot(self):
        """Plot the configuration with the given index of collision"""
        traces = []
        for i in range(self.n):
            c = self.C(i)
            p = self.P(i)
            x = np.array([0.0, c[0], p[0]]) + 0.1 * i
            y = np.array([0.0, c[1], p[1]])
            traces += [pyg.Scatter(x=x, y=y, name='L%d' % i)]
        data = pyg.Data(traces)
        layout = pyg.Layout(xaxis=pyg.XAxis(range=[-0.1, 0.4]),
                            yaxis=pyg.YAxis(range=[-0.1, 0.4]))

        unique_url = py.plot({'data': data, 'layout': layout},
                             filename='TIP Pose')
        print '==== plot_trace OK : ', unique_url

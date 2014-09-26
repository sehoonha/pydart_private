import numpy as np
from math import pi, sin, cos
from state_db import State, Control
# Plotting
import plotly.plotly as py
import plotly.graph_objs as pyg


class Plan:
    def __init__(self):
        self.x0 = State(0.2013, 0.3029, 0.1928, -0.0100, 0, 0.000)
        self.controls = [Control(2.4575, 0.1892, -0.0100, 1),
                         Control(2.7716, 0.2319, -0.0100, 2)]

    @property
    def n(self):
        """Num contacts"""
        return len(self.controls)

    def C(self, collision_index=-1):
        x = self.x0
        (th1, r1) = (x.th1, x.r1)
        x1 = r1 * sin(th1)
        y1 = r1 * cos(th1)
        return np.array([x1, y1])

    def P(self, index, collision_index=-1):
        if index == 0:
            return np.array([0.0, 0.0])
        else:
            theta = pi + self.x0.th1
            r2 = None
            for u in self.controls[:index]:
                theta -= (pi - u.th2)
                r2 = u.r2
            x2, y2 = r2 * sin(theta), r2 * cos(theta)
            p = self.C() + np.array([x2, y2])
            return p

    def __repr__(self):
        pass

    def plot(self):
        """Plot the configuration with the given index of collision"""
        traces = []
        c = self.C()
        for i in range(self.n + 1):
            p = self.P(i)
            x = [c[0], p[0]]
            y = [c[1], p[1]]
            traces += [pyg.Scatter(x=x, y=y, name='L%d' % i)]
        data = pyg.Data(traces)
        layout = pyg.Layout(xaxis=pyg.XAxis(range=[-0.1, 0.5]),
                            yaxis=pyg.YAxis(range=[-0.1, 0.5]))

        unique_url = py.plot({'data': data, 'layout': layout},
                             filename='TIP Pose')
        print '==== plot_trace OK : ', unique_url

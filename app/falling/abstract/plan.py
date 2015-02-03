import numpy as np
import math
from math import sin
from state_db import State, get_points, get_first_point
# Plotting
import plotly.plotly as py
import plotly.graph_objs as pyg


class Plan:
    def __init__(self, _x0, _path):
        self.x0 = _x0
        self.path = _path
        self.initialize()
        max_contact = max([u.c2 for u in self.controls])
        self.names = ["C%d" % c for c in range(max_contact + 1)]

    def initialize(self):
        self.states = [self.x0] + [entry.nx_0 for entry in self.path]
        self.controls = [entry.u for entry in self.path]

    @property
    def n(self):
        """Num contacts"""
        return len(self.controls)

    def state(self, index):
        return self.states[index + 1]

    def state_0(self, index):
        x0 = self.path[index].x
        x1 = self.path[index].nx_0
        x = State(x0.th1, x0.dth1, x0.r1, x1.dr1, x0.c1, x0.t)
        return x

    def r1(self, index):
        return self.state(index).r1

    def th1(self, index):
        return self.state(index).th1

    def r2(self, index):
        return self.control(index).r2

    def th2(self, index):
        return self.control(index).th2

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

    def DTH1(self, index):
        return self.state(index).dth1

    def J(self, index):
        impulses = [entry.v for entry in self.path]
        return impulses[index]

    def contact1(self, index):
        return int(self.states[index + 1].c1)

    def contact2(self, index):
        return int(self.controls[index].c2)

    def __str__(self):
        ret = "== Plan ==\n"
        ret += 'Impulses: '
        ret += ', '.join(["%.6f" % e.v for e in self.path])
        ret += '\n'
        for i, entry in enumerate(self.path):
            ret += '\n'
            ret += 'Contact %d' % i + '\n'
            ret += 'impulse: ' + str(entry.v) + '\n'
            ret += 'x0: ' + str(entry.x) + '\n'
            ret += 'nx_0: ' + str(entry.nx_0) + '\n'
            ret += 'u: ' + str(entry.u) + '\n'
            ret += 'nx_1: ' + str(entry.nx_1) + '\n'
        return ret

    def summary(self, prob):
        c_names = prob.contact_names()
        ret = "== Plan ==\n"
        ret += 'Contacts: '
        contacts = [c_names[self.contact1(0)]]
        contacts += [c_names[e.u.c2] for e in self.path]
        ret += ', '.join(contacts)
        ret += '\n'
        ret += 'Impulses: '
        ret += ', '.join(["%.6f" % e.v for e in self.path])
        ret += '\n'
        for i, entry in enumerate(self.path):
            ret += '\n'
            ret += 'Contact %d' % i + '\n'
            ret += 'name: ' + c_names[entry.u.c2] + '\n'
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
            j_value = j * 0.005 if j > 10.0 else j * 0.1

            con_name = self.names[self.control(i).c2]
            dth1 = self.DTH1(i)
            x = np.array([0.0, c[0], p[0], p[0]]) + x_offset
            y = np.array([0.0, c[1], p[1], j_value])
            text = [None, None, None, "j=%.4f" % j]
            x_offset = x[-1]
            name = 'L%d at %s (dth1:%.4f,j:%.4f)' % (i, con_name, dth1, j),
            max_offset = max(max_offset, max(x))
            max_offset = max(max_offset, max(y))
            min_offset = min(min_offset, min(x))
            min_offset = min(min_offset, min(y))

            traces += [pyg.Scatter(x=x, y=y, text=text,
                                   name=name,
                                   textfont=pyg.Font(size=20),
                                   mode='lines+markers+text')]
        data = pyg.Data(traces)

        origin = min_offset - 0.05
        size = (max_offset - min_offset) + 0.1
        layout = pyg.Layout(xaxis=pyg.XAxis(range=[origin, size]),
                            yaxis=pyg.YAxis(range=[origin, size]),
                            legend=pyg.Legend(x=0.5, y=1,
                                              font=pyg.Font(size=20),),
                            )

        unique_url = py.plot({'data': data, 'layout': layout},
                             filename='TIP Pose')
        print '==== plot_trace OK : ', unique_url
        # py.image.save_as({'data': data, 'layout': layout},
        #                  'plan.png', height=900, width=1000)

    def deriv(self, _state, _t):
        x = State(*_state)
        (th1, dth1, r1, dr1) = (x.th1, x.dth1, x.r1, x.dr1)

        (m, g) = (self.m, self.g)
        ddth1 = (m * r1 * (2 * dr1 * dth1 - g * sin(th1))) / (m * (r1 ** 2))
        return np.array([dth1, ddth1, dr1, 0, 0, 1.0])

    def step(self, x):
        # time = np.array([0.0, 0.005])
        # x0 = np.array(x)
        # X = odeint(self.deriv, x0, time)
        # return State(*X[-1])
        x0 = np.array(x)
        dx = self.deriv(x0, 0.0)
        x1 = x0 + 0.005 * dx
        return State(*x1)

    def com_traces(self):
        print 'generate com_trajectory'
        self.m = 149.55
        self.I = 25.7
        self.g = -9.8

        for i, x in enumerate(self.states):
            print 'Plan', i, x

        traces = []
        offset = 0.0
        cnt = 0
        for i in range(len(self.states) - 1):
            print
            print 'collision', i, 'offset=', offset
            x = self.state_0(i)
            x1 = self.states[i + 1]
            next_t = x1.t
            # Cx = [offset]
            # Cy = [0.0]
            Cx = []
            Cy = []
            text = [None]
            while x.t < next_t:
                if cnt % 2 == 0:
                    pts = get_first_point(x)
                    Cx += [pts.x1 + offset]
                    Cy += [pts.y1]
                    text += ["%.4f" % x.t]
                    print x, pts.x1, pts.y1
                cnt += 1
                x = self.step(x)
            offset = self.P(i)[0]
            print 'Checking the result'
            print 'x = ', x
            print 'x1 = ', x1
            print
            traces += [(Cx, Cy, text)]
            # traces += [pyg.Scatter(x=Cx, y=Cy, text=text,
            #                        # textfont=pyg.Font(size=20),
            #                        mode='lines+markers+text')]
        # data = pyg.Data(traces)
        # unique_url = py.plot({'data': data},
        #                      filename='COM Trajectories')
        # print '==== com_trajectory OK : ', unique_url
        return traces

    def impulse_traces(self, max_t):
        h = 0.01
        x = [0.0]
        y = [0.0]
        for i in range(len(self.states) - 1):
            x1 = self.states[i + 1]
            next_t = x1.t
            next_j = self.J(i)
            while x[-1] + 2 * h <= next_t:
                x += [x[-1] + h]
                y += [0.0]
            x += [next_t]
            y += [next_j]
            # x += [next_t]
            # y += [self.J(i)]

        # Fill until the last
        while x[-1] + h <= max_t:
            x += [x[-1] + h]
            y += [0.0]
        return [(x, y)]

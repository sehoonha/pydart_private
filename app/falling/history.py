import numpy as np
import plotly.plotly as py
from plotly.graph_objs import *
import csv

class History:
    def __init__(self, _sim):
        self.index = -1 # Use the latest
        self.sim = _sim
        self.world = self.sim.world
        self.histories = []
        self.callbacks = []

    def clear(self):
        self.histories = []
        
    def push(self):
        skel = self.world.skel
        data = {}
        data['t'] = self.world.t
        data['nframes'] = self.world.nframes
        data['contacts'] = self.world.contacts()
        data['skelcontacts'] = skel.external_contacts_and_body_id()
        data['contactedBodies'] = skel.contacted_body_names()
        data['l_hand.v'] = skel.body("l_hand").Cdot
        data['P'] = skel.P
        (data['P.x'], data['P.y']) = (skel.P[2], skel.P[1])
        data['C'] = skel.C
        (data['C.x'], data['C.y']) = (skel.C[2], skel.C[1])
        self.histories += [data]
        # Push all callback objects
        for cb in self.callbacks:
            cb.push(self)
        self.index = -1 # Use the latest

    def pop(self, index):
        self.index = index

    def get_frame(self):
        return self.histories[self.index]

    def get_frame_at(self, index):
        return self.histories[index]
        
    def plot(self):
        x = [ data['t'] for data in self.histories ]
        traces = []

        colors = ['red', 'green', 'blue', 'black']
        for i, name in enumerate(['th', 'r', 'C.x', 'C.y']):
            y = [ data[name] for data in self.histories ]
            traces += [ Scatter(x=x,y=y,name='FB_%s' % name, line=Line(color=colors[i]))]
        traces += self.sim.abstract_tip.plot(colors)
        data = Data(traces)
        # py.image.save_as({'data': data}, 'trajectories.png', height=900, width=1200)
        unique_url = py.plot(data, filename = 'Simulation history')

    def plotCOM(self):
        return self.plotTwoCsv()
        x = [ data['t'] for data in self.histories ]
        traces = []

        for name in ['C.x', 'C.y', 'P.x', 'P.y']:
            y = [ data[name] for data in self.histories ]
            traces += [ Scatter(x=x,y=y,name=name) ]
        data = Data(traces)
        unique_url = py.plot(data, filename = 'Simulation COM history')

    def vertical_impulses(self, window_size = 40):
        forces = []
        pivot_id = self.world.skel.body_index('r_foot')

        # print 'pivot = ', pivot_id
        # print self.world.skel.name_to_body
        # for idx, data in enumerate(self.histories[0:10]):
        #     print 'frame ', idx, [bid for (c, bid) in data['skelcontacts']]

        for contacts_ids in [ data['skelcontacts'] for data in self.histories ]:
            f_y = [float(-c[4]) for c, bid in contacts_ids if bid != pivot_id]
            forces += [np.sum(f_y) * self.world.dt]
        # print forces[0:10]

        operands = np.array(forces)
        for i in range(window_size - 1):
            if len(forces) == 1:
                return forces

            operands = np.delete(operands, 0)
            forces = np.delete(forces, -1)
            forces = forces + operands
        
        # n = len(self.histories)
        # window_size = 10
        # for i in range(n - window_size + 1):
        #     j = i + window_size
        #     force
        # for contacts in [ data['contacts'] for data in self.histories ]:
        #     f_y = [float(-c[4]) for c in contacts if -c[4] > 5.0]
        #     forces += [np.sum(f_y) * self.world.dt]
        return forces
        
    def max_impulse(self):
        impulses = self.vertical_impulses()
        return (0 if len(impulses) == 0 else max(impulses) )

        
    def plotImpact(self):
        x = [ data['t'] for data in self.histories ]
        # Plot vertical impact
        raw_impulses = self.vertical_impulses(1)
        impulses = self.vertical_impulses()
        max_impulses = np.maximum.accumulate(impulses)
            
        traces = []
        traces += [Scatter(x=x,y=raw_impulses,name="Raw Impulse")]
        traces += [Scatter(x=x,y=impulses,name="Impulse (Windowed)")]
        traces += [Scatter(x=x,y=max_impulses,name="Max Impulse")]

        data = Data(traces)
        layout = Layout(yaxis=YAxis(range=[0.0, 1.5]) )

        unique_url = py.plot(data, filename = 'Simulation Cumulative Impact history')
        # py.image.save_as({'data': data, 'layout' : layout}, 'plot_impact.png', height=900, width=1200)

    def writeData(self):
        with open('data.csv', 'w+') as csvfile:
            wr = csv.writer(csvfile)
            for data in self.histories:
                row = [data['t'], data['C.y'], data['P.y']]
                wr.writerow(row)

    def plotCSV(self, filename, tag):
        data = []
        with open(filename, 'r') as csvfile:
            rd = csv.reader(csvfile)
            for row in rd:
                data += [ [float(x) for x in row] ]

        names = ['t', tag + '_C.y', tag + '_P.y']
        traces = []
        for i in range(1, 3):
            x = [ row[0] for row in data ]
            y = [ row[i] for row in data ]
            traces += [Scatter(x=x,y=y,name=names[i])]
        return traces

    def plotTwoCsv(self):
        traces = []
        traces += self.plotCSV('data00.csv', 'naive')
        traces += self.plotCSV('data01.csv', 'control')
        data = Data(traces)
        py.image.save_as({'data': data}, 'two_momentums.png', height=900, width=1200)
        # unique_url = py.plot(data, filename = 'Two Simulation Momentums')
        
        

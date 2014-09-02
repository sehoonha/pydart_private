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
        data = {}
        data['t'] = self.world.t
        data['nframes'] = self.world.nframes
        data['contacts'] = self.world.contacts()
        data['contactedBodies'] = self.world.skel.contacted_body_names()
        data['l_hand.v'] = self.world.skel.body("l_hand").Cdot
        data['P'] = self.world.skel.P
        (data['P.x'], data['P.y']) = (self.world.skel.P[2], self.world.skel.P[1])
        data['C'] = self.world.skel.C
        (data['C.x'], data['C.y']) = (self.world.skel.C[2], self.world.skel.C[1])
        self.histories += [data]
        # Push all callback objects
        for cb in self.callbacks:
            cb.push(self)
        self.index = -1 # Use the latest

    def pop(self, index):
        self.index = index

    def get_frame(self):
        return self.histories[self.index]
        
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

    def plotImpact(self):
        x = [ data['t'] for data in self.histories ]
        # Plot vertical impact
        forces = []
        for contacts in [ data['contacts'] for data in self.histories ]:
            f = sum( [float(c[4]) for c in contacts] ) # Sum the y component of the force
            sum_force = f
            forces += [sum_force]
        traces = [Scatter(x=x,y=forces,name="F.y")]
        data = Data(traces)
        unique_url = py.plot(data, filename = 'Simulation Impact history')

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
        
        

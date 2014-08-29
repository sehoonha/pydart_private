import plotly.plotly as py
from plotly.graph_objs import *

class History:
    def __init__(self, _sim):
        self.index = -1 # Use the latest
        self.sim = _sim
        self.world = self.sim.world
        self.histories = []
        self.callbacks = []
        
    def push(self):
        data = {}
        data['t'] = self.world.t
        data['Contacts'] = self.world.contacts
        (data['P.x'], data['P.y']) = (self.world.skel.P[2], self.world.skel.P[1])
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
        py.image.save_as({'data': data}, 'trajectories.png', height=900, width=1200)
        # unique_url = py.plot(data, filename = 'Simulation history')

    def plotCOM(self):
        x = [ data['t'] for data in self.histories ]
        traces = []
        for name in ['C.x', 'C.y', 'P.x', 'P.y']:
            y = [ data[name] for data in self.histories ]
            traces += [ Scatter(x=x,y=y,name=name) ]
        # Plot vertical impact
        sum_force = 0.0
        forces = []
        for contacts in [ data['Contacts'] for data in self.histories ]:
            f = sum( [float(c[4]) for c in contacts] ) # Sum the y component of the force
            sum_force += f
            forces += [0.01 * sum_force]
        traces += [Scatter(x=x,y=forces,name="F.y(1/100 Scale)")]
        
        data = Data(traces)
        unique_url = py.plot(data, filename = 'Simulation COM history')
        

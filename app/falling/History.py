import plotly.plotly as py
from plotly.graph_objs import *

class History:
    def __init__(self, _world):
        self.index = -1 # Use the latest
        self.world = _world
        self.histories = []
        self.callbacks = []
        
    def push(self):
        data = {}
        data['t'] = self.world.getTime()
        data['C'] = self.world.getCOM()
        data['Contacts'] = self.world.getWorldContacts()
        self.histories += [data]
        # Push all callback objects
        for cb in self.callbacks:
            cb.push(self)
        self.index = -1 # Use the latest

    def pop(self, index):
        self.index = index

    def getFrame(self):
        return self.histories[self.index]
        
    def plotCOM(self):
        x = [ data['t'] for data in self.histories ]
        traces = []
        for name, i in [('C', 2), ('C', 1), ('Chat', 2), ('Chat', 1)]:
            y = [ data[name][i] for data in self.histories ]
            traces += [ Scatter(x=x,y=y,name='%s.%d' % (name, i)) ]
        data = Data(traces)
        py.image.save_as({'data': data}, 'plot.png')
        # unique_url = py.plot(data, filename = 'basic-line')

        

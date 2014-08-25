import plotly.plotly as py
from plotly.graph_objs import *

class History:
    def __init__(self, _world):
        self.world = _world
        self.histories = []
        self.callbacks = []
        
    def push(self):
        data = {}
        data['t'] = self.world.getTime()
        data['C'] = self.world.getCOM()
        self.histories += [data]
        # Push all callback objects
        for cb in self.callbacks:
            cb.push(self)

    def plotCOM(self):
        x = [ data['t'] for data in self.histories ]
        traces = []
        for name, i in [('C', 2), ('C', 1), ('Chat', 2), ('Chat', 1)]:
            y = [ data[name][i] for data in self.histories ]
            traces += [ Scatter(x=x,y=y,name='%s.%d' % (name, i)) ]
        data = Data(traces)
        py.image.save_as({'data': data}, 'plot.png')
        # unique_url = py.plot(data, filename = 'basic-line')

        

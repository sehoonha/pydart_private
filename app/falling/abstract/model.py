import csv

import plotly.plotly as py
from plotly.graph_objs import *

class TIP:
    def __init__(self):
        self.data = None
        pass

    def load_history(self, _csvfilename):
        self.data = []
        self.header = ['t', 'th', 'dth', 'r', 'C.x', 'C.y', 'dr', 'alpha', 'l']
        self.index = dict(zip(self.header, range(len(self.header))))
        with open(_csvfilename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.data += [[float(x) for x in row ]]
        # print self.data
        # print [self.data[i][self.index['t']] for i in range(len(self.data))]

    def column(self, name):
        return [self.data[i][self.index[name]] for i in range(len(self.data))]
        
    def commands(self):
        return [self.data[-1][self.index[name]] for name in ['r', 'l', 'alpha']]
                
    def plot(self, colors):
        x = self.column('t')
        traces = []

        for i, name in enumerate(['th', 'r', 'C.x', 'C.y']):
            y = self.column(name)
            line = Line(dash='dash', color = colors[i])
            traces += [ Scatter(x=x,y=y,name='TIP_%s' % name, line=line) ]
        return traces
            

        

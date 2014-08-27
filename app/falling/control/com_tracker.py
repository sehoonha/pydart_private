import numpy as np
import numpy.linalg as LA
import csv
from operator import itemgetter

class COMTracker:
    """
    # Usage
    self.ct = COMTracker(self, config.DATA_PATH + 'COM.csv')
    tau += self.ct.control( self.world.t, self.skel.C )
    """
    def __init__(self, _world, _csvfilename):
        self.world = _world
        self.loadCSV(_csvfilename)
        self.error = 0.0

    def loadCSV(self, _csvfilename):
        self.data = []
        with open(_csvfilename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.data += [ [float(x) for x in row] ]
            self.data += [ [99.9] + self.data[-1][1:3] ] # Put the sentinal row
            self.index = 0

    def get_chat(self):
        return np.array([0.0] + [self.data[self.index][x] for x in [2, 1]] )
        
    def control(self, t, C):
        if self.data[self.index][0] < t:
            self.index += 1
        Chat = self.get_chat()
        self.error += LA.norm(C - Chat)
        f = 0.0 * (C - Chat)
        f[0] = 0
        # print t, C, self.index, Chat, f, self.error
        return self.world.jt.control( ["l_heel", "r_heel"], f )

    def push(self, history):
        data = history.histories[-1]
        data['Chat'] = self.get_chat()



        
        

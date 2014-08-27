import math
import numpy as np
import numpy.linalg as LA
from numpy.linalg import norm

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import gltools

class TIP:
    """ Telescoptic Inverted Pendulum with A Massless End Effector """
    def __init__(self, _skel):
        self.skel = _skel

    def getAverageBodyPositions(self, bodynames, local):
        if not isinstance(local[0], list):
            local = [local] * len(bodynames)

        positions = []
        for i in range(len(bodynames)):
            # T = self.skel.getBodyNodeTransformation(bodynames[i])
            T = self.skel.body(bodynames[i]).transformation()
            positions += [ T.dot( np.append(local[i], [1.0]) ) ]
            
        avg = (sum(positions) / len(positions))[0:3]

        return avg

    def getP0(self):
        """Returns the origin """
        return self.getAverageBodyPositions(["l_foot", "r_foot"], [-0.05, 0.025, 0.0])

    def getP1(self):
        """Returns the COM """
        return self.skel.C

    def getP2(self):
        """Returns the End Effector"""
        # (y, z) = (0.0, 0.0)
        # return self.getAverageBodyPositions(["l_shin", "r_shin"], [[0, -y, -z], [0, y, z]] )
        (y, z) = (0.11, -0.01)
        return self.getAverageBodyPositions(["l_hand", "r_hand"], [[0, -y, -z], [0, y, z]] )

    def getD01(self):
        """Returns the distance between origin and COM """
        return norm( self.getP0() - self.getP1() )

    def getD12(self):
        """Returns the distance between COM and End Effector """
        return norm( self.getP1() - self.getP2() )

    def getTheta(self):
        C = self.getP1()
        return math.atan2(C[2], C[1])

    def getAngle(self):
        """Returns the angle between P0-P1-P2"""
        a = self.getP1() - self.getP0()
        b = self.getP1() - self.getP2()
        d = np.cross(a, b)[0]
        return math.pi - np.sign(-d) * math.acos( a.dot(b) / (norm(a) * norm(b)) )

    def getState(self):
        return np.array([self.getD01(), self.getD12(), self.getAngle()])

    def push(self, history):
        data = history.histories[-1]
        data['C.x'] = self.getP1()[2]
        data['C.y'] = self.getP1()[1]
        data['r'] = self.getD01()
        data['th'] = self.getTheta()

    def render(self):
        glLineWidth(3.0)
        gltools.glMove( [0.0, 0.0, 0.0] )
        glBegin(GL_LINE_STRIP)
        for p in [self.getP0(), self.getP1(), self.getP2()]:
            glVertex(p)
        glEnd()
        glColor(0.0, 0.5, 0.0, 1.0)
        gltools.glMove( self.getP0() )
        glutSolidSphere(0.03, 4, 2)
        glColor(0.0, 0.5, 0.5, 1.0)
        gltools.glMove( self.getP1() )
        glutSolidSphere(0.03, 4, 2)
        gltools.glMove( self.getP2() )
        glColor(0.0, 0.0, 0.5, 1.0)
        glutSolidSphere(0.03, 4, 2)
        glLineWidth(1.0)

        

        

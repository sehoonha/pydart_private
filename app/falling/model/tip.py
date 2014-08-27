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

    def average_body_positions(self, bodynames, local):
        if not isinstance(local[0], list):
            local = [local] * len(bodynames)

        positions = []
        for i in range(len(bodynames)):
            # T = self.skel.getBodyNodeTransformation(bodynames[i])
            T = self.skel.body(bodynames[i]).transformation()
            positions += [ T.dot( np.append(local[i], [1.0]) ) ]

        avg = (sum(positions) / len(positions))[0:3]
        return avg


    def p0(self):
        """Returns the origin """
        return self.average_body_positions(["l_foot", "r_foot"], [-0.05, 0.025, 0.0])

    def p1(self):
        """Returns the COM """
        return self.skel.C

    def p2(self):
        """Returns the End Effector"""
        # (y, z) = (0.0, 0.0)
        # return self.average_body_positions(["l_shin", "r_shin"], [[0, -y, -z], [0, y, z]] )
        (y, z) = (0.11, -0.01)
        return self.average_body_positions(["l_hand", "r_hand"], [[0, -y, -z], [0, y, z]] )

    def d01(self):
        """Returns the distance between origin and COM """
        return norm( self.p0() - self.p1() )

    def d12(self):
        """Returns the distance between COM and End Effector """
        return norm( self.p1() - self.p2() )

    def theta(self):
        C = self.p1()
        return math.atan2(C[2], C[1])

    def angle(self):
        """Returns the angle between P0-P1-P2"""
        a = self.p1() - self.p0()
        b = self.p1() - self.p2()
        d = np.cross(a, b)[0]
        return math.pi - np.sign(-d) * math.acos( a.dot(b) / (norm(a) * norm(b)) )

    def get_state(self):
        return np.array([self.d01(), self.d12(), self.angle()])

    def push(self, history):
        data = history.histories[-1]
        data['C.x'] = self.p1()[2]
        data['C.y'] = self.p1()[1]
        data['r'] = self.d01()
        data['th'] = self.theta()

    def render(self):
        glLineWidth(3.0)
        glColor(0.7, 0.0, 0.0, 1.0)
        gltools.glMove( [0.0, 0.0, 0.0] )
        glBegin(GL_LINE_STRIP)
        for p in [self.p0(), self.p1(), self.p2()]:
            glVertex(p)
        glEnd()
        glColor(0.0, 0.5, 0.0, 1.0)
        gltools.glMove( self.p0() )
        glutSolidSphere(0.03, 4, 2)
        glColor(0.0, 0.5, 0.5, 1.0)
        gltools.glMove( self.p1() )
        glutSolidSphere(0.03, 4, 2)
        gltools.glMove( self.p2() )
        glColor(0.0, 0.0, 0.5, 1.0)
        glutSolidSphere(0.03, 4, 2)
        glLineWidth(1.0)

    def __repr__(self):
        return str(["%.3f" % x for x in self.get_state()])

        

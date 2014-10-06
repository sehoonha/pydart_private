import math
import numpy as np
from numpy.linalg import norm

from OpenGL.GL import glLineWidth, glColor, glBegin, glEnd, glVertex, \
    GL_LINE_STRIP
from OpenGL.GLUT import glutSolidSphere
import gltools


class TIP(object):
    """
    Telescoptic Inverted Pendulum with A Massless End Effector
    c1 is the pivot contact
    c2 is the next contact
    """
    def __init__(self, _id, _c1, _c2):
        self.id = _id
        self.c1 = _c1
        self.c2 = _c2
        self.skel = _c1.skel

    def pivot_nodes(self):
        return self.c1.bodynames

    @property
    def p1(self):
        """Returns the pivot """
        return self.c1.p

    @property
    def p2(self):
        """Returns the stopper """
        return self.c2.p

    @property
    def C(self):
        """Returns the COM """
        return self.skel.C

    @property
    def r1(self):
        """Returns the distance between origin and COM """
        return norm(self.p1 - self.C)

    @property
    def r2(self):
        """Returns the distance between COM and End Effector """
        return norm(self.p2 - self.C)

    @property
    def th1(self):
        d = self.C - self.p1
        return math.atan2(d[2], d[1])

    def projected_Cdot(self):
        v = self.skel.Cdot
        v[0] = 0
        u = self.C - self.p1
        u[0] = 0
        u /= norm(u)
        # print v, u, np.dot(v, u), v - np.dot(v, u) * u
        return v - np.dot(v, u) * u

    @property
    def dth1(self):
        v1 = norm(self.projected_Cdot())
        # |v| = r dtheta
        return v1 / self.r1

    @property
    def th2(self):
        """Returns the angle between p1-C-p2"""
        a = self.C - self.p1
        b = self.C - self.p2
        d = np.cross(a, b)[0]
        pi = math.pi
        return pi - np.sign(-d) * math.acos(a.dot(b) / (norm(a) * norm(b)))

    def pose(self):
        return np.array([self.r1, self.r2, self.th2])

    def push(self, history):
        data = history.histories[-1]
        data['tip'] = str(self)

    def render(self):
        glLineWidth(3.0)
        glColor(0.7, 0.0, 0.0, 1.0)
        gltools.glMove([0.0, 0.0, 0.0])
        glBegin(GL_LINE_STRIP)
        for p in [self.p1, self.C, self.p2]:
            glVertex(p)
        glEnd()
        glColor(0.545, 0.000, 0.000, 1.0)
        gltools.glMove(self.p1)
        glutSolidSphere(0.03, 4, 2)
        glColor(1.000, 0.843, 0.000, 1.0)
        gltools.glMove(self.C)
        glutSolidSphere(0.03, 4, 2)
        glColor(0.294, 0.000, 0.510, 1.0)
        gltools.glMove(self.p2)
        glutSolidSphere(0.03, 4, 2)
        glLineWidth(1.0)

    def __str__(self):
        # values = [self.th1, self.dth1, self.r1,
        #           self.th2, self.r2]
        return "[%s, %s: (%.3f %.1f) %.3f (%.3f) %.3f]" % \
            (self.c1.name, self.c2.name, self.th1, self.dth1, self.r1,
             self.th2, self.r2)

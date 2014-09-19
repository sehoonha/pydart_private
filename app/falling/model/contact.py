import numpy as np
from OpenGL.GL import glPushMatrix, glPopMatrix, glColor, glTranslate
from OpenGL.GLUT import glutSolidSphere


class Contact(object):
    def __init__(self, _skel, _name, _bodynames, _positions):
        self.skel = _skel
        self.name = _name
        self.bodyname = _bodynames
        self.bodies = [self.skel.body(b) for b in _bodynames]
        self.local_positions = _positions

    @property
    def world_pos(self):
        positions = []
        for i in range(len(self.bodies)):
            T = self.bodies[i].T
            l = np.append(self.local_positions[i], [1.0])
            w = T.dot(l)
            positions += [w[0:3]]
        avg = (sum(positions) / len(positions))
        return avg

    def render(self):
        glColor(0, 1, 1)
        glPushMatrix()
        glTranslate(*self.world_pos)
        glutSolidSphere(0.02, 4, 2)
        glPopMatrix()

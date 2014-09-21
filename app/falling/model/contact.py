import numpy as np
from OpenGL.GL import glPushMatrix, glPopMatrix, glColor, glTranslate
from OpenGL.GLUT import glutSolidSphere


class Contact(object):
    def __init__(self, _skel, _id, _name, _bodynames, _positions):
        self.skel = _skel
        self.id = _id
        self.name = _name
        self.bodynames = _bodynames
        self.bodies = [self.skel.body(b) for b in _bodynames]
        self.local_positions = _positions

    def world_pos(self):
        positions = []
        for i in range(len(self.bodies)):
            T = self.bodies[i].T
            l = np.append(self.local_positions[i], [1.0])
            w = T.dot(l)
            positions += [w[0:3]]
        avg = (sum(positions) / len(positions))
        return avg

    @property
    def p(self):
        return self.world_pos()

    def render(self):
        glColor(0, 1, 1)
        glPushMatrix()
        glTranslate(*self.p)
        glutSolidSphere(0.02, 4, 2)
        glPopMatrix()

    def __repr__(self):
        ret = "Contact(skel, %r, %r, %r, %r)" % (self.id, self.name,
                                                 self.bodynames,
                                                 self.local_positions)
        return ret

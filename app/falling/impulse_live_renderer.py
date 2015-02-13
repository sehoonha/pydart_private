from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *


class ImpulseLiveRenderer:
    def __init__(self, sim):
        self.sim = sim
        self.history = self.sim.history

    def render(self):
        glPushMatrix()
        glColor3d(0.0, 0.0, 1.0)
        glTranslated(20, 480, 0)

        t_data, j_data = self.history.impulse_traces(hz=20)[0]

        max_impulse = 1.5 if self.sim.is_bioloid() else 500.0
        max_len = 150
        y_max_height = 200.0
        x_max_width = 350.0

        x = 0.0
        y = 0.0
        y_scale = y_max_height / max_impulse
        x_gap = 0.1
        x_len = (x_max_width / float(max_len)) - x_gap

        for t, j in zip(t_data, j_data):
            y2 = y - (y_scale * j + 1.0)
            glBegin(GL_QUADS)
            glVertex2f(x, y)
            glVertex2f(x + x_len, y)
            glVertex2f(x + x_len, y2)
            glVertex2f(x, y2)
            glEnd()
            x += (x_len + x_gap)
        glPopMatrix()

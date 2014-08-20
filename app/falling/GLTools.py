from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import math
import numpy as np

def renderAxis(LEN = 10.0):
    glDisable(GL_LIGHTING)
    glLineWidth(3.0)
    for i in range(3):
        v = np.zeros(3)
        v[i] = 1.0

        glColor(v)
        glBegin(GL_LINES)
        glVertex(v * LEN)
        glVertex(v * -LEN)
        glEnd()
    glLineWidth(1.0)

def renderChessBoard(n, sz):
    glDisable(GL_LIGHTING)
    cnt = 0
    step = sz / float(n)
    for x in np.linspace(-0.5 * sz, 0.5 * sz, n, endpoint = False):
        for z in np.linspace(-0.5 * sz, 0.5 * sz, n, endpoint = False):
            x2 = x + step
            z2 = z + step
            if cnt % 2 == 0:
                glColor(0.4, 0.4, 0.4)
            else:
                glColor(0.7, 0.7, 0.7)
                
            glBegin(GL_POLYGON)
            glVertex([x, 0, z])
            glVertex([x, 0, z2])
            glVertex([x2, 0, z2])
            glVertex([x2, 0, z])
            glEnd()
            cnt += 1
        cnt += 1

    glEnable(GL_LIGHTING)
    

def renderArrow(p, q):
    glPushMatrix()
    glBegin(GL_LINES)
    glVertex(p)
    glVertex(q)
    glEnd()
    glPopMatrix()

    
    

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import math
import numpy as np
from numpy.linalg import norm

def glMove(pos):
    glPopMatrix()
    glPushMatrix()
    glTranslate(pos[0], pos[1], pos[2])

def render_axis(LEN = 10.0):
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

def render_chessboard(n, sz):
    glDisable(GL_LIGHTING)
    cnt = 0
    step = sz / float(n)
    for x in np.linspace(-0.5 * sz, 0.5 * sz, n, endpoint = False):
        for z in np.linspace(-0.5 * sz, 0.5 * sz, n, endpoint = False):
            x2 = x + step
            z2 = z + step
            if cnt % 2 == 0:
                glColor(0.2, 0.2, 0.2)
            else:
                glColor(0.8, 0.8, 0.8)

            glBegin(GL_POLYGON)
            glVertex([x, 0, z])
            glVertex([x, 0, z2])
            glVertex([x2, 0, z2])
            glVertex([x2, 0, z])
            glEnd()
            cnt += 1
        cnt += 1

    glEnable(GL_LIGHTING)


def render_floor(n, sz):
    glDisable(GL_LIGHTING)
    cnt = 0
    step = sz / float(n)
    glColor(0.86, 0.86, 0.76)
    for x in np.linspace(-0.5 * sz, 0.5 * sz, n, endpoint = False):
        for z in np.linspace(-0.5 * sz, 0.5 * sz, n, endpoint = False):
            x2 = x + step
            z2 = z + step

            glBegin(GL_POLYGON)
            glVertex([x, 0, z])
            glVertex([x, 0, z2])
            glVertex([x2, 0, z2])
            glVertex([x2, 0, z])
            glEnd()
            cnt += 1
        cnt += 1

    glEnable(GL_LIGHTING)


def render_arrow(p, q):
    glDisable(GL_LIGHTING)
    glLineWidth(3)
    glPushMatrix()
    glBegin(GL_LINES)
    glVertex(p)
    glVertex(q)
    glEnd()
    glTranslate(p[0], p[1], p[2])
    glutSolidSphere(0.005, 10, 10)
    glPopMatrix()
    glLineWidth(1)
    glEnable(GL_LIGHTING)


def R_axis_angle(axis, angle):
    """Generate the rotation matrix from the axis-angle notation.
    Conversion equations
    ====================
    From Wikipedia (http://en.wikipedia.org/wiki/Rotation_matrix)::
        c = cos(angle); s = sin(angle); C = 1-c
        xs = x*s;   ys = y*s;   zs = z*s
        xC = x*C;   yC = y*C;   zC = z*C
        xyC = x*yC; yzC = y*zC; zxC = z*xC
        [ x*xC+c   xyC-zs   zxC+ys ]
        [ xyC+zs   y*yC+c   yzC-xs ]
        [ zxC-ys   yzC+xs   z*zC+c ]
    @param matrix:  The 3x3 rotation matrix to update.
    @type matrix:   3x3 numpy array
    @param axis:    The 3D rotation axis.
    @type axis:     numpy array, len 3
    @param angle:   The rotation angle.
    @type angle:    float
    """

    # Trig factors.
    ca = math.cos(angle)
    sa = math.sin(angle)
    C = 1 - ca

    # Depack the axis.
    x, y, z = axis

    # Multiplications (to remove duplicate calculations).
    xs = x * sa
    ys = y * sa
    zs = z * sa
    xC = x * C
    yC = y * C
    zC = z * C
    xyC = x * yC
    yzC = y * zC
    zxC = z * xC

    # Update the rotation matrix.
    matrix = np.zeros((3, 3))
    matrix[0, 0] = x * xC + ca
    matrix[0, 1] = xyC - zs
    matrix[0, 2] = zxC + ys
    matrix[1, 0] = xyC + zs
    matrix[1, 1] = y * yC + ca
    matrix[1, 2] = yzC - xs
    matrix[2, 0] = zxC - ys
    matrix[2, 1] = yzC + xs
    matrix[2, 2] = z * zC + ca
    return matrix


def render_arrow2(p, q, r_base=0.01, head_width=0.015, head_len=0.01):
    # glDisable(GL_LIGHTING)
    m_quadric = gluNewQuadric()
    gluQuadricNormals(m_quadric, GLU_SMOOTH)
    p = np.array(p)
    q = np.array(q)
    u = (q - p)
    u /= norm(u)
    P = q - head_len * u
    z = np.array([0, 0.0, 1.0])
    z /= norm(z)
    if norm(z - u) < 1e-8:
        axis = np.array([0, 0, 1])
        angle = 0.0
    else:
        axis = np.cross(z, u)
        axis /= norm(axis)
        angle = math.acos(u.dot(z))
    M = R_axis_angle(axis, angle)
    m = [M[0, 0], M[1, 0], M[2, 0], 0.0, M[0, 1], M[1, 1], M[2, 1], 0.0,
         M[0, 2], M[1, 2], M[2, 2], 0.0, P[0], P[1], P[2], 1.0]
    m2 = [M[0, 0], M[1, 0], M[2, 0], 0.0, M[0, 1], M[1, 1], M[2, 1], 0.0,
          M[0, 2], M[1, 2], M[2, 2], 0.0, p[0], p[1], p[2], 1.0]

    glPushMatrix()
    glMultMatrixd(m2)
    arrow_len = norm(q - p) - head_len
    glColor(0.9, 0.2, 0.2)
    gluCylinder(m_quadric, r_base, r_base, arrow_len, 10, 10)
    glPopMatrix()

    glColor(1.0, 0.0, 0.0)
    glPushMatrix()
    glMultMatrixd(m)
    glutSolidCone(head_width, head_len, 10, 3)
    glPopMatrix()
    gluDeleteQuadric(m_quadric)
    # glEnable(GL_LIGHTING)

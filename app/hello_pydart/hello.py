#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import math
import numpy as np

BUILD_PATH = '../build/'
DATA_PATH = '../data/'
sys.path.append(BUILD_PATH)
import pydart

# Some api in the chain is translating the keystrokes to this octal string
# so instead of saying: ESCAPE = 27, we use the following.
global window
ESCAPE = '\033'
window = 0
world = None
pd = None

class PDController:
    def __init__(self, _skel, _kp, _kd):
        self.skel = _skel
        self.ndofs = self.skel.ndofs
        self.kp = np.array( [_kp] * self.ndofs )
        self.kd = np.array( [_kd] * self.ndofs )
        self.target = None

    def control(self):
        q = self.skel.q
        qdot = self.skel.qdot
        
        tau = np.zeros( self.ndofs )
        for i in range(6, self.ndofs):
            tau[i] = -self.kp[i] * (q[i] - self.target[i]) - self.kd[i] * qdot[i]
            tau[i] = min(0.5, max(-0.5, tau[i]))
        return tau


def initGL(w, h):
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT,  GL_NICEST);

    glEnable(GL_DITHER);
    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    glClearColor(0.0, 0.0, 0.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT)

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [0.4, 0.4, 0.4, 1.0])
    glLightfv(GL_LIGHT0, GL_POSITION, [0.0, 20.0, -20.0, 0.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  [0.09, 0.09, 0.09, 1.0])
    glLightfv(GL_LIGHT1, GL_POSITION, [10.0, 10.0, 10.0, 0.0])
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  [0.09, 0.09, 0.09, 1.0])
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHTING);

    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

def resizeGL(w, h):
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(w)/float(h), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    
def drawGL():
    # Clear The Screen And The Depth Buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()					# Reset The View 
    glTranslatef(0.0, -0.1, -1.2)

    global world
    world.render()
    #  since this is double buffered, swap the buffers to display what just got drawn. 
    glutSwapBuffers()

# The function called whenever a key is pressed. Note the use of Python tuples to pass in: (key, x, y)  
def keyPressed(*args):
    if args[0] == ESCAPE:
        glutDestroyWindow(window)
        sys.exit()

def idle():
    global world
    global pd
    pd.target[7] = math.sin(world.t)
    pd.target[9] = -math.sin(world.t)
    world.skels[-1].forces = pd.control()
    world.step()

def renderTimer(timer):
    glutPostRedisplay()
    glutTimerFunc(20, renderTimer, 1)
    

################################################################################
# Begin main function

# Initialize PyDart
pydart.init()
world = pydart.create_world(1.0 / 2000.0)
world.add_skeleton(DATA_PATH + "sdf/ground.urdf", control = False)
world.add_skeleton(DATA_PATH + "urdf/BioloidGP/BioloidGP.URDF")
world.skels[-1].set_joint_damping(0.15)

# Set the skeleton pose. 
q = np.zeros(world.skels[-1].ndofs)
q[0] = -0.45 * math.pi
# q[4] = 0.221
q[4] = 0.25
q[5] = q[4]
world.skels[-1].q = q

# Create a pd controller
pd = PDController(world.skels[-1], 20.0, 1.0)
pd.target = world.skels[-1].q

# Init glut
global window
glutInit(())
glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
glutInitWindowSize(800, 600)
glutInitWindowPosition(0, 0)
window = glutCreateWindow("Hello, PyDart!")

# Init functions
# glutFullScreen()
glutDisplayFunc (drawGL)
glutIdleFunc(idle)
glutReshapeFunc (resizeGL)
glutKeyboardFunc (keyPressed)
glutTimerFunc(25, renderTimer, 1)
initGL(800, 600)

# Run
glutMainLoop()

# Print message to console, and kick off the main to get it rolling.
print "Hit ESC key to quit."
main()
    	

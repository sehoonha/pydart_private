import os, sys
import test_config
sys.path.append(test_config.BUILD_PATH)
import pydart_api
import numpy as np
import math

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt4 import QtGui
from PyQt4 import QtCore
from PyQt4.QtOpenGL import *
import trackball


print "Hello, Pydart_Api"
pydart_api.init()
pydart_api.createWorld(1.0 / 1000.0)
pydart_api.addSkeleton(test_config.DATA_PATH + "sdf/ground.urdf")

rid = pydart_api.addSkeleton(test_config.DATA_PATH + "urdf/BioloidGP/BioloidGP.URDF")
nDofs = pydart_api.getSkeletonNumDofs(rid)

print 'Robot ID = ', rid
print "# dofs = ", nDofs
q = pydart_api.getSkeletonPositions(rid, nDofs)
q[0] = -0.41 * math.pi
# q[4] = -0.043
q[4] = 0.6
pydart_api.setSkeletonPositions(rid, q)
print "position = ", pydart_api.getSkeletonPositions(rid, nDofs)

class WfWidget(QGLWidget):
    def __init__(self, parent = None):
        super(WfWidget, self).__init__(parent)
        self.width = 1280
        self.height = 720
        
        self.frameCount = 0
        self.tb = trackball.Trackball()
        self.lastPos = None

        self.idleTimer = QtCore.QTimer()
        self.idleTimer.timeout.connect(self.idleTimerEvent)
        self.idleTimer.start(0)

        
    def sizeHint(self):
        return QtCore.QSize(self.width, self.height)

    def paintGL(self):
        glEnable(GL_DEPTH_TEST);
        glClearColor( 0.95, 0.95, 0.95, 0.0 );
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        
        glLoadIdentity();
        glTranslate(0.0, 0.0, -1.0) # Camera
        glMultMatrixf(self.tb.matrix)

        pydart_api.render()

    def resizeGL(self, w, h):
        (self.width, self.height) = (w, h)
        glViewport(0, 0, w, h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        gluPerspective(45.0, float(w)/float(h), 0.01, 100.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

    def initializeGL(self):
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

        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [0.8, 0.8, 0.8, 1.0])
        glLightfv(GL_LIGHT0, GL_POSITION, [0.0, 50.0, 0.0, 0.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  [0.01, 0.01, 0.01, 1.0])
        glLightfv(GL_LIGHT1, GL_POSITION, [10.0, 10.0, 0.0, 0.0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE,  [0.4, 0.4, 0.4, 1.0])
        # glEnable(GL_LIGHT0);
        glEnable(GL_LIGHT1);
        glEnable(GL_LIGHTING);

        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glEnable(GL_COLOR_MATERIAL);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
    def idleTimerEvent(self):
        self.frameCount += 1
        rid = 1
        nDofs = pydart_api.getSkeletonNumDofs(rid)

        tau = np.zeros(nDofs)
        tau[10] = 1.0

        pydart_api.setSkeletonForces(rid, tau)
        pydart_api.stepWorld()
        # print "frame ", self.frameCount, "\n", pydart_api.getSkeletonPositions(rid, nDofs)

        self.updateGL()
            
    def mouseMoveEvent(self, event):
        if self.lastPos is None:
            self.lastPos = event.pos()
            return;
            
        (w, h) = (self.width, self.height)
        x = event.x()
        y = event.y()
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        self.tb.drag_to(x,y, -dx, -dy)
        self.lastPos = event.pos()

app = QtGui.QApplication(["Winfred's PyQt OpenGL"])
widget = WfWidget()
widget.show()
app.exec_()

pydart_api.destroy()

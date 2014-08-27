print 'Hello Pydart'

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt4 import QtGui
from PyQt4 import QtCore
from PyQt4.QtOpenGL import *

import trackball
from simulation import Simulation

# avconv -r 100 -i ./frame.%04d.png output.mp4

class GLWidget(QGLWidget):
    def __init__(self, parent = None):
        super(GLWidget, self).__init__(parent)
        self.width = 1280
        self.height = 720
        
        self.tb = trackball.Trackball()
        self.lastPos = None

        self.sim = None
        self.captureIndex = 0
        
    def sizeHint(self):
        return QtCore.QSize(self.width, self.height)

    def paintGL(self):
        glEnable(GL_DEPTH_TEST);
        glClearColor( 0.95, 0.95, 0.95, 0.0 );
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        
        glLoadIdentity();
        glTranslate(0.0, 0.0, -1.2) # Camera
        glMultMatrixf(self.tb.matrix)

        self.sim.render()

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
        
    def mousePressEvent(self, event):
        self.lastPos = event.pos()
        
    def mouseReleaseEvent(self, event):
        self.lastPos = None
        
    def mouseMoveEvent(self, event):
        (w, h) = (self.width, self.height)
        x = event.x()
        y = event.y()
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        self.tb.drag_to(x,y, dx, -dy)
        self.lastPos = event.pos()
        self.updateGL()

    def capture(self):
        img = self.grabFrameBuffer()
        filename = 'captures/frame.%04d.png' % self.captureIndex
        img.save(filename)
        print 'Capture to ', filename
        self.captureIndex += 1

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()

        # Check and create captures directory
        if not os.path.isdir('captures'):
            os.makedirs('captures')

        # Create a simulation
        self.sim = Simulation()

        self.initUI()

        self.idleTimer = QtCore.QTimer()
        self.idleTimer.timeout.connect(self.idleTimerEvent)
        self.idleTimer.start(0)

        self.renderTimer = QtCore.QTimer()
        self.renderTimer.timeout.connect(self.renderTimerEvent)
        self.renderTimer.start(25)


        
    def initUI(self):               
        # Create actions
        self.playAction = QtGui.QAction('Play', self)
        self.playAction.setCheckable(True)
        self.playAction.setShortcut('Space')

        self.captureAction = QtGui.QAction('Capture', self)
        self.captureAction.setCheckable(True)

        self.plotAction = QtGui.QAction('Plot', self)
        self.plotAction.triggered.connect(self.plotEvent)

        # Create a toolbar
        self.toolbar = self.addToolBar('Control')
        self.toolbar.addAction(self.playAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.captureAction)
        self.toolbar.addAction(self.plotAction)

        self.rangeSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.rangeSlider.valueChanged[int].connect(self.rangeSliderEvent)
        self.toolbar.addWidget(self.rangeSlider)
        
        self.setGeometry(0, 0, 1280, 720)
        # self.setWindowTitle('Toolbar')

        self.glwidget = GLWidget(self)
        self.glwidget.setGeometry(0, 30, 1280, 720)
        self.glwidget.sim = self.sim
        
    def idleTimerEvent(self):
        if not self.playAction.isChecked():
            return
        result = self.sim.step()

        if self.captureAction.isChecked() and self.sim.world.nframes % 20 == 1:
            self.glwidget.capture()

        if result:
            self.playAction.setChecked(False)

    def renderTimerEvent(self):
        self.glwidget.updateGL()
        self.statusBar().showMessage( self.sim.status_string() )
        self.rangeSlider.setRange(0, self.sim.world.nframes - 1)

    def rangeSliderEvent(self, value):
        self.sim.set_world_frame( value )

    def plotEvent(self):
        self.sim.history.plot()
        
glutInit(sys.argv)
app = QtGui.QApplication(["Falling controller with Pydart"])
# widget = WfWidget()
# widget.show()
w = MyWindow()
w.show()
app.exec_()


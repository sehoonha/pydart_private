print 'Hello Pydart'

import sys
import signal
def signal_handler(signal, frame):
    print 'You pressed Ctrl+C! Bye.'
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt4 import QtGui
from PyQt4 import QtCore
from PyQt4.QtOpenGL import *
from glwidget import GLWidget

import trackball
from simulation import Simulation

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()

        # Check and create captures directory
        if not os.path.isdir('captures'):
            os.makedirs('captures')

        # Create a simulation
        self.sim = Simulation()

        self.initUI()
        self.initToolbar()
        self.initMenu()
        
        self.idleTimer = QtCore.QTimer()
        self.idleTimer.timeout.connect(self.idleTimerEvent)
        self.idleTimer.start(0)

        self.renderTimer = QtCore.QTimer()
        self.renderTimer.timeout.connect(self.renderTimerEvent)
        self.renderTimer.start(25)

    def initUI(self):               
        self.setGeometry(0, 0, 1280, 720)
        # self.setWindowTitle('Toolbar')

        self.glwidget = GLWidget(self)
        self.glwidget.setGeometry(0, 30, 1280, 720)
        self.glwidget.sim = self.sim
        

    def initToolbar(self):
        # Create actions
        self.planAction = QtGui.QAction('Plan', self)
        self.planAction.triggered.connect(self.planEvent)

        self.playAction = QtGui.QAction('Play', self)
        self.playAction.setCheckable(True)
        self.playAction.setShortcut('Space')

        self.animAction = QtGui.QAction('Anim', self)
        self.animAction.setCheckable(True)

        self.captureAction = QtGui.QAction('Capture', self)
        self.captureAction.setCheckable(True)

        self.movieAction = QtGui.QAction('Movie', self)
        self.movieAction.triggered.connect(self.movieEvent)

        self.plotAction = QtGui.QAction('Plot', self)
        self.plotAction.triggered.connect(self.plotEvent)

        # Create a toolbar
        self.toolbar = self.addToolBar('Control')
        self.toolbar.addAction(self.planAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.playAction)
        self.toolbar.addAction(self.animAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.captureAction)
        self.toolbar.addAction(self.movieAction)
        self.toolbar.addAction(self.plotAction)

        self.rangeSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.rangeSlider.valueChanged[int].connect(self.rangeSliderEvent)
        self.toolbar.addWidget(self.rangeSlider)
        
    def initMenu(self):
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        #### Plot menu
        plotMenu = menubar.addMenu('&Plot')        
        self.plotCOMAction = QtGui.QAction('&COM', self)
        self.plotCOMAction.triggered.connect(self.plotCOMEvent)
        plotMenu.addAction(self.plotCOMAction)

    def idleTimerEvent(self):
        doCapture = False
        # Do animation
        if self.animAction.isChecked():
            v = self.rangeSlider.value() + 1
            if v <= self.rangeSlider.maximum():
                self.rangeSlider.setValue(v)
            else:
                self.animAction.setChecked(False)
            doCapture = (v % 4 == 1)
        # Do play
        elif self.playAction.isChecked():
            result = self.sim.step()
            if result:
                self.playAction.setChecked(False)
            doCapture = (self.sim.world.nframes % 4 == 1)

        if self.captureAction.isChecked() and doCapture:
            self.glwidget.capture()


    def renderTimerEvent(self):
        self.glwidget.updateGL()
        self.statusBar().showMessage( self.sim.status_string() )
        self.rangeSlider.setRange(0, self.sim.world.nframes - 1)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            print 'Escape key pressed! Bye.'
            self.close()

    def rangeSliderEvent(self, value):
        self.sim.set_world_frame( value )

    def movieEvent(self):
        os.system('avconv -r 100 -i ./captures/frame.%04d.png output.mp4')
        os.system('rm ./captures/frame.*.png')

    def plotEvent(self):
        self.sim.history.plot()

    def planEvent(self):
        self.sim.plan()

    def plotCOMEvent(self):
        self.sim.history.plotCOM()

glutInit(sys.argv)
app = QtGui.QApplication(["Falling controller with Pydart"])
# widget = WfWidget()
# widget.show()
w = MyWindow()
w.show()
app.exec_()


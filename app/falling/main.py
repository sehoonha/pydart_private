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
        self.initActions()
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
        zoom = -1.2 if self.sim.is_bioloid() else -4.2
        self.glwidget.zoom = zoom

    def initActions(self):
        # Create actions
        self.planAction = QtGui.QAction('Plan', self)
        self.planAction.triggered.connect(self.planEvent)

        self.nextAction = QtGui.QAction('Next', self)
        self.nextAction.triggered.connect(self.nextEvent)

        self.resetAction = QtGui.QAction('Reset', self)
        self.resetAction.triggered.connect(self.resetEvent)

        self.playAction = QtGui.QAction('Play', self)
        self.playAction.setCheckable(True)
        self.playAction.setShortcut('Space')

        self.animAction = QtGui.QAction('Anim', self)
        self.animAction.setCheckable(True)

        self.captureAction = QtGui.QAction('Capture', self)
        self.captureAction.setCheckable(True)

        self.printAction = QtGui.QAction('Print', self)
        self.printAction.triggered.connect(self.printEvent)

        self.movieAction = QtGui.QAction('Movie', self)
        self.movieAction.triggered.connect(self.movieEvent)

        self.screenshotAction = QtGui.QAction('Screenshot', self)
        self.screenshotAction.triggered.connect(self.screenshotEvent)

        self.plotAction = QtGui.QAction('Plot', self)
        self.plotAction.triggered.connect(self.plotEvent)

        self.plotCOMAction = QtGui.QAction('COM', self)
        self.plotCOMAction.triggered.connect(self.plotCOMEvent)

        self.plotImpactAction = QtGui.QAction('Impact', self)
        self.plotImpactAction.triggered.connect(self.plotImpactEvent)

        self.loadAction = QtGui.QAction('&Load Plan', self)
        self.loadAction.triggered.connect(self.loadEvent)

        self.saveAction = QtGui.QAction('&Save Plan', self)
        self.saveAction.triggered.connect(self.saveEvent)

        self.loadmAction = QtGui.QAction('&Load Motion', self)
        self.loadmAction.triggered.connect(self.loadmEvent)

        self.savemAction = QtGui.QAction('&Save Motion', self)
        self.savemAction.triggered.connect(self.savemEvent)

    def initToolbar(self):
        # Create a toolbar
        self.toolbar = self.addToolBar('Control')
        self.toolbar.addAction(self.planAction)
        self.toolbar.addAction(self.nextAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.resetAction)
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
        fileMenu.addAction(self.loadAction)
        fileMenu.addAction(self.saveAction)
        fileMenu.addSeparator()
        fileMenu.addAction(self.loadmAction)
        fileMenu.addAction(self.savemAction)

        # Recording menu
        recordingMenu = menubar.addMenu('&Recording')
        recordingMenu.addAction(self.screenshotAction)
        recordingMenu.addAction(self.printAction)
        recordingMenu.addSeparator()
        recordingMenu.addAction(self.captureAction)
        recordingMenu.addAction(self.movieAction)
        # Plot menu
        plotMenu = menubar.addMenu('&Plot')
        plotMenu.addAction(self.plotAction)
        plotMenu.addAction(self.plotCOMAction)
        plotMenu.addAction(self.plotImpactAction)

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
            doCapture = (len(self.sim) % 4 == 1)

        if self.captureAction.isChecked() and doCapture:
            self.glwidget.capture()

    def renderTimerEvent(self):
        self.glwidget.updateGL()
        self.statusBar().showMessage(self.sim.status_string())
        self.rangeSlider.setRange(0, len(self.sim) - 1)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            print 'Escape key pressed! Bye.'
            self.close()

    def rangeSliderEvent(self, value):
        self.sim.set_world_frame(value)

    def screenshotEvent(self):
        self.glwidget.capture()

    def movieEvent(self):
        os.system('avconv -r 100 -i ./captures/frame.%04d.png output.mp4')
        os.system('rm ./captures/frame.*.png')

    def printEvent(self):
        print repr(self.sim.skel.x)

    def planEvent(self):
        self.sim.plan()

    def nextEvent(self):
        self.sim.tip_controller.next_target()

    def resetEvent(self):
        self.sim.reset()

    def plotEvent(self):
        self.sim.plan.plot()

    def plotCOMEvent(self):
        self.sim.history.plotCOM()

    def plotImpactEvent(self):
        self.sim.history.plotImpact()

    def loadEvent(self):
        filename = QtGui.QFileDialog.getOpenFileName(self, 'Open file',
                                                     '.', '*.plan')
        if len(filename) == 0:
            print 'load cancel'
            return
        print 'load:', filename
        self.sim.load(filename)
        print 'load OK'

    def saveEvent(self):
        filename = QtGui.QFileDialog.getSaveFileName(self, 'Open file',
                                                     '.', '*.plan')
        if len(filename) == 0:
            print 'save cancel'
            return
        if filename[-5:] != '.plan':
            filename += '.plan'
        print 'save:', filename
        self.sim.save(filename)
        print 'save OK'

    def loadmEvent(self):
        filename = QtGui.QFileDialog.getOpenFileName(self, 'Open file',
                                                     '.', '*.motion')
        if len(filename) == 0:
            print 'load motion cancel'
            return

        print 'load motion:', filename
        self.sim.load_motion(filename)
        print 'load motion OK'
        self.rangeSlider.setRange(0, len(self.sim) - 1)

    def savemEvent(self):
        filename = QtGui.QFileDialog.getSaveFileName(self, 'Open file',
                                                     '.', '*.motion')
        if len(filename) == 0:
            print 'save motion cancel'
            return
        if filename[-7:] != '.motion':
            filename += '.motion'
        print 'save motion:', filename
        self.sim.save_motion(filename)
        print 'save motion OK'


glutInit(sys.argv)
app = QtGui.QApplication(["Falling controller with Pydart"])
# widget = WfWidget()
# widget.show()
w = MyWindow()
w.show()
app.exec_()

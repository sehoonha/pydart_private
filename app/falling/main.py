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
from trackball import Trackball


import threading
import time


def bake(win):
    print 'Bake worker', win.sim.name
    time.sleep(1)
    print 'Macro: setWindowTitle'
    time.sleep(1)
    win.glwidget.captureIndex = 0
    win.captureAction.setChecked(False)
    time.sleep(1)
    print 'Macro: reset'
    win.resetEvent()
    time.sleep(1)
    print 'Macro: play'
    win.playAction.setChecked(True)
    while win.playAction.isChecked():
        time.sleep(1)
    print 'Macro: go to the first'
    win.rangeSlider.setValue(0)
    win.rangeSliderEvent(0)
    time.sleep(1)
    print 'Macro: display the first frame'
    time.sleep(1)
    print 'Macro: animation with recording'
    win.captureAction.setChecked(True)
    win.animAction.setChecked(True)
    while win.animAction.isChecked():
        time.sleep(1)
    print 'Macro: create a movie'
    win.movieEvent()
    time.sleep(5)
    print 'Macro: done'


def worker(win):
    """thread worker function"""
    return
    plan_filename = 'GP_8.0N_handfree'
    plan_filename += '.plan'
    print 'plan_filename', plan_filename
    name = win.sim.name

    # # Naive, Clean
    # bake(win)

    # # Naive, Impulse
    # win.sim.show_impulse = True
    # win.sim.name = name.replace('clean', 'impulse')
    # bake(win)

    # Load plan
    print 'plan_filename', plan_filename
    win.sim.load(plan_filename)

    # Planned, Clean
    win.sim.show_impulse = False
    win.sim.name = name.replace('naive', 'planned')
    bake(win)

    # Planned, Impulse
    win.sim.show_impulse = True
    win.sim.name = name.replace('clean', 'impulse').replace('naive', 'planned')
    bake(win)
    print 'All bakings are done by worker function!!!'
    return


class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()

        # Check and create captures directory
        if not os.path.isdir('captures'):
            os.makedirs('captures')

        # Create a simulation
        self.sim = Simulation()
        self.setWindowTitle('Falling - ' + self.sim.name)

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

        # self.cam0Event()
        self.cam1Event()
        # self.sim.load('gp_step_1.5.plan')
        # self.sim.load('gp_step_5.plan')
        # self.sim.load('test.plan')

        t = threading.Thread(target=worker, args=(self, ))
        t.start()

    def initUI(self):
        self.setGeometry(0, 0, 1280, 720)
        # self.setWindowTitle('Toolbar')

        self.glwidget = GLWidget(self)
        self.glwidget.setGeometry(0, 30, 1280, 720)
        self.glwidget.sim = self.sim

    def initActions(self):
        # Create actions
        self.planAction = QtGui.QAction('Plan', self)
        self.planAction.triggered.connect(self.planEvent)

        self.ikAction = QtGui.QAction('IK', self)
        self.ikAction.triggered.connect(self.ikEvent)

        self.ikCurrAction = QtGui.QAction('Fix', self)
        self.ikCurrAction.triggered.connect(self.ikCurrEvent)

        self.prevAction = QtGui.QAction('Prev', self)
        self.prevAction.triggered.connect(self.prevEvent)

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

        self.textSummaryAction = QtGui.QAction('Text Summary', self)
        self.textSummaryAction.triggered.connect(self.textSummaryEvent)

        self.plotExeAction = QtGui.QAction('Plot Executed', self)
        self.plotExeAction.triggered.connect(self.plotExeEvent)

        self.plotCOMAction = QtGui.QAction('Plot COM', self)
        self.plotCOMAction.triggered.connect(self.plotCOMEvent)

        self.plotImpulseAction = QtGui.QAction('Plot Impulse', self)
        self.plotImpulseAction.triggered.connect(self.plotImpulseEvent)

        self.plotAccelAction = QtGui.QAction('Plot Accel', self)
        self.plotAccelAction.triggered.connect(self.plotAccelEvent)

        # File Menu
        self.loadAction = QtGui.QAction('&Load Plan', self)
        self.loadAction.triggered.connect(self.loadEvent)

        self.saveAction = QtGui.QAction('&Save Plan', self)
        self.saveAction.triggered.connect(self.saveEvent)

        self.exportMtnAction = QtGui.QAction('&Export Plan as .mtn', self)
        self.exportMtnAction.triggered.connect(self.exportMtnEvent)

        self.loadmAction = QtGui.QAction('&Load Motion', self)
        self.loadmAction.triggered.connect(self.loadmEvent)

        self.savemAction = QtGui.QAction('&Save Motion', self)
        self.savemAction.triggered.connect(self.savemEvent)

        # Camera Menu
        self.cam0Action = QtGui.QAction('Camera0', self)
        self.cam0Action.triggered.connect(self.cam0Event)

        self.cam1Action = QtGui.QAction('Camera1', self)
        self.cam1Action.triggered.connect(self.cam1Event)

        self.printCamAction = QtGui.QAction('Print Camera', self)
        self.printCamAction.triggered.connect(self.printCamEvent)

    def initToolbar(self):
        # Create a toolbar
        self.toolbar = self.addToolBar('Control')
        self.toolbar.addAction(self.planAction)
        self.toolbar.addAction(self.ikAction)
        self.toolbar.addAction(self.prevAction)
        self.toolbar.addAction(self.nextAction)
        self.toolbar.addAction(self.ikCurrAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.resetAction)
        self.toolbar.addAction(self.playAction)
        self.toolbar.addAction(self.animAction)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.screenshotAction)
        self.toolbar.addAction(self.captureAction)
        self.toolbar.addAction(self.movieAction)
        # self.toolbar.addAction(self.plotAction)

        self.rangeSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.rangeSlider.valueChanged[int].connect(self.rangeSliderEvent)
        self.toolbar.addWidget(self.rangeSlider)

    def initMenu(self):
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addAction(self.loadAction)
        fileMenu.addAction(self.saveAction)
        fileMenu.addSeparator()
        fileMenu.addAction(self.exportMtnAction)
        fileMenu.addSeparator()
        fileMenu.addAction(self.loadmAction)
        fileMenu.addAction(self.savemAction)

        # Camera menu
        cameraMenu = menubar.addMenu('&Camera')
        cameraMenu.addAction(self.cam0Action)
        cameraMenu.addAction(self.cam1Action)
        cameraMenu.addSeparator()
        cameraMenu.addAction(self.printCamAction)

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
        plotMenu.addAction(self.plotExeAction)
        plotMenu.addSeparator()
        plotMenu.addAction(self.textSummaryAction)
        plotMenu.addAction(self.plotCOMAction)
        plotMenu.addAction(self.plotImpulseAction)
        plotMenu.addSeparator()
        plotMenu.addAction(self.plotAccelAction)

    def idleTimerEvent(self):
        doCapture = False
        # Do animation
        if self.animAction.isChecked():
            v = self.rangeSlider.value() + 1
            if v <= self.rangeSlider.maximum():
                self.rangeSlider.setValue(v)
            else:
                self.animAction.setChecked(False)
            doCapture = (v % 10 == 1)
        # Do play
        elif self.playAction.isChecked():
            result = self.sim.step()
            if result:
                self.playAction.setChecked(False)
            doCapture = (len(self.sim) % 4 == 1)

        if self.captureAction.isChecked() and doCapture:
            self.glwidget.capture(self.sim.name)

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
        name = self.sim.name
        cmd = 'avconv -r 200 -i ./captures/%s.%%04d.png %s.mp4' % (name, name)
        print 'Movie command:', cmd
        os.system(cmd)
        os.system('mv captures images_%s' % name)
        os.system('mkdir captures')
        # os.system('avconv -r 100 -i ./captures/frame.%04d.png output.mp4')
        # os.system('rm ./captures/frame.*.png')

    def printEvent(self):
        print repr(self.sim.skel.x)

    def planEvent(self):
        self.sim.do_plan()

    def ikEvent(self):
        self.sim.do_ik()

    def ikCurrEvent(self):
        self.sim.do_ik_curr()

    def prevEvent(self):
        self.sim.tip_controller.prev_target()

    def nextEvent(self):
        self.sim.tip_controller.next_target()

    def resetEvent(self):
        self.sim.reset()

    def plotEvent(self):
        self.sim.plan.plot()

    def textSummaryEvent(self):
        name = self.sim.name
        print 'textSummaryEvent'
        print '-' * 80
        txt = self.sim.plan.summary(self.sim.prob)
        txt += '\n====\n'
        txt += self.sim.status_string()
        print txt
        filename = '%s_summary.txt' % name
        with open(filename, 'w+') as fout:
            fout.write(txt)
        print '-' * 80
        print 'Write to ', filename
        print '-' * 80

    def plotExeEvent(self):
        self.sim.tip_controller.executed_plan.plot()

    def plotCOMEvent(self):
        # self.sim.plan.com_trajectory()
        # self.sim.history.plotCOM()
        self.sim.plot_com()

    def plotImpulseEvent(self):
        self.sim.plot_impulse()

    def plotAccelEvent(self):
        self.sim.plot_accel()

    def loadEvent(self):
        filename = QtGui.QFileDialog.getOpenFileName(self, 'Open file',
                                                     '.', '*.plan')
        if len(filename) == 0:
            print 'load cancel'
            return
        print 'load:', filename
        self.sim.load(filename)
        self.setWindowTitle('Falling - ' + self.sim.name)
        print 'load OK'

    def saveEvent(self):
        # filename = QtGui.QFileDialog.getSaveFileName(self, 'Open file',
        #                                              '.', '*.plan')
        # if len(filename) == 0:
        #     print 'save cancel'
        #     return
        # if filename[-5:] != '.plan':
        #     filename += '.plan'
        filename = '%s.plan' % self.sim.name
        print 'save:', filename
        self.sim.save(filename)
        print 'save OK'

    def exportMtnEvent(self):
        self.sim.export_motion()

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
        # filename = QtGui.QFileDialog.getSaveFileName(self, 'Open file',
        #                                              '.', '*.motion')
        # if len(filename) == 0:
        #     print 'save motion cancel'
        #     return
        # if filename[-7:] != '.motion':
        #     filename += '.motion'
        filename = '%s.motion' % self.sim.name
        print 'save motion:', filename
        self.sim.save_motion(filename)
        print 'save motion OK'

    def cam0Event(self):
        print 'cam0Event'
        if self.sim.is_bioloid():
            self.glwidget.tb = Trackball(phi=2.999638867, theta=-11.65482418,
                                         zoom=1,
                                         rot=[-0.05616939096242645,
                                              -0.6826683588086851,
                                              -0.05049280622231401,
                                              0.7268145485061248],
                                         trans=[-0.39,
                                                -0.14,
                                                -0.7499999999999998])
        else:
            self.glwidget.tb = Trackball(phi=85.0,
                                         theta=-85.0,
                                         zoom=1,
                                         rot=[-0.018834950780547477,
                                              -0.7068710460660375,
                                              -0.01925124892805669,
                                              0.7068295114646171],
                                         trans=[-1.1100000000000005,
                                                -0.8100000000000005,
                                                -3.3600000000000114])

    def cam1Event(self):
        print 'cam1Event: frontview'
        if self.sim.is_bioloid():
            self.glwidget.tb = Trackball(phi=2.999638867, theta=-11.65482418,
                                         zoom=1,
                                         rot=[-0.05616939096242645,
                                              -0.6826683588086851,
                                              -0.05049280622231401,
                                              0.7268145485061248],
                                         trans=[-0.21,
                                                -0.14,
                                                -1.20])
        else:
            self.glwidget.tb = Trackball(phi=85.0,
                                         theta=-85.0,
                                         zoom=1,
                                         rot=[-0.018834950780547477,
                                              -0.7068710460660375,
                                              -0.01925124892805669,
                                              0.7068295114646171],
                                         trans=[-0.63,
                                                -0.73,
                                                -5.58])

    def printCamEvent(self):
        print 'printCamEvent'
        print '----'
        print repr(self.glwidget.tb)
        print '----'

glutInit(sys.argv)
app = QtGui.QApplication(["Falling controller with Pydart"])
# widget = WfWidget()
# widget.show()
w = MyWindow()
w.show()
app.exec_()

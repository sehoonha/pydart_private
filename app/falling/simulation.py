
import sys
import config
sys.path.append(config.PYDART_PATH)
import os
import time
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)
import cPickle as pickle

import pydart
import numpy as np
from OpenGL.GL import glPushMatrix, glPopMatrix, glColor
import gltools

from history import History
import events
from ik.ik_multi import IKMulti
import scene.configure
import scene.range_checker
import problem
import abstract.tip
import abstract.tip_v2
import abstract.twotip
import abstract.dynamic
import abstract.plan
import model.controller
import cProfile


class Simulation(object):
    def __init__(self):

        # Init api
        pydart.init()
        self.world = pydart.create_world(1.0 / 2000.0)
        self.world.add_skeleton(config.DATA_PATH + "sdf/ground.urdf",
                                control=False)
        # self.world.add_skeleton(config.DATA_PATH +
        #                         "urdf/BioloidGP/BioloidGP.URDF")
        self.world.add_skeleton(config.DATA_PATH +
                                "urdf/atlas/atlas_v3_no_head.urdf")

        self.skel = self.world.skel  # shortcut for the control skeleton
        for i, dof in enumerate(self.skel.dofs):
            print i, dof

        self.skel.set_joint_damping(0.15)

        # Configure the scene
        self.cfg = scene.configure.Configure(self)
        self.prob = problem.Problem(self, self.cfg.name)
        self.history = History(self)

        # # ### Now, configure the controllers
        self.tip_controller = model.controller.Controller(self.skel, self.prob)
        self.event_handler = events.Handler()

        # Reset to the initial state
        self.reset()

        print 'skel.m = ', self.skel.m
        print 'skel.approx_inertia_x = ', self.skel.approx_inertia_x()
        print 'skel.q = ', self.skel.q

        self.rc = scene.range_checker.RangeChecker(self)
        self.rc.check_all()

        # Abstract model
        self.abstract_tip = abstract.dynamic.DynamicTIP(self.prob, self.rc)

        # For handle callbacks properly..
        self.history.clear()
        self.history.push()

    def is_bioloid(self):
        return ("BioloidGP" in self.skel.filename)

    @property
    def tip(self):
        return self.tip_controller.tip()

    def plan(self):
        # Plan with Dynamic TIP
        self.abstract_tip.set_x0(self.tip_controller.tips)
        self.abstract_tip.plan_initial()
        # print
        # print 'start profiling..............................'
        # print
        # # cProfile.run('self.abstract_tip.plan_initial()')
        # cProfile.runctx('self.abstract_tip.plan_initial()',
        #                 globals(), locals())
        # print
        # print 'finish profiling....................!!!!!!!'
        # print
        x0 = self.abstract_tip.x0
        path = self.abstract_tip.path

        print 'x0 = ', str(x0)
        print 'path = ', str(path)
        # print 'sleep 5 seconds'
        # time.sleep(5)
        self.plan = abstract.plan.Plan(x0, path)
        print 'new plan is generated'
        self.plan.plot()
        self.tip_controller = model.controller.Controller(self.skel,
                                                          self.prob,
                                                          self.plan)
        print 'new tip controller is generated'

        self.ik = IKMulti(self, self.plan)
        self.ik.optimize(restore=False)
        # print
        # print 'start profiling..............................'
        # print
        # cProfile.runctx('self.ik.optimize(restore=False)',
        #                 globals(), locals())
        # print
        # print 'finish profiling....................!!!!!!!'
        # print
        self.tip_controller.targets = self.ik.targets

    def reset(self):
        # # Reset Pydart
        # self.skel.x = self.cfg.init_state
        # for i in range(self.):
        #     (b, f, p) = self.cfg.ext_force
        #     body = self.skel.body(b)
        #     body.add_ext_force_at(f, p)
        #     self.skel.tau = self.tip_controller.control()
        #     self.world.step()
        # state_after_pushed = self.skel.x
        # self.world.reset()
        # self.skel.x = state_after_pushed
        self.cfg.reset_simulation(self)

        # Reset inner structures
        self.tip_controller.reset()
        self.event_handler.clear()
        self.history.clear()
        self.history.push()
        self.history.callbacks = [self.tip_controller]

        self.terminated = dict()

    def step(self):
        self.skel.tau = self.tip_controller.control()
        self.world.step()
        self.history.push()

        if self.tip_controller.check_next():
            self.event_handler.push("proceed", 40)

        for e in self.event_handler.pop():
            # print 'New Event: ', e.name, 'at', self.world.nframes
            if e.name == "proceed":
                self.tip_controller.proceed()
                # print 'proceed:', self.tip_controller.is_terminated()
                if self.tip_controller.is_terminated():
                    self.event_handler.push("terminate", 0)
            elif e.name == 'pause':
                return True
            elif e.name == 'terminate':
                self.event_handler.push("terminate", 9999)
                return True

        self.event_handler.step()
        # print self.world.nframes, ':', self.event_handler
        return False

    def render(self):
        glPushMatrix()
        gltools.render_axis(10)
        # Draw chess board
        gltools.glMove([0.0, -0.01, 0.0])
        gltools.render_chessboard(10, 20.0)

        # Draw skeleton
        gltools.glMove([0, 0, 0])
        self.skel.render()

        gltools.glMove([0, 0, 0])
        glColor(1, 0, 0)
        gltools.render_arrow(self.skel.C,
                             self.skel.C + 0.2 * self.tip.projected_Cdot())

        # Draw TIP
        tip_index = self.history.get_frame()['tip_index']
        tips = self.tip_controller.tips
        for i in range(tip_index, len(tips)):
            tips[i].render()

        # Draw contacts
        gltools.glMove([0, 0, 0])
        glColor(0.7, 0.0, 0.3)
        for c in self.history.get_frame()['contacts']:
            gltools.render_arrow(c[0:3], c[0:3] - 0.001 * c[3:6])

        for c in self.prob.contacts:
            c.render()

        glPopMatrix()

    def status_string(self):
        data = self.history.get_frame()

        status = ""
        status += "T = %.4f (%d) " % (data['t'], data['nframes'])
        status += "C = (%.4f %.4f) " % (data['C.x'], data['C.y'])
        status += "P = (%.4f %.4f) " % (data['P.x'], data['P.y'])
        status += "Impulse = %.4f (max %.4f) " % (data['impulse'],
                                                  data['max_impulse'])
        status += "I = %.4f " % self.skel.approx_inertia_x()
        # status += "TIP = " + str(data['tip']) + " "
        tip = self.tip_controller.tip()
        status += "TIP = " + str(tip)
        status += "Contacted = %s " % str(data['contactedBodies'])

        return status

    def set_world_frame(self, i):
        self.world.set_frame(i)
        # pydart_api.setWorldSimFrame(i)
        self.history.pop(i)

    def save(self, filename):
        protocol = pickle.HIGHEST_PROTOCOL
        # protocol = 0
        with open(filename, 'w+') as fp:
            pickle.dump(self.plan, fp, protocol)
            pickle.dump(self.tip_controller, fp, protocol)

    def load(self, filename):
        with open(filename, 'r') as fp:
            self.plan = pickle.load(fp)
            self.tip_controller = pickle.load(fp)
        print self.plan

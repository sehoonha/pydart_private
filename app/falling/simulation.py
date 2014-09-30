import sys
import config
sys.path.append(config.PYDART_PATH)
import os
import time
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)

import pydart
import numpy as np
from OpenGL.GL import glPushMatrix, glPopMatrix, glColor
import gltools

from history import History
import events
from ik.ik_multi import IKMulti
import scene.config
import scene.range_checker
import problem
import abstract.tip
import abstract.tip_v2
import abstract.twotip
import abstract.dynamic
import abstract.plan
import model.controller


def STR(v):
    return str(["%.3f" % x for x in v])


class Simulation(object):
    def __init__(self):

        # Init api
        pydart.init()
        self.world = pydart.create_world(1.0 / 2000.0)
        self.world.add_skeleton(config.DATA_PATH + "sdf/ground.urdf",
                                control=False)
        self.world.add_skeleton(config.DATA_PATH +
                                "urdf/BioloidGP/BioloidGP.URDF")

        self.skel = self.world.skel  # shortcut for the control skeleton
        for i, dof in enumerate(self.skel.dofs):
            print i, dof

        self.skel.set_joint_damping(0.15)

        # Configure the scene
        self.cfg = scene.config.Config(self)
        print 'conditions = ', self.cfg.conditions
        self.prob = problem.Problem(self)
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
        # self.abstract_tip = abstract.tip_v2.TIPv2()
        # self.abstract_tip = abstract.tip.TIP()
        # self.abstract_twotip = abstract.twotip.TWOTIP()
        self.abstract_tip = abstract.dynamic.DynamicTIP(self.prob, self.rc)

        # For handle callbacks properly..
        self.history.clear()
        self.history.push()

    @property
    def tip(self):
        return self.tip_controller.tip()

    def plan(self):
        # Plan with Dynamic TIP
        self.abstract_tip.set_x0(self.tip_controller.tips)
        self.abstract_tip.plan_initial()
        x0 = self.abstract_tip.x0
        path = self.abstract_tip.path

        print 'x0 = ', repr(x0)
        print 'path = ', repr(self.abstract_tip.path)
        # print 'sleep 5 seconds'
        # time.sleep(5)
        pn = abstract.plan.Plan(x0, path)
        print 'new plan is generated'
        self.tip_controller = model.controller.Controller(self.skel,
                                                          self.prob, pn)
        print 'new tip controller is generated'

        # pn.plot()
        self.ik = IKMulti(self, pn)
        self.ik.optimize(restore=False)
        self.tip_controller.targets = self.ik.targets

    def reset(self):
        # print '== reset =='
        # Reset the configure
        if self.cfg.conditions:
            cond = self.cfg.conditions[0]
            self.cfg.config(*cond)

        # Reset Pydart
        self.skel.x = self.cfg.init_state
        for i in range(10):
            (b, f, p) = self.cfg.ext_force
            body = self.skel.body(b)
            body.add_ext_force_at(f, p)
            self.skel.tau = self.tip_controller.control()
            self.world.step()
        state_after_pushed = self.skel.x
        self.world.reset()
        self.skel.x = state_after_pushed

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
        # status += "l_hand.v = %s " % STR(data['l_hand.v'])
        status += "I = %.4f " % self.skel.approx_inertia_x()
        status += "TIP = " + str(data['tip']) + " "
        status += "Contacted = %s " % str(data['contactedBodies'])

        return status

    def set_world_frame(self, i):
        self.world.set_frame(i)
        # pydart_api.setWorldSimFrame(i)
        self.history.pop(i)

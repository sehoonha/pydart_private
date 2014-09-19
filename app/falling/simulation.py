import sys
import config
sys.path.append(config.PYDART_PATH)

import pydart
import numpy as np
from OpenGL.GL import glPushMatrix, glPopMatrix, glColor
import gltools

from history import History
from poses import BioloidGPPoses
import events
from control.pd import PDController
from model.tip import TIP
from model.contact import Contact
from ik.ik import IK
import abstract.tip
import abstract.tip_v2
import abstract.twotip
import abstract.dynamic


def confine(x, lo, hi):
    return min(max(lo, x), hi)


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
        self.history = History(self)

        # ### Now, configure the controllers
        # Abstract view of skeleton
        self.tips = [TIP(self.skel, 'rfoot', 'lfoot'),
                     TIP(self.skel, 'lfoot', 'hands')]
        # self.tips = [TIP(self.skel, 'feet', 'hands'), ]
        #              TIP(self.skel, 'hands', 'head')]

        self.event_handler = events.Handler()

        # Reset to the initial state
        self.reset()

        print 'skel.m = ', self.skel.m
        print 'skel.approx_inertia_x = ', self.skel.approx_inertia_x()
        print 'skel.q = ', self.skel.q

        # Abstract model
        # self.abstract_tip = abstract.tip_v2.TIPv2()
        # self.abstract_tip = abstract.tip.TIP()
        # self.abstract_twotip = abstract.twotip.TWOTIP()
        self.abstract_tip = abstract.dynamic.DynamicTIP()

        # Control
        self.maxTorque = 0.3 * 1.5
        self.pd = PDController(self.skel, 50.0, 1.0)
        self.pd.target = self.skel.q

        # For handle callbacks properly..
        self.history.clear()
        self.history.push()

        # Start to test more contact candidates
        defs = [
            # ("feet", ["l_foot", "r_foot"], [[-0.05, 0.025, 0]] * 2),
            ("hands", ["l_hand", "r_hand"],
             [[0, -0.11, 0.01], [0, 0.11, -0.01]]),
            ("knees", ["l_shin", "r_shin"], [[0, 0, 0], [0, 0, 0]]),
            ("l_heel", ["l_foot"], [[0.05, 0.025, 0.0]]),
            ("l_toe", ["l_foot"], [[-0.05, 0.025, 0.0]]),
            ("r_toe", ["r_foot"], [[-0.05, 0.025, 0.0]]),
            ("head", ["torso"], [[0.0, 0.0, 0.03]]),
        ]
        self.contacts = [Contact(self.skel, n, b, p) for n, b, p in defs]

    @property
    def tip(self):
        return self.tips[self.tip_index]

    def plan(self):
        # # Plan with TIP
        # self.abstract_tip.set_x0(self.tip)
        # self.abstract_tip.set_bounds(self.tip)
        # self.abstract_tip.optimize()
        # ik = IK(self)
        # self.pd.target = ik.optimize(restore=False)

        # # Direct planning in FB
        # ik = IK(self)
        # ik.optimize_with_fullbody_motion()

        # ### Plan with Double TIP
        # ik = IK(self)
        # self.pd.target = ik.optimize(restore = True)

        # # Plan with Sequential TIP
        # self.abstract_twotip.set_x0(self.tips)
        # self.abstract_twotip.set_bounds(self.tips)
        # self.abstract_twotip.simulate_random()
        # # self.abstract_twotip.optimize()
        # ik = IK(self)
        # self.pd.target = ik.optimize(restore=False)

        # Plan with Dynamic TIP
        self.abstract_tip.set_x0(self.tips)
        self.abstract_tip.set_bounds(self.tips)
        # self.abstract_tip.test_control()
        # self.abstract_tip.plan_initial()
        ik = IK(self)
        self.pd.target = ik.optimize(restore=False)

    def control(self):
        tau = np.zeros(self.skel.ndofs)
        # Track the initial pose
        tau += self.pd.control()

        # Erase the first six dof forces
        tau[0:6] = 0
        for i in range(6, self.skel.ndofs):
            tau[i] = confine(tau[i], -self.maxTorque, self.maxTorque)

        return tau

    def reset(self):
        # Reset Pydart
        # self.skel.x = BioloidGPPoses().stand()
        self.skel.x = BioloidGPPoses().stepping()
        # self.skel.x = BioloidGPPoses().side()

        self.world.reset()

        # ### Reset inner structures
        self.event_handler.clear()
        self.tip_index = 0
        self.history.clear()
        self.history.push()
        self.history.callbacks = [self.tip]

        self.terminated = dict()

    def step(self):
        # if self.world.nframes < 10:
        #     print 'push!!'
        #     torso = self.skel.body("torso")
        #     torso.add_ext_force_at([-99, 0, 0], [0, 0, 0.03])
        # elif self.world.nframes == 10:
        #     self.event_handler.push("pause", 0)

        self.skel.tau = self.control()
        self.world.step()
        self.history.push()

        return (self.world.t > 1.0)

        pivot_nodes = []
        for i in range(self.tip_index + 1):
            pivot_nodes += self.tips[i].pivot_nodes()

        if len(set(self.skel.contacted_body_names()) - set(pivot_nodes)) > 0:
            if self.tip_index < len(self.tips) - 1:
                # self.event_handler.push("terminate", 40)
                self.event_handler.push("proceed", 40)
            else:
                self.event_handler.push("terminate", 40)

        for e in self.event_handler.pop():
            # print 'New Event: ', e.name, 'at', self.world.nframes
            if e.name == "proceed":
                self.history.callbacks.remove(self.tip)
                self.tip_index += 1
                self.history.callbacks += [self.tip]
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

        # # Draw TIP
        # tip_index = self.history.get_frame()['tip_index']
        # for i in range(tip_index, len(self.tips)):
        #     self.tips[i].render()

        # Draw contacts
        gltools.glMove([0, 0, 0])
        glColor(0.7, 0.0, 0.3)
        for c in self.history.get_frame()['contacts']:
            gltools.render_arrow(c[0:3], c[0:3] - 0.001 * c[3:6])

        for c in self.contacts:
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

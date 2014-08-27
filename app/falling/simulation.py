import os, sys
import config
sys.path.append(config.PYDART_PATH)

import pydart
import pydart_api
import numpy as np
import numpy.linalg as LA
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import gltools

from history import *
from poses import *
from control.pd import *
from control.jt import *
from control.com_tracker import *
from model.tip import *
from ik.ik import *
import abstract.model

def confine(x, lo, hi):
    return min(max(lo, x), hi)

class Simulation:
    def __init__(self):

        # Init api
        pydart.init()
        self.world = pydart.create_world(1.0 / 2000.0)
        self.world.add_skeleton(config.DATA_PATH + "sdf/ground.urdf", control = False)
        self.world.add_skeleton(config.DATA_PATH + "urdf/BioloidGP/BioloidGP.URDF")

        self.skel = self.world.skel # shortcut for the control skeleton
        
        self.skel.set_joint_damping(0.15)
        self.skel.q = BioloidGPPoses().leaned_pose()

        print 'skel.q = ', self.skel.q

        # Record the history
        self.history = History(self)

        # Abstract view of skeleton
        self.tip = TIP(self.skel)
        self.history.callbacks += [self.tip]

        # Abstract model
        self.abstract_tip = abstract.model.TIP()
        self.abstract_tip.load_history(config.DATA_PATH + 'TIP.csv')
        
        # Control
        self.maxTorque = 0.3 * 1.5

        self.pd = PDController(self.skel, 50.0, 1.0)
        self.pd.target = self.skel.q

        # IK
        ik = IK(self)
        self.pd.target = ik.optimize(restore = True)

        self.history.push()

    def control(self):
        tau = np.zeros(self.skel.ndofs)
        # Track the initial pose
        tau += self.pd.control()

        # Erase the first six dof forces
        tau[0:6] = 0
        for i in range(6, self.skel.ndofs):
            tau[i] = confine(tau[i], -self.maxTorque, self.maxTorque)

        return tau

    def step(self):
        self.skel.forces = self.control()
        self.world.step()
        self.history.push()

        # if math.fabs(self.getTime() - 1.0000) < 0.0001:
        if len(set(self.skel.contacted_body_names()) - set(['l_foot', 'r_foot'])) > 0:
            return True
        return False


    def render(self):
        glPushMatrix()
        # Draw chess board
        gltools.glMove([0.0, -0.01, 0.0])
        gltools.render_chessboard(10, 20.0)

        # Draw skeleton
        gltools.glMove([0, 0, 0])
        self.skel.render()

        # Draw TIP
        self.tip.render()

        # Draw contacts
        gltools.glMove([0, 0, 0])
        glColor(0.7, 0.0, 0.3)
        for c in self.history.get_frame()['Contacts']:
            gltools.render_arrow(c[0:3], c[0:3] - 1.0 * c[3:6])


        glPopMatrix()


    def status_string(self):
        status = ""
        status += "T = %.4f (%d)" % (self.world.t, self.world.nframes)
        status += "COM = " + str(["%.3f" % x for x in self.skel.C]) + " "
        status += "TIP = " + str(self.tip) + " "
        status += "Contacted = " + str(self.skel.contacted_body_names()) + " "

        return status

    # def getTime(self):
    #     return pydart_api.getWorldTime()

    # def getSimFrames(self):
    #     return pydart_api.getWorldSimFrames()
            
    # def getPositions(self):
    #     return pydart_api.getSkeletonPositions(self.rid, self.ndofs)

    # def getVelocities(self):
    #     return pydart_api.getSkeletonVelocities(self.rid, self.ndofs)

    # def setPositions(self, q):
    #     pydart_api.setSkeletonPositions(self.rid, q)

    # def getCOM(self):
    #     return pydart_api.getSkeletonWorldCOM(self.rid)

    # def getContactedBodyNames(self):
    #     nbodies = pydart_api.getSkeletonNumBodies(self.rid)
    #     return [pydart_api.getSkeletonBodyName(self.rid, i) for i in range(nbodies) if pydart_api.getBodyNodeNumContacts(self.rid, i)]
        

    # def getBodyNodeTransformation(self, name):
    #     return pydart_api.getBodyNodeTransformation(self.rid, name)
        
    # def getBodyNodeWorldLinearJacobian(self, name):
    #     J = np.zeros((3, self.ndofs))
    #     pydart_api.getBodyNodeWorldLinearJacobian(self.rid, name, J)
    #     return J

    # def getWorldContacts(self):
    #     n = pydart_api.getWorldNumContacts()
    #     contacts = pydart_api.getWorldContacts(6 * n)
    #     return [contacts[6 * i : 6 * i + 6] for i in range(n)]
        
    # def numSimFrames(self):
    #     return pydart_api.getWorldSimFrames()

    def set_world_frame(self, i):
        self.world.set_frame(i)
        # pydart_api.setWorldSimFrame(i)
        self.history.pop(i);


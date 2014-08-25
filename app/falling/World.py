import os, sys
import Config
sys.path.append(Config.PYDART_PATH)

import pydart_api
import numpy as np
import numpy.linalg as LA
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import GLTools

from History import *
from Poses import *
from control.PDController import *
from control.JTController import *
from control.COMTracker import *
from model.TIP import *
from ik.IK import *

def confine(x, lo, hi):
    return min(max(lo, x), hi)

class World:
    def __init__(self):
        # Record the history
        self.history = History(self)

        # Init api
        pydart_api.init()
        pydart_api.createWorld(1.0 / 2000.0)
        pydart_api.addSkeleton(Config.DATA_PATH + "sdf/ground.urdf")

        # Configure the robot
        self.rid = pydart_api.addSkeleton(Config.DATA_PATH + "urdf/BioloidGP/BioloidGP.URDF")
        self.ndofs = pydart_api.getSkeletonNumDofs(self.rid)
        pydart_api.setSkeletonJointDamping(self.rid, 0.15)
        print "robot id = ", self.rid, ' nDofs = ', self.ndofs

        # Configure the pose
        self.setPositions(BioloidGPPoses().leanedPose())
        # self.setPositions(BioloidGPPoses().steppingPose())
        print 'positions = ', self.getPositions()

        # Simplified model
        self.tip = TIP(self)
        
        # Control
        self.maxTorque = 0.3 * 1.5

        self.pd = PDController(self.ndofs, 20.0, 1.0)
        self.pd.target = self.getPositions()
        self.jt = JTController(self)
        self.ct = COMTracker(self, Config.DATA_PATH + 'COM.csv')
        self.history.callbacks += [self.ct]

        # IK
        self.ik = IK(self)
        # self.ik.getCOMToBodyPoint().target = 0.10
        self.pd.target = self.ik.optimize(restore = True)

        self.history.push()

    def control(self):
        tau = np.zeros(self.ndofs)
        # Track the initial pose
        tau += self.pd.control( self.getPositions(), self.getVelocities() )

        # # Apply VF to feet and shrink the length
        # C = pydart_api.getSkeletonWorldCOM(self.rid)
        # u = C / LA.norm(C)
        # f = u * 20.0
        # tau += self.jt.control( ["l_foot", "r_foot"], f )

        # # w = np.array([1.0, 0.0, 0.0])
        # # v = np.cross(w, u)
        # v = np.array([0.0, -1.0, 0.0])
        # f_hand = v * 20.0
        # tau += self.jt.control( "l_hand", f_hand )
        # tau += self.jt.control( "r_hand", f_hand )
        
        # tau[10] += 2.0
        # tau[12] += 2.0

        # COM Tracker
        tau += self.ct.control( self.getTime(), pydart_api.getSkeletonWorldCOM(self.rid) )
        # Erase the first six dof forces
        tau[0:6] = 0
        for i in range(6, self.ndofs):
            tau[i] = confine(tau[i], -self.maxTorque, self.maxTorque)

        return tau

    def step(self):
        pydart_api.setSkeletonForces(self.rid, self.control())
        pydart_api.stepWorld()
        self.history.push()

        # if pydart_api.getWorldSimFrames() % 10 == 0:
        #     C = pydart_api.getSkeletonWorldCOM(self.rid)
        #     Cd = pydart_api.getSkeletonWorldCOMVelocity(self.rid)
        #     print "%.4f, %.4f, %.4f" % (pydart_api.getWorldTime(), Cd[1], math.atan2(C[2], C[1])),
        #     print "".join([", %.4f" % x for x in C]),
        #     print ", %.4f" % LA.norm(C)


        if math.fabs(self.getTime() - 1.0000) < 0.0001:
            return True
        return False

    def glMove(self, pos):
        glPopMatrix()
        glPushMatrix()
        glTranslate(pos[0], pos[1], pos[2])

    def render(self):
        glPushMatrix()
        # Draw skeleton
        pydart_api.renderSkeleton(self.rid)

        # for c in self.getWorldContacts():
        for c in self.history.getFrame()['Contacts']:
            GLTools.renderArrow(c[0:3], c[0:3] - 1.0 * c[3:6])

        # Draw chess board
        self.glMove([0.0, -0.01, 0.0])
        GLTools.renderChessBoard(10, 20.0)

        # # Draw COM
        # self.glMove( pydart_api.getSkeletonWorldCOM(self.rid) )
        # glColor(1.0, 0.0, 0.0, 0.8)
        # glutSolidSphere(0.05, 4, 2)

        # Draw TIP
        self.tip.render()

        glPopMatrix()


    def statusString(self):
        status = ""
        status += "T = %.4f (%d)" % (self.getTime(), self.getSimFrames())
        status += "COM = " + str(["%.3f" % x for x in self.getCOM()]) + " "
        status += "TIP = " + str(["%.3f" % x for x in self.tip.getState()]) + " "

        return status

    def getTime(self):
        return pydart_api.getWorldTime()

    def getSimFrames(self):
        return pydart_api.getWorldSimFrames()
            
    def getPositions(self):
        return pydart_api.getSkeletonPositions(self.rid, self.ndofs)

    def getVelocities(self):
        return pydart_api.getSkeletonVelocities(self.rid, self.ndofs)

    def setPositions(self, q):
        pydart_api.setSkeletonPositions(self.rid, q)

    def getCOM(self):
        return pydart_api.getSkeletonWorldCOM(self.rid)

    def getBodyNodeTransformation(self, name):
        return pydart_api.getBodyNodeTransformation(self.rid, name)
        
    def getBodyNodeWorldLinearJacobian(self, name):
        J = np.zeros((3, self.ndofs))
        pydart_api.getBodyNodeWorldLinearJacobian(self.rid, name, J)
        return J

    def getWorldContacts(self):
        n = pydart_api.getWorldNumContacts()
        contacts = pydart_api.getWorldContacts(6 * n)
        return [contacts[6 * i : 6 * i + 6] for i in range(n)]
        
    def numSimFrames(self):
        return pydart_api.getWorldSimFrames()

    def setWorldSimFrame(self, i):
        pydart_api.setWorldSimFrame(i)
        self.history.pop(i);


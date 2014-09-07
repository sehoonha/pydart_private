import math
import numpy as np

class BioloidGPPoses:
    def __init__(self):
        self.ndofs = 22
        names = ['rx', 'ry', 'rz', 'tx', 'ty', 'tz', 'l_hip', 'l_shoulder', 'r_hip', 'r_shoulder', 'l_thigh', 'l_arm', 'r_thigh', 'r_arm', 'l_shin', 'l_hand', 'r_shin', 'r_hand', 'l_heel', 'r_heel', 'l_foot',  'r_foot']
        self.dofs = { names[i] : i for i in range(len(names)) }

    def leaned_pose(self):
        q = np.zeros(self.ndofs)
        q[0] = -0.41 * math.pi
        q[4] = 0.221
        q[5] = q[4]
        return q

    def stepping_pose(self):
        q = np.zeros(self.ndofs)
        q[0] = -0.44 * math.pi
        q[4] = 0.225
        q[5] = q[4]
        q[self.dofs['r_hip']] = -0.1
        q[self.dofs['r_foot']] = 0.1
        q[self.dofs['l_hip']] = 0.1
        q[self.dofs['l_foot']] = -0.1
        q[self.dofs['l_thigh']] = 1.3
        
        return q
        

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

    def stepping2_pose(self):
        return np.array([ -1.36108996e+00,  -2.32449899e-04,  -1.38394301e-04,
        -2.65499732e-05,   2.25060793e-01,   2.24089733e-01,
         1.00355394e-01,  -1.62119860e-03,  -9.98813291e-02,
        -1.78460384e-03,   1.31169001e+00,   1.03413412e-04,
         1.54253901e-02,   8.46826654e-05,   3.50951937e-03,
         1.18418164e-03,   4.79084271e-03,   1.27505475e-03,
         7.38350432e-04,   1.11403096e-03,  -9.95010855e-02,
         9.96810523e-02])

    def stepping2_vel(self):
        return np.array([ 3.19884897,  0.28369571,  0.18538221,  0.03455344, -0.89728113,
        0.07490172, -0.30123542,  0.33764931, -0.20052682,  0.47040108,
        0.5076734 , -0.56221476, -0.44727244, -0.56073295, -1.19884757,
       -1.60686947,  0.37782936, -1.83634955,  1.4344175 ,  1.94303486,
       -1.91136289, -0.1167901 ])

        

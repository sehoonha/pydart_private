import math
import numpy as np


class BioloidGPPoses:
    def __init__(self):
        self.ndofs = 22
        names = ['rx', 'ry', 'rz', 'tx', 'ty', 'tz', 'l_hip', 'l_shoulder',
                 'r_hip', 'r_shoulder', 'l_thigh', 'l_arm', 'r_thigh',
                 'r_arm', 'l_shin', 'l_hand', 'r_shin', 'r_hand', 'l_heel',
                 'r_heel', 'l_foot', 'r_foot']
        self.dofs = {names[i]: i for i in range(len(names))}

    def stand_pose(self):
        q = np.zeros(self.ndofs)
        q[0] = -0.50 * math.pi
        q[4] = 0.231 + 0.0 / math.sqrt(2) * (10.0 / 9.0)
        q[5] = q[4]
        return q

    def lean_pose(self):
        q = np.zeros(self.ndofs)
        q[0] = -0.41 * math.pi
        q[4] = 0.228
        q[5] = q[4]
        return q

    def step_pose(self):
        q = np.zeros(self.ndofs)
        q[0] = -0.44 * math.pi
        q[4] = 0.225
        q[5] = q[4]
        q[self.dofs['r_hip']] = -0.1
        q[self.dofs['r_foot']] = 0.1
        q[self.dofs['l_hip']] = 0.3
        q[self.dofs['l_foot']] = -0.2
        q[self.dofs['l_thigh']] = 1.2
        q[self.dofs['l_heel']] = -0.3
        return q

    def step_low_pose(self):
        q = self.step_pose()
        q[0] += 0.07 * math.pi
        q[4] -= 0.02
        q[5] = q[4]

        q[self.dofs['r_thigh']] = 0.6
        q[self.dofs['r_shin']] = -0.6
        q[self.dofs['r_heel']] = 0.3
        q[self.dofs['l_thigh']] += 0.3
        return q

    def skate_pose(self):
        q = np.zeros(self.ndofs)
        q[0] = -0.14 * math.pi
        q[4] = 0.215
        q[5] = 0.130
        q[self.dofs['l_shoulder']] = 0.1
        q[self.dofs['l_hand']] = 1.2
        q[self.dofs['r_shoulder']] = 0.1
        q[self.dofs['r_hand']] = 1.2

        q[self.dofs['l_hip']] = 0.3
        q[self.dofs['l_foot']] = -0.2

        q[self.dofs['l_thigh']] = 0.9
        q[self.dofs['l_shin']] = 0.5
        q[self.dofs['l_heel']] = -0.3
        q[self.dofs['r_thigh']] = -0.5
        return q

    def back_pose(self):
        q = np.zeros(self.ndofs)
        q[3:6] = [-0.177, 0.172, 0.142]
        # q[3:6] += (np.random.rand(3) - 0.5) * 0.06
        print 'q[3:6] =', q[3:6]
        # q[0] += 0.07 * math.pi
        q[:3] = np.array([-0.05, -2.25, -2.15])
        # q[:3] += (np.random.rand(3) - 0.5) * 0.6
        # Side? q[:3] =  [-0.25776439  1.19493421  1.01768632]
        # Side? q[:3] =  [-1.16825849  1.02963722  1.93913537]
        # Back? q[:3] =  [ 0.11217433 -2.73289058 -2.20660662]
        print 'q[:3] = ', q[:3]

        q[self.dofs['l_thigh']] = 0.9
        q[self.dofs['r_thigh']] = 0.9
        q[self.dofs['l_heel']] = -0.9
        q[self.dofs['r_heel']] = -0.9

        q[self.dofs['l_shoulder']] = -1.0
        q[self.dofs['r_shoulder']] = -1.0
        # q[self.dofs['l_hand']] = 1.2
        # q[self.dofs['r_hand']] = 1.2

        return q

    def side_pose(self):
        q = np.zeros(self.ndofs)
        q[:3] = np.array([-1.2, 1.2, 1.2])
        q[3:6] = [0.148, 0.208, 0.225]
        q[self.dofs['l_shoulder']] = -0.5
        q[self.dofs['r_shoulder']] = -0.5
        return q

    def zeros(self):
        return np.zeros(self.ndofs)

    def stand(self):
        return np.concatenate((self.stand_pose(), self.zeros()))

    def step_low(self):
        return np.concatenate((self.step_low_pose(), self.zeros()))

    def step(self):
        return np.concatenate((self.step_pose(), self.zeros()))

    def lean(self):
        return np.concatenate((self.lean_pose(), self.zeros()))

    def skate(self):
        return np.concatenate((self.skate_pose(), self.zeros()))

    def back(self):
        return np.concatenate((self.back_pose(), self.zeros()))

    def side(self):
        return np.concatenate((self.side_pose(), self.zeros()))

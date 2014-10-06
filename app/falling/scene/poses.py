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
        q[4] = 0.221
        q[5] = q[4]
        return q

    def side_pose(self):
        q = np.zeros(self.ndofs)
        q[0] = -0.50 * math.pi
        q[4] = 0.231 + 0.0 / math.sqrt(2) * (10.0 / 9.0)
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

    def zeros(self):
        return np.zeros(self.ndofs)

    def stand(self):
        return np.concatenate((self.stand_pose(), self.zeros()))

    def side(self):
        return np.array([-1.57048603e+00, -2.04550780e-02, 3.08763227e-02,
                         1.28061914e-03, 2.31055139e-01, 2.31022384e-01,
                         2.01238546e-02, -1.29290010e-03, 2.26396219e-02,
                         1.16151163e-03, -3.33133459e-04, -1.61115052e-03,
                         8.11307023e-03, -1.50320445e-03, 3.88784244e-03,
                         8.90526147e-04, -1.82247430e-02, -2.20504426e-03,
                         1.40832443e-03, 9.33740535e-03, 9.47203922e-04,
                         8.52050841e-03, -1.17598753e-01, -8.78162533e+00,
                         9.50472210e-01, -1.17928509e+00, 5.54555380e-03,
                         8.00978042e-02, 4.55583337e+00, -2.16281811e-01,
                         5.85004643e+00, 4.71205108e-02, -5.61662504e-01,
                         -2.79618456e-02, 1.83678614e+00, 7.95037559e-03,
                         1.15679235e+00, -6.14398436e-01, -4.26728248e+00,
                         1.20858440e-01, -1.42211623e-01, 2.28758077e+00,
                         1.90969724e+00, 2.93574716e+00])

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

import numpy as np


class RangeChecker(object):
    def __init__(self, _sim):
        self.sim = _sim

    @property
    def skel(self):
        return self.sim.skel

    @property
    def prob(self):
        return self.sim.prob

    def check_all(self):
        for i in range(10):
            self.set_random_pose()

    def set_random_pose(self):
        param_desc = [(0, 'l_shoulder', 1.0),
                      (0, 'r_shoulder', 1.0),
                      (1, 'l_hand', 1.0),
                      (1, 'r_hand', 1.0),
                      (2, 'l_thigh', 1.0),
                      (3, 'r_thigh', 1.0),
                      (4, 'l_shin', 0.5),
                      (5, 'r_shin', 0.5),
                      (6, 'l_heel', 0.05),
                      (7, 'r_heel', 0.05) ]
        dim = max([i for i, dof, w in param_desc]) + 1
        lo = np.array([-1.57] * dim)
        hi = np.array([1.57] * dim)
        x = lo + np.random.rand(dim) * (hi - lo)
        q = self.skel.q
        for x_i, dof_name, _ in param_desc:
            dof_i = self.skel.dof_index(dof_name)
            q[dof_i] = x[x_i]
        # Validity check?
        self.skel.q = q

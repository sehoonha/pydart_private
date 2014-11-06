from poses import BioloidGPPoses, AtlasPoses
import numpy as np
from numpy.linalg import norm
from scipy.optimize import minimize


class Configure(object):
    def __init__(self, _sim):
        self.sim = _sim
        self.ext_force_steps = 200
        # == A set of configs ==
        # self.config('step', 1.5)
        # self.config('step', 5)
        # self.config('lean', 0.0)
        self.config('lean', 1.0)
        # self.config('skate', 10)
        # self.config('back', 3)
        # self.config('side', 10)
        # self.config('atlas_lean', 300)
        # self.config('atlas_lean', 300)
        # self.config('atlas_step', 3000)
        # self.config('atlas_back', 1000)

        self.conditions = None
        # self.conditions = self.generate()
        # self.cond_states = {}

    @property
    def skel(self):
        return self.sim.skel

    def config(self, class_name, force):
        print 'skeleton', self.skel.filename
        print 'config', class_name, force
        self.name = class_name
        # 1. Set the pose
        self.set_pose(class_name)
        # 2. Proceed the simulation
        if self.sim.is_bioloid():
            self.ext_force = ("torso", [0, 0, force], [0, 0, 0.03])
        else:
            self.ext_force = ("mtorso", [0, 0, force], [0, 0, 0.4])

    def set_pose(self, class_name):
        if class_name == 'step':
            self.init_state = BioloidGPPoses().step()
        elif class_name == 'lean':
            self.init_state = BioloidGPPoses().lean()
        elif class_name == 'skate':
            self.init_state = BioloidGPPoses().skate()
        elif class_name == 'back':
            self.init_state = BioloidGPPoses().back()
        elif class_name == 'side':
            self.init_state = BioloidGPPoses().side()
        elif class_name == 'atlas_lean':
            self.init_state = AtlasPoses().lean()
        elif class_name == 'atlas_step':
            self.init_state = AtlasPoses().step()
        elif class_name == 'atlas_back':
            self.init_state = AtlasPoses().back()
        else:
            print 'Invalid class_name:', class_name
            return
        self.skel.x = self.init_state

    def optimize(self, x):
        q = self.skel.q
        q[3:6] = x
        self.skel.q = q
        v = norm(self.skel.C - np.array([0.0, 1.11, -0.15])) ** 2
        print x, v
        return v

    def init_pose(self):
        return self.init_state[:self.skel.ndofs]

    def reset_simulation(self, sim):
        skel = sim.skel
        world = sim.world
        # Reset Pydart
        # self.init_state = AtlasPoses().back()
        skel.x = self.init_state
        # minimize(self.optimize, np.array([0, 0, 0]))

        # During the reset, we use the initial pose as a target pose
        saved_target = self.sim.tip_controller.pd.target
        self.sim.tip_controller.pd.target = skel.q

        for i in range(self.ext_force_steps):
            (b, f, p) = self.ext_force
            body = skel.body(b)
            body.add_ext_force_at(f, p)
            skel.tau = self.sim.tip_controller.control()
            world.step()
        self.sim.tip_controller.pd.target = saved_target

        # Reset the sim except the state
        state_after_pushed = skel.x
        world.reset()
        skel.x = state_after_pushed

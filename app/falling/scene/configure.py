from poses import BioloidGPPoses, AtlasPoses
import numpy as np
from numpy.linalg import norm
from scipy.optimize import minimize


class Configure(object):
    def __init__(self, _sim):
        self.sim = _sim
        # == A set of configs ==
        # self.config('step', 0.5, 200, -1.2)
        # self.config('step', 1.5, 200, -1.5)
        # self.config('step', 5, 200)
        # self.config('step', 6, 200)
        # self.config('step', 8, 200)
        # self.config('lean', 0.0)
        # self.config('lean', 1.0)
        # self.config('skate', 10)
        # self.config('back', 3)
        # self.config('side', 10)
        # self.config('atlas_lean', 1000.0, 200, px=-0.3)
        # self.config('atlas_lean', 2000.0, 200, px=-0.3)
        # self.config('atlas_lean', 300)
        # self.config('atlas_step', 1000.0, 200, -30.0)
        self.config('atlas_step', 2500.0, 200, 0)
        # self.config('atlas_back', 500, 200, px=0.2, py=0.0)
        # self.config('atlas_back', 600, 100, px=0.2, py=0.0)
        # self.config('atlas_back', 1000, 100, px=0.2, py=0.0)
        # self.config('atlas_back', 1600, 200, px=0.2, py=0.0, pz=-0.2)

        self.conditions = None
        # self.conditions = self.generate()
        # self.cond_states = {}

    @property
    def skel(self):
        return self.sim.skel

    def config(self, class_name, force, force_steps, side_force=0.0,
               px=0.0, py=0.0, pz=0.0):
        print 'skeleton', self.skel.filename
        print 'config', class_name, force, force_steps
        self.ext_force_steps = force_steps
        self.name = class_name
        # 1. Set the pose
        self.set_pose(class_name)
        # 2. Proceed the simulation
        if self.sim.is_bioloid():
            self.ext_force = ("torso", [side_force, 0, force], [0, 0, 0.03])
        else:
            self.ext_force = ("mtorso", [side_force, 0, force],
                              [px, py, 0.4 + pz])

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

    def target_point(self):
        (b, f, p) = self.ext_force
        body = self.skel.body(b)
        T = body.T
        pt = np.array(p + [1.0])
        return (T.dot(pt))[:3]

    def force(self):
        (b, f, p) = self.ext_force
        return np.array(f)

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
        self.saved_target_point = self.target_point()

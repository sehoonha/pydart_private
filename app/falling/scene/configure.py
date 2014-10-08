from poses import BioloidGPPoses


class Configure(object):
    def __init__(self, _sim):
        self.sim = _sim
        self.ext_force_steps = 100
        # == A set of configs ==
        # self.config('step', 15)
        # self.config('lean', 10)
        # self.config('skate', 10)
        # self.config('back', 3)
        self.config('side', 10)

        self.conditions = None
        # self.conditions = self.generate()
        # self.cond_states = {}

    @property
    def skel(self):
        return self.sim.skel

    def config(self, class_name, force):
        print 'config', class_name, force
        self.name = class_name
        # 1. Set the pose
        self.set_pose(class_name)
        # 2. Proceed the simulation
        self.ext_force = ("torso", [0, 0, force], [0, 0, 0.03])

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
        else:
            print 'Invalid class_name:', class_name
            return
        self.skel.x = self.init_state

    def reset_simulation(self, sim):
        skel = sim.skel
        world = sim.world
        # Reset Pydart
        self.init_state = BioloidGPPoses().side()
        skel.x = self.init_state
        for i in range(self.ext_force_steps):
            (b, f, p) = self.ext_force
            body = skel.body(b)
            body.add_ext_force_at(f, p)
            skel.tau = self.sim.tip_controller.control()
            world.step()

        # Reset the sim except the state
        state_after_pushed = skel.x
        world.reset()
        skel.x = state_after_pushed

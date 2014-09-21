from poses import BioloidGPPoses
import cond_to_state


class Config(object):
    def __init__(self, _sim):
        self.sim = _sim
        self.config('step', 1, 0)
        self.conditions = None
        # self.conditions = self.generate()
        # self.cond_states = {}

    @property
    def skel(self):
        return self.sim.skel

    def generate(self):
        # for class_name in ["forward", "backward", "step", "skate", "side"]:
        ret = []
        for class_name in ["step"]:
            for height_level in range(2):
                for push_level in range(2):
                    ret += [(class_name, height_level, push_level)]
        return ret

    def config_cached(self, class_name, height_level, push_level):
        key = (class_name, height_level, push_level)
        state = cond_to_state.cond_to_state[key]
        self.init_state = state
        self.ext_force = None
        return state

    def config(self, class_name, height_level, push_level):
        return self.config_cached(class_name, height_level, push_level)

        print 'config', class_name, height_level, push_level
        # 1. Set the pose
        self.set_pose(class_name, height_level)
        # 2. Proceed the simulation
        if push_level == 0:
            self.ext_force = ("torso", [0, 0, 10], [0, 0, 0.03])
        else:
            self.ext_force = ("torso", [0, 0, 50], [0, 0, 0.03])

    def set_pose(self, class_name, height_level):
        if class_name == 'step':
            if height_level == 0:  # C_y = 0.19
                self.init_state = BioloidGPPoses().step_low()
            elif height_level == 1:  # C_y = 0.17
                self.init_state = BioloidGPPoses().step()
        else:
            print 'Invalid class_name:', class_name
            return
        self.skel.x = self.init_state

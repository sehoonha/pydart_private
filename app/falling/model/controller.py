import numpy as np
import pd


class Controller(object):
    def __init__(self, _skel, _prob, _plan=None):
        self.skel = _skel
        self.prob = _prob

        self.tip_index = 0
        self.targets = [self.skel.q]
        if _plan is None:
            self.__init_default()
        else:
            self.__init_plan(_plan)
        # PD Control
        if self.skel.m < 10.0:
            self.pd = pd.PDController(self.skel, 60.0, 1.0, 0.5 * 1.5)
        else:
            self.pd = pd.PDController(self.skel, 600.0, 20.0, 500.0)
        self.update_target()

    def __init_default(self):
        self.tips = [self.prob.tips[0]]

    def __init_plan(self, _plan):
        self.plan = _plan
        n = self.plan.n
        self.tips = []
        for i in range(n):
            c1 = self.plan.contact1(i)
            c2 = self.plan.contact2(i)
            print 'c1, c2', c1, c2,
            e = self.prob.next_e[c1][c2]
            print 'e = ', e,
            self.tips += [self.prob.tips[e]]
            print 'tip = ', self.prob.tips[e]
        print '#', n, 'tips are initialized'

    def tip(self):
        if self.tip_index >= len(self.tips):
            return self.tips[-1]
        return self.tips[self.tip_index]

    def pivots(self):
        pivot_nodes = []
        n = min(self.tip_index + 1, len(self.tips))
        for i in range(n):
            pivot_nodes += self.tips[i].pivot_nodes()
        return set(pivot_nodes)

    def control(self):
        tau = np.zeros(self.skel.ndofs)
        tau += self.pd.control()
        tau[0:6] = 0
        return tau

    def update_target(self):
        if self.tip_index < len(self.targets):
            # print 'update_target OK', self.tip_index, len(self.targets)
            self.pd.target = self.targets[self.tip_index]
        else:
            # print 'update_target NG', self.tip_index, len(self.targets)
            pass

    def reset(self):
        self.tip_index = 0
        self.update_target()

    def check_next(self):  # Proceed to the next step
        contacts = self.skel.contacted_body_names()
        return (set(contacts) - self.pivots()) and \
            (self.tip_index < len(self.tips))

    def proceed(self):
        self.tip_index += 1
        self.update_target()

    def is_terminated(self):
        return (self.tip_index >= len(self.tips))

    def push(self, history):
        data = history.histories[-1]
        data['tip_index'] = self.tip_index
        data['tip_pivot_nodes'] = self.pivots()
        self.tip().push(history)

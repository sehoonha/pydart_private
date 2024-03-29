import numpy as np
import pd
from copy import deepcopy


class Controller(object):
    def __init__(self, _skel, _prob, _plan=None):
        self.skel = _skel
        self.prob = _prob

        self.tip_index = 0
        self.target_index = 0
        self.targets = [self.skel.q]
        if _plan is None:
            self.__init_default()
        else:
            self.__init_plan(_plan)
        # PD Control
        if self.skel.m < 10.0:
            self.pd = pd.PDController(self.skel, 60.0, 1.0, 0.5)
            # self.pd = pd.PDController(self.skel, 120.0, 1.0, 1.0)
        else:
            # self.pd = pd.PDController(self.skel, 600.0, 60.0, 0.3)
            self.pd = pd.AtlasPDController(self.skel)
            # self.pd = pd.PDController(self.skel, 600.0, 1.0, 0.1 * 500.0)
        self.update_target()

    def __init_default(self):
        self.tips = [self.prob.tips[0]]
        self.executed_path = []
        self.plan = None

    def __init_plan(self, _plan):
        self.plan = _plan
        self.executed_plan = deepcopy(self.plan)
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

    def update_target_with_balance(self):
        Cd = self.skel.Cdot
        bal2 = Cd[0]

        x0 = self.skel.dof_index('l_hip')
        y0 = self.skel.dof_index('r_hip')
        x1 = self.skel.dof_index('l_foot')
        y1 = self.skel.dof_index('r_foot')

        x2 = self.skel.dof_index('l_arm')
        y2 = self.skel.dof_index('r_arm')
        x3 = self.skel.dof_index('l_hand')
        y3 = self.skel.dof_index('r_hand')

        x4 = self.skel.dof_index('l_shin')

        qhat = self.targets[-1]  # Pick the last pose
        power = 1.0
        bal2 *= power
        print 'bal2:', bal2
        # qhat[x0] += bal2 * 1.0
        # qhat[y0] += bal2 * 1.0
        # qhat[x1] += bal2 * 1.0
        # qhat[y1] += bal2 * 1.0
        # qhat[x4] -= 0.1
        # qhat[x2] = 1.0
        qhat[y2] = 0.5
        # qhat[x3] = -1.57
        # qhat[y3] = -1.57
        self.pd.target = qhat

    def update_target_with_balance2(self):
        Cd = self.skel.Cdot
        bal2 = Cd[0]

        x0 = self.skel.dof_index('l_hip')
        y0 = self.skel.dof_index('r_hip')
        x1 = self.skel.dof_index('l_foot')
        y1 = self.skel.dof_index('r_foot')

        x2 = self.skel.dof_index('l_arm')
        y2 = self.skel.dof_index('r_arm')
        x3 = self.skel.dof_index('l_hand')
        y3 = self.skel.dof_index('r_hand')

        x4 = self.skel.dof_index('l_shin')

        qhat = self.targets[-1]  # Pick the last pose
        power = 1.0
        bal2 *= power
        print 'bal2:', bal2
        # qhat[x0] += bal2 * 1.0
        # qhat[y0] += bal2 * 1.0
        # qhat[x1] += bal2 * 1.0
        # qhat[y1] += bal2 * 1.0
        # qhat[x4] -= 0.1
        # qhat[x2] = 1.0
        qhat[y2] = 0.5
        # qhat[x3] = -1.57
        # qhat[y3] = -1.57
        self.pd.target = qhat

    def reset(self):
        self.tip_index = 0
        self.update_target()

    def has_next(self):
        return (self.tip_index < len(self.tips))

    def check_next(self):  # Proceed to the next step
        if self.tip_index >= len(self.tips):
            # # Debug code for the supporting arm
            # self.pd.target[11] -= 0.001
            # self.pd.target[15] -= 0.4
            return False

        contacts = set(self.skel.contacted_body_names())
        if "pelvis" in contacts:
            contacts.remove("pelvis")
            contacts.add("ltorso")
        pivots = self.pivots()

        # Special case: l_heel to l_toe
        if 'l_foot' in self.tip().c1.bodynames and \
           'l_foot' in self.tip().c2.bodynames:
            lfoot_contacts = self.skel.body('l_foot').contacts()
            # print '!!!!', len(lfoot_contacts)
            return (len(lfoot_contacts) >= 5)

        # # Special case: r_heel to l_heel
        # if 'torso' in self.tip().c1.bodynames and \
        #    'r_foot' in self.tip().c2.bodynames:
        #     return 'r_foot' in contacts

        # Generally, proceed to next if there's new contacts
        # print contacts, pivots, len(contacts - pivots)
        return (contacts - pivots)

    def proceed(self, sim):
        self.update_executed_plan(sim)
        self.tip_index += 1
        self.update_target()

    def update_executed_plan(self, sim):
        """ Ugly codes... :) """
        if self.plan is None:
            return
        if not hasattr(self, 'executed_plan'):
            self.executed_plan = deepcopy(self.plan)

        tip = self.tip()
        print '>>', tip.th1(), tip.dth1(), tip.r1(), tip.th2(), tip.r2()
        if self.tip_index > len(self.executed_plan.path):
            print '>>', 'failed_to_update'
            return
        entry = self.executed_plan.path[self.tip_index]
        new_nx_0 = entry.nx_0._replace(th1=tip.th1(), dth1=tip.dth1(),
                                       r1=tip.r1())
        new_u = entry.u._replace(th2=tip.th2(), r2=tip.r2())
        data = sim.history.get_frame()
        new_j = data['impulse']
        new_j_max = data['max_impulse']
        new_entry = entry._replace(nx_0=new_nx_0, u=new_u,
                                   v=new_j, v_max=new_j_max)
        self.executed_plan.path[self.tip_index] = new_entry
        self.executed_plan.initialize()
        # print self.executed_plan.path[self.tip_index]

    def is_terminated(self):
        return (self.tip_index >= len(self.tips))

    def push(self, history):
        data = history.histories[-1]
        data['tip_index'] = self.tip_index
        data['tip_pivot_nodes'] = self.pivots()
        self.tip().push(history)

    def prev_target(self):
        n = len(self.targets)
        self.target_index = (self.target_index - 1) % n
        self.skel.q = self.targets[self.target_index]
        print 'tip_controller.prev_target:', self.target_index

    def next_target(self):
        n = len(self.targets)
        self.target_index = (self.target_index + 1) % n
        self.skel.q = self.targets[self.target_index]
        print 'tip_controller.next_target:', self.target_index

    def target_as_list(self):
        return [tar.tolist() for tar in self.targets]

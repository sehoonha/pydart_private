import sys
import config
sys.path.append(config.PYDART_PATH)
import os
# import time
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)
import cPickle as pickle
# import jsonpickle

import pydart
import numpy as np
from OpenGL.GL import glPushMatrix, glPopMatrix, glColor, glMultMatrixf
import OpenGL.GL as gl
import gltools

from history import History
from impulse_live_renderer import ImpulseLiveRenderer
import events
import ik
# from ik.ik_multi import IKMulti
# from ik.ik_jac import IKJac
import scene.configure
import scene.range_checker
import problem
import abstract.tip
import abstract.tip_v2
import abstract.twotip
import abstract.dynamic
import abstract.plan
import model.controller
# import cProfile
import plotly.plotly as py
import plotly.graph_objs as pyg
import matplotlib.pyplot as plt
import gp


class Simulation(object):
    def __init__(self):
        self.show_impulse = False
        # self.name = 'GP_--N_handfree_naive_clean'
        # self.name = 'Atlas_##_--N_naive_clean'
        self.name = 'Atlas_##--N'
        # self.name = 'BioloidGP_--N'
        # self.name = 'BioloidGP_--N_from_one_foot'

        # Init api
        pydart.init()
        self.world = pydart.create_world(1.0 / 2000.0)
        self.world.add_skeleton(config.DATA_PATH + "sdf/ground.urdf",
                                control=False)
        if 'GP' in self.name:
            self.world.add_skeleton(config.DATA_PATH +
                                    "urdf/BioloidGP/BioloidGP.URDF")
            self.world.skel.set_joint_damping(0.15)
        # self.world.add_skeleton(config.DATA_PATH +
        #                         "urdf/atlas/atlas_v3_no_head.urdf")
        else:
            self.world.add_skeleton(config.DATA_PATH +
                                    "urdf/atlas/atlas_v3_no_head.urdf")
            self.world.skel.set_joint_damping(0.15)

        self.skel = self.world.skel  # shortcut for the control skeleton

        for i, dof in enumerate(self.skel.dofs):
            print i, dof, 'efforts:', self.skel.tau_lo[i], self.skel.tau_hi[i]

        # Configure the scene
        self.cfg = scene.configure.Configure(self)
        self.name = self.name.replace('--', '%.1f' % self.cfg.f_mag)
        if '##' in self.name and self.cfg.tag is not None:
            self.name = self.name.replace('##', self.cfg.tag)

        self.prob = problem.Problem(self, self.cfg.name)
        self.history = History(self)
        self.impulse_live_renderer = ImpulseLiveRenderer(self)

        # # ### Now, configure the controllers
        self.tip_controller = model.controller.Controller(self.skel, self.prob)
        self.tip_controller.pd.set_pd_params(self.name)
        self.event_handler = events.Handler()

        # Reset to the initial state
        self.reset()

        print 'skel.m = ', self.skel.m
        print 'skel.approx_inertia_x = ', self.skel.approx_inertia_x()
        print 'skel.q = ', self.skel.q

        self.rc = scene.range_checker.RangeChecker(self)
        self.rc.check_all()

        # Abstract model
        self.abstract_tip = abstract.dynamic.DynamicTIP(self.prob, self.rc)

        # For handle callbacks properly..
        self.history.clear()
        self.history.push()

    def is_bioloid(self):
        return ("BioloidGP" in self.skel.filename)

    @property
    def tip(self):
        return self.tip_controller.tip()

    def do_plan(self):
        # return self.do_ik()  # Just testing IK

        # Plan with Dynamic TIP
        self.abstract_tip.set_x0(self.tip_controller.tips)
        self.abstract_tip.plan_initial()
        # cProfile.runctx('self.abstract_tip.plan_initial()',
        #                 globals(), locals())
        x0 = self.abstract_tip.x0
        path = self.abstract_tip.path

        print 'self.x0 = ', str(x0)
        print 'self.path = ', str(path)
        # print 'sleep 5 seconds'
        # time.sleep(5)
        self.plan = abstract.plan.Plan(x0, path)
        self.plan.names = self.prob.contact_names()
        print 'new plan is generated'
        self.tip_controller = model.controller.Controller(self.skel,
                                                          self.prob,
                                                          self.plan)
        self.tip_controller.pd.set_pd_params(self.name)
        print 'new tip controller is generated'
        self.do_ik()
        self.name = self.name.replace('_naive', '')

    def do_ik(self):
        # self.plan.plot()

        # self.ik = IKMulti(self, self.plan)
        self.ik = ik.IKJac(self, self.plan)
        self.ik.optimize(restore=False)
        self.tip_controller.targets = self.ik.targets

    def do_ik_curr(self):
        curr = self.tip_controller.target_index
        print 'IK_curr:', curr
        # self.plan.plot()

        # # self.ik = IKMulti(self, self.plan)
        targets = self.tip_controller.targets
        self.ik = ik.IKJac(self, self.plan, curr, targets)
        self.ik.optimize(restore=False)
        self.tip_controller.targets = self.ik.targets

    def reset(self):
        self.cfg.reset_simulation(self)

        # Reset inner structures
        self.tip_controller.reset()
        self.event_handler.clear()
        self.history.clear()
        self.history.push()
        self.history.callbacks = [self.tip_controller]

        self.terminated = dict()

    def terminate_time(self):
        name = self.name
        if 'GP' in name:
            if '0.5' in name:
                return 1.25
            elif '1.5' in name:
                return 1.3
        else:
            if 'Lean_1000' in name:
                return 1.3
            elif 'Step_2500' in name:
                return 1.3
            elif 'Back' in name:
                return 1.35
        return 1.0

    def step(self):
        # if 'GP_0.5N' in self.name:
        #     if not self.tip_controller.has_next():  # For GP 0.5N
        #         self.tip_controller.update_target_with_balance()
        self.skel.tau = self.tip_controller.control()
        self.world.step()
        self.history.push()

        if self.tip_controller.check_next():
            self.event_handler.push("proceed", 40)

        for e in self.event_handler.pop():
            # print 'New Event: ', e.name, 'at', self.world.nframes
            if e.name == "proceed":
                self.tip_controller.proceed(self)
                # print 'proceed:', self.tip_controller.is_terminated()
                if self.tip_controller.is_terminated():
                    self.event_handler.push("terminate", 0)
            elif e.name == 'pause':
                return True
            elif e.name == 'terminate':
                self.event_handler.push("terminate", 9999)
                return True

        self.event_handler.step()
        # print self.world.nframes, ':', self.event_handler
        # return (self.terminate_time() <= self.world.t + 0.001)
        return False

    def render(self):
        glPushMatrix()
        # gltools.render_axis(10)
        # Draw chess board
        gltools.glMove([0.0, -0.01, 0.0])
        # gltools.render_chessboard(10, 20.0)
        gltools.render_floor(20, 40.0)

        # Draw skeleton
        gltools.glMove([0, 0, 0])
        glPushMatrix()
        M_s = [1.0, 0.0, 0.0, 0.0,
               1.0, 0.0, -1.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, -0.001, 0.0, 1.0]
        glMultMatrixf(M_s)
        glColor(0.0, 0.0, 0.0, 1.0)
        gl.glEnable(gl.GL_LIGHTING)
        self.skel.render_with_color(0.0, 0.0, 0.0)
        gl.glEnable(gl.GL_LIGHTING)
        glPopMatrix()

        gltools.glMove([0, 0, 0])
        self.skel.render()

        gltools.glMove([0, 0, 0])
        glColor(1, 0, 0)
        # gltools.render_arrow(self.skel.C,
        #                      self.skel.C + 0.2 * self.tip.projected_Cdot())

        # # Draw TIP
        # tip_index = self.history.get_frame()['tip_index']
        # tips = self.tip_controller.tips
        # for i in range(tip_index, len(tips)):
        #     tips[i].render()

        # for c in self.prob.contacts:
        #     c.render()

        # Draw contacts
        gltools.glMove([0, 0, 0])
        # glColor(0.7, 0.0, 0.3)
        # for c in self.history.get_frame()['contacts']:
        #     gltools.render_arrow(c[0:3], c[0:3] - 0.001 * c[3:6])

        if self.history.get_frame()['t'] < 10.0:
            gltools.glMove([0, 0, 0])
            q = self.cfg.saved_target_point
            d = self.cfg.force()
            d[0] = 0.0
            len_d = np.linalg.norm(d)
            d /= len_d
            l = len_d * 0.025 + 0.020

            # d[0] = 0.0
            # len_d = np.linalg.norm(d)
            # d /= len_d
            # len_d /= 60.0
            # l = len_d * 0.025

            p = q - l * d
            # p[0] = q[0]
            # gltools.render_arrow2([0, 0.4, 0], [0, 0.4, 0.4])
            rb = 0.01 * (len_d * 0.025 + 1.0)
            hw = 0.015 * (len_d * 0.05 + 1.0)
            hl = 0.015 * (len_d * 0.05 + 1.0)
            rb *= 0.5
            hw *= 0.5
            # rb *= 2.0
            # hw *= 2.0
            gltools.render_arrow2(p, q, rb, hw, hl)

        glPopMatrix()

    def status_string(self):
        data = self.history.get_frame()

        status = ""
        status += "T = %.4f (%d) " % (data['t'], data['nframes'])
        status += "C = (%.4f %.4f) " % (data['C.x'], data['C.y'])
        status += "P = (%.4f %.4f) " % (data['P.x'], data['P.y'])
        status += "Impulse = %.4f (max %.4f) " % (data['impulse'],
                                                  data['max_impulse'])
        status += "I = %.4f " % self.skel.approx_inertia_x()
        status += "TIP = " + str(data['tip']) + " "
        # tip = self.tip_controller.tip()
        # status += "TIP = " + str(tip)
        status += "Contacted = %s " % str(data['contactedBodies'])

        return status

    def __len__(self):
        return max(self.world.nframes, len(self.history))

    def set_world_frame(self, i):
        if i < self.world.nframes:
            self.world.set_frame(i)
        elif i < len(self.history):
            data = self.history.get_frame_at(i)
            self.skel.x = data['x']
        else:
            print 'set_world_frame: invalid index', i
        # pydart_api.setWorldSimFrame(i)
        self.history.pop(i)

    def save(self, filename):
        protocol = pickle.HIGHEST_PROTOCOL
        # protocol = 0
        with open(filename, 'w+') as fp:
            pickle.dump(self.plan, fp, protocol)
            pickle.dump(self.tip_controller.targets, fp, protocol)

    def export_motion(self):
        # Load motor map data
        mm = gp.MotorMap()
        mm.load(config.DATA_PATH +
                "urdf/BioloidGP/BioloidGPMotorMap.xml")
        # Translate plan into .mtn
        m = gp.Motion(mm)
        transition_times = [e.nx_0.t for e in self.plan.path]
        durations = np.diff([0.0] + transition_times)
        poses = self.tip_controller.targets
        m.add_page('Init', [self.cfg.init_pose()], [1.0])
        m.add_page('Falling', poses, durations)
        m.fill_with_empty_pages()
        # m.save('test.mtn')
        filename = '%s.mtn' % self.name
        m.save(filename)
        print 'export_motion OK:', filename

    def load(self, filename):
        self.name = self.name.replace('_naive', '')
        with open(filename, 'r') as fp:
            self.plan = pickle.load(fp)
            self.tip_controller = model.controller.Controller(self.skel,
                                                              self.prob,
                                                              self.plan)
            self.tip_controller.pd.set_pd_params(self.name)
            targets = pickle.load(fp)
            self.tip_controller.targets = targets

    def save_motion(self, filename):
        protocol = pickle.HIGHEST_PROTOCOL
        # protocol = 0
        with open(filename, 'w+') as fp:
            pickle.dump(self.history, fp, protocol)
        print 'save #', len(self.history), 'frames'

    def load_motion(self, filename):
        with open(filename, 'r') as fp:
            self.history = pickle.load(fp)
        print 'load #', len(self.history), 'frames'

    def plot_com(self):
        # mycolors = ['r', 'b', 'g', 'k', 'c']

        traces = []
        colors = []
        styles = []

        traces += self.history.com_traces()
        n = len(traces)
        # colors += mycolors[:n]
        colors += ['r'] * n
        styles += ['-'] * n
        if hasattr(self, 'plan') and self.plan is not None:
            traces += self.plan.com_traces()
            m = len(traces) - n
            # colors += mycolors[:n]
            colors += ['b'] * m
            styles += ['-'] * m

        # Ploting with matplotlib
        plt.ioff()
        fig = plt.figure()
        fig.set_size_inches(18.5, 10.5)
        pp = []
        for trace, c, ls in zip(traces, colors, styles):
            (x, y, text) = trace
            print 'plot'
            print 'x:', x
            print 'y:', y
            p = plt.plot(x, y, ls=ls, color=c, linewidth=2)
            pp.append(p[0])
        # plt.title('Compare %d Trials on %s' % (num_trials, prob_name),
        name = self.name.replace('_', ' ')
        name = name.replace('Lean', '')
        t = plt.title('%s' % name,
                      fontdict={'size': 40})
        t.set_y(0.92)
        font = {'size': 36}
        plt.xlabel('X', fontdict=font)
        plt.ylabel('Y', fontdict=font)
        pp = [pp[0]] + [None] * (n - 1) + [pp[n]] + [None] * (m - 1)
        legends = [None] * len(pp)
        legends[0] = 'Robot'
        legends[n] = 'Abstract Model'
        print 'legends:', legends
        plt.legend(pp, legends, fontsize=36,
                   # bbox_to_anchor=(0.15, 0.15))
                   loc='lower left')
                   # loc='lower right')

        # (lo, hi) = plt.axes().get_xlim()
        # plt.axes().set_xlim(0.0, 0.70)
        # plt.axes().set_xlim(0.0, 1.50)
        (lo, hi) = plt.axes().get_ylim()
        # plt.axes().set_ylim(0.0, hi + 0.1)
        # plt.axes().set_ylim(0.0, 0.2)  # For GP
        plt.axes().set_ylim(0.0, 1.2)  # For GP
        plt.tick_params(axis='x', labelsize=28)
        plt.tick_params(axis='y', labelsize=28)

        outputfile = '%s_com.png' % self.name
        plt.savefig(outputfile, bbox_inches='tight')
        plt.close(fig)
        plt.close(fig)

    def plot_impulse(self):
        traces = []
        traces += self.history.impulse_traces()
        colors = ['r']
        styles = ['-']
        max_t = max(traces[0][0])
        print 'max_t:', max_t
        if hasattr(self, 'plan') and self.plan is not None:
            traces += self.plan.impulse_traces(max_t=max_t)
            colors += ['b']
            styles += ['-']
        # Ploting with matplotlib
        plt.ioff()
        fig = plt.figure()
        fig.set_size_inches(18.5, 10.5)
        pp = []
        for trace, c, ls in zip(traces, colors, styles):
            (x, y) = trace
            print 'plot'
            print 'x:', x
            print 'y:', y
            p = plt.plot(x, y, ls=ls, color=c, linewidth=2)
            pp.append(p[0])
        # plt.title('Compare %d Trials on %s' % (num_trials, prob_name),
        name = self.name.replace('_', ' ')
        t = plt.title('%s' % name,
                      fontdict={'size': 32})
        t.set_y(0.92)
        font = {'size': 28}
        plt.xlabel('X', fontdict=font)
        plt.ylabel('Y', fontdict=font)
        legends = [None] * len(pp)
        legends[0] = 'Robot'
        legends[1] = 'Abstract Model'
        plt.legend(pp, legends, fontsize=26,
                   loc='upper left')

        # (lo, hi) = plt.axes().get_xlim()
        # plt.axes().set_xlim(0.0, 0.36)  # Walking
        # (lo, hi) = plt.axes().get_ylim()
        # plt.axes().set_ylim(0.0, 0.24)  # Walking

        outputfile = '%s_impulse.png' % self.name
        plt.savefig(outputfile, bbox_inches='tight')
        plt.close(fig)

    def plot_accel(self):
        t_data, y_data = [], []
        # filename = 'OneFootStance.csv'
        # filename = 'TwoFeetStance.csv'
        # filename = 'Rolling.csv'
        # filename = 'GP00_new_2.csv'
        # filename = 'GP01.csv'
        # filename = 'test.csv'
        # filename = 'test2.csv'
        # filename = 'yogamatt00.csv'
        filename = 'yogamatt01.csv'
        # filename = ['yogamatt00.csv', 'yogamatt01.csv']
        if isinstance(filename, str):
            with open(filename) as fin:
                for line in fin.readlines():
                    values = [float(x.strip()) for x in line.split(',')]
                    t_data += [values[0]]
                    accel = np.array([values[1], values[2], values[3]])
                    y_data += [np.linalg.norm(accel)]
                    # print values, t_data[-1], y_data[-1]
        else:
            offset = 0.0
            for f in filename:
                with open(f) as fin:
                    for line in fin.readlines():
                        values = [float(x.strip()) for x in line.split(',')]
                        t_data += [values[0] + offset]
                        accel = np.array([values[1], values[2], values[3]])
                        y_data += [np.linalg.norm(accel)]
                offset += max(t_data) + 10.0

        max_t = max(t_data)
        times = [(0, max_t)]
        colors = ['r']
        legends = ['All']
        interactive = True
        styles = None
        if 'TwoFeet' in filename:
            times = [(135.0, 139.0), (216.4, 220.4)]
            colors = ['r', 'b']
            legends = ['Planned', 'Unplanned']
            interactive = False
        elif 'OneFoot' in filename:
            times = [(81.0, 85.0), (122.0, 126.0)]  # 4.5 6.9
            colors = ['r', 'b']
            legends = ['Planned', 'Unplanned']
            interactive = False
        elif 'Rolling' in filename:
            times = [(154.5, 158.5), (197.5, 201.5)]  # 5.23 13.36
            colors = ['r', 'b']
            legends = ['Planned', 'Unplanned']
            interactive = False
        elif 'GP00_new.' in filename:
            times = [(119.5, 123.5), (235.0, 239.0)]  # 8.8 12.1
            colors = ['r', 'b']
            legends = ['Planned', 'Unplanned']
            interactive = False
        elif 'yogamatt00' in filename:
            times = [(152.9, 155.9), (223.7, 226.7)]  # 8.8 12.1
            colors = ['r', 'b']
            legends = ['Planned', 'Unplanned']
            interactive = False
        elif 'yogamatt01' in filename:
            x = 360.2396 + 10.0
            times = [(466.1 - x, 469.1 - x), (557.51 - x, 560.51 - x)]
            colors = ['r', 'b']
            legends = ['Planned', 'Unplanned']
            interactive = False
        # elif 'yogamatt00.csv' in filename:
        #     times = [(153.5, 156.5), (224.5, 227.5),  # 5.49 7.98
        #              (467.0, 470.0), (558.5, 561.5)]  # 5.50 8.04
        #     colors = ['r', 'r', 'b', 'b']
        #     styles = ['-', '--', '-', '--']
        #     legends = ['0.0N (Ours)', '0.0N (Baseline)',
        #                '0.5 (Ours)', '0.5 (Baseline)']
        #     interactive = False
        if styles is None:
            styles = ['-'] * len(colors)

        traces = []
        for lo, hi in times:
            trace_x = []
            trace_y = []
            for t, y in zip(t_data, y_data):
                if lo <= t and t <= hi:
                    trace_x += [t - lo]
                    trace_y += [y]
            traces += [(trace_x, trace_y)]

        if not interactive:
            plt.ioff()
        fig = plt.figure()
        fig.set_size_inches(18.5, 10.5)
        pp = []
        for trace, c, ls in zip(traces, colors, styles):
            (x, y) = trace
            print 'plot', max(y)
            # print 'x:', x
            # print 'y:', y
            p = plt.plot(x, y, color=c, ls=ls, linewidth=2)
            pp.append(p[0])
        t = plt.title('BioloidGP 0.5N',
                      fontdict={'size': 36})
        t.set_x(0.5)
        t.set_y(0.92)
        font = {'size': 28}
        plt.xlabel('Time', fontdict=font)
        plt.ylabel('Acceleration of Head', fontdict=font)
        plt.tick_params(axis='x', labelsize=28)
        plt.tick_params(axis='y', labelsize=28)
        (lo, hi) = plt.axes().get_ylim()
        plt.axes().set_ylim(0.0, 10.0)
        plt.legend(pp, legends, fontsize=26,
                   loc='upper right')

        if not interactive:
            if isinstance(filename, str):
                outputfile = '%s_impulse.png' % (filename.replace('.csv', ''))
            else:
                outputfile = '%s_all_impulse.png' \
                             % (filename[0].replace('.csv', ''))
            plt.savefig(outputfile, bbox_inches='tight')
            plt.close(fig)

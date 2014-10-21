import math
import numpy as np
from numpy.linalg import norm

import scipy.optimize


class ObjPt(object):
    def __init__(self, _con, _target):
        self.con = _con
        self.target = _target

    @property
    def P(self):
        return self.con.p

    def cost(self):
        diff = self.con.p - self.target
        return 0.5 * diff.dot(diff)

    def gradient(self):
        diff = self.con.p - self.target
        J = self.con.world_derivative()
        return diff.dot(J)


class IKJac(object):
    def __init__(self, _sim, _plan):
        self.sim = _sim
        self.plan = _plan
        self.objs = []
        self.objs += [ObjPt(self.prob.contact(2),
                            np.array([0.0, 0.3, 0.3]))]

    @property
    def prob(self):
        return self.sim.prob

    @property
    def skel(self):
        return self.sim.skel

    def cost(self, x):
        self.skel.q = x
        costs = [o.cost() for o in self.objs]
        return sum(costs)

    def gradient(self, x):
        self.skel.q = x
        gradients = [o.gradient() for o in self.objs]
        return sum(gradients)

    def optimize(self, restore=True):
        self.x0 = self.skel.x
        self.q0 = self.skel.q

        self.check_gradient()

        print "==== ik.IKJac optimize...."
        self.res = scipy.optimize.minimize(self.cost, self.q0,
                                           jac=self.gradient,
                                           method='SLSQP')
        print "==== result"
        print self.res
        print "==== ik.IKJac optimize....OK"

        if restore:
            self.skel.x = self.x0
        self.targets = [self.skel.q]
        return self.targets

    def check_gradient(self):
        np.set_printoptions(formatter={'float': '{: 0.6f}'.format})
        dim = len(self.q0)
        errors = []
        for i in range(5):
            x = (np.random.rand(dim) - 0.5) * 2.0
            g0 = self.gradient(x)
            g1 = scipy.optimize.approx_fprime(x, self.cost, 1e-6)
            g2 = scipy.optimize.approx_fprime(x, self.cost, 1e-9)
            err = scipy.optimize.check_grad(self.cost, self.gradient, x)
            errors += [err]
            print '---'
            print g0
            print g1
            print g2
            print err
            print
        print 'average error = ', sum(errors) / len(errors)

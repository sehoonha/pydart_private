from collections import namedtuple
from model.contact import Contact
from model.tip import TIP


class Range(namedtuple('Range', ['lo', 'hi'])):
    __slots__ = ()

    def is_in(self, x):
        return (self.lo <= x) and (x <= self.hi)

    def inter(self, rhs):
        lhs = self
        return Range(lo=max(lhs.lo, rhs.lo), hi=min(lhs.hi, rhs.hi))


class Problem(object):
    def __init__(self, _sim, _name):
        self.sim = _sim
        self.name = _name
        print '== start to define the problem =='
        self.define_contacts()
        self.define_edges()
        print '== end define the problem =='

    @property
    def skel(self):
        return self.sim.skel

    @property
    def n(self):
        return len(self.vertices)

    @property
    def m(self):
        return len(self.edges)

    def define_contacts(self):
        # Start to test more contact candidates
        defs = None
        if self.name == 'step':
            defs = [
                # ("feet", ["l_foot", "r_foot"], [[-0.05, 0.025, 0]] * 2),
                ("r_toe", ["r_foot"], [[-0.05, 0.025, 0.0]]),
                ("l_heel", ["l_foot"], [[0.05, 0.025, 0.0]]),
                ("l_toe", ["l_foot"], [[-0.05, 0.025, 0.0]]),
                ("hands", ["l_hand", "r_hand"],
                 [[0, -0.11, 0.01], [0, 0.11, -0.01]]),
                ("knees", ["l_shin", "r_shin"], [[0, 0, 0], [0, 0, 0]]),
                ("head", ["torso"], [[0.0, 0.0, 0.03]]),
            ]
        elif self.name == 'lean':
            defs = [
                ("toes", ["l_foot", "r_foot"],
                 [[-0.05, 0.025, 0.0], [-0.05, 0.025, 0.0]]),
                ("knees", ["l_shin", "r_shin"], [[0, 0, 0], [0, 0, 0]]),
                ("hands", ["l_hand", "r_hand"],
                 [[0, -0.11, 0.01], [0, 0.11, -0.01]]),
                ("head", ["torso"], [[0.0, 0.0, 0.03]]),
            ]
        elif self.name == 'skate':
            defs = [
                ("l_toe", ["l_foot"], [[-0.05, 0.025, 0.0]]),
                ("hands", ["l_hand", "r_hand"],
                 [[0, -0.11, 0.01], [0, 0.11, -0.01]]),
                ("head", ["torso"], [[0.0, 0.0, 0.03]]),
                ("r_heel", ["r_foot"], [[0.05, 0.025, 0.0]]),
            ]
        elif self.name == 'back':
            defs = [
                ("heels", ["l_foot", "r_foot"],
                 [[0.05, 0.025, 0.0], [0.05, 0.025, 0.0]]),
                ("hands", ["l_hand", "r_hand"],
                 [[0, -0.11, 0.01], [0, 0.11, -0.01]]),
                ("elbows", ["l_hand", "r_hand"],
                 [[0, 0.04, 0.01], [0, -0.04, -0.01]]),
                ("head", ["torso"], [[0.0, 0.0, 0.03]]),
            ]
        elif self.name == 'side':
            defs = [
                ("r_foot", ["r_foot"], [[0.0, 0.025, 0.03]]),
                ("r_hand", ["r_hand"], [[0.02, 0.05, -0.01]]),
                ("r_shoulder", ["r_shoulder"], [[0.0, 0.0, 0.0]]),
                ("head", ["torso"], [[0.0, 0.0, 0.03]]),
            ]

        self.vertices = [i for i in range(len(defs))]
        self.contacts = [Contact(self.skel, i, n, b, p)
                         for i, (n, b, p) in enumerate(defs)]
        print
        print 'vertices:', self.vertices
        print 'contacts:', self.contacts

    def define_edges(self):
        def find_index(name):
            for i, c in enumerate(self.contacts):
                if c.name == name:
                    return i
            return None

        # defs = [("r_toe", "l_toe")]
        # defs = [("r_toe", "l_heel"),
        #         ("l_heel", "l_toe")]
        # defs = [("r_toe", "l_heel"),
        #         ("l_heel", "l_toe"),
        #         ("l_toe", "hands")]
        # defs = [("r_toe", "l_heel"),
        #         ("r_toe", "l_toe"),
        #         ("r_toe", "hands"),
        #         ("l_heel", "l_toe"),
        #         ("l_toe", "hands"), ]
        if self.name == 'step':
            defs = [("r_toe", "l_heel"),
                    ("r_toe", "l_toe"),
                    ("r_toe", "hands"),
                    ("r_toe", "head"),
                    ("l_heel", "l_toe"),
                    ("l_toe", "hands"),
                    ("l_toe", "head"),
                    ("hands", "head"), ]
        elif self.name == 'lean':
            defs = [("toes", "knees"),
                    ("toes", "hands"),
                    ("knees", "hands"),
                    ("knees", "head"),
                    ("hands", "head"), ]
        elif self.name == 'skate':
            defs = [("l_toe", "hands"),
                    ("hands", "head"),
                    ("head", "r_heel"), ]
        elif self.name == 'back':
            defs = [("heels", "hands"),
                    ("heels", "elbows"),
                    ("hands", "elbows"),
                    ("hands", "head"),
                    ("elbows", "head"), ]
        elif self.name == 'side':
            defs = [("r_foot", "r_hand"),
                    ("r_hand", "r_shoulder"),
                    ("r_shoulder", "head"), ]

        self.edges = [(find_index(a), find_index(b)) for a, b in defs]
        self.tips = [TIP(id, self.contacts[i], self.contacts[j])
                     for id, (i, j) in enumerate(self.edges)]
        print
        print 'edges:', self.edges
        print 'tips:', [str(t) for t in self.tips]

        self.next_v = []
        self.next_e = [[None] * self.n for _ in range(self.n)]
        for i in range(self.n):
            self.next_v.append([b for (a, b) in self.edges if a == i])
            print i, 'next_v:', self.next_v[i]

        for i, (a, b) in enumerate(self.edges):
            self.next_e[a][b] = i
        print 'edge table:'
        for row in self.next_e:
            print row

    def contact(self, _name):
        if isinstance(_name, int):
            index = _name
            return self.contacts[index]
        else:
            return next(x for x in self.contacts if x.name == _name)

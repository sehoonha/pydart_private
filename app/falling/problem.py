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
    def __init__(self, _sim):
        self.sim = _sim
        self.define_contacts()
        self.define_edges()

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
        defs = [
            # ("feet", ["l_foot", "r_foot"], [[-0.05, 0.025, 0]] * 2),
            ("hands", ["l_hand", "r_hand"],
             [[0, -0.11, 0.01], [0, 0.11, -0.01]]),
            # ("knees", ["l_shin", "r_shin"], [[0, 0, 0], [0, 0, 0]]),
            ("l_heel", ["l_foot"], [[0.05, 0.025, 0.0]]),
            ("l_toe", ["l_foot"], [[-0.05, 0.025, 0.0]]),
            ("r_toe", ["r_foot"], [[-0.05, 0.025, 0.0]]),
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

        defs = [("r_toe", "l_heel"),
                ("r_toe", "l_toe"),
                ("r_toe", "hands"),
                ("r_toe", "head"),
                ("l_heel", "l_toe"),
                ("l_toe", "hands"),
                ("r_toe", "head"),
                ("hands", "head"), ]

        self.edges = [(find_index(a), find_index(b)) for a, b in defs]
        self.tips = [TIP(id, self.contacts[i], self.contacts[j])
                     for id, (i, j) in enumerate(self.edges)]
        print
        print 'edges:', self.edges
        print 'tips:', [str(t) for t in self.tips]

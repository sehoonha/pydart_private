"""
- Rule for properties
1. Shortcuts: q, qdot, tau, t, ...
2. Numbers: ndofs, nframes, ...
"""

import os.path
import pydart_api
import numpy as np

def init():
    pydart_api.init()

def create_world(step):
    return World(step)
    
class World(object):
    def __init__(self, step):
        self.id = pydart_api.createWorld(step)
        self.skels = []
        self.control_skel = None

    def add_skeleton(self, filename, control = True):
        self.skels += [Skeleton(self, filename)]
        if control:
            self.control_skel = self.skels[-1]

    @property
    def skel(self):
        """ returns the default control skeleton """
        return self.control_skel

    def time(self):
        return pydart_api.getWorldTime(self.id)

    @property
    def t(self):
        return self.time()

    def time_step(self):
        return pydart_api.getWorldTimeStep(self.id)

    @property
    def dt(self):
        return self.time_step()

    def set_time_step(self, _time_step):
        pydart_api.setWorldTimeStep(self.id, _time_step)

    @dt.setter
    def dt(self, _dt):
        self.set_time_step(_dt)

    def num_frames(self):
        return pydart_api.getWorldSimFrames(self.id)

    @property
    def nframes(self):
        return self.num_frames()
        
    def contacts(self):
        n = pydart_api.getWorldNumContacts(self.id)
        contacts = pydart_api.getWorldContacts(self.id, 7 * n)
        return [contacts[7 * i : 7 * (i + 1)] for i in range(n)]

    def reset(self):
        pydart_api.resetWorld(self.id)

    def step(self):
        pydart_api.stepWorld(self.id)

    def set_frame(self, i):
        pydart_api.setWorldSimFrame(self.id, i)

    def render(self):
        pydart_api.render(self.id)

    def __repr__(self):
        return "<World.%d at %.4f>" % (self.id, self.t)


class Skeleton(object):
    def __init__(self, _world, _filename):
        self.world = _world
        self.filename = _filename
        self.id = pydart_api.addSkeleton(self.world.id, _filename)

        # Initialize dofs
        _ndofs = pydart_api.getSkeletonNumDofs(self.world.id, self.id)
        self.dofs = [ Dof(self, i) for i in range(_ndofs) ]
        self.name_to_dof = { dof.name : dof for dof in self.dofs }

        # Initialize bodies
        _nbodies = pydart_api.getSkeletonNumBodies(self.world.id, self.id)
        self.bodies = [ Body(self, i) for i in range(_nbodies) ]
        self.name_to_body = { body.name : body for body in self.bodies }
        
    def set_joint_damping(self, _damping):
        pydart_api.setSkeletonJointDamping(self.world.id, self.id, _damping)

    def num_dofs(self):
        return len(self.dofs)

    @property
    def ndofs(self):
        return self.num_dofs()

    def num_bodies(self):
        return len(self.bodies)
        
    @property
    def nbodies(self):
        return self.num_bodies()

    def mass(self):
        return pydart_api.getSkeletonMass(self.world.id, self.id)
        
    @property
    def m(self):
        return self.mass()

    def positions(self):
        return pydart_api.getSkeletonPositions(self.world.id, self.id, self.ndofs)
        
    @property
    def q(self):
        return self.positions()

    def set_positions(self, _q):
        pydart_api.setSkeletonPositions(self.world.id, self.id, _q)
    
    @q.setter
    def q(self, _q):
        """ Setter also updates the internal skeleton kinematics """
        self.set_positions(_q)

    def velocities(self):
        return pydart_api.getSkeletonVelocities(self.world.id, self.id, self.ndofs)

    @property
    def qdot(self):
        return self.velocities()

    def set_velocities(self, _qdot):
        pydart_api.setSkeletonVelocities(self.world.id, self.id, _qdot)

    @qdot.setter
    def qdot(self, _qdot):
        """ Setter also updates the internal skeleton kinematics """
        self.set_velocities(_qdot)

    def body(self, query):
        if isinstance(query, str):
            return self.name_to_body[query]
        elif isinstance(query, int):
            return self.bodies[query]
        else:
            print 'No find...', query
            return None
    def body_index(self, _name):
        return self.name_to_body[_name].id

    def dof_index(self, _name):
        return self.name_to_dof[_name].id

    def world_com(self):
        return pydart_api.getSkeletonWorldCOM(self.world.id, self.id)

    @property
    def C(self):
        return self.world_com()

    def world_com_velocity(self):
        return pydart_api.getSkeletonWorldCOMVelocity(self.world.id, self.id)

    @property
    def Cdot(self):
        return self.world_com_velocity()

    def linear_momentum(self):
        return self.Cdot * self.m

    @property
    def P(self):
        return self.linear_momentum()

    def forces(self):
        return self._tau

    @property
    def tau(self):
        return self.forces()
        
    def set_forces(self, _tau):
        self._tau = _tau
        pydart_api.setSkeletonForces(self.world.id, self.id, _tau)

    @tau.setter
    def tau(self, _tau):
        self.set_forces(_tau)

    def approx_inertia(self, axis):
        """ Calculates the point-masses approximated inertia with respect to the axis """
        axis = np.array(axis) / np.linalg.norm(axis)
        I = 0
        C = self.C
        for body in self.bodies:
            d = body.C - C
            # Subtract the distance along the axis
            r_sq = np.linalg.norm(d) ** 2 - np.linalg.norm(d.dot(axis)) ** 2 
            I += body.m * r_sq
        return I
        
    def approx_inertia_x(self):
        return self.approx_inertia([1, 0, 0])

    def approx_inertia_y(self):
        return self.approx_inertia([1, 0, 0])

    def approx_inertia_z(self):
        return self.approx_inertia([1, 0, 0])

    def external_contacts_and_body_id(self):
        cid_cnt = dict()
        contacts = []
        for body in self.bodies:
            for c in body.contacts():
                contacts += [(c, body.id)]
                cid = int(c[6])
                if cid not in cid_cnt:
                    cid_cnt[cid] = 1
                else:
                    cid_cnt[cid] += 1
        return [(c, bid) for (c, bid) in contacts if cid_cnt[int(c[6])] < 2]
        
    def contacted_bodies(self):
        return [body for body in self.bodies if body.num_contacts() > 0]

    def contacted_body_names(self):
        return [body.name for body in self.contacted_bodies()]

    def render(self):
        pydart_api.renderSkeleton(self.world.id, self.id)
    
    def __repr__(self):
        return '<Skel.%d.%s>' % (self.id, os.path.basename(self.filename))


class Body(object):
    def __init__(self, _skel, _id):
        self.skel = _skel
        self._id = _id
        self.name = pydart_api.getSkeletonBodyName(self.wid, self.sid, self.id)

    @property
    def id(self):
        return self._id

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def sid(self):
        return self.skel.id

    def num_contacts(self):
        return pydart_api.getBodyNodeNumContacts(self.wid, self.sid, self.id)
        
    def contacts(self):
        n = self.num_contacts()
        contacts = pydart_api.getBodyNodeContacts(self.wid, self.sid, self.id, 7 * n)
        return [contacts[7 * i : 7 * (i + 1)] for i in range(n)]

    def mass(self):
        return pydart_api.getBodyNodeMass(self.wid, self.sid, self.id)

    @property
    def m(self):
        return self.mass()

    def inertia(self):
        return pydart_api.getBodyNodeInertia(self.wid, self.sid, self.id)

    @property
    def I(self):
        return self.inertia()
    
    def local_com(self):
        return pydart_api.getBodyNodeLocalCOM(self.wid, self.sid, self.id)

    def world_com(self):
        return pydart_api.getBodyNodeWorldCOM(self.wid, self.sid, self.id)

    @property
    def C(self):
        return self.world_com()

    def world_com_velocity(self):
        return pydart_api.getBodyNodeWorldCOMVelocity(self.wid, self.sid, self.id)

    @property
    def Cdot(self):
        return self.world_com_velocity()

    def transformation(self):
        return pydart_api.getBodyNodeTransformation(self.wid, self.sid, self.id)

    @property
    def T(self):
        return self.transformation()

    def world_linear_jacobian(self):
        J = np.zeros((3, self.skel.ndofs))
        pydart_api.getBodyNodeWorldLinearJacobian(self.wid, self.sid, self.id, J)
        return J
    
    @property
    def J(self):
        return self.world_linear_jacobian()

    def __repr__(self):
        return '<Body.%d.%s>' % (self.id, self.name)

class Dof(object):    
    def __init__(self, _skel, _id):
        self.skel = _skel
        self.id = _id
        self.name = pydart_api.getSkeletonDofName(self.wid, self.sid, self.id)

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def sid(self):
        return self.skel.id

    def __repr__(self):
        return '<Dof.%s at %d>' % (self.name, self.id)
    
        

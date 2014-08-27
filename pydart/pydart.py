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

    @property
    def t(self):
        return pydart_api.getWorldTime(self.id)

    @property
    def nframes(self):
        return pydart_api.getWorldSimFrames(self.id)

    @property
    def contacts(self):
        n = pydart_api.getWorldNumContacts(self.id)
        contacts = pydart_api.getWorldContacts(self.id, 6 * n)
        return [contacts[6 * i : 6 * i + 6] for i in range(n)]

    def step(self):
        pydart_api.stepWorld(self.id)

    def set_frame(self, i):
        pydart_api.setWorldSimFrame(self.id, i)


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

    @property
    def ndofs(self):
        return len(self.dofs)

    @property
    def nbodies(self):
        return len(self.bodies)
        
    @property
    def q(self):
        return pydart_api.getSkeletonPositions(self.world.id, self.id, self.ndofs)

    @q.setter
    def q(self, _q):
        """ Setter also updates the internal skeleton kinematics """
        pydart_api.setSkeletonPositions(self.world.id, self.id, _q)

    @property
    def qdot(self):
        return pydart_api.getSkeletonVelocities(self.world.id, self.id, self.ndofs)

    def body(self, query):
        if isinstance(query, str):
            return self.name_to_body[query]
        elif isinstance(query, int):
            return self.bodies[query]
        else:
            print 'No find...', query
            return None

    @property
    def C(self):
        return pydart_api.getSkeletonWorldCOM(self.world.id, self.id)

    @property
    def forces(self):
        return self._tau
        
    @forces.setter
    def forces(self, _tau):
        pydart_api.setSkeletonForces(self.world.id, self.id, _tau)
        

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
        self.id = _id
        self.name = pydart_api.getSkeletonBodyName(self.wid, self.sid, self.id)

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def sid(self):
        return self.skel.id

    def num_contacts(self):
        return pydart_api.getBodyNodeNumContacts(self.wid, self.sid, self.id)
        
    def transformation(self):
        return pydart_api.getBodyNodeTransformation(self.wid, self.sid, self.id)

    def world_linear_jacobian(self):
        J = np.zeros((3, self.skel.ndofs))
        pydart_api.getBodyNodeWorldLinearJacobian(self.wid, self.sid, self.id, J)
        return J
    
    def __repr__(self):
        return '<Body.%s>' % self.name

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
        return '<Dof.%s>' % self.name
    
        

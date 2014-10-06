/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "pydart_api.h"
#include <iostream>
#include <vector>
#include <map>
using std::cout;
using std::cerr;
using std::endl;

// Dart headers
#include "dart/renderer/LoadOpengl.h"
#include "dart/renderer/RenderInterface.h"
#include "dart/renderer/OpenGLRenderInterface.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/sdf/SoftSdfParser.h"
#include "dart/utils/urdf/DartLoader.h"

namespace pydart {

////////////////////////////////////////////////////////////
// class Manager
class Manager {
public:
    static void init();
    static void destroy();
    static Manager* getInstance() { return g_manager; }
    static dart::renderer::RenderInterface* getRI() {
        return g_ri;
      
    }

    static dart::simulation::World* world(int index = 0);
    static dart::dynamics::Skeleton* skeleton(int index);
    static dart::dynamics::Skeleton* skeleton(int wid, int skid);
    static int createWorld(double timestep);
    
protected:
    static Manager* g_manager;
    static dart::renderer::RenderInterface* g_ri;

    std::vector<dart::simulation::World*> worlds;
};

Manager* Manager::g_manager = NULL;
dart::renderer::RenderInterface* Manager::g_ri = NULL;

void Manager::init() {
    g_manager = new Manager();
    g_ri = new dart::renderer::OpenGLRenderInterface();
    //   g_ri->initialize();
    cout << "Initialize pydart manager OK" << endl;
}

void Manager::destroy() {
    if (g_manager) {
        delete g_manager;
        g_manager = NULL;
    }
    if (g_ri) {
        delete g_ri;
        g_ri = NULL;
    }
    cout << "Destroy pydart manager OK" << endl;
}

dart::simulation::World* Manager::world(int index) {
    Manager* manager = getInstance();
    return manager->worlds[index];
}

dart::dynamics::Skeleton* Manager::skeleton(int index) {
    return world()->getSkeleton(index);
}

dart::dynamics::Skeleton* Manager::skeleton(int wid, int skid) {
    return world(wid)->getSkeleton(skid);
}


int Manager::createWorld(double timestep) {
    Manager* manager = getInstance();

    dart::simulation::World* w = new dart::simulation::World();
    w->setTimeStep(timestep);
    w->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    int id = manager->worlds.size();
    manager->worlds.push_back(w);
    return id;
}

// class Manager
////////////////////////////////////////////////////////////

} // namespace pydart

using namespace pydart;

////////////////////////////////////////////////////////////////////////////////
// Init Functions
void init() {
    if (Manager::getInstance()) {
        Manager::destroy();
    }
    Manager::init();
}

void destroy() {
    Manager::destroy();
}


////////////////////////////////////////////////////////////////////////////////
// Manipulation Functions
int createWorld(double timestep) {
    return Manager::createWorld(timestep);
}

void destroyWorld(int wid) {
}

int addSkeleton(int wid, const char* const path, double frictionCoeff) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    dart::utils::DartLoader urdfLoader;
    Skeleton* skel = urdfLoader.parseSkeleton(path);
    cout << "skel [" << path << "] : friction = " << frictionCoeff << endl;
    for (int i = 0; i < skel->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = skel->getBodyNode(i);
        bn->setFrictionCoeff(frictionCoeff);
    }
    
    World* world = Manager::world(wid);
    int id = world->getNumSkeletons();
    world->addSkeleton(skel);
    return id;
}

void setSkeletonJointDamping(int wid, int skid, double damping) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);

    for (int i = 1; i < skel->getNumBodyNodes(); ++i) {
        dart::dynamics::Joint* joint = skel->getJoint(i);
        if (joint->getNumDofs() > 0) {
            for (int j = 0; j < joint->getNumDofs(); ++j) {
                joint->setDampingCoefficient(j, damping);
            }
        }
    }

}
////////////////////////////////////////////////////////////////////////////////
// Simulation Functions
void resetWorld(int wid) {
    using namespace dart::simulation;
    World* world = Manager::world(wid);
    world->reset();
}

void stepWorld(int wid) {
    using namespace dart::simulation;
    World* world = Manager::world(wid);
    world->step();
    world->bake();
}

void render(int wid) {
    using namespace dart::simulation;
    dart::renderer::RenderInterface* ri = Manager::getRI();
    World* world = Manager::world(wid);
    
    for (size_t i = 0; i < world->getNumSkeletons(); i++) {
        world->getSkeleton(i)->draw(ri);
    }
}

void renderSkeleton(int wid, int skid) {
    using namespace dart::dynamics;
    dart::renderer::RenderInterface* ri = Manager::getRI();
    Skeleton* skel = Manager::skeleton(wid, skid);
    skel->draw(ri);
}

////////////////////////////////////////////////////////////////////////////////
// World Functions
double getWorldTime(int wid) {
    using namespace dart::simulation;
    World* world = Manager::world(wid);
    return world->getTime();
}

double getWorldTimeStep(int wid) {
    using namespace dart::simulation;
    World* world = Manager::world(wid);
    return world->getTimeStep();
}

void setWorldTimeStep(int wid, double _timeStep) {
    using namespace dart::simulation;
    World* world = Manager::world(wid);
    world->setTimeStep(_timeStep);
}


int getWorldSimFrames(int wid) {
    using namespace dart::simulation;
    World* world = Manager::world(wid);
    return world->getSimFrames();
}

void setWorldSimFrame(int wid, int playFrame) {
    using namespace dart::simulation;
    World* world = Manager::world(wid);
    if (playFrame >= world->getRecording()->getNumFrames()) {
        return;
    }
    
    size_t nSkels = world->getNumSkeletons();
    for (size_t i = 0; i < nSkels; i++) {
        // size_t start = world->getIndex(i);
        // size_t size = world->getSkeleton(i)->getNumDofs();
        world->getSkeleton(i)->setPositions(world->getRecording()->getConfig(playFrame, i));
        world->getSkeleton(i)->computeForwardKinematics(true, true, false);
    }
}

int getWorldNumContacts(int wid) {
    dart::simulation::World* world = Manager::world(wid);
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    return cd->getNumContacts();
}

void getWorldContacts(int wid, double* outv, int len) {
    dart::simulation::World* world = Manager::world(wid);
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    int n = cd->getNumContacts();
    if (7 * n != len) {
        cerr << "getWorldContacts: 7n is needed for the output vector. n = " << n << ", len =  " << len << endl;
        return;
    }

    // ( v.x, v.y, v.z, p.x, p.y, p.z, id )
    
    int ptr = 0;
    for (size_t i = 0; i < n; i++) {
        Eigen::Vector3d v = cd->getContact(i).point;
        Eigen::Vector3d f = cd->getContact(i).force;
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = v(j);
        }
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = f(j);
        }
        outv[ptr++] = i;

    }    
}

////////////////////////////////////////////////////////////////////////////////
// Skeleton Attribute Functions
double getSkeletonMass(int wid, int skid) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    return skel->getMass();
}

int getSkeletonNumDofs(int wid, int skid) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    return skel->getNumDofs();
}

int getSkeletonNumBodies(int wid, int skid) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    return skel->getNumBodyNodes();
}

const char* getSkeletonBodyName(int wid, int skid, int bodyid) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(wid, skid);
    return skel->getBodyNode(bodyid)->getName().c_str();
}

const char* getSkeletonDofName(int wid, int skid, int dofid) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(wid, skid);
    return skel->getGenCoordInfo(dofid).joint->getName().c_str();
}

////////////////////////////////////////////////////////////////////////////////
// Skeleton Pose Functions
void getSkeletonPositions(int wid, int skid, double* outpose, int ndofs) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    
    Eigen::VectorXd q = skel->getPositions();
    for (int i = 0; i < q.size(); i++) {
        outpose[i] = q(i);
    }
}

void getSkeletonVelocities(int wid, int skid, double* outpose, int ndofs) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    
    Eigen::VectorXd qdot = skel->getVelocities();
    for (int i = 0; i < qdot.size(); i++) {
        outpose[i] = qdot(i);
    }
}

void setSkeletonPositions(int wid, int skid, double* inpose, int ndofs) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);

    Eigen::VectorXd q(ndofs);
    for (int i = 0; i < q.size(); i++) {
        q(i) = inpose[i];
    }
    skel->setPositions(q);
    skel->computeForwardKinematics(true, true, false);
}

void setSkeletonVelocities(int wid, int skid, double* inpose, int ndofs) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);

    Eigen::VectorXd q(ndofs);
    for (int i = 0; i < q.size(); i++) {
        q(i) = inpose[i];
    }
    skel->setVelocities(q);
    skel->computeForwardKinematics(true, true, false);
}

void setSkeletonForces(int wid, int skid, double* intorque, int ndofs) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);

    Eigen::VectorXd tau(ndofs);
    for (int i = 0; i < tau.size(); i++) {
        tau(i) = intorque[i];
    }
    skel->setForces(tau);
}

////////////////////////////////////////////////////////////////////////////////
// Skeleton Momentum Functions
void getSkeletonWorldCOM(int wid, int skid, double outv3[3]) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    Eigen::Vector3d C = skel->getWorldCOM();
    for (int i = 0; i < C.size(); i++) {
        outv3[i] = C(i);
    }
}

void getSkeletonWorldCOMVelocity(int wid, int skid, double outv3[3]) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    Eigen::Vector3d CV = skel->getWorldCOMVelocity();
    for (int i = 0; i < CV.size(); i++) {
        outv3[i] = CV(i);
    }
}

////////////////////////////////////////////////////////////////////////////////
// BodyNode Functions
double getBodyNodeMass(int wid, int skid, int bid) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    return bn->getMass();
}

void getBodyNodeInertia(int wid, int skid, int bid, double outv33[3][3]) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    bn->getMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    outv33[0][0] = Ixx;    outv33[1][1] = Iyy;    outv33[2][2] = Izz;
    outv33[0][1] = Ixy;    outv33[1][0] = Ixy; 
    outv33[0][2] = Ixz;    outv33[2][0] = Ixz; 
    outv33[1][2] = Iyz;    outv33[2][1] = Iyz; 
}

void getBodyNodeLocalCOM(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    const Eigen::Vector3d& x = bn->getLocalCOM();
    for (int i = 0; i < x.size(); i++) {
        outv3[i] = x(i);
    }
}


void getBodyNodeWorldCOM(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    const Eigen::Vector3d& x = bn->getWorldCOM();
    for (int i = 0; i < x.size(); i++) {
        outv3[i] = x(i);
    }
}

void getBodyNodeWorldCOMVelocity(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);
    const Eigen::Vector3d& x = bn->getWorldCOMVelocity();
    for (int i = 0; i < x.size(); i++) {
        outv3[i] = x(i);
    }
}

int getBodyNodeNumContacts(int wid, int skid, int bid) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);

    dart::simulation::World* world = Manager::world(wid);
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    int n = cd->getNumContacts();
    int cnt = 0;
    for (size_t i = 0; i < n; i++) {
        dart::collision::Contact& c = cd->getContact(i);
        if (c.bodyNode1 == bn || c.bodyNode2 == bn) {
            cnt++;
        }
    }
    return cnt;
}

void getBodyNodeContacts(int wid, int skid, int bid, double* outv, int len) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(wid, skid);
    dart::dynamics::BodyNode* bn = skel->getBodyNode(bid);

    dart::simulation::World* world = Manager::world(wid);
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    int n = cd->getNumContacts();

    int m = 0;
    for (size_t i = 0; i < n; i++) {
        dart::collision::Contact& c = cd->getContact(i);
        if (c.bodyNode1 != bn && c.bodyNode2 != bn) {
            continue;
        }
        m++;
    }

    if (7 * m != len) {
        cerr << "getBodyNodeContacts: 7m is needed for the output vector. m = " << m
             << ", n = " << n << ", len =  " << len << endl;
        return;
    }

    int ptr = 0;
    for (size_t i = 0; i < n; i++) {
        dart::collision::Contact& c = cd->getContact(i);
        if (c.bodyNode1 != bn && c.bodyNode2 != bn) {
            continue;
        }
        Eigen::Vector3d v = cd->getContact(i).point;
        Eigen::Vector3d f = cd->getContact(i).force;
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = v(j);
        }
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = f(j);
        }
        outv[ptr++] = i;

    }
}

void getBodyNodeTransformation(int wid, int skid, int bid, double outv44[4][4]) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    if (!body) {
        cerr << "cannot find the body : " << bid << endl;
    }
    const Eigen::Isometry3d& T = body->getTransform();
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            outv44[i][j] = T(i, j);
        }
    }
}

void getBodyNodeWorldLinearJacobian(int wid, int skid, int bid, double* array2, int nrows, int ncols) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    if (!body) {
        cerr << "cannot find the body : " << bid << endl;
    }

    int N = skel->getNumDofs();
    Eigen::MatrixXd J = body->getWorldLinearJacobian();
    Eigen::MatrixXd JF = Eigen::MatrixXd::Zero(3, N);

    for (int i = 0; i < J.cols(); i++) {
        int j = body->getDependentGenCoordIndex(i);
        JF.col(j) = J.col(i);
    }

    int ptr = 0;
    for (int i = 0; i < JF.rows(); i++) {
        for (int j = 0; j < JF.cols(); j++) {
            array2[ptr++] = JF(i, j);
        }
    }
}

void addBodyNodeExtForce(int wid, int skid, int bid, double inv3[3]) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    if (!body) {
        cerr << "cannot find the body : " << bid << endl;
    }
    Eigen::Vector3d f(inv3[0], inv3[1], inv3[2]);
    body->addExtForce(f);
}

void addBodyNodeExtForceAt(int wid, int skid, int bid, double inv3[3], double inv3_2[3]) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(wid, skid);
    BodyNode* body = skel->getBodyNode(bid);
    if (!body) {
        cerr << "cannot find the body : " << bid << endl;
    }
    Eigen::Vector3d f(inv3[0], inv3[1], inv3[2]);
    Eigen::Vector3d offset(inv3_2[0], inv3_2[1], inv3_2[2]);
    body->addExtForce(f, offset);
}






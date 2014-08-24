/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "pydart_api.h"
#include <iostream>
#include <vector>
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
    static dart::renderer::RenderInterface* getRI() { return g_ri; }

    static dart::simulation::World* world(int index = 0);
    static dart::dynamics::Skeleton* skeleton(int index);
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
    g_ri->initialize();
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

void init() {
    if (Manager::getInstance()) {
        Manager::destroy();
    }
    Manager::init();
}

void destroy() {
    Manager::destroy();
}


int createWorld(double timestep) {
    return Manager::createWorld(timestep);
}

void destroyWorld() {
}

double getWorldTime() {
    using namespace dart::simulation;
    World* world = Manager::world();
    return world->getTime();
}
 

int getWorldSimFrames() {
    using namespace dart::simulation;
    World* world = Manager::world();
    return world->getSimFrames();
}

void setWorldSimFrame(int playFrame) {
    using namespace dart::simulation;
    World* world = Manager::world();
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


int addSkeleton(const char* const path) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    dart::utils::DartLoader urdfLoader;
    Skeleton* skel = urdfLoader.parseSkeleton(path);

    World* world = Manager::world();
    int id = world->getNumSkeletons();
    world->addSkeleton(skel);
    return id;
}

void setSkeletonJointDamping(int skid, double damping) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);

    for (int i = 1; i < skel->getNumBodyNodes(); ++i) {
        dart::dynamics::Joint* joint = skel->getJoint(i);
        if (joint->getNumDofs() > 0) {
            for (int j = 0; j < joint->getNumDofs(); ++j) {
                joint->setDampingCoefficient(j, damping);
            }
        }
    }

}


// Attribute functions
double getSkeletonMass(int skid) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);
    return skel->getMass();
}

int getSkeletonNumBodies(int skid) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);
    return skel->getNumBodyNodes();
}

int getSkeletonNumDofs(int skid) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    World* world = Manager::world();
    Skeleton* skel = world->getSkeleton(skid);
    return skel->getNumDofs();
}

const char* getSkeletonBodyName(int skid, int bodyid) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(skid);
    return skel->getBodyNode(bodyid)->getName().c_str();
}

const char* getSkeletonDofName(int skid, int dofid) {
    dart::dynamics::Skeleton* skel = Manager::skeleton(skid);
    return skel->getGenCoordInfo(dofid).joint->getName().c_str();
}

void getSkeletonPositions(int skid, double* outpose, int ndofs) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);
    
    Eigen::VectorXd q = skel->getPositions();
    for (int i = 0; i < q.size(); i++) {
        outpose[i] = q(i);
    }
}

void getSkeletonVelocities(int skid, double* outpose, int ndofs) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);
    
    Eigen::VectorXd qdot = skel->getVelocities();
    for (int i = 0; i < qdot.size(); i++) {
        outpose[i] = qdot(i);
    }
}


void setSkeletonPositions(int skid, double* inpose, int ndofs) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);

    Eigen::VectorXd q(ndofs);
    for (int i = 0; i < q.size(); i++) {
        q(i) = inpose[i];
    }
    skel->setPositions(q);
    skel->computeForwardKinematics(true, true, false);
}


void setSkeletonForces(int skid, double* intorque, int ndofs) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);

    Eigen::VectorXd tau(ndofs);
    for (int i = 0; i < tau.size(); i++) {
        tau(i) = intorque[i];
    }
    skel->setForces(tau);
}

// Momentum functions
void getSkeletonWorldCOM(int skid, double outv3[3]) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);
    Eigen::Vector3d C = skel->getWorldCOM();
    for (int i = 0; i < C.size(); i++) {
        outv3[i] = C(i);
    }
}

void getSkeletonWorldCOMVelocity(int skid, double outv3[3]) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);
    Eigen::Vector3d CV = skel->getWorldCOMVelocity();
    for (int i = 0; i < CV.size(); i++) {
        outv3[i] = CV(i);
    }
}

// BodyNode functions
void getBodyNodeWorldLinearJacobian(int skid, const char* const bname, double* array2, int nrows, int ncols) {
    using namespace dart::dynamics;
    Skeleton* skel = Manager::skeleton(skid);
    BodyNode* body = skel->getBodyNode(bname);
    if (!body) {
        cerr << "cannot find the body : " << bname << endl;
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

// World query functions
int getWorldNumContacts() {
    dart::simulation::World* world = Manager::world();
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    return cd->getNumContacts();
}


void getWorldContacts(double* outv, int len) {
    dart::simulation::World* world = Manager::world();
    dart::collision::CollisionDetector* cd =
        world->getConstraintSolver()->getCollisionDetector();
    int n = cd->getNumContacts();
    if (6 * n != len) {
        cerr << "6n is needed for the output vector. n = " << n << ", len =  " << len << endl;
        return;
    }

    int ptr = 0;
    for (size_t i = 0; i < n; i++) {
        Eigen::Vector3d v = cd->getContact(i).point;
        Eigen::Vector3d f = cd->getContact(i).force / 10.0;
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = v(j);
        }
        for (int j = 0; j < 3; j++) {
            outv[ptr++] = f(j);
        }

    }    
}

// World functions
void stepWorld() {
    using namespace dart::simulation;
    World* world = Manager::world();
    world->step();
    world->bake();
}

void render() {
    using namespace dart::simulation;
    dart::renderer::RenderInterface* ri = Manager::getRI();
    World* world = Manager::world();
    
    for (size_t i = 0; i < world->getNumSkeletons(); i++) {
        world->getSkeleton(i)->draw(ri);
    }
}

void renderSkeleton(int skid) {
    using namespace dart::dynamics;
    dart::renderer::RenderInterface* ri = Manager::getRI();
    Skeleton* skel = Manager::skeleton(skid);
    skel->draw(ri);
}




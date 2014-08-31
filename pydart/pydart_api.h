/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef PYDART_PYDART_API_H
#define PYDART_PYDART_API_H

// Init Functions
void init();
void destroy();

// Manipulation Functions
int createWorld(double timestep);
void destroyWorld(int wid);
int addSkeleton(int wid, const char* const path);
void setSkeletonJointDamping(int wid, int skid, double damping);

// Simulation Functions
void resetWorld(int wid);
void stepWorld(int wid);
void render(int wid);
void renderSkeleton(int wid, int skid);

// World Functions
double getWorldTime(int wid);
int getWorldSimFrames(int wid);
void setWorldSimFrame(int wid, int playFrame);
int getWorldNumContacts(int wid);
void getWorldContacts(int wid, double* outv, int len);

// Skeleton Attribute Functions
double getSkeletonMass(int wid, int skid);
int getSkeletonNumBodies(int wid, int skid);
int getSkeletonNumDofs(int wid, int skid);
const char* getSkeletonBodyName(int wid, int skid, int bodyid);
const char* getSkeletonDofName(int wid, int skid, int dofid);

// Skeleton Pose Functions
void getSkeletonPositions(int wid, int skid, double* outpose, int ndofs);
void getSkeletonVelocities(int wid, int skid, double* outpose, int ndofs);
void setSkeletonPositions(int wid, int skid, double* inpose, int ndofs);
void setSkeletonVelocities(int wid, int skid, double* inpose, int ndofs);
void setSkeletonForces(int wid, int skid, double* intorque, int ndofs);

// Skeleton Momentum Functions
void getSkeletonWorldCOM(int wid, int skid, double outv3[3]);
void getSkeletonWorldCOMVelocity(int wid, int skid, double outv3[3]);

// BodyNode Functions
double getBodyNodeMass(int wid, int skid, int bid);
void getBodyNodeInertia(int wid, int skid, int bid, double outv33[3][3]);
void getBodyNodeLocalCOM(int wid, int skid, int bid, double outv3[3]);
void getBodyNodeWorldCOM(int wid, int skid, int bid, double outv3[3]);
void getBodyNodeWorldCOMVelocity(int wid, int skid, int bid, double outv3[3]);
int getBodyNodeNumContacts(int wid, int skid, int bid);
void getBodyNodeTransformation(int wid, int skid, int bid, double outv44[4][4]);
void getBodyNodeWorldLinearJacobian(int wid, int skid, int bid, double* array2, int nrows, int ncols);


#endif // #ifndef PYDART_PYDART_API_H


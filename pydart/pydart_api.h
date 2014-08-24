/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef PYDART_PYDART_API_H
#define PYDART_PYDART_API_H

void init();
void destroy();

// World Functions
int createWorld(double timestep);
void destroyWorld();
double getWorldTime();
int getWorldSimFrames();
void setWorldSimFrame(int playFrame);

// void setDefaultWorld(int wid = -1);
// int defaultWorldId();
// int createDefaultWorld();

int addSkeleton(const char* const path);

// Attribute functions
double getSkeletonMass(int skid);
int getSkeletonNumBodies(int skid);
int getSkeletonNumDofs(int skid);
const char* getSkeletonBodyName(int skid, int bodyid);
const char* getSkeletonDofName(int skid, int dofid);
void setSkeletonJointDamping(int skid, double damping);

// Pose functions
void getSkeletonPositions(int skid, double* outpose, int ndofs);
void getSkeletonVelocities(int skid, double* outpose, int ndofs);
void setSkeletonPositions(int skid, double* inpose, int ndofs);
void setSkeletonForces(int skid, double* intorque, int ndofs);

// Momentum functions
void getSkeletonWorldCOM(int skid, double outv3[3]);
void getSkeletonWorldCOMVelocity(int skid, double outv3[3]);

// BodyNode functions
void getBodyNodeTransformation(int skid, const char* const bname, double outv44[4][4]);
void getBodyNodeWorldLinearJacobian(int skid, const char* const bname, double* array2, int nrows, int ncols);

// World query functions
int getWorldNumContacts();
void getWorldContacts(double* outv, int len);

// World functions
void stepWorld();
void render();
void renderSkeleton(int skid);

#endif // #ifndef PYDART_PYDART_API_H


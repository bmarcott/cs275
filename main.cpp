/*******************************************************************************
Basic Example for ODE Usage
*******************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include "ode/ode.h"

#include <gl/gl.h>
#include <gl/glu.h>
#include "GL/glut.h"

#include <math.h>
#include <iostream>

using namespace std;

#include "Ball.h"
#include "FrameSaver.h"
#include "Timer.h"
#include "GDrawing.h"

///////////////////////////  OpenGL global variables ///////////////////////////
FrameSaver g_frameSaver;
Timer g_timer;

BallData *g_arcBall = NULL;
int g_width = 700;
int g_height = 700;
int g_button = -1;
float g_zoom = 1;
int g_previousY = 0;

int g_animate = 0;
int g_recording = 0;

void resetArcball();
void save_image();

const float PI = 3.1415926535;
const int STRLEN = 100;
typedef char STR[STRLEN];

#define X 0
#define Y 1
#define Z 2

// The eye point and look-at point.
double g_eye[3] = {0.0, 0.0, 50.0};
double g_ref[3] = {0.0, 0.0, 0.0};
double g_time = 0.0 ;
////////////////////////////////////////////////////////////////////////////////

/////////////////////////// ODE Global Variables ///////////////////////////////
dReal simulationTime = 0.0;
dReal simulationStep = 0.01;
struct MyObject
{
	dBodyID Body;		//The dynamics body.
	dGeomID Geom[1];	//Geometries representing this body.
};

MyObject Object;			//The rigid body (a box).
MyObject Rod;
MyObject body;				//Insect Body
MyObject frLeg;
MyObject flLeg;
MyObject middleRightLeg;
MyObject middleLeftLeg;
MyObject brLeg;
MyObject blLeg;
dWorldID World;				//Dynamics world.
dSpaceID Space;				//A space that defines collisions.
dJointGroupID jointgroup;   // contact group for the new joint
dJointGroupID ContactGroup;	//Group of contact joints for collision detection/handling.
dJointID mrLegJoint;             // the joint ID
dJointID Joint2;
dJointID frLegJoint;
dJointID flLegJoint;
dJointID mlLegJoint;
dJointID brLegJoint;
dJointID blLegJoint;

void createFrontRightLeg(dMatrix3 orient)
{
	frLeg.Body = dBodyCreate(World);
	dBodySetPosition(frLeg.Body, -3.5, 3.5, -2);
	dBodySetRotation(frLeg.Body, orient);
	dBodySetLinearVel(frLeg.Body, 0, 0, 0);
	dBodySetData(frLeg.Body, (void *)0);
	dMass frLegMass;
	dMassSetCapsule(&frLegMass, 1, 3, 0.15, 2);

	dBodySetMass( frLeg.Body, &frLegMass );

	frLeg.Geom[0] = dCreateCapsule(Space, 0.15, 5);
	dGeomSetBody(frLeg.Geom[0], frLeg.Body);

	frLegJoint = dJointCreateFixed(World, jointgroup);
	dJointAttach(frLegJoint, body.Body, frLeg.Body);
	dJointSetFixed(frLegJoint);

	//frLegJoint = dJointCreateHinge(World, jointgroup);
	//dJointSetHingeParam(frLegJoint, dParamHiStop, PI / 6);
	//dJointSetHingeParam(frLegJoint, dParamLoStop, -PI / 6);

	//dJointSetHingeAnchor(frLegJoint, -1, 5, -2);
	//dJointSetHingeAxis(frLegJoint, 0, 0, 1);
}

void createFrontLeftLeg(dMatrix3 orient)
{
	flLeg.Body = dBodyCreate(World);
	dBodySetPosition(flLeg.Body, 3.5, 3.5, -2);
	dBodySetRotation(flLeg.Body, orient);
	dBodySetLinearVel(flLeg.Body, 0, 0, 0);
	dBodySetData(flLeg.Body, (void *)0);
	dMass flLegMass;
	dMassSetCapsule(&flLegMass, 1, 3, 0.15, 2);

	dBodySetMass( frLeg.Body, &flLegMass );

	flLeg.Geom[0] = dCreateCapsule(Space, 0.15, 5);
	dGeomSetBody(flLeg.Geom[0], flLeg.Body);

	flLegJoint = dJointCreateFixed(World, jointgroup);
	dJointAttach(flLegJoint, body.Body, flLeg.Body);
	dJointSetFixed(flLegJoint);
/*
	flLegJoint = dJointCreateHinge(World, jointgroup);
	dJointSetHingeParam(flLegJoint, dParamHiStop, PI / 6);
	dJointSetHingeParam(flLegJoint, dParamLoStop, -PI / 6);
	
	dJointSetHingeAnchor(flLegJoint, 1, 5, 2);
	dJointSetHingeAxis(flLegJoint, 0, 0, 1);*/
}

void createMiddleRightLeg(dMatrix3 orient)
{
	middleRightLeg.Body = dBodyCreate(World);
	dBodySetPosition(middleRightLeg.Body, -3.5, 3.5, 0);
	dBodySetRotation(middleRightLeg.Body, orient);
	dBodySetLinearVel(middleRightLeg.Body, 0, 0, 0);
	dBodySetData(middleRightLeg.Body, (void *)0);
	dMass midRLegMass;
	dMassSetCapsule(&midRLegMass, 1, 3, 0.15, 2);
	
	dBodySetMass( frLeg.Body, &midRLegMass );
	
	middleRightLeg.Geom[0] = dCreateCapsule(Space, 0.15, 5);
	dGeomSetBody(middleRightLeg.Geom[0], middleRightLeg.Body);

	mrLegJoint = dJointCreateUniversal(World, jointgroup);
	dJointSetUniversalAnchor(mrLegJoint, -1, 5, 0);
	dJointSetUniversalAxis1(mrLegJoint, 0, 0, 1);
	dJointSetUniversalParam(mrLegJoint, dParamHiStop, PI / 6);
	dJointSetUniversalParam(mrLegJoint, dParamLoStop, -PI / 6);
	dJointSetUniversalAxis2(mrLegJoint, 0, 1, 0);
	dJointSetUniversalParam(mrLegJoint, dParamHiStop, PI / 6);
	dJointSetUniversalParam(mrLegJoint, dParamLoStop, -PI / 6);
	dJointAttach(mrLegJoint, body.Body, middleRightLeg.Body);

	/*Joint = dJointCreateHinge(World, jointgroup);
	dJointSetHingeParam(Joint, dParamHiStop, PI / 6);
	dJointSetHingeParam(Joint, dParamLoStop, -PI / 6);
	dJointAttach(Joint, body.Body, middleRightLeg.Body);
	dJointSetHingeAnchor(Joint, -1, 5, 0);
	dJointSetHingeAxis(Joint, 0, 0, 1);*/
}

void createMiddleLeftLeg(dMatrix3 orient)
{
	middleLeftLeg.Body = dBodyCreate(World);
	dBodySetPosition(middleLeftLeg.Body, 3.5, 3.5, 0);
	dBodySetRotation(middleLeftLeg.Body, orient);
	dBodySetLinearVel(middleLeftLeg.Body, 0, 0, 0);
	dBodySetData(middleLeftLeg.Body, (void *)0);
	dMass midLLegMass;
	dMassSetCapsule(&midLLegMass, 1, 3, 0.15, 2);

	dBodySetMass( frLeg.Body, &midLLegMass );

	middleLeftLeg.Geom[0] = dCreateCapsule(Space, 0.15, 5);
	dGeomSetBody(middleLeftLeg.Geom[0], middleLeftLeg.Body);

	mlLegJoint = dJointCreateUniversal(World, jointgroup);
	dJointSetUniversalAnchor(mlLegJoint, 1, 5, 0);
	dJointSetUniversalAxis1(  mlLegJoint, 0, 0, 1);
	dJointSetUniversalParam(mlLegJoint, dParamHiStop, PI / 6);
	dJointSetUniversalParam(mlLegJoint, dParamLoStop, -PI / 6);
	dJointSetUniversalAxis2 ( mlLegJoint, 0, 1, 0);
	dJointSetUniversalParam(mlLegJoint, dParamHiStop, PI / 6);
	dJointSetUniversalParam(mlLegJoint, dParamLoStop, -PI / 6);
	dJointAttach(mlLegJoint, body.Body, middleLeftLeg.Body);
	//dJointSetHingeAnchor(mlLegJoint, 1, 5, 0);
	//dJointSetHingeAxis(mlLegJoint, 0, 0, 1);
}

void createBackLegs()
{

	dReal xPos = -3;
	dReal yPos = 2.5;
	dReal zPos = 5.5;
	dReal xRot = -PI / 4;
	dReal yRot = PI / 6;
	dReal zRot = 0;
	dReal radius = 0.15;
	dReal length = 7.5;

	dMatrix3 brLegOrient;
	dRFromEulerAngles(brLegOrient, xRot, yRot, zRot);

	dMatrix3 blLegOrient;
<<<<<<< HEAD
<<<<<<< HEAD
	dRFromEulerAngles(blLegOrient, xRot, -yRot, zRot);
=======
	dRFromEulerAngles(blLegOrient, xRot, -yRot, zRot;)
>>>>>>> d691c16fa799ca013cc0e75955b9dae48719dc15
=======
	dRFromEulerAngles(blLegOrient, xRot, -yRot, zRot;)
>>>>>>> d691c16fa799ca013cc0e75955b9dae48719dc15

	//Back right leg
	brLeg.Body = dBodyCreate(World);
	dBodySetPosition(brLeg.Body, xPos, yPos, zPos);
	dBodySetRotation(brLeg.Body, brLegOrient);
	dBodySetLinearVel(brLeg.Body, 0, 0, 0);
	dBodySetData(brLeg.Body, (void *)0);
	dMass brLegMass;
	dMassSetCapsule(&brLegMass, 1, 3, radius, length);
<<<<<<< HEAD
<<<<<<< HEAD

	dBodySetMass( frLeg.Body, &brLegMass );

=======
>>>>>>> d691c16fa799ca013cc0e75955b9dae48719dc15
=======
>>>>>>> d691c16fa799ca013cc0e75955b9dae48719dc15
	brLeg.Geom[0] = dCreateCapsule(Space, radius, length);
	dGeomSetBody(brLeg.Geom[0], brLeg.Body);

	brLegJoint = dJointCreateFixed(World, jointgroup);
	dJointAttach(brLegJoint, body.Body, brLeg.Body);
	dJointSetFixed(brLegJoint);

	//Back left leg
	blLeg.Body = dBodyCreate(World);
	dBodySetPosition(blLeg.Body, -xPos, yPos, zPos);
	dBodySetRotation(blLeg.Body, blLegOrient);
	dBodySetLinearVel(blLeg.Body, 0, 0, 0);
	dBodySetData(blLeg.Body, (void *)0);
	dMass blLegMass;
<<<<<<< HEAD
<<<<<<< HEAD
	dMassSetCapsule(&blLegMass, 1, 3, radius, length);

	dBodySetMass( blLeg.Body, &blLegMass );

	blLeg.Geom[0] = dCreateCapsule(Space, radius, length);
=======
	dMassSetCapsule(&blLegMass, 1, 3, 0.15, 2);
	blLeg.Geom[0] = dCreateCapsule(Space, 0.15, 5);
>>>>>>> d691c16fa799ca013cc0e75955b9dae48719dc15
=======
	dMassSetCapsule(&blLegMass, 1, 3, 0.15, 2);
	blLeg.Geom[0] = dCreateCapsule(Space, 0.15, 5);
>>>>>>> d691c16fa799ca013cc0e75955b9dae48719dc15
	dGeomSetBody(blLeg.Geom[0], blLeg.Body);

	blLegJoint = dJointCreateFixed(World, jointgroup);
	dJointAttach(blLegJoint, body.Body, blLeg.Body);
	dJointSetFixed(blLegJoint);
}

void createBackRightLeg(dMatrix3 orient)
{
	brLeg.Body = dBodyCreate(World);
	dBodySetPosition(brLeg.Body, -3, 2.5, 5.5);
	dBodySetRotation(brLeg.Body, orient);
	dBodySetLinearVel(brLeg.Body, 0, 0, 0);
	dBodySetData(brLeg.Body, (void *)0);
	dMass brLegMass;
	dMassSetCapsule(&brLegMass, 1, 3, 0.15, 2);
	brLeg.Geom[0] = dCreateCapsule(Space, 0.15, 7.5);
	dGeomSetBody(brLeg.Geom[0], brLeg.Body);

	brLegJoint = dJointCreateFixed(World, jointgroup);
	dJointAttach(brLegJoint, body.Body, brLeg.Body);
	dJointSetFixed(brLegJoint);

	/*brLegJoint = dJointCreateHinge(World, jointgroup);
	dJointSetHingeParam(brLegJoint, dParamHiStop, PI / 6);
	dJointSetHingeParam(brLegJoint, dParamLoStop, -PI / 6);
	dJointAttach(brLegJoint, body.Body, brLeg.Body);
	dJointSetHingeAnchor(brLegJoint, -1, 5, 3.5);
	dJointSetHingeAxis(brLegJoint, 0, 0, 1);*/
}

void createBackLeftLeg(dMatrix3 orient)
{
	blLeg.Body = dBodyCreate(World);
	dBodySetPosition(blLeg.Body, 3.5, 2.5, 5.5);
	dBodySetRotation(blLeg.Body, orient);
	dBodySetLinearVel(blLeg.Body, 0, 0, 0);
	dBodySetData(blLeg.Body, (void *)0);
	dMass blLegMass;
	dMassSetCapsule(&blLegMass, 1, 3, 0.15, 2);
	dBodySetMass(blLeg.Body, &blLegMass);
	blLeg.Geom[0] = dCreateCapsule(Space, 0.15, 5);
	dGeomSetBody(blLeg.Geom[0], blLeg.Body);

	blLegJoint = dJointCreateFixed(World, jointgroup);
	dJointAttach(blLegJoint, body.Body, blLeg.Body);
	dJointSetFixed(blLegJoint);

	//blLegJoint = dJointCreateHinge(World, jointgroup);
	//dJointSetHingeParam(blLegJoint, dParamHiStop, PI / 6);
	//dJointSetHingeParam(blLegJoint, dParamLoStop, -PI / 6);
	//dJointAttach(blLegJoint, body.Body, blLeg.Body);
	//dJointSetHingeAnchor(blLegJoint, 1, 5, 3.5);
	//dJointSetHingeAxis(blLegJoint, 0, 0, 1);
}

////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
Function to initialize ODE.
*******************************************************************************/
void initODE()
{
	///////////////// Initializing the ODE general features ////////////////////

	dInitODE();								//Initialize library.
	World = dWorldCreate();					//Crate a new dynamics, empty world.
	Space = dSimpleSpaceCreate(0);			//Create a new space for collision (independent).
	ContactGroup = dJointGroupCreate(0);	//Create a joints container, without specifying size.

	dWorldSetGravity( World, 0.0, -1.0, 0 );	//Add gravity to this World.

	//Define error conrrection constants.
	dWorldSetERP( World, 0.2 );
	dWorldSetCFM( World, 1e-5 );

	//Set the velocity that interpenetrating objects will separate at.
	dWorldSetContactMaxCorrectingVel( World, 0.9 );

	//Set the safety area for contacts to be considered at rest.
	dWorldSetContactSurfaceLayer( World, 0.001 );

	//Automatically disable objects that have come to a rest.
	dWorldSetAutoDisableFlag( World, false );

	/////////////// Initializing the rigid bodies in the world /////////////////

	//Create a collision plane and add it to space. The next parameters are the
	//literal components of ax + by + cz = d.
	dCreatePlane( Space, 0.0, 1.0, 0.0, 0.0 );

	//Object.Body = dBodyCreate( World );		//Create a new body and attach it to object.
	//dBodySetPosition( Object.Body, 0.0, 10, 0.0 );		//Assign position to body.
	//dBodySetLinearVel( Object.Body, 0.0, 0.0, 0.0 );	//Object begins stationary.

	//dMatrix3 Orient;						//Assign a random orientation.
	//dReal axis[3];
	//axis[0] = dRandReal()*2.0 - 1.0;		//Define the axis.
	//axis[1] = dRandReal()*2.0 - 1.0;
	//axis[2] = dRandReal()*2.0 - 1.0;
	//dReal angle = dRandReal()*10.0 - 5.0;	//Define the angle.
	//dRFromAxisAndAngle( Orient,	axis[0], axis[1], axis[2], angle );		//Create rotation matrix.
	////dBodySetRotation( Object.Body, Orient );			//Assigns orientation to body.
	//dBodySetData( Object.Body, (void *)0 );	//No data associated with data.

	////Assigning the box mass that matches the desired geometry (a box). First, create a mass struct.
	//dMass M;
	//dReal sides[] = {2.0, 2.0, 0.5};		//Side lenghts along the x, y, and z axes.
	//dMassSetBox( &M, 0.5, sides[0], sides[1], sides[2] );	//Use automatic generation of moment of inertia and mass.
	//dBodySetMass( Object.Body, &M );		//Apply mass distribution to rigid body.

	////Creating the actual geometry for the object.
	//Object.Geom[0] = dCreateBox( Space, sides[0], sides[1], sides[2] );

	////Associate the geometry with the object's dynamics body.
	//dGeomSetBody( Object.Geom[0], Object.Body );

	////Create rod
	//Rod.Body = dBodyCreate(World);
	//dBodySetPosition(Rod.Body, 0, 6.7, 0);
	//dMatrix3 Orient2;
	//dReal axis2[3];
	//axis[0] = 1;
	//axis[1] = axis[2] = 0;
	//dReal angle2 = 90;
	//dRFromAxisAndAngle(Orient2, axis2[0], axis2[1], axis2[2], angle2);

	//dBodySetRotation(Rod.Body, Orient2);
	//dBodySetLinearVel(Rod.Body, 0, 0, 0);

	//dBodySetData(Rod.Body, (void *)0);
	//dMass M2;
	//dMassSetCylinder( &M2, 0.5, 3, 1, 4 );
	//dBodySetMass(Rod.Body, &M2);
	//Rod.Geom[0] = dCreateCylinder(Space, 1, 4);
	//dGeomSetBody(Rod.Geom[0], Rod.Body);

	//Create body
	body.Body = dBodyCreate(World);
	dBodySetPosition(body.Body, 0, 5, 0);
	dMatrix3 Orient3;
	dRFromAxisAndAngle(Orient3, 0, 0, 1, PI/2);
	//dBodySetRotation(body.Body, Orient3);
	dBodySetLinearVel(body.Body, 0, 0, 0);
	dBodySetData(body.Body, (void *)0);
	dMass bodyMass;
	dMassSetCapsule(&bodyMass, 1, 3, 1, 5);
	body.Geom[0] = dCreateCapsule(Space, 1, 5);
	dGeomSetBody(body.Geom[0], body.Body);

	dMatrix3 rLegOrient;
	dMatrix3 lLegOrient;
	dMatrix3 brLegOrient;

	dRFromEulerAngles(rLegOrient, PI / 2, -PI / 3, 0);
	dRFromEulerAngles(lLegOrient, PI / 2, PI / 3, 0);


	createFrontRightLeg(rLegOrient);
	createMiddleRightLeg(rLegOrient);
<<<<<<< HEAD
<<<<<<< HEAD
	//createBackRightLeg(brLegOrient);
=======
	createBackRightLeg(brLegOrient);
>>>>>>> d691c16fa799ca013cc0e75955b9dae48719dc15
=======
	createBackRightLeg(brLegOrient);
>>>>>>> d691c16fa799ca013cc0e75955b9dae48719dc15
	createFrontLeftLeg(lLegOrient);
	createMiddleLeftLeg(lLegOrient);
	//createBackLeftLeg(lLegOrient);
	createBackLegs();


	/*Joint2 = dJointCreateFixed(World, jointgroup);
	dJointAttach(Joint2, body.Body, 0);
	dJointSetFixed(Joint2);*/


	/*Joint = dJointCreateHinge(World, jointgroup);
	dJointAttach(Joint, Object.Body, Rod.Body);
	dJointSetHingeAnchor(Joint, 0, 10 - sides[0] / 2, 0);
	dJointSetHingeAxis(Joint, 1, 0, 0);*/

	


}


/*******************************************************************************
Function to clean the ODE system.
*******************************************************************************/
void closeODE()
{
	dJointGroupDestroy(jointgroup);
	dJointGroupDestroy( ContactGroup );		//Remove the contact joints.
	dSpaceDestroy( Space );					//Remove the space and all of its geoms.
	dWorldDestroy( World );					//Destroy all bodies and joints (not in a group).
}

/*******************************************************************************
Function to handle potential collisions between geometries.
*******************************************************************************/
static void nearCallBack( void *data, dGeomID o1, dGeomID o2 )
{
	int I;					//A temporary index for each contact.
	const int MAX_CONTACTS = 3;

	//Get the dynamics body for each potentially colliding geometry.
	dBodyID b1 = dGeomGetBody( o1 );
	dBodyID b2 = dGeomGetBody( o2 );

	//Create an array of dContact objects to hold the contact joints.
	dContact contacts[ MAX_CONTACTS ];

	//Initialize contact structures.
	for( I = 0; I < MAX_CONTACTS; I++ )
	{
		contacts[I].surface.mode = dContactBounce | dContactSoftCFM;
		if (o1 == frLeg.Geom[0] || o2 == frLeg.Geom[0] ||
			o1 == flLeg.Geom[0] || o2 == flLeg.Geom[0] ||
			o1 == brLeg.Geom[0] || o2 == brLeg.Geom[0] ||
			o1 == blLeg.Geom[0] || o2 == blLeg.Geom[0]) {
			contacts[I].surface.mu = 0;

		}
		else {
			contacts[I].surface.mu = 1;
		}
		//contacts[I].surface.mu = 1;
		contacts[I].surface.mu2 = 0;
		contacts[I].surface.bounce = 0.01;
		contacts[I].surface.bounce_vel = 0.1;
		contacts[I].surface.soft_cfm = 0.01;
	}

	//Now, do the actual collision test, passing as parameters the address of
	//a dContactGeom structure, and the offset to the next one in the list.
	if( int numc = dCollide( o1, o2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContact) ) )
	{
		//Add contacts detected to the contact joint group.
		for( I = 0; I < numc; I++ )
		{
			//Add contact joint by knowing its world, to a contact group. The last parameter
			//is the contact itself.
			dJointID c = dJointCreateContact( World, ContactGroup, contacts + I );
			dJointAttach( c, b1, b2 );		//Attach two bodies to this contact joint.
		}
	}
}

/*******************************************************************************
Function to perform the simulation loop for ODE.
*******************************************************************************/
void simulationLoop()
{
	//First, determine which geoms inside the space are potentially colliding.
	//The nearCallBack function will be responsible for analizing those potential collisions.
	//The second parameter indicates that no user data is being sent the callback routine.
	dSpaceCollide( Space, 0, &nearCallBack );

	//Next, advance the simulation, based on the step size given.
	dWorldStep( World, simulationStep );

	//Then, remove the temporary contact joints.
	dJointGroupEmpty( ContactGroup );

	//At this point, all geometries have been updated, so they can be drawn from display().
}

/*******************************************************************************
Function to transform an ODE orientation matrix into an OpenGL transformation
matrix (orientation plus position).
p -> a vector of three elements.
R -> the rotation matrix, in row vector format, with 12 elements.
M -> the resulting matrix, in vector format, with 16 elements.
*******************************************************************************/
void ODEToOpenGLMatrix( const dReal* p, const dReal* R, dReal* M)
{
	M[0]  = R[0]; M[1]  = R[4]; M[2]  = R[8];  M[3]  = 0;
    M[4]  = R[1]; M[5]  = R[5]; M[6]  = R[9];  M[7]  = 0;
    M[8]  = R[2]; M[9]  = R[6]; M[10] = R[10]; M[11] = 0;
    M[12] = p[0]; M[13] = p[1]; M[14] = p[2];  M[15] = 1;
}

/*******************************************************************************
Function to render a box, given it sides length, position and orientation.
*******************************************************************************/
void renderBox( const dReal sides[3], const dReal position[3], const dReal orientation[12] )
{
	glPushMatrix();					//Save current ModelView.
	
	dReal Matrix[16];				//The OpenGL version of the transformation matrix.
	ODEToOpenGLMatrix( position, orientation, Matrix );
	glMultMatrixd( Matrix );
	glScaled( sides[0], sides[1], sides[2] );	//Scale to have the right measure in sides.
	GDrawing::setColor( 0.5, 0.6, 0.7 );
	GDrawing::drawCube();

	glPopMatrix();					//Restore ModelView.
}

void renderCylinder(const dReal radius, const dReal length, const dReal position[3], const dReal orientation[12])
{
	glPushMatrix();					//Save current ModelView.

	dReal Matrix[16];				//The OpenGL version of the transformation matrix.
	//dReal  newPos[3] = { position[0], position[1] - length/2, position[2] };
	ODEToOpenGLMatrix(position, orientation, Matrix);

	glMultMatrixd(Matrix);
	glTranslated(0, 0, -length / 2);
	glScaled(radius, radius, length);	//Scale to have the right measure in sides.
	
	GDrawing::setColor(0.5, 0.6, 0.7);
	GDrawing::drawCylinder();

	glPopMatrix();					//Restore ModelView.

}

void renderCapsule(const dReal radius, const dReal length, const dReal position[3], const dReal orientation[12])
{
	glPushMatrix();					//Save current ModelView.

	dReal Matrix[16];				//The OpenGL version of the transformation matrix.
	//dReal  newPos[3] = { position[0], position[1] - length/2, position[2] };
	ODEToOpenGLMatrix(position, orientation, Matrix);

	glMultMatrixd(Matrix);

	dReal sphereR = .99 * radius;
	glPushMatrix();					//Save current ModelView.
	glTranslated(0, 0, length / 2);
	glScaled(sphereR, sphereR, sphereR);
	GDrawing::setColor(0.5, 0.6, 0.7);
	GDrawing::drawSphere();
	glPopMatrix(); //unscaled and at top

	glTranslated(0, 0, -length / 2);
	glPushMatrix();
	glScaled(radius, radius, length);	//Scale to have the right measure in sides.
	GDrawing::drawCylinder();
	glPopMatrix(); //unscaled and at center

	//glTranslated(0, 0, -length / 2);
	glScaled(sphereR, sphereR, sphereR);
	GDrawing::drawSphere();

	glPopMatrix();					//Restore ModelView.




}

/*******************************************************************************
Function to draw a geometry object.
*******************************************************************************/
void drawGeom( dGeomID g )
{
	if( !g )		//If the geometry object is missing, end the function.
		return;

	const dReal *position;		//Define pointers to internal positions and orientations.
	const dReal *orientation;	//Pointers to constant objects (so the objects will not change).

	position = dGeomGetPosition( g );		//Then, get the geometry position.
	orientation = dGeomGetRotation( g );	//And get existing geometry orientation.

	int type = dGeomGetClass( g );				//Get the type of geometry.
	
	if( type == dBoxClass )						//Is it a box?
	{
		
		dReal sides[3];
		dGeomBoxGetLengths( g, sides );				//Get length of sides.
		renderBox( sides, position, orientation );	//Render the actual box in environment.
	}
	else if (type == dCylinderClass)
	{
		dReal radius, length;
		dGeomCylinderGetParams(g, &radius, &length);
		renderCylinder(radius, length, position, orientation);
	}
	else if (type == dCapsuleClass)
	{
		dReal radius, length;
		dGeomCapsuleGetParams(g, &radius, &length);
		renderCapsule(radius, length, position, orientation);
	}
}

/*******************************************************************************
Function to reset the zoom-in zoom-out effect.
*******************************************************************************/
void resetArcball()
{
	Ball_Init(g_arcBall);
	Ball_Place(g_arcBall, qOne, 0.75);
}

/*******************************************************************************
Function that gets called for any keypresses.
*******************************************************************************/
void myKey(unsigned char key, int x, int y)
{
	float time;
	switch (key) 
	{
		case 'i':
			dJointAddUniversalTorques(mlLegJoint, -50, 0);
			dJointAddUniversalTorques(mrLegJoint, -50, 0);
		break;
		case 'o':
			dJointAddUniversalTorques(mlLegJoint, -25, 100);
			dJointAddUniversalTorques(mrLegJoint, -25, -100);
		break;
		case 'p':
			dJointAddUniversalTorques(mlLegJoint, 50, -50);
			dJointAddUniversalTorques(mrLegJoint, 50, 50);
		break;
		case 'g':
			/*dJointAddHingeTorque(mrLegJoint, 100);
			dJointAddHingeTorque(mlLegJoint, -100);*/
			break;
		case 'f':
			//dJointAddHingeTorque(frLegJoint, -100);
			//dJointAddHingeTorque(mrLegJoint, -100);
			//dJointAddHingeTorque(brLegJoint, -100);
			break;
		case 'd':
			//dJointAddHingeTorque(frLegJoint, 100);
			//dJointAddHingeTorque(mrLegJoint, 100);
			//dJointAddHingeTorque(brLegJoint, 100);
			break;
		case 'c':
			//dJointAddHingeTorque(flLegJoint, 100);
			//dJointAddHingeTorque(mlLegJoint, 100);
			//dJointAddHingeTorque(blLegJoint, 100);
			break;
		case 'v':
			//dJointAddHingeTorque(flLegJoint, -100);
			//dJointAddHingeTorque(mlLegJoint, -100);
			//dJointAddHingeTorque(blLegJoint, -100);
			break;
		case 'q':
		case 27:
			exit(0); 
		case 's':
			g_frameSaver.DumpPPM(g_width,g_height);
			break;
		case 'r':
			resetArcball();
			break;
		case 'a':
			g_animate = 1 - g_animate;
			//Reset the timer to point to the current time.		
			time = g_timer.GetElapsedTime();
			g_timer.Reset();
			break;
		case '0':
			//Reset your object.
			break ;
		case 'm':
			if( g_recording == 1 )
			{
				cout << "Frame recording disabled." << endl;
				g_recording = 0;
			}
			else
			{
				cout << "Frame recording enabled." << endl;
				g_recording = 1 ;
			}
			g_frameSaver.Toggle();
			break ;
		case 'h':
		case '?':
			GDrawing::plotInstructions();
			break;
	}
	glutPostRedisplay();
}

/*******************************************************************************
Function that performs most of the OpenGL intialization.
*******************************************************************************/
void myinit(void)
{
    GLfloat ambient[] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
    //GLfloat position[] = { -3.0, 3.0, 3.0, 0.0 };
	GLfloat position[] = { 0.0, 0.0, 30.0, 1.0 };
	GLfloat diffuse2[] = { 0.3, 0.3, 0.3, 1.0 };
    GLfloat specular2[] = { 0.3, 0.3, 0.3, 1.0 };
	GLfloat position2[] = { 0.0, 100.0, 00.0, 1.0 };
    
    GLfloat lmodel_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat local_view[] = { 0.0 };

    /**** Set lighting parameters ****/
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightf(GL_LIGHT0, GL_SHININESS, 100) ;
    glLightfv(GL_LIGHT0, GL_POSITION, position);
	
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE) ;

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse2);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular2);
    glLightfv(GL_LIGHT1, GL_POSITION, position2);
	glLightf(GL_LIGHT1, GL_SHININESS, 500) ;
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

	glPixelStorei(GL_PACK_ALIGNMENT,1);
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	glShadeModel(GL_SMOOTH);

	g_arcBall = new BallData;
	Ball_Init(g_arcBall);
	Ball_Place(g_arcBall,qOne,0.75);
}

/*******************************************************************************
Function that gets called by the event handler to draw the scene.
*******************************************************************************/
void display(void)
{
	//glClearColor (red, green, blue, alpha)
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);		//Set the background color.
	
	//OK, now clear the screen with the background color.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Load initial matrix transformation.
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Locate the camera.
	gluLookAt (g_eye[X], g_eye[Y], g_eye[Z], g_ref[X], g_ref[Y], g_ref[Z], 0.0, 1.0, 0.0);

	HMatrix arcball_rot;
	Ball_Value(g_arcBall,arcball_rot);
	glMultMatrixf((float *)arcball_rot);

	//Scale the scene in response to mouse commands.
	glScalef(g_zoom, g_zoom, g_zoom); 

	////////////////////// Draw the geometries in World ////////////////////////

	drawGeom( Object.Geom[0] );
	drawGeom( Rod.Geom[0] );
	drawGeom(body.Geom[0]);
	drawGeom(middleRightLeg.Geom[0]);
	drawGeom(middleLeftLeg.Geom[0]);
	drawGeom(brLeg.Geom[0]);
	drawGeom(blLeg.Geom[0]);
	drawGeom(frLeg.Geom[0]);
	drawGeom(flLeg.Geom[0]);

	glPushMatrix();						//Draw the collision plane.
	glTranslated( 0.0, -0.05, 0.0 );
	glScaled( 20.0, 0.1, 20.0 );
	GDrawing::setColor( 1.0, 0.2, 0.3 );
	GDrawing::drawCube();
	glPopMatrix();

	////////////////////////////////////////////////////////////////////////////

	glutSwapBuffers();
	if( g_recording == 1)
		g_frameSaver.DumpPPM(g_width,g_height);
}

/*******************************************************************************
Function that handles the window being resized
*******************************************************************************/
void myReshape(int w, int h)
{
	g_width = w;
	g_height = h;

	float asp = (float) w / (float) h ;

	cout << "New aspect ratio " << asp << endl;

	glViewport(0, 0, w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
    
	// This defines the field of view of the camera.
    // Making the first 4 parameters larger will give a larger field of 
	// view, therefore making the objects in the scene appear smaller.
	glOrtho(-asp*15, asp*15, -15, 15, -500,500);

	// Use either of the following functions to set up a perspective view
	//gluPerspective(20,(float) w/(float) h,1,100) ;
	//glFrustum(-1,1,-1,1,4,100);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

    // This sets the virtual camera:
    // gluLookAt( x,y,z,   x,y,z   x,y,z );
    //            camera  look-at camera-up
    //            pos'n    point   vector
    gluLookAt(g_eye[X], g_eye[Y], g_eye[Z],	g_ref[X], g_ref[Y], g_ref[Z], 0, 1, 0);

	HMatrix arcball_rot;
	Ball_Value(g_arcBall,arcball_rot);
	glMultMatrixf((float *)arcball_rot);
}

/*******************************************************************************
Event handler for mouse buttons.
*******************************************************************************/
void myMouseCB(int button, int state, int x, int y)
{
	g_button = button ;
	if( g_button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
	{
		HVect arcball_coords;
		arcball_coords.x = 2.0*(float)x/(float)g_width-1.0;
		arcball_coords.y = -2.0*(float)y/(float)g_height+1.0;
		Ball_Mouse(g_arcBall, arcball_coords) ;
		Ball_Update(g_arcBall);
		Ball_BeginDrag(g_arcBall);

	}
	if( g_button == GLUT_LEFT_BUTTON && state == GLUT_UP )
	{
		Ball_EndDrag(g_arcBall);
		g_button = -1 ;
	}
	if( g_button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN )
	{
		g_previousY = y ;
	}

	// Tell the system to redraw the window.
	glutPostRedisplay();
}

/*******************************************************************************
Event handler for mouse motion.
*******************************************************************************/
void myMotionCB(int x, int y)
{
	if( g_button == GLUT_LEFT_BUTTON )
	{
		HVect arcball_coords;
		arcball_coords.x = 2.0*(float)x/(float)g_width - 1.0 ;
		arcball_coords.y = -2.0*(float)y/(float)g_height + 1.0;
		Ball_Mouse(g_arcBall,arcball_coords);
		Ball_Update(g_arcBall);
		glutPostRedisplay();
	}
	else if( g_button == GLUT_RIGHT_BUTTON )
	{
		if( y - g_previousY > 0 )
			g_zoom  = g_zoom * 1.03;
		else 
			g_zoom  = g_zoom * 0.97;
		g_previousY = y;
		glutPostRedisplay();
	}
}

/*******************************************************************************
Function that hanles the idle time of the OpenGL main loop.
*******************************************************************************/
void idleCB(void)
{
	if( g_animate == 1 )
	{
		//if( g_recording == 0 )
		//	g_time = g_timer.GetElapsedTime() ;
		//else
		//	g_time += 0.033 ; // save at 30 frames per second.
		
		simulationLoop();		//Execute simulation.
		simulationTime += simulationStep;
		cout << "Current time: " << simulationTime << endl;

		glutPostRedisplay() ; 
	}
}

/*******************************************************************************
Main function: calls initialization, then hands over control to the event
handler, which calls display() whenever the screen needs to be redrawn.
*******************************************************************************/
int main(int argc, char** argv) 
{
	initODE();						//Initialize the dynamics world.
	
	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowPosition (0, 0);
	glutInitWindowSize(g_width,g_height);
	glutCreateWindow(argv[0]);

	myinit();

	glutIdleFunc(idleCB) ;
	glutReshapeFunc (myReshape);
	glutKeyboardFunc( myKey );
	glutMouseFunc(myMouseCB);
	glutMotionFunc(myMotionCB);
	GDrawing::plotInstructions();

	glutDisplayFunc(display);
	glutMainLoop();

	g_timer.Reset();
	return 0;         //Never reached
}




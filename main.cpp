
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
#include <vector>

#include "angle_conversions.h"
#include "ODEObject.h"
#include "FoodParticle.h"
#include "Animation.h"

#include "Ball.h"
#include "FrameSaver.h"
#include "Timer.h"
#include "GDrawing.h"

#include <random>
	

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

const int STRLEN = 100;
const double PI = 3.1415926535;
typedef char STR[STRLEN];

#define X 0
#define Y 1
#define Z 2
#define FOOD_COUNT 4

// The eye point and look-at point.
double g_eye[3] = {0.0, 500.0, 10.0};
double g_ref[3] = {0.0, 0.0, 0.0};
double g_time = 0.0 ;
////////////////////////////////////////////////////////////////////////////////

/////////////////////////// ODE Global Variables ///////////////////////////////
dReal simulationTime = 0.0f;
dReal simulationStep = 0.025f;

ODEObject head;
ODEObject body;				//Insect Body
ODEObject frLeg;
ODEObject frontRightOuterLeg;
ODEObject flLeg;
ODEObject frontLeftOuterLeg;
ODEObject middleRightLeg;
ODEObject middleRightOuterLeg;
ODEObject middleLeftLeg;
ODEObject middleLeftOuterLeg;
ODEObject brLeg;
ODEObject backRightOuterLeg;
ODEObject blLeg;
ODEObject backLeftOuterLeg;
dWorldID World;				//Dynamics world.
dSpaceID Space;				//A space that defines collisions.
dJointGroupID jointgroup;   // contact group for the new joint
dJointGroupID ContactGroup;	//Group of contact joints for collision detection/handling.
ODEObject target;

dJointID Joint;
dJointID Joint2;
dJointID frLegJoint;
dJointID froLegJoint;
dJointID flLegJoint;
dJointID floLegJoint;
dJointID mrLegJoint;             // the joint ID
dJointID mroLegJoint;
dJointID mlLegJoint;
dJointID mloLegJoint;
dJointID brLegJoint;
dJointID broLegJoint;
dJointID blLegJoint;
dJointID bloLegJoint;
dJointID invis_box_joint;

std::mt19937 rng_engine;

//*******WARNING**********
//must be same order as food particle vector in Animation.h!!!
//vector<FoodParticle> foodParticles;
//*******WARNING**********
std::vector<ODEObject> objectsToDestroy;

Animator animator(	&head, &body, 
				&mrLegJoint, &mlLegJoint, 
				&mroLegJoint, &mloLegJoint, 
				&brLegJoint, &blLegJoint, 
				1.0, 0.75, 
				1000.0, 2000.0);


/*
=================================================================================
createFixedLeg

	Use parameters to create leg body/geom and attach to body with fixed joint
=================================================================================
*/
void createFixedLeg(ODEObject &leg,
	ODEObject &bodyAttachedTo,
	dJointID& joint,
	dReal xPos, dReal yPos, dReal zPos,
	dReal xRot, dReal yRot, dReal zRot,
	dReal radius,
	dReal length)
{
	dMatrix3 legOrient;
	dRFromEulerAngles(legOrient, xRot, yRot, zRot);

	//position and orientation
	leg.Body = dBodyCreate(World);
	dBodySetPosition(leg.Body, xPos, yPos, zPos);
	dBodySetRotation(leg.Body, legOrient);
	dBodySetLinearVel(leg.Body, 0, 0, 0);
	dBodySetData(leg.Body, (void *)0);

	//mass
	dMass legMass;
	dMassSetCapsule(&legMass, 1, 3, radius, length);
	dBodySetMass(leg.Body, &legMass);

	//geometry
	leg.Geom = dCreateCapsule(Space, radius, length);
	dGeomSetBody(leg.Geom, leg.Body);

	//fixed joint
	joint = dJointCreateFixed(World, jointgroup);
	dJointAttach(joint, bodyAttachedTo.Body, leg.Body);
	dJointSetFixed(joint);
}

#include "vec.h"
#include <cmath>

void createHingeLeg(ODEObject &leg,
	ODEObject &bodyAttachedTo,
	dJointID &joint,
	dReal xPos, dReal yPos, dReal zPos,
	dReal xRot, dReal yRot, dReal zRot,
	dReal radius, dReal length,
	dReal maxAngle, dReal minAngle,
	dReal anchorXPos, dReal anchorYPos, dReal anchorZPos)
{
	dMatrix3 legOrient;
	dRFromEulerAngles(legOrient, xRot, yRot, zRot);

	//position and orientation
	leg.Body = dBodyCreate(World);
	dBodySetPosition(leg.Body, xPos, yPos, zPos);
	dBodySetRotation(leg.Body, legOrient);
	dBodySetLinearVel(leg.Body, 0, 0, 0);
	dBodySetData(leg.Body, (void *)0);

	//mass
	dMass legMass;
	dMassSetCapsule(&legMass, 1, 3, radius, length);
	//dBodySetMass(leg.Body, &legMass);

	//geometry
	leg.Geom = dCreateCapsule(Space, radius, length);
	dGeomSetBody(leg.Geom, leg.Body);

	//hinge joint
	joint = dJointCreateHinge(World, jointgroup);

	//attach and anchor
	dJointAttach(joint, bodyAttachedTo.Body, leg.Body);
	dJointSetHingeAnchor(joint, anchorXPos, anchorYPos, anchorZPos);

	//axes
	dJointSetHingeAxis(joint, 0, 1, 0);

	//Max and min angles
	dJointSetHingeParam(joint, dParamHiStop, maxAngle);
	dJointSetHingeParam(joint, dParamLoStop, minAngle);

}





/*
=================================================================================
createUniversalLeg

Use parameters to create leg body/geom and attach to body with universal joint

**Warning**
mass is not set
=================================================================================
*/
void createUniversalLeg(ODEObject &leg,
	ODEObject &bodyAttachedTo,
	dJointID& joint,
	dReal xPos, dReal yPos, dReal zPos,
	dReal xRot, dReal yRot, dReal zRot,
	dReal radius, dReal length,
	dReal maxAngle,	dReal minAngle,
	dReal anchorXPos, dReal anchorYPos, dReal anchorZPos)
{
	dMatrix3 legOrient;
	dRFromEulerAngles(legOrient, xRot, yRot, zRot);

	//position and orientation
	leg.Body = dBodyCreate(World);
	dBodySetPosition(leg.Body, xPos, yPos, zPos);
	dBodySetRotation(leg.Body, legOrient);
	dBodySetLinearVel(leg.Body, 0, 0, 0);
	dBodySetData(leg.Body, (void *)0);

	//mass
	dMass legMass;
	dMassSetCapsule(&legMass, 1, 3, radius, length);
	//dBodySetMass(leg.Body, &legMass);

	//geometry
	leg.Geom = dCreateCapsule(Space, radius, length);
	dGeomSetBody(leg.Geom, leg.Body);

	//universal joint
	joint = dJointCreateUniversal(World, jointgroup);

	//attach and anchor
	dJointAttach(joint, bodyAttachedTo.Body, leg.Body);
	dJointSetUniversalAnchor(joint, anchorXPos, anchorYPos, anchorZPos);

	//axes
	dJointSetUniversalAxis1(joint, 0, 0, 1);
	dJointSetUniversalAxis2(joint, 0, 1, 0);

	//Max and min angles
	dJointSetUniversalParam(joint, dParamHiStop, maxAngle);
	dJointSetUniversalParam(joint, dParamLoStop, minAngle);
	dJointSetUniversalParam(joint, dParamHiStop2, maxAngle);
	dJointSetUniversalParam(joint, dParamLoStop2, minAngle);
}


/*
=================================================================================
createUniversalLeg

Use parameters to create leg body/geom and attach to body with universal joint

**Warning**
mass is not set
=================================================================================
*/
void createUniversalSquareLeg(ODEObject &leg,
	dJointID& joint,
	dReal xPos, dReal yPos, dReal zPos,
	dReal xRot, dReal yRot, dReal zRot,
	dReal sides[3],
	dReal maxAngle, dReal minAngle,
	dReal anchorXPos, dReal anchorYPos, dReal anchorZPos)
{
	dMatrix3 legOrient;
	dRFromEulerAngles(legOrient, xRot, yRot, zRot);

	//position and orientation
	leg.Body = dBodyCreate(World);
	dBodySetPosition(leg.Body, xPos, yPos, zPos);
	dBodySetRotation(leg.Body, legOrient);
	dBodySetLinearVel(leg.Body, 0, 0, 0);
	dBodySetData(leg.Body, (void *)0);

	//mass
	dMass legMass;
	dMassSetBox(&legMass, .5, sides[0], sides[1], sides[2]);
	dBodySetMass(leg.Body, &legMass);

	//geometry
	leg.Geom = dCreateBox(Space, sides[0], sides[1], sides[2]);
	dGeomSetBody(leg.Geom, leg.Body);

	//universal joint
	joint = dJointCreateUniversal(World, jointgroup);

	//attach and anchor
	dJointAttach(joint, body.Body, leg.Body);
	dJointSetUniversalAnchor(joint, anchorXPos, anchorYPos, anchorZPos);

	//axes
	dJointSetUniversalAxis1(joint, 0, 0, 1);
	dJointSetUniversalAxis2(joint, 0, 1, 0);

	//Max and min angles
	dJointSetUniversalParam(joint, dParamHiStop, maxAngle);
	dJointSetUniversalParam(joint, dParamLoStop, minAngle);
	dJointSetUniversalParam(joint, dParamHiStop2, maxAngle);
	dJointSetUniversalParam(joint, dParamLoStop2, minAngle);
}


/*
=================================================================================
createFrontLegs

create front two legs of insect with fixed joints
=================================================================================
*/
void createFrontLegs()
{
	const dReal xPos = -1;
	const dReal yPos = 3.7;
	const dReal zPos = -3.2;
	const dReal xRot = PI / 3;
	const dReal yRot = -PI/8;
	const dReal zRot = 0;
	const dReal radius = 0.15;
	const dReal length = 2;

	const dReal xPosOuter = -1.5;
	const dReal yPosOuter = 2.5;
	const dReal zPosOuter = -3.7;
	const dReal xRotOuter = PI/2;
	const dReal yRotOuter = 0;
	const dReal zRotOuter = 0;
	const dReal radiusOuter = 0.15;
	const dReal lengthOuter = 1;

	//create front right leg
	createFixedLeg(frLeg, body, frLegJoint, xPos, yPos, zPos, xRot, yRot, zRot, radius, length);
	createFixedLeg(frontRightOuterLeg, frLeg, froLegJoint, xPosOuter, yPosOuter, zPosOuter, xRotOuter, yRotOuter, zRotOuter, radiusOuter, lengthOuter);
	//create front left leg
	createFixedLeg(flLeg, body, flLegJoint, -xPos, yPos, zPos, xRot, -yRot, zRot, radius, length);
	createFixedLeg(frontLeftOuterLeg, flLeg, floLegJoint, -xPosOuter, yPosOuter, zPosOuter, xRotOuter, -yRotOuter, zRotOuter, radiusOuter, lengthOuter);
	
}

/*
=================================================================================
void createMiddleLegs

create middle two legs of insect with universal joints

**Warning**
Masses not set due to createUniversalLeg function
=================================================================================
*/
void createMiddleLegs()
{
	const dReal xPos = -3.2;
	const dReal yPos = 3.5;
	const dReal zPos = 0;
	const dReal xRot = PI / 2;
	const dReal yRot = -PI / 3;
	const dReal zRot = 0;
	const dReal radius = 0.15;
	const dReal length = 5;
	const dReal maxAngle = PI / 4;
	const dReal minAngle = -PI / 4;
	const dReal anchorXPos = -1;
	const dReal anchorYPos = 4.7;
	const dReal anchorZPos = 0;
	dReal sides[3];
	sides[0] = 5;
	sides[1] = sides[2] = .15;

	//create middle right leg
	createUniversalLeg(middleRightLeg,
		body,
		mrLegJoint,
		xPos, yPos, zPos,
		xRot, yRot, zRot,
		radius, length,
		maxAngle, minAngle,
		anchorXPos, anchorYPos, anchorZPos);
	//createUniversalSquareLeg(middleRightLeg,
	//	mrLegJoint,
	//	xPos, yPos, zPos,
	//	xRot, yRot, zRot,
	//	sides,
	//	maxAngle, minAngle,
	//	anchorXPos, anchorYPos, anchorZPos);

	//create middle left leg
	createUniversalLeg(middleLeftLeg,
		body,
		mlLegJoint,
		-xPos, yPos, zPos,
		xRot, -yRot, zRot,
		radius, length,
		maxAngle, minAngle,
		-anchorXPos, anchorYPos, anchorZPos);

	const dReal xPosOuter = -8;
	const dReal yPosOuter = 2;
	const dReal zPosOuter = 0;
	const dReal xRotOuter = 0;
	const dReal yRotOuter = PI / 2;
	const dReal zRotOuter = 0;
	const dReal radiusOuter = 0.15;
	const dReal lengthOuter = 5;
	const dReal maxAngleOuter = PI / 4;
	const dReal minAngleOuter = -PI / 4;
	const dReal anchorXPosOuter = -5.4;
	const dReal anchorYPosOuter = 2.0;
	const dReal anchorZPosOuter = 0;

	createHingeLeg(middleRightOuterLeg,
		middleRightLeg,
		mroLegJoint,
		xPosOuter, yPosOuter, zPosOuter,
		xRotOuter, yRotOuter, zRotOuter,
		radiusOuter, lengthOuter,
		maxAngleOuter, minAngleOuter,
		anchorXPosOuter, anchorYPosOuter, anchorZPosOuter);

	//// target
	//dMatrix3 Orient3;
	//target.Body = dBodyCreate(World);
	//dBodySetPosition(target.Body, anchorXPosOuter, anchorYPosOuter, anchorZPosOuter);
	//dRFromEulerAngles(Orient3, 0.0, 0.0, 0.0);
	//dBodySetRotation(target.Body, Orient3);
	//dBodySetLinearVel(target.Body, 0.0, 0.0, 0.0);
	//dBodySetData(target.Body, (void *)0);
	//dMass targetMass;
	//dMassSetBox(&targetMass, 1.0, 1.0, 1.0, 1.0);
	//dBodySetMass(target.Body, &targetMass);
	//target.Geom = dCreateBox(Space, 1.0, 1.0, 1.0);
	//dGeomSetBody(target.Geom, target.Body);

	createHingeLeg(middleLeftOuterLeg,
		middleLeftLeg,
		mloLegJoint,
		-xPosOuter, yPosOuter, zPosOuter,
		xRotOuter, -yRotOuter, zRotOuter,
		radiusOuter, lengthOuter,
		maxAngleOuter, minAngleOuter,
		-anchorXPosOuter, anchorYPosOuter, anchorZPosOuter);

	/*createFixedLeg(middleLeftOuterLeg,
	middleLeftLeg,
	mloLegJoint,
	-xPosOuter, yPosOuter, zPosOuter,
	xRotOuter, -yRotOuter, zRotOuter,
	radiusOuter, lengthOuter);*/

	//createFixedLeg(middleRightOuterLeg,
	//	middleRightLeg,
	//	mroLegJoint,
	//	xPosOuter, yPosOuter, zPosOuter,
	//	xRotOuter, yRotOuter, zRotOuter,
	//	radiusOuter, lengthOuter);



	/*createUniversalSquareLeg(middleLeftLeg,
	mlLegJoint,
	3, 1, 0,
	0, 0, 0,
	sides,
	maxAngle, minAngle,
	0, 1, 0);*/

	/*createUniversalSquareLeg(middleLeftLeg,
	mlLegJoint,
	-xPos, yPos, zPos,
	xRot, -yRot, zRot,
	sides,
	maxAngle, minAngle,
	1.25, anchorYPos, anchorZPos);*/
}

/*
=================================================================================
void createBackLegs

create back two legs of insect with fixed joints
=================================================================================
*/
void createBackLegs()
{
	const dReal xPos = -2.4;
	const dReal yPos = 3.3;
	const dReal zPos = 3;
	const dReal xRot = -PI / 4;
	const dReal yRot = PI / 5;
	const dReal zRot = 0;
	const dReal radius = 0.15;
	const dReal length = 5;

	const dReal maxAngle = PI / 8;
	const dReal minAngle = -PI / 8;
	const dReal anchorXPos = -1;
	const dReal anchorYPos = 4.8;
	const dReal anchorZPos = 1.5;

	//create back right leg
	//createFixedLeg(brLeg, body, brLegJoint, xPos, yPos, zPos, xRot, yRot, zRot, radius, length);
	createUniversalLeg(brLeg, body, brLegJoint, xPos, yPos, zPos, xRot, yRot, zRot, radius, length,
		maxAngle, minAngle, anchorXPos, anchorYPos, anchorZPos
		);
	//create back left leg
	//createFixedLeg(blLeg, body, blLegJoint, -xPos, yPos, zPos, xRot, -yRot, zRot, radius, length);
	createUniversalLeg(blLeg, body, blLegJoint, -xPos, yPos, zPos, xRot, -yRot, zRot, radius, length,
		maxAngle, minAngle, -anchorXPos, anchorYPos, anchorZPos
		);
	//const dReal maxAngle = PI / 4;
	//const dReal minAngle = -PI / 4;
	//const dReal anchorXPos = -1;
	//const dReal anchorYPos = 4.7;
	//const dReal anchorZPos = 0;
	//dReal sides[3];
	//sides[0] = 5;
	//sides[1] = sides[2] = .15;

	//createUniversalLeg_BACK( brLeg, body, brLegJoint, xPos, yPos, zPos, xRot, yRot, zRot, radius, length, maxAngle, minAngle, anchorXPos, anchorYPos, anchorZPos );
	//createUniversalLeg_BACK( blLeg, body, blLegJoint, -xPos, yPos, zPos, xRot, -yRot, zRot, radius, length, maxAngle, minAngle, anchorXPos, anchorYPos, anchorZPos );

	const dReal xPosOuter = -4.4;
	const dReal yPosOuter = 1.8;
	const dReal zPosOuter = 7.0;
	const dReal xRotOuter = 0.0;
	const dReal yRotOuter = PI / 16;
	const dReal zRotOuter = 0.0;
	const dReal radiusOuter = 0.15;
	const dReal lengthOuter = 5.0;

	createFixedLeg(backLeftOuterLeg,
		blLeg,
		bloLegJoint,
		-xPosOuter, yPosOuter, zPosOuter,
		xRotOuter, -yRotOuter, zRotOuter,
		radiusOuter, lengthOuter);

	createFixedLeg(backRightOuterLeg,
		brLeg,
		broLegJoint,
		xPosOuter, yPosOuter, zPosOuter,
		xRotOuter, yRotOuter, zRotOuter,
		radiusOuter, lengthOuter);

}


void createInvisibleHead( float* pos )
{
	dMatrix3 head_orientation;
	dRFromEulerAngles(head_orientation, 0.0, 0.0, 0.0);

	//position and orientation
	head.Body = dBodyCreate(World);
	dBodySetPosition(head.Body, pos[ 0 ], pos[ 1 ], pos[ 2 ]);
	dBodySetRotation(head.Body, head_orientation);
	dBodySetLinearVel(head.Body, 0, 0, 0);
	dBodySetData(head.Body, (void *)0);

	//mass
	dMass head_mass;
	dMassSetBox(&head_mass, 1.0, 1.0, 1.0, 1.0);
	dBodySetMass(head.Body, &head_mass);

	//geometry
	head.Geom = dCreateBox(Space, 1.0, 1.0, 1.0);
	dGeomSetBody(head.Geom, head.Body);

	//fixed joint
	invis_box_joint = dJointCreateFixed(World, jointgroup);
	dJointAttach(invis_box_joint, body.Body, head.Body);
	dJointSetFixed(invis_box_joint);
}

void addFoodParticle( dReal position[3], int food_prize, dWorldID* world, dSpaceID* space )
{
	animator.foodParticles.push_back( FoodParticle( position, food_prize, *world, *space ) );
}

void addRandomFoodParticle( int food_prize, dWorldID* world, dSpaceID* space )
{
	dReal position[3] = { 0.0f };

	position[0] = ( static_cast< int >( rng_engine() & 0x7FFFFFFF ) % 101 ) - 50;
	position[1] = 20.0f;
	position[2] = ( static_cast< int >( rng_engine() & 0x7FFFFFFF ) % 101 ) - 50;

	animator.foodParticles.push_back( FoodParticle( position, food_prize, *world, *space ) );
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

	dWorldSetGravity( World, 0.0, -9.81, 0 );	//Add gravity to this World.

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

	const dReal xPos = 0;
	const dReal yPos = 5;
	const dReal zPos = 0;
	const dReal xRot = 0;
	const dReal yRot = 0;
	const dReal zRot = 0;
	const dReal radius = .75;
	const dReal length = 4;
	const dReal sides[3] = { 2, 2, 2 };
	

	//Create body
	body.Body = dBodyCreate(World);
	dBodySetPosition(body.Body, xPos, yPos, zPos);
	dMatrix3 Orient3;
	//dRFromAxisAndAngle(Orient3, 0, 0, 1, 3.14/4);
	dRFromEulerAngles(Orient3, xRot, yRot, zRot);
	//dRFromEulerAngles(Orient3, 0, 0, 0);
	dBodySetRotation(body.Body, Orient3);
	dBodySetLinearVel(body.Body, 0, 0, 0);
	dBodySetData(body.Body, (void *)0);
	dMass bodyMass;
	dMassSetCapsule(&bodyMass, 1.0, 3, radius, length);
	//dMassSetBox(&bodyMass, 10, sides[0], sides[1], sides[2]);
	dBodySetMass(body.Body, &bodyMass);
	body.Geom = dCreateCapsule(Space, radius, length);
	//body.Geom = dCreateBox(Space, sides[0], sides[1], sides[2]);
	dGeomSetBody(body.Geom, body.Body);

	float head_pos[ 3 ] = { 0.0f, 5.0f, -1.0f };
	createInvisibleHead( head_pos );
	
	createFrontLegs();
	createMiddleLegs();
	createBackLegs();

	rng_engine.seed( std::chrono::duration_cast< std::chrono::microseconds >( std::chrono::system_clock::now().time_since_epoch() ).count() );

	const int foodPrize = 10;

	dReal position[3] = { 10.0f, 0.0f, -20.0f };
	addFoodParticle( position, foodPrize, &World, &Space );
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

bool areGeomIDsOfLegAndSoughtFoodParticle(dGeomID o1, dGeomID o2) {

	dGeomID foodParticleGeomID = animator.foodParticles[animator.getIndexOfFoodParticleSought()].odeObject.Geom;

	return o1 == frontLeftOuterLeg.Geom && o2 == foodParticleGeomID ||
		o1 == foodParticleGeomID && o2 == frontLeftOuterLeg.Geom ||
		o1 == frontRightOuterLeg.Geom && o2 == foodParticleGeomID ||
		o1 == foodParticleGeomID && o2 == frontRightOuterLeg.Geom ||
		o1 == frLeg.Geom && o2 == foodParticleGeomID ||
		o1 == foodParticleGeomID && o2 == frLeg.Geom ||
		o1 == flLeg.Geom && o2 == foodParticleGeomID ||
		o1 == foodParticleGeomID && o2 == flLeg.Geom;
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
		if (o1 == frLeg.Geom || o2 == frLeg.Geom ||
			o1 == frontRightOuterLeg.Geom || o2 == frontRightOuterLeg.Geom ||
			o1 == flLeg.Geom || o2 == flLeg.Geom ||
			o1 == frontLeftOuterLeg.Geom || o2 == frontLeftOuterLeg.Geom ||
			o1 == brLeg.Geom || o2 == brLeg.Geom ||
			o1 == backRightOuterLeg.Geom || o2 == backRightOuterLeg.Geom ||
			o1 == blLeg.Geom || o2 == blLeg.Geom ||
			o1 == backLeftOuterLeg.Geom || o2 == backLeftOuterLeg.Geom )
		{
			if ( o1 == backLeftOuterLeg.Geom || o2 == backLeftOuterLeg.Geom )
			{
				contacts[I].surface.mu = animator.back_left_outer_leg_friction;
			}
			else if ( o1 == backRightOuterLeg.Geom || o2 == backRightOuterLeg.Geom )
			{
				contacts[I].surface.mu = animator.back_right_outer_leg_friction;
			}
			else
			{
				contacts[I].surface.mu = 40.0;
			}
		}
		else
		{
			contacts[I].surface.mu = 30.0;
		}

		if (animator.getIndexOfFoodParticleSought() > -1)
		{
			auto temp_idx = animator.getIndexOfFoodParticleSought();
			dGeomID foodParticleGeomID = animator.foodParticles[temp_idx].odeObject.Geom;

			if (areGeomIDsOfLegAndSoughtFoodParticle(o1, o2)) {

				temp_idx = animator.getIndexOfFoodParticleSought();
				objectsToDestroy.push_back(animator.foodParticles[temp_idx].odeObject);

				animator.current_energy_level += animator.foodParticles[temp_idx].prize;

				animator.removeSoughtFoodParticle();

				if (animator.foodParticles.size() == 0)
				{
					//animator.active = false;
				}	
			}
			
		}

		//contacts[I].surface.mu = 1;
		//contacts[I].surface.mu2 = 10;
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

	int dummy2 = 0;
}


/*******************************************************************************
Function to transform an ODE orientation matrix into an OpenGL transformation
matrix (orientation plus position).
p -> a vector of three elements.
R -> the rotation matrix, in row vector format, with 12 elements.
M -> the resulting matrix, in vector format, with 16 elements.
*******************************************************************************/
void ODEToOpenGLMatrix(const dReal* p, const dReal* R, dReal* M)
{
	M[0] = R[0]; M[1] = R[4]; M[2] = R[8];  M[3] = 0;
	M[4] = R[1]; M[5] = R[5]; M[6] = R[9];  M[7] = 0;
	M[8] = R[2]; M[9] = R[6]; M[10] = R[10]; M[11] = 0;
	M[12] = p[0]; M[13] = p[1]; M[14] = p[2];  M[15] = 1;
}

/*******************************************************************************
Function to render a box, given it sides length, position and orientation.
*******************************************************************************/
void renderBox(const dReal sides[3], const dReal position[3], const dReal orientation[12], int colored = 0)
{
	glPushMatrix();					//Save current ModelView.

	dReal Matrix[16];				//The OpenGL version of the transformation matrix.
	ODEToOpenGLMatrix(position, orientation, Matrix);
	glMultMatrixd(Matrix);
	glScaled(sides[0], sides[1], sides[2]);	//Scale to have the right measure in sides.
	
	if ( colored == 0 ) 
		GDrawing::setColor(0.1f, 0.1f, 0.1f);
	else if ( colored == 1 )
		GDrawing::setColor(1.0f, 1.0f, 0.0f);
	else if ( colored == 2 )
		GDrawing::setColor(1.0f, 0.0f, 0.0f);
	
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

	GDrawing::setColor(0.25, 0.25, 0.25);
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
	GDrawing::setColor( 66.0f/255.0f, 33.0f/255.0f, 0.0f/255.0f);
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
Function to perform the simulation loop for ODE.
*******************************************************************************/
void simulationLoop()
{
	//First, determine which geoms inside the space are potentially colliding.
	//The nearCallBack function will be responsible for analizing those potential collisions.
	//The second parameter indicates that no user data is being sent the callback routine.
	dSpaceCollide(Space, 0, &nearCallBack);

	//Next, advance the simulation, based on the step size given.
	dWorldStep(World, simulationStep);

	//Then, remove the temporary contact joints.
	dJointGroupEmpty(ContactGroup);

	if (animator.active)
	{
		dMatrix3 orient;
		dRFromEulerAngles(orient, 0.0, 0.0, 0.0);

		const dReal sides[3] = { 1, 1, 1 };

		animator.Move();
	}

	for (auto i = 0; i < objectsToDestroy.size(); ++i)
	{
		const dBodyID foodParticleBodyID = objectsToDestroy[i].Body;
		const dGeomID foodParticleGeomID = objectsToDestroy[i].Geom;
	
		dGeomDestroy(foodParticleGeomID);
		dBodyDestroy(foodParticleBodyID);	
	}

	for (auto i = objectsToDestroy.size(); i > 0; --i){
		
		objectsToDestroy.erase(objectsToDestroy.begin() + i - 1);

	}

	//animator.updateEnergyLevel();
	animator.dealWithBoundries();

	if ( animator.starving )
	{
		animator.seekFoodParticleWithHighestScore();
	
		//printf( "# known food particles = %d\n", animator.known_target_counter );
		//printf( "\t\ttarget = %f.3, %f.3\n\n", animator.target_position[0], animator.target_position[2] );
	}

	// FOR DEBUGGING
	//for ( const auto& e : animator.foodParticles )
	//{
	//	if ( e.known )
	//	{
	//		const dReal* position = dBodyGetPosition( e.odeObject.Body );
	//
	//		//printf( "x: %f.2, z: %f.2\n", position[0], position[2] );
	//	}
	//}

	//At this point, all geometries have been updated, so they can be drawn from display().
}

/*******************************************************************************
Function to draw a geometry object.
*******************************************************************************/





void drawGeom( dGeomID g, int colored = 0 )
{
	if( !g )		//If the geometry object is missing, end the function.
		return;

	const dReal *position;		//Define pointers to internal positions and orientations.
	const dReal *orientation;	//Pointers to constant objects (so the objects will not change).

	int type = dGeomGetClass( g );				//Get the type of geometry.

	position = dGeomGetPosition( g );		//Then, get the geometry position.
	orientation = dGeomGetRotation( g );	//And get existing geometry orientation.
	
	if( type == dBoxClass )						//Is it a box?
	{		
		dReal sides[3];
		dGeomBoxGetLengths( g, sides );				//Get length of sides.
		renderBox( sides, position, orientation, colored );	//Render the actual box in environment.
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
	const double speed = 1.0;
	switch (key) 
	{
		case 'q':
		case 27:
			exit(0); 
		case 's':
			g_frameSaver.DumpPPM(g_width,g_height);
			break;
		case 'x':
			addRandomFoodParticle( 100, &World, &Space );
			animator.scanForFood( animator.foodParticles.back() );
			break;
		case 'r':
			resetArcball();
			break;
		case '1':
			animator.active = !animator.active;
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
				std::cout << "Frame recording disabled." << std::endl;
				g_recording = 0;
			}
			else
			{
				std::cout << "Frame recording enabled." << std::endl;
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

	//drawGeom( Object.Geom );
	//drawGeom( Rod.Geom );
	drawGeom(body.Geom);
	drawGeom(middleRightLeg.Geom);
	drawGeom(middleLeftLeg.Geom);
	drawGeom(middleLeftOuterLeg.Geom);
	drawGeom(middleRightOuterLeg.Geom);
	drawGeom(brLeg.Geom);
	drawGeom(backRightOuterLeg.Geom);
	drawGeom(blLeg.Geom);
	drawGeom(backLeftOuterLeg.Geom);
	drawGeom(frLeg.Geom);
	drawGeom(frontRightOuterLeg.Geom);
	drawGeom(flLeg.Geom);
	drawGeom(frontLeftOuterLeg.Geom);

	for (auto i = 0; i < animator.foodParticles.size(); ++i)
	{
		drawGeom(animator.foodParticles.at(i).odeObject.Geom, animator.foodParticles.at(i).colored );

	}

	drawGeom(target.Geom);

	glPushMatrix();						//Draw the collision plane.
	glTranslated( 0.0, -0.05, 0.0 );
	glScaled( 140.0, 0.1, 140.0 );
	GDrawing::setColor( 0.4, 0.4, 1.0 );
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

	std::cout << "New aspect ratio " << asp << std::endl;

	glViewport(0, 0, w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
    
	// This defines the field of view of the camera.
    // Making the first 4 parameters larger will give a larger field of 
	// view, therefore making the objects in the scene appear smaller.
	glOrtho(-asp*15, asp*15, -15, 15, -500, 2000);

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
			g_zoom  = g_zoom * 1.03f;
		else 
			g_zoom  = g_zoom * 0.97f;
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
		//cout << "Current time: " << simulationTime << endl;

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



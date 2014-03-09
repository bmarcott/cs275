
#pragma once

#include <ode/ode.h>
#include "ODEObject.h"


struct FoodParticle
{
	bool known;
	dReal distance;

	dReal prize;
	ODEObject odeObject;

	int colored;

	FoodParticle(dReal position[3],
				 dReal prize,
				 dWorldID world, dSpaceID space) : known( false ),
												   distance( -1000000.0f ),
												   prize( prize ),
												   colored( 0 )
	{
		createFoodParticleODEObject(position, world, space);
	};

	void createFoodParticleODEObject(dReal position[3], dWorldID world, dSpaceID space)
	{
		dMatrix3 orient;
		odeObject.Body = dBodyCreate(world);
		dBodySetPosition(odeObject.Body, position[0], position[1], position[2]);
		dRFromEulerAngles(orient, 0.0, 0.0, 0.0);
		dBodySetRotation(odeObject.Body, orient);
		dBodySetLinearVel(odeObject.Body, 0.0, 0.0, 0.0);
		dBodySetData(odeObject.Body, (void *)0);
		dMass foodMass;
		dMassSetBox(&foodMass, 1.0, 1.0, 1.0, 1.0);
		dBodySetMass(odeObject.Body, &foodMass);
		odeObject.Geom = dCreateBox(space, 2.0, 2.0, 2.0);
		dGeomSetBody(odeObject.Geom, odeObject.Body);

	}
};
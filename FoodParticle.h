#pragma once

#include <ode/ode.h>
#include <chrono>
#include <cmath>
#include "ODEObject.h"


struct FoodParticle
{
	dReal prize;
	ODEObject odeObject;

	FoodParticle(dReal position[3], dReal p, dWorldID world, dSpaceID space)
	{
		prize = p;
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
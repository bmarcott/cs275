#pragma once

#include <ode/ode.h>

struct ODEObject
{
	dBodyID Body;		//The dynamics body.
	dGeomID Geom;	//Geometries representing this body.
};
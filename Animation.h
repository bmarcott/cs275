
#pragma once

#include <ode/ode.h>
#include <chrono>
#include <cmath>
#include "angle_conversions.h"
#include "vec.h"

struct Animator {
	bool active;
	
	bool turn_right;
	bool turn_left;
	
	dReal right_leg_friction;
	dReal left_leg_friction;

	dReal position[3];
	dReal max_vert_force;
	dReal max_horiz_force;
	Animator(dJointID* right, dJointID* left,
				  dJointID* back_right, dJointID* back_left,
				  double speed, 
				  dReal max_vert_force, dReal max_horiz_force,
				  dReal position[3])
												  : active(false), turn_right(false), turn_left(false),
												  right_leg_friction(0.0), left_leg_friction(0.0),
												   speed( speed ),
												   max_vert_force(max_vert_force), max_horiz_force(max_horiz_force ),
												   right_leg( right ), left_leg( left ),
												   back_left( back_left ), back_right( back_right ),
												   previous_angle_at_dir_change( 0.0 ), first_time( true )
	{
		this->position[0] = position[0];
		this->position[1] = position[1];
		this->position[2] = position[2];
	}

	void moveDown( void )
	{
		dJointSetUniversalParam(*left_leg, dParamVel, speed);
		dJointSetUniversalParam(*left_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*left_leg, dParamVel2, 0);
		dJointSetUniversalParam(*left_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*right_leg, dParamVel, -speed);
		dJointSetUniversalParam(*right_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*right_leg, dParamVel2, 0);
		dJointSetUniversalParam(*right_leg, dParamFMax2, max_horiz_force);
	}

	void moveUp( void )
	{
		dJointSetUniversalParam(*left_leg, dParamVel, -speed);
		dJointSetUniversalParam(*left_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*left_leg, dParamVel2, 0);
		dJointSetUniversalParam(*left_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*right_leg, dParamVel, speed);
		dJointSetUniversalParam(*right_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*right_leg, dParamVel2, 0);
		dJointSetUniversalParam(*right_leg, dParamFMax2, max_horiz_force);
	}

	void moveBack( void )
	{
		dJointSetUniversalParam(*left_leg, dParamVel, 0);
		dJointSetUniversalParam(*left_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*left_leg, dParamVel2, speed);
		dJointSetUniversalParam(*left_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*right_leg, dParamVel, 0);
		dJointSetUniversalParam(*right_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*right_leg, dParamVel2, -speed);
		dJointSetUniversalParam(*right_leg, dParamFMax2, max_horiz_force);
	}

	void moveForward( void )
	{
		dJointSetUniversalParam(*left_leg, dParamVel, 0);
		dJointSetUniversalParam(*left_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*left_leg, dParamVel2, -speed);
		dJointSetUniversalParam(*left_leg, dParamFMax2, 100);

		dJointSetUniversalParam(*right_leg, dParamVel, 0);
		dJointSetUniversalParam(*right_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*right_leg, dParamVel2, speed);
		dJointSetUniversalParam(*right_leg, dParamFMax2, max_horiz_force);
	}

	void Move( void )
	{
		auto current_h_angle = Radians_To_Degrees( dJointGetUniversalAngle2( *right_leg ) );
		auto current_v_angle = Radians_To_Degrees( dJointGetUniversalAngle1( *left_leg ) );


		static double target_h_angle = 25.0;
		static double target_v_angle = -1.8;

		static bool first_time = true;
		static bool forward = true;

		if ( first_time ) moveForward();

		// once legs reached forward target angle
		if ( forward && current_h_angle > target_h_angle )
		{
			// once legs reached ground, move back
			if ( current_v_angle > target_v_angle )
			{
				decideDirection();
				moveBack();
				first_time = false;
				forward = false;
				target_h_angle = -target_h_angle;
			}
			else
			{
				moveDown();
			}
		}
		else if ( !forward && current_h_angle < target_h_angle )
		{
			// as soon as legs are above the ground, move forward
			if ( current_v_angle < target_v_angle )
			{

				moveForward();
				forward = true;
				target_h_angle = -target_h_angle;
			}
			else
			{
				moveUp();
			}
		}
	}
	






	// true = right, false = left
	void decideDirection(void)
	{
		double x_target = position[0];
		double y_target = position[1];
		double z_target = position[2];

		const dReal* body_position = dGeomGetPosition(body.Geom[0]);
		const dReal* box_position = dGeomGetPosition(invis_box.Geom[0]);		//Then, get the geometry position.

		Angel::vec4 body_orientation(box_position[0] - body_position[0],
			box_position[1] - body_position[1],
			box_position[2] - body_position[2],
			0.0f);
		body_orientation = Angel::normalize(body_orientation);
		body_orientation.w = 0.0;
		Angel::vec4 target_orientation(x_target - body_position[0],
			y_target - body_position[1],
			z_target - body_position[2],
			0.0f);
		target_orientation = Angel::normalize(target_orientation);
		target_orientation.w = 0.0;

		Angel::vec4 normal(0.0f, 1.0f, 0.0f, 0.0f);
		Angel::vec4 normal_to_body_orientation = Angel::cross(body_orientation, normal);
		normal_to_body_orientation = normalize(normal_to_body_orientation);
		normal_to_body_orientation.w = 0.0;
		
		// Beta = angle between normal_to_body_orientation and target_orientation
		auto cosBeta = Angel::dot(target_orientation, normal_to_body_orientation);
		auto absCosBeta = std::abs(cosBeta);
		auto beta = Radians_To_Degrees( std::acos(cosBeta) );
		
		// Alpha = angle between body_orientation and target_orientation
		auto cosAlpha = Angel::dot(target_orientation, body_orientation);
		if (cosAlpha < 0) 
		{
			absCosBeta = 1 - absCosBeta;
		}
		auto frictionScale = std::fmax(absCosBeta, 0.1f);
		
		
	

		/*if (90.0 - beta < 5.0)
		{
			left_leg_friction = 0.0;
			right_leg_friction = 0.0;
		}*/
		if (cosBeta >= 0.0)
		{
			right_leg_friction = 0.0;
			left_leg_friction = frictionScale * 20.0;
		}
		else if (cosBeta < 0.0)
		{
			right_leg_friction = frictionScale * 20.0;
			left_leg_friction = 0.0;
		}
		
		printf("cosine = %f.5\n", cosBeta);
		printf("angle  = %f.5\n\n", beta);
		printf("frictionScale = %f.5\n", frictionScale);
		printf("left leg friction = %f.5\n", left_leg_friction);
		printf("rightq leg friction = %f.5\n\n", right_leg_friction);
	}


private:

	bool first_time;

	dJointID* right_leg;
	dJointID* left_leg;

	dJointID* back_right;
	dJointID* back_left;

	double  speed;

	dReal previous_angle_at_dir_change;

	
};



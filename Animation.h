
#pragma once

#include <ode/ode.h>
#include <chrono>
#include <cmath>
#include "angle_conversions.h"
#include "vec.h"
#include "FoodParticle.h"
#include <algorithm>


enum class Boundry {
	min_X,
	max_X,
	min_Z,
	max_Z
};


struct Animator {
	bool active;
	bool first_time;
	bool starving;
	bool out_of_boundry;

	int known_target_counter;
	dReal max_visible_distance;
	dReal field_of_view;

	dReal right_leg_friction;
	dReal left_leg_friction;

	dReal max_vert_force;
	dReal max_horiz_force;
	
	dReal boundry_length;
	dReal boundry_threshold;

	__int64  energy_consumption_period; // in milliseconds
	int		 energy_consumption_per_period;
	int		 current_energy_level;
	int		 starvation_threshold;
	
	int ndxOfFoodParticleSought;

	ODEObject* head;
	ODEObject* body;

	dJointID* right_leg;
	dJointID* left_leg;

	dJointID* back_right;
	dJointID* back_left;

	float  speed;

	dReal previous_angle_at_dir_change;

	std::vector<FoodParticle> foodParticles;
	dReal target_position[3];

	Animator( ODEObject* head, ODEObject* body,
			  dJointID* right, dJointID* left,
			  dJointID* back_right, dJointID* back_left,
			  float speed,
			  dReal max_vert_force, dReal max_horiz_force)
												  : active(false), first_time( true ), starving( true ), out_of_boundry( false ),
												    known_target_counter( 0 ),
												    max_visible_distance( 40.0f ), field_of_view( 85.0f ),
												    right_leg_friction(0.0f), left_leg_friction(0.0f),
												    max_vert_force(max_vert_force), max_horiz_force(max_horiz_force ),
													boundry_length( 50.0f ), boundry_threshold( 49.5f ),
												    energy_consumption_period( 1000ll ), energy_consumption_per_period( 1 ),
													current_energy_level( 100 ), starvation_threshold( 15 ),
												    head( head ), body( body ),
													right_leg( right ), left_leg( left ),
												    back_right( back_right ), back_left( back_left ),
												    speed( speed ),
												    previous_angle_at_dir_change( 0.0f )
	{
		target_position[0] = 0.0f;
		target_position[1] = 0.0f;
		target_position[2] = 0.0f;
	}

	int getIndexOfFoodParticleSought( void )
	{
		return ndxOfFoodParticleSought;
	}
	
	dReal findEuclideanDistance(const dReal position1[3], const dReal position2[3])
	{
		dReal xComponent = position2[0] - position1[0];
		xComponent *= xComponent;

		dReal yComponent = position2[1] - position1[1];
		yComponent *= yComponent;

		dReal zComponent = position2[2] - position1[2];
		zComponent *= zComponent;

		return std::sqrtf( xComponent + yComponent + zComponent );
	}

	void updateEnergyLevel( void )
	{
		static auto prev_time_point = std::chrono::high_resolution_clock::now();
		
		auto elapsed_time = std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::high_resolution_clock::now() - prev_time_point ).count();
		if ( elapsed_time > energy_consumption_period )
		{
			prev_time_point = std::chrono::high_resolution_clock::now();
			//starving 
			current_energy_level -= energy_consumption_per_period;
		}

		//printf( "*** Current Energy Level: %d\t Starving? %s\n", current_energy_level, ( starving ) ? "True" : "False" );

		if ( current_energy_level > starvation_threshold )
		{
			active = false;
			starving = false;
			return;
		}

		if ( starving )
		{
			return;
		}

		active = true;
		starving = true;
		
		return;
	}

	void scanForFood(FoodParticle& food_particle)
	{
		if ( food_particle.known )
		{
			return;
		}

		auto alpha = findAlpha( food_particle );
		
		if ( alpha < field_of_view )
		{
			const dReal* head_position = dGeomGetPosition(head->Geom);
			const dReal* food_position = dGeomGetPosition(food_particle.odeObject.Geom);

			if ( findEuclideanDistance( head_position, food_position ) < max_visible_distance )
			{
				food_particle.known = true;
				food_particle.distance = getFoodParticleScore(food_particle);
				food_particle.colored = 1;
				++known_target_counter;
			}
		}
		/*else
		{
			food_particle.known = false;
			food_particle.colored = 0;
		}*/
	}

	dReal getFoodParticleScore(FoodParticle foodParticle)
	{
		const dReal* head_position = dGeomGetPosition(head->Geom);
		const dReal* food_position = dGeomGetPosition(foodParticle.odeObject.Geom);
		return findEuclideanDistance(head_position, food_position);
	}

	void updateFoodParticleScores( void )
	{
		for (auto i = 0; i < foodParticles.size(); ++i)
		{
			if ( foodParticles[i].known )
			{
				const dReal* head_position = dGeomGetPosition(head->Geom);
				const dReal* food_position = dGeomGetPosition(foodParticles[i].odeObject.Geom);
				foodParticles[i].distance = getFoodParticleScore(foodParticles[i]);
			}
			else
			{
				scanForFood( foodParticles[i] );
			}
		}
	}

	void removeSoughtFoodParticle( void )
	{
		foodParticles.erase(foodParticles.begin() + ndxOfFoodParticleSought);
		ndxOfFoodParticleSought = -1;
		right_leg_friction = 0.0f;
		left_leg_friction = 0.0f;
		--known_target_counter;
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
		dJointSetUniversalParam(*left_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*right_leg, dParamVel, 0);
		dJointSetUniversalParam(*right_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*right_leg, dParamVel2, speed);
		dJointSetUniversalParam(*right_leg, dParamFMax2, max_horiz_force);
	}


	void Move( void )
	{
		auto current_h_angle = Radians_To_Degrees( dJointGetUniversalAngle2( *right_leg ) );
		auto current_v_angle = Radians_To_Degrees( dJointGetUniversalAngle1( *left_leg ) );


		static double target_h_angle = 45.0;
		static double target_v_angle = -1.8;

		static bool first_time = true;
		static bool forward = true;

		if ( first_time ) moveForward();

		// once legs reach forward target angle
		if ( forward && current_h_angle > target_h_angle )
		{
			// once legs reach ground, move back
			if ( current_v_angle > target_v_angle )
			{
				if ( known_target_counter > 0 || out_of_boundry )
				{
					decideDirection();
				}

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
		//legs have reached backward target angle
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
	

	void seekFoodParticleWithHighestScore( void )
	{
		updateFoodParticleScores();

		dReal min_distance = 1000000.0f;
		int min_idx = -1;

		for ( auto i = 0; i < foodParticles.size(); ++i )
		{
			if ( foodParticles[i].known ) 
				foodParticles[i].colored = 1;
		}

		for (auto i = 0; i < foodParticles.size(); ++i)
		{
			if (foodParticles[i].known && foodParticles[i].distance < min_distance)
			{
				min_distance = foodParticles[i].distance;
				min_idx = i;
			}
		}

		if ( min_idx == -1 )
		{
			return;
		}

		ndxOfFoodParticleSought = min_idx;
		foodParticles[min_idx].colored = 2;
		const dReal* food_position = dGeomGetPosition(foodParticles[min_idx].odeObject.Geom);

		target_position[0] = food_position[0];
		target_position[1] = food_position[1];
		target_position[2] = food_position[2];
	}

	Angel::vec4 getCurrentOrientation( void )
	{
		const dReal* body_position = dGeomGetPosition(body->Geom);
		const dReal* box_position = dGeomGetPosition(head->Geom);		//Then, get the geometry position.

		Angel::vec4 body_orientation(box_position[0] - body_position[0],
			box_position[1] - body_position[1],
			box_position[2] - body_position[2],
			0.0f);
		body_orientation = Angel::normalize(body_orientation);
		body_orientation.w = 0.0;

		return body_orientation;
	}

	void dealWithBoundries( void )
	{
		auto head_position = dBodyGetPosition( head->Body );
	
		Angel::vec4 body_orientation = getCurrentOrientation();
		Angel::vec4 new_direction = Angel::cross( body_orientation, Angel::vec4( 0.0f, 1.0f, 0.0f, 0.0f ) );
		new_direction = Angel::normalize( new_direction );
		new_direction.w = 0.0f;
		
		if ( head_position[0] < -boundry_threshold || head_position[0] > boundry_threshold || head_position[2] < -boundry_threshold || head_position[2] > boundry_threshold )
		{
			out_of_boundry = true;

			auto body_position = dBodyGetPosition( body->Body );
			auto current_slope = body_orientation.z / body_orientation.x;
			
			if ( head_position[0] > boundry_threshold )
			{
				if ( current_slope < 0.0f )
				{
					target_position[0] = 0.0f;
					target_position[2] = -boundry_length;
				}
				else
				{
					target_position[0] = 0.0f;
					target_position[2] = boundry_length;
				}
			}
			else if ( head_position[0] < -boundry_threshold )
			{
				if ( current_slope < 0.0f )
				{
					target_position[0] = 0.0f;
					target_position[2] = boundry_length;
				}
				else
				{
					target_position[0] = 0.0f;
					target_position[2] = -boundry_length;
				}
			}
			else if ( head_position[2] > boundry_threshold )
			{
				if ( current_slope < 0.0f )
				{
					target_position[0] = -boundry_length;
					target_position[2] = 0.0f;
				}
				else
				{
					target_position[0] = boundry_length;
					target_position[2] = 0.0f;
				}
			}
			else if ( head_position[2] < -boundry_threshold )
			{
				if ( current_slope < 0.0f )
				{
					target_position[0] = boundry_length;
					target_position[2] = 0.0f;
				}
				else
				{
					target_position[0] = -boundry_length;
					target_position[2] = 0.0f;
				}
			}
		}
		else
		{
			out_of_boundry = false;

			return;
		}
	}

	float findAlpha( const FoodParticle& food_particle )
	{
		const dReal* target_position = dBodyGetPosition( food_particle.odeObject.Body );
		double x_target = target_position[0];
		double y_target = target_position[1];
		double z_target = target_position[2];

		const dReal* body_position = dGeomGetPosition(body->Geom);

		Angel::vec4 body_orientation = getCurrentOrientation();
		
		Angel::vec4 target_orientation(x_target - body_position[0],
			y_target - body_position[1],
			z_target - body_position[2],
			0.0f);
		target_orientation = Angel::normalize(target_orientation);
		target_orientation.w = 0.0;

		// Alpha = angle between body_orientation and target_orientation
		auto cosAlpha = Angel::dot(target_orientation, body_orientation);
		auto alpha = Radians_To_Degrees( std::acos(cosAlpha) );

		return alpha;
	}

	void decideDirection( void )
	{
		double x_target = target_position[0];
		double y_target = target_position[1];
		double z_target = target_position[2];

		const dReal* body_position = dGeomGetPosition(body->Geom);

		Angel::vec4 body_orientation = getCurrentOrientation();
		
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
		auto alpha = Radians_To_Degrees( std::acos(cosAlpha) );
		if (cosAlpha < 0) 
		{
			absCosBeta = 1 - absCosBeta;
		}
		auto frictionScale = std::fmax(absCosBeta, 0.2f);

		if (cosBeta >= 0.0)
		{
			right_leg_friction = 0.0;
			left_leg_friction = frictionScale * 30.0;
		}
		else if (cosBeta < 0.0)
		{
			right_leg_friction = frictionScale * 30.0;
			left_leg_friction = 0.0;
		}
		
		/*printf("cosine = %f.5\n", cosBeta);
		printf("angle  = %f.5\n\n", beta);
		printf("frictionScale = %f.5\n", frictionScale);
		printf("left leg friction = %f.5\n", left_leg_friction);
		printf("rightq leg friction = %f.5\n\n", right_leg_friction);*/
	}

private:
};



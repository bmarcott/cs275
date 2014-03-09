
#pragma once

#include <ode/ode.h>
#include <chrono>
#include <cmath>
#include "angle_conversions.h"
#include "vec.h"
#include "FoodParticle.h"
#include <algorithm>

struct Animator {
	bool active;
	
	bool turn_right;
	bool turn_left;
	
	dReal middle_right_outer_leg_friction;
	dReal middle_left_outer_leg_friction;

	dReal target_position[3];
	dReal max_vert_force;
	dReal max_horiz_force;
	
	const dReal max_middle_outer_leg_force = 500;

	vector<FoodParticle> knownFoodParticles;
	vector<dReal> knownFoodParticleScores;

	int ndxOfFoodParticleSought;

	Animator(dJointID* middle_right_inner_leg, dJointID* middle_left_inner_leg,
				dJointID* middle_right_outer_leg, dJointID* middle_left_outer_leg,
				  dJointID* back_right_leg, dJointID* back_left_leg,
				  double speed, double back_leg_speed,
				  dReal max_vert_force, dReal max_horiz_force)
												  : active(false), turn_right(false), turn_left(false),
												  middle_right_outer_leg_friction(0.0), middle_left_outer_leg_friction(0.0),
												  speed(speed), back_leg_speed( back_leg_speed),
												   max_vert_force(max_vert_force), max_horiz_force(max_horiz_force ),
												   midde_right_inner_leg(middle_right_inner_leg), midde_left_inner_leg(middle_left_inner_leg),
												   middle_right_outer_leg(middle_right_outer_leg), middle_left_outer_leg(middle_left_outer_leg),
												   back_right_leg(back_right_leg), back_left_leg(back_left_leg),
												   previous_angle_at_dir_change( 0.0 ), first_time( true )
	{
	}

	int getIndexOfFoodParticleSought()
	{
		return ndxOfFoodParticleSought;
	}
	
	dReal findEuclideanDistance(const dReal position1[3], const dReal position2[3])
	{
		dReal xComponent = pow(position2[0] - position1[0], 2);
		dReal yComponent = pow(position2[1] - position1[1], 2);
		dReal zComponent = pow(position2[2] - position1[2], 2);

		return pow(xComponent + yComponent + zComponent, 0.5);
	}

	void addKnownFoodParticle(FoodParticle food_particle) {
		knownFoodParticles.push_back(food_particle);
		knownFoodParticleScores.push_back(getFoodParticleScore(food_particle));

	}

	dReal getFoodParticleScore(FoodParticle foodParticle)
	{
		const dReal* head_position = dGeomGetPosition(invis_box.Geom);
		const dReal* food_position = dGeomGetPosition(foodParticle.odeObject.Geom);
		return foodParticle.prize - findEuclideanDistance(head_position, food_position);
	}

	void updateFoodParticleScores()
	{
		for (auto i = 0; i < knownFoodParticles.size(); ++i)
		{
			const dReal* head_position = dGeomGetPosition(invis_box.Geom);
			const dReal* food_position = dGeomGetPosition(knownFoodParticles[i].odeObject.Geom);
			knownFoodParticleScores[i] = getFoodParticleScore(knownFoodParticles[i]);

		}
	}

	void removeSoughtFoodParticle()
	{
		knownFoodParticles.erase(knownFoodParticles.begin() + ndxOfFoodParticleSought);
		knownFoodParticleScores.erase(knownFoodParticleScores.begin() + ndxOfFoodParticleSought);
		ndxOfFoodParticleSought = -1;
	}
	
	void seekFoodParticleWithHighestScore()
	{
		
		updateFoodParticleScores();

		dReal maxScore = 0;
		int maxI = -1;
		for (auto i = 0; i < knownFoodParticleScores.size(); ++i)
		{
			if (knownFoodParticleScores[i] > maxScore)
			{
				maxScore = knownFoodParticleScores[i];
				maxI = i;
			}
		}

		ndxOfFoodParticleSought = maxI;
		const dReal* food_position = dGeomGetPosition(knownFoodParticles[maxI].odeObject.Geom);
		target_position[0] = food_position[0];
		target_position[1] = food_position[1];
		target_position[2] = food_position[2];


	}

	
	void moveBackLegBack()
	{
		dJointSetUniversalParam(*back_left_leg, dParamVel, 0);
		dJointSetUniversalParam(*back_left_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*back_left_leg, dParamVel2, back_left_leg_scale * back_leg_speed);
		dJointSetUniversalParam(*back_left_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*back_right_leg, dParamVel, 0);
		dJointSetUniversalParam(*back_right_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*back_right_leg, dParamVel2, -back_right_leg_scale * back_leg_speed);
		dJointSetUniversalParam(*back_right_leg, dParamFMax2, max_horiz_force);
	}
	void moveBackLegForward(void)
	{
		dJointSetUniversalParam(*back_left_leg, dParamVel, 0);
		dJointSetUniversalParam(*back_left_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*back_left_leg, dParamVel2, -back_leg_speed);
		dJointSetUniversalParam(*back_left_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*back_right_leg, dParamVel, 0);
		dJointSetUniversalParam(*back_right_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*back_right_leg, dParamVel2, back_leg_speed);
		dJointSetUniversalParam(*back_right_leg, dParamFMax2, max_horiz_force);
	}
	void moveMiddleInnerLegDown( void )
	{
		dJointSetUniversalParam(*midde_left_inner_leg, dParamVel, speed);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamVel2, 0);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*midde_right_inner_leg, dParamVel, -speed);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamVel2, 0);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamFMax2, max_horiz_force);
	}

	void moveMiddleInnerLegUp(void)
	{
		dJointSetUniversalParam(*midde_left_inner_leg, dParamVel, -speed);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamVel2, 0);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*midde_right_inner_leg, dParamVel, speed);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamVel2, 0);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamFMax2, max_horiz_force);
	}

	void moveMiddleOuterLegBack(void)
	{
		dJointSetHingeParam(*middle_right_outer_leg, dParamVel, -speed * 2 / 3);
		dJointSetHingeParam(*middle_right_outer_leg, dParamFMax, max_middle_outer_leg_force);

		dJointSetHingeParam(*middle_left_outer_leg, dParamVel, speed * 2 / 3);
		dJointSetHingeParam(*middle_left_outer_leg, dParamFMax, max_middle_outer_leg_force);
	}

	void moveMiddleOuterLegForward(void)
	{
		dJointSetHingeParam(*middle_right_outer_leg, dParamVel, speed * 2 / 3);
		dJointSetHingeParam(*middle_right_outer_leg, dParamFMax, max_middle_outer_leg_force);

		dJointSetHingeParam(*middle_left_outer_leg, dParamVel, -speed * 2 / 3);
		dJointSetHingeParam(*middle_left_outer_leg, dParamFMax, max_middle_outer_leg_force);
	}

	void moveMiddleInnerLegBack(void)
	{
		dJointSetUniversalParam(*midde_left_inner_leg, dParamVel, 0);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamVel2, speed);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*midde_right_inner_leg, dParamVel, 0);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamVel2, -speed);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamFMax2, max_horiz_force);
	}

	void moveMiddleInnerLegForward(void)
	{
		dJointSetUniversalParam(*midde_left_inner_leg, dParamVel, 0);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamVel2, -speed);
		dJointSetUniversalParam(*midde_left_inner_leg, dParamFMax2, max_horiz_force);

		dJointSetUniversalParam(*midde_right_inner_leg, dParamVel, 0);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamFMax, max_vert_force);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamVel2, speed);
		dJointSetUniversalParam(*midde_right_inner_leg, dParamFMax2, max_horiz_force);
	}

	void Move( void )
	{
		auto current_h_angle = Radians_To_Degrees( dJointGetUniversalAngle2( *midde_right_inner_leg ) );
		auto current_v_angle = Radians_To_Degrees( dJointGetUniversalAngle1( *midde_left_inner_leg ) );


		static double target_backward_horiz_angle = -45.0;
		static double target_forward_horiz_angle = 45;
		static double target_upward_vert_angle = -20;
		static double target_downward_vert_angle = 1.8;

		static bool first_time = true;
		static bool forward = true;

		if (first_time) moveMiddleInnerLegForward();

		// once legs reach forward target angle
		if (forward && current_h_angle > target_forward_horiz_angle)
		{
			// once legs reach ground, move back
			if (current_v_angle > target_downward_vert_angle)
			{
				decideDirection();
				moveMiddleInnerLegBack();
				moveMiddleOuterLegBack();
				moveBackLegBack();
				first_time = false;
				forward = false;
				//target_h_angle = -target_h_angle;
			}
			else
			{
				moveMiddleInnerLegDown();
			}
		}
		//legs have reached backward target angle
		else if (!forward && current_h_angle < target_backward_horiz_angle)
		{
			// as soon as legs are above the ground, move forward
			if ( current_v_angle < target_upward_vert_angle )
			{
				moveMiddleInnerLegForward();
				moveMiddleOuterLegForward();
				moveBackLegForward();
				forward = true;
				//target_h_angle = -target_h_angle;
			}
			else
			{
				moveMiddleInnerLegUp();
			}
		}
	}
	






	// true = right, false = left
	void decideDirection(void)
	{
		double x_target = target_position[0];
		double y_target = target_position[1];
		double z_target = target_position[2];

		const dReal* body_position = dGeomGetPosition(body.Geom);
		const dReal* box_position = dGeomGetPosition(invis_box.Geom);		//Then, get the geometry position.

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
		auto frictionScale = std::fmax(absCosBeta, 0.2f);
		
		/*if (90.0 - beta < 5.0)
		{
			middle_left_outer_leg_friction = 0.0;
			middle_right_outer_leg_friction = 0.0;right_leg
		}*/
		if (cosBeta >= 0.0)
		{
			//middle_right_outer_leg_friction = 0.0;
			//middle_left_outer_leg_friction = frictionScale * 30.0;

			back_right_leg_scale = 1 - frictionScale;
			back_left_leg_scale = 1 + frictionScale;

		}
		else if (cosBeta < 0.0)
		{
			//middle_right_outer_leg_friction = frictionScale * 30.0;
			//middle_left_outer_leg_friction = 0.0;


			back_right_leg_scale = 1 + frictionScale;
			back_left_leg_scale = 1 - frictionScale;
			
		}
		
		printf("cosine = %f.5\n", cosBeta);
		printf("angle  = %f.5\n\n", beta);
		printf("frictionScale = %f.5\n", frictionScale);
		printf("left leg friction = %f.5\n", middle_left_outer_leg_friction);
		printf("rightq leg friction = %f.5\n\n", middle_right_outer_leg_friction);
	}


private:

	bool first_time;

	dJointID* midde_right_inner_leg;
	dJointID* midde_left_inner_leg;

	dJointID* middle_right_outer_leg;
	dJointID* middle_left_outer_leg;

	dJointID* back_right_leg;
	dJointID* back_left_leg;

	dReal back_right_leg_scale;
	dReal back_left_leg_scale;

	double  speed;
	double back_leg_speed;

	dReal previous_angle_at_dir_change;

	
};



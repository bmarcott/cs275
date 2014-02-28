
#pragma once

#include <ode/ode.h>
#include <chrono>
#include <cmath>
#include "angular_measure.h"

struct Move_Forward {
	bool active;
	bool reverse;
	bool turn_right;
	bool turn_left;
	bool skip_timer;

	Move_Forward( dJointID* right, dJointID* left,
				  dJointID* back_right, dJointID* back_left,
				  double speed,
				  long long time_gap,
				  dReal left_velocity1_down,  dReal left_velocity2_down,
				  dReal right_velocity1_down, dReal right_velocity2_down,
				  dReal left_velocity1_up,    dReal left_velocity2_up,
				  dReal right_velocity1_up,   dReal right_velocity2_up )
												 : active( false ), reverse( false ), turn_right( false ), turn_left( false ), skip_timer( false ),
												   speed( speed ),
												   time_gap( time_gap ),
												   right_leg( right ), left_leg( left ),
												   back_left( back_left ), back_right( back_right ),
												   left_velocity1_down( left_velocity1_down ), left_velocity2_down( left_velocity2_down ),
												   right_velocity1_down( right_velocity1_down ), right_velocity2_down( right_velocity2_down ),
												   left_velocity1_up( left_velocity1_up ), left_velocity2_up( left_velocity2_up ),
												   right_velocity1_up( right_velocity1_up ), right_velocity2_up( right_velocity2_up ),
												   previous_angle_at_dir_change( 0.0 ), first_time( true )
	{
	}

	void Forward( void )
	{
		static double prev_vertical_rot_angle = 0.0;

		auto alpha = Radians_To_Degrees( dJointGetUniversalAngle2( *right_leg ) );
		auto angle_change = std::abs( alpha - previous_angle_at_dir_change );

		//printf( "%f.6\n", angle_change );

		double temp_threshold = 80.0;

		if ( first_time ) temp_threshold = 40.0;

		if ( angle_change > temp_threshold )
		{
			previous_angle_at_dir_change = alpha;

			skip_timer = true;
			reverse = !reverse;
		}
		else if ( !skip_timer && std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::high_resolution_clock::now() - prev_time_point ).count() < time_gap )
		{
			return;
		}

		if ( skip_timer )
		{
			skip_timer = false;
		}

		if ( reverse )
		{
			if ( Radians_To_Degrees( dJointGetUniversalAngle1( *left_leg ) ) < -6.0 )
			{
				dJointSetUniversalParam(*left_leg, dParamVel, speed);
				dJointSetUniversalParam(*left_leg, dParamFMax, 100.0);
				dJointSetUniversalParam(*left_leg, dParamVel2, 0.0);
				dJointSetUniversalParam(*left_leg, dParamFMax2, 100.0);

				dJointSetUniversalParam(*right_leg, dParamVel, -speed);
				dJointSetUniversalParam(*right_leg, dParamFMax, 100);
				dJointSetUniversalParam(*right_leg, dParamVel2, 0);
				dJointSetUniversalParam(*right_leg, dParamFMax2, 100);
			}
			else
			{
				dJointSetUniversalParam(*left_leg, dParamVel, 0);
				dJointSetUniversalParam(*left_leg, dParamFMax, 100);
				dJointSetUniversalParam(*left_leg, dParamVel2, speed);
				dJointSetUniversalParam(*left_leg, dParamFMax2, 100);

				dJointSetUniversalParam(*right_leg, dParamVel, 0);
				dJointSetUniversalParam(*right_leg, dParamFMax, 100);
				dJointSetUniversalParam(*right_leg, dParamVel2, -speed);
				dJointSetUniversalParam(*right_leg, dParamFMax2, 100);
			}
		}
		else
		{			
			if ( Radians_To_Degrees( dJointGetUniversalAngle1( *left_leg ) ) > -6.0 )
			{
				dJointSetUniversalParam(*left_leg, dParamVel, -speed);
				dJointSetUniversalParam(*left_leg, dParamFMax, 100);
				dJointSetUniversalParam(*left_leg, dParamVel2, 0);
				dJointSetUniversalParam(*left_leg, dParamFMax2, 100);

				dJointSetUniversalParam(*right_leg, dParamVel, speed);
				dJointSetUniversalParam(*right_leg, dParamFMax, 100);
				dJointSetUniversalParam(*right_leg, dParamVel2, 0);
				dJointSetUniversalParam(*right_leg, dParamFMax2, 100);
			}
			else
			{
				dJointSetUniversalParam(*left_leg, dParamVel, 0);
				dJointSetUniversalParam(*left_leg, dParamFMax, 100);
				dJointSetUniversalParam(*left_leg, dParamVel2, -speed);
				dJointSetUniversalParam(*left_leg, dParamFMax2, 100);

				dJointSetUniversalParam(*right_leg, dParamVel, 0);
				dJointSetUniversalParam(*right_leg, dParamFMax, 100);
				dJointSetUniversalParam(*right_leg, dParamVel2, speed);
				dJointSetUniversalParam(*right_leg, dParamFMax2, 100);
			}
		}

		prev_time_point = std::chrono::high_resolution_clock::now();
	}
	

	//---------------------------------------------

	dReal Get_Left_velocity1_Down( void ) const
	{
		return left_velocity1_down;
	}

	void Set_Left_velocity1_Down( dReal velocity )
	{
		left_velocity1_down = velocity;
	}

	//---------------------------------------------

	dReal Get_Left_velocity2_Down( void ) const
	{
		return left_velocity2_down;
	}

	void Set_Left_velocity2_Down( dReal velocity )
	{
		left_velocity2_down = velocity;
	}

	//---------------------------------------------

	dReal Get_Left_velocity1_Up( void ) const
	{
		return left_velocity1_up;
	}

	void Set_Left_velocity1_Up( dReal velocity )
	{
		left_velocity1_up = velocity;
	}

	//---------------------------------------------

	dReal Get_Left_velocity2_Up( void ) const
	{
		return left_velocity2_up;
	}

	void Set_Left_velocity2_Up( dReal velocity )
	{
		left_velocity2_up = velocity;
	}

	//---------------------------------------------

	dReal Get_Right_velocity1_Down( void ) const
	{
		return right_velocity1_down;
	}

	void Set_Right_velocity1_Down( dReal velocity )
	{
		right_velocity1_down = velocity;
	}

	//---------------------------------------------

	dReal Get_Right_velocity2_Down( void ) const
	{
		return right_velocity2_down;
	}

	void Set_Right_velocity2_Down( dReal velocity )
	{
		right_velocity2_down = velocity;
	}

	//---------------------------------------------

	dReal Get_Right_velocity1_Up( void ) const
	{
		return right_velocity1_up;
	}

	void Set_Right_velocity1_Up( dReal velocity )
	{
		right_velocity1_up = velocity;
	}

	//---------------------------------------------

	dReal Get_Right_velocity2_Up( void ) const
	{
		return right_velocity2_up;
	}

	void Set_Right_velocity2_Up( dReal velocity )
	{
		right_velocity2_up = velocity;
	}

	//---------------------------------------------

private:

	bool first_time;

	dJointID* right_leg;
	dJointID* left_leg;

	dJointID* back_right;
	dJointID* back_left;

	double  speed;
	__int64 time_gap;

	dReal left_velocity1_down;
	dReal left_velocity2_down;

	dReal right_velocity1_down;
	dReal right_velocity2_down;

	dReal left_velocity1_up;
	dReal left_velocity2_up;

	dReal right_velocity1_up;
	dReal right_velocity2_up;

	std::chrono::high_resolution_clock::time_point prev_time_point;

	dReal previous_angle_at_dir_change;
};



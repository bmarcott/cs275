
#include "Angel\mat.h"

void Get_Aligned_Axis( float rot_x,
					   float rot_y,
					   float rot_z,
					   Angel::vec4 new_system[ 3 ] )
{
	Angel::vec4 x( 1.0f, 0.0f, 0.0f, 0.0f );
	Angel::vec4 y( 0.0f, 1.0f, 0.0f, 0.0f );
	Angel::vec4 z( 0.0f, 0.0f, 1.0f, 0.0f );

	auto rot_x_mat = RotateX( -rot_x );
	auto rot_y_mat = RotateY( -rot_y );
	auto rot_z_mat = RotateZ( -rot_z );

	auto transf = rot_x_mat * rot_y_mat * rot_z_mat;

	new_system[ 0 ] = mvmult( transf, x );
	new_system[ 1 ] = mvmult( transf, y );
	new_system[ 2 ] = mvmult( transf, z );

	normalize( new_system[ 0 ] );
	normalize( new_system[ 1 ] );
	normalize( new_system[ 2 ] );
}


/*
 * motion.cpp
 *
 *  Created on: 2024/03/17
 *      Author: sato1
 */

#include "../Inc/ctrl_task.h"
#include "../../Params/turn_table.h"

float get_turn_table_value(float time_period,float time)
{
	int std_a = (int)(time/time_period);
	int std_b = std_a + 1;
	float m = time/time_period - (float)(std_a);
	float n = (float)(std_b) -time/time_period;
	float turn_table_value =  (n*accel_table[std_a] + m*accel_table[std_b]);
	return turn_table_value;
}

float get_turn_table_diff_value(float time_period,float time)
{

}

void  Motion::SetIdeal_search_straight(){
	if(motion_set.length_deccel.get() < ( motion_set.end_length.get() - vehicle->ego.length.get()))
	{
		//
		vehicle->ideal.accel.set(motion_set.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(vehicle->ideal.velo.get() > motion_set.max_velo.get())
		{
			vehicle->ideal.velo.set( motion_set.max_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else if(vehicle->ego.length.get() < motion_set.end_length.get())
	{
		vehicle->ideal.accel.set(motion_set.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(motion_set.end_velo.get() == 0.0f)
		{
			if(vehicle->ideal.velo.get()< 0.15)
			{
				vehicle->ideal.velo.set( 0.150f);		vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);	vehicle->ideal.rad_accel.set(0.0f);
			}
		}
		else if(vehicle->ideal.velo.get() < motion_set.end_velo.get())
		{
			vehicle->ideal.velo.set( motion_set.end_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else
	{
		if(motion_set.end_velo.get() == 0.0f)
		{
			vehicle->ideal.velo.set( 0.0f);			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.rad_velo.set( 0.0f);		vehicle->ideal.rad_accel.set(0.0f);
			Init_Motion_stop_brake(400);
		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
		}
	}

	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}
void  Motion::SetIdeal_search_turn(){

}

void Motion::SetIdeal_straight()
{
	if(motion_set.length_deccel.get() < ( motion_set.end_length.get() - vehicle->ego.length.get()))
	{
		//accel & constant velo running set up
		vehicle->ideal.accel.set(motion_set.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(vehicle->ideal.velo.get() > motion_set.max_velo.get())
		{
			vehicle->ideal.velo.set( motion_set.max_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else if(vehicle->ego.length.get() < motion_set.end_length.get())
	{
		vehicle->ideal.accel.set(motion_set.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(motion_set.end_velo.get() == 0.0f)
		{
			if(vehicle->ideal.velo.get()< 0.15)
			{
				vehicle->ideal.velo.set( 0.150f);		vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);	vehicle->ideal.rad_accel.set(0.0f);
			}
		}
		else if(vehicle->ideal.velo.get() < motion_set.end_velo.get())
		{
			vehicle->ideal.velo.set( motion_set.end_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else
	{
		if(motion_set.end_velo.get() == 0.0f)
		{
			vehicle->ideal.velo.set( 0.0f);			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.rad_velo.set( 0.0f);		vehicle->ideal.rad_accel.set(0.0f);
			Init_Motion_stop_brake(400);
		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
		}
	}

	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_diagonal		( )
{
	if(motion_set.length_deccel.get() < ( motion_set.end_length.get() - vehicle->ego.length.get()))
	{
		vehicle->ideal.accel.set(motion_set.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(vehicle->ideal.velo.get() > motion_set.max_velo.get())
		{
			vehicle->ideal.velo.set( motion_set.max_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else if(vehicle->ego.length.get() < motion_set.end_length.get())
	{
		vehicle->ideal.accel.set(motion_set.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(motion_set.end_velo.get() == 0.0f)
		{
			if(vehicle->ideal.velo.get()< 0.15)
			{
				vehicle->ideal.velo.set( 0.150f);		vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);	vehicle->ideal.rad_accel.set(0.0f);
			}
		}
		else if(vehicle->ideal.velo.get() < motion_set.end_velo.get())
		{
			vehicle->ideal.velo.set( motion_set.end_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else
	{
		if(motion_set.end_velo.get() == 0.0f)
		{
			vehicle->ideal.velo.set( 0.0f);			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.rad_velo.set( 0.0f);		vehicle->ideal.rad_accel.set(0.0f);
			Init_Motion_stop_brake(400);
		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
		}
	}

	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_pivot_turn()
{
	vehicle->ideal.velo.set(0.0f);
	vehicle->ideal.accel.set(0.0f);

	if(ABS(motion_set.radian_deccel.get()) < (ABS(motion_set.end_radian.get()) - ABS(vehicle->ego.radian.get())))
	{
		vehicle->ideal.rad_accel.set(motion_set.rad_accel.get());
		vehicle->ideal.rad_velo.set(vehicle->ideal.rad_velo.get() + motion_set.rad_accel.get()*(float)deltaT_ms/1000.0f);
		if(ABS(vehicle->ideal.rad_velo.get()) > ABS(motion_set.rad_max_velo.get()))
		{
			vehicle->ideal.rad_velo.set(motion_set.rad_max_velo.get());
		}
	}
	else if(ABS(vehicle->ego.radian.get()) < ABS(motion_set.end_radian.get()))
	{
		vehicle->ideal.rad_accel.set(motion_set.rad_deccel.get());
		vehicle->ideal.rad_velo.set(vehicle->ideal.rad_velo.get() + motion_set.rad_accel.get()*(float)deltaT_ms/1000.0f);
		if(ABS(vehicle->ideal.rad_velo.get()) > ABS(motion_set.rad_max_velo.get()))
		{
			if(vehicle->ideal.rad_velo.get() <= 0.0 && motion_set.end_radian.get()> 0.0)
			{
				vehicle->ideal.rad_velo.set(0.0) ;
			}
			else if(vehicle->ideal.rad_velo.get() >= 0.0 && motion_set.end_radian.get() < 0.0)
			{
				vehicle->ideal.rad_velo.set(0.0) ;
			}
		}
	}
	else
	{
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
		vehicle->ideal.rad_accel.set(0.0f);

		Init_Motion_stop_brake(400);
	}

	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_turn_in		( ){}
void Motion::SetIdeal_turn_out		( ){}
void Motion::SetIdeal_long_turn		( ){}
void Motion::SetIdeal_turn_v90		( ){}

void Motion::SetIdeal_fix_wall		( ){}
void Motion::SetIdeal_stop_brake	( ){}

void Motion::SetIdeal_free_rotation_set	(){}

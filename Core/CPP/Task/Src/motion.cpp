/*
 * motion.cpp
 *
 *  Created on: 2024/03/17
 *      Author: sato1
 */

#include "../Inc/ctrl_task.h"
#include "../Inc/run_typedef.h"
#include "../../Params/turn_table.h"

const float detect_wall_edge_st = 10.0f;
const float detect_wall_edge_di = 10.0f;

float get_turn_table_value(float time_period_ms,float time_ms)
{
	float turn_table_value = 0.0f;
	if(time_ms <= time_period_ms)
	{
		//calc array position
		int std_a = (int)((time_ms*1000.0f/time_period_ms));
		int std_b = std_a + 1;
		//
		float m = ((time_ms*1000.0f/time_period_ms)) - (float)(std_a);
		float n = (float)(std_b) -((time_ms*1000.0f/time_period_ms));
		turn_table_value =  (n*accel_table[std_a] + m*accel_table[std_b]);
		return turn_table_value;
	}
	else	;

	return turn_table_value;
}

void Motion::SetIdeal_wall_control()
{
	if(motion_state_get() == STRAIGHT_STATE || motion_state_get() == DIAGONAL_STATE)
	{
		if(motion_state_get() == STRAIGHT_STATE) ir_sens->EnableIrSensStraight();
		if(motion_state_get() == DIAGONAL_STATE) ir_sens->EnableIrSensDiagonal();

		if(vehicle->ideal.velo.get() <= 0.150)
		{
			ir_sens->DisableIrSens();
		}
		else
		{
			ir_sens->SetWallControl_RadVelo(vehicle, deltaT_ms);
		}
	}
	else
	{
		ir_sens->DisableIrSens();
	}
}

void  Motion::SetIdeal_search_straight(){

	motion_state_set(STRAIGHT_STATE);

	if(motion_plan.length_deccel.get() < ( motion_plan.end_length.get() - vehicle->ego.length.get()))
	{
		//accel & constant velo running set up
		vehicle->ideal.accel.set(motion_plan.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(vehicle->ideal.velo.get() > motion_plan.max_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.max_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}

		if(ir_sens->Division_Wall_Correction() == True)
		{
			if(motion_plan.end_length.get()== 90.0f)
			{
				if(ABS(vehicle->ego.length.get() - 55.0) < 8.0f)
				{
					vehicle->ego.length.set((vehicle->ego.length.get() + 55.0)/2.0f);
				}
			}
		}
	}
	else if(vehicle->ego.length.get() < motion_plan.end_length.get())
	{
		vehicle->ideal.accel.set(motion_plan.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(motion_plan.end_velo.get() == 0.0f)
		{
			if(vehicle->ideal.velo.get()< 0.15)
			{
				vehicle->ideal.velo.set( 0.150f);		vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);	vehicle->ideal.rad_accel.set(0.0f);
			}
		}
		else if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.end_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else
	{
		if(motion_plan.end_velo.get() == 0.0f)
		{
			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			Init_Motion_stop_brake(400);
			return;
		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			//vehicle->ideal.radian.set(0.0f);

			vehicle->ego.length.set(0.0f);
			//vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}
void  Motion::SetIdeal_search_turn()
{
	static float turn_start_time_ms;
	vehicle->ideal.velo.set(motion_plan.velo.get());
	vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{

		}
		else
		{
			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			motion_plan.turn_state.set(Post_Turn);
			turn_start_time_ms = 0.0f;
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{

		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);


			vehicle->ego.length.set(0.0f);
			//vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);

}

void Motion::SetIdeal_straight()
{
	motion_state_set(STRAIGHT_STATE);
	float offset = 0.0f;
	if(motion_plan.max_velo.get() > motion_plan.end_velo.get())
	{
		offset = 10.0f;
	}

	if((motion_plan.length_deccel.get()+offset) < ( motion_plan.end_length.get() - vehicle->ego.length.get()))
	{
		//accel & constant velo running set up
		vehicle->ideal.accel.set(motion_plan.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(vehicle->ideal.velo.get() > motion_plan.max_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.max_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else if(vehicle->ego.length.get() < motion_plan.end_length.get())
	{
		vehicle->ideal.accel.set(motion_plan.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(motion_plan.end_velo.get() == 0.0f)
		{
			if(vehicle->ideal.velo.get()< 0.15)
			{
				vehicle->ideal.velo.set( 0.150f);		vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);	vehicle->ideal.rad_accel.set(0.0f);
			}
		}
		else if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.end_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else
	{
		if(motion_plan.end_velo.get() == 0.0f)
		{
			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			Init_Motion_stop_brake(400);
			return;
		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			//vehicle->ideal.radian.set(0.0f);

			vehicle->ego.length.set(0.0f);
			//vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_diagonal		( )
{
	motion_state_set(DIAGONAL_STATE);
	if(motion_plan.length_deccel.get() < ( motion_plan.end_length.get() - vehicle->ego.length.get()))
	{
		vehicle->ideal.accel.set(motion_plan.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(vehicle->ideal.velo.get() > motion_plan.max_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.max_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else if(vehicle->ego.length.get() < motion_plan.end_length.get())
	{
		vehicle->ideal.accel.set(motion_plan.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(motion_plan.end_velo.get() == 0.0f)
		{
			if(vehicle->ideal.velo.get()< 0.15)
			{
				vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.velo.set( 0.150f);
				vehicle->ideal.rad_accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);
			}
		}
		else if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.end_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else
	{
		if(motion_plan.end_velo.get() == 0.0f)
		{
			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			Init_Motion_stop_brake(400);
			return;
		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			//vehicle->ideal.radian.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_pivot_turn()
{
	vehicle->ideal.velo.set(0.0f);
	vehicle->ideal.accel.set(0.0f);

	motion_state_set(PIVTURN_STATE);
	if(ABS(motion_plan.radian_deccel.get()) < (ABS(motion_plan.end_radian.get()) - ABS(vehicle->ego.radian.get())))
	{
		vehicle->ideal.rad_accel.set(motion_plan.rad_accel.get());
		vehicle->ideal.rad_velo.set(vehicle->ideal.rad_velo.get() + vehicle->ideal.rad_accel.get()*(float)deltaT_ms/1000.0f);
		if(ABS(vehicle->ideal.rad_velo.get()) > ABS(motion_plan.rad_max_velo.get()))
		{
			vehicle->ideal.rad_velo.set(motion_plan.rad_max_velo.get());
		}
	}
	else if(ABS(vehicle->ego.radian.get()) < ABS(motion_plan.end_radian.get()))
	{
		vehicle->ideal.rad_accel.set(motion_plan.rad_deccel.get());
		vehicle->ideal.rad_velo.set(vehicle->ideal.rad_velo.get() + vehicle->ideal.rad_accel.get()*(float)deltaT_ms/1000.0f);
		if(ABS(vehicle->ideal.rad_velo.get()) > ABS(motion_plan.rad_max_velo.get()))
		{
			if(vehicle->ideal.rad_velo.get() <= 0.0 && motion_plan.end_radian.get()> 0.0)
			{
				vehicle->ideal.rad_velo.set(0.0) ;
			}
			else if(vehicle->ideal.rad_velo.get() >= 0.0 && motion_plan.end_radian.get() < 0.0)
			{
				vehicle->ideal.rad_velo.set(0.0) ;
			}
		}
	}
	else
	{
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
		vehicle->ideal.radian.set(0.0f);
		vehicle->ideal.turn_slip_theta.set(0.0f);

		vehicle->ego.length.set(0.0f);
		vehicle->ego.radian.set(0.0f);
		vehicle->ego.turn_slip_theta.set(0.0f);

		vehicle->ego.turn_x.set(0.0f);
		vehicle->ego.turn_y.set(0.0f);
		vehicle->ideal.turn_x.set(0.0f);
		vehicle->ideal.turn_y.set(0.0f);

		Init_Motion_stop_brake(400);
		return;
	}

	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_turn_in		( )
{
	static float turn_start_time_ms;
	vehicle->ideal.velo.set(motion_plan.velo.get());
	vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= turn_motion_param.param->Lstart)
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				vehicle->ego.length.set((vehicle->ego.length.get() + detect_wall_edge_st)/2.0f);
			}
		}
		else
		{
			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			motion_plan.turn_state.set(Post_Turn);
			turn_start_time_ms = 0.0f;
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(DIAGONAL_STATE);
		if(vehicle->ego.length.get() <= turn_motion_param.param->Lend)
		{

		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			//vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}
void Motion::SetIdeal_turn_out		( ){
	static float turn_start_time_ms;
	vehicle->ideal.velo.set(motion_plan.velo.get());
	vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(DIAGONAL_STATE);
		if(vehicle->ego.length.get() <= turn_motion_param.param->Lstart)
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				vehicle->ego.length.set((vehicle->ego.length.get() + detect_wall_edge_di)/2.0f);
			}
		}
		else
		{
			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			motion_plan.turn_state.set(Post_Turn);
			turn_start_time_ms = 0.0f;
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= turn_motion_param.param->Lend)
		{

		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			//vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}
void Motion::SetIdeal_long_turn		( )
{
	static float turn_start_time_ms;
	vehicle->ideal.velo.set(motion_plan.velo.get());
	vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= turn_motion_param.param->Lstart)
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				vehicle->ego.length.set((vehicle->ego.length.get() + detect_wall_edge_st)/2.0f);
			}
		}
		else
		{
			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			motion_plan.turn_state.set(Post_Turn);
			turn_start_time_ms = 0.0f;
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= turn_motion_param.param->Lend)
		{

		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			//vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}
void Motion::SetIdeal_turn_v90		( )
{
	static float turn_start_time_ms;
	vehicle->ideal.velo.set(motion_plan.velo.get());
	vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(DIAGONAL_STATE);
		if(vehicle->ego.length.get() <= turn_motion_param.param->Lstart)
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				vehicle->ego.length.set((vehicle->ego.length.get() + detect_wall_edge_di)/2.0f);
			}
		}
		else
		{
			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			motion_plan.turn_state.set(Post_Turn);
			turn_start_time_ms = 0.0f;
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(DIAGONAL_STATE);
		if(vehicle->ego.length.get() <= turn_motion_param.param->Lend)
		{

		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			//vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_fix_wall		( )
{
	if(run_time_ms_get() <= run_time_limit_ms_get())
	{
		if(ir_sens->sen_fr.distance < 70.0 && ir_sens->sen_fl.distance < 70.0 )
		{
			float sp_err = 0.0f;		float om_err = 0.0f;
			float r_err = 0.0;			float l_err = 0.0;
			if(ir_sens->sen_fr.distance > 50.0 && ir_sens->sen_fl.distance > 50.0 )
			{
				r_err = (ir_sens->sen_fr.distance - 45.0);
				l_err = (ir_sens->sen_fl.distance - 45.0);
			}
			else
			{
				int r_check = 0;
				int l_check = 0;
				if((ir_sens->sen_fr.distance > 43.0)
				&&(ir_sens->sen_r.distance > 36.0 && ir_sens->sen_l.distance > 39.0))
				{
					r_err = (ir_sens->sen_fr.distance - 45.0);
					r_check = 1;
				}
				if((ir_sens->sen_fl.distance > 43.0)
				&&(ir_sens->sen_r.distance > 36.0 && ir_sens->sen_l.distance > 39.0))
				{
					l_err = (ir_sens->sen_fl.distance - 45.0);
					l_check = 1;
				}
				if(r_check == 1 || r_check == 1)
				{
					if(r_check == 1) r_err = (ir_sens->sen_fr.distance - 45.0);
					if(l_check == 1) l_err = (ir_sens->sen_fl.distance - 45.0);
					if(r_check == 1 && l_check == 0) r_err = r_err * 2;
					if(r_check == 0 && l_check == 1) l_err = l_err * 2;
				}
				else
				{
					r_err = -10.0f;
					l_err = -10.0f;
					//run_time_ms_set(run_time_limit_ms_get() - 0.50f);
				}
			}

			sp_err = (r_err+ l_err)/2.0f;
			om_err = (r_err- l_err)/2.0f;

			float target_acc = (1.0 * sp_err - 100.0*vehicle->ideal.velo.get());
			float target_velo = vehicle->ideal.velo.get() + target_acc/1000.0f;
			float max_set_velo = 0.3;
			if(target_velo >=  max_set_velo)
			{
				target_acc = 0.0;
				target_velo = max_set_velo;
			}
			else if(target_velo <= -max_set_velo){
				target_acc = 0.0;
				target_velo = -max_set_velo;
			}
			vehicle->ideal.accel.set(target_acc);
			vehicle->ideal.velo.set(target_velo);


			float target_rad_accel = (10.0*om_err - 20.0*vehicle->ideal.rad_velo.get());
			float target_rad_velo  = vehicle->ideal.rad_velo.get() + target_rad_accel/1000.0;

			float max_set_rad_velo = 10.0;
			if(target_rad_velo >= max_set_rad_velo)
			{
				target_rad_accel = 0.0;
				target_rad_velo = max_set_rad_velo;
			}
			else if(target_rad_velo <= -max_set_rad_velo)
			{
				target_rad_accel = 0.0;
				target_rad_velo = -max_set_rad_velo;
			}

			vehicle->ideal.rad_accel.set(target_rad_accel);
			vehicle->ideal.rad_velo.set(target_rad_velo);
		}

	}
	else
	{

		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
		vehicle->ideal.radian.set(0.0f);
		vehicle->ideal.turn_slip_theta.set(0.0f);

		vehicle->ego.length.set(0.0f);
		vehicle->ego.radian.set(0.0f);
		vehicle->ego.turn_slip_theta.set(0.0f);

		vehicle->ego.turn_x.set(0.0f);
		vehicle->ego.turn_y.set(0.0f);
		vehicle->ideal.turn_x.set(0.0f);
		vehicle->ideal.turn_y.set(0.0f);

		Init_Motion_stop_brake(100);
		return;

	}

	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);

}
void Motion::SetIdeal_stop_brake	( )
{
	if(run_time_ms_get() <= run_time_limit_ms_get())
	{
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		//vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
		//vehicle->ideal.radian.set(0.0f);
	}
	else
	{
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
		vehicle->ideal.radian.set(0.0f);
		vehicle->ideal.turn_slip_theta.set(0.0f);

		vehicle->ego.length.set(0.0f);
		vehicle->ego.radian.set(0.0f);
		vehicle->ego.turn_slip_theta.set(0.0f);

		vehicle->ego.turn_x.set(0.0f);
		vehicle->ego.turn_y.set(0.0f);
		vehicle->ideal.turn_x.set(0.0f);
		vehicle->ideal.turn_y.set(0.0f);
		//Init_Motion_stop_brake(200);
		motion_exeStatus_set(complete);
		motion_state_set(NOP_STATE);
		motion_pattern_set(No_run);

		return;
	}

	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_free_rotation_set	()
{
	if(run_time_ms_get() <= run_time_limit_ms_get())
	{
		vehicle->motor_out_r = 200;		vehicle->motor_out_l = 200;
	}
	else
	{
		vehicle->motor_out_r = 0;		vehicle->motor_out_l = 0;
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
		vehicle->ideal.radian.set(0.0f);
		vehicle->ideal.turn_slip_theta.set(0.0f);

		vehicle->ego.length.set(0.0f);
		vehicle->ego.radian.set(0.0f);
		vehicle->ego.turn_slip_theta.set(0.0f);

		vehicle->ego.turn_x.set(0.0f);
		vehicle->ego.turn_y.set(0.0f);
		vehicle->ideal.turn_x.set(0.0f);
		vehicle->ideal.turn_y.set(0.0f);
		motion_exeStatus_set(complete);
	}

	run_time_ms_update();

}

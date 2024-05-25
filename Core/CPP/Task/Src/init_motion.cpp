/*
 * set_motion.cpp
 *
 *  Created on: 2024/03/17
 *      Author: sato1
 */


#include "../Inc/ctrl_task.h"
#include "../../Params/turn_table.h"

void Motion::Motion_start()
{
	vehicle->ego_initialize();
	vehicle->ideal_initialize();
	motion_enable_set();
	motion_pattern_set(No_run);
	motion_exeStatus_set(complete);
	motion_state_set(NOP_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
}

void Motion::Motion_end()
{
	motion_disable_set();
	motion_pattern_set(No_run);
	motion_exeStatus_set(complete);
	motion_state_set(NOP_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
}

void Motion::Init_Motion_free_rotation_set( )
{
	motion_plan.velo.init();
	motion_plan.max_velo.init();
	motion_plan.end_velo.init();
	motion_plan.accel.init();
	motion_plan.deccel.init();
	motion_plan.end_length.init();
	motion_plan.length_accel.init();
	motion_plan.length_deccel.init();

	motion_plan.rad_accel.init();
	motion_plan.rad_deccel.init();
	motion_plan.rad_max_velo.init();
	motion_plan.end_radian.init();
	motion_plan.radian_accel.init();
	motion_plan.radian_deccel.init();
	motion_plan.turn_r_min.init();
	motion_plan.turn_state.init();
	motion_plan.turn_time_ms.init		();

	motion_pattern_set(motor_free);
	motion_exeStatus_set(execute);
	motion_state_set(NOP_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
	run_time_limit_ms_set(500.0f);

}

void Motion::Init_Motion_search_straight(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_plan.velo.set				(vehicle->ideal.velo.get());
	motion_plan.max_velo.set			(max_sp);
	motion_plan.end_velo.set			(end_sp);
	motion_plan.accel.set			(acc);
	motion_plan.deccel.set			((-1.0f)*acc);
	motion_plan.end_length.set		(len_target);
	motion_plan.length_accel.set 	(1000*(max_sp*max_sp-motion_plan.velo.get()*motion_plan.velo.get())/(2.0*ABS(motion_plan.accel.get())));
	motion_plan.length_deccel.set	(1000*(max_sp*max_sp-end_sp*end_sp)/(2.0*ABS(motion_plan.deccel.get())));

	motion_plan.rad_accel.init		();
	motion_plan.rad_deccel.init		();
	motion_plan.rad_max_velo.init	();
	motion_plan.end_radian.init		();
	motion_plan.radian_accel.init	();
	motion_plan.radian_deccel.init	();
	motion_plan.turn_r_min.init		();
	motion_plan.turn_state.init		();
	motion_plan.turn_time_ms.init		();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	motion_pattern_set(Search_st_section);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

}

void Motion::Init_Motion_search_turn	(const t_param *turn_param,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set		(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set			(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param = *turn_param;

	(motion_plan.end_radian.get() > 0) ? motion_pattern_set(Search_slalom_L): motion_pattern_set(Search_slalom_R);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

}

void Motion::Init_Motion_straight		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_plan.velo.set				(vehicle->ideal.velo.get());
	motion_plan.max_velo.set			(max_sp);
	motion_plan.end_velo.set			(end_sp);
	motion_plan.accel.set			(acc);
	motion_plan.deccel.set			((-1.0f)*acc);
	motion_plan.end_length.set		(len_target);
	motion_plan.length_accel.set 	(1000*(max_sp*max_sp-motion_plan.velo.get()*motion_plan.velo.get())/(2.0*ABS(motion_plan.accel.get())));
	motion_plan.length_deccel.set	(1000*(max_sp*max_sp-end_sp*end_sp)/(2.0*ABS(motion_plan.deccel.get())));

	motion_plan.rad_accel.init		();
	motion_plan.rad_deccel.init		();
	motion_plan.rad_max_velo.init	();
	motion_plan.end_radian.init		();
	motion_plan.radian_accel.init	();
	motion_plan.radian_deccel.init	();
	motion_plan.turn_r_min.init		();
	motion_plan.turn_state.init		();
	motion_plan.turn_time_ms.init		();

	//Set control gain
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	vehicle->ego_integral_init();
	vehicle->ideal_integral_init();


	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(sp_gain->Kp, sp_gain->Ki, sp_gain->Kd);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(om_gain->Kp, om_gain->Ki, om_gain->Kd);
	vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	motion_pattern_set(Straight);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

}

void Motion::Init_Motion_diagonal		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_plan.velo.set				(vehicle->ideal.velo.get());
	motion_plan.max_velo.set			(max_sp);
	motion_plan.end_velo.set			(end_sp);
	motion_plan.accel.set			(acc);
	motion_plan.deccel.set			((-1.0f)*acc);
	motion_plan.end_length.set		(len_target);
	motion_plan.length_accel.set 	(1000*(max_sp*max_sp-motion_plan.velo.get()*motion_plan.velo.get())/(2.0*ABS(motion_plan.accel.get())));
	motion_plan.length_deccel.set	(1000*(max_sp*max_sp-end_sp*end_sp)/(2.0*ABS(motion_plan.deccel.get())));

	motion_plan.rad_accel.init		();
	motion_plan.rad_deccel.init		();
	motion_plan.rad_max_velo.init	();
	motion_plan.end_radian.init		();
	motion_plan.radian_accel.init	();
	motion_plan.radian_deccel.init	();
	motion_plan.turn_r_min.init		();
	motion_plan.turn_state.init		();
	motion_plan.turn_time_ms.init		();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	motion_pattern_set(Diagonal);
	motion_exeStatus_set(execute);
	motion_state_set(DIAGONAL_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

}

void Motion::Init_Motion_pivot_turn	(float rad_target,float rad_acc,float rad_velo,const t_pid_gain *sp_gain ,const t_pid_gain *om_gain  )
{

	motion_plan.velo.set				(0.0f);
	motion_plan.max_velo.set			(0.0f);
	motion_plan.end_velo.set			(0.0f);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set 	(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(rad_acc);
	motion_plan.rad_deccel.set		(-rad_acc);
	motion_plan.rad_max_velo.set		(rad_velo);
	motion_plan.end_radian.set		(rad_target);
	motion_plan.radian_accel.init	();//check
	motion_plan.radian_deccel.init	();//check
	motion_plan.turn_r_min.set		(0.0f);
	motion_plan.turn_state.init			();//check
	motion_plan.turn_time_ms.init		();

	//Set control gain
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	(motion_plan.end_radian.get() > 0) ? motion_pattern_set(Pivot_turn_L) : motion_pattern_set(Pivot_turn_R);
	motion_exeStatus_set(execute);
	motion_state_set(PIVTURN_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

}

void Motion::Init_Motion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set		(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set			(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	motion_pattern_set(run_pt);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
}

void Motion::Init_Motion_turn_out		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set		(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set		(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	motion_pattern_set(run_pt);
	motion_exeStatus_set(execute);
	motion_state_set(DIAGONAL_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
}

void Motion::Init_Motion_long_turn	(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set		(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set		(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	motion_pattern_set(run_pt);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
}

void Motion::Init_Motion_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set		(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set		(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	motion_pattern_set(run_pt);
	motion_exeStatus_set(execute);
	motion_state_set(DIAGONAL_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
}

void Motion::Init_Motion_fix_wall		(float set_time,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	motion_plan.velo.init();
	motion_plan.max_velo.init();
	motion_plan.end_velo.init();
	motion_plan.accel.init();
	motion_plan.deccel.init();
	motion_plan.end_length.init();
	motion_plan.length_accel.init();
	motion_plan.length_deccel.init();

	motion_plan.rad_accel.init();
	motion_plan.rad_deccel.init();
	motion_plan.rad_max_velo.init();
	motion_plan.end_radian.init();
	motion_plan.radian_accel.init();
	motion_plan.radian_deccel.init();
	motion_plan.turn_r_min.init();
	motion_plan.turn_state.init();
	motion_plan.turn_time_ms.init		();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	motion_pattern_set(Fix_wall);
	motion_exeStatus_set(execute);
	motion_state_set(BRAKE_STATE);
	run_time_ms_reset();
	run_time_limit_ms_set(set_time);
}
void Motion::Init_Motion_stop_brake	(float set_time,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	motion_plan.velo.init();
	motion_plan.max_velo.init();
	motion_plan.end_velo.init();
	motion_plan.accel.init();
	motion_plan.deccel.init();
	motion_plan.end_length.init();
	motion_plan.length_accel.init();
	motion_plan.length_deccel.init();

	motion_plan.rad_accel.init();
	motion_plan.rad_deccel.init();
	motion_plan.rad_max_velo.init();
	motion_plan.end_radian.init();
	motion_plan.radian_accel.init();
	motion_plan.radian_deccel.init();
	motion_plan.turn_r_min.init();
	motion_plan.turn_state.init();
	motion_plan.turn_time_ms.init		();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;


	motion_pattern_set(run_brake);
	motion_exeStatus_set(execute);
	motion_state_set(BRAKE_STATE);
	run_time_ms_reset();
	run_time_limit_ms_set(set_time);
}

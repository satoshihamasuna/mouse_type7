/*
 * set_motion.cpp
 *
 *  Created on: 2024/03/17
 *      Author: sato1
 */


#include "../Inc/ctrl_task.h"



void Motion::Init_Motion_free_rotation_set( )
{
	motion_set.velo.init();
	motion_set.max_velo.init();
	motion_set.end_velo.init();
	motion_set.accel.init();
	motion_set.deccel.init();
	motion_set.end_length.init();
	motion_set.length_accel.init();
	motion_set.length_deccel.init();

	motion_set.rad_accel.init();
	motion_set.rad_deccel.init();
	motion_set.rad_max_velo.init();
	motion_set.end_radian.init();
	motion_set.radian_accel.init();
	motion_set.radian_deccel.init();
	motion_set.turn_r_min.init();
	motion_set.turn_state.init();
}

void Motion::Init_Motion_search_straight(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_set.velo.set				(vehicle->ideal.velo.get());
	motion_set.max_velo.set			(max_sp);
	motion_set.end_velo.set			(end_sp);
	motion_set.accel.set			(acc);
	motion_set.deccel.set			((-1.0f)*acc);
	motion_set.end_length.set		(len_target);
	motion_set.length_accel.set 	(1000*(max_sp*max_sp-motion_set.velo.get()*motion_set.velo.get())/(2.0*ABS(motion_set.accel.get())));
	motion_set.length_deccel.set	(1000*(max_sp*max_sp-end_sp*end_sp)/(2.0*ABS(motion_set.deccel.get())));

	motion_set.rad_accel.init		();
	motion_set.rad_deccel.init		();
	motion_set.rad_max_velo.init	();
	motion_set.end_radian.init		();
	motion_set.radian_accel.init	();
	motion_set.radian_deccel.init	();
	motion_set.turn_r_min.init		();
	motion_set.turn_state.init		();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	motion_pattern_set(Search_st_section);
}

void Motion::Init_Motion_search_turn	(const t_param *turn_param,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_set.velo.set				(turn_param->param->velo);
	motion_set.max_velo.set			(turn_param->param->velo);
	motion_set.end_velo.set			(turn_param->param->velo);
	motion_set.accel.set			(0.0f);
	motion_set.deccel.set			(0.0f);
	motion_set.end_length.set		(0.0f);
	motion_set.length_accel.set		(0.0f);
	motion_set.length_deccel.set	(0.0f);

	motion_set.rad_accel.set		(0.0f);//計算できないわけではない
	motion_set.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_set.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_set.end_radian.set		(turn_param->param->degree/180*PI);
	motion_set.radian_accel.set		(0.0f);//計算できないわけではない
	motion_set.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_set.turn_r_min.set		(turn_param->param->r_min);
	motion_set.turn_state.set			(Prev_Turn);

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param = *turn_param;

	(motion_set.end_radian.get() > 0) ? motion_pattern_set(Search_slalom_L): motion_pattern_set(Search_slalom_R);
}

void Motion::Init_Motion_straight		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_set.velo.set				(vehicle->ideal.velo.get());
	motion_set.max_velo.set			(max_sp);
	motion_set.end_velo.set			(end_sp);
	motion_set.accel.set			(acc);
	motion_set.deccel.set			((-1.0f)*acc);
	motion_set.end_length.set		(len_target);
	motion_set.length_accel.set 	(1000*(max_sp*max_sp-motion_set.velo.get()*motion_set.velo.get())/(2.0*ABS(motion_set.accel.get())));
	motion_set.length_deccel.set	(1000*(max_sp*max_sp-end_sp*end_sp)/(2.0*ABS(motion_set.deccel.get())));

	motion_set.rad_accel.init		();
	motion_set.rad_deccel.init		();
	motion_set.rad_max_velo.init	();
	motion_set.end_radian.init		();
	motion_set.radian_accel.init	();
	motion_set.radian_deccel.init	();
	motion_set.turn_r_min.init		();
	motion_set.turn_state.init		();

	//Set control gain
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	motion_pattern_set(Straight);
}

void Motion::Init_Motion_diagonal		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_set.velo.set				(vehicle->ideal.velo.get());
	motion_set.max_velo.set			(max_sp);
	motion_set.end_velo.set			(end_sp);
	motion_set.accel.set			(acc);
	motion_set.deccel.set			((-1.0f)*acc);
	motion_set.end_length.set		(len_target);
	motion_set.length_accel.set 	(1000*(max_sp*max_sp-motion_set.velo.get()*motion_set.velo.get())/(2.0*ABS(motion_set.accel.get())));
	motion_set.length_deccel.set	(1000*(max_sp*max_sp-end_sp*end_sp)/(2.0*ABS(motion_set.deccel.get())));

	motion_set.rad_accel.init		();
	motion_set.rad_deccel.init		();
	motion_set.rad_max_velo.init	();
	motion_set.end_radian.init		();
	motion_set.radian_accel.init	();
	motion_set.radian_deccel.init	();
	motion_set.turn_r_min.init		();
	motion_set.turn_state.init		();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	motion_pattern_set(Diagonal);

}

void Motion::Init_Motion_pivot_turn	(float rad_target,float rad_acc,float rad_velo,const t_pid_gain *sp_gain ,const t_pid_gain *om_gain  )
{

	motion_set.velo.set				(0.0f);
	motion_set.max_velo.set			(0.0f);
	motion_set.end_velo.set			(0.0f);
	motion_set.accel.set			(0.0f);
	motion_set.deccel.set			(0.0f);
	motion_set.end_length.set		(0.0f);
	motion_set.length_accel.set 	(0.0f);
	motion_set.length_deccel.set	(0.0f);

	motion_set.rad_accel.set		(rad_acc);
	motion_set.rad_deccel.set		(-rad_acc);
	motion_set.rad_max_velo.set		(rad_velo);
	motion_set.end_radian.set		(rad_target);
	motion_set.radian_accel.init	();//check
	motion_set.radian_deccel.init	();//check
	motion_set.turn_r_min.set		(0.0f);
	motion_set.turn_state.init			();//check

	//Set control gain
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	(motion_set.end_radian.get() > 0) ? motion_pattern_set(Pivot_turn_L) : motion_pattern_set(Pivot_turn_R);

}

void Motion::Init_Motion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_set.velo.set				(turn_param->param->velo);
	motion_set.max_velo.set			(turn_param->param->velo);
	motion_set.end_velo.set			(turn_param->param->velo);
	motion_set.accel.set			(0.0f);
	motion_set.deccel.set			(0.0f);
	motion_set.end_length.set		(0.0f);
	motion_set.length_accel.set		(0.0f);
	motion_set.length_deccel.set	(0.0f);

	motion_set.rad_accel.set		(0.0f);//計算できないわけではない
	motion_set.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_set.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_set.end_radian.set		(turn_param->param->degree/180*PI);
	motion_set.radian_accel.set		(0.0f);//計算できないわけではない
	motion_set.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_set.turn_r_min.set		(turn_param->param->r_min);
	motion_set.turn_state.set			(Prev_Turn);

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	motion_pattern_set(run_pt);
}

void Motion::Init_Motion_turn_out		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	motion_set.velo.set				(turn_param->param->velo);
	motion_set.max_velo.set			(turn_param->param->velo);
	motion_set.end_velo.set			(turn_param->param->velo);
	motion_set.accel.set			(0.0f);
	motion_set.deccel.set			(0.0f);
	motion_set.end_length.set		(0.0f);
	motion_set.length_accel.set		(0.0f);
	motion_set.length_deccel.set	(0.0f);

	motion_set.rad_accel.set		(0.0f);//計算できないわけではない
	motion_set.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_set.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_set.end_radian.set		(turn_param->param->degree/180*PI);
	motion_set.radian_accel.set		(0.0f);//計算できないわけではない
	motion_set.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_set.turn_r_min.set		(turn_param->param->r_min);
	motion_set.turn_state.set		(Prev_Turn);

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	motion_pattern_set(run_pt);
}

void Motion::Init_Motion_long_turn	(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	motion_set.velo.set				(turn_param->param->velo);
	motion_set.max_velo.set			(turn_param->param->velo);
	motion_set.end_velo.set			(turn_param->param->velo);
	motion_set.accel.set			(0.0f);
	motion_set.deccel.set			(0.0f);
	motion_set.end_length.set		(0.0f);
	motion_set.length_accel.set		(0.0f);
	motion_set.length_deccel.set	(0.0f);

	motion_set.rad_accel.set		(0.0f);//計算できないわけではない
	motion_set.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_set.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_set.end_radian.set		(turn_param->param->degree/180*PI);
	motion_set.radian_accel.set		(0.0f);//計算できないわけではない
	motion_set.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_set.turn_r_min.set		(turn_param->param->r_min);
	motion_set.turn_state.set		(Prev_Turn);

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	motion_pattern_set(run_pt);
}

void Motion::Init_Motion_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	motion_set.velo.set				(turn_param->param->velo);
	motion_set.max_velo.set			(turn_param->param->velo);
	motion_set.end_velo.set			(turn_param->param->velo);
	motion_set.accel.set			(0.0f);
	motion_set.deccel.set			(0.0f);
	motion_set.end_length.set		(0.0f);
	motion_set.length_accel.set		(0.0f);
	motion_set.length_deccel.set	(0.0f);

	motion_set.rad_accel.set		(0.0f);//計算できないわけではない
	motion_set.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_set.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_set.end_radian.set		(turn_param->param->degree/180*PI);
	motion_set.radian_accel.set		(0.0f);//計算できないわけではない
	motion_set.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_set.turn_r_min.set		(turn_param->param->r_min);
	motion_set.turn_state.set		(Prev_Turn);

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	motion_pattern_set(run_pt);
}

void Motion::Init_Motion_fix_wall		(float set_time,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	motion_set.velo.init();
	motion_set.max_velo.init();
	motion_set.end_velo.init();
	motion_set.accel.init();
	motion_set.deccel.init();
	motion_set.end_length.init();
	motion_set.length_accel.init();
	motion_set.length_deccel.init();

	motion_set.rad_accel.init();
	motion_set.rad_deccel.init();
	motion_set.rad_max_velo.init();
	motion_set.end_radian.init();
	motion_set.radian_accel.init();
	motion_set.radian_deccel.init();
	motion_set.turn_r_min.init();
	motion_set.turn_state.init();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	motion_pattern_set(Fix_wall);
}
void Motion::Init_Motion_stop_brake	(float set_time,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	motion_set.velo.init();
	motion_set.max_velo.init();
	motion_set.end_velo.init();
	motion_set.accel.init();
	motion_set.deccel.init();
	motion_set.end_length.init();
	motion_set.length_accel.init();
	motion_set.length_deccel.init();

	motion_set.rad_accel.init();
	motion_set.rad_deccel.init();
	motion_set.rad_max_velo.init();
	motion_set.end_radian.init();
	motion_set.radian_accel.init();
	motion_set.radian_deccel.init();
	motion_set.turn_r_min.init();
	motion_set.turn_state.init();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	motion_pattern_set(run_brake);
}

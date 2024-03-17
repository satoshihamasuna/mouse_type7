/*
 * ctrl.h
 *
 *  Created on: 2024/03/15
 *      Author: sato1
 */

#ifndef CPP_TASK_INC_CTRL_TASK_H_
#define CPP_TASK_INC_CTRL_TASK_H_

#include <stdio.h>
#include "run_task.h"
#include "sensing_task.h"

#include "../../Pheripheral/Include/typedef.h"
#include "../../Params/run_param.h"
#include "../../Component/Inc/singleton.h"
#include "../../Component/Inc/controll.h"
#include "../../Module/Inc/vehicle.h"

class Motion
{
	private:
		t_run_pattern		motion_pattern  = No_run;
		t_run_mode			motion_state	= NOP_MODE;

		t_param 			turn_motion_param;
		t_straight_param 	straight_motion_param;

		struct motion_set_params
		{
				param_element velo;
				param_element max_velo;
				param_element end_velo;
				param_element accel;
				param_element deccel;
				param_element length;
				param_element rad_accel;
				param_element rad_deccel;
				param_element rad_max_velo;
				param_element radian;
				param_element turn_r_min;
				turn_dir_element turn_d;
		};

		typedef enum{
			execute	    = 2,
			complete    = 1,
			error 		= 0,
		}t_exeStatus;

		t_exeStatus		  motion_exeStatus;

		void  SetIdeal_search_straight(Vehicle *vehicle,SensingTask *ir_sen);
		void  SetIdeal_search_turn(Vehicle *vehicle,SensingTask *ir_sen);

		void SetIdeal_straight		(Vehicle *vehicle,SensingTask *ir_sens);
		void SetIdeal_diagonal		(Vehicle *vehicle,SensingTask *ir_sens);

		void SetIdeal_pivot_turn	(Vehicle *vehicle,SensingTask *ir_sens);

		void SetIdeal_turn_in		(Vehicle *vehicle,SensingTask *ir_sens);
		void SetIdeal_turn_out		(Vehicle *vehicle,SensingTask *ir_sens);
		void SetIdeal_long_turn		(Vehicle *vehicle,SensingTask *ir_sens);
		void SetIdeal_turn_v90		(Vehicle *vehicle,SensingTask *ir_sens);

		void SetIdeal_fix_wall		(Vehicle *vehicle,SensingTask *ir_sens);
		void SetIdeal_stop_brake	(Vehicle *vehicle,SensingTask *ir_sens);

	public:

		PID_Setting Gain_while_straight;
		PID_Setting Gain_while_turn;
		motion_set_params motion_set;

		inline t_run_pattern motion_pattern_get() 							 	{	return motion_pattern;				}
		inline void			 motion_pattern_set(t_run_pattern _motion_pattern)  {	motion_pattern	= _motion_pattern;	}

		inline t_run_mode 	 motion_state_get() 							 	{	return motion_state;				}
		inline void			 motion_state_set(t_run_mode _motion_state)   	 	{	motion_state	= _motion_state;	}

		void SetMotion_free_rotation_set	();
		void SetMotion_search_straight(float len_target,float acc,float max_sp,float end_sp);
		void SetMotion_search_turn	(const t_param *turn_param);

		void SetMotion_straight		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void SetMotion_diagonal		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void SetMotion_pivot_turn	(float rad_target,float rad_acc,float rad_velo);

		void SetMotion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void SetMotion_turn_out		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void SetMotion_long_turn	(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void SetMotion_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void SetMotion_fix_wall		(float set_time);
		void SetMotion_stop_brake	();

		inline t_exeStatus 	 motion_exeStatus_get() 							 	{	return motion_exeStatus;				}
		inline void			 motion_exeStatus_set(t_exeStatus _motion_exeStatus)   	{	motion_exeStatus	= _motion_exeStatus;	}
		t_exeStatus 		 motion_execute();

};


class CtrlTask:public Motion
{
	private:
		int error_cnt;
		void motion_ideal_param_set();
		Vehicle *vehicle;
		SensingTask *ir_sens;

	public:
		CtrlTask(Vehicle *_vehicle,SensingTask *_ir_sens)
		{
			vehicle = _vehicle;
			ir_sens = _ir_sens;
		}

		float delta_t;
		float run_time;
		float run_time_limit;

		void motion_prev_controll();
		void motion_controll();
		void motion_post_controll();
		t_bool is_controll_enable();


};


class CtrlTask_type7:public CtrlTask,public Singleton<CtrlTask_type7>{
	public:
		CtrlTask_type7(Vehicle *v = &Vehicle_type7::getInstance(),SensingTask *ir = &SensingTask::getInstance()):CtrlTask(v,ir){}
};

#endif /* CPP_TASK_INC_CTRL_TASK_H_ */

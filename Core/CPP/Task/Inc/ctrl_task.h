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
		t_bool				is_motion_enable	= False;

		t_param 			turn_motion_param;
		t_straight_param 	straight_motion_param;

		int deltaT_ms;
		Vehicle *vehicle;
		SensingTask *ir_sens;

		struct motion_set_params
		{
				param_element velo;
				param_element max_velo;
				param_element end_velo;
				param_element accel;
				param_element deccel;
				param_element end_length;
				param_element length_accel;
				param_element length_deccel;
				param_element rad_accel;
				param_element rad_deccel;
				param_element rad_max_velo;
				param_element end_radian;
				param_element radian_accel;
				param_element radian_deccel;
				param_element turn_r_min;
				turn_dir_element turn_state;
		};

		typedef enum{
			execute	    = 2,
			complete    = 1,
			error 		= 0,
		}t_exeStatus;

		t_exeStatus		  motion_exeStatus;

		motion_set_params motion_set;

		float delta_t;
		float run_time;
		float run_time_limit;

		void Motion_EndSetting_turn();
		void Motion_EndSetting_straight();

	protected:
		//update Ideal parameters
		void  SetIdeal_search_straight();
		void  SetIdeal_search_turn();

		void SetIdeal_straight		();
		void SetIdeal_diagonal		();

		void SetIdeal_pivot_turn	();

		void SetIdeal_turn_in		();
		void SetIdeal_turn_out		();
		void SetIdeal_long_turn		();
		void SetIdeal_turn_v90		();

		void SetIdeal_fix_wall		();
		void SetIdeal_stop_brake	();

		void SetIdeal_free_rotation_set	( );

	public:

		Motion(Vehicle *_vehicle,SensingTask *_ir_sens,int _deltaT_ms)
		{
			vehicle = _vehicle;
			ir_sens = _ir_sens;
			deltaT_ms = _deltaT_ms;
		}


		inline t_run_pattern motion_pattern_get() 							 	{	return motion_pattern;				}
		inline void			 motion_pattern_set(t_run_pattern _motion_pattern)  {	motion_pattern	= _motion_pattern;	}

		inline t_run_mode 	 motion_state_get() 							 	{	return motion_state;				}
		inline void			 motion_state_set(t_run_mode _motion_state)   	 	{	motion_state	= _motion_state;	}


		//Initialize motion parameters
		void Init_Motion_free_rotation_set	();
		void Init_Motion_search_straight(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_search_turn	(const t_param *turn_param,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void Init_Motion_straight		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_diagonal		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void Init_Motion_pivot_turn	(float rad_target,float rad_acc,float rad_velo,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void Init_Motion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_turn_out		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_long_turn	(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void Init_Motion_fix_wall		(float set_time,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_stop_brake	(float set_time,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		inline t_exeStatus 	 motion_exeStatus_get() 							 	{	return motion_exeStatus;				}
		inline void			 motion_exeStatus_set(t_exeStatus _motion_exeStatus)   	{	motion_exeStatus	= _motion_exeStatus;	}
		t_exeStatus 		 motion_execute();

		inline	void 		motion_enable_set()										{ 	is_motion_enable		= True;		}
		inline	void 		motion_disable_set()									{ 	is_motion_enable		= False;	}
		inline  t_bool 		motion_is_enable_get()									{	return	is_motion_enable;			}
};


class CtrlTask:public Motion
{
	private:
		int error_cnt;
		void motion_ideal_param_set();
		Vehicle *vehicle;
		SensingTask *ir_sens;
		int ctr_deltaT_ms;
	public:
		CtrlTask(Vehicle *_vehicle,SensingTask *_ir_sens,int _ctr_deltaT_ms):Motion(_vehicle,_ir_sens,_ctr_deltaT_ms)
		{
			vehicle = _vehicle;
			ir_sens = _ir_sens;
			ctr_deltaT_ms = _ctr_deltaT_ms;
		}

		void motion_prev_controll();
		void motion_controll();
		void motion_post_controll();
		inline t_bool is_controll_enable()	{	return	motion_is_enable_get();	}


};


class CtrlTask_type7:public CtrlTask,public Singleton<CtrlTask_type7>{
	public:
		CtrlTask_type7(Vehicle *v = &Vehicle_type7::getInstance(),SensingTask *ir = &SensingTask::getInstance()):CtrlTask(v,ir,1){}
};

#endif /* CPP_TASK_INC_CTRL_TASK_H_ */

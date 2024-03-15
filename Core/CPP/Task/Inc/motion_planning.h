/*
 * motion_planning.h
 *
 *  Created on: 2024/03/16
 *      Author: sato1
 */

#ifndef CPP_TASK_INC_MOTION_PLANNING_H_
#define CPP_TASK_INC_MOTION_PLANNING_H_


#include <stdio.h>
#include "run_task.h"
#include "ctrl_task.h"

#include "../../Pheripheral/Include/typedef.h"
#include "../../Params/run_param.h"
#include "../../Component/Inc/singleton.h"
#include "../../Component/Inc/controll.h"
#include "../../Module/Inc/vehicle.h"


class motion_planning
{
	private:
		ctrl_task* ctrl_task;
	public:
		motion_planning(ctrl_task* ctrl_task_)
		{
			ctrl_task = ctrl_task_;
		}
		void motion_start();
		void free_rotation();
		void search_straight(float len_target,float acc,float max_sp,float end_sp);
		void straight(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void diagonal(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void pivot_turn(float rad_target,float rad_acc,float rad_velo);
		void searchSlalom(const t_param *turn_param);
		void turn_in(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void turn_out(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void long_turn(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void turn_v90(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void fix_wall(float set_time);
		void stop_brake();
		t_bool motion_execute();
		t_bool motion_error();
};

#endif /* CPP_TASK_INC_MOTION_PLANNING_H_ */

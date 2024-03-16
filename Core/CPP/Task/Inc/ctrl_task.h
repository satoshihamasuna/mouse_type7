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

class MotionParams
{
	private:
		t_run_pattern		motion_pattern  = No_run;
		t_run_mode			motion_state	= NOP_MODE;

		t_param 			turn_motion_param;
		t_straight_param 	straight_motion_param;

	public:
		PID_Setting Gain_while_straight;
		PID_Setting Gain_while_turn;

		inline t_run_pattern motion_pattern_get() 							 	{	return motion_pattern;				}
		inline void			 motion_pattern_set(t_run_pattern _motion_pattern)  {	motion_pattern	= _motion_pattern;		}

		inline t_run_mode 	 motion_state_get() 							 	{	return motion_state;				}
		inline void			 motion_state_set(t_run_mode _motion_state)   	 	{	motion_state	= _motion_state;		}
};

class CtrlTask
{
private:
	int error_cnt;
	t_run_pattern	run_pattern = No_run;
	t_run_mode		run_state	= NOP_MODE;
	void motion_ideal_param_set();
public:
	Vehicle *vehicle;
	SensingTask *ir_sens;
	MotionParams motion_param;
	CtrlTask(Vehicle *_vehicle,SensingTask *_ir_sens)
	{
		vehicle = _vehicle;
		ir_sens = _ir_sens;
	}

	float delta_t;
	float run_time;
	float run_time_limit;

	void motion_param_initial_set();
	void motion_prev_controll();
	void motion_controll();
	void motion_post_controll();
	t_bool is_controll_enable();

	inline t_run_pattern run_pattern_get() 							 {	return run_pattern;				}
	inline void			 run_pattern_set(t_run_pattern _run_pattern) {	run_pattern	= _run_pattern;		}

	inline t_run_mode 	 run_state_get() 							 {	return run_state;				}
	inline void			 run_state_set(t_run_mode _run_state)   	 {	run_state	= _run_state;		}
};


class CtrlTask_type7:public CtrlTask,public Singleton<CtrlTask_type7>{
	public:
		CtrlTask_type7(Vehicle *v = &Vehicle_type7::getInstance(),SensingTask *ir = &SensingTask::getInstance()):CtrlTask(v,ir){}
};

#endif /* CPP_TASK_INC_CTRL_TASK_H_ */

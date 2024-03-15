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

#include "../../Pheripheral/Include/typedef.h"
#include "../../Params/run_param.h"
#include "../../Component/Inc/singleton.h"
#include "../../Component/Inc/controll.h"
#include "../../Module/Inc/vehicle.h"

class ctrl_task
{
private:
	int error_cnt;
	t_run_pattern	run_pattern = No_run;
	t_run_mode		run_state	= NOP_MODE;
	void ideal_param_set();
public:
	Vehicle *vehicle;
	ctrl_task(Vehicle *_vehicle)
	{
		vehicle = _vehicle;
	}
	PID_Setting Gain_while_straight;
	PID_Setting Gain_while_turn;

	float delta_t;
	float run_time;
	float run_time_limit;

	const t_param *turn_param = nullptr;

	void motionPrevControll();
	void motionControll();
	void motionPostControll();
	t_bool isControllEnable();

};


class ctrl_task_type7:public ctrl_task,public Singleton<ctrl_task_type7>{
	public:
    ctrl_task_type7(Vehicle *v = &Vehicle_type7::getInstance()):ctrl_task(v){}
};

#endif /* CPP_TASK_INC_CTRL_TASK_H_ */

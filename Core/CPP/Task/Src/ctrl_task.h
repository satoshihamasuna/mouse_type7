/*
 * ctrl_task.h
 *
 *  Created on: 2024/03/15
 *      Author: sato1
 */

#include <stdio.h>
#include "run_task.h"

#include "../../Pheripheral/Include/typedef.h"
#include "../../Component/Inc/singleton.h"
#include "../../Component/Inc/controll.h"

#include "../../Module/Inc/vehicle.h"

#include "../../Params/run_param.h"

#include "../Inc/ctrl_task.h"

void ctrl_task::ideal_param_set()
{

}

void ctrl_task::motionPrevControll()
{
	ideal_param_set();

	switch(rT.get_run_mode_state())
	{
		case NOP_MODE:
		case TURN_MODE:
		case SPIN_TURN_MODE:
			ct.speed_ctrl.Gain_Set(turn_gain_set.get_sp_gain().Kp
								  ,turn_gain_set.get_sp_gain().Ki
								  ,turn_gain_set.get_sp_gain().Kd);

			ct.omega_ctrl.Gain_Set(turn_gain_set.get_om_gain().Kp
								  ,turn_gain_set.get_om_gain().Ki
								  ,turn_gain_set.get_om_gain().Kd);
			break;
		case STRAIGHT_MODE:
		case DIAGONAL_MODE:
			ct.speed_ctrl.Gain_Set(straight_gain_set.get_sp_gain().Kp
								  ,straight_gain_set.get_sp_gain().Ki
								  ,straight_gain_set.get_sp_gain().Kd);

			ct.omega_ctrl.Gain_Set(straight_gain_set.get_om_gain().Kp
								  ,straight_gain_set.get_om_gain().Ki
								  ,straight_gain_set.get_om_gain().Kd);

			break;
	}
}

void ctrl_task::motionControll()
{

}

void ctrl_task::motionPostControll()
{

}

t_bool ctrl_task::is_controll_enable()
{

}



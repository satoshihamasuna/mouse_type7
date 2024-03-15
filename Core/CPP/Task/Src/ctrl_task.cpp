/*
 * ctrl_task.cpp
 *
 *  Created on: 2024/03/15
 *      Author: sato1
 */

#include <stdio.h>
#include "../Inc/run_task.h"

#include "../../Pheripheral/Include/index.h"
#include "../../Pheripheral/Include/typedef.h"

#include "../../Component/Inc/singleton.h"
#include "../../Component/Inc/controll.h"
#include "../../Component/Inc/path_follow.h"

#include "../../Module/Inc/vehicle.h"

#include "../../Params/run_param.h"

#include "../Inc/ctrl_task.h"
#include "../Inc/motion.h"
#include "../Inc/run_task.h"
#include "../Inc/sensing_task.h"

void ctrl_task::ideal_param_set()
{

}

void ctrl_task::motion_prev_controll()
{
	ideal_param_set();
/*
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
*/
}

void ctrl_task::motion_controll()
{
	vehicle->V_r = 0.0f;			vehicle->V_l = 0.0f;
	vehicle->motor_out_r = 0;		vehicle->motor_out_l = 0;

	if(is_controll_enable() == True && run_pattern_get() != motor_free)
	{
		//calc motor speed(rpm)
		float motor_r_rpm = (1.0f)*RAD_2_RPM*GEAR_N*(vehicle->ideal.velo.get()*1000/TIRE_RADIUS
													+ 1.0f*TREAD_WIDTH*vehicle->ideal.rad_velo.get()/(2*TIRE_RADIUS));

		float motor_l_rpm = (1.0f)*RAD_2_RPM*GEAR_N*(vehicle->ideal.velo.get()*1000/TIRE_RADIUS
													- 1.0f*TREAD_WIDTH*vehicle->ideal.rad_velo.get()/(2*TIRE_RADIUS));
		//calc friction()
		float friction = 0.0f;
		if(run_pattern_get() != Fix_wall || run_pattern_get() != run_brake)
			friction = (ABS(vehicle->ideal.velo.get()) > 0.15) ? (float)(SIGN(vehicle->ideal.velo.get()))*0.05*9.8/(1+0.0*ABS(vehicle->ideal.accel.get())):0.0f;

		//calc motor induce ampere
		float motor_r_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*(vehicle->ideal.accel.get()+friction)/1000*(TIRE_RADIUS)/2 + MOUSE_INERTIA*vehicle->ideal.rad_accel.get() *TIRE_RADIUS/(TREAD_WIDTH/2.0f)) +  MOTOR_BR*motor_r_rpm/MOTOR_K_TR*0.0;
		float motor_l_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*(vehicle->ideal.accel.get()+friction)/1000*(TIRE_RADIUS)/2 - MOUSE_INERTIA*vehicle->ideal.rad_accel.get() *TIRE_RADIUS/(TREAD_WIDTH/2.0f)) +  MOTOR_BR*motor_l_rpm/MOTOR_K_TR*0.0;

		//calc motor induce voltage
		float sp_FF_controll_r =  MOTOR_R*motor_r_ampere + MOTOR_K_ER*motor_r_rpm/1000;
		float sp_FF_controll_l =  MOTOR_R*motor_l_ampere + MOTOR_K_ER*motor_l_rpm/1000;


//		if(rT.get_run_mode_state() == TURN_MODE)
//		{
//			//omega_controll_input = path_follow_class::getInstance().calc_control_yaw_rate(target.velo, mouse.velo, target.rad_velo, mouse.rad_velo);
//		}
//		else
//		{
//			//path_follow_class::getInstance().reset_position_error();
//			//path_follow_class::getInstance().reset_yaw_theta_error();
//		}

		//feedback controll
		vehicle->sp_feedback.set(vehicle->Vehicle_controller.speed_ctrl.Controll(vehicle->ideal.velo.get()		,vehicle->ego.velo.get()	, 1.0));
		vehicle->om_feedback.set(vehicle->Vehicle_controller.omega_ctrl.Controll(vehicle->ideal.rad_velo.get()	,vehicle->ego.rad_velo.get(), 1.0));

		//feedforward controll
		vehicle->sp_feedforward.set((sp_FF_controll_r + sp_FF_controll_l)/2.0);
		vehicle->om_feedforward.set((sp_FF_controll_r - sp_FF_controll_l)/2.0);

		//calc anti windup
		float ctrl_battery = vehicle->battery.get();
		if(ctrl_battery < 3.30f) ctrl_battery = 3.30f;

		float ctrl_limit = MAX( ABS( vehicle->sp_feedforward.get() + vehicle->om_feedforward.get()),
								ABS(-vehicle->sp_feedforward.get() + vehicle->om_feedforward.get()));

		if( ctrl_limit < ctrl_battery )
		{
			vehicle->sp_feedback.set(vehicle->Vehicle_controller.speed_ctrl.Anti_windup_1( vehicle->sp_feedforward.get(),ctrl_battery - ctrl_limit));
			vehicle->om_feedback.set(vehicle->Vehicle_controller.omega_ctrl.Anti_windup_2( vehicle->om_feedforward.get(),ctrl_battery - ctrl_limit));
		}
		else
		{
			vehicle->sp_feedback.set(vehicle->Vehicle_controller.speed_ctrl.Anti_windup_1( 	 vehicle->sp_feedback.get() + vehicle->sp_feedforward.get()
																							,ctrl_battery - vehicle->sp_feedforward.get()
																						  ));
			vehicle->om_feedback.set(vehicle->Vehicle_controller.omega_ctrl.Anti_windup_2( 	 vehicle->om_feedback.get() + vehicle->om_feedforward.get()
																							,ctrl_battery - vehicle->om_feedforward.get()
																						  ));
		}


		//
		vehicle->V_r =  vehicle->sp_feedforward.get() + vehicle->om_feedforward.get() + vehicle->sp_feedback.get() + vehicle->om_feedback.get();
		vehicle->V_l = -vehicle->sp_feedforward.get() + vehicle->om_feedforward.get() - vehicle->sp_feedback.get() + vehicle->om_feedback.get();

		float duty_r = vehicle->V_r/vehicle->battery.get();
		float duty_l = vehicle->V_l/vehicle->battery.get();

		if(ABS(duty_r) > 1.0){
			vehicle->motor_out_r = (int)(SIGN(duty_r) * 4.0f * 250.0f);
		}else{
			vehicle->motor_out_r  = (int)(duty_r * 1000.0f);
		}
		Motor_SetDuty_Right(vehicle->motor_out_r);

		if(ABS(duty_l) > 1.0){
			vehicle->motor_out_l = (int)(SIGN(duty_l) * 4.0f * 250.0f);
		}else{
			vehicle->motor_out_l = (int)(duty_l * 1000.0f);
		}

		Motor_SetDuty_Left(vehicle->motor_out_l);

	}

}

void ctrl_task::motion_post_controll()
{

}

t_bool ctrl_task::is_controll_enable()
{

}



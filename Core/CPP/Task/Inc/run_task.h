/*
 * run_task.h
 *
 *  Created on: 2023/06/16
 *      Author: sato1
 */

#ifndef CPP_INC_RUN_TASK_H_
#define CPP_INC_RUN_TASK_H_

#include "../../Pheripheral/Include/typedef.h"
#include "../../Module/Inc/vehicle.h"
#include "../../Component/Inc/controll.h"


typedef enum{
	No_run				= 0,
	Straight 			= 1,
	Diagonal			= 2,
	Long_turnR90		= 3,
	Long_turnL90		= 4,
	Long_turnR180		= 5,
	Long_turnL180		= 6,
	Turn_in_R45			= 7,
	Turn_in_L45			= 8,
	Turn_out_R45		= 9,
	Turn_out_L45		= 10,
	Turn_in_R135		= 11,
	Turn_in_L135		= 12,
	Turn_out_R135		= 13,
	Turn_out_L135		= 14,
	Turn_RV90			= 15,
	Turn_LV90			= 16,
	Diagonal_R			= 17,
	Diagonal_L			= 18,
	Search_st_section	= 19,
	Search_st_half		= 20,
	Pivot_turn_R		= 21,
	Pivot_turn_L		= 22,
	Search_slalom_R		= 23,
	Search_slalom_L		= 24,
	run_brake			= 25,
	motor_free			= 26,
	Fix_wall			= 27,
}t_run_pattern;

typedef enum{
	NOP_MODE 		= 0,
	STRAIGHT_MODE 	= 1,
	DIAGONAL_MODE 	= 2,
	TURN_MODE 		= 3,
	SPIN_TURN_MODE  = 4,
}t_run_mode;

typedef struct{
	float velo;
	float r_min;
	float Lstart;
	float Lend;
	float degree;
	t_turn_dir turn_dir;
}t_turn_param_table;

typedef struct{
	t_turn_param_table const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_param;

typedef struct{
	//float base_velo;
	float max_velo;
	float acc;
}t_velo_param;

typedef struct{
	t_velo_param const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_straight_param;

typedef enum
{
	Non_controll = 0,
	Enable_st = 1,
	Enable_di = 2,
}t_wall_controll;

class PID_Setting
{
	private:
		t_pid_gain sp_gain;
		t_pid_gain om_gain;
	public:
		void set_gain(t_pid_gain _sp_gain,t_pid_gain _om_gain)
		{
			sp_gain = _sp_gain;
			om_gain = _om_gain;
		}
		void set_sp_gain(float _kp,float _ki,float _kd)
		{
			sp_gain.Kp = _kp;
			sp_gain.Ki = _ki;
			sp_gain.Kd = _kd;
		}
		void set_om_gain(float _kp,float _ki,float _kd)
		{
			om_gain.Kp = _kp;
			om_gain.Ki = _ki;
			om_gain.Kd = _kd;
		}

		t_pid_gain get_sp_gain()
		{
			return sp_gain;
		}

		t_pid_gain get_om_gain()
		{
			return om_gain;
		}
};





#endif /* CPP_INC_RUN_TASK_H_ */

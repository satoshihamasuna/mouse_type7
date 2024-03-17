/*
 * vhecle.h
 *
 *  Created on: 2024/03/14
 *      Author: sato1
 */

#ifndef CPP_MODULE_INC_VEHICLE_H_
#define CPP_MODULE_INC_VEHICLE_H_

#include "../../Pheripheral/Include/typedef.h"
//#include "../../Component/Inc/half_float.h"
#include "../../Component/Inc/singleton.h"
#include "../../Component/Inc/controll.h"

typedef struct{
	float velo;
	float accel;
	float length;
	float rad_accel;
	float rad_velo;
	float radian;
	float x_point;
	float turn_x;
	float turn_x_dash;
	float turn_y;
	float turn_y_dash;
	float turn_slip_theta;
	float turn_slip_dot;
}t_machine_param;

typedef enum{
	Turn_None 	= 0,
	Turn_R 		= 1,
	Turn_L		= 2,
	Prev_Turn	= 3,
	Post_Turn	= 4,
}t_turn_dir;

typedef struct{
	float velo;
	float max_velo;
	float end_velo;
	float accel;
	float deccel;
	float length;
	float rad_accel;
	float rad_deccel;
	float rad_max_velo;
	float radian;
	t_turn_dir turn_d;
}t_motion_param;

class param_element
{
	private:
		float param;
	public:
		inline void set(float value) 	{	param = value;	};
		inline void init()				{	param = 0.0f;	};
		inline float get()				{   return param;	};
};

class turn_dir_element
{
	private:
		t_turn_dir param;
	public:
		inline void set(t_turn_dir dir)	{	param = dir;	};
		inline void init()				{	param = Turn_None;	};
		inline t_turn_dir get()			{   return param;	};
};

struct machine_params
{
		param_element velo;
		param_element accel;
		param_element horizon_accel;
		param_element z_accel;
		param_element length;
		param_element rad_accel;
		param_element rad_velo;
		param_element radian;
		param_element x_point;
		param_element turn_x;
		param_element turn_x_dash;
		param_element turn_y;
		param_element turn_y_dash;
		param_element turn_slip_theta;
		param_element turn_slip_dot;
};

class Vehicle
{
	public:
		machine_params ego;
		void ego_initialize()
		{
			ego.velo.init();
			ego.accel.init();
			ego.horizon_accel.init();
			ego.z_accel.init();
			ego.length.init();
			ego.rad_accel.init();
			ego.rad_velo.init();
			ego.radian.init();
			ego.x_point.init();
			ego.turn_x.init();
			ego.turn_x_dash.init();
			ego.turn_y.init();
			ego.turn_y_dash.init();
			ego.turn_slip_theta.init();
			ego.turn_slip_dot.init();
		}

		machine_params ideal;
		void ideal_initialize()
		{
			ideal.velo.init();
			ideal.accel.init();
			ideal.horizon_accel.init();
			ideal.z_accel.init();
			ideal.length.init();
			ideal.rad_accel.init();
			ideal.rad_velo.init();
			ideal.radian.init();
			ideal.x_point.init();
			ideal.turn_x.init();
			ideal.turn_x_dash.init();
			ideal.turn_y.init();
			ideal.turn_y_dash.init();
			ideal.turn_slip_theta.init();
			ideal.turn_slip_dot.init();
		}

		/*
		motion_set_params motion_set;
		void motion_set_initialize()
		{
			motion_set.velo.init();
			motion_set.max_velo.init();
			motion_set.end_velo.init();
			motion_set.accel.init();
			motion_set.deccel.init();
			motion_set.length.init();
			motion_set.rad_accel.init();
			motion_set.rad_deccel.init();
			motion_set.rad_max_velo.init();
			motion_set.radian.init();
			motion_set.turn_d.init();
		}
		*/

		param_element battery;

		mouse_Controll Vehicle_controller;
		float V_r,V_l;
		int motor_out_r,motor_out_l;
		param_element sp_feedforward,sp_feedback;
		param_element om_feedforward,om_feedback;

};

class Vehicle_type7:public Vehicle,public Singleton<Vehicle_type7>{};

#endif /* CPP_MODULE_INC_VEHICLE_H_ */

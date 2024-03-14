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

class param_element
{
	private:
		float param;
	public:
		void set(float value) 	{	param = value;	};
		void init()				{	param = 0.0f;	};
		float get()				{   return param;	};
};

class machine_params
{
	public:
		param_element velo;
		param_element accel;
		param_element horizon_accel;
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

class Vehicle:public Singleton<Vehicle>
{
	public:
		machine_params ego;
		void ego_initialize()
		{
			ego.velo.init();
			ego.accel.init();
			ego.horizon_accel.init();
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
};

#endif /* CPP_MODULE_INC_VEHICLE_H_ */

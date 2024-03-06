/*
 * sensing_task.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SENSING_TASK_H_
#define CPP_INC_SENSING_TASK_H_

#include "../Component/Inc/singleton.h"
#include "run_task.h"
#include "../Pheripheral/Include/typedef.h"


#define SIDE_THRESHOLD		(65.0)
#define FRONT_THRESHOLD		(122.0)

typedef struct{
	int16_t value;
	t_bool is_wall;
	t_bool is_controll;
	float distance;
	float controll_th;
	uint16_t controll_cnt;
	float error;
	int16_t value_log[20];
	int value_sum;
	float avg_distance;
}t_sensor;

class SensingTask:public Singleton<SensingTask>
{
	private:
		float Sensor_CalcDistance(t_sensor_dir dir,int16_t value);
		float IrSensor_adc2voltage(int16_t value);
		float IrSensor_Vce(int16_t value);
		float IrSensor_SensingCurrent(int16_t value);
		float IrSensor_RelativeCurrent(int16_t value);

	public:
		t_sensor sen_fr,sen_fl,sen_r,sen_l;
		t_bool r_check,l_check,wall_correction;

		t_wall_state conv_Sensin2Wall(t_sensor_dir sens_dir);
		float IrSensor_Irradiance(int16_t value);
		void IrSensorSet();
		void IrSensorDistanceSet();
		int16_t IrSensor_Avg();
		void IrSensorWallSet();
		void SetWallControll_RadVelo(t_machine_param *target_,t_machine_param *machine_,float delta_tms);
		t_bool Division_Wall_Correction()
		{
			t_bool flag = False;
			if(sen_r.is_wall == False && r_check == True && wall_correction == False)
			{
				flag = True;
				wall_correction = True;
			}
			if(sen_l.is_wall == False && l_check == True && wall_correction == False)
			{
				flag = True;
				wall_correction = True;
			}

			r_check = sen_r.is_wall;l_check = sen_l.is_wall;
			return flag;
		}
		void Division_Wall_Correction_Reset()
		{
			r_check = l_check = wall_correction = False;
		}
};


#endif /* CPP_INC_SENSING_TASK_H_ */

/*
 * sensing_task.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SENSING_TASK_H_
#define CPP_INC_SENSING_TASK_H_

#include "run_task.h"
//#include "ctrl_task.h"

#include "../../Component/Inc/singleton.h"
#include "../../Pheripheral/Include/typedef.h"

#define STRAIGHT_REF		(45.0)
#define DIAGONAL_REF		(32.0)

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


class IrSensTask
{
	private:
		float Sensor_CalcDistance(t_sensor_dir dir,int16_t value);
		float IrSensor_adc2voltage(int16_t value);
		float IrSensor_Vce(int16_t value);
		float IrSensor_SensingCurrent(int16_t value);
		float IrSensor_RelativeCurrent(int16_t value);
		float IrSensor_Irradiance(int16_t value);
		float	 wall_ref = STRAIGHT_REF;
		t_bool	 isEnableIrSens = False;
	public:
		t_sensor sen_fr,sen_fl,sen_r,sen_l;
		t_bool 	 r_check,l_check,wall_correction;
		t_wall_state conv_Sensin2Wall(t_sensor_dir sens_dir);
		virtual 		void IrSensorSet();
		void IrSensorReferenceSet(float ref_value);
		void IrSensorDistanceSet();
		void IrSensorWallSet();
		void SetWallControll_RadVelo(Vehicle *vehicle,float delta_tms);
		inline void EnableIrSens()		{isEnableIrSens = True;}
		inline void DisableIrSens()			{isEnableIrSens = False;}
		void EnableIrSensStraight()		{	EnableIrSens();		IrSensorReferenceSet(STRAIGHT_REF);	}
		void EnableIrSensDiagonal()		{	EnableIrSens();		IrSensorReferenceSet(DIAGONAL_REF);	}
		int16_t IrSensor_Avg();
};

class IrSensTask_type7: public IrSensTask,public Singleton<IrSensTask_type7>
{
	public:
		void IrSensorSet() override
		{
			sen_fl.value =  Sensor_GetValue(sensor_fl);
			sen_fr.value =  Sensor_GetValue(sensor_fr);
			sen_l.value  =  Sensor_GetValue(sensor_sl);
			sen_r.value  =  Sensor_GetValue(sensor_sr);
			IrSensorDistanceSet();
			IrSensorWallSet();
		}

};

#endif /* CPP_INC_SENSING_TASK_H_ */

/*
 * ir_sens_task.h
 *
 *  Created on: 2024/03/21
 *      Author: sato1
 */


/*
 * sensing_task.cpp
 *
 *  Created on: 2023/06/14
 *      Author: sato1
 */


#include "../Inc/sensing_task.h"
#include "../../Params/sens_table.h"
#include "../../Pheripheral/Include/typedef.h"
#include "../../Pheripheral/Include/index.h"
#include "../Inc/motion.h"

t_wall_state IrSensTask::conv_Sensin2Wall(t_sensor_dir sens_dir)
{
	switch(sens_dir){
		case sensor_fl:
			return ((sen_fl.is_wall)?WALL:NOWALL);
		case sensor_fr:
			return ((sen_fr.is_wall)?WALL:NOWALL);
		case sensor_sl:
			return ((sen_l.is_wall)?WALL:NOWALL);
		case sensor_sr:
			return ((sen_r.is_wall)?WALL:NOWALL);
		default :
			return NOWALL;
	}
}

void IrSensTask::IrSensorSet()
{
	sen_fl.value =  100;//Sensor_GetValue(sensor_fl);
	sen_fr.value =  100;//Sensor_GetValue(sensor_fr);
	sen_l.value  =  100;//Sensor_GetValue(sensor_sl);
	sen_r.value  =  100;//Sensor_GetValue(sensor_sr);
	IrSensorDistanceSet();
	IrSensorWallSet();
}

float IrSensTask::IrSensor_adc2voltage(int16_t value)
{
	return (float)(value)/4096.0*3.3;
}

float IrSensTask::IrSensor_Vce(int16_t value)
{
	return MAX(0.01,(3.30f - IrSensor_adc2voltage(value)));
}

float IrSensTask::IrSensor_SensingCurrent(int16_t value)
{
	return (IrSensor_adc2voltage(value))/1000.0;
}

float IrSensTask::IrSensor_RelativeCurrent(int16_t value)
{
	return (IrSensor_SensingCurrent(value))/((6.4)/5*IrSensor_Vce(value))*1000.0f;
}

float IrSensTask::IrSensor_Irradiance(int16_t value)
{
	float irradiance = IrSensor_RelativeCurrent(value);
	if(irradiance > 4.0)
	{
		irradiance = irradiance * 2 -4.0f;
	}
	return irradiance;
}

float IrSensTask::Sensor_CalcDistance(t_sensor_dir dir,int16_t value)
{
	float distance = 0.0f;
	int array_length = 0;
	int count = 0;
	float m,n;
	switch(dir)
	{
		case sensor_fr:
			array_length = sizeof(sens_front_length_table) / sizeof(uint16_t);
			if(value >= sens_fr_table[0]) distance = (float)sens_front_length_table[0];
			else if (value <= sens_fr_table[array_length-1]) distance = (float)sens_front_length_table[array_length-1];
			else{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_fr_table[count] && value > sens_fr_table[count+1]) break;
				}
				m = (float)(sens_fr_table[count] - value);
				n = (float)(value - sens_fr_table[count+1]);
				distance = (n*(float)sens_front_length_table[count] + m*(float)sens_front_length_table[count+1])/(m+n);
			}
			break;
		case sensor_fl:
			array_length = sizeof(sens_front_length_table) / sizeof(uint16_t);
			if(value >= sens_fl_table[0]) distance = (float)sens_front_length_table[0];
			else if (value <= sens_fl_table[array_length-1]) distance = (float)sens_front_length_table[array_length-1];
			else
			{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_fl_table[count] && value > sens_fl_table[count+1]) break;
				}
				m = (float)(sens_fl_table[count] - value);
				n = (float)(value - sens_fl_table[count+1]);
				distance = (n*(float)sens_front_length_table[count] + m*(float)sens_front_length_table[count+1])/(m+n);
			}
			break;
		case sensor_sr:
			array_length = sizeof(sens_side_length_table) / sizeof(uint16_t);
			if(value >= sens_sr_table[0]) distance = (float)sens_side_length_table[0];
			else if (value <= sens_sr_table[array_length-1]) distance = (float)sens_side_length_table[array_length-1];
			else
			{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_sr_table[count] && value > sens_sr_table[count+1]) break;
				}
				m = (float)(sens_sr_table[count] - value);
				n = (float)(value - sens_sr_table[count+1]);
				distance = (n*(float)sens_side_length_table[count] + m*(float)sens_side_length_table[count+1])/(m+n);
			}
			break;
		case sensor_sl:
			array_length = sizeof(sens_side_length_table) / sizeof(uint16_t);
			if(value >= sens_sl_table[0]) distance = (float)sens_side_length_table[0];
			else if (value <= sens_sl_table[array_length-1]) distance = (float)sens_side_length_table[array_length-1];
			else
			{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_sl_table[count] && value > sens_sl_table[count+1]) break;
				}
				m = (float)(sens_sl_table[count] - value);
				n = (float)(value - sens_sl_table[count+1]);
				distance = (n*(float)sens_side_length_table[count] + m*(float)sens_side_length_table[count+1])/(m+n);
			}
			break;
	}
	return distance;
}


void IrSensTask::IrSensorDistanceSet()
{
	static int i = 0;
	i = i + 1;
	if(i == 20) i = 0;
	sen_fl.value_sum = sen_fl.value_sum - sen_fl.value_log[i%20];
	sen_fr.value_sum = sen_fr.value_sum - sen_fr.value_log[i%20];
	sen_r.value_sum = sen_r.value_sum - sen_r.value_log[i%20];
	sen_l.value_sum = sen_l.value_sum - sen_l.value_log[i%20];

	sen_fl.value_log[i%20] = sen_fl.value;
	sen_fr.value_log[i%20] = sen_fr.value;
	sen_r.value_log[i%20]  = sen_r.value;
	sen_l.value_log[i%20]  = sen_l.value;

	sen_fl.value_sum = sen_fl.value_sum + sen_fl.value_log[i%20];
	sen_fr.value_sum = sen_fr.value_sum + sen_fr.value_log[i%20];
	sen_r.value_sum = sen_r.value_sum + sen_r.value_log[i%20];
	sen_l.value_sum = sen_l.value_sum + sen_l.value_log[i%20];

	sen_fl.distance = Sensor_CalcDistance(sensor_fl,sen_fl.value);
	sen_fr.distance = Sensor_CalcDistance(sensor_fr,sen_fr.value);
	sen_l.distance = Sensor_CalcDistance(sensor_sl,sen_l.value);
	sen_r.distance = Sensor_CalcDistance(sensor_sr,sen_r.value);

	sen_fl.avg_distance = Sensor_CalcDistance(sensor_fl,(int16_t)(sen_fl.value_sum/20));
	sen_fr.avg_distance = Sensor_CalcDistance(sensor_fr,(int16_t)(sen_fr.value_sum/20));
	sen_l.avg_distance = Sensor_CalcDistance(sensor_sl,(int16_t)(sen_l.value_sum/20));
	sen_r.avg_distance = Sensor_CalcDistance(sensor_sr,(int16_t)(sen_r.value_sum/20));
}

void IrSensTask::IrSensorWallSet()
{
	sen_fr.is_wall 	= (sen_fr.distance <= FRONT_THRESHOLD)? True:False;
	sen_fl.is_wall 	= (sen_fl.distance <= FRONT_THRESHOLD)? True:False;
	sen_r.is_wall 	= (sen_r.distance <= SIDE_THRESHOLD)? True:False;
	sen_l.is_wall 	= (sen_l.distance <= SIDE_THRESHOLD)? True:False;


	sen_fr.controll_cnt = (sen_fr.is_wall == True) ? sen_fr.controll_cnt + 1 : 0;
	sen_fl.controll_cnt = (sen_fl.is_wall == True) ? sen_fl.controll_cnt + 1 : 0;
	sen_r.controll_cnt = (sen_r.is_wall == True && ABS(sen_r.distance - sen_r.avg_distance) < 1.0) ? sen_r.controll_cnt + 1 : 0;
	sen_l.controll_cnt = (sen_l.is_wall == True && ABS(sen_l.distance - sen_l.avg_distance) < 1.0) ? sen_l.controll_cnt + 1 : 0;


	sen_fr.controll_th = (sen_fr.controll_cnt > 10) ? FRONT_THRESHOLD : 90.0;
	sen_fl.controll_th = (sen_fl.controll_cnt > 10) ? FRONT_THRESHOLD : 90.0;
	//need to update
	if(isEnableIrSens == True)
	{
		if(wall_ref >= STRAIGHT_REF)
		{
			sen_r.controll_th = (sen_r.controll_cnt > 10) ? SIDE_THRESHOLD: wall_ref;
			sen_l.controll_th = (sen_l.controll_cnt > 10) ? SIDE_THRESHOLD: wall_ref;
		}
		else
		{
			sen_r.controll_th = (sen_r.controll_cnt > 10) ? wall_ref: wall_ref;
			sen_l.controll_th = (sen_l.controll_cnt > 10) ? wall_ref: wall_ref;
		}

		sen_r.is_controll 	= (sen_r.is_wall == True && sen_r.distance <= sen_r.controll_th)? True:False;
		sen_l.is_controll 	= (sen_l.is_wall == True && sen_l.distance <= sen_l.controll_th)? True:False;

		if(wall_ref >= STRAIGHT_REF)
		{
			sen_r.is_controll 	= (sen_fr.distance > SIDE_THRESHOLD+10.0)? sen_r.is_controll:False;
			sen_l.is_controll 	= (sen_fl.distance > SIDE_THRESHOLD+10.0)? sen_l.is_controll:False;
		}
		else
		{
			sen_r.is_controll 	= (sen_fr.distance <= 80.0)? False:sen_r.is_controll;
			sen_l.is_controll 	= (sen_fl.distance <= 80.0)? False:sen_l.is_controll;
		}

		sen_r.error	= (sen_r.is_controll == True) ? sen_r.distance - wall_ref : 0.0;
		sen_l.error	= (sen_l.is_controll == True) ? sen_l.distance - wall_ref : 0.0;
	}
	else
	{

		sen_r.controll_th = DIAGONAL_REF;
		sen_l.controll_th = DIAGONAL_REF;

		sen_r.is_controll 	= False;
		sen_l.is_controll 	= False;

		sen_r.error	=  0.0;
		sen_l.error	=  0.0;
	}
}

void IrSensTask::IrSensorReferenceSet(float ref_value)
{
	 wall_ref = ref_value;
}

void IrSensTask::SetWallControll_RadVelo(Vehicle *vehicle,float delta_tms)
{
	float ir_rad_acc_controll = 0.0;
	const float k1 = 1.0;
	const float k2 = 20.0;
	float s 	= 0.0f;
	float s_dot = 0.0f;

	//sensor_output = k1*ydiff/1000.0 + k2/1000.0*theta;

	if(sen_r.is_controll == True && sen_l.is_controll == True)
	{
		ir_rad_acc_controll = -(sen_l.error - sen_r.error)/2.0;
		vehicle->ego.x_point.set(ir_rad_acc_controll);
	}
	else
	{
		ir_rad_acc_controll = -(sen_l.error - sen_r.error);
		if(sen_r.is_controll == True || sen_l.is_controll == True)
			vehicle->ego.x_point.set((ir_rad_acc_controll+vehicle->ego.x_point.get())/2.0);
	}

	if(sen_r.is_controll == True || sen_l.is_controll == True)
	{
		s 		= ir_rad_acc_controll;
		s_dot 	= k1*vehicle->ideal.velo.get()*1000.0*vehicle->ideal.radian.get()*1.0 + k2*vehicle->ideal.rad_velo.get();
	}

	else
	{
		s 		= k1*vehicle->ego.x_point.get()+k2*vehicle->ego.radian.get();//k2*machine_->radian;//
		s_dot 	= k1*vehicle->ideal.velo.get()*1000.0*vehicle->ideal.radian.get()*1.0 + k2*vehicle->ideal.rad_velo.get();
	}

	float target_rad_acc	= 	(-1.0)*300.0*s/k2 - 60.0*1.0/k2*s_dot
							     - k1/k2*(vehicle->ideal.accel.get()*1000.0*vehicle->ego.radian.get()*1.0
							    		 + vehicle->ideal.velo.get()*vehicle->ego.rad_velo.get()*1000.0);
	float target_rad_velo	= vehicle->ideal.rad_velo.get() + target_rad_acc*delta_tms/1000.0f;
	vehicle->ideal.rad_accel.set(target_rad_acc);
	vehicle->ideal.rad_velo.set(target_rad_velo);

}



int16_t IrSensTask::IrSensor_Avg()
{
	return (sen_l.value + sen_r.value)/2 ;
}



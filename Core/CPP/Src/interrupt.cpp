/*
 * interrupt.c
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */


#include "../Inc/motion.h"
#include <math.h>
#include "../Pheripheral/Include/index.h"
#include "../Inc/interrupt.h"
#include "../Inc/sensing_task.h"
#include "../Inc/log_data.h"
#include "../Component/Inc/controll.h"
#include "../Component/Inc/half_float.h"
#include "../Component/Inc/Kalman_filter.h"
float lambda_slip;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    //if (htim == &htim11){
    	//Interrupt::getInstance().preprocess();
    	//Interrupt::getInstance().main();
    	//Interrupt::getInstance().postprocess();
    //}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	Interrupt::getInstance().preprocess();
	Interrupt::getInstance().main();
	Interrupt::getInstance().postprocess();
}


void Interrupt_Initialize(){
	//HAL_TIM_Base_Start_IT(&htim11);
}

void Interrupt::preprocess(){

	SensingTask::getInstance().IrSensorSet();

	Encoder_SetSpeed_Left();
	Encoder_SetSpeed_Right();
	t_encoder Renc = Encoder_GetProperty_Right();
	t_encoder Lenc = Encoder_GetProperty_Left();
	acc_sum = acc_sum - acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = read_accel_y_axis();
	acc_sum = acc_sum + acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	velo_sum = velo_sum - velo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	velo_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = (Renc.wheel_speed - Lenc.wheel_speed)/2.0;
	velo_sum = velo_sum + velo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	Rvelo_sum = Rvelo_sum - Rvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	Rvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = Renc.wheel_speed;
	Rvelo_sum = Rvelo_sum + Rvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	Lvelo_sum = Lvelo_sum - Lvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	Lvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = Lenc.wheel_speed;
	Lvelo_sum = Lvelo_sum + Lvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];



	acc_time_cnt = (acc_time_cnt == (ACC_BUFF_SIZE-1))? 0:acc_time_cnt + 1;
	float sp = KalmanFilter::getInstance().calc_speed_filter((-1.0)*acc_sum/((float)(ACC_BUFF_SIZE)), velo_sum/((float)(ACC_BUFF_SIZE)));
	controll_task::getInstance().mouse.velo 	  = (-1.0/2.0f)*acc_sum/((float)(ACC_BUFF_SIZE))*((float)(ACC_BUFF_SIZE))/1000.0f+((Renc.wheel_speed) - (Lenc.wheel_speed))/2.0;
	lambda_slip = (MAX(sp,(Renc.wheel_speed - Lenc.wheel_speed)/2.0) == 0.0f) ? 0.0 : (sp-(Renc.wheel_speed - Lenc.wheel_speed)/2.0)/MAX(sp,(Renc.wheel_speed - Lenc.wheel_speed)/2.0);
	if(lambda_slip >= 0.2) lambda_slip = 0.2;
	else if(lambda_slip <= -0.2) lambda_slip = -0.2;
	//(読み間違い対策)
	controll_task::getInstance().mouse.length  += controll_task::getInstance().mouse.velo;//(1.0)*((((float)(Renc.sp_pulse)) - ((float)(Lenc.sp_pulse))))/2.0*MMPP;
	controll_task::getInstance().mouse.accel    = (-1.0)*acc_sum/((float)(ACC_BUFF_SIZE));
	controll_task::getInstance().mouse.rad_velo = (-1.0)*read_gyro_z_axis()*PI/180;
	controll_task::getInstance().mouse.radian  += controll_task::getInstance().mouse.rad_velo/1000.0;
	//if(controll_task::getInstance().run_task == Straight || controll_task::getInstance().run_task == Diagonal || controll_task::getInstance().run_task == Search_st_section )
	//{
	controll_task::getInstance().mouse.x_point += (1.0)*(SIGN(Renc.wheel_speed)*ABS(((float)(Renc.sp_pulse)) - SIGN(Lenc.wheel_speed)*ABS((float)(Lenc.sp_pulse))))/2.0*MMPP*controll_task::getInstance().mouse.radian;
	if(controll_task::getInstance().rT.get_run_mode_state() == TURN_MODE )
	{
		controll_task::getInstance().mouse.turn_slip_dot =  -250.0f*controll_task::getInstance().mouse.turn_slip_theta/controll_task::getInstance().mouse.velo-controll_task::getInstance().mouse.rad_velo;
		controll_task::getInstance().mouse.turn_slip_theta += controll_task::getInstance().mouse.turn_slip_dot/1000.0f;
		controll_task::getInstance().mouse.turn_x_dash = controll_task::getInstance().mouse.velo*sin((-1.0)*(controll_task::getInstance().mouse.radian + controll_task::getInstance().mouse.turn_slip_theta)  );
		controll_task::getInstance().mouse.turn_y_dash = controll_task::getInstance().mouse.velo*cos(controll_task::getInstance().mouse.radian + controll_task::getInstance().mouse.turn_slip_theta );
		controll_task::getInstance().mouse.turn_x += controll_task::getInstance().mouse.turn_x_dash;
		controll_task::getInstance().mouse.turn_y += controll_task::getInstance().mouse.turn_y_dash;
	}
	//}
	//else
	//{
		//controll_task::getInstance().mouse.x_point = 0.0;
	//}
}

void Interrupt::main()
{
	controll_task::getInstance().motion_inInterrupt();
	controll_task::getInstance().motionControll();
}

void Interrupt::postprocess()
{

	if(LogData::getInstance().log_enable == True)
	{

		LogData::getInstance().data[0][LogData::getInstance().data_count%1000] = float_to_half(controll_task::getInstance().mouse.velo);
		LogData::getInstance().data[1][LogData::getInstance().data_count%1000] = float_to_half(controll_task::getInstance().target.velo);
		LogData::getInstance().data[2][LogData::getInstance().data_count%1000] = float_to_half(controll_task::getInstance().mouse.rad_velo);
		LogData::getInstance().data[3][LogData::getInstance().data_count%1000] = float_to_half(controll_task::getInstance().target.rad_velo);
		LogData::getInstance().data[4][LogData::getInstance().data_count%1000] = float_to_half(controll_task::getInstance().mouse.length);
		LogData::getInstance().data[5][LogData::getInstance().data_count%1000] = float_to_half((-1.0)*read_accel_y_axis());//Battery_GetVoltage()  ;
		LogData::getInstance().data[6][LogData::getInstance().data_count%1000] = float_to_half(controll_task::getInstance().mouse.turn_x);
		LogData::getInstance().data[7][LogData::getInstance().data_count%1000] = float_to_half(controll_task::getInstance().mouse.turn_y);
		LogData::getInstance().data[8][LogData::getInstance().data_count%1000] = float_to_half(SensingTask::getInstance().sen_r.distance);//Rvelo_sum/((float)(ACC_BUFF_SIZE));
		LogData::getInstance().data[9][LogData::getInstance().data_count%1000] = float_to_half(SensingTask::getInstance().sen_l.distance);//Lvelo_sum/((float)(ACC_BUFF_SIZE));
		LogData::getInstance().data[10][LogData::getInstance().data_count%1000] = float_to_half(controll_task::getInstance().mouse.x_point);
		LogData::getInstance().data[11][LogData::getInstance().data_count%1000] = float_to_half(controll_task::getInstance().V_l);
		LogData::getInstance().data_count++;
		if(LogData::getInstance().data_count >= 1000) LogData::getInstance().data_count = 999;
	}
	controll_task::getInstance().motionPostControll();
	time_count = time_count + 1;
	IMU_read_DMA_Start();
}


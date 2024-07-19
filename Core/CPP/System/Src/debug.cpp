/*
 * debug.cpp
 *
 *  Created on: 2023/12/03
 *      Author: sato1
 */
#include <stdio.h>

#include "../../Pheripheral/Include/index.h"
#include "../../Pheripheral/Include/macro.h"
#include "../../Pheripheral/Include/typedef.h"

#include "../../Subsys/Inc/search_class.h"
#include "../../Subsys/Inc/make_map_class.h"
#include "../../Subsys/Inc/wall_class.h"
#include "../../Subsys/Inc/make_path.h"

#include "../../Component/Inc/controller.h"
#include "../../Component/Inc/Kalman_filter.h"
#include "../../Component/Inc/queue_class.h"
#include "../../Component/Inc/priority_queue.h"
#include "../../Component/Inc/path_follow.h"

#include "../../Task/Inc/sensing_task.h"
#include "../../Task/Inc/ctrl_task.h"

#include "../../Module/Inc/interrupt.h"
#include "../../Module/Inc/log_data.h"
#include "../../Module/Inc/flash.h"
#include "../../Module/Inc/communicate.h"

#include "../../Params/run_param.h"



#include "../Inc/mode.h"

#define ENABLE (0x01 << 4)

namespace Mode
{
	void Debug(const  t_straight_param *st_param,const t_param *const *turn_mode,int suction)
	{
		t_bool debug_end = False;
		uint8_t mode = Return_LED_Status() & 0x30;
		uint8_t param = 0x00;
		uint8_t enable = 0x00;


		Motion *motion = &(CtrlTask_type7::getInstance());
		IrSensTask *irsens = (CtrlTask_type7::getInstance().return_irObj());

		path_follow_class::getInstance().set_path_follow_gain(50.0, 0.0);
		uint32_t time = Interrupt::getInstance().return_time_count();

		while(debug_end == False)
		{
			enable = Mode::Select(enable,0x01,Encoder_GetProperty_Left());
			param = (enable == 0x00) ? Mode::Select(param,0x0f,Encoder_GetProperty_Right()) : param;

			Battery_LimiterVoltage();
			if(enable == 0x01)
			{
				if((Interrupt::getInstance().return_time_count() - time) > 400)
				{
					time = Interrupt::getInstance().return_time_count();
					Indicate_LED((Return_LED_Status() != (mode|param)) ?  mode|param : 0x00);
				}
			}
			else
			{
				Indicate_LED(mode|param);
			}
			switch((enable<<4)|param)
			{
				case ENABLE|0x00:
				   if(irsens->IrSensor_Avg() > 2000){
					      const static t_pid_gain debug_sp_gain = {16.0,0.1,0.2};
					      const static t_pid_gain debug_om_gain = {0.60f, 0.01f, 0.00f};
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();

						  //float suction_value = 250.0/1000.0f*7.20;
						  //int stay_time 	= (int)(suction_value/0.05) + 300;
						  //motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  //motion->Init_Motion_straight(90.0*7.0,6.0,0.4,0.0,&debug_sp_gain,&debug_om_gain);
						  motion->exe_Motion_backward();
						  //motion->execute_Motion();

						  LogData::getInstance().log_enable = False;
						  motion->Motion_end();

						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  enable = 0x00;
						  HAL_Delay(500);

					}
					break;
				case ENABLE|0x01:
				   if(irsens->IrSensor_Avg() > 2000){
					      const static t_pid_gain debug_sp_gain = {16.0,0.1,0.2};
					      const static t_pid_gain debug_om_gain = {0.60f, 0.01f, 0.00f};
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  KalmanFilter::getInstance().filter_init();
						  motion->Motion_start();
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->Init_Motion_diagonal(DIAG_SECTION*10.0,6.5,0.5,0.0,&debug_sp_gain,&debug_om_gain);
						  motion->execute_Motion();

						  LogData::getInstance().log_enable = False;
						  motion->Motion_end();
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x02:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  KalmanFilter::getInstance().filter_init();

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  motion->exe_Motion_long_turn(turn_mode[Long_turnR180],Long_turnR180,st_param->sp_gain,st_param->om_gain);
						  motion->exe_Motion_long_turn(turn_mode[Long_turnL180],Long_turnL180,st_param->sp_gain,st_param->om_gain);
						  motion->exe_Motion_long_turn(turn_mode[Long_turnR180],Long_turnR180,st_param->sp_gain,st_param->om_gain);
						  motion->exe_Motion_straight(SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x03:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  KalmanFilter::getInstance().filter_init();

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_long_turn(turn_mode[Long_turnR90],Long_turnR90,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_straight( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);

						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x04:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_in(turn_mode[Turn_in_R45],Turn_in_R45,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_diagonal( DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);

						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x05:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }


						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_in(turn_mode[Turn_in_R135],Turn_in_R135,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_diagonal( DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);

						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x06:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_out(turn_mode[Turn_out_R45],Turn_out_R45,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_straight( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);

						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x07:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_out(turn_mode[Turn_out_R135],Turn_out_R135,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_straight( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);


						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x08:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_v90(turn_mode[Turn_RV90],Turn_RV90,st_param->sp_gain,st_param->om_gain);
						  motion->exe_Motion_turn_v90(turn_mode[Turn_LV90],Turn_LV90,st_param->sp_gain,st_param->om_gain);
						  //motion->exe_Motion_turn_v90(turn_mode[Turn_RV90],Turn_RV90,st_param->sp_gain,st_param->om_gain);
						  motion->exe_Motion_diagonal( DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);


						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x09:
				if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);
						  motion->exe_Motion_diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_in(turn_mode[Turn_out_L135],Turn_out_L135,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_straight(SECTION,st_param->param->acc,st_param->param->max_velo,0.0f,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);


						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0A:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_in(turn_mode[Turn_in_L135],Turn_in_L135,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_diagonal( DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);


						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0B:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_out(turn_mode[Turn_out_L45],Turn_out_L45,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_straight( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);


						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0C:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_out(turn_mode[Turn_out_L135],Turn_out_L135,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_straight( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);


						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0D:
					if(irsens->IrSensor_Avg() > 2000){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  motion->Motion_start();
						  float suction_value = suction/1000.0f*7.20;
						  int stay_time 	= (int)(suction_value/0.05) + 300;
						  motion->exe_Motion_suction_start(suction/1000.0f*7.20, stay_time);

						  motion->exe_Motion_diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);

						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->exe_Motion_turn_v90(turn_mode[Turn_LV90],Turn_LV90,st_param->sp_gain,st_param->om_gain);

						  motion->exe_Motion_diagonal( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);

						  motion->Motion_end();
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);


						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0E:
					if(irsens->IrSensor_Avg() > 2000){
						for(int i = 0;i < 11;i++)
						{
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
						}
						LogData::getInstance().indicate_data();
						enable = 0x00;
					}
					break;
				case ENABLE|0x0F:
					if(irsens->IrSensor_Avg() > 2000){
						for(int i = 0;i < 11;i++)
						{
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
						}
						debug_end = True;
					}
					break;
				default:
					break;
			}
		}

	}

}





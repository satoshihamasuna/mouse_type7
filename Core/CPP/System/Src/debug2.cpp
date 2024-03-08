/*
 * debug2.cpp
 *
 *  Created on: 2023/12/03
 *      Author: sato1
 */

#include "../../Pheripheral/Include/index.h"
#include "../../Pheripheral/Include/macro.h"
#include "../../Pheripheral/Include/typedef.h"

#include "../../Subsys/Inc/search_class.h"
#include "../../Subsys/Inc/make_map_class.h"
#include "../../Subsys/Inc/wall_class.h"
#include "../../Subsys/Inc/make_path.h"

#include "../../Component/Inc/controll.h"
#include "../../Component/Inc/Kalman_filter.h"
#include "../../Component/Inc/queue_class.h"
#include "../../Component/Inc/priority_queue.h"
#include "../../Component/Inc/path_follow.h"

#include "../../Task/Inc/sensing_task.h"
#include "../../Task/Inc/motion.h"

#include "../../Module/Inc/interrupt.h"
#include "../../Module/Inc/log_data.h"
#include "../../Module/Inc/flash.h"
#include "../../Module/Inc/communicate.h"

#include "../../Params/run_param.h"

//#include "../../Inc/path_follow.h"

#include "../Inc/mode.h"
#define ENABLE (0x01 << 4)

namespace Mode
{
	void Debug2(const  t_straight_param *st_param,const t_param *const *turn_mode,int suction)
	{
		t_bool debug_end = False;
		uint8_t mode = Return_LED_Status() & 0x30;
		uint8_t param = 0x00;
		uint8_t enable = 0x00;

		ring_queue<1024,t_MapNode> maze_q;
		motion_plan mp(&controll_task::getInstance());
		//Search solve_maze;
		wall_class wall_data(&SensingTask::getInstance());
		wall_data.init_maze();
		make_map map_data(&wall_data,&maze_q);
		Dijkstra run_path(&wall_data);
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
				   if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
							for(int i = 10; i <= suction; i = i + 10)
							{
								FAN_Motor_SetDuty(i);;
								HAL_Delay(3);
							}
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  const  t_straight_param *debug_st_param = &st_param_2200;
						  mp.straight( 90.0*8.0,debug_st_param->param->acc,debug_st_param->param->max_velo,0.0,debug_st_param->sp_gain,debug_st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x01:
				   if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start();
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.straight( 45.0,6.0,0.32,0.32,&search_sp_gain,&search_om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  for(int i = 0;i < 8;i++)
						  {
						  mp.searchSlalom( &param_R90_search);
						  while(controll_task::getInstance().run_task !=No_run){}
						  }
						  mp.straight(45.0,6.0,0.32,0.0,&search_sp_gain,&search_om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x02:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
							for(int i = 10; i <= suction; i = i + 10)
							{
								FAN_Motor_SetDuty(i);;
								HAL_Delay(3);
							}
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}

						  for(int i = 0; i < 4;i++)
						  {
							  mp.long_turn(turn_mode[Long_turnR180],Long_turnR180,st_param->sp_gain,st_param->om_gain);
							  while(controll_task::getInstance().run_task !=No_run){}
						  }
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x03:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  for(int i = 0; i < 4;i++)
						  {
							  mp.long_turn(turn_mode[Long_turnR90],Long_turnR90,st_param->sp_gain,st_param->om_gain);
							  while(controll_task::getInstance().run_task !=No_run){}
						  }
						  mp.straight( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x04:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_in(turn_mode[Turn_in_R45],Turn_in_R45,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.turn_out(turn_mode[Turn_out_L45],Turn_out_L45,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x05:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_in(turn_mode[Turn_in_L45],Turn_in_L45,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.turn_out(turn_mode[Turn_out_R45],Turn_out_R45,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x06:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_in(turn_mode[Turn_in_R135],Turn_in_R135,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.turn_out(turn_mode[Turn_out_L135],Turn_out_L135,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x07:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_in(turn_mode[Turn_in_L135],Turn_in_L135,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.turn_out(turn_mode[Turn_out_R135],Turn_out_R135,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x08:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.diagonal(DIAG_SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_v90(turn_mode[Turn_RV90],Turn_RV90,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.turn_v90(turn_mode[Turn_LV90],Turn_LV90,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.diagonal( DIAG_SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x09:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.diagonal(DIAG_SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_v90(turn_mode[Turn_LV90],Turn_LV90,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.turn_v90(turn_mode[Turn_RV90],Turn_RV90,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.diagonal( DIAG_SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0A:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight(SECTION,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_in(turn_mode[Turn_in_L45],Turn_in_L45,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.diagonal( DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0B:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_out(turn_mode[Turn_out_L45],Turn_out_L45,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0C:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_out(turn_mode[Turn_out_L135],Turn_out_L135,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.straight( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0D:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  KalmanFilter::getInstance().filter_init();
						  mp.motion_start( );
						  mp.fix_wall( 400);
						  for(int i = 50; i <= suction; i = i + 50)
						  {
							  FAN_Motor_SetDuty(i);;
							  HAL_Delay(5);
						  }
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.diagonal(DIAG_SECTION*2,st_param->param->acc,st_param->param->max_velo,st_param->param->max_velo,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  mp.turn_v90(turn_mode[Turn_LV90],Turn_LV90,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  mp.diagonal( SECTION,st_param->param->acc,st_param->param->max_velo,0.0,st_param->sp_gain,st_param->om_gain);
						  while(controll_task::getInstance().run_task !=No_run){}
						  HAL_Delay(200);
						  FAN_Motor_SetDuty(0);;
						  HAL_Delay(200);
						  LogData::getInstance().log_enable = False;
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0E:
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
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
					if(SensingTask::getInstance().IrSensor_Avg() > 2500){
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





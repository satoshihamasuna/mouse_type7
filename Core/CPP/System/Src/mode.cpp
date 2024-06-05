/*
 * mode.cpp
 *
 *  Created on: 2023/07/22
 *      Author: sato1
 */

#include "../../Pheripheral/Include/index.h"
#include "../../Pheripheral/Include/macro.h"
#include "../../Pheripheral/Include/typedef.h"

//#include "../../Pheripheral/Include/interrupt.h"

#include "../../Subsys/Inc/search_class.h"
#include "../../Subsys/Inc/make_map_class.h"
#include "../../Subsys/Inc/wall_class.h"
#include "../../Subsys/Inc/make_path.h"
#include "../Inc/myshell.h"

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

	void indicate_error()
	{
		while(button_status() != True)
		{
			Indicate_LED(0xff);
			HAL_Delay(80);
			Indicate_LED(0x00);
			HAL_Delay(80);
		}
	}

	uint8_t Select(uint8_t _param,uint8_t max,t_encoder enc)
	{
		uint8_t param =  _param;
		if(enc.wheel_speed > 0.2){
			param = (param == max) ? 0 : param + 1 ;
			HAL_Delay(100);
		}
		else if(enc.wheel_speed  < -0.2){
			param = (param == 0) ? max : param - 1 ;
			HAL_Delay(100);
		}
		return param;
	}


	void Demo()
	{
		t_bool demo_end = False;
		uint8_t mode = Return_LED_Status() & 0x30;
		uint8_t param = 0x00;
		uint8_t enable = 0x00;
		uint32_t time = Interrupt::getInstance().return_time_count();


		Motion *motion = &(CtrlTask_type7::getInstance());
		IrSensTask *irsens = (CtrlTask_type7::getInstance().return_irObj());

		Search solve_maze;
		wall_class wall_data(irsens);
		wall_data.init_maze();

		ring_queue<1024,t_MapNode> maze_q;
		make_map map_data(&wall_data,&maze_q);
		Dijkstra run_path(&wall_data);


		t_position start,goal;
		start.x = start.y = 0;start.dir = North;
		goal.x = MAZE_GOAL_X, goal.y = MAZE_GOAL_Y;
		uint8_t goal_size = MAZE_GOAL_SIZE;

		path_follow_class::getInstance().set_path_follow_gain(50.0, 0.0);

		while(demo_end == False)
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
					if(irsens->IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

						Indicate_LED(mode|param);

						t_position return_pos = solve_maze.search_adachi_1_acc(start, goal, goal_size, &wall_data, &map_data,motion);
						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						motion->Motion_end();
						write_save_data(&wall_data);
						solve_maze.search_adachi_2_acc(return_pos, start, 1, &wall_data, &map_data,motion);
						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						write_save_data(&wall_data);
						motion->Motion_end();
						enable = 0x00;
					}
					break;
				case ENABLE|0x01:
					if(irsens->IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

						Indicate_LED(mode|param);

						t_position return_pos = solve_maze.search_adachi_1(start, goal, goal_size, &wall_data, &map_data,motion);
						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						write_save_data(&wall_data);
						solve_maze.search_adachi_1(return_pos, start, 1, &wall_data, &map_data,motion);
						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						write_save_data(&wall_data);
						enable = 0x00;
					}
					break;
				case ENABLE|0x02:
					if(irsens->IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

						Indicate_LED(mode|param);

						t_position return_pos = solve_maze.search_adachi_3_acc(start, goal, goal_size, &wall_data, &map_data,motion);
						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						write_save_data(&wall_data);
						solve_maze.search_adachi_2_acc(return_pos, start, 1, &wall_data, &map_data,motion);
						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						write_save_data(&wall_data);
						enable = 0x00;
					}
					break;
				case ENABLE|0x03:
					if(irsens->IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

						Indicate_LED(mode|param);

						t_position return_pos = solve_maze.search_adachi_3_acc(start, goal, goal_size, &wall_data, &map_data,motion);
						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						write_save_data(&wall_data);
						solve_maze.search_adachi_3_acc(return_pos, start, 1, &wall_data, &map_data,motion);
						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						write_save_data(&wall_data);
						enable = 0x00;
					}
					break;
				case ENABLE|0x04:
					break;
				case ENABLE|0x05:
					break;
				case ENABLE|0x06:
				   if(irsens->IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

				  		run_path.turn_time_set(mode_1000);
						run_path.run_Dijkstra(		start, Dir_None, goal,MAZE_GOAL_SIZE,
															st_mode_500_v0, (int)(sizeof(st_mode_500_v0)/sizeof(t_straight_param *const)),
															di_mode_500_v0, (int)(sizeof(di_mode_500_v0)/sizeof(t_straight_param *const)), mode_500,motion);


						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						enable = 0x00;
					}
					break;
				case ENABLE|0x07:
					if(irsens->IrSensor_Avg() > 2500)
					{
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
						read_save_data(&wall_data);
						enable = 0x00;
					}
					break;
				case ENABLE|0x08:
				   if(irsens->IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

				  		run_path.turn_time_set(mode_1000);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal,MAZE_GOAL_SIZE,600,
															st_mode_1000_v0, (int)(sizeof(st_mode_1000_v0)/sizeof(t_straight_param *const)),
															di_mode_1000_v0, (int)(sizeof(di_mode_1000_v0)/sizeof(t_straight_param *const)), mode_1000,motion);

						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						enable = 0x00;
					}
					break;
				case ENABLE|0x09:
				   if(irsens->IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

				  		run_path.turn_time_set(mode_1000);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,600,
															st_mode_1000_v1, (int)(sizeof(st_mode_1000_v1)/sizeof(t_straight_param *const)),
															di_mode_1000_v1, (int)(sizeof(di_mode_1000_v1)/sizeof(t_straight_param *const)), mode_1000,motion);

						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						enable = 0x00;
					}
					break;
				case ENABLE|0x0A:
				   if(irsens->IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}

				  		run_path.turn_time_set(mode_1200);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,600,
															st_mode_1200_v0, (int)(sizeof(st_mode_1200_v0)/sizeof(t_straight_param *const)),
															di_mode_1200_v0, (int)(sizeof(di_mode_1200_v0)/sizeof(t_straight_param *const)), mode_1200,motion);

						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						enable = 0x00;
					}
					break;
				case ENABLE|0x0B:
				   if(irsens->IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}


				  		run_path.turn_time_set(mode_1400);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,700,
															st_mode_1400_v0, (int)(sizeof(st_mode_1400_v0)/sizeof(t_straight_param *const)),
															di_mode_1400_v0, (int)(sizeof(di_mode_1400_v0)/sizeof(t_straight_param *const)), mode_1400,motion);
						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						enable = 0x00;
					}
					break;
				case ENABLE|0x0C:
				   if(irsens->IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}


				  		run_path.turn_time_set(mode_1400);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,700,
															st_mode_1400_v1, (int)(sizeof(st_mode_1400_v1)/sizeof(t_straight_param *const)),
															di_mode_1400_v1, (int)(sizeof(di_mode_1400_v1)/sizeof(t_straight_param *const)), mode_1400,motion);

						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						enable = 0x00;
					}
					break;
				case ENABLE|0x0D:
				   if(irsens->IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}


				  		run_path.turn_time_set(mode_1400);
						run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,700,
															st_mode_1500_v0, (int)(sizeof(st_mode_1500_v0)/sizeof(t_straight_param *const)),
															di_mode_1500_v0, (int)(sizeof(di_mode_1500_v0)/sizeof(t_straight_param *const)), mode_1500,motion);

						if(motion->motion_exeStatus_get() == error)
						{
							Mode::indicate_error();
							enable = 0x00;
							break;
						}
						enable = 0x00;
					}
					break;
				case ENABLE|0x0E:
				   if(irsens->IrSensor_Avg() > 2500)
				   {
						for(int i = 0;i < 11;i++)
						{
							(i%2 == 0) ? Indicate_LED(Mode_State()):Indicate_LED(0x00|0x00);
							HAL_Delay(50);
						}
						t_position start,goal;
				  		start.x = start.y = 0;start.dir = North;
				  		goal.x = MAZE_GOAL_X, goal.y = MAZE_GOAL_Y;
				  		run_path.di_param_set(di_mode_1000_v0, 1);
				  		run_path.di_param_set(di_mode_1000_v0, 1);
				  		run_path.turn_time_set(mode_1000);
						run_path.check_run_Dijkstra(start, Dir_None, goal, 2);

						Mode_Disable();
					}
					break;
				case ENABLE|0x0F:
					if(irsens->IrSensor_Avg() > 2500){
						for(int i = 0;i < 11;i++)
						{
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
						}
						demo_end = True;
					}
					break;
				default:
					break;
			}
		}
	}





	void Select_Mode()
	{
		uint8_t mode = 0;
		uint8_t enable = 0;

		while(1)
		{
			enable 	= Mode::Select(enable,0x01,Encoder_GetProperty_Left() );
			mode = (enable == 0x00) ? Mode::Select(mode  ,0x03,Encoder_GetProperty_Right()):mode;
			Battery_LimiterVoltage();
			Indicate_LED(mode << 4);
			switch((enable << 4)|mode)
			{
				case ENABLE|0x00:
					Mode::Interface_Check();
					enable = 0;
					break;
				case ENABLE|0x01:
					Mode::Demo();
					enable = 0;
					break;
				case ENABLE|0x02:
					//Mode::Debug(&st_param_700,mode_700,0);
					Mode::Debug(&st_param_1200,mode_1200,500);
				    Mode::Debug(&st_param_1400_acc2G,mode_1400,500);
					Mode::Debug(&st_param_1500_acc2G,mode_1500,500);
					enable = 0;
					break;
				case ENABLE|0x03:
					Myshell_Initialize();
					Myshell_Execute();
					enable = 0;
					break;
				default:
					break;
			}
		}
	}
}


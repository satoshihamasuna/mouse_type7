/*
 * main.cpp
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */

#include <iostream>
#include <stdio.h>

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

#include "../../Inc/sensing_task.h"
#include "../../Inc/motion.h"
#include "../../Inc/interrupt.h"
#include "../../Inc/log_data.h"
#include "../../Inc/run_param.h"
#include "../../Inc/flash.h"
#include "../../Inc/path_follow.h"

#include "../Inc/mode.h"


void Module_Initialize()
{
	  IMU_initialize();
	  Sensor_Initialize();
	  Motor_Initialize();
	  FAN_Motor_Initialize();
	  Encoder_Initialize();
	  Interrupt_Initialize();
	  IMU_read_DMA_Start();
}

void CPP_Main()
{
	  Module_Initialize();
	  uint8_t setup = 0x01;
	  for (int i = 0;i < 8; i++)
	  {
		  Indicate_LED(setup << i);
		  HAL_Delay(50);
	  }
	  Mode::Select_Mode();
}

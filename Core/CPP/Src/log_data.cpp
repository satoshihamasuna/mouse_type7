/*
 * log_data.cpp
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */



#include "../Pheripheral/Include/index.h"
#include "../Inc/log_data.h"
#include "stdio.h"
#include "../Component/Inc/half_float.h"
void LogData::indicate_data()
{
	for(int i = 0; i< 1000;i++)
	{
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[0][i]),half_to_float(data[1][i]),
				half_to_float(data[2][i]),half_to_float(data[3][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[4][i]),half_to_float(data[5][i]),
				half_to_float(data[6][i]),half_to_float(data[7][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf\n",
				half_to_float(data[8][i]),half_to_float(data[9][i]),
				half_to_float(data[10][i]),half_to_float(data[11][i]));
		HAL_Delay(2);
	}
}


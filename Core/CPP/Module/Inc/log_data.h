/*
 * log_data.h
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */

#ifndef CPP_INC_LOG_DATA_H_
#define CPP_INC_LOG_DATA_H_

#include "../../Pheripheral/Include/typedef.h"
#include "../../Component/Inc/half_float.h"
#include "../../Component/Inc/singleton.h"

#define LOG_DATA_SIZE 1000
#define LOG_DATA_NUM  20

class LogData:public Singleton<LogData>
{

	public:
	    t_bool log_enable = False;
		const int data_size = LOG_DATA_SIZE;
		const int data_num  = LOG_DATA_NUM;
		int data_count = 0;
		half_float data[LOG_DATA_NUM][LOG_DATA_SIZE];
		void indicate_data();
		void init_log()
		{
			for(int i = 0; i < data_num ;i++)
			{
				for(int j = 0;j < data_size;j++)
				{
					data[i][j] = float_to_half(0.0f);
				}
			}
		}

		void logging();
};




#endif /* CPP_INC_LOG_DATA_H_ */

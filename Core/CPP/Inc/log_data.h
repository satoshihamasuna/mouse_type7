/*
 * log_data.h
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */

#ifndef CPP_INC_LOG_DATA_H_
#define CPP_INC_LOG_DATA_H_

#include "../Pheripheral/Include/typedef.h"
#include "../Component/Inc/half_float.h"
#include "../Component/Inc/singleton.h"

class LogData:public Singleton<LogData>
{

	public:
	    t_bool log_enable = False;
		const int data_size = 1000;
		int data_count = 0;
		half_float data[30][1000];
		void indicate_data();
};




#endif /* CPP_INC_LOG_DATA_H_ */

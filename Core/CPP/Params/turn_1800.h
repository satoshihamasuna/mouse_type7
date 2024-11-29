/*
 * turn_1800.h
 *
 *  Created on: 2024/11/18
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1800_H_
#define CPP_PARAMS_TURN_1800_H_


#include "typedef_run_param.h"


//All parameters no adjusted
//k = 250,alpha = 1.0
const static t_pid_gain sp_gain_turn90_1800 = {18.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1800 = {0.8, 0.1, 0.0};
const static t_turn_param_table slalom_L90_1800_table = {1.80f, 53.00f,11.36,42.27, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1800_table = {1.80f,-53.00f,11.36,42.27,-90.0f,Turn_R};
const static t_param param_L90_1800 = {&slalom_L90_1800_table,&sp_gain_turn90_1800,&om_gain_turn90_1800};
const static t_param param_R90_1800 = {&slalom_R90_1800_table,&sp_gain_turn90_1800,&om_gain_turn90_1800};
//k = 200,alpha = 1.0
const static t_pid_gain sp_gain_turn180_1800 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1800 = {0.6, 0.1, 0.0};
const static t_turn_param_table slalom_L180_1800_table = {1.80f, 48.00f,8.93,48.16, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1800_table = {1.80f,-48.00f,8.93,48.16,-180.0f,Turn_R};
const static t_param param_L180_1800 = {&slalom_L180_1800_table,&sp_gain_turn180_1800,&om_gain_turn180_1800};
const static t_param param_R180_1800 = {&slalom_R180_1800_table,&sp_gain_turn180_1800,&om_gain_turn180_1800};

//k = 300
//not adjust
const static t_pid_gain sp_gain_turnV90_1800 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1800 = {0.6, 0.02, 0.0};
const static t_turn_param_table slalom_LV90_1800_table = {1.80f, 38.0f,3.0,26.50, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1800_table = {1.80f,-38.0f,3.0,26.50,-90.0f,Turn_R};
const static t_param param_LV90_1800 = {&slalom_LV90_1800_table,&sp_gain_turnV90_1800,&om_gain_turnV90_1800};
const static t_param param_RV90_1800 = {&slalom_RV90_1800_table,&sp_gain_turnV90_1800,&om_gain_turnV90_1800};
//k = 300
const static t_pid_gain sp_gain_turnIn45_1800 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1800 = {0.6, 0.02, 0.0};
const static t_turn_param_table slalom_inL45_1800_table = {1.80f, 55.0f,6.28,39.48, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1800_table = {1.80f,-55.0f,6.28,39.48,-45.0f,Turn_R};
const static t_param param_inL45_1800 = {&slalom_inL45_1800_table,&sp_gain_turnIn45_1800,&om_gain_turnIn45_1800};
const static t_param param_inR45_1800 = {&slalom_inR45_1800_table,&sp_gain_turnIn45_1800,&om_gain_turnIn45_1800};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1800 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1800 = {0.6, 0.02, 0.0};
const static t_turn_param_table slalom_outL45_1800_table = {1.80f, 60.0f,18.24,20.49, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1800_table = {1.80f,-60.0f,18.24,20.49,-45.0f,Turn_R};
const static t_param param_outL45_1800 = {&slalom_outL45_1800_table,&sp_gain_turnOut45_1800,&om_gain_turnOut45_1800};
const static t_param param_outR45_1800 = {&slalom_outR45_1800_table,&sp_gain_turnOut45_1800,&om_gain_turnOut45_1800};


const static t_pid_gain sp_gain_turnIn135_1800 = {15.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_1800 = {0.6, 0.02, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1800_table = {1.80f, 38.0f,18.79,37.96, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1800_table = {1.80f,-38.0f,18.79,37.96,-135.0f,Turn_R};
const static t_param param_inL135_1800 = {&slalom_inL135_1800_table,&sp_gain_turnIn135_1800,&om_gain_turnIn135_1800};
const static t_param param_inR135_1800 = {&slalom_inR135_1800_table,&sp_gain_turnIn135_1800,&om_gain_turnIn135_1800};

//
const static t_pid_gain sp_gain_turnOut135_1800 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_1800 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1800_table = {1.80f, 38.0f,11.20,45.77, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1800_table = {1.80f,-38.0f,11.20,45.77,-135.0f,Turn_R};
const static t_param param_outL135_1800 = {&slalom_outL135_1800_table,&sp_gain_turnOut135_1800,&om_gain_turnOut135_1800};
const static t_param param_outR135_1800 = {&slalom_outR135_1800_table,&sp_gain_turnOut135_1800,&om_gain_turnOut135_1800};

const static t_param *const mode_1800[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1800,		&param_L90_1800,
												&param_R180_1800,	&param_L180_1800,
												&param_inR45_1800,	&param_inL45_1800,
												&param_outR45_1800,	&param_outL45_1800,
												&param_inR135_1800,	&param_inL135_1800,
												&param_outR135_1800,	&param_outL135_1800,
												&param_RV90_1800,	&param_LV90_1800
											};

const static t_param *const mode_1800_acc[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1800,		&param_L90_1800,
												&param_R180_1800,	&param_L180_1800,
												NULL,	NULL,
												NULL,	NULL,
												NULL,	NULL,
												NULL,	NULL,
												NULL,					NULL	,
												NULL,					NULL
											};


#endif /* CPP_PARAMS_TURN_1800_H_ */

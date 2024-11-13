/*
 * turn_1800.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1800_H_
#define CPP_PARAMS_TURN_1800_H_

#include "typedef_run_param.h"

const static t_pid_gain sp_gain_turn90_1800 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1800 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_1800_table = {1.80f, 50.50f,19.56,38.43, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1800_table = {1.80f,-50.50f,19.56,38.43,-90.0f,Turn_R};
const static t_param param_L90_1800 = {&slalom_L90_1800_table,&sp_gain_turn90_1800,&om_gain_turn90_1800};
const static t_param param_R90_1800 = {&slalom_R90_1800_table,&sp_gain_turn90_1800,&om_gain_turn90_1800};

const static t_pid_gain sp_gain_turn180_1800 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1800 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1800_table = {1.60f, 50.50f,8.80,30.10, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1800_table = {1.60f,-50.50f,8.80,30.10,-180.0f,Turn_R};
const static t_param param_L180_1800 = {&slalom_L180_1800_table,&sp_gain_turn180_1800,&om_gain_turn180_1800};
const static t_param param_R180_1800 = {&slalom_R180_1800_table,&sp_gain_turn180_1800,&om_gain_turn180_1800};
//k = 300
//not adjust
const static t_pid_gain sp_gain_turnV90_1800 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1800 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_1800_table = {1.80f, 38.0f,9.50,29.66, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1800_table = {1.80f,-38.0f,9.50,29.66,-90.0f,Turn_R};
const static t_param param_LV90_1800 = {&slalom_LV90_1800_table,&sp_gain_turnV90_1800,&om_gain_turnV90_1800};
const static t_param param_RV90_1800 = {&slalom_RV90_1800_table,&sp_gain_turnV90_1800,&om_gain_turnV90_1800};
//k = 300
const static t_pid_gain sp_gain_turnIn45_1800 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1800 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_1800_table = {1.80f, 53.0f,9.84,41.94, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1800_table = {1.80f,-53.0f,9.84,41.94,-45.0f,Turn_R};
const static t_param param_inL45_1800 = {&slalom_inL45_1800_table,&sp_gain_turnIn45_1800,&om_gain_turnIn45_1800};
const static t_param param_inR45_1800 = {&slalom_inR45_1800_table,&sp_gain_turnIn45_1800,&om_gain_turnIn45_1800};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1800 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1800 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,18.235,20.24, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,18.235,20.24,-45.0f,Turn_R};
const static t_param param_outL45_1800 = {&slalom_outL45_1800_table,&sp_gain_turnOut45_1800,&om_gain_turnOut45_1800};
const static t_param param_outR45_1800 = {&slalom_outR45_1800_table,&sp_gain_turnOut45_1800,&om_gain_turnOut45_1800};


const static t_pid_gain sp_gain_turnIn135_1800 = {10.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_1800 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1800_table = {1.80f, 44.0f,13.17,26.34, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1800_table = {1.80f,-44.0f,13.17,26.34,-135.0f,Turn_R};
const static t_param param_inL135_1800 = {&slalom_inL135_1800_table,&sp_gain_turnIn135_1800,&om_gain_turnIn135_1800};
const static t_param param_inR135_1800 = {&slalom_inR135_1800_table,&sp_gain_turnIn135_1800,&om_gain_turnIn135_1800};

//
const static t_pid_gain sp_gain_turnOut135_1800 = {10.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_1800 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1800_table = {1.80f, 42.0f,11.85,40.11, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1800_table = {1.80f,-42.0f,11.85,40.1,-135.0f,Turn_R};
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





#endif /* CPP_PARAMS_TURN_1800_H_ */

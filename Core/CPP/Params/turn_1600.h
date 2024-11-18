/*
 * turn_1600.h
 *
 *  Created on: 2024/11/18
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1600_H_
#define CPP_PARAMS_TURN_1600_H_

#include "typedef_run_param.h"

const static t_pid_gain sp_gain_turn90_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1600 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_L90_1600_table = {1.60f, 50.50f,19.56,38.43, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1600_table = {1.60f,-50.50f,19.56,38.43,-90.0f,Turn_R};
const static t_param param_L90_1600 = {&slalom_L90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};
const static t_param param_R90_1600 = {&slalom_R90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};

const static t_pid_gain sp_gain_turn180_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1600 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1600_table = {1.60f, 50.50f,8.80,35.10, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1600_table = {1.60f,-50.50f,8.80,35.10,-180.0f,Turn_R};
const static t_param param_L180_1600 = {&slalom_L180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600};
const static t_param param_R180_1600 = {&slalom_R180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600};
//k = 300
//not adjust
const static t_pid_gain sp_gain_turnV90_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1600 = {0.8, 0.1, 0.005};
const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 38.0f,9.50,29.66, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-38.0f,9.50,29.66,-90.0f,Turn_R};
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 38.0f,4.08,17.95, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-38.0f,4.08,17.95,-90.0f,Turn_R};
const static t_param param_LV90_1600 = {&slalom_LV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600};
const static t_param param_RV90_1600 = {&slalom_RV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600};
//k = 300
const static t_pid_gain sp_gain_turnIn45_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1600 = {0.8, 0.1, 0.005};
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 52.0f,8.42,43.32, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-52.0f,8.42,43.32,-45.0f,Turn_R};
const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 53.0f,9.84,41.94, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-53.0f,9.84,41.94,-45.0f,Turn_R};
const static t_param param_inL45_1600 = {&slalom_inL45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600};
const static t_param param_inR45_1600 = {&slalom_inR45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1600 = {0.8, 0.1, 0.005};
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 52.0f,27.16,24.67, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-52.0f,27.16,24.67,-45.0f,Turn_R};
const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 55.0f,18.235,20.24, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-55.0f,18.235,20.24,-45.0f,Turn_R};
const static t_param param_outL45_1600 = {&slalom_outL45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600};
const static t_param param_outR45_1600 = {&slalom_outR45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600};


const static t_pid_gain sp_gain_turnIn135_1600 = {15.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_1600 = {0.8, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 44.0f,13.17,26.34, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-44.0f,13.17,26.34,-135.0f,Turn_R};
const static t_param param_inL135_1600 = {&slalom_inL135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600};
const static t_param param_inR135_1600 = {&slalom_inR135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600};

//
const static t_pid_gain sp_gain_turnOut135_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_1600 = {0.8, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 42.0f,11.85,40.11, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-42.0f,11.85,40.1,-135.0f,Turn_R};
const static t_param param_outL135_1600 = {&slalom_outL135_1600_table,&sp_gain_turnOut135_1600,&om_gain_turnOut135_1600};
const static t_param param_outR135_1600 = {&slalom_outR135_1600_table,&sp_gain_turnOut135_1600,&om_gain_turnOut135_1600};

const static t_param *const mode_1600[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1600,		&param_L90_1600,
												&param_R180_1600,	&param_L180_1600,
												&param_inR45_1600,	&param_inL45_1600,
												&param_outR45_1600,	&param_outL45_1600,
												&param_inR135_1600,	&param_inL135_1600,
												&param_outR135_1600,	&param_outL135_1600,
												&param_RV90_1600,	&param_LV90_1600
											};

const static t_param *const mode_1600_acc[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1600,		&param_L90_1600,
												&param_R180_1600,	&param_L180_1600,
												NULL,					NULL	,
												NULL,					NULL	,
												&param_inR135_1600,	&param_inL135_1600,
												&param_outR135_1600,	&param_outL135_1600,
												NULL,					NULL	,
												NULL,					NULL
											};

#endif /* CPP_PARAMS_TURN_1600_H_ */

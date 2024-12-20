/*
 * turn_1600.h
 *
 *  Created on: 2024/11/18
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1600_H_
#define CPP_PARAMS_TURN_1600_H_

#include "typedef_run_param.h"
//k = 250
const static t_pid_gain sp_gain_turn90_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1600 = {0.6, 0.1, 0.0};
const static t_turn_param_table slalom_L90_1600_table = {1.60f, 50.0f,16.19-3.0,34.97+8.0, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1600_table = {1.60f,-50.0f,16.19-3.0,34.97+8.0,-90.0f,Turn_R};
const static t_param param_L90_1600 = {&slalom_L90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};
const static t_param param_R90_1600 = {&slalom_R90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};
//k = 250
const static t_pid_gain sp_gain_turn180_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1600 = {0.6, 0.1, 0.0};
const static t_turn_param_table slalom_L180_1600_table = {1.60f, 50.50f,(8.14-0.0),(35.29-0.0), 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1600_table = {1.60f,-50.50f,(8.14-0.0),(35.29-0.0),-180.0f,Turn_R};
const static t_param param_L180_1600 = {&slalom_L180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600};
const static t_param param_R180_1600 = {&slalom_R180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600};

//k = 250
const static t_pid_gain sp_gain_turnV90_1600 = {20.0, 0.10, 0.00};
const static t_pid_gain om_gain_turnV90_1600 = {0.6, 0.05, 0.00};
const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 38.0f,9.50,29.66, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-38.0f,9.50,29.66,-90.0f,Turn_R};
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 38.0f,10.45,28.67, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-38.0f,10.45,28.67,-90.0f,Turn_R};
const static t_param param_LV90_1600 = {&slalom_LV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600};
const static t_param param_RV90_1600 = {&slalom_RV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600};
//k = 250
const static t_pid_gain sp_gain_turnIn45_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1600 = {0.6, 0.05, 0.005};
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 52.0f,8.42,43.32, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-52.0f,8.42,43.32,-45.0f,Turn_R};
const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,7.74,40.69, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,7.74,40.69,-45.0f,Turn_R};
const static t_param param_inL45_1600 = {&slalom_inL45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600};
const static t_param param_inR45_1600 = {&slalom_inR45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600};

//k = 250
const static t_pid_gain sp_gain_turnOut45_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1600 = {0.6, 0.05, 0.005};
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 52.0f,27.16,24.67, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-52.0f,27.16,24.67,-45.0f,Turn_R};
const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.35,20.45, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.35,20.45,-45.0f,Turn_R};
const static t_param param_outL45_1600 = {&slalom_outL45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600};
const static t_param param_outR45_1600 = {&slalom_outR45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600};

//k = 250
const static t_pid_gain sp_gain_turnIn135_1600 = {15.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_1600 = {0.6, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 44.0f,10.51,23.73, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-44.0f,10.51,23.73,-135.0f,Turn_R};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 40.0f,13.66,24.55+15.0, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-40.0f,13.66,24.55+15.0,-135.0f,Turn_R};
const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 44.0f,12.63,30.77, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-44.0f,12.63,30.77,-135.0f,Turn_R};
const static t_param param_inL135_1600 = {&slalom_inL135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600};
const static t_param param_inR135_1600 = {&slalom_inR135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600};

//k = 250

const static t_pid_gain sp_gain_turnOut135_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_1600 = {0.6, 0.05, 0.0};
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 44.0f,2.8619,31.5052, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-44.0f,2.8619,31.5052,-135.0f,Turn_R};
const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 43.0f,8.17,41.69, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-43.0f,8.17,41.69,-135.0f,Turn_R};
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

const static t_param *const mode_1600_acc2[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1600,		&param_L90_1600,
												&param_R180_1600,	&param_L180_1600,
												&param_inR45_1600,	&param_inL45_1600,
												&param_outR45_1600,	&param_outL45_1600,
												&param_inR135_1600,	&param_inL135_1600,
												&param_outR135_1600,	&param_outL135_1600,
												NULL,					NULL	,
												NULL,					NULL
											};

#endif /* CPP_PARAMS_TURN_1600_H_ */

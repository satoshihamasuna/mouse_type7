/*
 * turn_1400.h
 *
 *  Created on: 2024/11/18
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_1400_H_
#define CPP_PARAMS_TURN_1400_H_

#include "typedef_run_param.h"


//k = 250
const static t_pid_gain sp_gain_turn90_1400 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1400 = {0.6, 0.1, 0.0};
const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,22.01,38.31, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,22.01,38.31,-90.0f,Turn_R};
const static t_param param_L90_1400 = {&slalom_L90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400};
const static t_param param_R90_1400 = {&slalom_R90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400};

const static t_pid_gain sp_gain_turn180_1400 = {20.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1400 = {0.6, 0.1, 0.0};
//const static t_turn_param_table slalom_L180_1400_table = {1.40f, 51.0f,9.57,29.12, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1400_table = {1.40f,-51.0f,9.57,29.12,-180.0f,Turn_R};
const static t_turn_param_table slalom_L180_1400_table = {1.40f, 48.0f,12.30,29.35, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1400_table = {1.40f,-48.0f,12.30,29.35,-180.0f,Turn_R};
//const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 40.0f,19.73,36.36, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-40.0f,19.73,36.36,-90.0f,Turn_R};
const static t_param param_L180_1400 = {&slalom_L180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};
const static t_param param_R180_1400 = {&slalom_R180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};

//not adjust
//k = 300
const static t_pid_gain sp_gain_turnV90_1400 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1400 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 38.50f,11.55-2.0,26.99, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-38.50f,11.55-2.0,26.99,-90.0f,Turn_R};
//const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 38.50f,5.66,16.27, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-38.50f,5.66,16.27,-90.0f,Turn_R};
const static t_param param_LV90_1400 = {&slalom_LV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};
const static t_param param_RV90_1400 = {&slalom_RV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};
//k = 300
const static t_pid_gain sp_gain_turnIn45_1400 = {20.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1400 = {0.6, 0.05, 0.00};
//const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 50.5f,8.28,40.77, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-55.0f,8.28,40.77,-45.0f,Turn_R};
const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 50.5f,10.18,38.17, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-50.5f,10.18,38.17,-45.0f,Turn_R};
const static t_param param_inL45_1400 = {&slalom_inL45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};
const static t_param param_inR45_1400 = {&slalom_inR45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};

//k = 220
//k = 300
const static t_pid_gain sp_gain_turnOut45_1400 = {20.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1400 = {0.6, 0.05, 0.00};
const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 50.5f,26.9126,24.9808+8.0, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-50.5f,26.9126,24.9808+8.0,-45.0f,Turn_R};
//const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 50.5f,28.54,19.64, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-50.5f,28.54,19.64,-45.0f,Turn_R};
const static t_param param_outL45_1400 = {&slalom_outL45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};
const static t_param param_outR45_1400 = {&slalom_outR45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};

//k220
const static t_pid_gain sp_gain_turnIn135_1400 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn135_1400 = {0.6, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 42.0f,21.48,31.96, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-42.0f,21.48,31.86,-135.0f,Turn_R};
const static t_param param_inL135_1400 = {&slalom_inL135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};
const static t_param param_inR135_1400 = {&slalom_inR135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};

//220
const static t_pid_gain sp_gain_turnOut135_1400 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_1400 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 42.0f,13.88,39.77, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-42.0f,13.88,39.77,-135.0f,Turn_R};
const static t_param param_outL135_1400 = {&slalom_outL135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};
const static t_param param_outR135_1400 = {&slalom_outR135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};

const static t_param *const mode_1400[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1400,		&param_L90_1400,
												&param_R180_1400,	&param_L180_1400,
												&param_inR45_1400,	&param_inL45_1400,
												&param_outR45_1400,	&param_outL45_1400,
												&param_inR135_1400,	&param_inL135_1400,
												&param_outR135_1400,	&param_outL135_1400,
												&param_RV90_1400,	&param_LV90_1400
											};


const static t_param *const mode_1400_acc[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1400,		&param_L90_1400,
												&param_R180_1400,	&param_L180_1400,
												&param_inR45_1400,	&param_inL45_1400,
												&param_outR45_1400,	&param_outL45_1400,
												&param_inR135_1400,	&param_inL135_1400,
												&param_outR135_1400,	&param_outL135_1400,
												&param_RV90_1400,	&param_LV90_1400,
												NULL,					NULL
											};


#endif /* CPP_PARAMS_TURN_1400_H_ */

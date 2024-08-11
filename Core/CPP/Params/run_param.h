/*
 * run_pram.h
 *
 *  Created on: 2023/06/21
 *      Author: sato1
 */

#ifndef CPP_INC_RUN_PARAM_H_
#define CPP_INC_RUN_PARAM_H_

#include "../Component/Inc/controller.h"

#include "../Module/Inc/vehicle.h"

typedef struct{
	float velo;
	float r_min;
	float Lstart;
	float Lend;
	float degree;
	t_turn_dir turn_dir;
}t_turn_param_table;

typedef struct{
	//float base_velo;
	float max_velo;
	float acc;
}t_velo_param;

typedef struct{
	t_turn_param_table const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_param;

typedef struct{
	t_velo_param const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_straight_param;


const static t_pid_gain basic_sp_gain = {15.0,0.15,0.0};
const static t_pid_gain basic_om_gain = {0.60f, 0.01f, 0.00f};


const static t_pid_gain search_sp_gain = {15.0,0.15,0.0};
const static t_pid_gain search_om_gain = {0.60f, 0.01f, 0.00f};

const static t_pid_gain sp_gain_search_turn = {15.0,0.1,0.0};//{12.0,0.1,0.0};
const static t_pid_gain om_gain_search_turn = {0.60, 0.01, 0.00};//{0.50f, 0.0005f, 0.001f};
const static t_turn_param_table slalom_L90_table = {0.32f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table = {0.32f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const static t_param param_L90_search = {&slalom_L90_table ,&sp_gain_search_turn,&om_gain_search_turn};
const static t_param param_R90_search = {&slalom_R90_table, &sp_gain_search_turn,&om_gain_search_turn};

const static t_pid_gain sp_gain_300 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_300 = {0.05, 0.01, 0.00};
const static t_velo_param param_300 = {0.30f,4.0f};
const static t_straight_param st_param_300 = {&param_300,&sp_gain_300,&om_gain_300};

const static t_pid_gain sp_gain_320 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_320 = {0.65, 0.01, 0.00};
const static t_velo_param param_320 = {0.32f,4.0f};
const static t_straight_param st_param_320 = {&param_320,&sp_gain_320,&om_gain_320};

const static t_pid_gain sp_gain_450 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_450 = {0.65, 0.01, 0.00};
const static t_velo_param param_450 = {0.45f,6.0f};
const static t_straight_param st_param_450 = {&param_450,&sp_gain_450,&om_gain_450};

const static t_pid_gain sp_gain_500 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_500 = {0.60, 0.01, 0.00};
const static t_velo_param param_500 = {0.50f,6.0f};
const static t_straight_param st_param_500 = {&param_500,&sp_gain_500,&om_gain_500};

const static t_pid_gain sp_gain_600 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_600 = {0.60, 0.01, 0.00};
const static t_velo_param param_600 = {0.60f,6.0f};
const static t_straight_param st_param_600 = {&param_600,&sp_gain_600,&om_gain_600};

const static t_pid_gain sp_gain_700 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_700 = {0.6, 0.02, 0.00};
const static t_velo_param param_700 = {0.70f,6.0f};
const static t_straight_param st_param_700 = {&param_700,&sp_gain_700,&om_gain_700};


const static t_pid_gain sp_gain_1000 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1000 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1000 = {1.0f,9.0f};
const static t_straight_param st_param_1000 = {&param_1000,&sp_gain_1000,&om_gain_1000};

const static t_pid_gain sp_gain_1050 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1050 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1050 = {1.05f,9.0f};
const static t_straight_param st_param_1050 = {&param_1050,&sp_gain_1050,&om_gain_1050};

const static t_pid_gain sp_gain_1100 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1100 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1100 = {1.10f,9.0f};
const static t_straight_param st_param_1100 = {&param_1100,&sp_gain_1100,&om_gain_1100};

const static t_pid_gain sp_gain_1200 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1200 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1200 = {1.20f,10.0f};
const static t_straight_param st_param_1200 = {&param_1200,&sp_gain_1200,&om_gain_1200};

const static t_pid_gain sp_gain_1300 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1300 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1300 = {1.30f,10.0f};
const static t_straight_param st_param_1300 = {&param_1300,&sp_gain_1300,&om_gain_1300};

const static t_pid_gain sp_gain_1400 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1400 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1400 = {1.40f,12.0f};
const static t_velo_param param_1400_acc2G = {1.40f,20.0f};
const static t_straight_param st_param_1400 = {&param_1400,&sp_gain_1400,&om_gain_1400};
const static t_straight_param st_param_1400_acc2G  = {&param_1400_acc2G ,&sp_gain_1400,&om_gain_1400};

const static t_pid_gain sp_gain_1500 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1500 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1500 = {1.50f,12.0f};
const static t_velo_param param_1500_acc2G = {1.50f,20.0f};
const static t_straight_param st_param_1500 = {&param_1500,&sp_gain_1500,&om_gain_1500};
const static t_straight_param st_param_1500_acc2G  = {&param_1500_acc2G ,&sp_gain_1500,&om_gain_1500};

const static t_pid_gain sp_gain_1600 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1600 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1600 = {1.60f,12.0f};
const static t_velo_param param_1600_acc2G = {1.60f,20.0f};
const static t_straight_param st_param_1600 = {&param_1600,&sp_gain_1600,&om_gain_1600};
const static t_straight_param st_param_1600_acc2G  = {&param_1600_acc2G ,&sp_gain_1600,&om_gain_1600};

const static t_pid_gain sp_gain_1700 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1700 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1700 = {1.70f,12.0f};
const static t_velo_param param_1700_acc2G = {1.70f,20.0f};
const static t_straight_param st_param_1700 = {&param_1700,&sp_gain_1700,&om_gain_1700};
const static t_straight_param st_param_1700_acc2G = {&param_1700_acc2G,&sp_gain_1700,&om_gain_1700};

const static t_pid_gain sp_gain_1800 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1800 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1800 = {1.80f,12.0f};
const static t_velo_param param_1800_acc2G = {1.80f,20.0f};
const static t_straight_param st_param_1800 = {&param_1800,&sp_gain_1800,&om_gain_1800};
const static t_straight_param st_param_1800_acc2G = {&param_1800_acc2G,&sp_gain_1800,&om_gain_1800};

const static t_pid_gain sp_gain_1900 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_1900 = {0.60f, 0.01f, 0.00f};
const static t_velo_param param_1900 = {1.90f,12.0f};
const static t_velo_param param_1900_acc2G = {1.90f,20.0f};
const static t_straight_param st_param_1900 = {&param_1900,&sp_gain_1900,&om_gain_1900};
const static t_straight_param st_param_1900_acc2G = {&param_1900_acc2G,&sp_gain_1900,&om_gain_1900};

const static t_pid_gain sp_gain_2000 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2000 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2000 = {2.0f,12.0f};
const static t_velo_param param_2000_acc2G = {2.0f,20.0f};
const static t_straight_param st_param_2000 = {&param_2000,&sp_gain_2000,&om_gain_2000};
const static t_straight_param st_param_2000_acc2G = {&param_2000_acc2G,&sp_gain_2000,&om_gain_2000};


const static t_pid_gain sp_gain_2100 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2100 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2100 = {2.1f,12.0f};
const static t_velo_param param_2100_acc2G = {2.1f,20.0f};
const static t_straight_param st_param_2100 = {&param_2100,&sp_gain_2100,&om_gain_2100};
const static t_straight_param st_param_2100_acc2G = {&param_2100_acc2G,&sp_gain_2100,&om_gain_2100};

const static t_pid_gain sp_gain_2200 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2200 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2200 = {2.2f,12.0f};
const static t_velo_param param_2200_acc2G = {2.2f,20.0f};
const static t_straight_param st_param_2200 = {&param_2200,&sp_gain_2200,&om_gain_2200};
const static t_straight_param st_param_2200_acc2G = {&param_2200_acc2G,&sp_gain_2200,&om_gain_2200};

const static t_pid_gain sp_gain_2300 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2300 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2300 = {2.3f,12.0f};
const static t_velo_param param_2300_acc2G = {2.3f,20.0f};
const static t_straight_param st_param_2300 = {&param_2300,&sp_gain_2300,&om_gain_2300};
const static t_straight_param st_param_2300_acc2G = {&param_2300_acc2G,&sp_gain_2300,&om_gain_2300};

const static t_pid_gain sp_gain_2400 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2400 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2400 = {2.4f,12.0f};
const static t_velo_param param_2400_acc2G = {2.4f,20.0f};
const static t_straight_param st_param_2400 = {&param_2400,&sp_gain_2400,&om_gain_2400};
const static t_straight_param st_param_2400_acc2G = {&param_2400_acc2G,&sp_gain_2400,&om_gain_2400};

const static t_pid_gain sp_gain_2500 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2500 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2500 = {2.5f,12.0f};
const static t_velo_param param_2500_acc2G = {2.5f,20.0f};
const static t_straight_param st_param_2500 = {&param_2500,&sp_gain_2500,&om_gain_2500};
const static t_straight_param st_param_2500_acc2G = {&param_2500_acc2G,&sp_gain_2500,&om_gain_2500};

const static t_pid_gain sp_gain_2600 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2600 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2600 = {2.6f,15.0f};
const static t_velo_param param_2600_acc2G = {2.6f,20.0f};
const static t_straight_param st_param_2600 = {&param_2600,&sp_gain_2600,&om_gain_2600};
const static t_straight_param st_param_2600_acc2G = {&param_2600_acc2G,&sp_gain_2600,&om_gain_2600};

const static t_pid_gain sp_gain_2700 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2700 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2700 = {2.7f,15.0f};
const static t_velo_param param_2700_acc2G = {2.7f,20.0f};
const static t_straight_param st_param_2700 = {&param_2700,&sp_gain_2700,&om_gain_2700};
const static t_straight_param st_param_2700_acc2G = {&param_2700_acc2G,&sp_gain_2700,&om_gain_2700};


const static t_pid_gain sp_gain_2800 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2800 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2800 = {2.8f,15.0f};
const static t_velo_param param_2800_acc2G = {2.8f,20.0f};
const static t_straight_param st_param_2800 = {&param_2800,&sp_gain_2800,&om_gain_2800};
const static t_straight_param st_param_2800_acc2G = {&param_2800_acc2G,&sp_gain_2800,&om_gain_2800};


const static t_pid_gain sp_gain_2900 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2900 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_2900 = {2.9f,15.0f};
const static t_velo_param param_2900_acc2G = {2.9f,20.0f};
const static t_straight_param st_param_2900 = {&param_2900,&sp_gain_2900,&om_gain_2900};
const static t_straight_param st_param_2900_acc2G = {&param_2900_acc2G,&sp_gain_2900,&om_gain_2900};

const static t_pid_gain sp_gain_3000 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_3000 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_3000 = {3.0f,15.0f};
const static t_velo_param param_3000_acc2G = {3.0f,20.0f};
const static t_straight_param st_param_3000 = {&param_3000,&sp_gain_3000,&om_gain_3000};
const static t_straight_param st_param_3000_acc2G = {&param_3000_acc2G,&sp_gain_3000,&om_gain_3000};

const static t_pid_gain sp_gain_3200 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_3200 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_3200 = {3.2f,15.0f};
const static t_velo_param param_3200_acc2G = {3.2f,20.0f};
const static t_straight_param st_param_3200 = {&param_3200,&sp_gain_3200,&om_gain_3200};
const static t_straight_param st_param_3200_acc2G = {&param_3200_acc2G,&sp_gain_3200,&om_gain_3200};

const static t_pid_gain sp_gain_3400 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_3400 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_3400 = {3.4f,15.0f};
const static t_velo_param param_3400_acc2G = {3.4f,20.0f};
const static t_straight_param st_param_3400 = {&param_3400,&sp_gain_3400,&om_gain_3400};
const static t_straight_param st_param_3400_acc2G = {&param_3400_acc2G,&sp_gain_3400,&om_gain_3400};

const static t_pid_gain sp_gain_3600 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_3600 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_3600 = {3.6f,15.0f};
const static t_velo_param param_3600_acc2G = {3.6f,20.0f};
const static t_straight_param st_param_3600 = {&param_3600,&sp_gain_3600,&om_gain_3600};
const static t_straight_param st_param_3600_acc2G = {&param_3600_acc2G,&sp_gain_3600,&om_gain_3600};

const static t_pid_gain sp_gain_3800 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_3800 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_3800 = {3.8f,15.0f};
const static t_velo_param param_3800_acc2G = {3.8f,20.0f};
const static t_straight_param st_param_3800 = {&param_3800,&sp_gain_3800,&om_gain_3800};
const static t_straight_param st_param_3800_acc2G = {&param_3800_acc2G,&sp_gain_3800,&om_gain_3800};

const static t_pid_gain sp_gain_4000 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_4000 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_4000 = {4.0f,15.0f};
const static t_velo_param param_4000_acc2G = {4.0f,20.0f};
const static t_straight_param st_param_4000 = {&param_4000,&sp_gain_4000,&om_gain_4000};
const static t_straight_param st_param_4000_acc2G = {&param_4000_acc2G,&sp_gain_4000,&om_gain_4000};

const static t_pid_gain sp_gain_4200 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_4200 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_4200 = {4.2f,15.0f};
const static t_velo_param param_4200_acc2G = {4.2f,20.0f};
const static t_straight_param st_param_4200 = {&param_4200,&sp_gain_4200,&om_gain_4200};
const static t_straight_param st_param_4200_acc2G = {&param_4200_acc2G,&sp_gain_4200,&om_gain_4200};

const static t_pid_gain sp_gain_4400 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_4400 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_4400 = {4.4f,15.0f};
const static t_velo_param param_4400_acc2G = {4.4f,20.0f};
const static t_straight_param st_param_4400 = {&param_4400,&sp_gain_4400,&om_gain_4400};
const static t_straight_param st_param_4400_acc2G = {&param_4400_acc2G,&sp_gain_4400,&om_gain_4400};

const static t_pid_gain sp_gain_4600 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_4600 = {0.60f, 0.05f, 0.00f};
const static t_velo_param param_4600 = {4.6f,15.0f};
const static t_velo_param param_4600_acc2G = {4.6f,20.0f};
const static t_straight_param st_param_4600 = {&param_4600,&sp_gain_4600,&om_gain_4600};
const static t_straight_param st_param_4600_acc2G = {&param_4600_acc2G,&sp_gain_4600,&om_gain_4600};




const static t_straight_param *const st_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const st_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const st_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};
const static t_straight_param *const st_mode_700_v0[] = {&st_param_700};
const static t_straight_param *const st_mode_1000_v0[] = {&st_param_1000};
const static t_straight_param *const st_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
const static t_straight_param *const st_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
const static t_straight_param *const st_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,
														  &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000	};

const static t_straight_param *const st_mode_1400_v0[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000	};

const static t_straight_param *const st_mode_1400_v1[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000,&st_param_3200	,&st_param_3400		};
const static t_straight_param *const st_mode_1400_v2[] = {	&st_param_1400_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};

const static t_straight_param *const st_mode_1500_v0[] = {	&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000		};
const static t_straight_param *const st_mode_1500_v1[] = {	&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G		};

const static t_straight_param *const st_mode_1500_v2[] = {	&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};


const static t_straight_param *const st_mode_1600_v1[] = {	&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};


const static t_straight_param *const di_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const di_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const di_mode_500_v0[] = {&st_param_500,&st_param_600,&st_param_700};
const static t_straight_param *const di_mode_700_v0[] = {&st_param_700};
const static t_straight_param *const di_mode_1000_v0[] = {&st_param_1000};
const static t_straight_param *const di_mode_1000_v1[] = {&st_param_1000,&st_param_1100,&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_2000};
const static t_straight_param *const di_mode_1200_v0[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500};
const static t_straight_param *const di_mode_1200_v1[] = {&st_param_1200,&st_param_1300,&st_param_1400,&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,
		  	  	  	  	  	  	  	  	  	  	  	  	  &st_param_2200,&st_param_2400,&st_param_2600,&st_param_2800,&st_param_3000	};
const static t_straight_param *const di_mode_1400_v0[] = {&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000};
const static t_straight_param *const di_mode_1400_v1[] = {	&st_param_1400,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
															&st_param_2600,&st_param_2800,&st_param_3000	};
const static t_straight_param *const di_mode_1400_v2[] =  {	&st_param_1400_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};

const static t_straight_param *const di_mode_1500_v0[] = {&st_param_1500,&st_param_1600,&st_param_1800,&st_param_2000,&st_param_2200,&st_param_2400,
														  &st_param_2600,&st_param_2800,&st_param_3000	};
const static t_straight_param *const di_mode_1500_v1[] = {	&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G		};
const static t_straight_param *const di_mode_1500_v2[] =  {	&st_param_1500_acc2G,&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};

const static t_straight_param *const di_mode_1600_v1[] =  {	&st_param_1600_acc2G,&st_param_1800_acc2G,&st_param_2000_acc2G,&st_param_2200_acc2G,&st_param_2400_acc2G,
															&st_param_2600_acc2G,&st_param_2800_acc2G,&st_param_3000_acc2G,&st_param_3200_acc2G,&st_param_3400_acc2G,&st_param_3600_acc2G,
															&st_param_3800_acc2G,&st_param_4000_acc2G,&st_param_4200_acc2G,&st_param_4400_acc2G,&st_param_4600_acc2G,};


const static t_pid_gain sp_gain_dummy = {0.0f,0.0f,0.0f};
const static t_pid_gain om_gain_dummy = {0.0f, 0.0f, 0.0f};
const static t_turn_param_table slalom_dummy = {0.0f,0.0f,0.0f,0.0f,0.0f,Turn_L};
const static t_param param_dummy = {&slalom_dummy,&sp_gain_dummy,&om_gain_dummy};

const static t_pid_gain sp_gain_turn90_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn90_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_300_table = {0.30f, 37.5f,39.05,39.80, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_300_table = {0.30f,-37.5f,39.05,39.80,-90.0f,Turn_R};
const static t_param param_L90_300 = {&slalom_L90_300_table,&sp_gain_turn90_300,&om_gain_turn90_300};
const static t_param param_R90_300 = {&slalom_R90_300_table,&sp_gain_turn90_300,&om_gain_turn90_300};

const static t_pid_gain sp_gain_turn180_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turn180_300 = {0.4, 0.005, 0.0};
const static t_turn_param_table slalom_L180_300_table = {0.30f, 42.5f,23.63,24.47, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_300_table = {0.30f,-42.5f,23.63,24.47,-180.0f,Turn_R};
const static t_param param_L180_300 = {&slalom_L180_300_table,&sp_gain_turn180_300,&om_gain_turn180_300};
const static t_param param_R180_300 = {&slalom_R180_300_table,&sp_gain_turn180_300,&om_gain_turn180_300};

const static t_pid_gain sp_gain_turnV90_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnV90_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_300_table = {0.30f, 30.0f,22.78,23.50, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_300_table = {0.30f,-30.0f,22.78,23.50,-90.0f,Turn_R};
const static t_param param_LV90_300 = {&slalom_LV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};
const static t_param param_RV90_300 = {&slalom_RV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};

const static t_pid_gain sp_gain_turnIn45_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn45_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_300_table = {0.30f, 30.0f,27.04,46.34, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_300_table = {0.30f,-30.0f,27.04,46.34,-45.0f,Turn_R};
const static t_param param_inL45_300 = {&slalom_inL45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};
const static t_param param_inR45_300 = {&slalom_inR45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};

const static t_pid_gain sp_gain_turnOut45_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut45_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_300_table = {0.30f, 30.0f,45.68,27.70, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_300_table = {0.30f,-30.0f,45.68,27.70,-45.0f,Turn_R};
const static t_param param_outL45_300 = {&slalom_outL45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};
const static t_param param_outR45_300 = {&slalom_outR45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};

const static t_pid_gain sp_gain_turnIn135_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnIn135_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL135_300_table = {0.30f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_300_table = {0.30f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_300 = {&slalom_inL135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};
const static t_param param_inR135_300 = {&slalom_inR135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};

const static t_pid_gain sp_gain_turnOut135_300 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_300 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_300_table = {0.30f, 30.0f,37.57,46.07, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_300_table = {0.30f,-30.0f,37.57,46.07,-135.0f,Turn_R};
const static t_param param_outL135_300 = {&slalom_outL135_300_table,&sp_gain_turnOut135_300,&om_gain_turnOut135_300};
const static t_param param_outR135_300 = {&slalom_outR135_300_table,&sp_gain_turnOut135_300,&om_gain_turnOut135_300};

const static t_param *const mode_300[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_300,		&param_L90_300,
												&param_R180_300,	&param_L180_300,
												&param_inR45_300,	&param_inL45_300,
												&param_outR45_300,	&param_outL45_300,
												&param_inR135_300,	&param_inL135_300,
												&param_outR135_300,	&param_outL135_300,
												&param_RV90_300,	&param_LV90_300
											};


const static t_pid_gain sp_gain_turn90_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turn90_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_L90_500_table = {0.50f, 37.5f,39.05,39.80, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_500_table = {0.50f,-37.5f,39.05,39.80,-90.0f,Turn_R};
const static t_param param_L90_500 = {&slalom_L90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};
const static t_param param_R90_500 = {&slalom_R90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};

const static t_pid_gain sp_gain_turn180_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turn180_500 = {0.6, 0.005, 0.0};
const static t_turn_param_table slalom_L180_500_table = {0.50f, 42.5f,23.63,24.47, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_500_table = {0.50f,-42.5f,23.63,24.47,-180.0f,Turn_R};
const static t_param param_L180_500 = {&slalom_L180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};
const static t_param param_R180_500 = {&slalom_R180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};

const static t_pid_gain sp_gain_turnV90_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnV90_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_500_table = {0.50f, 41.0f,6.36,12.60, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-41.0f,6.36,12.60,-90.0f,Turn_R};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};

const static t_pid_gain sp_gain_turnIn45_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn45_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_500_table = {0.50f, 50.0f,13.31,38.00, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-50.0f,13.31,38.00,-45.0f,Turn_R};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};

const static t_pid_gain sp_gain_turnOut45_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut45_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_500_table = {0.50f, 50.0f,19.33,32.01, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-50.0f,19.33,32.01,-45.0f,Turn_R};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};

const static t_pid_gain sp_gain_turnIn135_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn135_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_inL135_500_table = {0.50f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};

const static t_pid_gain sp_gain_turnOut135_500 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_500_table = {0.50f, 30.0f,37.57,46.07, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_500_table = {0.50f,-30.0f,37.57,46.07,-135.0f,Turn_R};
const static t_param param_outL135_500 = {&slalom_outL135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};
const static t_param param_outR135_500 = {&slalom_outR135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};

const static t_param *const mode_500[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_500,		&param_L90_500,
												&param_R180_500,	&param_L180_500,
												&param_inR45_500,	&param_inL45_500,
												&param_outR45_500,	&param_outL45_500,
												&param_inR135_500,	&param_inL135_500,
												&param_outR135_500,	&param_outL135_500,
												&param_RV90_500,	&param_LV90_500
											};



//-----------velo = 500 mm/s parameters
const static t_pid_gain sp_gain_turn90_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn90_700 = {0.8, 0.002, 0.0};
const static t_turn_param_table slalom_L90_700_table = {0.70f, 42.5f,29.97,38.86, 90.0f,Turn_L};// k= 100
const static t_turn_param_table slalom_R90_700_table = {0.70f,-42.5f,29.97,38.86,-90.0f,Turn_R};// k= 100
const static t_param param_L90_700 = {&slalom_L90_700_table,&sp_gain_turn90_700,&om_gain_turn90_700};
const static t_param param_R90_700 = {&slalom_R90_700_table,&sp_gain_turn90_700,&om_gain_turn90_700};

const static t_pid_gain sp_gain_turn180_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn180_700 = {0.8, 0.02, 0.0};
const static t_turn_param_table slalom_L180_700_table = {0.70f, 43.5f,19.28,30.11, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_700_table = {0.70f,-43.5f,19.28,30.11,-180.0f,Turn_R};
const static t_param param_L180_700 = {&slalom_L180_700_table,&sp_gain_turn180_700,&om_gain_turn180_700};
const static t_param param_R180_700 = {&slalom_R180_700_table,&sp_gain_turn180_700,&om_gain_turn180_700};

const static t_pid_gain sp_gain_turnV90_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnV90_700 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_LV90_700_table = {0.70f, 38.50f,9.18,18.18, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_700_table = {0.70f,-38.50f,9.18,18.18,-90.0f,Turn_R};
const static t_param param_LV90_700 = {&slalom_LV90_700_table,&sp_gain_turnV90_700,&om_gain_turnV90_700};
const static t_param param_RV90_700 = {&slalom_RV90_700_table,&sp_gain_turnV90_700,&om_gain_turnV90_700};

const static t_pid_gain sp_gain_turnIn45_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn45_700 = {0.4, 0.02, 0.0};
const static t_turn_param_table slalom_inL45_700_table = {0.70f, 48.5f,13.12,40.31, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_700_table = {0.70f,-48.5f,13.12,40.31,-45.0f,Turn_R};
const static t_param param_inL45_700 = {&slalom_inL45_700_table,&sp_gain_turnIn45_700,&om_gain_turnIn45_700};
const static t_param param_inR45_700 = {&slalom_inR45_700_table,&sp_gain_turnIn45_700,&om_gain_turnIn45_700};

const static t_pid_gain sp_gain_turnOut45_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut45_700 = {0.4, 0.02, 0.0};;
const static t_turn_param_table slalom_outL45_700_table = {0.70f, 48.5f,31.81,24.64, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_700_table = {0.70f,-48.5f,31.81,24.64,-45.0f,Turn_R};
const static t_param param_outL45_700 = {&slalom_outL45_700_table,&sp_gain_turnOut45_700,&om_gain_turnOut45_700};
const static t_param param_outR45_700 = {&slalom_outR45_700_table,&sp_gain_turnOut45_700,&om_gain_turnOut45_700};

const static t_pid_gain sp_gain_turnIn135_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn135_700 = {0.8, 0.02, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_700_table = {0.70f, 40.5f,13.29,14.42, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_700_table = {0.70f,-40.5f,13.29,14.42,-135.0f,Turn_R};
const static t_param param_inL135_700 = {&slalom_inL135_700_table,&sp_gain_turnIn135_700,&om_gain_turnIn135_700};
const static t_param param_inR135_700 = {&slalom_inR135_700_table,&sp_gain_turnIn135_700,&om_gain_turnIn135_700};

const static t_pid_gain sp_gain_turnOut135_700 = {15.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_700 = {0.8, 0.02, 0.0};
const static t_turn_param_table slalom_outL135_700_table = {0.70f, 40.5f,5.62,22.17, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_700_table = {0.70f,-40.5f,5.62,22.17,-135.0f,Turn_R};
const static t_param param_outL135_700 = {&slalom_outL135_700_table,&sp_gain_turnOut135_700,&om_gain_turnOut135_700};
const static t_param param_outR135_700 = {&slalom_outR135_700_table,&sp_gain_turnOut135_700,&om_gain_turnOut135_700};

const static t_param *const mode_700[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_700,		&param_L90_700,
												&param_R180_700,	&param_L180_700,
												&param_inR45_700,	&param_inL45_700,
												&param_outR45_700,	&param_outL45_700,
												&param_inR135_700,	&param_inL135_700,
												&param_outR135_700,	&param_outL135_700,
												&param_RV90_700,	&param_LV90_700
											};

//k = 200

const static t_pid_gain sp_gain_turn90_1000 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L90_1000_table = {1.00f, 42.5f,29.48,39.30, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1000_table = {1.00f,-42.5f,29.48,39.30,-90.0f,Turn_R};
const static t_param param_L90_1000 = {&slalom_L90_1000_table,&sp_gain_turn90_1000,&om_gain_turn90_1000};
const static t_param param_R90_1000 = {&slalom_R90_1000_table,&sp_gain_turn90_1000,&om_gain_turn90_1000};

const static t_pid_gain sp_gain_turn180_1000 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_L180_1000_table = {1.00f, 45.0f,18.54,28.49, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1000_table = {1.00f,-45.0f,18.54,28.49,-180.0f,Turn_R};
const static t_param param_L180_1000 = {&slalom_L180_1000_table,&sp_gain_turn180_1000,&om_gain_turn180_1000};
const static t_param param_R180_1000 = {&slalom_R180_1000_table,&sp_gain_turn180_1000,&om_gain_turn180_1000};

//not adjust
const static t_pid_gain sp_gain_turnV90_1000 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_LV90_1000_table = {1.00f, 37.5f,10.20,19.62, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1000_table = {1.00f,-37.5f,10.20,19.62,-90.0f,Turn_R};
const static t_param param_LV90_1000 = {&slalom_LV90_1000_table,&sp_gain_turnV90_1000,&om_gain_turnV90_1000};
const static t_param param_RV90_1000 = {&slalom_RV90_1000_table,&sp_gain_turnV90_1000,&om_gain_turnV90_1000};

const static t_pid_gain sp_gain_turnIn45_1000 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_inL45_1000_table = {1.00f, 45.0f,15.52,42.41, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1000_table = {1.00f,-45.0f,15.52,42.41,-45.0f,Turn_R};
const static t_param param_inL45_1000 = {&slalom_inL45_1000_table,&sp_gain_turnIn45_1000,&om_gain_turnIn45_1000};
const static t_param param_inR45_1000 = {&slalom_inR45_1000_table,&sp_gain_turnIn45_1000,&om_gain_turnIn45_1000};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1000 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL45_1000_table = {1.00f, 50.0f,31.04,21.30, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1000_table = {1.00f,-50.0f,31.04,21.30,-45.0f,Turn_R};
const static t_param param_outL45_1000 = {&slalom_outL45_1000_table,&sp_gain_turnOut45_1000,&om_gain_turnOut45_1000};
const static t_param param_outR45_1000 = {&slalom_outR45_1000_table,&sp_gain_turnOut45_1000,&om_gain_turnOut45_1000};


const static t_pid_gain sp_gain_turnIn135_1000 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn135_1000 = {0.4, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1000_table = {1.00f, 37.5f,27.18,29.87, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1000_table = {1.00f,-37.5f,27.18,29.87,-135.0f,Turn_R};
const static t_param param_inL135_1000 = {&slalom_inL135_1000_table,&sp_gain_turnIn135_1000,&om_gain_turnIn135_1000};
const static t_param param_inR135_1000 = {&slalom_inR135_1000_table,&sp_gain_turnIn135_1000,&om_gain_turnIn135_1000};

//
const static t_pid_gain sp_gain_turnOut135_1000 = {6.0, 0.05, 0.0};
const static t_pid_gain om_gain_turnOut135_1000 = {0.4, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1000_table = {1.00f, 38.0f,18.00,35.81, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1000_table = {1.00f,-38.0f,18.00,35.81,-135.0f,Turn_R};
const static t_param param_outL135_1000 = {&slalom_outL135_1000_table,&sp_gain_turnOut135_1000,&om_gain_turnOut135_1000};
const static t_param param_outR135_1000 = {&slalom_outR135_1000_table,&sp_gain_turnOut135_1000,&om_gain_turnOut135_1000};

const static t_param *const mode_1000[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1000,		&param_L90_1000,
												&param_R180_1000,	&param_L180_1000,
												&param_inR45_1000,	&param_inL45_1000,
												&param_outR45_1000,	&param_outL45_1000,
												&param_inR135_1000,	&param_inL135_1000,
												&param_outR135_1000,	&param_outL135_1000,
												&param_RV90_1000,	&param_LV90_1000
											};


const static t_pid_gain sp_gain_turn90_1200 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1200 = {0.6, 0.01, 0.0};
const static t_turn_param_table slalom_L90_1200_table = {1.20f, 46.5f,26.97,38.94, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1200_table = {1.20f,-46.5f,24.56,38.94,-90.0f,Turn_R};
const static t_param param_L90_1200 = {&slalom_L90_1200_table,&sp_gain_turn90_1200,&om_gain_turn90_1200};
const static t_param param_R90_1200 = {&slalom_R90_1200_table,&sp_gain_turn90_1200,&om_gain_turn90_1200};

const static t_pid_gain sp_gain_turn180_1200 = {20.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1200 = {0.6, 0.1, 0.0};
/*
const static t_turn_param_table slalom_L180_1200_table = {1.20f, 49.0f,13.17,27.04, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1200_table = {1.20f,-49.0f,13.17,27.04,-180.0f,Turn_R};
*/
const static t_turn_param_table slalom_L180_1200_table = {1.20f, 46.0f,18.99,31.27, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1200_table = {1.20f,-46.0f,18.99,31.27,-180.0f,Turn_R};
const static t_param param_L180_1200 = {&slalom_L180_1200_table,&sp_gain_turn180_1200,&om_gain_turn180_1200};
const static t_param param_R180_1200 = {&slalom_R180_1200_table,&sp_gain_turn180_1200,&om_gain_turn180_1200};

//not adjust
const static t_pid_gain sp_gain_turnV90_1200 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1200 = {0.6, 0.01, 0.0};
const static t_turn_param_table slalom_LV90_1200_table = {1.20f, 41.0f,9.58,21.35, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1200_table = {1.20f,-41.0f,9.58,21.35,-90.0f,Turn_R};
const static t_param param_LV90_1200 = {&slalom_LV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200};
const static t_param param_RV90_1200 = {&slalom_RV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200};

const static t_pid_gain sp_gain_turnIn45_1200 = {20.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1200 = {0.6, 0.01, 0.0};
const static t_turn_param_table slalom_inL45_1200_table = {1.20f, 50.0f,12.52,41.14, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1200_table = {1.20f,-50.0f,12.52,41.14,-45.0f,Turn_R};
const static t_param param_inL45_1200 = {&slalom_inL45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200};
const static t_param param_inR45_1200 = {&slalom_inR45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1200 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1200 = {0.6, 0.01, 0.0};
const static t_turn_param_table slalom_outL45_1200_table = {1.20f, 50.0f,31.21,22.48, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1200_table = {1.20f,-50.0f,31.21,22.48,-45.0f,Turn_R};
const static t_param param_outL45_1200 = {&slalom_outL45_1200_table,&sp_gain_turnOut45_1200,&om_gain_turnOut45_1200};
const static t_param param_outR45_1200 = {&slalom_outR45_1200_table,&sp_gain_turnOut45_1200,&om_gain_turnOut45_1200};


const static t_pid_gain sp_gain_turnIn135_1200 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn135_1200 = {0.6, 0.05, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1200_table = {1.20f, 40.0f,24.34,30.49, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1200_table = {1.20f,-40.0f,24.34,30.49,-135.0f,Turn_R};
const static t_param param_inL135_1200 = {&slalom_inL135_1200_table,&sp_gain_turnIn135_1200,&om_gain_turnIn135_1200};
const static t_param param_inR135_1200 = {&slalom_inR135_1200_table,&sp_gain_turnIn135_1200,&om_gain_turnIn135_1200};

//
const static t_pid_gain sp_gain_turnOut135_1200 = {12.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_1200 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_outL135_1200_table = {1.20f, 41.0f,16.73,38.29, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1200_table = {1.20f,-41.0f,16.73,38.29,-135.0f,Turn_R};
const static t_param param_outL135_1200 = {&slalom_outL135_1200_table,&sp_gain_turnOut135_1200,&om_gain_turnOut135_1200};
const static t_param param_outR135_1200 = {&slalom_outR135_1200_table,&sp_gain_turnOut135_1200,&om_gain_turnOut135_1200};

const static t_param *const mode_1200[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1200,		&param_L90_1200,
												&param_R180_1200,	&param_L180_1200,
												&param_inR45_1200,	&param_inL45_1200,
												&param_outR45_1200,	&param_outL45_1200,
												&param_inR135_1200,	&param_inL135_1200,
												&param_outR135_1200,	&param_outL135_1200,
												&param_RV90_1200,	&param_LV90_1200
											};

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
const static t_turn_param_table slalom_L180_1400_table = {1.40f, 46.0f,14.40,31.24, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1400_table = {1.40f,-46.0f,14.40,31.24,-180.0f,Turn_R};
//const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 40.0f,19.73,36.36, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-40.0f,19.73,36.36,-90.0f,Turn_R};
const static t_param param_L180_1400 = {&slalom_L180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};
const static t_param param_R180_1400 = {&slalom_R180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};

//not adjust
//k = 300
const static t_pid_gain sp_gain_turnV90_1400 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1400 = {0.6, 0.1, 0.0};
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
//const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 55.0f,26.22,22.11, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-55.0f,26.22,22.11,-45.0f,Turn_R};
const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 50.5f,28.54,19.64, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-50.5f,28.54,19.64,-45.0f,Turn_R};
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




//k = 250
const static t_pid_gain sp_gain_turn90_1500 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1500 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_L90_1500_table = {1.50f, 55.5f,14.32,32.76, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1500_table = {1.50f,-55.5f,14.32,32.76,-90.0f,Turn_R};
const static t_param param_L90_1500 = {&slalom_L90_1500_table,&sp_gain_turn90_1500,&om_gain_turn90_1500};
const static t_param param_R90_1500 = {&slalom_R90_1500_table,&sp_gain_turn90_1500,&om_gain_turn90_1500};

const static t_pid_gain sp_gain_turn180_1500 = {20.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1500 = {0.6, 0.05, 0.0};
//const static t_turn_param_table slalom_L180_1500_table = {1.50f, 52.50f,6.91,28.66, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1500_table = {1.50f,-52.50f,6.91,28.66,-180.0f,Turn_R};
const static t_turn_param_table slalom_L180_1500_table = {1.50f, 46.0f,13.69,33.69, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1500_table = {1.50f,-46.0f,13.69,33.69,-180.0f,Turn_R};
const static t_param param_L180_1500 = {&slalom_L180_1500_table,&sp_gain_turn180_1500,&om_gain_turn180_1500};
const static t_param param_R180_1500 = {&slalom_R180_1500_table,&sp_gain_turn180_1500,&om_gain_turn180_1500};
//k = 400
//not adjust
const static t_pid_gain sp_gain_turnV90_1500 = {20.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnV90_1500 = {0.8, 0.1, 0.0};
const static t_turn_param_table slalom_LV90_1500_table = {1.50f, 40.0f,6.76,26.60, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_1500_table = {1.50f,-40.0f,6.76,26.60,-90.0f,Turn_R};
//const static t_turn_param_table slalom_LV90_1500_table = {1.50f, 37.00f,10.85,20.72, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1500_table = {1.50f,-37.00f,10.85,20.72,-90.0f,Turn_R};
const static t_param param_LV90_1500 = {&slalom_LV90_1500_table,&sp_gain_turnV90_1500,&om_gain_turnV90_1500};
const static t_param param_RV90_1500 = {&slalom_RV90_1500_table,&sp_gain_turnV90_1500,&om_gain_turnV90_1500};

//k = 300
const static t_pid_gain sp_gain_turnIn45_1500 = {20.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn45_1500 = {0.6, 0.05, 0.0};
//const static t_turn_param_table slalom_inL45_1500_table = {1.50f, 52.50f,9.02,42.23, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1500_table = {1.50f,-52.50f,9.02,42.23,-45.0f,Turn_R};
const static t_turn_param_table slalom_inL45_1500_table = {1.50f, 55.0f,7.01,37.83, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_1500_table = {1.50f,-55.0f,7.02,37.83,-45.0f,Turn_R};
const static t_param param_inL45_1500 = {&slalom_inL45_1500_table,&sp_gain_turnIn45_1500,&om_gain_turnIn45_1500};
const static t_param param_inR45_1500 = {&slalom_inR45_1500_table,&sp_gain_turnIn45_1500,&om_gain_turnIn45_1500};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1500 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut45_1500 = {0.6, 0.05, 0.0};
//const static t_turn_param_table slalom_outL45_1500_table = {1.50f, 52.5f,27.75,23.57, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1500_table = {1.50f,-52.5f,27.75,23.57,-45.0f,Turn_R};
const static t_turn_param_table slalom_outL45_1500_table = {1.50f, 55.0f,25.09,17.83, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_1500_table = {1.50f,-55.0f,25.09,17.83,-45.0f,Turn_R};
const static t_param param_outL45_1500 = {&slalom_outL45_1500_table,&sp_gain_turnOut45_1500,&om_gain_turnOut45_1500};
const static t_param param_outR45_1500 = {&slalom_outR45_1500_table,&sp_gain_turnOut45_1500,&om_gain_turnOut45_1500};

//k = 250 , Kp = 4.0
const static t_pid_gain sp_gain_turnIn135_1500 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnIn135_1500 = {0.8f, 0.1, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1500_table = {1.50f, 42.5f,(16.54),26.17, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1500_table = {1.50f,-42.5f,(16.54),26.17,-135.0f,Turn_R};
const static t_param param_inL135_1500 = {&slalom_inL135_1500_table,&sp_gain_turnIn135_1500,&om_gain_turnIn135_1500};
const static t_param param_inR135_1500 = {&slalom_inR135_1500_table,&sp_gain_turnIn135_1500,&om_gain_turnIn135_1500};

//
const static t_pid_gain sp_gain_turnOut135_1500 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_1500 = {0.8, 0.1, 0.0};
const static t_turn_param_table slalom_outL135_1500_table = {1.50f, 42.50f,8.91,34.95, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_1500_table = {1.50f,-42.50f,8.91,34.95,-135.0f,Turn_R};
const static t_param param_outL135_1500 = {&slalom_outL135_1500_table,&sp_gain_turnOut135_1500,&om_gain_turnOut135_1500};
const static t_param param_outR135_1500 = {&slalom_outR135_1500_table,&sp_gain_turnOut135_1500,&om_gain_turnOut135_1500};

const static t_param *const mode_1500[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1500,		&param_L90_1500,
												&param_R180_1500,	&param_L180_1500,
												&param_inR45_1500,	&param_inL45_1500,
												&param_outR45_1500,	&param_outL45_1500,
												&param_inR135_1500,	&param_inL135_1500,
												&param_outR135_1500,	&param_outL135_1500,
												&param_RV90_1500,	&param_LV90_1500
											};

const static t_pid_gain sp_gain_turn90_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn90_1600 = {0.6, 0.05, 0.0};
const static t_turn_param_table slalom_L90_1600_table = {1.60f, 50.50f,19.56,38.43, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_1600_table = {1.60f,-50.50f,19.56,38.43,-90.0f,Turn_R};
const static t_param param_L90_1600 = {&slalom_L90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};
const static t_param param_R90_1600 = {&slalom_R90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};

const static t_pid_gain sp_gain_turn180_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turn180_1600 = {0.8, 0.1, 0.0};
const static t_turn_param_table slalom_L180_1600_table = {1.60f, 50.50f,8.80,30.10, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_1600_table = {1.60f,-50.50f,8.80,30.10,-180.0f,Turn_R};
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
const static t_pid_gain om_gain_turnIn135_1600 = {0.8, 0.1, 0.0};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 44.0f,13.17,26.34, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-44.0f,13.17,26.34,-135.0f,Turn_R};
const static t_param param_inL135_1600 = {&slalom_inL135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600};
const static t_param param_inR135_1600 = {&slalom_inR135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600};

//
const static t_pid_gain sp_gain_turnOut135_1600 = {15.0, 0.05, 0.00};
const static t_pid_gain om_gain_turnOut135_1600 = {0.8, 0.1, 0.0};
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

#endif /* CPP_INC_RUN_PARAM_H_ */

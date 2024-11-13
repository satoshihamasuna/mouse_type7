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
#include "typedef_run_param.h"
#include "turn_300.h"
#include "turn_500.h"
#include "turn_700.h"
#include "turn_1000.h"
#include "turn_1200.h"
#include "turn_1400.h"
#include "turn_1500.h"
#include "turn_1600.h"


const static t_pid_gain basic_sp_gain = {10.0,0.15,0.0};
const static t_pid_gain basic_om_gain = {0.40f, 0.01f, 0.00f};

const static t_pid_gain search_sp_gain = {12.0,0.15,0.0};
const static t_pid_gain search_om_gain = {0.60f, 0.01f, 0.00f};

const static t_pid_gain sp_gain_search_turn = {12.0,0.1,0.0};//{12.0,0.1,0.0};
const static t_pid_gain om_gain_search_turn = {0.60, 0.01, 0.00};//{0.50f, 0.0005f, 0.001f};
const static t_turn_param_table slalom_L90_table = {0.32f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const static t_turn_param_table slalom_R90_table = {0.32f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const static t_param param_L90_search = {&slalom_L90_table ,&sp_gain_search_turn,&om_gain_search_turn};
const static t_param param_R90_search = {&slalom_R90_table, &sp_gain_search_turn,&om_gain_search_turn};

const static t_pid_gain sp_gain_300 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_300 = {0.65, 0.01, 0.00};
const static t_velo_param param_300 = {0.30f,4.0f};
const static t_straight_param st_param_300 = {&param_300,&sp_gain_300,&om_gain_300};

const static t_pid_gain sp_gain_320 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_320 = {0.65, 0.01, 0.00};
const static t_velo_param param_320 = {0.32f,4.0f};
const static t_straight_param st_param_320 = {&param_320,&sp_gain_320,&om_gain_320};

const static t_pid_gain sp_gain_450 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_450 = {0.4, 0.005, 0.0};
const static t_velo_param param_450 = {0.45f,6.0f};
const static t_straight_param st_param_450 = {&param_450,&sp_gain_450,&om_gain_450};

const static t_pid_gain sp_gain_500 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_500 = {0.4, 0.005, 0.0};
const static t_velo_param param_500 = {0.50f,6.0f};
const static t_straight_param st_param_500 = {&param_500,&sp_gain_500,&om_gain_500};

const static t_pid_gain sp_gain_600 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_600 = {0.4, 0.005, 0.0};
const static t_velo_param param_600 = {0.60f,6.0f};
const static t_straight_param st_param_600 = {&param_600,&sp_gain_600,&om_gain_600};

const static t_pid_gain sp_gain_700 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_700 = {0.4, 0.005, 0.0};
const static t_velo_param param_700 = {0.70f,6.0f};
const static t_straight_param st_param_700 = {&param_700,&sp_gain_700,&om_gain_700};


const static t_pid_gain sp_gain_1000 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1000 = {0.4, 0.005, 0.0};
const static t_velo_param param_1000 = {1.0f,9.0f};
const static t_straight_param st_param_1000 = {&param_1000,&sp_gain_1000,&om_gain_1000};

const static t_pid_gain sp_gain_1050 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1050 = {0.4, 0.005, 0.0};
const static t_velo_param param_1050 = {1.05f,9.0f};
const static t_straight_param st_param_1050 = {&param_1050,&sp_gain_1050,&om_gain_1050};

const static t_pid_gain sp_gain_1100 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1100 = {0.4, 0.005, 0.0};
const static t_velo_param param_1100 = {1.10f,9.0f};
const static t_straight_param st_param_1100 = {&param_1100,&sp_gain_1100,&om_gain_1100};

const static t_pid_gain sp_gain_1200 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1200 = {0.4, 0.005, 0.0};
const static t_velo_param param_1200 = {1.20f,10.0f};
const static t_straight_param st_param_1200 = {&param_1200,&sp_gain_1200,&om_gain_1200};

const static t_pid_gain sp_gain_1300 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1300 = {0.4, 0.005, 0.0};
const static t_velo_param param_1300 = {1.30f,10.0f};
const static t_straight_param st_param_1300 = {&param_1300,&sp_gain_1300,&om_gain_1300};

const static t_pid_gain sp_gain_1400 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1400 = {0.4, 0.01, 0.0};
const static t_velo_param param_1400 = {1.40f,12.0f};
const static t_velo_param param_1400_acc2G = {1.40f,20.0f};
const static t_straight_param st_param_1400 = {&param_1400,&sp_gain_1400,&om_gain_1400};
const static t_straight_param st_param_1400_acc2G  = {&param_1400_acc2G ,&sp_gain_1400,&om_gain_1400};

const static t_pid_gain sp_gain_1500 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1500 = {0.4, 0.01, 0.0};
const static t_velo_param param_1500 = {1.50f,12.0f};
const static t_velo_param param_1500_acc2G = {1.50f,20.0f};
const static t_straight_param st_param_1500 = {&param_1500,&sp_gain_1500,&om_gain_1500};
const static t_straight_param st_param_1500_acc2G  = {&param_1500_acc2G ,&sp_gain_1500,&om_gain_1500};

const static t_pid_gain sp_gain_1600 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1600 = {0.4, 0.01, 0.0};
const static t_velo_param param_1600 = {1.60f,12.0f};
const static t_velo_param param_1600_acc2G = {1.60f,20.0f};
const static t_straight_param st_param_1600 = {&param_1600,&sp_gain_1600,&om_gain_1600};
const static t_straight_param st_param_1600_acc2G  = {&param_1600_acc2G ,&sp_gain_1600,&om_gain_1600};

const static t_pid_gain sp_gain_1700 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1700 = {0.4, 0.01, 0.0};
const static t_velo_param param_1700 = {1.70f,12.0f};
const static t_velo_param param_1700_acc2G = {1.70f,20.0f};
const static t_straight_param st_param_1700 = {&param_1700,&sp_gain_1700,&om_gain_1700};
const static t_straight_param st_param_1700_acc2G = {&param_1700_acc2G,&sp_gain_1700,&om_gain_1700};

const static t_pid_gain sp_gain_1800 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1800 = {0.4, 0.01, 0.0};;
const static t_velo_param param_1800 = {1.80f,12.0f};
const static t_velo_param param_1800_acc2G = {1.80f,20.0f};
const static t_straight_param st_param_1800 = {&param_1800,&sp_gain_1800,&om_gain_1800};
const static t_straight_param st_param_1800_acc2G = {&param_1800_acc2G,&sp_gain_1800,&om_gain_1800};

const static t_pid_gain sp_gain_1900 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_1900 = {0.4, 0.01, 0.0};
const static t_velo_param param_1900 = {1.90f,12.0f};
const static t_velo_param param_1900_acc2G = {1.90f,20.0f};
const static t_straight_param st_param_1900 = {&param_1900,&sp_gain_1900,&om_gain_1900};
const static t_straight_param st_param_1900_acc2G = {&param_1900_acc2G,&sp_gain_1900,&om_gain_1900};

const static t_pid_gain sp_gain_2000 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_2000 = {0.4, 0.01, 0.0};
const static t_velo_param param_2000 = {2.0f,12.0f};
const static t_velo_param param_2000_acc2G = {2.0f,20.0f};
const static t_straight_param st_param_2000 = {&param_2000,&sp_gain_2000,&om_gain_2000};
const static t_straight_param st_param_2000_acc2G = {&param_2000_acc2G,&sp_gain_2000,&om_gain_2000};


const static t_pid_gain sp_gain_2100 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_2100 = {0.4, 0.01, 0.0};
const static t_velo_param param_2100 = {2.1f,12.0f};
const static t_velo_param param_2100_acc2G = {2.1f,20.0f};
const static t_straight_param st_param_2100 = {&param_2100,&sp_gain_2100,&om_gain_2100};
const static t_straight_param st_param_2100_acc2G = {&param_2100_acc2G,&sp_gain_2100,&om_gain_2100};

const static t_pid_gain sp_gain_2200 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_2200 = {0.4, 0.01, 0.0};
const static t_velo_param param_2200 = {2.2f,12.0f};
const static t_velo_param param_2200_acc2G = {2.2f,20.0f};
const static t_straight_param st_param_2200 = {&param_2200,&sp_gain_2200,&om_gain_2200};
const static t_straight_param st_param_2200_acc2G = {&param_2200_acc2G,&sp_gain_2200,&om_gain_2200};

const static t_pid_gain sp_gain_2300 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_2300 = {0.4, 0.01, 0.0};
const static t_velo_param param_2300 = {2.3f,12.0f};
const static t_velo_param param_2300_acc2G = {2.3f,20.0f};
const static t_straight_param st_param_2300 = {&param_2300,&sp_gain_2300,&om_gain_2300};
const static t_straight_param st_param_2300_acc2G = {&param_2300_acc2G,&sp_gain_2300,&om_gain_2300};

const static t_pid_gain sp_gain_2400 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_2400 = {0.4, 0.01, 0.0};
const static t_velo_param param_2400 = {2.4f,12.0f};
const static t_velo_param param_2400_acc2G = {2.4f,20.0f};
const static t_straight_param st_param_2400 = {&param_2400,&sp_gain_2400,&om_gain_2400};
const static t_straight_param st_param_2400_acc2G = {&param_2400_acc2G,&sp_gain_2400,&om_gain_2400};

const static t_pid_gain sp_gain_2500 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_2500 = {0.4, 0.01, 0.0};
const static t_velo_param param_2500 = {2.5f,12.0f};
const static t_velo_param param_2500_acc2G = {2.5f,20.0f};
const static t_straight_param st_param_2500 = {&param_2500,&sp_gain_2500,&om_gain_2500};
const static t_straight_param st_param_2500_acc2G = {&param_2500_acc2G,&sp_gain_2500,&om_gain_2500};

const static t_pid_gain sp_gain_2600 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_2600 = {0.4, 0.01, 0.0};
const static t_velo_param param_2600 = {2.6f,15.0f};
const static t_velo_param param_2600_acc2G = {2.6f,20.0f};
const static t_straight_param st_param_2600 = {&param_2600,&sp_gain_2600,&om_gain_2600};
const static t_straight_param st_param_2600_acc2G = {&param_2600_acc2G,&sp_gain_2600,&om_gain_2600};

const static t_pid_gain sp_gain_2700 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2700 = {0.4, 0.01, 0.0};
const static t_velo_param param_2700 = {2.7f,15.0f};
const static t_velo_param param_2700_acc2G = {2.7f,20.0f};
const static t_straight_param st_param_2700 = {&param_2700,&sp_gain_2700,&om_gain_2700};
const static t_straight_param st_param_2700_acc2G = {&param_2700_acc2G,&sp_gain_2700,&om_gain_2700};


const static t_pid_gain sp_gain_2800 = {15.0,0.15,0.0};
const static t_pid_gain om_gain_2800 = {0.4, 0.01, 0.0};
const static t_velo_param param_2800 = {2.8f,15.0f};
const static t_velo_param param_2800_acc2G = {2.8f,20.0f};
const static t_straight_param st_param_2800 = {&param_2800,&sp_gain_2800,&om_gain_2800};
const static t_straight_param st_param_2800_acc2G = {&param_2800_acc2G,&sp_gain_2800,&om_gain_2800};


const static t_pid_gain sp_gain_2900 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_2900 = {0.4, 0.01, 0.0};
const static t_velo_param param_2900 = {2.9f,15.0f};
const static t_velo_param param_2900_acc2G = {2.9f,20.0f};
const static t_straight_param st_param_2900 = {&param_2900,&sp_gain_2900,&om_gain_2900};
const static t_straight_param st_param_2900_acc2G = {&param_2900_acc2G,&sp_gain_2900,&om_gain_2900};

const static t_pid_gain sp_gain_3000 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_3000 = {0.4, 0.01, 0.0};
const static t_velo_param param_3000 = {3.0f,15.0f};
const static t_velo_param param_3000_acc2G = {3.0f,20.0f};
const static t_straight_param st_param_3000 = {&param_3000,&sp_gain_3000,&om_gain_3000};
const static t_straight_param st_param_3000_acc2G = {&param_3000_acc2G,&sp_gain_3000,&om_gain_3000};

const static t_pid_gain sp_gain_3200 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_3200 = {0.4, 0.01, 0.0};
const static t_velo_param param_3200 = {3.2f,15.0f};
const static t_velo_param param_3200_acc2G = {3.2f,20.0f};
const static t_straight_param st_param_3200 = {&param_3200,&sp_gain_3200,&om_gain_3200};
const static t_straight_param st_param_3200_acc2G = {&param_3200_acc2G,&sp_gain_3200,&om_gain_3200};

const static t_pid_gain sp_gain_3400 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_3400 = {0.4, 0.01, 0.0};
const static t_velo_param param_3400 = {3.4f,15.0f};
const static t_velo_param param_3400_acc2G = {3.4f,20.0f};
const static t_straight_param st_param_3400 = {&param_3400,&sp_gain_3400,&om_gain_3400};
const static t_straight_param st_param_3400_acc2G = {&param_3400_acc2G,&sp_gain_3400,&om_gain_3400};

const static t_pid_gain sp_gain_3600 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_3600 = {0.4, 0.01, 0.0};
const static t_velo_param param_3600 = {3.6f,15.0f};
const static t_velo_param param_3600_acc2G = {3.6f,20.0f};
const static t_straight_param st_param_3600 = {&param_3600,&sp_gain_3600,&om_gain_3600};
const static t_straight_param st_param_3600_acc2G = {&param_3600_acc2G,&sp_gain_3600,&om_gain_3600};

const static t_pid_gain sp_gain_3800 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_3800 = {0.4, 0.01, 0.0};
const static t_velo_param param_3800 = {3.8f,15.0f};
const static t_velo_param param_3800_acc2G = {3.8f,20.0f};
const static t_straight_param st_param_3800 = {&param_3800,&sp_gain_3800,&om_gain_3800};
const static t_straight_param st_param_3800_acc2G = {&param_3800_acc2G,&sp_gain_3800,&om_gain_3800};

const static t_pid_gain sp_gain_4000 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_4000 = {0.4, 0.01, 0.0};
const static t_velo_param param_4000 = {4.0f,15.0f};
const static t_velo_param param_4000_acc2G = {4.0f,20.0f};
const static t_straight_param st_param_4000 = {&param_4000,&sp_gain_4000,&om_gain_4000};
const static t_straight_param st_param_4000_acc2G = {&param_4000_acc2G,&sp_gain_4000,&om_gain_4000};

const static t_pid_gain sp_gain_4200 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_4200 = {0.4, 0.01, 0.0};
const static t_velo_param param_4200 = {4.2f,15.0f};
const static t_velo_param param_4200_acc2G = {4.2f,20.0f};
const static t_straight_param st_param_4200 = {&param_4200,&sp_gain_4200,&om_gain_4200};
const static t_straight_param st_param_4200_acc2G = {&param_4200_acc2G,&sp_gain_4200,&om_gain_4200};

const static t_pid_gain sp_gain_4400 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_4400 = {0.4, 0.01, 0.0};
const static t_velo_param param_4400 = {4.4f,15.0f};
const static t_velo_param param_4400_acc2G = {4.4f,20.0f};
const static t_straight_param st_param_4400 = {&param_4400,&sp_gain_4400,&om_gain_4400};
const static t_straight_param st_param_4400_acc2G = {&param_4400_acc2G,&sp_gain_4400,&om_gain_4400};

const static t_pid_gain sp_gain_4600 = {10.0,0.15,0.0};
const static t_pid_gain om_gain_4600 = {0.4, 0.01, 0.0};
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


const static t_param *const *const acc_mode_1400[] = {mode_1400_acc,mode_1600_acc};


#endif /* CPP_INC_RUN_PARAM_H_ */

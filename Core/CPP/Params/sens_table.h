/*
 * sens_table.h
 *
 *  Created on: 2023/06/24
 *      Author: sato1
 */

#ifndef INCLUDE_SENS_TABLE_H_
#define INCLUDE_SENS_TABLE_H_

const static int16_t sens_side_length_table[]=
{
		25,
		30,
		35,
		40,
		45,
		50,
		55,
		60,
		65,
		70,
		75,
		80
};

const static int16_t sens_sr_table[]=
{
		3240,//25
		2780,//30
		2040,//35
		1500,//40
		1150,//45
		850,//50
		700,//55
		590,//60
		470,//65
		420,//70
		390,//75
		335//80
};

const static int16_t sens_sl_table[]=
{
		3380,//25
		2700,//30
		1880,//35
		1260,//40
		980,//45
		675,//50
		550,//55
		420,//60
		350,//65
		290,//70
		230,//75
		190//80
};
const static int16_t sens_front_length_table[]=
{
		40,
		45,
		50,
		55,
		60,
		65,
		70,
		75,
		80,
		85,
		90,
		95,
		100,
		105,
		110,
		115,
		120,
		125
};

const static int16_t sens_fr_table[]=
{
		2500,//40
		2300,//45
		1650,//50
		1170,//55
		900,//60
		680,//65
		550,//70
		470,//75
		380,//80
		310,//85
		270,//90
		220,//95
		180,//100
		160,//105
		140,//110
		120,//115
		110,//120
		100
};

const static int16_t sens_fl_table[]=
{
		2100,//40
		2000,//45
		1500,//50
		1150,//55
		900,//60
		690,//65
		560,//70
		460,//75
		380,//80
		330,//85
		240,//90
		220,//95
		200,//100
		170,//105
		150,//110
		125,//115
		110,//120
		100//125
};



#endif /* INCLUDE_SENS_TABLE_H_ */

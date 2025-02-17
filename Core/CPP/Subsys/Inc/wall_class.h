/*
 * wall_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_WALL_CLASS_H_
#define CPP_INC_WALL_CLASS_H_

#include "../../Pheripheral/Include/typedef.h"
#include "../../Pheripheral/Include/macro.h"
#include "../../Task/Inc/sensing_task.h"


typedef struct
{
	int8_t x;
	int8_t y;
	t_wall wall;
}t_histry_wall;

class wall_class
{
	IrSensTask *ir_sens;
	uint8_t histry_cnt;

	public:
		wall_class(IrSensTask *ir_sens_)
		{
			ir_sens = ir_sens_;
		}
		IrSensTask *return_irObj() {return ir_sens;};
		t_wall wall[MAZE_SIZE_X][MAZE_SIZE_Y];
		t_histry_wall histry_wall[MAZE_SIZE];
		void init_maze();
		void set_wall(t_position pos);
		t_bool is_unknown(uint16_t x,uint16_t y);
		void goal_set_vwall(int gx,int gy,int goal_size){
			if(goal_size == 3)
			{
				wall[gx+1][gy+1].north = wall[gx+1][gy+1].east = wall[gx+1][gy+1].south = wall[gx+1][gy+1].west = VWALL;
				wall[gx+1][gy+2].south  = wall[gx+2][gy+1].west = wall[gx+1][gy+0].north = wall[gx+0][gy+1].east = VWALL;
			}

		}
		void goal_clear_vwall(int gx,int gy,int goal_size){
			if(goal_size == 3)
			{
				wall[gx+1][gy+1].north = wall[gx+1][gy+1].east = wall[gx+1][gy+1].south = wall[gx+1][gy+1].west = NOWALL;
				wall[gx+1][gy+2].south = wall[gx+2][gy+1].west = wall[gx+1][gy+0].north = wall[gx+0][gy+1].east = NOWALL;
			}
		}
		t_wall_state get_WallState(t_position pos);

		void histry_init()
		{
			for(int i = 0; i < MAZE_SIZE;i++)
			{
				histry_wall[i].x = -1;
				histry_wall[i].y = -1;
				histry_wall[i].wall.north = UNKNOWN;
				histry_wall[i].wall.south = UNKNOWN;
				histry_wall[i].wall.east = UNKNOWN;
				histry_wall[i].wall.west = UNKNOWN;
			}
			histry_cnt = 0;
		}

		void histry_set(int x,int y,t_wall wall)
		{
			histry_wall[histry_cnt].x = x;
			histry_wall[histry_cnt].y = y;
			histry_wall[histry_cnt].wall.north = wall.north;
			histry_wall[histry_cnt].wall.south = wall.south;
			histry_wall[histry_cnt].wall.east = wall.east;
			histry_wall[histry_cnt].wall.west = wall.east;
			histry_cnt++;
		}

		void histry_delete(int num)
		{
			if(num > histry_cnt) num = histry_cnt;
			for(int i = 0; i < num; i++)
			{
				histry_wall[histry_cnt-i].x = -1;
				histry_wall[histry_cnt-i].y = -1;
				histry_wall[histry_cnt-i].wall.north = UNKNOWN;
				histry_wall[histry_cnt-i].wall.south = UNKNOWN;
				histry_wall[histry_cnt-i].wall.east = UNKNOWN;
				histry_wall[histry_cnt-i].wall.west = UNKNOWN;
			}
		}

		void histry2wall()
		{

		}

};

/*
class wall_class_type7: public wall_class,public Singleton<wall_class_type7>
{
public:
	wall_class_type7(IrSensTask *ir = &IrSensTask_type7::getInstance()):wall_class(ir){}
};
*/

#endif /* CPP_INC_WALL_CLASS_H_ */

/*
 * search_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SEARCH_CLASS_H_
#define CPP_INC_SEARCH_CLASS_H_


#include "wall_class.h"
#include "make_map_class.h"
#include "adachi_class.h"

#include "../../Task/Inc/ctrl_task.h"

class Search
{
	private:
		wall_class *wall_property;
		make_map   *map_property;
		Motion *motion;
	public:
		t_position search_adachi_1(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion );
		t_position search_adachi_1_acc(	t_position start_pos,t_position goal_pos,int goal_size,
												wall_class *_wall,make_map *_map,Motion *motion );

		t_position search_adachi_2(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion );

		t_position search_adachi_2_acc(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion );

		t_position search_adachi_3(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion );

		t_position search_adachi_3_acc(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion );


		t_bool i_am_goal(int x,int y,int gx,int gy,int goal_size);
		t_bool i_am_goal(t_position pos,t_position g_pos,int goal_size);

};


#endif /* CPP_INC_SEARCH_CLASS_H_ */

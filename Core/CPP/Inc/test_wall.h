/*
 * test_wall.h
 *
 *  Created on: 2023/07/06
 *      Author: sato1
 */

#ifndef CPP_INC_TEST_WALL_H_
#define CPP_INC_TEST_WALL_H_
//#include "../Application/Inc/wall_class.h"
#include "../Pheripheral/Include/typedef.h"
#include "../Pheripheral/Include/macro.h"
#include "../Pheripheral/Include/index.h"

#define TEST_GOAL_X 17
#define TEST_GOAL_Y 13
#define TEST_GOAL_SIZE 3

namespace test_maze
{
	const t_bool test_wall[MAZE_SIZE*4] = {
		False,True,True,True,False,True,True,True,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,False,True,True,False,False,False,True,True,False,True,True,False,
		False,True,False,True,False,False,False,True,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,False,True,False,True,False,False,False,True,False,True,False,False,False,True,False,True,
		False,True,False,True,False,True,False,True,False,False,True,True,True,False,True,False,False,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,False,False,True,True,False,True,False,False,False,False,True,True,True,False,True,False,False,True,True,False,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,False,True,False,True,False,True,False,True,True,True,False,True,True,True,False,True,False,True,False,True,
		False,False,False,True,True,True,False,False,False,True,False,True,False,False,True,True,False,True,False,False,False,False,True,True,False,False,True,False,True,False,False,False,False,False,True,False,False,False,True,False,False,True,True,False,False,True,False,True,False,False,True,True,False,False,True,False,False,False,True,False,False,True,True,False,True,False,False,True,True,True,False,False,True,True,False,True,False,True,False,True,False,False,True,True,True,True,False,False,False,True,False,True,False,False,True,True,False,False,True,False,False,True,True,False,False,True,False,True,False,True,False,True,True,False,False,True,True,False,True,False,True,False,True,False,False,True,False,False,
		False,True,False,True,False,False,True,True,False,True,False,False,False,True,False,True,False,False,False,True,True,True,False,False,False,True,False,True,False,False,True,True,True,True,False,False,False,True,False,True,False,False,False,True,False,True,False,False,False,False,False,True,False,False,False,False,False,False,False,False,False,True,False,False,False,False,True,True,True,False,True,False,False,True,True,False,False,True,False,True,True,False,False,True,False,True,True,False,False,True,False,True,False,False,False,True,False,False,False,False,False,True,False,False,False,True,False,True,True,False,False,True,True,False,True,False,True,False,True,False,False,True,True,False,False,True,False,True,
		False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,True,True,False,False,True,False,True,True,False,False,True,False,True,True,False,False,True,False,True,True,True,False,True,False,True,False,True,False,False,False,True,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,False,True,True,True,True,False,False,False,True,False,True,False,False,True,True,True,True,False,False,False,True,False,True,True,False,False,True,True,False,False,False,True,True,False,False,False,False,False,True,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,False,True,False,True,
		True,False,False,True,False,True,False,False,True,False,False,True,True,True,False,False,True,True,False,True,True,False,False,True,True,False,False,False,False,True,True,False,False,True,False,True,False,False,False,True,False,True,True,False,False,True,False,True,True,False,False,True,True,False,False,False,True,False,False,False,True,True,False,False,False,True,False,True,False,True,False,True,False,False,True,True,False,True,False,False,False,True,False,True,False,True,True,True,True,False,False,True,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,False,False,True,True,False,False,True,False,True,False,True,False,True,True,True,False,False,True,False,True,
		False,False,True,True,False,True,False,False,False,False,True,True,False,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,True,False,True,False,True,False,True,False,True,False,True,True,True,False,True,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,False,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,False,True,True,True,False,True,False,True,False,True,False,True,False,False,True,True,True,False,True,False,True,True,False,False,
		False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,False,True,False,False,False,True,False,False,False,True,False,False,True,True,False,True,False,False,True,True,True,False,False,False,False,True,True,True,True,False,False,False,False,True,True,True,False,True,False,True,False,True,False,False,False,True,False,True,False,False,False,False,True,False,False,False,True,False,True,True,False,False,True,True,False,True,False,True,True,False,False,False,False,True,True,True,False,True,False,True,True,False,False,False,False,False,True,False,True,False,False,False,True,False,True,True,False,False,True,True,False,True,False,False,True,True,False,
		False,True,False,True,False,True,False,True,False,True,False,True,True,True,False,True,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,False,True,True,True,False,True,False,True,True,False,False,False,False,True,True,True,False,False,False,True,False,True,False,True,False,True,False,True,False,False,False,True,True,True,False,False,True,False,True,True,False,False,True,True,False,True,False,True,False,True,False,False,True,True,False,True,False,False,True,False,False,True,False,False,False,True,False,False,True,False,False,False,True,False,True,True,False,False,True,True,False,True,False,False,True,True,False,False,True,False,True,
		False,True,False,True,False,True,False,True,True,False,False,True,False,True,True,False,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,True,True,True,True,False,True,True,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,False,False,True,True,True,False,True,False,False,True,True,False,True,False,False,True,False,True,True,False,True,True,False,True,True,True,False,True,True,True,False,True,True,False,False,True,True,False,True,False,False,True,True,False,False,True,False,True,False,True,False,True,
		False,True,False,True,False,False,False,True,False,True,True,False,False,True,False,True,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,False,False,True,False,False,True,False,False,True,True,False,False,True,True,True,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,True,False,False,True,False,False,False,False,True,True,True,True,False,False,True,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,False,False,False,True,False,True,False,True,False,True,
		False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,True,False,False,False,True,False,False,False,True,False,False,False,True,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,False,False,False,False,True,False,False,False,True,True,True,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,False,True,False,True,True,False,False,True,False,True,True,False,False,False,True,True,False,False,True,False,False,False,True,False,False,False,True,False,False,True,True,False,False,True,False,True,False,True,False,True,False,True,False,True,
		False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,False,False,True,False,True,False,True,False,False,False,True,False,True,False,True,False,True,True,True,False,False,True,False,True,False,False,False,True,False,False,False,False,False,True,False,False,False,True,False,True,False,False,False,True,False,False,False,False,False,True,False,False,False,False,True,True,False,False,True,False,False,True,True,False,False,True,False,True,False,True,False,True,False,False,True,True,True,True,False,False,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,
		False,True,False,True,False,True,False,True,True,False,False,True,False,True,True,False,False,False,False,True,True,True,True,False,False,False,False,True,True,False,True,False,True,True,True,False,False,False,False,True,False,True,False,False,False,True,False,True,False,False,False,True,False,False,False,False,False,True,False,False,False,True,False,True,False,False,False,True,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,True,False,True,True,False,False,True,False,True,True,False,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,
		False,True,False,True,True,False,False,True,True,False,True,False,False,True,False,False,False,False,False,True,True,True,True,False,True,False,False,True,True,False,True,False,False,True,True,False,False,True,False,True,False,False,False,True,False,False,False,False,False,True,False,False,False,True,False,True,False,False,False,True,False,False,False,False,False,True,False,False,True,False,False,True,True,False,False,False,True,True,False,False,False,True,False,True,False,True,False,True,False,False,True,True,True,True,False,False,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,
		False,True,False,True,False,False,True,True,True,False,True,False,True,True,False,False,False,False,False,True,True,False,True,False,True,False,True,False,False,True,True,False,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,False,False,False,False,True,False,False,True,True,False,True,True,False,False,True,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,False,True,False,True,True,False,False,True,False,True,True,False,True,False,False,True,True,False,False,False,True,False,False,False,True,False,False,False,True,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,
		False,True,False,True,False,True,False,True,False,False,True,True,True,False,True,False,True,False,False,False,True,False,True,False,True,False,True,False,True,True,False,False,False,True,False,True,False,True,False,True,True,False,False,True,True,False,False,False,True,True,False,False,True,True,False,True,True,False,False,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,True,False,False,True,True,False,True,False,True,True,False,False,True,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,False,False,False,True,False,True,False,True,False,True,
		False,True,False,True,False,True,False,True,True,False,False,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,True,False,False,True,False,True,True,False,False,True,False,True,True,False,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,False,True,True,True,False,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,False,True,False,True,False,True,False,True,
		False,True,False,True,False,True,False,True,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,False,True,True,False,True,False,False,False,True,False,True,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,False,False,True,True,False,True,True,False,False,False,True,True,True,True,False,False,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,False,True,False,True,
		False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,False,True,False,False,False,True,False,False,False,True,False,False,False,True,False,False,False,True,False,False,True,True,False,False,True,False,True,False,True,False,True,True,False,False,True,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,True,True,True,False,False,True,True,True,False,True,False,True,False,True,False,True,False,False,False,True,True,True,False,False,False,False,True,True,True,True,False,False,False,False,True,True,False,False,True,False,False,False,True,False,True,True,True,False,False,False,True,True,False,True,True,False,False,True,False,True,
		False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,True,False,False,True,True,False,True,False,False,True,True,False,True,False,True,True,False,False,True,False,True,True,False,False,False,False,True,True,False,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,True,False,False,False,False,True,True,True,False,False,False,False,False,False,False,True,False,False,False,False,False,True,False,True,True,False,False,False,True,False,True,False,True,False,True,
		False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,False,True,True,True,False,True,False,True,True,False,False,False,False,True,True,True,True,False,False,False,False,True,True,True,True,False,False,True,False,False,True,True,True,False,False,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,True,False,False,False,True,True,True,False,True,True,False,True,True,False,True,True,False,False,False,False,False,False,True,False,True,True,False,False,False,True,False,True,
		False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,True,False,True,False,True,True,True,False,False,True,True,True,True,False,False,False,False,True,True,True,True,False,False,False,False,True,True,True,False,True,False,True,False,True,False,True,True,False,False,False,False,False,True,True,False,False,False,True,False,True,False,False,True,True,False,False,False,True,True,False,False,True,False,False,True,True,False,True,True,False,True,False,True,False,True,False,True,True,True,False,True,False,True,
		False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,True,False,True,False,False,False,True,True,True,False,False,False,False,True,True,True,True,False,False,True,False,True,True,True,False,False,False,True,False,True,False,True,False,True,False,False,True,True,False,False,False,False,True,False,False,True,False,False,True,True,False,False,True,False,True,False,False,False,True,False,False,False,False,False,True,False,False,False,False,True,True,True,False,False,False,False,True,False,False,False,True,False,True,
		False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,False,True,False,False,False,True,True,True,False,False,False,False,True,True,True,True,False,False,False,False,True,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,True,False,True,False,True,False,True,True,False,False,True,False,True,False,False,True,False,False,True,True,False,False,False,True,True,False,False,False,False,False,True,False,True,True,False,False,True,False,True,False,True,False,True,
		False,True,False,True,False,True,False,True,True,False,False,True,False,True,True,False,True,False,False,True,True,False,False,False,True,False,False,False,True,False,False,False,True,False,False,False,True,True,False,False,True,False,False,True,True,True,False,False,False,False,True,True,True,True,False,False,False,False,True,True,True,True,False,False,False,False,True,True,False,False,True,False,False,False,True,False,False,False,True,False,False,True,True,False,False,False,True,True,True,False,False,False,False,False,True,False,True,True,False,False,False,False,True,True,True,False,True,False,False,False,True,False,True,True,False,False,False,False,False,True,False,True,False,False,False,True,False,True,
		False,True,False,True,True,False,False,True,False,True,True,False,True,False,False,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,True,True,False,False,False,True,True,True,False,True,False,True,True,False,False,True,False,True,True,True,False,False,False,False,True,True,False,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,True,False,False,True,False,True,True,False,False,True,False,True,False,False,True,True,False,True,False,False,False,False,True,True,True,False,False,False,False,False,True,False,False,True,False,False,True,False,False,True,False,True,False,False,
		False,False,False,True,True,True,True,False,False,True,False,True,False,False,True,True,True,False,True,False,True,False,True,False,False,True,True,False,False,False,True,True,True,False,True,False,True,True,False,False,False,True,False,True,False,True,True,True,False,True,True,True,False,True,True,True,False,True,True,True,False,True,False,True,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,True,True,True,False,True,False,True,False,False,False,True,False,True,False,False,True,False,False,True,False,True,False,False,False,False,True,True,True,True,False,False,False,True,False,True,True,False,True,True,False,True,False,False,
		False,False,False,True,True,True,True,False,False,True,False,True,True,False,False,True,True,False,True,False,False,True,True,False,False,True,False,True,False,True,False,True,False,False,True,True,False,True,True,False,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,False,False,True,False,False,False,False,False,False,False,False,False,False,False,False,False,True,False,False,False,False,False,True,True,False,False,False,False,True,False,False,True,False,False,True,False,True,True,False,True,False,False,True,False,False,False,False,False,False,True,False,True,False,False,False,False,False,True,False,False,True,False,False,
		False,False,False,True,False,True,True,False,True,False,False,True,True,False,True,False,True,False,True,False,True,True,False,False,True,False,False,True,True,True,False,False,False,True,False,True,False,True,False,True,True,True,False,True,True,True,False,True,True,True,False,True,True,True,False,True,True,True,False,True,True,True,False,True,True,False,False,True,True,False,False,False,True,False,False,False,True,False,False,False,True,True,False,False,True,True,False,True,True,False,True,True,True,False,False,False,True,False,True,False,True,False,False,False,True,False,True,False,True,True,False,False,True,True,False,True,False,True,True,True,False,True,False,True,False,True,False,True,
		True,True,False,True,True,False,False,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,True,False,False,True,False,False,True,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,True,False,False,False,True,False,False,False,True,True,False,False,
	};


	void test_wall_set(wall_class *wall_property)
	{
		for(uint8_t j = 0;j < MAZE_SIZE_Y;j++)
		{
			for(uint8_t i = 0; i < MAZE_SIZE_X;i++)
			{
				wall_property->wall[i][j].north = (uint8_t)test_wall[ j * MAZE_SIZE_Y*4 + i * 4 + 0 ];
				wall_property->wall[i][j].east  = (uint8_t)test_wall[ j * MAZE_SIZE_Y*4 + i * 4 + 1 ];
				wall_property->wall[i][j].south = (uint8_t)test_wall[ j * MAZE_SIZE_Y*4 + i * 4 + 2 ];
				wall_property->wall[i][j].west  = (uint8_t)test_wall[ j * MAZE_SIZE_Y*4 + i * 4 + 3 ];
			}
		}
	}
}

#endif /* CPP_INC_TEST_WALL_H_ */

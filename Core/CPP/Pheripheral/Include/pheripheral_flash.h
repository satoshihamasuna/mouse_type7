/*
 * flash.h
 *
 *  Created on: 2024/03/06
 *      Author: sato1
 */

#ifndef CPP_PHERIPHERAL_INCLUDE_FLASH_H_
#define CPP_PHERIPHERAL_INCLUDE_FLASH_H_


#include "stm32f4xx_hal.h"

#include "index.h"
#include "macro.h"
#include "../../Subsys/Inc/wall_class.h"

#include <string.h>
#include <stdint.h>

#define START_ADDRESS  	0x8060000
#define END_ADDRESS    	0x807FFFF

#define MAP_START_ADDRESS 	0x8060000
#define MAP_END_ADDRESS	  	0x806FFFF

#define WALL_START_ADDRESS  0x8070000
#define WALL_END_ADDRESS	0x807FFFF

void eraseALL();
void eraseFlash( void );
void writeFlash(uint32_t address, uint16_t *data );
void loadFlash(uint32_t address,uint16_t *data, uint32_t size );



#endif /* CPP_PHERIPHERAL_INCLUDE_FLASH_H_ */

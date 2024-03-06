/*
 * pheripheral_flash.cpp
 *
 *  Created on: 2024/03/06
 *      Author: sato1
 */


#include "stm32f4xx_hal.h"

#include "../Include/index.h"
#include "../Include/macro.h"
#include "../Include/pheripheral_flash.h"
#include "../../Subsys/Inc/wall_class.h"

#include <string.h>
#include <stdint.h>

void eraseALL(){
	HAL_FLASH_Unlock();
	eraseFlash();
	HAL_FLASH_Lock();
}


void eraseFlash( void )
{
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;	// select sector
	erase.Sector = FLASH_SECTOR_7;		       // set selector7
	erase.NbSectors = 1;		// set to erase one sector
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;	// set voltage range (2.7 to 3.6V)

	uint32_t pageError = 0;

	HAL_FLASHEx_Erase(&erase, &pageError);	// erase sector
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
*/
void writeFlash(uint32_t address, uint16_t *data )
{
	//HAL_FLASH_Unlock();		// unlock flash

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, *data); // write byte

	//HAL_FLASH_Lock();		// lock flash
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data read data
 * @param uint32_t size read data size
*/
void loadFlash(uint32_t address,uint16_t *data, uint32_t size )
{
	memcpy(data, (uint16_t*) address, size); // copy data
}

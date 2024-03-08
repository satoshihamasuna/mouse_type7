/*
 * communicate.h
 *
 *  Created on: 2024/03/08
 *      Author: sato1
 */

#ifndef CPP_MODULE_INC_COMMUNICATE_H_
#define CPP_MODULE_INC_COMMUNICATE_H_

#include "../../Component/Inc/singleton.h"
#include "../../Pheripheral/Include/macro.h"

uint8_t 	Communicate_TerminalRecv( void );		// 1文字受信
void 		Communicate_Initialize( void );			// printfとscanfを使用するための設定
#endif /* CPP_MODULE_INC_COMMUNICATE_H_ */

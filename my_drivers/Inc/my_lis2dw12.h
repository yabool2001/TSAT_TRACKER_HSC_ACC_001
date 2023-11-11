/*
 * my_lis2dw12.h
 *
 *  Created on: Nov 9, 2023
 *      Author: mzeml
 */

#ifndef MY_LIS2DW12_H_
#define MY_LIS2DW12_H_

#include "lis2dw12_reg.h"
#include <stdbool.h>

#define ACC_ID				0x44U// LIS2DW12 Device Identification (Who am I)
#define ACC_WAKEUP_THS		4
#define ACC_WAKEUP_DUR		2
#define ACC_LIR				1

//ACC


bool 	my_lis2dw12_init ( stmdev_ctx_t* ) ;
uint8_t my_lis2dw12_get_id ( stmdev_ctx_t* ) ;

#endif /* MY_LIS2DW12_H_ */

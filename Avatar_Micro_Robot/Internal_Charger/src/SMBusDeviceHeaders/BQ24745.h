/*******************************************************************************
File: BQ24745.h

Description: This module provides macros to facilitate access to configuration
  bits and registers within the BQ242745 multi-chemistry battery charger.
*******************************************************************************/
#ifndef BQ24745_H
#define BQ24745_H
/*---------------------------Macros-------------------------------------------*/

// pre-defined responses
#define BQ24745_MANUFACTURER_ID 	0x0040
#define BQ24745_DEVICE_ID 				0x0006
#define BQ24745_SLAVE_ADDRESS     0x0009  // 7-bit address (not bit-shifted 1)

// read or write addresses
#define BQ24745_CHARGE_CURRENT 	  0x14
#define BQ24745_CHARGE_VOLTAGE 	  0x15
#define BQ24745_INPUT_CURRENT 		0x3F

#endif

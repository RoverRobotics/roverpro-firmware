/*******************************************************************************
File: BQ242745.h

Description: This module provides macros to facilitate access to configuration
  bits and registers within the BQ242745 multi-chemistry battery charger.
*******************************************************************************/
#ifndef BQ242745_H
#define BQ242745_H
/*---------------------------Macros-------------------------------------------*/

// pre-defined responses
#define BQ242745_MANUFACTURER_ID 	0x0040
#define BQ242745_DEVICE_ID 				0x0006


// read or write addresses
#define BQ242745_CHARGE_CURRENT 	0x14
#define BQ242745_CHARGE_VOLTAGE 	0x15
#define BQ242745_INPUT_CURRENT 		0x3F


#endif

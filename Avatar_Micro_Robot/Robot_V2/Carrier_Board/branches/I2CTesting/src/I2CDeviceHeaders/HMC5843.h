/*******************************************************************************
File: HMC5843.h

Description: This module provides macros to facilitate access to configuration
  bits and registers within the HMC5843 humidity sensor.

TODO: finish and organize in a useful way
*******************************************************************************/
#ifndef HMC5843_H
#define HMC5843_H
/*---------------------------Macros-------------------------------------------*/

#define HMC5843_ADDRESS         	0x1E

#define HMC5843_CONFIG_A          0
#define HMC5843_CONFIG_B          1
#define HMC5843_MODE              2

// read-only
#define HMC5843_X_A               3   // MSB of data
#define HMC5843_X_B               4   // LSB of data
#define HMC5843_Y_A               5
#define HMC5843_Y_B               6
#define HMC5843_Z_A               7
#define HMC5843_Z_B               8

#define HMC5843_STATUS            9
#define HMC5843_REN_BIT           2
#define HMC5843_LOCK_BIT          2
#define HMC5843_RDY_BIT           2



#define HMC5843_ID_A              10
#define HMC5843_ID_B              11
#define HMC5843_ID_C              12

#endif

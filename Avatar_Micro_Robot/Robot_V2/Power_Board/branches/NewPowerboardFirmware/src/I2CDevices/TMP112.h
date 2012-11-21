/*******************************************************************************
File: TMP112.h

Description: This module provides macros to facilitate access to configuration
  bits and registers within the I2C-interfaced temperature sensor, TMP112.
*******************************************************************************/
#ifndef TMP112_H
#define TMP112_H
/*---------------------------Macros-------------------------------------------*/

// Note: the slave address is pin-programmable (see schematic)

#define TMP112_TEMP   0x00  // the subaddress from which to grab the first of
                            // two bytes of data? TODO: CONFIRM THIS


#endif

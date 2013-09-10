/*******************************************************************************
File: ADXL345.h

Description: This module provides macros to facilitate access to configuration
  bit and registers within the ADXL345 accelerometer.

TODO: finish and organize in a useful way
*******************************************************************************/
#ifndef ADXL345_H
#define ADXL345_H
/*---------------------------Macros-------------------------------------------*/

#define ADXL345_ADDRESS           0x53


#define ADXL345_DEVID             0x00
#define ADXL345_THRESH_TAP        0x1d
#define ADXL345_OFSX              0x1e
#define ADXL345_OFSY              0x1f
#define ADXL345_OFSZ              0x20
#define ADXL345_DUR               0x21
#define ADXL345_LATENT            0x22
#define ADXL345_WINDOW            0x23
#define ADXL345_THRESH_ACT        0x24
#define ADXL345_THRESH_INACT      0x25
#define ADXL345_TIME_INACT        0x26
#define ADXL345_ACT_INACT_CTL     0x27
#define ADXL345_THRESH_FF         0x28
#define ADXL345_TIME_FF           0x29
#define ADXL345_TAP_AXES          0x2a
#define ADXL345_ACT_TAP_STATUS    0x2b
#define ADXL345_BW_RATE           0x2c
#define ADXL345_POWER_CTL         0x2d
#define ADXL345_INT_ENABLE        0x2e
#define ADXL345_INT_MAP           0x2f
#define ADXL345_INT_SOURCE        0x30
#define ADXL345_DATA_FORMAT       0x31
#define ADXL345_DATAX0            0x32
#define ADXL345_DATAX1            0x33
#define ADXL345_DATAY0            0x34
#define ADXL345_DATAY1            0x35
#define ADXL345_DATAZ0            0x36
#define ADXL345_DATAZ1            0x37
#define ADXL345_FIFO_CTL          0x38
#define ADXL345_FIFO_STATUS       0x39

// useful bits within the registers (organize better)
#define ADXL345_MEASURE_BIT       3
#define ADXL345_FULL_RES_BIT      3
#define ADXL345_RANGE_BIT0        0
#define ADXL345_RANGE_BIT1        1


#endif

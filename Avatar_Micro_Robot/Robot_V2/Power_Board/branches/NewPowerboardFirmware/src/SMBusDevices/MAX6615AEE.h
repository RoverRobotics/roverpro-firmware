/*******************************************************************************
File: MAX6615AEE.h

Description: This module provides macros to facilitate access to configuration
  bits and registers within the I2C-interfaced fan controller, MAX6615AEE.
  Note that this device can communicate with either I2C or SMBus voltage 
  thresholds.
*******************************************************************************/
#ifndef MAX6615AEE_H
#define MAX6615AEE_H
/*---------------------------Macros-------------------------------------------*/

// Note: the slave address is pin-programmable (see schematic)

// sub-addresses from which to request data (read-only)
#define MAX6615AEE_TEMP1            0x00
#define MAX6615AEE_TEMP2            0x01
#define MAX6615AEE_DEVICE_REVISION  0xFD
#define MAX6615AEE_DEVICE_ID        0xFE
#define MAX6615AEE_MANUFACTURER_ID  0xFF
#define MAX6615AEE_OT_STATUS        0x05
#define MAX6615AEE_TEMP1_LSB        0x1C
#define MAX6615AEE_TEMP2_LSB        0x1E
#define MAX6615AEE_TACH1_VALUE      0x18
#define MAX6615AEE_TACH2_VALUE      0x19
#define MAX6615AEE_PWM1_INSTANT_DC  0x0D
#define MAX6615AEE_PWM2_INSTANT_DC  0x0E

// configurable sub-addresses (from which we can also read back, read/write)
#define MAX6615AEE_CONFIG       	  0x02  // see table 4 of datasheet
#define MAX6615AEE_TEMP1_OT_LIMIT   0x03
#define MAX6615AEE_TEMP2_OT_LIMIT   0x04
#define MAX6615AEE_OT_MASK          0x06
#define MAX6615AEE_PWM1_START_DC    0x07
#define MAX6615AEE_PWM2_START_DC    0x08
#define MAX6615AEE_PWM1_MAX_DC      0x09
#define MAX6615AEE_PWM2_MAX_DC      0x0A
#define MAX6615AEE_PWM1_TARGET_DC   0x0B
#define MAX6615AEE_PWM2_TARGET_DC   0x0C
#define MAX6615AEE_1_TEMP_START     0x0F
#define MAX6615AEE_2_TEMP_START     0x10
#define MAX6615AEE_FAN_CONFIG       0x11
#define MAX6615AEE_DC_RATE          0x12  // duty cycle Rate-Of-Change
#define MAX6615AEE_DC_STEP_SIZE     0x13
#define MAX6615AEE_PWM_FREQ_SELECT  0x14
#define MAX6615AEE_GPIO_FN          0x15
#define MAX6615AEE_GPIO_VALUE       0x16
#define MAX6615AEE_THERM_OFFSET     0x17
#define MAX6615AEE_TACH1_LIMIT      0x1A
#define MAX6615AEE_TACH2_LIMIT      0x1B

#endif

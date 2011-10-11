/**
 * @file stdhdr.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex one-size-fits-all firmware. Designed for the PIC24FJ256GB106 only.
 *
 */

#include <stdint.h>

#define FCY 20000000UL
#define I2C_RATE_SETTING 0X50 // 200 KHz for 32 MHz clock frequency

#include <libpic30.h>

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include <USB/usb.h>
#include "i2c.h"
#include <TimeDelay.h>
//#include <usb_config.h>
#include "interrupt_switch.h"



extern void block_ms(unsigned int ms);


#include "../HardwareProfile.h"

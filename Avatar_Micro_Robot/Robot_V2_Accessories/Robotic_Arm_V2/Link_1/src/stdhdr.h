/**
 * @file stdhdr.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex one-size-fits-all firmware. Designed for the PIC24FJ256GB106 only.
 *
 */

#ifndef STDHDR_H
#define STDHDR_H


#define FCY 16000000UL        // instruction clock
#define I2C_RATE_SETTING 0Xff // 200 KHz for 32 MHz clock frequency

#include <stdint.h>
#include <libpic30.h>

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "USB/usb.h"
#include "i2c.h"
#include "PwrMgnt.h"
#include "interrupt_switch.h"

// PROTOTYPES FOR PROJECT stdfunction.h

extern void block_ms(unsigned int ms);
extern void writeI2C( unsigned char add, unsigned char v );
extern int readI2C( unsigned char add );
extern void writeI2CReg( unsigned char add, unsigned char v, unsigned char w);
unsigned char readI2C_Reg(unsigned char add, unsigned char reg);

#include "../HardwareProfile.h"

#endif

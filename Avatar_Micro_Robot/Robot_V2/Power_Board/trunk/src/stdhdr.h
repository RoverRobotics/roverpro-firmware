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
#define I2C_RATE_SETTING 0X50 // 200 KHz for 32 MHz clock frequency

#include <stdint.h>
#include <libpic30.h>

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "USB/usb.h"
#include "i2c.h"
#include "PwrMgnt.h"
#include "interrupt_switch.h"
#include "periph_i2c.h"

// PROTOTYPES FOR PROJECT stdfunction.h

extern void block_ms(unsigned int ms);
unsigned char readI2C1_Reg(unsigned char add, unsigned char reg);
unsigned char readI2C2_Reg(unsigned char add, unsigned char reg);
unsigned char readI2C3_Reg(unsigned char add, unsigned char reg);
void writeI2C1Reg( unsigned char add, unsigned char v, unsigned char w);
void writeI2C2Reg( unsigned char add, unsigned char v, unsigned char w);
void writeI2C3Reg( unsigned char add, unsigned char v, unsigned char w);

#include "../HardwareProfile.h"

#endif

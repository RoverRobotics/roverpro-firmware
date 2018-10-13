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

#include <stdint.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "USB/usb.h"
#include "interrupt_switch.h"
#include "periph_i2c.h"
#include <stdbool.h>

// PIC24-specific breakpoint command:
#define BREAKPOINT __asm__ __volatile__(".pword 0xDA4000")
#ifdef __DEBUG
#define BREAKPOINT_IF(condition)                                                                   \
    if (condition) {                                                                               \
        BREAKPOINT;                                                                                \
    }
#else
#define BREAKPOINT_IF(condition)
#endif

// PROTOTYPES FOR PROJECT stdfunction.h

void block_ms(unsigned int ms);
unsigned char readI2C1_Reg(unsigned char add, unsigned char reg);
unsigned char readI2C2_Reg(unsigned char add, unsigned char reg);
unsigned char readI2C3_Reg(unsigned char add, unsigned char reg);
void writeI2C1Reg(unsigned char add, unsigned char v, unsigned char w);
void writeI2C2Reg(unsigned char add, unsigned char v, unsigned char w);
void writeI2C3Reg(unsigned char add, unsigned char v, unsigned char w);

void readI2C2_Block(unsigned char add, unsigned char reg, unsigned char block_length,
                    unsigned char *output);
void readI2C3_Block(unsigned char add, unsigned char reg, unsigned char block_length,
                    unsigned char *output);

/** return the nearest integer within the given range */
int clamp(int value, int lo, int hi);

/** compute the mean of an array of integers */
int mean(int count, int *values);
long mean_l(int count, long *values);

#include "../HardwareProfile.h"
#endif

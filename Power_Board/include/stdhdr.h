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
#include <stdbool.h>

// PIC24-specific breakpoint command:
#define BREAKPOINT() __asm__ __volatile__(".pword 0xDA4000")
#define BREAKPOINT_IF(condition)                                                                   \
    if (condition) {                                                                               \
        BREAKPOINT();                                                                              \
    }

// PROTOTYPES FOR PROJECT stdfunction.h

/// Block for the specified amount of time. Prevents a Watchdog Timer reset in the event of a long
/// wait.
void block_ms(uint16_t ms);

/** return the nearest integer within the given range */
int clamp(int value, int lo, int hi);
/** return the nearest float within the given range */
float clamp_f(float value, float lo, float hi);

/** compute the mean of an array of ints */
int16_t mean(size_t count, int16_t *values);
/** compute the mean of an array of longs */
uint16_t mean_u(size_t count, uint16_t *values);
#include "HardwareProfile.h"

#endif

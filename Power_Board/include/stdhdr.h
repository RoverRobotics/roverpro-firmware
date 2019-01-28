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

#include <stdlib.h>
#include <stdint.h>
#include "xc.h"
#include <stdbool.h>

/// Instruction clock frequency in Hz
#define FCY 16000000UL

/// When built in release mode, does nothing.
/// When built in DEBUG, if a PICkit is attached, halts execution. Note the behavior of timers and
/// peripherals can be configgered in the IDE When built in DEBUG, if no PICkit is attached,
/// restarts execution.
#define BREAKPOINT()                                                                               \
    {                                                                                              \
#if __DEBUG __builtin_software_breakpoint();                                               \
#endif /* note the NOP prevents a problem with `switch{default: BREAKPOINT()}`, which      \
                  would otherwise crash */                                                         \
            __builtin_nop();                                                                       \
    }
/// Conditional breakpoint
#define BREAKPOINT_IF(condition)                                                                   \
    if (condition) {                                                                               \
        BREAKPOINT();                                                                              \
    }                                                                                              \
#endif

/// Block for the specified amount of time. Prevents a Watchdog Timer reset in the event of a long
/// wait.
void block_ms(uint16_t ms);

/// return the nearest integer within the given range
int16_t clamp(int16_t value, int16_t lo, int16_t hi);

/// return the nearest float within the given range
float clamp_f(float value, float lo, float hi);

/// compute the mean of an array of ints
int16_t mean(size_t count, int16_t *values);

/// compute the mean of an array of longs
uint16_t mean_u(size_t count, uint16_t *values);

#include "HardwareProfile.h"

#endif

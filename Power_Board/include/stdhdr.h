/// @file
/// Common declarations.

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
/// peripherals can be configgered in the IDE. When built in DEBUG, if no PICkit is attached,
/// restarts execution.
// The NOP is important. It prevents a problem with `switch{default:  BREAKPOINT()}`, which would
// otherwise crash. It also allows the debugger to stop on this line instead of overshooting.
#if (__DEBUG)
#define BREAKPOINT()                                                                               \
    {                                                                                              \
        __builtin_software_breakpoint();                                                           \
        __builtin_nop();                                                                           \
    }
#else
#define BREAKPOINT() ((void)0)
#endif

/// Conditional breakpoint
#define BREAKPOINT_IF(condition)                                                                   \
    if (condition) {                                                                               \
        BREAKPOINT();                                                                              \
    }

/// Block for the specified amount of time. Periodically resets the Watchdog Timer so a long wait
/// doesn't trigger a reset
void block_ms(uint16_t ms);

/// @return the nearest integer within the given range (inclusive)
int16_t clamp(int16_t value, int16_t lo, int16_t hi);

/// @return the nearest float within the given range
float clamp_f(float value, float lo, float hi);

/// @return the mean of the array of ints
int16_t mean(size_t count, int16_t *values);

/// @return the mean of the array of unsigned ints
uint16_t mean_u(size_t count, uint16_t *values);

#endif
